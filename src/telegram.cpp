// Telegram Bot API delivery — PROJECT_BRIEF.md 9.6.

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <esp_heap_caps.h>

#include "config.h"
#include "telegram.h"

// TLS records are 16 kB at most and mbedtls copies through its own buffer, so
// feeding it the whole 120 kB body in one write() buys nothing and makes a
// stall harder to attribute. This is also the granularity at which a dropped
// link is noticed.
#define TG_WRITE_CHUNK 4096

// Enough of the API's JSON error to be actionable in a serial log
// ({"ok":false,"error_code":400,"description":"..."}) without pulling a whole
// response body into RAM.
#define TG_ERR_CAPTURE 192

// Response header lines skipped to reach that body. Telegram sends about ten;
// this only bounds a reply whose headers never end.
#define TG_MAX_HEADER_LINES 40

bool telegram_configured()
{
	// Both are string literals from config.h/secrets.h, so this folds to a
	// constant at compile time.
	return TELEGRAM_TOKEN[0] != '\0' && TELEGRAM_CHAT_ID[0] != '\0';
}

// ---------------------------------------------------------------------------
// One request, start to finish.
//
// `body` is sent verbatim with the given Content-Type. Returns true on 2xx.
// ---------------------------------------------------------------------------
static bool tg_post(const char *method, const char *content_type,
                    const uint8_t *body, size_t body_len)
{
	if (!telegram_configured()) {
		log_e("telegram: no token/chat_id compiled in (see include/secrets.h)");
		return false;
	}

	WiFiClientSecure client;
#if TELEGRAM_INSECURE_TLS
	// Deliberate — see config.h and PROJECT_BRIEF.md 9.6.
	client.setInsecure();
#else
	client.setCACert(TELEGRAM_ROOT_CA);
#endif
	client.setHandshakeTimeout(TELEGRAM_HANDSHAKE_S);

	const uint32_t t0 = millis();
	if (!client.connect(TELEGRAM_HOST, TELEGRAM_PORT, TELEGRAM_TIMEOUT_MS)) {
		// Covers DNS failure too, which on a static-IP node (9.4) usually
		// means NET_DNS is pointing at something that is not a resolver.
		log_w("telegram: connect to %s:%d failed", TELEGRAM_HOST, TELEGRAM_PORT);
		return false;
	}
	log_i("telegram: TLS up in %lu ms", (unsigned long)(millis() - t0));

	String req;
	req.reserve(256 + sizeof(TELEGRAM_TOKEN));
	req += "POST /bot";
	req += TELEGRAM_TOKEN;
	req += "/";
	req += method;
	req += " HTTP/1.1\r\n";
	req += "Host: " TELEGRAM_HOST "\r\n";
	req += "User-Agent: wildlife-node\r\n";
	req += "Content-Type: ";
	req += content_type;
	req += "\r\n";
	req += "Content-Length: ";
	req += (uint32_t)body_len;
	req += "\r\n";
	// One request per connection. Keeping it alive would mean holding the TLS
	// session across a light sleep, and the radio is about to go down anyway.
	req += "Connection: close\r\n\r\n";

	if (client.write((const uint8_t *)req.c_str(), req.length()) != req.length()) {
		log_w("telegram: header write failed");
		client.stop();
		return false;
	}

	size_t off = 0;
	while (off < body_len) {
		const size_t want = (body_len - off < TG_WRITE_CHUNK)
		                        ? (body_len - off) : (size_t)TG_WRITE_CHUNK;
		const int wrote = client.write(body + off, want);
		if (wrote <= 0) {
			log_w("telegram: body write stalled at %u/%u",
			      (unsigned)off, (unsigned)body_len);
			client.stop();
			return false;
		}
		off += (size_t)wrote;

		if (millis() - t0 > TELEGRAM_TIMEOUT_MS) {
			log_w("telegram: upload timed out at %u/%u",
			      (unsigned)off, (unsigned)body_len);
			client.stop();
			return false;
		}
	}

	// Status line.
	while (!client.available()) {
		if (!client.connected()) {
			log_w("telegram: connection closed before a reply");
			client.stop();
			return false;
		}
		if (millis() - t0 > TELEGRAM_TIMEOUT_MS) {
			log_w("telegram: no reply within %d ms", TELEGRAM_TIMEOUT_MS);
			client.stop();
			return false;
		}
		delay(10);
	}

	const String status = client.readStringUntil('\n');   // "HTTP/1.1 200 OK"
	int code = 0;
	const int sp = status.indexOf(' ');
	if (sp > 0) {
		code = status.substring(sp + 1, sp + 4).toInt();
	}

	const bool ok = (code >= 200 && code < 300);
	if (!ok) {
		// Telegram puts the reason in the JSON body, not the status line, and
		// it is the difference between a wrong chat id and a revoked token.
		// The headers come first and on their own run well past
		// TG_ERR_CAPTURE, so read through to the blank line that ends them.
		// Every read here is a timed one: available() reads 0 between TLS
		// records, so it cannot tell a reply still arriving from one that has
		// ended.
		long content_len = -1;
		for (int i = 0; i < TG_MAX_HEADER_LINES; i++) {
			String line = client.readStringUntil('\n');
			line.trim();
			if (line.length() == 0) {
				break;   // end of the headers, or nothing more arrived
			}
			const int colon = line.indexOf(':');
			if (colon > 0 &&
			    line.substring(0, colon).equalsIgnoreCase("Content-Length")) {
				content_len = line.substring(colon + 1).toInt();
			}
		}

		size_t want = TG_ERR_CAPTURE;
		if (content_len >= 0 && (size_t)content_len < want) {
			want = (size_t)content_len;
		}
		// Stream::readBytes() by name, which waits out each gap with a
		// timeout. The override WiFiClientSecure would otherwise inherit,
		// NetworkClient::readBytes(), stops at the first -1 from read(), and
		// the TLS read() returns -1 whenever nothing is decrypted yet.
		char reply[TG_ERR_CAPTURE + 1];
		const size_t got = client.Stream::readBytes(reply, want);
		for (size_t i = 0; i < got; i++) {
			if (reply[i] == '\r' || reply[i] == '\n') {
				reply[i] = ' ';
			}
		}
		reply[got] = '\0';
		log_e("telegram: %s -> HTTP %d %s", method, code, reply);
	} else {
		log_i("telegram: %s ok (%u bytes, %lu ms)", method,
		      (unsigned)body_len, (unsigned long)(millis() - t0));
	}

	client.stop();
	return ok;
}

// ---------------------------------------------------------------------------
// multipart/form-data body, assembled in PSRAM.
// ---------------------------------------------------------------------------
static void append_field(String &s, const char *name, const char *value)
{
	s += "--" TELEGRAM_BOUNDARY "\r\n";
	s += "Content-Disposition: form-data; name=\"";
	s += name;
	s += "\"\r\n\r\n";
	s += value;
	s += "\r\n";
}

bool telegram_send_photo(const uint8_t *jpeg, size_t len,
                         const String &caption)
{
	if (!jpeg || len == 0) {
		return false;
	}

	String head;
	head.reserve(512 + caption.length());
	append_field(head, "chat_id", TELEGRAM_CHAT_ID);
	if (caption.length()) {
		// Bot API caps captions at 1024 characters and rejects the whole
		// request over that — never worth losing the photo for.
		const String cap = caption.length() > 1000 ? caption.substring(0, 1000)
		                                           : caption;
		append_field(head, "caption", cap.c_str());
	}
	head += "--" TELEGRAM_BOUNDARY "\r\n";
	head += "Content-Disposition: form-data; name=\"photo\"; "
	        "filename=\"capture.jpg\"\r\n";
	head += "Content-Type: image/jpeg\r\n\r\n";

	static const char TAIL[] = "\r\n--" TELEGRAM_BOUNDARY "--\r\n";
	const size_t tail_len = sizeof(TAIL) - 1;
	const size_t total    = head.length() + len + tail_len;

	// PSRAM, explicitly (9.6). The frame is already there; this is the second
	// copy of it and the largest single allocation in the send path.
	uint8_t *body = (uint8_t *)heap_caps_malloc(total, MALLOC_CAP_SPIRAM);
	if (!body) {
		log_e("telegram: no PSRAM for a %u byte multipart body",
		      (unsigned)total);
		return false;
	}

	memcpy(body, head.c_str(), head.length());
	memcpy(body + head.length(), jpeg, len);
	memcpy(body + head.length() + len, TAIL, tail_len);

	const bool ok = tg_post("sendPhoto",
	                        "multipart/form-data; boundary=" TELEGRAM_BOUNDARY,
	                        body, total);
	heap_caps_free(body);
	return ok;
}

// ---------------------------------------------------------------------------
// Telemetry with no photo. Small enough to build on the heap as a String.
// ---------------------------------------------------------------------------
bool telegram_send_message(const String &text)
{
	if (!text.length()) {
		return false;
	}

	String body;
	body.reserve(256 + text.length());
	append_field(body, "chat_id", TELEGRAM_CHAT_ID);
	const String msg = text.length() > 4000 ? text.substring(0, 4000) : text;
	append_field(body, "text", msg.c_str());
	body += "--" TELEGRAM_BOUNDARY "--\r\n";

	return tg_post("sendMessage",
	               "multipart/form-data; boundary=" TELEGRAM_BOUNDARY,
	               (const uint8_t *)body.c_str(), body.length());
}
