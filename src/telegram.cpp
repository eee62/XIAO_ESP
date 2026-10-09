// Telegram Bot API delivery — PROJECT_BRIEF.md 9.6.

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <esp_heap_caps.h>
#include <strings.h>

#include "config.h"
#include "telegram.h"
#include "tg_updates.h"

// TLS records are 16 kB at most and mbedtls copies through its own buffer, so
// feeding it a whole payload in one write() buys nothing and makes a stall
// harder to attribute. This is also the granularity at which a dropped link
// is noticed and the overall cap is checked.
#define TG_WRITE_CHUNK 4096

// Enough of the API's JSON error to be actionable in a serial log
// ({"ok":false,"error_code":400,"description":"..."}) without pulling a whole
// response body into RAM.
#define TG_ERR_CAPTURE 192

// Response header lines skipped to reach that body. Telegram sends about ten;
// this only bounds a reply whose headers never end.
#define TG_MAX_HEADER_LINES 40

// A sendDocument response read whole for its message_id and date (tg_sent_t).
// It echoes the caption (cut to 1000 characters, telegram_send_file()) and
// describes the file, so ~2 kB; PSRAM, and only when asked for.
#define TG_SENT_CAPTURE 8192

bool telegram_configured()
{
	// Both are string literals from config.h/secrets.h, so this folds to a
	// constant at compile time.
	return TELEGRAM_TOKEN[0] != '\0' && TELEGRAM_CHAT_ID[0] != '\0';
}

// ---------------------------------------------------------------------------
// One request, start to finish.
//
// The body goes out as a list of segments written back to back, so a
// multipart upload streams its head, the payload straight from where it
// already sits in PSRAM, and its tail, without assembling a copy. Content-
// Length is their sum, known before the first byte. Returns true on 2xx.
//
// Two limits, neither of which scales badly with size (config.h):
//   - a stall: TELEGRAM_STALL_MS without write progress. ssl_client.cpp does
//     the watching: connect() stores its timeout as the socket timeout, and
//     send_ssl_data() fails a write that makes no progress for that long;
//   - a cap on the whole upload and reply, TELEGRAM_POST_BASE_S plus
//     Content-Length / `min_bps`, counted from the TLS session being up.
//     `min_bps` is the caller's floor: TELEGRAM_MIN_BPS for a still or a
//     message, TELEGRAM_CLIP_MIN_BPS for a clip. The connect has its own bounds: DNS, the TCP connect (also
//     TELEGRAM_STALL_MS) and TELEGRAM_HANDSHAKE_S.
// ---------------------------------------------------------------------------
struct tg_seg_t {
	const uint8_t *data;
	size_t         len;
};

// ---------------------------------------------------------------------------
// Reading a response with one deadline (millis()) on every wait: the status
// line, the headers and the body, by Content-Length or chunked. available()
// and read() never block on this client (the socket is non-blocking and
// available() only drains what TLS has already decrypted), so the deadline is
// the only wait there is.
// ---------------------------------------------------------------------------
struct tg_rd_t {
	WiFiClientSecure *c;
	uint32_t          deadline_ms;
	uint8_t           buf[256];
	int               pos;
	int               len;
	bool              timed_out;
};

static int rd_getc(tg_rd_t *r)
{
	while (r->pos >= r->len) {
		const int avail = r->c->available();
		if (avail > 0) {
			const int got = r->c->read(r->buf, avail < (int)sizeof(r->buf)
			                                       ? (size_t)avail : sizeof(r->buf));
			if (got > 0) {
				r->pos = 0;
				r->len = got;
				break;
			}
		} else if (!r->c->connected()) {
			return -1;
		}
		if ((int32_t)(millis() - r->deadline_ms) >= 0) {
			r->timed_out = true;
			return -1;
		}
		delay(5);
	}
	return r->buf[r->pos++];
}

// One line, CR and LF stripped, cut to cap - 1 characters (the rest of a long
// line is read and dropped). False if the line did not end.
static bool rd_line(tg_rd_t *r, char *line, size_t cap)
{
	size_t n = 0;
	for (;;) {
		const int c = rd_getc(r);
		if (c < 0) {
			return false;
		}
		if (c == '\n') {
			break;
		}
		if (c != '\r' && n + 1 < cap) {
			line[n++] = (char)c;
		}
	}
	line[n] = '\0';
	return true;
}

// "HTTP/1.1 200 OK" -> 200; 0 if there was no status line by the deadline.
static int rd_status(tg_rd_t *r)
{
	char line[64];
	if (!rd_line(r, line, sizeof(line))) {
		return 0;
	}
	const char *sp = strchr(line, ' ');
	return sp ? atoi(sp + 1) : 0;
}

static bool rd_headers(tg_rd_t *r, long *content_len, bool *chunked)
{
	*content_len = -1;
	*chunked     = false;
	char line[128];
	for (int i = 0; i < TG_MAX_HEADER_LINES; i++) {
		if (!rd_line(r, line, sizeof(line))) {
			return false;
		}
		if (line[0] == '\0') {
			return true;
		}
		if (strncasecmp(line, "Content-Length:", 15) == 0) {
			*content_len = atol(line + 15);
		} else if (strncasecmp(line, "Transfer-Encoding:", 18) == 0 &&
		           strcasestr(line + 18, "chunked")) {
			*chunked = true;
		}
	}
	return false;
}

// The body into buf, NUL-terminated. TG_GET_ERR_READ if it did not all arrive
// by the deadline, TG_GET_ERR_SIZE if it would not fit; 0 when it is whole.
static int rd_body(tg_rd_t *r, long content_len, bool chunked, char *buf,
                   size_t cap, size_t *len)
{
	size_t n = 0;
	*len   = 0;
	buf[0] = '\0';
	if (chunked) {
		for (;;) {
			char line[32];
			if (!rd_line(r, line, sizeof(line))) {
				return TG_GET_ERR_READ;
			}
			const long size = strtol(line, nullptr, 16);
			if (size <= 0) {
				break;   // the last chunk; trailers are not needed
			}
			for (long i = 0; i < size; i++) {
				const int c = rd_getc(r);
				if (c < 0) {
					return TG_GET_ERR_READ;
				}
				if (n + 1 >= cap) {
					return TG_GET_ERR_SIZE;
				}
				buf[n++] = (char)c;
			}
			if (!rd_line(r, line, sizeof(line))) {   // the CRLF after the chunk
				return TG_GET_ERR_READ;
			}
		}
	} else {
		// Content-Length, or (without one) until the server closes, which
		// Connection: close makes the end of the body.
		for (long i = 0; content_len < 0 || i < content_len; i++) {
			const int c = rd_getc(r);
			if (c < 0) {
				if (content_len < 0 && !r->timed_out) {
					break;
				}
				return TG_GET_ERR_READ;
			}
			if (n + 1 >= cap) {
				return TG_GET_ERR_SIZE;
			}
			buf[n++] = (char)c;
		}
	}
	buf[n] = '\0';
	*len   = n;
	return 0;
}

// `resp`, when given, receives the body of a 2xx reply (resp_cap bytes at
// most, NUL-terminated, *resp_len long; 0 long when it did not arrive whole),
// read inside the same cap as the upload, so no wait is added.
static bool tg_post(const char *method, const char *content_type,
                    const tg_seg_t *segs, int nsegs, uint32_t min_bps,
                    char *resp = nullptr, size_t resp_cap = 0,
                    size_t *resp_len = nullptr)
{
	if (!telegram_configured()) {
		log_e("telegram: no token/chat_id compiled in (see include/secrets.h)");
		return false;
	}

	size_t body_len = 0;
	for (int i = 0; i < nsegs; i++) {
		body_len += segs[i].len;
	}

	WiFiClientSecure client;
#if TELEGRAM_INSECURE_TLS
	// Deliberate — see config.h and PROJECT_BRIEF.md 9.6.
	client.setInsecure();
#else
	client.setCACert(TELEGRAM_ROOT_CA);
#endif
	client.setHandshakeTimeout(TELEGRAM_HANDSHAKE_S);

	const uint32_t t_connect = millis();
	if (!client.connect(TELEGRAM_HOST, TELEGRAM_PORT, TELEGRAM_STALL_MS)) {
		// Covers DNS failure too, which on a static-IP node (9.4) usually
		// means NET_DNS is pointing at something that is not a resolver.
		log_w("telegram: connect to %s:%d failed", TELEGRAM_HOST, TELEGRAM_PORT);
		return false;
	}
	const uint32_t t0 = millis();
	const uint32_t cap_ms = (uint32_t)TELEGRAM_POST_BASE_S * 1000u +
	                        (uint32_t)((uint64_t)body_len * 1000u / min_bps);
	log_i("telegram: TLS up in %lu ms; %u bytes, allowed %lu s",
	      (unsigned long)(t0 - t_connect), (unsigned)body_len,
	      (unsigned long)(cap_ms / 1000));

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

	size_t sent = 0;
	for (int i = 0; i < nsegs; i++) {
		size_t off = 0;
		while (off < segs[i].len) {
			const size_t left = segs[i].len - off;
			const size_t want = left < TG_WRITE_CHUNK ? left : (size_t)TG_WRITE_CHUNK;
			const int wrote = client.write(segs[i].data + off, want);
			if (wrote <= 0) {
				// No progress for TELEGRAM_STALL_MS, or the link dropped.
				log_w("telegram: body write stalled at %u/%u",
				      (unsigned)sent, (unsigned)body_len);
				client.stop();
				return false;
			}
			off  += (size_t)wrote;
			sent += (size_t)wrote;

			if (millis() - t0 > cap_ms) {
				log_w("telegram: upload over its %lu s cap at %u/%u",
				      (unsigned long)(cap_ms / 1000), (unsigned)sent,
				      (unsigned)body_len);
				client.stop();
				return false;
			}
		}
	}
	const uint32_t up_ms = millis() - t0;

	// Status line. Telegram has the whole body by now bar what is still in
	// the TCP send buffer, so the reply gets one stall's grace from the last
	// write, inside the same overall cap.
	const uint32_t t_sent = millis();
	// Status line.
	while (!client.available()) {
		if (!client.connected()) {
			log_w("telegram: connection closed before a reply");
			client.stop();
			return false;
		}
		if (millis() - t_sent > TELEGRAM_STALL_MS || millis() - t0 > cap_ms) {
			log_w("telegram: no reply %lu ms after the upload",
			      (unsigned long)(millis() - t_sent));
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
		if (resp && resp_cap && resp_len) {
			// The status line came in under the cap; the rest of the reply
			// gets what is left of it.
			*resp_len = 0;
			tg_rd_t rd = {};
			rd.c           = &client;
			rd.deadline_ms = t0 + cap_ms;
			long content_len;
			bool chunked;
			if (!rd_headers(&rd, &content_len, &chunked) ||
			    rd_body(&rd, content_len, chunked, resp, resp_cap, resp_len) != 0) {
				*resp_len = 0;
				log_w("telegram: %s reply body not read whole", method);
			}
		}
		// Upload rate is the number to watch on a weak link: it is what
		// the floor (TELEGRAM_MIN_BPS, TELEGRAM_CLIP_MIN_BPS) has to stay
		// under.
		log_i("telegram: %s ok (%u bytes, upload %lu ms, %lu B/s, total %lu ms)",
		      method, (unsigned)body_len, (unsigned long)up_ms,
		      (unsigned long)(up_ms ? (uint64_t)body_len * 1000u / up_ms : 0),
		      (unsigned long)(millis() - t_connect));
	}

	client.stop();
	return ok;
}

// ---------------------------------------------------------------------------
// multipart/form-data. Only the small parts are assembled; the payload is
// written from where it already sits. 9.6 has the whole body assembled in
// PSRAM; its point, keeping a frame off the stack, still holds, and streaming
// saves the second PSRAM copy of every payload.
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

bool telegram_send_file(const char *method, const char *field,
                        const char *filename, const char *content_type,
                        const uint8_t *data, size_t len, const String &caption,
                        uint32_t min_bps, tg_sent_t *sent)
{
	if (sent) {
		*sent = {};
	}
	if (!data || len == 0) {
		return false;
	}

	String head;
	head.reserve(512 + caption.length());
	append_field(head, "chat_id", TELEGRAM_CHAT_ID);
	if (caption.length()) {
		// Bot API caps captions at 1024 characters and rejects the whole
		// request over that — never worth losing the upload for.
		const String cap = caption.length() > 1000 ? caption.substring(0, 1000)
		                                           : caption;
		append_field(head, "caption", cap.c_str());
	}
	head += "--" TELEGRAM_BOUNDARY "\r\n";
	head += "Content-Disposition: form-data; name=\"";
	head += field;
	head += "\"; filename=\"";
	head += filename;
	head += "\"\r\nContent-Type: ";
	head += content_type;
	head += "\r\n\r\n";

	static const char TAIL[] = "\r\n--" TELEGRAM_BOUNDARY "--\r\n";
	const tg_seg_t segs[] = {
		{(const uint8_t *)head.c_str(), head.length()},
		{data, len},
		{(const uint8_t *)TAIL, sizeof(TAIL) - 1},
	};
	if (!sent) {
		return tg_post(method, "multipart/form-data; boundary=" TELEGRAM_BOUNDARY,
		               segs, 3, min_bps);
	}

	// The caller wants the message's id and date (the reply window). A failed
	// allocation or an unreadable reply leaves sent->ok false: the photo still
	// went out, only the window will not open.
	char *resp = (char *)heap_caps_malloc(TG_SENT_CAPTURE, MALLOC_CAP_SPIRAM);
	size_t resp_len = 0;
	const bool ok = tg_post(method, "multipart/form-data; boundary=" TELEGRAM_BOUNDARY,
	                        segs, 3, min_bps, resp, resp ? TG_SENT_CAPTURE : 0,
	                        &resp_len);
	if (ok && resp && resp_len) {
		sent->ok = tg_parse_sent(resp, resp_len, &sent->message_id, &sent->date);
	}
	if (ok && !sent->ok) {
		log_w("telegram: %s accepted, but its message_id/date could not be read",
		      method);
	}
	heap_caps_free(resp);
	return ok;
}

bool telegram_send_photo(const uint8_t *jpeg, size_t len, const String &caption,
                         tg_sent_t *sent)
{
#if TELEGRAM_STILL_AS_DOCUMENT
	// The original bytes. sendPhoto has Telegram recompress the image on its
	// side, which throws away exactly the detail the still settings are for.
	// A departure from 9.6, which names sendPhoto; set
	// TELEGRAM_STILL_AS_DOCUMENT to 0 for the brief's behaviour.
	return telegram_send_file("sendDocument", "document", "capture.jpg",
	                          "image/jpeg", jpeg, len, caption, TELEGRAM_MIN_BPS,
	                          sent);
#else
	return telegram_send_file("sendPhoto", "photo", "capture.jpg",
	                          "image/jpeg", jpeg, len, caption, TELEGRAM_MIN_BPS,
	                          sent);
#endif
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

	const tg_seg_t seg = {(const uint8_t *)body.c_str(), body.length()};
	return tg_post("sendMessage",
	               "multipart/form-data; boundary=" TELEGRAM_BOUNDARY, &seg, 1,
	               TELEGRAM_MIN_BPS);
}

// ---------------------------------------------------------------------------
// getUpdates, for the reply window (config.h). Not in 9.6, which only sends.
// ---------------------------------------------------------------------------

// The socket timeout is what connect() hands the TCP connect and every write
// (ssl_client.cpp). The only way to set it alongside an SNI host name is the
// protected member the timed connect() overloads write.
class TgClient : public WiFiClientSecure {
public:
	void set_socket_timeout_ms(int ms) { _timeout = ms; }
};

int telegram_get_updates(const IPAddress &ip, int64_t offset, int limit,
                         int poll_s, uint32_t deadline_ms, char *buf, size_t cap,
                         size_t *len)
{
	*len = 0;
	if (cap) {
		buf[0] = '\0';
	}
	if (!telegram_configured() || cap == 0) {
		return TG_GET_ERR_CONNECT;
	}
	const int32_t left = (int32_t)(deadline_ms - millis());
	if (left < (int32_t)REPLY_MIN_REQUEST_S * 1000) {
		return TG_GET_ERR_TIME;
	}

	// A third of what is left each for the TCP connect, the handshake and the
	// request write (which fits the TCP send buffer at once, so in practice
	// takes none of it). The response then waits to the deadline. So nothing
	// here can run past it: this is the stall-timeout approach of tg_post(),
	// cut to fit the window.
	const uint32_t third = (uint32_t)left / 3;
	uint32_t hs_s = third / 1000;
	if (hs_s > (uint32_t)TELEGRAM_HANDSHAKE_S) {
		hs_s = TELEGRAM_HANDSHAKE_S;
	}
	if (hs_s < 1) {
		hs_s = 1;   // REPLY_MIN_REQUEST_S keeps a third at 1 s or more
	}
	const uint32_t sock_ms = third < (uint32_t)TELEGRAM_STALL_MS ? third : TELEGRAM_STALL_MS;

	TgClient client;
#if TELEGRAM_INSECURE_TLS
	client.setInsecure();   // as tg_post(); config.h, 9.6
	const char *ca = nullptr;
#else
	const char *ca = TELEGRAM_ROOT_CA;
#endif
	client.setHandshakeTimeout(hs_s);
	client.set_socket_timeout_ms((int)sock_ms);

	const uint32_t t_connect = millis();
	if (!client.connect(ip, TELEGRAM_PORT, TELEGRAM_HOST, ca, nullptr, nullptr)) {
		log_w("telegram: getUpdates connect to %s failed", ip.toString().c_str());
		return TG_GET_ERR_CONNECT;
	}

	// The server holds a long poll open for up to `timeout` seconds and answers
	// at once when an update arrives. It has to answer before the deadline,
	// with REPLY_POLL_MARGIN_S for the answer to arrive.
	const int32_t after = (int32_t)(deadline_ms - millis()) / 1000 - REPLY_POLL_MARGIN_S;
	int timeout_s = poll_s;
	if (timeout_s > after) {
		timeout_s = after;
	}
	if (timeout_s < 0) {
		timeout_s = 0;
	}

	char req[256 + sizeof(TELEGRAM_TOKEN)];
	int n = snprintf(req, sizeof(req),
	                 "GET /bot%s/getUpdates?limit=%d&timeout=%d"
	                 "&allowed_updates=%%5B%%22message%%22%%5D",
	                 TELEGRAM_TOKEN, limit, timeout_s);
	if (offset != 0 && n > 0 && n < (int)sizeof(req)) {
		n += snprintf(req + n, sizeof(req) - n, "&offset=%lld", (long long)offset);
	}
	if (n > 0 && n < (int)sizeof(req)) {
		n += snprintf(req + n, sizeof(req) - n,
		              " HTTP/1.1\r\nHost: " TELEGRAM_HOST "\r\n"
		              "User-Agent: wildlife-node\r\nConnection: close\r\n\r\n");
	}
	if (n <= 0 || n >= (int)sizeof(req) ||
	    client.write((const uint8_t *)req, n) != (size_t)n) {
		log_w("telegram: getUpdates request write failed");
		client.stop();
		return TG_GET_ERR_WRITE;
	}

	tg_rd_t rd = {};
	rd.c           = &client;
	rd.deadline_ms = deadline_ms;
	const int code = rd_status(&rd);
	long content_len;
	bool chunked;
	int  err = 0;
	if (code == 0 || !rd_headers(&rd, &content_len, &chunked)) {
		err = TG_GET_ERR_READ;
	} else {
		err = rd_body(&rd, content_len, chunked, buf, cap, len);
	}
	client.stop();

	if (err) {
		log_w("telegram: getUpdates %s after %lu ms (status %d)",
		      err == TG_GET_ERR_SIZE ? "body too big" : "reply incomplete",
		      (unsigned long)(millis() - t_connect), code);
		*len = 0;
		return err;
	}
	if (code != 200) {
		// 409 means a webhook is set on this bot, or another getUpdates
		// consumer; 401 a revoked token. The description says which.
		log_e("telegram: getUpdates -> HTTP %d %.160s", code, buf);
	} else {
		log_i("telegram: getUpdates ok, %u bytes, poll %d s, %lu ms",
		      (unsigned)*len, timeout_s, (unsigned long)(millis() - t_connect));
	}
	return code;
}
