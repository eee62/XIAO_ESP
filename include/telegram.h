// Telegram Bot API delivery — PROJECT_BRIEF.md 9.6.
//
// Replaces the raw `image/jpeg` POST to a LAN server. Stills go out as
// multipart/form-data over TLS, to sendDocument or sendPhoto
// (TELEGRAM_STILL_AS_DOCUMENT); telemetry-only reports (9.3) go to
// sendMessage.
//
// Credentials are compile-time (include/secrets.h, via config.h). There is no
// runtime configuration store and no delivery spool: a frame that does not go
// out on the wake that captured it is dropped. See 7 for why — persisting it
// would mean a microSD on the ungated 3V3 rail.
//
// Every call assumes WiFi is already associated and leaves it that way — the
// caller owns the radio, because one association can carry a photo and the
// reply window after it (config.h), and bringing the link up per request would
// undo 9.4.
//
// Sending is 9.6. Receiving, telegram_get_updates(), is not in the brief: it
// is the reply window's (config.h, Reply window), the one place the node reads
// anything back from the chat.
#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <stddef.h>
#include <stdint.h>

// True when a token and a chat id were actually compiled in. Checked before
// the radio comes up, so a node built without secrets.h sleeps instead of
// spending an association discovering it cannot send.
bool telegram_configured();

// What the API said about a message it accepted, for the reply window: a
// reply has to point at this message_id and be dated no earlier than `date`.
// `ok` is false when the response did not come back whole enough to read
// both, even though the send itself succeeded.
struct tg_sent_t {
	bool    ok;
	int64_t message_id;
	int64_t date;   // Unix seconds, Telegram's clock
};

// Upload one file as multipart/form-data: `method` is the Bot API method
// (sendPhoto, sendDocument), `field` the form field it expects the file in
// (photo, document). `caption` may be empty. The payload is streamed from
// where it lies, between a small head and tail, never copied; Content-Length
// is known up front. `min_bps` is the slowest upload allowed to finish
// (config.h, TELEGRAM_MIN_BPS / TELEGRAM_CLIP_MIN_BPS). Returns true only on
// an HTTP 2xx from the API. With `sent`, the response body is read too, inside
// the same upload cap, and the message's id and date are filled in.
bool telegram_send_file(const char *method, const char *field,
                        const char *filename, const char *content_type,
                        const uint8_t *data, size_t len, const String &caption,
                        uint32_t min_bps, tg_sent_t *sent = nullptr);

// Upload one JPEG still: sendDocument with the original bytes, or sendPhoto
// (which Telegram recompresses) when TELEGRAM_STILL_AS_DOCUMENT is 0.
bool telegram_send_photo(const uint8_t *jpeg, size_t len, const String &caption,
                         tg_sent_t *sent = nullptr);

// Telemetry with no photo attached.
bool telegram_send_message(const String &text);

// telegram_get_updates() failures. Positive returns are HTTP status codes.
enum {
	TG_GET_ERR_TIME    = -1,   // too little time left to start a request
	TG_GET_ERR_CONNECT = -2,   // TCP connect or TLS handshake failed
	TG_GET_ERR_WRITE   = -3,
	TG_GET_ERR_READ    = -4,   // no complete response by the deadline
	TG_GET_ERR_SIZE    = -5,   // the body did not fit in `cap`
};

// One getUpdates request (Bot API long polling) for the reply window, with
// every wait inside it ending by `deadline_ms` (millis()): the TCP connect, the
// handshake and the request write get a third of the time left each, and the
// response wait runs to the deadline. Connects to `ip`, the address
// TELEGRAM_HOST resolved to on this association, with TELEGRAM_HOST for SNI,
// so there is no DNS lookup to outlast the deadline. `offset` 0 is left out of
// the request; `poll_s` caps the server-side long-poll wait, which is cut
// further to end REPLY_POLL_MARGIN_S before the deadline. Only "message"
// updates are asked for. The body lands in `buf`, NUL-terminated, *len bytes.
// Returns the HTTP status (200 on success) or a TG_GET_ERR_*.
int telegram_get_updates(const IPAddress &ip, int64_t offset, int limit,
                         int poll_s, uint32_t deadline_ms, char *buf, size_t cap,
                         size_t *len);
