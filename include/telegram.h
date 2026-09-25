// Telegram Bot API delivery — PROJECT_BRIEF.md 9.6.
//
// Replaces the raw `image/jpeg` POST to a LAN server. Photos go to sendPhoto
// as multipart/form-data over TLS; telemetry-only reports (9.3) go to
// sendMessage.
//
// Credentials are compile-time (include/secrets.h, via config.h). There is no
// runtime configuration store and no delivery spool: a frame that does not go
// out on the wake that captured it is dropped. See 7 for why — persisting it
// would mean a microSD on the ungated 3V3 rail.
//
// Both calls assume WiFi is already associated and leave it that way — the
// caller owns the radio, because a burst delivers several frames over one
// association and bringing the link up per frame would undo 9.4.
#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// True when a token and a chat id were actually compiled in. Checked before
// the radio comes up, so a node built without secrets.h sleeps instead of
// spending an association discovering it cannot send.
bool telegram_configured();

// Upload one JPEG. `caption` may be empty. The multipart body is assembled in
// PSRAM (9.6) — an SVGA frame is 60-120 kB and must not go near the stack.
// Returns true only on an HTTP 2xx from the API.
bool telegram_send_photo(const uint8_t *jpeg, size_t len, const String &caption);

// Telemetry with no photo attached.
bool telegram_send_message(const String &text);
