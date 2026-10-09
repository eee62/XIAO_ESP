// Reading Telegram's replies to an entry photo (config.h, Reply window).
//
// Plain C++ with no Arduino or ESP-IDF in it, so tools/tg_updates_host_test.sh
// can build it on the host and throw stale, foreign and malformed updates at
// it. telegram.cpp fetches the JSON; this decides what it means.
//
// The JSON reading is deliberately narrow: it walks the text far enough to find
// the few members asked for, by exact key, and it treats anything it cannot
// walk (a truncated body, a bad escape, nesting past a limit) as absent. Absent
// never turns into a decision: a reply that cannot be read counts as no reply,
// and a body that was cut short reads as no updates at all.
#pragma once

#include <stddef.h>
#include <stdint.h>

// One JSON value's text, from its first byte to just past its last.
struct js_t {
	const char *p;
	size_t      n;
};

// The value `key` in object `obj`, by exact key. False if `obj` is not an
// object, has no such member, or cannot be walked as far as it.
bool js_member(js_t obj, const char *key, js_t *out);
// A dotted path of members, "message.chat.id".
bool js_path(js_t root, const char *path, js_t *out);
// Element `idx` of array `arr`.
bool js_index(js_t arr, size_t idx, js_t *out);
// A JSON integer (no fraction, no exponent) that fits in int64_t.
bool js_int(js_t v, int64_t *out);
bool js_is_true(js_t v);
// A JSON string, unescaped into `buf` (always NUL-terminated, cut to cap - 1
// bytes). \u escapes outside ASCII become '?': only ASCII is ever matched.
// False if `v` is not a well-formed string.
bool js_string(js_t v, char *buf, size_t cap);

// What a reply asked for.
enum tg_reply_t : uint8_t {
	TG_REPLY_NONE = 0,   // no reply that counts (yet)
	TG_REPLY_KEEP,       // "keep" or "yes": record the whole visit
	TG_REPLY_STOP,       // "stop" or "no": stop after the first clip
};

// The episode a reply has to be about.
struct tg_episode_ref_t {
	int64_t     chat_id;        // TELEGRAM_CHAT_ID as a number
	int64_t     photo_msg_id;   // message_id of this episode's entry photo, or -1
	int64_t     clip_msg_id;    // message_id of its first clip, or -1
	int64_t     photo_date;     // the photo's date (the clip's if the photo's
	                            // is unknown), Unix seconds by Telegram's clock
	const char *tag;            // the episode's short id, as in the captions
	int64_t     floor_update;   // only update_ids above this count; -1: any
};

// The outcome of one getUpdates body.
struct tg_scan_t {
	bool       ok;              // {"ok":true,"result":[...]} as expected
	int        updates;         // elements of result whose update_id was read
	int64_t    max_update_id;   // the highest of those, -1 if none
	tg_reply_t reply;           // from the newest update that counts
	int64_t    reply_update_id;
};

// Scan a getUpdates response for a reply to `ep`. An update counts only if all
// of these hold:
//   - its update_id is above ep->floor_update
//   - it is a new message (not an edit, not a channel post) from a person,
//     in ep->chat_id
//   - its date is no earlier than the photo's: it was written after the
//     photo existed, by Telegram's own clock on both sides
//   - it refers to this episode: a reply to the photo's or the first clip's
//     own message, or text that carries ep->tag as a word
//   - its first word is keep, yes, stop or no (any case, a leading '/' and a
//     trailing "@botname" allowed, so the bot commands /keep and /stop work)
// The newest update that counts wins. Every update_id read, counting or not,
// raises max_update_id, so the caller can step the offset past them all.
void tg_scan_updates(const char *body, size_t len, const tg_episode_ref_t *ep,
                     tg_scan_t *out);

// message_id and date from a sendDocument/sendPhoto response:
// {"ok":true,"result":{"message_id":..,"date":..,...}}. False unless both read.
bool tg_parse_sent(const char *body, size_t len, int64_t *message_id, int64_t *date);

// TELEGRAM_CHAT_ID's text as a number. False for "@channelname" and the like,
// which no update's chat.id can match, so a node set up that way never acts
// on a reply.
bool tg_chat_id_number(const char *text, int64_t *out);
