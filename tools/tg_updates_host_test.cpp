// Host test for the reply reader (src/tg_updates.cpp). Built and run by
// tools/tg_updates_host_test.sh. Each case is a getUpdates body and the
// decision it must produce; the stale and foreign ones must produce none.

#include <stdio.h>
#include <string.h>
#include <string>

#include "tg_updates.h"

static int g_fail = 0;

static void check(bool ok, const char *what)
{
	printf("%s  %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) {
		g_fail++;
	}
}

// The episode every case is judged against: photo message 500 in chat
// -100123, sent at 1700000000, tagged K7QF, nothing processed before 900.
static tg_episode_ref_t ep()
{
	tg_episode_ref_t e;
	e.chat_id      = -100123;
	e.photo_msg_id = 500;
	e.photo_date   = 1700000000;
	e.tag          = "K7QF";
	e.floor_update = 900;
	return e;
}

// One update with the given parts; "" leaves a part out.
static std::string upd(long id, const char *kind, long chat, long date,
                       const char *text, long reply_to, bool is_bot = false)
{
	std::string m = "{\"message_id\":777,\"from\":{\"id\":42,\"is_bot\":";
	m += is_bot ? "true" : "false";
	m += ",\"first_name\":\"E\"},\"chat\":{\"id\":" + std::to_string(chat) +
	     ",\"type\":\"group\"},\"date\":" + std::to_string(date);
	if (reply_to) {
		m += ",\"reply_to_message\":{\"message_id\":" + std::to_string(reply_to) +
		     ",\"date\":1699999990,\"caption\":\"person\\nvisit #K7QF\"}";
	}
	if (text[0]) {
		m += ",\"text\":\"";
		m += text;
		m += "\"";
	}
	m += "}";
	return "{\"update_id\":" + std::to_string(id) + ",\"" + kind + "\":" + m + "}";
}

static std::string body(const std::string &a, const std::string &b = "")
{
	std::string s = "{\"ok\":true,\"result\":[" + a;
	if (!b.empty()) {
		s += "," + b;
	}
	return s + "]}";
}

static tg_scan_t scan(const std::string &s, tg_episode_ref_t e = ep())
{
	tg_scan_t out;
	tg_scan_updates(s.data(), s.size(), &e, &out);
	return out;
}

int main()
{
	const long C = -100123, D = 1700000000;

	// Replies that count.
	check(scan(body(upd(901, "message", C, D + 5, "keep", 500))).reply == TG_REPLY_KEEP,
	      "keep, as a reply to the photo");
	check(scan(body(upd(901, "message", C, D + 5, "Yes!", 500))).reply == TG_REPLY_KEEP,
	      "Yes!, as a reply to the photo");
	check(scan(body(upd(901, "message", C, D + 5, "stop", 500))).reply == TG_REPLY_STOP,
	      "stop, as a reply to the photo");
	check(scan(body(upd(901, "message", C, D, "No", 500))).reply == TG_REPLY_STOP,
	      "No, same second as the photo");
	check(scan(body(upd(901, "message", C, D + 5, "keep #k7qf", 0))).reply == TG_REPLY_KEEP,
	      "keep #k7qf, not a reply but tagged");
	check(scan(body(upd(901, "message", C, D + 5, "/keep@wildbot K7QF", 0))).reply == TG_REPLY_KEEP,
	      "/keep@wildbot K7QF");
	check(scan(body(upd(901, "message", C, D + 5, "\\/stop", 500))).reply == TG_REPLY_STOP,
	      "escaped slash, /stop");
	{
		const tg_scan_t r = scan(body(upd(901, "message", C, D + 5, "keep", 500),
		                              upd(902, "message", C, D + 9, "stop", 500)));
		check(r.reply == TG_REPLY_STOP && r.reply_update_id == 902 && r.max_update_id == 902,
		      "keep then stop in one batch: the newest wins");
	}

	// Stale, foreign or unclear: never a decision.
	check(scan(body(upd(900, "message", C, D + 5, "keep", 500))).reply == TG_REPLY_NONE,
	      "update_id at the floor (already processed)");
	check(scan(body(upd(901, "message", C, D - 1, "keep", 500))).reply == TG_REPLY_NONE,
	      "dated before the photo");
	check(scan(body(upd(901, "message", C, D + 5, "keep", 499))).reply == TG_REPLY_NONE,
	      "a reply to an older message");
	check(scan(body(upd(901, "message", C, D + 5, "keep", 0))).reply == TG_REPLY_NONE,
	      "keep with no reference to the episode");
	check(scan(body(upd(901, "message", C, D + 5, "keep #K7QX", 0))).reply == TG_REPLY_NONE,
	      "keep with another episode's tag");
	check(scan(body(upd(901, "message", C, D + 5, "keep K7QFZ", 0))).reply == TG_REPLY_NONE,
	      "tag only as part of a longer word");
	check(scan(body(upd(901, "message", -100124, D + 5, "keep", 500))).reply == TG_REPLY_NONE,
	      "another chat");
	check(scan(body(upd(901, "edited_message", C, D + 5, "keep", 500))).reply == TG_REPLY_NONE,
	      "an edited message");
	check(scan(body(upd(901, "channel_post", C, D + 5, "keep", 500))).reply == TG_REPLY_NONE,
	      "a channel post");
	check(scan(body(upd(901, "message", C, D + 5, "keep", 500, true))).reply == TG_REPLY_NONE,
	      "from a bot");
	check(scan(body(upd(901, "message", C, D + 5, "keeper", 500))).reply == TG_REPLY_NONE,
	      "keeper is not keep");
	check(scan(body(upd(901, "message", C, D + 5, "please keep", 500))).reply == TG_REPLY_NONE,
	      "keep not as the first word");
	check(scan(body(upd(901, "message", C, D + 5, "", 500))).reply == TG_REPLY_NONE,
	      "no text (a sticker, a photo)");
	check(scan(body(upd(901, "message", C, D + 5, "keep \\\"update_id\\\":99999", 0))).reply ==
	          TG_REPLY_NONE,
	      "text holding JSON-looking escapes, untagged");
	{
		const tg_scan_t r = scan(body(upd(901, "message", C, D + 5, "keep", 500)) + "");
		std::string cut = body(upd(901, "message", C, D + 5, "keep", 500));
		cut.resize(cut.size() - 10);
		const tg_scan_t t = scan(cut);
		check(r.reply == TG_REPLY_KEEP && !t.ok && t.reply == TG_REPLY_NONE,
		      "a body cut short is no updates, not a reply");
	}
	check(!scan("{\"ok\":false,\"error_code\":409,\"description\":\"Conflict\"}").ok,
	      "409 (a webhook is set): not ok");
	check(scan("{\"ok\":true,\"result\":[]}").ok && scan("{\"ok\":true,\"result\":[]}").max_update_id == -1,
	      "empty result: ok, nothing seen");
	{
		const tg_scan_t r = scan(body(upd(950, "message", C, D + 5, "hello", 0),
		                              upd(951, "edited_message", C, D + 5, "keep", 500)));
		check(r.reply == TG_REPLY_NONE && r.max_update_id == 951 && r.updates == 2,
		      "non-replies still step the offset past themselves");
	}
	{
		// A huge nesting depth must fail cleanly, not overflow the stack.
		std::string deep = "{\"ok\":true,\"result\":[";
		for (int i = 0; i < 5000; i++) deep += "[";
		const tg_scan_t r = scan(deep);
		check(r.reply == TG_REPLY_NONE && r.updates == 0, "deep nesting fails cleanly");
	}

	// sendDocument's response.
	{
		const char *ok = "{\"ok\":true,\"result\":{\"message_id\":500,\"from\":{\"id\":1,"
		                 "\"is_bot\":true},\"chat\":{\"id\":-100123},\"date\":1700000000,"
		                 "\"document\":{\"file_name\":\"capture.jpg\"},\"caption\":\"x\"}}";
		int64_t id = 0, date = 0;
		check(tg_parse_sent(ok, strlen(ok), &id, &date) && id == 500 && date == 1700000000,
		      "sendDocument response: message_id and date");
		check(!tg_parse_sent(ok, 60, &id, &date), "sendDocument response cut short");
	}

	// TELEGRAM_CHAT_ID.
	{
		int64_t v = 0;
		check(tg_chat_id_number("-1001234567890", &v) && v == -1001234567890LL, "numeric chat id");
		check(!tg_chat_id_number("@mychannel", &v), "@channel chat id is refused");
		check(!tg_chat_id_number("", &v), "empty chat id is refused");
	}

	printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "passed", g_fail);
	return g_fail ? 1 : 0;
}
