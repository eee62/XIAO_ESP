// Reading Telegram's replies to an entry photo (tg_updates.h).

#include "tg_updates.h"

#include <string.h>

// Deeper than any getUpdates body goes (a reply's reply_to_message is about six
// levels down). Past it a value is unreadable, not a stack overflow.
#define JS_MAX_DEPTH 24

// ---------------------------------------------------------------------------
// The walker. js_skip() returns the index just past the value starting at
// s[i] (after leading whitespace), or 0 when it cannot: 0 is never a valid end,
// since a value is at least one byte long.
// ---------------------------------------------------------------------------
static size_t js_ws(const char *s, size_t n, size_t i)
{
	while (i < n && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) {
		i++;
	}
	return i;
}

static size_t js_skip_string(const char *s, size_t n, size_t i)
{
	// s[i] is the opening quote.
	for (i++; i < n; i++) {
		const unsigned char c = (unsigned char)s[i];
		if (c == '"') {
			return i + 1;
		}
		if (c == '\\') {
			i++;   // the escaped character; \u's four hex digits are not quotes
		} else if (c < 0x20) {
			return 0;   // a raw control character: not JSON
		}
	}
	return 0;   // ran off the end: truncated
}

static size_t js_skip(const char *s, size_t n, size_t i, int depth)
{
	i = js_ws(s, n, i);
	if (i >= n || depth > JS_MAX_DEPTH) {
		return 0;
	}
	const char c = s[i];
	if (c == '"') {
		return js_skip_string(s, n, i);
	}
	if (c == '{' || c == '[') {
		const char close = c == '{' ? '}' : ']';
		i = js_ws(s, n, i + 1);
		if (i < n && s[i] == close) {
			return i + 1;
		}
		for (;;) {
			if (c == '{') {
				if (i >= n || s[i] != '"') {
					return 0;
				}
				i = js_skip_string(s, n, i);
				if (!i) {
					return 0;
				}
				i = js_ws(s, n, i);
				if (i >= n || s[i] != ':') {
					return 0;
				}
				i++;
			}
			i = js_skip(s, n, i, depth + 1);
			if (!i) {
				return 0;
			}
			i = js_ws(s, n, i);
			if (i >= n) {
				return 0;
			}
			if (s[i] == close) {
				return i + 1;
			}
			if (s[i] != ',') {
				return 0;
			}
			i = js_ws(s, n, i + 1);
		}
	}
	if (c == '-' || (c >= '0' && c <= '9')) {
		size_t j = i + 1;
		while (j < n && ((s[j] >= '0' && s[j] <= '9') || s[j] == '.' || s[j] == 'e' ||
		                 s[j] == 'E' || s[j] == '+' || s[j] == '-')) {
			j++;
		}
		// A number that runs into the end of the buffer may have lost digits.
		return j < n ? j : 0;
	}
	static const char *const lits[] = {"true", "false", "null"};
	for (const char *lit : lits) {
		const size_t len = strlen(lit);
		if (n - i >= len && memcmp(s + i, lit, len) == 0) {
			return i + len;
		}
	}
	return 0;
}

bool js_member(js_t obj, const char *key, js_t *out)
{
	const char *s = obj.p;
	const size_t n = obj.n;
	size_t i = js_ws(s, n, 0);
	if (i >= n || s[i] != '{') {
		return false;
	}
	const size_t klen = strlen(key);
	i = js_ws(s, n, i + 1);
	if (i < n && s[i] == '}') {
		return false;
	}
	for (;;) {
		if (i >= n || s[i] != '"') {
			return false;
		}
		const size_t kstart = i + 1;
		i = js_skip_string(s, n, i);
		if (!i) {
			return false;
		}
		// Raw comparison: the keys asked for are plain ASCII, and a key spelled
		// with escapes is not one of them.
		const bool match = (i - 1 - kstart) == klen && memcmp(s + kstart, key, klen) == 0;
		i = js_ws(s, n, i);
		if (i >= n || s[i] != ':') {
			return false;
		}
		const size_t vstart = js_ws(s, n, i + 1);
		const size_t vend = js_skip(s, n, vstart, 1);
		if (!vend) {
			return false;
		}
		if (match) {
			out->p = s + vstart;
			out->n = vend - vstart;
			return true;
		}
		i = js_ws(s, n, vend);
		if (i >= n || s[i] != ',') {
			return false;   // '}' (not found) or malformed
		}
		i = js_ws(s, n, i + 1);
	}
}

bool js_path(js_t root, const char *path, js_t *out)
{
	js_t cur = root;
	char key[32];
	while (*path) {
		const char *dot = strchr(path, '.');
		const size_t len = dot ? (size_t)(dot - path) : strlen(path);
		if (len == 0 || len >= sizeof(key)) {
			return false;
		}
		memcpy(key, path, len);
		key[len] = '\0';
		if (!js_member(cur, key, &cur)) {
			return false;
		}
		path += len + (dot ? 1 : 0);
	}
	*out = cur;
	return true;
}

bool js_index(js_t arr, size_t idx, js_t *out)
{
	const char *s = arr.p;
	const size_t n = arr.n;
	size_t i = js_ws(s, n, 0);
	if (i >= n || s[i] != '[') {
		return false;
	}
	i = js_ws(s, n, i + 1);
	if (i < n && s[i] == ']') {
		return false;
	}
	for (size_t k = 0;; k++) {
		const size_t vstart = js_ws(s, n, i);
		const size_t vend = js_skip(s, n, vstart, 1);
		if (!vend) {
			return false;
		}
		if (k == idx) {
			out->p = s + vstart;
			out->n = vend - vstart;
			return true;
		}
		i = js_ws(s, n, vend);
		if (i >= n || s[i] != ',') {
			return false;
		}
		i++;
	}
}

bool js_int(js_t v, int64_t *out)
{
	size_t i = 0;
	bool neg = false;
	if (i < v.n && v.p[i] == '-') {
		neg = true;
		i++;
	}
	if (i >= v.n) {
		return false;
	}
	uint64_t acc = 0;
	for (; i < v.n; i++) {
		const char c = v.p[i];
		if (c < '0' || c > '9') {
			return false;   // a fraction, an exponent or not a number
		}
		if (acc > (UINT64_C(0x7FFFFFFFFFFFFFFF) - (uint64_t)(c - '0')) / 10) {
			return false;
		}
		acc = acc * 10 + (uint64_t)(c - '0');
	}
	*out = neg ? -(int64_t)acc : (int64_t)acc;
	return true;
}

bool js_is_true(js_t v)
{
	return v.n == 4 && memcmp(v.p, "true", 4) == 0;
}

static int hexval(char c)
{
	return c >= '0' && c <= '9' ? c - '0'
	     : c >= 'a' && c <= 'f' ? c - 'a' + 10
	     : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
}

bool js_string(js_t v, char *buf, size_t cap)
{
	if (cap == 0) {
		return false;
	}
	buf[0] = '\0';
	if (v.n < 2 || v.p[0] != '"' || v.p[v.n - 1] != '"') {
		return false;
	}
	size_t o = 0;
	for (size_t i = 1; i < v.n - 1; i++) {
		char c = v.p[i];
		if (c == '\\') {
			if (++i >= v.n - 1) {
				return false;
			}
			switch (v.p[i]) {
			case '"':  c = '"';  break;
			case '\\': c = '\\'; break;
			case '/':  c = '/';  break;
			case 'b':  c = '\b'; break;
			case 'f':  c = '\f'; break;
			case 'n':  c = '\n'; break;
			case 'r':  c = '\r'; break;
			case 't':  c = '\t'; break;
			case 'u': {
				if (i + 4 > v.n - 2) {
					return false;   // the four hex digits run past the closing quote
				}
				int cp = 0;
				for (int k = 1; k <= 4; k++) {
					const int h = hexval(v.p[i + k]);
					if (h < 0) {
						return false;
					}
					cp = cp * 16 + h;
				}
				i += 4;
				c = (cp > 0 && cp < 0x80) ? (char)cp : '?';
				break;
			}
			default:
				return false;
			}
		}
		if (o + 1 < cap) {
			buf[o++] = c;
		}
	}
	buf[o] = '\0';
	return true;
}

// ---------------------------------------------------------------------------
// Replies
// ---------------------------------------------------------------------------
static char lower(char c)
{
	return c >= 'A' && c <= 'Z' ? (char)(c - 'A' + 'a') : c;
}

static bool is_alnum(char c)
{
	return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

// keep / yes / stop / no as the first word of `text`, else none.
static tg_reply_t reply_word(const char *text)
{
	const char *p = text;
	while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
		p++;
	}
	if (*p == '/') {
		p++;   // a bot command: /keep, /stop
	}
	char w[8];
	size_t n = 0;
	while (is_alnum(*p)) {
		if (n + 1 >= sizeof(w)) {
			return TG_REPLY_NONE;   // a longer word: "keeper", "nobody"
		}
		w[n++] = lower(*p++);
	}
	w[n] = '\0';
	if (*p == '@') {
		// "/keep@somebot": the command's addressee; the word ends at the '@'.
	} else if (is_alnum(*p)) {
		return TG_REPLY_NONE;
	}
	if (strcmp(w, "keep") == 0 || strcmp(w, "yes") == 0) {
		return TG_REPLY_KEEP;
	}
	if (strcmp(w, "stop") == 0 || strcmp(w, "no") == 0) {
		return TG_REPLY_STOP;
	}
	return TG_REPLY_NONE;
}

// Does `text` carry `tag` as a word of its own (any case, '#' allowed)?
static bool has_tag(const char *text, const char *tag)
{
	const size_t tl = strlen(tag);
	if (tl == 0) {
		return false;
	}
	for (const char *p = text; *p; p++) {
		if (p != text && is_alnum(p[-1])) {
			continue;
		}
		size_t k = 0;
		while (k < tl && p[k] && lower(p[k]) == lower(tag[k])) {
			k++;
		}
		if (k == tl && !is_alnum(p[tl])) {
			return true;
		}
	}
	return false;
}

// One update: does it count as a reply to `ep` (tg_updates.h)?
static tg_reply_t judge_update(js_t u, const tg_episode_ref_t *ep)
{
	js_t m, v;
	int64_t x;
	// "message" only: an edited_message is an old message changed, and it
	// must not be able to turn a stale reply into a fresh one.
	if (!js_member(u, "message", &m)) {
		return TG_REPLY_NONE;
	}
	if (!js_path(m, "chat.id", &v) || !js_int(v, &x) || x != ep->chat_id) {
		return TG_REPLY_NONE;
	}
	if (js_path(m, "from.is_bot", &v) && js_is_true(v)) {
		return TG_REPLY_NONE;
	}
	if (!js_member(m, "date", &v) || !js_int(v, &x) || x < ep->photo_date) {
		return TG_REPLY_NONE;
	}
	char text[160];
	if (!js_member(m, "text", &v) || !js_string(v, text, sizeof(text))) {
		return TG_REPLY_NONE;
	}
	const tg_reply_t word = reply_word(text);
	if (word == TG_REPLY_NONE) {
		return TG_REPLY_NONE;
	}
	const bool replies_to_photo = js_path(m, "reply_to_message.message_id", &v) &&
	                              js_int(v, &x) && x == ep->photo_msg_id;
	if (!replies_to_photo && !has_tag(text, ep->tag)) {
		return TG_REPLY_NONE;
	}
	return word;
}

void tg_scan_updates(const char *body, size_t len, const tg_episode_ref_t *ep,
                     tg_scan_t *out)
{
	memset(out, 0, sizeof(*out));
	out->max_update_id   = -1;
	out->reply_update_id = -1;

	const js_t root = {body, len};
	js_t v, result;
	if (!js_member(root, "ok", &v) || !js_is_true(v) ||
	    !js_member(root, "result", &result)) {
		return;
	}
	out->ok = true;
	for (size_t i = 0;; i++) {
		js_t u;
		if (!js_index(result, i, &u)) {
			break;   // the end of result
		}
		int64_t id;
		if (!js_member(u, "update_id", &v) || !js_int(v, &id)) {
			continue;
		}
		out->updates++;
		if (id > out->max_update_id) {
			out->max_update_id = id;
		}
		if (id <= ep->floor_update) {
			continue;   // processed already, or from before this episode's window
		}
		const tg_reply_t r = judge_update(u, ep);
		if (r != TG_REPLY_NONE && id > out->reply_update_id) {
			out->reply           = r;
			out->reply_update_id = id;
		}
	}
}

bool tg_parse_sent(const char *body, size_t len, int64_t *message_id, int64_t *date)
{
	const js_t root = {body, len};
	js_t v;
	return js_member(root, "ok", &v) && js_is_true(v) &&
	       js_path(root, "result.message_id", &v) && js_int(v, message_id) &&
	       js_path(root, "result.date", &v) && js_int(v, date);
}

bool tg_chat_id_number(const char *text, int64_t *out)
{
	const js_t v = {text, strlen(text)};
	return v.n > 0 && js_int(v, out);
}
