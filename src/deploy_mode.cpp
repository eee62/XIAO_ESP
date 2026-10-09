// Deployment mode — AP + async web UI for siting the node.
//
// Interface and entry conditions: include/deploy_mode.h and the deployment
// block in include/config.h.
//
// Threading: every request handler below runs on the AsyncTCP task. Camera
// *power* transitions, esp_camera_init()/deinit() and frame-size changes
// happen only in deploy_mode_service(), i.e. on the Arduino loop task,
// exactly as they do in the normal duty cycle. So do full-size stills and the
// cold-capture test: handlers only ask for them and serve the result. The one
// thing a handler pulls itself is the live stream, and only while holding a
// use-count taken under g_cam_lock — which is what stops the service task
// resizing or tearing the sensor down underneath an in-flight response.

#include "deploy_mode.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <memory>
#include <new>

// Before the conditional include below — it is what defines DEPLOY_CAPTIVE_DNS.
#include "config.h"
#include "netcfg.h"
#include "telegram.h"

#if DEPLOY_CAPTIVE_DNS
#include <DNSServer.h>
#endif

#define MJPEG_BOUNDARY "wildlifeframe"

// How often deploy_mode_service() runs. Also the presence sampling interval —
// the radar holds OT2 for RADAR_UNMANNED_DELAY_S (10 s at the least) after the
// last presence, so this is three orders of magnitude faster than it needs to
// be.
#define SERVICE_TICK_MS 10

// A deferred action cannot run inside a request handler: the response has to
// be flushed to the browser first, and neither esp_restart() nor deep sleep
// would let that happen.
#define ACTION_DELAY_MS 400

// How long service_action() waits for an in-flight response to let go of the
// camera before giving up on a clean deinit. 1 s at SERVICE_TICK_MS.
#define SHUTDOWN_DRAIN_TICKS 100

static AsyncWebServer g_server(DEPLOY_HTTP_PORT);
#if DEPLOY_CAPTIVE_DNS
static DNSServer g_dns;
#endif

// ---------------------------------------------------------------------------
// Camera arbitration
//
// PREVIEW streams at DEPLOY_PREVIEW_FRAMESIZE; FULL is CAM_FRAMESIZE, for
// stills. Only the loop task changes mode, and only while no handler holds a
// use. SWITCHING marks a change in progress, so no stream can start against a
// sensor that is being resized, power-cycled or run through the cold test.
// ---------------------------------------------------------------------------
enum cam_mode_t : uint8_t { CAM_OFF = 0, CAM_PREVIEW, CAM_FULL, CAM_SWITCHING };

// The enum is ordered by size for the 4:3 sizes involved; see config.h.
static_assert(DEPLOY_PREVIEW_FRAMESIZE <= CAM_FRAMESIZE,
              "DEPLOY_PREVIEW_FRAMESIZE must not be larger than CAM_FRAMESIZE");

static SemaphoreHandle_t g_cam_lock;
static cam_mode_t  g_cam_mode        = CAM_OFF;
static int         g_cam_users       = 0;
static uint32_t    g_cam_request_ms  = 0;   // last request for the sensor at all
static uint32_t    g_full_request_ms = 0;   // last request for it at full size
static bool        g_stream_open     = false;   // async task only; single-threaded there
// What the driver actually set for CAM_FRAMESIZE: esp_camera_init() clamps a
// size past the sensor's maximum, FRAMESIZE_5MP to QSXGA on the OV5640.
static framesize_t g_full_size       = CAM_FRAMESIZE;

// Loop task only.
static uint32_t    g_cam_retry_at    = 0;
static uint32_t    g_settle_until    = 0;   // no still from a frame started before this

static uint32_t g_frames_served   = 0;
static size_t   g_last_frame_len  = 0;

// A JPEG copied out of the driver into PSRAM, shared between the loop task
// that made it and whichever responses are sending it: a slow client never
// holds a camera buffer, and a newer still never frees one mid-send.
struct psram_blob_t {
	uint8_t *data = nullptr;
	size_t   len  = 0;
	~psram_blob_t() { heap_caps_free(data); }
};
typedef std::shared_ptr<psram_blob_t> blob_ref;

// Takes ownership of a heap_caps buffer.
static blob_ref blob_adopt(uint8_t *data, size_t len)
{
	blob_ref b(new (std::nothrow) psram_blob_t());
	if (!b) {
		heap_caps_free(data);
		return blob_ref();
	}
	b->data = data;
	b->len  = len;
	return b;
}

static blob_ref blob_copy(const uint8_t *src, size_t len)
{
	uint8_t *data = (uint8_t *)heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
	if (!data) {
		return blob_ref();
	}
	memcpy(data, src, len);
	return blob_adopt(data, len);
}

// Full-size stills, all under g_cam_lock. A request bumps g_still_want; the
// loop task answers every request made so far with one frame, or a failure,
// by catching g_still_done up.
static blob_ref g_still;
static uint32_t g_still_want   = 0;
static uint32_t g_still_done   = 0;
static bool     g_still_failed = false;
static uint32_t g_still_req_ms = 0;   // newest request; the frame must be newer

// Cold-capture test. g_cold holds the last result with its jpeg pointer
// cleared; the JPEG itself is g_cold_jpeg. All under g_cam_lock except the
// request flag.
static volatile bool     g_cold_req     = false;
static volatile uint32_t g_cold_req_ms  = 0;
static bool              g_cold_running = false;
static bool              g_cold_busy    = false;   // refused: a stream held the camera
static uint32_t          g_cold_seq     = 0;       // results published
static cold_test_t       g_cold         = {};
static blob_ref          g_cold_jpeg;

// Test clip, the same way: g_clip without its buffer pointer, the file itself
// in g_clip_avi.
static volatile bool     g_clip_req     = false;
static volatile uint32_t g_clip_req_ms  = 0;
static bool              g_clip_running = false;
static bool              g_clip_busy    = false;
static uint32_t          g_clip_seq     = 0;
static test_clip_t       g_clip         = {};
static blob_ref          g_clip_avi;

// Presence walk test. g_pir_high_ms is how long D1 was high the last time, rise
// to fall, sampled every SERVICE_TICK_MS: after one brief pass that is the
// sensor's hold after the last presence (the radar's unmanned delay), which is
// what lengthens a clip's tail beyond VIDEO_END_QUIET_S.
//
// PIR-era assumption, kept as it is: "one brief pass" reads as the hold time
// only if the tester leaves the radar's range altogether. The AM312 dropped its
// output ~10 s after the last movement wherever the tester then stood; the
// radar keeps OT2 high for anyone still in range, moving or not, so a tester
// who stops a few metres off reads a longer "last high", or none until they
// leave.
static bool     g_pir_level       = false;
static uint32_t g_pir_edges       = 0;
static uint32_t g_pir_last_edge_s = 0;
static uint32_t g_pir_rise_ms     = 0;
static uint32_t g_pir_high_ms     = 0;

// Deferred actions requested over HTTP.
enum { ACTION_NONE = 0, ACTION_ARM, ACTION_REBOOT };
static volatile uint8_t  g_action    = ACTION_NONE;
static volatile uint32_t g_action_at = 0;

// Take a use on the camera for the live stream. Fails unless the sensor is up
// at preview size, which is the caller's cue to answer 503 and let the client
// retry — bringing it up or resizing it here would mean doing it on the async
// task. The attempt itself counts as a request, so the loop task will.
static bool cam_use_begin()
{
	bool ok = false;
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	g_cam_request_ms = millis();
	if (g_cam_mode == CAM_PREVIEW) {
		g_cam_users++;
		ok = true;
	}
	xSemaphoreGive(g_cam_lock);
	return ok;
}

static void cam_use_end()
{
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	if (g_cam_users > 0) {
		g_cam_users--;
	}
	g_cam_request_ms = millis();
	xSemaphoreGive(g_cam_lock);
}

// When a frame started, on the millis() clock (esp_timer, like the stamp
// cam_hal.c puts on each buffer).
static uint32_t fb_start_ms(const camera_fb_t *fb)
{
	return (uint32_t)(fb->timestamp.tv_sec * 1000 + fb->timestamp.tv_usec / 1000);
}

// ---------------------------------------------------------------------------
// JPEG feed for /stream (multipart, open-ended). /snapshot used to share it;
// stills now come from the loop task instead (service_still()).
//
// The state is owned by a shared_ptr captured in the response's filler lambda,
// so a client that walks away mid-frame destroys the response, destroys the
// lambda, and releases the frame buffer and the camera use with it. There is
// no other cleanup path and none is needed.
// ---------------------------------------------------------------------------
struct jpeg_feed_t {
	camera_fb_t *fb        = nullptr;
	String       hdr;
	size_t       cursor    = 0;
	bool         finished  = false;

	~jpeg_feed_t()
	{
		if (fb) {
			esp_camera_fb_return(fb);
		}
		g_stream_open = false;
		cam_use_end();
	}
};

static size_t jpeg_feed_fill(const std::shared_ptr<jpeg_feed_t> &st,
                             uint8_t *out, size_t maxLen)
{
	if (st->finished) {
		return 0;
	}

	if (!st->fb) {
		st->fb = esp_camera_fb_get();
		if (!st->fb) {
			// Sensor hiccup. Ending the response beats spinning on
			// RESPONSE_TRY_AGAIN; the page reloads the stream on its own.
			st->finished = true;
			return 0;
		}
		g_frames_served++;
		g_last_frame_len = st->fb->len;
		st->cursor = 0;
		st->hdr  = "\r\n--" MJPEG_BOUNDARY "\r\n"
		           "Content-Type: image/jpeg\r\n"
		           "Content-Length: ";
		st->hdr += st->fb->len;
		st->hdr += "\r\n\r\n";
	}

	const size_t hlen  = st->hdr.length();
	const size_t total = hlen + st->fb->len;

	size_t n = 0;
	while (n < maxLen && st->cursor < total) {
		const uint8_t *src;
		size_t avail;
		if (st->cursor < hlen) {
			src   = (const uint8_t *)st->hdr.c_str() + st->cursor;
			avail = hlen - st->cursor;
		} else {
			src   = st->fb->buf + (st->cursor - hlen);
			avail = total - st->cursor;
		}
		const size_t room  = maxLen - n;
		const size_t chunk = avail < room ? avail : room;
		memcpy(out + n, src, chunk);
		n          += chunk;
		st->cursor += chunk;
	}

	if (st->cursor >= total) {
		esp_camera_fb_return(st->fb);
		st->fb = nullptr;
	}
	return n;
}

// ---------------------------------------------------------------------------
// Status JSON
// ---------------------------------------------------------------------------
#if DETECTION_ENABLED
#if DETECT_MODEL == DETECT_MODEL_PEDESTRIAN
#define DEPLOY_MODEL_NAME "pedestrian"
#else
#define DEPLOY_MODEL_NAME "face"
#endif
#else
#define DEPLOY_MODEL_NAME "disabled"
#endif

// sta_ssid comes from include/secrets.h, which a person edits by hand, so it
// can still contain characters that would produce malformed JSON and leave the
// status page silently stuck. Bytes are read unsigned deliberately: `char` is
// signed on xtensa, and a UTF-8 SSID would trip the control-character branch.
static String json_escape(const String &in)
{
	String out;
	out.reserve(in.length() + 8);
	for (unsigned int i = 0; i < in.length(); i++) {
		const unsigned char c = (unsigned char)in[i];
		if (c == '"' || c == '\\') {
			out += '\\';
			out += (char)c;
		} else if (c < 0x20) {
			char esc[7];
			snprintf(esc, sizeof(esc), "\\u%04x", c);
			out += esc;
		} else {
			out += (char)c;
		}
	}
	return out;
}

// Home network card (netcfg.h). What the next wake's static fast path will use,
// read once when deployment mode starts and kept up to date by the two POST
// routes, so /status does not touch NVS on every poll. Only the async task
// reads or writes it.
static netcfg_t g_net;
static bool     g_net_saved = false;

static void net_refresh()
{
	g_net_saved = netcfg_load(&g_net);
	if (!g_net_saved) {
		netcfg_builtin(&g_net);
	}
}

static String net_addr(uint32_t a)
{
	char b[16];
	netcfg_format(a, b);
	return String(b);
}

static String status_json()
{
	deploy_status_t s;
	deploy_fill_status(&s);

	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	const cam_mode_t  mode  = g_cam_mode;
	const int         users = g_cam_users;
	const framesize_t full  = g_full_size;
	xSemaphoreGive(g_cam_lock);
	static const char *const MODE_NAMES[] = {"off", "preview", "full", "switching"};

	String j;
	j.reserve(1024);
	j += "{\"uptime\":";        j += now_s();
	j += ",\"cam\":";           j += mode != CAM_OFF ? "true" : "false";
	j += ",\"cam_mode\":\"";     j += MODE_NAMES[mode];
	j += "\"";
	j += ",\"cam_users\":";     j += users;
	// Active camera settings: stills are CAM_FRAMESIZE as the driver set it,
	// at the quality the next capture starts from (config.h,
	// CAM_QUALITY_STEP); orientation is what camera_apply_settings() writes.
	j += ",\"still_w\":";       j += resolution[full].width;
	j += ",\"still_h\":";       j += resolution[full].height;
	j += ",\"prev_w\":";        j += resolution[DEPLOY_PREVIEW_FRAMESIZE].width;
	j += ",\"prev_h\":";        j += resolution[DEPLOY_PREVIEW_FRAMESIZE].height;
	j += ",\"q\":";             j += s.cam_quality;
	j += ",\"vflip\":";         j += CAM_VFLIP;
	j += ",\"hmirror\":";       j += CAM_HMIRROR;
	j += ",\"frames\":";        j += g_frames_served;
	j += ",\"frame_len\":";     j += (uint32_t)g_last_frame_len;
	j += ",\"pir\":";           j += g_pir_level ? 1 : 0;
	j += ",\"pir_edges\":";     j += g_pir_edges;
	j += ",\"pir_last\":";      j += g_pir_last_edge_s;
	j += ",\"pir_high_ms\":";   j += g_pir_high_ms;
	j += ",\"unmanned_s\":";    j += RADAR_UNMANNED_DELAY_S;
	j += ",\"vbat_mv\":";       j += battery_mv();
	j += ",\"trig_total\":";    j += s.triggers_total;
	j += ",\"trig_since\":";    j += s.triggers_since_report;
	j += ",\"supp_total\":";    j += s.suppressed_total;
	j += ",\"supp_since\":";    j += s.suppressed_since_report;
	j += ",\"derr_total\":";    j += s.detect_errors_total;
	j += ",\"derr_since\":";    j += s.detect_errors_since_report;
	j += ",\"sent_total\":";    j += s.photos_sent_total;
	j += ",\"sent_since\":";    j += s.photos_sent_since_report;
	j += ",\"dropped\":";       j += s.photos_dropped_total;
	j += ",\"capped\":";        j += s.capped_total;
	j += ",\"wind_streak\":";   j += s.wind_streak;
	j += ",\"backoff_left\":";  j += s.backoff_left_s;
	j += ",\"backoff_total\":"; j += s.backoff_s_total;
	j += ",\"last_report\":";   j += s.last_report_s;
	j += ",\"ap_cache\":";      j += s.have_ap_cache ? "true" : "false";
	j += ",\"net_mode\":\"";     j += s.using_dhcp ? "dhcp" : "static";
	j += "\"";
	j += ",\"net_ip\":\"";       j += net_addr(g_net.ip);
	j += "\",\"net_src\":\"";    j += g_net_saved ? "saved" : "config.h";
	j += "\"";
	j += ",\"ap_ch\":";         j += s.ap_channel;
	j += ",\"clients\":";       j += WiFi.softAPgetStationNum();
	j += ",\"heap\":";          j += (uint32_t)ESP.getFreeHeap();
	j += ",\"heap_min\":";      j += (uint32_t)ESP.getMinFreeHeap();
	j += ",\"psram_free\":";    j += (uint32_t)ESP.getFreePsram();
	j += ",\"psram_size\":";    j += (uint32_t)ESP.getPsramSize();
	j += ",\"model\":\"" DEPLOY_MODEL_NAME "\"";
	j += ",\"thr\":";           j += String(DETECT_SCORE_THRESHOLD, 2);
	j += ",\"persons_only\":";  j += SEND_ONLY_PERSONS && DETECTION_ENABLED ? "true" : "false";
	j += ",\"per_ep\":";        j += PHOTOS_PER_EPISODE;
	j += ",\"gap\":";           j += PRESENCE_GAP_S;
	j += ",\"quiet\":";         j += VIDEO_END_QUIET_S;
	j += ",\"clips_sent\":";    j += s.clips_sent_total;
	j += ",\"clips_dropped\":"; j += s.clips_dropped_total;
	j += ",\"clip_s\":";        j += s.clip_s_total;
	j += ",\"fuse_trips\":";    j += s.clip_fuse_trips_total;
	j += ",\"r_keep\":";        j += s.replies_keep_total;
	j += ",\"r_stop\":";        j += s.replies_stop_total;
	j += ",\"r_none\":";        j += s.replies_none_total;
	j += ",\"r_err\":";         j += s.reply_errors_total;
	j += ",\"r_wait\":";        j += REPLY_WAIT_S;
	j += ",\"v_on\":";          j += VIDEO_ENABLED ? "true" : "false";
	j += ",\"v_w\":";           j += resolution[VIDEO_FRAMESIZE].width;
	j += ",\"v_h\":";           j += resolution[VIDEO_FRAMESIZE].height;
	j += ",\"v_q\":";           j += VIDEO_JPEG_QUALITY;
	j += ",\"v_fps\":";         j += VIDEO_FPS;
	j += ",\"v_max_s\":";       j += VIDEO_MAX_CLIP_S;
	j += ",\"v_max_kb\":";      j += (uint32_t)(VIDEO_MAX_BYTES / 1024);
	j += ",\"v_fuse\":";        j += VIDEO_MAX_CLIPS_PER_DAY;
	j += ",\"test_clip_s\":";   j += DEPLOY_TEST_CLIP_S;
	j += ",\"sta_ssid\":\"";  j += json_escape(WIFI_SSID);
	j += "\"";
	j += ",\"telegram\":";     j += telegram_configured() ? "true" : "false";
	j += "}";
	return j;
}

static String clip_json()
{
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	const test_clip_t r       = g_clip;
	const uint32_t    seq     = g_clip_seq;
	const bool        running = g_clip_running || g_clip_req;
	const bool        busy    = g_clip_busy;
	xSemaphoreGive(g_cam_lock);

	String j;
	j.reserve(192);
	j += "{\"seq\":";      j += seq;
	j += ",\"running\":";  j += running ? "true" : "false";
	j += ",\"busy\":";     j += busy ? "true" : "false";
	j += ",\"ok\":";       j += r.ok ? "true" : "false";
	j += ",\"frames\":";   j += r.frames;
	j += ",\"dur_ms\":";   j += r.dur_ms;
	j += ",\"fps\":";      j += String(r.fps, 2);
	j += ",\"len\":";      j += (uint32_t)r.len;
	j += ",\"w\":";        j += resolution[VIDEO_FRAMESIZE].width;
	j += ",\"h\":";        j += resolution[VIDEO_FRAMESIZE].height;
	j += "}";
	return j;
}

static String cold_json()
{
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	const cold_test_t r       = g_cold;
	const uint32_t    seq     = g_cold_seq;
	const bool        running = g_cold_running || g_cold_req;
	const bool        busy    = g_cold_busy;
	xSemaphoreGive(g_cam_lock);
	static const char *const DETECT_NAMES[] = {"off", "none", "hit", "error"};

	String j;
	j.reserve(320);
	j += "{\"seq\":";       j += seq;
	j += ",\"running\":";   j += running ? "true" : "false";
	j += ",\"busy\":";      j += busy ? "true" : "false";
	j += ",\"ok\":";        j += r.ok ? "true" : "false";
	j += ",\"w2s\":";       j += r.wake_to_shutter_ms;
	j += ",\"cap_ms\":";    j += r.capture_ms;
	j += ",\"len\":";       j += (uint32_t)r.jpeg_len;
	j += ",\"q\":";         j += r.quality;
	j += ",\"det\":\"";      j += DETECT_NAMES[r.detect];
	j += "\"";
	j += ",\"score\":";     j += String(r.score, 2);
	j += ",\"load_ms\":";   j += r.load_ms;
	j += ",\"dec_ms\":";    j += r.decode_ms;
	j += ",\"inf_ms\":";    j += r.infer_ms;
	j += "}";
	return j;
}

// ---------------------------------------------------------------------------
// The page. Self-contained: a node being sited has no internet, and nor does
// the phone that just joined CAM-SETUP.
// ---------------------------------------------------------------------------
static const char DEPLOY_PAGE[] = R"PAGE(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CAM-SETUP</title>
<style>
:root{--bg:#14161a;--card:#1d2026;--line:#2c313a;--fg:#e8eaed;--dim:#9aa3b0;
--accent:#6ea8fe;--warn:#f0a35e;--ok:#5fd08a;--hot:#ff6b6b}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--fg);
font:14px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
max-width:560px;margin-inline:auto}
h1{font-size:17px;margin:0}
header{display:flex;align-items:baseline;justify-content:space-between;
gap:8px;margin-bottom:14px}
header small{color:var(--dim)}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;
padding:12px;margin-bottom:12px}
.view{position:relative;background:#000;border-radius:7px;overflow:hidden;
aspect-ratio:4/3;display:flex;align-items:center;justify-content:center}
.view img{width:100%;height:100%;object-fit:contain;display:none}
.view img.shown{display:block}
#vmsg{color:var(--dim);font-size:13px;padding:12px;text-align:center}
.row{display:flex;gap:8px;margin-top:10px}
button{flex:1;padding:10px;border-radius:7px;border:1px solid var(--line);
background:#262b33;color:var(--fg);font:inherit;font-weight:500;cursor:pointer}
button:active{transform:translateY(1px)}
button.on{background:var(--accent);border-color:var(--accent);color:#0d1117}
button.warn{border-color:var(--warn);color:var(--warn)}
button.hot{border-color:var(--hot);color:var(--hot)}
.pir{display:flex;align-items:center;gap:12px}
.dot{width:14px;height:14px;border-radius:50%;background:#39404b;flex:none;
transition:background .15s}
.dot.hot{background:var(--hot);box-shadow:0 0 10px var(--hot)}
.pir b{font-weight:600}
.pir span{color:var(--dim);margin-left:auto;text-align:right;font-size:12px}
.grid{display:grid;grid-template-columns:1fr auto;gap:5px 14px;font-size:13px}
.grid div:nth-child(odd){color:var(--dim)}
.grid div:nth-child(even){text-align:right;font-variant-numeric:tabular-nums}
.sec{color:var(--dim);font-size:11px;text-transform:uppercase;
letter-spacing:.07em;margin:2px 0 8px}
#toast{position:fixed;left:50%;bottom:18px;transform:translateX(-50%);
background:#000c;border:1px solid var(--line);border-radius:7px;padding:8px 14px;
font-size:13px;opacity:0;transition:opacity .2s;pointer-events:none}
#toast.show{opacity:1}
a{color:var(--accent)}
.fbig{display:flex;align-items:baseline;gap:12px;margin-bottom:8px}
#fsize{font-size:44px;font-weight:700;line-height:1;font-variant-numeric:tabular-nums}
.fbig small{color:var(--dim);font-size:14px}
.fview{background:#000;border-radius:7px;overflow:hidden;display:flex;
justify-content:center}
#fcv{display:block}
.fsub{color:var(--dim);font-size:12px;margin-top:6px}
.fld{display:flex;align-items:center;gap:10px;margin-top:8px}
.fld label{flex:0 0 84px;color:var(--dim);font-size:13px}
.fld input{flex:1;min-width:0;padding:8px;border-radius:7px;border:1px solid var(--line);
background:#14161a;color:var(--fg);font:inherit;font-variant-numeric:tabular-nums}
</style></head><body>

<header><h1>Wildlife node</h1><small id="conn">connecting…</small></header>

<section class="card">
  <div class="sec">Preview &mdash; aim the lens</div>
  <div class="view"><img id="view" alt=""><div id="vmsg">waiting for camera…</div></div>
  <div class="row">
    <button id="live" class="on">Live</button>
    <button id="snap">Snapshot</button>
    <button id="focus">Focus</button>
  </div>
</section>

<section class="card" id="fcard" hidden>
  <div class="sec">Focus &mdash; centre at 100%, one sensor pixel per screen pixel</div>
  <div class="fbig"><span id="fsize">&mdash;</span><small id="fpeak"></small></div>
  <div class="fview"><canvas id="fcv"></canvas></div>
  <div class="fsub" id="fsub">Turn the lens slowly. Same scene, same quality:
    the bigger the JPEG, the more fine detail is in focus.</div>
  <div class="row"><button id="freset">Reset peak</button></div>
</section>

<section class="card">
  <div class="sec">Cold capture &mdash; what a presence wake produces</div>
  <div class="grid" id="cold"><div>Result</div><div>not run yet</div></div>
  <div class="row"><button id="coldbtn">Run cold capture</button></div>
  <div class="grid" id="clip" style="margin-top:12px"></div>
  <div class="row"><button id="clipbtn">Record test clip</button></div>
</section>

<section class="card pir">
  <div class="dot" id="pirdot"></div>
  <b id="pirtxt">Radar idle</b>
  <span id="pirsub">&mdash;</span>
</section>

<section class="card">
  <div class="sec">Status</div>
  <div class="grid" id="stat"></div>
</section>

<section class="card">
  <div class="sec">Home network &mdash; the node's address on your router</div>
  <div class="grid" id="net"><div>In use</div><div>loading…</div></div>
  <div class="fld"><label for="nip">IP address</label>
    <input id="nip" inputmode="decimal" autocomplete="off" placeholder="192.168.1.50"></div>
  <div class="fld"><label for="ngw">Gateway</label>
    <input id="ngw" inputmode="decimal" autocomplete="off" placeholder="default: IP's first three + .1"></div>
  <div class="fld"><label for="ndns">DNS</label>
    <input id="ndns" inputmode="decimal" autocomplete="off" placeholder="default: the gateway"></div>
  <div class="fld"><label for="nsn">Subnet</label>
    <input id="nsn" inputmode="decimal" autocomplete="off" placeholder="default: 255.255.255.0"></div>
  <div class="row">
    <button id="netsave" class="on">Save</button>
    <button id="netclear" class="warn">Forget saved IP</button>
  </div>
</section>

<section class="card">
  <div class="sec">When the aim is right</div>
  <div class="row">
    <button id="arm" class="warn">Arm &amp; sleep</button>
    <button id="reboot" class="hot">Reboot</button>
  </div>
</section>

<div id="toast"></div>

<script>
var $=function(s){return document.getElementById(s)};
var live=true, streaming=false, fails=0, focusing=false, peak=0;

function toast(m){var t=$('toast');t.textContent=m;t.className='show';
  clearTimeout(toast.t);toast.t=setTimeout(function(){t.className=''},2200)}

function dur(s){if(s<0)return'-';var d=Math.floor(s/86400);s%=86400;
  var h=Math.floor(s/3600);s%=3600;var m=Math.floor(s/60);s%=60;
  if(d)return d+'d '+h+'h';if(h)return h+'h '+m+'m';if(m)return m+'m '+s+'s';
  return s+'s'}
function kb(n){return n<1024?n+' B':n<1048576?(n/1024).toFixed(0)+' kB':
  (n/1048576).toFixed(2)+' MB'}
function kb1(n){return (n/1024).toFixed(1)+' kB'}

function startStream(){if(streaming)return;streaming=true;
  var v=$('view');v.onload=function(){v.className='shown';$('vmsg').textContent=''};
  v.onerror=function(){streaming=false;v.className='';
    $('vmsg').textContent='stream dropped, retrying…'};
  v.src='/stream?'+Date.now()}
function stopStream(){streaming=false;var v=$('view');v.src='';v.className='';
  $('vmsg').textContent='camera off'}

function pauseLive(){live=false;$('live').className='';stopStream()}
function stopFocus(){focusing=false;$('focus').className='';$('fcard').hidden=true}

$('live').onclick=function(){live=!live;this.className=live?'on':'';
  if(live)stopFocus();else{stopStream();$('vmsg').textContent='preview paused'}}
// Full size: the loop task switches the sensor up and back, so this can take
// a second or two.
$('snap').onclick=function(){stopFocus();pauseLive();
  $('vmsg').textContent='taking a full-size snapshot…';
  var v=$('view');v.onload=function(){v.className='shown';$('vmsg').textContent=''};
  v.onerror=function(){$('vmsg').textContent='snapshot failed'};
  v.src='/snapshot?'+Date.now()}

// Focus view: back-to-back full-size snapshots, the centre drawn at one
// sensor pixel per device pixel (so CSS size = pixels / devicePixelRatio),
// and each JPEG's size as a crude sharpness meter.
$('focus').onclick=function(){if(focusing){stopFocus();return}
  pauseLive();focusing=true;peak=0;this.className='on';$('fcard').hidden=false;
  $('vmsg').textContent='focus view below';focusLoop()}
$('freset').onclick=function(){peak=0;$('fpeak').textContent=''}
function focusLoop(){if(!focusing)return;
  fetch('/snapshot?'+Date.now(),{cache:'no-store'}).then(function(r){return r.blob()})
  .then(function(b){if(!focusing)return;if(!b.size)throw 0;
    if(b.size>peak)peak=b.size;
    $('fsize').textContent=kb1(b.size);$('fpeak').textContent='peak '+kb1(peak);
    var u=URL.createObjectURL(b),im=new Image();
    im.onload=function(){URL.revokeObjectURL(u);drawCentre(im);setTimeout(focusLoop,0)};
    im.onerror=function(){URL.revokeObjectURL(u);setTimeout(focusLoop,500)};
    im.src=u})
  .catch(function(){$('fsub').textContent='snapshot failed, retrying…';
    setTimeout(focusLoop,1000)})}
function drawCentre(im){
  var c=$('fcv'),dpr=window.devicePixelRatio||1,box=c.parentNode.clientWidth;
  var w=Math.min(Math.round(box*dpr),im.naturalWidth),
      h=Math.min(Math.round(box*dpr*3/4),im.naturalHeight);
  c.width=w;c.height=h;c.style.width=(w/dpr)+'px';c.style.height=(h/dpr)+'px';
  var x=c.getContext('2d');x.imageSmoothingEnabled=false;
  x.drawImage(im,(im.naturalWidth-w)>>1,(im.naturalHeight-h)>>1,w,h,0,0,w,h);
  $('fsub').textContent='centre '+w+'×'+h+' of '+im.naturalWidth+'×'+
    im.naturalHeight+', 1:1'}

// Cold capture: the loop task cuts the rail, then runs the presence wake's photo
// path. The result arrives as a new seq on GET /coldtest.
function gridRows(id,a){$(id).innerHTML=a.map(function(r){
  return'<div>'+r[0]+'</div><div>'+r[1]+'</div>'}).join('')}
function coldRows(a){gridRows('cold',a)}
$('coldbtn').onclick=function(){stopFocus();pauseLive();
  $('vmsg').textContent='cold capture running…';
  coldRows([['Result','running: rail off, then a cold start…']]);
  fetch('/coldtest',{method:'POST'}).then(function(r){return r.json()})
    .then(function(j){coldPoll(j.seq)})
    .catch(function(){coldRows([['Result','request failed']])})}
function coldPoll(seq){fetch('/coldtest',{cache:'no-store'})
  .then(function(r){return r.json()})
  .then(function(d){if(d.running||d.seq<=seq){
    setTimeout(function(){coldPoll(seq)},500);return}coldShow(d)})
  .catch(function(){setTimeout(function(){coldPoll(seq)},1000)})}
function coldShow(d){
  if(d.busy){coldRows([['Result','camera busy; close other previews and retry']]);return}
  if(!d.ok){$('vmsg').textContent='cold capture failed';
    coldRows([['Result','FAILED: no frame (see the serial log)'],
      ['Capture',d.cap_ms+' ms']]);return}
  var det=d.det=='off'?'no detector in this build':
    d.det=='error'?'ERROR: could not judge (sent unjudged in the field)':
    (d.det=='hit'?'PERSON, score ':'no person, best ')+d.score.toFixed(2);
  var r=[['Result','captured'],
    ['Wake-to-shutter',d.w2s+' ms from rail on (a presence wake adds its boot)'],
    ['Capture',d.cap_ms+' ms, rail on to rail off'],
    ['JPEG',kb1(d.len)+' · quality '+d.q],['Detection',det]];
  if(d.det!='off'&&d.det!='error'||d.dec_ms)
    r.push(['Detect time','decode '+d.dec_ms+' ms · inference '+d.inf_ms+' ms']);
  if(d.det!='off')r.push(['Model load',d.load_ms+' ms']);
  r.push(['Image','<a href="/coldtest.jpg?'+d.seq+'" target="_blank">open full size</a>']);
  coldRows(r);
  var v=$('view');v.onload=function(){v.className='shown';$('vmsg').textContent=''};
  v.onerror=function(){$('vmsg').textContent='could not load the capture'};
  v.src='/coldtest.jpg?'+d.seq}

// Test clip: the presence-video settings for a few seconds, recorded on the
// loop task, then offered as a download to open in Files, Photos or VLC.
var clipSecs=5;
$('clipbtn').onclick=function(){stopFocus();pauseLive();
  $('vmsg').textContent='recording a test clip…';
  gridRows('clip',[['Test clip','recording '+clipSecs+' s…']]);
  fetch('/testclip',{method:'POST'}).then(function(r){return r.json()})
    .then(function(j){clipPoll(j.seq)})
    .catch(function(){gridRows('clip',[['Test clip','request failed']])})}
function clipPoll(seq){fetch('/testclip',{cache:'no-store'})
  .then(function(r){return r.json()})
  .then(function(d){if(d.running||d.seq<=seq){
    setTimeout(function(){clipPoll(seq)},500);return}clipShow(d)})
  .catch(function(){setTimeout(function(){clipPoll(seq)},1000)})}
function clipShow(d){
  $('vmsg').textContent='preview paused';
  if(d.busy){gridRows('clip',[['Test clip','camera busy; close other previews and retry']]);return}
  if(!d.ok){gridRows('clip',[['Test clip','FAILED: no frames (see the serial log)']]);return}
  gridRows('clip',[['Test clip',d.frames+' frames · '+d.fps.toFixed(2)+' fps · '+
      (d.dur_ms/1000).toFixed(1)+' s'],
    ['File',kb(d.len)+' · '+d.w+'×'+d.h+' MJPEG AVI'],
    ['Download','<a href="/testclip.avi?'+d.seq+'" download="test.avi">test.avi</a>']])}

function act(path,label,confirmText){
  if(!confirm(confirmText))return;
  stopFocus();pauseLive();
  fetch(path,{method:'POST'}).then(function(){toast(label)})
    .catch(function(){toast(label)})}
$('arm').onclick=function(){act('/arm','Arming — node will sleep',
  'Leave deployment mode and start the normal radar duty cycle?')}
$('reboot').onclick=function(){act('/reboot','Rebooting',
  'Reboot the node? It will come back in normal mode.')}

// Home network: GET /netcfg shows what the next wake will use; Save and Forget
// answer with one line for the toast, an error when the node refused it.
function netLoad(){fetch('/netcfg',{cache:'no-store'}).then(function(r){return r.json()})
  .then(function(n){gridRows('net',[
    ['In use',n.ip+' · '+(n.src=='saved'?'saved':'built-in (config.h)')],
    ['Gateway · DNS',n.gw+' · '+n.dns],['Subnet',n.sn],
    ['Built-in',n.builtin],
    ['Wakes connect',n.dhcp?'by DHCP: the static address failed its check':
      'with this static address']])})
  .catch(function(){gridRows('net',[['In use','could not read']])})}
function netPost(path,body){fetch(path,{method:'POST',body:body})
  .then(function(r){return r.text()}).then(function(t){toast(t);netLoad()})
  .catch(function(){toast('request failed')})}
$('netsave').onclick=function(){var b=new URLSearchParams();
  ['ip','gw','dns','sn'].forEach(function(k){
    b.append(k,$({ip:'nip',gw:'ngw',dns:'ndns',sn:'nsn'}[k]).value.trim())});
  netPost('/netcfg',b)}
$('netclear').onclick=function(){
  if(confirm('Forget the saved IP and go back to the built-in (config.h) address?'))
    netPost('/netcfg/clear',null)}
netLoad();

var rows=[];
function row(k,v){rows.push('<div>'+k+'</div><div>'+v+'</div>')}

function render(d){
  $('conn').textContent=d.clients+' client'+(d.clients==1?'':'s')+
    ' · '+location.host;

  $('pirdot').className='dot'+(d.pir?' hot':'');
  $('pirtxt').textContent=d.pir?'PRESENCE (radar)':'Radar idle';
  $('pirsub').innerHTML=d.pir_edges+' trip'+(d.pir_edges==1?'':'s')+
    ' this session<br>'+(d.pir_edges?('last '+dur(d.uptime-d.pir_last)+' ago'):
    'walk-test the field of view')+
    (d.pir_high_ms?'<br>last high '+(d.pir_high_ms/1000).toFixed(1)+
    ' s (one brief pass = the radar\'s unmanned delay; config.h expects '+
    d.unmanned_s+' s)':'');

  rows=[];
  row('Uptime',dur(d.uptime));
  row('Battery',d.vbat_mv<0?'no divider fitted':(d.vbat_mv/1000).toFixed(2)+' V');
  row('Camera',d.cam?d.cam_mode+(d.cam_users?' · streaming':''):'powered down');
  row('Still',d.still_w+'×'+d.still_h+' · quality '+d.q);
  row('Preview',d.prev_w+'×'+d.prev_h);
  row('Orientation','v-flip '+(d.vflip?'on':'off')+' · mirror '+(d.hmirror?'on':'off'));
  row('Last frame',d.frames?kb(d.frame_len)+' · '+d.frames+' served':'—');
  row('Triggers',d.trig_total+' total · '+d.trig_since+' unreported');
  row('Photos sent',d.sent_total+' total · '+d.sent_since+' unreported · '+
    d.dropped+' dropped');
  row('Suppressed (no person)',d.supp_total+' total · '+d.supp_since+
    ' unreported · '+d.capped+' edges inside a visit, not photographed');
  row('Detect errors',d.derr_total+' total · '+d.derr_since+' unreported');
  row('Last report',d.last_report?dur(d.uptime-d.last_report)+' ago':'never');
  clipSecs=d.test_clip_s;
  row('Clips',d.clips_sent+' sent · '+d.clips_dropped+' dropped · '+
    dur(d.clip_s)+' recorded');
  row('Video',!d.v_on?'off':d.v_w+'×'+d.v_h+' · q '+d.v_q+' · '+d.v_fps+' fps · ≤'+
    d.v_max_s+' s or '+(d.v_max_kb/1024).toFixed(1)+' MB a clip · whole visit, '+
    'back to back');
  row('Clip fuse',d.v_fuse+'/day · '+(d.fuse_trips?'TRIPPED '+d.fuse_trips+'×':
    'never tripped'));
  row('Replies',!d.r_wait?'window off (whole visits)':d.r_keep+' keep · '+d.r_stop+
    ' stop · '+d.r_none+' none'+(d.r_err?' ('+d.r_err+' window errors)':'')+
    ' · '+d.r_wait+' s window');
  row('Gate',(d.persons_only?'entry photo must hold a person':'off, everything')+
    ' · '+d.per_ep+' photo per visit');
  row('Presence timing','quiet window '+d.quiet+' s · visit gap '+d.gap+' s');
  row('Wind',d.backoff_left?'Radar ignored for '+dur(d.backoff_left)+' more':
    (d.wind_streak?d.wind_streak+' empty photos in a row':'calm')+
    (d.backoff_total?' · ignored '+dur(d.backoff_total)+' so far':''));
  row('Detection',d.model+(d.model=='disabled'?'':' @ '+d.thr));
  row('Uplink',d.sta_ssid+' → Telegram '+(d.telegram?'ready':'NOT CONFIGURED'));
  row('Home IP',d.net_ip+' · '+(d.net_src=='saved'?'saved':'built-in (config.h)'));
  row('Network',d.net_mode=='dhcp'?'DHCP (the static address failed its check)':
    'static');
  row('AP cache',d.ap_cache?'ch '+d.ap_ch:'none (full scan next)');
  row('Free heap',kb(d.heap)+' (min '+kb(d.heap_min)+')');
  row('Free PSRAM',kb(d.psram_free)+' / '+kb(d.psram_size));
  $('stat').innerHTML=rows.join('');

  // Only a request brings a powered-down sensor back (DEPLOY_CAM_IDLE_MS),
  // so a live view with the camera down asks for it rather than waiting.
  if(d.cam_mode=='preview'&&live&&!streaming)startStream();
  if(d.cam_mode!='preview'&&streaming)stopStream();
  if(d.cam_mode!='preview'&&!streaming&&live){
    if(!d.cam)fetch('/wake',{method:'POST'});
    $('vmsg').textContent=d.cam?'switching to preview…':'camera warming up…'}
}

function poll(){
  fetch('/status',{cache:'no-store'}).then(function(r){return r.json()})
    .then(function(d){fails=0;render(d)})
    .catch(function(){if(++fails>2)$('conn').textContent='lost contact'})
    .then(function(){setTimeout(poll,1000)})}
poll();
</script></body></html>)PAGE";

// ---------------------------------------------------------------------------
// Routes
// ---------------------------------------------------------------------------
static void install_routes()
{
	g_server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
		AsyncWebServerResponse *res =
		    req->beginResponse(200, "text/html; charset=utf-8", DEPLOY_PAGE);
		res->addHeader("Cache-Control", "no-store");
		req->send(res);
	});

	g_server.on("/status", HTTP_GET, [](AsyncWebServerRequest *req) {
		AsyncWebServerResponse *res =
		    req->beginResponse(200, "application/json", status_json());
		res->addHeader("Cache-Control", "no-store");
		req->send(res);
	});

	// One full-size still (CAM_FRAMESIZE), for the aim check and the focus
	// view. The loop task takes it (service_still()); this response only
	// waits for it. The length is unknown when the headers go out, so it is
	// chunked, and the filler answers RESPONSE_TRY_AGAIN, which AsyncTCP's
	// poll re-asks about twice a second, until the frame is there. A failure
	// or DEPLOY_STILL_WAIT_MS without one ends it empty, which the page
	// treats as a failed snapshot.
	g_server.on("/snapshot", HTTP_GET, [](AsyncWebServerRequest *req) {
		struct snap_t {
			uint32_t want;
			uint32_t t0;
			blob_ref blob;
			size_t   cursor = 0;
			bool     done   = false;
		};
		std::shared_ptr<snap_t> st(new (std::nothrow) snap_t());
		if (!st) {
			req->send(503, "text/plain", "out of memory");
			return;
		}
		st->t0 = millis();
		xSemaphoreTake(g_cam_lock, portMAX_DELAY);
		g_cam_request_ms = g_full_request_ms = g_still_req_ms = st->t0;
		st->want = ++g_still_want;
		xSemaphoreGive(g_cam_lock);

		AsyncWebServerResponse *res = req->beginChunkedResponse(
		    "image/jpeg",
		    [st](uint8_t *buf, size_t maxLen, size_t) -> size_t {
			    if (st->done) {
				    return 0;
			    }
			    if (!st->blob) {
				    xSemaphoreTake(g_cam_lock, portMAX_DELAY);
				    const bool answered = (int32_t)(g_still_done - st->want) >= 0;
				    const bool failed   = g_still_failed;
				    blob_ref   blob     = g_still;
				    if (!answered) {
					    // Still wanted: keep the sensor at full size.
					    g_cam_request_ms = g_full_request_ms = millis();
				    }
				    xSemaphoreGive(g_cam_lock);
				    if (!answered) {
					    if (millis() - st->t0 > (uint32_t)DEPLOY_STILL_WAIT_MS) {
						    st->done = true;
						    return 0;
					    }
					    return RESPONSE_TRY_AGAIN;
				    }
				    if (failed || !blob) {
					    st->done = true;
					    return 0;
				    }
				    st->blob = blob;
			    }
			    const size_t left = st->blob->len - st->cursor;
			    const size_t n    = left < maxLen ? left : maxLen;
			    memcpy(buf, st->blob->data + st->cursor, n);
			    st->cursor += n;
			    if (n == 0) {
				    st->done = true;
			    }
			    return n;
		    });
		res->addHeader("Cache-Control", "no-store");
		req->send(res);
	});

	// The last cold-capture test's JPEG, with a real Content-Length.
	g_server.on("/coldtest.jpg", HTTP_GET, [](AsyncWebServerRequest *req) {
		xSemaphoreTake(g_cam_lock, portMAX_DELAY);
		blob_ref blob = g_cold_jpeg;
		xSemaphoreGive(g_cam_lock);
		if (!blob) {
			req->send(404, "text/plain", "no cold capture yet");
			return;
		}
		AsyncWebServerResponse *res = req->beginResponse(
		    "image/jpeg", blob->len,
		    [blob](uint8_t *buf, size_t maxLen, size_t index) -> size_t {
			    const size_t left = blob->len - index;
			    const size_t n    = left < maxLen ? left : maxLen;
			    memcpy(buf, blob->data + index, n);
			    return n;
		    });
		res->addHeader("Cache-Control", "no-store");
		req->send(res);
	});

	// Start a cold-capture test. It runs on the loop task (service_coldtest()),
	// since it cycles camera power. The reply carries the result count so far;
	// the page polls GET /coldtest until it moves.
	g_server.on("/coldtest", HTTP_POST, [](AsyncWebServerRequest *req) {
		xSemaphoreTake(g_cam_lock, portMAX_DELAY);
		const uint32_t seq  = g_cold_seq;
		const bool     busy = g_cold_running || g_cold_req;
		xSemaphoreGive(g_cam_lock);
		if (!busy) {
			g_cold_req_ms = millis();
			g_cold_req    = true;
		}
		String j = "{\"seq\":";
		j += seq;
		j += "}";
		req->send(202, "application/json", j);
	});

	g_server.on("/coldtest", HTTP_GET, [](AsyncWebServerRequest *req) {
		AsyncWebServerResponse *res =
		    req->beginResponse(200, "application/json", cold_json());
		res->addHeader("Cache-Control", "no-store");
		req->send(res);
	});

	// Record a test clip on the loop task (service_testclip()); the page polls
	// GET /testclip, as for the cold capture.
	g_server.on("/testclip", HTTP_POST, [](AsyncWebServerRequest *req) {
		xSemaphoreTake(g_cam_lock, portMAX_DELAY);
		const uint32_t seq  = g_clip_seq;
		const bool     busy = g_clip_running || g_clip_req;
		xSemaphoreGive(g_cam_lock);
		if (!busy) {
			g_clip_req_ms = millis();
			g_clip_req    = true;
		}
		String j = "{\"seq\":";
		j += seq;
		j += "}";
		req->send(202, "application/json", j);
	});

	g_server.on("/testclip", HTTP_GET, [](AsyncWebServerRequest *req) {
		AsyncWebServerResponse *res =
		    req->beginResponse(200, "application/json", clip_json());
		res->addHeader("Cache-Control", "no-store");
		req->send(res);
	});

	// The file, as a download: an attachment named test.avi, so a phone
	// saves it for Files or VLC rather than trying to show it inline.
	g_server.on("/testclip.avi", HTTP_GET, [](AsyncWebServerRequest *req) {
		xSemaphoreTake(g_cam_lock, portMAX_DELAY);
		blob_ref blob = g_clip_avi;
		xSemaphoreGive(g_cam_lock);
		if (!blob) {
			req->send(404, "text/plain", "no test clip yet");
			return;
		}
		AsyncWebServerResponse *res = req->beginResponse(
		    "video/x-msvideo", blob->len,
		    [blob](uint8_t *buf, size_t maxLen, size_t index) -> size_t {
			    const size_t left = blob->len - index;
			    const size_t n    = left < maxLen ? left : maxLen;
			    memcpy(buf, blob->data + index, n);
			    return n;
		    });
		res->addHeader("Content-Disposition", "attachment; filename=\"test.avi\"");
		res->addHeader("Cache-Control", "no-store");
		req->send(res);
	});

	// The page asks for the preview here when it finds the sensor powered
	// down: only a request brings it back (DEPLOY_CAM_IDLE_MS).
	g_server.on("/wake", HTTP_POST, [](AsyncWebServerRequest *req) {
		xSemaphoreTake(g_cam_lock, portMAX_DELAY);
		g_cam_request_ms = millis();
		xSemaphoreGive(g_cam_lock);
		req->send(204);
	});

	// Live MJPEG. One at a time: a second stream would halve the frame rate of
	// the first for no benefit, and both would be fighting one frame buffer.
	g_server.on("/stream", HTTP_GET, [](AsyncWebServerRequest *req) {
		if (g_stream_open) {
			req->send(503, "text/plain", "stream busy");
			return;
		}
		if (!cam_use_begin()) {
			req->send(503, "text/plain", "preview not ready");
			return;
		}
		std::shared_ptr<jpeg_feed_t> st(new (std::nothrow) jpeg_feed_t());
		if (!st) {
			cam_use_end();
			req->send(503, "text/plain", "out of memory");
			return;
		}
		g_stream_open = true;

		AsyncWebServerResponse *res = req->beginChunkedResponse(
		    "multipart/x-mixed-replace; boundary=" MJPEG_BOUNDARY,
		    [st](uint8_t *buf, size_t maxLen, size_t) -> size_t {
			    return jpeg_feed_fill(st, buf, maxLen);
		    });
		res->addHeader("Cache-Control", "no-store");
		req->send(res);
	});

	// Home network (netcfg.h). /netcfg/clear first: a plain "/netcfg" handler
	// also matches "/netcfg/..." (the server's backward-compatible prefix
	// rule), and handlers are tried in the order they are added.
	g_server.on("/netcfg/clear", HTTP_POST, [](AsyncWebServerRequest *req) {
		bool had = false;
		if (!netcfg_clear(&had)) {
			req->send(500, "text/plain", "Could not erase the saved IP (NVS error)");
			return;
		}
		deploy_net_changed();
		net_refresh();
		req->send(200, "text/plain",
		          (had ? "Forgot the saved IP; built-in " : "Nothing was saved; built-in ") +
		              net_addr(g_net.ip) + " is in use");
	});

	g_server.on("/netcfg", HTTP_GET, [](AsyncWebServerRequest *req) {
		netcfg_t b;
		netcfg_builtin(&b);
		deploy_status_t s;
		deploy_fill_status(&s);
		String j;
		j.reserve(256);
		j += "{\"src\":\"";       j += g_net_saved ? "saved" : "config.h";
		j += "\",\"ip\":\"";      j += net_addr(g_net.ip);
		j += "\",\"gw\":\"";      j += net_addr(g_net.gw);
		j += "\",\"sn\":\"";      j += net_addr(g_net.sn);
		j += "\",\"dns\":\"";     j += net_addr(g_net.dns);
		j += "\",\"builtin\":\""; j += net_addr(b.ip);
		j += "\",\"dhcp\":";      j += s.using_dhcp ? "true" : "false";
		j += "}";
		AsyncWebServerResponse *res = req->beginResponse(200, "application/json", j);
		res->addHeader("Cache-Control", "no-store");
		req->send(res);
	});

	// Checked here, not only in the page: four numbers 0-255 each, and the
	// rules in netcfg_check(). Empty optional fields take their defaults.
	g_server.on("/netcfg", HTTP_POST, [](AsyncWebServerRequest *req) {
		auto field = [req](const char *name) -> String {
			const AsyncWebParameter *p = req->getParam(name, true);
			String v = p ? p->value() : String();
			v.trim();
			return v;
		};
		const String ip = field("ip"), gw = field("gw"), dns = field("dns"), sn = field("sn");
		netcfg_t c = {};
		const char *err = nullptr;
		if (!ip.length()) {
			err = "IP address is required";
		} else if (!netcfg_parse_ip(ip.c_str(), &c.ip)) {
			err = "IP address: four numbers 0-255, like 192.168.1.50";
		} else if (gw.length() && !netcfg_parse_ip(gw.c_str(), &c.gw)) {
			err = "Gateway: four numbers 0-255, like 192.168.1.1";
		} else if (sn.length() && !netcfg_parse_ip(sn.c_str(), &c.sn)) {
			err = "Subnet: four numbers 0-255, like 255.255.255.0";
		} else if (dns.length() && !netcfg_parse_ip(dns.c_str(), &c.dns)) {
			err = "DNS: four numbers 0-255, like 192.168.1.1";
		} else {
			if (!gw.length()) {
				c.gw = netcfg_default_gw(c.ip);
			}
			if (!sn.length()) {
				c.sn = NETCFG_DEFAULT_SN;
			}
			if (!dns.length()) {
				c.dns = c.gw;
			}
			err = netcfg_check(c);
		}
		if (err) {
			req->send(400, "text/plain", err);
			return;
		}
		bool changed = false;
		if (!netcfg_save(c, &changed)) {
			req->send(500, "text/plain", "Could not save the IP (NVS error)");
			return;
		}
		deploy_net_changed();
		net_refresh();
		log_i("net: home address %s saved%s", ip.c_str(), changed ? "" : " (unchanged)");
		req->send(200, "text/plain",
		          "Saved " + net_addr(c.ip) + " (gw " + net_addr(c.gw) +
		              (changed ? "); used from the next wake" : "); it was already saved"));
	});

	g_server.on("/arm", HTTP_POST, [](AsyncWebServerRequest *req) {
		g_action    = ACTION_ARM;
		g_action_at = millis() + ACTION_DELAY_MS;
		req->send(200, "text/plain", "arming");
	});

	g_server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *req) {
		g_action    = ACTION_REBOOT;
		g_action_at = millis() + ACTION_DELAY_MS;
		req->send(200, "text/plain", "rebooting");
	});

	// Captive-portal probes (Android /generate_204, iOS /hotspot-detect.html,
	// Windows /connecttest.txt) all land here and get pointed at the page.
	g_server.onNotFound([](AsyncWebServerRequest *req) {
		AsyncWebServerResponse *res = req->beginResponse(302, "text/plain", "");
		res->addHeader("Location", "http://" + WiFi.softAPIP().toString() + "/");
		req->send(res);
	});
}

// ---------------------------------------------------------------------------
// Entry
// ---------------------------------------------------------------------------

// Power the sensor up in `mode`. Loop task only. camera_up() initialises it
// at CAM_FRAMESIZE, so the frame buffers fit a full still, and the preview is
// then a resize down (config.h, DEPLOY_PREVIEW_FRAMESIZE).
static bool cam_bring_up(cam_mode_t mode)
{
	if (!camera_up()) {
		return false;
	}
	sensor_t *s = esp_camera_sensor_get();
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	g_full_size = s->status.framesize;
	xSemaphoreGive(g_cam_lock);
	if (mode == CAM_PREVIEW && s->set_framesize(s, DEPLOY_PREVIEW_FRAMESIZE) != 0) {
		camera_down();
		return false;
	}
	// A cold sensor: no still until exposure has had the capture path's own
	// warm-up (config.h, CAM_WARMUP_MS).
	g_settle_until = millis() + CAM_WARMUP_MS;
	return true;
}

void deploy_mode_begin()
{
	g_cam_lock = xSemaphoreCreateMutex();
	net_refresh();

	// Bring the sensor up before the first request arrives, so the page has a
	// picture by the time it has finished loading. This also starts the idle
	// timer that powers it back down if nobody looks.
	const bool cam_ok = cam_bring_up(CAM_PREVIEW);
	g_cam_mode       = cam_ok ? CAM_PREVIEW : CAM_OFF;
	g_cam_request_ms = millis();
	if (!cam_ok) {
		log_e("deployment mode: camera failed to start; UI will retry");
	}

	WiFi.persistent(false);
	WiFi.mode(WIFI_AP);

	IPAddress ip(DEPLOY_AP_IP), gw(DEPLOY_AP_IP), sn(DEPLOY_AP_NETMASK);
	if (!WiFi.softAPConfig(ip, gw, sn)) {
		log_w("softAPConfig rejected; AP will use its default subnet");
	}
	if (!WiFi.softAP(DEPLOY_AP_SSID, DEPLOY_AP_PASSWORD, DEPLOY_AP_CHANNEL,
	                 /*ssid_hidden=*/0, DEPLOY_AP_MAX_CLIENTS)) {
		log_e("softAP(%s) failed", DEPLOY_AP_SSID);
	}
	// No power save: this mode is attended and mains- or bench-powered, and a
	// sleeping AP makes the preview stutter.
	WiFi.setSleep(false);

#if DEPLOY_CAPTIVE_DNS
	g_dns.setErrorReplyCode(DNSReplyCode::NoError);
	g_dns.start(53, "*", WiFi.softAPIP());
#endif

	install_routes();
	g_server.begin();

	log_i("deployment mode: AP \"%s\" up, http://%s/", DEPLOY_AP_SSID,
	      WiFi.softAPIP().toString().c_str());
}

// ---------------------------------------------------------------------------
// Service tick — owns camera power and size, stills, the cold-capture test,
// the presence walk test and deferred actions.
// ---------------------------------------------------------------------------
static void service_camera()
{
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	const uint32_t   now  = millis();
	const bool       idle = (now - g_cam_request_ms) >= (uint32_t)DEPLOY_CAM_IDLE_MS;
	const cam_mode_t on   = (now - g_full_request_ms) < (uint32_t)DEPLOY_FULL_HOLD_MS
	                            ? CAM_FULL : CAM_PREVIEW;
	const cam_mode_t from = g_cam_mode;
	cam_mode_t to = from;
	// Deciding and publishing under one lock is the whole point: cam_use_begin()
	// takes its use against g_cam_mode in the same critical section, so a
	// handler can never be mid-frame when the sensor is resized or torn down.
	if (from != CAM_OFF) {
		if (g_cam_users == 0) {
			to = idle ? CAM_OFF : on;
		}
	} else if (!idle && (int32_t)(now - g_cam_retry_at) >= 0) {
		to = on;
	}
	if (to != from) {
		g_cam_mode = CAM_SWITCHING;
	}
	xSemaphoreGive(g_cam_lock);
	if (to == from) {
		return;
	}

	cam_mode_t result = to;
	if (to == CAM_OFF) {
		camera_down();
		log_i("camera idle for %d ms, powered down", DEPLOY_CAM_IDLE_MS);
	} else if (from == CAM_OFF) {
		if (!cam_bring_up(to)) {
			result = CAM_OFF;
			g_cam_retry_at = millis() + DEPLOY_CAM_RETRY_MS;
			log_e("camera failed to come back up, retrying in %d ms",
			      DEPLOY_CAM_RETRY_MS);
		}
	} else {
		// A resize on a running sensor, within the buffers sized at init.
		sensor_t *s = esp_camera_sensor_get();
		const framesize_t size = to == CAM_FULL ? g_full_size : DEPLOY_PREVIEW_FRAMESIZE;
		if (s && s->set_framesize(s, size) == 0) {
			g_settle_until = millis() + DEPLOY_SWITCH_SETTLE_MS;
		} else {
			camera_down();
			result = CAM_OFF;
			g_cam_retry_at = millis() + DEPLOY_CAM_RETRY_MS;
			log_e("camera resize failed; powered down, retrying in %d ms",
			      DEPLOY_CAM_RETRY_MS);
		}
	}
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	g_cam_mode = result;
	xSemaphoreGive(g_cam_lock);
}

// Answer the /snapshot requests made so far with one full-size frame, from a
// frame that started after the newest of them and after the sensor settled.
// Blocks this task for up to a frame time per tick while one is pending.
static void service_still()
{
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	const uint32_t want    = g_still_want;
	const uint32_t req_ms  = g_still_req_ms;
	const bool     pending = want != g_still_done;
	const bool     ready   = g_cam_mode == CAM_FULL;
	xSemaphoreGive(g_cam_lock);
	if (!pending || !ready) {
		return;
	}

	camera_fb_t *fb = esp_camera_fb_get();
	blob_ref still;
	if (fb) {
		const uint32_t t = fb_start_ms(fb);
		if ((int32_t)(t - req_ms) < 0 || (int32_t)(t - g_settle_until) < 0) {
			esp_camera_fb_return(fb);
			return;   // too early; the next tick takes a newer one
		}
		still = blob_copy(fb->buf, fb->len);
		esp_camera_fb_return(fb);
	}
	if (!still) {
		log_w("full-size still failed (%s)", fb ? "no PSRAM for the copy" : "no frame");
	}

	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	if (still) {
		g_still = still;
		g_frames_served++;
		g_last_frame_len = still->len;
	}
	g_still_done   = want;
	g_still_failed = !still;
	xSemaphoreGive(g_cam_lock);
}

// Take the camera for a job that runs its own bring-up on this task (the
// cold capture, the test clip): no stream may hold a use. True, with the
// sensor powered down and the mode SWITCHING, once it is free. False while a
// stream still holds it; *refused once that has lasted as long as shutdown
// would wait. The page stops its stream first, so a holder is a response the
// async task has not torn down yet.
static bool job_take_camera(uint32_t req_ms, bool *refused)
{
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	const bool       free = g_cam_users == 0;
	const cam_mode_t from = g_cam_mode;
	if (free) {
		g_cam_mode = CAM_SWITCHING;
	}
	xSemaphoreGive(g_cam_lock);

	*refused = !free && millis() - req_ms >= SHUTDOWN_DRAIN_TICKS * SERVICE_TICK_MS;
	if (!free) {
		return false;
	}
	if (from != CAM_OFF) {
		camera_down();
	}
	return true;
}

// The cold-capture test: rail off for DEPLOY_COLDTEST_OFF_MS, then the
// presence wake's photo path (deploy_cold_test() in main.cpp). Blocks this task
// for the few seconds it takes; the captive DNS and the presence walk test wait.
static void service_coldtest()
{
	if (!g_cold_req) {
		return;
	}
	bool refused = false;
	if (!job_take_camera(g_cold_req_ms, &refused)) {
		if (refused) {
			g_cold_req = false;
			xSemaphoreTake(g_cam_lock, portMAX_DELAY);
			g_cold      = {};
			g_cold_busy = true;
			g_cold_jpeg.reset();
			g_cold_seq++;
			xSemaphoreGive(g_cam_lock);
			log_w("cold-capture test refused: a stream still holds the camera");
		}
		return;
	}
	g_cold_req = false;
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	g_cold_running = true;
	xSemaphoreGive(g_cam_lock);

	delay(DEPLOY_COLDTEST_OFF_MS);
	log_i("cold-capture test: rail off for %d ms, running the photo path",
	      DEPLOY_COLDTEST_OFF_MS);

	cold_test_t r;
	deploy_cold_test(&r);
	blob_ref jpeg;
	if (r.jpeg) {
		jpeg = blob_adopt(r.jpeg, r.jpeg_len);
		r.jpeg = nullptr;
	}

	// capture() has cut the rail again; service_camera() brings the preview
	// back if anything still wants it.
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	g_cold         = r;
	g_cold_busy    = false;
	g_cold_jpeg    = jpeg;
	g_cold_seq++;
	g_cold_running = false;
	g_cam_mode     = CAM_OFF;
	xSemaphoreGive(g_cam_lock);
}

// The test clip (deploy_test_clip() in main.cpp): DEPLOY_TEST_CLIP_S of the
// presence-video settings, a few seconds of this task like the cold test.
static void service_testclip()
{
	if (!g_clip_req) {
		return;
	}
	bool refused = false;
	if (!job_take_camera(g_clip_req_ms, &refused)) {
		if (refused) {
			g_clip_req = false;
			xSemaphoreTake(g_cam_lock, portMAX_DELAY);
			g_clip      = {};
			g_clip_busy = true;
			g_clip_avi.reset();
			g_clip_seq++;
			xSemaphoreGive(g_cam_lock);
			log_w("test clip refused: a stream still holds the camera");
		}
		return;
	}
	g_clip_req = false;
	// The last clip's buffer goes first: the new one is sized from what is
	// free (config.h, VIDEO_MAX_BYTES).
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	g_clip_running = true;
	g_clip_avi.reset();
	xSemaphoreGive(g_cam_lock);

	test_clip_t r;
	deploy_test_clip(&r);
	blob_ref avi;
	if (r.avi) {
		avi   = blob_adopt(r.avi, r.len);
		r.avi = nullptr;
	}

	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	g_clip         = r;
	g_clip_busy    = false;
	g_clip_avi     = avi;
	g_clip_seq++;
	g_clip_running = false;
	g_cam_mode     = CAM_OFF;
	xSemaphoreGive(g_cam_lock);
}

static void service_pir()
{
	const bool level = gpio_get_level(PIN_PIR) == 1;
	if (level && !g_pir_level) {
		g_pir_edges++;
		g_pir_last_edge_s = now_s();
		g_pir_rise_ms     = millis();
	} else if (!level && g_pir_level) {
		g_pir_high_ms = millis() - g_pir_rise_ms;
	}
	g_pir_level = level;
}

static void service_action()
{
	const uint8_t action = g_action;
	if (action == ACTION_NONE || (int32_t)(millis() - g_action_at) < 0) {
		return;
	}
	g_action = ACTION_NONE;

	g_server.end();
#if DEPLOY_CAPTIVE_DNS
	g_dns.stop();
#endif

	// A /stream still on the wire holds a camera use and may be sitting inside
	// esp_camera_fb_get() right now. g_server.end() drops the connection, but
	// the async task needs a few ticks to run the response destructor and give
	// the use back; deinit()ing the sensor before then faults.
	bool drained = false;
	for (int i = 0; i < SHUTDOWN_DRAIN_TICKS && !drained; i++) {
		xSemaphoreTake(g_cam_lock, portMAX_DELAY);
		drained = g_cam_users == 0;
		xSemaphoreGive(g_cam_lock);
		if (!drained) {
			delay(SERVICE_TICK_MS);
		}
	}

	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	const bool was_on = g_cam_mode != CAM_OFF;
	g_cam_mode = CAM_OFF;
	xSemaphoreGive(g_cam_lock);

	if (was_on && drained) {
		camera_down();
	} else if (was_on) {
		// Never drained. Skip the deinit rather than pull the driver out from
		// under the async task: both exits below reset the camera peripheral
		// anyway, and enter_deep_sleep() cuts the rail on its way out.
		log_w("camera still in use at shutdown, skipping deinit");
	}

	if (action == ACTION_REBOOT) {
		log_i("reboot requested over HTTP");
		Serial.flush();
		esp_restart();
	}

	log_i("armed over HTTP; entering the normal duty cycle");
	// PIR-era assumption, kept as it is: enter_deep_sleep() waits at most
	// PIR_IDLE_MAX_S for D1 to fall, then arms the active-high wake. An AM312
	// fell ~10 s after the operator stopped moving; the radar holds OT2 for as
	// long as the operator stays in range, so arming from beside the node
	// sleeps with D1 still high, wakes at once, and starts a visit: a photo of
	// the operator, which the detector will most likely pass, and a recording.
	enter_deep_sleep();   // does not return
}

void deploy_mode_service()
{
#if DEPLOY_CAPTIVE_DNS
	g_dns.processNextRequest();
#endif
	service_pir();
	service_coldtest();
	service_testclip();
	service_camera();
	service_still();
	service_action();
	delay(SERVICE_TICK_MS);
}
