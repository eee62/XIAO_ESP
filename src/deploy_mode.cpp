// Deployment mode — AP + async web UI for siting the node.
//
// Interface and entry conditions: include/deploy_mode.h and the deployment
// block in include/config.h.
//
// Threading: every request handler below runs on the AsyncTCP task. Camera
// *power* transitions and esp_camera_init()/deinit() happen only in
// deploy_mode_service(), i.e. on the Arduino loop task, exactly as they do in
// the normal duty cycle. Handlers may only pull frames, and only while holding
// a use-count taken under g_cam_lock — which is what stops the service task
// tearing the sensor down underneath an in-flight response.

#include "deploy_mode.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <esp_camera.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <memory>
#include <new>

// Before the conditional include below — it is what defines DEPLOY_CAPTIVE_DNS.
#include "config.h"
#include "telegram.h"

#if DEPLOY_CAPTIVE_DNS
#include <DNSServer.h>
#endif

#define MJPEG_BOUNDARY "wildlifeframe"

// How often deploy_mode_service() runs. Also the PIR sampling interval — the
// AM312 holds its output for ~10 s, so this is three orders of magnitude
// faster than it needs to be.
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
// ---------------------------------------------------------------------------
static SemaphoreHandle_t g_cam_lock;
static bool     g_cam_on          = false;
static int      g_cam_users       = 0;
static uint32_t g_cam_request_ms  = 0;
static uint32_t g_cam_retry_at    = 0;
static bool     g_stream_open     = false;   // async task only; single-threaded there

static uint32_t g_frames_served   = 0;
static size_t   g_last_frame_len  = 0;

// PIR walk test.
static bool     g_pir_level       = false;
static uint32_t g_pir_edges       = 0;
static uint32_t g_pir_last_edge_s = 0;

// Deferred actions requested over HTTP.
enum { ACTION_NONE = 0, ACTION_ARM, ACTION_REBOOT };
static volatile uint8_t  g_action    = ACTION_NONE;
static volatile uint32_t g_action_at = 0;

// Take a use on the camera. Fails while the sensor is down, which is the
// caller's cue to answer 503 and let the client retry — bringing it up here
// would mean running esp_camera_init() on the async task.
static bool cam_use_begin()
{
	bool ok = false;
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	g_cam_request_ms = millis();
	if (g_cam_on) {
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

// ---------------------------------------------------------------------------
// JPEG feed shared by /snapshot (one frame, known length) and /stream
// (multipart, open-ended).
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
	bool         multipart = false;
	bool         finished  = false;

	~jpeg_feed_t()
	{
		if (fb) {
			esp_camera_fb_return(fb);
		}
		if (multipart) {
			g_stream_open = false;
		}
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
		if (st->multipart) {
			st->hdr  = "\r\n--" MJPEG_BOUNDARY "\r\n"
			           "Content-Type: image/jpeg\r\n"
			           "Content-Length: ";
			st->hdr += st->fb->len;
			st->hdr += "\r\n\r\n";
		}
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
		if (!st->multipart) {
			st->finished = true;   // /snapshot is one frame and done
		}
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

static String status_json()
{
	deploy_status_t s;
	deploy_fill_status(&s);

	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	const bool cam_on = g_cam_on;
	const int  users  = g_cam_users;
	xSemaphoreGive(g_cam_lock);

	String j;
	j.reserve(768);
	j += "{\"uptime\":";        j += now_s();
	j += ",\"cam\":";           j += cam_on ? "true" : "false";
	j += ",\"cam_users\":";     j += users;
	j += ",\"frames\":";        j += g_frames_served;
	j += ",\"frame_len\":";     j += (uint32_t)g_last_frame_len;
	j += ",\"pir\":";           j += g_pir_level ? 1 : 0;
	j += ",\"pir_edges\":";     j += g_pir_edges;
	j += ",\"pir_last\":";      j += g_pir_last_edge_s;
	j += ",\"vbat_mv\":";       j += battery_mv();
	j += ",\"trig_total\":";    j += s.triggers_total;
	j += ",\"trig_since\":";    j += s.triggers_since_report;
	j += ",\"supp_total\":";    j += s.suppressed_total;
	j += ",\"supp_since\":";    j += s.suppressed_since_report;
	j += ",\"last_report\":";   j += s.last_report_s;
	j += ",\"ap_cache\":";      j += s.have_ap_cache ? "true" : "false";
	j += ",\"net_mode\":\"";     j += s.using_dhcp ? "dhcp" : "static";
	j += "\"";
	j += ",\"ap_ch\":";         j += s.ap_channel;
	j += ",\"clients\":";       j += WiFi.softAPgetStationNum();
	j += ",\"heap\":";          j += (uint32_t)ESP.getFreeHeap();
	j += ",\"heap_min\":";      j += (uint32_t)ESP.getMinFreeHeap();
	j += ",\"psram_free\":";    j += (uint32_t)ESP.getFreePsram();
	j += ",\"psram_size\":";    j += (uint32_t)ESP.getPsramSize();
	j += ",\"model\":\"" DEPLOY_MODEL_NAME "\"";
	j += ",\"thr\":";           j += String(DETECT_SCORE_THRESHOLD, 2);
	j += ",\"burst_n\":";       j += BURST_COUNT;
	j += ",\"burst_w\":";       j += BURST_WINDOW;
	j += ",\"burst_settle\":";  j += BURST_SETTLE;
	j += ",\"sta_ssid\":\"";  j += json_escape(WIFI_SSID);
	j += "\"";
	j += ",\"telegram\":";     j += telegram_configured() ? "true" : "false";
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
</style></head><body>

<header><h1>Wildlife node</h1><small id="conn">connecting…</small></header>

<section class="card">
  <div class="sec">Preview &mdash; aim the lens</div>
  <div class="view"><img id="view" alt=""><div id="vmsg">waiting for camera…</div></div>
  <div class="row">
    <button id="live" class="on">Live</button>
    <button id="snap">Snapshot</button>
  </div>
</section>

<section class="card pir">
  <div class="dot" id="pirdot"></div>
  <b id="pirtxt">PIR idle</b>
  <span id="pirsub">&mdash;</span>
</section>

<section class="card">
  <div class="sec">Status</div>
  <div class="grid" id="stat"></div>
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
var live=true, streaming=false, fails=0;

function toast(m){var t=$('toast');t.textContent=m;t.className='show';
  clearTimeout(toast.t);toast.t=setTimeout(function(){t.className=''},2200)}

function dur(s){if(s<0)return'-';var d=Math.floor(s/86400);s%=86400;
  var h=Math.floor(s/3600);s%=3600;var m=Math.floor(s/60);s%=60;
  if(d)return d+'d '+h+'h';if(h)return h+'h '+m+'m';if(m)return m+'m '+s+'s';
  return s+'s'}
function kb(n){return n<1024?n+' B':n<1048576?(n/1024).toFixed(0)+' kB':
  (n/1048576).toFixed(2)+' MB'}

function startStream(){if(streaming)return;streaming=true;
  var v=$('view');v.onload=function(){v.className='shown';$('vmsg').textContent=''};
  v.onerror=function(){streaming=false;v.className='';
    $('vmsg').textContent='stream dropped, retrying…'};
  v.src='/stream?'+Date.now()}
function stopStream(){streaming=false;var v=$('view');v.src='';v.className='';
  $('vmsg').textContent='camera off'}

$('live').onclick=function(){live=!live;this.className=live?'on':'';
  if(!live){stopStream();$('vmsg').textContent='preview paused'}}
$('snap').onclick=function(){live=false;$('live').className='';stopStream();
  var v=$('view');v.onload=function(){v.className='shown';$('vmsg').textContent=''};
  v.onerror=function(){$('vmsg').textContent='snapshot failed'};
  v.src='/snapshot?'+Date.now()}

function act(path,label,confirmText){
  if(!confirm(confirmText))return;
  live=false;$('live').className='';stopStream();
  fetch(path,{method:'POST'}).then(function(){toast(label)})
    .catch(function(){toast(label)})}
$('arm').onclick=function(){act('/arm','Arming — node will sleep',
  'Leave deployment mode and start the normal PIR duty cycle?')}
$('reboot').onclick=function(){act('/reboot','Rebooting',
  'Reboot the node? It will come back in normal mode.')}

var rows=[];
function row(k,v){rows.push('<div>'+k+'</div><div>'+v+'</div>')}

function render(d){
  $('conn').textContent=d.clients+' client'+(d.clients==1?'':'s')+
    ' · '+location.host;

  $('pirdot').className='dot'+(d.pir?' hot':'');
  $('pirtxt').textContent=d.pir?'PIR ACTIVE':'PIR idle';
  $('pirsub').innerHTML=d.pir_edges+' trip'+(d.pir_edges==1?'':'s')+
    ' this session<br>'+(d.pir_edges?('last '+dur(d.uptime-d.pir_last)+' ago'):
    'walk-test the field of view');

  rows=[];
  row('Uptime',dur(d.uptime));
  row('Battery',d.vbat_mv<0?'no divider fitted':(d.vbat_mv/1000).toFixed(2)+' V');
  row('Camera',d.cam?'on'+(d.cam_users?' · streaming':''):'powered down');
  row('Last frame',d.frames?kb(d.frame_len)+' · '+d.frames+' served':'—');
  row('Triggers',d.trig_total+' total · '+d.trig_since+' unreported');
  row('Suppressed',d.supp_total+' total · '+d.supp_since+' unreported');
  row('Last report',d.last_report?dur(d.uptime-d.last_report)+' ago':'never');
  row('Burst rule',d.burst_n+' in '+d.burst_w+'s, settle '+d.burst_settle+'s');
  row('Detection',d.model+(d.model=='disabled'?'':' @ '+d.thr));
  row('Uplink',d.sta_ssid+' → Telegram '+(d.telegram?'ready':'NOT CONFIGURED'));
  row('Network',d.net_mode);
  row('AP cache',d.ap_cache?'ch '+d.ap_ch:'none (full scan next)');
  row('Free heap',kb(d.heap)+' (min '+kb(d.heap_min)+')');
  row('Free PSRAM',kb(d.psram_free)+' / '+kb(d.psram_size));
  $('stat').innerHTML=rows.join('');

  if(d.cam&&live&&!streaming)startStream();
  if(!d.cam&&streaming)stopStream();
  if(!d.cam&&!streaming&&live)$('vmsg').textContent='camera warming up…';
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

	// One frame, with a real Content-Length so the browser can cache-bust it
	// and show a still while the aim is checked.
	g_server.on("/snapshot", HTTP_GET, [](AsyncWebServerRequest *req) {
		if (!cam_use_begin()) {
			req->send(503, "text/plain", "camera down");
			return;
		}
		camera_fb_t *fb = esp_camera_fb_get();
		if (!fb) {
			cam_use_end();
			req->send(503, "text/plain", "capture failed");
			return;
		}
		g_frames_served++;
		g_last_frame_len = fb->len;

		std::shared_ptr<jpeg_feed_t> st(new (std::nothrow) jpeg_feed_t());
		if (!st) {
			esp_camera_fb_return(fb);
			cam_use_end();
			req->send(503, "text/plain", "out of memory");
			return;
		}
		st->fb = fb;
		const size_t len = fb->len;

		AsyncWebServerResponse *res = req->beginResponse(
		    "image/jpeg", len,
		    [st](uint8_t *buf, size_t maxLen, size_t) -> size_t {
			    return jpeg_feed_fill(st, buf, maxLen);
		    });
		res->addHeader("Cache-Control", "no-store");
		req->send(res);
	});

	// Live MJPEG. One at a time: a second stream would halve the frame rate of
	// the first for no benefit, and both would be fighting one frame buffer.
	g_server.on("/stream", HTTP_GET, [](AsyncWebServerRequest *req) {
		if (g_stream_open) {
			req->send(503, "text/plain", "stream busy");
			return;
		}
		if (!cam_use_begin()) {
			req->send(503, "text/plain", "camera down");
			return;
		}
		std::shared_ptr<jpeg_feed_t> st(new (std::nothrow) jpeg_feed_t());
		if (!st) {
			cam_use_end();
			req->send(503, "text/plain", "out of memory");
			return;
		}
		st->multipart = true;
		g_stream_open = true;

		AsyncWebServerResponse *res = req->beginChunkedResponse(
		    "multipart/x-mixed-replace; boundary=" MJPEG_BOUNDARY,
		    [st](uint8_t *buf, size_t maxLen, size_t) -> size_t {
			    return jpeg_feed_fill(st, buf, maxLen);
		    });
		res->addHeader("Cache-Control", "no-store");
		req->send(res);
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
void deploy_mode_begin()
{
	g_cam_lock = xSemaphoreCreateMutex();

	// Bring the sensor up before the first request arrives, so the page has a
	// picture by the time it has finished loading. This also starts the idle
	// timer that powers it back down if nobody looks.
	g_cam_on         = camera_up();
	g_cam_request_ms = millis();
	if (!g_cam_on) {
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
// Service tick — owns camera power, the PIR walk test and deferred actions.
// ---------------------------------------------------------------------------
static void service_camera()
{
	xSemaphoreTake(g_cam_lock, portMAX_DELAY);
	const bool idle = (millis() - g_cam_request_ms) >= (uint32_t)DEPLOY_CAM_IDLE_MS;
	// Deciding and publishing under one lock is the whole point: cam_use_begin()
	// takes its use against g_cam_on in the same critical section, so a handler
	// can never be mid-frame when we decide to tear the sensor down.
	const bool down = g_cam_on && g_cam_users == 0 && idle;
	if (down) {
		g_cam_on = false;
	}
	const bool up = !g_cam_on && !down && !idle;
	xSemaphoreGive(g_cam_lock);

	if (down) {
		camera_down();
		log_i("preview idle for %d ms, camera powered down", DEPLOY_CAM_IDLE_MS);
	} else if (up && (int32_t)(millis() - g_cam_retry_at) >= 0) {
		const bool ok = camera_up();
		xSemaphoreTake(g_cam_lock, portMAX_DELAY);
		g_cam_on = ok;
		xSemaphoreGive(g_cam_lock);
		if (!ok) {
			g_cam_retry_at = millis() + DEPLOY_CAM_RETRY_MS;
			log_e("camera failed to come back up, retrying in %d ms",
			      DEPLOY_CAM_RETRY_MS);
		}
	}
}

static void service_pir()
{
	const bool level = gpio_get_level(PIN_PIR) == 1;
	if (level && !g_pir_level) {
		g_pir_edges++;
		g_pir_last_edge_s = now_s();
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
	const bool was_on = g_cam_on;
	g_cam_on = false;
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
	enter_deep_sleep();   // does not return
}

void deploy_mode_service()
{
#if DEPLOY_CAPTIVE_DNS
	g_dns.processNextRequest();
#endif
	service_pir();
	service_camera();
	service_action();
	delay(SERVICE_TICK_MS);
}
