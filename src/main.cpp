// Wildlife camera node — PIR-triggered capture with burst filtering and
// on-device human detection.
//
// Written against PROJECT_BRIEF.md. Section references below point at it.
//
// Cycle (9.1):
//   deep sleep -> ext0 wake on PIR -> power camera -> cold-init sensor ->
//   capture JPEG to PSRAM -> cut camera power -> decide -> maybe send ->
//   deep sleep.
//
// The capture is first and nothing is allowed in front of it, the radio least
// of all: bringing up esp_wifi blocks this task for tens of milliseconds, and
// the subject is walking. Association is started afterwards, once the frame is
// safe in PSRAM, and then overlaps whatever comes next — see setup().
//
// Burst filter (9.2): an isolated trigger is sent straight out; a burst is
// buffered and only sent if detection finds a human once the burst settles.
//
// Delivery (9.6): frames go to a Telegram chat as multipart/form-data over
// TLS. There is no local persistence — a frame that does not go out on the
// wake that captured it is dropped, because the only place to keep it would be
// a microSD on the ungated 3V3 rail and that costs ~1 mA of standing draw
// against a ~340 uA sleep budget (7).

#include <Arduino.h>
#include <WiFi.h>
#include <new>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <sys/time.h>

#include "config.h"
#include "deploy_mode.h"
#include "telegram.h"

#if DETECTION_ENABLED
#include "dl_image_jpeg.hpp"
#if DETECT_MODEL == DETECT_MODEL_PEDESTRIAN
#include "pedestrian_detect.hpp"
#else
#include "human_face_detect.hpp"
#endif
#endif

// ---------------------------------------------------------------------------
// Camera pinout — XIAO ESP32S3 Sense, OV5640 on the 24-pin FPC.
// PROJECT_BRIEF.md 8, VERIFIED from the schematic. Do not reassign.
// PWDN and RESET are left at -1 deliberately: PWDN (JA1 p8) is held low by
// R10 and RESET (JA1 p6) is pulled up by R9, neither is under our control
// (2, 7). Power sequencing is the AO3401A load switch on PIN_CAM_POWER.
// ---------------------------------------------------------------------------
#define CAM_PIN_PWDN  -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK  10
#define CAM_PIN_SIOD  40
#define CAM_PIN_SIOC  39
#define CAM_PIN_Y9    48
#define CAM_PIN_Y8    11
#define CAM_PIN_Y7    12
#define CAM_PIN_Y6    14
#define CAM_PIN_Y5    16
#define CAM_PIN_Y4    18
#define CAM_PIN_Y3    17
#define CAM_PIN_Y2    15
#define CAM_PIN_VSYNC 38
#define CAM_PIN_HREF  47
#define CAM_PIN_PCLK  13

// Give up waiting for the AM312 to drop rather than spin forever.
#define PIR_IDLE_MAX_S 30

// Empty slot marker for the trigger ring. Not 0 — now_s() legitimately returns
// 0 during the first second after a cold boot.
#define TRIGGER_SLOT_EMPTY 0xFFFFFFFFu

// "Never" for rtc_last_report_attempt_s. Not 0, for the same reason.
#define REPORT_NEVER 0xFFFFFFFFu

// Marks rtc_reset_stats as initialised; anything else in there is garbage.
#define RESET_STATS_MAGIC 0x52535431u   // "RST1"

// ---------------------------------------------------------------------------
// State that must survive deep sleep — RTC slow memory (9.2).
// ---------------------------------------------------------------------------
RTC_DATA_ATTR static bool     rtc_initialised = false;
RTC_DATA_ATTR static uint32_t rtc_trigger_log[TRIGGER_LOG_SIZE];
RTC_DATA_ATTR static uint8_t  rtc_trigger_head = 0;

RTC_DATA_ATTR static uint32_t rtc_triggers_total = 0;
RTC_DATA_ATTR static uint32_t rtc_triggers_since_report = 0;
RTC_DATA_ATTR static uint32_t rtc_suppressed_since_report = 0;
RTC_DATA_ATTR static uint32_t rtc_suppressed_total = 0;
RTC_DATA_ATTR static uint32_t rtc_last_report_s = 0;

// When a telemetry-carrying transmission was last *attempted*, delivered or
// not. The report schedule runs off this, not rtc_last_report_s, so an uplink
// outage costs one attempt per TELEMETRY_MAX_SILENCE_S instead of one per
// wake. rtc_last_report_s stays the last success, for the counters and the
// deployment-mode status page.
RTC_DATA_ATTR static uint32_t rtc_last_report_attempt_s = REPORT_NEVER;

// Cached association parameters (9.4) — skips the scan on every wake.
RTC_DATA_ATTR static bool     rtc_have_ap = false;
RTC_DATA_ATTR static uint8_t  rtc_bssid[6];
RTC_DATA_ATTR static uint8_t  rtc_channel = 0;

// Set once the static block in config.h has been proven wrong for this LAN, so
// later wakes stop paying WIFI_CONNECT_TIMEOUT_MS to rediscover that. Cleared
// again on any association failure — see wifi_wait_connected().
RTC_DATA_ATTR static bool     rtc_use_dhcp = false;

// Abnormal resets since power-on (see reset_was_abnormal()), for the report
// (9.3). This is the only way to find out whether the brownouts in 10 happen
// in the field. Not RTC_DATA_ATTR: that is reloaded by exactly the resets
// being counted. RTC_NOINIT_ATTR survives everything except a power-on, which
// is also when it holds garbage — hence the magic.
struct reset_stats_t {
	uint32_t magic;
	uint32_t brownout;
	uint32_t crash;      // panic and watchdog resets
};
RTC_NOINIT_ATTR static reset_stats_t rtc_reset_stats;

// ---------------------------------------------------------------------------
// Burst frame buffer.
//
// IMPORTANT: this lives in PSRAM, which loses its contents in deep sleep —
// esp_deep_sleep_start() powers down VDD_SPI unconditionally. 9.2 says to keep
// buffering while "sleeping between triggers", so for the duration of a burst
// only we use LIGHT sleep, which retains RAM and PSRAM. A burst costs roughly
// BURST_SETTLE seconds at light-sleep current (order 1 mA) instead of ~340 uA;
// at a few bursts a day that is well under 1% of the 9 mAh/day budget in 7.
// Everything outside a burst still uses deep sleep.
// ---------------------------------------------------------------------------
struct frame_t {
	uint8_t *data;
	size_t   len;
	uint32_t ts;
	float    score;
	bool     hit;
};

static frame_t g_burst[BURST_MAX_FRAMES];
static int     g_burst_n = 0;

// ---------------------------------------------------------------------------
// Monotonic seconds since first power-on.
//
// Deliberately NOT esp_timer_get_time(): that is backed by the high-resolution
// systimer, which lives in the digital domain and restarts from zero on every
// deep-sleep wake. BURST_WINDOW has to span deep sleeps, so it needs the RTC
// timer instead.
//
// gettimeofday() is backed by ESP-IDF system time, whose default configuration
// is "RTC and high-resolution timer": the RTC timer keeps counting through
// every sleep mode and through every reset except a power-on reset. Nothing
// here ever calls settimeofday(), so this is simply uptime-since-power-on
// counted from the epoch.
//
// A power-on reset (battery swap) zeroes this - and also clears RTC slow
// memory, so the trigger log resets with it. The two stay consistent.
// ---------------------------------------------------------------------------
uint32_t now_s()
{
	struct timeval tv;
	gettimeofday(&tv, nullptr);
	return (uint32_t)tv.tv_sec;
}

// ---------------------------------------------------------------------------
// Trigger bookkeeping
// ---------------------------------------------------------------------------
static void record_trigger(uint32_t t)
{
	rtc_trigger_log[rtc_trigger_head] = t;
	rtc_trigger_head = (rtc_trigger_head + 1) % TRIGGER_LOG_SIZE;
	rtc_triggers_total++;
	rtc_triggers_since_report++;
}

static int triggers_in_window(uint32_t t)
{
	int n = 0;
	for (int i = 0; i < TRIGGER_LOG_SIZE; i++) {
		uint32_t e = rtc_trigger_log[i];
		if (e != TRIGGER_SLOT_EMPTY && t >= e && (t - e) <= (uint32_t)BURST_WINDOW) {
			n++;
		}
	}
	return n;
}

// ---------------------------------------------------------------------------
// Camera power (5) and cold init (9.1 step 3)
// ---------------------------------------------------------------------------
static void camera_power_off()
{
	gpio_hold_dis(PIN_CAM_POWER);
	gpio_set_level(PIN_CAM_POWER, CAM_POWER_OFF_LEVEL);
	// Latch the low through deep sleep. Floating would also read as "off"
	// per 5, but latching removes any doubt about a floating pad partially
	// biasing the 2N3904.
	gpio_hold_en(PIN_CAM_POWER);
}

static void camera_power_on()
{
	gpio_hold_dis(PIN_CAM_POWER);
	gpio_set_level(PIN_CAM_POWER, CAM_POWER_ON_LEVEL);
	delay(CAM_LDO_SETTLE_MS);
}

// Park the camera interface whenever its rail is about to go, or has gone.
//
// esp32-camera leaves these pins set up for a live sensor. SCCB_Init() turns
// the internal pullups on for SIOD/SIOC, and the I2C driver's teardown in
// esp_camera_deinit() does not turn them off. The DVP pins stay floating
// inputs. XCLK stays an output: on the S3 the LCD_CAM peripheral generates
// it, not LEDC, so there is no ledc_stop() to park it low, and
// esp_camera_deinit() leaves the pin routed to that clock. With the OV5640
// unpowered, each of those feeds it through its pin clamps, the same leak
// path as the R9 leak 7 accepts. Then gpio_deep_sleep_hold_en() freezes the
// digital-only pads among them (IO38/39/40/47/48) in that state for the
// whole sleep.
//
// Parked means input and output off, pulldown on. esp_camera_init() sets all
// of them up again on the next wake: ll_cam_set_pin() restores the DVP pins
// as floating inputs and XCLK as an output, and the I2C driver re-enables the
// SCCB pullups and clears the pulldown.
static void camera_pins_quiesce()
{
	static const int pins[] = {
		CAM_PIN_SIOD, CAM_PIN_SIOC,
		CAM_PIN_Y2, CAM_PIN_Y3, CAM_PIN_Y4, CAM_PIN_Y5,
		CAM_PIN_Y6, CAM_PIN_Y7, CAM_PIN_Y8, CAM_PIN_Y9,
		CAM_PIN_VSYNC, CAM_PIN_HREF, CAM_PIN_PCLK, CAM_PIN_XCLK,
	};
	for (int pin : pins) {
		gpio_set_direction((gpio_num_t)pin, GPIO_MODE_DISABLE);
		gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLDOWN_ONLY);
	}
}

bool camera_up()
{
	camera_power_on();

	camera_config_t cfg = {};
	cfg.pin_pwdn     = CAM_PIN_PWDN;
	cfg.pin_reset    = CAM_PIN_RESET;
	cfg.pin_xclk     = CAM_PIN_XCLK;
	cfg.pin_sccb_sda = CAM_PIN_SIOD;
	cfg.pin_sccb_scl = CAM_PIN_SIOC;
	cfg.pin_d7       = CAM_PIN_Y9;
	cfg.pin_d6       = CAM_PIN_Y8;
	cfg.pin_d5       = CAM_PIN_Y7;
	cfg.pin_d4       = CAM_PIN_Y6;
	cfg.pin_d3       = CAM_PIN_Y5;
	cfg.pin_d2       = CAM_PIN_Y4;
	cfg.pin_d1       = CAM_PIN_Y3;
	cfg.pin_d0       = CAM_PIN_Y2;
	cfg.pin_vsync    = CAM_PIN_VSYNC;
	cfg.pin_href     = CAM_PIN_HREF;
	cfg.pin_pclk     = CAM_PIN_PCLK;
	cfg.xclk_freq_hz = CAM_XCLK_HZ;
	cfg.ledc_timer   = LEDC_TIMER_0;
	cfg.ledc_channel = LEDC_CHANNEL_0;
	cfg.pixel_format = PIXFORMAT_JPEG;
	cfg.frame_size   = CAM_FRAMESIZE;
	cfg.jpeg_quality = CAM_JPEG_QUALITY;
	cfg.fb_count     = 1;
	cfg.fb_location  = CAMERA_FB_IN_PSRAM;
	cfg.grab_mode    = CAMERA_GRAB_LATEST;

	esp_err_t err = esp_camera_init(&cfg);
	if (err != ESP_OK) {
		log_e("esp_camera_init failed: 0x%x", err);
		// A failed probe has already brought SCCB up and torn it down again,
		// which leaves the pullups on just as a clean deinit does.
		camera_pins_quiesce();
		camera_power_off();
		return false;
	}
	return true;
}

void camera_down()
{
	esp_camera_deinit();
	camera_pins_quiesce();
	camera_power_off();
}

// Capture one JPEG into PSRAM and cut camera power before returning (9.1
// step 5 — the camera must be off before anything slow happens).
static bool capture(frame_t *out)
{
	if (!camera_up()) {
		return false;
	}

	for (int i = 0; i < CAM_WARMUP_FRAMES; i++) {
		camera_fb_t *warm = esp_camera_fb_get();
		if (warm) {
			esp_camera_fb_return(warm);
		}
	}

	bool ok = false;
	camera_fb_t *fb = esp_camera_fb_get();
	if (fb && fb->len > 0) {
		uint8_t *copy = (uint8_t *)heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM);
		if (copy) {
			memcpy(copy, fb->buf, fb->len);
			out->data  = copy;
			out->len   = fb->len;
			out->ts    = now_s();
			out->score = 0.0f;
			out->hit   = false;
			ok = true;
		} else {
			log_e("PSRAM alloc failed for %u byte frame", (unsigned)fb->len);
		}
	}
	if (fb) {
		esp_camera_fb_return(fb);
	}

	camera_down();
	return ok;
}

// ---------------------------------------------------------------------------
// Burst buffer
// ---------------------------------------------------------------------------
static void buffer_push(const frame_t &f)
{
	if (g_burst_n == BURST_MAX_FRAMES) {
		// Drop the oldest. A long burst is junk by definition; the frames
		// nearest the settle point are the ones worth judging.
		heap_caps_free(g_burst[0].data);
		memmove(&g_burst[0], &g_burst[1], sizeof(frame_t) * (BURST_MAX_FRAMES - 1));
		g_burst_n--;
	}
	g_burst[g_burst_n++] = f;
}

static void buffer_clear()
{
	for (int i = 0; i < g_burst_n; i++) {
		heap_caps_free(g_burst[i].data);
		g_burst[i].data = nullptr;
	}
	g_burst_n = 0;
}

// ---------------------------------------------------------------------------
// Detection (9.2)
// ---------------------------------------------------------------------------
#if DETECTION_ENABLED

#if DETECT_MODEL == DETECT_MODEL_PEDESTRIAN
typedef PedestrianDetect detector_t;
static const char *DETECT_MODEL_NAME = "pedestrian";
#else
typedef HumanFaceDetect  detector_t;
static const char *DETECT_MODEL_NAME = "face";
#endif

// Returns the number of buffered frames containing a human, and tags each
// frame with its best score.
//
// `on_first_hit` is invoked the moment the first frame scores, and is how the
// radio gets started early without being started speculatively. The alternative
// — associating before detection runs — would light the radio for every burst,
// and the whole premise of 9.2 is that most bursts are wind in a branch and
// resolve to nothing. Firing on the first hit means the association overlaps
// the remaining frames exactly when there is going to be something to send,
// and never runs at all on a burst that gets suppressed.
static int detect_over_buffer(void (*on_first_hit)())
{
	if (g_burst_n == 0) {
		return 0;
	}

	uint32_t t0 = millis();
	detector_t *det = new (std::nothrow) detector_t();
	if (!det) {
		// Out of memory for the model. Fail open: a frame we could not judge
		// is better sent than silently dropped.
		log_e("detector alloc failed, sending burst unfiltered");
		for (int i = 0; i < g_burst_n; i++) {
			g_burst[i].hit = true;
		}
		if (on_first_hit) {
			on_first_hit();
		}
		return g_burst_n;
	}

	// Push the configured threshold into the model's own postprocessor (9.2:
	// "the detection-confidence threshold should be configurable"). Without
	// this the model filters at its compiled-in default first — 0.7 for
	// pedestrian, 0.5 for face — and a lower DETECT_SCORE_THRESHOLD would
	// silently do nothing, since the post-filter below can only tighten.
	det->set_score_thr(DETECT_SCORE_THRESHOLD);
#if DETECT_MODEL == DETECT_MODEL_FACE
	// The face model is two-stage (MSR then MNP); both stages need the threshold.
	det->set_score_thr(DETECT_SCORE_THRESHOLD, 1);
#endif

	int hits = 0;
	for (int i = 0; i < g_burst_n; i++) {
		dl::image::jpeg_img_t jpeg = {
			.data     = (void *)g_burst[i].data,
			.data_len = g_burst[i].len,
		};
		dl::image::img_t img = dl::image::sw_decode_jpeg(
			jpeg, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
		if (!img.data) {
			log_w("frame %d: jpeg decode failed", i);
			continue;
		}

		auto &results = det->run(img);
		for (const auto &r : results) {
			if (r.score > g_burst[i].score) {
				g_burst[i].score = r.score;
			}
		}
		heap_caps_free(img.data);

		if (g_burst[i].score >= DETECT_SCORE_THRESHOLD) {
			g_burst[i].hit = true;
			hits++;
			if (hits == 1 && on_first_hit) {
				// This burst is going out. Start the radio now so the
				// handshake runs against the remaining frames' inference
				// rather than after it.
				on_first_hit();
			}
		}
		log_i("frame %d: best score %.2f%s", i, g_burst[i].score,
		      g_burst[i].hit ? " HIT" : "");
	}

	delete det;
	log_i("%s detection over %d frames: %d hits in %lu ms",
	      DETECT_MODEL_NAME, g_burst_n, hits, (unsigned long)(millis() - t0));
	return hits;
}

#else  // !DETECTION_ENABLED

// bench-nodetect build (PROJECT_BRIEF.md 11 step 5): no esp-dl linked. Bursts
// are buffered and discarded so the power profile still matches the real
// firmware while the load switch is being validated.
//
// Nothing ever hits here, so on_first_hit is never called and this build never
// raises the radio for a burst — which is what keeps the sleep-current
// measurement that step 5 is after comparable to the real firmware's.
static int detect_over_buffer(void (*on_first_hit)())
{
	(void)on_first_hit;
	log_w("detection disabled at compile time; discarding %d buffered frames",
	      g_burst_n);
	return 0;
}

#endif // DETECTION_ENABLED

// ---------------------------------------------------------------------------
// Telemetry (9.3)
// ---------------------------------------------------------------------------
int32_t battery_mv()
{
#if BATT_SENSE_ENABLED
	analogSetPinAttenuation(PIN_BATT_SENSE, ADC_11db);
	uint32_t acc = 0;
	for (int i = 0; i < 8; i++) {
		acc += analogReadMilliVolts(PIN_BATT_SENSE);
	}
	return (int32_t)((acc / 8.0f) * BATT_DIVIDER_RATIO);
#else
	// No divider fitted yet — see config.h. Report unknown, never a guess.
	return -1;
#endif
}

// Copy the RTC counters out for the deployment-mode status page, so that code
// never reaches into this file's state directly (deploy_mode.h).
void deploy_fill_status(deploy_status_t *out)
{
	out->triggers_total          = rtc_triggers_total;
	out->triggers_since_report   = rtc_triggers_since_report;
	out->suppressed_total        = rtc_suppressed_total;
	out->suppressed_since_report = rtc_suppressed_since_report;
	out->last_report_s           = rtc_last_report_s;
	out->have_ap_cache           = rtc_have_ap;
	out->using_dhcp              = rtc_use_dhcp;
	out->ap_channel              = rtc_channel;
	memcpy(out->ap_bssid, rtc_bssid, sizeof(out->ap_bssid));
}

// ---------------------------------------------------------------------------
// WiFi (9.4) — static IP and a cached BSSID/channel on the fast path, with a
// DHCP fallback underneath it. config.h carries the reasoning for the
// fallback; this is the mechanism.
//
// Split into begin/wait rather than one blocking wifi_up() so that the
// association can overlap the work that follows the capture (9.1). That
// overlap is real and not bookkeeping: esp_wifi's driver runs in its own
// FreeRTOS tasks, CONFIG_FREERTOS_UNICORE is off on this target, and so the
// handshake proceeds on the other core while detection has this one.
// ---------------------------------------------------------------------------

// 0.0.0.0 — the value NetworkInterface::config() reads as "use DHCP".
//
// Deliberately NOT written as INADDR_NONE. That name is ambiguous in a
// translation unit holding both the Arduino core and lwIP: cores/esp32/
// IPAddress.h declares it as `const IPAddress(0, 0, 0, 0)`, while lwip/inet.h
// defines it as a macro equal to 0xffffffff, and nothing #undefs either. Which
// one is in scope depends on include order. config() reads the two as
// opposites — it starts the DHCP client on a zero local_ip and would install
// 255.255.255.255 as a static address — so the intent is spelled out here
// instead of left to the preprocessor.
static const IPAddress NET_ADDR_DHCP(0, 0, 0, 0);

static bool     s_wifi_pending   = false;  // begin() issued, outcome unknown
static bool     s_wifi_connected = false;
static bool     s_wifi_dhcp      = false;  // which phase is currently in flight
static uint32_t s_wifi_t0        = 0;

static void wifi_down()
{
	WiFi.disconnect(true, false);
	WiFi.mode(WIFI_OFF);
	s_wifi_pending   = false;
	s_wifi_connected = false;
}

static void wifi_ip_config(bool dhcp)
{
	if (dhcp) {
		// Clears the static configuration and starts the DHCP client.
		// Confirmed against NetworkInterface::config(): a zero local_ip stops
		// DHCPC, zeroes the netif's IP info, and then takes the
		// esp_netif_dhcpc_start() branch.
		WiFi.config(NET_ADDR_DHCP, NET_ADDR_DHCP, NET_ADDR_DHCP);
		return;
	}
	IPAddress ip(NET_STATIC_IP), gw(NET_GATEWAY), sn(NET_SUBNET), dns(NET_DNS);
	if (!WiFi.config(ip, gw, sn, dns)) {
		log_w("static IP config rejected; this attempt will use DHCP");
	}
}

// Kick off association and return without waiting. Idempotent, so the callers
// that start the radio early and the send path that needs it can both call it.
static void wifi_begin_async()
{
	if (s_wifi_pending || s_wifi_connected) {
		return;
	}
	// Credentials before the radio (9.6). A node built without secrets.h has
	// nowhere to send, so associating would be energy spent proving it.
	if (!telegram_configured()) {
		return;
	}

#if WIFI_REMEMBER_DHCP
	s_wifi_dhcp = rtc_use_dhcp;
#else
	s_wifi_dhcp = false;
#endif

	WiFi.persistent(false);
	WiFi.mode(WIFI_STA);
	WiFi.setSleep(WIFI_PS_NONE);
	wifi_ip_config(s_wifi_dhcp);

	if (rtc_have_ap) {
		WiFi.begin(WIFI_SSID, WIFI_PASSWORD, rtc_channel, rtc_bssid, true);
	} else {
		WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
	}

	s_wifi_t0      = millis();
	s_wifi_pending = true;
	log_i("wifi: associating (%s, %s)", s_wifi_dhcp ? "dhcp" : "static",
	      rtc_have_ap ? "cached ap" : "scan");
}

// WL_CONNECTED is raised on the GOT_IP event, not on association, so this
// waits for an actual address in both phases.
static bool wifi_await(uint32_t timeout_ms)
{
	while (WiFi.status() != WL_CONNECTED) {
		if (millis() - s_wifi_t0 > timeout_ms) {
			return false;
		}
		delay(5);
	}
	return true;
}

#if WIFI_VALIDATE_STATIC
// Is the address we just installed actually good for anything on this LAN?
//
// This exists because a static configuration cannot fail the way DHCP fails.
// There is no server to refuse us and no lease to not arrive: the core raises
// GOT_IP as soon as association completes, so WL_CONNECTED proves we are on
// the AP and proves nothing whatsoever about whether 192.168.1.50 means
// anything here. Without this probe a renumbered router turns the node into a
// brick that reports itself healthy on every wake and drops every frame at the
// TLS connect. See config.h.
static bool wifi_reachable()
{
	IPAddress resolved;
	const uint32_t t0 = millis();
	const bool ok = WiFi.hostByName(TELEGRAM_HOST, resolved) == 1;
	if (ok) {
		log_i("wifi: %s -> %s (%lu ms)", TELEGRAM_HOST,
		      resolved.toString().c_str(), (unsigned long)(millis() - t0));
	} else {
		log_w("wifi: cannot resolve %s via %s (%lu ms)", TELEGRAM_HOST,
		      WiFi.dnsIP().toString().c_str(),
		      (unsigned long)(millis() - t0));
	}
	return ok;
}
#else
static bool wifi_reachable() { return true; }
#endif

static void wifi_cache_ap()
{
	memcpy(rtc_bssid, WiFi.BSSID(), 6);
	rtc_channel = WiFi.channel();
	rtc_have_ap = true;
}

// Block until the association started by wifi_begin_async() has resolved, one
// way or the other, running the static -> DHCP fallback if it has to.
static bool wifi_wait_connected()
{
	if (s_wifi_connected) {
		return true;
	}
	if (!s_wifi_pending) {
		wifi_begin_async();
		if (!s_wifi_pending) {
			return false;   // no credentials; nothing was started
		}
	}

	const bool was_static = !s_wifi_dhcp;

	// Distinguishes the two ways phase 1 can fail, because they earn different
	// amounts of trust. A proven-bad static address is a standing fact about
	// this LAN and is worth remembering across wakes; a failed association is
	// weather, and remembering it would cost the 9.4 fast path for nothing.
	//
	// A failed lookup on the static address is only a suspicion, though: an
	// internet or DNS outage upstream of the router fails it too. It becomes
	// proof only if a DHCP lease then passes the same lookup (phase 2).
	bool static_suspect = false;

	if (wifi_await(was_static ? WIFI_CONNECT_TIMEOUT_MS : WIFI_DHCP_TIMEOUT_MS)) {
		if (wifi_reachable()) {
			wifi_cache_ap();
			s_wifi_connected = true;
			log_i("wifi up in %lu ms, ch %u, %s, ip %s",
			      (unsigned long)(millis() - s_wifi_t0), rtc_channel,
			      was_static ? "static" : "dhcp",
			      WiFi.localIP().toString().c_str());
			return true;
		}
		if (!was_static) {
			// A DHCP lease that cannot resolve TELEGRAM_HOST. The LAN handed
			// out a working address, so the outage is upstream of it: the
			// router's internet link or its DNS forwarder. Association
			// worked, so the AP is still worth caching, but there is nothing
			// to send to. Drop the radio now so the send paths drop their
			// frames at once, instead of each TLS connect waiting out its own
			// DNS timeout.
			log_w("wifi: %s does not resolve on a dhcp lease; uplink down",
			      TELEGRAM_HOST);
			wifi_cache_ap();
			wifi_down();
			return false;
		}
		// Associated, addressed, and the lookup failed. Either the static
		// block is wrong for this LAN, which is the case the fallback exists
		// for and the only one a timeout alone would never have caught, or
		// the uplink is down. Phase 2 tells the two apart.
		log_w("wifi: static address %s cannot resolve %s",
		      WiFi.localIP().toString().c_str(), TELEGRAM_HOST);
		static_suspect = true;
	} else {
		log_w("wifi: %s attempt timed out after %lu ms",
		      was_static ? "static" : "dhcp",
		      (unsigned long)(millis() - s_wifi_t0));
		// Association itself did not complete, which says nothing about
		// addressing. Two consequences:
		//   - the cached BSSID/channel is the likeliest culprit (the AP moved
		//     channel or went away), so drop it and let the retry scan;
		//   - do not let a radio-side failure strand the node on DHCP next
		//     wake, because DHCP was never the problem.
		const bool had_cache = rtc_have_ap;
		if (rtc_have_ap) {
			rtc_have_ap = false;
			log_w("wifi: ap cache invalidated");
		}
		rtc_use_dhcp = false;

		if (!had_cache) {
			// No cache to blame: this attempt was already a full scan of
			// every channel and still found nothing to associate with. The
			// AP is down, out of range, or the passphrase is wrong, and a
			// second identical scan would only confirm it — at eight more
			// seconds of radio on a 9 mAh/day budget (7), on every wake, for
			// as long as the outage lasts. Give up for this wake instead.
			s_wifi_pending = false;
			wifi_down();
			return false;
		}
	}

	s_wifi_pending = false;

#if WIFI_DHCP_TIMEOUT_MS > 0
	if (was_static) {
		// ---- Phase 2: full retry on DHCP ---------------------------------
		// More than "the same attempt with different addressing": if
		// association was what failed, the AP cache has just been dropped, so
		// this is a fresh scan as well. That covers both failures the first
		// phase cannot tell apart — a stale BSSID and a stale static block —
		// in the one extra attempt the budget in 7 will pay for.
		WiFi.disconnect(true, false);
		delay(10);

		s_wifi_dhcp = true;
		wifi_ip_config(true);

		if (rtc_have_ap) {
			WiFi.begin(WIFI_SSID, WIFI_PASSWORD, rtc_channel, rtc_bssid, true);
		} else {
			WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
		}
		s_wifi_t0      = millis();
		s_wifi_pending = true;
		log_i("wifi: retrying on dhcp");

		if (wifi_await(WIFI_DHCP_TIMEOUT_MS)) {
			wifi_cache_ap();
			if (!wifi_reachable()) {
				// Not on a lease either, so the outage is upstream, as in
				// phase 1, and says nothing against the static block. Do not
				// remember DHCP over it; give up on this wake.
				log_w("wifi: %s does not resolve on a dhcp lease either; "
				      "uplink down", TELEGRAM_HOST);
				wifi_down();
				return false;
			}
			s_wifi_connected = true;
#if WIFI_REMEMBER_DHCP
			// Skip the doomed static phase on later wakes — but only when the
			// static block is what was actually wrong: it failed the lookup
			// that this lease has just passed. If phase 1 merely failed to
			// associate, DHCP was incidental to the recovery and pinning the
			// node to it would throw away 9.4's fast path over a transient
			// radio problem.
			if (static_suspect) {
				rtc_use_dhcp = true;
			}
#endif
			log_i("wifi up on dhcp in %lu ms, ch %u, ip %s",
			      (unsigned long)(millis() - s_wifi_t0), rtc_channel,
			      WiFi.localIP().toString().c_str());
			return true;
		}
		log_w("wifi: dhcp fallback timed out too");
	}
#endif

	wifi_down();
	return false;
}

// Blocking convenience for the paths that have nothing to overlap.
static bool wifi_up()
{
	wifi_begin_async();
	return wifi_wait_connected();
}

// ---------------------------------------------------------------------------
// Telemetry (9.3)
//
// Carried as the photo caption rather than as query parameters, so a delivered
// frame is also a status report. A node that is sending photographs therefore
// needs no separate telemetry transmission at all.
// ---------------------------------------------------------------------------
static String telemetry_text(const char *reason, float score, int idx, int total)
{
	String c;
	c.reserve(256);

	// Empty unless secrets.h sets it. A string literal, so this folds away.
	if (TELEGRAM_CAPTION_PREFIX[0] != '\0') {
		c += TELEGRAM_CAPTION_PREFIX;
		c += '\n';
	}

	c += reason;
	if (total > 1) {
		c += " (";
		c += (idx + 1);
		c += '/';
		c += total;
		c += ')';
	}
	c += '\n';

	if (score > 0.0f) {
		c += "score ";
		c += String(score, 2);
		c += '\n';
	}

	c += "triggers ";
	c += rtc_triggers_total;
	c += " (+";
	c += rtc_triggers_since_report;
	c += " since report)\nsuppressed ";
	c += rtc_suppressed_total;
	c += " (+";
	c += rtc_suppressed_since_report;
	c += ")\nbattery ";

	const int32_t mv = battery_mv();
	if (mv < 0) {
		c += "n/a";   // no divider fitted — config.h, never a fabricated value
	} else {
		c += mv;
		c += " mV";
	}

	c += "\nresets: brownout ";
	c += rtc_reset_stats.brownout;
	c += ", crash ";
	c += rtc_reset_stats.crash;

	c += "\nuptime ";
	c += now_s();
	c += " s";
	return c;
}

// ---------------------------------------------------------------------------
// Sending (9.6) — Telegram sendPhoto, straight from PSRAM.
//
// There is no retry store. A frame that cannot be delivered on this wake is
// dropped, and the only trace of it is the trigger counters, which is exactly
// what makes maybe_send_telemetry_only() below worth keeping: it is the one
// mechanism that lets a node with a dead uplink still be noticed as alive.
// ---------------------------------------------------------------------------
static bool send_frames(const char *reason, bool only_hits)
{
	int total = 0;
	for (int i = 0; i < g_burst_n; i++) {
		if (!only_hits || g_burst[i].hit) {
			total++;
		}
	}
	if (total == 0) {
		return true;
	}

	// Credentials before the radio. A node built without secrets.h should go
	// straight back to sleep, not spend an association proving it cannot send.
	if (!telegram_configured()) {
		log_e("no Telegram credentials compiled in (include/secrets.h); "
		      "dropping %d frame(s)", total);
		return false;
	}

	rtc_last_report_attempt_s = now_s();   // every caption carries telemetry
	if (!wifi_up()) {
		log_w("no uplink; dropping %d frame(s)", total);
		return false;
	}

	int sent = 0, idx = 0;
	for (int i = 0; i < g_burst_n; i++) {
		if (only_hits && !g_burst[i].hit) {
			continue;
		}
		const int n = ++idx;
		const String cap = telemetry_text(reason, g_burst[i].score, n - 1, total);
		if (telegram_send_photo(g_burst[i].data, g_burst[i].len, cap)) {
			sent++;
		} else {
			log_w("frame %d/%d not delivered, dropped", n, total);
		}
	}
	wifi_down();

	if (sent > 0) {
		// Counters are cumulative "since last report" (9.3) — clear them only
		// once a report has actually landed somewhere.
		rtc_triggers_since_report   = 0;
		rtc_suppressed_since_report = 0;
		rtc_last_report_s           = now_s();
	}
	log_i("sent %d/%d frames", sent, total);
	return sent == total;
}

// Is a bare telemetry report owed right now? Split out of the function below
// because setup() needs the answer *before* detection runs: when a report is
// due the node is going to transmit whatever the detector decides, so the
// radio can be started up front rather than waiting on the first hit.
static bool telemetry_due()
{
	if (!telegram_configured()) {
		return false;
	}
	return rtc_last_report_attempt_s == REPORT_NEVER ||
	       (now_s() - rtc_last_report_attempt_s) >= (uint32_t)TELEMETRY_MAX_SILENCE_S;
}

// Seconds until the next report is owed, for the deep-sleep timer. Never less
// than TELEMETRY_WAKE_FLOOR_S, so a report that is already owed cannot turn
// into a wake loop.
static uint32_t telemetry_wake_in_s()
{
	uint32_t left = 0;
	if (rtc_last_report_attempt_s != REPORT_NEVER) {
		const uint32_t since = now_s() - rtc_last_report_attempt_s;
		if (since < (uint32_t)TELEMETRY_MAX_SILENCE_S) {
			left = (uint32_t)TELEMETRY_MAX_SILENCE_S - since;
		}
	}
	if (left < (uint32_t)TELEMETRY_WAKE_FLOOR_S) {
		left = TELEMETRY_WAKE_FLOOR_S;
	}
	return left;
}

// Report even when every frame was suppressed, so a node sitting in front of a
// windy branch still tells us its trigger rate (9.3).
static void maybe_send_telemetry_only()
{
	if (!telemetry_due()) {
		return;
	}
	rtc_last_report_attempt_s = now_s();
	if (!wifi_up()) {
		return;
	}

	if (telegram_send_message(telemetry_text("telemetry", 0.0f, 0, 0))) {
		rtc_triggers_since_report   = 0;
		rtc_suppressed_since_report = 0;
		rtc_last_report_s           = now_s();
	}
	wifi_down();
}

// ---------------------------------------------------------------------------
// Sleep
// ---------------------------------------------------------------------------
// The AM312 holds its output high for ~10 s (1). Arming an active-high level
// wake while it is still asserted would re-wake us immediately, so wait it out
// in light sleep first.
static void wait_for_pir_idle()
{
	uint32_t guard = 0;
	while (gpio_get_level(PIN_PIR) == 1 && guard < PIR_IDLE_MAX_S) {
		esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
		esp_sleep_enable_ext0_wakeup(PIN_PIR, 0);   // wake when it falls
		esp_sleep_enable_timer_wakeup(1000000ULL);
		const esp_err_t serr = esp_light_sleep_start();
		// Arming ext0 routed D1 to its RTC function, and nothing routes it
		// back after a light sleep. Until it is, the gpio_get_level() above
		// can read stale and end this wait with the AM312 still high; the
		// level-triggered deep-sleep wake then fires at once and one walk-by
		// logs as several triggers — exactly what the burst filter counts.
		rtc_gpio_deinit(PIN_PIR);
		if (serr != ESP_OK) {
			delay(1000);   // rejected; wait at CPU idle rather than spinning
		}
		guard++;
	}
}

// Never returns: ends in esp_deep_sleep_start(). Marked so the compiler
// enforces that no caller falls through into code using stale state.
[[noreturn]] void enter_deep_sleep()
{
	buffer_clear();
	camera_power_off();
	// Normally a repeat of camera_down(). Deployment mode's shutdown can get
	// here without it: when a stream will not let go of the sensor,
	// service_action() skips the deinit and leaves the rail to be cut above,
	// with XCLK still running and the SCCB pullups on. And a wake that never
	// ran the camera, a power-on for one, would otherwise latch these pads in
	// whatever state reset left them.
	camera_pins_quiesce();
	wifi_down();
	wait_for_pir_idle();

	// ext0 switches the pad to its RTC function, so its pulls must be set
	// through the RTC IO registers to survive deep sleep. Same rule as the
	// digital side in setup(): no pulldown unless PIR_INTERNAL_PULLDOWN.
#if PIR_INTERNAL_PULLDOWN
	rtc_gpio_pulldown_en(PIN_PIR);
#else
	rtc_gpio_pulldown_dis(PIN_PIR);
#endif
	rtc_gpio_pullup_dis(PIN_PIR);

	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
	esp_sleep_enable_ext0_wakeup(PIN_PIR, 1);   // AM312 is active-high (9.1)

	// Wake for the periodic report even if nothing moves. Without this a node
	// that sees no motion never reports, and a quiet node looks the same as a
	// mute one (9.6). The wake goes through setup()'s non-PIR branch. By
	// now_s() it can arrive slightly early, because the RC slow clock is
	// recalibrated between arming and waking. telemetry_due() then says no,
	// and the floor makes that one more short sleep, not a loop.
	if (telegram_configured()) {
		const uint32_t report_in_s = telemetry_wake_in_s();
		esp_sleep_enable_timer_wakeup((uint64_t)report_in_s * 1000000ULL);
		log_i("telemetry timer armed for %lu s", (unsigned long)report_in_s);
	}

	// Holds every digital-only pad (IO26-48) as it is now for the whole
	// sleep, which keeps the camera pins parked by camera_pins_quiesce().
	// PIN_CAM_POWER does not rely on this: GPIO1 is an RTC pad, and the
	// gpio_hold_en() in camera_power_off() takes the RTC hold path, which
	// holds it through deep sleep by itself.
	gpio_deep_sleep_hold_en();

	log_i("sleeping; %lu triggers total, %lu suppressed",
	      (unsigned long)rtc_triggers_total, (unsigned long)rtc_suppressed_total);
	Serial.flush();
	esp_deep_sleep_start();

	// esp_deep_sleep_start() does not return; this only satisfies [[noreturn]].
	for (;;) {
	}
}

// ---------------------------------------------------------------------------
// Burst handling (9.2)
// ---------------------------------------------------------------------------
// Stay resident in light sleep, absorbing further triggers, until the scene
// goes quiet for BURST_SETTLE seconds. Only then is detection worth its energy.
static void run_burst()
{
	const uint32_t burst_start = now_s();

	while (true) {
		wait_for_pir_idle();

		if (now_s() - burst_start >= (uint32_t)BURST_MAX_DURATION) {
			log_w("burst exceeded %d s, forcing settle", BURST_MAX_DURATION);
			break;
		}

		esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
		esp_sleep_enable_ext0_wakeup(PIN_PIR, 1);
		esp_sleep_enable_timer_wakeup((uint64_t)BURST_SETTLE * 1000000ULL);

		esp_err_t serr = esp_light_sleep_start();
		rtc_gpio_deinit(PIN_PIR);   // back to digital; see wait_for_pir_idle()
		esp_sleep_wakeup_cause_t wcause = esp_sleep_get_wakeup_cause();

		if (serr != ESP_OK) {
			// Sleep was rejected — typically a wakeup source already pending.
			// Never spin here: at ~250 mA a hot loop would eat days of budget
			// out of the cell before BURST_MAX_DURATION cut it off.
			log_w("light sleep rejected (%d), backing off", (int)serr);
			delay(1000);
			continue;
		}
		if (wcause == ESP_SLEEP_WAKEUP_TIMER) {
			break;   // quiet for BURST_SETTLE — the burst has settled
		}
		if (wcause != ESP_SLEEP_WAKEUP_EXT0) {
			delay(100);
			continue;   // not a PIR edge; do not count it as a trigger
		}

		record_trigger(now_s());
		frame_t f;
		if (capture(&f)) {
			buffer_push(f);
		}
		log_i("burst trigger, %d frames buffered", g_burst_n);
	}
}

// ---------------------------------------------------------------------------
// Deployment mode entry (config.h)
// ---------------------------------------------------------------------------
// True once BOOT has been held low continuously for DEPLOY_BUTTON_HOLD_MS.
// `window_ms` is how long to keep waiting for the press to *start*; a press
// already under way is always seen through to its verdict. window_ms == 0
// therefore costs one GPIO read when nothing is held, which is what the PIR
// path needs — 9.4 budgets 1.5 s for the whole wake-to-sent window — and what
// the telemetry timer's wake needs, since nobody is standing at the node then.
//
// Sampled here rather than at the reset edge because GPIO 0 is a strapping
// pin: held across reset it selects the ROM's download boot and this firmware
// never runs at all. config.h has the entry procedure.
static bool deploy_button_held(uint32_t window_ms)
{
	const uint32_t t0 = millis();
	uint32_t low_ms = 0;

	for (;;) {
		if (gpio_get_level(PIN_DEPLOY_BUTTON) == DEPLOY_BUTTON_ACTIVE) {
			if (low_ms >= (uint32_t)DEPLOY_BUTTON_HOLD_MS) {
				return true;
			}
			low_ms += DEPLOY_BUTTON_POLL_MS;
		} else {
			low_ms = 0;
			if (millis() - t0 >= window_ms) {
				return false;
			}
		}
		delay(DEPLOY_BUTTON_POLL_MS);
	}
}

// Set once deployment mode has taken over, so loop() knows it is live.
static bool s_deploy_mode = false;

// ---------------------------------------------------------------------------
// Abnormal resets (10)
// ---------------------------------------------------------------------------
// The resets that mean the node fell over rather than was restarted on
// purpose: a brownout, or a crash of some kind. ESP_RST_SW is deliberately not
// here. The only esp_restart() in this firmware is deployment mode's /reboot,
// which an operator asks for; nothing restarts on an error path.
static bool reset_was_abnormal(esp_reset_reason_t r)
{
	switch (r) {
	case ESP_RST_BROWNOUT:
	case ESP_RST_PANIC:
	case ESP_RST_INT_WDT:
	case ESP_RST_TASK_WDT:
	case ESP_RST_WDT:
		return true;
	default:
		return false;
	}
}

// Once per boot: count this boot's reset if it was abnormal. Brownouts get
// their own counter because they are what 10 is worried about; the rest of the
// list is a crash of some kind.
static void reset_stats_note(esp_reset_reason_t r)
{
	if (r == ESP_RST_POWERON || rtc_reset_stats.magic != RESET_STATS_MAGIC) {
		memset(&rtc_reset_stats, 0, sizeof(rtc_reset_stats));
		rtc_reset_stats.magic = RESET_STATS_MAGIC;
	}
	if (r == ESP_RST_BROWNOUT) {
		rtc_reset_stats.brownout++;
	} else if (reset_was_abnormal(r)) {
		rtc_reset_stats.crash++;
	}
}

// ---------------------------------------------------------------------------
// Wake deadline (config.h)
// ---------------------------------------------------------------------------
// Dispatched on the esp_timer task, which is pinned to core 0 at priority 22.
// This firmware runs on the Arduino loop task, core 1, priority 1, so nothing
// the loop task gets stuck in, blocked or spinning, can hold the abort off.
// Only something on core 0 could: interrupts left disabled, which the
// interrupt watchdog catches; a higher-priority task that never yields; or
// another esp_timer callback that never returns, since those run one at a
// time. ISR dispatch would get past the last, but it is not compiled into
// these builds' sdkconfig.
//
// A deadline that passes during one of the light sleeps in run_burst() or
// wait_for_pir_idle() fires as soon as the node wakes: esp_light_sleep_start()
// winds esp_timer's counter forward by the time slept, and the S3's systimer
// raises an alarm whose target is already behind the counter. Those sleeps
// last BURST_SETTLE at most.
static void wake_deadline_expired(void *)
{
	esp_system_abort("wake deadline");
}

// config.h works WAKE_DEADLINE_S out by hand. PIR_IDLE_MAX_S lives in this
// file, so this is where the sum can be checked.
static_assert(WAKE_DEADLINE_S > BURST_MAX_DURATION + BURST_SETTLE + 2 * PIR_IDLE_MAX_S +
                                (WIFI_CONNECT_TIMEOUT_MS + WIFI_DHCP_TIMEOUT_MS) / 1000 +
                                BURST_MAX_FRAMES * TELEGRAM_TIMEOUT_MS / 1000,
              "WAKE_DEADLINE_S no longer covers the longest normal wake (config.h)");

// Never stopped: every path after setup() arms it ends in deep sleep, which
// takes the timer down with everything else.
static void wake_deadline_arm()
{
	esp_timer_create_args_t args = {};
	args.callback        = wake_deadline_expired;
	args.dispatch_method = ESP_TIMER_TASK;
	args.name            = "wake_deadline";

	esp_timer_handle_t timer;
	if (esp_timer_create(&args, &timer) != ESP_OK ||
	    esp_timer_start_once(timer, (uint64_t)WAKE_DEADLINE_S * 1000000ULL) != ESP_OK) {
		log_e("wake deadline not armed; a hang will keep the node up");
	}
}

// ---------------------------------------------------------------------------
// setup(): the whole cycle. Control reaches loop() only in deployment mode.
// ---------------------------------------------------------------------------
void setup()
{
	Serial.begin(115200);

	// Camera off first, before anything else can take time (5: off is the
	// intended state at boot and reset).
	//
	// No gpio_reset_pin() on this pin: it turns the pad's ~45 k pullup on.
	// Driven low against that pullup the pin burns ~70 uA, and
	// camera_power_off()'s gpio_hold_en() latches the pullup into deep sleep,
	// a fifth of the budget in 7. Before the output is enabled the pullup
	// also briefly biases the 2N3904 on. Instead, the pad is reconfigured
	// while the RTC hold from the last sleep still pins it (gpio_config()
	// does not release that hold), and the hold is released last, so the pin
	// goes from the latched low straight to a driven low.
	gpio_deep_sleep_hold_dis();
	gpio_set_level(PIN_CAM_POWER, CAM_POWER_OFF_LEVEL);  // latch low first
	gpio_config_t cam_pwr = {};
	cam_pwr.pin_bit_mask = 1ULL << PIN_CAM_POWER;
	cam_pwr.mode         = GPIO_MODE_OUTPUT;
	cam_pwr.pull_up_en   = GPIO_PULLUP_DISABLE;
	cam_pwr.pull_down_en = GPIO_PULLDOWN_ENABLE;
	cam_pwr.intr_type    = GPIO_INTR_DISABLE;
	gpio_config(&cam_pwr);                               // no pullup, ever
	gpio_hold_dis(PIN_CAM_POWER);                        // hold released last

	// A deep-sleep ext0 wake leaves D1 on its RTC function too.
	// gpio_reset_pin() happens to undo that internally, but the hand-back is
	// the requirement (Espressif's sleep-mode docs), so it is explicit here.
	rtc_gpio_deinit(PIN_PIR);
	gpio_reset_pin(PIN_PIR);
	gpio_set_direction(PIN_PIR, GPIO_MODE_INPUT);
#if PIR_INTERNAL_PULLDOWN
	gpio_set_pull_mode(PIN_PIR, GPIO_PULLDOWN_ONLY);
#else
	// The AM312 drives D1 both ways; a pull would only divide its high
	// against R2 (config.h, PIR_INTERNAL_PULLDOWN).
	gpio_set_pull_mode(PIN_PIR, GPIO_FLOATING);
#endif

	// BOOT button. The XIAO already pulls GPIO 0 up externally, with a
	// capacitor on the line, and the button shorts it to ground. The internal
	// pullup is redundant but harmless; the delay lets the pad settle before
	// the first read.
	gpio_reset_pin(PIN_DEPLOY_BUTTON);
	gpio_set_direction(PIN_DEPLOY_BUTTON, GPIO_MODE_INPUT);
	gpio_set_pull_mode(PIN_DEPLOY_BUTTON, GPIO_PULLUP_ONLY);
	delay(1);

	reset_stats_note(esp_reset_reason());

	if (!rtc_initialised) {
		memset(rtc_trigger_log, 0xFF, sizeof(rtc_trigger_log));  // TRIGGER_SLOT_EMPTY
		memset(rtc_bssid, 0, sizeof(rtc_bssid));
		rtc_initialised = true;
		log_i("cold boot");
	}

	const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

	// Deployment mode, before anything else acts on the wake: a reset boot
	// would otherwise spend the entry window on the air sending telemetry.
	// PIR and telemetry-timer wakes only check for a press already held.
	if (deploy_button_held((cause == ESP_SLEEP_WAKEUP_EXT0 ||
	                        cause == ESP_SLEEP_WAKEUP_TIMER)
	                           ? 0u : (uint32_t)DEPLOY_ENTRY_WINDOW_MS)) {
		log_i("BOOT held — deployment mode, sleep bypassed");
		s_deploy_mode = true;
		deploy_mode_begin();
		return;   // setup() returns only here; loop() takes over
	}

	// Deployment mode has returned by now. Everything below ends in deep
	// sleep and runs against the wake deadline (config.h).
	wake_deadline_arm();

	if (cause != ESP_SLEEP_WAKEUP_EXT0) {
		// Power-on or reset, not a real trigger. Report once so a node that
		// has been reset in the field says so, then go to sleep. The
		// telemetry timer armed in enter_deep_sleep() lands here too, and
		// reports if one is due.
		log_i("wake cause %d (not PIR)", (int)cause);

		// Unless the reset was the node falling over. RTC_DATA_ATTR state is
		// reloaded on every reset except a deep-sleep wake, so after a
		// brownout, panic or watchdog the report looks overdue, and the first
		// thing the node would do is associate and handshake TLS. That is the
		// load that just browned it out (10), or possibly the code path that
		// just crashed. The result is a reset loop that runs until the DW01
		// cuts the cell off (6). Sleep instead, with the schedule pushed out
		// as if a report had just been tried: the next one goes out with the
		// next trigger's frame, or when the telemetry timer fires.
		const esp_reset_reason_t rst = esp_reset_reason();
		if (reset_was_abnormal(rst)) {
			log_w("abnormal reset (reason %d): sleeping without the radio",
			      (int)rst);
			rtc_last_report_attempt_s = now_s();
			enter_deep_sleep();
		}

		maybe_send_telemetry_only();
		enter_deep_sleep();
	}

	const uint32_t t_wake = now_s();
	record_trigger(t_wake);

	// ---- Capture first, unconditionally -----------------------------------
	// Nothing is allowed in front of this. The AM312 has already spent its own
	// detection latency getting us here and the subject is still walking, so
	// every millisecond between the wake and the shutter is scene that is gone.
	//
	// The radio in particular is NOT started ahead of the capture, tempting as
	// the overlap looks: esp_wifi_init() plus esp_wifi_start() is tens of
	// milliseconds of *blocking* setup on this task, and paying it here would
	// trade the photograph for the transport that carries it. The radio starts
	// below, once the frame is safe in PSRAM.
	frame_t f;
	if (!capture(&f)) {
		log_e("capture failed");
		enter_deep_sleep();
	}
	buffer_push(f);

	const int in_window = triggers_in_window(t_wake);
	log_i("trigger; %d in last %d s", in_window, BURST_WINDOW);

	if (in_window < BURST_COUNT) {
		// ISOLATED (9.2): send immediately, no detection. An animal or person
		// walking past trips the PIR once or twice — detection would cost
		// energy for nothing.
		//
		// This branch is a guaranteed transmission, so the radio can start
		// right now and associate underneath the credential check and caption
		// build in send_frames().
#if WIFI_EARLY_START
		wifi_begin_async();
#endif
		send_frames("isolated", /*only_hits=*/false);
		enter_deep_sleep();
	}

	// BURST (9.2): do not send yet. Keep buffering through light sleep until
	// the scene settles, then let detection decide.
	//
	// The radio stays off for the whole of run_burst(), and this is the one
	// place in the cycle where starting it early would be actively harmful: a
	// burst can hold us here for up to BURST_MAX_DURATION (600 s), and an
	// associated station cannot idle at light-sleep current — it has to keep
	// waking for its DTIM beacons. Ten minutes of that would outspend the
	// entire daily budget in 7.
	run_burst();

	// ---- Radio and inference in parallel ----------------------------------
	// Safe now: the burst has settled so nothing further will be captured, and
	// every frame is already in PSRAM. Association is a few hundred ms to a
	// couple of seconds; inference over up to BURST_MAX_FRAMES frames is the
	// longest single step in the whole cycle. Overlapping them makes the wake
	// cost about the larger of the two instead of their sum.
	//
	// Started up front only when a telemetry report is already owed, because
	// that is the one case where the transmission is certain regardless of
	// what the detector finds. Otherwise the radio waits for the first hit —
	// see detect_over_buffer().
#if WIFI_EARLY_START
	if (telemetry_due()) {
		wifi_begin_async();
	}
	const int buffered = g_burst_n;
	const int hits = detect_over_buffer(wifi_begin_async);
#else
	const int buffered = g_burst_n;
	const int hits = detect_over_buffer(nullptr);
#endif

	if (hits > 0) {
		// Counted first: send_frames() builds the captions from these and
		// then clears the since-report ones, so counting afterwards would
		// report this burst's suppressions with the next report instead.
		rtc_suppressed_since_report += (buffered - hits);
		rtc_suppressed_total        += (buffered - hits);
		send_frames("burst_detected", /*only_hits=*/true);
	} else {
		rtc_suppressed_since_report += buffered;
		rtc_suppressed_total        += buffered;
		log_i("burst of %d frames suppressed", buffered);
		maybe_send_telemetry_only();
	}

	enter_deep_sleep();
}

void loop()
{
	// Reached only in deployment mode; every other path ends setup() in
	// esp_deep_sleep_start().
	if (!s_deploy_mode) {
		delay(1000);
		return;
	}
	deploy_mode_service();
}
