// Wildlife camera node — presence-triggered capture, judged by on-device
// person detection before anything is sent, then video for as long as a
// confirmed visit lasts.
//
// Written against PROJECT_BRIEF.md. Section references below point at it. The
// sensor is an LD2410S radar (OT2 on PIN_PIR), not the brief's AM312 PIR; the
// PIR names here are that history (config.h, PIN_PIR).
//
// Cycle (9.1):
//   deep sleep -> ext0 wake on the presence line -> power camera -> cold-init
//   sensor -> capture JPEG to PSRAM -> cut camera power -> decide -> maybe send
//   -> (person confirmed: clips back to back while the radar shows presence)
//   -> deep sleep. 9.1 ends the wake after the photo; a confirmed visit keeps it
//   going, camera and radio still never on together (step 5, 10).
//
// The capture is first and nothing is allowed in front of it, the radio least
// of all: bringing up esp_wifi blocks this task for tens of milliseconds, and
// the subject is walking. Association is started afterwards, once the frame is
// safe in PSRAM and judged — see handle_trigger().
//
// Trigger flow (config.h; replaces 9.2's isolated/burst rule, deliberately):
// each presence episode takes ONE photo at its rising edge and detection
// judges it with the radio off. No person: nothing is sent or recorded for the
// whole episode. A person, or a detector that could not run (fail open): the
// photo goes out, then clips are recorded back to back for as long as the
// radar shows presence. Episodes are tracked in RTC memory across the deep
// sleeps between sensor edges; a branch in wind is held off by a backoff.
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

#include "avi.h"
#include "config.h"
#include "deploy_mode.h"
#include "netcfg.h"
#include "telegram.h"
#include "tg_updates.h"

#if DETECTION_ENABLED
#include "dl_image_define.hpp"
#include "esp_jpeg_dec.h"
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

// Give up waiting for the presence line to drop rather than spin forever. The
// radar holds OT2 for RADAR_UNMANNED_DELAY_S after the last presence, so the
// wait outlasts that by 20 s: 30 s at the radar's 10 s minimum, as it was
// against the AM312's ~10 s pulse. A radar retuned to a longer delay lengthens
// it to match, and the wake deadline's static_assert below checks the sum.
#define PIR_IDLE_MAX_S (RADAR_UNMANNED_DELAY_S + 20)

// "Never" for rtc_last_report_attempt_s. Not 0: now_s() legitimately returns
// 0 during the first second after a cold boot.
#define REPORT_NEVER 0xFFFFFFFFu

// Empty slot in the clip-time ring, for the same reason.
#define CLIP_SLOT_EMPTY 0xFFFFFFFFu

// Marks rtc_reset_stats as initialised; anything else in there is garbage.
#define RESET_STATS_MAGIC 0x52535431u   // "RST1"

// The reply window opens only where it can work (config.h, Reply window):
// REPLY_WAIT_S 0 turns it off, there is nothing to keep without video, and
// its requests use the address that WIFI_VALIDATE_STATIC's lookup resolves.
#define REPLY_WINDOW_BUILT (REPLY_WAIT_S > 0 && VIDEO_ENABLED && WIFI_VALIDATE_STATIC)

// ---------------------------------------------------------------------------
// State that must survive deep sleep — RTC slow memory (9.2).
// ---------------------------------------------------------------------------
RTC_DATA_ATTR static bool     rtc_initialised = false;

RTC_DATA_ATTR static uint32_t rtc_triggers_total = 0;
RTC_DATA_ATTR static uint32_t rtc_triggers_since_report = 0;
RTC_DATA_ATTR static uint32_t rtc_suppressed_since_report = 0;
RTC_DATA_ATTR static uint32_t rtc_suppressed_total = 0;
// Frames the detector could not judge (decode or allocation failure). Kept
// apart from "suppressed", which means judged and found empty: these are sent
// unfiltered instead (handle_trigger()).
RTC_DATA_ATTR static uint32_t rtc_detect_errors_total = 0;
RTC_DATA_ATTR static uint32_t rtc_detect_errors_since_report = 0;
// Photos delivered, and photos that were going out but could not be.
RTC_DATA_ATTR static uint32_t rtc_photos_sent_total = 0;
RTC_DATA_ATTR static uint32_t rtc_photos_sent_since_report = 0;
RTC_DATA_ATTR static uint32_t rtc_photos_dropped_total = 0;
RTC_DATA_ATTR static uint32_t rtc_photos_dropped_since_report = 0;
// Rising edges counted but not photographed: their episode already had its
// entry photo (PHOTOS_PER_EPISODE), so the verdict on it stands.
RTC_DATA_ATTR static uint32_t rtc_capped_total = 0;
RTC_DATA_ATTR static uint32_t rtc_capped_since_report = 0;
RTC_DATA_ATTR static uint32_t rtc_last_report_s = 0;

// When a telemetry-carrying transmission was last *attempted*, delivered or
// not. The report schedule runs off this, not rtc_last_report_s, so an uplink
// outage costs one attempt per TELEMETRY_MAX_SILENCE_S instead of one per
// wake. rtc_last_report_s stays the last success, for the counters and the
// deployment-mode status page.
RTC_DATA_ATTR static uint32_t rtc_last_report_attempt_s = REPORT_NEVER;

// JPEG quality number the next capture starts at: CAM_JPEG_QUALITY until a
// frame overflows the driver's buffer, then whatever worked (config.h,
// CAM_QUALITY_STEP).
RTC_DATA_ATTR static uint8_t  rtc_cam_quality = CAM_JPEG_QUALITY;

// Presence episode (config.h, PRESENCE_GAP_S). The node deep-sleeps between
// sensor edges inside an episode too, so this lives in RTC memory. The
// ext0 wake is armed for the opposite of d1_high, which is how a wake tells
// a rising edge from a falling one.
enum : uint8_t { EP_PERSON_UNCHECKED = 0, EP_PERSON_NO, EP_PERSON_YES };
struct episode_t {
	bool     open;
	bool     d1_high;       // D1 as last seen
	uint8_t  photos;        // taken, against PHOTOS_PER_EPISODE (sent or not)
	uint8_t  person;        // EP_PERSON_*: the entry photo's verdict
	bool     confirmed;     // the entry photo passed the gate: recording allowed
	bool     video_done;    // no more recording this episode
	uint16_t clips;         // recorded; no limit per visit, the day fuse bounds it
	uint64_t start_ms;      // now_ms() at the edge that opened it
	uint64_t fall_ms;       // now_ms() when D1 last fell
	char     tag[5];        // the visit's short id, #TAG in its captions
	uint8_t  reply;         // TG_REPLY_*: the answer, after the first clip
	bool     asked;         // the reply window ran for this visit
	bool     photo_ref;     // the entry photo's message_id and date are known
	int64_t  photo_msg_id;  // for the reply window, which runs after the
	int64_t  photo_date;    //   first clip, possibly on a later wake
};
RTC_DATA_ATTR static episode_t rtc_ep;

// Wind backoff (config.h). While rtc_backoff is set D1's wake is not armed at
// all and only the timer wakes the node, at rtc_backoff_end_s. rtc_listen_s
// is the last trigger, or the moment D1 was armed again after a backoff:
// WIND_QUIET_RESET_S counts from it.
RTC_DATA_ATTR static uint8_t  rtc_wind_streak     = 0;
RTC_DATA_ATTR static uint8_t  rtc_backoff_level   = 0;
RTC_DATA_ATTR static bool     rtc_backoff         = false;
RTC_DATA_ATTR static uint32_t rtc_backoff_start_s = 0;
RTC_DATA_ATTR static uint32_t rtc_backoff_end_s   = 0;
RTC_DATA_ATTR static uint32_t rtc_listen_s        = 0;
// Seconds the radar was ignored, for the report (9.3): the honest stand-in for
// the triggers that could not be counted meanwhile.
RTC_DATA_ATTR static uint32_t rtc_backoff_s_total = 0;
RTC_DATA_ATTR static uint32_t rtc_backoff_s_since_report = 0;

// Presence clips (config.h). Start times of the last VIDEO_MAX_CLIPS_PER_DAY,
// a ring, for the rolling 24 h fuse; CLIP_SLOT_EMPTY when unused. Clips sent,
// dropped (recorded but not delivered), and recorded milliseconds. The fuse
// trips are recordings it refused: "has it ever tripped" for the report.
RTC_DATA_ATTR static uint32_t rtc_clip_times[VIDEO_MAX_CLIPS_PER_DAY];
RTC_DATA_ATTR static uint8_t  rtc_clip_head = 0;
RTC_DATA_ATTR static uint32_t rtc_clips_sent_total = 0;
RTC_DATA_ATTR static uint32_t rtc_clips_sent_since_report = 0;
RTC_DATA_ATTR static uint32_t rtc_clips_dropped_total = 0;
RTC_DATA_ATTR static uint32_t rtc_clips_dropped_since_report = 0;
RTC_DATA_ATTR static uint32_t rtc_clip_ms_total = 0;
RTC_DATA_ATTR static uint32_t rtc_clip_ms_since_report = 0;
RTC_DATA_ATTR static uint32_t rtc_clip_fuse_trips_total = 0;

// The reply window (config.h): the highest Telegram update_id it has stepped
// past, so that nothing at or below it can count as an answer; -1 before the
// first window since power-on.
RTC_DATA_ATTR static int64_t  rtc_tg_floor_update = -1;
// What the windows came to, for the report (9.3): an answer of keep or stop,
// or none (the window closed, or could not run). Errors are the nones that
// were the node's fault or the link's rather than silence: a window that could
// not open after a delivered photo, or ended on a failed request. The usual
// none is nobody answering, and a node whose errors match its nones is not
// reading the chat at all (a webhook on the bot, say).
RTC_DATA_ATTR static uint32_t rtc_replies_keep_total = 0;
RTC_DATA_ATTR static uint32_t rtc_replies_keep_since_report = 0;
RTC_DATA_ATTR static uint32_t rtc_replies_stop_total = 0;
RTC_DATA_ATTR static uint32_t rtc_replies_stop_since_report = 0;
RTC_DATA_ATTR static uint32_t rtc_replies_none_total = 0;
RTC_DATA_ATTR static uint32_t rtc_replies_none_since_report = 0;
RTC_DATA_ATTR static uint32_t rtc_reply_errors_total = 0;
RTC_DATA_ATTR static uint32_t rtc_reply_errors_since_report = 0;

// Cached association parameters (9.4) — skips the scan on every wake.
RTC_DATA_ATTR static bool     rtc_have_ap = false;
RTC_DATA_ATTR static uint8_t  rtc_bssid[6];
RTC_DATA_ATTR static uint8_t  rtc_channel = 0;

// Set once the static block in config.h has been proven wrong for this LAN, so
// later wakes stop paying WIFI_CONNECT_TIMEOUT_MS to rediscover that. Cleared
// again on any association failure — see wifi_wait_connected().
RTC_DATA_ATTR static bool     rtc_use_dhcp = false;

// now_s() when a DHCP lease was last obtained, so WIFI_HOSTNAME last reached
// the router (config.h, HOSTNAME_REFRESH_INTERVAL_S); DHCP_REFRESH_NEVER
// since power-on.
#define DHCP_REFRESH_NEVER 0xFFFFFFFFu
RTC_DATA_ATTR static uint32_t rtc_last_dhcp_refresh_s = DHCP_REFRESH_NEVER;
// now_s() of the last refresh that got no lease, for HOSTNAME_REFRESH_RETRY_S;
// DHCP_REFRESH_NEVER when the last one did, or none has failed.
RTC_DATA_ATTR static uint32_t rtc_last_refresh_fail_s = DHCP_REFRESH_NEVER;

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
// One captured still, in PSRAM. PSRAM loses its contents in deep sleep
// (esp_deep_sleep_start() powers down VDD_SPI), so a frame lives exactly as
// long as the wake that took it: it is judged, sent or dropped, and freed.
// ---------------------------------------------------------------------------
struct frame_t {
	uint8_t *data;
	size_t   len;
	uint32_t ts;
	float    score;
	bool     hit;
	bool     err;    // detection could not run on it; sent unjudged
	uint32_t shutter_ms;   // capture()'s t_ref_ms to the frame's start
	uint8_t  quality;      // JPEG quality number it was taken at
};

// ---------------------------------------------------------------------------
// Monotonic seconds since first power-on.
//
// Deliberately NOT esp_timer_get_time(): that is backed by the high-resolution
// systimer, which lives in the digital domain and restarts from zero on every
// deep-sleep wake. Episodes and the wind backoff have to span deep sleeps, so
// they need the RTC timer instead.
//
// gettimeofday() is backed by ESP-IDF system time, whose default configuration
// is "RTC and high-resolution timer": the RTC timer keeps counting through
// every sleep mode and through every reset except a power-on reset. Nothing
// here ever calls settimeofday(), so this is simply uptime-since-power-on
// counted from the epoch.
//
// A power-on reset (battery swap) zeroes this - and also clears RTC slow
// memory, so the episode and backoff state reset with it. The two stay
// consistent.
// ---------------------------------------------------------------------------
uint32_t now_s()
{
	struct timeval tv;
	gettimeofday(&tv, nullptr);
	return (uint32_t)tv.tv_sec;
}

// The same clock in milliseconds, for episode timing.
static uint64_t now_ms()
{
	struct timeval tv;
	gettimeofday(&tv, nullptr);
	return (uint64_t)tv.tv_sec * 1000u + (uint64_t)tv.tv_usec / 1000u;
}

// ---------------------------------------------------------------------------
// Trigger bookkeeping
// ---------------------------------------------------------------------------
static void record_trigger()
{
	rtc_triggers_total++;
	rtc_triggers_since_report++;
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

// Sensor settings after every init (config.h). Each setter's own register
// writes are in ov5640.c; config.h says what each default reproduces. A
// failed write is logged and the capture goes ahead on the table values: a
// slightly different picture beats no picture.
static void camera_apply_settings()
{
	sensor_t *s = esp_camera_sensor_get();
	if (!s) {
		return;
	}
	// CAM_GAINCEILING is a raw OV5640 register value, and the other sensors
	// esp32-camera drives read these setters differently.
	if (s->id.PID != OV5640_PID) {
		log_w("sensor PID 0x%x is not an OV5640; settings left at defaults",
		      s->id.PID);
		return;
	}

	int err = 0;
	err |= s->set_vflip(s, CAM_VFLIP);
	err |= s->set_hmirror(s, CAM_HMIRROR);
	err |= s->set_lenc(s, CAM_LENC);
	err |= s->set_bpc(s, CAM_BPC);
	err |= s->set_wpc(s, CAM_WPC);
	err |= s->set_raw_gma(s, CAM_RAW_GMA);
#if CAM_AE_LEVEL != CAM_KEEP
	err |= s->set_ae_level(s, CAM_AE_LEVEL);
#endif
	err |= s->set_gainceiling(s, (gainceiling_t)CAM_GAINCEILING);
#if CAM_SHARPNESS != CAM_KEEP
	err |= s->set_sharpness(s, CAM_SHARPNESS);
#endif
	err |= s->set_denoise(s, CAM_DENOISE);
	err |= s->set_aec2(s, CAM_AEC2);
	if (err) {
		log_w("one or more sensor settings failed to apply");
	}
}

// Power up and cold-init the sensor at `size` and `quality`: stills use
// camera_up(), the presence clips VIDEO_FRAMESIZE / VIDEO_JPEG_QUALITY.
static bool camera_up_as(framesize_t size, int quality)
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
	cfg.frame_size   = size;
	cfg.jpeg_quality = quality;
	// Two buffers, not one, so a discarded warm-up frame does not also cost
	// the frame after it. In the installed cam_hal.c a JPEG frame ends at the
	// VSYNC that starts the next one; cam_task() then queues it and looks for
	// a free buffer to start on. With fb_count = 1 there is none, because the
	// only one is sitting in the queue, so that next frame is skipped and
	// capture resumes only at the VSYNC after esp_camera_fb_return(): every
	// fb_get() after a return waits two frame times. With two buffers and
	// GRAB_LATEST (a one-deep queue whose newest frame replaces the older)
	// the sensor streams back to back and fb_get() hands over the latest
	// finished frame. The second buffer costs one more w x h / 5 of PSRAM.
	cfg.fb_count     = 2;
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
	camera_apply_settings();
	return true;
}

bool camera_up()
{
	return camera_up_as(CAM_FRAMESIZE, rtc_cam_quality);
}

void camera_down()
{
	esp_camera_deinit();
	camera_pins_quiesce();
	camera_power_off();
}

// When the frame started, on the millis() clock. cam_hal.c stamps each buffer
// with esp_timer_get_time() at the VSYNC it starts on, and millis() reads the
// same timer.
static uint32_t fb_start_ms(const camera_fb_t *fb)
{
	return (uint32_t)(fb->timestamp.tv_sec * 1000 + fb->timestamp.tv_usec / 1000);
}

// Capture one JPEG into PSRAM and cut camera power before returning (9.1
// step 5 — the camera must be off before anything slow happens).
//
// `t_ref_ms` is the millis() the wake-to-shutter log counts from: setup()
// entry for the wake's own trigger, the light-sleep wake for a burst trigger.
static bool capture(frame_t *out, uint32_t t_ref_ms)
{
	if (!camera_up()) {
		return false;
	}

	// Timed warm-up (config.h). The test is on each frame's start, not on
	// when fb_get() returned it: with GRAB_LATEST the frame handed over can
	// have begun a frame time earlier, and it is the exposure that has to
	// fall after the deadline. Signed, because the first frames can start
	// before t_init: streaming begins inside esp_camera_init().
	const uint32_t t_init = millis();
	int  q = rtc_cam_quality;
	bool raised = false;
	int  discarded = 0;
	camera_fb_t *fb;
	for (;;) {
		fb = esp_camera_fb_get();
		if (!fb) {
			// Most likely a JPEG that outgrew the frame buffer (config.h,
			// CAM_QUALITY_STEP). A coarser quality, once; the sensor takes it
			// while streaming, and the warm-up carries on where it was.
			sensor_t *s = esp_camera_sensor_get();
			const int q2 = q + CAM_QUALITY_STEP > 63 ? 63 : q + CAM_QUALITY_STEP;
			if (raised || q2 == q || !s || s->set_quality(s, q2) != 0) {
				break;
			}
			log_w("no frame at JPEG quality %d; retrying once at %d", q, q2);
			q = q2;
			raised = true;
			continue;
		}
		if (discarded >= CAM_WARMUP_MIN_FRAMES &&
		    (int32_t)(fb_start_ms(fb) - t_init) >= CAM_WARMUP_MS) {
			break;
		}
		esp_camera_fb_return(fb);
		discarded++;
	}
	if (fb) {
		log_i("wake-to-shutter %lu ms (frame start; %d warm-up frames over "
		      "%lu ms), %u bytes at quality %d",
		      (unsigned long)(fb_start_ms(fb) - t_ref_ms), discarded,
		      (unsigned long)(fb_start_ms(fb) - t_init), (unsigned)fb->len, q);

		// What the next wake starts at. A raised value stands; otherwise a
		// frame well inside the buffer earns one step back toward
		// CAM_JPEG_QUALITY. The buffer is w x h / 5, cam_hal.c's own
		// FRAME_SIZE_AUTO formula, since camera_fb_t does not carry it.
		const size_t cap = (size_t)fb->width * fb->height / 5;
		int next = q;
		if (!raised && q > CAM_JPEG_QUALITY &&
		    fb->len < cap * CAM_QUALITY_RELAX_PCT / 100) {
			next = q - CAM_QUALITY_STEP < CAM_JPEG_QUALITY
			           ? CAM_JPEG_QUALITY : q - CAM_QUALITY_STEP;
		}
		if (next != rtc_cam_quality) {
			log_i("JPEG quality for the next capture: %d -> %d",
			      rtc_cam_quality, next);
			rtc_cam_quality = (uint8_t)next;
		}
	} else {
		log_e("fb_get failed after %d warm-up frames, quality %d", discarded, q);
		// Nothing worked, but the next wake may as well start from the
		// coarser setting, and raise again from there if it has to.
		if (raised) {
			rtc_cam_quality = (uint8_t)q;
		}
	}

	bool ok = false;
	if (fb && fb->len > 0) {
		uint8_t *copy = (uint8_t *)heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM);
		if (copy) {
			memcpy(copy, fb->buf, fb->len);
			out->data  = copy;
			out->len   = fb->len;
			out->ts    = now_s();
			out->score = 0.0f;
			out->hit   = false;
			out->err   = false;
			out->shutter_ms = fb_start_ms(fb) - t_ref_ms;
			out->quality    = (uint8_t)q;
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

// Decode one buffered JPEG to RGB888 for the detector, at the smallest
// power-of-two reduction that brings its width to DETECT_DECODE_MAX_W or less
// (config.h). The caller frees img.data with heap_caps_free(); on any failure
// it is nullptr.
//
// esp_new_jpeg is called directly because esp-dl 3.3.12's sw_decode_jpeg()
// is a thin wrapper over it that always opens the decoder with scale {0, 0}:
// it can only decode at full size. The decoder itself takes an output scale in
// its open-time config (multiples of 8, down to 1/8). The other candidate,
// esp32-camera's jpg2rgb565() with a jpg_scale_t, goes through the prebuilt
// esp_jpeg (tjpgd) with no byte-swap option set, and its RGB565 byte order
// cannot be checked against esp-dl's RGB565LE/BE types from the installed
// sources; RGB888 also saves the detector a colour conversion.
//
// The result needs no further resizing here: the model's ImagePreprocessor
// warps any RGB888 img_t to its input tensor's size, and the postprocessor
// scales boxes back to the img_t it was given.
static dl::image::img_t decode_for_detect(const frame_t &f)
{
	dl::image::img_t img = {};
	img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

	// The scale is fixed when the decoder is opened, so the source size has
	// to come from a first, unscaled open.
	jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();   // RGB888, no scale
	jpeg_dec_handle_t dec = nullptr;
	jpeg_dec_io_t io = {};
	jpeg_dec_header_info_t info = {};
	io.inbuf     = f.data;
	io.inbuf_len = (int)f.len;
	if (jpeg_dec_open(&cfg, &dec) != JPEG_ERR_OK) {
		return img;
	}
	const jpeg_error_t herr = jpeg_dec_parse_header(dec, &io, &info);
	jpeg_dec_close(dec);
	if (herr != JPEG_ERR_OK) {
		return img;
	}

	int shift = 0;
	while (shift < 3 && (info.width >> shift) > DETECT_DECODE_MAX_W) {
		shift++;
	}
	uint16_t w = info.width, h = info.height;
	if (shift > 0) {
		w = (info.width >> shift) & ~7u;
		h = (info.height >> shift) & ~7u;
		cfg.scale.width  = w;
		cfg.scale.height = h;
	}

	io = {};
	io.inbuf     = f.data;
	io.inbuf_len = (int)f.len;
	if (jpeg_dec_open(&cfg, &dec) != JPEG_ERR_OK) {
		return img;
	}
	int out_len = 0;
	if (jpeg_dec_parse_header(dec, &io, &info) == JPEG_ERR_OK &&
	    jpeg_dec_get_outbuf_len(dec, &out_len) == JPEG_ERR_OK &&
	    out_len >= (int)w * h * 3) {
		// 16-byte aligned, as jpeg_dec_process() requires on the S3. Sized
		// by the decoder's own figure, which may round up past w x h x 3.
		void *buf = heap_caps_aligned_alloc(16, out_len, MALLOC_CAP_SPIRAM);
		if (buf) {
			io.outbuf = (uint8_t *)buf;
			if (jpeg_dec_process(dec, &io) == JPEG_ERR_OK) {
				img.data   = buf;
				img.width  = w;
				img.height = h;
			} else {
				heap_caps_free(buf);
			}
		}
	}
	jpeg_dec_close(dec);
	return img;
}

// Construct the detector with the configured threshold and load its model.
// nullptr if there is no memory for it. *load_ms is the whole of that.
//
// Both models load lazily inside their first run() by default, which would
// bill the load to the first frame's inference time. get_raw_model() loads
// it here instead, so the per-frame numbers mean what they say.
static detector_t *detector_open(uint32_t *load_ms)
{
	const uint32_t t0 = millis();
	detector_t *det = new (std::nothrow) detector_t();
	if (!det) {
		return nullptr;
	}
	// Push the configured threshold into the model's own postprocessor (9.2:
	// "the detection-confidence threshold should be configurable"). Without
	// this the model filters at its compiled-in default first — 0.7 for
	// pedestrian, 0.5 for face — and a lower DETECT_SCORE_THRESHOLD would
	// silently do nothing, since the post-filter can only tighten.
	det->set_score_thr(DETECT_SCORE_THRESHOLD);
#if DETECT_MODEL == DETECT_MODEL_FACE
	// The face model is two-stage (MSR then MNP); both stages need the threshold.
	det->set_score_thr(DETECT_SCORE_THRESHOLD, 1);
#endif
	det->get_raw_model();
	*load_ms = millis() - t0;
	return det;
}

// Judge one frame: sets f.score and f.hit, or f.err when it cannot be
// decoded. Returns false on the error. Decode and inference are timed apart,
// since both sit on the path of every photo.
static bool detect_frame(detector_t *det, frame_t &f, uint32_t *dec_ms,
                         uint32_t *inf_ms)
{
	const uint32_t t_dec = millis();
	dl::image::img_t img = decode_for_detect(f);
	*dec_ms = millis() - t_dec;
	*inf_ms = 0;
	if (!img.data) {
		f.err = true;
		return false;
	}

	const uint32_t t_inf = millis();
	auto &results = det->run(img);
	for (const auto &r : results) {
		if (r.score > f.score) {
			f.score = r.score;
		}
	}
	*inf_ms = millis() - t_inf;
	heap_caps_free(img.data);
	f.hit = f.score >= DETECT_SCORE_THRESHOLD;
	log_i("%ux%u decode %lu ms, inference %lu ms, best score %.2f%s",
	      img.width, img.height, (unsigned long)*dec_ms, (unsigned long)*inf_ms,
	      f.score, f.hit ? " HIT" : "");
	return true;
}

// Judge one photo, radio off: model load, decode and inference, each timed.
// Sets f.score and f.hit, or f.err when it could not be judged. Returns false
// on that error.
static bool judge_frame(frame_t &f)
{
	const uint32_t t0 = millis();
	uint32_t load_ms = 0, dec_ms = 0, inf_ms = 0;
	detector_t *det = detector_open(&load_ms);
	if (!det) {
		log_e("detector alloc failed; the photo goes out unjudged");
		f.err = true;
		return false;
	}
	const bool ok = detect_frame(det, f, &dec_ms, &inf_ms);
	delete det;
	log_i("%s: load %lu ms, decode %lu ms, inference %lu ms, %lu ms in all%s",
	      DETECT_MODEL_NAME, (unsigned long)load_ms, (unsigned long)dec_ms,
	      (unsigned long)inf_ms, (unsigned long)(millis() - t0),
	      ok ? "" : " (decode failed)");
	return ok;
}

#else  // !DETECTION_ENABLED

// bench-nodetect build (PROJECT_BRIEF.md 11 step 5): no esp-dl linked, so no
// photo is ever judged. The entry photo goes out unfiltered and the episode
// records on presence alone, with no AI gate (config.h, Trigger flow).

#endif // DETECTION_ENABLED

// ---------------------------------------------------------------------------
// Deployment mode's cold-capture test (deploy_mode.h). The same capture() and
// detection a presence wake runs, so what it reports is what the field gets, minus
// the boot before setup() that a real wake also pays.
// ---------------------------------------------------------------------------
void deploy_cold_test(cold_test_t *out)
{
	*out = {};
	const uint32_t t0 = millis();
	frame_t f = {};
	const bool ok = capture(&f, t0);
	out->capture_ms = millis() - t0;
	if (!ok) {
		return;
	}
	out->ok                 = true;
	out->wake_to_shutter_ms = f.shutter_ms;
	out->quality            = f.quality;
	out->jpeg               = f.data;
	out->jpeg_len           = f.len;

#if DETECTION_ENABLED
	detector_t *det = detector_open(&out->load_ms);
	if (!det) {
		out->detect = COLD_DETECT_ERROR;
		return;
	}
	if (detect_frame(det, f, &out->decode_ms, &out->infer_ms)) {
		out->detect = f.hit ? COLD_DETECT_HIT : COLD_DETECT_NONE;
		out->score  = f.score;
	} else {
		out->detect = COLD_DETECT_ERROR;
	}
	delete det;
#else
	out->detect = COLD_DETECT_OFF;
#endif
}

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
	out->triggers_total             = rtc_triggers_total;
	out->triggers_since_report      = rtc_triggers_since_report;
	out->suppressed_total           = rtc_suppressed_total;
	out->suppressed_since_report    = rtc_suppressed_since_report;
	out->detect_errors_total        = rtc_detect_errors_total;
	out->detect_errors_since_report = rtc_detect_errors_since_report;
	out->photos_sent_total          = rtc_photos_sent_total;
	out->photos_sent_since_report   = rtc_photos_sent_since_report;
	out->photos_dropped_total       = rtc_photos_dropped_total;
	out->capped_total               = rtc_capped_total;
	out->wind_streak                = rtc_wind_streak;
	out->backoff_left_s             = rtc_backoff && (int32_t)(rtc_backoff_end_s - now_s()) > 0
	                                      ? rtc_backoff_end_s - now_s() : 0;
	out->backoff_s_total            = rtc_backoff_s_total;
	out->clips_sent_total           = rtc_clips_sent_total;
	out->clips_dropped_total        = rtc_clips_dropped_total;
	out->clip_s_total               = (rtc_clip_ms_total + 500) / 1000;
	out->clip_fuse_trips_total      = rtc_clip_fuse_trips_total;
	out->replies_keep_total         = rtc_replies_keep_total;
	out->replies_stop_total         = rtc_replies_stop_total;
	out->replies_none_total         = rtc_replies_none_total;
	out->reply_errors_total         = rtc_reply_errors_total;
	out->last_report_s              = rtc_last_report_s;
	out->have_ap_cache              = rtc_have_ap;
	out->using_dhcp                 = rtc_use_dhcp;
	out->ap_channel                 = rtc_channel;
	out->cam_quality                = rtc_cam_quality;
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
static bool     s_wifi_refresh   = false;  // the attempt in flight is a hostname refresh
static bool     s_refresh_tried  = false;  // at most one refresh attempt per wake
// TELEGRAM_HOST's address as wifi_reachable() resolved it on this association.
// The reply window's requests connect to it, so none of them waits on DNS
// (config.h, Reply window).
static IPAddress s_tg_ip;
static bool      s_tg_ip_ok      = false;

static void wifi_down()
{
	WiFi.disconnect(true, false);
	WiFi.mode(WIFI_OFF);
	s_wifi_pending   = false;
	s_wifi_connected = false;
	s_tg_ip_ok       = false;
}

// The static block for the fast path (9.4): the address saved from deployment
// mode's Home network card if there is one, else config.h's NET_* (netcfg.h).
// Read from NVS at most once a wake, by wifi_begin_async() just before it
// switches the station on, so the read adds no radio-on time and sits after
// the capture like the rest of the radio work (9.1). Wakes that never
// associate never read it.
static bool     s_net_loaded = false;
static bool     s_net_saved  = false;
static netcfg_t s_net;

static void net_load_once()
{
	if (s_net_loaded) {
		return;
	}
	s_net_loaded = true;
	const int64_t t0 = esp_timer_get_time();
	s_net_saved = netcfg_load(&s_net);
	if (!s_net_saved) {
		netcfg_builtin(&s_net);
	}
	char ip[16];
	netcfg_format(s_net.ip, ip);
	log_i("net: static %s (%s), NVS read in %lld us", ip,
	      s_net_saved ? "saved" : "config.h", (long long)(esp_timer_get_time() - t0));
}

static IPAddress to_ip(uint32_t a)
{
	return IPAddress((uint8_t)(a >> 24), (uint8_t)(a >> 16), (uint8_t)(a >> 8),
	                 (uint8_t)a);
}

// Deployment mode saved or forgot a home address (deploy_mode.h). The next
// wake tries the static path again, with whatever is now in force, instead of
// going straight to DHCP on a verdict about the old address; if the new one
// fails WIFI_VALIDATE_STATIC's lookup, the fallback below sets rtc_use_dhcp
// again exactly as before.
void deploy_net_changed()
{
	rtc_use_dhcp = false;
	s_net_loaded = false;
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
	net_load_once();   // a no-op here: wifi_begin_async() has read it
	if (!WiFi.config(to_ip(s_net.ip), to_ip(s_net.gw), to_ip(s_net.sn),
	                 to_ip(s_net.dns))) {
		log_w("static IP config rejected; this attempt will use DHCP");
	}
}

// Station hostname (config.h, WIFI_HOSTNAME), before every association. In
// core 3.3.11 WiFi.setHostname() only stores the name
// (NetworkManager::setHostname()); WiFiGenericClass::mode() writes it to the
// STA netif with esp_netif_set_hostname() when it switches STA on, before
// esp_wifi_set_mode(). Both begin() paths below switch it on: wifi_down() and
// disconnect(true, ...) turn STA off, and the WiFi.mode() or WiFi.config()
// that follows turns it back on, which also starts the DHCP client that sends
// the name. An STA already on has no such switch, so the name is written to
// its netif directly as well.
static void wifi_set_hostname()
{
	WiFi.setHostname(WIFI_HOSTNAME);
	if ((WiFi.getMode() & WIFI_MODE_STA) && !WiFi.STA.setHostname(WIFI_HOSTNAME)) {
		log_w("wifi: could not set hostname %s", WIFI_HOSTNAME);
	}
}

#define DHCP_REFRESH_ENABLED (HOSTNAME_REFRESH_INTERVAL_S > 0 && WIFI_DHCP_TIMEOUT_MS > 0)

// A DHCP lease was just obtained: WIFI_HOSTNAME has reached the router.
static void dhcp_lease_noted()
{
	rtc_last_dhcp_refresh_s = now_s();
	rtc_last_refresh_fail_s = DHCP_REFRESH_NEVER;
}

// Is this wake's first association due to be a hostname refresh (config.h,
// HOSTNAME_REFRESH_INTERVAL_S)? Only when the normal path would be static: a
// node on rtc_use_dhcp sends the name on every association anyway. Not within
// HOSTNAME_REFRESH_RETRY_S of a refresh that got no lease.
static bool dhcp_refresh_due()
{
#if DHCP_REFRESH_ENABLED
	const uint32_t t = now_s();
	return !s_wifi_dhcp && !s_refresh_tried &&
	       (rtc_last_dhcp_refresh_s == DHCP_REFRESH_NEVER ||
	        t - rtc_last_dhcp_refresh_s >= (uint32_t)HOSTNAME_REFRESH_INTERVAL_S) &&
	       (rtc_last_refresh_fail_s == DHCP_REFRESH_NEVER ||
	        t - rtc_last_refresh_fail_s >= (uint32_t)HOSTNAME_REFRESH_RETRY_S);
#else
	return false;
#endif
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
	// A hostname refresh replaces the static fast path for this one attempt;
	// wifi_wait_connected() falls back to that path if it fails.
	s_wifi_refresh = dhcp_refresh_due();
	if (s_wifi_refresh) {
		s_refresh_tried = true;
		s_wifi_dhcp     = true;
	}

	if (!s_wifi_dhcp) {
		net_load_once();   // before WiFi.mode() turns the radio on
	}
	WiFi.persistent(false);
	wifi_set_hostname();
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
	log_i("wifi: associating (%s, %s)",
	      s_wifi_refresh ? "dhcp, hostname refresh" : s_wifi_dhcp ? "dhcp" : "static",
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
		s_tg_ip    = resolved;
		s_tg_ip_ok = true;
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

#if DHCP_REFRESH_ENABLED
	// ---- Hostname refresh (config.h, HOSTNAME_REFRESH_INTERVAL_S) --------
	// A DHCP attempt in place of the static fast path, once a refresh is due.
	// The lease alone counts as success: its DISCOVER and REQUEST carried
	// WIFI_HOSTNAME. If it also resolves TELEGRAM_HOST this wake sends over
	// it; if not, the normal path below runs as on any other wake. With no
	// lease at all the refresh is still owed, and the first wake that
	// associates HOSTNAME_REFRESH_RETRY_S or more later tries it again. It
	// touches neither rtc_use_dhcp nor the AP
	// cache, except to refresh the cache on success as every successful
	// phase below does.
	if (s_wifi_refresh) {
		s_wifi_refresh = false;
		if (wifi_await(WIFI_DHCP_TIMEOUT_MS)) {
			dhcp_lease_noted();
			log_i("wifi: hostname refresh: lease %s as \"%s\" in %lu ms",
			      WiFi.localIP().toString().c_str(), WIFI_HOSTNAME,
			      (unsigned long)(millis() - s_wifi_t0));
			if (wifi_reachable()) {
				wifi_cache_ap();
				s_wifi_connected = true;
				return true;
			}
			log_w("wifi: refresh lease cannot resolve %s; normal path",
			      TELEGRAM_HOST);
		} else {
			rtc_last_refresh_fail_s = now_s();
			log_w("wifi: hostname refresh got no lease in %lu ms; normal path, "
			      "next refresh in %d s at the earliest",
			      (unsigned long)(millis() - s_wifi_t0), HOSTNAME_REFRESH_RETRY_S);
		}
		WiFi.disconnect(true, false);
		delay(10);
		s_wifi_pending = false;
		wifi_begin_async();   // s_refresh_tried is set: the normal path
		if (!s_wifi_pending) {
			return false;
		}
	}
#endif

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
		if (!was_static) {
			dhcp_lease_noted();
		}
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
		wifi_set_hostname();
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
			dhcp_lease_noted();
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
	c.reserve(512);

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

	// Totals, then (+ since the last report that landed), as 9.3 asks.
	c += "triggers ";
	c += rtc_triggers_total;
	c += " (+";
	c += rtc_triggers_since_report;
	c += " since report)\nphotos sent ";
	c += rtc_photos_sent_total;
	c += " (+";
	c += rtc_photos_sent_since_report;
	c += "), dropped ";
	c += rtc_photos_dropped_total;
	c += " (+";
	c += rtc_photos_dropped_since_report;
	c += ")\nsuppressed (no person) ";
	c += rtc_suppressed_total;
	c += " (+";
	c += rtc_suppressed_since_report;
	c += "), capped ";
	c += rtc_capped_total;
	c += " (+";
	c += rtc_capped_since_report;
	c += ")\ndetect errors ";
	c += rtc_detect_errors_total;
	c += " (+";
	c += rtc_detect_errors_since_report;
	c += ")\nclips sent ";
	c += rtc_clips_sent_total;
	c += " (+";
	c += rtc_clips_sent_since_report;
	c += "), dropped ";
	c += rtc_clips_dropped_total;
	c += " (+";
	c += rtc_clips_dropped_since_report;
	c += "), ";
	c += (rtc_clip_ms_total + 500) / 1000;
	c += " s recorded (+";
	c += (rtc_clip_ms_since_report + 500) / 1000;
	c += ")\nclip fuse (";
	c += VIDEO_MAX_CLIPS_PER_DAY;
	c += "/day) ";
	if (rtc_clip_fuse_trips_total == 0) {
		c += "never tripped";
	} else {
		c += "TRIPPED ";
		c += rtc_clip_fuse_trips_total;
		c += "x";
	}
	c += "\nreplies keep ";
	c += rtc_replies_keep_total;
	c += " (+";
	c += rtc_replies_keep_since_report;
	c += "), stop ";
	c += rtc_replies_stop_total;
	c += " (+";
	c += rtc_replies_stop_since_report;
	c += "), none ";
	c += rtc_replies_none_total;
	c += " (+";
	c += rtc_replies_none_since_report;
	c += "; window errors ";
	c += rtc_reply_errors_total;
	c += " (+";
	c += rtc_reply_errors_since_report;
	c += "))";
	c += "\nradar ignored (wind) ";
	c += rtc_backoff_s_total;
	c += " s (+";
	c += rtc_backoff_s_since_report;
	c += ")\nbattery ";

	const int32_t mv = battery_mv();
	if (mv < 0) {
		c += "n/a";   // no divider fitted — config.h, never a fabricated value
	} else {
		c += mv;
		c += " mV";
	}

	c += "\njpeg quality ";
	c += rtc_cam_quality;

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
// Sending (9.6) — Telegram sendDocument (or sendPhoto, config.h), streamed
// straight from PSRAM.
//
// There is no retry store. A frame that cannot be delivered on this wake is
// dropped, and the only trace of it is the trigger counters, which is exactly
// what makes maybe_send_telemetry_only() below worth keeping: it is the one
// mechanism that lets a node with a dead uplink still be noticed as alive.
// ---------------------------------------------------------------------------
// A report has landed: the since-report counters start again (9.3).
static void report_landed()
{
	rtc_triggers_since_report       = 0;
	rtc_suppressed_since_report     = 0;
	rtc_detect_errors_since_report  = 0;
	rtc_photos_sent_since_report    = 0;
	rtc_photos_dropped_since_report = 0;
	rtc_capped_since_report         = 0;
	rtc_backoff_s_since_report      = 0;
	rtc_clips_sent_since_report     = 0;
	rtc_clips_dropped_since_report  = 0;
	rtc_clip_ms_since_report        = 0;
	rtc_replies_keep_since_report   = 0;
	rtc_replies_stop_since_report   = 0;
	rtc_replies_none_since_report   = 0;
	rtc_reply_errors_since_report   = 0;
	rtc_last_report_s               = now_s();
}

// Send one photo: radio up, upload, radio down. The camera is already off
// (9.1 step 5). Counted as sent before the caption is built, so the caption
// includes it, and taken back if the upload fails.
//
// With `sent` (a reply window will follow the first clip, config.h) the caption
// says so, and the message's id and date come back in *sent for that window.
static bool send_photo(const frame_t &f, const char *reason, tg_sent_t *sent)
{
	// Credentials before the radio. A node built without secrets.h should go
	// straight back to sleep, not spend an association proving it cannot send.
	if (!telegram_configured()) {
		log_e("no Telegram credentials compiled in (include/secrets.h); "
		      "photo dropped");
		rtc_photos_dropped_total++;
		rtc_photos_dropped_since_report++;
		return false;
	}

	rtc_last_report_attempt_s = now_s();   // every caption carries telemetry
	bool ok = false;
	if (wifi_up()) {
		rtc_photos_sent_total++;
		rtc_photos_sent_since_report++;
		String why = reason;
		why += "\nentry photo of visit #";
		why += rtc_ep.tag;
		if (sent) {
			why += "\na clip follows; reply keep for more clips of this visit, "
			       "stop for none";
		}
		const String cap = telemetry_text(why.c_str(), f.score, 0, 0);
		ok = telegram_send_photo(f.data, f.len, cap, sent);
		if (!ok) {
			rtc_photos_sent_total--;
			rtc_photos_sent_since_report--;
		}
	} else {
		log_w("no uplink");
	}
	wifi_down();

	if (ok) {
		// Counters are cumulative "since last report" (9.3) — clear them only
		// once a report has actually landed somewhere.
		report_landed();
	} else {
		log_w("photo not delivered, dropped");
		rtc_photos_dropped_total++;
		rtc_photos_dropped_since_report++;
	}
	return ok;
}

// Is a bare telemetry report owed right now?
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
		report_landed();
	}
	wifi_down();
}

// ---------------------------------------------------------------------------
// Sleep
// ---------------------------------------------------------------------------
// The radar holds OT2 high for RADAR_UNMANNED_DELAY_S after the last presence
// (config.h), where 1 has the AM312 holding for ~10 s. Arming an active-high
// level wake while it is still asserted would re-wake us immediately, so wait it
// out in light sleep first, for at most PIR_IDLE_MAX_S.
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
		// can read stale and end this wait with OT2 still high; the
		// level-triggered deep-sleep wake then fires at once and one visit
		// logs as several triggers, each a wake and a boot.
		rtc_gpio_deinit(PIN_PIR);
		if (serr != ESP_OK) {
			delay(1000);   // rejected; wait at CPU idle rather than spinning
		}
		guard++;
	}
}

// Camera and radio off, as every sleep starts.
static void park_for_sleep()
{
	camera_power_off();
	// Normally a repeat of camera_down(). Deployment mode's shutdown can get
	// here without it: when a stream will not let go of the sensor,
	// service_action() skips the deinit and leaves the rail to be cut above,
	// with XCLK still running and the SCCB pullups on. And a wake that never
	// ran the camera, a power-on for one, would otherwise latch these pads in
	// whatever state reset left them.
	camera_pins_quiesce();
	wifi_down();
}

// Arm the given wake sources and deep-sleep. `ext0_level` is the D1 level to
// wake on, or -1 for none; `timer_ms` 0 means no timer. Milliseconds, so the
// presence gap (PRESENCE_GAP_S, counted from the real fall) is not rounded up
// to a whole second. Shared by enter_deep_sleep() and the trigger flow's own
// sleeps (sleep_for_state()).
[[noreturn]] static void deep_sleep_now(int ext0_level, uint32_t timer_ms)
{
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
	if (ext0_level >= 0) {
		esp_sleep_enable_ext0_wakeup(PIN_PIR, ext0_level);
	}
	if (timer_ms > 0) {
		esp_sleep_enable_timer_wakeup((uint64_t)timer_ms * 1000ULL);
	}
	log_i("sleeping: wake on D1 %s%s, timer %.1f s",
	      ext0_level < 0 ? "never" : ext0_level ? "high" : "low",
	      rtc_ep.open ? " (episode open)" : rtc_backoff ? " (wind backoff)" : "",
	      timer_ms / 1000.0f);

	// Holds every digital-only pad (IO26-48) as it is now for the whole
	// sleep, which keeps the camera pins parked by camera_pins_quiesce().
	// PIN_CAM_POWER does not rely on this: GPIO1 is an RTC pad, and the
	// gpio_hold_en() in camera_power_off() takes the RTC hold path, which
	// holds it through deep sleep by itself.
	gpio_deep_sleep_hold_en();

	Serial.flush();
	esp_deep_sleep_start();

	// esp_deep_sleep_start() does not return; this only satisfies [[noreturn]].
	for (;;) {
	}
}

// Never returns: ends in esp_deep_sleep_start(). Marked so the compiler
// enforces that no caller falls through into code using stale state.
//
// The idle sleep: wake on the next rising edge on D1, or the telemetry timer.
// Also the exit for abnormal resets and for deployment mode's arm, so it knows
// nothing of episodes; the trigger flow uses sleep_for_state().
[[noreturn]] void enter_deep_sleep()
{
	park_for_sleep();
	wait_for_pir_idle();

	// Wake for the periodic report even if nobody comes. Without this a node
	// that sees no presence never reports, and a quiet node looks the same as a
	// mute one (9.6). The wake goes through setup()'s timer path. By
	// now_s() it can arrive slightly early, because the RC slow clock is
	// recalibrated between arming and waking. telemetry_due() then says no,
	// and the floor makes that one more short sleep, not a loop.
	const uint32_t report_in_s = telegram_configured() ? telemetry_wake_in_s() : 0;

	log_i("%lu triggers total, %lu photos sent, %lu suppressed",
	      (unsigned long)rtc_triggers_total, (unsigned long)rtc_photos_sent_total,
	      (unsigned long)rtc_suppressed_total);
	deep_sleep_now(1, report_in_s * 1000u);   // presence is active-high (9.1)
}

// ---------------------------------------------------------------------------
// Trigger flow (config.h) — replaces 9.2's isolated/burst rule
// ---------------------------------------------------------------------------
// Does this visit record in full (config.h, Reply window)? On "keep", or with
// the window turned off; otherwise the first clip is the visit's only one.
static bool visit_recorded_whole()
{
	return REPLY_WAIT_S == 0 || rtc_ep.reply == TG_REPLY_KEEP;
}

// What came of the reply window, for logs and captions.
static const char *reply_name()
{
	if (REPLY_WAIT_S == 0) {
		return "window off";
	}
	if (!rtc_ep.asked) {
		return "no window";
	}
	return rtc_ep.reply == TG_REPLY_KEEP ? "keep" :
	       rtc_ep.reply == TG_REPLY_STOP ? "stop" : "none";
}

static void episode_open()
{
	memset(&rtc_ep, 0, sizeof(rtc_ep));
	rtc_ep.open     = true;
	rtc_ep.d1_high  = true;
	rtc_ep.start_ms = now_ms();
	// The visit's tag (config.h, Reply window): four characters, no 0/O or
	// 1/I to misread. Random, so a reply carrying another visit's tag cannot
	// match this one; the date rule is what keeps old messages out.
	static const char ALPHABET[] = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789";
	uint32_t r = esp_random();
	for (int i = 0; i < 4; i++) {
		rtc_ep.tag[i] = ALPHABET[r & 31];
		r >>= 5;
	}
	rtc_ep.tag[4] = '\0';
	log_i("episode opened, visit #%s", rtc_ep.tag);
}

static void episode_close(const char *why)
{
	if (rtc_ep.open) {
		log_i("episode #%s closed (%s) after %lu s, %u photo(s), %u clip(s), "
		      "person %s, reply %s", rtc_ep.tag,
		      why, (unsigned long)((now_ms() - rtc_ep.start_ms) / 1000),
		      rtc_ep.photos, (unsigned)rtc_ep.clips,
		      rtc_ep.person == EP_PERSON_YES ? "yes" :
		      rtc_ep.person == EP_PERSON_NO  ? "no"  : "not checked",
		      reply_name());
	}
	rtc_ep.open = false;
}

// The node stops listening to the radar for a while (config.h, wind
// backoff). Closes the episode: with D1's wake unarmed there is no following it.
static void backoff_start()
{
	const uint8_t  level = rtc_backoff_level < 16 ? rtc_backoff_level : 16;
	uint32_t len = (uint32_t)WIND_BACKOFF_MIN_S << level;
	if (len > (uint32_t)WIND_BACKOFF_MAX_S || len < (uint32_t)WIND_BACKOFF_MIN_S) {
		len = WIND_BACKOFF_MAX_S;
	}
	if (rtc_backoff_level < 255) {
		rtc_backoff_level++;
	}
	rtc_backoff         = true;
	rtc_backoff_start_s = now_s();
	rtc_backoff_end_s   = rtc_backoff_start_s + len;
	episode_close("wind backoff");
	log_w("wind: %u photos in a row without a person; radar ignored for %lu s",
	      rtc_wind_streak, (unsigned long)len);
}

static void backoff_finish()
{
	const uint32_t t = now_s();
	const uint32_t spent = t - rtc_backoff_start_s;
	rtc_backoff_s_total        += spent;
	rtc_backoff_s_since_report += spent;
	rtc_backoff  = false;
	rtc_listen_s = t;   // WIND_QUIET_RESET_S counts from here
	log_i("wind backoff over after %lu s; D1 armed", (unsigned long)spent);
}

// ---------------------------------------------------------------------------
// Reply window (config.h) — the one place the node reads the chat
// ---------------------------------------------------------------------------
static int clips_last_day();   // Presence video, below

// May the window open for this visit? Asked before the photo goes out, so its
// caption only invites an answer the node will wait for.
static bool reply_window_wanted()
{
#if REPLY_WINDOW_BUILT
	int64_t chat;
	if (!tg_chat_id_number(TELEGRAM_CHAT_ID, &chat)) {
		log_w("reply window: TELEGRAM_CHAT_ID is not numeric; no window");
		return false;
	}
	if (clips_last_day() >= VIDEO_MAX_CLIPS_PER_DAY) {
		log_i("reply window: the clip fuse is full, so no clip could follow; "
		      "no window");
		return false;
	}
	return true;
#else
	return false;
#endif
}

// Wait up to REPLY_WAIT_S for "keep" or "stop", once the first clip (`clip`)
// has been delivered, on the association that delivered it. Sets
// rtc_ep.reply, which stays TG_REPLY_NONE for no answer and for every
// failure: both mean no second clip. False when the window could not run or
// ended on an error. The radio stays up; the caller takes it down.
static bool reply_window(const tg_sent_t &clip)
{
#if REPLY_WINDOW_BUILT
	rtc_ep.reply = TG_REPLY_NONE;
	const uint32_t t_open   = millis();
	const uint32_t deadline = t_open + (uint32_t)REPLY_WAIT_S * 1000u;

	// An answer may reply to the photo or to this clip. It must be dated no
	// earlier than the photo; if the photo's response could not be read, no
	// earlier than the clip, which only loses answers given while the clip was
	// being recorded and sent.
	tg_episode_ref_t ref = {};
	ref.photo_msg_id = rtc_ep.photo_ref ? rtc_ep.photo_msg_id : -1;
	ref.clip_msg_id  = clip.ok ? clip.message_id : -1;
	ref.photo_date   = rtc_ep.photo_ref ? rtc_ep.photo_date : clip.date;
	ref.tag          = rtc_ep.tag;
	ref.floor_update = rtc_tg_floor_update;
	const bool have_ref = rtc_ep.photo_ref || clip.ok;
	if (!have_ref || !s_tg_ip_ok || !tg_chat_id_number(TELEGRAM_CHAT_ID, &ref.chat_id)) {
		log_w("reply window: %s; no window, so no clip after the first",
		      !have_ref   ? "neither the photo's nor the clip's message_id came back" :
		      !s_tg_ip_ok ? "no resolved address for " TELEGRAM_HOST
		                  : "TELEGRAM_CHAT_ID is not numeric");
		return false;
	}

	char *buf = (char *)heap_caps_malloc(REPLY_BUF_BYTES, MALLOC_CAP_SPIRAM);
	if (!buf) {
		log_e("reply window: no PSRAM for its %d-byte buffer; no window",
		      REPLY_BUF_BYTES);
		return false;
	}
	rtc_ep.asked = true;
	// Modem sleep for the wait (config.h): the radio wakes for the AP's DTIM
	// beacons instead of listening throughout. The AP holds anything addressed
	// to the node until the next one, so an answer arrives a beacon late at
	// worst; every wait below is on the deadline, not on a packet's timing.
	if (!WiFi.setSleep(WIFI_PS_MIN_MODEM)) {
		log_w("reply window: modem sleep refused; waiting at full power");
	}
	log_i("reply window: open for %d s on visit #%s (photo %lld, clip %lld), "
	      "modem sleep", REPLY_WAIT_S, rtc_ep.tag, (long long)ref.photo_msg_id,
	      (long long)ref.clip_msg_id);

	// 1. What is there already. offset -N returns the newest N updates and has
	// Telegram forget every older one, so a backlog from before this visit is
	// behind the line from here on. The N are judged like any other: an
	// answer given while the clip was being recorded or sent counts.
	size_t    len = 0;
	tg_scan_t sc  = {};
	int code = telegram_get_updates(s_tg_ip, -REPLY_BACKLOG_N, REPLY_BACKLOG_N, 0,
	                                deadline, buf, REPLY_BUF_BYTES, &len);
	bool failed = code != 200;
	if (!failed) {
		tg_scan_updates(buf, len, &ref, &sc);
		if (sc.ok && sc.max_update_id >= 0 && sc.max_update_id < ref.floor_update) {
			// Telegram renumbers, lower, after a week with no updates. The
			// newest update is then the line, and the date and reference
			// rules judge what was read on their own.
			log_w("reply window: update %lld is below the last processed %lld; "
			      "Telegram has renumbered", (long long)sc.max_update_id,
			      (long long)ref.floor_update);
			ref.floor_update = -1;
			tg_scan_updates(buf, len, &ref, &sc);
		}
		failed = !sc.ok;
	}
	int64_t offset = 0;   // 0 is left out of the request
	if (!failed) {
		if (sc.max_update_id >= 0) {
			ref.floor_update = sc.max_update_id;
			offset           = sc.max_update_id + 1;
		}
		rtc_ep.reply = sc.reply;
	}

	// 2. Long-poll until an answer counts or the window closes. An update that
	// is not an answer (other chatter in the chat) ends a poll early; the
	// offset steps past it and the next poll waits on. An HTTP error will not
	// clear inside the window (409: a webhook or another reader on this
	// token; 401: the token), so it ends the window; a lost connection is
	// tried again, twice, while time is left.
	int polls = 0, retries = 0;
	while (!failed && rtc_ep.reply == TG_REPLY_NONE) {
		code = telegram_get_updates(s_tg_ip, offset, REPLY_BACKLOG_N, REPLY_WAIT_S,
		                            deadline, buf, REPLY_BUF_BYTES, &len);
		if (code == TG_GET_ERR_TIME) {
			break;   // the window is over
		}
		if (code != 200) {
			if (code > 0 || code == TG_GET_ERR_SIZE || ++retries > 2) {
				failed = true;
			} else {
				delay(200);
			}
			continue;
		}
		polls++;
		tg_scan_updates(buf, len, &ref, &sc);
		if (!sc.ok) {
			failed = true;
			continue;
		}
		if (sc.max_update_id > ref.floor_update) {
			ref.floor_update = sc.max_update_id;
			offset           = sc.max_update_id + 1;
		}
		rtc_ep.reply = sc.reply;
	}
	WiFi.setSleep(WIFI_PS_NONE);
	heap_caps_free(buf);
	// Only ever the newest update seen (or, after a renumbering, the new
	// line): nothing at or below it can count in a later window.
	rtc_tg_floor_update = ref.floor_update;

	log_i("reply window: %s after %lu ms, %d poll(s)%s", reply_name(),
	      (unsigned long)(millis() - t_open), polls,
	      failed ? ", ended on an error" : "");
	return !failed;
#else
	(void)clip;
	return false;
#endif
}

// Count what a window came to (rtc_replies_*). `clean` is reply_window()'s
// verdict: false makes a none an error as well.
static void reply_counted(bool clean)
{
	if (rtc_ep.reply == TG_REPLY_KEEP) {
		rtc_replies_keep_total++;
		rtc_replies_keep_since_report++;
	} else if (rtc_ep.reply == TG_REPLY_STOP) {
		rtc_replies_stop_total++;
		rtc_replies_stop_since_report++;
	} else {
		rtc_replies_none_total++;
		rtc_replies_none_since_report++;
		if (!clean) {
			rtc_reply_errors_total++;
			rtc_reply_errors_since_report++;
		}
	}
}

// One rising edge on D1 that opened an episode or continued one. Only the first
// of an episode is photographed (config.h, PHOTOS_PER_EPISODE): the capture
// comes first and nothing is allowed in front of it, the radio least of all
// (9.1), detection judges the photo with the radio still off, and the verdict
// is the gate for the whole episode (rtc_ep.confirmed). A confirmed episode
// sends the photo now and records afterwards (video_check(), which holds the
// reply window after the first clip, config.h); any other sends
// nothing and records nothing. No second photo, no fresh check: a later edge in
// the same episode is counted and nothing more.
static void handle_trigger(uint32_t t_ref_ms)
{
	const uint32_t t = now_s();
	if (t - rtc_listen_s >= (uint32_t)WIND_QUIET_RESET_S &&
	    (rtc_wind_streak || rtc_backoff_level)) {
		log_i("wind: %d s without a trigger; streak cleared", WIND_QUIET_RESET_S);
		rtc_wind_streak   = 0;
		rtc_backoff_level = 0;
	}
	rtc_listen_s = t;
	record_trigger();

	if (rtc_ep.photos >= PHOTOS_PER_EPISODE) {
		rtc_capped_total++;
		rtc_capped_since_report++;
		log_i("edge counted, not photographed: the episode has its entry photo "
		      "(%s)", rtc_ep.confirmed ? "confirmed" : "gated off");
		return;
	}
	rtc_ep.photos++;   // spent even if the capture fails: there is no second try

	frame_t f = {};
	if (!capture(&f, t_ref_ms)) {
		// No photo, so no verdict to gate on. Not failed open: a camera that
		// cannot take a still will not record either, and an ungated clip of
		// whatever tripped the radar is what the gate exists to prevent.
		log_e("capture failed: no photo, no verdict, nothing recorded this episode");
		return;
	}

	bool        confirmed;
	const char *reason;
#if DETECTION_ENABLED
	if (!judge_frame(f)) {
		// Fail open (config.h): a photo that could not be judged is better sent
		// than lost, and the episode records as if a person had been seen.
		rtc_detect_errors_total++;
		rtc_detect_errors_since_report++;
		confirmed = true;
		reason    = "detect error: sent unjudged";
	} else if (f.hit) {
		rtc_ep.person     = EP_PERSON_YES;
		rtc_wind_streak   = 0;
		rtc_backoff_level = 0;
		confirmed = true;
		reason    = "person";
	} else {
		rtc_ep.person = EP_PERSON_NO;
		// With the gate off (SEND_ONLY_PERSONS 0) an empty photo says nothing
		// about wind, as in bench-nodetect, which has no streak either.
		if (SEND_ONLY_PERSONS && rtc_wind_streak < 255) {
			rtc_wind_streak++;
		}
		confirmed = !SEND_ONLY_PERSONS;
		reason    = "no person (SEND_ONLY_PERSONS 0)";
	}
#else
	// bench-nodetect: no detector, so no gate (config.h, Trigger flow).
	confirmed = true;
	reason    = "unfiltered (no detector in this build)";
#endif
	rtc_ep.confirmed = confirmed;

	if (confirmed) {
		// The first clip follows at once (video_check()); the reply window
		// waits until that clip is delivered (config.h, Reply window), and
		// needs this photo's message_id and date to recognise an answer.
		tg_sent_t  sent = {};
		const bool ok   = send_photo(f, reason, reply_window_wanted() ? &sent : nullptr);
		heap_caps_free(f.data);
		if (!ok) {
			// The uplink is down, so a clip would be dropped too: no recording
			// this episode (config.h, Presence video).
			rtc_ep.video_done = true;
		} else if (sent.ok) {
			rtc_ep.photo_ref    = true;
			rtc_ep.photo_msg_id = sent.message_id;
			rtc_ep.photo_date   = sent.date;
		}
	} else {
		rtc_suppressed_total++;
		rtc_suppressed_since_report++;
		log_i("no person (best %.2f); suppressed, nothing sent or recorded this "
		      "episode", f.score);
		heap_caps_free(f.data);
	}

	if (rtc_wind_streak >= WIND_STREAK_BACKOFF) {
		backoff_start();
	}
}

// ---------------------------------------------------------------------------
// Presence video (config.h)
// ---------------------------------------------------------------------------
// There is no motion-proof timing here any more: the 10-second rule it served
// was a PIR-era false-trigger proxy, made redundant by radar presence plus the
// entry photo's AI confirmation (config.h, Trigger flow).
enum clip_end_t { CLIP_END_QUIET = 0, CLIP_END_TIME, CLIP_END_FULL, CLIP_END_CAMERA };
static const char *const CLIP_END_NAMES[] = {
	"presence ended", "time cap", "buffer full", "camera error",
};

struct clip_t {
	uint8_t   *buf;         // PSRAM: the finished AVI; the caller frees it
	size_t     len;
	uint32_t   frames;
	uint32_t   dur_ms;
	float      fps;
	clip_end_t end;
	int        pir_edges;   // rising edges while recording: same visitor
	uint64_t   d1_fall_ms;  // CLIP_END_QUIET: now_ms() when D1 fell
};

// Record one clip with the radio off, and cut the camera before returning
// (9.1 step 5). `max_s` caps its length; with `follow_pir` it also ends once
// D1 has been low for VIDEO_END_QUIET_S, the quiet window. False, with nothing
// left allocated, if no frame was recorded.
static bool record_clip(clip_t *out, uint32_t max_s, bool follow_pir)
{
	memset(out, 0, sizeof(*out));
	if (!camera_up_as(VIDEO_FRAMESIZE, VIDEO_JPEG_QUALITY)) {
		log_e("clip: camera did not start");
		return false;
	}

	// Sized after the camera has taken its own two frame buffers, so they are
	// out of what is free. No photo is held by now (handle_trigger() has sent
	// and freed it). VIDEO_PSRAM_RESERVE is kept free in all, wherever it lies;
	// the largest block only says how big one allocation can be.
	const size_t free_b  = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
	const size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
	size_t cap = free_b > (size_t)VIDEO_PSRAM_RESERVE ? free_b - VIDEO_PSRAM_RESERVE : 0;
	if (cap > largest) {
		cap = largest;
	}
	if (cap > (size_t)VIDEO_MAX_BYTES) {
		cap = VIDEO_MAX_BYTES;
	}
	uint8_t *buf = cap >= 64 * 1024 ? (uint8_t *)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM)
	                                : nullptr;
	if (!buf) {
		log_e("clip: no PSRAM for a buffer (%u free, largest block %u)",
		      (unsigned)free_b, (unsigned)largest);
		camera_down();
		return false;
	}
	log_i("clip: %u-byte buffer; PSRAM free %u before it, %u after, largest "
	      "block was %u", (unsigned)cap, (unsigned)free_b,
	      (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM), (unsigned)largest);

	// Frames are taken on a VIDEO_FPS schedule from the sensor's faster
	// stream (GRAB_LATEST hands over the newest), the first no earlier than
	// VIDEO_WARMUP_MS after init. A late frame does not push the schedule
	// back; after a long stall it restarts instead of bursting to catch up.
	avi_t avi;
	bool  begun = false;
	const uint32_t interval_ms = 1000 / VIDEO_FPS;
	uint32_t next_due  = millis() + VIDEO_WARMUP_MS;
	uint32_t t_first   = 0;
	uint32_t low_since = 0;
	bool     was_high  = gpio_get_level(PIN_PIR) == 1;
	bool     raised    = false;
	clip_end_t end     = CLIP_END_TIME;
	for (;;) {
		camera_fb_t *fb = esp_camera_fb_get();
		if (!fb) {
			// Most likely every frame outgrowing the driver's buffer (config.h,
			// VIDEO_FRAMESIZE): coarser by CAM_QUALITY_STEP, once, as capture()
			// does. The schedule carries on; the gap is the 4 s timeout.
			sensor_t *s = esp_camera_sensor_get();
			const int q2 = VIDEO_JPEG_QUALITY + CAM_QUALITY_STEP;
			if (!raised && q2 <= 63 && s && s->set_quality(s, q2) == 0) {
				log_w("clip: no frame at JPEG quality %d; carrying on at %d",
				      VIDEO_JPEG_QUALITY, q2);
				raised = true;
				continue;
			}
			end = CLIP_END_CAMERA;
			break;
		}
		const uint32_t fs = fb_start_ms(fb);
		if ((int32_t)(fs - next_due) >= 0) {
			if (!begun) {
				begun   = avi_begin(&avi, buf, cap, fb->width, fb->height, VIDEO_FPS);
				t_first = fs;
			}
			const uint64_t t_us = (uint64_t)fb->timestamp.tv_sec * 1000000u +
			                      (uint64_t)fb->timestamp.tv_usec;
			if (!begun || !avi_add_frame(&avi, fb->buf, fb->len, t_us)) {
				esp_camera_fb_return(fb);
				end = CLIP_END_FULL;
				break;
			}
			next_due += interval_ms;
			if ((int32_t)(fs - next_due) > (int32_t)interval_ms) {
				next_due = fs + interval_ms;
			}
		}
		esp_camera_fb_return(fb);

		const uint32_t now = millis();
		if (begun && now - t_first >= max_s * 1000u) {
			end = CLIP_END_TIME;
			break;
		}
		if (follow_pir) {
			const bool high = gpio_get_level(PIN_PIR) == 1;
			if (high && !was_high) {
				out->pir_edges++;
			}
			was_high = high;
			if (high) {
				low_since = 0;
			} else if (low_since == 0) {
				low_since = now | 1;
			} else if (now - low_since >= (uint32_t)VIDEO_END_QUIET_S * 1000u) {
				end = CLIP_END_QUIET;
				out->d1_fall_ms = now_ms() - (millis() - low_since);
				break;
			}
		}
	}
	camera_down();

	if (!begun || avi.frames == 0) {
		log_e("clip: no frames recorded (%s)", CLIP_END_NAMES[end]);
		heap_caps_free(buf);
		return false;
	}
	out->frames = avi.frames;
	out->dur_ms = avi_duration_us(&avi) / 1000;
	out->fps    = avi_fps(&avi);
	out->len    = avi_finish(&avi);
	out->buf    = buf;
	out->end    = end;
	log_i("clip: %lu frames, %.2f fps, %.1f s, %u bytes, ended on %s, "
	      "%d D1 edges while recording", (unsigned long)out->frames, out->fps,
	      out->dur_ms / 1000.0f, (unsigned)out->len, CLIP_END_NAMES[end],
	      out->pir_edges);
	return true;
}

// Send a recorded clip: radio up, upload, radio down. Counted as sent before
// the caption is built and taken back if the upload fails, as for photos.
//
// With `sent` (the first clip, with the reply window to follow, config.h) the
// caption asks for an answer, the message's id and date come back in *sent,
// and a delivered clip leaves the radio up for the window; the caller takes it
// down.
static bool send_clip(const clip_t &c, tg_sent_t *sent)
{
	String why = "clip ";
	why += rtc_ep.clips;
	why += " of this visit, ";
	why += String(c.dur_ms / 1000.0f, 1);
	why += " s, ";
	why += String(c.fps, 1);
	why += " fps, ";
	why += c.frames;
	why += " frames, ";
	why += (uint32_t)((c.len + 512) / 1024);
	why += " kB, ended: ";
	why += CLIP_END_NAMES[c.end];
	why += "\nperson this visit: ";
	why += rtc_ep.person == EP_PERSON_YES ? "yes" :
	       rtc_ep.person == EP_PERSON_NO  ? "no"  : "not checked";
	why += "\nvisit #";
	why += rtc_ep.tag;
	if (sent) {
		why += "\nreply keep for more clips of this visit, stop for none, within ";
		why += REPLY_WAIT_S;
		why += " s";
	} else {
		why += ", reply: ";
		why += reply_name();
		why += visit_recorded_whole() ? " (whole visit)" : " (this is the only clip)";
	}

	bool ok = false;
	if (telegram_configured()) {
		rtc_last_report_attempt_s = now_s();   // the caption carries telemetry
		if (wifi_up()) {
			rtc_clips_sent_total++;
			rtc_clips_sent_since_report++;
			const String cap = telemetry_text(why.c_str(), 0.0f, 0, 0);
			ok = telegram_send_file("sendDocument", "document", "clip.avi",
			                        "video/x-msvideo", c.buf, c.len, cap,
			                        TELEGRAM_CLIP_MIN_BPS, sent);
			if (!ok) {
				rtc_clips_sent_total--;
				rtc_clips_sent_since_report--;
			}
		}
		if (!ok || !sent) {
			wifi_down();
		}
	} else {
		log_e("no Telegram credentials compiled in (include/secrets.h)");
	}

	if (ok) {
		report_landed();
	} else {
		log_w("clip not delivered, dropped");
		rtc_clips_dropped_total++;
		rtc_clips_dropped_since_report++;
	}
	return ok;
}

static int clips_last_day()
{
	const uint32_t t = now_s();
	int n = 0;
	for (int i = 0; i < VIDEO_MAX_CLIPS_PER_DAY; i++) {
		const uint32_t e = rtc_clip_times[i];
		if (e != CLIP_SLOT_EMPTY && t - e < 24u * 3600u) {
			n++;
		}
	}
	return n;
}

// Defined with the wake deadline below. Replaces the pending deadline, if any,
// with a fresh one `seconds` from now.
static void wake_deadline_arm(uint32_t seconds);

// Record the visit as clips back to back, each sent before the next is
// recorded (config.h, Presence video): all of it on a "keep" answer, the first
// clip only otherwise (Reply window). Returns when presence has ended, a send
// failed, the fuse refused, the camera gave out, or the one clip a visit
// without "keep" gets is done, with the episode left for sleep_for_state():
// either d1_high false and fall_ms set, or video_done.
//
// Each clip cycle, record then send, runs under its own fresh deadline
// (WAKE_CLIP_DEADLINE_S, config.h): a stuck step trips in minutes, while a
// visit of any length can run across as many clips as the fuse allows.
static void video_run()
{
	for (;;) {
		if (gpio_get_level(PIN_PIR) == 0) {
			// Presence ended while the photo or the previous clip was being
			// sent. The episode runs on: a new edge inside PRESENCE_GAP_S is
			// the same visit and records again. The real fall time is not
			// known (nothing watched D1 during the upload), so the gap counts
			// from now.
			rtc_ep.d1_high = false;
			rtc_ep.fall_ms = now_ms();
			return;
		}
		if (!visit_recorded_whole() && rtc_ep.clips >= 1) {
			// No "keep" after the first clip (config.h, Reply window): that
			// clip, already sent, was this visit's only one. Checked after D1,
			// so a visit that has ended keeps its fall time for the gap.
			log_i("visit #%s: reply %s, so no clip after the first", rtc_ep.tag,
			      reply_name());
			rtc_ep.video_done = true;
			return;
		}
		if (clips_last_day() >= VIDEO_MAX_CLIPS_PER_DAY) {
			// The fuse (config.h), not a normal limit. The entry photo has
			// already gone out; only the recording is refused.
			rtc_clip_fuse_trips_total++;
			log_w("clip fuse: %d clips in the last 24 h; recording refused "
			      "(trip %lu)", clips_last_day(),
			      (unsigned long)rtc_clip_fuse_trips_total);
			rtc_ep.video_done = true;
			return;
		}
		wake_deadline_arm(WAKE_CLIP_DEADLINE_S);
		const uint32_t t_start = now_s();
		clip_t c;
		if (!record_clip(&c, VIDEO_MAX_CLIP_S, true)) {
			rtc_ep.video_done = true;
			return;
		}
		// Counted against the fuse once something was actually recorded.
		rtc_clip_times[rtc_clip_head] = t_start;
		rtc_clip_head = (rtc_clip_head + 1) % VIDEO_MAX_CLIPS_PER_DAY;
		rtc_ep.clips++;
		rtc_clip_ms_total        += c.dur_ms;
		rtc_clip_ms_since_report += c.dur_ms;
		// The first clip's delivery opens the reply window (config.h), on the
		// clip's own association: the answer decides whether a second clip
		// starts. Its buffer is freed first; the camera is already off.
		tg_sent_t  clip_ref = {};
		const bool ask  = rtc_ep.clips == 1 && reply_window_wanted();
		const bool sent = send_clip(c, ask ? &clip_ref : nullptr);
		heap_caps_free(c.buf);
		if (sent && ask) {
			const bool clean = reply_window(clip_ref);
			wifi_down();
			reply_counted(clean);
		}

		if (!sent) {
			// The uplink is down; the next clip would be dropped as well.
			log_w("no more clips this visit");
			rtc_ep.video_done = true;
			return;
		}
		if (c.end == CLIP_END_QUIET) {
			// D1 fell VIDEO_END_QUIET_S before the clip ended, which is less
			// than PRESENCE_GAP_S: the visit may not be over. The episode runs
			// on from the real fall, and closes once the gap has passed since
			// then; a new edge before that is this visitor again. If D1 rose
			// during the upload, the level wake fires at once and sees it.
			rtc_ep.d1_high = false;
			rtc_ep.fall_ms = c.d1_fall_ms;
			return;
		}
		if (c.end == CLIP_END_CAMERA) {
			// The sensor stopped delivering frames. Another clip would most
			// likely do the same, and with no per-visit limit only the fuse
			// would end the loop.
			log_w("camera error ended the clip; no more clips this visit");
			rtc_ep.video_done = true;
			return;
		}
		log_i("clip stopped on %s: checking presence for the next one",
		      CLIP_END_NAMES[c.end]);
	}
}

// Called once per wake, after any trigger: record if this episode's entry photo
// confirmed it and D1 is up. D1 up with the episode confirmed and recording not
// done is the entry itself, or a retrigger inside the gap that resumes the same
// visit; video_run() never returns without d1_high false or video_done, so no
// other wake finds all three. Nothing slow follows a clip cycle, so the last
// clip's deadline also covers what is left of the wake: a bare report is never
// due (send_clip() stamps the attempt) and the open episode sleeps directly,
// with no PIR_IDLE_MAX_S wait.
static void video_check()
{
#if VIDEO_ENABLED
	if (rtc_ep.open && rtc_ep.confirmed && !rtc_ep.video_done && rtc_ep.d1_high) {
		video_run();
	}
#endif
}

// Deployment mode's test clip (deploy_mode.h): the same recorder as a
// presence clip, with D1 out of it.
void deploy_test_clip(test_clip_t *out)
{
	memset(out, 0, sizeof(*out));
	clip_t c;
	if (!record_clip(&c, DEPLOY_TEST_CLIP_S, false)) {
		return;
	}
	out->ok     = true;
	out->avi    = c.buf;
	out->len    = c.len;
	out->frames = c.frames;
	out->dur_ms = c.dur_ms;
	out->fps    = c.fps;
}

// Milliseconds until `deadline_ms`, at least 50 (a 0 timer would mean none).
// The RTC slow clock can run a little fast over a sleep; a wake that lands
// early finds the deadline not yet reached and sleeps this last bit again.
static uint32_t ms_until(uint64_t deadline_ms)
{
	const uint64_t t = now_ms();
	return deadline_ms > t + 50 ? (uint32_t)(deadline_ms - t) : 50;
}

static uint32_t min_timer(uint32_t a, uint32_t b)
{
	return a == 0 ? b : b == 0 ? a : a < b ? a : b;
}

// Sleep until whatever the current state is waiting for. Never returns.
//   backoff:           the timer alone, D1 unarmed
//   episode, D1 high:  D1 falling
//   episode, D1 low:   D1 rising (a retrigger), or the gap running out
//   otherwise:         enter_deep_sleep(), the idle sleep
// The telemetry timer is folded into every one of them. If D1 has already
// changed since it was last seen, the level wake fires at once and the next
// wake handles the edge: nothing is lost, it just costs a boot.
[[noreturn]] static void sleep_for_state()
{
	const uint32_t report_ms = telegram_configured() ? telemetry_wake_in_s() * 1000u : 0;
	if (rtc_backoff) {
		park_for_sleep();
		const uint32_t left = (int32_t)(rtc_backoff_end_s - now_s()) > 0
		                          ? rtc_backoff_end_s - now_s() : 1;
		deep_sleep_now(-1, min_timer(left * 1000u, report_ms));
	}
	if (rtc_ep.open) {
		park_for_sleep();
		if (rtc_ep.d1_high) {
			deep_sleep_now(0, report_ms);
		}
		const uint32_t gap = ms_until(rtc_ep.fall_ms + PRESENCE_GAP_S * 1000u);
		deep_sleep_now(1, min_timer(gap, report_ms));
	}
	enter_deep_sleep();
}

// A wake inside an open episode: follow D1, and handle a retrigger.
static void episode_event(bool d1, uint32_t t_ref_ms)
{
	// The gap may have run out before this wake: nothing listens to D1 while a
	// clip uploads, for one. An edge after that is a new visit, with its own
	// entry photo and its own gate, not a retrigger of the old one; treating it
	// as one would record a stranger on the previous visitor's verdict.
	if (!rtc_ep.d1_high &&
	    now_ms() - rtc_ep.fall_ms >= (uint64_t)PRESENCE_GAP_S * 1000u) {
		episode_close("D1 quiet");
		if (d1) {
			episode_open();
			handle_trigger(t_ref_ms);
		}
		return;
	}
	if (d1 && !rtc_ep.d1_high) {
		rtc_ep.d1_high = true;
		log_i("retrigger %lu s into the episode",
		      (unsigned long)((now_ms() - rtc_ep.start_ms) / 1000));
		handle_trigger(t_ref_ms);
	} else if (!d1 && rtc_ep.d1_high) {
		rtc_ep.d1_high = false;
		rtc_ep.fall_ms = now_ms();
	}
}

// ---------------------------------------------------------------------------
// Deployment mode entry (config.h)
// ---------------------------------------------------------------------------
// True once BOOT has been held low continuously for DEPLOY_BUTTON_HOLD_MS.
// `window_ms` is how long to keep waiting for the press to *start*; a press
// already under way is always seen through to its verdict. window_ms == 0
// therefore costs one GPIO read when nothing is held, which is what the
// presence wake needs — 9.4 budgets 1.5 s for the whole wake-to-sent window — and what
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
// A deadline that passes during one of the light sleeps in
// wait_for_pir_idle() fires as soon as the node wakes: esp_light_sleep_start()
// winds esp_timer's counter forward by the time slept, and the S3's systimer
// raises an alarm whose target is already behind the counter. Those sleeps
// last a second each.
static void wake_deadline_expired(void *)
{
	esp_system_abort("wake deadline");
}

// config.h works WAKE_DEADLINE_S and WAKE_CLIP_DEADLINE_S out by hand.
// PIR_IDLE_MAX_S lives in this file, so this is where the sums can be checked.
// Seconds throughout; config.h derives each term.
//
// Pixels in a frame size, from esp32-camera's resolution table (sensor.c),
// which is not constexpr. Only the sizes this firmware might be set to; any
// other gives 0 and fails the build below, so the table gets extended rather
// than the check skipped.
static constexpr uint32_t framesize_pixels(framesize_t f)
{
	return f == FRAMESIZE_VGA   ?  640u *  480u :
	       f == FRAMESIZE_SVGA  ?  800u *  600u :
	       f == FRAMESIZE_XGA   ? 1024u *  768u :
	       f == FRAMESIZE_HD    ? 1280u *  720u :
	       f == FRAMESIZE_SXGA  ? 1280u * 1024u :
	       f == FRAMESIZE_UXGA  ? 1600u * 1200u :
	       f == FRAMESIZE_FHD   ? 1920u * 1080u :
	       f == FRAMESIZE_QXGA  ? 2048u * 1536u :
	       f == FRAMESIZE_QSXGA ? 2560u * 1920u :
	       f == FRAMESIZE_5MP   ? 2592u * 1944u : 0u;
}
static_assert(framesize_pixels(CAM_FRAMESIZE) != 0,
              "add CAM_FRAMESIZE to framesize_pixels()");

#define DNS_LOOKUP_MAX_S  (7 * CONFIG_LWIP_DNS_MAX_SERVERS)
#define TG_CONNECT_MAX_S  (DNS_LOOKUP_MAX_S + TELEGRAM_STALL_MS / 1000 + \
                           TELEGRAM_HANDSHAKE_S)
// Multipart head (chat_id, a caption cut to 1000 bytes, the file part's
// headers) and tail, rounded up.
#define TG_MULTIPART_MAX  1536
// One tg_post() of `bytes` with the floor `bps`, the rate term rounded up.
#define TG_POST_MAX_S(bytes, bps) (TG_CONNECT_MAX_S + TELEGRAM_POST_BASE_S + \
                                   ((bytes) + (bps) - 1) / (bps) +           \
                                   TELEGRAM_STALL_MS / 1000)
// The driver's JPEG buffer (cam_hal.c, FRAME_SIZE_AUTO) bounds a still.
#define STILL_MAX_BYTES   (framesize_pixels(CAM_FRAMESIZE) / 5 + TG_MULTIPART_MAX)
// One association, both phases, worst case.
#define WIFI_UP_MAX_S     ((WIFI_CONNECT_TIMEOUT_MS + WIFI_DHCP_TIMEOUT_MS) / 1000 + \
                           2 * DNS_LOOKUP_MAX_S)
#define CLIP_MAX_BYTES    (VIDEO_MAX_BYTES + TG_MULTIPART_MAX)
// What no single timeout bounds ahead of the entry photo: a capture, ~10 s at
// worst (init, CAM_WARMUP_MS, two 4 s fb_get() timeouts when a frame overflows
// its buffer), and a detection, allowed 10 s (it is seconds; nothing times it).
#define ENTRY_UNBOUNDED_S 20
// Likewise inside one clip cycle: camera bring-up ~1 s, up to two 4 s
// fb_get() timeouts (record_clip()), and the time cap checked up to one more
// 4 s timeout late.
#define CLIP_UNBOUNDED_S  (1 + 2 * 4 + 4)
// The reply window (config.h): REPLY_WAIT_S, which every request in it is cut to
// fit, plus 2 s for what runs between those checks (the last step of a TLS
// handshake, which is CPU-bound, and parsing).
#if REPLY_WINDOW_BUILT
#define REPLY_WINDOW_MAX_S (REPLY_WAIT_S + 2)
#else
#define REPLY_WINDOW_MAX_S 0
#endif
// A failed hostname refresh ahead of the normal path, at most once a wake: a
// lease that arrives at the deadline, then a DNS lookup that fails.
#if DHCP_REFRESH_ENABLED
#define DHCP_REFRESH_MAX_S (WIFI_DHCP_TIMEOUT_MS / 1000 + DNS_LOOKUP_MAX_S)
#else
#define DHCP_REFRESH_MAX_S 0
#endif
static_assert(framesize_pixels(VIDEO_FRAMESIZE) != 0 &&
              framesize_pixels(VIDEO_FRAMESIZE) <= 1280u * 720u,
              "VIDEO_FRAMESIZE must be HD or smaller (config.h)");
static_assert(PHOTOS_PER_EPISODE == 1,
              "the entry photo's verdict gates the episode; a second photo has "
              "no place in the flow (config.h)");
static_assert(VIDEO_MAX_CLIPS_PER_DAY >= 1, "VIDEO_MAX_CLIPS_PER_DAY is the fuse (config.h)");
static_assert(RADAR_UNMANNED_DELAY_S >= 10,
              "the LD2410S's unmanned delay is 10 s at the least (config.h)");
// Two deadlines, not one (config.h, Wake deadline). A visit has no clip limit of
// its own, so no single deadline could be both short and long enough for the
// 30 clips the fuse allows; each stretch gets its own instead.
//
// The entry stretch, from setup() to the first clip, or to the end of a wake
// that records none: the idle wait, a hostname refresh, the entry photo's
// capture and detection, and its association and upload (whose reply body,
// the photo's message_id, is read inside the same cap).
static_assert(WAKE_DEADLINE_S > PIR_IDLE_MAX_S + DHCP_REFRESH_MAX_S +
                                ENTRY_UNBOUNDED_S +
                                PHOTOS_PER_EPISODE *
                                    (WIFI_UP_MAX_S +
                                     TG_POST_MAX_S(STILL_MAX_BYTES, TELEGRAM_MIN_BPS)),
              "WAKE_DEADLINE_S no longer covers the entry stretch of a wake (config.h)");
// One clip cycle: the recording, the camera's own delays, a hostname refresh
// (the first association of a wake is a clip's when a retrigger resumes a
// visit), an association and the upload, and, after the first clip, the
// reply window on that association. Only the first cycle has a window, but
// every cycle is held to the same bound.
static_assert(WAKE_CLIP_DEADLINE_S > VIDEO_MAX_CLIP_S + CLIP_UNBOUNDED_S +
                                     DHCP_REFRESH_MAX_S + WIFI_UP_MAX_S +
                                     TG_POST_MAX_S(CLIP_MAX_BYTES, TELEGRAM_CLIP_MIN_BPS) +
                                     REPLY_WINDOW_MAX_S,
              "WAKE_CLIP_DEADLINE_S no longer covers one clip cycle (config.h)");

// One timer, re-armed: setup() starts it at WAKE_DEADLINE_S, and every clip
// cycle replaces what is left of it with a fresh WAKE_CLIP_DEADLINE_S. Never
// stopped for good: every path after setup() arms it ends in deep sleep, which
// takes the timer down with everything else.
static esp_timer_handle_t s_wake_timer = nullptr;

static void wake_deadline_arm(uint32_t seconds)
{
	if (!s_wake_timer) {
		esp_timer_create_args_t args = {};
		args.callback        = wake_deadline_expired;
		args.dispatch_method = ESP_TIMER_TASK;
		args.name            = "wake_deadline";
		if (esp_timer_create(&args, &s_wake_timer) != ESP_OK) {
			s_wake_timer = nullptr;
			log_e("wake deadline not armed; a hang will keep the node up");
			return;
		}
	} else {
		// ESP_ERR_INVALID_STATE if it is not running, which is fine: it is
		// about to be started again either way.
		esp_timer_stop(s_wake_timer);
	}
	if (esp_timer_start_once(s_wake_timer, (uint64_t)seconds * 1000000ULL) != ESP_OK) {
		log_e("wake deadline not armed; a hang will keep the node up");
	}
}

// ---------------------------------------------------------------------------
// setup(): the whole cycle. Control reaches loop() only in deployment mode.
// ---------------------------------------------------------------------------
void setup()
{
	// First thing, so wake-to-shutter (capture()) covers everything this
	// firmware does before the frame, Serial included.
	const uint32_t t_setup = millis();
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
	// The radar's OT2 is push-pull and drives D1 both ways; a pull would only
	// burn current while it is high (config.h, PIR_INTERNAL_PULLDOWN).
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
		memset(rtc_bssid, 0, sizeof(rtc_bssid));
		memset(rtc_clip_times, 0xFF, sizeof(rtc_clip_times));   // CLIP_SLOT_EMPTY
		rtc_initialised = true;
		log_i("cold boot");
	}

	const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

	// Deployment mode, before anything else acts on the wake: a reset boot
	// would otherwise spend the entry window on the air sending telemetry.
	// Presence and telemetry-timer wakes only check for a press already held.
	if (deploy_button_held((cause == ESP_SLEEP_WAKEUP_EXT0 ||
	                        cause == ESP_SLEEP_WAKEUP_TIMER)
	                           ? 0u : (uint32_t)DEPLOY_ENTRY_WINDOW_MS)) {
		log_i("BOOT held — deployment mode, sleep bypassed");
		// Whatever episode or backoff was running is over: the node comes
		// out of this mode through enter_deep_sleep(), listening afresh.
		episode_close("deployment mode");
		rtc_backoff = false;
		s_deploy_mode = true;
		deploy_mode_begin();
		return;   // setup() returns only here; loop() takes over
	}

	// Deployment mode has returned by now. Everything below ends in deep
	// sleep and runs against the wake deadline (config.h).
	wake_deadline_arm(WAKE_DEADLINE_S);

	if (cause != ESP_SLEEP_WAKEUP_EXT0 && cause != ESP_SLEEP_WAKEUP_TIMER) {
		// Power-on or reset, not a real trigger. Report once so a node that
		// has been reset in the field says so, then go to sleep.
		log_i("wake cause %d (not presence or timer)", (int)cause);

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

	// ---- The trigger flow (config.h) ---------------------------------------
	// Driven by the state in RTC memory and D1 as it is now, not by the wake
	// cause alone: a timer wake may be a backoff ending, an episode's gap
	// running out or the telemetry report, and ext0 means whichever level
	// sleep_for_state() armed. The capture, when there is one, is still the
	// first slow thing to happen (9.1).
	const bool d1 = gpio_get_level(PIN_PIR) == 1;

	if (rtc_backoff) {
		if ((int32_t)(now_s() - rtc_backoff_end_s) < 0) {
			// The telemetry timer, mid-backoff.
			maybe_send_telemetry_only();
			sleep_for_state();
		}
		backoff_finish();
		if (d1) {
			// Still high: exactly what the armed ext0 wake would fire on at
			// once, so take it as the trigger now.
			episode_open();
			handle_trigger(t_setup);
		}
	} else if (rtc_ep.open) {
		episode_event(d1, t_setup);
	} else if (cause == ESP_SLEEP_WAKEUP_EXT0) {
		episode_open();
		handle_trigger(t_setup);
	}
	// Anything else is the telemetry timer with nothing going on.

	// After the trigger, never before it: the entry photo is judged and sent
	// (radio down again) by the time a clip starts, so the camera and the
	// radio are never up together (9.1 step 5, 10).
	video_check();

	maybe_send_telemetry_only();
	sleep_for_state();
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
