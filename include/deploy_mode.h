// Deployment mode — the operator-facing half of the firmware.
//
// Entered by holding the XIAO's BOOT button shortly after start-up (see the
// deployment-mode block in config.h for the entry procedure and why the button
// cannot be sampled at the reset edge). In this mode the node never sleeps: it
// raises a WPA2 AP and serves an async web UI on 192.168.4.1 for aiming the
// lens, walk-testing the radar and reading the RTC telemetry counters.
//
// Split out of main.cpp so the normal duty cycle stays readable. The two
// halves meet only through the small surface below.
#pragma once

#include <stddef.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// Provided by main.cpp. Deployment mode reuses the real camera bring-up rather
// than configuring its own sensor, so what the preview shows is exactly what a
// presence capture would have produced.
// ---------------------------------------------------------------------------
bool     camera_up();
void     camera_down();
int32_t  battery_mv();
uint32_t now_s();

[[noreturn]] void enter_deep_sleep();

// Snapshot of the RTC slow-memory counters (PROJECT_BRIEF.md 9.3), copied out
// by main.cpp so deployment mode does not reach into its state directly.
struct deploy_status_t {
	uint32_t triggers_total;
	uint32_t triggers_since_report;
	uint32_t suppressed_total;
	uint32_t suppressed_since_report;
	uint32_t detect_errors_total;         // frames detection could not judge
	uint32_t detect_errors_since_report;
	uint32_t photos_sent_total;
	uint32_t photos_sent_since_report;
	uint32_t photos_dropped_total;
	uint32_t capped_total;                // edges inside a visit that has its photo
	uint8_t  wind_streak;                 // no-person photos in a row
	uint32_t backoff_left_s;              // 0 unless the radar is being ignored
	uint32_t backoff_s_total;             // seconds the radar has been ignored
	uint32_t clips_sent_total;
	uint32_t clips_dropped_total;         // recorded, not delivered
	uint32_t clip_s_total;                // seconds of clip recorded
	uint32_t clip_fuse_trips_total;       // recordings the daily fuse refused
	uint32_t replies_keep_total;          // reply windows (config.h) answered keep
	uint32_t replies_stop_total;          //   answered stop
	uint32_t replies_none_total;          //   with no answer, errors included
	uint32_t reply_errors_total;          //   that failed rather than went quiet
	uint32_t last_report_s;
	bool     have_ap_cache;
	uint8_t  ap_channel;
	uint8_t  ap_bssid[6];
	// True once the static block in config.h has been proven wrong for this
	// LAN and wakes are going straight to DHCP. Surfaced on the status page
	// because it is otherwise invisible in the field, and it is the single
	// most useful thing to know when a sited node has stopped reporting.
	bool     using_dhcp;
	// JPEG quality number the next still starts at (config.h,
	// CAM_QUALITY_STEP): CAM_JPEG_QUALITY unless a frame has overflowed.
	uint8_t  cam_quality;
};

void deploy_fill_status(deploy_status_t *out);

// The Home network card saved or forgot an address (netcfg.h). Clears
// rtc_use_dhcp, so the next wake tries the static path with what is now in
// force rather than staying on DHCP over the old address's failure.
void deploy_net_changed();

// The presence wake's photo path on demand, for the cold-capture test: capture()
// from a dead rail (power on, cold init, warm-up, one frame, rail off), then
// detection on that frame. Loop task only, with the camera already down.
enum {
	COLD_DETECT_OFF = 0,   // no detector in this build (bench-nodetect)
	COLD_DETECT_NONE,      // judged, no person
	COLD_DETECT_HIT,       // person at or above DETECT_SCORE_THRESHOLD
	COLD_DETECT_ERROR,     // could not judge (decode or model allocation)
};
struct cold_test_t {
	bool     ok;                   // a frame was captured
	uint32_t wake_to_shutter_ms;   // rail on to the kept frame's start
	uint32_t capture_ms;           // rail on to rail off
	uint8_t *jpeg;                 // PSRAM; the caller frees it (heap_caps_free)
	size_t   jpeg_len;
	int      quality;
	int      detect;               // COLD_DETECT_*
	float    score;
	uint32_t load_ms;              // detector construction and model load
	uint32_t decode_ms;
	uint32_t infer_ms;
};
void deploy_cold_test(cold_test_t *out);

// A DEPLOY_TEST_CLIP_S clip with the presence-video settings (config.h,
// VIDEO_*), D1 ignored, for the test-clip button. Loop task only, with
// the camera already down.
struct test_clip_t {
	bool     ok;
	uint8_t *avi;        // PSRAM, the finished file; the caller frees it
	size_t   len;
	uint32_t frames;
	uint32_t dur_ms;
	float    fps;        // measured, as written into the headers
};
void deploy_test_clip(test_clip_t *out);

// ---------------------------------------------------------------------------
// Provided by deploy_mode.cpp.
// ---------------------------------------------------------------------------

// Raise the AP and start the HTTP server. Returns with both running; setup()
// then simply returns, and loop() takes over.
void deploy_mode_begin();

// Called from loop(). Owns every camera power transition (so esp_camera_init()
// never runs on the async task) and services the captive-portal DNS.
void deploy_mode_service();
