// Deployment mode — the operator-facing half of the firmware.
//
// Entered by holding the XIAO's BOOT button shortly after start-up (see the
// deployment-mode block in config.h for the entry procedure and why the button
// cannot be sampled at the reset edge). In this mode the node never sleeps: it
// raises a WPA2 AP and serves an async web UI on 192.168.4.1 for aiming the
// lens, walk-testing the PIR and reading the RTC telemetry counters.
//
// Split out of main.cpp so the normal duty cycle stays readable. The two
// halves meet only through the small surface below.
#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------
// Provided by main.cpp. Deployment mode reuses the real camera bring-up rather
// than configuring its own sensor, so what the preview shows is exactly what a
// PIR capture would have produced.
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
	uint32_t last_report_s;
	bool     have_ap_cache;
	uint8_t  ap_channel;
	uint8_t  ap_bssid[6];
	// True once the static block in config.h has been proven wrong for this
	// LAN and wakes are going straight to DHCP. Surfaced on the status page
	// because it is otherwise invisible in the field, and it is the single
	// most useful thing to know when a sited node has stopped reporting.
	bool     using_dhcp;
};

void deploy_fill_status(deploy_status_t *out);

// ---------------------------------------------------------------------------
// Provided by deploy_mode.cpp.
// ---------------------------------------------------------------------------

// Raise the AP and start the HTTP server. Returns with both running; setup()
// then simply returns, and loop() takes over.
void deploy_mode_begin();

// Called from loop(). Owns every camera power transition (so esp_camera_init()
// never runs on the async task) and services the captive-portal DNS.
void deploy_mode_service();
