// Wildlife camera node — all field-tunable constants live here.
// PROJECT_BRIEF.md 9.2: "Tunable constants (keep in one place, they will need
// field adjustment)".
#pragma once

#include <driver/gpio.h>

// Credentials — WiFi and the Telegram bot token — live in include/secrets.h,
// which is gitignored. include/secrets.h.example is the template.
//
// __has_include keeps a fresh checkout building without one: every value it
// would define has a safe default below, and a node built with no secrets.h
// simply fails to associate and refuses to post, rather than failing to
// compile. That matters because bench-nodetect (11 step 5) is a power
// measurement that has no business needing a live bot token.
#if defined(__has_include)
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#endif

// ---------------------------------------------------------------------------
// Pins — PROJECT_BRIEF.md 8. VERIFIED against the schematic. Do not reassign
// the camera pins.
// ---------------------------------------------------------------------------
#define PIN_PIR        GPIO_NUM_2  // D1, RTC-capable, ext0 wake, AM312 active-high
#define PIN_CAM_POWER  GPIO_NUM_1  // D0, drives the 2N3904 base

// Whether D1 gets the ESP32-S3's internal pulldown. Leave it at 0.
//
// The AM312's output is push-pull, so it pulls D1 low by itself and needs no
// help. It is also not a bare CMOS pin: the module runs the sensor from its
// own 3.0 V regulator and feeds Vout through a 20 k series resistor (R2).
// Against the ~45 k internal pulldown that is a divider, and a triggered PIR
// then gives 3.0 V x 45 / (45 + 20) = ~2.1 V at D1 — under the 0.75 x VDD =
// ~2.5 V the S3 needs for a guaranteed high, on the node's only wake input
// (9.1).
//
// If a broken-wire failsafe is wanted (D1 left floating), fit it outside:
// >= 1 M from D1 to GND still leaves ~2.9 V at D1 when triggered. The
// internal pull is too strong for the job.
#define PIR_INTERNAL_PULLDOWN  0

// PROJECT_BRIEF.md 5: HIGH = NPN conducts = FET on = camera powered.
// LOW or floating = camera unpowered. Off is the deep-sleep/boot/reset
// default and MUST NOT be inverted.
#define CAM_POWER_ON_LEVEL   1
#define CAM_POWER_OFF_LEVEL  0

// SGM2036S LDO settle + OV5640 power-on sequence before SCCB will answer.
#define CAM_LDO_SETTLE_MS    50

// OV5640 AEC/AGC needs a couple of frames to converge after a cold start.
// Every wake is a cold start (PROJECT_BRIEF.md 9.1 step 3), so the first frame
// off the sensor is usually mis-exposed. Costs ~2 frame times of camera-on.
#define CAM_WARMUP_FRAMES    2

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------
// SVGA keeps the JPEG small (fast to send) and, more importantly, keeps the
// decoded RGB888 buffer for detection at ~1.4 MB instead of ~5.8 MB at UXGA.
#define CAM_FRAMESIZE        FRAMESIZE_SVGA  // 800x600
#define CAM_JPEG_QUALITY     12              // 0..63, lower = better = bigger
#define CAM_XCLK_HZ          20000000

// ---------------------------------------------------------------------------
// Burst filter — PROJECT_BRIEF.md 9.2
// ---------------------------------------------------------------------------
#define BURST_COUNT          3    // N: triggers within BURST_WINDOW to call it a burst
#define BURST_WINDOW         60   // W seconds
#define BURST_SETTLE         30   // quiet seconds before running detection
#define BURST_MAX_FRAMES     8    // PSRAM frames buffered per burst; oldest dropped
#define TRIGGER_LOG_SIZE     16   // RTC ring of trigger timestamps; >= BURST_COUNT

// Hard ceiling on how long one burst may hold us in light sleep. A genuinely
// pathological scene (branch in sustained wind) must not pin us there forever.
#define BURST_MAX_DURATION   600  // seconds

// ---------------------------------------------------------------------------
// Detection — PROJECT_BRIEF.md 9.2: "Model choice and the detection-confidence
// threshold should be configurable."
// ---------------------------------------------------------------------------
#define DETECT_MODEL_PEDESTRIAN 1
#define DETECT_MODEL_FACE       2

#ifndef DETECTION_ENABLED
#define DETECTION_ENABLED 1
#endif
#ifndef DETECT_MODEL
#define DETECT_MODEL DETECT_MODEL_PEDESTRIAN
#endif
// Applied to the model's own postprocessor via set_score_thr(), so lowering it
// below the model's compiled-in default genuinely loosens detection. Model
// defaults for reference: pedestrian 0.70, face 0.50. Raise it if the node
// sends junk; lower it if it misses people at the far end of the scene.
#ifndef DETECT_SCORE_THRESHOLD
#define DETECT_SCORE_THRESHOLD 0.50f
#endif

// ---------------------------------------------------------------------------
// WiFi — PROJECT_BRIEF.md 9.4: static IP and a cached BSSID/channel, so the
// common wake skips both the scan and the DHCP exchange.
//
// 9.4 says "no DHCP", and for the fast path that still holds: DHCP is never
// on the critical path of a healthy wake. What is new below is a *fallback*.
// A static address is a claim about a LAN that the node cannot verify and
// that the operator can invalidate without touching the firmware — moving the
// node to a different router, or renumbering the one it has. When that claim
// goes stale the node associates perfectly and is then unreachable forever,
// which is the one failure mode a box in a field cannot recover from on its
// own. DHCP is the recovery path, not the normal one.
//
// Set the credentials in include/secrets.h, not here. The defaults exist only
// so that a checkout without that file still compiles.
// ---------------------------------------------------------------------------
#ifndef WIFI_SSID
#define WIFI_SSID        "CHANGEME"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD    "CHANGEME"
#endif

// NET_DNS matters more than it used to: 9.6 resolves api.telegram.org, so a
// static config with a dead resolver now costs delivery, not just lookups.
#define NET_STATIC_IP    192, 168, 1, 50
#define NET_GATEWAY      192, 168, 1, 1
#define NET_SUBNET       255, 255, 255, 0
#define NET_DNS          192, 168, 1, 1

// Static phase. This is an association budget, not an addressing one: with a
// static address there is no DHCP round trip, so GOT_IP follows association
// immediately and 4 s is generous.
#define WIFI_CONNECT_TIMEOUT_MS   4000

// DHCP phase. Deliberately longer than the static phase — this one *does*
// include a DISCOVER/OFFER/REQUEST/ACK exchange with whatever server the LAN
// has, and a busy consumer router can take seconds to answer. Set to 0 to
// disable the fallback entirely and keep strict 9.4 behaviour.
#define WIFI_DHCP_TIMEOUT_MS      8000

// Whether to prove the static address actually works before trusting it.
//
// This is the reason the fallback needs more than a timeout to fire. A static
// configuration cannot fail the way DHCP fails: the core raises WL_CONNECTED
// on the GOT_IP event, and with a static address that event follows
// association whether or not the address means anything on this LAN. Point
// this node at a 192.168.0.x router while NET_STATIC_IP still says
// 192.168.1.50 and it will report itself connected, sail past every timeout,
// and drop every frame at the TLS connect instead.
//
// So the static phase is gated on a name lookup of TELEGRAM_HOST rather than
// on WL_CONNECTED alone. That probe is very close to free: it is the same
// resolution telegram.cpp's connect would perform moments later, and lwIP
// caches the result, so a healthy node pays for it once and the send reuses
// it. An unhealthy one converts a silent permanent outage into one DHCP retry.
#define WIFI_VALIDATE_STATIC      1

// Remember in RTC memory that the static block is wrong for this LAN and go
// straight to DHCP on subsequent wakes, instead of re-paying the static phase
// every single time. Set only when the static address fails the
// WIFI_VALIDATE_STATIC lookup and a DHCP lease then passes it. When the lookup
// fails on the lease too, the router's internet link or DNS forwarder is down,
// which says nothing about the static block: nothing is remembered, WiFi goes
// down, and that wake's frames are dropped at once rather than each waiting
// out a DNS timeout in the TLS connect. Cleared whenever association itself
// fails, so a node does not stay on DHCP forever after one bad night — the
// next full retry tries static again.
#define WIFI_REMEMBER_DHCP        1

// Start associating *while* detection runs, rather than after it (9.1).
//
// Set to 0 for the PROJECT_BRIEF.md 11 step 5 bench measurement: that step is
// characterising the load switch and the sleep floor, and overlapping the
// radio with inference changes the peak-current profile enough to muddy it.
// See section 10 — the 470 uF at the battery input is sized for a TX burst,
// not for a TX burst on top of esp-dl at 240 MHz.
#ifndef WIFI_EARLY_START
#define WIFI_EARLY_START          1
#endif

// ---------------------------------------------------------------------------
// Telegram delivery — PROJECT_BRIEF.md 9.6
// ---------------------------------------------------------------------------
#define TELEGRAM_HOST     "api.telegram.org"
#define TELEGRAM_PORT     443

// Set these in include/secrets.h. They default to empty so that a firmware
// image built from a clean checkout cannot post into anyone's chat: the send
// path checks for them and declines rather than trying.
#ifndef TELEGRAM_TOKEN
#define TELEGRAM_TOKEN            ""
#endif
#ifndef TELEGRAM_CHAT_ID
#define TELEGRAM_CHAT_ID          ""
#endif
// Optional; prefixed to every caption, for when several nodes share a chat.
#ifndef TELEGRAM_CAPTION_PREFIX
#define TELEGRAM_CAPTION_PREFIX   ""
#endif

// Covers DNS + TCP + TLS handshake + upload. Deliberately far longer than the
// 4 s the old LAN POST allowed: a cold TLS handshake on an ESP32-S3 is 1-3 s by
// itself, and the camera is already unpowered by this point (9.1 step 5), so
// waiting here costs radio time rather than the expensive kind.
#define TELEGRAM_TIMEOUT_MS       15000
#define TELEGRAM_HANDSHAKE_S      10

// Chain validation off by default. See 9.6 for the reasoning: a node that may
// be unreachable for months fails silently and permanently the day a pinned CA
// rotates, which is a worse outcome than the MITM this would prevent. Set to 0
// and supply TELEGRAM_ROOT_CA if the threat model justifies it.
#define TELEGRAM_INSECURE_TLS     1

// Fixed boundary. Safe because the body is assembled here and a JPEG cannot
// contain this byte sequence at a position that would matter -- the length is
// declared up front via Content-Length, so no scanning for it takes place.
#define TELEGRAM_BOUNDARY  "----wildlifenode6f2a91c4b7"

// ---------------------------------------------------------------------------
// Telemetry — PROJECT_BRIEF.md 9.3
// ---------------------------------------------------------------------------
// Battery sense: PROJECT_BRIEF.md 8 does not allocate a pin or specify a
// divider, and the XIAO ESP32S3 has no usable on-board battery divider. Leave
// this at 0 until a divider is actually fitted; telemetry then reports a null
// voltage rather than a fabricated one. D3/GPIO4 is ADC1_CH3 and free.
//
// The divider to fit first:
//
//   BAT+ -- 1 M --+-- D3 (GPIO4)
//                 |
//                 +-- 1 M ---- GND    ratio 2.0, ~2 uA from a full cell
//                 |
//                 +-- 100 nF - GND
//
// ~2 uA is under 1% of the ~340 uA sleep budget in 7. The 100 nF is not
// optional: the divider is a 500 k source, far too weak to charge the ADC's
// sampling capacitor during a conversion, so the capacitor is what the ADC
// actually samples.
#define BATT_SENSE_ENABLED   0
#define PIN_BATT_SENSE       GPIO_NUM_4
#define BATT_DIVIDER_RATIO   2.0f  // (R_top + R_bot) / R_bot

// Force a report even when nothing is being sent, so a node that suppresses
// everything for days still reports its trigger rate and battery. This is the
// only thing that gets a failed delivery noticed at all now that undelivered
// frames are dropped rather than persisted (9.6).
//
// This drives a real wake: enter_deep_sleep() arms the RTC timer for whatever
// is left of this period since the last report *attempt*, so a node that sees
// no motion at all still reports. That is the quiet-vs-mute case in 9.6.
// Each report is a full association plus TLS handshake, a few tenths of a
// mAh. A day with no motion pays for four of them, roughly a tenth of the 7
// daily budget, and that cost is accepted. Days with photos pay less, because
// every send restarts the clock. Counting from the attempt rather than the
// last success means an uplink outage costs one try per period, not one per
// wake.
#define TELEMETRY_MAX_SILENCE_S  (6 * 3600)

// Shortest telemetry timer ever armed. A report that is already owed (never
// tried since power-on, say, or overdue after a long burst) therefore wakes
// the node this far ahead rather than at once.
#define TELEMETRY_WAKE_FLOOR_S   (10 * 60)

// ---------------------------------------------------------------------------
// Wake deadline
// ---------------------------------------------------------------------------
// The detection builds turn the task watchdog off (CONFIG_ESP_TASK_WDT_INIT=n
// in platformio.ini), and the one bench-nodetect keeps only watches core 0's
// idle task, never the loop task this firmware runs on. So nothing else ends
// a wake that hangs in a TLS read that never returns or a library deadlock:
// the node would stay up, radio and all, until the DW01 cut the cell off (6).
//
// setup() arms a one-shot esp_timer for this long once it knows the wake is
// not deployment mode, and on expiry the node calls esp_system_abort(). Every
// build prints and reboots on a panic, so the next boot sees ESP_RST_PANIC
// and takes the abnormal-reset path: counted as a crash in the report (9.3),
// then back to sleep without the radio. Deployment mode is attended and never
// sleeps, so it is never on the clock.
//
// This has to sit comfortably above the longest normal wake, which is a burst
// held open to its cap and then delivered in full:
//
//     600 s  BURST_MAX_DURATION   burst held open until the cap
//   +  30 s  BURST_SETTLE         a last settle sleep, ended by a trigger
//   +  60 s  2 x PIR_IDLE_MAX_S   PIR held high after that trigger, and again
//                                 before deep sleep (main.cpp)
//   +  12 s  WIFI_CONNECT_TIMEOUT_MS + WIFI_DHCP_TIMEOUT_MS
//   + 120 s  BURST_MAX_FRAMES x TELEGRAM_TIMEOUT_MS
//   = 822 s
//
// 20 minutes is ~45% over that. The margin is for what those constants do not
// bound: camera bring-up, detection, DNS lookups, and TLS requests that run
// past TELEGRAM_TIMEOUT_MS (the handshake has its own TELEGRAM_HANDSHAKE_S,
// and the upload is only checked between writes). main.cpp checks the sum at
// compile time, so retuning a term past this fails the build instead of
// cutting long bursts short. A hang that runs the full 20 minutes with the
// radio up costs something like 30 mAh, a few days of the 7 budget, rather
// than the cell.
#define WAKE_DEADLINE_S          (20 * 60)

// ---------------------------------------------------------------------------
// Deployment mode — held-BOOT-button setup/aiming interface.
//
// Not in PROJECT_BRIEF.md: this is an operator-facing mode for siting the node
// (aim the lens, walk-test the PIR, read telemetry) without a laptop or a
// serial cable. It never sleeps and it is never entered by accident, so none
// of the power budget in 7 applies to it.
//
// GPIO 0 is an ESP32-S3 *strapping* pin. Holding BOOT across the reset edge
// makes the ROM select Joint Download Boot and the sketch never runs — chip
// behaviour, not something firmware can override. So the button is sampled
// after boot instead:
//
//   power up (or press RESET) with BOOT released, then press and hold BOOT
//   until the LED-free "deployment mode" line appears on serial / the
//   CAM-SETUP AP shows up (~1 s).
//
// Held at a PIR wake it is also honoured, so an already-deployed node can be
// put back into setup mode by waving at the PIR with BOOT held — no power
// cycle needed.
// ---------------------------------------------------------------------------
#define PIN_DEPLOY_BUTTON     GPIO_NUM_0  // XIAO BOOT button, shorts IO0 to GND
#define DEPLOY_BUTTON_ACTIVE  0           // active-low; the XIAO pulls IO0 up

// How long after boot we keep watching for the press to *start*. A press
// already in progress is always seen through to its verdict, so a PIR or
// telemetry-timer wake passes 0 here and pays a single GPIO read when nothing
// is held.
#define DEPLOY_ENTRY_WINDOW_MS   3000
#define DEPLOY_BUTTON_HOLD_MS    250   // continuous low needed to count as held
#define DEPLOY_BUTTON_POLL_MS    5

// Both can be overridden from include/secrets.h, which is where a real
// password belongs: this file is committed.
#ifndef DEPLOY_AP_SSID
#define DEPLOY_AP_SSID        "CAM-SETUP"
#endif
// 8..63 chars for WPA2; set to nullptr for an open AP. Anyone associated can
// see the live preview, so leaving this set is the sane default.
#ifndef DEPLOY_AP_PASSWORD
#define DEPLOY_AP_PASSWORD    "changeme123"
#endif
#define DEPLOY_AP_CHANNEL     1
#define DEPLOY_AP_MAX_CLIENTS 2
#define DEPLOY_AP_IP          192, 168, 4, 1
#define DEPLOY_AP_NETMASK     255, 255, 255, 0
#define DEPLOY_HTTP_PORT      80

// Answer every DNS query with our own address so a phone's captive-portal
// probe opens the page on association instead of dropping the network.
#define DEPLOY_CAPTIVE_DNS    1

// Even in an attended mode the camera is still the biggest load (7), so it is
// powered down again once nothing has asked for a frame for this long. The
// next request powers it back up.
#define DEPLOY_CAM_IDLE_MS    15000

// Back-off after a failed esp_camera_init(). Without it a sensor that will not
// start gets re-initialised — and its rail cycled — every service tick.
#define DEPLOY_CAM_RETRY_MS   2000
