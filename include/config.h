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

// Exposure and white-balance warm-up. The rail is cut between captures (5), so
// every wake is a cold sensor (9.1 step 3): AEC/AGC and AWB start from the
// register defaults, and the first frames are mis-exposed and off-colour.
//
// Timed, not counted. Convergence takes wall-clock time, and a fixed frame
// count means less at high resolution, where each frame takes longer. So
// frames are discarded until one *starts* at least CAM_WARMUP_MS after init
// and at least CAM_WARMUP_MIN_FRAMES have been thrown away. Both are bench
// starting points; deployment mode's cold-capture test reports the resulting
// wake-to-shutter time. Costs CAM_WARMUP_MS of camera-on per capture: at the
// ~250 mA active figure in 7, 800 ms is ~0.06 mAh.
//
// There is no early exit on "AEC settled". The installed ov5640.c only ever
// writes the stable-range limits (0x3A0F/10/1B/1E, set_ae_level()); nothing
// in it reads a converged flag back, and no status register for one could be
// confirmed from the sources here, so none is guessed at.
#define CAM_WARMUP_MS          800
#define CAM_WARMUP_MIN_FRAMES  3

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------
// Stills at the largest size the installed OV5640 driver produces: QSXGA,
// 2560 x 1920. FRAMESIZE_5MP (2592 x 1944, the sensor's full array) exists in
// esp32-camera's enum, but the OV5640 entry in its sensor.c table caps at
// QSXGA and esp_camera_init() clamps anything larger down to that with a
// warning, so asking for 5MP buys nothing.
//
// Detection no longer limits this: it decodes at a reduced scale
// (DETECT_DECODE_MAX_W), 640 x 480 from QSXGA. What does limit it is the lens
// and the upload. A third-party fixed-focus lens may not resolve 5 MP. If a
// 3 MP (FRAMESIZE_QXGA, 2048 x 1536) frame looks just as sharp in deployment
// mode's focus view, the smaller size is the better trade: every byte costs
// upload energy, and detection sees the same 640-wide decode either way.
//
// Size and memory at QSXGA:
//   - The driver's JPEG buffer is w x h / 5 = 983 kB, so no still is bigger
//     (CAM_QUALITY_STEP handles a scene that would be). A busy outdoor scene
//     at quality 10 is typically 0.5-1 MB.
//   - PSRAM while capturing: the driver's two buffers (2 x 983 kB) plus the
//     one copy capture() keeps, ~3 MB. That is the high point for stills.
//     Detection holds that copy, the 921 kB RGB888 decode and the model (the
//     pedestrian weights are 435 kB), ~3 MB with its working memory, the
//     driver's buffers already freed. Nothing buffers frames any more: each
//     wake handles one still at a time, so there is no frame count to cap.
//   - Uploading one: ~0.5 MB at ~200 kB/s is ~3 s on top of ~3 s of
//     association and TLS, ~0.4 mAh at 7's 250 mA; a full 983 kB still is
//     ~0.55 mAh. At the TELEGRAM_MIN_BPS floor a 983 kB still may take 62 s,
//     ~4.5 mAh with the handshake. At SVGA it was ~100 kB and ~0.25 mAh.
#define CAM_FRAMESIZE        FRAMESIZE_QSXGA
// 0..63, lower = better = bigger.
#define CAM_JPEG_QUALITY     10

// Frame-buffer overflow. In JPEG mode esp32-camera sizes each frame buffer at
// width x height / 5 (cam_hal.c, CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE_AUTO). A
// busy scene, foliage at high resolution and a fine quality, can outgrow it:
// the driver truncates the frame, drops it for its missing EOI marker, and
// esp_camera_fb_get() returns NULL only after its 4 s timeout. capture() then
// raises the quality number by CAM_QUALITY_STEP and tries once more.
//
// The quality that worked is kept in RTC memory for the next wake, so a busy
// scene pays the 4 s once rather than on every trigger. It creeps back one
// step at a time toward CAM_JPEG_QUALITY whenever a frame comes in under
// CAM_QUALITY_RELAX_PCT of the buffer, so one windy afternoon does not cost
// detail for the rest of the deployment. Telemetry reports the value in use.
//
// The Kconfig alternative, CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE_CUSTOM, exists
// but cannot take effect here: esp32-camera is a prebuilt archive in the
// Arduino libs, and the detect envs' custom_sdkconfig rebuild does not
// compile it. bench-nodetect has no rebuild at all.
#define CAM_QUALITY_STEP       4
#define CAM_QUALITY_RELAX_PCT  60
#define CAM_XCLK_HZ          20000000

// Sensor settings, applied by camera_apply_settings() after every init: the
// rail is cut between captures, so each wake starts from the driver's register
// table (9.1 step 3). Deployment mode's preview goes through the same
// camera_up(), so what it shows is what a PIR capture gets.
//
// Each default is the value the installed ov5640.c's register table already
// loads, checked setter by setter, so a stock build writes nothing new. Only
// CAM_JPEG_QUALITY above differs from before. CAM_KEEP means "do not call the
// setter", for the two settings whose setter cannot reproduce the table.
#define CAM_KEEP             99

// Orientation. 0/0 is the sensor's own; check it in the deployment preview.
#define CAM_VFLIP            0
#define CAM_HMIRROR          0

// ISP blocks in 0x5000, all on in the table (0xa7): lens shading correction,
// black- and white-pixel correction, raw gamma. 1 = on.
#define CAM_LENC             1
#define CAM_BPC              1
#define CAM_WPC              1
#define CAM_RAW_GMA          1

// -5..5 or CAM_KEEP. set_ae_level(0) targets a luminance of 55, brighter than
// the table's own AEC window (0x3A0F/10 = 0x30/0x28, about level -1, which no
// level reproduces exactly), so any value here is a change, not a default.
#define CAM_AE_LEVEL         CAM_KEEP

// AGC ceiling as the *raw* 0x3A18/19 value, in the sensor's 1/16 gain steps:
// ov5640.c's set_gainceiling() writes its argument straight into those two
// registers. The table has 0x0F8 = 248 = 15.5x. Do not pass a GAINCEILING_xX
// enum here: those are 0..6, which this driver would write as a ceiling of
// under 0.4x.
#define CAM_GAINCEILING      248

// -3..3 or CAM_KEEP. The table leaves sharpening automatic (0x5308 bit 6
// set); set_sharpness() clears that bit and switches to fixed offsets, so any
// value here gives up the automatic mode.
#define CAM_SHARPNESS        CAM_KEEP

// 0..8. 0 leaves 0x5308 bit 4 clear, as the table has it; 1..8 set it and
// write (n - 1) x 4 to 0x5306.
#define CAM_DENOISE          0

// 0x3A00 bit 2, the only thing set_aec2() touches. ov5640.c does not name the
// bit; it is usually described as the OV5640's night mode, which lets AEC
// stretch exposure past a frame in low light. Off: a longer exposure blurs a
// walking subject. Written explicitly rather than left to the sensor's reset
// value, which the register table does not set.
#define CAM_AEC2             0

// ---------------------------------------------------------------------------
// Trigger flow — replaces PROJECT_BRIEF.md 9.2
//
// 9.2's isolated/burst rule existed to decide when detection was worth its
// energy. Detection now runs on every photo, with the radio off, and decides
// what goes out, so the isolated/burst split, its trigger window and the
// light-sleep burst buffer are gone. This is a deliberate departure from the
// brief, which is left as it is.
//
// Every PIR trigger captures a still, detection judges it, and the radio
// comes up only for a photo that is going out. Between PIR events the node is
// in deep sleep, inside a presence episode as much as outside one.
//
// Energy per trigger, from the ~250 mA active figure in 7 (bench to confirm):
//   ~0.2 s boot, ~0.6 s cold init, CAM_WARMUP_MS, a frame   ~0.12 mAh
//   detection: model load, a reduced decode, inference      ~0.04 mAh
//   total for a photo that is judged and dropped            ~0.16-0.2 mAh
//   sending it: association, TLS, and the upload            +~0.4-0.55 mAh
//     for a QSXGA still at ~200 kB/s (see CAM_FRAMESIZE); ~4.5 mAh at the
//     TELEGRAM_MIN_BPS floor
// Against 7's ~9 mAh/day (8.2 of it sleep), ten person photos a day are
// ~6-7 mAh.
// ---------------------------------------------------------------------------

// 1: only photos the detector scores at DETECT_SCORE_THRESHOLD or above go
// out, plus any it could not judge (fail open). 0: every photo goes out,
// which is the only way animals get through: the models find people only.
// bench-nodetect has no detector and always sends unfiltered.
#define SEND_ONLY_PERSONS      1

// Sent photos per presence episode, at most. A lingering person keeps
// retriggering the PIR; the clip covers that. Once an episode has this many,
// its further triggers are counted ("capped") but not photographed, which
// saves a capture and a detection each.
#define PHOTOS_PER_EPISODE     3

// Presence episodes. One opens on a PIR rising edge when none is open, stays
// open while D1 is high or has been low for less than PRESENCE_GAP_S, and
// closes once D1 has been low that long. Kept in RTC memory.
#define PRESENCE_GAP_S         8

// AM312 hold time: how long D1 stays high after the last motion, in seconds
// (fractions allowed). 2.5 s is an estimate for a typical AM312, whose hold
// is fixed by the module at around 2-3 s and cannot be adjusted, unlike the
// potentiometer PIR boards that the brief's ~10 s figure fits. It is NOT a
// measurement of this unit. To measure it: deployment mode's PIR card shows
// how long D1 was last high; wave once, briefly, and that is the hold time
// plus the wave. Set it here; nothing else needs to change. It decides how
// late a presence clip can start (VIDEO_PRESENCE_MIN_S + PIR_HOLD_S).
#define PIR_HOLD_S             2.5

// Wind backoff. A branch in wind retriggers the PIR all day and never holds
// a person. After WIND_STREAK_BACKOFF photos in a row judged "no person", the
// node stops listening to the PIR for WIND_BACKOFF_MIN_S, doubling after each
// further such photo up to WIND_BACKOFF_MAX_S, and wakes on the timer alone.
// When the backoff ends the PIR is armed again; if D1 is still high, that is
// a trigger at once, so steady wind costs one capture per backoff period.
// A person photo ends it, and so does WIND_QUIET_RESET_S of listening
// without a single trigger.
//
// What that bounds, at ~0.18 mAh a capture, in wind that never stops:
//   WIND_BACKOFF_MAX_S   900: 96 captures/day, ~17 mAh/day, 15 min deaf
//   WIND_BACKOFF_MAX_S  1800: 48 captures/day,  ~9 mAh/day, 30 min deaf
//   WIND_BACKOFF_MAX_S  3600: 24 captures/day,  ~4 mAh/day, 60 min deaf
// Deaf means just that: a person who arrives mid-backoff is photographed
// when it ends, if D1 is still high then. Telemetry reports the time the PIR
// was ignored rather than inventing trigger counts for it. bench-nodetect has
// no detector, so no streak, so no backoff.
#define WIND_STREAK_BACKOFF    3
#define WIND_BACKOFF_MIN_S     60
#define WIND_BACKOFF_MAX_S     1800
#define WIND_QUIET_RESET_S     1800

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

// Detection decodes each JPEG at the smallest power-of-two reduction (1/1,
// 1/2, 1/4 or 1/8, the decoder's limit) that brings its width to this or less.
// The model's preprocessor resizes whatever it is given to its own input size,
// so a bigger decode buys nothing but time and PSRAM, and a full-size decode
// stops fitting at all at high resolution: 2592 x 1944 RGB888 is 15 MB, more
// than all 8 MB of PSRAM. 800 decodes SVGA at full size and QSXGA at 1/4
// (640 x 480, 0.9 MB).
#define DETECT_DECODE_MAX_W    800

// ---------------------------------------------------------------------------
// Presence video
//
// A clip is recorded when the PIR shows presence for VIDEO_PRESENCE_MIN_S or
// longer and, with VIDEO_REQUIRE_PERSON, the detector has confirmed a person
// during the episode. The camera and the radio are never on together (9.1
// step 5, 10): record with the radio off, camera off, then send.
//
// The 10-second rule is about motion, not the episode's age. The AM312 holds
// D1 high for PIR_HOLD_S after the last motion, so a 1 s walk-by alone keeps
// it high for PIR_HOLD_S; an age check would film everyone who passes. Motion
// is proven at episode time T >= VIDEO_PRESENCE_MIN_S by a rising edge at T,
// or by D1 still high at T + PIR_HOLD_S. With continuous motion there are no
// new edges, so the earliest clip starts about VIDEO_PRESENCE_MIN_S +
// PIR_HOLD_S after arrival, plus a boot and camera init: at 2.5 s, motion is
// proven at 12.5 s and the first frame is ~13.6 s in (~0.25 s boot, ~0.55 s
// power-up and cold init, VIDEO_WARMUP_MS). A 15 s visit therefore gets a
// clip, catching its last ~1.4 s and then the VIDEO_END_QUIET_S tail after D1
// falls at 17.5 s: ~7.9 s of clip in all, ~12.9 s for a 20 s visit. A visitor
// who leaves before ~13.6 s is in the photos only. (With the brief's 10 s
// hold it would be ~21 s, after a 15 s visitor left.)
//
// The person gate (VIDEO_REQUIRE_PERSON) keeps a branch in steady wind, which
// can hold D1 high for minutes, from being filmed. If a photo earlier in the
// episode held a person, the clip starts at once. If not, one fresh still is
// taken and judged, at most once per episode; no person means no clip this
// episode. A person there starts the clip straight away: the still is held
// in PSRAM through the recording and sent after it, before the clip's own
// upload, so the visitor is filmed instead of waiting out an association and
// an upload first. It counts toward PHOTOS_PER_EPISODE as any photo does. A
// trigger photo taken at the very moment motion is proven serves as that
// fresh check. bench-nodetect has no detector and records on the 10-second
// rule alone.
//
// Recording stops at the first of: D1 low for VIDEO_END_QUIET_S, the clip
// reaching VIDEO_MAX_CLIP_S, or the buffer filling. VIDEO_END_QUIET_S is
// shorter than PRESENCE_GAP_S, so a clip that ends on quiet leaves the
// episode open: it closes PRESENCE_GAP_S after D1 fell, and a visitor who
// moves again before then is the same visit, whose new edge can start another
// clip. If it stopped on a cap and D1 is still high once the clip is sent,
// another is recorded. Either way, at most VIDEO_MAX_CLIPS_PER_EPISODE. A
// clip that fails to send ends clips for the episode, since the next would be
// dropped too. PIR edges during recording are the same visitor: they keep the
// clip going but are not triggers, and no still is taken for them.
//
// The file is MJPEG in AVI (src/avi.cpp), built in one PSRAM buffer of
// min(VIDEO_MAX_BYTES, free PSRAM - VIDEO_PSRAM_RESERVE, largest free block),
// and sent with sendDocument as clip.avi, video/x-msvideo. sendVideo wants
// MP4/H.264, which is not worth encoding on the S3.
//
// Buffer against time: VGA JPEGs at VIDEO_JPEG_QUALITY 12 should run
// ~30-60 kB (SVGA at 14 was estimated at ~35-75 kB; VGA has 0.64 of the
// pixels, and 12 is a little bigger than 14), 240-480 kB/s at 8 fps, so 5 MB
// lasts ~11-22 s: the whole ~8-13 s clip of a 15-20 s visit even at the top
// of that range, with the quiet tail. A frame over the driver's buffer
// (w x h / 5, 61,440 bytes at VGA) is dropped by it, so record_clip()
// coarsens the quality once, as capture() does for stills. These sizes are
// estimates; deployment mode's test clip reports real ones.
//
// PSRAM, 8 MB. The worst case is the fresh check's still held through the
// recording at its 983,040-byte maximum (CAM_FRAMESIZE). Allowing 128 kB for
// everything else in PSRAM, and 256 kB for the WiFi driver and lwIP while
// sending (CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP; mbedTLS allocates
// internally), free PSRAM at each peak is:
//   recording: clip buffer, still, two 61,440-byte VGA frame buffers
//     8,388,608 - 131,072 - 983,040 - 122,880 - 5,242,880 = 1,908,736 (1.82 MB)
//   sending the still: clip buffer, still, WiFi; frame buffers freed
//     8,388,608 - 131,072 - 983,040 - 262,144 - 5,242,880 = 1,769,472 (1.69 MB)
//   sending the clip: the still freed by then          2,752,512 (2.62 MB)
// Keeping 1.5 MB free at both peaks allows a buffer of at most ~5.19 MB, so
// 5 MB; 6 MB would leave ~0.69 MB while the still is sent. With no still held
// (a person seen earlier in the visit) both peaks have ~2.6-2.8 MB free.
// VIDEO_PSRAM_RESERVE holds the same margin at run time whatever the
// allowances turn out to be: when less is free than budgeted, the buffer
// shrinks, not the margin.
//
// Energy per clip, from the ~250 mA active figure in 7 (bench to confirm;
// recording has the radio off, so 250 mA likely overstates that part):
//   camera init and warm-up, ~1 s                          ~0.07 mAh
//   recording, up to VIDEO_MAX_CLIP_S                      ~2.1 mAh at 30 s
//   association, TLS, a 5 MB upload at ~200 kB/s (~29 s)   ~2.0 mAh
//   total                                                  ~4.2 mAh
// At the TELEGRAM_CLIP_MIN_BPS floor the upload alone is ~5.9 mAh (~85 s),
// ~8.1 mAh a clip. The daily cap is what bounds this. VIDEO_MAX_CLIPS_PER_DAY
// 3 is ~12.6 mAh on a day that reaches it (~24 mAh if every upload crawled at
// the floor). That does NOT fit 7's ~9 mAh/day: 8.2 mAh of that is sleep,
// which leaves ~0.8 mAh/day for everything else, less than one clip. No cap
// of one or more fits. A node that hit this cap every day would draw
// ~22 mAh/day (~33 at the floor) and last ~5 months (~3.4 at the floor)
// instead of ~a year; clips on only a few days a week keep the average near
// budget.
// ---------------------------------------------------------------------------
#define VIDEO_ENABLED                1
#define VIDEO_PRESENCE_MIN_S         10
#define VIDEO_REQUIRE_PERSON         1
// Never above HD, the usual limit for M-JPEG playback on phones; main.cpp
// checks it.
#define VIDEO_FRAMESIZE              FRAMESIZE_VGA
#define VIDEO_FPS                    8
#define VIDEO_JPEG_QUALITY           12
// Short: a clip wants to start, and a clip's first frames going slightly off
// in colour cost less than the moment they would miss.
#define VIDEO_WARMUP_MS              300
#define VIDEO_END_QUIET_S            4
#define VIDEO_MAX_CLIP_S             30
#define VIDEO_MAX_BYTES              (5 * 1024 * 1024)
// Free PSRAM left once the clip buffer is allocated: the 1.5 MB margin plus
// the 256 kB the send that follows may allocate there in the detect builds
// (lwIP and the WiFi driver's buffers, CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP).
// Counted against all free PSRAM, not the largest block: the margin is free
// memory wherever it lies. See the PSRAM budget above.
#define VIDEO_PSRAM_RESERVE          ((1536 + 256) * 1024)
#define VIDEO_MAX_CLIPS_PER_EPISODE  3
// In any rolling 24 hours, by now_s(); a clip counts, by its start time, once
// it has actually been recorded. See the energy note above.
#define VIDEO_MAX_CLIPS_PER_DAY      3

// bench-nodetect has no detector to confirm a person with.
#if !DETECTION_ENABLED
#undef  VIDEO_REQUIRE_PERSON
#define VIDEO_REQUIRE_PERSON         0
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

// The station's DHCP hostname, set before every association (main.cpp,
// wifi_set_hostname()). It is what the router's client list shows for a
// DHCP lease; on the static-address fast path nothing is sent that carries
// it. Overridable from include/secrets.h, as DEPLOY_AP_SSID is. Not to be
// confused with DEPLOY_AP_SSID: that names deployment mode's own access
// point, this names the node as a client of someone else's.
#ifndef WIFI_HOSTNAME
#define WIFI_HOSTNAME    "iPhone"
#endif

// Hostname refresh. The name reaches the router only in DHCP, and with no
// admin access to the router there is no other way to name the node there.
// So once this long has passed since the last lease (or since power-on), the
// next wake that associates does it on DHCP instead of the static fast path,
// exactly as the DHCP fallback below does. The lease alone is the refresh,
// and that wake sends over it if TELEGRAM_HOST resolves on it; otherwise it
// falls through to the normal path for that wake, unchanged. A refresh that
// gets no lease within WIFI_DHCP_TIMEOUT_MS is still owed, and the first wake
// that associates HOSTNAME_REFRESH_RETRY_S or more after it tries again. Any
// lease counts, including one the DHCP fallback gets on a normal wake, and
// restarts the HOSTNAME_REFRESH_INTERVAL_S clock. Wakes that are not refresh wakes
// associate exactly as before. The capture is never affected: the radio
// comes up only after it (9.1).
//
// Cost: a DHCP exchange in place of an instant static GOT_IP, ~0.5-3 s more
// radio at ~250 mA (7), ~0.03-0.2 mAh a refresh, so ~0.03-0.2 mAh/day at
// 24 h. A refresh that gets no lease costs the full WIFI_DHCP_TIMEOUT_MS,
// ~0.56 mAh, and HOSTNAME_REFRESH_RETRY_S bounds how often that is paid while
// DHCP stays broken: at most 86400 / HOSTNAME_REFRESH_RETRY_S failures a
// day, 24 at 3600, ~13 mAh/day if the node associates at least hourly. It
// associates only to send, at least every TELEMETRY_MAX_SILENCE_S, so a quiet
// node pays ~2.2 mAh/day (four failures) and a day of visits more. 0 disables
// the refresh.
#define HOSTNAME_REFRESH_INTERVAL_S  86400
#define HOSTNAME_REFRESH_RETRY_S     3600

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

// Stills go out with sendDocument: the original JPEG, byte for byte, as a
// capture.jpg file. sendPhoto (0) has Telegram recompress the image to its
// own photo size and quality, which undoes the still settings above.
#define TELEGRAM_STILL_AS_DOCUMENT 1

// Upload limits. There are two, because no single fixed timeout suits both a
// 100 kB SVGA still and a multi-megabyte one on a weak link. The old
// TELEGRAM_TIMEOUT_MS ran from before the connect, so connect plus the whole
// upload had to fit in 15 s.
//
// TELEGRAM_STALL_MS is how long the link may make no progress. ssl_client.cpp
// takes connect()'s timeout as its socket timeout and uses it for the TCP
// connect and for every write, which fails after that long without a byte
// accepted; tg_post() also gives the reply that long after the last write.
//
// TELEGRAM_POST_BASE_S + bytes / TELEGRAM_MIN_BPS caps one upload and its
// reply, counted from the TLS session being up. TELEGRAM_MIN_BPS is the
// slowest link still worth the battery: at 16 kB/s a 1 MB still is allowed
// 30 + 64 s. A link that slow costs radio-on time at ~250 mA (7), about
// 7 mAh for that still, so the floor is an energy decision as much as a
// patience one. The camera is off throughout (9.1 step 5).
//
// Clips have their own floor, TELEGRAM_CLIP_MIN_BPS. At 16 kB/s a 5 MB clip
// would be allowed 30 + 328 s of radio, ~25 mAh; at 64 kB/s it is 30 + 82 s,
// and a link too slow for that drops the clip rather than the battery. The
// still floor stays at 16 kB/s, where a still is worth the wait.
//
// A cold TLS handshake on an ESP32-S3 is 1-3 s by itself; TELEGRAM_HANDSHAKE_S
// bounds it.
#define TELEGRAM_STALL_MS         15000
#define TELEGRAM_POST_BASE_S      30
#define TELEGRAM_MIN_BPS          16000
#define TELEGRAM_CLIP_MIN_BPS     64000
#define TELEGRAM_HANDSHAKE_S      10

// Chain validation off by default. See 9.6 for the reasoning: a node that may
// be unreachable for months fails silently and permanently the day a pinned CA
// rotates, which is a worse outcome than the MITM this would prevent. Set to 0
// and supply TELEGRAM_ROOT_CA if the threat model justifies it.
#define TELEGRAM_INSECURE_TLS     1

// Fixed boundary. Safe because the length is declared up front via
// Content-Length, so a JPEG that happens to contain this byte sequence is
// never scanned for it.
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
// This has to sit above the longest wake that is slow but not hung. That is
// a presence-video wake: a trigger photo, the video gate's fresh still (sent
// after the first clip is recorded), then VIDEO_MAX_CLIPS_PER_EPISODE clips,
// each recorded and sent before the next. Every term is a timeout or a cap
// in the code:
//
//       30 s  PIR_IDLE_MAX_S       D1 waited out before the idle sleep
//                                  (main.cpp)
//   +   29 s  8 + 21 s             a failed hostname refresh, once a wake
//                                  (HOSTNAME_REFRESH_INTERVAL_S): a lease at
//                                  WIFI_DHCP_TIMEOUT_MS that cannot resolve
//   +  414 s  2 x (54 + 153 s)     two stills: an association and a
//                                  tg_post() each
//   +  771 s  3 x (30 + 54 + 173)  the clips: VIDEO_MAX_CLIP_S of recording,
//                                  an association and a tg_post() of
//                                  VIDEO_MAX_BYTES each
//   = 1244 s
//
// An association is at most 54 s: WIFI_CONNECT_TIMEOUT_MS +
// WIFI_DHCP_TIMEOUT_MS, and a DNS lookup in wifi_reachable() on the static
// address and again on the DHCP lease.
//
// A DNS lookup ends only when lwIP gives up: four tries per configured
// server, 1, 1, 2 and 3 s apart, for up to CONFIG_LWIP_DNS_MAX_SERVERS (3)
// servers, so 21 s. NetworkManager::hostByName() adds no timeout of its own.
//
// One tg_post() of B bytes takes at most:
//      46 s  the connect: TELEGRAM_HOST resolved again (21 s; a cache hit
//            only while the record's TTL lasts), the TCP connect
//            (TELEGRAM_STALL_MS) and TELEGRAM_HANDSHAKE_S
//   +  TELEGRAM_POST_BASE_S + B / floor, the upload cap, where the floor is
//            TELEGRAM_MIN_BPS for a still, TELEGRAM_CLIP_MIN_BPS for a clip
//   +  15 s  one TELEGRAM_STALL_MS past the cap: the cap is checked between
//            writes, and a write returns only on progress or a full stall
// For a still, B is the driver's frame buffer at CAM_FRAMESIZE (w x h / 5; a
// bigger frame never reaches PSRAM) plus 1.5 kB of multipart head and tail.
// At QSXGA that is 985 kB, 62 s at the floor rate: 46 + 30 + 62 + 15 = 153 s.
// For a clip, B is VIDEO_MAX_BYTES plus the same, 82 s at the clip floor for
// 5 MB: 46 + 30 + 82 + 15 = 173 s.
//
// 32 minutes leaves 676 s for what no single timeout bounds: two captures,
// ~10 s each at worst (init, CAM_WARMUP_MS, and two 4 s fb_get() timeouts when
// a frame overflows its buffer), two detections of seconds each, and three
// video bring-ups of about a second, each with up to two 4 s fb_get()
// timeouts of its own (record_clip()). main.cpp checks the sum at compile time,
// so retuning a term past this fails the build instead of cutting slow wakes
// short. It stays one deadline for the whole wake: under 45 minutes,
// re-arming it per phase buys too little to be worth the complication. A hang
// that runs the full 32 minutes with the radio up costs something like
// 50 mAh, most of a week of the 7 budget, rather than the cell.
#define WAKE_DEADLINE_S          (32 * 60)

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

// Live preview size. Stills (/snapshot, the focus view, the cold-capture test)
// are CAM_FRAMESIZE. The sensor is initialised at CAM_FRAMESIZE and dropped to
// this size for streaming, because esp32-camera sizes its frame buffers once,
// at init: the sensor can switch below the init size on the fly but never
// above it. Must not be larger than CAM_FRAMESIZE.
#define DEPLOY_PREVIEW_FRAMESIZE  FRAMESIZE_VGA

// How long the sensor stays at full size after the last still request, so the
// focus view's back-to-back snapshots do not bounce it between sizes.
#define DEPLOY_FULL_HOLD_MS       3000

// After a size switch the first frame is partial and exposure is adapting to
// the new line timing; a still is never taken from a frame that started
// sooner than this after the switch.
#define DEPLOY_SWITCH_SETTLE_MS   500

// How long /snapshot waits for the loop task to produce a full-size still.
#define DEPLOY_STILL_WAIT_MS      15000

// Cold-capture test: the rail is held off this long before the test powers
// it back up, so the sensor and its decoupling genuinely lose power, as they
// do between PIR wakes (9.1 step 3).
#define DEPLOY_COLDTEST_OFF_MS    1000

// Deployment mode's test clip: this many seconds with the current VIDEO_*
// settings, served as test.avi to download, for checking phone playback
// without Telegram.
#define DEPLOY_TEST_CLIP_S        5
