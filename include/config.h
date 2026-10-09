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
// PIN_PIR is the firmware's one presence input, and "D1" below means this pin.
// The brief (1, 8) puts an AM312 PIR on it. This node's sensor is an LD2410S
// radar whose OT2 digital output drives it instead, active-high like the
// AM312 was; the brief predates the radar and is not edited (12 already
// anticipates a sensor swap on the same GPIO). The PIR_* and "PIR" names in
// the code are that history: they mean this input. PIR_INTERNAL_PULLDOWN below
// argues from the radar's output, not the AM312's.
#define PIN_PIR        GPIO_NUM_2  // D1, RTC-capable, ext0 wake, active-high
#define PIN_CAM_POWER  GPIO_NUM_1  // D0, drives the 2N3904 base

// Whether D1 gets the ESP32-S3's internal pulldown. Leave it at 0: no internal
// pull is needed, up or down.
//
// The LD2410S's OT2 is a push-pull 0-3.3 V output wired straight to D1, with
// nothing in series. It drives D1 low when nobody is there and high when
// somebody is, so the line never floats while the radar is powered and a pull
// has nothing to hold. Its high is the radar's own 3.3 V, over the
// 0.75 x VDD = ~2.5 V the S3 needs for a guaranteed high on the node's only
// wake input (9.1).
//
// The reason this used to be 0 is gone. The AM312 module fed its output
// through a 20 k series resistor (R2), which made a divider with the ~45 k
// pull and left a triggered PIR at ~2.1 V, under that threshold. With no series
// resistor there is no divider, so the pull would no longer break the high, but
// it would cost: ~45 k against a driven 3.3 V is ~73 uA for as long as OT2 is
// high. A radar holds OT2 high for as long as someone stands in range, and the
// node deep-sleeps through that inside an episode (sleep_for_state()), so the
// pull would add a fifth again to the sleep budget in 7 for those hours.
//
// If a broken-wire failsafe is wanted (D1 left floating if the lead comes
// off), fit it outside: >= 1 M from D1 to GND costs ~3.3 uA while OT2 is high
// and leaves the high untouched. The internal pull is too strong for the job.
#define PIR_INTERNAL_PULLDOWN  0

// Sleep current with the radar. PROJECT_BRIEF.md 7 is now STALE and has to be
// updated by hand: its sleep table has the AM312 at ~15 uA, and the LD2410S
// draws ~45 uA at its minimum refresh rate (the setting tools/radar_config.py
// writes; a faster refresh draws more, and these figures then no longer hold).
//   ESP32-S3 deep sleep + Sense board baseline   ~64 uA   (7, unchanged)
//   R9 leak into the gated camera               ~260 uA   (7, unchanged)
//   LD2410S at minimum refresh                   ~45 uA   (7 has AM312 ~15 uA)
//   total sleeping                              ~370 uA   (7 has ~340 uA)
// That is ~8.9 mAh/day before a single wake, up from ~8.2, and 3400 mAh of
// sleep alone lasts ~380 days, not ~415. 11 step 5's bench figure moves the
// same way: expect ~370 uA, not ~340. The sleep figures in this file use
// ~370; where they cite 7 for it, read 7 as corrected here.

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
// camera_up(), so what it shows is what a presence capture gets.
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
// Trigger flow — replaces PROJECT_BRIEF.md 9.2, and departs from 1, 8 and 9.1
//
// The brief is left as it is. Where the firmware departs from it:
//   9.2   The isolated/burst rule existed to decide when detection was worth
//         its energy. Detection now runs on the first photo of every episode,
//         radio off, and that one verdict gates the whole episode. The
//         isolated/burst split, its trigger window and the light-sleep burst
//         buffer are gone.
//   1, 8  The presence sensor is an LD2410S radar, its OT2 output on PIN_PIR,
//         not the AM312 PIR. A radar holds OT2 high for its own unmanned delay
//         after the last presence, as the PIR held its output after the last
//         motion, so "D1 high" still means present-or-just-was. Unlike the
//         PIR it also holds OT2 high for as long as someone stays in range,
//         standing still included.
//   9.1   "Return to deep sleep" after one photo no longer holds for a visit
//         the AI confirms: the node stays awake and records until the radar
//         says the visit is over. The order inside the wake does hold: camera
//         first, radio after (step 5, 10), for the video as for the photo.
//   9.6   Still holds. Nothing is spooled: a photo or a clip that cannot be
//         sent on the wake that took it is dropped, and counted.
//
// Each presence episode, at the OT2 rising edge that opens it:
//   1. Capture ONE photo at once, camera before radio (9.1).
//   2. Person detection on it, radio off:
//        no person  Send nothing, record nothing; count it suppressed and go
//                   back to sleep. This one verdict gates the whole episode, so
//                   wind, a pet or the radar's back lobe cost a capture and a
//                   detection and produce no traffic.
//        can't run  Fail open: treated as a person (photo sent, recording
//                   starts), counted as a detect error.
//        person     Send the photo, the instant alert, tagged #TAG.
//   3. Record the first clip at once and send it (Presence video, below).
//   4. Wait up to REPLY_WAIT_S for "keep" or "stop" (Reply window, below).
//   5. On "keep", record on for as long as the radar shows presence;
//      otherwise the first clip was the only one.
// There is no second look: no fresh check, no further photos
// (PHOTOS_PER_EPISODE), no person gate on the video. The entry photo's result
// stands for the episode. The old rule that held a clip back until
// VIDEO_PRESENCE_MIN_S (10 s) of motion was a PIR-era false-trigger proxy, made
// redundant by radar presence plus this AI confirmation, and is removed with
// its timing code.
//
// What that costs: a person the radar sees before the camera does (outside the
// lens, or beyond the detector's range) is judged on an empty frame, and the
// episode stays gated off for as long as OT2 stays high, since nothing looks
// again. The next chance is the next episode.
//
// bench-nodetect (DETECTION_ENABLED 0) has no detector, so the AI gate cannot
// run there. It records on presence alone, no gate: the entry photo goes out
// unfiltered and every episode counts as confirmed, because that is the only
// way the build does anything. It is the build for 11 step 5, a power
// measurement, not a deployment. SEND_ONLY_PERSONS 0 does the same in a detect
// build.
//
// Between sensor edges the node is in deep sleep, inside an episode as much as
// outside one. Costs per photo, clip and episode are in the energy note under
// Presence video.
// ---------------------------------------------------------------------------

// 1: the entry photo's detection is the gate (above). A photo scored at
// DETECT_SCORE_THRESHOLD or above, or one the detector could not judge (fail
// open), goes out and the episode records; any other sends nothing and records
// nothing. 0: no gate. Every episode counts as confirmed and its photo and
// video go out for anything that holds the radar, which is the only way animals
// get through, since the models find people only. (That now covers the video
// too: the separate VIDEO_REQUIRE_PERSON gate is gone.) bench-nodetect has no
// detector and always runs ungated.
#define SEND_ONLY_PERSONS      1

// Photos per presence episode: the entry photo and no more; the video covers
// the rest of the visit. A rising edge inside an episode that has its photo is
// counted ("capped") and nothing is captured, which saves a capture and a
// detection each. It stays a macro so the figures in the energy note read from
// it, but the flow takes the entry photo's verdict as the whole episode's gate
// and cannot take a second one, so main.cpp asserts it is 1.
#define PHOTOS_PER_EPISODE     1

// Presence episodes. One opens on an OT2 rising edge when none is open, stays
// open while D1 is high or has been low for less than PRESENCE_GAP_S, and
// closes once D1 has been low that long. Kept in RTC memory. Inside one it is a
// single visit: the gate was settled at its edge, so a person who steps out and
// back within the gap gets no second photo and does not lose the recording.
#define PRESENCE_GAP_S         8

// The radar's unmanned delay: how long OT2 stays high after the last presence
// the LD2410S sees. It replaces the AM312's fixed ~10 s output pulse (1, and
// 12's note on it) and the PIR_HOLD_S that once modelled it.
//
// This constant does not set anything. The delay is a parameter stored in the
// radar itself, written once over its UART by tools/radar_config.py; the
// firmware never talks to the radar. So this only tells the firmware what the
// radar was given, and it MUST match it: run the tool with the same value
// (it reads this line for its default). 10 is the datasheet's minimum.
//
// What it lengthens or bounds:
//   - every clip's tail: OT2 falls this long after the person leaves, and the
//     clip runs on VIDEO_END_QUIET_S past that (Presence video, below)
//   - PIR_IDLE_MAX_S (main.cpp), the wait for D1 to fall before the idle
//     sleep, is derived from it so that it always outlasts it
// Deployment mode's walk test measures it: "last high" after one brief pass,
// walking right out of the radar's range, should read about this.
#define RADAR_UNMANNED_DELAY_S 10

// Wind backoff. A branch in wind can retrigger the radar all day and never
// hold a person. After WIND_STREAK_BACKOFF photos in a row judged "no person",
// the node stops listening to the radar for WIND_BACKOFF_MIN_S, doubling after
// each further such photo up to WIND_BACKOFF_MAX_S, and wakes on the timer
// alone. When the backoff ends D1's wake is armed again; if D1 is still high,
// that is a trigger at once, so steady wind costs one capture per backoff
// period. A person photo ends it, and so does WIND_QUIET_RESET_S of listening
// without a single trigger.
//
// What that bounds, at ~0.18 mAh a capture, in wind that never stops:
//   WIND_BACKOFF_MAX_S   900: 96 captures/day, ~17 mAh/day, 15 min deaf
//   WIND_BACKOFF_MAX_S  1800: 48 captures/day,  ~9 mAh/day, 30 min deaf
//   WIND_BACKOFF_MAX_S  3600: 24 captures/day,  ~4 mAh/day, 60 min deaf
// Deaf means just that: a person who arrives mid-backoff is photographed
// when it ends, if D1 is still high then. Telemetry reports the time the
// radar was ignored rather than inventing trigger counts for it. bench-nodetect has
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
// When the entry photo confirms a person (Trigger flow, above) the node records
// for as long as the radar shows presence, and sends what it recorded. Nothing
// holds the first clip back. The old rule that waited for VIDEO_PRESENCE_MIN_S
// (10 s) of motion, the motion-proof timing built on PIR_HOLD_S, and the
// VIDEO_REQUIRE_PERSON gate with its one fresh still are all removed. The 10 s
// rule was a PIR-era false-trigger proxy: a PIR cannot tell a person from a
// branch, so "it kept moving for 10 s" stood in for "worth filming". Radar
// presence plus the AI confirmation at the edge do that job, which makes the
// proxy redundant. The camera and the radio are never on together (9.1 step 5,
// 10): record with the radio off and the camera off, then send.
//
// The first clip always, at once; then, on a "keep" answer (Reply window,
// below), back to back, with no limit per visit (VIDEO_MAX_CLIPS_PER_EPISODE
// is removed). With "stop" or no answer, the first clip is the visit's only
// one: it is recorded and sent as below, and nothing follows it, not even
// after a retrigger inside PRESENCE_GAP_S. A clip ends at the first of:
//   - D1 low for VIDEO_END_QUIET_S, the quiet window: presence has ended. The
//     radar's own unmanned delay (RADAR_UNMANNED_DELAY_S) is already inside
//     OT2, so a clip runs on for that delay plus VIDEO_END_QUIET_S after the
//     person leaves, 14 s at the defaults
//   - VIDEO_MAX_CLIP_S
//   - the buffer filling
// On "keep", a clip that ended on one of the last two is sent and, if D1 is
// still high, the next starts straight away; so on for as long as the visit
// lasts. Recording stops only when presence ends, when a send fails (the
// photo's or a clip's: the uplink is down and the next clip would be dropped
// too), at the daily fuse below, or, without "keep", after the first clip.
//
// At each clip boundary D1 is sampled once. Low means the visit ended while the
// photo or the previous clip was being sent: no further clip. VIDEO_END_QUIET_S
// is shorter than PRESENCE_GAP_S, so the episode stays open until
// PRESENCE_GAP_S after D1 fell, and a rising edge inside that is the same
// visit: it resumes recording, with no second photo. Past the gap an edge is a
// new episode, with its own photo and its own gate.
//
// Two limits on "the full visit". The clips are not gapless: a clip fills most
// of PSRAM, so it must be sent before the next is recorded, and camera and
// radio never overlap, so the visit goes unrecorded during every upload. That
// is about half of it at ~200 kB/s (30 s recorded, ~29 s uploading) and about a
// quarter at the TELEGRAM_CLIP_MIN_BPS floor, and on "keep" the reply window
// after the first clip adds its wait to that first gap unless the answer was
// already in. And the first clip starts after the entry photo is sent,
// because that photo is the instant alert. Boot, capture, detection, an
// association and an upload come first: roughly 8-13 s from the edge (an
// estimate from the figures in this file: ~2 s to a photo in PSRAM, ~1 s of
// detection, ~4-9 s to send it, ~1 s of camera bring-up; the log has the real
// times). The radar holds OT2 for RADAR_UNMANNED_DELAY_S after the person
// leaves, so D1 is still high then for anyone seen within ~0-3 s of the edge:
// nearly every confirmed visit gets its first clip, though a brief pass may
// have left the frame by then.
//
// The file is MJPEG in AVI (src/avi.cpp), built in one PSRAM buffer of
// min(VIDEO_MAX_BYTES, free PSRAM - VIDEO_PSRAM_RESERVE, largest free block),
// and sent with sendDocument as clip.avi, video/x-msvideo. sendVideo wants
// MP4/H.264, which is not worth encoding on the S3.
//
// Buffer against time: VGA JPEGs at VIDEO_JPEG_QUALITY 12 should run
// ~30-60 kB (SVGA at 14 was estimated at ~35-75 kB; VGA has 0.64 of the
// pixels, and 12 is a little bigger than 14), 240-480 kB/s at 8 fps, so 5 MB
// lasts ~11-22 s. A frame over the driver's buffer (w x h / 5, 61,440 bytes at
// VGA) is dropped by it, so record_clip() coarsens the quality once, as
// capture() does for stills. These sizes are estimates; deployment mode's test
// clip reports real ones.
//
// PSRAM, 8 MB. No photo is held while a clip is recorded or sent: the entry
// photo is judged, sent and freed before the first clip's buffer is allocated,
// so the still stage (the driver's two 983,040-byte buffers, the copy and
// detection, ~3 MB) never coexists with a clip buffer. The record+send peak is
// therefore the clip buffer plus the camera's or the radio's own share.
// Allowing 128 kB for everything else in PSRAM, and 256 kB for the WiFi driver
// and lwIP while sending (CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP; mbedTLS
// allocates internally):
//   recording: clip buffer, two 61,440-byte VGA frame buffers
//     in use 5,242,880 + 122,880 + 131,072 = 5,496,832 (5.24 MB)
//     free   8,388,608 - 5,496,832         = 2,891,776 (2.76 MB)
//   sending the clip: clip buffer, WiFi; frame buffers freed
//     in use 5,242,880 + 262,144 + 131,072 = 5,636,096 (5.38 MB)
//     free   8,388,608 - 5,636,096         = 2,752,512 (2.62 MB)
// Sending is the peak, and a long visit repeats it for every clip. Budgeted,
// not measured. The still held through recording that used to cap the buffer at
// ~5.19 MB is gone, so VIDEO_MAX_BYTES could grow; it is left at 5 MB.
// VIDEO_PSRAM_RESERVE holds the margin at run time whatever the allowances turn
// out to be: when less is free than budgeted, or fragmented, the buffer
// shrinks, not the margin.
//
// Energy, from the ~250 mA active figure in 7 (bench to confirm; recording has
// the radio off, so 250 mA likely overstates that part). None of it includes
// sleep, which is ~370 uA with the radar (Sleep current, under Pins), not 7's
// ~340 uA AM312 build.
//   entry photo, judged and sent: boot, cold init, warm-up and a frame ~0.12,
//     detection ~0.04, association + TLS + a QSXGA upload ~0.4-0.55  ~0.6-0.7 mAh
//   entry photo, judged and dropped (no person)                      ~0.16-0.2 mAh
//   one clip: camera init + warm-up ~0.07, recording at 250 mA, then
//     association + TLS + upload:
//       filling its 30 s cap (~5 MB, ~29 s up at ~200 kB/s) 0.07 + 2.1 + 2.0 = ~4.2 mAh
//       the same at the TELEGRAM_CLIP_MIN_BPS floor (~85 s) 0.07 + 2.1 + 5.9 = ~8.1 mAh
//       ~10 s (2.4-4.8 MB, 15-27 s up with the handshake) 0.07 + 0.7 + 1.0-1.9 = ~1.8-2.6 mAh
// Per episode:
//   no person                                                 ~0.2 mAh
//   short visit, 1 photo + 1 clip of ~10 s            ~0.6-0.7 + 1.8-2.6 = ~2.4-3.3 mAh
//   the same with the clip filling its 30 s cap               ~0.7 + 4.2 = ~4.9 mAh
//   long visit that reaches the fuse, 1 photo + 30 clips   ~0.7 + 30 x 4.2 = ~127 mAh
//     (~244 mAh if every upload crawls at the floor)
// The node is awake at ~250 mA for the whole of a visit, so the rule of thumb
// is ~4.2 mAh per minute of presence, however it splits between recording and
// uploading. The fuse bounds clips, not minutes: 30 clips span ~30 min awake at
// ~200 kB/s and ~58 min at the floor.
//
// Against 7's ~9 mAh/day: sleep alone is now ~8.9 of it with the radar (was
// 8.2), which leaves almost nothing for everything else, so any day with a
// visit overspends it. A fuse day is ~127 + 8.9 = ~136 mAh (~253 at the
// floor): 15 days (28) of 7's budget in one, ~4% (~7%) of the cell. A node that
// hit the fuse every day would last ~25 days (~13), not a year. The old limits
// (3 clips a visit, 3 a day) held that worst case near 22 mAh/day (~33 at the
// floor).
//
// Recording the whole visit is expensive BY DESIGN. A long genuine visit costs
// what it costs, ~4 mAh a minute; the fuse is there to stop a stuck-on radar,
// not to ration visitors.
// ---------------------------------------------------------------------------
#define VIDEO_ENABLED                1
// Never above HD, the usual limit for M-JPEG playback on phones; main.cpp
// checks it.
#define VIDEO_FRAMESIZE              FRAMESIZE_VGA
#define VIDEO_FPS                    8
#define VIDEO_JPEG_QUALITY           12
// Short: a clip wants to start, and a clip's first frames going slightly off
// in colour cost less than the moment they would miss.
#define VIDEO_WARMUP_MS              300
// The quiet window: D1 low this long ends a clip, and with it the recording.
#define VIDEO_END_QUIET_S            4
#define VIDEO_MAX_CLIP_S             30
#define VIDEO_MAX_BYTES              (5 * 1024 * 1024)
// Free PSRAM left once the clip buffer is allocated: the 1.5 MB margin plus
// the 256 kB the send that follows may allocate there in the detect builds
// (lwIP and the WiFi driver's buffers, CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP).
// Counted against all free PSRAM, not the largest block: the margin is free
// memory wherever it lies. See the PSRAM budget above.
#define VIDEO_PSRAM_RESERVE          ((1536 + 256) * 1024)
// THE DAILY FUSE. This is a fuse, not a normal limit: it exists only to stop a
// stuck-on radar or a pathological day from flattening the cell, and at 30 it
// never fires in normal use. In any rolling 24 hours, by now_s(), a clip counts
// by its start time once it has actually been recorded. When the count reaches
// this, recording is refused (for that episode and the ones after it, until the
// oldest clip is 24 h old); the entry photo still goes out. Every refusal is
// counted and reported ("clip fuse ... TRIPPED" in the caption and on the
// status page). A fuse day costs ~135 mAh: see the energy note above.
#define VIDEO_MAX_CLIPS_PER_DAY      30

// ---------------------------------------------------------------------------
// Reply window — not in PROJECT_BRIEF.md, and a departure from 9.6, which
// only ever sends: the node now also reads the chat, for one short window per
// visit.
//
// A confirmed visit goes photo, first clip, window: the entry photo is sent
// and the first clip is recorded at once, exactly as before the window
// existed. Once that clip is delivered, the radio stays up on the same
// association for up to REPLY_WAIT_S, in modem sleep, and the node reads the
// Bot API's getUpdates for an answer. The answer decides the SECOND clip:
//   "keep" or "yes"  carry on: clips back to back for as long as the radar
//                    shows presence, no limit per visit (the daily fuse,
//                    VIDEO_MAX_CLIPS_PER_DAY, still applies)
//   "stop" or "no",  no second clip. The first was recorded and sent whatever
//   or no answer     the answer: a reply never recalls footage, and no answer,
//                    the common case, is the same as stop.
// An answer given any time after the photo counts, while the clip was still
// being recorded or sent included: the window's first read picks it up and
// ends the window at once. Otherwise the window waits, so REPLY_WAIT_S is the
// time left to answer after the first clip lands in the chat, enough to watch
// it first. The camera is off throughout the window (9.1 step 5, 10); a kept
// visit's second clip starts when the answer is in, so a "keep" sent before
// the first clip lands costs no extra gap, and one sent later puts that wait
// between the first two clips.
//
// How to answer: swipe-reply to the photo or the first clip with keep or stop
// (a reply to the bot's own message reaches it even in a group with privacy
// mode on), or send "keep #TAG" / "/keep TAG" with the visit's tag from the
// caption. Case does not matter; only the first word is read.
//
// What counts, so that nothing stale can start a recording (tg_updates.h has
// the exact rules, and tools/tg_updates_host_test.sh checks them):
//   - Each visit gets a random four-character tag (episode_open()), shown in
//     the photo's and the clips' captions as #TAG.
//   - An answer must be a new message (not an edit) from a person, in
//     TELEGRAM_CHAT_ID, dated no earlier than the photo by Telegram's own clock
//     (the photo's date and message_id come back in the sendDocument response
//     and are kept with the episode), and must reply to the photo's or the
//     first clip's own message_id, or carry the tag.
//   - The last update_id processed is kept in RTC memory. The window's first
//     read asks for the newest REPLY_BACKLOG_N updates with offset
//     -REPLY_BACKLOG_N, which makes Telegram forget everything older, and only
//     update_ids above the RTC record count. A backlog, or last visit's "keep"
//     that was never confirmed, is behind the line before any answer is looked
//     at. (Telegram restarts its numbering, lower, after a week with no
//     updates; a newest update below the RTC record means that, and the rules
//     above judge what was read on their own.) A chat busy enough to post more
//     than REPLY_BACKLOG_N messages between the photo and the clip's delivery
//     can push an answer out of that read; it then counts as no answer.
//   - An answer that cannot be read whole counts as no answer.
//
// Never past REPLY_WAIT_S: every request in the window connects to the
// address TELEGRAM_HOST resolved to on this association (no DNS, the one wait
// lwIP will not cut short), gives the TCP connect, the handshake and the
// request write a third of the time left each (the stall-timeout approach of
// TELEGRAM_STALL_MS, scaled down), and waits for the response no later than
// the window's end. The server's long poll is asked to answer
// REPLY_POLL_MARGIN_S before then. The wake deadline counts the window inside
// the first clip's cycle, at REPLY_WAIT_S plus 2 s for the CPU-bound steps
// between those checks.
//
// Modem sleep (WIFI_PS_MIN_MODEM) for the window only: the radio sleeps
// between the AP's DTIM beacons instead of listening throughout, and the AP
// holds anything for the node until the next one, so an answer arrives a
// beacon (~100-300 ms) late at worst and the handshakes take a little longer.
// Every other transfer keeps WIFI_PS_NONE (wifi_begin_async()), so uploads are
// not slowed. The energy note under Presence video has the saving.
//
// Needs WIFI_VALIDATE_STATIC (that lookup is where the address comes from)
// and a numeric TELEGRAM_CHAT_ID: with an "@channel" id no update's chat can
// match, so every window would end with no answer. Both are checked when the
// window opens; without them it does not open and the visit stops after its
// first clip, as for no answer. It also does not open when the daily fuse is
// full once the first clip is counted, since no second clip could follow, nor
// when neither the photo's nor the clip's message_id came back. If the first
// clip is never recorded (D1 already low once the photo is sent) or not
// delivered, there is no window either.
//
// Every confirmed visit opens the window: a person in a detect build, a frame
// the detector could not judge (fail open), and every episode in
// bench-nodetect (no detector, so no gate) or with SEND_ONLY_PERSONS 0. On
// bench-nodetect that means each visit with a clip costs the window's radio
// time too, and a bench node with no reply from anyone records one clip per
// visit. That is
// fine for 11 step 5's sleep-current measurement, which this does not touch.
//
// Telegram rules the setup must respect:
//   - No webhook on the bot: getUpdates answers 409 while one is set. That is
//     logged and the window ends with no answer.
//   - One getUpdates reader per bot token. Several nodes sharing a token would
//     each step the offset past the others' answers; give each node its own
//     bot (they can share a chat, with TELEGRAM_CAPTION_PREFIX).
//   - allowed_updates is sent as ["message"], which Telegram remembers for the
//     bot until another getUpdates call changes it.
//
// 0 turns the window off. The node then never reads the chat, and with no way
// to say "keep" or "stop" a confirmed visit records in full, as before the
// window existed.
#define REPLY_WAIT_S             25
// Kept back from the server's long-poll timeout so its answer lands inside
// the window.
#define REPLY_POLL_MARGIN_S      2
// No request starts with less than this left in the window: a third of it each
// for connect, handshake and write must still be a workable second.
#define REPLY_MIN_REQUEST_S      4
// How many of the newest updates the window's first read takes (and its later
// polls at most): enough to hold an answer given while the first clip was
// recorded and sent, in a chat that is not busy.
#define REPLY_BACKLOG_N          10
// One getUpdates response body, in PSRAM while the window is open (after the
// clip's buffer is freed, so not at the PSRAM peak). The largest single update
// is a 4096-character message escaped as \uXXXX plus the caption it replies
// to, ~32 kB, so this holds REPLY_BACKLOG_N of the worst kind. A body that does
// not fit counts as no answer.
#define REPLY_BUF_BYTES          (REPLY_BACKLOG_N * 33 * 1024)

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
// DHCP stays broken: at most one failure per HOSTNAME_REFRESH_RETRY_S. At
// 6 h that is four a day, ~2.2 mAh/day at worst, however often the node
// associates: the same as the four telemetry wakes (TELEMETRY_MAX_SILENCE_S)
// a quiet node has anyway, so a day of visits costs no more than a quiet day.
// The price is a name that reaches the router up to 6 h later once DHCP is
// back. 0 disables the refresh.
#define HOSTNAME_REFRESH_INTERVAL_S  86400
#define HOSTNAME_REFRESH_RETRY_S     21600

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
// ~2 uA is under 1% of the ~370 uA sleep total (Sleep current, under Pins). The 100 nF is not
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
// no presence at all still reports. That is the quiet-vs-mute case in 9.6.
// Each report is a full association plus TLS handshake, a few tenths of a
// mAh. A day with no presence pays for four of them, roughly a tenth of the 7
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
// setup() arms a one-shot esp_timer once it knows the wake is not deployment
// mode, and on expiry the node calls esp_system_abort(). Every build prints
// and reboots on a panic, so the next boot sees ESP_RST_PANIC and takes the
// abnormal-reset path: counted as a crash in the report (9.3), then back to
// sleep without the radio. Deployment mode is attended and never sleeps, so it
// is never on the clock.
//
// Two deadlines, not one. A visit has no clip limit of its own (Presence
// video), so a single deadline would have to cover the 30 clips the fuse allows:
// 140 minutes, in which a hang with the radio up burns ~220 mAh before anything
// notices. Instead the same timer is re-armed per stretch of the wake, and each
// stretch only has to cover itself:
//
//   WAKE_DEADLINE_S       the entry stretch: from setup() to the first clip, or
//                         to the end of a wake that records none
//   WAKE_CLIP_DEADLINE_S  one clip cycle, record then send, re-armed fresh at
//                         the start of each, so a stuck step trips within one
//                         cycle and a visit of any length can still run
//                         across as many clips as the fuse allows
//
// Both are checked against their derivations at compile time (main.cpp), so
// retuning a term past its deadline fails the build instead of cutting slow
// wakes short.
//
// Every term below is a timeout or a cap in the code.
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
// The entry stretch must cover the longest wake that records nothing as well
// as the lead-in to a clip:
//
//       30 s  PIR_IDLE_MAX_S       D1 waited out before the idle sleep
//                                  (main.cpp): RADAR_UNMANNED_DELAY_S + 20
//   +   29 s  8 + 21 s             a failed hostname refresh, once a wake
//                                  (HOSTNAME_REFRESH_INTERVAL_S): a lease at
//                                  WIFI_DHCP_TIMEOUT_MS that cannot resolve
//   +   20 s                       what no timeout bounds: a capture, ~10 s at
//                                  worst (init, CAM_WARMUP_MS, two 4 s
//                                  fb_get() timeouts when a frame overflows
//                                  its buffer), and a detection, allowed 10 s
//   +  207 s  54 + 153 s           the entry photo (PHOTOS_PER_EPISODE): an
//                                  association and a tg_post(), whose reply
//                                  body (the message_id the reply window
//                                  needs) is read inside the same cap
//   = 286 s   against 360 s, 74 s to spare
//
// A wake that sends no photo (no person) and owes a bare report is shorter:
// 29 + 54 + 92 s for the association and a small tg_post(), plus the 30 s
// idle wait, and the capture and detection.
//
// One clip cycle:
//
//       30 s  VIDEO_MAX_CLIP_S     recording
//   +   13 s  1 + 2 x 4 + 4        camera bring-up, up to two 4 s fb_get()
//                                  timeouts (record_clip()), and the time cap
//                                  checked up to one timeout late
//   +   29 s  8 + 21 s             the hostname refresh again: the first
//                                  association of a wake is a clip's when a
//                                  retrigger resumes a visit that already has
//                                  its photo
//   +   54 s                       an association
//   +  173 s                       a tg_post() of VIDEO_MAX_BYTES, its reply
//                                  body read inside the same cap
//   +   27 s  REPLY_WAIT_S + 2     the reply window, after the first clip only,
//                                  on that clip's association: every request
//                                  in it is cut to fit REPLY_WAIT_S, and 2 s
//                                  covers the CPU-bound steps between its
//                                  checks (main.cpp, REPLY_WINDOW_MAX_S)
//   = 326 s   against 360 s, 34 s to spare
//
// Nothing slow follows a clip cycle: a bare report is never due after one
// (send_clip() stamps the attempt) and the episode is still open, so the sleep
// is direct, with no idle wait. Between cycles the timer is simply re-armed.
//
// The cost of a hang is therefore one stretch, not the visit: 6 minutes at the
// ~94 mA average that 50 mAh per 32 minutes implied is ~9 mAh, a day's
// budget in 7, not 220. A trip mid-visit loses the episode (RTC memory does not
// survive a panic reset), so the node sleeps without the radio and the next
// edge starts a new visit.
#define WAKE_DEADLINE_S          (6 * 60)
#define WAKE_CLIP_DEADLINE_S     (6 * 60)

// ---------------------------------------------------------------------------
// Deployment mode — held-BOOT-button setup/aiming interface.
//
// Not in PROJECT_BRIEF.md: this is an operator-facing mode for siting the node
// (aim the lens, walk-test the radar, read telemetry) without a laptop or a
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
// Held at a presence wake it is also honoured, so an already-deployed node can
// be put back into setup mode by stepping in front of the radar with BOOT held
// — no power cycle needed.
// ---------------------------------------------------------------------------
#define PIN_DEPLOY_BUTTON     GPIO_NUM_0  // XIAO BOOT button, shorts IO0 to GND
#define DEPLOY_BUTTON_ACTIVE  0           // active-low; the XIAO pulls IO0 up

// How long after boot we keep watching for the press to *start*. A press
// already in progress is always seen through to its verdict, so a presence or
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
// do between presence wakes (9.1 step 3).
#define DEPLOY_COLDTEST_OFF_MS    1000

// Deployment mode's test clip: this many seconds with the current VIDEO_*
// settings, served as test.avi to download, for checking phone playback
// without Telegram.
#define DEPLOY_TEST_CLIP_S        5
