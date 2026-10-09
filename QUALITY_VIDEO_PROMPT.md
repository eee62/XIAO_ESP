# Still quality, AI-filtered photos and presence video: wildlife camera node

You're working in my ESP32-S3 wildlife camera repo. The hardware:

- Seeed XIAO ESP32S3 Sense
- third-party fixed-focus OV5640
- AM312 PIR
- camera rail switched by an AO3401A through a 2N3904

The firmware uses Arduino core 3.3.x on pioarduino (ESP-IDF 5.5). `PROJECT_BRIEF.md` is the authoritative hardware spec. Everything in `FIX_PROMPT.md` is already in the code.

This round has three parts:

- **Part A:** get the best still the OV5640 can deliver.
- **Part B:** a new trigger flow:
  - Every PIR trigger takes a photo. On-board person detection checks it, and only photos with a person get sent.
  - When the PIR shows presence for 10 s or longer and the AI has confirmed a person during that visit, record video and send it.
- **Part C:** raise the still resolution to 5 MP, once Part B's memory budget is known.

## Ground rules

- Start by reading `PROJECT_BRIEF.md`, `config.h`, `main.cpp`, `deploy_mode.*` and `telegram.*`.
- Verify each finding before you change anything. Check it against the code and the installed sources: esp32-camera in the framework libs (`ov5640.c`, `cam_hal.c`, `img_converters.h`), esp-dl 3.3.12 and esp_new_jpeg under `managed_components`, and the Arduino core. If a finding doesn't hold for these versions, skip it and tell me why.
- Keep changes surgical. Match the existing style: tabs, and comments that explain *why* and cite `PROJECT_BRIEF.md` section numbers.
- Don't change camera pin assignments. Don't invert `CAM_POWER_ON_LEVEL` / `CAM_POWER_OFF_LEVEL`.
- The camera and the radio are never on at the same time (§9.1 step 5, §10). This applies to video too.
- Don't edit `PROJECT_BRIEF.md` or `include/secrets.h`. Editing `include/secrets.h.example` is fine.
- **Part B deliberately replaces the brief's §9.2 isolated/burst rule.** Don't edit the brief. Where the code departs from §9.2, say so in a comment.
- New tunables go in `config.h` with a comment, like the existing ones.
- Pin any new library dependency to an exact version or commit, like AsyncTCP in `platformio.ini`.
- Make one commit per numbered item, with the message starting with the number. Each commit must build.
- Build only. Don't flash the board or open a serial monitor.
- All three envs must build: `wildlife`, `wildlife-face`, `bench-nodetect`.
- Any change that can lengthen a wake must keep the `WAKE_DEADLINE_S` derivation in `config.h` and the `static_assert` in `main.cpp` honest.
- **Do Part A first. Before writing any Part B code, post the Part B plan (see "Plan first") and wait for my OK.**

## Hardware facts the code can't tell you

- **The camera has a fixed-focus lens.** It's a third-party OV5640 module with no autofocus motor. Focus is set mechanically by the lens, so don't add any autofocus code. The firmware's only help with focus is the deployment-mode focus check in item 6.
- **Every wake is a cold sensor.** The rail is cut between captures, so every register is lost.
- **Resolution:** the sensor's maximum is 2592×1944.
- **PSRAM is 8 MB and does not survive deep sleep.** See the burst-buffer comment in `main.cpp`. There's no SD card, deliberately (§7). Anything recorded must fit in PSRAM and be sent on the same wake.
- **Phone:** I read Telegram on an iPhone.
- **AM312 behaviour:**
  - The output goes high on motion and is retriggerable: it stays high while motion continues.
  - It falls a fixed hold time after the last motion. I haven't measured that hold time yet. The brief says ~10 s; many AM312 listings say ~2 s. It becomes the tunable `PIR_HOLD_S` in Part B.
  - A person standing still reads as "gone".
- **The detection model only finds people.** The pedestrian model doesn't detect animals.

## Part A: still image quality

### 1. Exposure and white-balance warm-up

`CAM_WARMUP_FRAMES = 2` is too short. The OV5640's auto exposure and white balance don't settle that fast from cold, and it's worse at high resolution, where the frame rate is low.

- Make the warm-up time-based. Discard frames until at least `CAM_WARMUP_MS` has passed since init *and* `CAM_WARMUP_MIN_FRAMES` frames have been read. Start at 800 ms and 3 frames; I'll tune them on the bench with item 6.
- Optional early exit: if you can confirm from the datasheet a register that shows AEC is inside its stable range, you may end the warm-up once it is. `CAM_WARMUP_MS` stays the cap either way. Don't guess register meanings.
- Check what `fb_count = 1` does in the installed `cam_hal.c`. If discarding a frame also costs the next one, use `fb_count = 2` with `CAMERA_GRAB_LATEST` for stills.
- Log wake-to-shutter time on every capture: from `setup()` entry to the frame that's kept.

### 2. Sensor settings after every init

Nothing is set on the sensor after `esp_camera_init()`. The image is whatever the driver defaults are, and none of it is tunable.

- Add `camera_apply_settings()`, called from `camera_up()` on success, so the deployment preview matches real captures.
- Add config macros for:
  - `CAM_VFLIP` and `CAM_HMIRROR` (default 0; I'll check them in the preview)
  - lens correction
  - BPC/WPC
  - raw gamma
  - AE level
  - gain ceiling
  - sharpness
  - denoise
  - `set_aec2`: check what it does in `ov5640.c`, and default it off, because longer exposures blur a walking subject.
- For each one, check in `ov5640.c` that the setter is implemented and what the driver default is. Only set supported ones. Default each macro to the driver's value unless there's a reason to change it, and tell me which ones you changed and why.
- Lower `CAM_JPEG_QUALITY` to 10 (a lower number means better quality). Item 4 handles the overflow risk.

### 3. Detection must decode at a reduced scale

`detect_over_buffer()` decodes each frame to full-size RGB888. That's fine at SVGA, but at 5 MP it's about 15 MB, more than all of PSRAM. The decode fails, the frame is skipped with a log line, and it counts as suppressed. In Part B, detection runs on every photo, so this would silently drop everything.

- Decode at the smallest power-of-two reduction that brings the width to `DETECT_DECODE_MAX_W` or less (default 800).
- Find the mechanism in the installed sources and justify your choice. The candidates:
  - a scale option in esp-dl 3.3.12's `sw_decode_jpeg`
  - esp_new_jpeg's decoder config (scale/clipper)
  - esp32-camera's `jpg2rgb565()` with `jpg_scale_t`, matched to esp-dl's RGB565 pixel type and byte order
- Confirm the detector accepts the resulting `img_t`, and that its preprocessor resizes to the model input.
- Log decode time and inference time separately. Detection will be on the path of every photo, so I need both numbers.
- Count decode failures in telemetry (for example `detect errors N`), not as "suppressed".

### 4. Frame-buffer overflow at high resolution

In JPEG mode the esp32-camera driver sizes its frame buffer from the frame size. Check the formula in `cam_hal.c` under `CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE_AUTO`. A busy scene like foliage at 5 MP and quality 10 can exceed it. The driver then drops the frame, and `capture()` fails only after the `fb_get` timeout.

- On a failed `fb_get`, raise the JPEG quality number by `CAM_QUALITY_STEP` and retry once.
- Remember the value that worked in RTC memory, log it, and report it in telemetry.
- For the detect envs, check whether a Kconfig option exists for a custom JPEG buffer size, and if so consider it in `custom_sdkconfig`. `bench-nodetect` can't use it, so the retry is the portable fix.

### 5. Telegram: original quality and large uploads

There are three problems.

- **Recompression.** `sendPhoto` recompresses on Telegram's side, so I never see the original.
  - Add `TELEGRAM_STILL_AS_DOCUMENT`, default 1. It sends stills via `sendDocument` with the original bytes, a `.jpg` filename and `image/jpeg`.
  - 0 keeps `sendPhoto`.
- **Timeout.** `tg_post()` checks `TELEGRAM_TIMEOUT_MS` against a `t0` taken before the connect, so connect plus the whole upload must finish in 15 s. A 5 MP frame on a weak link won't, and a video certainly won't.
  - Replace it with a stall timeout: fail when there's no write progress for `TELEGRAM_STALL_MS`.
  - Add an overall cap that scales with size: a base time plus bytes / `TELEGRAM_MIN_BPS`.
  - Redo the `WAKE_DEADLINE_S` derivation to match.
- **Double copy.** `telegram_send_photo()` copies the entire multipart body into a second PSRAM buffer.
  - Write the head, the payload and the tail in sequence instead. Content-Length is known up front.
  - Keep one generic sender (method, form field name, filename, content type, caption, payload pointer and length) used by sendPhoto, sendDocument and, later, the video.

### 6. Deployment mode

I'll tune all of the above from the preview.

- **Preview size.** Stream the live preview at `DEPLOY_PREVIEW_FRAMESIZE` (default VGA). `/snapshot` returns a frame at `CAM_FRAMESIZE`.
- **Cold-capture test.** Add a button and endpoint that runs the exact photo path: rail off, power on, cold init, warm-up, detection. Show:
  - the result
  - wake-to-shutter time
  - JPEG size
  - detection score and time

  The live preview comes from a warm sensor, so it can't show what a PIR wake produces. Camera power changes stay on the loop task, per the threading rule at the top of `deploy_mode.cpp`.
- **Focus check.** The lens is fixed, so I focus it by hand. Add a "focus" view that repeatedly takes full-size snapshots and shows two things:
  - the centre at 100% scale (one sensor pixel per screen pixel, not scaled to fit the phone)
  - each snapshot's JPEG size, large and live

  At the same scene and quality, a bigger JPEG means more fine detail, so it works as a crude sharpness meter while I turn the lens.
- **Status page.** Add the active camera settings: frame size, quality, orientation.

## Part B: new trigger flow

### Photos: every trigger, AI decides

- Every PIR trigger captures a full-quality still, even if the visitor is only there for a moment. That includes every retrigger seen while awake. The capture still comes before anything slow (§9.1).
- Run person detection on it immediately, with the radio off.
- **Person found** (score ≥ `DETECT_SCORE_THRESHOLD`): send it.
- **No person:** drop it and count it as suppressed.
- **Fail open.** If the detector can't run (allocation or decode failure), send the photo and count a detect error. A photo we couldn't judge is better sent than lost.
- **Radio start.** Start associating only once a photo is known to be going out, the way `on_first_hit` does now, never before detection finishes.
- **`SEND_ONLY_PERSONS`**, default 1. When 0, every photo is sent unfiltered, which is the only way animals get through (see hardware facts).
- **`bench-nodetect`** has no detector, so it sends every photo unfiltered.
- **Photos per visit.** Send at most `PHOTOS_PER_EPISODE` person photos per presence episode (defined below). A lingering person shouldn't produce a photo on every retrigger; the video covers that.

The old isolated/burst split and `run_burst()` only existed to decide when detection was worth its energy. Detection now runs on every photo, so propose in the plan whether to simplify them or remove them. Whatever you choose, these must survive:

- **Bounded wind cost.** A windy branch retriggering all day, with no person, must not cost a full wake per trigger forever. Give the per-trigger cost in mAh (wake, capture and detection) and propose a backoff. Examples: light sleep between rapid triggers, or a cap on captures per minute once N triggers in a row had no person.
- **Honest telemetry.** The §9.3 counters stay meaningful: triggers, photos sent, photos suppressed and detect errors.

### Video: PIR presence for 10 s, confirmed by AI

**Presence episode.** An episode opens on a PIR rising edge when none is open. It stays open while D1 is high, or has been low for less than `PRESENCE_GAP_S`. It closes once D1 has been low for `PRESENCE_GAP_S` or longer. Episode state must survive deep sleep, so it lives in RTC memory.

**The 10-second rule.** Recording starts only when motion has continued for at least `VIDEO_PRESENCE_MIN_S` (10 s) *and* the AI has confirmed a person during this episode (see `VIDEO_REQUIRE_PERSON` below).

- **Why not just check the age.** Don't simply check the episode's age. The AM312 hold time alone keeps D1 high for `PIR_HOLD_S` after a single one-second walk-by, so an age check would start a video for everyone who passes. A visit shorter than 10 s must never start a clip.
- **Proving motion at time T.** Motion is proven at episode time T when either:
  - a new rising edge arrives at T, or
  - D1 is still high at T + `PIR_HOLD_S`. The output falls `PIR_HOLD_S` after the last motion, so a high at that point means something was still moving at T.
- **The trigger.** As soon as motion is proven at some T ≥ `VIDEO_PRESENCE_MIN_S`, apply the person gate below, then start recording with the radio off.
- **Earliest clip start.** The earliest a clip can start is about `VIDEO_PRESENCE_MIN_S + PIR_HOLD_S` after arrival, plus camera init. That delay is expected.
- **Collisions.** If that moment falls while a photo is still uploading, don't cut the upload short; start recording as soon as it ends. If detection is still running, let it finish first.
- **`VIDEO_REQUIRE_PERSON`**, default 1. This is the person gate. It exists because a branch in steady wind can hold the PIR active and would otherwise produce clips of the branch. When the 10 s rule fires:
  - **Person already seen.** If a photo earlier in this episode had a person, record at once. That reuses the result already computed, so it costs no extra AI.
  - **No person seen yet.** The first photo may have caught the person at the edge of the frame. And because the AM312 stays high through continuous motion, a lingering person produces no new edges and so no new photos. So capture one fresh still and run detection on it.
    - Person found: send that still as a photo (it counts toward `PHOTOS_PER_EPISODE`), then record.
    - No person: no clip this episode.
  - **Cost.** At most one fresh check per episode. Only episodes that pass the 10 s rule ever pay for it, so short visits still cost nothing extra.
  - When 0, record on the 10 s rule alone.

**Recording.** Re-initialise the camera with video settings: `VIDEO_FRAMESIZE`, `VIDEO_JPEG_QUALITY`, `fb_count = 2`, `CAMERA_GRAB_LATEST`, and a short warm-up. Pace frames to `VIDEO_FPS`. Stop at whichever comes first:

- D1 has been low continuously for `VIDEO_END_QUIET_S`
- `VIDEO_MAX_CLIP_S`
- the buffer is full

**Sending.** Camera off, finalise the file, radio up, send the clip. The AI gate was at the start, so the clip itself is always sent. The caption has:

- duration, measured fps, frame count, size and end reason
- whether any photo in the episode had a person: yes, no, or not checked
- the usual telemetry

**Long visits.** If the clip hit its time or buffer cap and D1 is still active after sending, record another clip, up to `VIDEO_MAX_CLIPS_PER_EPISODE`. If a clip fails to send, stop recording clips for this episode: the uplink is down, and the next clip would be dropped too.

**Daily cap.** At most `VIDEO_MAX_CLIPS_PER_DAY` clips in any rolling 24 h, tracked in RTC memory via `now_s()`.

**`bench-nodetect`.** There's no detector in this env, so `VIDEO_REQUIRE_PERSON` is forced to 0 there and video runs on the 10 s rule alone.

### Starting values

| Tunable | Start |
|---|---|
| `SEND_ONLY_PERSONS` | 1 |
| `PHOTOS_PER_EPISODE` | 3 |
| `VIDEO_ENABLED` | 1 |
| `VIDEO_PRESENCE_MIN_S` | 10 (my requirement) |
| `PIR_HOLD_S` | 10, per the brief; I'll measure and correct it |
| `PRESENCE_GAP_S` | 8 |
| `VIDEO_REQUIRE_PERSON` | 1 |
| `VIDEO_FRAMESIZE` | `FRAMESIZE_SVGA`; never above HD, the usual limit for phone M-JPEG playback |
| `VIDEO_FPS` | 8 |
| `VIDEO_JPEG_QUALITY` | 14 |
| `VIDEO_END_QUIET_S` | 10 |
| `VIDEO_MAX_CLIP_S` | 30 |
| `VIDEO_MAX_BYTES` | 4 MB |
| `VIDEO_MAX_CLIPS_PER_EPISODE` | 3 |
| `VIDEO_MAX_CLIPS_PER_DAY` | 6 |

### File format and delivery

- **Container.** MJPEG in AVI, laid out as:
  - RIFF `AVI `
  - `hdrl`: `avih`, plus one `vids` stream with the `MJPG` fourcc and a BITMAPINFOHEADER
  - `movi`: `00dc` chunks, padded to even length
  - an `idx1` index
- **Frame rate.** Write the *measured* average frame rate into the headers, so playback runs at real speed.
- **Buffer.** Build the AVI in place in one PSRAM buffer. Reserve header space at the front, append frames as chunks, and put the index at the end. No second copy.
- **Buffer size.** Size the buffer at runtime as `min(VIDEO_MAX_BYTES, largest free PSRAM block − reserve)`.
- **Delivery.** Send via `sendDocument` as `.avi` / `video/x-msvideo`, using item 5's streaming sender. `sendVideo` expects MP4/H.264, and on-device H.264 isn't worth it.

### Constraints

- Don't hook any of this into `enter_deep_sleep()`. That function is also the exit for abnormal resets and for the deployment-mode arm.
- No photo or clip is sent twice.
- Decide whether PIR edges during recording count as triggers, and document the choice. They come from the same visitor.
- **Telemetry and status page.** Add:
  - photos sent and suppressed
  - detect errors
  - clips sent and dropped
  - total clip seconds
- **Energy.** In the `config.h` comments, add:
  - a per-trigger mAh estimate for photo plus detection
  - a per-clip mAh estimate for recording plus upload
  - what the daily clip cap means against the §7 budget
- **Test clip.** Add a deployment-mode "test clip" button. It records a few seconds with the current video settings and serves the `.avi` for download, so I can check phone playback without Telegram.

### Plan first

Before writing Part B code, post:

- the new flow as a small state diagram, and what happens to the isolated/burst code
- timelines for:
  - a 1 s walk-by: photo, detection, sent only if a person, no clip
  - a 15 s stay
  - a windy branch, with and without `VIDEO_REQUIRE_PERSON`
- the wind backoff you propose
- the RTC state you're adding
- the PSRAM budget at the worst point
- the new wake-deadline sum. If it grows unreasonable (over ~45 min), propose an alternative such as re-arming at phase boundaries, but don't implement it without asking.
- per-trigger and per-clip energy estimates

Then wait for my OK.

### Host test

If `ffmpeg`/`ffprobe` are installed, compile the AVI writer for the host with a few sample JPEGs. Check with ffprobe that the duration, fps and frame count come out right.

## Part C: resolution

### 7. Raise the still resolution

`CAM_FRAMESIZE` is SVGA, which is 0.48 MP of a 5 MP sensor.

- Default stills to the largest size the installed OV5640 driver supports: `FRAMESIZE_5MP` if that enum exists, otherwise `FRAMESIZE_QSXGA`.
- Keep it tunable, and rewrite its comment. The old reason, the RGB888 decode size, is handled by item 3.
- Also say in the comment that a third-party lens may not resolve 5 MP. If a 3 MP (`FRAMESIZE_QXGA`) crop looks just as sharp on the bench, the smaller size is the better trade, because every byte costs upload energy.
- Re-check Part B's PSRAM budget at this size. Any frame buffering that survived Part B must be capped by bytes, not frame count.
- Update the per-photo upload estimate in `config.h`.

## When you're done

Build every env with no new warnings. Then give me:

1. The commits, one line each.
2. Anything skipped or done differently, and why.
3. Anything else you noticed.
4. The numbers:
   - free PSRAM at each peak
   - the new `WAKE_DEADLINE_S`
   - detection time per photo
   - energy per trigger and per clip
5. A bench checklist, including at least:
   - **Focus:** use the focus view with a target at the real scene distance. Compare 5 MP and 3 MP crops.
   - **Warm-up:** take cold captures at a few `CAM_WARMUP_MS` values. Note when colour and exposure stop changing.
   - **5 MP reliability:** take 20 cold captures of a busy scene, like foliage. Record any overflow retries.
   - **Wake-to-shutter time** at the final settings.
   - **Photo filter:** walk past for 1 s and get a photo in Telegram. Wave a hand or hang a jacket in front of it: no photo, and the suppressed count goes up.
   - **Delivery:** stills arrive at full resolution, and a large upload succeeds on a weak link.
   - **Sleep current:** still ~340 µA (§7).
   - **Short visit:** under 10 s gives no clip.
   - **Long visit:** stay 15–20 s. The clip starts about `VIDEO_PRESENCE_MIN_S + PIR_HOLD_S` after arrival, and ends about `VIDEO_END_QUIET_S` after I leave.
   - **Video playback:** a test clip plays on the iPhone, in Telegram and in Files or VLC.
   - **Wind:** shake a branch in front of the PIR for 20 s or more. The wind backoff works, the log shows one fresh person check at most, and no clip is recorded.
   - **Edge of frame:** stand where the PIR sees you but the camera barely does, then step into view and stay. The fresh check at the 10 s mark catches you and the clip starts.
   - **Low cell:** a full clip upload at low cell voltage doesn't brown out (§10).
   - **Hardware (not code):** measure the AM312 hold time. Wave once, time how long the deployment-mode PIR indicator stays lit, and set `PIR_HOLD_S` to that.
