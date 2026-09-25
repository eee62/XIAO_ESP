# Firmware fixes from a hardware review: wildlife camera node

You're working in my ESP32-S3 wildlife camera repo. The hardware is a Seeed XIAO ESP32S3 Sense with an OV5640 camera and an AM312 PIR sensor. The camera rail is switched by an AO3401A P-MOSFET driven through a 2N3904. The firmware uses the Arduino core.

`PROJECT_BRIEF.md` is the authoritative hardware spec. The firmware (`config.h`, `main.cpp`) was reviewed against the actual parts: the AO3401A datasheet, the AM312 module's schematic, and the XIAO ESP32S3 schematic and pinout. Fix the problems below.

## Ground rules

- Start by reading `PROJECT_BRIEF.md`, `config.h` and `main.cpp`. Then grep `deploy_mode.*` and `telegram.*` for anything that touches the same pins or functions.
- Verify each finding before you change anything. Check it against the code and, where you can, the installed ESP-IDF / Arduino-ESP32 / esp32-camera / esp-dl sources. If a finding doesn't hold for this code or these library versions, skip it and tell me why rather than forcing it.
- Keep changes surgical. Don't reformat, rename or restructure unrelated code.
- Match the existing style: tabs, and comments that explain *why* and cite `PROJECT_BRIEF.md` section numbers.
- Don't change camera pin assignments, and don't invert `CAM_POWER_ON_LEVEL` / `CAM_POWER_OFF_LEVEL`.
- Don't edit `PROJECT_BRIEF.md` or `include/secrets.h`. Editing `include/secrets.h.example` is fine.
- New tunables go in `config.h` with a comment, like the existing ones.
- If the repo uses git, make one commit per numbered item, with the message starting with the number.
- Build only. Don't flash the board or open a serial monitor.
- If you notice other problems in `deploy_mode.*` or `telegram.*`, report them. Don't fix anything beyond this list without asking.

## Hardware facts the code can't tell you

- **AM312 module.** The sensor output reaches the module's Vout pin through a 20 kΩ series resistor (R2). The sensor runs from the module's own 3.0 V regulator (HT7530-1). Its output is push-pull (CMOS), so it drives low by itself.
- **ESP32-S3.** Internal pull-ups and pull-downs are ≈ 45 kΩ typical. A guaranteed input high needs ≥ 0.75 × VDD ≈ 2.5 V. GPIO0–21 are RTC pads; IO26–48 are digital-only.
- **XIAO ESP32S3.** The BOOT line (GPIO0) already has an external pull-up and a capacitor. The 5V header pin is USB VBUS only, so it's dead on battery.
- **Camera power.** GPIO1 (D0) drives the 2N3904 base through 91 kΩ. The collector pulls the AO3401A gate low against a 91 kΩ pull-up to VIN. HIGH = camera powered; LOW or floating = off.
- **Budget.** PROJECT_BRIEF.md §7 targets ~340 µA asleep, so ~70 µA of leakage is a fifth of it.

## Part A: bugs

### 1. PIR input: drop the internal pull-down

`setup()` sets `GPIO_PULLDOWN_ONLY` on `PIN_PIR`, and `enter_deep_sleep()` calls `rtc_gpio_pulldown_en(PIN_PIR)`. With the module's 20 kΩ series resistor, a triggered PIR gives 3.0 V × 45/(45 + 20) ≈ 2.1 V at D1. That's below the S3's 2.5 V guaranteed-high level, on the node's only wake input.

- Add `PIR_INTERNAL_PULLDOWN` to `config.h`, default `0`. Its comment should explain the divider. It should also say that a broken-wire failsafe belongs in an external ≥ 1 MΩ from D1 to GND (still ≈ 2.9 V when triggered), not the internal pull.
- In `setup()`, use `GPIO_FLOATING` unless the flag is set.
- In `enter_deep_sleep()`, use `rtc_gpio_pulldown_dis(PIN_PIR)` unless the flag is set. Keep `rtc_gpio_pullup_dis`.
- Make any other pull configuration of `PIN_PIR` in the repo consistent with this.

### 2. Camera power pin: never enable its pull-up

`gpio_reset_pin()` enables the internal pull-up. The PIR and BOOT pins override that afterwards; `PIN_CAM_POWER` never does. So GPIO1 is driven low against ~45 kΩ to 3.3 V, which costs ~70 µA. `gpio_hold_en()` then latches that pull-up into deep sleep. The pin also passes briefly through a pulled-up state at boot, which can tick the 2N3904 on.

Replace the `PIN_CAM_POWER` setup at the top of `setup()` with something equivalent to:

```cpp
gpio_deep_sleep_hold_dis();
gpio_set_level(PIN_CAM_POWER, CAM_POWER_OFF_LEVEL);   // output latch low first
gpio_config_t cam_pwr = {};
cam_pwr.pin_bit_mask = 1ULL << PIN_CAM_POWER;
cam_pwr.mode         = GPIO_MODE_OUTPUT;
cam_pwr.pull_up_en   = GPIO_PULLUP_DISABLE;
cam_pwr.pull_down_en = GPIO_PULLDOWN_ENABLE;
cam_pwr.intr_type    = GPIO_INTR_DISABLE;
gpio_config(&cam_pwr);                                // no gpio_reset_pin(): no pull-up, ever
gpio_hold_dis(PIN_CAM_POWER);                         // release the latch last
```

Confirm in the installed ESP-IDF that `gpio_config()` doesn't release the RTC hold as a side effect. If it does, keep an order that never floats the pin or pulls it up. Grep the repo for any other `gpio_reset_pin()` or pull change on this pin.

### 3. Park the camera interface pins whenever the camera rail is cut

esp32-camera enables the internal pull-ups on the camera's I2C (SCCB) lines, SIOD = IO40 and SIOC = IO39. `esp_camera_deinit()` leaves them on; check this in the installed version.

`gpio_deep_sleep_hold_en()` in `enter_deep_sleep()` then freezes every digital-only pad (IO26–48) in its awake state for the whole sleep. Those pull-ups therefore feed the unpowered OV5640 through its pin clamps. That's the same leak path as the R9 leak §7 accepts. The camera's parallel data (DVP) pins are left as floating inputs, or whatever the installed driver leaves them as.

- Add `static void camera_pins_quiesce()` above `camera_up()`. For SIOD, SIOC, Y2–Y9, VSYNC, HREF and PCLK, it should call `gpio_set_direction(pin, GPIO_MODE_DISABLE)` and `gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY)`.
- Leave XCLK alone if `esp_camera_deinit()` parks it low via `ledc_stop()`. If it doesn't, include XCLK too.
- Call the helper right after `esp_camera_deinit()` in `camera_down()`.
- Also call it in `camera_up()`'s `esp_camera_init()` failure path, before `camera_power_off()`. A failed probe has already brought SCCB up and torn it down.
- Confirm `esp_camera_init()` reconfigures every pin you touch on the next wake: SCCB init restores its pull-ups, and the DVP setup restores floating inputs.
- If the library source isn't available locally, apply the fix anyway. It's harmless, and I'll confirm it with a current measurement.
- Keep `gpio_deep_sleep_hold_en()`, since it now latches the parked state, but fix its comment. GPIO1 is an RTC pad, so `gpio_hold_en()` alone holds it through deep sleep; check that `gpio_hold_en()` takes the RTC-hold path in the installed IDF. `gpio_deep_sleep_hold_en()` is what holds the digital-only pads.
- If deploy mode powers the camera without going through `camera_up()` / `camera_down()`, apply the same parking there.

### 4. Hand D1 back to the digital side after every ext0 light sleep

Espressif's sleep-mode docs say a pad used as an ext0 wake source is left configured as an RTC IO after waking. It must be `rtc_gpio_deinit()`'d before it's used as a digital GPIO again.

`wait_for_pir_idle()` and `run_burst()` arm ext0 for light sleep, then read `PIN_PIR` with `gpio_get_level()`. A stale read can end the idle wait while the AM312 is still high. The level-triggered deep-sleep wake then fires instantly, and one walk-by counts as several triggers. Repeated triggers are exactly what the burst filter keys on.

- Call `rtc_gpio_deinit(PIN_PIR)` immediately after each `esp_light_sleep_start()` in both functions.
- Call it once more in `setup()`, before `PIN_PIR` is configured.

### 5. Stay off the radio after an abnormal reset

`RTC_DATA_ATTR` variables are reloaded on every reset except a deep-sleep wake. Brownout, panic and watchdog resets report wake cause `UNDEFINED`. So after one of those resets:

1. `setup()` takes the non-PIR branch.
2. `rtc_last_report_s` is 0, so `telemetry_due()` is true.
3. The first thing the node does is associate and handshake TLS.

That's the load that just browned it out, or possibly the code path that just crashed. The result is a reset loop that runs until the cell hits the DW01 cutoff.

- In the non-EXT0 branch, before `maybe_send_telemetry_only()`, check `esp_reset_reason()`. If it's `ESP_RST_BROWNOUT`, `ESP_RST_PANIC`, `ESP_RST_INT_WDT`, `ESP_RST_TASK_WDT` or `ESP_RST_WDT`: log it, push the telemetry schedule out (item 8), and `enter_deep_sleep()`.
- If anything in the repo calls `esp_restart()` on an error path, treat `ESP_RST_SW` the same way.
- A normal power-on still reports once. Include `<esp_system.h>`.

### 6. Suppressed-frame count lags one report

In the `hits > 0` branch of `setup()`, `send_frames()` builds the captions and clears the since-report counters first. Only then do `rtc_suppressed_since_report` and `rtc_suppressed_total += (buffered - hits)` run. So a burst's suppressed frames are missing from its own captions and land in the next report instead.

- Move the two `+=` lines above the `send_frames()` call.

### 7. An internet outage pins the node to DHCP

The static-IP phase is judged by resolving `TELEGRAM_HOST`. If the router is up but its internet connection or DNS forwarder is down, that lookup fails. The DHCP retry then gets a lease, and `rtc_use_dhcp = true` is set, even though the static block was fine. `rtc_use_dhcp` is only cleared by an association failure, which may never come. Every later wake then pays for a DHCP exchange.

- In the phase-2 success path, set `rtc_use_dhcp` only if `wifi_reachable()` succeeds on the DHCP lease.
- If the name doesn't resolve on a DHCP lease (phase 1 or 2), the outage is upstream. Still cache the AP, since association worked. Then bring WiFi down and return `false`, so the send paths drop frames at once instead of each TLS connect waiting out its own DNS timeout.
- Update the `WIFI_REMEMBER_DHCP` comment in `config.h` to match.

## Part B: gap against the brief

### 8. Periodic telemetry needs a wake source

Nothing arms a timer wake, so `TELEMETRY_MAX_SILENCE_S` is only checked when something else wakes the node. A node that sees no motion never reports. That's the "quiet vs mute" case in §9.6.

- Add an `RTC_DATA_ATTR` variable `rtc_last_report_attempt_s`. Set it whenever a telemetry-carrying transmission is attempted, success or not: `send_frames()` with at least one frame, or `maybe_send_telemetry_only()`. Also set it in item 5's abnormal-reset branch.
- Use a sentinel for "never", not 0. `now_s()` is legitimately 0 right after power-on, for the same reason as `TRIGGER_SLOT_EMPTY`.
- Make `telemetry_due()` key off the last attempt, so an outage can't cause a retry on every wake. Keep `rtc_last_report_s` (last success) for the counters and the deploy-mode status page.
- In `enter_deep_sleep()`, after the existing disable-all call, arm `esp_sleep_enable_timer_wakeup()` for the time left until `TELEMETRY_MAX_SILENCE_S` after the last attempt. Do this only if `telegram_configured()`, and use a floor of about 10 minutes.
- A timer wake goes through the existing non-PIR branch. Pass 0 to `deploy_button_held()` for timer wakes as well as PIR wakes, instead of the 3 s window.
- Keep `TELEMETRY_MAX_SILENCE_S` at 6 h. That's my decision, so don't change it. Update its comment to say it now drives a real wake. Each report is a full association plus TLS handshake (a few tenths of a mAh). On a day with no motion that's four extra wakes, roughly a tenth of the §7 daily budget, and I accept that cost. Days with photos cost less, because every send restarts the clock.

## Part C: housekeeping

9. Two comments are wrong: the one in `setup()` saying nothing pulls GPIO0 up, and "against the internal pullup" on `DEPLOY_BUTTON_ACTIVE` in `config.h`. The XIAO has an external pull-up and capacitor on BOOT. The internal pull-up is redundant but harmless, so fix the comments only.
10. In the battery-sense block of `config.h`, document the divider to fit before enabling it:
    - 1 MΩ from BAT+ to D3 and 1 MΩ from D3 to GND (ratio 2.0, ~2 µA).
    - 100 nF from D3 to GND, so the ADC can sample a 500 kΩ source.

    Don't enable it.
11. Wrap `DEPLOY_AP_SSID` and `DEPLOY_AP_PASSWORD` in `#ifndef` so `secrets.h` can override them. Add both to `include/secrets.h.example`.
12. Remove `HTTP_TIMEOUT_MS` if nothing uses it.

## Part D: optional, in its own commit after A–C build clean

13. Count abnormal resets (the reasons in item 5) in an `RTC_NOINIT_ATTR` struct guarded by a magic value. Re-initialise it on power-on or a bad magic. Add the counts to `telemetry_text()`, e.g. `resets: brownout 2, crash 0`. This is the only way to find out in the field whether §10's brownouts are happening.

## Part E: check, and only change if broken

- **Build config.** The XIAO's ESP32-S3R8 has octal PSRAM (`board_build.arduino.memory_type = qio_opi` in PlatformIO) and 8 MB flash. The app partition must hold the esp-dl model.
- **esp-dl.** Check that `set_score_thr(float, int)`, `sw_decode_jpeg()` and the `jpeg_img_t` field order match the pinned version.
- **USB serial.** On battery with no USB host attached, `Serial` output and `Serial.flush()` mustn't block. Check the installed core's USB-CDC behaviour.
- **Other pins.** Grep for any other pin that's set up with `gpio_reset_pin()` and then driven as an output without its pull mode being set.

## When you're done

Build every environment with no new warnings. With PlatformIO, `pio run` builds them all, including the bench-nodetect / `DETECTION_ENABLED=0` one. Then give me:

1. The commits, one line each.
2. Any item you skipped or implemented differently, and why.
3. Anything else you noticed.
4. A bench checklist for me, including at least:
   - Sleep current, old firmware vs new (target ~340 µA per §7).
   - D1 voltage with the PIR triggered (≈ 3.0 V expected now).
   - One walk-by produces exactly one trigger in the log.
   - Low-cell brownout test: the node sleeps instead of reset-looping.
   - Router with its internet unplugged: the node isn't left on DHCP afterwards.
   - Hardware checks (not code):
     - The AM312 is powered from 3V3 or BAT+, not the 5V pin.
     - Diode-test the AO3401A before soldering. Short gate to source, then red on drain and black on source should read ≈ 0.6 V; reversed it should read open.
     - The 2N3904 is E-B-C with the flat face toward you.
     - Watch for resets exactly at camera power-on at a low cell. If you see them, add ~10 kΩ in series with the 2N3904 collector and ~100 nF from gate to source.
