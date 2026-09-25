# Wildlife Camera Node — Project Brief

Battery-powered, PIR-triggered camera node. Wakes on motion, captures JPEG,
sends over WiFi to a home server. Deployed unattended at a remote property.
Target runtime: 6+ months on one 18650 cell.

This document is the authoritative hardware spec. Firmware must be written
against it. Where it says VERIFIED, the fact was read directly off Seeed's
published schematic — do not second-guess those.

---

## 1. Hardware

| Item | Part | Notes |
|---|---|---|
| MCU | Seeed XIAO ESP32-S3 | 8 MB PSRAM |
| Expansion | XIAO ESP32S3 Sense Exp. Board v1.1 | schematic rev v1.0 |
| Camera | OV5640, 24-pin FPC (JA1, AFC01-S24FCC-00) | DVP parallel |
| Motion | AM312 PIR | digital out, ~10 s output pulse |
| Battery | 1× 18650 Li-ion, unprotected cell | ~3400 mAh |
| Charge/protect | TP4056 module, 6-pad version (TP4056 + DW01 + 8205) | Type-C |
| Load switch | AO3401A P-MOSFET (SOT-23) on SOT23-3 breakout | |
| Level shift | 2N3904 NPN (TO-92) | |
| Passives | 2× 91 kΩ, 100 µF/16 V, 470 µF/16 V, 0.1 µF ceramic | all through-hole |
| Carrier | Perfboard, 2.54 mm | |

Not used: SD card slot, onboard microphone.

The microSD slot is deliberately **DISABLED**. It was enabled briefly and
reverted; 7 records the power measurement that decided it.

---

## 2. The core problem this design solves

VERIFIED: Seeed's own measurements for the Sense board in deep sleep:

- No peripherals connected: **63.8 µA**
- Camera connected: **3.00 mA**
- Camera + SD card: **3.98 mA**

The OV5640 cannot be powered down in software. Its PWDN pin (JA1 pin 8) is
held low by a 10 kΩ pulldown (R10) — VERIFIED. SCCB standby (register 0x3008)
reduces but does not eliminate the draw, because the camera's supply rails stay
energised.

At 3.00 mA the cell lasts about 9 days. The fix is a hardware load switch that
physically removes power from the camera between captures.

---

## 3. Camera power path (VERIFIED from schematic)

```
B2B connector JA3 pins 1 and 16  →  VIN (3.7–5 V)
    → R14 (0 Ω)  →  VCC_IN
         ├→ U1  SGM2036S-2.8V LDO  →  VCC_2V8 → camera AVDD (JA1 p4), DOVDD (p11)
         └→ U2  SGM2036S-1.3V LDO  →  VCC_1V8 → camera DVDD (JA1 p10)

B2B connector JA3 pin 12  →  VCC_3V3  →  SD card, microphone, R9 pullup only
```

**The camera does NOT run from 3V3.** Gating 3V3 would leave the camera
powered and would break the SD card and mic instead. This was caught during
schematic review and the design was corrected.

VIN on the expansion board feeds nothing except the two camera LDOs.

Note for anything else that might be added later: VCC_3V3 is **not** downstream
of the AO3401A. The load switch gates VIN → VCC_IN, the camera LDOs and nothing
else, so any peripheral on 3V3 stays energised through deep sleep and firmware
cannot power it down. This is what rules out the SD card (7).

Both LDOs have their EN pin (pin 3) tied directly to their IN pin (pin 4), so
they are permanently enabled. This is the root cause of the standing draw.

---

## 4. Required board modification

**Remove R14** (0 Ω link, VIN → VCC_IN) from the Sense expansion board, then
bridge the AO3401A across its two pads.

Component designators are NOT silkscreened on the board. R14 must be located
electrically: it is the 2-terminal 0 Ω part with one end on B2B pins 1/16 (VIN)
and the other end on pin 4 (IN) of both LDOs. The two LDOs are the only 4-pin
parts on the board. Seeed's KiCad/Eagle library ZIP (same wiki resources page
as the schematic) shows the physical placement.

Do not remove any component until it is positively identified.

---

## 5. Switch circuit

A P-MOSFET alone will not work here. VIN sits at 3.7–4.2 V from the cell (up to
~5 V on USB) while the GPIO only reaches 3.3 V, so gate-high gives Vgs ≈ −0.9 V
— at the AO3401A's threshold, not reliably off. An NPN level shift is required.

```
VIN ─┬──────────────── AO3401A  SOURCE
     └── 91 kΩ ──┬──── AO3401A  GATE
                 │
                 └──── 2N3904 COLLECTOR
                       2N3904 EMITTER → GND
                       2N3904 BASE → 91 kΩ → GPIO (camera power control)

AO3401A DRAIN ──→ R14 pad (VCC_IN side) ──→ LDO inputs
                  + 100 µF ∥ 0.1 µF to GND at this node
```

Logic:
- GPIO **HIGH** → NPN conducts → gate pulled low → FET on → **camera powered**
- GPIO **LOW or floating** → gate pulled to VIN → FET off → **camera unpowered**

Off is the default state during deep sleep, boot, and reset. This is
intentional and must not be inverted.

MOSFET orientation matters: source to VIN, drain to the camera side. Reversed,
the body diode conducts around the switch and the camera never powers down.

---

## 6. Power rail

```
18650 (+/−)  →  TP4056  B+ / B−
TP4056 OUT+ / OUT−  →  XIAO BAT pads   (470 µF across this node)
```

The cell connects ONLY to B+/B−. Everything downstream draws from OUT+/OUT−,
so the DW01/8205 over-discharge cutoff protects an otherwise unprotected cell.

Charging is via the TP4056's USB-C port. **Do not connect USB to the XIAO while
the TP4056 is connected** — two charge circuits on one cell.

---

## 7. Residual leakage (accepted, do not chase)

VERIFIED: camera RESET (JA1 pin 6) has a 10 kΩ pullup (R9) to VCC_3V3, which
stays energised when the camera is gated off. This pushes roughly 260 µA into
the camera's reset pin through its ESD clamp.

This is accepted. Expected sleep budget:

| Source | Draw |
|---|---|
| ESP32-S3 deep sleep + Sense board baseline | ~64 µA |
| R9 leak into gated camera | ~260 µA |
| AM312 PIR quiescent | ~15 µA |
| **Total sleeping** | **~340 µA** |

Active: ~250 mA for ~1.5 s per capture-and-send cycle, plus the TLS handshake
in 9.6.

At ~10 triggers/day this gives roughly 9 mAh/day → ~1 year theoretical,
6–10 months realistic after cold-weather capacity loss and self-discharge.

Trigger rate is the dominant variable. Firmware should make it observable.

### Why there is no microSD card

This was tried and reverted, and the numbers are worth keeping so it is not
re-proposed. Fitting the card adds roughly 1 mA of standing draw: Seeed measure
the same board at 3.00 mA with the camera attached and 3.98 mA with camera + SD
(2). The slot sits on VCC_3V3, which the AO3401A does not gate (3), so that
draw cannot be switched off in firmware — `SD.end()` releases the bus, not the
rail.

That would put the sleep total at ~1.34 mA, about 32 mAh/day before a single
photo is taken: roughly 3 months from a 3400 mAh cell instead of a year. The
card is therefore not fitted, and undeliverable frames are dropped rather than
spooled (9.6).

Re-opening this would need a second load switch on VCC_3V3 — which would also
remove the ~260 µA R9 leak on that same rail — and the card unmounted before
the rail drops. That is a board change, not a firmware one.

---

## 8. GPIO allocation

Taken by the camera (VERIFIED from schematic, do not reassign):
IO10 XMCLK, IO11 Y8, IO12 Y7, IO13 PCLK, IO14 Y6, IO15 Y2, IO16 Y5,
IO17 Y3, IO18 Y4, IO38 VSYNC, IO39 SCL, IO40 SDA, IO47 HREF, IO48 Y9

Taken by unused peripherals (free to reuse): D2/D8/D9/D10 (SD),
IO41/IO42 (mic), IO21 (user LED).

Assigned for this project:

| Function | Pin | Requirement |
|---|---|---|
| PIR input | D1 / GPIO2 | must be RTC-capable for ext0 wake |
| Camera power control | D0 / GPIO1 | drives 2N3904 base |

GPIO 0–21 are RTC-capable on the ESP32-S3, so D0/D1/D3 are all valid choices
for the PIR if reassignment is needed.

---

## 9. Firmware requirements

### 9.1 Basic cycle

1. Deep sleep, ext0 wake on PIR rising edge (AM312 is active-high)
2. On wake: assert camera power GPIO, wait for LDO settle
3. **Full camera re-initialisation** — power was removed, so all registers are
   lost. Treat every wake as a cold boot of the sensor. Do not attempt to
   resume from a saved state.
4. Capture JPEG to PSRAM
5. De-assert camera power GPIO before doing anything slow
6. Connect WiFi, send, disconnect
7. Return to deep sleep

Camera power should be on for the minimum time possible — it is the largest
single current draw in the cycle. Sending should happen with the camera off.

### 9.2 Burst filtering with on-device detection

The purpose is to suppress repeated false triggers (moving vegetation, sun
across the scene) without spending energy on detection in the common case.

```
On PIR wake:
    capture photo → buffer in PSRAM
    record timestamp in RTC memory

    if this is an ISOLATED trigger (< N triggers in the last W seconds):
        → send immediately, no detection
        → sleep

    if this is a BURST (>= N triggers in the last W seconds):
        → do NOT send yet
        → continue buffering, keep sleeping between triggers
        → once the burst settles, run person detection across buffered frames
        → if a human is detected: connect WiFi and send those frames
        → if not: discard all, sleep
```

Rationale: a single animal or person walking past trips the PIR once or twice —
send directly, detection costs nothing. A branch in wind trips it repeatedly —
that is exactly when detection earns its energy cost, because the alternative is
sending dozens of junk frames.

Tunable constants (keep in one place, they will need field adjustment):
- `BURST_COUNT` (N) — suggested starting value 3
- `BURST_WINDOW` (W) — suggested starting value 60 s
- `BURST_SETTLE` — quiet period before running detection, suggested 30 s

Burst state (trigger timestamps, counters) must live in RTC slow memory so it
survives deep sleep.

8 MB PSRAM is ample for buffering several JPEGs. No hardware change needed.

Detection: Espressif's ESP-WHO human/face detection models target the S3.
Model choice and the detection-confidence threshold should be configurable.

### 9.3 Telemetry

The node should report, with each transmission or periodically:
- battery voltage
- trigger count since last report
- how many triggers were suppressed by detection

Trigger rate is the single biggest unknown in the power budget. Without this
data there is no way to tune the burst thresholds or decide whether the PIR
needs upgrading.

### 9.4 WiFi

Fast-connect is required — WiFi association time dominates the active window.
Use static IP, cached BSSID and channel, and skip DHCP. Target under 1.5 s from
wake to sent.

Note that 9.6 moves delivery off the LAN and onto an HTTPS endpoint, so the
1.5 s target no longer describes wake-to-delivered. It still describes
wake-to-associated, which is the part this section controls.

### 9.6 Telegram delivery

Photos go to the Bot API `sendPhoto` endpoint as `multipart/form-data` over
HTTPS, replacing the raw `image/jpeg` POST to a LAN server. Telemetry (9.3)
rides in the caption; a report with no photo uses `sendMessage`.

Credentials — WiFi and the bot token — are compile-time, in `include/secrets.h`,
which is gitignored; `include/secrets.h.example` is the template. `config.h`
pulls it in via `__has_include` and defaults every value, so a checkout without
it still builds and simply declines to associate or post. Re-keying a node
means a reflash, which is the trade accepted by not fitting a card (7).

**A frame that cannot be delivered on the wake that captured it is dropped.**
There is no local spool, because the only place to put one is a microSD on the
ungated 3V3 rail (7). The consequence is that an uplink outage loses captures
outright, and the only evidence is the trigger counters — which is what makes
the periodic report in 9.3 worth keeping, since it is the one signal that
distinguishes a quiet node from a mute one.

Costs, since they work against 9.4:

- The TLS handshake to api.telegram.org is 1–3 s on a cold connect and needs
  working DNS. Wake-to-delivered is therefore seconds, not sub-1.5 s. The
  camera is already unpowered by then (9.1 step 5), so this is radio and CPU
  time, not camera time — it is the cheap kind of slow.
- The multipart body is assembled in PSRAM, never on the stack: an SVGA capture
  is 60–120 kB against 8 MB of PSRAM (9.2).
- Certificate validation is off by default (`TELEGRAM_INSECURE_TLS`). Pinning a
  root CA into a device that may be unreachable for months trades one failure
  mode — MITM on the operator's own uplink — for a worse one: silent total
  failure the day that CA rotates. Set it to 0 and supply a CA if the threat
  model justifies it.

---

## 10. Brownout risk

The ESP32-S3 brownout threshold is ~2.43 V by default. During a WiFi TX burst
the cell sags across its own internal resistance plus the TP4056 protection
FETs plus the load switch. This worsens as the cell discharges and as
temperature drops.

The 470 µF at the XIAO battery input exists to absorb this. It is a standard
electrolytic, not low-ESR — Seeed's supplier did not stock low-ESR in this
value. If brownouts appear under load on the bench, upgrading this cap to a
low-ESR or solid-polymer part is the first fix to try.

Bench-test WiFi transmission at low cell voltage before deploying.

---

## 11. Build order

1. Measure the 18650. **Below 2.5 V, discard it** — deeply discharged Li-ion
   can develop internal shorts and the TP4056 will attempt to charge it anyway.
2. Charge via TP4056, rest a week, re-measure. A cell that self-discharges
   significantly should not go into a sealed box for months.
3. Locate and remove R14.
4. Build the switch circuit on perfboard.
5. Bench-test: confirm the camera actually powers down. Measure sleep current —
   expect ~340 µA, not 3 mA. **If it still reads milliamps, stop and debug
   before proceeding.**
6. Firmware.
7. Enclosure and deployment.

---

## 12. Open items

- Male header pins (排針) may not have been ordered — only the female socket.
  Needed if the XIAO is to be socketed rather than hard-wired.
- IP65 enclosure not yet fitted. Needs internal floor for perfboard + 18650
  + XIAO stack, plus a lens port and cable glands.
- R14's physical location on the PCB is not yet confirmed. Seeed's KiCad
  library ZIP is the fastest way to find it.
- PIR false-trigger rate is unknown. If it proves high in the field, the
  Panasonic EKMB1303113K is a drop-in replacement on the same GPIO with better
  rejection — but note it has no built-in output timer, unlike the AM312's
  ~10 s pulse, so wake handling would need adjusting.

---

## 13. Reference

Expansion board schematic:
https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_ExpBoard_v1.0_SCH.pdf

Seeed XIAO ESP32S3 wiki (power measurements, KiCad library):
https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/
