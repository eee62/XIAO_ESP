#!/usr/bin/env python3
"""Configure the node's HLK-LD2410S radar once, over a USB-to-TTL adapter.

    python3 -m pip install pyserial          # once
    tools/radar_config.py                    # find the adapter, set, verify
    tools/radar_config.py --port /dev/cu.usbserial-0001
    tools/radar_config.py --read-only        # just show what the radar holds
    tools/radar_config.py --self-test        # check the frames, no radar needed

Sets two things, then reads them back to confirm the radar kept them:
  - the unmanned delay: how long OT2 stays high after the last presence.
    It MUST equal RADAR_UNMANNED_DELAY_S in include/config.h, which this tool
    reads for its default. The firmware never talks to the radar; it only
    trusts that number.
  - the minimum refresh rate: both reporting frequencies (status and
    distance) at 0.5 Hz, the slowest the radar allows, which is its ~45 uA
    lowest-power setting (manual Table 2-1, "Minimum refresh rate power
    consumption: average 45uA"). The cost is latency: the radar re-decides
    "someone / no one" every 2 s, so OT2 can rise up to ~2 s after a person
    appears, which adds to the node's wake-to-shutter time.
Nothing else (detection gates, response speed, thresholds) is written.

Source: Hi-Link's "HLK-LD2410S User manual-V1.3.pdf" (in the project folder,
from Hi-Link's Google Drive folder linked on hlktech.net; the file is named
V1.3 but its cover and version history say V1.2, 2023-10-31), section 7.3,
cross-checked against Hi-Link's "HLK-LD2410S serial communication
protocol-V1.00" (2024-08-23), section 2.2, from the same folder. Where they
differ, the protocol document's table agrees with both documents' worked
examples, and that is what is used:
  - the manual prints the write command as "0x7000"; that is its two bytes
    in wire order (70 00). The word, little-endian, is 0x0070, and reading
    the parameters back is 0x0071 (manual "0x7100").
  - the manual's Table 5-2 lists 0x0B twice and leaves out 0x02; the
    protocol's Table 2-2 and every example frame have 0x02 = status reporting
    frequency, 0x0C = distance reporting frequency, 0x0B = response speed.
  - a frequency is sent in tenths of a hertz: the examples send 5 for 0.5 Hz.

Frame (all little-endian, manual 7.1):
  FD FC FB FA | length (2) | command word (2) | value | 04 03 02 01
where length counts the command word and value. An ACK echoes the command
word with 0x0100 set, then a 2-byte status (0 = success). Serial: 115200 8N1.
"""
import argparse
import os
import re
import struct
import sys
import time

HEAD = bytes.fromhex("FDFCFBFA")
TAIL = bytes.fromhex("04030201")

CMD_ENABLE_CONFIG = 0x00FF   # value 0x0001 (manual 7.3.2)
CMD_END_CONFIG    = 0x00FE   # (7.3.3)
CMD_WRITE_GENERIC = 0x0070   # "0x7000" in the manual (7.3.6)
CMD_READ_GENERIC  = 0x0071   # "0x7100" in the manual (7.3.7)

# Parameter words (protocol V1.00 Table 2-2).
P_FARTHEST_GATE = 0x05
P_NEAREST_GATE  = 0x0A
P_UNMANNED_S    = 0x06   # 10..120 s
P_STATUS_HZ10   = 0x02   # status reporting frequency, tenths of Hz, 5..80
P_DIST_HZ10     = 0x0C   # distance reporting frequency, tenths of Hz, 5..80
P_RESPONSE      = 0x0B   # 5 normal, 10 fast

ALL_PARAMS = [P_FARTHEST_GATE, P_NEAREST_GATE, P_UNMANNED_S, P_STATUS_HZ10,
              P_DIST_HZ10, P_RESPONSE]
NAMES = {
    P_FARTHEST_GATE: "farthest distance gate",
    P_NEAREST_GATE:  "nearest distance gate",
    P_UNMANNED_S:    "unmanned delay (s)",
    P_STATUS_HZ10:   "status reporting frequency (Hz)",
    P_DIST_HZ10:     "distance reporting frequency (Hz)",
    P_RESPONSE:      "response speed (5 normal, 10 fast)",
}

MIN_REFRESH_HZ10 = 5   # 0.5 Hz, the slowest either frequency goes

WIRING = """\
Wiring, radar to the USB-to-TTL adapter (manual Table 4-1). Do this with the
radar unplugged from the camera node, so it has one supply only:

    radar RX   ->  adapter TXD
    radar OT1  ->  adapter RXD      (OT1 is the radar's UART TX)
    radar GND  ->  adapter GND
    radar 3V3  ->  adapter 3.3V pin

    NEVER 5V: the radar's supply is 3.0-3.6 V. Many adapters have both pins
    side by side; check which one the red wire is on before plugging in.
    OT2 is not used here.
"""


def fail(msg):
    print("ERROR: " + msg, file=sys.stderr)
    sys.exit(1)


def frame(cmd, value=b""):
    body = struct.pack("<H", cmd) + value
    return HEAD + struct.pack("<H", len(body)) + body + TAIL


def write_generic_frame(params):
    value = b"".join(struct.pack("<HI", word, val) for word, val in params)
    return frame(CMD_WRITE_GENERIC, value)


def read_generic_frame(words):
    return frame(CMD_READ_GENERIC, b"".join(struct.pack("<H", w) for w in words))


def hexs(b):
    return " ".join("%02X" % x for x in b)


def show_value(word, val):
    if word in (P_STATUS_HZ10, P_DIST_HZ10):
        return "%g" % (val / 10.0)
    return str(val)


def config_h_delay():
    """RADAR_UNMANNED_DELAY_S from include/config.h, or None."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                        "include", "config.h")
    try:
        with open(path) as f:
            m = re.search(r"^#define\s+RADAR_UNMANNED_DELAY_S\s+(\d+)\b", f.read(), re.M)
    except OSError:
        return None
    return int(m.group(1)) if m else None


class Radar:
    def __init__(self, port):
        import serial  # pyserial; imported here so --self-test needs nothing
        self.ser = serial.Serial(port, 115200, bytesize=8, parity="N",
                                 stopbits=1, timeout=0.05)
        self.buf = b""

    def close(self):
        self.ser.close()

    def _ack(self, cmd, timeout_s):
        """The next command frame (FD FC FB FA ...) whose word is cmd|0x0100.
        The radar streams its own report frames (F4 F3 F2 F1 ...) all the
        time; anything that is not our ACK is skipped."""
        want = cmd | 0x0100
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self.buf += self.ser.read(256)
            while True:
                i = self.buf.find(HEAD)
                if i < 0:
                    self.buf = self.buf[-3:]
                    break
                self.buf = self.buf[i:]
                if len(self.buf) < 6:
                    break
                n = struct.unpack_from("<H", self.buf, 4)[0]
                if n > 1024:
                    self.buf = self.buf[4:]   # not a real header
                    continue
                if len(self.buf) < 6 + n + 4:
                    break
                fr, self.buf = self.buf[:6 + n + 4], self.buf[6 + n + 4:]
                if fr[-4:] != TAIL or n < 2:
                    continue
                if struct.unpack_from("<H", fr, 6)[0] == want:
                    return fr[8:6 + n]   # what follows the command word
        return None

    def command(self, cmd, value=b"", timeout_s=1.0, tries=3):
        """Send one command; its ACK payload after the status word. Fails
        unless the status is 0."""
        fr = frame(cmd, value)
        for attempt in range(tries):
            self.ser.reset_input_buffer()
            self.buf = b""
            self.ser.write(fr)
            payload = self._ack(cmd, timeout_s)
            if payload is not None:
                if len(payload) < 2:
                    fail("short ACK for command 0x%04X" % cmd)
                status = struct.unpack_from("<H", payload)[0]
                if status != 0:
                    fail("radar refused command 0x%04X (status %d)" % (cmd, status))
                return payload[2:]
        fail("no ACK for command 0x%04X after %d tries. Check the wiring "
             "(radar RX to adapter TXD, OT1 to RXD) and the port." % (cmd, tries))

    def enable_config(self):
        self.command(CMD_ENABLE_CONFIG, struct.pack("<H", 0x0001))

    def end_config(self):
        self.command(CMD_END_CONFIG)

    def read_params(self, words):
        """{word: value}, read inside its own enable/end pair."""
        self.enable_config()
        try:
            data = self.command(CMD_READ_GENERIC,
                                b"".join(struct.pack("<H", w) for w in words))
        finally:
            self.end_config()
        if len(data) < 4 * len(words):
            fail("read-back returned %d bytes for %d parameters"
                 % (len(data), len(words)))
        return {w: struct.unpack_from("<I", data, 4 * i)[0]
                for i, w in enumerate(words)}

    def write_params(self, params):
        self.enable_config()
        try:
            self.command(CMD_WRITE_GENERIC,
                         b"".join(struct.pack("<HI", w, v) for w, v in params))
        finally:
            self.end_config()


def find_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    # Silicon Labs CP210x (CP2102: VID 10C4, PID EA60), then anything that
    # looks like a USB serial adapter on macOS.
    cp = [p for p in ports if p.vid == 0x10C4]
    usb = [p for p in ports if re.search(r"usbserial|SLAB_USBtoUART|usbmodem", p.device)]
    pick = cp or usb
    # macOS lists each port twice; the cu. device is the one to open.
    pick = [p for p in pick if "/cu." in p.device] or pick
    if len(pick) == 1:
        return pick[0].device
    if not pick:
        fail("no USB-to-TTL adapter found. Plug it in, or pass --port. Ports seen: "
             + (", ".join(p.device for p in ports) or "none"))
    fail("several adapters found, pass --port: " + ", ".join(p.device for p in pick))


def self_test():
    """The frames this tool builds, against the bytes printed in the manual
    and the protocol document. No radar or pyserial needed."""
    cases = [
        ("enable configuration (7.3.2)",
         frame(CMD_ENABLE_CONFIG, struct.pack("<H", 1)),
         "FD FC FB FA 04 00 FF 00 01 00 04 03 02 01"),
        ("end configuration (7.3.3)",
         frame(CMD_END_CONFIG),
         "FD FC FB FA 02 00 FE 00 04 03 02 01"),
        ("write generic parameters, the manual's example (7.3.6)",
         write_generic_frame([(P_FARTHEST_GATE, 12), (P_NEAREST_GATE, 0),
                              (P_UNMANNED_S, 40), (P_STATUS_HZ10, 5),
                              (P_DIST_HZ10, 5), (P_RESPONSE, 5)]),
         "FD FC FB FA 26 00 70 00 05 00 0C 00 00 00 0A 00 00 00 00 00 "
         "06 00 28 00 00 00 02 00 05 00 00 00 0C 00 05 00 00 00 "
         "0B 00 05 00 00 00 04 03 02 01"),
        ("read generic parameters, the protocol's example (2.2.8)",
         read_generic_frame(ALL_PARAMS),
         "FD FC FB FA 0E 00 71 00 05 00 0A 00 06 00 02 00 0C 00 0B 00 04 03 02 01"),
    ]
    bad = 0
    for name, got, want in cases:
        ok = got == bytes.fromhex(want.replace(" ", ""))
        bad += not ok
        print("%s  %s" % ("ok  " if ok else "FAIL", name))
        if not ok:
            print("      got  " + hexs(got) + "\n      want " + want)
    print("passed" if not bad else "FAILED: %d" % bad)
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", help="serial device, e.g. /dev/cu.usbserial-0001")
    default_delay = config_h_delay()
    ap.add_argument("--unmanned-delay", type=int, default=default_delay,
                    help="seconds, 10-120 (default: RADAR_UNMANNED_DELAY_S "
                         "from include/config.h, %s)" % default_delay)
    ap.add_argument("--read-only", action="store_true",
                    help="read and print the radar's parameters, write nothing")
    ap.add_argument("--self-test", action="store_true",
                    help="check the command frames against the manual's examples")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    print(WIRING)
    delay = args.unmanned_delay
    if not args.read_only:
        if delay is None:
            fail("RADAR_UNMANNED_DELAY_S not found in include/config.h; "
                 "pass --unmanned-delay")
        if not 10 <= delay <= 120:
            fail("unmanned delay must be 10-120 s (manual Table 5-2); got %d" % delay)
        if default_delay is not None and delay != default_delay:
            print("WARNING: %d s differs from RADAR_UNMANNED_DELAY_S = %d in "
                  "include/config.h. Change one so they match.\n"
                  % (delay, default_delay))

    port = args.port or find_port()
    print("Port: %s, 115200 8N1\n" % port)
    radar = Radar(port)
    try:
        before = radar.read_params(ALL_PARAMS)
        print("The radar holds now:")
        for w in ALL_PARAMS:
            print("  %-36s %s" % (NAMES[w], show_value(w, before[w])))
        if args.read_only:
            return 0

        want = [(P_UNMANNED_S, delay),
                (P_STATUS_HZ10, MIN_REFRESH_HZ10),
                (P_DIST_HZ10, MIN_REFRESH_HZ10)]
        print("\nWriting: unmanned delay %d s, status and distance reporting "
              "%g Hz" % (delay, MIN_REFRESH_HZ10 / 10.0))
        radar.write_params(want)

        # A fresh enable/read/end, after the write's own end-configuration,
        # so what comes back is what the radar kept, not an echo.
        time.sleep(0.2)
        after = radar.read_params(ALL_PARAMS)
    finally:
        radar.close()

    print("\nRead back:")
    bad = 0
    for w in ALL_PARAMS:
        target = dict(want).get(w)
        mark = ""
        if target is not None:
            mark = "  OK" if after[w] == target else "  MISMATCH (wanted %s)" % show_value(w, target)
            bad += after[w] != target
        elif after[w] != before[w]:
            mark = "  CHANGED (was %s, not written)" % show_value(w, before[w])
        print("  %-36s %s%s" % (NAMES[w], show_value(w, after[w]), mark))
    if bad:
        fail("%d parameter(s) did not read back as written" % bad)
    print("\nSaved and confirmed. To be sure it survives power-off, unplug "
          "the adapter, plug it back in and run:\n  tools/radar_config.py --read-only")
    return 0


if __name__ == "__main__":
    sys.exit(main())
