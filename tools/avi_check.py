#!/usr/bin/env python3
"""Structural check of an MJPEG AVI from src/avi.cpp, for tools/avi_host_test.sh.

    avi_check.py FILE.avi FRAMES DURATION_US

Walks the RIFF tree the way a demuxer does and fails loudly on anything a
player would trip over: sizes that disagree with the file, a missing or
misplaced chunk, index entries that do not point at the frames, frames that
are not complete JPEGs. Then checks that frame count, frame rate and duration
(dwLength * dwScale / dwRate, the figure ffprobe reports for this layout)
match what the writer measured. If ffprobe is on PATH it is asked as well.
"""
import json
import shutil
import struct
import subprocess
import sys


def fail(msg):
    print("FAIL: " + msg)
    sys.exit(1)


def chunks(data, start, end):
    pos = start
    while pos < end:
        if pos + 8 > end:
            fail("truncated chunk header at %d" % pos)
        cc = data[pos:pos + 4].decode("latin-1")
        size = struct.unpack_from("<I", data, pos + 4)[0]
        if pos + 8 + size > end:
            fail("chunk %r at %d runs past its parent (%d > %d)"
                 % (cc, pos, pos + 8 + size, end))
        yield cc, pos, size
        pos += 8 + size + (size & 1)
    if pos != end:
        fail("chunks overrun their parent: %d != %d" % (pos, end))


def main():
    path, want_frames, want_dur = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    data = open(path, "rb").read()

    if data[:4] != b"RIFF" or data[8:12] != b"AVI ":
        fail("not RIFF AVI")
    riff_size = struct.unpack_from("<I", data, 4)[0]
    if riff_size + 8 != len(data):
        fail("RIFF size %d + 8 != file length %d" % (riff_size, len(data)))

    top = list(chunks(data, 12, len(data)))
    names = [(c, data[p + 8:p + 12].decode("latin-1") if c == "LIST" else None)
             for c, p, _ in top]
    if names != [("LIST", "hdrl"), ("LIST", "movi"), ("idx1", None)]:
        fail("top-level layout is %r" % names)
    (_, hdrl, hdrl_size), (_, movi, movi_size), (_, idx1, idx1_size) = top

    hdr = list(chunks(data, hdrl + 12, hdrl + 8 + hdrl_size))
    if [c for c, _, _ in hdr] != ["avih", "LIST"]:
        fail("hdrl holds %r" % [c for c, _, _ in hdr])
    avih = struct.unpack_from("<10I", data, hdr[0][1] + 8)
    us_frame, _, _, flags, total_frames, _, streams, _, width, height = avih
    if not flags & 0x10:
        fail("AVIF_HASINDEX not set")
    if streams != 1:
        fail("dwStreams %d" % streams)

    strl_pos, strl_size = hdr[1][1], hdr[1][2]
    if data[strl_pos + 8:strl_pos + 12] != b"strl":
        fail("second hdrl LIST is not strl")
    strl = list(chunks(data, strl_pos + 12, strl_pos + 8 + strl_size))
    if [c for c, _, _ in strl] != ["strh", "strf"]:
        fail("strl holds %r" % [c for c, _, _ in strl])
    sp = strl[0][1] + 8
    if data[sp:sp + 4] != b"vids" or data[sp + 4:sp + 8] != b"MJPG":
        fail("stream is %r/%r" % (data[sp:sp + 4], data[sp + 4:sp + 8]))
    scale, rate, _, length = struct.unpack_from("<4I", data, sp + 20)
    fp = strl[1][1] + 8
    bi_w, bi_h = struct.unpack_from("<ii", data, fp + 4)
    if data[fp + 16:fp + 20] != b"MJPG":
        fail("biCompression %r" % data[fp + 16:fp + 20])
    if (bi_w, bi_h) != (width, height):
        fail("strf %dx%d vs avih %dx%d" % (bi_w, bi_h, width, height))

    frames = list(chunks(data, movi + 12, movi + 8 + movi_size))
    for i, (c, p, s) in enumerate(frames):
        if c != "00dc":
            fail("movi chunk %d is %r" % (i, c))
        jpg = data[p + 8:p + 8 + s]
        if jpg[:2] != b"\xff\xd8" or jpg[-2:] != b"\xff\xd9":
            fail("frame %d is not a whole JPEG" % i)

    if idx1_size != 16 * len(frames):
        fail("idx1 has %d bytes for %d frames" % (idx1_size, len(frames)))
    movi_fourcc = movi + 8
    for i, (c, p, s) in enumerate(frames):
        cc, fl, off, sz = struct.unpack_from("<4sIII", data, idx1 + 8 + 16 * i)
        if cc != b"00dc" or not fl & 0x10:
            fail("idx1 entry %d is %r flags %x" % (i, cc, fl))
        if movi_fourcc + off != p or sz != s:
            fail("idx1 entry %d points at %d/%d, frame is at %d/%d"
                 % (i, movi_fourcc + off, sz, p, s))

    n = len(frames)
    if not (n == total_frames == length == want_frames):
        fail("frame counts: chunks %d avih %d strh %d expected %d"
             % (n, total_frames, length, want_frames))
    fps = rate / scale
    dur_us = length * scale * 1e6 / rate
    want_fps = n * 1e6 / want_dur
    if abs(fps - want_fps) > 0.001 * want_fps:
        fail("fps %.4f, measured %.4f" % (fps, want_fps))
    if abs(dur_us - want_dur) > 0.001 * want_dur:
        fail("duration %.0f us, measured %d" % (dur_us, want_dur))
    if abs(us_frame - want_dur / n) > 1:
        fail("dwMicroSecPerFrame %d, expected %.1f" % (us_frame, want_dur / n))

    print("ok: %d frames, %dx%d, %.3f fps, %.3f s, %d bytes"
          % (n, width, height, fps, dur_us / 1e6, len(data)))

    if shutil.which("ffprobe"):
        out = subprocess.run(
            ["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0",
             "-show_entries", "stream=codec_name,width,height,r_frame_rate,"
             "avg_frame_rate,nb_read_frames:format=duration", "-of", "json", path],
            capture_output=True, text=True, check=True).stdout
        info = json.loads(out)
        st, fmt = info["streams"][0], info["format"]
        num, den = (int(x) for x in st["avg_frame_rate"].split("/"))
        if st["codec_name"] != "mjpeg" or int(st["nb_read_frames"]) != n:
            fail("ffprobe: %r" % st)
        if abs(num / den - fps) > 0.001 * fps:
            fail("ffprobe fps %s vs %.4f" % (st["avg_frame_rate"], fps))
        if abs(float(fmt["duration"]) - dur_us / 1e6) > 0.01:
            fail("ffprobe duration %s vs %.3f" % (fmt["duration"], dur_us / 1e6))
        print("ffprobe agrees: %s frames, %s fps, %s s"
              % (st["nb_read_frames"], st["avg_frame_rate"], fmt["duration"]))
    else:
        print("(ffprobe not installed; structural check only)")


if __name__ == "__main__":
    main()
