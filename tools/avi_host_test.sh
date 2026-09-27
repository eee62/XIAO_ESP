#!/bin/sh
# Host test for the AVI writer (src/avi.cpp). Builds it with the host
# compiler, writes clips from a few sample JPEGs, and checks each with
# tools/avi_check.py (and ffprobe, when installed).
#
#   tools/avi_host_test.sh [a.jpg b.jpg ...]
#
# With no arguments the samples are made with macOS sips from a system image,
# at a few sizes and qualities so the frames differ in length, odd and even.
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

c++ -std=c++17 -O1 -Wall -Wextra -Werror -I"$root/include" \
	"$root/src/avi.cpp" "$root/tools/avi_host_test.cpp" -o "$work/avi_host_test"

if [ "$#" -gt 0 ]; then
	jpegs="$*"
else
	src=$(find /System/Library/CoreServices -name '*.icns' 2>/dev/null | head -n 1)
	[ -n "$src" ] || { echo "no sample image found; pass JPEGs as arguments"; exit 2; }
	jpegs=""
	i=0
	for q in 30 55 80 95; do
		out="$work/s$i.jpg"
		sips -s format jpeg -s formatOptions "$q" -z 600 800 "$src" --out "$out" >/dev/null
		jpegs="$jpegs $out"
		i=$((i + 1))
	done
fi
ls -l $jpegs | awk '{print "sample", $5, "bytes"}'

# run NAME EXPECT_FRAMES EXPECT_FPS EXPECT_FULL -- CAP FRAMES FPS_X1000 JITTER_US
run() {
	name=$1 want_frames=$2 want_fps=$3 want_full=$4; shift 5
	res=$("$work/avi_host_test" "$work/$name.avi" "$@" $jpegs)
	echo "$name: $res"
	frames=$(echo "$res" | sed 's/.*frames=\([0-9]*\).*/\1/')
	dur=$(echo "$res" | sed 's/.*duration_us=\([0-9]*\).*/\1/')
	fps=$(echo "$res" | sed 's/.*fps=\([0-9.]*\).*/\1/')
	full=$(echo "$res" | sed 's/.*full=\([0-9]*\).*/\1/')
	python3 "$root/tools/avi_check.py" "$work/$name.avi" "$frames" "$dur"
	python3 - "$frames" "$fps" "$full" "$want_frames" "$want_fps" "$want_full" <<'PY'
import sys
frames, fps, full, want_frames, want_fps, want_full = sys.argv[1:]
if want_frames != "*" and frames != want_frames:
    sys.exit("FAIL: %s frames, expected %s" % (frames, want_frames))
if abs(float(fps) - float(want_fps)) > 0.001 * float(want_fps):
    sys.exit("FAIL: measured %s fps, expected %s" % (fps, want_fps))
if full != want_full:
    sys.exit("FAIL: full=%s, expected %s" % (full, want_full))
PY
}

# 30 s at a steady 8 fps: 240 frames, 8.000 fps, 30.000 s.
run steady8   240 8.0 0 -- 8000000 240 8000 0
# A camera that only managed 7.3 fps, with jitter: the headers must carry the
# measured rate, not VIDEO_FPS.
run jitter73  100 7.3 0 -- 8000000 100 7300 20000
# A buffer too small for the request: must stop cleanly on "full" and still
# finish into a valid file inside its capacity.
run full      '*' 8.0 1 -- 60000   500 8000 0
# One frame: falls back to the nominal rate (8).
run single    1   8.0 0 -- 8000000 1   8000 0
echo "all AVI host tests passed"
