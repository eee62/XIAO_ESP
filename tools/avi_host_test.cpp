// Host-side driver for the AVI writer (src/avi.cpp), used by
// tools/avi_host_test.sh. Not part of any firmware build.
//
//   avi_host_test OUT.avi CAP_BYTES FRAMES FPS_X1000 JITTER_US a.jpg [b.jpg ...]
//
// Appends FRAMES frames, cycling through the given JPEGs, stamped as a camera
// running at FPS_X1000 / 1000 frames per second with up to +/- JITTER_US of
// deterministic jitter on every frame but the first and last. Stops early if
// the buffer fills, as the firmware does. Prints one line of key=value
// results for the checker.
#include "avi.h"

#include <stdio.h>
#include <stdlib.h>
#include <vector>

static std::vector<uint8_t> slurp(const char *path)
{
	std::vector<uint8_t> v;
	FILE *f = fopen(path, "rb");
	if (!f) {
		perror(path);
		exit(2);
	}
	uint8_t tmp[65536];
	size_t n;
	while ((n = fread(tmp, 1, sizeof(tmp), f)) > 0) {
		v.insert(v.end(), tmp, tmp + n);
	}
	fclose(f);
	return v;
}

int main(int argc, char **argv)
{
	if (argc < 7) {
		fprintf(stderr, "usage: %s OUT CAP FRAMES FPS_X1000 JITTER_US JPEG...\n", argv[0]);
		return 2;
	}
	const char  *out       = argv[1];
	const size_t cap       = strtoul(argv[2], nullptr, 10);
	const int    frames    = atoi(argv[3]);
	const double fps       = atof(argv[4]) / 1000.0;
	const long   jitter_us = atol(argv[5]);

	std::vector<std::vector<uint8_t>> jpegs;
	for (int i = 6; i < argc; i++) {
		jpegs.push_back(slurp(argv[i]));
	}

	std::vector<uint8_t> buf(cap);
	avi_t a;
	if (!avi_begin(&a, buf.data(), cap, 800, 600, 8)) {
		fprintf(stderr, "avi_begin failed\n");
		return 1;
	}

	const double interval = 1e6 / fps;
	int added = 0;
	bool full = false;
	for (int i = 0; i < frames; i++) {
		long j = 0;
		if (i > 0 && i < frames - 1 && jitter_us > 0) {
			j = (long)((i * 7919L) % (2 * jitter_us + 1)) - jitter_us;
		}
		const uint64_t t = 5000000ull + (uint64_t)(i * interval) + j;
		const std::vector<uint8_t> &jp = jpegs[i % jpegs.size()];
		if (!avi_add_frame(&a, jp.data(), jp.size(), t)) {
			full = true;
			break;
		}
		added++;
	}

	const uint32_t dur = avi_duration_us(&a);
	const float    f   = avi_fps(&a);
	const size_t   len = avi_finish(&a);
	if (len > cap) {
		fprintf(stderr, "finished length %zu past cap %zu\n", len, cap);
		return 1;
	}

	FILE *o = fopen(out, "wb");
	if (!o || fwrite(buf.data(), 1, len, o) != len) {
		perror(out);
		return 2;
	}
	fclose(o);
	printf("frames=%d full=%d duration_us=%u fps=%.4f len=%zu\n", added,
	       full ? 1 : 0, dur, f, len);
	return 0;
}
