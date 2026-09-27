// MJPEG-in-AVI writer. See include/avi.h for the layout and the contract.

#include "avi.h"

#include <string.h>

// Fixed header block: RIFF (12) + LIST hdrl (12) + avih (8 + 56)
// + LIST strl (12) + strh (8 + 56) + strf (8 + 40) + LIST movi (12).
#define AVI_OFS_RIFF_SIZE   4
#define AVI_OFS_HDRL        12
#define AVI_OFS_AVIH        24
#define AVI_OFS_STRL        88
#define AVI_OFS_STRH        100
#define AVI_OFS_STRF        164
#define AVI_OFS_MOVI        212
#define AVI_HEADER_LEN      224

#define AVI_CHUNK_HDR       8
#define AVI_IDX_ENTRY       16

#define AVIF_HASINDEX       0x00000010u
#define AVIIF_KEYFRAME      0x00000010u

static void put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static void put16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static uint32_t get32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
	       (uint32_t)p[3] << 24;
}

static void put_fourcc(uint8_t *p, const char *cc)
{
	memcpy(p, cc, 4);
}

// Everything a frame costs once the clip is finished: its chunk, padded, plus
// its index entry.
static size_t frame_cost(size_t len)
{
	return AVI_CHUNK_HDR + len + (len & 1) + AVI_IDX_ENTRY;
}

bool avi_begin(avi_t *a, uint8_t *buf, size_t cap, uint16_t width,
               uint16_t height, uint32_t nominal_fps)
{
	memset(a, 0, sizeof(*a));
	// Headers plus the idx1 chunk header, which avi_finish() always writes.
	if (!buf || cap < AVI_HEADER_LEN + AVI_CHUNK_HDR) {
		return false;
	}
	a->buf         = buf;
	a->cap         = cap;
	a->len         = AVI_HEADER_LEN;
	a->width       = width;
	a->height      = height;
	a->nominal_fps = nominal_fps ? nominal_fps : 1;
	memset(buf, 0, AVI_HEADER_LEN);
	return true;
}

bool avi_add_frame(avi_t *a, const uint8_t *jpeg, size_t len, uint64_t t_us)
{
	if (!a->buf || !jpeg || len == 0 || len > 0x7FFFFFFFu) {
		return false;
	}
	// Room for this frame and every index entry so far, including its own,
	// plus the idx1 header. Checked in full here so avi_finish() never can
	// fail for space.
	const size_t need = a->len + frame_cost(len) +
	                    (size_t)a->frames * AVI_IDX_ENTRY + AVI_CHUNK_HDR;
	if (need > a->cap) {
		return false;
	}

	uint8_t *p = a->buf + a->len;
	put_fourcc(p, "00dc");
	put32(p + 4, (uint32_t)len);
	memcpy(p + AVI_CHUNK_HDR, jpeg, len);
	if (len & 1) {
		p[AVI_CHUNK_HDR + len] = 0;   // RIFF chunks are word-aligned
	}
	a->len += AVI_CHUNK_HDR + len + (len & 1);

	if (a->frames == 0) {
		a->t_first_us = t_us;
	}
	a->t_last_us = t_us;
	if (len > a->max_frame) {
		a->max_frame = (uint32_t)len;
	}
	a->frames++;
	return true;
}

uint32_t avi_duration_us(const avi_t *a)
{
	if (a->frames == 0) {
		return 0;
	}
	if (a->frames == 1 || a->t_last_us <= a->t_first_us) {
		return (uint32_t)(1000000u / a->nominal_fps) * a->frames;
	}
	// N frame starts span N - 1 intervals; the clip lasts N of them.
	const uint64_t span = a->t_last_us - a->t_first_us;
	return (uint32_t)(span * a->frames / (a->frames - 1));
}

float avi_fps(const avi_t *a)
{
	const uint32_t d = avi_duration_us(a);
	return d ? (float)a->frames * 1e6f / (float)d : 0.0f;
}

size_t avi_finish(avi_t *a)
{
	if (!a->buf || a->frames == 0) {
		return 0;
	}
	uint8_t *b = a->buf;
	const uint32_t dur_us   = avi_duration_us(a);
	const uint32_t us_frame = (dur_us + a->frames / 2) / a->frames;
	// Rate as dwRate / dwScale with a scale of 1000, so a measured 7.46 fps
	// is written as 7460 / 1000 rather than rounded to a whole number.
	const uint32_t scale = 1000;
	const uint32_t rate  = (uint32_t)(((uint64_t)a->frames * 1000000000ull +
	                                   dur_us / 2) / dur_us);
	const uint32_t max_bps = (uint32_t)((uint64_t)a->max_frame * rate / scale);
	const size_t   movi_end = a->len;

	// ---- idx1, walked back out of the movi chunks -------------------------
	// Offsets are relative to the 'movi' fourcc, so the first chunk is at 4.
	uint8_t *idx = b + movi_end;
	put_fourcc(idx, "idx1");
	put32(idx + 4, a->frames * AVI_IDX_ENTRY);
	uint8_t *e = idx + AVI_CHUNK_HDR;
	size_t pos = AVI_HEADER_LEN;
	for (uint32_t i = 0; i < a->frames; i++) {
		const uint32_t clen = get32(b + pos + 4);
		put_fourcc(e, "00dc");
		put32(e + 4, AVIIF_KEYFRAME);
		put32(e + 8, (uint32_t)(pos - (AVI_OFS_MOVI + 8)));
		put32(e + 12, clen);
		e   += AVI_IDX_ENTRY;
		pos += AVI_CHUNK_HDR + clen + (clen & 1);
	}
	a->len = (size_t)(e - b);

	// ---- headers ----------------------------------------------------------
	put_fourcc(b, "RIFF");
	put32(b + AVI_OFS_RIFF_SIZE, (uint32_t)(a->len - 8));
	put_fourcc(b + 8, "AVI ");

	put_fourcc(b + AVI_OFS_HDRL, "LIST");
	put32(b + AVI_OFS_HDRL + 4, AVI_OFS_MOVI - AVI_OFS_HDRL - 8);
	put_fourcc(b + AVI_OFS_HDRL + 8, "hdrl");

	uint8_t *h = b + AVI_OFS_AVIH;
	put_fourcc(h, "avih");
	put32(h + 4, 56);
	h += 8;
	put32(h + 0, us_frame);          // dwMicroSecPerFrame
	put32(h + 4, max_bps);           // dwMaxBytesPerSec
	put32(h + 8, 0);                 // dwPaddingGranularity
	put32(h + 12, AVIF_HASINDEX);    // dwFlags
	put32(h + 16, a->frames);        // dwTotalFrames
	put32(h + 20, 0);                // dwInitialFrames
	put32(h + 24, 1);                // dwStreams
	put32(h + 28, a->max_frame);     // dwSuggestedBufferSize
	put32(h + 32, a->width);         // dwWidth
	put32(h + 36, a->height);        // dwHeight
	memset(h + 40, 0, 16);           // dwReserved[4]

	put_fourcc(b + AVI_OFS_STRL, "LIST");
	put32(b + AVI_OFS_STRL + 4, AVI_OFS_MOVI - AVI_OFS_STRL - 8);
	put_fourcc(b + AVI_OFS_STRL + 8, "strl");

	uint8_t *s = b + AVI_OFS_STRH;
	put_fourcc(s, "strh");
	put32(s + 4, 56);
	s += 8;
	put_fourcc(s + 0, "vids");       // fccType
	put_fourcc(s + 4, "MJPG");       // fccHandler
	put32(s + 8, 0);                 // dwFlags
	put16(s + 12, 0);                // wPriority
	put16(s + 14, 0);                // wLanguage
	put32(s + 16, 0);                // dwInitialFrames
	put32(s + 20, scale);            // dwScale
	put32(s + 24, rate);             // dwRate: fps = rate / scale
	put32(s + 28, 0);                // dwStart
	put32(s + 32, a->frames);        // dwLength, in frames
	put32(s + 36, a->max_frame);     // dwSuggestedBufferSize
	put32(s + 40, 0xFFFFFFFFu);      // dwQuality: default
	put32(s + 44, 0);                // dwSampleSize: varies per frame
	put16(s + 48, 0);                // rcFrame
	put16(s + 50, 0);
	put16(s + 52, a->width);
	put16(s + 54, a->height);

	uint8_t *f = b + AVI_OFS_STRF;
	put_fourcc(f, "strf");
	put32(f + 4, 40);
	f += 8;
	put32(f + 0, 40);                // biSize
	put32(f + 4, a->width);          // biWidth
	put32(f + 8, a->height);         // biHeight
	put16(f + 12, 1);                // biPlanes
	put16(f + 14, 24);               // biBitCount
	put_fourcc(f + 16, "MJPG");      // biCompression
	put32(f + 20, (uint32_t)a->width * a->height * 3);   // biSizeImage
	put32(f + 24, 0);                // biXPelsPerMeter
	put32(f + 28, 0);                // biYPelsPerMeter
	put32(f + 32, 0);                // biClrUsed
	put32(f + 36, 0);                // biClrImportant

	put_fourcc(b + AVI_OFS_MOVI, "LIST");
	put32(b + AVI_OFS_MOVI + 4, (uint32_t)(movi_end - AVI_OFS_MOVI - 8));
	put_fourcc(b + AVI_OFS_MOVI + 8, "movi");

	return a->len;
}
