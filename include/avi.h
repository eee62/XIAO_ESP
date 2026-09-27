// MJPEG-in-AVI writer for the presence clips (config.h, VIDEO_*).
//
// The file is built in place in one caller-owned buffer (PSRAM on the node):
// header space is reserved at the front, frames are appended as `00dc`
// chunks as they arrive, and avi_finish() fills the headers in and writes the
// `idx1` index after the last frame. Nothing is copied a second time, so the
// finished file can be streamed straight from the buffer (item 5's sender).
//
// Layout:
//   RIFF 'AVI '
//     LIST 'hdrl'
//       'avih'                  main header
//       LIST 'strl'
//         'strh'  'vids' 'MJPG' stream header
//         'strf'                BITMAPINFOHEADER
//     LIST 'movi'
//       '00dc' <jpeg> [pad]     one per frame, padded to even length
//     'idx1'                    one entry per frame
//
// The frame rate written into the headers is the *measured* average over the
// clip, from the frames' own timestamps, so playback runs at real speed even
// when the camera did not hold VIDEO_FPS.
//
// No Arduino or ESP-IDF dependency: tools/avi_host_test.sh compiles this file
// on the host and checks its output.
#pragma once

#include <stddef.h>
#include <stdint.h>

struct avi_t {
	uint8_t *buf;
	size_t   cap;
	size_t   len;           // bytes written so far
	uint16_t width;
	uint16_t height;
	uint32_t frames;
	uint32_t max_frame;     // largest JPEG, for dwSuggestedBufferSize
	uint64_t t_first_us;    // start of the first frame
	uint64_t t_last_us;     // start of the last frame
	uint32_t nominal_fps;   // used only when a clip has a single frame
};

// Start a clip in `buf`. False if `cap` cannot hold even the headers.
bool avi_begin(avi_t *a, uint8_t *buf, size_t cap, uint16_t width,
               uint16_t height, uint32_t nominal_fps);

// Append one JPEG, started at `t_us` (any monotonic microsecond clock).
// False, with nothing written, when the frame and the index entries it
// would need no longer fit: the buffer is full.
bool avi_add_frame(avi_t *a, const uint8_t *jpeg, size_t len, uint64_t t_us);

// Write the headers and the index. Returns the finished file's length, the
// bytes to send from a->buf; 0 if the clip has no frames.
size_t avi_finish(avi_t *a);

// The measured clip: duration covers every frame, one average interval each,
// so frames / duration is the rate avi_finish() writes.
uint32_t avi_duration_us(const avi_t *a);
float    avi_fps(const avi_t *a);
