// SPDX-License-Identifier: AGPL-3.0-only
// TETRA speech decoder (ACELP, ETSI EN 300 395-2): a thin wrapper around the ETSI reference code.
//
// The reference code is not part of this repository and may not be redistributed. It is compiled
// in only when the archive ETSI publishes with the standard is present at build time, see the
// README of this component. Without it the functions below exist but tetra_codec_available()
// returns false and tetra_codec_decode() produces silence.
//
// The reference code keeps its state in static variables: one decoder, used from one task.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TETRA_CODEC_FRAME_BITS    137  // one speech frame, in the order of the codec's serial stream
#define TETRA_CODEC_FRAME_SAMPLES 240  // 30 ms at 8 kHz

bool tetra_codec_available(void);

// Start of a new call: forget the previous one.
void tetra_codec_reset(void);

// Decode one speech frame. bits: one bit per byte. bad_frame: the channel decoder's bad frame
// indicator (or a stolen half slot); the bits are then ignored and the frame is concealed.
void tetra_codec_decode(const uint8_t bits[TETRA_CODEC_FRAME_BITS], bool bad_frame,
                        int16_t pcm[TETRA_CODEC_FRAME_SAMPLES]);

#ifdef __cplusplus
}
#endif
