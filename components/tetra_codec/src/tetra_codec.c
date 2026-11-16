// SPDX-License-Identifier: AGPL-3.0-only
// TETRA speech decoder: wrapper around the ETSI EN 300 395-2 reference code.
//
// The reference sources are compiled as part of this one translation unit, unmodified. What they
// need to build and behave on a small target is arranged here:
//
//  - Their globals have everyday names (add, sub, round, Overflow, ...). This file is compiled
//    with -fwhole-program, which makes everything in it local except the functions marked
//    EXPORT below. As a side effect the compiler can inline the basic operators.
//  - Word32 is "long" in source.h, which is 64 bit on a 64 bit host. The typedefs are given
//    here and source.h's own are switched off through its include guard.
//  - round() would be taken for the C library function of that name.
//  - div_s() prints a message and calls exit() when it is handed a bad operand pair. That can
//    happen with a damaged frame that passed the CRC, and must not end the application: both
//    calls are compiled out and the division carries on with a meaningless (but harmless) result.

#include "tetra_codec.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define EXPORT __attribute__((externally_visible))

#ifdef TETRA_CODEC_ETSI

#define TYPEDEF_H
typedef int16_t Word16;
typedef int32_t Word32;
typedef int     Flag;

#define round       etsi_round
#define printf(...) ((void)0)
#define exit(code)  ((void)0)

// Each file defines short macros for itself (p, L, last, ...). In this order none of them
// meets an identifier of the same name in a later file; the host test compares the output of
// this build with a build of the separate files to make sure of it.
#include "tetra_op.c"
#include "fbas_tet.c"
#include "fexp_tet.c"
#include "fmat_tet.c"
#include "sub_dsp.c"
#include "sub_sc_d.c"
#undef pit_min
#undef pit_max
#include "sdec_tet.c"

#undef printf
#undef exit

EXPORT bool tetra_codec_available(void) {
    return true;
}

EXPORT void tetra_codec_reset(void) {
    Init_Decod_Tetra();
    Overflow = 0;
    Carry    = 0;
}

EXPORT void tetra_codec_decode(const uint8_t bits[TETRA_CODEC_FRAME_BITS], bool bad_frame,
                               int16_t pcm[TETRA_CODEC_FRAME_SAMPLES]) {
    Word16 serial[1 + TETRA_CODEC_FRAME_BITS];
    Word16 parm[24];
    serial[0] = bad_frame ? 1 : 0;
    for (int i = 0; i < TETRA_CODEC_FRAME_BITS; i++) serial[1 + i] = bits[i] & 1;
    Bits2prm_Tetra(serial, parm);
    Decod_Tetra(parm, pcm);
    Post_Process(pcm, (Word16)TETRA_CODEC_FRAME_SAMPLES);
}

#else  // no reference code at build time

EXPORT bool tetra_codec_available(void) {
    return false;
}

EXPORT void tetra_codec_reset(void) {
}

EXPORT void tetra_codec_decode(const uint8_t bits[TETRA_CODEC_FRAME_BITS], bool bad_frame,
                               int16_t pcm[TETRA_CODEC_FRAME_SAMPLES]) {
    (void)bits;
    (void)bad_frame;
    memset(pcm, 0, sizeof(int16_t) * TETRA_CODEC_FRAME_SAMPLES);
}

#endif
