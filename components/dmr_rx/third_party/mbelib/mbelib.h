/*
 * Copyright (C) 2010 mbelib Author
 * GPG Key ID: 0xEA5EFE2C (9E7A 5527 9CDC EBF7 BF1B  D772 4F98 E863 EA5E FE2C)
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS.  IN NO EVENT SHALL ISC BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/* Modified for dmr_rx (tanmatsu-sdr), see NOTICE.md: reduced to the
 * AMBE 3600x2450 (DMR) code path, symbols prefixed, speech synthesis made
 * single-precision float only and re-entrant. */

#ifndef DMRRX_MBELIB_H
#define DMRRX_MBELIB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define mbe_checkGolayBlock             dmrrx_mbe_checkGolayBlock
#define mbe_golay2312                   dmrrx_mbe_golay2312
#define mbe_eccAmbe3600x2450C0          dmrrx_mbe_eccAmbe3600x2450C0
#define mbe_eccAmbe3600x2450Data        dmrrx_mbe_eccAmbe3600x2450Data
#define mbe_demodulateAmbe3600x2450Data dmrrx_mbe_demodulateAmbe3600x2450Data

/*
 * Prototypes from ecc.c
 */
void mbe_checkGolayBlock (long int *block);
int mbe_golay2312 (char *in, char *out);

/*
 * Prototypes from ambe3600x2450_ecc.c
 */
int mbe_eccAmbe3600x2450C0 (char ambe_fr[4][24]);
int mbe_eccAmbe3600x2450Data (char ambe_fr[4][24], char *ambe_d);
void mbe_demodulateAmbe3600x2450Data (char ambe_fr[4][24]);

/*
 * Speech decoder (compiled only with DMR_RX_WITH_VOICE)
 */
#define mbe_moveMbeParms                dmrrx_mbe_moveMbeParms
#define mbe_useLastMbeParms             dmrrx_mbe_useLastMbeParms
#define mbe_initMbeParms                dmrrx_mbe_initMbeParms
#define mbe_initSynth                   dmrrx_mbe_initSynth
#define mbe_spectralAmpEnhance          dmrrx_mbe_spectralAmpEnhance
#define mbe_synthesizeSilencef          dmrrx_mbe_synthesizeSilencef
#define mbe_synthesizeSpeechf           dmrrx_mbe_synthesizeSpeechf
#define mbe_floattoshort                dmrrx_mbe_floattoshort
#define mbe_decodeAmbe2450Parms         dmrrx_mbe_decodeAmbe2450Parms
#define mbe_processAmbe2450Dataf        dmrrx_mbe_processAmbe2450Dataf

/* Samples of one frame during which the synthesis window of one side
 * (previous or current frame parameters) is non-zero. */
#define MBE_SYNTH_SPAN 105

struct mbe_parameters
{
  float w0;
  int L;
  int repeat;
  float gamma;
  unsigned char Vl[57];
  float Ml[57];
  float log2Ml[58];             /* [57] is always 0, see mbe_decodeAmbe2450Parms */
  float PHIl[57];
  float PSIl[57];
};

typedef struct mbe_parameters mbe_parms;

/* Synthesiser state that upstream kept in the C library (rand ()) or on the
 * stack. */
struct mbe_synth_state
{
  uint32_t rand_state;                  /* PRNG replacing rand () */
  float work[2][MBE_SYNTH_SPAN];        /* unvoiced part of one harmonic: [0] previous, [1] current frame */
};

typedef struct mbe_synth_state mbe_synth;

/* Return values of mbe_processAmbe2450Dataf */
#define MBE_FRAME_OK      0     /* decoded normally */
#define MBE_FRAME_REPEAT  1     /* previous parameters repeated */
#define MBE_FRAME_ERASURE 2     /* erasure frame: silence, state re-initialised */
#define MBE_FRAME_TONE    3     /* tone frame: silence, state re-initialised */
#define MBE_FRAME_MUTE    4     /* more than 3 repeats: silence, state re-initialised */

/*
 * Prototypes from ambe3600x2450.c
 */
int mbe_decodeAmbe2450Parms (const char *ambe_d, mbe_parms * cur_mp, mbe_parms * prev_mp);
int mbe_processAmbe2450Dataf (float *aout_buf, int repeat, const char ambe_d[49], mbe_parms * cur_mp, mbe_parms * prev_mp, mbe_parms * prev_mp_enhanced, mbe_synth * syn);

/*
 * Prototypes from mbelib.c
 */
void mbe_moveMbeParms (const mbe_parms * cur_mp, mbe_parms * prev_mp);
void mbe_useLastMbeParms (mbe_parms * cur_mp, const mbe_parms * prev_mp);
void mbe_initMbeParms (mbe_parms * cur_mp, mbe_parms * prev_mp, mbe_parms * prev_mp_enhanced);
void mbe_initSynth (mbe_synth * syn);
void mbe_spectralAmpEnhance (mbe_parms * cur_mp);
void mbe_synthesizeSilencef (float *aout_buf);
void mbe_synthesizeSpeechf (float *aout_buf, mbe_parms * cur_mp, mbe_parms * prev_mp, mbe_synth * syn);
void mbe_floattoshort (const float *float_buf, int16_t *aout_buf, float gain);

#ifdef __cplusplus
}
#endif

#endif
