/*
 * dmr_rx - AMBE+2 vocoder glue (stage 2, DMR_RX_WITH_VOICE).
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include <stdlib.h>
#include <string.h>

#include "dmr_internal.h"

#ifndef DMR_RX_WITH_VOICE

namespace dmrrx {
size_t vocoder_size()
{
    return 0;
}
void vocoder_init(Vocoder *)
{
}
void vocoder_decode(Vocoder *, const char *, int, bool, float, int16_t *pcm)
{
    memset(pcm, 0, DMR_RX_PCM_FRAME_SAMPLES * sizeof(int16_t));
}
} // namespace dmrrx

extern "C" {
dmr_vocoder_t *dmr_vocoder_create(void)
{
    return nullptr;
}
void dmr_vocoder_destroy(dmr_vocoder_t *)
{
}
void dmr_vocoder_reset(dmr_vocoder_t *)
{
}
void dmr_vocoder_decode(dmr_vocoder_t *, const uint8_t *, int, int, float, int16_t *pcm)
{
    if (pcm)
        memset(pcm, 0, DMR_RX_PCM_FRAME_SAMPLES * sizeof(int16_t));
}
}

#else /* DMR_RX_WITH_VOICE */

#include "mbelib.h"

namespace dmrrx {

/* AMBE+2 (AMBE 3600x2450) decoder state of one timeslot: the three parameter
 * sets of mbelib plus what upstream kept in globals / on the stack (random
 * generator, work buffers). No pointers, so it can be placed anywhere. */
struct Vocoder {
    mbe_parms cur;
    mbe_parms prev;
    mbe_parms prev_enhanced;
    mbe_synth syn;
    float aout[DMR_RX_PCM_FRAME_SAMPLES];
};

size_t vocoder_size()
{
    return sizeof(Vocoder);
}

void vocoder_init(Vocoder *v)
{
    mbe_initMbeParms(&v->cur, &v->prev, &v->prev_enhanced);
    mbe_initSynth(&v->syn); /* reseeds the noise generator: decoding is reproducible per call */
    memset(v->aout, 0, sizeof(v->aout));
}

/*
 * Error concealment policy:
 *  - bad (C0 uncorrectable) or errs >= 6 (C0 + C1 Golay distance): the
 *    frame is not used, the parameters of the previous frame are repeated
 *    (mbelib's mbe_useLastMbeParms + repeat counter). After 3 consecutive
 *    repeats the output is muted (silence) and the decoder state is
 *    re-initialised, exactly as upstream does for errs2 > 3.
 *  - otherwise the frame is decoded normally and the repeat counter cleared.
 *  - erasure frames (b0 120..123) and tone frames (b0 126..127) are handled
 *    as upstream mbelib does: 20 ms of silence and the decoder state is
 *    re-initialised (tones are not synthesised). As upstream, this check
 *    comes first, also for frames that would otherwise be repeated.
 *  - AMBE silence frames (b0 124..125) are synthesised as upstream does
 *    (L = 14, all bands unvoiced, comfort noise at the decoded gain).
 */
void vocoder_decode(Vocoder *v, const char ambe_d[49], int errs, bool bad, float gain,
                    int16_t pcm[DMR_RX_PCM_FRAME_SAMPLES])
{
    const int repeat = (bad || errs >= 6) ? 1 : 0;
    mbe_processAmbe2450Dataf(v->aout, repeat, ambe_d, &v->cur, &v->prev, &v->prev_enhanced, &v->syn);
    /* gain 1.0 is upstream's scaling (x7), saturating at +-32760 */
    mbe_floattoshort(v->aout, pcm, gain);
}

} // namespace dmrrx

struct dmr_vocoder {
    dmrrx::Vocoder v;
};

extern "C" {
dmr_vocoder_t *dmr_vocoder_create(void)
{
    dmr_vocoder_t *v = (dmr_vocoder_t *)malloc(sizeof(dmr_vocoder_t));
    if (v)
        dmrrx::vocoder_init(&v->v);
    return v;
}
void dmr_vocoder_destroy(dmr_vocoder_t *v)
{
    free(v);
}
void dmr_vocoder_reset(dmr_vocoder_t *v)
{
    if (v)
        dmrrx::vocoder_init(&v->v);
}
void dmr_vocoder_decode(dmr_vocoder_t *v, const uint8_t ambe49[7], int errs, int bad, float gain,
                        int16_t pcm[DMR_RX_PCM_FRAME_SAMPLES])
{
    if (!pcm)
        return;
    if (!v || !ambe49) {
        memset(pcm, 0, DMR_RX_PCM_FRAME_SAMPLES * sizeof(int16_t));
        return;
    }
    char ambe_d[49];
    for (int i = 0; i < 49; i++)
        ambe_d[i] = (char)((ambe49[i >> 3] >> (7 - (i & 7))) & 1);
    dmrrx::vocoder_decode(&v->v, ambe_d, errs, bad != 0, gain, pcm);
}
}

#endif
