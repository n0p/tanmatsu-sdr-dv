// SPDX-License-Identifier: AGPL-3.0-only
// Host test of the tetra_codec wrapper against the ETSI reference code built file by file.
//
// A speech-like test signal goes through the reference coder; the frames are then decoded by the
// reference decoder and by the wrapper (the single-translation-unit build the firmware uses) and
// the two outputs must be identical: clean, with bad frames, and with random bits.
// (ref_adapt.h is force-included: Word16 / Word32 and the standard headers.)
#include <math.h>
#include <time.h>
#include "source.h"
#include "tetra_codec.h"

#undef printf
#undef exit

#define FRAMES 700  // 21 s
#define L      240

extern Word16* new_speech;

static uint32_t rnd_state = 12345;
static uint32_t rnd(void) {
    rnd_state = rnd_state * 1664525u + 1013904223u;
    return rnd_state >> 8;
}

// Voiced segments (glottal pulses through three formant resonators, moving pitch), unvoiced
// noise bursts and pauses.
static void make_signal(int16_t* out, int n) {
    double y1[3] = {0}, y2[3] = {0};
    double phase = 0, f0 = 120;
    int    seg_left = 0, kind = 0;
    double form[3] = {700, 1200, 2600}, amp = 0;
    for (int i = 0; i < n; i++) {
        if (seg_left == 0) {
            kind     = rnd() % 4;  // 0 pause, 1 noise, 2..3 voiced
            seg_left = 400 + rnd() % 2400;
            f0       = 90 + rnd() % 160;
            form[0]  = 300 + rnd() % 600;
            form[1]  = 900 + rnd() % 1400;
            form[2]  = 2400 + rnd() % 800;
            amp      = 1500 + rnd() % 9000;
        }
        seg_left--;
        double x = 0;
        if (kind >= 2) {
            f0    += 0.002 * ((int)(rnd() % 21) - 10);
            phase += f0 / 8000.0;
            if (phase >= 1.0) {
                phase -= 1.0;
                x      = 1.0;
            }
        } else if (kind == 1) {
            x = ((int)(rnd() % 2001) - 1000) / 12000.0;
        }
        double s = 0;
        for (int k = 0; k < 3; k++) {
            double r = 0.96, w = 2 * M_PI * form[k] / 8000.0;
            double y = x + 2 * r * cos(w) * y1[k] - r * r * y2[k];
            y2[k]    = y1[k];
            y1[k]    = y;
            s       += y / (k + 1);
        }
        s      *= amp * (kind == 1 ? 3.0 : 0.25);
        out[i]  = (int16_t)(s > 32000 ? 32000 : s < -32000 ? -32000 : s);
    }
}

static Word16 serial[FRAMES][138];

static int compare(const char* what, int n_frames) {
    static Word16  ref[L];
    static int16_t got[L];
    Word16         parm[24];
    int            bad = 0;
    double         energy = 0;
    Init_Decod_Tetra();
    tetra_codec_reset();
    for (int f = 0; f < n_frames; f++) {
        uint8_t bits[TETRA_CODEC_FRAME_BITS];
        for (int i = 0; i < 137; i++) bits[i] = (uint8_t)serial[f][1 + i];
        Word16 tmp[138];
        memcpy(tmp, serial[f], sizeof(tmp));
        Bits2prm_Tetra(tmp, parm);
        Decod_Tetra(parm, ref);
        Post_Process(ref, (Word16)L);
        tetra_codec_decode(bits, serial[f][0] != 0, got);
        if (memcmp(ref, got, sizeof(ref)) != 0) bad++;
        for (int i = 0; i < L; i++) energy += (double)got[i] * got[i];
    }
    printf("  %-34s %4d frames, %4d differ, output rms %7.1f   %s\n", what, n_frames, bad,
           sqrt(energy / (n_frames * (double)L)), bad ? "FAIL" : "ok");
    return bad != 0;
}

int main(void) {
    static int16_t pcm[FRAMES * L];
    static Word16  syn[L];
    Word16         ana[23];
    int            fail = 0;

    if (!tetra_codec_available()) {
        printf("test_codec: wrapper built without the reference code\n");
        return 1;
    }
    make_signal(pcm, FRAMES * L);
    Init_Pre_Process();
    Init_Coder_Tetra();
    for (int f = 0; f < FRAMES; f++) {
        memcpy(new_speech, pcm + f * L, sizeof(int16_t) * L);
        Pre_Process(new_speech, (Word16)L);
        Coder_Tetra(ana, syn);
        Prm2bits_Tetra(ana, serial[f]);
    }
    printf("test_codec: wrapper against the reference decoder\n");
    fail |= compare("clean frames", FRAMES);

    for (int f = 0; f < FRAMES; f++) serial[f][0] = (f % 9 == 4) || (f % 100 >= 90 && f % 100 < 96);
    fail |= compare("bad frames (single and runs of 6)", FRAMES);

    for (int f = 0; f < FRAMES; f++) {
        serial[f][0] = 0;
        for (int i = 1; i < 138; i++) serial[f][i] = (rnd() >> 7) & 1;
    }
    fail |= compare("random bits", FRAMES);

    for (int f = 0; f < FRAMES; f++) {
        for (int i = 1; i < 138; i++) serial[f][i] = f & 1;
    }
    fail |= compare("all zeros / all ones", FRAMES);

    // Cost of a frame.
    {
        uint8_t bits[TETRA_CODEC_FRAME_BITS];
        int16_t out[L];
        struct timespec t0, t1;
        tetra_codec_reset();
        clock_gettime(CLOCK_MONOTONIC, &t0);
        for (int r = 0; r < 20; r++) {
            for (int f = 0; f < FRAMES; f++) {
                for (int i = 0; i < 137; i++) bits[i] = (uint8_t)((rnd() >> 7) & 1);
                tetra_codec_decode(bits, false, out);
            }
        }
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double us = ((t1.tv_sec - t0.tv_sec) * 1e9 + (t1.tv_nsec - t0.tv_nsec)) / (20.0 * FRAMES) / 1e3;
        printf("  decode: %.1f us per 30 ms frame on this host (%.2f %% of real time)\n", us, us / 300.0);
    }
    printf("test_codec: %s\n", fail ? "FAILED" : "all passed");
    return fail;
}
