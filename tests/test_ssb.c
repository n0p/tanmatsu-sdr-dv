// SPDX-License-Identifier: AGPL-3.0-only
// Host check of the SSB demodulator: gcc -O2 -Imain tests/test_ssb.c main/dsp.c -lm
#include <stdio.h>
#include <stdlib.h>
#include "dsp.h"

#define FS  48000.0
#define N   48000
#define BLK 341  // what the firmware feeds per USB block

static cf32_t in[N];
static float  out[N + 64];

// A tone rf_hz away from 0 Hz in the input, carrier at carrier_hz. Returns the audio level in dB
// relative to the input tone and the audio frequency.
static void run(ssb_t* s, int sideband, double carrier_hz, double rf_hz, double amp, double* db, double* f_out,
                int* n_out) {
    for (int i = 0; i < N; i++) {
        double ph = 2.0 * M_PI * rf_hz * i / FS;
        in[i].re  = (float)(amp * cos(ph));
        in[i].im  = (float)(amp * sin(ph));
    }
    ssb_reset(s);
    ssb_tune(s, sideband, (float)carrier_hz);
    int no = 0;
    for (int pos = 0; pos < N; pos += BLK) {
        int take = N - pos < BLK ? N - pos : BLK;
        no      += ssb_run(s, in + pos, take, out + no);
    }
    // Skip the filter transients, then measure power and count rising zero crossings.
    int    skip = 4800, cross = 0;
    double p    = 0;
    for (int i = skip; i < no; i++) {
        p += (double)out[i] * out[i];
        if (out[i - 1] < 0 && out[i] >= 0) cross++;
    }
    int m  = no - skip;
    *db    = 10 * log10(2 * p / m / (amp * amp) + 1e-30);  // a real tone of the same amplitude reads 0 dB
    *f_out = cross * FS / m;
    *n_out = no;
}

int main(void) {
    ssb_t s;
    if (!ssb_init(&s, 512)) return 2;
    s.agc    = false;
    int fail = 0;

    static const double audio[] = {70, 150, 300, 500, 1000, 1500, 2000, 2700, 2850, 3200, 4000};
    static const double carr[]  = {0, 6000, -4321};
    printf("%8s %6s %7s   %9s %9s   %9s\n", "carrier", "side", "audio", "wanted dB", "Hz", "other dB");
    for (unsigned c = 0; c < sizeof(carr) / sizeof(carr[0]); c++) {
        for (int sb = 1; sb >= -1; sb -= 2) {
            for (unsigned i = 0; i < sizeof(audio) / sizeof(audio[0]); i++) {
                double db, f, db_o, f_o;
                int    no;
                run(&s, sb, carr[c], carr[c] + sb * audio[i], 0.5, &db, &f, &no);
                run(&s, sb, carr[c], carr[c] - sb * audio[i], 0.5, &db_o, &f_o, &no);
                printf("%8.0f %6s %7.0f   %9.2f %9.1f   %9.2f\n", carr[c], sb > 0 ? "USB" : "LSB", audio[i], db, f,
                       db_o);
                if (no != N) fail++;                                                            // rate must be kept
                if (audio[i] >= 300 && audio[i] <= 2700 && (fabs(db) > 0.5 || fabs(f - audio[i]) > 2)) fail++;
                if (audio[i] >= 70 && db_o > -55) fail++;                                       // opposite sideband
                if (audio[i] >= 3200 && db > -55) fail++;                                       // adjacent channel
            }
        }
    }

    // AGC: a weak and a strong tone must come out at the same level, without clipping.
    s.agc = true;
    for (double amp = 1e-4; amp < 1.0; amp *= 30) {
        double db, f;
        int    no;
        run(&s, 1, 0, 1000, amp, &db, &f, &no);
        double peak = 0;
        for (int i = 24000; i < no; i++) peak = fabs(out[i]) > peak ? fabs(out[i]) : peak;
        printf("AGC  in %8.5f  ->  out peak %.3f\n", amp, peak);
        if (peak < 0.25 || peak > 0.45) fail++;
    }
    printf(fail ? "FAIL (%d)\n" : "PASS\n", fail);
    return fail != 0;
}
