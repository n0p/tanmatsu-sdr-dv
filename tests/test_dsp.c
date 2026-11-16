// SPDX-License-Identifier: AGPL-3.0-only
// Host check of the front-end decimation chain: gcc -O2 -Imain tests/test_dsp.c main/dsp.c -lm
#include <stdio.h>
#include <stdlib.h>
#include "dsp.h"

#define FS 1152000.0
#define N  (1152000 / 4)

static uint8_t raw[2 * N + 256];
static cf32_t  s1[N / 4 + 64], ch[N / 8 + 64];

// Feeds a tone at channel-relative offset f_hz, returns the measured (level dB, frequency Hz) at 48 kHz.
static void run(stage1_t* s, cfir_t* f, double f_hz, double amp, double* db, double* f_out, int decim2) {
    for (int i = 0; i < N; i++) {
        double ph      = 2.0 * M_PI * (FS / 4 + f_hz) * i / FS;
        raw[2 * i]     = (uint8_t)lrint(127.5 + 127.0 * amp * cos(ph));
        raw[2 * i + 1] = (uint8_t)lrint(127.5 + 127.0 * amp * sin(ph));
    }
    cfir_reset(f);
    int nout = 0, carry = 0, pos = 0;
    static uint8_t blk[2 * (8192 + 128)];
    while (pos < N) {  // block-wise, like the firmware, to exercise the history carry
        int take = N - pos < 8192 ? N - pos : 8192;
        for (int i = 0; i < 2 * take; i++) blk[2 * carry + i] = raw[2 * pos + i];
        pos += take;
        int consumed, total = carry + take;
        int n1 = stage1_run(s, blk, total, s1, &consumed);
        carry  = total - consumed;
        for (int i = 0; i < 2 * carry; i++) blk[i] = blk[2 * consumed + i];
        nout += cfir_run(f, s1, n1, ch + nout);
    }
    double   p = 0, dph = 0;
    fmdisc_t d = {ch[199]};
    for (int i = 200; i < nout; i++) {
        p   += ch[i].re * ch[i].re + ch[i].im * ch[i].im;
        dph += fmdisc_run(&d, ch[i]);
    }
    int m  = nout - 200;
    *db    = 10 * log10(p / m + 1e-30);
    *f_out = dph / m / (2 * M_PI) * (FS / s->decim / decim2);
}

int main(void) {
    stage1_t s;
    cfir_t   f48, f72;
    stage1_init(&s, 40, 8, 72000.0f / 1152000.0f, 60.0f);
    cfir_init(&f48, 30, 3, 8192);
    cfir_design(&f48, 20000.0f / 144000.0f, 60.0f);
    cfir_init(&f72, 44, 2, 8192);
    cfir_design(&f72, 19000.0f / 144000.0f, 60.0f);
    double offs[] = {0, 1000, -1000, 5000, -5000, 9000, 12500, -12500, 20000, 30000, 43000, 60000, 96000, -96000, 139000, 144000, -288000, 288000, 500000};
    int    fail   = 0;
    printf("%10s  %9s %9s   %9s %9s\n", "offset", "48k dB", "48k Hz", "72k dB", "72k Hz");
    for (unsigned i = 0; i < sizeof(offs) / sizeof(offs[0]); i++) {
        double db1, f1, db2, f2;
        run(&s, &f48, offs[i], 0.5, &db1, &f1, 3);
        run(&s, &f72, offs[i], 0.5, &db2, &f2, 2);
        printf("%10.0f  %9.2f %9.1f   %9.2f %9.1f\n", offs[i], db1, f1, db2, f2);
        double a = fabs(offs[i]);
        if (a <= 9000 && (fabs(db1 + 6.02) > 0.3 || fabs(f1 - offs[i]) > 5)) fail++;
        if (a <= 12500 && (fabs(db2 + 6.02) > 0.3 || fabs(f2 - offs[i]) > 5)) fail++;
        if (a >= 43000 && (db1 > -50 || db2 > -50)) fail++;
    }
    printf(fail ? "FAIL (%d)\n" : "PASS\n", fail);
    return fail != 0;
}
