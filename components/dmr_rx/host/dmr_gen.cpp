/*
 * dmr_gen - write a synthetic DMR discriminator recording (48 kS/s, mono,
 * int16 little endian raw or .wav) for dmr_rx_file and other decoders.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dmr_gen_lib.h"
#include "test_vectors.h"

using namespace dmrgen;

static void usage()
{
    fprintf(stderr,
            "usage: dmr_gen [options] -o out.wav|out.s16\n"
            "  --mode bs|ms|dm1|dm2   sync family (default bs: repeater downlink with CACH)\n"
            "  --cc N                 colour code (default 1)\n"
            "  --src ID --dst ID      addresses (default 2345678 -> 91)\n"
            "  --private              unit-to-unit call instead of a group call\n"
            "  --enc                  encrypted call (privacy bit, PI header)\n"
            "  --alias TEXT           talker alias\n"
            "  --superframes N        length of the call (default 20 = 7.2 s)\n"
            "  --snr dB               add noise (SNR in the 24 kHz discriminator bandwidth)\n"
            "  --dc X --gain X        DC offset (full scale = 1) and gain (default 1: rad/sample)\n"
            "  --invert               invert the polarity\n"
            "  --ppm X                sample clock error\n"
            "  --seed N               noise seed\n"
            "The scenario: idle / CSBK / call on timeslot 1, idle and a second call on timeslot 2 (bs),\n"
            "or one call (ms, dm1, dm2).\n");
}

int main(int argc, char **argv)
{
    Mode mode = MODE_BS;
    int cc    = 1;
    CallSpec call;
    call.src         = 2345678;
    call.superframes = 20;
    Channel ch;
    const char *out = nullptr;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        auto val      = [&]() -> const char * {
            if (i + 1 >= argc) {
                usage();
                exit(2);
            }
            return argv[++i];
        };
        if (!strcmp(a, "--mode")) {
            const char *m = val();
            mode          = !strcmp(m, "ms") ? MODE_MS : !strcmp(m, "dm1") ? MODE_DM1 : !strcmp(m, "dm2") ? MODE_DM2 : MODE_BS;
        } else if (!strcmp(a, "--cc")) {
            cc = atoi(val()) & 15;
        } else if (!strcmp(a, "--src")) {
            call.src = (uint32_t)atol(val());
        } else if (!strcmp(a, "--dst")) {
            call.dst = (uint32_t)atol(val());
        } else if (!strcmp(a, "--private")) {
            call.group = false;
        } else if (!strcmp(a, "--enc")) {
            call.encrypted = true;
        } else if (!strcmp(a, "--alias")) {
            call.alias = val();
        } else if (!strcmp(a, "--superframes")) {
            call.superframes = atoi(val());
        } else if (!strcmp(a, "--snr")) {
            ch.noise  = true;
            ch.snr_db = atof(val());
        } else if (!strcmp(a, "--dc")) {
            ch.dc = atof(val());
        } else if (!strcmp(a, "--gain")) {
            ch.gain = atof(val());
        } else if (!strcmp(a, "--invert")) {
            ch.invert = true;
        } else if (!strcmp(a, "--ppm")) {
            ch.ppm = atof(val());
        } else if (!strcmp(a, "--seed")) {
            ch.seed = (uint32_t)atol(val());
        } else if (!strcmp(a, "-o")) {
            out = val();
        } else {
            usage();
            return 2;
        }
    }
    if (!out) {
        usage();
        return 2;
    }

    SlotScript s1(mode, (uint8_t)cc), s2(mode, (uint8_t)cc);
    static const uint8_t short_lc[4] = {0x01, 0x80, 0x00, 0x86};
    if (mode == MODE_BS) {
        s1.idle(8);
        s1.csbk(kCsbkPdu, 2);
        s1.idle(2);
        s1.call(call);
        s1.idle(8);
        CallSpec c2;
        c2.src = 1001;
        c2.dst = 2002;
        c2.group = false;
        c2.superframes = call.superframes / 2;
        s2.idle(20);
        s2.call(c2);
        s2.idle((int)s1.bursts.size() - (int)s2.bursts.size());
    } else {
        s1.silence(4);
        s1.call(call);
        s1.silence(4);
    }
    const std::vector<float> x = modulate(mode, s1.bursts, s2.bursts, mode == MODE_BS ? short_lc : nullptr, ch);

    FILE *f = fopen(out, "wb");
    if (!f) {
        perror(out);
        return 1;
    }
    const size_t len = strlen(out);
    if (len > 4 && !strcmp(out + len - 4, ".wav")) {
        const uint32_t bytes = (uint32_t)(x.size() * 2);
        uint8_t h[44]        = {'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 16, 0, 0, 0, 1, 0, 1, 0,
                                0x80, 0xBB, 0, 0, 0x00, 0x77, 0x01, 0, 2, 0, 16, 0, 'd', 'a', 't', 'a', 0, 0, 0, 0};
        const uint32_t riff  = bytes + 36;
        memcpy(h + 4, &riff, 4);
        memcpy(h + 40, &bytes, 4);
        fwrite(h, 1, 44, f);
    }
    /* int16 full scale = pi rad/sample (the scaling of the Tanmatsu captures) */
    for (float v : x) {
        double d = (double)v / 3.14159265358979323846 * 32767.0;
        if (d > 32767.0)
            d = 32767.0;
        if (d < -32768.0)
            d = -32768.0;
        const int16_t q = (int16_t)lrint(d);
        fwrite(&q, 2, 1, f);
    }
    fclose(f);
    fprintf(stderr, "%s: %zu samples (%.2f s)\n", out, x.size(), (double)x.size() / 48000.0);
    return 0;
}
