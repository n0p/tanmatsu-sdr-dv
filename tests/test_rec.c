// SPDX-License-Identifier: AGPL-3.0-only
// Host check of the recording formats. Writes one file per format into the directory given as the
// argument, plus a copy of each as it stood on disk after a checkpoint (what a power cut leaves):
//
//   gcc -O2 -Imain -Icomponents/opus/libopus/include -o /tmp/test_rec tests/test_rec.c main/recfmt.c libopus.a -lm
//   /tmp/test_rec /tmp/rec_out
//
// libopus.a is the vendored encoder built for the PC (same sources and defines as the firmware, see
// components/opus/CMakeLists.txt). The files are then checked with opusinfo / opusdec / ffmpeg:
// the test prints the length and tone each one has to decode to.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "recfmt.h"

#define SECONDS 7.321  // not a whole number of frames or blocks
#define TONE_HZ 1000.0
#define AMP     8000.0

static void copy_file(const char* from, const char* to) {
    FILE*  a = fopen(from, "rb");
    FILE*  b = fopen(to, "wb");
    char   buf[4096];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), a)) > 0) fwrite(buf, 1, n, b);
    fclose(a);
    fclose(b);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s <output directory>\n", argv[0]);
        return 2;
    }
    int             total = (int)(SECONDS * REC_IN_RATE);
    int16_t*        pcm   = malloc(sizeof(int16_t) * (size_t)total);
    for (int i = 0; i < total; i++) pcm[i] = (int16_t)lrint(AMP * sin(2 * M_PI * TONE_HZ * i / REC_IN_RATE));
    const char* tags[] = {"TITLE=test tone", "DESCRIPTION=1 kHz"};
    int         fail   = 0;
    srand(1);
    for (int fmt = 0; fmt < REC_FMT_COUNT; fmt++) {
        char path[512], cut[512];
        snprintf(path, sizeof(path), "%s/fmt%d.%s", argv[1], fmt, rec_format_ext(fmt));
        snprintf(cut, sizeof(cut), "%s/fmt%d-cut.%s", argv[1], fmt, rec_format_ext(fmt));
        FILE*         f = fopen(path, "wb");
        rec_writer_t* w = f ? recw_open(f, fmt, tags, 2, 0x1234u + (unsigned)fmt) : NULL;
        if (!w) {
            printf("%s: cannot open\n", path);
            return 1;
        }
        int  pos = 0, cut_at = 0;
        bool ok  = true;
        while (pos < total) {  // uneven pieces, like the audio arrives
            int n = 1 + rand() % 3000;
            if (n > total - pos) n = total - pos;
            ok   = recw_write(w, pcm + pos, n) && ok;
            pos += n;
            if (!cut_at && pos > total / 2) {
                ok     = recw_checkpoint(w) && ok;
                cut_at = pos;
                copy_file(path, cut);
            }
        }
        uint32_t ms = recw_ms(w);
        ok          = recw_close(w) && ok;
        long size   = ftell(f);
        fclose(f);
        printf("%-10s %-16s %s  %u ms  %ld bytes  %.1f kbit/s  = %.1f MB/hour   (checkpoint at %.3f s)\n", rec_format_name(fmt),
               path, ok ? "ok" : "WRITE FAILED", (unsigned)ms, size, size * 8.0 / SECONDS / 1000.0, size / SECONDS * 3600 / 1e6,
               (double)cut_at / REC_IN_RATE);
        if (!ok || ms != (uint32_t)(SECONDS * 1000)) fail = 1;
    }
    printf("expect %d samples at 48 kHz (%.3f s), %.0f Hz, peak %.0f\n", total, SECONDS, TONE_HZ, AMP);
    return fail;
}
