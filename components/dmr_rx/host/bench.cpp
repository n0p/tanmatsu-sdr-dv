/*
 * dmr_rx host tools - CPU cost of the receiver.
 *
 * Feeds synthetic 48 kS/s discriminator signals in blocks of 960 samples
 * (20 ms) and reports ns per input sample, the realtime factor on this
 * machine and what that means for a core that is P4_FACTOR times slower
 * (default 15, the rule of thumb for the ESP32-P4 against a desktop core).
 * With the vocoder built in, its cost is measured separately as the
 * difference between runs with and without a PCM callback.
 *
 * usage: bench [seconds of signal (default 60)] [P4 factor (default 15)]
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>

#include "dmr_gen_lib.h"
#include "dmr_rx.h"

using namespace dmrgen;

static double now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

struct Counts {
    unsigned long events = 0, voice = 0, data = 0, pcm = 0;
    long pcm_sum = 0;
};

static void on_event(void *user, const dmr_event_t *ev)
{
    Counts *c = (Counts *)user;
    c->events++;
    c->voice += ev->type == DMR_EVT_VOICE_BURST;
    c->data += ev->type == DMR_EVT_DATA_BURST;
}

static void on_pcm(void *user, uint8_t, const int16_t *pcm, size_t n)
{
    Counts *c = (Counts *)user;
    c->pcm++;
    for (size_t i = 0; i < n; i++)
        c->pcm_sum += pcm[i];
}

struct Result {
    double ns_per_sample;
    Counts counts;
};

static Result measure(const std::vector<float> &x, bool with_pcm)
{
    Result best;
    best.ns_per_sample = 1e30;
    for (int rep = 0; rep < 5; rep++) {
        Counts c;
        dmr_rx_config_t cfg;
        dmr_rx_config_default(&cfg);
        cfg.event_cb   = on_event;
        cfg.event_user = &c;
        if (with_pcm) {
            cfg.pcm_cb   = on_pcm;
            cfg.pcm_user = &c;
        }
        dmr_rx_t *rx = dmr_rx_create(&cfg);
        if (!rx) {
            fprintf(stderr, "dmr_rx_create failed\n");
            exit(1);
        }
        const double t0 = now_ns();
        for (size_t pos = 0; pos < x.size(); pos += 960)
            dmr_rx_process_fm(rx, x.data() + pos, x.size() - pos < 960 ? x.size() - pos : 960);
        const double t1 = now_ns();
        dmr_rx_destroy(rx);
        const double ns = (t1 - t0) / (double)x.size();
        if (ns < best.ns_per_sample) {
            best.ns_per_sample = ns;
            best.counts        = c;
        }
    }
    return best;
}

static double g_factor = 15.0;

static void report(const char *name, const Result &r)
{
    const double rt = 1e9 / (r.ns_per_sample * 48000.0);
    printf("%-44s %7.1f ns/sample  %6.0fx realtime  -> %5.2f %% of a core %gx slower   (%lu data, %lu voice bursts)\n",
           name, r.ns_per_sample, rt, 100.0 * g_factor / rt, g_factor, r.counts.data, r.counts.voice);
}

int main(int argc, char **argv)
{
    const int seconds = argc > 1 ? atoi(argv[1]) : 60;
    if (argc > 2)
        g_factor = atof(argv[2]);
    const int periods = seconds * 1000 / 60;

    Channel ch;
    ch.noise  = true;
    ch.snr_db = 15;
    ch.ppm    = 20;
    static const uint8_t kShortLc[4] = {0x01, 0x80, 0x00, 0x86};

    /* repeater: a call on TS1, TS2 idle */
    SlotScript a1(MODE_BS, 1), a2(MODE_BS, 1);
    {
        CallSpec c;
        c.superframes = (periods - 3) / 6;
        c.alias       = "BENCH";
        a1.call(c);
        a2.idle(periods);
    }
    const std::vector<float> one_call = modulate(MODE_BS, a1.bursts, a2.bursts, kShortLc, ch);

    /* repeater: calls on both timeslots */
    SlotScript b1(MODE_BS, 1), b2(MODE_BS, 1);
    {
        CallSpec c;
        c.superframes = (periods - 3) / 6;
        b1.call(c);
        c.src         = 1001;
        c.dst         = 2002;
        c.group       = false;
        c.first_frame = 11;
        b2.call(c);
    }
    const std::vector<float> two_calls = modulate(MODE_BS, b1.bursts, b2.bursts, kShortLc, ch);

    /* repeater idling on both timeslots */
    SlotScript c1(MODE_BS, 1), c2(MODE_BS, 1);
    c1.idle(periods);
    c2.idle(periods);
    const std::vector<float> idle = modulate(MODE_BS, c1.bursts, c2.bursts, kShortLc, ch);

    /* no carrier: discriminator noise */
    SlotScript n1(MODE_BS, 1), n2(MODE_BS, 1);
    n1.silence(periods);
    n2.silence(periods);
    const std::vector<float> noise = modulate(MODE_BS, n1.bursts, n2.bursts, nullptr, ch);

    printf("dmr_rx bench: %d s of signal per case, blocks of 960 samples, best of 5 runs; state %zu bytes, vocoder %s\n",
           seconds, dmr_rx_state_size(), dmr_rx_has_voice() ? "built in" : "not built");
    printf("signalling only (no PCM callback):\n");
    const Result r_noise = measure(noise, false);
    const Result r_idle  = measure(idle, false);
    const Result r_one   = measure(one_call, false);
    const Result r_two   = measure(two_calls, false);
    report("  no signal (discriminator noise)", r_noise);
    report("  repeater, both timeslots idle", r_idle);
    report("  repeater, voice on TS1, TS2 idle", r_one);
    report("  repeater, voice on both timeslots", r_two);

    if (dmr_rx_has_voice()) {
        printf("with the AMBE+2 decoder (PCM callback set):\n");
        const Result v_one = measure(one_call, true);
        const Result v_two = measure(two_calls, true);
        report("  repeater, voice on TS1, TS2 idle", v_one);
        report("  repeater, voice on both timeslots", v_two);
        const double frames1 = (double)v_one.counts.pcm;
        const double frames2 = (double)v_two.counts.pcm;
        if (frames1 > 0 && frames2 > 0) {
            const double per1 = (v_one.ns_per_sample - r_one.ns_per_sample) * (double)one_call.size() / frames1;
            const double per2 = (v_two.ns_per_sample - r_two.ns_per_sample) * (double)two_calls.size() / frames2;
            printf("  vocoder alone: %.0f ns per 20 ms frame (one call), %.0f ns (two calls) -> %.2f %% of a core %gx "
                   "slower per call\n",
                   per1, per2, 100.0 * per1 * g_factor / 20e6, g_factor);
        }
    }
    return 0;
}
