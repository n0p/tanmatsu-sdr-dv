/* tetra_rx - CPU cost and state size on the host
 *
 * Reports the cost of tetra_rx_process_cf32() per input sample while locked
 * to a cell and while searching, and the cost of the channel decoding and PDU
 * parsing of one burst on its own.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "tetra_rx.h"
#include "tetra_ctx.h"
#include "tetra_gen.h"
#include "dsp_util.h"

#define SPS 4
#define FS 72000.0
#define SLOT_SAMPLES (255 * SPS)
#define N_SLOTS 2400            /* 34 s of signal */

static unsigned n_events[16];

static void on_event(void *user, const tetra_rx_event_t *ev)
{
	(void) user;
	if (ev->type < 16)
		n_events[ev->type]++;
}

static double now(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
	return (double) ts.tv_sec + 1e-9 * (double) ts.tv_nsec;
}

static tetra_rx_t *rx_new(uint32_t flags)
{
	tetra_rx_config_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.event_cb = on_event;
	cfg.flags = flags;
	return tetra_rx_create(&cfg);
}

/* best of several runs, in ns per complex input sample */
static double run(const float *iq, size_t n, size_t block, uint32_t flags, tetra_rx_status_t *st)
{
	double best = 1e30;
	int rep;

	for (rep = 0; rep < 5; rep++) {
		tetra_rx_t *rx = rx_new(flags);
		double t0 = now(), t;
		size_t i;

		for (i = 0; i < n; i += block)
			tetra_rx_process_cf32(rx, iq + 2 * i, n - i < block ? n - i : block);
		t = now() - t0;
		if (t < best)
			best = t;
		if (st)
			tetra_rx_get_status(rx, st);
		tetra_rx_destroy(rx);
	}
	return 1e9 * best / (double) n;
}

static void report(const char *what, double ns)
{
	printf("  %-46s %7.1f ns/sample  %6.2f ms per second of signal  (%.0f x realtime)\n", what, ns,
	       ns * FS * 1e-6, 1e9 / (ns * FS));
}

int main(void)
{
	size_t n = (size_t) N_SLOTS * SLOT_SAMPLES, i;
	float *iq = malloc(sizeof(float) * 2 * n), *noise = malloc(sizeof(float) * 2 * n);
	int8_t (*soft)[510] = malloc((size_t) N_SLOTS * 510);
	uint8_t *type = malloc(N_SLOTS);
	tetra_gen_cfg_t gc;
	tetra_gen_t *gen;
	dqpsk_mod_t *mod = dqpsk_mod_new(SPS);
	tetra_rx_status_t st;
	tetra_rx_t *rx;
	double ph = 0.0, t0, t, best;
	rng_t rng;
	unsigned k, rep;

	if (!iq || !noise || !soft || !type || !mod)
		return 2;
	/* a busy cell: MCCH with fragmented signalling on TN1, a speech call
	 * on TN2, Eb/N0 15 dB, 300 Hz carrier offset */
	tetra_gen_default_cfg(&gc);
	gc.traffic_tn = 2;
	gen = tetra_gen_new(&gc);
	rng_seed(&rng, 1);
	for (k = 0; k < N_SLOTS; k++) {
		const tetra_gen_slot_t *s = tetra_gen_next(gen);
		unsigned b;

		dqpsk_mod(mod, s->bits, 510, iq + 2 * (size_t) k * SLOT_SAMPLES);
		for (b = 0; b < 510; b++)
			soft[k][b] = s->bits[b] ? -64 : 64;
		type[k] = s->burst == 2 ? TETRA_TRAIN_SYNC : s->burst == 1 ? TETRA_TRAIN_NORM_2 : TETRA_TRAIN_NORM_1;
	}
	freq_shift(iq, n, 300.0, FS, &ph);
	add_awgn(&rng, iq, n, SPS / (2.0 * pow(10.0, 1.5)));
	for (i = 0; i < 2 * n; i++) {
		iq[i] *= 0.25f;
		noise[i] = (float) (0.05 * rng_gauss(&rng));
	}

	printf("tetra_rx bench (%s)\n", tetra_rx_version());
	printf("input: %u slots = %.1f s at 72 kHz\n", (unsigned) N_SLOTS, (double) n / FS);
	printf("locked to a cell (TN1 control channel, TN2 speech):\n");
	report("blocks of 1024 samples", run(iq, n, 1024, 0, &st));
	printf("    slots %u, BSCH %u/%u, AACH %u/%u, SCH/F %u/%u, TCH/S %u/%u, sync losses %u\n",
	       (unsigned) st.slots, (unsigned) st.chan[TETRA_RX_CH_BSCH].ok, (unsigned) st.chan[TETRA_RX_CH_BSCH].fail,
	       (unsigned) st.chan[TETRA_RX_CH_AACH].ok, (unsigned) st.chan[TETRA_RX_CH_AACH].fail,
	       (unsigned) st.chan[TETRA_RX_CH_SCH_F].ok, (unsigned) st.chan[TETRA_RX_CH_SCH_F].fail,
	       (unsigned) st.chan[TETRA_RX_CH_TCH_S].ok, (unsigned) st.chan[TETRA_RX_CH_TCH_S].fail,
	       (unsigned) st.sync_losses);
	report("blocks of 256 samples", run(iq, n, 256, 0, NULL));
	report("blocks of 4096 samples", run(iq, n, 4096, 0, NULL));
	report("one sample per call", run(iq, n, 1, 0, NULL));
	report("blocks of 1024, TETRA_RX_FLAG_NO_TRAFFIC", run(iq, n, 1024, TETRA_RX_FLAG_NO_TRAFFIC, NULL));
	printf("searching (noise only):\n");
	report("blocks of 1024 samples", run(noise, n, 1024, 0, &st));
	printf("    slots %u, lock acquisitions %u\n", (unsigned) st.slots, (unsigned) st.lock_count);

	/* channel decoding + MAC / LLC / MLE / CMCE parsing only: the bursts
	 * of the same signal as error-free soft bits */
	best = 1e30;
	for (rep = 0; rep < 5; rep++) {
		rx = rx_new(0);
		tetra_lmac_set_cell(rx, gc.mcc, gc.mnc, gc.colour_code);
		rx->time.tn = gc.start_tn;
		rx->time.fn = gc.start_fn;
		rx->time.mn = gc.start_mn;
		rx->time_valid = 1;
		memset(n_events, 0, sizeof(n_events));
		t0 = now();
		for (k = 0; k < N_SLOTS; k++)
			tetra_lmac_slot(rx, soft[k], type[k]);
		t = now() - t0;
		if (t < best)
			best = t;
		tetra_rx_get_status(rx, &st);
		tetra_rx_destroy(rx);
	}
	printf("channel decoding and PDU parsing alone (%u bursts: %u signalling PDUs, %u TCH/S slots):\n",
	       (unsigned) N_SLOTS, n_events[TETRA_RX_EV_SIGNAL], n_events[TETRA_RX_EV_TRAFFIC]);
	printf("  %.2f us per burst = %.1f ns per input sample (%.0f bursts per second of CPU)\n",
	       1e6 * best / N_SLOTS, 1e9 * best / (double) n, N_SLOTS / best);

	printf("state: sizeof(struct tetra_rx) = %u bytes, one allocation in tetra_rx_create(), none later\n",
	       (unsigned) sizeof(struct tetra_rx));
	free(iq);
	free(noise);
	free(soft);
	free(type);
	dqpsk_mod_free(mod);
	tetra_gen_free(gen);
	return 0;
}
