/* host-side DSP helpers for the tetra_rx tests and CLI (not part of the
 * library; double precision is fine here)
 * SPDX-License-Identifier: AGPL-3.0-or-later */
#ifndef DSP_UTIL_H
#define DSP_UTIL_H

#include <stddef.h>
#include <stdint.h>

/* xorshift64* generator, deterministic across platforms */
typedef struct {
	uint64_t s;
	int have_spare;
	double spare;
} rng_t;
void rng_seed(rng_t *r, uint64_t seed);
uint32_t rng_u32(rng_t *r);
double rng_uniform(rng_t *r);   /* [0,1) */
double rng_gauss(rng_t *r);     /* N(0,1) */

/* pi/4-DQPSK modulator: bits (one per byte, 2 per symbol) to complex baseband
 * with root raised cosine alpha 0.35 pulse shaping at 'sps' samples/symbol.
 * Unit average power. Streaming: keeps phase and filter state. */
typedef struct {
	int sps;
	int span;           /* pulse half length in symbols */
	double *pulse;      /* (2*span*sps+1) taps */
	double *hist_re;    /* last 2*span+1 symbols */
	double *hist_im;
	int phase;          /* in units of pi/4 */
} dqpsk_mod_t;
dqpsk_mod_t *dqpsk_mod_new(int sps);
void dqpsk_mod_free(dqpsk_mod_t *m);
/* n_bits must be even; writes n_bits/2*sps complex samples (interleaved) */
void dqpsk_mod(dqpsk_mod_t *m, const uint8_t *bits, size_t n_bits, float *out);

/* Arbitrary ratio resampler (windowed sinc). */
typedef struct {
	double ratio;       /* in_rate / out_rate */
	double fc;          /* cutoff in cycles per input sample */
	int half;           /* kernel half width in input samples */
	int nphase;
	float *table;       /* nphase x (2*half) */
	float *buf;         /* interleaved history of 2*half samples + block */
	size_t buf_n;       /* complex samples in buf */
	size_t buf_cap;
	double pos;         /* position of the next output sample in buf */
} resamp_t;
resamp_t *resamp_new(double in_rate, double out_rate, double bw_hz);
void resamp_free(resamp_t *r);
/* returns the number of complex output samples written (out must hold at
 * least n / ratio + 2) */
size_t resamp_process(resamp_t *r, const float *in, size_t n, float *out);

/* frequency shift in place (hz may be negative); *phase is kept across calls */
void freq_shift(float *iq, size_t n, double hz, double rate, double *phase);
/* add white Gaussian noise of the given power per complex sample */
void add_awgn(rng_t *r, float *iq, size_t n, double noise_power);

#endif
