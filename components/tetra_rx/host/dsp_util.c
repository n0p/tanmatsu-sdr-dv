/* host-side DSP helpers for the tetra_rx tests and CLI
 * SPDX-License-Identifier: AGPL-3.0-or-later */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "dsp_util.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void rng_seed(rng_t *r, uint64_t seed)
{
	r->s = seed ? seed : 0x9e3779b97f4a7c15ull;
	r->have_spare = 0;
}

uint32_t rng_u32(rng_t *r)
{
	r->s ^= r->s >> 12;
	r->s ^= r->s << 25;
	r->s ^= r->s >> 27;
	return (uint32_t) ((r->s * 0x2545f4914f6cdd1dull) >> 32);
}

double rng_uniform(rng_t *r)
{
	return rng_u32(r) / 4294967296.0;
}

double rng_gauss(rng_t *r)
{
	double u, v, s;

	if (r->have_spare) {
		r->have_spare = 0;
		return r->spare;
	}
	do {
		u = 2.0 * rng_uniform(r) - 1.0;
		v = 2.0 * rng_uniform(r) - 1.0;
		s = u * u + v * v;
	} while (s >= 1.0 || s == 0.0);
	s = sqrt(-2.0 * log(s) / s);
	r->spare = v * s;
	r->have_spare = 1;
	return u * s;
}

/* ---- modulator ------------------------------------------------------------- */

static double rrc(double t, double a)
{
	double d = 4.0 * a * t;

	if (fabs(t) < 1e-9)
		return 1.0 - a + 4.0 * a / M_PI;
	if (fabs(fabs(d) - 1.0) < 1e-9)
		return a / sqrt(2.0) * ((1.0 + 2.0 / M_PI) * sin(M_PI / (4.0 * a)) +
					(1.0 - 2.0 / M_PI) * cos(M_PI / (4.0 * a)));
	return (sin(M_PI * t * (1.0 - a)) + d * cos(M_PI * t * (1.0 + a))) / (M_PI * t * (1.0 - d * d));
}

dqpsk_mod_t *dqpsk_mod_new(int sps)
{
	dqpsk_mod_t *m = calloc(1, sizeof(*m));
	int n, k;
	double e = 0.0;

	m->sps = sps;
	m->span = 12;
	n = 2 * m->span * sps + 1;
	m->pulse = calloc((size_t) n, sizeof(double));
	m->hist_re = calloc((size_t) (2 * m->span + 1), sizeof(double));
	m->hist_im = calloc((size_t) (2 * m->span + 1), sizeof(double));
	for (k = 0; k < n; k++) {
		m->pulse[k] = rrc((double) (k - m->span * sps) / sps, 0.35);
		e += m->pulse[k] * m->pulse[k];
	}
	/* unit average power: sum(g^2) = sps */
	e = sqrt((double) sps / e);
	for (k = 0; k < n; k++)
		m->pulse[k] *= e;
	return m;
}

void dqpsk_mod_free(dqpsk_mod_t *m)
{
	if (!m)
		return;
	free(m->pulse);
	free(m->hist_re);
	free(m->hist_im);
	free(m);
}

void dqpsk_mod(dqpsk_mod_t *m, const uint8_t *bits, size_t n_bits, float *out)
{
	/* EN 300 392-2 clause 5.5.2.3 */
	static const int dphase[2][2] = { { 1, 3 }, { -1, -3 } };
	int nh = 2 * m->span + 1;
	size_t i;
	int j, k;

	for (i = 0; i + 1 < n_bits; i += 2) {
		m->phase = (m->phase + dphase[bits[i] & 1][bits[i + 1] & 1]) & 7;
		memmove(m->hist_re, m->hist_re + 1, (size_t) (nh - 1) * sizeof(double));
		memmove(m->hist_im, m->hist_im + 1, (size_t) (nh - 1) * sizeof(double));
		m->hist_re[nh - 1] = cos(m->phase * M_PI / 4.0);
		m->hist_im[nh - 1] = sin(m->phase * M_PI / 4.0);
		/* output the sps samples centred span symbols ago */
		for (j = 0; j < m->sps; j++) {
			double re = 0.0, im = 0.0;

			/* sample time t = (centre symbol) + j/sps; symbol h (0..nh-1)
			 * sits at time h; centre = span. Pulse index of symbol h:
			 * (t - h) * sps + span*sps */
			for (k = 0; k < nh; k++) {
				int idx = (m->span - k) * m->sps + j + m->span * m->sps;

				if (idx < 0 || idx > 2 * m->span * m->sps)
					continue;
				re += m->hist_re[k] * m->pulse[idx];
				im += m->hist_im[k] * m->pulse[idx];
			}
			*out++ = (float) re;
			*out++ = (float) im;
		}
	}
}

/* ---- resampler ------------------------------------------------------------- */

resamp_t *resamp_new(double in_rate, double out_rate, double bw_hz)
{
	resamp_t *r = calloc(1, sizeof(*r));
	double lim = 0.5 * (in_rate < out_rate ? in_rate : out_rate);
	double trans;
	int p, k;

	/* pass band bw_hz, stop band at the lower Nyquist frequency */
	if (bw_hz > 0.85 * lim)
		bw_hz = 0.85 * lim;
	r->ratio = in_rate / out_rate;
	r->fc = 0.5 * (bw_hz + lim) / in_rate;
	trans = (lim - bw_hz) / in_rate;
	r->half = (int) ceil(3.3 / trans / 2.0) + 2;    /* Blackman, ~-74 dB */
	r->nphase = 512;
	r->table = malloc(sizeof(float) * (size_t) r->nphase * (size_t) (2 * r->half));
	for (p = 0; p < r->nphase; p++) {
		double frac = (double) p / r->nphase;

		for (k = 0; k < 2 * r->half; k++) {
			/* tap k weights input sample (base - half + 1 + k), output at base + frac */
			double x = frac - (double) (k - r->half + 1);
			double w = x / r->half;
			double win = fabs(w) < 1.0 ? 0.42 + 0.5 * cos(M_PI * w) + 0.08 * cos(2.0 * M_PI * w) : 0.0;
			double s = fabs(x) < 1e-12 ? 1.0 : sin(2.0 * M_PI * r->fc * x) / (2.0 * M_PI * r->fc * x);

			r->table[p * 2 * r->half + k] = (float) (2.0 * r->fc * s * win);
		}
	}
	r->buf_cap = 0;
	r->buf_n = (size_t) (2 * r->half);
	r->buf = NULL;
	r->pos = r->half;
	return r;
}

void resamp_free(resamp_t *r)
{
	if (!r)
		return;
	free(r->table);
	free(r->buf);
	free(r);
}

size_t resamp_process(resamp_t *r, const float *in, size_t n, float *out)
{
	size_t need = r->buf_n + n, n_out = 0, keep, drop;

	if (need > r->buf_cap) {
		size_t old = r->buf_cap;

		r->buf_cap = need + 4096;
		r->buf = realloc(r->buf, r->buf_cap * 2 * sizeof(float));
		if (old == 0)
			memset(r->buf, 0, r->buf_n * 2 * sizeof(float));
	}
	memcpy(r->buf + 2 * r->buf_n, in, n * 2 * sizeof(float));
	r->buf_n += n;

	while (r->pos + r->half < (double) r->buf_n) {
		size_t base = (size_t) r->pos;
		double frac = r->pos - (double) base;
		int p = (int) (frac * r->nphase);
		const float *t = r->table + (size_t) p * 2 * (size_t) r->half;
		const float *x = r->buf + 2 * (base - (size_t) r->half + 1);
		float re = 0.0f, im = 0.0f;
		int k;

		for (k = 0; k < 2 * r->half; k++) {
			re += t[k] * x[2 * k];
			im += t[k] * x[2 * k + 1];
		}
		out[2 * n_out] = re;
		out[2 * n_out + 1] = im;
		n_out++;
		r->pos += r->ratio;
	}
	/* keep the history the next output needs */
	keep = (size_t) (2 * r->half) + 2;
	if (r->buf_n > keep && (size_t) r->pos > (size_t) r->half + 1) {
		drop = (size_t) r->pos - (size_t) r->half - 1;
		if (drop > r->buf_n)
			drop = r->buf_n;
		memmove(r->buf, r->buf + 2 * drop, (r->buf_n - drop) * 2 * sizeof(float));
		r->buf_n -= drop;
		r->pos -= (double) drop;
	}
	return n_out;
}

void freq_shift(float *iq, size_t n, double hz, double rate, double *phase)
{
	double w = 2.0 * M_PI * hz / rate, ph = *phase;
	size_t i;

	for (i = 0; i < n; i++) {
		double c = cos(ph), s = sin(ph);
		double re = iq[2 * i], im = iq[2 * i + 1];

		iq[2 * i] = (float) (re * c - im * s);
		iq[2 * i + 1] = (float) (re * s + im * c);
		ph += w;
		if (ph > M_PI)
			ph -= 2.0 * M_PI;
		else if (ph < -M_PI)
			ph += 2.0 * M_PI;
	}
	*phase = ph;
}

void add_awgn(rng_t *r, float *iq, size_t n, double noise_power)
{
	double s = sqrt(noise_power / 2.0);
	size_t i;

	for (i = 0; i < 2 * n; i++)
		iq[i] += (float) (s * rng_gauss(r));
}
