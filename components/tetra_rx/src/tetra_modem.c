/* tetra_rx - pi/4-DQPSK modem for the TETRA continuous downlink
 *
 * 72 kHz complex baseband in, one slot of 510 soft bits out.
 *
 *   NCO (carrier offset) -> RRC matched filter -> square-law (Oerder & Meyr)
 *   symbol timing estimate -> cubic interpolator -> differential detector
 *   -> burst synchroniser correlating against the training sequences
 *   -> per-slot feed-forward frequency correction -> soft bits
 *
 * The chain is scale invariant (no sample-domain AGC); the soft bits are
 * normalised per slot. Everything is single precision.
 *
 * Original work for the tetra_rx component; this replaces osmo-tetra's
 * external GNU Radio demodulator and its hard-bit tetra_burst_sync.c.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "tetra_ctx.h"

#define TM_PI          3.14159265358979f
#define TM_TWO_PI      6.28318530717959f
#define TM_FS          72000.0f
#define TM_FSYM        18000.0f

#define TM_TIMING_BETA 0.002f      /* timing tone IIR, per sample */
#define TM_TIMING_GAIN 0.05f       /* strobe loop gain, per symbol */

#define TM_TH_ACQ      0.60f       /* tentative lock */
#define TM_TH_CONFIRM  0.50f       /* confirmation on the following slot */
#define TM_TH_BAD      0.50f       /* below: slot counts towards sync loss */
#define TM_TH_DECODE   0.45f       /* below: slot is not decoded */
#define TM_SHIFT_MARGIN 0.15f
#define TM_BAD_SLOTS   8       /* leaky count of slots below TM_TH_BAD */
#define TM_CONFIRM_SLOTS 3     /* consecutive good slots needed to declare lock */
#define TM_IDLE_SLOTS  288     /* 4 multiframes without any good CRC-16: false lock */
#define TM_FREQ_GAIN   0.2f
#define TM_FREQ_MAX    8000.0f
#define TM_FREQ_GAIN_TRACK 0.05f
#define TM_FREQ_SLEW    30.0f   /* Hz per slot while tracking */
#define TM_SETTLE_HZ   200.0f
#define TM_FLY_SYMS    36000u      /* 2 s */
#define TM_SOFT_NOM    40.0f

/* ---- initialisation -------------------------------------------------------- */

static float bessel_i0(float x)
{
	float sum = 1.0f, term = 1.0f, q = x * x * 0.25f;
	int k;

	for (k = 1; k < 24; k++) {
		term *= q / (float) (k * k);
		sum += term;
	}
	return sum;
}

/* root raised cosine, t in symbols */
static float rrc(float t, float a)
{
	float d = 4.0f * a * t;

	if (fabsf(t) < 1e-6f)
		return 1.0f - a + 4.0f * a / TM_PI;
	if (fabsf(fabsf(d) - 1.0f) < 1e-4f)
		return a * 0.70710678f * ((1.0f + 2.0f / TM_PI) * sinf(TM_PI / (4.0f * a)) +
					  (1.0f - 2.0f / TM_PI) * cosf(TM_PI / (4.0f * a)));
	return (sinf(TM_PI * t * (1.0f - a)) + d * cosf(TM_PI * t * (1.0f + a))) /
	       (TM_PI * t * (1.0f - d * d));
}

static void set_ref(float ref[][2], uint8_t *copy, enum tetra_train_seq type, unsigned int nsym)
{
	unsigned int len, i;
	const uint8_t *b = tetra_burst_train_seq_bits(type, &len);

	for (i = 0; i < nsym && 2 * i + 1 < len; i++) {
		/* EN 300 392-2 clause 5.5.2.3: (0,0) +pi/4, (0,1) +3pi/4,
		 * (1,1) -3pi/4, (1,0) -pi/4 */
		ref[i][0] = b[2 * i + 1] ? -1.0f : 1.0f;
		ref[i][1] = b[2 * i] ? -1.0f : 1.0f;
		copy[2 * i] = b[2 * i];
		copy[2 * i + 1] = b[2 * i + 1];
	}
}

static void set_nco(struct tetra_modem *m, float hz)
{
	float w;

	if (hz > TM_FREQ_MAX)
		hz = TM_FREQ_MAX;
	if (hz < -TM_FREQ_MAX)
		hz = -TM_FREQ_MAX;
	m->freq_hz = hz;
	w = -TM_TWO_PI * hz / TM_FS;
	m->nco_dc = cosf(w);
	m->nco_ds = sinf(w);
}

void tetra_modem_reset(struct tetra_rx *rx)
{
	struct tetra_modem *m = &rx->m;

	m->dc_i = m->dc_q = 0.0f;
	m->nco_c = 1.0f;
	m->nco_s = 0.0f;
	m->nco_cnt = 0;
	set_nco(m, 0.0f);
	m->freq_saved = 0.0f;
	m->freq_est = 0.0f;
	m->freq_good = 0.0f;
	memset(m->mf_i, 0, sizeof(m->mf_i));
	memset(m->mf_q, 0, sizeof(m->mf_q));
	m->mf_pos = 0;
	memset(m->yi, 0, sizeof(m->yi));
	memset(m->yq, 0, sizeof(m->yq));
	m->tc_re = m->tc_im = 0.0f;
	m->phase4 = 0;
	m->cnt = 2.0f;
	m->tau = 0.0f;
	m->adj_acc = 0.0f;
	m->prev_i = m->prev_q = 0.0f;
	memset(m->dr, 0, sizeof(m->dr));
	memset(m->di, 0, sizeof(m->di));
	memset(m->dm, 0, sizeof(m->dm));
	m->nsym = 0;
	m->state = TM_ST_UNLOCKED;
	m->bad_slots = 0;
	m->settle = 0;
	m->fly_valid = 0;
	m->slot_start = 0;
	m->unlocked_syms = 0;
	m->fly_start = 0;
	m->level = 0.0f;
	m->snr_lin = 0.0f;
	m->ber = 0.0f;
	m->ppm = 0.0f;
}

void tetra_modem_init(struct tetra_rx *rx)
{
	struct tetra_modem *m = &rx->m;
	float e = 0.0f, i0b = bessel_i0(3.0f);
	int k;

	/* RRC alpha 0.35 over +-5 symbols, Kaiser beta 3 window: with the
	 * transmit RRC this leaves about -40 dB of ISI and rejects the adjacent
	 * 25 kHz channel by more than 60 dB. */
	for (k = 0; k < TM_MF_TAPS; k++) {
		float x = (float) (k - TM_MF_TAPS / 2) / (float) (TM_MF_TAPS / 2);
		float w = bessel_i0(3.0f * sqrtf(1.0f - x * x)) / i0b;

		m->taps[k] = w * rrc((float) (k - TM_MF_TAPS / 2) * 0.25f, 0.35f);
		e += m->taps[k] * m->taps[k];
	}
	/* sum(h^2) = 1/4: a unit power signal gives unit power symbols */
	e = 0.5f / sqrtf(e);
	for (k = 0; k < TM_MF_TAPS; k++)
		m->taps[k] *= e;

	set_ref(m->ref_q, m->bits_q, TETRA_TRAIN_NORM_3, 11);
	set_ref(m->ref_n, m->bits_n, TETRA_TRAIN_NORM_1, 11);
	set_ref(m->ref_p, m->bits_p, TETRA_TRAIN_NORM_2, 11);
	set_ref(m->ref_y, m->bits_y, TETRA_TRAIN_SYNC, 19);
	tetra_modem_reset(rx);
}

/* ---- burst synchronisation ------------------------------------------------- */

struct corr {
	float re, im, mag;
};

/* correlate n differential symbols starting at 'start' with a reference */
static void corr(const struct tetra_modem *m, uint32_t start, const float ref[][2],
		 unsigned int n, struct corr *c)
{
	float re = 0.0f, im = 0.0f, mag = 0.0f;
	unsigned int k;

	for (k = 0; k < n; k++) {
		uint32_t i = (start + k) & TM_RING_MASK;
		float dr = m->dr[i], di = m->di[i];

		re += dr * ref[k][0] + di * ref[k][1];
		im += di * ref[k][0] - dr * ref[k][1];
		mag += m->dm[i];
	}
	c->re = re;
	c->im = im;
	c->mag = mag;
}

struct slot_meas {
	float score;                /* 0..1 normalised coherent correlation */
	float re, im;               /* coherent sum (angle = rotation per symbol) */
	enum tetra_train_seq type;
};

/* Hypothesis: symbol 0 of a slot is at ring position S. The tail sequence q
 * straddles both slot boundaries, the middle of the slot holds training
 * sequence n, p (normal burst) or y (synchronisation burst). */
static void slot_measure(const struct tetra_modem *m, uint32_t S, struct slot_meas *sm)
{
	struct corr q1, q2, cn, cp, cy;
	float sn, sp, sy, re, im, mag;

	corr(m, S - 5, m->ref_q, 11, &q1);
	corr(m, S + 250, m->ref_q, 11, &q2);
	corr(m, S + 122, m->ref_n, 11, &cn);
	corr(m, S + 122, m->ref_p, 11, &cp);
	corr(m, S + 107, m->ref_y, 19, &cy);

	/* burst type from the middle training sequence alone */
	sn = cn.re * cn.re + cn.im * cn.im;
	sp = cp.re * cp.re + cp.im * cp.im;
	sy = (cy.re * cy.re + cy.im * cy.im) * (cn.mag * cn.mag);
	if (sy > (sn > sp ? sn : sp) * (cy.mag * cy.mag)) {
		sm->type = TETRA_TRAIN_SYNC;
		re = cy.re;
		im = cy.im;
		mag = cy.mag;
	} else if (sp > sn) {
		sm->type = TETRA_TRAIN_NORM_2;
		re = cp.re;
		im = cp.im;
		mag = cp.mag;
	} else {
		sm->type = TETRA_TRAIN_NORM_1;
		re = cn.re;
		im = cn.im;
		mag = cn.mag;
	}
	re += q1.re + q2.re;
	im += q1.im + q2.im;
	mag += q1.mag + q2.mag;
	sm->re = re;
	sm->im = im;
	sm->score = mag > 0.0f ? sqrtf(re * re + im * im) / (1.41421356f * mag) : 0.0f;
}

static void sync_event(struct tetra_rx *rx, int locked)
{
	tetra_rx_event_t *ev = tetra_rx_ev_begin(rx);

	ev->u.sync.locked = (uint8_t) locked;
	ev->u.sync.freq_offset_hz = rx->m.freq_est;
	rx->st.locked = (uint8_t) locked;
	tetra_rx_emit(rx, TETRA_RX_EV_SYNC);
}

/* time_sym: symbol index of the start of the slot rx->time stands for */
static void unlock(struct tetra_rx *rx, uint32_t time_sym)
{
	struct tetra_modem *m = &rx->m;

	m->state = TM_ST_UNLOCKED;
	m->unlocked_syms = 0;
	set_nco(m, m->freq_good);
	m->freq_est = m->freq_good;
	m->fly_valid = rx->time_valid;
	m->fly_start = time_sym;
	rx->time_valid = 0;
	rx->st.sync_losses++;
	tetra_umac_sync_lost(rx);
	sync_event(rx, 0);
}

static inline int8_t soft_clip(float v)
{
	if (v > 127.0f)
		return 127;
	if (v < -127.0f)
		return -127;
	return (int8_t) v;
}

static unsigned int bit_errors(const int8_t *soft, const uint8_t *ref, unsigned int n)
{
	unsigned int i, e = 0;

	for (i = 0; i < n; i++)
		e += (unsigned int) ((soft[i] < 0) != ref[i]);
	return e;
}

/* Demodulate the slot starting at S into m->soft. theta: rotation per symbol
 * estimated from the training sequences. Returns the refined rotation. */
static float slot_demod(struct tetra_rx *rx, uint32_t S, float theta, enum tetra_train_seq type)
{
	struct tetra_modem *m = &rx->m;
	float c = cosf(theta), s = sinf(theta);
	float are = 0.0f, aim = 0.0f, amp = 0.0f, pwr = 0.0f, mag = 0.0f, scale, a, var, th2;
	unsigned int k, errs, nbits;

	/* decision directed refinement over the whole slot */
	for (k = 0; k < TM_SLOT_SYMS; k++) {
		uint32_t i = (S + k) & TM_RING_MASK;
		float xr = m->dr[i] * c + m->di[i] * s;
		float xi = m->di[i] * c - m->dr[i] * s;
		float sr = xr >= 0.0f ? 1.0f : -1.0f;
		float si = xi >= 0.0f ? 1.0f : -1.0f;

		are += xr * sr + xi * si;
		aim += xi * sr - xr * si;
	}
	th2 = atan2f(aim, are);
	theta += th2;
	c = cosf(theta);
	s = sinf(theta);

	for (k = 0; k < TM_SLOT_SYMS; k++) {
		uint32_t i = (S + k) & TM_RING_MASK;

		amp += fabsf(m->dr[i] * c + m->di[i] * s) + fabsf(m->di[i] * c - m->dr[i] * s);
		pwr += m->dr[i] * m->dr[i] + m->di[i] * m->di[i];
		mag += m->dm[i];
	}
	a = amp / (2.0f * TM_SLOT_SYMS);                /* amplitude per component */
	var = pwr / (2.0f * TM_SLOT_SYMS) - a * a;      /* noise per component */
	scale = a > 0.0f ? TM_SOFT_NOM / a : 0.0f;

	for (k = 0; k < TM_SLOT_SYMS; k++) {
		uint32_t i = (S + k) & TM_RING_MASK;
		float xr = m->dr[i] * c + m->di[i] * s;
		float xi = m->di[i] * c - m->dr[i] * s;

		m->soft[2 * k] = soft_clip(xi * scale);
		m->soft[2 * k + 1] = soft_clip(xr * scale);
	}

	/* known bits: tail (q11..q22 at the start, q1..q10 at the end) and the
	 * training sequence in the middle */
	errs = bit_errors(m->soft, m->bits_q + 10, 12) + bit_errors(m->soft + 500, m->bits_q, 10);
	nbits = 22;
	if (type == TETRA_TRAIN_SYNC) {
		errs += bit_errors(m->soft + 214, m->bits_y, 38);
		nbits += 38;
	} else {
		errs += bit_errors(m->soft + 244, type == TETRA_TRAIN_NORM_2 ? m->bits_p : m->bits_n, 22);
		nbits += 22;
	}
	rx->st.train_bits += nbits;
	rx->st.train_bit_errors += errs;
	m->ber += ((float) errs / (float) nbits - m->ber) * 0.0625f;
	if (var > 1e-30f) {
		float snr = a * a / var;

		if (snr > 1e4f)
			snr = 1e4f;
		m->snr_lin += (snr - m->snr_lin) * 0.125f;
	}
	m->level += (mag / (float) TM_SLOT_SYMS - m->level) * 0.125f;
	return theta;
}

/* Called when 262 symbols of the slot at m->slot_start are in the ring. */
static void slot_due(struct tetra_rx *rx)
{
	struct tetra_modem *m = &rx->m;
	struct slot_meas sm, alt;
	uint32_t S = m->slot_start;
	float theta, hz;
	uint32_t crc;
	int decode;

	slot_measure(m, S, &sm);
	/* symbol slips: follow if a neighbouring alignment is clearly better */
	slot_measure(m, S - 1, &alt);
	if (alt.score > sm.score + TM_SHIFT_MARGIN) {
		sm = alt;
		S -= 1;
	} else {
		slot_measure(m, S + 1, &alt);
		if (alt.score > sm.score + TM_SHIFT_MARGIN) {
			sm = alt;
			S += 1;
		}
	}
	m->slot_start = S + TM_SLOT_SYMS;
	theta = atan2f(sm.im, sm.re);
	decode = sm.score >= TM_TH_DECODE;

	if (m->state == TM_ST_TENTATIVE) {
		if (sm.score < TM_TH_CONFIRM) {
			m->state = TM_ST_UNLOCKED;
			set_nco(m, m->freq_saved);
			return;
		}
		if (++m->confirm < TM_CONFIRM_SLOTS) {
			set_nco(m, m->freq_hz + TM_FREQ_GAIN * theta * (TM_FSYM / TM_TWO_PI));
			m->adj_acc = 0.0f;
			return;
		}
		m->state = TM_ST_LOCKED;
		m->bad_slots = 0;
		m->locked_slots = 0;
		m->idle_slots = 0;
		m->freq_est = m->freq_hz + theta * (TM_FSYM / TM_TWO_PI);
		rx->st.lock_count++;
		/* TDMA time flywheel: keep the frame count over a short loss of
		 * synchronisation if the slot grid is still the same */
		if (m->fly_valid) {
			uint32_t delta = S - m->fly_start;
			uint32_t k = (delta + TM_SLOT_SYMS / 2) / TM_SLOT_SYMS;
			int32_t r = (int32_t) (delta - k * TM_SLOT_SYMS);

			if (delta < TM_FLY_SYMS && r >= -2 && r <= 2) {
				tetra_rx_time_add(&rx->time, k);
				rx->time_valid = 1;
			}
			m->fly_valid = 0;
		}
		sync_event(rx, 1);
		if (m->settle)
			decode = 0;
	} else if (sm.score < TM_TH_BAD) {
		if (++m->bad_slots >= TM_BAD_SLOTS) {
			unlock(rx, S);
			return;
		}
	} else if (m->bad_slots)
		m->bad_slots--;

	rx->st.slots++;
	if (decode) {
		theta = slot_demod(rx, S, theta, sm.type);
		if (rx->cfg.flags & TETRA_RX_FLAG_BURST_EVENTS) {
			tetra_rx_event_t *ev = tetra_rx_ev_begin(rx);

			ev->u.burst.burst_type = sm.type == TETRA_TRAIN_SYNC ? 2 :
				sm.type == TETRA_TRAIN_NORM_2 ? 1 : 0;
			ev->u.burst.sync_score = sm.score;
			ev->u.burst.n_bits = TETRA_BITS_PER_TS;
			ev->u.burst.soft = m->soft;
			tetra_rx_emit(rx, TETRA_RX_EV_BURST);
		}
		tetra_lmac_slot(rx, m->soft, sm.type);
	} else
		tetra_lmac_slot(rx, NULL, sm.type);

	/* A continuous downlink carries at least the BSCH once per multiframe.
	 * Training sequences without any decodable block are not a TMO
	 * downlink (noise, DMO, uplink, another system): give up the lock. */
	crc = rx->st.chan[TETRA_RX_CH_BSCH].ok + rx->st.chan[TETRA_RX_CH_SB2].ok +
	      rx->st.chan[TETRA_RX_CH_SCH_HD].ok + rx->st.chan[TETRA_RX_CH_SCH_F].ok +
	      rx->st.chan[TETRA_RX_CH_STCH].ok;
	if (crc != m->crc_seen) {
		m->crc_seen = crc;
		m->idle_slots = 0;
		m->freq_good = m->freq_est;
	} else if (++m->idle_slots >= TM_IDLE_SLOTS) {
		unlock(rx, m->slot_start);
		return;
	}

	/* carrier loop: move the NCO towards the measured offset */
	if (sm.score >= TM_TH_CONFIRM) {
		hz = theta * (TM_FSYM / TM_TWO_PI);
		m->freq_est += (m->freq_hz + hz - m->freq_est) * 0.125f;
		/* fast for the first slots after lock, then slow and slew
		 * limited so that a few bad slots cannot pull the NCO away */
		if (m->locked_slots < 16) {
			m->locked_slots++;
			hz *= TM_FREQ_GAIN;
		} else {
			hz *= TM_FREQ_GAIN_TRACK;
			if (hz > TM_FREQ_SLEW)
				hz = TM_FREQ_SLEW;
			if (hz < -TM_FREQ_SLEW)
				hz = -TM_FREQ_SLEW;
		}
		set_nco(m, m->freq_hz + hz);
	}
	/* sample clock offset from the strobe adjustments of this slot */
	m->ppm += (m->adj_acc * (1e6f / (4.0f * TM_SLOT_SYMS)) - m->ppm) * 0.0625f;
	m->adj_acc = 0.0f;
}

static void symbol(struct tetra_rx *rx, float dr, float di)
{
	struct tetra_modem *m = &rx->m;
	uint32_t i = m->nsym & TM_RING_MASK;

	m->dr[i] = dr;
	m->di[i] = di;
	m->dm[i] = sqrtf(dr * dr + di * di);
	m->nsym++;

	if (m->state == TM_ST_UNLOCKED) {
		struct slot_meas sm;

		if (++m->unlocked_syms == 18000u) {
			/* nothing for a second: forget the old carrier estimate */
			set_nco(m, 0.0f);
			m->freq_good = 0.0f;
			m->adj_acc = 0.0f;
		}
		if (m->nsym < 266)
			return;
		slot_measure(m, m->nsym - 261, &sm);
		if (sm.score > TM_TH_ACQ) {
			float hz = atan2f(sm.im, sm.re) * (TM_FSYM / TM_TWO_PI);

			m->state = TM_ST_TENTATIVE;
			m->confirm = 0;
			m->slot_start = m->nsym - 261 + TM_SLOT_SYMS;
			m->freq_saved = m->freq_hz;
			m->settle = fabsf(hz) > TM_SETTLE_HZ;
			set_nco(m, m->freq_hz + hz);
			m->adj_acc = 0.0f;
		}
		return;
	}
	if (m->nsym - m->slot_start == 262)
		slot_due(rx);
}

/* ---- sample processing ----------------------------------------------------- */

static inline void sample(struct tetra_rx *rx, float xi, float xq)
{
	struct tetra_modem *m = &rx->m;
	const float *t = m->taps, *bi, *bq;
	float ri, rq, c, s, yi, yq, p;
	uint32_t pos;
	int k;

	if (rx->cfg.flags & TETRA_RX_FLAG_DC_BLOCK) {
		m->dc_i += (xi - m->dc_i) * (1.0f / 2048.0f);
		m->dc_q += (xq - m->dc_q) * (1.0f / 2048.0f);
		xi -= m->dc_i;
		xq -= m->dc_q;
	}

	/* carrier offset removal */
	ri = xi * m->nco_c - xq * m->nco_s;
	rq = xi * m->nco_s + xq * m->nco_c;
	c = m->nco_c * m->nco_dc - m->nco_s * m->nco_ds;
	s = m->nco_s * m->nco_dc + m->nco_c * m->nco_ds;
	if ((++m->nco_cnt & 255u) == 0) {
		float g = 1.5f - 0.5f * (c * c + s * s);

		c *= g;
		s *= g;
	}
	m->nco_c = c;
	m->nco_s = s;

	/* matched filter (symmetric taps) */
	pos = m->mf_pos;
	m->mf_i[pos] = m->mf_i[pos + TM_MF_TAPS] = ri;
	m->mf_q[pos] = m->mf_q[pos + TM_MF_TAPS] = rq;
	if (++pos == TM_MF_TAPS)
		pos = 0;
	m->mf_pos = pos;
	bi = m->mf_i + pos;
	bq = m->mf_q + pos;
	yi = t[TM_MF_TAPS / 2] * bi[TM_MF_TAPS / 2];
	yq = t[TM_MF_TAPS / 2] * bq[TM_MF_TAPS / 2];
	for (k = 0; k < TM_MF_TAPS / 2; k++) {
		yi += t[k] * (bi[k] + bi[TM_MF_TAPS - 1 - k]);
		yq += t[k] * (bq[k] + bq[TM_MF_TAPS - 1 - k]);
	}
	m->yi[0] = m->yi[1];
	m->yi[1] = m->yi[2];
	m->yi[2] = m->yi[3];
	m->yi[3] = yi;
	m->yq[0] = m->yq[1];
	m->yq[1] = m->yq[2];
	m->yq[2] = m->yq[3];
	m->yq[3] = yq;

	/* symbol timing: phase of the symbol rate line in |y|^2 */
	p = (yi * yi + yq * yq) * TM_TIMING_BETA;
	m->tc_re *= 1.0f - TM_TIMING_BETA;
	m->tc_im *= 1.0f - TM_TIMING_BETA;
	switch (m->phase4) {
	case 0:
		m->tc_re += p;
		break;
	case 1:
		m->tc_im -= p;
		break;
	case 2:
		m->tc_re -= p;
		break;
	default:
		m->tc_im += p;
		break;
	}

	/* strobe: the symbol instant lies between yi[1] and yi[2] */
	m->cnt -= 1.0f;
	if (m->cnt < 1.0f) {
		float mu = m->cnt > 0.0f ? m->cnt : 0.0f;
		float c0 = -mu * (mu - 1.0f) * (mu - 2.0f) * (1.0f / 6.0f);
		float c1 = (mu + 1.0f) * (mu - 1.0f) * (mu - 2.0f) * 0.5f;
		float c2 = -(mu + 1.0f) * mu * (mu - 2.0f) * 0.5f;
		float c3 = (mu + 1.0f) * mu * (mu - 1.0f) * (1.0f / 6.0f);
		float si = c0 * m->yi[0] + c1 * m->yi[1] + c2 * m->yi[2] + c3 * m->yi[3];
		float sq = c0 * m->yq[0] + c1 * m->yq[1] + c2 * m->yq[2] + c3 * m->yq[3];
		float dr = si * m->prev_i + sq * m->prev_q;
		float di = sq * m->prev_i - si * m->prev_q;
		float err, adj;

		m->prev_i = si;
		m->prev_q = sq;

		if ((m->nsym & 7u) == 0) {
			float tau = atan2f(m->tc_im, m->tc_re) * (-4.0f / TM_TWO_PI);

			m->tau = tau < 0.0f ? tau + 4.0f : tau;
		}
		err = m->tau - ((float) ((m->phase4 + 2u) & 3u) + mu);
		if (err >= 2.0f)
			err -= 4.0f;
		else if (err < -2.0f)
			err += 4.0f;
		adj = TM_TIMING_GAIN * err;
		m->cnt += 4.0f + adj;
		m->adj_acc += adj;
		symbol(rx, dr, di);
	}
	m->phase4 = (m->phase4 + 1u) & 3u;
}

void tetra_modem_cf32(struct tetra_rx *rx, const float *iq, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		sample(rx, iq[2 * i], iq[2 * i + 1]);
}

void tetra_modem_cs16(struct tetra_rx *rx, const int16_t *iq, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		sample(rx, (float) iq[2 * i] * (1.0f / 32768.0f), (float) iq[2 * i + 1] * (1.0f / 32768.0f));
}
