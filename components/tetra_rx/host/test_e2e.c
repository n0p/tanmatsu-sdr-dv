/* tetra_rx end-to-end tests on synthetic signals
 *
 * tetra_gen (encoder path built on the vendored osmo-tetra encoder
 * primitives, see tetra_gen.h) -> pi/4-DQPSK RRC modulator at 288 kHz ->
 * channel impairments -> model of the application front end (8 bit ADC,
 * loose low-pass, decimation to 72 kHz) -> tetra_rx. Every event of the
 * receiver is compared with what the generator sent.
 *
 * These are synthetic signals: they prove the receiver against the encoder
 * and the stated impairments, not against real networks (see README for the
 * tests on recordings).
 *
 * usage: test_e2e [-v] [test name ...]
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tetra_rx.h"
#include "tetra_gen.h"
#include "dsp_util.h"
#include "print_events.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FS_HI    288000.0
#define SPS      16
#define SLOT_HI  (255 * SPS)
#define PERIOD   (60 * 18 * 4)      /* slots per hyperframe */
#define DEC_TAPS 11
#define NO_NOISE 99.0

static int g_verbose;
static int g_failures;

static void check(int cond, const char *fmt, ...)
{
	va_list ap;

	if (cond)
		return;
	g_failures++;
	printf("    FAIL: ");
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	printf("\n");
}

/* ---- signal ---------------------------------------------------------------- */

struct slot_rec {
	uint8_t tn, fn, mn, kind, burst, has_sysinfo, off;
	uint16_t aach, hn;
	uint8_t tch[2][137];
	uint8_t bits[TETRA_GEN_SLOT_BITS];
};

struct sig_cfg {
	tetra_gen_cfg_t gen;
	unsigned n_slots;
	double ebn0_db;             /* NO_NOISE: none */
	double cfo_hz;
	double ppm;                 /* sample clock error */
	double adj_db, adj_hz;      /* other carrier, power relative to the wanted one; adj_hz 0: none */
	int quant_bits;             /* ADC resolution, 0: none */
	double rms_fs;              /* rms of the composite signal relative to full scale */
	unsigned off_from, off_to;  /* the carrier is off in slots [from, to) */
	int random_bits;            /* unstructured pi/4-DQPSK */
	int random_start;           /* random carrier phase and timing */
	uint64_t seed;
};

struct sig {
	struct sig_cfg cfg;
	tetra_gen_t *gen;
	struct slot_rec *slots;
	unsigned n_slots;
	long t0;                    /* TDMA time of slot 0 in slots */
	float *iq;                  /* 72 kHz */
	size_t n;
};

static void sig_default(struct sig_cfg *c)
{
	memset(c, 0, sizeof(*c));
	tetra_gen_default_cfg(&c->gen);
	c->gen.traffic_tn = 2;
	c->n_slots = 600;
	c->ebn0_db = NO_NOISE;
	c->rms_fs = 0.25;
	c->seed = 1;
}

static long tdma_slots(unsigned mn, unsigned fn, unsigned tn)
{
	return ((long) (mn - 1) * 18 + (long) (fn - 1)) * 4 + (long) (tn - 1);
}

static struct sig *sig_build(const struct sig_cfg *cfg)
{
	struct sig *s = calloc(1, sizeof(*s));
	dqpsk_mod_t *mod = dqpsk_mod_new(SPS), *adj = cfg->adj_hz != 0.0 ? dqpsk_mod_new(SPS) : NULL;
	resamp_t *rs = cfg->ppm != 0.0 ? resamp_new(FS_HI * (1.0 + cfg->ppm * 1e-6), FS_HI, 100e3) : NULL;
	float *a = malloc(sizeof(float) * 2 * SLOT_HI), *b = malloc(sizeof(float) * 2 * SLOT_HI);
	float *r = malloc(sizeof(float) * 2 * (SLOT_HI + 64));
	double noise = cfg->ebn0_db < NO_NOISE ? SPS / (2.0 * pow(10.0, cfg->ebn0_db / 10.0)) : 0.0;
	double adj_amp = adj ? pow(10.0, cfg->adj_db / 20.0) : 0.0;
	double gain = cfg->rms_fs / sqrt(1.0 + adj_amp * adj_amp + noise);
	double ph = 0.0, adj_ph = 0.0, full = cfg->quant_bits ? (double) (1 << (cfg->quant_bits - 1)) : 0.0;
	double h[DEC_TAPS], hsum = 0.0;
	float zi[DEC_TAPS] = { 0 }, zq[DEC_TAPS] = { 0 };
	unsigned zpos = 0, dec = 0, skip = 0, i, k, hn;
	uint8_t rb[TETRA_GEN_SLOT_BITS];
	rng_t rng;
	size_t j, m;

	rng_seed(&rng, cfg->seed * 7919u + 17u);
	s->cfg = *cfg;
	s->gen = tetra_gen_new(&cfg->gen);
	s->n_slots = cfg->n_slots;
	s->slots = calloc(cfg->n_slots, sizeof(*s->slots));
	s->iq = malloc(sizeof(float) * 2 * ((size_t) cfg->n_slots * (SLOT_HI / 4 + 2) + 16));
	s->t0 = tdma_slots(cfg->gen.start_mn, cfg->gen.start_fn, cfg->gen.start_tn);
	hn = cfg->gen.hyperframe;
	if (cfg->random_start) {
		ph = 2.0 * M_PI * rng_uniform(&rng) - M_PI;
		skip = rng_u32(&rng) % SLOT_HI;
	}

	/* The application decimates the 1.152 MS/s RTL-SDR stream with a 44 tap
	 * low-pass (fc about 19 kHz); the same filter length in time is 11 taps
	 * at 288 kHz: a very loose filter that leaves the neighbours in. */
	for (k = 0; k < DEC_TAPS; k++) {
		double x = (double) k - (DEC_TAPS - 1) / 2.0, fc = 19000.0 / FS_HI;
		double sinc = x == 0.0 ? 1.0 : sin(2.0 * M_PI * fc * x) / (2.0 * M_PI * fc * x);

		h[k] = sinc * (0.54 - 0.46 * cos(2.0 * M_PI * k / (DEC_TAPS - 1)));
		hsum += h[k];
	}
	for (k = 0; k < DEC_TAPS; k++)
		h[k] /= hsum;

	for (i = 0; i < cfg->n_slots; i++) {
		const tetra_gen_slot_t *gs = tetra_gen_next(s->gen);
		struct slot_rec *sr = &s->slots[i];

		if (i && gs->mn == 1 && gs->fn == 1 && gs->tn == 1)
			hn++;
		sr->tn = gs->tn;
		sr->fn = gs->fn;
		sr->mn = gs->mn;
		sr->kind = gs->kind;
		sr->burst = gs->burst;
		sr->has_sysinfo = gs->has_sysinfo;
		sr->aach = gs->aach;
		sr->hn = (uint16_t) hn;
		sr->off = i >= cfg->off_from && i < cfg->off_to;
		memcpy(sr->tch, gs->tch, sizeof(sr->tch));
		memcpy(sr->bits, gs->bits, sizeof(sr->bits));
		if (cfg->random_bits)
			for (k = 0; k < TETRA_GEN_SLOT_BITS; k++)
				sr->bits[k] = (uint8_t) (rng_u32(&rng) >> 31);

		dqpsk_mod(mod, sr->bits, TETRA_GEN_SLOT_BITS, a);
		if (sr->off)
			memset(a, 0, sizeof(float) * 2 * SLOT_HI);
		if (adj) {
			for (k = 0; k < TETRA_GEN_SLOT_BITS; k++)
				rb[k] = (uint8_t) (rng_u32(&rng) >> 31);
			dqpsk_mod(adj, rb, TETRA_GEN_SLOT_BITS, b);
			freq_shift(b, SLOT_HI, cfg->adj_hz, FS_HI, &adj_ph);
			for (j = 0; j < 2 * SLOT_HI; j++)
				a[j] += (float) adj_amp * b[j];
		}
		if (noise > 0.0)
			add_awgn(&rng, a, SLOT_HI, noise);
		freq_shift(a, SLOT_HI, cfg->cfo_hz, FS_HI, &ph);
		if (rs) {
			m = resamp_process(rs, a, SLOT_HI, r);
		} else {
			memcpy(r, a, sizeof(float) * 2 * SLOT_HI);
			m = SLOT_HI;
		}
		for (j = 0; j < m; j++) {
			double vi = (double) r[2 * j] * gain, vq = (double) r[2 * j + 1] * gain;

			if (skip) {
				skip--;
				continue;
			}
			if (cfg->quant_bits) {
				vi = floor(vi * full + 0.5);
				vq = floor(vq * full + 0.5);
				vi = (vi < -full ? -full : vi > full - 1.0 ? full - 1.0 : vi) / full;
				vq = (vq < -full ? -full : vq > full - 1.0 ? full - 1.0 : vq) / full;
			}
			zi[zpos] = (float) vi;
			zq[zpos] = (float) vq;
			zpos = (zpos + 1) % DEC_TAPS;
			if (++dec == 4) {
				double yi = 0.0, yq = 0.0;

				dec = 0;
				for (k = 0; k < DEC_TAPS; k++) {
					yi += h[k] * (double) zi[(zpos + k) % DEC_TAPS];
					yq += h[k] * (double) zq[(zpos + k) % DEC_TAPS];
				}
				s->iq[2 * s->n] = (float) yi;
				s->iq[2 * s->n + 1] = (float) yq;
				s->n++;
			}
		}
	}
	dqpsk_mod_free(mod);
	if (adj)
		dqpsk_mod_free(adj);
	if (rs)
		resamp_free(rs);
	free(a);
	free(b);
	free(r);
	return s;
}

static void sig_free(struct sig *s)
{
	tetra_gen_free(s->gen);
	free(s->slots);
	free(s->iq);
	free(s);
}

/* ---- receiver side: compare every event with the generator ----------------- */

struct ctx {
	const struct sig *sig;      /* NULL: do not compare */
	tetra_rx_t *rx;
	int await_cell;             /* ignore events until the BSCH of sig */
	uint64_t hash;              /* over all events as text */
	size_t pos;                 /* samples fed including the current block */
	unsigned n_ev[16];
	long first_bsch;            /* slot index of the first BSCH, -1 */
	size_t t_lock, t_bsch;      /* sample position of first lock / first BSCH */
	unsigned excl_from, excl_to; /* slots not expected to be received */
	uint8_t *seen;              /* per generator message: 1 received, 2 received wrong */
	size_t seen_cap;
	unsigned sig_unknown, sig_dup;
	uint8_t *slot_seen;         /* per slot: bit 0 BSCH, 1 SYSINFO, 2 AACH, 3 TRAFFIC ok, 4 burst */
	unsigned aach_wrong, tch_bfi, tch_err, tch_spurious, enc_wrong;
	unsigned lchan_noaach;      /* STCH / SCH mixed up in a slot whose ACCESS-ASSIGN was lost */
	unsigned bad;               /* events that contradict the generator although the CRC passed */
	unsigned bsch_changed;
	uint64_t ber_bits, ber_errs;
	unsigned misaligned;
	uint8_t enc_last;
	char err[600];
};

static void bad(struct ctx *c, const tetra_rx_event_t *ev, const char *fmt, ...)
{
	va_list ap;
	size_t n;

	if (c->bad++)
		return;
	va_start(ap, fmt);
	vsnprintf(c->err, sizeof(c->err), fmt, ap);
	va_end(ap);
	n = strlen(c->err);
	if (n + 4 < sizeof(c->err)) {
		strcpy(c->err + n, " | ");
		format_event(c->err + n + 3, sizeof(c->err) - n - 3, ev);
	}
}

static long slot_index(const struct ctx *c, const tetra_rx_time_t *t)
{
	long d;

	if (!t->valid || t->tn < 1 || t->fn < 1 || t->mn < 1)
		return -1;
	d = (tdma_slots(t->mn, t->fn, t->tn) - c->sig->t0 + PERIOD) % PERIOD;
	return d < (long) c->sig->n_slots ? d : -1;
}

#define CMP(field) do { if (c->field != e->field) return #field; } while (0)

static const char *cmp_cmce(const tetra_rx_cmce_t *c, const tetra_rx_cmce_t *e,
			    const tetra_gen_msg_t *m)
{
	CMP(parse_ok);
	CMP(fields);
	if (e->fields & TETRA_RX_CMCE_F_CALL_ID)
		CMP(call_id);
	if (e->fields & TETRA_RX_CMCE_F_CALL_TIMEOUT)
		CMP(call_timeout);
	if (e->fields & TETRA_RX_CMCE_F_HOOK)
		CMP(hook_method);
	if (e->fields & TETRA_RX_CMCE_F_SIMPLEX)
		CMP(duplex);
	if (e->fields & TETRA_RX_CMCE_F_BASIC_SERVICE) {
		CMP(circuit_mode);
		CMP(e2e_encryption);
		CMP(comm_type);
		CMP(slots_or_speech);
	}
	if (e->fields & TETRA_RX_CMCE_F_TX_GRANT)
		CMP(tx_grant);
	if (e->fields & TETRA_RX_CMCE_F_TX_REQ_PERM)
		CMP(tx_req_permission);
	if (e->fields & TETRA_RX_CMCE_F_ENC_CONTROL)
		CMP(enc_control);
	if (e->fields & TETRA_RX_CMCE_F_PRIORITY)
		CMP(call_priority);
	if (e->fields & TETRA_RX_CMCE_F_CALL_OWNER)
		CMP(call_ownership);
	if (e->fields & TETRA_RX_CMCE_F_NOTIFICATION)
		CMP(notification);
	if (e->fields & TETRA_RX_CMCE_F_DISC_CAUSE)
		CMP(disconnect_cause);
	if (e->fields & TETRA_RX_CMCE_F_TEMP_ADDR)
		CMP(temp_address);
	if (e->fields & TETRA_RX_CMCE_F_PARTY) {
		CMP(party.present);
		CMP(party.ssi);
		if (e->party.present == 2) {
			CMP(party.mcc);
			CMP(party.mnc);
		}
	}
	if (e->fields & TETRA_RX_CMCE_F_STATUS)
		CMP(status);
	if (e->fields & TETRA_RX_CMCE_F_SDS) {
		CMP(sds_type);
		CMP(sds_bits);
		if (!c->sds_data || memcmp(c->sds_data, m->sdu + m->sds_off, e->sds_bits))
			return "sds_data";
	}
	if (e->fields & TETRA_RX_CMCE_F_PROTOCOL_ID)
		CMP(protocol_id);
	if (e->fields & TETRA_RX_CMCE_F_TEXT) {
		CMP(text_coding);
		if (strcmp(c->text, e->text))
			return "text";
	}
	return NULL;
}

static const char *cmp_signal(struct ctx *c, const tetra_rx_ev_signal_t *s,
			      const tetra_gen_msg_t *m, long idx)
{
	if (!m->complete)
		return "message not sent completely";
	if (idx != (long) m->last_slot)
		return "slot";
	if (s->addr.type != (m->usage_marker ? TETRA_RX_ADDR_SSI_USAGE : TETRA_RX_ADDR_SSI))
		return "address type";
	if (m->usage_marker && s->addr.usage_marker != m->usage_marker)
		return "usage marker";
	if (s->encryption_mode != m->enc_mode)
		return "encryption mode";
	if (s->lchan != m->lchan) {
		/* without the ACCESS-ASSIGN of the slot the receiver cannot know */
		if (c->slot_seen[m->first_slot] & 4)
			return "logical channel";
		c->lchan_noaach++;
	}
	if (s->fragmented != (m->fragments > 1))
		return "fragmented";
	if (m->enc_mode) {
		if (!s->encrypted || s->sdu || s->have_llc || s->have_mle || s->have_cmce ||
		    s->chan_alloc.present)
			return "encrypted PDU not left alone";
		return NULL;
	}
	if (s->encrypted)
		return "encrypted";
	if (s->slot_grant != m->slot_grant)
		return "slot grant";
	if (s->chan_alloc.present != m->chan_alloc)
		return "channel allocation";
	if (m->chan_alloc && (s->chan_alloc.timeslots != m->chan_alloc_ts ||
			      s->chan_alloc.carrier != m->chan_alloc_carrier ||
			      s->chan_alloc.ul_dl != 3 || s->chan_alloc.alloc_type != 0 ||
			      s->chan_alloc.ext_carrier))
		return "channel allocation element";
	if (m->chan_alloc && m->chan_alloc_carrier == c->sig->cfg.gen.main_carrier &&
	    s->chan_alloc.dl_freq_hz && s->chan_alloc.dl_freq_hz !=
	    400000000u + 25000u * c->sig->cfg.gen.main_carrier + 6250u)
		return "channel allocation frequency";
	if (!s->have_llc || s->llc_pdu != m->llc)
		return "LLC PDU type";
	if (s->llc_fcs_present != (m->llc == 5) || s->llc_fcs_ok != (m->llc == 5))
		return "LLC FCS";
	if (!s->have_mle || s->pdisc != m->pdisc || s->pdu_type != m->pdu_type)
		return "PDU type";
	if (s->sdu_bits != m->sdu_bits || memcmp(s->sdu, m->sdu, m->sdu_bits))
		return "SDU";
	if (s->have_cmce != m->have_cmce)
		return "have_cmce";
	if (m->have_cmce)
		return cmp_cmce(&s->cmce, &m->cmce, m);
	return NULL;
}

static void on_signal(struct ctx *c, const tetra_rx_event_t *ev, long idx)
{
	const tetra_rx_ev_signal_t *s = &ev->u.signal;
	const tetra_gen_msg_t *m;
	const char *why;
	size_t i;

	if (s->mac_pdu != TETRA_RX_MAC_RESOURCE || s->addr.ssi < TETRA_GEN_SSI_BASE ||
	    s->addr.ssi - TETRA_GEN_SSI_BASE >= tetra_gen_msg_count(c->sig->gen)) {
		c->sig_unknown++;
		bad(c, ev, "signalling that was never sent");
		return;
	}
	i = s->addr.ssi - TETRA_GEN_SSI_BASE;
	m = tetra_gen_msg(c->sig->gen, i);
	if (i >= c->seen_cap) {
		size_t cap = 2 * i + 256;

		c->seen = realloc(c->seen, cap);
		memset(c->seen + c->seen_cap, 0, cap - c->seen_cap);
		c->seen_cap = cap;
	}
	if (c->seen[i]) {
		c->sig_dup++;
		bad(c, ev, "message received twice");
		return;
	}
	why = cmp_signal(c, s, m, idx);
	c->seen[i] = why ? 2 : 1;
	if (why)
		bad(c, ev, "%s differs (%s, message %zu)", why, tetra_gen_kind_name(m->kind), i);
	/* a call set-up addressed with the usage marker tells the encryption state */
	if (!why && m->usage_marker)
		c->enc_last = m->enc_mode ? TETRA_RX_ENC_ENCRYPTED : TETRA_RX_ENC_CLEAR;
}

static void on_sysinfo(struct ctx *c, const tetra_rx_event_t *ev, const struct slot_rec *sr)
{
	const tetra_rx_ev_sysinfo_t *si = &ev->u.sysinfo;
	const tetra_gen_cfg_t *g = &c->sig->cfg.gen;
	static const uint32_t off_hz[4] = { 0, 6250, (uint32_t) -6250, 12500 };
	uint32_t dl = 100000000u * g->freq_band + 25000u * g->main_carrier + off_hz[g->freq_offset];

	if (!sr->has_sysinfo)
		bad(c, ev, "SYSINFO in a slot without");
	else if (si->main_carrier != g->main_carrier || si->freq_band != g->freq_band ||
		 si->freq_offset != g->freq_offset || si->duplex_spacing != g->duplex_spacing ||
		 si->reverse_operation != g->reverse_operation || si->location_area != g->location_area ||
		 si->cck_valid || si->hyperframe != sr->hn || si->bs_service_details != g->bs_service_details ||
		 si->subscriber_class != 0xffff || si->option_field != 0 || si->option_value != 0xfffff ||
		 si->ms_txpwr_max_cell != 5 || si->rxlev_access_min != 3 || si->access_parameter != 7 ||
		 si->radio_dl_timeout != 3 || si->num_secondary_cch != 0 || si->dl_freq_hz != dl ||
		 (g->duplex_spacing == 0 && g->freq_band == 4 && !g->reverse_operation &&
		  si->ul_freq_hz != dl - 10000000u))
		bad(c, ev, "SYSINFO content");
}

static void on_traffic(struct ctx *c, const tetra_rx_event_t *ev, const struct slot_rec *sr, long idx)
{
	const tetra_rx_ev_traffic_t *t = &ev->u.traffic;
	const tetra_gen_cfg_t *g = &c->sig->cfg.gen;

	if (t->enc_state == (g->traffic_encrypted ? TETRA_RX_ENC_CLEAR : TETRA_RX_ENC_ENCRYPTED))
		c->enc_wrong++;
	switch (sr->kind) {
	case TETRA_GEN_SLOT_TCH:
		if (t->stolen[0] || t->stolen[1] || t->n_frames != 2 || t->n_soft != 432 || !t->soft ||
		    t->usage_marker != g->usage_marker) {
			c->tch_spurious++;
			break;
		}
		if (!t->crc_ok) {
			c->tch_bfi++;
			if (!t->bfi[0] || !t->bfi[1])
				bad(c, ev, "bad frame not flagged");
			break;
		}
		if (t->bfi[0] || t->bfi[1])
			bad(c, ev, "good frame flagged bad");
		if (memcmp(t->frame, sr->tch, sizeof(sr->tch)))
			c->tch_err++;       /* possible: the class 0 bits are unprotected */
		else
			c->slot_seen[idx] |= 8;
		break;
	case TETRA_GEN_SLOT_TCH_STOLEN1:
	case TETRA_GEN_SLOT_TCH_STOLEN2:
		if (t->stolen[0] != 1 || t->stolen[1] != (sr->kind == TETRA_GEN_SLOT_TCH_STOLEN2) ||
		    !t->bfi[0] || !t->bfi[1] || t->n_frames != 0 ||
		    t->n_soft != (sr->kind == TETRA_GEN_SLOT_TCH_STOLEN2 ? 0 : 216))
			c->tch_spurious++;
		else
			c->slot_seen[idx] |= 8;
		break;
	default:
		c->tch_spurious++;
		break;
	}
}

static void on_event(void *user, const tetra_rx_event_t *ev)
{
	struct ctx *c = user;
	char line[4200];
	const struct slot_rec *sr;
	long idx;
	unsigned i;

	format_event(line, sizeof(line), ev);
	c->hash = fnv1a(c->hash, line);
	if (ev->type == TETRA_RX_EV_BURST)
		for (i = 0; i < ev->u.burst.n_bits; i++)
			c->hash = (c->hash ^ (uint8_t) ev->u.burst.soft[i]) * 0x100000001b3ull;
	if (g_verbose && ev->type != TETRA_RX_EV_BURST && ev->type != TETRA_RX_EV_AACH)
		printf("      %7.3f s  %s\n", (double) c->pos / 72000.0, line);
	c->n_ev[ev->type & 15]++;

	if (ev->type == TETRA_RX_EV_SYNC) {
		if (ev->u.sync.locked && !c->t_lock)
			c->t_lock = c->pos;
		return;
	}
	if (!c->sig)
		return;
	if (c->await_cell) {
		if (ev->type != TETRA_RX_EV_BSCH || ev->u.bsch.mcc != c->sig->cfg.gen.mcc)
			return;
		c->await_cell = 0;
	}
	if (ev->type == TETRA_RX_EV_BSCH) {
		const tetra_rx_ev_bsch_t *b = &ev->u.bsch;
		const tetra_gen_cfg_t *g = &c->sig->cfg.gen;
		tetra_rx_time_t t = { 1, b->tn, b->fn, b->mn };

		idx = slot_index(c, &t);
		if (idx < 0 || c->sig->slots[idx].burst != 2 || b->mcc != g->mcc || b->mnc != g->mnc ||
		    b->colour_code != g->colour_code || b->system_code != 1 || b->sharing_mode ||
		    b->ts_reserved_frames || b->u_plane_dtx || b->frame18_ext ||
		    b->neighbour_cell_broadcast != 2 || b->cell_load != 1 || b->late_entry != 1 ||
		    ev->time.tn != b->tn || ev->time.fn != b->fn || ev->time.mn != b->mn) {
			bad(c, ev, "BSCH content");
			return;
		}
		c->slot_seen[idx] |= 1;
		c->bsch_changed += b->changed;
		if (c->first_bsch < 0) {
			c->first_bsch = idx;
			c->t_bsch = c->pos;
		}
		return;
	}
	idx = slot_index(c, &ev->time);
	if (idx < 0) {
		if (ev->type != TETRA_RX_EV_BURST)
			bad(c, ev, "event without a valid time");
		return;
	}
	sr = &c->sig->slots[idx];
	switch (ev->type) {
	case TETRA_RX_EV_SYSINFO:
		on_sysinfo(c, ev, sr);
		c->slot_seen[idx] |= 2;
		break;
	case TETRA_RX_EV_AACH:
		if (ev->u.aach.raw != sr->aach)
			c->aach_wrong++;
		else
			c->slot_seen[idx] |= 4;
		break;
	case TETRA_RX_EV_SIGNAL:
		on_signal(c, ev, idx);
		break;
	case TETRA_RX_EV_TRAFFIC:
		on_traffic(c, ev, sr, idx);
		break;
	case TETRA_RX_EV_BURST: {
		unsigned errs = 0;

		for (i = 0; i < TETRA_GEN_SLOT_BITS; i++)
			errs += (ev->u.burst.soft[i] < 0) != sr->bits[i];
		if (sr->off)
			break;
		if (errs > TETRA_GEN_SLOT_BITS / 4) {
			c->misaligned++;
			break;
		}
		c->ber_bits += TETRA_GEN_SLOT_BITS;
		c->ber_errs += errs;
		c->slot_seen[idx] |= 16;
		break;
	}
	default:
		break;
	}
}

static void ctx_attach(struct ctx *c, const struct sig *s)
{
	free(c->slot_seen);
	free(c->seen);
	c->seen = NULL;
	c->seen_cap = 0;
	c->sig = s;
	c->slot_seen = s ? calloc(s->n_slots, 1) : NULL;
	c->first_bsch = -1;
	c->excl_from = c->excl_to = 0;
	c->enc_last = TETRA_RX_ENC_UNKNOWN;
}

static void ctx_init(struct ctx *c, const struct sig *s, uint32_t flags)
{
	tetra_rx_config_t cfg;

	memset(c, 0, sizeof(*c));
	memset(&cfg, 0, sizeof(cfg));
	cfg.event_cb = on_event;
	cfg.user = c;
	cfg.flags = flags;
	c->hash = 0xcbf29ce484222325ull;
	c->rx = tetra_rx_create(&cfg);
	ctx_attach(c, s);
}

static void ctx_free(struct ctx *c)
{
	tetra_rx_destroy(c->rx);
	free(c->slot_seen);
	free(c->seen);
}

enum { API_CF32 = 0, API_CS16 };

/* block: samples per call; 0: random sizes 1..3000 */
static void feed(struct ctx *c, const float *iq, size_t n, size_t block, int api)
{
	int16_t *tmp = api == API_CS16 ? malloc(sizeof(int16_t) * 2 * 4096) : NULL;
	rng_t rng;
	size_t i = 0, k, b;

	rng_seed(&rng, 99);
	while (i < n) {
		b = block ? block : 1 + rng_u32(&rng) % 3000;
		if (b > 4096)
			b = 4096;
		if (b > n - i)
			b = n - i;
		c->pos += b;
		if (tmp) {
			for (k = 0; k < 2 * b; k++) {
				float v = iq[2 * i + k] * 32768.0f;

				tmp[k] = (int16_t) (v > 32767.0f ? 32767.0f : v < -32768.0f ? -32768.0f : lrintf(v));
			}
			tetra_rx_process_cs16(c->rx, tmp, b);
		} else
			tetra_rx_process_cf32(c->rx, iq + 2 * i, b);
		i += b;
	}
	free(tmp);
}

#define RX_FLAGS (TETRA_RX_FLAG_AACH_EVENTS | TETRA_RX_FLAG_BURST_EVENTS)

/* ---- evaluation ------------------------------------------------------------ */

struct result {
	unsigned exp_bsch, got_bsch, exp_si, got_si, exp_aach, got_aach;
	unsigned exp_tch, got_tch, exp_stolen, got_stolen, exp_burst, got_burst;
	unsigned exp_msg, got_msg, exp_frag, got_frag, exp_stch, got_stch, exp_enc, got_enc;
	unsigned kind_exp[TETRA_GEN_KIND_COUNT], kind_got[TETRA_GEN_KIND_COUNT];
	unsigned max_frags;
	double ber, mer, t_lock, t_bsch;
	tetra_rx_status_t st;
};

/* Expected: everything the generator sent after the slot in which the
 * receiver decoded its first BSCH, except the last slots still inside the
 * receiver and the excluded range. Sync losses are not excused. */
static void evaluate(const struct ctx *c, struct result *r)
{
	const struct sig *s = c->sig;
	long lo = c->first_bsch < 0 ? (long) s->n_slots : c->first_bsch;
	long hi = (long) s->n_slots - 3, i;
	size_t k, n = tetra_gen_msg_count(s->gen);

	memset(r, 0, sizeof(*r));
	for (i = lo + 1; i < hi; i++) {
		const struct slot_rec *sr = &s->slots[i];
		unsigned seen = c->slot_seen[i];

		if ((c->excl_to && i >= (long) c->excl_from && i < (long) c->excl_to) || sr->off)
			continue;
		r->exp_burst++;
		r->got_burst += (seen >> 4) & 1;
		r->exp_aach++;
		r->got_aach += (seen >> 2) & 1;
		if (sr->burst == 2) {
			r->exp_bsch++;
			r->got_bsch += seen & 1;
		}
		if (sr->has_sysinfo) {
			r->exp_si++;
			r->got_si += (seen >> 1) & 1;
		}
		if (sr->kind == TETRA_GEN_SLOT_TCH) {
			r->exp_tch++;
			r->got_tch += (seen >> 3) & 1;
		} else if (sr->kind == TETRA_GEN_SLOT_TCH_STOLEN1 || sr->kind == TETRA_GEN_SLOT_TCH_STOLEN2) {
			r->exp_stolen++;
			r->got_stolen += (seen >> 3) & 1;
		}
	}
	for (k = 0; k < n; k++) {
		const tetra_gen_msg_t *m = tetra_gen_msg(s->gen, k);
		unsigned ok = k < c->seen_cap && c->seen[k] == 1;

		if (!m->complete || (long) m->first_slot <= lo || (long) m->last_slot >= hi)
			continue;
		if (c->excl_to && m->last_slot >= c->excl_from && m->first_slot < c->excl_to)
			continue;
		r->exp_msg++;
		r->got_msg += ok;
		r->kind_exp[m->kind]++;
		r->kind_got[m->kind] += ok;
		if (m->fragments > 1) {
			r->exp_frag++;
			r->got_frag += ok;
			if (m->fragments > r->max_frags)
				r->max_frags = m->fragments;
		}
		if (m->lchan == TETRA_RX_CH_STCH) {
			r->exp_stch++;
			r->got_stch += ok;
		}
		if (m->enc_mode) {
			r->exp_enc++;
			r->got_enc += ok;
		}
	}
	r->ber = c->ber_bits ? (double) c->ber_errs / (double) c->ber_bits : 1.0;
	r->mer = r->exp_msg ? 1.0 - (double) r->got_msg / r->exp_msg : 1.0;
	r->t_lock = c->t_lock ? (double) c->t_lock / 72000.0 : -1.0;
	r->t_bsch = c->t_bsch ? (double) c->t_bsch / 72000.0 : -1.0;
	tetra_rx_get_status(c->rx, &r->st);
}

/* everything must have been received, bit exact */
static void check_exact(const struct ctx *c, const struct result *r, const char *what)
{
	check(c->bad == 0, "%s: %u events differ from what was sent: %s", what, c->bad, c->err);
	check(r->exp_msg > 0 && r->got_msg == r->exp_msg, "%s: messages %u of %u", what, r->got_msg, r->exp_msg);
	check(r->got_bsch == r->exp_bsch && r->exp_bsch, "%s: BSCH %u of %u", what, r->got_bsch, r->exp_bsch);
	check(r->got_si == r->exp_si && r->exp_si, "%s: SYSINFO %u of %u", what, r->got_si, r->exp_si);
	check(r->got_aach == r->exp_aach, "%s: AACH %u of %u", what, r->got_aach, r->exp_aach);
	check(r->got_tch == r->exp_tch, "%s: TCH/S slots %u of %u", what, r->got_tch, r->exp_tch);
	check(r->got_stolen == r->exp_stolen, "%s: stolen slots %u of %u", what, r->got_stolen, r->exp_stolen);
	check(c->aach_wrong == 0 && c->tch_bfi == 0 && c->tch_err == 0 && c->tch_spurious == 0 &&
	      c->enc_wrong == 0 && c->misaligned == 0,
	      "%s: aach_wrong %u tch_bfi %u tch_err %u tch_spurious %u enc_wrong %u misaligned %u", what,
	      c->aach_wrong, c->tch_bfi, c->tch_err, c->tch_spurious, c->enc_wrong, c->misaligned);
	check(c->ber_errs == 0, "%s: %llu bit errors", what, (unsigned long long) c->ber_errs);
	check(r->st.sync_losses == 0 && r->st.lock_count == 1, "%s: locks %u, losses %u", what,
	      r->st.lock_count, r->st.sync_losses);
}

static void print_row(const char *label, const struct ctx *c, const struct result *r)
{
	const tetra_rx_chan_stat_t *ch = r->st.chan;

	printf("    %-22s lock %5.2fs cell %5.2fs | BER %8.2e | BSCH %3u/%-3u SI %3u/%-3u AACH %4u/%-4u "
	       "TCH %3u/%-3u msg %3u/%-3u (MER %5.3f) | CRC fail HD %u/%u F %u/%u STCH %u/%u | "
	       "df %+7.1f Hz %+6.1f ppm SNR %4.1f dB | losses %u bad %u\n",
	       label, r->t_lock, r->t_bsch, r->ber, r->got_bsch, r->exp_bsch, r->got_si, r->exp_si,
	       r->got_aach, r->exp_aach, r->got_tch, r->exp_tch, r->got_msg, r->exp_msg, r->mer,
	       ch[TETRA_RX_CH_SCH_HD].fail + ch[TETRA_RX_CH_SB2].fail,
	       ch[TETRA_RX_CH_SCH_HD].fail + ch[TETRA_RX_CH_SCH_HD].ok + ch[TETRA_RX_CH_SB2].fail + ch[TETRA_RX_CH_SB2].ok,
	       ch[TETRA_RX_CH_SCH_F].fail, ch[TETRA_RX_CH_SCH_F].fail + ch[TETRA_RX_CH_SCH_F].ok,
	       ch[TETRA_RX_CH_STCH].fail, ch[TETRA_RX_CH_STCH].fail + ch[TETRA_RX_CH_STCH].ok,
	       (double) r->st.freq_offset_hz, (double) r->st.timing_ppm, (double) r->st.evm_snr_db,
	       r->st.sync_losses, c->bad);
	if (c->bad)
		printf("      first: %s\n", c->err);
}

/* build, receive, evaluate */
static void run(const struct sig_cfg *cfg, size_t block, int api, struct ctx *c, struct result *r,
		struct sig **keep)
{
	struct sig *s = sig_build(cfg);

	ctx_init(c, s, RX_FLAGS);
	feed(c, s->iq, s->n, block, api);
	evaluate(c, r);
	if (keep)
		*keep = s;
	else {
		c->sig = NULL;
		sig_free(s);
	}
}

/* ---- theory ---------------------------------------------------------------- */

static double bessel_i(int k, double x)
{
	double term = 1.0, sum;
	int m;

	for (m = 1; m <= k; m++)
		term *= x / 2.0 / m;
	sum = term;
	for (m = 1; m < 400; m++) {
		term *= (x / 2.0) * (x / 2.0) / ((double) m * (double) (m + k));
		sum += term;
		if (term < 1e-17 * sum)
			break;
	}
	return sum;
}

/* bit error rate of differentially detected (pi/4-)DQPSK with Gray coding in AWGN */
static double dqpsk_ber(double ebn0_db)
{
	double g = pow(10.0, ebn0_db / 10.0);
	double a = sqrt(2.0 * g * (1.0 - sqrt(0.5))), b = sqrt(2.0 * g * (1.0 + sqrt(0.5)));
	double e = exp(-(a * a + b * b) / 2.0), q = 0.0, p = 1.0;
	int k;

	for (k = 0; k < 80; k++) {
		q += p * bessel_i(k, a * b);
		p *= a / b;
	}
	return e * (q - 0.5 * bessel_i(0, a * b));
}

/* ---- tests ----------------------------------------------------------------- */

static void test_clean(void)
{
	struct sig_cfg cfg;
	struct ctx c;
	struct result r;
	unsigned k, v;

	printf("clean signal: every event against the generator\n");
	for (v = 0; v < 3; v++) {
		static const char *const names[] = {
			"TN1 MCCH, TN2 traffic", "four control channels", "TN1 MCCH, TN4 traffic, seed 2"
		};

		sig_default(&cfg);
		cfg.n_slots = 1400;
		if (v == 1) {
			cfg.gen.traffic_tn = 0;
			cfg.gen.all_control = 1;
			cfg.gen.start_mn = 60;
			cfg.gen.start_fn = 15;
			cfg.gen.start_tn = 3;
		} else if (v == 2) {
			cfg.gen.traffic_tn = 4;
			cfg.gen.seed = 2;
			cfg.gen.mcc = 262;
			cfg.gen.mnc = 1001;
			cfg.gen.colour_code = 1;
			cfg.gen.start_mn = 31;
		}
		run(&cfg, 1024, API_CF32, &c, &r, NULL);
		print_row(names[v], &c, &r);
		check_exact(&c, &r, names[v]);
		check(r.exp_frag >= 10 && r.max_frags >= 3, "fragmented messages: %u, up to %u blocks",
		      r.exp_frag, r.max_frags);
		check(r.exp_enc >= 5, "encrypted PDUs: %u", r.exp_enc);
		for (k = 0; k < TETRA_GEN_KIND_COUNT; k++)
			check(r.kind_exp[k] >= 5 && r.kind_got[k] == r.kind_exp[k], "%s: %u of %u",
			      tetra_gen_kind_name((uint8_t) k), r.kind_got[k], r.kind_exp[k]);
		if (v != 1) {
			check(r.exp_tch > 200 && r.exp_stolen > 50 && r.exp_stch > 50,
			      "traffic slots %u, stolen %u, STCH messages %u", r.exp_tch, r.exp_stolen, r.exp_stch);
			check(c.enc_last == TETRA_RX_ENC_CLEAR && r.st.slot[cfg.gen.traffic_tn - 1].enc_state ==
			      TETRA_RX_ENC_CLEAR, "traffic not reported clear");
			check(r.st.slot[cfg.gen.traffic_tn - 1].dl_usage == cfg.gen.usage_marker,
			      "usage marker in the status");
		}
		check(fabs((double) r.st.freq_offset_hz) < 5.0 && fabs((double) r.st.timing_ppm) < 3.0,
		      "offset estimates %f Hz, %f ppm", (double) r.st.freq_offset_hz, (double) r.st.timing_ppm);
		check(r.st.cell_valid && r.st.mcc == cfg.gen.mcc && r.st.mnc == cfg.gen.mnc &&
		      r.st.colour_code == cfg.gen.colour_code && r.st.sysinfo_valid &&
		      r.st.sysinfo.location_area == cfg.gen.location_area, "status cell identity");
		check(r.st.train_bit_errors == 0 && r.st.frag_dropped == 0, "training errors %u, fragments dropped %u",
		      r.st.train_bit_errors, r.st.frag_dropped);
		{
			static float pts[2 * TETRA_RX_CONSTELLATION_MAX];
			size_t n = tetra_rx_get_constellation(c.rx, pts, 300), i;
			unsigned quad[4] = {0, 0, 0, 0};
			double worst = 0.0, sum = 0.0;

			for (i = 0; i < n; i++) {
				double er = fabs((double) pts[2 * i]) - 0.70710678;
				double ei = fabs((double) pts[2 * i + 1]) - 0.70710678;
				double e = sqrt(er * er + ei * ei);

				quad[(pts[2 * i] < 0.0f) + 2 * (pts[2 * i + 1] < 0.0f)]++;
				sum += e * e;
				if (e > worst)
					worst = e;
			}
			/* the filters of generator and receiver are truncated RRCs: about 6 % of ISI
			 * remain, the same 24 dB that the status reports as SNR on a clean signal */
			check(n == 300 && sqrt(sum / 300.0) < 0.08 && worst < 0.2,
			      "constellation: %u points, error rms %.3f, worst %.3f", (unsigned) n,
			      sqrt(sum / 300.0), worst);
			if (g_verbose)
				printf("      constellation error rms %.3f, worst %.3f\n", sqrt(sum / 300.0), worst);
			check(quad[0] > 30 && quad[1] > 30 && quad[2] > 30 && quad[3] > 30,
			      "constellation quadrants %u %u %u %u", quad[0], quad[1], quad[2], quad[3]);
			check(tetra_rx_get_constellation(c.rx, pts, 100000) == TETRA_RX_CONSTELLATION_MAX,
			      "constellation size limit");
		}
		printf("      messages: %u (fragmented %u, up to %u MAC blocks; on STCH %u; encrypted %u);"
		       " kinds:", r.exp_msg, r.exp_frag, r.max_frags, r.exp_stch, r.exp_enc);
		for (k = 0; k < TETRA_GEN_KIND_COUNT; k++)
			printf(" %u", r.kind_exp[k]);
		printf("\n");
		ctx_free(&c);
	}
}

static uint64_t status_hash(const tetra_rx_status_t *st)
{
	char buf[512];
	unsigned i, n = 0;

	n += (unsigned) snprintf(buf + n, sizeof(buf) - n, "%u %a %a %a %a %a %u %u %u %u %u %u %llu",
				 st->locked, (double) st->freq_offset_hz, (double) st->timing_ppm,
				 (double) st->level_dbfs, (double) st->evm_snr_db, (double) st->train_ber,
				 st->train_bits, st->train_bit_errors, st->slots, st->lock_count,
				 st->sync_losses, st->frag_dropped, (unsigned long long) st->samples);
	for (i = 0; i < TETRA_RX_CH_COUNT; i++)
		n += (unsigned) snprintf(buf + n, sizeof(buf) - n, " %u/%u", st->chan[i].ok, st->chan[i].fail);
	return fnv1a(0xcbf29ce484222325ull, buf);
}

static void test_blocks(void)
{
	static const struct {
		size_t block;
		int api;
		const char *name;
	} v[] = {
		{ 1024, API_CF32, "cf32, 1024" }, { 1, API_CF32, "cf32, 1" }, { 7, API_CF32, "cf32, 7" },
		{ 4096, API_CF32, "cf32, 4096" }, { 0, API_CF32, "cf32, random 1..3000" },
		{ 1024, API_CS16, "cs16, 1024" }, { 1, API_CS16, "cs16, 1" }, { 0, API_CS16, "cs16, random" },
	};
	struct sig_cfg cfg;
	struct sig *s;
	uint64_t ref[2][2] = { { 0, 0 }, { 0, 0 } };
	unsigned i, j;

	printf("block size invariance (noisy signal with offsets, all events incl. soft bits hashed)\n");
	sig_default(&cfg);
	cfg.n_slots = 500;
	cfg.ebn0_db = 9.0;
	cfg.cfo_hz = 731.0;
	cfg.ppm = 23.0;
	cfg.random_start = 1;
	s = sig_build(&cfg);
	/* the cs16 and cf32 paths must agree when given the same values */
	for (j = 0; j < 2 * s->n; j++)
		s->iq[j] = (float) lrintf(s->iq[j] * 32768.0f) / 32768.0f;
	for (i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
		struct ctx c;
		tetra_rx_status_t st;
		uint64_t sh;

		ctx_init(&c, s, RX_FLAGS | TETRA_RX_FLAG_BLOCK_EVENTS);
		feed(&c, s->iq, s->n, v[i].block, v[i].api);
		tetra_rx_get_status(c.rx, &st);
		sh = status_hash(&st);
		printf("    %-22s events %016llx status %016llx (%u SIGNAL, %u TRAFFIC, %u BURST)\n", v[i].name,
		       (unsigned long long) c.hash, (unsigned long long) sh, c.n_ev[TETRA_RX_EV_SIGNAL],
		       c.n_ev[TETRA_RX_EV_TRAFFIC], c.n_ev[TETRA_RX_EV_BURST]);
		if (i == 0) {
			ref[0][0] = c.hash;
			ref[0][1] = sh;
			check(c.n_ev[TETRA_RX_EV_SIGNAL] > 50 && c.n_ev[TETRA_RX_EV_TRAFFIC] > 50,
			      "too few events for a meaningful comparison");
		}
		check(c.hash == ref[0][0] && sh == ref[0][1], "%s differs from %s", v[i].name, v[0].name);
		ctx_free(&c);
	}
	sig_free(s);
}

static void test_awgn(void)
{
	static const double pts[] = { 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 17, 20 };
	struct sig_cfg cfg;
	unsigned i;

	printf("AWGN sweep (2100 slots = 30 s each; Eb = energy per channel bit; MER = signalling\n"
	       "messages lost after the first BSCH, sync losses included)\n");
	for (i = 0; i < sizeof(pts) / sizeof(pts[0]); i++) {
		struct ctx c;
		struct result r;
		char label[64];

		sig_default(&cfg);
		cfg.n_slots = 2100;
		cfg.ebn0_db = pts[i];
		cfg.random_start = 1;
		cfg.seed = 10 + i;
		cfg.gen.seed = 10 + i;
		run(&cfg, 1024, API_CF32, &c, &r, NULL);
		snprintf(label, sizeof(label), "Eb/N0 %4.1f dB", pts[i]);
		print_row(label, &c, &r);
		printf("      theory BER %.2e; TCH/S: %u of %u slots bit exact, %u bad CRC, %u good CRC with class 0"
		       " errors; AACH wrong %u; spurious traffic %u; channel type unknown %u\n", dqpsk_ber(pts[i]),
		       r.got_tch, r.exp_tch, c.tch_bfi, c.tch_err, c.aach_wrong, c.tch_spurious, c.lchan_noaach);
		check(c.bad == 0, "Eb/N0 %.0f dB: %u events contradict the generator: %s", pts[i], c.bad, c.err);
		check(r.ber < 2.0 * dqpsk_ber(pts[i]) + 1e-5, "Eb/N0 %.0f dB: BER %.2e, theory %.2e", pts[i],
		      r.ber, dqpsk_ber(pts[i]));
		check(r.t_bsch > 0 && r.t_bsch < (pts[i] >= 8 ? 2.5 : 8.0), "Eb/N0 %.0f dB: cell after %.2f s",
		      pts[i], r.t_bsch);
		if (pts[i] >= 10)
			check(r.mer < 0.01 && r.st.sync_losses == 0, "Eb/N0 %.0f dB: MER %.3f, %u sync losses",
			      pts[i], r.mer, r.st.sync_losses);
		if (pts[i] >= 14)
			check(r.mer == 0.0 && r.got_tch == r.exp_tch && r.got_aach == r.exp_aach,
			      "Eb/N0 %.0f dB: not error free", pts[i]);
		ctx_free(&c);
	}
}

static void test_cfo(void)
{
	static const double pts[] = { -2500, -2000, -1000, -200, 200, 1000, 2000, 2500, 3000 };
	struct sig_cfg cfg;
	unsigned i;

	printf("carrier frequency offset (Eb/N0 10 dB, 500 slots)\n");
	for (i = 0; i < sizeof(pts) / sizeof(pts[0]); i++) {
		struct ctx c;
		struct result r;
		char label[64];

		sig_default(&cfg);
		cfg.n_slots = 500;
		cfg.ebn0_db = 10.0;
		cfg.cfo_hz = pts[i];
		cfg.random_start = 1;
		cfg.seed = 30 + i;
		run(&cfg, 1024, API_CF32, &c, &r, NULL);
		snprintf(label, sizeof(label), "CFO %+5.0f Hz", pts[i]);
		print_row(label, &c, &r);
		if (fabs(pts[i]) <= 2500) {
			check(c.bad == 0, "%s: %s", label, c.err);
			check(r.t_bsch > 0 && r.t_bsch < 2.5, "%s: cell after %.2f s", label, r.t_bsch);
			check(fabs((double) r.st.freq_offset_hz - pts[i]) < 30.0, "%s: estimate %.1f Hz", label,
			      (double) r.st.freq_offset_hz);
			check(r.mer < 0.02 && r.st.sync_losses == 0, "%s: MER %.3f, %u losses", label, r.mer,
			      r.st.sync_losses);
		}
		ctx_free(&c);
	}
}

static void test_ppm(void)
{
	static const double pts[] = { -100, -50, -20, 20, 50, 100 };
	struct sig_cfg cfg;
	unsigned i;

	printf("sample clock offset (Eb/N0 10 dB, CFO 500 Hz, 1500 slots = 21 s)\n");
	for (i = 0; i < sizeof(pts) / sizeof(pts[0]); i++) {
		struct ctx c;
		struct result r;
		char label[64];

		sig_default(&cfg);
		cfg.n_slots = 1500;
		cfg.ebn0_db = 10.0;
		cfg.cfo_hz = 500.0;
		cfg.ppm = pts[i];
		cfg.random_start = 1;
		cfg.seed = 50 + i;
		run(&cfg, 1024, API_CF32, &c, &r, NULL);
		snprintf(label, sizeof(label), "clock %+4.0f ppm", pts[i]);
		print_row(label, &c, &r);
		check(c.bad == 0, "%s: %s", label, c.err);
		check(r.st.sync_losses == 0 && r.st.lock_count == 1, "%s: %u losses", label, r.st.sync_losses);
		check(r.mer < 0.01, "%s: MER %.3f", label, r.mer);
		/* the sign convention: a positive value means the receiver's clock is slow,
		 * i.e. the signal appears compressed; only the magnitude is checked here */
		check(fabs(fabs((double) r.st.timing_ppm) - fabs(pts[i])) < 8.0, "%s: estimate %.1f ppm", label,
		      (double) r.st.timing_ppm);
		ctx_free(&c);
	}
}

static void test_start(void)
{
	struct sig_cfg cfg;
	rng_t rng;
	double worst_lock = 0.0, worst_cell = 0.0, sum_cell = 0.0;
	unsigned i, n = 40;

	printf("acquisition: %u starts with random TDMA time, timing, phase, CFO +-2 kHz, clock +-50 ppm"
	       " (Eb/N0 10 dB)\n", n);
	rng_seed(&rng, 4711);
	for (i = 0; i < n; i++) {
		struct ctx c;
		struct result r;

		sig_default(&cfg);
		cfg.n_slots = 260;
		cfg.ebn0_db = 10.0;
		cfg.cfo_hz = 4000.0 * rng_uniform(&rng) - 2000.0;
		cfg.ppm = 100.0 * rng_uniform(&rng) - 50.0;
		cfg.random_start = 1;
		cfg.seed = 100 + i;
		cfg.gen.seed = 100 + i;
		cfg.gen.start_tn = (uint8_t) (1 + rng_u32(&rng) % 4);
		cfg.gen.start_fn = (uint8_t) (1 + rng_u32(&rng) % 18);
		cfg.gen.start_mn = (uint8_t) (1 + rng_u32(&rng) % 60);
		cfg.gen.traffic_tn = (uint8_t) (i % 4 == 0 ? 0 : 2 + i % 3);
		cfg.gen.all_control = i % 2;
		run(&cfg, 64, API_CF32, &c, &r, NULL);
		if (g_verbose)
			print_row("start", &c, &r);
		check(c.bad == 0, "start %u: %s", i, c.err);
		check(r.t_bsch > 0 && r.t_bsch < 2.5, "start %u: cell after %.2f s", i, r.t_bsch);
		check(r.mer < 0.05 && r.st.sync_losses == 0, "start %u: MER %.3f, %u losses", i, r.mer,
		      r.st.sync_losses);
		check(fabs((double) r.st.freq_offset_hz - cfg.cfo_hz) < 30.0, "start %u: CFO %.1f, estimate %.1f",
		      i, cfg.cfo_hz, (double) r.st.freq_offset_hz);
		if (r.t_lock > worst_lock)
			worst_lock = r.t_lock;
		if (r.t_bsch > worst_cell)
			worst_cell = r.t_bsch;
		sum_cell += r.t_bsch;
		ctx_free(&c);
	}
	printf("    burst sync after at most %.2f s; cell identity (first BSCH) after %.2f s on average,"
	       " %.2f s at most\n", worst_lock, sum_cell / n, worst_cell);
}

static void test_quant(void)
{
	static const struct {
		double rms;
		double ebn0;
		const char *name;
	} v[] = {
		{ 0.25, NO_NOISE, "8 bit, -12 dBFS" },
		{ 0.02, NO_NOISE, "8 bit, rms 2.6 LSB" },
		{ 0.008, NO_NOISE, "8 bit, rms 1.0 LSB" },
		{ 0.004, NO_NOISE, "8 bit, rms 0.5 LSB" },
		{ 0.5, NO_NOISE, "8 bit, -6 dBFS (clips)" },
		{ 1.0, NO_NOISE, "8 bit, 0 dBFS (clips)" },
		{ 2.0, NO_NOISE, "8 bit, +6 dBFS (clips)" },
		{ 0.02, 10.0, "8 bit, 2.6 LSB, 10 dB" },
		{ 0.7, 10.0, "8 bit, -3 dBFS, 10 dB" },
	};
	struct sig_cfg cfg;
	unsigned i;

	printf("8 bit quantisation at 288 kHz (level = rms of signal + noise; 700 slots)\n");
	for (i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
		struct ctx c;
		struct result r;

		sig_default(&cfg);
		cfg.n_slots = 700;
		cfg.quant_bits = 8;
		cfg.rms_fs = v[i].rms;
		cfg.ebn0_db = v[i].ebn0;
		cfg.cfo_hz = 300.0;
		cfg.random_start = 1;
		cfg.seed = 70 + i;
		run(&cfg, 1024, API_CF32, &c, &r, NULL);
		print_row(v[i].name, &c, &r);
		check(c.bad == 0, "%s: %s", v[i].name, c.err);
		if (v[i].rms >= 0.008 && v[i].rms <= 1.0)
			check(r.mer < 0.01 && r.st.sync_losses == 0 && r.t_bsch > 0 && r.t_bsch < 2.5,
			      "%s: MER %.3f, %u losses, cell after %.2f s", v[i].name, r.mer, r.st.sync_losses,
			      r.t_bsch);
		ctx_free(&c);
	}
}

static void test_adjacent(void)
{
	static const struct {
		double hz, db;
		int bits;
		int must;
	} v[] = {
		{ 25000, 10, 0, 1 },  { -25000, 20, 0, 1 }, { 25000, 30, 0, 1 },  { -25000, 40, 0, 0 },
		{ 25000, 50, 0, 0 },  { 25000, 10, 8, 1 },  { -25000, 20, 8, 1 }, { 25000, 30, 8, 1 },
		{ -25000, 40, 8, 0 }, { 50000, 30, 8, 1 },  { 50000, 40, 8, 0 },  { 75000, 0, 8, 0 },
		{ 75000, 10, 8, 0 },  { -75000, 20, 8, 0 },
	};
	struct sig_cfg cfg;
	unsigned i;

	printf("other carriers (power relative to the wanted carrier; wanted Eb/N0 14 dB, CFO 400 Hz;\n"
	       "8 bit: ADC at 288 kHz with the composite at -12 dBFS; 700 slots).\n"
	       "A carrier at +-75 kHz aliases to -+3 kHz in the 72 kHz stream: only the front end\n"
	       "filter (modelled: 11 taps at 288 kHz = 44 taps at 1.152 MHz) can remove it.\n");
	for (i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
		struct ctx c;
		struct result r;
		char label[64];

		sig_default(&cfg);
		cfg.n_slots = 700;
		cfg.ebn0_db = 14.0;
		cfg.cfo_hz = 400.0;
		cfg.adj_hz = v[i].hz;
		cfg.adj_db = v[i].db;
		cfg.quant_bits = v[i].bits;
		cfg.random_start = 1;
		cfg.seed = 90 + i;
		run(&cfg, 1024, API_CF32, &c, &r, NULL);
		snprintf(label, sizeof(label), "%+3.0f kHz %+3.0f dB %s", v[i].hz / 1000.0, v[i].db,
			 v[i].bits ? "8 bit" : "float");
		print_row(label, &c, &r);
		check(c.bad == 0, "%s: %s", label, c.err);
		if (v[i].must)
			check(r.mer < 0.01 && r.st.sync_losses == 0 && r.t_bsch > 0 && r.t_bsch < 2.5,
			      "%s: MER %.3f, %u losses, cell after %.2f s", label, r.mer, r.st.sync_losses, r.t_bsch);
		ctx_free(&c);
	}
}

static void test_encrypted(void)
{
	struct sig_cfg cfg;
	struct ctx c;
	struct result r;

	printf("air interface encrypted call: flagged, never parsed\n");
	sig_default(&cfg);
	cfg.n_slots = 1000;
	cfg.gen.traffic_encrypted = 1;
	cfg.gen.traffic_tn = 3;
	cfg.gen.usage_marker = 44;
	run(&cfg, 1024, API_CF32, &c, &r, NULL);
	print_row("encrypted call on TN3", &c, &r);
	check_exact(&c, &r, "encrypted");
	check(r.exp_enc > 80 && r.got_enc == r.exp_enc, "encrypted PDUs %u of %u", r.got_enc, r.exp_enc);
	check(c.enc_last == TETRA_RX_ENC_ENCRYPTED && r.st.slot[2].enc_state == TETRA_RX_ENC_ENCRYPTED,
	      "traffic not reported encrypted");
	printf("      %u encrypted MAC PDUs reported with header only; %u traffic slots flagged encrypted\n",
	       r.got_enc, r.got_tch);
	ctx_free(&c);
}

static void test_dropout(void)
{
	static const unsigned gap[] = { 3, 12, 70, 300 };
	struct sig_cfg cfg;
	unsigned i;

	printf("carrier interrupted (Eb/N0 12 dB; noise continues during the gap)\n");
	for (i = 0; i < sizeof(gap) / sizeof(gap[0]); i++) {
		struct sig *s;
		struct ctx c;
		struct result r;
		char label[64];

		sig_default(&cfg);
		cfg.n_slots = 900 + gap[i];
		cfg.ebn0_db = 12.0;
		cfg.cfo_hz = -900.0;
		cfg.ppm = 30.0;
		cfg.off_from = 400;
		cfg.off_to = 400 + gap[i];
		cfg.random_start = 1;
		cfg.seed = 120 + i;
		s = sig_build(&cfg);
		ctx_init(&c, s, RX_FLAGS);
		/* re-acquisition is allowed 2 s */
		c.excl_from = cfg.off_from - 2;
		c.excl_to = cfg.off_to + 142;
		feed(&c, s->iq, s->n, 1024, API_CF32);
		evaluate(&c, &r);
		snprintf(label, sizeof(label), "gap of %u slots", gap[i]);
		print_row(label, &c, &r);
		printf("      locks %u, losses %u\n", r.st.lock_count, r.st.sync_losses);
		check(c.bad == 0, "%s: %s", label, c.err);
		check(r.mer < 0.01 && r.got_bsch + 1 >= r.exp_bsch, "%s: MER %.3f, BSCH %u/%u", label, r.mer,
		      r.got_bsch, r.exp_bsch);
		check(r.st.locked && r.st.time.valid, "%s: not locked at the end", label);
		c.sig = NULL;
		ctx_free(&c);
		sig_free(s);
	}
}

static void test_cell_change(void)
{
	struct sig_cfg ca, cb;
	struct sig *a, *b;
	struct ctx c;
	struct result r;
	unsigned v;

	printf("change to another cell, without and with tetra_rx_reset()\n");
	for (v = 0; v < 2; v++) {
		sig_default(&ca);
		ca.n_slots = 400;
		ca.ebn0_db = 12.0;
		ca.cfo_hz = 800.0;
		ca.random_start = 1;
		ca.seed = 140;
		sig_default(&cb);
		cb.n_slots = 500;
		cb.ebn0_db = 12.0;
		cb.cfo_hz = -1500.0;
		cb.random_start = 1;
		cb.seed = 141;
		cb.gen.seed = 5;
		cb.gen.mcc = 204;
		cb.gen.mnc = 1337;
		cb.gen.colour_code = 5;
		cb.gen.main_carrier = 2001;
		cb.gen.location_area = 77;
		cb.gen.start_mn = 44;
		cb.gen.start_fn = 7;
		cb.gen.start_tn = 2;
		cb.gen.traffic_tn = 3;
		cb.gen.usage_marker = 9;
		a = sig_build(&ca);
		b = sig_build(&cb);
		ctx_init(&c, a, RX_FLAGS);
		feed(&c, a->iq, a->n, 1024, API_CF32);
		evaluate(&c, &r);
		print_row("cell A", &c, &r);
		check(c.bad == 0 && r.mer < 0.01, "cell A: MER %.3f %s", r.mer, c.err);
		if (v)
			tetra_rx_reset(c.rx);
		ctx_attach(&c, b);
		c.await_cell = 1;
		c.t_lock = c.t_bsch = 0;
		c.pos = 0;
		c.bsch_changed = 0;
		c.bad = 0;
		feed(&c, b->iq, b->n, 1024, API_CF32);
		evaluate(&c, &r);
		print_row(v ? "cell B after reset" : "cell B", &c, &r);
		check(c.bad == 0, "cell B: %s", c.err);
		check(!c.await_cell && c.bsch_changed == 1, "cell B: %u BSCH flagged as changed", c.bsch_changed);
		check(r.t_bsch > 0 && r.t_bsch < 3.0, "cell B: cell after %.2f s", r.t_bsch);
		check(r.mer < 0.01, "cell B: MER %.3f", r.mer);
		check(r.st.mcc == cb.gen.mcc && r.st.mnc == cb.gen.mnc && r.st.sysinfo.main_carrier == 2001 &&
		      r.st.sysinfo.location_area == 77, "cell B: status identity");
		check(fabs((double) r.st.freq_offset_hz + 1500.0) < 30.0, "cell B: CFO estimate %.1f",
		      (double) r.st.freq_offset_hz);
		c.sig = NULL;
		ctx_free(&c);
		sig_free(a);
		sig_free(b);
	}
}

static void test_no_signal(void)
{
	static const char *const names[] = {
		"noise", "unstructured DQPSK", "unstructured, 6 dB", "CW carrier at 1 kHz", "silence"
	};
	struct sig_cfg cfg;
	unsigned v, j;

	printf("no TETRA downlink (1400 slots = 20 s each): nothing may be decoded\n");
	for (v = 0; v < 5; v++) {
		struct sig *s;
		struct ctx c;
		tetra_rx_status_t st;

		sig_default(&cfg);
		cfg.n_slots = 1400;
		cfg.seed = 160 + v;
		cfg.ebn0_db = v == 2 ? 6.0 : v == 1 ? 20.0 : 0.0;
		cfg.random_bits = v == 1 || v == 2;
		if (v == 0 || v >= 3)
			cfg.off_to = cfg.n_slots;
		s = sig_build(&cfg);
		if (v == 3) {
			double ph = 0.0;

			for (j = 0; j < s->n; j++) {
				s->iq[2 * j] = 0.3f;
				s->iq[2 * j + 1] = 0.0f;
			}
			freq_shift(s->iq, s->n, 1000.0, 72000.0, &ph);
		} else if (v == 4)
			memset(s->iq, 0, sizeof(float) * 2 * s->n);
		ctx_init(&c, NULL, RX_FLAGS);
		feed(&c, s->iq, s->n, 1024, API_CF32);
		tetra_rx_get_status(c.rx, &st);
		printf("    %-22s burst sync events %u (locks %u), BSCH %u, SYSINFO %u, SIGNAL %u, TRAFFIC %u,"
		       " slots demodulated %u\n", names[v], c.n_ev[TETRA_RX_EV_SYNC], st.lock_count,
		       c.n_ev[TETRA_RX_EV_BSCH], c.n_ev[TETRA_RX_EV_SYSINFO], c.n_ev[TETRA_RX_EV_SIGNAL],
		       c.n_ev[TETRA_RX_EV_TRAFFIC], st.slots);
		check(!c.n_ev[TETRA_RX_EV_BSCH] && !c.n_ev[TETRA_RX_EV_SYSINFO] && !c.n_ev[TETRA_RX_EV_SIGNAL] &&
		      !c.n_ev[TETRA_RX_EV_TRAFFIC] && !c.n_ev[TETRA_RX_EV_AACH], "%s: events decoded", names[v]);
		check(!st.cell_valid && !st.time.valid, "%s: cell reported", names[v]);
		check(st.slots < 150, "%s: %u slots demodulated", names[v], st.slots);
		ctx_free(&c);
		sig_free(s);
	}
}

static const struct {
	const char *name;
	void (*fn)(void);
} tests[] = {
	{ "clean", test_clean },         { "blocks", test_blocks },     { "awgn", test_awgn },
	{ "cfo", test_cfo },             { "ppm", test_ppm },           { "start", test_start },
	{ "quant", test_quant },         { "adjacent", test_adjacent }, { "encrypted", test_encrypted },
	{ "dropout", test_dropout },     { "cell", test_cell_change },  { "nosignal", test_no_signal },
};

int main(int argc, char **argv)
{
	unsigned i;
	int a, sel = 0;

	setvbuf(stdout, NULL, _IOLBF, 0);
	for (a = 1; a < argc; a++) {
		if (!strcmp(argv[a], "-v"))
			g_verbose = 1;
		else
			sel = 1;
	}
	for (i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
		int want = !sel;

		for (a = 1; a < argc; a++)
			if (!strcmp(argv[a], tests[i].name))
				want = 1;
		if (want)
			tests[i].fn();
	}
	printf("test_e2e: %s (%d failed checks)\n", g_failures ? "FAILED" : "all passed", g_failures);
	return g_failures ? 1 : 0;
}
