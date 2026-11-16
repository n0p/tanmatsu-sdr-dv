/* tetra_rx - unit tests of the coding blocks and of the MAC / CMCE parsers
 *
 * Every block is checked against something that does not come from the code
 * under test: test vectors printed by osmo-tetra's own conv_enc_test, standard
 * check values, the formulas of EN 300 392-2 written out again here, or PDUs
 * assembled bit by bit from the tables of the standard.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tetra_rx.h"
#include "tetra_ctx.h"
#include "tetra_gen.h"
#include "dsp_util.h"
#include "test_vectors.h"

#include <lower_mac/crc_simple.h>
#include <lower_mac/tetra_conv_enc.h>
#include <lower_mac/tetra_interleave.h>
#include <lower_mac/tetra_rm3014.h>
#include <lower_mac/tetra_scramb.h>

static unsigned n_checks, n_failed;
static const char *cur_test = "";

__attribute__((format(printf, 2, 3)))
static void check(int ok, const char *fmt, ...)
{
	va_list ap;

	n_checks++;
	if (ok)
		return;
	n_failed++;
	printf("    FAIL [%s]: ", cur_test);
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	printf("\n");
}

static void begin(const char *name)
{
	cur_test = name;
	printf("%s\n", name);
}

static void str2bits(const char *s, uint8_t *bits)
{
	size_t i;

	for (i = 0; s[i]; i++)
		bits[i] = s[i] == '1';
}

static unsigned put(uint8_t *b, unsigned pos, unsigned n, uint32_t v)
{
	unsigned i;

	for (i = 0; i < n; i++)
		b[pos + i] = (v >> (n - 1 - i)) & 1;
	return pos + n;
}

static void rnd_bits(rng_t *r, uint8_t *b, unsigned n)
{
	unsigned i;

	for (i = 0; i < n; i++)
		b[i] = rng_u32(r) & 1;
}

static void bits2soft(const uint8_t *bits, int8_t *soft, unsigned n)
{
	unsigned i;

	for (i = 0; i < n; i++)
		soft[i] = bits[i] ? -64 : 64;
}

/* ---- event capture ---------------------------------------------------- */

#define CAP_MAX 48
#define CAP_BITS 700

struct cap {
	unsigned n, overflow;
	tetra_rx_event_t ev[CAP_MAX];
	uint8_t data[CAP_MAX][CAP_BITS];
};

static void on_event(void *user, const tetra_rx_event_t *ev)
{
	struct cap *c = user;
	tetra_rx_event_t *e;
	uint8_t *d;

	if (c->n >= CAP_MAX) {
		c->overflow++;
		return;
	}
	e = &c->ev[c->n];
	d = c->data[c->n];
	c->n++;
	*e = *ev;
	if (ev->type == TETRA_RX_EV_SIGNAL && ev->u.signal.sdu && ev->u.signal.sdu_bits <= CAP_BITS) {
		const tetra_rx_ev_signal_t *s = &ev->u.signal;

		memcpy(d, s->sdu, s->sdu_bits);
		e->u.signal.sdu = d;
		if (s->cmce.sds_data)
			e->u.signal.cmce.sds_data = d + (s->cmce.sds_data - s->sdu);
	} else if (ev->type == TETRA_RX_EV_BLOCK && ev->u.block.bits) {
		memcpy(d, ev->u.block.bits, ev->u.block.n_bits);
		e->u.block.bits = d;
	}
}

static unsigned cap_count(const struct cap *c, tetra_rx_event_type_t type)
{
	unsigned i, n = 0;

	for (i = 0; i < c->n; i++)
		n += c->ev[i].type == type;
	return n;
}

static const tetra_rx_event_t *cap_get(const struct cap *c, tetra_rx_event_type_t type, unsigned idx)
{
	unsigned i;

	for (i = 0; i < c->n; i++)
		if (c->ev[i].type == type && idx-- == 0)
			return &c->ev[i];
	return NULL;
}

/* a receiver that believes it is locked to cell (mcc, mnc, cc) at tn/fn/mn */
static tetra_rx_t *rx_new(struct cap *c, uint32_t flags, uint16_t mcc, uint16_t mnc, uint8_t cc,
			  unsigned tn, unsigned fn, unsigned mn)
{
	tetra_rx_config_t cfg;
	tetra_rx_t *rx;

	memset(c, 0, sizeof(*c));
	memset(&cfg, 0, sizeof(cfg));
	cfg.event_cb = on_event;
	cfg.user = c;
	cfg.flags = flags;
	rx = tetra_rx_create(&cfg);
	if (!rx) {
		printf("tetra_rx_create failed\n");
		exit(2);
	}
	tetra_lmac_set_cell(rx, mcc, mnc, cc);
	rx->time.tn = tn;
	rx->time.fn = fn;
	rx->time.mn = mn;
	rx->time_valid = 1;
	return rx;
}

/* ---- CRC-16 ----------------------------------------------------------- */

static uint16_t crc16_ref(const uint8_t *bits, unsigned n)
{
	uint16_t crc = 0xffff;
	unsigned i;

	for (i = 0; i < n; i++) {
		unsigned fb = ((crc >> 15) ^ bits[i]) & 1;

		crc = (uint16_t) (crc << 1);
		if (fb)
			crc ^= 0x1021;
	}
	return crc;
}

static void test_crc16(void)
{
	static const char msg[] = "123456789";
	uint8_t bits[400];
	rng_t r;
	unsigned i, n, t;
	uint16_t crc;

	begin("CRC-16 (EN 300 392-2 8.2.3.3)");
	for (i = 0; i < 72; i++)
		bits[i] = (msg[i / 8] >> (7 - i % 8)) & 1;
	/* CRC-16/CCITT-FALSE check value */
	check(crc16_ccitt_bits(bits, 72) == 0x29b1, "check value: 0x%04x", crc16_ccitt_bits(bits, 72));

	rng_seed(&r, 1);
	for (t = 0; t < 200; t++) {
		n = 1 + rng_u32(&r) % 300;
		rnd_bits(&r, bits, n);
		crc = crc16_ccitt_bits(bits, n);
		check(crc == crc16_ref(bits, n), "bitwise reference, %u bits", n);
		/* the transmitter appends the inverted CRC: the receiver then finds 0x1d0f */
		put(bits, n, 16, (uint16_t) ~crc);
		check(crc16_ccitt_bits(bits, n + 16) == 0x1d0f, "residue, %u bits", n);
		bits[rng_u32(&r) % (n + 16)] ^= 1;
		check(crc16_ccitt_bits(bits, n + 16) != 0x1d0f, "single error not detected");
	}
}

/* ---- scrambling ------------------------------------------------------- */

/* EN 300 392-2 8.2.5.2, equations 8.41 and 8.42 */
static void scramb_ref(uint16_t mcc, uint16_t mnc, uint8_t cc, uint8_t *out, unsigned n)
{
	static const uint8_t taps[] = { 1, 2, 4, 5, 7, 8, 10, 11, 12, 16, 22, 23, 26, 32 };
	uint8_t e[31], p[32 + 432];     /* p[k + 31] = p(k) */
	unsigned i, k;

	for (i = 0; i < 10; i++)
		e[1 + i] = (mcc >> (9 - i)) & 1;
	for (i = 0; i < 14; i++)
		e[11 + i] = (mnc >> (13 - i)) & 1;
	for (i = 0; i < 6; i++)
		e[25 + i] = (cc >> (5 - i)) & 1;
	p[0] = p[1] = 1;                /* p(-31), p(-30) */
	for (i = 0; i <= 29; i++)
		p[31 - i] = e[1 + i];   /* p(-i) = e(1 + i) */
	for (k = 1; k <= n; k++) {
		unsigned v = 0;

		for (i = 0; i < sizeof(taps); i++)
			v ^= p[k + 31 - taps[i]];
		p[k + 31] = (uint8_t) v;
		out[k - 1] = (uint8_t) v;
	}
}

static void test_scrambler(void)
{
	static const struct { uint16_t mcc, mnc; uint8_t cc; } cells[] = {
		{ 0, 0, 0 }, { 214, 30, 7 }, { 214, 999, 14 }, { 302, 2013, 29 }, { 1023, 16383, 63 }, { 1, 1, 1 },
	};
	uint8_t a[432], b[432], d[432];
	unsigned i, k;

	begin("scrambling sequence (8.2.5.2)");
	check(tetra_scramb_get_init(0, 0, 0) == SCRAMB_INIT, "BSCH sequence is the all-zero colour code");
	for (i = 0; i < sizeof(cells) / sizeof(cells[0]); i++) {
		uint32_t init = tetra_scramb_get_init(cells[i].mcc, cells[i].mnc, cells[i].cc);

		tetra_scramb_get_bits(init, a, 432);
		scramb_ref(cells[i].mcc, cells[i].mnc, cells[i].cc, b, 432);
		check(!memcmp(a, b, 432), "cell %u/%u/%u differs from the recurrence of the standard",
		      cells[i].mcc, cells[i].mnc, cells[i].cc);
		/* tetra_scramb_bits() XORs the same sequence */
		for (k = 0; k < 432; k++)
			d[k] = (k * 7 + i) & 1;
		tetra_scramb_bits(init, d, 432);
		for (k = 0; k < 432; k++)
			if (d[k] != (((k * 7 + i) & 1) ^ b[k]))
				break;
		check(k == 432, "tetra_scramb_bits");
	}
}

/* ---- RM(30,14) -------------------------------------------------------- */

static void test_rm3014(void)
{
	/* generator matrix rows as printed by osmo-tetra's conv_enc_test */
	static const uint32_t rows[14] = {
		0x20009b60, 0x10002de0, 0x0800fc20, 0x0400e03c, 0x0200983a, 0x01005436, 0x00802c2e,
		0x0040ffdf, 0x00208339, 0x001042b5, 0x000821ad, 0x00041273, 0x0002096b, 0x000104e7,
	};
	uint16_t col[14], info;
	unsigned i, t, mind = 30, ne;
	unsigned miscorr = 0;
	rng_t r;

	begin("RM(30,14) (8.2.3.2)");
	tetra_rm3014_dec_init(col);
	for (i = 0; i < 14; i++)
		check(tetra_rm3014_compute((uint16_t) (1u << (13 - i))) == rows[i], "generator row %u", i);
	for (i = 1; i < 16384; i++) {
		uint32_t cw = tetra_rm3014_compute((uint16_t) i), ref = 0;
		unsigned w = (unsigned) __builtin_popcount(cw), k;

		for (k = 0; k < 14; k++)
			if (i & (1u << (13 - k)))
				ref ^= rows[k];
		if (cw != ref) {
			check(0, "not linear at %u", i);
			break;
		}
		if (w < mind)
			mind = w;
	}
	check(mind == 8, "minimum distance %u", mind);

	rng_seed(&r, 2);
	for (ne = 0; ne <= 4; ne++) {
		unsigned bad = 0;

		for (t = 0; t < 3000; t++) {
			uint16_t in = (uint16_t) (rng_u32(&r) & 0x3fff);
			uint32_t cw = tetra_rm3014_compute(in), mask = 0;
			int n;

			while ((unsigned) __builtin_popcount(mask) < ne)
				mask |= 1u << (rng_u32(&r) % 30);
			n = tetra_rm3014_dec(col, cw ^ mask, &info);
			if (ne <= 3)
				bad += n != (int) ne || info != in;
			else
				bad += n != -1;
		}
		check(bad == 0, "%u bit errors: %u of 3000 decoded wrongly", ne, bad);
	}
	/* beyond the minimum distance bound a wrong codeword can be accepted;
	 * count how often for the README */
	for (t = 0; t < 20000; t++) {
		uint16_t in = (uint16_t) (rng_u32(&r) & 0x3fff);
		uint32_t cw = tetra_rm3014_compute(in), mask = 0;

		while (__builtin_popcount(mask) < 6)
			mask |= 1u << (rng_u32(&r) % 30);
		if (tetra_rm3014_dec(col, cw ^ mask, &info) >= 0 && info != in)
			miscorr++;
	}
	printf("    6 bit errors: wrong codeword accepted in %u of 20000 trials\n", miscorr);
	check(miscorr < 20000 / 5, "too many mis-corrections");
}

/* ---- interleaver ------------------------------------------------------ */

static void test_interleaver(void)
{
	static const struct { unsigned K, a; } p[] = { { 120, 11 }, { 216, 101 }, { 432, 103 } };
	uint8_t in[432], out[432], back[432];
	unsigned i, t;
	rng_t r;

	begin("block interleaver (8.2.4.1)");
	rng_seed(&r, 3);
	for (t = 0; t < 3; t++) {
		unsigned K = p[t].K, a = p[t].a, bad = 0;

		for (i = 0; i < K; i++)
			in[i] = (uint8_t) (rng_u32(&r) & 0xff);
		block_interleave(K, a, in, out);
		/* b4(k) = b3(i), k = 1 + ((a * i) mod K), i = 1..K */
		for (i = 1; i <= K; i++)
			bad += out[(a * i) % K] != in[i - 1];
		check(bad == 0, "(%u,%u): %u positions differ from the formula", K, a, bad);
		block_deinterleave(K, a, out, back);
		check(!memcmp(in, back, K), "(%u,%u): round trip", K, a);
	}
}

/* ---- RCPC code and Viterbi decoder ------------------------------------ */

static void cch_encode(const uint8_t *type2, unsigned n2, unsigned n3, uint8_t *type3)
{
	uint8_t mother[4 * 288];
	struct conv_enc_state ces;

	conv_enc_init(&ces);
	conv_enc_input(&ces, (uint8_t *) type2, (int) n2, mother);
	get_punctured_rate(TETRA_RCPC_PUNCT_2_3, mother, (int) n3, type3);
}

static void cch_decode(const int8_t *soft3, unsigned n2, unsigned n3, uint8_t *type2)
{
	int8_t dp[4 * 288];
	uint16_t dec[288];

	memset(dp, 0, sizeof(dp));
	tetra_rcpc_depunct(TETRA_RCPC_PUNCT_2_3, (const uint8_t *) soft3, (int) n3, (uint8_t *) dp);
	tetra_viterbi_cch(dp, type2, (int) n2, dec);
}

static void test_rcpc_viterbi(void)
{
	uint8_t t1[268], t2[288], t3[432], t5[432], ref5[432], ref2[288], ref1[268], out[288];
	int8_t soft[432];
	unsigned i, t, bad, n_err;
	rng_t r;

	begin("RCPC rate 2/3 + Viterbi (8.2.3.1)");
	str2bits(vec_schf_type2, ref2);
	str2bits(vec_schf_type5, ref5);
	str2bits(vec_schf_type1, ref1);

	/* the complete encoder chain reproduces the upstream vector */
	tetra_gen_encode_cch(ref1, 2, SCRAMB_INIT, t5);
	check(!memcmp(t5, ref5, 432), "encoder chain: type-5 bits differ from the osmo-tetra vector");

	/* decoder chain on the upstream type-5 bits */
	memcpy(t5, ref5, 432);
	tetra_scramb_bits(SCRAMB_INIT, t5, 432);
	block_deinterleave(432, 103, t5, t3);
	bits2soft(t3, soft, 432);
	cch_decode(soft, 288, 432, out);
	check(!memcmp(out, ref2, 288), "decoder: type-2 bits differ from the osmo-tetra vector");
	check(crc16_ccitt_bits(out, 284) == 0x1d0f, "CRC of the vector");

	/* every single channel bit error and every single erasure is corrected */
	bad = 0;
	for (i = 0; i < 432; i++) {
		soft[i] = (int8_t) -soft[i];
		cch_decode(soft, 288, 432, out);
		bad += memcmp(out, ref2, 288) != 0;
		soft[i] = 0;
		cch_decode(soft, 288, 432, out);
		bad += memcmp(out, ref2, 288) != 0;
		soft[i] = t3[i] ? -64 : 64;
	}
	check(bad == 0, "%u single errors / erasures not corrected", bad);

	/* random blocks of the three block sizes, hard decisions with errors */
	rng_seed(&r, 4);
	for (n_err = 0; n_err <= 2; n_err++) {
		static const struct { unsigned n1, n2, n3; } sz[] = { { 60, 80, 120 }, { 124, 144, 216 }, { 268, 288, 432 } };
		static const double ber[] = { 0.0, 0.01, 0.03 };
		unsigned ok = 0, total = 0;

		for (t = 0; t < 600; t++) {
			unsigned n1 = sz[t % 3].n1, n2 = sz[t % 3].n2, n3 = sz[t % 3].n3;
			uint16_t crc;

			memset(t2, 0, sizeof(t2));
			rnd_bits(&r, t1, n1);
			memcpy(t2, t1, n1);
			crc = (uint16_t) ~crc16_ccitt_bits(t2, n1);
			put(t2, n1, 16, crc);   /* + 4 tail bits */
			cch_encode(t2, n2, n3, t3);
			bits2soft(t3, soft, n3);
			for (i = 0; i < n3; i++)
				if (rng_uniform(&r) < ber[n_err])
					soft[i] = (int8_t) -soft[i];
			cch_decode(soft, n2, n3, out);
			total++;
			if (!memcmp(out, t2, n2))
				ok++;
			else
				/* an undetected error would be worse than a lost block */
				check(crc16_ccitt_bits(out, n1 + 16) != 0x1d0f, "wrong block with a good CRC");
		}
		printf("    hard decisions, channel BER %.0f %%: %u of %u blocks decoded\n", 100.0 * ber[n_err], ok, total);
		/* limits: osmo-tetra's own decoder (libosmocore) on the same kind of
		 * input decodes 84 % / 32 % of full slot blocks at 1 % / 3 %, this
		 * decoder 94 % / 47 %; the mix of block sizes here is easier */
		check(n_err ? ok * 100 >= total * (n_err == 1 ? 92 : 50) : ok == total,
		      "BER %.2f: only %u of %u blocks", ber[n_err], ok, total);
	}
}

/* ---- lower MAC on the upstream burst ---------------------------------- */

static void test_lmac_burst(void)
{
	uint8_t burst[510], ref1[268], bb[30];
	int8_t soft[510];
	struct cap c;
	tetra_rx_t *rx;
	const tetra_rx_event_t *e;
	tetra_rx_status_t st;
	unsigned i;

	begin("lower MAC: osmo-tetra's continuous normal downlink burst");
	str2bits(vec_ndb_burst, burst);
	str2bits(vec_schf_type1, ref1);
	/* the upstream test leaves the broadcast block unscrambled; put in a
	 * real ACCESS-ASSIGN (header 0: two access fields, common control) */
	tetra_gen_encode_aach(0x0123 & 0x0fff, SCRAMB_INIT, bb);
	memcpy(burst + 230, bb, 14);
	memcpy(burst + 266, bb + 14, 16);
	bits2soft(burst, soft, 510);

	rx = rx_new(&c, TETRA_RX_FLAG_AACH_EVENTS | TETRA_RX_FLAG_BLOCK_EVENTS, 0, 0, 0, 3, 7, 21);
	tetra_lmac_slot(rx, soft, TETRA_TRAIN_NORM_1);
	e = cap_get(&c, TETRA_RX_EV_AACH, 0);
	check(e && e->u.aach.raw == 0x0123 && e->u.aach.corrected_bits == 0 && e->u.aach.header == 0 &&
	      e->time.tn == 3 && e->time.fn == 7 && e->time.mn == 21, "ACCESS-ASSIGN");
	e = cap_get(&c, TETRA_RX_EV_BLOCK, 1);
	check(e && e->u.block.chan == TETRA_RX_CH_SCH_F && e->u.block.crc_ok && e->u.block.n_bits == 268 &&
	      !memcmp(e->u.block.bits, ref1, 268), "SCH/F block differs from the osmo-tetra type-1 bits");
	tetra_rx_get_status(rx, &st);
	check(st.chan[TETRA_RX_CH_SCH_F].ok == 1 && st.chan[TETRA_RX_CH_AACH].ok == 1, "counters");
	check(st.time.valid && st.time.tn == 3 && st.time.fn == 7 && st.time.mn == 21, "status time");
	check(rx->time.tn == 4 && rx->time.fn == 7, "time advance");

	/* three bit errors in the broadcast block, a few in the SCH/F block */
	c.n = 0;
	rx->time.tn = 3;
	soft[231] = (int8_t) -soft[231];
	soft[240] = (int8_t) -soft[240];
	soft[270] = (int8_t) -soft[270];
	for (i = 20; i < 200; i += 37)
		soft[i] = (int8_t) -soft[i];
	tetra_lmac_slot(rx, soft, TETRA_TRAIN_NORM_1);
	e = cap_get(&c, TETRA_RX_EV_AACH, 0);
	check(e && e->u.aach.raw == 0x0123 && e->u.aach.corrected_bits == 3, "ACCESS-ASSIGN with 3 bit errors");
	e = cap_get(&c, TETRA_RX_EV_BLOCK, 1);
	check(e && e->u.block.crc_ok && !memcmp(e->u.block.bits, ref1, 268), "SCH/F block with bit errors");
	tetra_rx_destroy(rx);
}

/* ---- BSCH ------------------------------------------------------------- */

static void test_bsch(void)
{
	uint8_t sync[60], sb5[120], hd1[124], hd5[216], bb[30], burst[510];
	int8_t soft[510];
	struct cap c;
	tetra_rx_t *rx;
	const tetra_rx_event_t *e;
	tetra_rx_config_t cfg;
	uint32_t init = tetra_scramb_get_init(214, 30, 7);
	unsigned pos = 0;

	begin("BSCH: SYNC PDU (21.4.4.2) and D-MLE-SYNC (18.4.2.1) in a synchronisation burst");
	pos = put(sync, pos, 4, 0);     /* system code */
	pos = put(sync, pos, 6, 7);     /* colour code */
	pos = put(sync, pos, 2, 2);     /* timeslot number: TN3 */
	pos = put(sync, pos, 5, 17);    /* frame number */
	pos = put(sync, pos, 6, 42);    /* multiframe number */
	pos = put(sync, pos, 2, 0);     /* sharing mode */
	pos = put(sync, pos, 3, 0);     /* TS reserved frames */
	pos = put(sync, pos, 1, 1);     /* U-plane DTX */
	pos = put(sync, pos, 1, 0);     /* frame 18 extension */
	pos = put(sync, pos, 1, 0);     /* reserved */
	pos = put(sync, pos, 10, 214);  /* MCC */
	pos = put(sync, pos, 14, 30);   /* MNC */
	pos = put(sync, pos, 2, 1);     /* neighbour cell broadcast */
	pos = put(sync, pos, 2, 2);     /* cell load CA */
	pos = put(sync, pos, 1, 1);     /* late entry supported */
	check(pos == 60, "SYNC PDU length");
	tetra_gen_encode_cch(sync, 0, SCRAMB_INIT, sb5);
	/* second half slot: a Null PDU, scrambled with the colour code of the cell */
	memset(hd1, 0, sizeof(hd1));
	pos = put(hd1, 0, 7, 0);
	pos = put(hd1, pos, 6, 2);
	pos = put(hd1, pos, 3, 0);
	hd1[16] = 1;
	tetra_gen_encode_cch(hd1, 1, init, hd5);
	tetra_gen_encode_aach(0, init, bb);
	build_sync_c_d_burst(burst, sb5, bb, hd5);
	bits2soft(burst, soft, 510);

	memset(&c, 0, sizeof(c));
	memset(&cfg, 0, sizeof(cfg));
	cfg.event_cb = on_event;
	cfg.user = &c;
	cfg.flags = TETRA_RX_FLAG_BLOCK_EVENTS;
	rx = tetra_rx_create(&cfg);
	tetra_lmac_slot(rx, soft, TETRA_TRAIN_SYNC);
	e = cap_get(&c, TETRA_RX_EV_BSCH, 0);
	check(e && e->u.bsch.mcc == 214 && e->u.bsch.mnc == 30 && e->u.bsch.colour_code == 7 &&
	      e->u.bsch.tn == 3 && e->u.bsch.fn == 17 && e->u.bsch.mn == 42 && e->u.bsch.u_plane_dtx == 1 &&
	      e->u.bsch.neighbour_cell_broadcast == 1 && e->u.bsch.cell_load == 2 && e->u.bsch.late_entry == 1 &&
	      e->u.bsch.changed == 1 && e->time.valid && e->time.tn == 3, "BSCH event");
	/* the other blocks of the same burst are decoded with the new cell */
	check(rx->st.chan[TETRA_RX_CH_AACH].ok == 1 && rx->st.chan[TETRA_RX_CH_SB2].ok == 1,
	      "broadcast block / second half slot of the first burst");
	check(rx->time.tn == 4 && rx->time.fn == 17 && rx->time.mn == 42, "time after the slot");

	/* Direct Mode synchronisation bursts (system code 1xxx) are not a cell */
	c.n = 0;
	tetra_rx_reset(rx);
	sync[0] = 1;
	tetra_gen_encode_cch(sync, 0, SCRAMB_INIT, sb5);
	build_sync_c_d_burst(burst, sb5, bb, hd5);
	bits2soft(burst, soft, 510);
	tetra_lmac_slot(rx, soft, TETRA_TRAIN_SYNC);
	check(cap_count(&c, TETRA_RX_EV_BSCH) == 0 && !rx->cell_valid && rx->st.chan[TETRA_RX_CH_BSCH].ok == 1,
	      "DMO system code accepted");
	tetra_rx_destroy(rx);
}

/* ---- TCH/S ------------------------------------------------------------ */

static void test_tch(void)
{
	static const uint8_t punct1[6] = { 1, 1, 0, 1, 0, 0 };
	static const uint8_t punct2[12] = { 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0 };
	uint8_t t1[274], t4[432], codec[274], back[274], seen[274];
	int8_t soft[432];
	struct cap c;
	tetra_rx_t *rx;
	const tetra_rx_event_t *e;
	unsigned i, k, g, t, bad, n1 = 0, n2 = 0;
	rng_t r;

	begin("TCH/S channel coding and speech frame bit order");
	rng_seed(&r, 5);

	/* parity: matrix form in the decoder, CRC polynomial form in the generator */
	bad = 0;
	for (t = 0; t < 2000; t++) {
		rnd_bits(&r, t1, 60);
		bad += tetra_tch_parity(t1) != tetra_gen_tch_crc(t1);
	}
	check(bad == 0, "class 2 parity: %u of 2000 differ", bad);

	/* puncturing: 112 class 1 bits at rate 8/12, 72 class 2 bits at 8/18 */
	bad = 0;
	for (k = 0; k < 184; k++)
		for (g = 0; g < 3; g++) {
			int keep = k < 112 ? punct1[(k % 2) * 3 + g] : punct2[((k - 112) % 4) * 3 + g];

			bad += tetra_tch_punct(k, g) != keep;
			if (keep && k < 112)
				n1++;
			else if (keep)
				n2++;
		}
	check(bad == 0 && n1 == 168 && n2 == 162, "puncturing pattern (%u, %u, %u)", bad, n1, n2);

	/* bit order: type-1 [class 0 | class 1 | class 2], the two frames bit
	 * interleaved, maps one to one onto 2 x 137 codec bits */
	memset(seen, 0, sizeof(seen));
	bad = 0;
	for (i = 0; i < 274; i++) {
		unsigned set = 0, where = 0;

		memset(t1, 0, sizeof(t1));
		memset(codec, 0xff, sizeof(codec));
		t1[i] = 1;
		tetra_acelp_type2_to_codec(t1, codec);
		for (k = 0; k < 274; k++) {
			if (codec[k] > 1)
				bad++;          /* position never written */
			else if (codec[k]) {
				set++;
				where = k;
			}
		}
		if (set != 1 || seen[where] || where / 137 != (i & 1))
			bad++;
		else
			seen[where] = 1;
	}
	check(bad == 0, "type-1 to codec order is not a bijection (%u)", bad);
	rnd_bits(&r, t1, 274);
	tetra_acelp_type2_to_codec(t1, codec);
	tetra_acelp_codec_to_acelp(codec, back);
	check(!memcmp(t1, back, 274), "codec order round trip");

	/* full slot: encoder written from the polynomials -> decoder */
	rx = rx_new(&c, 0, 214, 30, 7, 2, 5, 1);
	for (t = 0; t < 50; t++) {
		rnd_bits(&r, t1, 274);
		tetra_gen_encode_tch(t1, t4);
		bits2soft(t4, soft, 432);
		/* errors: any single bit of the coded classes is corrected */
		if (t) {
			unsigned row = 6 + rng_u32(&r) % 18, col = rng_u32(&r) % 18;

			if (row * 18 + col >= 102)
				soft[col * 24 + row] = (int8_t) -soft[col * 24 + row];
		}
		c.n = 0;
		rx->cur.is_traffic = 9;
		tetra_tch_full_slot(rx, soft);
		tetra_acelp_type2_to_codec(t1, codec);
		e = cap_get(&c, TETRA_RX_EV_TRAFFIC, 0);
		check(e && e->u.traffic.crc_ok && e->u.traffic.n_frames == 2 && !e->u.traffic.bfi[0] &&
		      !e->u.traffic.bfi[1] && e->u.traffic.usage_marker == 9 && e->u.traffic.n_soft == 432 &&
		      !memcmp(e->u.traffic.frame, codec, 274), "TCH/S slot %u", t);
	}
	/* errors in the class 2 bits that the decoder cannot repair set the BFI */
	bad = 0;
	for (t = 0; t < 200; t++) {
		rnd_bits(&r, t1, 274);
		tetra_gen_encode_tch(t1, t4);
		for (i = 0; i < 432; i++)
			if (rng_uniform(&r) < 0.2)
				t4[i] ^= 1;
		bits2soft(t4, soft, 432);
		c.n = 0;
		rx->cur.is_traffic = 9;
		tetra_tch_full_slot(rx, soft);
		e = cap_get(&c, TETRA_RX_EV_TRAFFIC, 0);
		if (!e || (e->u.traffic.crc_ok != !e->u.traffic.bfi[0]))
			bad++;
		else if (e->u.traffic.crc_ok)
			bad += 100;     /* 8 bit parity: about 1 in 256 slips through */
	}
	check(bad < 500, "BFI at 20 %% channel BER (%u)", bad);
	tetra_rx_destroy(rx);
}

/* ---- AACH lost on a traffic slot -------------------------------------- */

static void traffic_slot(tetra_rx_t *rx, rng_t *r, uint16_t aach, int break_aach)
{
	uint8_t t1[274], t4[432], bb[30], burst[510];
	int8_t soft[510];
	unsigned i;

	rnd_bits(r, t1, 274);
	tetra_gen_encode_tch(t1, t4);
	tetra_scramb_bits(rx->scramb_init, t4, 432);
	tetra_gen_encode_aach(aach, rx->scramb_init, bb);
	if (break_aach)
		for (i = 0; i < 4; i++)
			bb[3 + 7 * i] ^= 1;     /* 4 errors: never decodable (d = 8) */
	build_norm_c_d_burst(burst, t4, bb, t4 + 216, 0);
	bits2soft(burst, soft, 510);
	tetra_lmac_slot(rx, soft, TETRA_TRAIN_NORM_1);
	/* the other three timeslots are not demodulated */
	for (i = 0; i < 3; i++)
		tetra_lmac_slot(rx, NULL, TETRA_TRAIN_NORM_1);
}

static void test_aach_lost(void)
{
	/* header 1: DL usage marker (6 bit) + access field; marker 9 = traffic */
	const uint16_t traffic = (uint16_t) ((1u << 12) | (9u << 6) | 0x15);
	const uint16_t unalloc = (uint16_t) ((1u << 12) | (0u << 6) | 0x15);
	struct cap c;
	tetra_rx_t *rx;
	tetra_rx_status_t st;
	rng_t r;
	unsigned i;

	begin("traffic slot whose ACCESS-ASSIGN is lost");
	rng_seed(&r, 6);
	rx = rx_new(&c, 0, 214, 30, 7, 2, 3, 1);

	traffic_slot(rx, &r, traffic, 1);
	check(cap_count(&c, TETRA_RX_EV_TRAFFIC) == 0 && rx->st.chan[TETRA_RX_CH_SCH_F].fail == 1,
	      "unknown slot must be tried as signalling");
	traffic_slot(rx, &r, traffic, 0);
	check(cap_count(&c, TETRA_RX_EV_TRAFFIC) == 1, "traffic slot");
	/* two frames without ACCESS-ASSIGN are bridged, the third is not */
	traffic_slot(rx, &r, traffic, 1);
	traffic_slot(rx, &r, traffic, 1);
	check(cap_count(&c, TETRA_RX_EV_TRAFFIC) == 3 && rx->st.chan[TETRA_RX_CH_TCH_S].ok == 3 &&
	      c.ev[c.n - 1].u.traffic.usage_marker == 9 && c.ev[c.n - 1].u.traffic.crc_ok, "two frames bridged");
	traffic_slot(rx, &r, traffic, 1);
	check(cap_count(&c, TETRA_RX_EV_TRAFFIC) == 3, "third frame without ACCESS-ASSIGN");
	traffic_slot(rx, &r, traffic, 0);
	check(cap_count(&c, TETRA_RX_EV_TRAFFIC) == 4, "traffic again");
	tetra_rx_get_status(rx, &st);
	check(st.slot[1].valid && st.slot[1].dl_usage == 9 && st.chan[TETRA_RX_CH_AACH].fail == 4, "status");

	/* frame 18 of a traffic timeslot is signalling whatever was before */
	rx->time.tn = 2;
	rx->time.fn = 18;
	c.n = 0;
	traffic_slot(rx, &r, traffic, 1);
	check(cap_count(&c, TETRA_RX_EV_TRAFFIC) == 0, "frame 18 taken for traffic");
	check(rx->time.tn == 2 && rx->time.fn == 1 && rx->time.mn == 2, "multiframe wrap");

	/* the call ends: the slot is unallocated at once (ACCESS-ASSIGN readable) */
	for (i = 0; i < 2; i++)
		traffic_slot(rx, &r, traffic, 0);
	c.n = 0;
	traffic_slot(rx, &r, unalloc, 0);
	check(cap_count(&c, TETRA_RX_EV_TRAFFIC) == 0, "unallocated slot taken for traffic");
	tetra_rx_get_status(rx, &st);
	check(st.slot[1].dl_usage == TETRA_RX_DL_UNALLOCATED, "status after the call");
	tetra_rx_destroy(rx);
}

/* ---- TDMA time -------------------------------------------------------- */

static void test_time(void)
{
	struct tetra_tdma_time t = { 0, 0, 1, 1, 1 }, u;
	unsigned i, k, bad = 0;
	static const uint32_t steps[] = { 2, 3, 4, 5, 8, 71, 72, 73, 4319, 4320, 4321, 100000 };
	rng_t r;

	begin("TDMA time");
	for (i = 0; i < 4320; i++) {
		unsigned tn = i % 4 + 1, fn = (i / 4) % 18 + 1, mn = i / 72 + 1;

		bad += t.tn != tn || t.fn != fn || t.mn != mn;
		tetra_rx_time_add(&t, 1);
	}
	check(bad == 0, "%u of 4320 slots wrong", bad);
	check(t.tn == 1 && t.fn == 1 && t.mn == 1, "hyperframe wrap: %u/%u/%u", (unsigned) t.tn, (unsigned) t.fn, (unsigned) t.mn);

	rng_seed(&r, 7);
	bad = 0;
	for (i = 0; i < 300; i++) {
		uint32_t n = steps[i % (sizeof(steps) / sizeof(steps[0]))];

		t.tn = 1 + rng_u32(&r) % 4;
		t.fn = 1 + rng_u32(&r) % 18;
		t.mn = 1 + rng_u32(&r) % 60;
		u = t;
		tetra_rx_time_add(&u, n);
		for (k = 0; k < n; k++)
			tetra_rx_time_add(&t, 1);
		bad += t.tn != u.tn || t.fn != u.fn || t.mn != u.mn || u.tn < 1 || u.tn > 4;
	}
	check(bad == 0, "adding several slots at once: %u wrong", bad);
}

/* ---- CMCE parser ------------------------------------------------------ */

static void test_cmce(void)
{
	uint8_t b[400];
	tetra_rx_cmce_t c;
	unsigned n, i;

	begin("CMCE PDUs written from EN 300 392-2 clause 14.7.1");

	/* D-TX CEASED: type, call identifier, transmission request permission, O-bit */
	n = put(b, 0, 5, TETRA_RX_CMCE_D_TX_CEASED);
	n = put(b, n, 14, 0x2a51);
	n = put(b, n, 1, 1);
	n = put(b, n, 1, 0);
	tetra_cmce_parse(&c, b, n);
	check(c.parse_ok && c.call_id == 0x2a51 && c.tx_req_permission == 1 &&
	      c.fields == (TETRA_RX_CMCE_F_CALL_ID | TETRA_RX_CMCE_F_TX_REQ_PERM), "D-TX CEASED");
	tetra_cmce_parse(&c, b, n - 1);
	check(!c.parse_ok && c.fields == 0 && c.call_id == 0, "truncated PDU accepted");
	b[n] = 0;
	tetra_cmce_parse(&c, b, n + 1);
	check(!c.parse_ok && c.fields == 0, "PDU with a surplus bit accepted");

	/* D-RELEASE with a notification indicator (type 2 element) */
	n = put(b, 0, 5, TETRA_RX_CMCE_D_RELEASE);
	n = put(b, n, 14, 9421);
	n = put(b, n, 5, 13);
	n = put(b, n, 1, 1);            /* O-bit */
	n = put(b, n, 1, 1);            /* P-bit: notification indicator */
	n = put(b, n, 6, 33);
	n = put(b, n, 1, 0);            /* M-bit: no type 3 elements */
	tetra_cmce_parse(&c, b, n);
	check(c.parse_ok && c.call_id == 9421 && c.disconnect_cause == 13 && c.notification == 33 &&
	      (c.fields & TETRA_RX_CMCE_F_NOTIFICATION), "D-RELEASE");

	/* D-SETUP: group call, clear speech, with calling party TSI */
	n = put(b, 0, 5, TETRA_RX_CMCE_D_SETUP);
	n = put(b, n, 14, 77);
	n = put(b, n, 4, 9);            /* call time-out */
	n = put(b, n, 1, 0);            /* hook method */
	n = put(b, n, 1, 0);            /* simplex */
	n = put(b, n, 3, 0);            /* circuit mode: speech */
	n = put(b, n, 1, 0);            /* clear */
	n = put(b, n, 2, 1);            /* point to multipoint */
	n = put(b, n, 2, 3);
	n = put(b, n, 2, 3);            /* transmission granted to another user */
	n = put(b, n, 1, 0);
	n = put(b, n, 4, 5);            /* call priority */
	n = put(b, n, 1, 1);            /* O-bit */
	n = put(b, n, 1, 0);            /* notification indicator */
	n = put(b, n, 1, 0);            /* temporary address */
	n = put(b, n, 1, 1);            /* calling party */
	n = put(b, n, 2, 2);            /* SSI + extension */
	n = put(b, n, 24, 1234567);
	n = put(b, n, 10, 214);
	n = put(b, n, 14, 30);
	n = put(b, n, 1, 0);            /* M-bit */
	tetra_cmce_parse(&c, b, n);
	check(c.parse_ok && c.call_id == 77 && c.call_timeout == 9 && c.duplex == 0 && c.circuit_mode == 0 &&
	      c.e2e_encryption == 0 && c.comm_type == 1 && c.tx_grant == 3 && c.call_priority == 5 &&
	      c.party.present == 2 && c.party.ssi == 1234567 && c.party.mcc == 214 && c.party.mnc == 30,
	      "D-SETUP");
	b[5 + 14 + 6 + 3] = 1;          /* end-to-end encrypted */
	tetra_cmce_parse(&c, b, n);
	check(c.parse_ok && c.e2e_encryption == 1, "D-SETUP, end-to-end encryption flag");

	/* D-TX GRANTED with the transmitting party */
	n = put(b, 0, 5, TETRA_RX_CMCE_D_TX_GRANTED);
	n = put(b, n, 14, 77);
	n = put(b, n, 2, 0);            /* granted */
	n = put(b, n, 1, 0);
	n = put(b, n, 1, 0);            /* encryption control: clear */
	n = put(b, n, 1, 0);            /* reserved */
	n = put(b, n, 1, 1);            /* O-bit */
	n = put(b, n, 1, 0);            /* notification indicator */
	n = put(b, n, 1, 1);            /* transmitting party */
	n = put(b, n, 2, 1);
	n = put(b, n, 24, 4711);
	n = put(b, n, 1, 0);
	tetra_cmce_parse(&c, b, n);
	check(c.parse_ok && c.call_id == 77 && c.tx_grant == 0 && c.enc_control == 0 && c.party.present == 1 &&
	      c.party.ssi == 4711, "D-TX GRANTED");

	/* D-STATUS */
	n = put(b, 0, 5, TETRA_RX_CMCE_D_STATUS);
	n = put(b, n, 2, 1);
	n = put(b, n, 24, 0xabcdef);
	n = put(b, n, 16, 0x8003);
	n = put(b, n, 1, 0);
	tetra_cmce_parse(&c, b, n);
	check(c.parse_ok && c.party.ssi == 0xabcdef && c.status == 0x8003 && (c.fields & TETRA_RX_CMCE_F_STATUS),
	      "D-STATUS");
	/* reserved calling party type identifier: layout unknown */
	put(b, 5, 2, 3);
	tetra_cmce_parse(&c, b, n);
	check(!c.parse_ok, "reserved party type accepted");

	/* D-SDS-DATA type 4: simple text messaging, Latin-1 */
	n = put(b, 0, 5, TETRA_RX_CMCE_D_SDS_DATA);
	n = put(b, n, 2, 1);
	n = put(b, n, 24, 1000);
	n = put(b, n, 2, 3);
	n = put(b, n, 11, 16 + 5 * 8);
	n = put(b, n, 8, 0x02);         /* protocol identifier: simple text messaging */
	n = put(b, n, 1, 0);
	n = put(b, n, 7, 1);            /* ISO 8859-1 */
	for (i = 0; i < 5; i++)
		n = put(b, n, 8, (uint8_t) "Gr\xfc\xdf" "e"[i]);
	n = put(b, n, 1, 0);
	tetra_cmce_parse(&c, b, n);
	check(c.parse_ok && c.sds_type == 3 && c.sds_bits == 56 && c.protocol_id == 0x02 && c.text_coding == 1 &&
	      (c.fields & TETRA_RX_CMCE_F_TEXT) && !strcmp(c.text, "Gr\xc3\xbc\xc3\x9f" "e") &&
	      c.sds_data == b + 5 + 26 + 2 + 11, "D-SDS-DATA text: \"%s\"", c.text);
	/* length indicator beyond the PDU */
	put(b, 5 + 26 + 2, 11, 57);
	tetra_cmce_parse(&c, b, n);
	check(!c.parse_ok, "SDS length beyond the PDU accepted");

	/* D-SDS-DATA type 2 (32 bit user defined data) */
	n = put(b, 0, 5, TETRA_RX_CMCE_D_SDS_DATA);
	n = put(b, n, 2, 1);
	n = put(b, n, 24, 1000);
	n = put(b, n, 2, 1);
	n = put(b, n, 32, 0xdeadbeef);
	n = put(b, n, 1, 0);
	tetra_cmce_parse(&c, b, n);
	check(c.parse_ok && c.sds_type == 1 && c.sds_bits == 32 && !(c.fields & TETRA_RX_CMCE_F_TEXT) &&
	      bits_to_uint(c.sds_data, 32) == 0xdeadbeef, "D-SDS-DATA 32 bit");

	/* D-FACILITY and reserved types are not parsed */
	n = put(b, 0, 5, TETRA_RX_CMCE_D_FACILITY);
	n = put(b, n, 20, 0x12345);
	tetra_cmce_parse(&c, b, n);
	check(!c.parse_ok && c.fields == 0, "D-FACILITY parsed");
}

/* ---- upper MAC -------------------------------------------------------- */

/* MAC-RESOURCE header with an SSI, table 21.55: 43 bits */
static unsigned mac_resource(uint8_t *b, unsigned pos, int fill, unsigned enc, unsigned len_ind, uint32_t ssi)
{
	pos = put(b, pos, 2, 0);        /* MAC-RESOURCE */
	pos = put(b, pos, 1, (uint32_t) fill);
	pos = put(b, pos, 1, 0);        /* position of grant */
	pos = put(b, pos, 2, enc);
	pos = put(b, pos, 1, 0);        /* random access flag */
	pos = put(b, pos, 6, len_ind);
	pos = put(b, pos, 3, 1);        /* address type: SSI */
	pos = put(b, pos, 24, ssi);
	pos = put(b, pos, 1, 0);        /* power control flag */
	pos = put(b, pos, 1, 0);        /* slot granting flag */
	pos = put(b, pos, 1, 0);        /* channel allocation flag */
	return pos;
}

static unsigned null_pdu(uint8_t *b, unsigned pos)
{
	pos = put(b, pos, 7, 0);
	pos = put(b, pos, 6, 2);        /* two octets */
	pos = put(b, pos, 3, 0);        /* address type: null PDU */
	return pos;
}

/* D-TX CEASED in BL-UDATA: 28 bit TM-SDU */
static unsigned tm_sdu_tx_ceased(uint8_t *b, unsigned pos, unsigned call_id)
{
	pos = put(b, pos, 4, 2);        /* BL-UDATA */
	pos = put(b, pos, 3, TETRA_RX_PD_CMCE);
	pos = put(b, pos, 5, TETRA_RX_CMCE_D_TX_CEASED);
	pos = put(b, pos, 14, call_id);
	pos = put(b, pos, 1, 0);
	pos = put(b, pos, 1, 0);
	return pos;
}

/* [43 header | 28 TM-SDU | 1 fill bit] = 9 octets */
static unsigned pdu_tx_ceased(uint8_t *b, unsigned pos, uint32_t ssi, unsigned call_id)
{
	pos = mac_resource(b, pos, 1, 0, 9, ssi);
	pos = tm_sdu_tx_ceased(b, pos, call_id);
	return put(b, pos, 1, 1);
}

static void test_umac(void)
{
	uint8_t b[300], sds[200];
	struct cap c;
	tetra_rx_t *rx;
	const tetra_rx_event_t *e;
	const tetra_rx_ev_signal_t *s;
	unsigned n, i, k, tl;
	uint32_t fcs;
	rng_t r;

	begin("upper MAC: MAC-RESOURCE / MAC-FRAG / MAC-END, LLC, MLE");
	rng_seed(&r, 8);
	rx = rx_new(&c, 0, 214, 30, 7, 1, 2, 1);

	/* one PDU and a Null PDU in a half slot */
	memset(b, 0, sizeof(b));
	n = pdu_tx_ceased(b, 0, 0x123456, 0x1555);
	check(n == 72, "test PDU length %u", n);
	n = null_pdu(b, n);
	b[n] = 1;
	tetra_umac_block(rx, b, 124, TETRA_RX_CH_SCH_HD);
	e = cap_get(&c, TETRA_RX_EV_SIGNAL, 0);
	s = e ? &e->u.signal : NULL;
	check(c.n == 1 && s && s->mac_pdu == TETRA_RX_MAC_RESOURCE && s->lchan == TETRA_RX_CH_SCH_HD &&
	      s->addr.type == TETRA_RX_ADDR_SSI && s->addr.ssi == 0x123456 && !s->encrypted && !s->fragmented &&
	      s->have_llc && s->llc_pdu == 2 && !s->llc_fcs_present && s->have_mle && s->pdisc == TETRA_RX_PD_CMCE &&
	      s->pdu_type == TETRA_RX_CMCE_D_TX_CEASED && s->have_cmce && s->cmce.parse_ok &&
	      s->cmce.call_id == 0x1555 && s->sdu_bits == 24 && !strcmp(s->pdu_name, "D-TX CEASED") &&
	      e->time.tn == 1 && e->time.fn == 2, "single MAC-RESOURCE (%u events, name %s)", c.n, s ? s->pdu_name : "");

	/* three PDUs in a full slot */
	c.n = 0;
	memset(b, 0, sizeof(b));
	n = 0;
	for (i = 0; i < 3; i++)
		n = pdu_tx_ceased(b, n, 100 + i, 200 + i);
	n = null_pdu(b, n);
	b[n] = 1;
	tetra_umac_block(rx, b, 268, TETRA_RX_CH_SCH_F);
	check(c.n == 3, "three PDUs in one block: %u events", c.n);
	for (i = 0; i < 3 && i < c.n; i++)
		check(c.ev[i].u.signal.addr.ssi == 100 + i && c.ev[i].u.signal.cmce.call_id == 200 + i &&
		      c.ev[i].u.signal.lchan == TETRA_RX_CH_SCH_F, "PDU %u of three", i);

	/* the last PDU of a block may end within its last octet (23.4.3.3):
	 * 116 bits + fill bits, length indication 16 octets in a 124 bit block */
	c.n = 0;
	memset(b, 0, sizeof(b));
	n = mac_resource(b, 0, 1, 0, 16, 42);
	n = put(b, n, 4, 2);
	n = put(b, n, 3, TETRA_RX_PD_CMCE);
	n = put(b, n, 5, TETRA_RX_CMCE_D_SDS_DATA);
	n = put(b, n, 2, 1);
	n = put(b, n, 24, 99);
	n = put(b, n, 2, 1);
	n = put(b, n, 32, 0x12345678);
	n = put(b, n, 1, 0);
	check(n == 116, "length %u", n);
	b[n] = 1;
	tetra_umac_block(rx, b, 124, TETRA_RX_CH_SCH_HD);
	s = c.n ? &c.ev[0].u.signal : NULL;
	check(c.n == 1 && s->cmce.parse_ok && s->cmce.sds_bits == 32 &&
	      bits_to_uint(s->cmce.sds_data, 32) == 0x12345678 && s->cmce.party.ssi == 99, "PDU cut in its last octet");
	/* a length that does not fit the block is not a PDU */
	c.n = 0;
	put(b, 7, 6, 17);
	tetra_umac_block(rx, b, 124, TETRA_RX_CH_SCH_HD);
	check(c.n == 0, "PDU longer than the block accepted");

	/* fragmentation over three MAC blocks: 81 + 120 + 24 bits of TM-SDU */
	rnd_bits(&r, sds, 173);
	put(sds, 0, 8, 0x40);
	for (k = 0; k < 4; k++) {
		/* k = 0: complete; 1: aged out; 2: a block lost in between; 3: complete again */
		uint8_t tm[225], blk[124];

		n = put(tm, 0, 4, 2);
		n = put(tm, n, 3, TETRA_RX_PD_CMCE);
		n = put(tm, n, 5, TETRA_RX_CMCE_D_SDS_DATA);
		n = put(tm, n, 2, 1);
		n = put(tm, n, 24, 555);
		n = put(tm, n, 2, 3);
		n = put(tm, n, 11, 173);
		memcpy(tm + n, sds, 173);
		n = put(tm, n + 173, 1, 0);
		check(n == 225, "TM-SDU length %u", n);

		c.n = 0;
		mac_resource(blk, 0, 0, 0, 0x3f, 7000 + k);
		memcpy(blk + 43, tm, 81);
		tetra_umac_block(rx, blk, 124, TETRA_RX_CH_SCH_HD);
		tetra_umac_slot_end(rx, 1);
		if (k == 2)
			tetra_umac_block(rx, NULL, 0, TETRA_RX_CH_SCH_HD);

		n = put(blk, 0, 2, 1);          /* MAC-FRAG */
		n = put(blk, n, 1, 0);
		n = put(blk, n, 1, 0);          /* no fill bits */
		memcpy(blk + 4, tm + 81, 120);
		tetra_umac_block(rx, blk, 124, TETRA_RX_CH_SCH_HD);
		for (i = 0; i < (k == 1 ? 7u : 1u); i++)
			tetra_umac_slot_end(rx, 1);

		memset(blk, 0, sizeof(blk));
		n = put(blk, 0, 2, 1);          /* MAC-END */
		n = put(blk, n, 1, 1);
		n = put(blk, n, 1, 1);          /* fill bits */
		n = put(blk, n, 1, 0);          /* position of grant */
		n = put(blk, n, 6, 5);          /* 5 octets */
		n = put(blk, n, 1, 0);          /* slot granting flag */
		n = put(blk, n, 1, 0);          /* channel allocation flag */
		memcpy(blk + n, tm + 201, 24);
		n = put(blk, n + 24, 1, 1);
		n = null_pdu(blk, 40);
		tetra_umac_block(rx, blk, 124, TETRA_RX_CH_SCH_HD);
		tetra_umac_slot_end(rx, 1);

		s = c.n ? &c.ev[0].u.signal : NULL;
		if (k == 0 || k == 3)
			check(c.n == 1 && s->fragmented && s->addr.ssi == 7000 + k && s->cmce.parse_ok &&
			      s->cmce.sds_type == 3 && s->cmce.sds_bits == 173 && s->cmce.protocol_id == 0x40 &&
			      !memcmp(s->cmce.sds_data, sds, 173) && s->sdu_bits == 221 &&
			      s->lchan == TETRA_RX_CH_SCH_HD, "reassembly %u (%u events)", k, c.n);
		else
			check(c.n == 0, "incomplete reassembly %u delivered", k);
	}
	check(rx->st.frag_dropped == 2, "dropped fragments: %u", (unsigned) rx->st.frag_dropped);

	/* air interface encrypted PDU: header only */
	c.n = 0;
	memset(b, 0, sizeof(b));
	n = mac_resource(b, 0, 0, 1, 9, 0xfedcba);
	rnd_bits(&r, b + n, 29);
	n = null_pdu(b, 72);
	tetra_umac_block(rx, b, 124, TETRA_RX_CH_SCH_HD);
	s = c.n ? &c.ev[0].u.signal : NULL;
	check(c.n == 1 && s->encrypted && s->encryption_mode == 1 && s->addr.ssi == 0xfedcba && !s->have_llc &&
	      !s->have_mle && !s->have_cmce && s->sdu == NULL && s->sdu_bits == 0, "encrypted PDU");

	/* LLC frame check sequence, also for a TL-SDU shorter than 32 bits (annex C) */
	for (k = 0; k < 4; k++) {
		uint8_t tlsdu[64];

		tl = put(tlsdu, 0, 3, TETRA_RX_PD_CMCE);
		if (k < 2) {
			tl = put(tlsdu, tl, 5, TETRA_RX_CMCE_D_TX_CEASED);
			tl = put(tlsdu, tl, 14, 321);
			tl = put(tlsdu, tl, 2, 0);
		} else {
			tl = put(tlsdu, tl, 5, TETRA_RX_CMCE_D_STATUS);
			tl = put(tlsdu, tl, 2, 1);
			tl = put(tlsdu, tl, 24, 5);
			tl = put(tlsdu, tl, 16, 0x8001);
			tl = put(tlsdu, tl, 1, 0);
		}
		fcs = tetra_gen_fcs(tlsdu, tl);
		if (k & 1)
			fcs ^= 0x00010000;
		c.n = 0;
		memset(b, 0, sizeof(b));
		n = mac_resource(b, 0, 1, 0, (43 + 5 + tl + 32 + 8) / 8, 31);
		n = put(b, n, 4, 5);            /* BL-DATA with FCS */
		n = put(b, n, 1, 1);            /* N(S) */
		memcpy(b + n, tlsdu, tl);
		n = put(b, n + tl, 32, fcs);
		b[n] = 1;
		tetra_umac_block(rx, b, 268, TETRA_RX_CH_SCH_F);
		s = c.n ? &c.ev[0].u.signal : NULL;
		if (k & 1)
			check(c.n == 1 && s->have_llc && s->llc_pdu == 5 && s->llc_fcs_present && !s->llc_fcs_ok &&
			      !s->have_mle && !s->have_cmce, "bad FCS (%u bit TL-SDU)", tl);
		else
			check(c.n == 1 && s->llc_fcs_present && s->llc_fcs_ok && s->have_mle && s->cmce.parse_ok &&
			      s->sdu_bits == tl, "good FCS (%u bit TL-SDU)", tl);
	}
	{
		/* CRC-32/BZIP2 check value: for whole octets annex C is that CRC */
		static const char msg[] = "123456789";
		uint8_t mb[72];

		for (i = 0; i < 72; i++)
			mb[i] = (msg[i / 8] >> (7 - i % 8)) & 1;
		check(tetra_gen_fcs(mb, 72) == 0xfc891918u, "FCS check value 0x%08x", (unsigned) tetra_gen_fcs(mb, 72));
	}

	/* MM and unparsed CMCE PDUs: type and raw bits */
	c.n = 0;
	memset(b, 0, sizeof(b));
	n = mac_resource(b, 0, 1, 0, 9, 77);
	n = put(b, n, 4, 2);
	n = put(b, n, 3, TETRA_RX_PD_MM);
	n = put(b, n, 4, 5);                    /* D-LOCATION UPDATE ACCEPT */
	n = put(b, n, 17, 0x15a5a);
	b[n] = 1;
	n = mac_resource(b, 72, 1, 0, 9, 78);
	n = put(b, n, 4, 2);
	n = put(b, n, 3, TETRA_RX_PD_CMCE);
	n = put(b, n, 5, TETRA_RX_CMCE_D_FACILITY);
	n = put(b, n, 16, 0xa5a5);
	b[n] = 1;
	null_pdu(b, 144);
	tetra_umac_block(rx, b, 268, TETRA_RX_CH_SCH_F);
	check(c.n == 2, "MM + D-FACILITY: %u events", c.n);
	if (c.n == 2) {
		s = &c.ev[0].u.signal;
		check(s->have_mle && s->pdisc == TETRA_RX_PD_MM && s->pdu_type == 5 && !s->have_cmce &&
		      s->sdu_bits == 24 && bits_to_uint(s->sdu + 7, 17) == 0x15a5a &&
		      !strcmp(s->pdu_name, "D-LOCATION UPDATE ACCEPT"), "MM PDU (%s)", s->pdu_name);
		s = &c.ev[1].u.signal;
		check(s->have_cmce && !s->cmce.parse_ok && s->cmce.fields == 0 && s->pdu_type == TETRA_RX_CMCE_D_FACILITY &&
		      s->sdu_bits == 24 && bits_to_uint(s->sdu + 8, 16) == 0xa5a5, "unparsed CMCE PDU (%s)", s->pdu_name);
	}

	/* stealing channel: "second half slot stolen" is the length indication
	 * 111110 of the last MAC-RESOURCE of the first half slot, which may
	 * be a Null PDU (blocks as seen on a live network) */
	{
		static const struct {
			const char *what;
			int pdu, li, chan, stolen;
		} cases[] = {
			{ "Null PDU, second half not stolen", 0, 2, TETRA_RX_CH_STCH, 0 },
			{ "Null PDU with 111110", 0, 0x3e, TETRA_RX_CH_STCH, 1 },
			{ "PDU + Null PDU, second half not stolen", 1, 2, TETRA_RX_CH_STCH, 0 },
			{ "PDU + Null PDU with 111110", 1, 0x3e, TETRA_RX_CH_STCH, 1 },
			{ "Null PDU with 111110 outside the stealing channel", 0, 0x3e, TETRA_RX_CH_SCH_HD, 0 },
		};

		for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
			c.n = 0;
			memset(b, 0, sizeof(b));
			n = cases[i].pdu ? pdu_tx_ceased(b, 0, 0x123456, 0x1555) : 0;
			n = put(b, n, 7, 0);
			n = put(b, n, 6, (uint32_t) cases[i].li);
			n = put(b, n, 3, 0);
			b[n] = 1;
			rx->cur.blk2_stolen = 0;
			tetra_umac_block(rx, b, 124, (uint8_t) cases[i].chan);
			check(rx->cur.blk2_stolen == cases[i].stolen && c.n == (unsigned) cases[i].pdu, "%s: stolen %u, %u events",
			      cases[i].what, rx->cur.blk2_stolen, c.n);
		}
		/* in the PDU itself */
		c.n = 0;
		memset(b, 0, sizeof(b));
		n = mac_resource(b, 0, 1, 0, 0x3e, 77);
		n = tm_sdu_tx_ceased(b, n, 0x0aaa);
		b[n] = 1;
		rx->cur.blk2_stolen = 0;
		tetra_umac_block(rx, b, 124, TETRA_RX_CH_STCH);
		check(rx->cur.blk2_stolen == 1 && c.n == 1 && c.ev[0].u.signal.cmce.call_id == 0x0aaa,
		      "MAC-RESOURCE with 111110: stolen %u, %u events", rx->cur.blk2_stolen, c.n);
		rx->cur.blk2_stolen = 0;
	}

	/* carrier numbers */
	check(tetra_dl_carrier_hz(3, 3650, 3) == 391262500u && tetra_dl_carrier_hz(4, 1000, 0) == 425000000u &&
	      tetra_dl_carrier_hz(3, 3655, 2) == 391368750u, "carrier frequency");
	tetra_rx_destroy(rx);
}

int main(void)
{
	test_crc16();
	test_scrambler();
	test_rm3014();
	test_interleaver();
	test_rcpc_viterbi();
	test_lmac_burst();
	test_bsch();
	test_tch();
	test_aach_lost();
	test_time();
	test_cmce();
	test_umac();
	if (n_failed)
		printf("test_unit: %u of %u checks FAILED\n", n_failed, n_checks);
	else
		printf("test_unit: all passed (%u checks)\n", n_checks);
	return n_failed ? 1 : 0;
}
