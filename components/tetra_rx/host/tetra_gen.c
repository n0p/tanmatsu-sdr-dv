/* tetra_gen - synthetic TETRA TMO downlink generator for the tetra_rx tests
 * (see tetra_gen.h)
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "tetra_gen.h"
#include "dsp_util.h"

#include <lower_mac/crc_simple.h>
#include <lower_mac/tetra_conv_enc.h>
#include <lower_mac/tetra_interleave.h>
#include <lower_mac/tetra_rm3014.h>
#include <lower_mac/tetra_scramb.h>
#include <phy/tetra_burst.h>

/* vendored osmo-tetra tch_reordering.c (EN 300 395-2 table 4) */
void tetra_acelp_type2_to_codec(const uint8_t *in, uint8_t *out);

#define SCH_F_BITS  268
#define SCH_HD_BITS 124

/* ---- channel coding ------------------------------------------------------ */

static const struct {
	uint16_t n1, n2, n345, a;
} cch_par[3] = {
	{ 60, 80, 120, 11 },    /* BSCH */
	{ 124, 144, 216, 101 }, /* SCH/HD, BNCH, STCH */
	{ 268, 288, 432, 103 }, /* SCH/F */
};

void tetra_gen_encode_cch(const uint8_t *type1, int kind, uint32_t scramb_init, uint8_t *type5)
{
	uint8_t type2[288], mother[4 * 288], type3[432];
	struct conv_enc_state ces;
	unsigned int n1 = cch_par[kind].n1, n2 = cch_par[kind].n2, n345 = cch_par[kind].n345;
	uint16_t crc;
	unsigned int i;

	memset(type2, 0, sizeof(type2));
	memcpy(type2, type1, n1);
	crc = (uint16_t) ~crc16_ccitt_bits(type2, n1);
	for (i = 0; i < 16; i++)
		type2[n1 + i] = (crc >> (15 - i)) & 1;
	/* + 4 tail bits (zero) */
	conv_enc_init(&ces);
	conv_enc_input(&ces, type2, (int) n2, mother);
	get_punctured_rate(TETRA_RCPC_PUNCT_2_3, mother, (int) n345, type3);
	block_interleave(n345, cch_par[kind].a, type3, type5);
	tetra_scramb_bits(scramb_init, type5, (int) n345);
}

void tetra_gen_encode_aach(uint16_t info, uint32_t scramb_init, uint8_t *type5)
{
	static int init;
	uint32_t cw;
	unsigned int i;

	if (!init) {
		tetra_rm3014_init();
		init = 1;
	}
	cw = tetra_rm3014_compute(info);
	for (i = 0; i < 30; i++)
		type5[i] = (cw >> (29 - i)) & 1;
	tetra_scramb_bits(scramb_init, type5, 30);
}

/* 7 bit CRC with g(X) = 1 + X^3 + X^7 over the class 2 bits (bit i is the
 * coefficient of X^i) and one overall parity bit */
unsigned int tetra_gen_tch_crc(const uint8_t *c2)
{
	unsigned int r = 0, p = 0;
	int i;

	for (i = 59; i >= 0; i--) {
		unsigned int fb = ((r >> 6) ^ c2[i]) & 1;

		r = (r << 1) & 0x7f;
		if (fb)
			r ^= 0x09;
		p ^= c2[i] & 1;
	}
	for (i = 0; i < 7; i++)
		p ^= (r >> i) & 1;
	return r | (p << 7);
}

void tetra_gen_encode_tch(const uint8_t *type1, uint8_t *type4)
{
	/* transmitted bits of the rate 1/3 mother code: class 1 has a period of
	 * two input bits, class 2 of four */
	static const uint8_t punct1[6] = { 1, 1, 0, 1, 0, 0 };
	static const uint8_t punct2[12] = { 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0 };
	uint8_t u[184], t3[432];
	unsigned int par = tetra_gen_tch_crc(type1 + 214);
	unsigned int d1 = 0, d2 = 0, d3 = 0, d4 = 0;
	unsigned int n = 0, k, g, r, c;

	memcpy(t3, type1, 102);     /* class 0 is not protected */
	n = 102;
	memcpy(u, type1 + 102, 172);
	for (k = 0; k < 8; k++)
		u[172 + k] = (par >> k) & 1;
	memset(u + 180, 0, 4);      /* tail */

	for (k = 0; k < 184; k++) {
		unsigned int b = u[k] & 1;
		unsigned int out[3];

		out[0] = b ^ d1 ^ d2 ^ d3 ^ d4;  /* 1 + D + D2 + D3 + D4 */
		out[1] = b ^ d1 ^ d3 ^ d4;       /* 1 + D + D3 + D4 */
		out[2] = b ^ d2 ^ d4;            /* 1 + D2 + D4 */
		for (g = 0; g < 3; g++) {
			unsigned int keep = k < 112 ? punct1[(k % 2) * 3 + g]
						    : punct2[((k - 112) % 4) * 3 + g];
			if (keep)
				t3[n++] = (uint8_t) out[g];
		}
		d4 = d3;
		d3 = d2;
		d2 = d1;
		d1 = b;
	}
	assert(n == 432);
	/* 24 x 18 matrix interleaver: written row by row, sent column by column */
	for (r = 0; r < 24; r++)
		for (c = 0; c < 18; c++)
			type4[c * 24 + r] = t3[r * 18 + c];
}

uint32_t tetra_gen_fcs(const uint8_t *bits, unsigned int n)
{
	/* EN 300 392-2 annex C, written out literally: the first 32 bits (or
	 * all, if fewer) are complemented, M(x) x^32 is divided by G(x), the
	 * remainder is complemented */
	uint32_t crc = 0;
	unsigned int i;

	for (i = 0; i < n; i++) {
		uint32_t fb = ((crc >> 31) ^ bits[i] ^ (i < 32)) & 1;

		crc <<= 1;
		if (fb)
			crc ^= 0x04c11db7u;
	}
	return ~crc;
}

/* ---- bit writer ---------------------------------------------------------- */

struct bw {
	uint8_t *b;
	unsigned int n, max;
};

static void put(struct bw *w, unsigned int nbits, uint32_t v)
{
	unsigned int i;

	assert(w->n + nbits <= w->max);
	for (i = 0; i < nbits; i++)
		w->b[w->n++] = (v >> (nbits - 1 - i)) & 1;
}

static void put_bits(struct bw *w, const uint8_t *bits, unsigned int n)
{
	assert(w->n + n <= w->max);
	memcpy(w->b + w->n, bits, n);
	w->n += n;
}

/* ---- generator state ----------------------------------------------------- */

struct pend {
	uint8_t valid;
	uint32_t idx;               /* message index */
	uint16_t len, pos;          /* TM-SDU bits: total / already sent */
	uint8_t tm[TETRA_GEN_SDU_MAX + 64];
};

struct tetra_gen {
	tetra_gen_cfg_t cfg;
	rng_t rng;
	uint32_t scr;
	uint8_t tn, fn, mn;
	uint16_t hn;
	uint32_t index;
	tetra_gen_slot_t slot;
	tetra_gen_msg_t *msgs;
	size_t n_msgs, cap;
	struct pend pend[4];        /* next message of each timeslot */
	struct pend tmp;            /* one-off messages (half slots, stealing) */
	uint8_t frag[4];            /* pend[] is being sent in fragments */
	uint32_t rot[4], kind_rot, small_rot, steal_rot, call_rot;
};

static uint32_t rnd(tetra_gen_t *g, uint32_t n)
{
	return rng_u32(&g->rng) % n;
}

static void rnd_bits(tetra_gen_t *g, struct bw *w, unsigned int n)
{
	while (n--)
		put(w, 1, rng_u32(&g->rng) >> 31);
}

/* ---- layer 3 PDUs -------------------------------------------------------- */

static unsigned int utf8_enc(char *out, uint32_t cp)
{
	if (cp < 0x80) {
		out[0] = (char) cp;
		return 1;
	}
	if (cp < 0x800) {
		out[0] = (char) (0xc0 | (cp >> 6));
		out[1] = (char) (0x80 | (cp & 0x3f));
		return 2;
	}
	out[0] = (char) (0xe0 | (cp >> 12));
	out[1] = (char) (0x80 | ((cp >> 6) & 0x3f));
	out[2] = (char) (0x80 | (cp & 0x3f));
	return 3;
}

static void put_party(tetra_gen_t *g, struct bw *w, tetra_rx_cmce_t *c, int ext)
{
	c->party.ssi = 1 + rnd(g, 0xfffffe);
	if (ext) {
		c->party.mcc = (uint16_t) rnd(g, 1000);
		c->party.mnc = (uint16_t) rnd(g, 16384);
		c->party.present = 2;
		put(w, 2, 2);
		put(w, 24, c->party.ssi);
		put(w, 10, c->party.mcc);
		put(w, 14, c->party.mnc);
	} else {
		c->party.present = 1;
		put(w, 2, 1);
		put(w, 24, c->party.ssi);
	}
	c->fields |= TETRA_RX_CMCE_F_PARTY;
}

static void put_call_id(tetra_gen_t *g, struct bw *w, tetra_rx_cmce_t *c)
{
	c->call_id = (uint16_t) rnd(g, 16384);
	c->fields |= TETRA_RX_CMCE_F_CALL_ID;
	put(w, 14, c->call_id);
}

static void put_basic_service(tetra_gen_t *g, struct bw *w, tetra_rx_cmce_t *c)
{
	c->circuit_mode = 0;        /* speech */
	c->e2e_encryption = 0;
	c->comm_type = (uint8_t) rnd(g, 4);
	c->slots_or_speech = (uint8_t) rnd(g, 4);
	c->fields |= TETRA_RX_CMCE_F_BASIC_SERVICE;
	put(w, 3, c->circuit_mode);
	put(w, 1, c->e2e_encryption);
	put(w, 2, c->comm_type);
	put(w, 2, c->slots_or_speech);
}

static void put_tx_grant(tetra_gen_t *g, struct bw *w, tetra_rx_cmce_t *c)
{
	c->tx_grant = (uint8_t) rnd(g, 4);
	c->tx_req_permission = (uint8_t) rnd(g, 2);
	c->fields |= TETRA_RX_CMCE_F_TX_GRANT | TETRA_RX_CMCE_F_TX_REQ_PERM;
	put(w, 2, c->tx_grant);
	put(w, 1, c->tx_req_permission);
}

/* type 2 elements: P-bit + element */
static void put_opt_notification(tetra_gen_t *g, struct bw *w, tetra_rx_cmce_t *c, int present)
{
	put(w, 1, (uint32_t) present);
	if (present) {
		c->notification = (uint8_t) rnd(g, 64);
		c->fields |= TETRA_RX_CMCE_F_NOTIFICATION;
		put(w, 6, c->notification);
	}
}

static void put_opt_temp_addr(tetra_gen_t *g, struct bw *w, tetra_rx_cmce_t *c, int present)
{
	put(w, 1, (uint32_t) present);
	if (present) {
		c->temp_address = rnd(g, 1u << 24);
		c->fields |= TETRA_RX_CMCE_F_TEMP_ADDR;
		put(w, 24, c->temp_address);
	}
}

/* text of n characters: PDU bits (8 or 16 per character) and expected UTF-8 */
static void put_text(tetra_gen_t *g, struct bw *w, tetra_rx_cmce_t *c, unsigned int n, int ucs2)
{
	static const char ascii[] =
		"ABCDEFGHIJKLMNOPQRSTUVWXYZ abcdefghijklmnopqrstuvwxyz 0123456789 .,-!?";
	static const uint16_t latin[] = { 0xe4, 0xf6, 0xfc, 0xe9, 0xdf, 0xc5, 0xf1 };
	static const uint16_t wide[] = { 0x20ac, 0x3a9, 0x416, 0x4e2d };
	unsigned int i, pos = 0;

	for (i = 0; i < n; i++) {
		uint32_t cp, r = rnd(g, 16);

		if (r == 0)
			cp = latin[rnd(g, sizeof(latin) / sizeof(latin[0]))];
		else if (r == 1 && ucs2)
			cp = wide[rnd(g, sizeof(wide) / sizeof(wide[0]))];
		else
			cp = (uint32_t) ascii[rnd(g, sizeof(ascii) - 1)];
		put(w, ucs2 ? 16 : 8, cp);
		pos += utf8_enc(c->text + pos, cp);
	}
	assert(pos < TETRA_RX_TEXT_MAX);
	c->text[pos] = 0;
	c->fields |= TETRA_RX_CMCE_F_TEXT;
	c->text_coding = ucs2 ? 26 : 1;
}

/* Build the TL-SDU of message m. 'small': at most 69 bits. 'param': text
 * length / body size selector (0 = random). */
static void build_sdu(tetra_gen_t *g, tetra_gen_msg_t *m, int small)
{
	struct bw w = { m->sdu, 0, TETRA_GEN_SDU_MAX };
	tetra_rx_cmce_t *c = &m->cmce;
	unsigned int opt, n, v, len_pos, start;

	switch (m->kind) {
	case TETRA_GEN_D_SETUP:
		m->pdisc = TETRA_RX_PD_CMCE;
		m->pdu_type = TETRA_RX_CMCE_D_SETUP;
		put(&w, 3, m->pdisc);
		put(&w, 5, m->pdu_type);
		put_call_id(g, &w, c);
		c->call_timeout = (uint8_t) rnd(g, 16);
		c->hook_method = (uint8_t) rnd(g, 2);
		c->duplex = (uint8_t) rnd(g, 2);
		put(&w, 4, c->call_timeout);
		put(&w, 1, c->hook_method);
		put(&w, 1, c->duplex);
		put_basic_service(g, &w, c);
		put_tx_grant(g, &w, c);
		c->call_priority = (uint8_t) rnd(g, 16);
		put(&w, 4, c->call_priority);
		c->fields |= TETRA_RX_CMCE_F_CALL_TIMEOUT | TETRA_RX_CMCE_F_HOOK |
			     TETRA_RX_CMCE_F_SIMPLEX | TETRA_RX_CMCE_F_PRIORITY;
		opt = small ? 0 : rnd(g, 16);
		put(&w, 1, opt != 0);                   /* O-bit */
		if (opt) {
			put_opt_notification(g, &w, c, opt & 1);
			put_opt_temp_addr(g, &w, c, (opt >> 1) & 1);
			put(&w, 1, (opt >> 2) & 1);
			if (opt & 4)
				put_party(g, &w, c, (opt >> 3) & 1);
			put(&w, 1, 0);                  /* M-bit */
		}
		break;
	case TETRA_GEN_D_CONNECT:
		m->pdisc = TETRA_RX_PD_CMCE;
		m->pdu_type = TETRA_RX_CMCE_D_CONNECT;
		put(&w, 3, m->pdisc);
		put(&w, 5, m->pdu_type);
		put_call_id(g, &w, c);
		c->call_timeout = (uint8_t) rnd(g, 16);
		c->hook_method = (uint8_t) rnd(g, 2);
		c->duplex = (uint8_t) rnd(g, 2);
		put(&w, 4, c->call_timeout);
		put(&w, 1, c->hook_method);
		put(&w, 1, c->duplex);
		put_tx_grant(g, &w, c);
		c->call_ownership = (uint8_t) rnd(g, 2);
		put(&w, 1, c->call_ownership);
		c->fields |= TETRA_RX_CMCE_F_CALL_TIMEOUT | TETRA_RX_CMCE_F_HOOK |
			     TETRA_RX_CMCE_F_SIMPLEX | TETRA_RX_CMCE_F_CALL_OWNER;
		opt = small ? rnd(g, 2) : rnd(g, 16);
		put(&w, 1, opt != 0);
		if (opt) {
			put(&w, 1, opt & 1);
			if (opt & 1) {
				c->call_priority = (uint8_t) rnd(g, 16);
				c->fields |= TETRA_RX_CMCE_F_PRIORITY;
				put(&w, 4, c->call_priority);
			}
			put(&w, 1, (opt >> 1) & 1);
			if (opt & 2)
				put_basic_service(g, &w, c);
			put_opt_temp_addr(g, &w, c, (opt >> 2) & 1);
			put_opt_notification(g, &w, c, (opt >> 3) & 1);
			put(&w, 1, 0);
		}
		break;
	case TETRA_GEN_D_TX_GRANTED:
		m->pdisc = TETRA_RX_PD_CMCE;
		m->pdu_type = TETRA_RX_CMCE_D_TX_GRANTED;
		put(&w, 3, m->pdisc);
		put(&w, 5, m->pdu_type);
		put_call_id(g, &w, c);
		put_tx_grant(g, &w, c);
		c->enc_control = (uint8_t) rnd(g, 2);
		c->fields |= TETRA_RX_CMCE_F_ENC_CONTROL;
		put(&w, 1, c->enc_control);
		put(&w, 1, 0);                          /* reserved */
		opt = small ? rnd(g, 2) : rnd(g, 8);
		put(&w, 1, opt != 0);
		if (opt) {
			put_opt_notification(g, &w, c, opt & 1);
			put(&w, 1, (opt >> 1) & 1);
			if (opt & 2)
				put_party(g, &w, c, (opt >> 2) & 1);
			put(&w, 1, 0);
		}
		break;
	case TETRA_GEN_D_TX_CEASED:
		m->pdisc = TETRA_RX_PD_CMCE;
		m->pdu_type = TETRA_RX_CMCE_D_TX_CEASED;
		put(&w, 3, m->pdisc);
		put(&w, 5, m->pdu_type);
		put_call_id(g, &w, c);
		c->tx_req_permission = (uint8_t) rnd(g, 2);
		c->fields |= TETRA_RX_CMCE_F_TX_REQ_PERM;
		put(&w, 1, c->tx_req_permission);
		opt = rnd(g, 2);
		put(&w, 1, opt);
		if (opt) {
			put_opt_notification(g, &w, c, 1);
			put(&w, 1, 0);
		}
		break;
	case TETRA_GEN_D_RELEASE:
		m->pdisc = TETRA_RX_PD_CMCE;
		m->pdu_type = TETRA_RX_CMCE_D_RELEASE;
		put(&w, 3, m->pdisc);
		put(&w, 5, m->pdu_type);
		put_call_id(g, &w, c);
		c->disconnect_cause = (uint8_t) rnd(g, 32);
		c->fields |= TETRA_RX_CMCE_F_DISC_CAUSE;
		put(&w, 5, c->disconnect_cause);
		opt = rnd(g, 2);
		put(&w, 1, opt);
		if (opt) {
			put_opt_notification(g, &w, c, 1);
			put(&w, 1, 0);
		}
		break;
	case TETRA_GEN_D_STATUS:
		m->pdisc = TETRA_RX_PD_CMCE;
		m->pdu_type = TETRA_RX_CMCE_D_STATUS;
		put(&w, 3, m->pdisc);
		put(&w, 5, m->pdu_type);
		put_party(g, &w, c, 0);
		c->status = (uint16_t) rnd(g, 65536);
		c->fields |= TETRA_RX_CMCE_F_STATUS;
		put(&w, 16, c->status);
		put(&w, 1, 0);
		break;
	case TETRA_GEN_D_SDS_TEXT:
		m->pdisc = TETRA_RX_PD_CMCE;
		m->pdu_type = TETRA_RX_CMCE_D_SDS_DATA;
		put(&w, 3, m->pdisc);
		put(&w, 5, m->pdu_type);
		put_party(g, &w, c, !small && rnd(g, 4) == 0);
		c->sds_type = 3;
		put(&w, 2, 3);
		len_pos = w.n;
		put(&w, 11, 0);                         /* length, patched below */
		start = w.n;
		v = rnd(g, 4);
		/* a third of the texts is long enough to need MAC fragmentation */
		n = rnd(g, 3) == 0 ? 20 + rnd(g, v == 3 ? 40 : 90) : 1 + rnd(g, 17);
		if (v == 2) {
			/* simple text messaging */
			c->protocol_id = 0x02;
			put(&w, 8, c->protocol_id);
			put(&w, 1, 0);
			put(&w, 7, 1);                  /* ISO 8859-1 */
			put_text(g, &w, c, n, 0);
		} else {
			/* SDS-TL, SDS-TRANSFER */
			c->protocol_id = 0x82;
			put(&w, 8, c->protocol_id);
			put(&w, 4, 0);                  /* SDS-TRANSFER */
			put(&w, 2, rnd(g, 4));          /* delivery report request */
			put(&w, 1, 0);                  /* service selection */
			put(&w, 1, v == 1);             /* storage / forward control */
			put(&w, 8, rnd(g, 256));        /* message reference */
			if (v == 1) {
				put(&w, 5, rnd(g, 32)); /* validity period */
				put(&w, 3, 1);          /* forward address: SSI */
				put(&w, 24, rnd(g, 1u << 24));
			}
			put(&w, 1, v == 1);             /* time stamp used */
			put(&w, 7, v == 3 ? 26 : 1);    /* text coding scheme */
			if (v == 1)
				put(&w, 24, rnd(g, 1u << 24));
			put_text(g, &w, c, n, v == 3);
		}
		c->sds_bits = (uint16_t) (w.n - start);
		m->sds_off = (uint16_t) start;
		c->fields |= TETRA_RX_CMCE_F_SDS | TETRA_RX_CMCE_F_PROTOCOL_ID;
		{
			struct bw l = { m->sdu, len_pos, len_pos + 11 };

			put(&l, 11, c->sds_bits);
		}
		put(&w, 1, 0);
		break;
	case TETRA_GEN_D_SDS_SHORT:
		m->pdisc = TETRA_RX_PD_CMCE;
		m->pdu_type = TETRA_RX_CMCE_D_SDS_DATA;
		put(&w, 3, m->pdisc);
		put(&w, 5, m->pdu_type);
		put_party(g, &w, c, 0);
		c->sds_type = (uint8_t) rnd(g, small ? 2 : 4);
		put(&w, 2, c->sds_type);
		if (c->sds_type == 3) {
			/* type 4 with a protocol that is not a text service */
			n = 8 + 8 * (1 + rnd(g, 12));
			put(&w, 11, n);
			m->sds_off = (uint16_t) w.n;
			c->protocol_id = 0x0a;
			put(&w, 8, c->protocol_id);
			rnd_bits(g, &w, n - 8);
			c->fields |= TETRA_RX_CMCE_F_PROTOCOL_ID;
		} else {
			n = 16u << c->sds_type;
			m->sds_off = (uint16_t) w.n;
			rnd_bits(g, &w, n);
		}
		c->sds_bits = (uint16_t) n;
		c->fields |= TETRA_RX_CMCE_F_SDS;
		put(&w, 1, 0);
		break;
	case TETRA_GEN_MM_LOC_UPD_ACC:
		m->pdisc = TETRA_RX_PD_MM;
		m->pdu_type = 5;                        /* D-LOCATION UPDATE ACCEPT */
		put(&w, 3, m->pdisc);
		put(&w, 4, m->pdu_type);
		rnd_bits(g, &w, 10 + rnd(g, small ? 40 : 100));
		break;
	case TETRA_GEN_MLE_NWRK_BCAST:
		m->pdisc = TETRA_RX_PD_MLE;
		m->pdu_type = 2;                        /* D-NWRK-BROADCAST */
		put(&w, 3, m->pdisc);
		put(&w, 3, m->pdu_type);
		if (small)
			rnd_bits(g, &w, 20 + rnd(g, 40));
		else
			rnd_bits(g, &w, rnd(g, 2) ? 60 + rnd(g, 120) : 250 + rnd(g, 700));
		break;
	default:
		/* TETRA_GEN_ENCRYPTED: the content never reaches the receiver */
		m->enc_mode = (uint8_t) (1 + rnd(g, 3));
		rnd_bits(g, &w, 30 + rnd(g, small ? 30 : 150));
		break;
	}
	m->have_cmce = m->pdisc == TETRA_RX_PD_CMCE && !m->enc_mode;
	if (m->have_cmce)
		c->parse_ok = 1;
	m->sdu_bits = (uint16_t) w.n;
	assert(!small || w.n <= 69);
}

/* ---- messages ------------------------------------------------------------ */

static uint32_t msg_alloc(tetra_gen_t *g, uint8_t kind)
{
	tetra_gen_msg_t *m;

	if (g->n_msgs == g->cap) {
		g->cap = g->cap ? 2 * g->cap : 256;
		g->msgs = realloc(g->msgs, g->cap * sizeof(*g->msgs));
		assert(g->msgs);
	}
	m = &g->msgs[g->n_msgs];
	memset(m, 0, sizeof(*m));
	m->ssi = TETRA_GEN_SSI_BASE + (uint32_t) g->n_msgs;
	m->kind = kind;
	return (uint32_t) g->n_msgs++;
}

/* LLC: wrap the TL-SDU into a basic link PDU -> TM-SDU in p->tm */
static void make_tm_sdu(tetra_gen_t *g, struct pend *p, uint32_t idx)
{
	tetra_gen_msg_t *m = &g->msgs[idx];
	struct bw w = { p->tm, 0, sizeof(p->tm) };

	switch (m->llc) {
	case 0:                     /* BL-ADATA: N(R), N(S) */
		put(&w, 4, 0);
		put(&w, 2, rnd(g, 4));
		break;
	case 1:                     /* BL-DATA: N(S) */
	case 5:
		put(&w, 4, m->llc);
		put(&w, 1, rnd(g, 2));
		break;
	default:                    /* BL-UDATA */
		put(&w, 4, 2);
		break;
	}
	put_bits(&w, m->sdu, m->sdu_bits);
	if (m->llc == 5)
		put(&w, 32, tetra_gen_fcs(m->sdu, m->sdu_bits));
	if (m->enc_mode) {
		/* air interface encrypted: any bits */
		unsigned int n = w.n;

		w.n = 0;
		rnd_bits(g, &w, n);
	}
	p->valid = 1;
	p->idx = idx;
	p->len = (uint16_t) w.n;
	p->pos = 0;
}

static void pend_new(tetra_gen_t *g, struct pend *p, int kind, int small, int stch)
{
	static const uint8_t seq[] = {
		TETRA_GEN_D_SETUP, TETRA_GEN_D_SDS_TEXT, TETRA_GEN_D_CONNECT,
		TETRA_GEN_D_STATUS, TETRA_GEN_MLE_NWRK_BCAST, TETRA_GEN_D_TX_GRANTED,
		TETRA_GEN_D_SDS_SHORT, TETRA_GEN_D_TX_CEASED, TETRA_GEN_ENCRYPTED,
		TETRA_GEN_D_RELEASE, TETRA_GEN_MM_LOC_UPD_ACC, TETRA_GEN_D_SDS_TEXT,
	};
	static const uint8_t seq_small[] = {
		TETRA_GEN_D_STATUS, TETRA_GEN_D_TX_GRANTED, TETRA_GEN_D_SETUP,
		TETRA_GEN_D_RELEASE, TETRA_GEN_D_SDS_SHORT, TETRA_GEN_D_TX_CEASED,
		TETRA_GEN_D_CONNECT, TETRA_GEN_MM_LOC_UPD_ACC, TETRA_GEN_MLE_NWRK_BCAST,
	};
	tetra_gen_msg_t *m;
	uint32_t idx, r;

	if (kind < 0) {
		if (small)
			kind = seq_small[g->small_rot++ % sizeof(seq_small)];
		else
			kind = seq[g->kind_rot++ % sizeof(seq)];
	}
	idx = msg_alloc(g, (uint8_t) kind);
	m = &g->msgs[idx];
	build_sdu(g, m, small);

	r = rnd(g, 8);
	if (kind == TETRA_GEN_MLE_NWRK_BCAST)
		m->llc = 2;
	else if (r == 0)
		m->llc = 0;
	else if (r <= 2)
		m->llc = 1;
	else if (r == 3 && !small)
		m->llc = 5;
	else
		m->llc = 2;

	if (!small && kind != TETRA_GEN_ENCRYPTED) {
		m->slot_grant = rnd(g, 8) == 0;
		if (rnd(g, 8) == 0) {
			m->chan_alloc = 1;
			m->chan_alloc_carrier = (uint16_t) rnd(g, 4096);
			m->chan_alloc_ts = (uint8_t) (1 + rnd(g, 15));
		}
	}
	/* signalling of an encrypted call is encrypted as well */
	if (stch && g->cfg.traffic_encrypted)
		m->enc_mode = 1;
	make_tm_sdu(g, p, idx);
}

/* the call set-up message that tells which usage marker / channel the
 * traffic uses and whether it is encrypted */
static void pend_call(tetra_gen_t *g, struct pend *p)
{
	uint32_t idx = msg_alloc(g, TETRA_GEN_D_SETUP);
	tetra_gen_msg_t *m = &g->msgs[idx];

	build_sdu(g, m, 0);
	m->llc = 2;
	m->usage_marker = g->cfg.usage_marker;
	m->chan_alloc = 1;
	m->chan_alloc_carrier = g->cfg.main_carrier;
	m->chan_alloc_ts = (uint8_t) (8 >> (g->cfg.traffic_tn - 1));
	if (g->cfg.traffic_encrypted) {
		m->kind = TETRA_GEN_ENCRYPTED;
		m->enc_mode = 1;
		m->have_cmce = 0;
	}
	make_tm_sdu(g, p, idx);
}

/* ---- MAC ----------------------------------------------------------------- */

static unsigned int res_hdr_bits(const tetra_gen_msg_t *m)
{
	return 43u + (m->usage_marker ? 6u : 0u) + (m->slot_grant ? 8u : 0u) +
	       (m->chan_alloc ? 25u : 0u);
}

enum { RES_NORMAL = 0, RES_START_FRAG, RES_2ND_STOLEN };

/* MAC-RESOURCE, EN 300 392-2 clause 21.4.3.1 */
static void send_resource(tetra_gen_t *g, struct bw *w, struct pend *p, uint8_t lchan, int mode)
{
	tetra_gen_msg_t *m = &g->msgs[p->idx];
	unsigned int hdr = res_hdr_bits(m), room = w->max - w->n, start = w->n;
	unsigned int n_sdu, len_ind, fill, end;

	if (mode == RES_START_FRAG) {
		n_sdu = room - hdr;
		len_ind = 0x3f;
		fill = 0;
		end = w->max;
	} else {
		unsigned int len = hdr + p->len;

		assert(len <= room);
		n_sdu = p->len;
		if (mode == RES_2ND_STOLEN) {
			len_ind = 0x3e;
			fill = len < room;
			end = w->max;
		} else {
			/* length in octets; the last octet may be cut by the end
			 * of the MAC block */
			len_ind = (len + 7) / 8;
			fill = (len & 7) && len < room;
			end = start + 8 * len_ind;
			if (end > w->max)
				end = w->max;
		}
	}

	put(w, 2, 0);               /* MAC-RESOURCE */
	put(w, 1, fill);
	put(w, 1, 0);               /* position of grant */
	put(w, 2, m->enc_mode);
	put(w, 1, 0);               /* random access flag */
	put(w, 6, len_ind);
	if (m->usage_marker) {
		put(w, 3, 6);
		put(w, 24, m->ssi);
		put(w, 6, m->usage_marker);
	} else {
		put(w, 3, 1);
		put(w, 24, m->ssi);
	}
	put(w, 1, 0);               /* power control flag */
	put(w, 1, m->slot_grant);
	if (m->slot_grant) {
		put(w, 4, 1 + rnd(g, 6));
		put(w, 4, rnd(g, 4));
	}
	put(w, 1, m->chan_alloc);
	if (m->chan_alloc) {
		/* random bits if encrypted: the receiver cannot read the element */
		if (m->enc_mode)
			rnd_bits(g, w, 25);
		else {
			put(w, 2, 0);       /* replace */
			put(w, 4, m->chan_alloc_ts);
			put(w, 2, 3);       /* uplink + downlink */
			put(w, 1, 0);       /* CLCH permission */
			put(w, 1, 0);       /* cell change flag */
			put(w, 12, m->chan_alloc_carrier);
			put(w, 1, 0);       /* no extended carrier numbering */
			put(w, 2, 1);       /* monitoring pattern */
		}
	}
	assert(w->n - start == hdr);
	put_bits(w, p->tm + p->pos, n_sdu);
	p->pos = (uint16_t) (p->pos + n_sdu);
	if (fill) {
		put(w, 1, 1);
		while (w->n < end)
			put(w, 1, 0);
	}
	assert(w->n == end);

	m->lchan = lchan;
	m->first_slot = g->index;
	m->fragments = 1;
	if (mode != RES_START_FRAG) {
		m->complete = 1;
		m->last_slot = g->index;
		p->valid = 0;
	}
}

/* MAC-END with the rest of the TM-SDU, clause 21.4.3.3 */
static void send_end(tetra_gen_t *g, struct bw *w, struct pend *p)
{
	tetra_gen_msg_t *m = &g->msgs[p->idx];
	unsigned int rest = (unsigned int) (p->len - p->pos);
	unsigned int len = 13 + rest, len_ind = (len + 7) / 8;
	unsigned int room = w->max - w->n, fill = (len & 7) && len < room;
	unsigned int end = w->n + 8 * len_ind;

	assert(len <= room);
	if (end > w->max)
		end = w->max;
	put(w, 2, 1);
	put(w, 1, 1);               /* MAC-END */
	put(w, 1, fill);
	put(w, 1, 0);               /* position of grant */
	put(w, 6, len_ind);
	put(w, 1, 0);               /* slot granting flag */
	put(w, 1, 0);               /* channel allocation flag */
	put_bits(w, p->tm + p->pos, rest);
	if (fill) {
		put(w, 1, 1);
		while (w->n < end)
			put(w, 1, 0);
	}
	m->fragments++;
	m->complete = 1;
	m->last_slot = g->index;
	p->valid = 0;
}

/* MAC-FRAG, clause 21.4.3.2: fills the block */
static void send_frag(tetra_gen_t *g, struct bw *w, struct pend *p)
{
	unsigned int n = w->max - w->n - 4, rest = (unsigned int) (p->len - p->pos);
	unsigned int fill = 0;

	/* the rest does not fit MAC-END but would more than fill this block:
	 * keep one octet for MAC-END and use fill bits here */
	if (n >= rest) {
		assert(rest > 8);
		n = rest - 8;
		fill = 1;
	}
	put(w, 2, 1);
	put(w, 1, 0);               /* MAC-FRAG */
	put(w, 1, fill);
	put_bits(w, p->tm + p->pos, n);
	if (fill) {
		put(w, 1, 1);
		while (w->n < w->max)
			put(w, 1, 0);
	}
	p->pos = (uint16_t) (p->pos + n);
	g->msgs[p->idx].fragments++;
}

/* Null PDU and fill bits up to the end of the block */
static void blk_finish_li(struct bw *w, unsigned int len_ind)
{
	unsigned int room = w->max - w->n;

	if (room >= 16) {
		put(w, 2, 0);
		put(w, 1, room > 16);   /* fill bit indication */
		put(w, 1, 0);
		put(w, 2, 0);
		put(w, 1, 0);
		put(w, 6, len_ind);     /* 2: 16 bits */
		put(w, 3, 0);           /* null address */
		if (room > 16)
			put(w, 1, 1);
	}
	while (w->n < w->max)
		put(w, 1, 0);
}

static void blk_finish(struct bw *w)
{
	blk_finish_li(w, 2);
}

/* Fill one signalling MAC block of timeslot tn.
 * quiet: null PDU only; small: only messages that fit a half slot and no
 * fragmentation (traffic channels: the next block may be far away) */
static void fill_block(tetra_gen_t *g, unsigned int tn, uint8_t *out, unsigned int size,
		       uint8_t lchan, int quiet, int small)
{
	struct bw w = { out, 0, size };
	struct pend *p = &g->pend[tn - 1];
	unsigned int n_pdu = 0;

	memset(out, 0, size);

	if (g->frag[tn - 1]) {
		if ((unsigned int) (p->len - p->pos) + 13 <= size) {
			send_end(g, &w, p);
			g->frag[tn - 1] = 0;
			n_pdu++;
		} else {
			send_frag(g, &w, p);
			return;
		}
	}

	while (!quiet && !g->cfg.no_messages && n_pdu < 3) {
		unsigned int room = size - w.n, len;

		if (room < 60)
			break;
		if (!p->valid) {
			if (tn == 1 && g->cfg.traffic_tn && !small && (g->call_rot++ % 12) == 0)
				pend_call(g, p);
			else
				pend_new(g, p, -1, small, lchan == TETRA_RX_CH_STCH);
		}
		len = res_hdr_bits(&g->msgs[p->idx]) + p->len;
		if (len <= room) {
			send_resource(g, &w, p, lchan, RES_NORMAL);
		} else if (w.n == 0 && size == SCH_F_BITS && len > SCH_F_BITS) {
			send_resource(g, &w, p, lchan, RES_START_FRAG);
			g->frag[tn - 1] = 1;
			return;
		} else if (w.n == 0 && size < SCH_F_BITS) {
			/* the waiting message needs a full slot: send a short one */
			pend_new(g, &g->tmp, -1, 1, lchan == TETRA_RX_CH_STCH);
			send_resource(g, &w, &g->tmp, lchan, RES_NORMAL);
		} else
			break;
		n_pdu++;
		if (rnd(g, 5) < 2)
			break;
	}
	blk_finish(&w);
}

/* ---- broadcast PDUs ------------------------------------------------------ */

/* SYNC PDU, clause 21.4.4.2 with D-MLE-SYNC, clause 18.4.2.1 */
static void build_sync(const tetra_gen_t *g, uint8_t *out)
{
	struct bw w = { out, 0, 60 };

	put(&w, 4, 1);              /* system code */
	put(&w, 6, g->cfg.colour_code);
	put(&w, 2, g->tn - 1u);
	put(&w, 5, g->fn);
	put(&w, 6, g->mn);
	put(&w, 2, 0);              /* sharing mode: continuous transmission */
	put(&w, 3, 0);              /* TS reserved frames */
	put(&w, 1, 0);              /* U-plane DTX */
	put(&w, 1, 0);              /* frame 18 extension */
	put(&w, 1, 0);              /* reserved */
	put(&w, 10, g->cfg.mcc);
	put(&w, 14, g->cfg.mnc);
	put(&w, 2, 2);              /* neighbour cell broadcast */
	put(&w, 2, 1);              /* cell load */
	put(&w, 1, 1);              /* late entry supported */
	assert(w.n == 60);
}

/* SYSINFO PDU, clause 21.4.4.1 with D-MLE-SYSINFO, clause 18.4.2.2 */
static void build_sysinfo(const tetra_gen_t *g, uint8_t *out)
{
	struct bw w = { out, 0, SCH_HD_BITS };

	put(&w, 2, 2);              /* broadcast PDU */
	put(&w, 2, 0);              /* SYSINFO */
	put(&w, 12, g->cfg.main_carrier);
	put(&w, 4, g->cfg.freq_band);
	put(&w, 2, g->cfg.freq_offset);
	put(&w, 3, g->cfg.duplex_spacing);
	put(&w, 1, g->cfg.reverse_operation);
	put(&w, 2, 0);              /* number of common secondary control channels */
	put(&w, 3, 5);              /* MS_TXPWR_MAX_CELL */
	put(&w, 4, 3);              /* RXLEV_ACCESS_MIN */
	put(&w, 4, 7);              /* ACCESS_PARAMETER */
	put(&w, 4, 3);              /* RADIO_DOWNLINK_TIMEOUT */
	put(&w, 1, 0);              /* hyperframe number follows */
	put(&w, 16, g->hn);
	put(&w, 2, 0);              /* option: even multiframe definition */
	put(&w, 20, 0xfffff);
	assert(w.n == 82);
	put(&w, 14, g->cfg.location_area);
	put(&w, 16, 0xffff);        /* subscriber class */
	put(&w, 12, g->cfg.bs_service_details);
	assert(w.n == SCH_HD_BITS);
}

/* ---- slots --------------------------------------------------------------- */

void tetra_gen_default_cfg(tetra_gen_cfg_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->mcc = 901;
	cfg->mnc = 9999;
	cfg->colour_code = 33;
	cfg->main_carrier = 1234;
	cfg->freq_band = 4;
	cfg->freq_offset = 1;
	cfg->duplex_spacing = 0;
	cfg->location_area = 4242;
	cfg->hyperframe = 1000;
	cfg->bs_service_details = 0xd75;
	cfg->start_tn = 1;
	cfg->start_fn = 1;
	cfg->start_mn = 1;
	cfg->usage_marker = 17;
	cfg->seed = 1;
}

tetra_gen_t *tetra_gen_new(const tetra_gen_cfg_t *cfg)
{
	tetra_gen_t *g = calloc(1, sizeof(*g));

	assert(g);
	g->cfg = *cfg;
	rng_seed(&g->rng, cfg->seed);
	g->scr = tetra_scramb_get_init(cfg->mcc, cfg->mnc, cfg->colour_code);
	g->tn = cfg->start_tn;
	g->fn = cfg->start_fn;
	g->mn = cfg->start_mn;
	g->hn = cfg->hyperframe;
	return g;
}

void tetra_gen_free(tetra_gen_t *g)
{
	if (!g)
		return;
	free(g->msgs);
	free(g);
}

size_t tetra_gen_msg_count(const tetra_gen_t *g)
{
	return g->n_msgs;
}

const tetra_gen_msg_t *tetra_gen_msg(const tetra_gen_t *g, size_t i)
{
	return i < g->n_msgs ? &g->msgs[i] : NULL;
}

const char *tetra_gen_kind_name(uint8_t kind)
{
	static const char *const names[TETRA_GEN_KIND_COUNT] = {
		"D-SETUP", "D-CONNECT", "D-TX-GRANTED", "D-TX-CEASED", "D-RELEASE",
		"D-STATUS", "D-SDS-DATA(text)", "D-SDS-DATA", "D-LOCATION-UPDATE-ACCEPT",
		"D-NWRK-BROADCAST", "encrypted",
	};

	return kind < TETRA_GEN_KIND_COUNT ? names[kind] : "?";
}

enum { B_NDB1 = 0, B_NDB2 = 1, B_SB = 2 };

const tetra_gen_slot_t *tetra_gen_next(tetra_gen_t *g)
{
	tetra_gen_slot_t *s = &g->slot;
	const unsigned int tn = g->tn, fn = g->fn, mn = g->mn;
	const unsigned int f1 = 0x04, f2 = 0x15;    /* access fields */
	const int traffic = g->cfg.traffic_tn && tn == g->cfg.traffic_tn;
	const int ctrl = tn == 1 || (g->cfg.all_control && !traffic);
	uint8_t a[SCH_F_BITS], b[SCH_HD_BITS];      /* type-1 blocks */
	uint8_t sync[60];
	uint8_t t5[432], sb[120], bb[30];
	int burst = B_NDB1;
	int raw_tch = 0;                            /* t5 already holds the slot */
	unsigned int i;

	memset(s, 0, sizeof(*s));
	s->index = g->index;
	s->tn = (uint8_t) tn;
	s->fn = (uint8_t) fn;
	s->mn = (uint8_t) mn;
	s->kind = ctrl || traffic ? TETRA_GEN_SLOT_CONTROL : TETRA_GEN_SLOT_UNALLOC;

	if (fn == 18) {
		/* frame 18: control on every timeslot, BSCH and BNCH rotate */
		const unsigned int bsch_tn = 4 - ((mn + 3) % 4);
		const unsigned int bnch_tn = 4 - ((mn + 1) % 4);

		s->aach = (uint16_t) ((f1 << 6) | f2);
		if (tn == bsch_tn) {
			burst = B_SB;
			if (ctrl || traffic)
				fill_block(g, tn, b, SCH_HD_BITS, TETRA_RX_CH_SB2, 0, traffic);
			else {
				build_sysinfo(g, b);
				s->has_sysinfo = 1;
			}
		} else if (tn == bnch_tn) {
			burst = B_NDB2;
			fill_block(g, tn, a, SCH_HD_BITS, TETRA_RX_CH_SCH_HD, !(ctrl || traffic), traffic);
			build_sysinfo(g, b);
			s->has_sysinfo = 1;
		} else
			fill_block(g, tn, a, SCH_F_BITS, TETRA_RX_CH_SCH_F, !(ctrl || traffic), traffic);
	} else if (traffic) {
		const unsigned int um = g->cfg.usage_marker;
		unsigned int steal = g->cfg.no_messages ? 0 : g->steal_rot++ % 9;
		struct bw w;

		s->aach = (uint16_t) ((3u << 12) | (um << 6) | um);
		switch (steal) {
		case 3:
			/* first half slot stolen, speech in the second */
			s->kind = TETRA_GEN_SLOT_TCH_STOLEN1;
			burst = B_NDB2;
			fill_block(g, tn, a, SCH_HD_BITS, TETRA_RX_CH_STCH, 0, 1);
			break;
		case 6:
			/* both stolen: one PDU per half slot */
			s->kind = TETRA_GEN_SLOT_TCH_STOLEN2;
			burst = B_NDB2;
			w = (struct bw) { a, 0, SCH_HD_BITS };
			memset(a, 0, sizeof(a));
			pend_new(g, &g->tmp, -1, 1, 1);
			/* "second half slot stolen" is announced either in the
			 * PDU itself or, as live networks do when there is room,
			 * in a Null PDU after it */
			if (((g->steal_rot / 9) & 1) &&
			    res_hdr_bits(&g->msgs[g->tmp.idx]) + g->tmp.len <= SCH_HD_BITS - 20) {
				send_resource(g, &w, &g->tmp, TETRA_RX_CH_STCH, RES_NORMAL);
				blk_finish_li(&w, 0x3e);
			} else
				send_resource(g, &w, &g->tmp, TETRA_RX_CH_STCH, RES_2ND_STOLEN);
			fill_block(g, tn, b, SCH_HD_BITS, TETRA_RX_CH_STCH, 0, 1);
			break;
		case 8:
			/* both stolen: one PDU fragmented over the two half slots */
			s->kind = TETRA_GEN_SLOT_TCH_STOLEN2;
			burst = B_NDB2;
			do {
				pend_new(g, &g->tmp, TETRA_GEN_D_SDS_TEXT, 0, 1);
				i = res_hdr_bits(&g->msgs[g->tmp.idx]);
				/* must not fit one half slot but fit the two */
				if (i + g->tmp.len <= SCH_HD_BITS ||
				    i + g->tmp.len > 2 * SCH_HD_BITS - 13)
					g->tmp.valid = 0;       /* never sent */
			} while (!g->tmp.valid);
			w = (struct bw) { a, 0, SCH_HD_BITS };
			memset(a, 0, sizeof(a));
			send_resource(g, &w, &g->tmp, TETRA_RX_CH_STCH, RES_START_FRAG);
			w = (struct bw) { b, 0, SCH_HD_BITS };
			memset(b, 0, sizeof(b));
			send_end(g, &w, &g->tmp);
			blk_finish(&w);
			break;
		default: {
			uint8_t t1[274];

			s->kind = TETRA_GEN_SLOT_TCH;
			for (i = 0; i < sizeof(t1); i++)
				t1[i] = (uint8_t) (rng_u32(&g->rng) >> 31);
			tetra_acelp_type2_to_codec(t1, &s->tch[0][0]);
			tetra_gen_encode_tch(t1, t5);
			tetra_scramb_bits(g->scr, t5, 432);
			raw_tch = 1;
			break;
		}
		}
	} else if (ctrl) {
		s->aach = tn == 1 ? (uint16_t) ((f1 << 6) | f2)
				  : (uint16_t) ((1u << 12) | (TETRA_RX_DL_ASSIGNED_CCH << 6) | f2);
		switch (g->rot[tn - 1]++ % 5) {
		case 2:
			burst = B_NDB2;
			fill_block(g, tn, a, SCH_HD_BITS, TETRA_RX_CH_SCH_HD, 0, 0);
			fill_block(g, tn, b, SCH_HD_BITS, TETRA_RX_CH_SCH_HD, 0, 0);
			break;
		case 4:
			burst = B_NDB2;
			fill_block(g, tn, a, SCH_HD_BITS, TETRA_RX_CH_SCH_HD, 0, 0);
			build_sysinfo(g, b);
			s->has_sysinfo = 1;
			break;
		default:
			fill_block(g, tn, a, SCH_F_BITS, TETRA_RX_CH_SCH_F, 0, 0);
			break;
		}
	} else {
		/* unallocated physical channel */
		s->aach = (uint16_t) ((1u << 12) | (TETRA_RX_DL_UNALLOCATED << 6) | f2);
		switch (g->rot[tn - 1]++ % 3) {
		case 0:
			burst = B_SB;
			build_sysinfo(g, b);
			s->has_sysinfo = 1;
			break;
		case 1:
			fill_block(g, tn, a, SCH_F_BITS, TETRA_RX_CH_SCH_F, 1, 0);
			break;
		default:
			burst = B_NDB2;
			fill_block(g, tn, a, SCH_HD_BITS, TETRA_RX_CH_SCH_HD, 1, 0);
			build_sysinfo(g, b);
			s->has_sysinfo = 1;
			break;
		}
	}

	tetra_gen_encode_aach(s->aach, g->scr, bb);
	s->burst = (uint8_t) burst;
	if (raw_tch) {
		build_norm_c_d_burst(s->bits, t5, bb, t5 + 216, 0);
	} else if (burst == B_SB) {
		build_sync(g, sync);
		tetra_gen_encode_cch(sync, 0, SCRAMB_INIT, sb);
		tetra_gen_encode_cch(b, 1, g->scr, t5);
		build_sync_c_d_burst(s->bits, sb, bb, t5);
	} else if (burst == B_NDB2) {
		tetra_gen_encode_cch(a, 1, g->scr, t5);
		if (s->kind == TETRA_GEN_SLOT_TCH_STOLEN1) {
			/* stands in for the half slot speech coding */
			for (i = 0; i < 216; i++)
				t5[216 + i] = (uint8_t) (rng_u32(&g->rng) >> 31);
		} else
			tetra_gen_encode_cch(b, 1, g->scr, t5 + 216);
		build_norm_c_d_burst(s->bits, t5, bb, t5 + 216, 1);
	} else {
		tetra_gen_encode_cch(a, 2, g->scr, t5);
		build_norm_c_d_burst(s->bits, t5, bb, t5 + 216, 0);
	}

	/* advance the TDMA time */
	g->index++;
	if (++g->tn > 4) {
		g->tn = 1;
		if (++g->fn > 18) {
			g->fn = 1;
			if (++g->mn > 60) {
				g->mn = 1;
				g->hn++;
			}
		}
	}
	return s;
}
