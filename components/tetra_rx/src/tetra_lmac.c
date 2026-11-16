/* tetra_rx - lower MAC: descrambling, de-interleaving, de-puncturing,
 * Viterbi decoding and CRC of the downlink logical channels.
 *
 * Derived from osmo-tetra src/lower_mac/tetra_lower_mac.c
 * (C) 2011 by Harald Welte <laforge@gnumonks.org>, AGPL-3.0-or-later.
 *
 * Differences to the original:
 *  - works on soft decisions (int8, positive = 0) up to the Viterbi decoder
 *  - no msgb/talloc: all buffers live in struct tetra_rx
 *  - the scrambling sequences are cached instead of being regenerated for
 *    every block
 *  - AACH is error-corrected with an RM(30,14) decoder
 *  - traffic slots are passed to the TCH/S decoder instead of a dump file
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <stdint.h>
#include <string.h>

#include "tetra_ctx.h"

#include <lower_mac/crc_simple.h>
#include <lower_mac/tetra_conv_enc.h>
#include <lower_mac/tetra_interleave.h>
#include <lower_mac/tetra_scramb.h>

struct tetra_blk_param {
	uint16_t type345_bits;
	uint16_t type2_bits;
	uint16_t type1_bits;
	uint16_t interleave_a;
};

/* EN 300 392-2 clause 8.3.1 */
static const struct tetra_blk_param blk_sb1 = { 120, 80, 60, 11 };
static const struct tetra_blk_param blk_hd = { 216, 144, 124, 101 };
static const struct tetra_blk_param blk_f = { 432, 288, 268, 103 };

void tetra_lmac_init(struct tetra_rx *rx)
{
	tetra_scramb_get_bits(SCRAMB_INIT, rx->scr_bsch, sizeof(rx->scr_bsch));
	tetra_rm3014_dec_init(rx->rm_col);
}

void tetra_lmac_set_cell(struct tetra_rx *rx, uint16_t mcc, uint16_t mnc, uint8_t cc)
{
	if (rx->cell_valid && rx->mcc == mcc && rx->mnc == mnc && rx->colour_code == cc)
		return;
	rx->mcc = mcc;
	rx->mnc = mnc;
	rx->colour_code = cc;
	rx->scramb_init = tetra_scramb_get_init(mcc, mnc, cc);
	tetra_scramb_get_bits(rx->scramb_init, rx->scr_cell, sizeof(rx->scr_cell));
	rx->cell_valid = 1;
}

/* soft descrambling: type-5 -> type-4 */
static void descramble(int8_t *out, const int8_t *in, const uint8_t *seq, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		out[i] = seq[i] ? (int8_t) -in[i] : in[i];
}

/* type-5 soft bits -> type-2 hard bits in rx->t2 (zero padded); returns CRC ok */
static int decode_cch(struct tetra_rx *rx, const int8_t *soft, const uint8_t *seq,
		      const struct tetra_blk_param *bp)
{
	descramble(rx->t4, soft, seq, bp->type345_bits);
	block_deinterleave(bp->type345_bits, bp->interleave_a,
			   (const uint8_t *) rx->t4, (uint8_t *) rx->t3);
	/* punctured positions are erasures (0) */
	memset(rx->t3dp, 0, 4u * bp->type2_bits);
	tetra_rcpc_depunct(TETRA_RCPC_PUNCT_2_3, (const uint8_t *) rx->t3,
			   bp->type345_bits, (uint8_t *) rx->t3dp);
	tetra_viterbi_cch(rx->t3dp, rx->t2, bp->type2_bits, rx->vit);
	memset(rx->t2 + bp->type2_bits, 0, sizeof(rx->t2) - bp->type2_bits);
	return crc16_ccitt_bits(rx->t2, bp->type1_bits + 16u) == TETRA_CRC_OK;
}

static void count(struct tetra_rx *rx, uint8_t chan, int ok)
{
	if (ok)
		rx->st.chan[chan].ok++;
	else
		rx->st.chan[chan].fail++;
}

static void block_event(struct tetra_rx *rx, uint8_t chan, int blk_num, int ok,
			const uint8_t *bits, unsigned int n)
{
	tetra_rx_event_t *ev;

	if (!(rx->cfg.flags & TETRA_RX_FLAG_BLOCK_EVENTS))
		return;
	ev = tetra_rx_ev_begin(rx);
	ev->u.block.chan = chan;
	ev->u.block.blk_num = (uint8_t) blk_num;
	ev->u.block.crc_ok = (uint8_t) ok;
	ev->u.block.n_bits = (uint16_t) n;
	ev->u.block.bits = bits;
	tetra_rx_emit(rx, TETRA_RX_EV_BLOCK);
}

static void rx_sig_block(struct tetra_rx *rx, const int8_t *soft, const struct tetra_blk_param *bp,
			 uint8_t chan, int blk_num)
{
	int ok = decode_cch(rx, soft, rx->scr_cell, bp);

	count(rx, chan, ok);
	block_event(rx, chan, blk_num, ok, rx->t2, bp->type1_bits);
	if (ok)
		tetra_umac_block(rx, rx->t2, bp->type1_bits, chan);
	else
		tetra_umac_block(rx, NULL, 0, chan);
}

static void rx_bbk(struct tetra_rx *rx, const int8_t *soft)
{
	uint32_t cw = 0;
	uint16_t info;
	uint8_t bits[14];
	int i, n;

	for (i = 0; i < 30; i++) {
		unsigned int b = (soft[i] < 0) ^ rx->scr_cell[i];
		cw = (cw << 1) | b;
	}
	n = tetra_rm3014_dec(rx->rm_col, cw, &info);
	count(rx, TETRA_RX_CH_AACH, n >= 0);
	rx->cur.aach_ok = n >= 0;
	rx->cur.is_traffic = 0;
	rx->cur.blk1_stolen = 0;
	rx->cur.blk2_stolen = 0;
	if (n < 0) {
		tetra_umac_aach_lost(rx);
		return;
	}
	for (i = 0; i < 14; i++)
		bits[i] = (info >> (13 - i)) & 1;
	block_event(rx, TETRA_RX_CH_AACH, 0, 1, bits, 14);
	tetra_umac_aach(rx, info, n);
}

/* TP-SAP: called by the (vendored) tetra_burst_rx_cb() with the pieces of
 * one burst; 'bits' really are int8 soft bits here. */
void tp_sap_udata_ind(enum tp_sap_data_type type, int blk_num, const uint8_t *bits, unsigned int len, void *priv)
{
	struct tetra_rx *rx = priv;
	const int8_t *soft = (const int8_t *) bits;
	int ok;

	(void) len;

	if (type == TPSAP_T_SB1) {
		ok = decode_cch(rx, soft, rx->scr_bsch, &blk_sb1);
		count(rx, TETRA_RX_CH_BSCH, ok);
		block_event(rx, TETRA_RX_CH_BSCH, blk_num, ok, rx->t2, blk_sb1.type1_bits);
		if (ok)
			tetra_umac_bsch(rx, rx->t2);
		return;
	}

	/* everything else needs the scrambling code and the frame number */
	if (!rx->cell_valid || !rx->time_valid)
		return;

	switch (type) {
	case TPSAP_T_BBK:
		rx_bbk(rx, soft);
		break;
	case TPSAP_T_SB2:
		rx_sig_block(rx, soft, &blk_hd, TETRA_RX_CH_SB2, blk_num);
		break;
	case TPSAP_T_NDB:
		if (!rx->cur.is_traffic) {
			rx_sig_block(rx, soft, &blk_hd, TETRA_RX_CH_SCH_HD, blk_num);
			break;
		}
		/* Slot stealing, EN 300 392-2 clause 19.4.4 / 23.8.4: on a traffic
		 * channel training sequence 2 says that block 1 is stolen; the MAC
		 * header in block 1 says whether block 2 is stolen as well. */
		if (blk_num == BLK_1) {
			rx->cur.blk1_stolen = 1;
			rx->cur.blk2_stolen = 0;
			ok = decode_cch(rx, soft, rx->scr_cell, &blk_hd);
			count(rx, TETRA_RX_CH_STCH, ok);
			block_event(rx, TETRA_RX_CH_STCH, blk_num, ok, rx->t2, blk_hd.type1_bits);
			rx->cur.blk1_ok = (uint8_t) ok;
			if (ok)
				tetra_umac_block(rx, rx->t2, blk_hd.type1_bits, TETRA_RX_CH_STCH);
			else
				tetra_umac_block(rx, NULL, 0, TETRA_RX_CH_STCH);
		} else {
			if (rx->cur.blk2_stolen || !rx->cur.blk1_ok) {
				/* stolen, or unknown because block 1 was lost: in the
				 * latter case a good CRC decides */
				ok = decode_cch(rx, soft, rx->scr_cell, &blk_hd);
				if (rx->cur.blk2_stolen || ok) {
					count(rx, TETRA_RX_CH_STCH, ok);
					block_event(rx, TETRA_RX_CH_STCH, blk_num, ok, rx->t2, blk_hd.type1_bits);
				}
				if (ok) {
					rx->cur.blk2_stolen = 1;
					rx->cur.in_blk2 = 1;
					tetra_umac_block(rx, rx->t2, blk_hd.type1_bits, TETRA_RX_CH_STCH);
				} else if (rx->cur.blk2_stolen)
					tetra_umac_block(rx, NULL, 0, TETRA_RX_CH_STCH);
			}
			if (!(rx->cfg.flags & TETRA_RX_FLAG_NO_TRAFFIC)) {
				descramble(rx->t4, soft, rx->scr_cell, 216);
				tetra_tch_half_slot(rx, rx->t4);
			}
		}
		break;
	case TPSAP_T_SCH_F:
		if (!rx->cur.is_traffic) {
			rx_sig_block(rx, soft, &blk_f, TETRA_RX_CH_SCH_F, blk_num);
			break;
		}
		if (!(rx->cfg.flags & TETRA_RX_FLAG_NO_TRAFFIC)) {
			descramble(rx->t4, soft, rx->scr_cell, 432);
			tetra_tch_full_slot(rx, rx->t4);
		}
		break;
	default:
		break;
	}
}

/* osmo-tetra's tetra_tdma_time_add_tn() (tetra_tdma.c, not vendored) is only
 * correct for one slot. */
void tetra_rx_time_add(struct tetra_tdma_time *t, uint32_t slots)
{
	uint32_t x = ((t->mn - 1) * 18 + (t->fn - 1)) * 4 + (t->tn - 1);

	x += slots % (60u * 18u * 4u);
	t->tn = x % 4 + 1;
	x /= 4;
	t->fn = x % 18 + 1;
	x /= 18;
	t->mn = x % 60 + 1;
}

/* One slot from the modem. soft == NULL: the slot was not demodulated
 * (bad signal), only the TDMA time advances. */
void tetra_lmac_slot(struct tetra_rx *rx, const int8_t *soft, enum tetra_train_seq type)
{
	rx->cur.is_traffic = 0;
	rx->cur.blk1_stolen = 0;
	rx->cur.blk2_stolen = 0;
	rx->cur.blk1_ok = 0;
	rx->cur.in_blk2 = 0;
	rx->cur.aach_ok = 0;

	if (soft)
		tetra_burst_rx_cb((const uint8_t *) soft, TETRA_BITS_PER_TS, type, rx);

	if (rx->time_valid) {
		tetra_umac_slot_end(rx, soft != NULL);
		rx->st.time.tn = (uint8_t) rx->time.tn;
		rx->st.time.fn = (uint8_t) rx->time.fn;
		rx->st.time.mn = (uint8_t) rx->time.mn;
		tetra_rx_time_add(&rx->time, 1);
	}
}
