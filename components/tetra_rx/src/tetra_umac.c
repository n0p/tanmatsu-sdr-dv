/* tetra_rx - upper MAC, LLC and MLE dispatch
 *
 * Derived from osmo-tetra src/tetra_upper_mac.c, src/tetra_llc.c and
 * src/tetra_mle.c
 * (C) 2011 by Harald Welte <laforge@gnumonks.org>, AGPL-3.0-or-later.
 *
 * Differences to the original:
 *  - no msgb/talloc/printf: PDUs are parsed in place and reported as events
 *  - MAC fragments are reassembled into fixed per-timeslot buffers
 *  - PDU lengths are clamped to the MAC block (EN 300 392-2 clause 23.4.2.2)
 *    and all offsets are validated before use
 *  - fragment discard rules of clause 23.4.3.1.1 b) i)..iii)
 *  - MAC-D-BLCK, MAC-U-SIGNAL and ACCESS-DEFINE are handled with their real
 *    lengths; stolen-channel fragmentation (clause 23.8.4.2.2)
 *  - no decryption: encrypted PDUs are reported with their MAC header only
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
#include <errno.h>

#include "tetra_ctx.h"

#include <tetra_llc_pdu.h>
#include <tetra_mle_pdu.h>
#include <tetra_mm_pdu.h>
#include <tetra_cmce_pdu.h>
#include <tetra_sndcp_pdu.h>

void tetra_umac_reset(struct tetra_rx *rx)
{
	unsigned int i;

	for (i = 0; i < 4; i++) {
		rx->frag[i].active = 0;
		rx->frag[i].len = 0;
	}
	memset(rx->um_enc, TETRA_RX_ENC_UNKNOWN, sizeof(rx->um_enc));
	memset(rx->ts_enc, TETRA_RX_ENC_UNKNOWN, sizeof(rx->ts_enc));
	memset(rx->tn_use, 0, sizeof(rx->tn_use));
	rx->sid_valid = 0;
	rx->st.sysinfo_valid = 0;
	memset(rx->st.slot, 0, sizeof(rx->st.slot));
}

/* burst synchronisation lost: MAC blocks were missed */
void tetra_umac_sync_lost(struct tetra_rx *rx)
{
	unsigned int i;

	for (i = 0; i < 4; i++) {
		if (rx->frag[i].active == 1)
			rx->st.frag_dropped++;
		rx->frag[i].active = 0;
		rx->frag[i].len = 0;
	}
}

/* ---- BSCH ---------------------------------------------------------------- */

/* SYNC PDU, EN 300 392-2 clause 21.4.4.2 + D-MLE-SYNC clause 18.4.2.1 */
void tetra_umac_bsch(struct tetra_rx *rx, const uint8_t *bits)
{
	tetra_rx_event_t *ev;
	tetra_rx_ev_bsch_t *b;
	uint8_t sys = (uint8_t) bits_to_uint(bits, 4);
	uint8_t cc = (uint8_t) bits_to_uint(bits + 4, 6);
	uint8_t tn = (uint8_t) (bits_to_uint(bits + 10, 2) + 1);
	uint8_t fn = (uint8_t) bits_to_uint(bits + 12, 5);
	uint8_t mn = (uint8_t) bits_to_uint(bits + 17, 6);
	uint16_t mcc = (uint16_t) bits_to_uint(bits + 31, 10);
	uint16_t mnc = (uint16_t) bits_to_uint(bits + 41, 14);
	int changed;

	/* system code 1xxx is Direct Mode; out of range counters cannot come
	 * from a V+D base station */
	if ((sys & 8) || fn < 1 || fn > 18 || mn < 1 || mn > 60)
		return;

	changed = !rx->cell_valid || rx->mcc != mcc || rx->mnc != mnc || rx->colour_code != cc;
	if (changed)
		tetra_umac_reset(rx);
	tetra_lmac_set_cell(rx, mcc, mnc, cc);
	rx->time.tn = tn;
	rx->time.fn = fn;
	rx->time.mn = mn;
	rx->time_valid = 1;

	rx->st.cell_valid = 1;
	rx->st.mcc = mcc;
	rx->st.mnc = mnc;
	rx->st.colour_code = cc;

	ev = tetra_rx_ev_begin(rx);
	b = &ev->u.bsch;
	b->mcc = mcc;
	b->mnc = mnc;
	b->colour_code = cc;
	b->system_code = sys;
	b->tn = tn;
	b->fn = fn;
	b->mn = mn;
	b->sharing_mode = (uint8_t) bits_to_uint(bits + 23, 2);
	b->ts_reserved_frames = (uint8_t) bits_to_uint(bits + 25, 3);
	b->u_plane_dtx = bits[28];
	b->frame18_ext = bits[29];
	b->neighbour_cell_broadcast = (uint8_t) bits_to_uint(bits + 55, 2);
	b->cell_load = (uint8_t) bits_to_uint(bits + 57, 2);
	b->late_entry = bits[59];
	b->changed = (uint8_t) changed;
	tetra_rx_emit(rx, TETRA_RX_EV_BSCH);
}

/* ---- AACH ---------------------------------------------------------------- */

static uint8_t traffic_enc_state(const struct tetra_rx *rx, uint8_t usage_marker)
{
	uint8_t e = rx->um_enc[usage_marker & 63];

	if (e == TETRA_RX_ENC_UNKNOWN && rx->time.tn >= 1 && rx->time.tn <= 4)
		e = rx->ts_enc[rx->time.tn - 1];
	return e;
}

uint8_t tetra_umac_traffic_enc(const struct tetra_rx *rx)
{
	return traffic_enc_state(rx, rx->cur.is_traffic);
}

/* The traffic usage of a timeslot is remembered for TN_USE_HOLD frames so that
 * a lost ACCESS-ASSIGN does not turn a traffic slot into a signalling slot,
 * and so that what was learned about the encryption of a call is forgotten
 * when the call has left the timeslot. */
#define TN_USE_HOLD 2

static void tn_use_miss(struct tetra_rx *rx, unsigned int tn)
{
	if (!rx->tn_use[tn - 1].usage)
		return;
	if (++rx->tn_use[tn - 1].miss > TN_USE_HOLD) {
		rx->um_enc[rx->tn_use[tn - 1].usage & 63] = TETRA_RX_ENC_UNKNOWN;
		rx->ts_enc[tn - 1] = TETRA_RX_ENC_UNKNOWN;
		rx->tn_use[tn - 1].usage = 0;
		rx->tn_use[tn - 1].miss = 0;
	}
}

/* The ACCESS-ASSIGN of this slot could not be decoded. */
void tetra_umac_aach_lost(struct tetra_rx *rx)
{
	unsigned int tn = rx->time.tn;

	/* frame 18 never carries traffic */
	if (tn < 1 || tn > 4 || rx->time.fn == 18)
		return;
	if (rx->tn_use[tn - 1].usage && rx->tn_use[tn - 1].miss < TN_USE_HOLD)
		rx->cur.is_traffic = rx->tn_use[tn - 1].usage;
	tn_use_miss(rx, tn);
}

void tetra_umac_aach(struct tetra_rx *rx, uint16_t info, int corrected)
{
	struct tetra_acc_ass_decoded aad;
	uint8_t bits[14];
	unsigned int i, tn = rx->time.tn;
	int f18 = rx->time.fn == 18;

	for (i = 0; i < 14; i++)
		bits[i] = (info >> (13 - i)) & 1;
	memset(&aad, 0, sizeof(aad));
	macpdu_decode_access_assign(&aad, bits, f18);

	if ((aad.pres & TETRA_ACC_ASS_PRES_DL_USAGE) && aad.dl_usage > 3)
		rx->cur.is_traffic = (uint8_t) aad.dl_usage;

	/* frame 18 is the control frame of every timeslot: its ACCESS-ASSIGN
	 * says nothing about the usage in frames 1 to 17 */
	if (tn >= 1 && tn <= 4 && !f18) {
		if (rx->cur.is_traffic) {
			if (rx->tn_use[tn - 1].usage != rx->cur.is_traffic) {
				/* another call: what was seen on this timeslot is void */
				rx->ts_enc[tn - 1] = TETRA_RX_ENC_UNKNOWN;
				rx->tn_use[tn - 1].usage = rx->cur.is_traffic;
			}
			rx->tn_use[tn - 1].miss = 0;
		} else
			tn_use_miss(rx, tn);

		rx->st.slot[tn - 1].valid = 1;
		rx->st.slot[tn - 1].dl_usage = (aad.pres & TETRA_ACC_ASS_PRES_DL_USAGE) ?
			(uint8_t) aad.dl_usage : TETRA_RX_DL_COMMON_CCH;
		rx->st.slot[tn - 1].ul_usage = (aad.pres & TETRA_ACC_ASS_PRES_UL_USAGE) ?
			(uint8_t) aad.ul_usage : 0;
		rx->st.slot[tn - 1].enc_state = rx->cur.is_traffic ?
			traffic_enc_state(rx, rx->cur.is_traffic) : TETRA_RX_ENC_UNKNOWN;
	}

	if (rx->cfg.flags & TETRA_RX_FLAG_AACH_EVENTS) {
		tetra_rx_event_t *ev = tetra_rx_ev_begin(rx);
		tetra_rx_ev_aach_t *a = &ev->u.aach;

		a->header = aad.hdr;
		a->frame18 = (uint8_t) f18;
		a->dl_usage_valid = (aad.pres & TETRA_ACC_ASS_PRES_DL_USAGE) ? 1 : 0;
		a->dl_usage = (uint8_t) aad.dl_usage;
		a->ul_usage_valid = (aad.pres & TETRA_ACC_ASS_PRES_UL_USAGE) ? 1 : 0;
		a->ul_usage = (uint8_t) aad.ul_usage;
		for (i = 0; i < 2; i++) {
			a->access_valid[i] = (aad.pres & (TETRA_ACC_ASS_PRES_ACCESS1 << i)) ? 1 : 0;
			a->access_code[i] = aad.access[i].access_code;
			a->base_frame_len[i] = (uint8_t) aad.access[i].base_frame_len;
		}
		a->corrected_bits = (uint8_t) corrected;
		a->raw = info;
		tetra_rx_emit(rx, TETRA_RX_EV_AACH);
	}
}

/* ---- TM-SDU -> LLC -> MLE ------------------------------------------------ */

struct mac_info {
	uint8_t mac_pdu;
	uint8_t lchan;
	tetra_rx_addr_t addr;
	uint8_t encryption_mode;
	uint8_t slot_grant;
	tetra_rx_chan_alloc_t chan_alloc;
};

static void fill_chan_alloc(const struct tetra_rx *rx, tetra_rx_chan_alloc_t *out,
			    const struct tetra_chan_alloc_decoded *cad)
{
	out->present = 1;
	out->alloc_type = cad->type;
	out->timeslots = cad->timeslot;
	out->ul_dl = cad->ul_dl;
	out->clch_permission = cad->clch_perm;
	out->cell_change = cad->cell_chg_f;
	out->carrier = cad->carrier_nr;
	out->ext_carrier = cad->ext_carr_pres;
	if (cad->ext_carr_pres)
		out->dl_freq_hz = tetra_dl_carrier_hz(cad->ext_carr.freq_band, cad->carrier_nr,
						      cad->ext_carr.freq_offset);
	else if (rx->sid_valid)
		out->dl_freq_hz = tetra_dl_carrier_hz(rx->last_sid.freq_band, cad->carrier_nr,
						      rx->last_sid.freq_offset);
	else
		out->dl_freq_hz = 0;
}

static void fill_addr(tetra_rx_addr_t *out, const struct tetra_addr *in)
{
	out->type = in->type;
	out->ssi = in->ssi;
	out->event_label = in->event_label;
	out->usage_marker = in->usage_marker;
}

/* Emit one TETRA_RX_EV_SIGNAL. sdu == NULL: no (readable) TM-SDU. */
static void rx_tm_sdu(struct tetra_rx *rx, const struct mac_info *mi, uint8_t *sdu,
		      unsigned int len, int fragmented, int encrypted)
{
	tetra_rx_event_t *ev = tetra_rx_ev_begin(rx);
	tetra_rx_ev_signal_t *s = &ev->u.signal;
	struct tetra_llc_pdu lpp;

	s->mac_pdu = mi->mac_pdu;
	s->lchan = mi->lchan;
	s->addr = mi->addr;
	s->encryption_mode = mi->encryption_mode;
	s->encrypted = (uint8_t) encrypted;
	s->fragmented = (uint8_t) fragmented;
	s->slot_grant = mi->slot_grant;
	s->chan_alloc = mi->chan_alloc;
	s->pdu_name = "";
	s->sdu = sdu;
	s->sdu_bits = (uint16_t) len;

	if (!sdu || encrypted || len < 4 ||
	    mi->mac_pdu == TETRA_RX_MAC_BROADCAST || mi->mac_pdu == TETRA_RX_MAC_U_SIGNAL)
		goto out;

	memset(&lpp, 0, sizeof(lpp));
	tetra_llc_pdu_parse(&lpp, sdu, (int) len);
	s->have_llc = 1;
	s->llc_pdu = (uint8_t) bits_to_uint(sdu, 4);

	switch (lpp.pdu_type) {
	case TLLC_PDUT_DEC_BL_ADATA:
	case TLLC_PDUT_DEC_BL_DATA:
	case TLLC_PDUT_DEC_BL_UDATA:
	case TLLC_PDUT_DEC_BL_ACK:
		break;
	default:
		/* advanced link segments are not reassembled: LLC type + raw TM-SDU */
		goto out;
	}
	if (lpp.tl_sdu_len > len || lpp.tl_sdu < sdu)
		goto out;
	s->llc_fcs_present = lpp.have_fcs ? 1 : 0;
	s->llc_fcs_ok = (lpp.have_fcs && !lpp.fcs_invalid) ? 1 : 0;
	if (lpp.have_fcs && lpp.fcs_invalid)
		goto out;
	if (lpp.tl_sdu_len < 3)
		goto out;

	/* MLE: protocol discriminator, EN 300 392-2 clause 18.5.21 */
	s->have_mle = 1;
	s->sdu = lpp.tl_sdu;
	s->sdu_bits = (uint16_t) lpp.tl_sdu_len;
	s->pdisc = (uint8_t) bits_to_uint(lpp.tl_sdu, 3);
	s->pdu_name = tetra_get_mle_pdisc_name(s->pdisc);
	switch (s->pdisc) {
	case TETRA_RX_PD_MM:
		if (lpp.tl_sdu_len >= 7) {
			s->pdu_type = (uint8_t) bits_to_uint(lpp.tl_sdu + 3, 4);
			s->pdu_name = tetra_get_mm_pdut_name(s->pdu_type, 0);
		}
		break;
	case TETRA_RX_PD_CMCE:
		if (lpp.tl_sdu_len >= 8) {
			s->pdu_type = (uint8_t) bits_to_uint(lpp.tl_sdu + 3, 5);
			s->pdu_name = tetra_get_cmce_pdut_name(s->pdu_type, 0);
			tetra_cmce_parse(&s->cmce, lpp.tl_sdu + 3, lpp.tl_sdu_len - 3);
			s->have_cmce = 1;
			/* end-to-end encrypted speech is as unusable as air
			 * interface encrypted speech */
			if (mi->addr.type == TETRA_RX_ADDR_SSI_USAGE && s->cmce.parse_ok &&
			    (((s->cmce.fields & TETRA_RX_CMCE_F_BASIC_SERVICE) && s->cmce.e2e_encryption) ||
			     ((s->cmce.fields & TETRA_RX_CMCE_F_ENC_CONTROL) && s->cmce.enc_control)))
				rx->um_enc[mi->addr.usage_marker & 63] = TETRA_RX_ENC_ENCRYPTED;
		}
		break;
	case TETRA_RX_PD_SNDCP:
		if (lpp.tl_sdu_len >= 7) {
			s->pdu_type = (uint8_t) bits_to_uint(lpp.tl_sdu + 3, 4);
			s->pdu_name = tetra_get_sndcp_pdut_name(s->pdu_type, 0);
		}
		break;
	case TETRA_RX_PD_MLE:
		if (lpp.tl_sdu_len >= 6) {
			s->pdu_type = (uint8_t) bits_to_uint(lpp.tl_sdu + 3, 3);
			s->pdu_name = tetra_get_mle_pdut_name(s->pdu_type, 0);
		}
		break;
	default:
		break;
	}
out:
	tetra_rx_emit(rx, TETRA_RX_EV_SIGNAL);
}

/* ---- fragments ----------------------------------------------------------- */

static void frag_drop(struct tetra_rx *rx, struct tetra_fragslot *fs)
{
	if (fs->active == 1)
		rx->st.frag_dropped++;
	fs->active = 0;
	fs->len = 0;
}

static void frag_append(struct tetra_rx *rx, struct tetra_fragslot *fs, const uint8_t *bits, unsigned int len)
{
	if (fs->active != 1)
		return;
	if (fs->len + len > TETRA_FRAG_BITS) {
		frag_drop(rx, fs);
		return;
	}
	memcpy(fs->bits + fs->len, bits, len);
	fs->len = (uint16_t) (fs->len + len);
	fs->age = 0;
}

/* Fill bit deletion, clause 23.4.3.2: remove the last '1' and the zeros
 * after it. Returns the new end or -1 if there is no '1' after 'start'. */
static int strip_fill_bits(const uint8_t *p, unsigned int start, unsigned int end)
{
	while (end > start) {
		end--;
		if (p[end])
			return (int) end;
	}
	return -1;
}

/* ---- MAC PDUs ------------------------------------------------------------ */
/* All handlers return the number of bits consumed, or -1 if nothing more can
 * be parsed in this MAC block. 'rem' is the number of bits left in it. */

static int rx_resource(struct tetra_rx *rx, uint8_t *p, unsigned int rem, uint8_t chan)
{
	struct tetra_fragslot *fs = &rx->frag[(rx->time.tn - 1) & 3];
	struct tetra_resrc_decoded rsd;
	struct mac_info mi;
	unsigned int pdu_len;
	int hdr, end, ret;
	uint8_t enc;

	memset(&rsd, 0, sizeof(rsd));
	hdr = macpdu_decode_resource(&rsd, p, 0);
	if (rsd.addr.type == ADDR_TYPE_NULL || hdr <= 0) {
		/* Null PDU: nothing else in this block. In the first half slot of
		 * the stealing channel its length indication still tells whether
		 * the second half slot is stolen (this is how live networks
		 * announce it when the first half slot has room left). */
		if (rsd.addr.type == ADDR_TYPE_NULL && chan == TETRA_RX_CH_STCH &&
		    rsd.macpdu_length == MACPDU_LEN_2ND_STOLEN)
			rx->cur.blk2_stolen = 1;
		return -1;
	}

	if (rsd.macpdu_length == MACPDU_LEN_2ND_STOLEN) {
		if (chan != TETRA_RX_CH_STCH)
			return -1;      /* only defined on the stealing channel */
		rx->cur.blk2_stolen = 1;
		pdu_len = rem;
		ret = -1;
	} else if (rsd.macpdu_length == MACPDU_LEN_START_FRAG) {
		/* on STCH the final fragment follows in the second half slot */
		if (chan == TETRA_RX_CH_STCH)
			rx->cur.blk2_stolen = 1;
		pdu_len = rem;
		ret = -1;
	} else if (rsd.macpdu_length > 0) {
		pdu_len = (unsigned int) rsd.macpdu_length * 8;
		/* the last PDU may be cut by the end of the MAC block, but only
		 * within its last octet */
		if (pdu_len >= rem + 8)
			return -1;
		if (pdu_len > rem)
			pdu_len = rem;
		ret = (int) pdu_len;
	} else
		return -1;          /* reserved length indication */

	memset(&mi, 0, sizeof(mi));
	mi.mac_pdu = TETRA_RX_MAC_RESOURCE;
	mi.lchan = chan;
	fill_addr(&mi.addr, &rsd.addr);
	mi.encryption_mode = rsd.encryption_mode;
	mi.slot_grant = rsd.slot_granting.pres;
	if (rsd.chan_alloc_pres && !rsd.is_encrypted)
		fill_chan_alloc(rx, &mi.chan_alloc, &rsd.cad);

	/* air interface encryption state seen for this usage marker / slot */
	enc = rsd.encryption_mode ? TETRA_RX_ENC_ENCRYPTED : TETRA_RX_ENC_CLEAR;
	/* The PDU that assigns the usage marker decides. Other PDUs on the
	 * stealing channel are only a hint: one encrypted PDU marks the call
	 * as encrypted, clear ones (e.g. broadcasts) do not clear that again. */
	if (rsd.addr.type == ADDR_TYPE_SSI_USAGE)
		rx->um_enc[rsd.addr.usage_marker & 63] = enc;
	if (chan == TETRA_RX_CH_STCH) {
		uint8_t *ts = &rx->ts_enc[(rx->time.tn - 1) & 3];
		uint8_t *um = &rx->um_enc[rx->cur.is_traffic & 63];

		if (enc == TETRA_RX_ENC_ENCRYPTED || *ts == TETRA_RX_ENC_UNKNOWN)
			*ts = enc;
		if (rx->cur.is_traffic && (enc == TETRA_RX_ENC_ENCRYPTED || *um == TETRA_RX_ENC_UNKNOWN))
			*um = enc;
	}

	if (rsd.macpdu_length == MACPDU_LEN_START_FRAG)
		frag_drop(rx, fs);  /* clause 23.4.3.1.1 b) i) */

	if (rsd.is_encrypted) {
		/* channel allocation and TM-SDU are encrypted: header only */
		if (rsd.macpdu_length == MACPDU_LEN_START_FRAG)
			fs->active = 2; /* swallow the continuation silently */
		rx_tm_sdu(rx, &mi, NULL, 0, rsd.macpdu_length == MACPDU_LEN_START_FRAG, 1);
		return ret;
	}

	end = (int) pdu_len;
	if (rsd.fill_bits)
		end = strip_fill_bits(p, (unsigned int) hdr, pdu_len);
	if (end < hdr)
		return -1;          /* inconsistent header */

	if (rsd.macpdu_length == MACPDU_LEN_START_FRAG) {
		fs->active = 1;
		fs->age = 0;
		fs->len = 0;
		fs->hdr.addr = mi.addr;
		fs->hdr.encryption_mode = mi.encryption_mode;
		fs->hdr.lchan = chan;
		fs->hdr.slot_grant = mi.slot_grant;
		fs->hdr.chan_alloc = mi.chan_alloc;
		frag_append(rx, fs, p + hdr, (unsigned int) (end - hdr));
		return -1;
	}

	rx_tm_sdu(rx, &mi, end > hdr ? p + hdr : NULL, (unsigned int) (end - hdr), 0, 0);
	return ret;
}

static int rx_frag(struct tetra_rx *rx, uint8_t *p, unsigned int rem)
{
	struct tetra_fragslot *fs = &rx->frag[(rx->time.tn - 1) & 3];
	int end = (int) rem;

	if (p[3])
		end = strip_fill_bits(p, 4, rem);
	if (end < 4) {
		frag_drop(rx, fs);
		return -1;
	}
	if (fs->active == 2)
		fs->age = 0;
	frag_append(rx, fs, p + 4, (unsigned int) end - 4);
	return -1;              /* MAC-FRAG always fills the block */
}

static int rx_end(struct tetra_rx *rx, uint8_t *p, unsigned int rem)
{
	struct tetra_fragslot *fs = &rx->frag[(rx->time.tn - 1) & 3];
	struct tetra_chan_alloc_decoded cad;
	struct mac_info mi;
	unsigned int len_ind = bits_to_uint(p + 5, 6);
	unsigned int pdu_len, n = 11;
	uint8_t slot_grant, chan_alloc;
	int end;

	/* table 21.59: 2..34 octets */
	if (len_ind < 2 || len_ind > 34 || len_ind * 8 >= rem + 8) {
		frag_drop(rx, fs);
		return -1;
	}
	pdu_len = len_ind * 8;
	if (pdu_len > rem)
		pdu_len = rem;
	if (fs->active != 1) {
		/* no (readable) start: nothing to deliver */
		fs->active = 0;
		return (int) pdu_len;
	}

	slot_grant = p[n++];
	if (slot_grant)
		n += 8;
	chan_alloc = p[n++];
	memset(&cad, 0, sizeof(cad));
	if (chan_alloc)
		n += (unsigned int) macpdu_decode_chan_alloc(&cad, p + n);
	end = (int) pdu_len;
	if (p[3])
		end = strip_fill_bits(p, n, pdu_len);
	if (end < (int) n) {
		frag_drop(rx, fs);
		return (int) pdu_len;
	}
	frag_append(rx, fs, p + n, (unsigned int) end - n);
	if (fs->active != 1)
		return (int) pdu_len;   /* overflow */

	memset(&mi, 0, sizeof(mi));
	mi.mac_pdu = TETRA_RX_MAC_RESOURCE;
	mi.lchan = fs->hdr.lchan;
	mi.addr = fs->hdr.addr;
	mi.encryption_mode = fs->hdr.encryption_mode;
	mi.slot_grant = fs->hdr.slot_grant | slot_grant;
	mi.chan_alloc = fs->hdr.chan_alloc;
	if (chan_alloc)
		fill_chan_alloc(rx, &mi.chan_alloc, &cad);
	fs->active = 0;
	rx_tm_sdu(rx, &mi, fs->bits, fs->len, 1, 0);
	fs->len = 0;
	return (int) pdu_len;
}

static int rx_suppl(struct tetra_rx *rx, uint8_t *p, unsigned int rem, uint8_t chan)
{
	struct mac_info mi;
	unsigned int hdr;
	int end;

	memset(&mi, 0, sizeof(mi));
	mi.lchan = chan;

	if (chan == TETRA_RX_CH_STCH) {
		/* MAC-U-SIGNAL, table 21.80: U-plane signalling of the users */
		if (rem < 124)
			return -1;
		if (p[2] && !rx->cur.in_blk2)
			rx->cur.blk2_stolen = 1;
		mi.mac_pdu = TETRA_RX_MAC_U_SIGNAL;
		rx_tm_sdu(rx, &mi, p + 3, 121, 0, 0);
		return -1;
	}

	/* MAC-D-BLCK, table 21.61 (fills the MAC block) */
	if (p[2] != 0 || rem < 18)
		return -1;
	mi.mac_pdu = TETRA_RX_MAC_D_BLCK;
	mi.encryption_mode = (uint8_t) bits_to_uint(p + 4, 2);
	mi.addr.type = TETRA_RX_ADDR_EVENT_LABEL;
	mi.addr.event_label = (uint16_t) bits_to_uint(p + 6, 10);
	mi.slot_grant = p[17];
	hdr = 18 + (mi.slot_grant ? 8u : 0u);
	if (mi.encryption_mode) {
		rx_tm_sdu(rx, &mi, NULL, 0, 0, 1);
		return -1;
	}
	end = (int) rem;
	if (p[3])
		end = strip_fill_bits(p, hdr, rem);
	if (end < (int) hdr)
		return -1;
	rx_tm_sdu(rx, &mi, end > (int) hdr ? p + hdr : NULL, (unsigned int) end - hdr, 0, 0);
	return -1;
}

static void rx_sysinfo(struct tetra_rx *rx, const uint8_t *p)
{
	struct tetra_si_decoded *sid = &rx->last_sid;
	tetra_rx_event_t *ev;
	tetra_rx_ev_sysinfo_t *s;

	memset(sid, 0, sizeof(*sid));
	macpdu_decode_sysinfo(sid, p);
	rx->sid_valid = 1;

	ev = tetra_rx_ev_begin(rx);
	s = &ev->u.sysinfo;
	s->main_carrier = sid->main_carrier;
	s->freq_band = sid->freq_band;
	s->freq_offset = sid->freq_offset;
	s->duplex_spacing = sid->duplex_spacing;
	s->reverse_operation = sid->reverse_operation;
	s->dl_freq_hz = tetra_dl_carrier_hz(sid->freq_band, sid->main_carrier, sid->freq_offset);
	s->ul_freq_hz = tetra_ul_carrier_hz(sid->freq_band, sid->main_carrier, sid->freq_offset,
					    sid->duplex_spacing, sid->reverse_operation);
	s->num_secondary_cch = sid->num_of_csch;
	s->ms_txpwr_max_cell = sid->ms_txpwr_max_cell;
	s->rxlev_access_min = sid->rxlev_access_min;
	s->access_parameter = sid->access_parameter;
	s->radio_dl_timeout = sid->radio_dl_timeout;
	s->cck_valid = sid->cck_valid_no_hf ? 1 : 0;
	if (sid->cck_valid_no_hf)
		s->cck_id = sid->cck_id;
	else
		s->hyperframe = sid->hyperframe_number;
	s->option_field = sid->option_field;
	s->option_value = sid->frame_bitmap;
	s->location_area = sid->mle_si.la;
	s->subscriber_class = sid->mle_si.subscr_class;
	s->bs_service_details = sid->mle_si.bs_service_details;

	rx->st.sysinfo = *s;
	rx->st.sysinfo_valid = 1;
	tetra_rx_emit(rx, TETRA_RX_EV_SYSINFO);
}

static int rx_bcast(struct tetra_rx *rx, uint8_t *p, unsigned int rem, uint8_t chan)
{
	unsigned int btype = bits_to_uint(p + 2, 2);
	struct mac_info mi;
	unsigned int len = rem;
	int ret = -1;

	if (btype == TETRA_MAC_BC_SYSINFO) {
		if (rem < 124)
			return -1;
		rx_sysinfo(rx, p);
		return -1;
	}
	if (btype == TETRA_MAC_BC_ACCESS_DEFINE) {
		/* table 21.79: 29 bits, optional field, 3 filler bits */
		static const uint8_t opt_len[4] = { 0, 16, 24, 0 };
		unsigned int opt;

		if (rem < 32)
			return -1;
		opt = bits_to_uint(p + 27, 2);
		if (opt == 3)
			return -1;
		len = 32u + opt_len[opt];
		if (len > rem)
			return -1;
		ret = (int) len;
	}
	memset(&mi, 0, sizeof(mi));
	mi.mac_pdu = TETRA_RX_MAC_BROADCAST;
	mi.lchan = chan;
	rx_tm_sdu(rx, &mi, p, len, 0, 0);
	return ret;
}

/* One CRC-checked MAC block (bits != NULL) or a block that failed its CRC. */
void tetra_umac_block(struct tetra_rx *rx, uint8_t *bits, unsigned int n_bits, uint8_t chan)
{
	struct tetra_fragslot *fs = &rx->frag[(rx->time.tn - 1) & 3];
	unsigned int off = 0;

	if (!bits) {
		/* clause 23.4.3.1.1 b) ii): a lost MAC block ends a reassembly */
		frag_drop(rx, fs);
		return;
	}

	while (off + 16 < n_bits) {
		uint8_t *p = bits + off;
		unsigned int rem = n_bits - off;
		int n;

		switch (bits_to_uint(p, 2)) {
		case TETRA_PDU_T_MAC_RESOURCE:
			n = rx_resource(rx, p, rem, chan);
			break;
		case TETRA_PDU_T_MAC_FRAG_END:
			/* continuation fragments are the first PDU of a block */
			if (p[2] == TETRA_MAC_FRAGE_FRAG)
				n = rx_frag(rx, p, rem);
			else
				n = rx_end(rx, p, rem);
			break;
		case TETRA_PDU_T_BROADCAST:
			n = rx_bcast(rx, p, rem, chan);
			break;
		default:
			n = rx_suppl(rx, p, rem, chan);
			break;
		}
		if (n <= 0)
			break;
		off += (unsigned int) n;
	}
}

/* End of a slot: fragment ageing, clause 23.4.3.1.1 b) iii). */
void tetra_umac_slot_end(struct tetra_rx *rx, int demodulated)
{
	struct tetra_fragslot *fs = &rx->frag[(rx->time.tn - 1) & 3];

	if (!fs->active)
		return;
	if (!demodulated) {
		frag_drop(rx, fs);
		return;
	}
	if (++fs->age > TETRA_N203)
		frag_drop(rx, fs);
}
