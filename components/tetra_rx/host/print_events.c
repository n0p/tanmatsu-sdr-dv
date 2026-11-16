/* text formatting of tetra_rx events (CLI and tests)
 * SPDX-License-Identifier: AGPL-3.0-or-later */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "print_events.h"

#include <tetra_llc_pdu.h>

struct sb {
	char *buf;
	size_t size, len;
};

static void put(struct sb *s, const char *fmt, ...)
{
	va_list ap;
	int n;

	if (s->len + 1 >= s->size)
		return;
	va_start(ap, fmt);
	n = vsnprintf(s->buf + s->len, s->size - s->len, fmt, ap);
	va_end(ap);
	if (n > 0)
		s->len += (size_t) n < s->size - s->len ? (size_t) n : s->size - s->len - 1;
}

static void put_bits_hex(struct sb *s, const uint8_t *bits, unsigned int n)
{
	unsigned int i, v = 0;

	for (i = 0; i < n; i++) {
		v = (v << 1) | (bits[i] & 1);
		if ((i & 3) == 3) {
			put(s, "%x", v);
			v = 0;
		}
	}
	if (n & 3)
		put(s, "%x", v << (4 - (n & 3)));
}

static const char *mac_name(uint8_t t)
{
	switch (t) {
	case TETRA_RX_MAC_RESOURCE:
		return "MAC-RESOURCE";
	case TETRA_RX_MAC_BROADCAST:
		return "MAC-BROADCAST";
	case TETRA_RX_MAC_D_BLCK:
		return "MAC-D-BLCK";
	case TETRA_RX_MAC_U_SIGNAL:
		return "MAC-U-SIGNAL";
	default:
		return "MAC-?";
	}
}

static void put_addr(struct sb *s, const tetra_rx_addr_t *a)
{
	switch (a->type) {
	case TETRA_RX_ADDR_SSI:
		put(s, " SSI=%u", a->ssi);
		break;
	case TETRA_RX_ADDR_EVENT_LABEL:
		put(s, " EL=%u", a->event_label);
		break;
	case TETRA_RX_ADDR_USSI:
		put(s, " USSI=%u", a->ssi);
		break;
	case TETRA_RX_ADDR_SMI:
		put(s, " SMI=%u", a->ssi);
		break;
	case TETRA_RX_ADDR_SSI_EVENT:
		put(s, " SSI=%u EL=%u", a->ssi, a->event_label);
		break;
	case TETRA_RX_ADDR_SSI_USAGE:
		put(s, " SSI=%u UM=%u", a->ssi, a->usage_marker);
		break;
	case TETRA_RX_ADDR_SMI_EVENT:
		put(s, " SMI=%u EL=%u", a->ssi, a->event_label);
		break;
	default:
		break;
	}
}

static void put_cmce(struct sb *s, const tetra_rx_cmce_t *c)
{
	if (!c->parse_ok) {
		put(s, " [not parsed]");
		return;
	}
	if (c->fields & TETRA_RX_CMCE_F_CALL_ID)
		put(s, " call_id=%u", c->call_id);
	if (c->fields & TETRA_RX_CMCE_F_BASIC_SERVICE)
		put(s, " svc(circuit=%u e2ee=%u comm=%u speech/slots=%u)", c->circuit_mode,
		    c->e2e_encryption, c->comm_type, c->slots_or_speech);
	if (c->fields & TETRA_RX_CMCE_F_SIMPLEX)
		put(s, " %s", c->duplex ? "duplex" : "simplex");
	if (c->fields & TETRA_RX_CMCE_F_HOOK)
		put(s, " hook=%u", c->hook_method);
	if (c->fields & TETRA_RX_CMCE_F_CALL_TIMEOUT)
		put(s, " timeout=%u", c->call_timeout);
	if (c->fields & TETRA_RX_CMCE_F_TX_GRANT)
		put(s, " tx_grant=%u", c->tx_grant);
	if (c->fields & TETRA_RX_CMCE_F_TX_REQ_PERM)
		put(s, " tx_req_perm=%u", c->tx_req_permission);
	if (c->fields & TETRA_RX_CMCE_F_ENC_CONTROL)
		put(s, " enc_control=%u", c->enc_control);
	if (c->fields & TETRA_RX_CMCE_F_PRIORITY)
		put(s, " prio=%u", c->call_priority);
	if (c->fields & TETRA_RX_CMCE_F_CALL_OWNER)
		put(s, " owner=%u", c->call_ownership);
	if (c->fields & TETRA_RX_CMCE_F_NOTIFICATION)
		put(s, " notif=%u", c->notification);
	if (c->fields & TETRA_RX_CMCE_F_DISC_CAUSE)
		put(s, " cause=%u", c->disconnect_cause);
	if (c->fields & TETRA_RX_CMCE_F_TEMP_ADDR)
		put(s, " temp_addr=%u", c->temp_address);
	if (c->fields & TETRA_RX_CMCE_F_PARTY) {
		if (c->party.present == 2)
			put(s, " party=%u:%u:%u", c->party.mcc, c->party.mnc, c->party.ssi);
		else
			put(s, " party=%u", c->party.ssi);
	}
	if (c->fields & TETRA_RX_CMCE_F_STATUS)
		put(s, " status=%u", c->status);
	if (c->fields & TETRA_RX_CMCE_F_SDS) {
		put(s, " sds_type=%u bits=%u", c->sds_type, c->sds_bits);
		if (c->fields & TETRA_RX_CMCE_F_PROTOCOL_ID)
			put(s, " pid=0x%02x", c->protocol_id);
		if (c->fields & TETRA_RX_CMCE_F_TEXT)
			put(s, " coding=%u text=\"%s\"", c->text_coding, c->text);
		else if (c->sds_data) {
			put(s, " data=");
			put_bits_hex(s, c->sds_data, c->sds_bits);
		}
	}
}

int format_event(char *buf, size_t size, const tetra_rx_event_t *ev)
{
	struct sb s = { buf, size, 0 };
	unsigned int i;

	if (size)
		buf[0] = 0;
	if (ev->time.valid)
		put(&s, "%02u/%02u/%u ", ev->time.mn, ev->time.fn, ev->time.tn);
	else
		put(&s, "--/--/- ");

	switch (ev->type) {
	case TETRA_RX_EV_SYNC:
		put(&s, "SYNC %s freq=%+.0f Hz", ev->u.sync.locked ? "locked" : "lost",
		    (double) ev->u.sync.freq_offset_hz);
		break;
	case TETRA_RX_EV_BSCH: {
		const tetra_rx_ev_bsch_t *b = &ev->u.bsch;

		put(&s, "BSCH MCC=%u MNC=%u CC=%u sys=%u sharing=%u rsvd=%u dtx=%u f18ext=%u ncb=%u load=%u late=%u%s",
		    b->mcc, b->mnc, b->colour_code, b->system_code, b->sharing_mode, b->ts_reserved_frames,
		    b->u_plane_dtx, b->frame18_ext, b->neighbour_cell_broadcast, b->cell_load, b->late_entry,
		    b->changed ? " NEW" : "");
		break;
	}
	case TETRA_RX_EV_SYSINFO: {
		const tetra_rx_ev_sysinfo_t *si = &ev->u.sysinfo;

		put(&s, "SYSINFO carrier=%u band=%u offset=%u duplex=%u rev=%u DL=%u UL=%u LA=%u "
		    "sccch=%u txpwr=%u rxlev=%u acc=%u rdt=%u ", si->main_carrier, si->freq_band,
		    si->freq_offset, si->duplex_spacing, si->reverse_operation, si->dl_freq_hz, si->ul_freq_hz,
		    si->location_area, si->num_secondary_cch, si->ms_txpwr_max_cell, si->rxlev_access_min,
		    si->access_parameter, si->radio_dl_timeout);
		if (si->cck_valid)
			put(&s, "cck=%u ", si->cck_id);
		else
			put(&s, "hyperframe=%u ", si->hyperframe);
		put(&s, "opt=%u:0x%05x subscr=0x%04x svc=0x%03x", si->option_field, si->option_value,
		    si->subscriber_class, si->bs_service_details);
		break;
	}
	case TETRA_RX_EV_AACH: {
		const tetra_rx_ev_aach_t *a = &ev->u.aach;

		put(&s, "AACH hdr=%u", a->header);
		if (a->dl_usage_valid)
			put(&s, " dl=%u", a->dl_usage);
		if (a->ul_usage_valid)
			put(&s, " ul=%u", a->ul_usage);
		for (i = 0; i < 2; i++)
			if (a->access_valid[i])
				put(&s, " acc%u=%c/%u", i + 1, 'A' + a->access_code[i], a->base_frame_len[i]);
		put(&s, " raw=0x%04x fix=%u", a->raw, a->corrected_bits);
		break;
	}
	case TETRA_RX_EV_SIGNAL: {
		const tetra_rx_ev_signal_t *g = &ev->u.signal;

		put(&s, "SIG %s %s", tetra_rx_chan_name(g->lchan), mac_name(g->mac_pdu));
		put_addr(&s, &g->addr);
		if (g->encryption_mode)
			put(&s, " ENCRYPTED(mode %u)", g->encryption_mode);
		if (g->fragmented)
			put(&s, " frag");
		if (g->slot_grant)
			put(&s, " grant");
		if (g->chan_alloc.present)
			put(&s, " chan_alloc(type=%u ts=0x%x updl=%u carrier=%u DL=%u)", g->chan_alloc.alloc_type,
			    g->chan_alloc.timeslots, g->chan_alloc.ul_dl, g->chan_alloc.carrier,
			    g->chan_alloc.dl_freq_hz);
		if (g->have_llc) {
			put(&s, " %s", tetra_get_llc_pdut_name(g->llc_pdu));
			if (g->llc_fcs_present)
				put(&s, "(FCS %s)", g->llc_fcs_ok ? "ok" : "BAD");
		}
		if (g->have_mle) {
			put(&s, " %s", tetra_rx_pdisc_name(g->pdisc));
			if (g->pdu_name[0] && g->pdu_name != tetra_rx_pdisc_name(g->pdisc))
				put(&s, " %s", g->pdu_name);
			if (g->have_cmce)
				put_cmce(&s, &g->cmce);
		}
		if (g->sdu && g->sdu_bits) {
			put(&s, " sdu[%u]=", g->sdu_bits);
			put_bits_hex(&s, g->sdu, g->sdu_bits);
		}
		break;
	}
	case TETRA_RX_EV_TRAFFIC: {
		const tetra_rx_ev_traffic_t *t = &ev->u.traffic;
		static const char *const enc[] = { "enc?", "clear", "ENCRYPTED" };

		put(&s, "TCH/S um=%u %s frames=%u stolen=%u%u bfi=%u%u", t->usage_marker, enc[t->enc_state % 3],
		    t->n_frames, t->stolen[0], t->stolen[1], t->bfi[0], t->bfi[1]);
		for (i = 0; i < 2; i++) {
			if (i < t->n_frames || (t->n_frames == 1 && !t->bfi[i])) {
				if (t->bfi[i])
					continue;
				put(&s, " f%u=", i);
				put_bits_hex(&s, t->frame[i], TETRA_RX_SPEECH_FRAME_BITS);
			}
		}
		break;
	}
	case TETRA_RX_EV_BLOCK: {
		const tetra_rx_ev_block_t *b = &ev->u.block;

		put(&s, "BLOCK %s blk=%u crc=%u ", tetra_rx_chan_name(b->chan), b->blk_num, b->crc_ok);
		put_bits_hex(&s, b->bits, b->n_bits);
		break;
	}
	case TETRA_RX_EV_BURST:
		put(&s, "BURST type=%u score=%.2f", ev->u.burst.burst_type, (double) ev->u.burst.sync_score);
		break;
	default:
		put(&s, "?");
		break;
	}
	return (int) s.len;
}

void print_status(FILE *f, const tetra_rx_status_t *st)
{
	unsigned int i;

	fprintf(f, "status: locked=%u freq_offset=%+.1f Hz timing=%+.1f ppm level=%.1f dBFS snr=%.1f dB\n",
		st->locked, (double) st->freq_offset_hz, (double) st->timing_ppm, (double) st->level_dbfs,
		(double) st->evm_snr_db);
	fprintf(f, "        slots=%u locks=%u sync_losses=%u training BER=%.2e (%u/%u) frag_dropped=%u\n",
		st->slots, st->lock_count, st->sync_losses,
		st->train_bits ? (double) st->train_bit_errors / st->train_bits : 0.0,
		st->train_bit_errors, st->train_bits, st->frag_dropped);
	for (i = 0; i < TETRA_RX_CH_COUNT; i++) {
		uint32_t tot = st->chan[i].ok + st->chan[i].fail;

		fprintf(f, "        %-7s ok=%-6u fail=%-6u (%.2f %% ok)\n", tetra_rx_chan_name((uint8_t) i),
			st->chan[i].ok, st->chan[i].fail, tot ? 100.0 * st->chan[i].ok / tot : 0.0);
	}
	if (st->cell_valid)
		fprintf(f, "        cell: MCC=%u MNC=%u CC=%u\n", st->mcc, st->mnc, st->colour_code);
	if (st->sysinfo_valid)
		fprintf(f, "        sysinfo: DL=%u Hz UL=%u Hz LA=%u svc=0x%03x\n", st->sysinfo.dl_freq_hz,
			st->sysinfo.ul_freq_hz, st->sysinfo.location_area, st->sysinfo.bs_service_details);
	for (i = 0; i < 4; i++)
		if (st->slot[i].valid)
			fprintf(f, "        TN%u: dl_usage=%u ul_usage=%u enc_state=%u\n", i + 1,
				st->slot[i].dl_usage, st->slot[i].ul_usage, st->slot[i].enc_state);
}

uint64_t fnv1a(uint64_t h, const char *s)
{
	if (!h)
		h = 0xcbf29ce484222325ull;
	while (*s) {
		h ^= (uint8_t) *s++;
		h *= 0x100000001b3ull;
	}
	return h;
}
