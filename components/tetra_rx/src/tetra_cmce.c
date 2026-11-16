/* tetra_rx - CMCE downlink PDU parser (EN 300 392-2 clause 14.7.1, 14.8 and
 * the SDS-TL text services of clause 29).
 *
 * Original code for the tetra_rx component (osmo-tetra only names the CMCE
 * PDU types). Every element is read through a bounds-checked bit reader; a
 * PDU whose encoding does not fit its length is reported with parse_ok = 0
 * and no fields rather than with guessed values.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <stdint.h>
#include <string.h>

#include "tetra_ctx.h"

struct br {
	const uint8_t *b;
	unsigned int len;
	unsigned int pos;
	int err;
};

static uint32_t rd(struct br *r, unsigned int n)
{
	uint32_t v = 0;
	unsigned int i;

	if (r->err || n > r->len - r->pos) {
		r->err = 1;
		return 0;
	}
	for (i = 0; i < n; i++)
		v = (v << 1) | (r->b[r->pos + i] & 1u);
	r->pos += n;
	return v;
}

/* O-bit: returns 1 if optional elements follow */
static int obit(struct br *r)
{
	return (int) rd(r, 1);
}

/* P-bit of a type 2 element */
static int pbit(struct br *r)
{
	return (int) rd(r, 1);
}

/* Type 3/4 elements: M-bit, 4 bit identifier, 11 bit length, data ...
 * They are skipped (length checked). */
static void skip_type34(struct br *r)
{
	while (!r->err && rd(r, 1)) {
		unsigned int len;

		(void) rd(r, 4);
		len = rd(r, 11);
		if (r->err || len > r->len - r->pos) {
			r->err = 1;
			return;
		}
		r->pos += len;
	}
}

static void basic_service(tetra_rx_cmce_t *c, struct br *r)
{
	c->circuit_mode = (uint8_t) rd(r, 3);
	c->e2e_encryption = (uint8_t) rd(r, 1);
	c->comm_type = (uint8_t) rd(r, 2);
	c->slots_or_speech = (uint8_t) rd(r, 2);
	c->fields |= TETRA_RX_CMCE_F_BASIC_SERVICE;
}

/* calling / transmitting party type identifier + conditional address */
static void party(tetra_rx_cmce_t *c, struct br *r)
{
	unsigned int cpti = rd(r, 2);

	switch (cpti) {
	case 1:
		c->party.ssi = rd(r, 24);
		c->party.present = 1;
		break;
	case 2:
		c->party.ssi = rd(r, 24);
		c->party.mcc = (uint16_t) rd(r, 10);
		c->party.mnc = (uint16_t) rd(r, 14);
		c->party.present = 2;
		break;
	default:
		/* 0 and 3 are reserved: the layout that follows is unknown */
		r->err = 1;
		return;
	}
	c->fields |= TETRA_RX_CMCE_F_PARTY;
}

static void call_id(tetra_rx_cmce_t *c, struct br *r)
{
	c->call_id = (uint16_t) rd(r, 14);
	c->fields |= TETRA_RX_CMCE_F_CALL_ID;
}

static void notification(tetra_rx_cmce_t *c, struct br *r)
{
	if (pbit(r)) {
		c->notification = (uint8_t) rd(r, 6);
		c->fields |= TETRA_RX_CMCE_F_NOTIFICATION;
	}
}

static void temp_address(tetra_rx_cmce_t *c, struct br *r)
{
	if (pbit(r)) {
		c->temp_address = rd(r, 24);
		c->fields |= TETRA_RX_CMCE_F_TEMP_ADDR;
	}
}

static void tx_grant(tetra_rx_cmce_t *c, struct br *r)
{
	c->tx_grant = (uint8_t) rd(r, 2);
	c->tx_req_permission = (uint8_t) rd(r, 1);
	c->fields |= TETRA_RX_CMCE_F_TX_GRANT | TETRA_RX_CMCE_F_TX_REQ_PERM;
}

/* ---- text ---------------------------------------------------------------- */

static unsigned int utf8_put(char *out, unsigned int pos, unsigned int max, uint32_t cp)
{
	if (cp == 0)
		cp = 0xfffd;
	if (cp < 0x80) {
		if (pos + 1 >= max)
			return pos;
		out[pos++] = (char) cp;
	} else if (cp < 0x800) {
		if (pos + 2 >= max)
			return pos;
		out[pos++] = (char) (0xc0 | (cp >> 6));
		out[pos++] = (char) (0x80 | (cp & 0x3f));
	} else if (cp < 0x10000) {
		if (pos + 3 >= max)
			return pos;
		out[pos++] = (char) (0xe0 | (cp >> 12));
		out[pos++] = (char) (0x80 | ((cp >> 6) & 0x3f));
		out[pos++] = (char) (0x80 | (cp & 0x3f));
	} else {
		if (pos + 4 >= max)
			return pos;
		out[pos++] = (char) (0xf0 | (cp >> 18));
		out[pos++] = (char) (0x80 | ((cp >> 12) & 0x3f));
		out[pos++] = (char) (0x80 | ((cp >> 6) & 0x3f));
		out[pos++] = (char) (0x80 | (cp & 0x3f));
	}
	return pos;
}

/* Text of the SDS-TL text messaging / simple text messaging services
 * (EN 300 392-2 clause 29.5.4). Only the 8 bit Latin-1 and the UCS-2/UTF-16BE
 * coding schemes are converted; anything else stays raw. */
static void decode_text(tetra_rx_cmce_t *c, struct br *r, unsigned int coding)
{
	unsigned int pos = 0;
	unsigned int left = r->len - r->pos;

	c->text_coding = (uint8_t) coding;
	if (coding == 1 && (left % 8) == 0) {
		while (r->len - r->pos >= 8)
			pos = utf8_put(c->text, pos, TETRA_RX_TEXT_MAX, rd(r, 8));
	} else if (coding == 26 && (left % 16) == 0) {
		while (r->len - r->pos >= 16) {
			uint32_t cp = rd(r, 16);

			if (cp >= 0xd800 && cp < 0xdc00 && r->len - r->pos >= 16) {
				uint32_t lo = rd(r, 16);

				if (lo >= 0xdc00 && lo < 0xe000)
					cp = 0x10000 + ((cp - 0xd800) << 10) + (lo - 0xdc00);
				else
					cp = 0xfffd;
			} else if (cp >= 0xd800 && cp < 0xe000)
				cp = 0xfffd;
			pos = utf8_put(c->text, pos, TETRA_RX_TEXT_MAX, cp);
		}
	} else
		return;
	c->text[pos] = 0;
	c->fields |= TETRA_RX_CMCE_F_TEXT;
}

static void sds_type4(tetra_rx_cmce_t *c, const uint8_t *data, unsigned int len)
{
	struct br r = { data, len, 0, 0 };
	unsigned int pid, coding, ts;

	if (len < 8)
		return;
	pid = rd(&r, 8);
	c->protocol_id = (uint8_t) pid;
	c->fields |= TETRA_RX_CMCE_F_PROTOCOL_ID;

	if (pid == 0x02 || pid == 0x09) {
		/* simple text messaging: reserved bit + text coding scheme */
		(void) rd(&r, 1);
		coding = rd(&r, 7);
		if (!r.err)
			decode_text(c, &r, coding);
		return;
	}
	if (pid == 0x82 || pid == 0x89) {
		/* SDS-TL, table 29.20: only SDS-TRANSFER carries user text */
		unsigned int mt = rd(&r, 4);
		unsigned int sfc;

		if (r.err || mt != 0)
			return;
		(void) rd(&r, 2);       /* delivery report request */
		(void) rd(&r, 1);       /* service selection / short form report */
		sfc = rd(&r, 1);        /* storage / forward control */
		(void) rd(&r, 8);       /* message reference */
		if (sfc) {
			unsigned int fat;

			(void) rd(&r, 5);   /* validity period */
			fat = rd(&r, 3);    /* forward address type */
			switch (fat) {
			case 0: (void) rd(&r, 8); break;   /* SNA */
			case 1: (void) rd(&r, 24); break;  /* SSI */
			case 2: (void) rd(&r, 48); break;  /* TSI */
			case 7: break;                     /* no forward address */
			default: return;                   /* external number: not parsed */
			}
		}
		ts = rd(&r, 1);         /* timestamp used */
		coding = rd(&r, 7);
		if (ts)
			(void) rd(&r, 24);
		if (!r.err)
			decode_text(c, &r, coding);
	}
}

/* ---- PDUs ---------------------------------------------------------------- */

void tetra_cmce_parse(tetra_rx_cmce_t *c, const uint8_t *bits, unsigned int len)
{
	struct br r = { bits, len, 0, 0 };
	unsigned int pdu_type;
	int known = 1;

	memset(c, 0, sizeof(*c));
	pdu_type = rd(&r, 5);

	switch (pdu_type) {
	case TETRA_RX_CMCE_D_ALERT:
		call_id(c, &r);
		(void) rd(&r, 3);       /* call time-out, set-up phase */
		(void) rd(&r, 1);       /* reserved */
		c->duplex = (uint8_t) rd(&r, 1);
		(void) rd(&r, 1);       /* call queued */
		c->fields |= TETRA_RX_CMCE_F_SIMPLEX;
		if (obit(&r)) {
			if (pbit(&r))
				basic_service(c, &r);
			notification(c, &r);
			skip_type34(&r);
		}
		break;
	case TETRA_RX_CMCE_D_CALL_PROCEEDING:
		call_id(c, &r);
		(void) rd(&r, 3);       /* call time-out, set-up phase */
		c->hook_method = (uint8_t) rd(&r, 1);
		c->duplex = (uint8_t) rd(&r, 1);
		c->fields |= TETRA_RX_CMCE_F_SIMPLEX | TETRA_RX_CMCE_F_HOOK;
		if (obit(&r)) {
			if (pbit(&r))
				basic_service(c, &r);
			if (pbit(&r))
				(void) rd(&r, 3);   /* call status */
			notification(c, &r);
			skip_type34(&r);
		}
		break;
	case TETRA_RX_CMCE_D_CONNECT:
		call_id(c, &r);
		c->call_timeout = (uint8_t) rd(&r, 4);
		c->hook_method = (uint8_t) rd(&r, 1);
		c->duplex = (uint8_t) rd(&r, 1);
		tx_grant(c, &r);
		c->call_ownership = (uint8_t) rd(&r, 1);
		c->fields |= TETRA_RX_CMCE_F_CALL_TIMEOUT | TETRA_RX_CMCE_F_HOOK |
			     TETRA_RX_CMCE_F_SIMPLEX | TETRA_RX_CMCE_F_CALL_OWNER;
		if (obit(&r)) {
			if (pbit(&r)) {
				c->call_priority = (uint8_t) rd(&r, 4);
				c->fields |= TETRA_RX_CMCE_F_PRIORITY;
			}
			if (pbit(&r))
				basic_service(c, &r);
			temp_address(c, &r);
			notification(c, &r);
			skip_type34(&r);
		}
		break;
	case TETRA_RX_CMCE_D_CONNECT_ACK:
		call_id(c, &r);
		c->call_timeout = (uint8_t) rd(&r, 4);
		tx_grant(c, &r);
		c->fields |= TETRA_RX_CMCE_F_CALL_TIMEOUT;
		if (obit(&r)) {
			notification(c, &r);
			skip_type34(&r);
		}
		break;
	case TETRA_RX_CMCE_D_DISCONNECT:
	case TETRA_RX_CMCE_D_RELEASE:
		call_id(c, &r);
		c->disconnect_cause = (uint8_t) rd(&r, 5);
		c->fields |= TETRA_RX_CMCE_F_DISC_CAUSE;
		if (obit(&r)) {
			notification(c, &r);
			skip_type34(&r);
		}
		break;
	case TETRA_RX_CMCE_D_INFO:
		call_id(c, &r);
		(void) rd(&r, 1);       /* reset call time-out timer (T310) */
		(void) rd(&r, 1);       /* poll request */
		if (obit(&r)) {
			if (pbit(&r))
				(void) rd(&r, 14);  /* new call identifier */
			if (pbit(&r)) {
				c->call_timeout = (uint8_t) rd(&r, 4);
				c->fields |= TETRA_RX_CMCE_F_CALL_TIMEOUT;
			}
			if (pbit(&r))
				(void) rd(&r, 3);   /* call time-out, set-up phase */
			if (pbit(&r)) {
				c->call_ownership = (uint8_t) rd(&r, 1);
				c->fields |= TETRA_RX_CMCE_F_CALL_OWNER;
			}
			if (pbit(&r))
				(void) rd(&r, 9);   /* modify */
			if (pbit(&r))
				(void) rd(&r, 3);   /* call status */
			temp_address(c, &r);
			notification(c, &r);
			if (pbit(&r))
				(void) rd(&r, 6);   /* poll response percentage */
			if (pbit(&r))
				(void) rd(&r, 6);   /* poll response number */
			skip_type34(&r);
		}
		break;
	case TETRA_RX_CMCE_D_SETUP:
		call_id(c, &r);
		c->call_timeout = (uint8_t) rd(&r, 4);
		c->hook_method = (uint8_t) rd(&r, 1);
		c->duplex = (uint8_t) rd(&r, 1);
		basic_service(c, &r);
		tx_grant(c, &r);
		c->call_priority = (uint8_t) rd(&r, 4);
		c->fields |= TETRA_RX_CMCE_F_CALL_TIMEOUT | TETRA_RX_CMCE_F_HOOK |
			     TETRA_RX_CMCE_F_SIMPLEX | TETRA_RX_CMCE_F_PRIORITY;
		if (obit(&r)) {
			notification(c, &r);
			temp_address(c, &r);
			if (pbit(&r))
				party(c, &r);
			skip_type34(&r);
		}
		break;
	case TETRA_RX_CMCE_D_STATUS:
		party(c, &r);
		c->status = (uint16_t) rd(&r, 16);
		c->fields |= TETRA_RX_CMCE_F_STATUS;
		if (obit(&r))
			skip_type34(&r);
		break;
	case TETRA_RX_CMCE_D_SDS_DATA: {
		unsigned int n;

		party(c, &r);
		c->sds_type = (uint8_t) rd(&r, 2);
		switch (c->sds_type) {
		case 0: n = 16; break;
		case 1: n = 32; break;
		case 2: n = 64; break;
		default: n = rd(&r, 11); break;
		}
		if (r.err || n > r.len - r.pos) {
			r.err = 1;
			break;
		}
		c->sds_bits = (uint16_t) n;
		c->sds_data = r.b + r.pos;
		c->fields |= TETRA_RX_CMCE_F_SDS;
		r.pos += n;
		if (obit(&r))
			skip_type34(&r);
		if (!r.err && c->sds_type == 3)
			sds_type4(c, c->sds_data, c->sds_bits);
		break;
	}
	case TETRA_RX_CMCE_D_TX_CEASED:
	case TETRA_RX_CMCE_D_TX_WAIT:
		call_id(c, &r);
		c->tx_req_permission = (uint8_t) rd(&r, 1);
		c->fields |= TETRA_RX_CMCE_F_TX_REQ_PERM;
		if (obit(&r)) {
			notification(c, &r);
			skip_type34(&r);
		}
		break;
	case TETRA_RX_CMCE_D_TX_CONTINUE:
		call_id(c, &r);
		(void) rd(&r, 1);       /* continue */
		c->tx_req_permission = (uint8_t) rd(&r, 1);
		c->fields |= TETRA_RX_CMCE_F_TX_REQ_PERM;
		if (obit(&r)) {
			notification(c, &r);
			skip_type34(&r);
		}
		break;
	case TETRA_RX_CMCE_D_TX_GRANTED:
	case TETRA_RX_CMCE_D_TX_INTERRUPT:
		call_id(c, &r);
		tx_grant(c, &r);
		c->enc_control = (uint8_t) rd(&r, 1);
		(void) rd(&r, 1);       /* reserved */
		c->fields |= TETRA_RX_CMCE_F_ENC_CONTROL;
		if (obit(&r)) {
			notification(c, &r);
			if (pbit(&r))
				party(c, &r);
			skip_type34(&r);
		}
		break;
	case TETRA_RX_CMCE_D_CALL_RESTORE:
		call_id(c, &r);
		tx_grant(c, &r);
		(void) rd(&r, 1);       /* reset call time-out timer (T310) */
		if (obit(&r)) {
			if (pbit(&r))
				(void) rd(&r, 14);  /* new call identifier */
			if (pbit(&r)) {
				c->call_timeout = (uint8_t) rd(&r, 4);
				c->fields |= TETRA_RX_CMCE_F_CALL_TIMEOUT;
			}
			if (pbit(&r))
				(void) rd(&r, 3);   /* call status */
			if (pbit(&r))
				(void) rd(&r, 9);   /* modify */
			notification(c, &r);
			skip_type34(&r);
		}
		break;
	default:
		/* D-FACILITY, U-plane related and reserved types: type + raw bits */
		known = 0;
		break;
	}

	/* A correctly encoded PDU ends exactly at the end of the SDU. */
	if (!known || r.err || r.pos != r.len) {
		memset(c, 0, sizeof(*c));
		return;
	}
	c->parse_ok = 1;
}
