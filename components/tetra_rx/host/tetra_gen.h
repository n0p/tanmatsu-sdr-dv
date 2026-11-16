/* tetra_gen - synthetic TETRA TMO downlink generator for the tetra_rx tests
 *
 * Produces the 510 bits of consecutive continuous-downlink slots of one cell:
 * BSCH, BNCH (SYSINFO), AACH, SCH/F, SCH/HD, STCH and TCH/S with a frame 18
 * structure, MAC-RESOURCE / MAC-FRAG / MAC-END, LLC basic link, MLE, CMCE and
 * MM test PDUs. Everything the generator sends is logged so that a test can
 * compare it with what the receiver reports.
 *
 * The encoder path is deliberately separate from the decoder under test:
 * channel coding uses the vendored osmo-tetra encoder primitives (CRC,
 * RCPC encoder and puncturer, block interleaver, scrambler, RM(30,14), burst
 * builders); the PDUs are written by a bit writer in this file straight from
 * the tables of EN 300 392-2; the speech channel coding is written out from
 * the code polynomials. (The decoder uses its own Viterbi and RM decoders and
 * parsers.)
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#ifndef TETRA_GEN_H
#define TETRA_GEN_H

#include <stddef.h>
#include <stdint.h>

#include "tetra_rx.h"

#define TETRA_GEN_SLOT_BITS 510
#define TETRA_GEN_SDU_MAX   1200
#define TETRA_GEN_SSI_BASE  1000000u

typedef struct {
	uint16_t mcc, mnc;
	uint8_t colour_code;
	uint16_t main_carrier;
	uint8_t freq_band, freq_offset, duplex_spacing, reverse_operation;
	uint16_t location_area;
	uint16_t hyperframe;
	uint16_t bs_service_details;
	uint8_t start_tn, start_fn, start_mn;
	uint8_t traffic_tn;        /* 0: no traffic channel, 2..4: timeslot carrying TCH/S */
	uint8_t usage_marker;      /* traffic usage marker 4..63 */
	uint8_t traffic_encrypted; /* signalling for the call uses air interface encryption */
	uint8_t all_control;       /* TN2..4 are assigned control channels with messages
	                              (else they are unallocated: SB / null PDUs) */
	uint8_t no_messages;       /* broadcast and null PDUs only */
	uint64_t seed;
} tetra_gen_cfg_t;

enum tetra_gen_kind {
	TETRA_GEN_D_SETUP = 0,
	TETRA_GEN_D_CONNECT,
	TETRA_GEN_D_TX_GRANTED,
	TETRA_GEN_D_TX_CEASED,
	TETRA_GEN_D_RELEASE,
	TETRA_GEN_D_STATUS,
	TETRA_GEN_D_SDS_TEXT,      /* SDS type 4, text (SDS-TL or simple text) */
	TETRA_GEN_D_SDS_SHORT,     /* SDS type 1..3 user defined data */
	TETRA_GEN_MM_LOC_UPD_ACC,
	TETRA_GEN_MLE_NWRK_BCAST,
	TETRA_GEN_ENCRYPTED,       /* MAC-RESOURCE with encryption mode != 0 */
	TETRA_GEN_KIND_COUNT
};

typedef struct {
	uint32_t ssi;              /* MAC address, unique: TETRA_GEN_SSI_BASE + index */
	uint8_t kind;              /* enum tetra_gen_kind */
	uint8_t complete;          /* all MAC blocks have been transmitted */
	uint8_t lchan;             /* tetra_rx_chan_t of the first MAC block */
	uint8_t fragments;         /* MAC blocks used; 1 = not fragmented */
	uint8_t enc_mode;          /* MAC encryption mode; != 0: the TM-SDU is random bits */
	uint8_t usage_marker;      /* != 0: addressed with SSI + usage marker */
	uint8_t chan_alloc;        /* MAC-RESOURCE carries a channel allocation */
	uint16_t chan_alloc_carrier;
	uint8_t chan_alloc_ts;     /* timeslot bitmap */
	uint8_t slot_grant;
	uint8_t llc;               /* raw LLC PDU type: 0 BL-ADATA, 1 BL-DATA, 2 BL-UDATA,
	                              5 BL-DATA + FCS */
	uint8_t pdisc;             /* MLE protocol discriminator */
	uint8_t pdu_type;
	uint8_t have_cmce;         /* cmce holds what the receiver must report */
	tetra_rx_cmce_t cmce;      /* (sds_data is not used, see sds_off) */
	uint16_t sds_off;          /* offset of the SDS user data in sdu[] */
	uint16_t sdu_bits;         /* TL-SDU (from the protocol discriminator) */
	uint8_t sdu[TETRA_GEN_SDU_MAX];
	uint32_t first_slot, last_slot;  /* generator slot index of first / last MAC block */
} tetra_gen_msg_t;

enum tetra_gen_slot_kind {
	TETRA_GEN_SLOT_CONTROL = 0,
	TETRA_GEN_SLOT_UNALLOC,
	TETRA_GEN_SLOT_TCH,        /* two speech frames */
	TETRA_GEN_SLOT_TCH_STOLEN1,/* first half slot stolen */
	TETRA_GEN_SLOT_TCH_STOLEN2 /* both half slots stolen */
};

typedef struct {
	uint32_t index;            /* counts from 0 */
	uint8_t tn, fn, mn;
	uint8_t kind;              /* enum tetra_gen_slot_kind */
	uint8_t burst;             /* 0 NDB (one logical channel), 1 NDB (two), 2 SB */
	uint8_t has_sysinfo;
	uint16_t aach;             /* the 14 ACCESS-ASSIGN bits */
	uint8_t tch[2][137];       /* speech frames in codec order (TETRA_GEN_SLOT_TCH) */
	uint8_t bits[TETRA_GEN_SLOT_BITS];
} tetra_gen_slot_t;

typedef struct tetra_gen tetra_gen_t;

void tetra_gen_default_cfg(tetra_gen_cfg_t *cfg);
tetra_gen_t *tetra_gen_new(const tetra_gen_cfg_t *cfg);
void tetra_gen_free(tetra_gen_t *g);
/* the next slot; the pointer stays valid until the next call */
const tetra_gen_slot_t *tetra_gen_next(tetra_gen_t *g);
/* log of all messages created so far (index = ssi - TETRA_GEN_SSI_BASE);
 * the pointers are invalidated by tetra_gen_next() */
size_t tetra_gen_msg_count(const tetra_gen_t *g);
const tetra_gen_msg_t *tetra_gen_msg(const tetra_gen_t *g, size_t i);
const char *tetra_gen_kind_name(uint8_t kind);

/* ---- channel encoders (also used directly by the unit tests) ------------- */

/* signalling: kind 0 = BSCH (60 type-1 bits -> 120), 1 = half slot (124 ->
 * 216), 2 = full slot (268 -> 432); type5 = scrambled bits */
void tetra_gen_encode_cch(const uint8_t *type1, int kind, uint32_t scramb_init, uint8_t *type5);
/* ACCESS-ASSIGN: 14 bits -> 30 scrambled bits */
void tetra_gen_encode_aach(uint16_t info, uint32_t scramb_init, uint8_t *type5);
/* TCH/S: 274 type-1 bits [class 0: 102 | class 1: 112 | class 2: 60] ->
 * 432 type-4 bits (not scrambled) */
void tetra_gen_encode_tch(const uint8_t *type1, uint8_t *type4);
/* the 8 parity bits of the 60 class 2 bits, bit j = parity bit j */
unsigned int tetra_gen_tch_crc(const uint8_t *c2);
/* CRC-32 frame check sequence of the LLC over n bits */
uint32_t tetra_gen_fcs(const uint8_t *bits, unsigned int n);

#endif
