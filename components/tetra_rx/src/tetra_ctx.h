/* tetra_rx - internal per-instance context
 *
 * Replaces the global state of osmo-tetra (t_phy_state, _tcd, fragslots,
 * g_llcs, msgb/talloc buffers) with one structure allocated in
 * tetra_rx_create(). Nothing in the processing path allocates.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#ifndef TETRA_CTX_H
#define TETRA_CTX_H

#include <stdint.h>
#include <stdbool.h>

#include "tetra_rx.h"

#include <tetra_common.h>
#include <tetra_mac_pdu.h>
#include <tetra_tdma.h>
#include <phy/tetra_burst.h>

/* ---- modem ------------------------------------------------------------ */
#define TM_MF_TAPS     41          /* RRC alpha 0.35, +-5 symbols, Kaiser beta 3 */
#define TM_RING        1024        /* differential symbol history (power of 2) */
#define TM_RING_MASK   (TM_RING - 1)
#define TM_SLOT_SYMS   255

struct tetra_modem {
	/* front end */
	float dc_i, dc_q;
	float nco_c, nco_s;         /* rotator state */
	float nco_dc, nco_ds;       /* rotator step */
	float freq_hz;              /* carrier offset being removed by the NCO */
	float freq_saved;           /* NCO frequency before a tentative lock */
	float freq_est;             /* smoothed total offset estimate */
	uint32_t nco_cnt;
	float taps[TM_MF_TAPS];
	float mf_i[2 * TM_MF_TAPS];
	float mf_q[2 * TM_MF_TAPS];
	uint32_t mf_pos;
	/* timing recovery */
	float yi[4], yq[4];         /* last matched filter outputs, [3] newest */
	float tc_re, tc_im;         /* square-law timing tone accumulator */
	uint32_t phase4;            /* index of the newest sample modulo 4 */
	float cnt;                  /* strobe position relative to yi[1] */
	float tau;                  /* current timing estimate (samples, 0..4) */
	float adj_acc;              /* accumulated strobe adjustments (for ppm) */
	float prev_i, prev_q;       /* previous symbol */
	/* differential symbol ring */
	float dr[TM_RING], di[TM_RING], dm[TM_RING];
	uint32_t nsym;              /* symbols written (index of next symbol) */
	/* burst synchronisation */
	uint8_t state;              /* TM_ST_* */
	uint8_t bad_slots;
	float freq_good;           /* carrier estimate when a block last passed its CRC */
	uint16_t locked_slots;      /* saturating count of slots since lock */
	uint8_t confirm;           /* good slots seen in the tentative state */
	uint16_t idle_slots;       /* slots since the last block with a good CRC-16 */
	uint32_t crc_seen;
	uint8_t settle;             /* do not decode the confirming slot */
	uint8_t fly_valid;          /* TDMA time can be restored after a re-lock */
	uint32_t slot_start;        /* symbol index of symbol 0 of the current slot */
	uint32_t unlocked_syms;
	uint32_t fly_start;         /* slot_start of the first slot not processed */
	/* reference symbols: (re, im) signs of the expected differential symbol */
	float ref_q[11][2], ref_n[11][2], ref_p[11][2], ref_y[19][2];
	uint8_t bits_q[22], bits_n[22], bits_p[22], bits_y[38];
	/* metrics */
	float level;                /* smoothed mean |d| (= power) */
	float snr_lin;              /* smoothed constellation SNR */
	float ber;                  /* smoothed training BER */
	float ppm;
	/* slot soft bits (positive = 0) */
	int8_t soft[TETRA_BITS_PER_TS];
};

enum { TM_ST_UNLOCKED = 0, TM_ST_TENTATIVE, TM_ST_LOCKED };

/* ---- MAC fragment reassembly (replaces struct fragslot + msgb) -------- */
#define TETRA_FRAG_BITS TETRA_RX_MAX_SDU_BITS
#define TETRA_N203      6

struct tetra_fraghdr {
	tetra_rx_addr_t addr;
	uint8_t encryption_mode;
	uint8_t lchan;
	uint8_t slot_grant;
	tetra_rx_chan_alloc_t chan_alloc;
};

struct tetra_fragslot {
	uint8_t active;             /* 0 idle, 1 collecting, 2 skipping an encrypted SDU */
	uint8_t age;
	uint16_t len;
	struct tetra_fraghdr hdr;
	uint8_t bits[TETRA_FRAG_BITS];
};

/* ---- receiver --------------------------------------------------------- */
struct tetra_rx {
	tetra_rx_config_t cfg;
	struct tetra_modem m;

	/* cell data (osmo-tetra: struct tetra_cell_data) */
	uint8_t cell_valid;
	uint16_t mcc, mnc;
	uint8_t colour_code;
	uint32_t scramb_init;
	uint8_t scr_cell[432];      /* scrambling sequence of the cell */
	uint8_t scr_bsch[120];      /* scrambling sequence of the BSCH */
	uint16_t rm_col[14];        /* RM(30,14) parity columns */

	struct tetra_tdma_time time;
	uint8_t time_valid;

	/* current burst (osmo-tetra: tms->cur_burst) */
	struct {
		uint8_t is_traffic;     /* DL usage marker if traffic, else 0 */
		uint8_t blk1_stolen;
		uint8_t blk2_stolen;
		uint8_t blk1_ok;        /* CRC of the stolen block 1 */
		uint8_t in_blk2;        /* currently parsing block 2 */
		uint8_t aach_ok;
	} cur;

	struct tetra_si_decoded last_sid;
	uint8_t sid_valid;

	/* encryption state learned from signalling */
	uint8_t um_enc[64];         /* per traffic usage marker */
	uint8_t ts_enc[4];          /* per timeslot */
	/* traffic usage of each timeslot, kept over lost ACCESS-ASSIGN blocks */
	struct {
		uint8_t usage;          /* last traffic usage marker, 0 = none */
		uint8_t miss;           /* frames since it was last confirmed */
	} tn_use[4];

	/* channel decoder work area */
	int8_t t4[432];
	int8_t t3[432];
	int8_t t3dp[4 * 288];
	uint8_t t2[512];            /* type-2 bits (type-1 + CRC + tail), zero padded */
	uint16_t vit[288];          /* Viterbi decisions */

	struct tetra_fragslot frag[4];

	tetra_rx_status_t st;
	tetra_rx_event_t ev;        /* event scratch (kept off the stack) */
};

/* ---- internal API ----------------------------------------------------- */

/* tetra_modem.c */
void tetra_modem_init(struct tetra_rx *rx);
void tetra_modem_reset(struct tetra_rx *rx);
void tetra_modem_cf32(struct tetra_rx *rx, const float *iq, size_t n);
void tetra_modem_cs16(struct tetra_rx *rx, const int16_t *iq, size_t n);

/* tetra_viterbi.c: 16-state soft decision decoders.
 * in: n*N soft values (positive = 0, negative = 1, 0 = punctured),
 * out: n decoded bits, dec: scratch of n words. The trellis starts and ends in state 0. */
void tetra_viterbi_cch(const int8_t *in, uint8_t *out, int n, uint16_t *dec);  /* rate 1/4, EN 300 392-2 8.2.3.1.1 */
void tetra_viterbi_tch(const int8_t *in, uint8_t *out, int n, uint16_t *dec);  /* rate 1/3, EN 300 395-2 */

/* RM(30,14) bounded distance decoder (up to 3 bit errors).
 * Returns the number of corrected bits, or -1 if uncorrectable. */
void tetra_rm3014_dec_init(uint16_t col[14]);
int tetra_rm3014_dec(const uint16_t col[14], uint32_t rx30, uint16_t *info);

/* tetra_lmac.c */
void tetra_lmac_init(struct tetra_rx *rx);
void tetra_lmac_slot(struct tetra_rx *rx, const int8_t *soft, enum tetra_train_seq type);
void tetra_lmac_set_cell(struct tetra_rx *rx, uint16_t mcc, uint16_t mnc, uint8_t cc);
/* advance by any number of slots (upstream tetra_tdma_time_add_tn() only handles 1) */
void tetra_rx_time_add(struct tetra_tdma_time *t, uint32_t slots);

/* tetra_umac.c */
void tetra_umac_reset(struct tetra_rx *rx);
void tetra_umac_sync_lost(struct tetra_rx *rx);
void tetra_umac_bsch(struct tetra_rx *rx, const uint8_t *bits);
void tetra_umac_aach(struct tetra_rx *rx, uint16_t info, int corrected);
void tetra_umac_aach_lost(struct tetra_rx *rx);
/* bits == NULL: the block failed its CRC */
void tetra_umac_block(struct tetra_rx *rx, uint8_t *bits, unsigned int n_bits, uint8_t chan);
void tetra_umac_slot_end(struct tetra_rx *rx, int demodulated);
uint8_t tetra_umac_traffic_enc(const struct tetra_rx *rx);

/* tetra_cmce.c */
void tetra_cmce_parse(tetra_rx_cmce_t *c, const uint8_t *bits, unsigned int len);

/* tetra_tch.c */
/* type4: descrambled soft bits (432 for a full slot, 216 = block 2 of a slot
 * whose block 1 was stolen) */
void tetra_tch_full_slot(struct tetra_rx *rx, const int8_t *type4);
void tetra_tch_half_slot(struct tetra_rx *rx, const int8_t *type4);
unsigned int tetra_tch_parity(const uint8_t *c2);
int tetra_tch_punct(unsigned int k, unsigned int g);
void tetra_acelp_type2_to_codec(const uint8_t *in, uint8_t *out);
void tetra_acelp_codec_to_acelp(const uint8_t *in, uint8_t *out);

/* tetra_rx.c */
void tetra_rx_emit(struct tetra_rx *rx, tetra_rx_event_type_t type);
tetra_rx_event_t *tetra_rx_ev_begin(struct tetra_rx *rx);

#endif /* TETRA_CTX_H */
