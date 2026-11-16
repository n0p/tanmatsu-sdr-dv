/* tetra_rx - TCH/S (speech traffic channel) channel decoding,
 * EN 300 395-2 clause 5.
 *
 * One full traffic slot carries two 137 bit ACELP frames:
 *
 *   type-4 (432)  --matrix de-interleaver 24 x 18-->  type-3 (432)
 *   type-3 = [ class 0: 102 unprotected | class 1: 168 | class 2: 162 ]
 *   class 1 (112 bits) and class 2 (60 bits + 8 parity + 4 tail) are encoded
 *   by ONE run of the 16-state rate 1/3 mother code, punctured to 8/12
 *   (class 1) and 8/18 (class 2)
 *   type-1 = [ class 0 | class 1 | class 2 ], the two frames bit-interleaved
 *
 * The standard text was not available while writing this. The structure
 * (interleaver, layout, code polynomials, puncturing, tail) was confirmed on
 * off-air clear speech: Viterbi decoding followed by re-encoding reproduces
 * all 330 coded bits of every slot. The parity equations below were solved
 * from two thirds of those slots and checked on the remaining third; they
 * turn out to be a 7 bit CRC with g(X) = 1 + X^3 + X^7 over the class 2 bits
 * plus one overall parity bit. See README for the test evidence.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <stdint.h>
#include <string.h>

#include "tetra_ctx.h"

#define TCH_N0   102 /* class 0 bits of two frames */
#define TCH_N1   112
#define TCH_N2   60
#define TCH_NPAR 8
#define TCH_NTAIL 4
#define TCH_NCONV (TCH_N1 + TCH_N2 + TCH_NPAR + TCH_NTAIL) /* 184 */

/* parity bit j = XOR of the class 2 bits i with bit i set in tch_par[j] */
static const uint64_t tch_par[TCH_NPAR] = {
	0x0b79cacc1b5d191ull,
	0x16f3959836ba322ull,
	0x2de72b306d74644ull,
	0x50b79cacc1b5d19ull,
	0xa16f3959836ba32ull,
	0x42de72b306d7464ull,
	0x85bce5660dae8c8ull,
	0xf928b9bbf6cb08full,
};

/* the 8 parity bits for 60 class 2 bits, bit j = parity j */
unsigned int tetra_tch_parity(const uint8_t *c2)
{
	uint64_t w = 0;
	unsigned int i, j, p = 0;

	for (i = 0; i < TCH_N2; i++)
		w |= (uint64_t) (c2[i] & 1) << i;
	for (j = 0; j < TCH_NPAR; j++) {
		uint64_t x = w & tch_par[j];

		x ^= x >> 32;
		x ^= x >> 16;
		x ^= x >> 8;
		x ^= x >> 4;
		x ^= x >> 2;
		x ^= x >> 1;
		p |= (unsigned int) (x & 1) << j;
	}
	return p;
}

/* 1 if mother code bit g (0..2) of input bit k is transmitted.
 * Class 1: rate 8/12, period 6, kept {1,2,4}; class 2: rate 8/18,
 * period 12, kept {1,2,3,4,5,7,8,10,11} (1-based within the period). */
int tetra_tch_punct(unsigned int k, unsigned int g)
{
	if (k < TCH_N1)
		return ((0x0bu >> ((k & 1) * 3 + g)) & 1) != 0;
	k -= TCH_N1;
	return ((0x6dfu >> ((k & 3) * 3 + g)) & 1) != 0;
}

static tetra_rx_ev_traffic_t *begin(struct tetra_rx *rx, const int8_t *type4, unsigned int n)
{
	tetra_rx_event_t *ev = tetra_rx_ev_begin(rx);
	tetra_rx_ev_traffic_t *t = &ev->u.traffic;

	t->usage_marker = rx->cur.is_traffic;
	t->enc_state = tetra_umac_traffic_enc(rx);
	t->soft = n ? type4 : NULL;
	t->n_soft = (uint16_t) n;
	return t;
}

void tetra_tch_full_slot(struct tetra_rx *rx, const int8_t *type4)
{
	tetra_rx_ev_traffic_t *t = begin(rx, type4, 432);
	const int8_t *c;
	uint8_t *t1 = rx->t2;
	unsigned int r, col, k, g, ok;

	/* matrix de-interleaver: type-3 bit (r, col) of a 24 x 18 matrix
	 * written row by row was transmitted column by column */
	for (r = 0; r < 24; r++)
		for (col = 0; col < 18; col++)
			rx->t3[r * 18 + col] = type4[col * 24 + r];

	for (k = 0; k < TCH_N0; k++)
		t1[k] = rx->t3[k] < 0;

	/* de-puncture, erasures are 0 */
	c = rx->t3 + TCH_N0;
	for (k = 0; k < TCH_NCONV; k++)
		for (g = 0; g < 3; g++)
			rx->t3dp[3 * k + g] = tetra_tch_punct(k, g) ? *c++ : 0;
	tetra_viterbi_tch(rx->t3dp, t1 + TCH_N0, TCH_NCONV, rx->vit);

	ok = 1;
	for (k = 0; k < TCH_NPAR; k++)
		if (t1[TCH_N0 + TCH_N1 + TCH_N2 + k] !=
		    ((tetra_tch_parity(t1 + TCH_N0 + TCH_N1) >> k) & 1))
			ok = 0;
	for (k = 0; k < TCH_NTAIL; k++)
		if (t1[TCH_N0 + TCH_N1 + TCH_N2 + TCH_NPAR + k])
			ok = 0;

	if (ok)
		rx->st.chan[TETRA_RX_CH_TCH_S].ok++;
	else
		rx->st.chan[TETRA_RX_CH_TCH_S].fail++;

	tetra_acelp_type2_to_codec(t1, &t->frame[0][0]);
	t->n_frames = 2;
	t->crc_ok = (uint8_t) ok;
	t->bfi[0] = t->bfi[1] = (uint8_t) !ok;
	tetra_rx_emit(rx, TETRA_RX_EV_TRAFFIC);
}

/* First half slot stolen. The single speech frame in the second half slot
 * uses a different (shorter) coding that could not be verified; it is
 * reported as a bad frame, the soft bits are passed on. */
void tetra_tch_half_slot(struct tetra_rx *rx, const int8_t *type4)
{
	tetra_rx_ev_traffic_t *t = begin(rx, type4, rx->cur.blk2_stolen ? 0 : 216);

	t->stolen[0] = 1;
	t->stolen[1] = rx->cur.blk2_stolen;
	t->bfi[0] = t->bfi[1] = 1;
	tetra_rx_emit(rx, TETRA_RX_EV_TRAFFIC);
}
