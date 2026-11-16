/* tetra_rx - lean 16-state soft-decision Viterbi decoders and RM(30,14)
 * decoder.
 *
 * osmo-tetra decodes through libosmocore's generic osmo_conv_decode(), which
 * allocates its trellis on the heap for every block. This is a fixed-size
 * replacement: integer metrics, 16 path metrics on the stack, one 16-bit
 * decision word per trellis step in a caller supplied buffer.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <stdint.h>
#include <string.h>

#include "tetra_ctx.h"

/* Encoder state s: bit 0 = most recent input ... bit 3 = oldest (D^4).
 * Next state = ((s << 1) | in) & 15. */

/* EN 300 392-2 8.2.3.1.1: G1 = 1+D+D4, G2 = 1+D2+D3+D4, G3 = 1+D+D2+D4,
 * G4 = 1+D+D3+D4. Output symbol: bit 0 = g1 (sent first) ... bit 3 = g4. */
static unsigned int cch_out(unsigned int s, unsigned int b)
{
	unsigned int d1 = s & 1, d2 = (s >> 1) & 1, d3 = (s >> 2) & 1, d4 = (s >> 3) & 1;
	unsigned int g1 = b ^ d1 ^ d4;
	unsigned int g2 = b ^ d2 ^ d3 ^ d4;
	unsigned int g3 = b ^ d1 ^ d2 ^ d4;
	unsigned int g4 = b ^ d1 ^ d3 ^ d4;
	return g1 | (g2 << 1) | (g3 << 2) | (g4 << 3);
}

/* EN 300 395-2 (speech): G1 = 1+D+D2+D3+D4, G2 = 1+D+D3+D4, G3 = 1+D2+D4.
 * Output symbol: bit 0 = g1 (sent first) ... bit 2 = g3. */
static unsigned int tch_out(unsigned int s, unsigned int b)
{
	unsigned int d1 = s & 1, d2 = (s >> 1) & 1, d3 = (s >> 2) & 1, d4 = (s >> 3) & 1;
	unsigned int g1 = b ^ d1 ^ d2 ^ d3 ^ d4;
	unsigned int g2 = b ^ d1 ^ d3 ^ d4;
	unsigned int g3 = b ^ d2 ^ d4;
	return g1 | (g2 << 1) | (g3 << 2);
}

#define VIT_NEG (-(1 << 28))

static void viterbi(const int8_t *in, uint8_t *out, int n, uint16_t *dec, int N,
		    unsigned int (*outf)(unsigned int, unsigned int))
{
	int32_t pm[16], nm[16], bm[16];
	uint8_t otab[16][2];
	unsigned int s, state;
	int t, k;

	for (s = 0; s < 16; s++) {
		otab[s][0] = (uint8_t) outf(s, 0);
		otab[s][1] = (uint8_t) outf(s, 1);
	}
	pm[0] = 0;
	for (s = 1; s < 16; s++)
		pm[s] = VIT_NEG;

	for (t = 0; t < n; t++) {
		const int8_t *r = in + t * N;
		unsigned int nsym = 1u << N;
		unsigned int d = 0;

		/* branch metric = correlation with the hypothesised symbol;
		 * bm[sym ^ all-ones] = -bm[sym] */
		for (s = 0; s < nsym / 2; s++) {
			int32_t m = 0;
			for (k = 0; k < N; k++)
				m += ((s >> k) & 1) ? -r[k] : r[k];
			bm[s] = m;
			bm[(nsym - 1) ^ s] = -m;
		}
		for (s = 0; s < 16; s++) {
			unsigned int p0 = s >> 1, p1 = p0 | 8, b = s & 1;
			int32_t m0 = pm[p0] + bm[otab[p0][b]];
			int32_t m1 = pm[p1] + bm[otab[p1][b]];
			if (m1 > m0) {
				nm[s] = m1;
				d |= 1u << s;
			} else
				nm[s] = m0;
		}
		dec[t] = (uint16_t) d;
		memcpy(pm, nm, sizeof(pm));
	}

	/* the tail bits force the encoder back to state 0 */
	state = 0;
	for (t = n - 1; t >= 0; t--) {
		out[t] = state & 1;
		state = (state >> 1) | (((dec[t] >> state) & 1u) << 3);
	}
}

void tetra_viterbi_cch(const int8_t *in, uint8_t *out, int n, uint16_t *dec)
{
	viterbi(in, out, n, dec, 4, cch_out);
}

void tetra_viterbi_tch(const int8_t *in, uint8_t *out, int n, uint16_t *dec)
{
	viterbi(in, out, n, dec, 3, tch_out);
}

/* ---- RM(30,14), EN 300 392-2 8.2.3.2 ----------------------------------
 * Systematic code [14 info | 16 parity], minimum distance 8 (checked by the
 * unit test), so up to 3 bit errors are corrected by syndrome search. The
 * vendored tetra_rm3014_decode() only strips the parity. */
#include <lower_mac/tetra_rm3014.h>

void tetra_rm3014_dec_init(uint16_t col[14])
{
	int i;

	tetra_rm3014_init();
	for (i = 0; i < 14; i++)
		col[i] = (uint16_t) (tetra_rm3014_compute((uint16_t) (1u << (13 - i))) & 0xffff);
}

/* syndrome contribution of codeword bit position k (0 = first info bit) */
static inline uint16_t rm_col(const uint16_t col[14], int k)
{
	return k < 14 ? col[k] : (uint16_t) (1u << (29 - k));
}

int tetra_rm3014_dec(const uint16_t col[14], uint32_t rx30, uint16_t *info)
{
	uint16_t inf = (uint16_t) ((rx30 >> 16) & 0x3fff);
	uint16_t syn = (uint16_t) ((tetra_rm3014_compute(inf) ^ rx30) & 0xffff);
	int i, j, k;

	*info = inf;
	if (syn == 0)
		return 0;
	for (i = 0; i < 30; i++) {
		uint16_t s1 = syn ^ rm_col(col, i);
		if (s1 == 0) {
			if (i < 14)
				*info = inf ^ (uint16_t) (1u << (13 - i));
			return 1;
		}
	}
	for (i = 0; i < 30; i++) {
		uint16_t s1 = syn ^ rm_col(col, i);
		for (j = i + 1; j < 30; j++) {
			uint16_t s2 = s1 ^ rm_col(col, j);
			if (s2 == 0) {
				if (i < 14)
					inf ^= (uint16_t) (1u << (13 - i));
				if (j < 14)
					inf ^= (uint16_t) (1u << (13 - j));
				*info = inf;
				return 2;
			}
		}
	}
	for (i = 0; i < 30; i++) {
		uint16_t s1 = syn ^ rm_col(col, i);
		for (j = i + 1; j < 30; j++) {
			uint16_t s2 = s1 ^ rm_col(col, j);
			/* a single-bit s2 in the parity part or a matching info column */
			for (k = j + 1; k < 30; k++) {
				if (s2 == rm_col(col, k)) {
					if (i < 14)
						inf ^= (uint16_t) (1u << (13 - i));
					if (j < 14)
						inf ^= (uint16_t) (1u << (13 - j));
					if (k < 14)
						inf ^= (uint16_t) (1u << (13 - k));
					*info = inf;
					return 3;
				}
			}
		}
	}
	return -1;
}
