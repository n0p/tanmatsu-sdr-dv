/*
 * dmr_rx - thin wrappers around the vendored FEC code that add what the
 * upstream decoders do not report: validity and corrected-bit counts.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#ifndef DMR_RX_FEC_H
#define DMR_RX_FEC_H

#include <stdint.h>

#include "BPTC19696.h"

namespace dmrrx {

/* A burst is handled in the MMDVM layout: 264 bits, MSB first, 33 bytes. */
enum { kBurstBytes = 33, kBurstBits = 264 };

struct SlotType {
    bool ok;          /* Golay(20,8,7): at most 3 bit errors               */
    uint8_t cc;
    uint8_t data_type;
    uint8_t errs;     /* Hamming distance received <-> re-encoded codeword */
};
SlotType slot_type_decode(const uint8_t *burst);

struct Emb {
    bool ok;          /* QR(16,7,6): at most 2 bit errors                  */
    uint8_t cc;
    uint8_t pi;
    uint8_t lcss;
    uint8_t errs;
};
Emb emb_decode(const uint8_t *burst);

/* BPTC(196,96). errs is the Hamming distance between the 196 received info
 * field bits and the re-encoded decoder output; a large value means the
 * block was not a (correctable) BPTC codeword at all. */
struct Bptc {
    uint8_t data[12];
    uint8_t errs;
};
void bptc_decode(CBPTC19696 &bptc, const uint8_t *burst, Bptc &out);
/* Largest re-encode distance that is still accepted as "this is a BPTC
 * block" for PDUs that carry no CRC of their own. */
enum { kBptcPlausibleErrs = 12 };

/* TACT, Hamming(7,4). bits: AT, TC, LCSS1, LCSS0, P2, P1, P0. */
struct Tact {
    bool ok;          /* false if decoding was impossible (never: d=3)     */
    uint8_t errs;     /* 0 or 1                                            */
    uint8_t at;
    uint8_t tc;
    uint8_t lcss;
};
/* cach: 24 CACH bits, MSB first, in transmission order. payload17 receives
 * the 17 payload bits (one per byte). */
Tact cach_decode(const uint8_t cach[3], uint8_t payload17[17]);

/* AMBE+2 (3600x2450) frame: 72 bits in transmission order (MSB first). */
struct AmbeFrame {
    uint8_t ambe49[7]; /* 49 parameter bits, MSB first                     */
    uint8_t errs;      /* C0 Golay(24,12) + C1 Golay(23,12) bit errors     */
    bool bad;          /* C0 uncorrectable (4 errors detected)             */
    char ambe_d[49];   /* same bits, one per char (mbelib layout)          */
};
void ambe_frame_decode(const uint8_t frame72[9], AmbeFrame &out);

/* Bit helpers */
static inline unsigned get_bit(const uint8_t *p, unsigned i)
{
    return (p[i >> 3] >> (7u - (i & 7u))) & 1u;
}
static inline void put_bit(uint8_t *p, unsigned i, unsigned v)
{
    const uint8_t m = (uint8_t)(0x80u >> (i & 7u));
    if (v)
        p[i >> 3] |= m;
    else
        p[i >> 3] &= (uint8_t)~m;
}
static inline unsigned popcount8(unsigned v)
{
    v = (v & 0x55u) + ((v >> 1) & 0x55u);
    v = (v & 0x33u) + ((v >> 2) & 0x33u);
    return (v + (v >> 4)) & 0x0Fu;
}

} // namespace dmrrx

#endif
