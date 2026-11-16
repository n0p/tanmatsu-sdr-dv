/*
 * dmr_rx - FEC wrappers. See dmr_fec.h.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include "dmr_fec.h"

#include <string.h>

#include "DMREMB.h"
#include "DMRSlotType.h"

extern "C" {
#include "mbelib.h"
}

/* DMR AMBE interleave schedule, from DSD (ISC licence). */
#define _MAIN
namespace {
#include "dmr_const.h"
}
#undef _MAIN

namespace dmrrx {

/* ---------------------------------------------------------------------- */

SlotType slot_type_decode(const uint8_t *burst)
{
    SlotType r;
    CDMRSlotType st;
    st.putData(burst);
    r.cc        = st.getColorCode();
    r.data_type = st.getDataType();

    /* Re-encode and compare the 20 slot type bits (burst bits 98..107 and
     * 156..165). */
    uint8_t re[kBurstBytes];
    memset(re, 0, sizeof(re));
    st.getData(re);
    unsigned d = 0;
    d += popcount8((burst[12] ^ re[12]) & 0x3Fu);
    d += popcount8((burst[13] ^ re[13]) & 0xF0u);
    d += popcount8((burst[19] ^ re[19]) & 0x0Fu);
    d += popcount8((burst[20] ^ re[20]) & 0xFCu);
    r.errs = (uint8_t)d;
    r.ok   = d <= 3;
    return r;
}

Emb emb_decode(const uint8_t *burst)
{
    Emb r;
    CDMREMB emb;
    emb.putData(burst);
    r.cc   = emb.getColorCode();
    r.pi   = emb.getPI() ? 1 : 0;
    r.lcss = emb.getLCSS();

    uint8_t re[kBurstBytes];
    memset(re, 0, sizeof(re));
    emb.getData(re);
    unsigned d = 0;
    d += popcount8((burst[13] ^ re[13]) & 0x0Fu);
    d += popcount8((burst[14] ^ re[14]) & 0xF0u);
    d += popcount8((burst[18] ^ re[18]) & 0x0Fu);
    d += popcount8((burst[19] ^ re[19]) & 0xF0u);
    r.errs = (uint8_t)d;
    r.ok   = d <= 2;
    return r;
}

void bptc_decode(CBPTC19696 &bptc, const uint8_t *burst, Bptc &out)
{
    bptc.decode(burst, out.data);

    uint8_t re[kBurstBytes];
    memset(re, 0, sizeof(re));
    bptc.encode(out.data, re);

    unsigned d = 0;
    for (unsigned i = 0; i < 12; i++)
        d += popcount8(burst[i] ^ re[i]);
    d += popcount8((burst[12] ^ re[12]) & 0xC0u);
    d += popcount8((burst[20] ^ re[20]) & 0x03u); /* bits 166, 167 */
    for (unsigned i = 21; i < 33; i++)
        d += popcount8(burst[i] ^ re[i]);
    out.errs = (uint8_t)(d > 255 ? 255 : d);
}

/* ---------------------------------------------------------------------- */
/* CACH: TS 102 361-1 9.1.4 (interleaving, table 9.4) and B.4.1
 * Hamming(7,4) for the TACT. Written from the specification; the bit
 * positions agree with the CACH encoder of the MMDVM firmware. */

/* cach bit i (transmission order) -> position in [TACT(7) | payload(17)] */
static const uint8_t kCachDeinterleave[24] = {0, 7, 8,  9,  1,  10, 11, 12, 2, 13, 14, 15,
                                              3, 16, 4, 17, 18, 19, 5,  20, 21, 22, 6, 23};

Tact cach_decode(const uint8_t cach[3], uint8_t payload17[17])
{
    uint8_t d[24];
    for (unsigned i = 0; i < 24; i++)
        d[kCachDeinterleave[i]] = (uint8_t)get_bit(cach, i);

    /* Parity check matrix H (B.4.1):
     *   1 1 1 0 1 0 0
     *   0 1 1 1 0 1 0
     *   1 1 0 1 0 0 1 */
    const unsigned s2 = d[0] ^ d[1] ^ d[2] ^ d[4];
    const unsigned s1 = d[1] ^ d[2] ^ d[3] ^ d[5];
    const unsigned s0 = d[0] ^ d[1] ^ d[3] ^ d[6];
    const unsigned syn = (s2 << 2) | (s1 << 1) | s0;
    /* syndrome -> erroneous bit position (column of H) */
    static const int8_t kPos[8] = {-1, 6, 5, 3, 4, 0, 2, 1};

    Tact t;
    t.ok   = true;
    t.errs = 0;
    if (syn != 0) {
        d[kPos[syn]] ^= 1u;
        t.errs = 1;
    }
    t.at   = d[0];
    t.tc   = d[1];
    t.lcss = (uint8_t)((d[2] << 1) | d[3]);
    for (unsigned i = 0; i < 17; i++)
        payload17[i] = d[7 + i];
    return t;
}

/* ---------------------------------------------------------------------- */
/* AMBE+2 frame FEC: deinterleave (DSD schedule), then mbelib's Golay
 * decoders. mbelib only reports errors in the data half of the codeword, so
 * the full codeword distance is recomputed here from the corrected data
 * (Golay(23,12) generator polynomial 0xC75, the one mbelib's tables use). */

static unsigned golay23_parity(unsigned data12)
{
    unsigned v = data12 << 11;
    for (int i = 22; i >= 11; i--) {
        if (v & (1u << i))
            v ^= 0xC75u << (i - 11);
    }
    return v & 0x7FFu;
}

static unsigned popcount32(uint32_t v)
{
    unsigned n = 0;
    while (v) {
        v &= v - 1;
        n++;
    }
    return n;
}

/* word: char per bit, index 22 = MSB of the data, index 0 = LSB of parity.
 * Returns the distance between the received word and the codeword that the
 * (already corrected) data bits imply. */
static unsigned golay23_distance(const char *rx, const char *corrected)
{
    unsigned rxw = 0, data = 0;
    for (int i = 22; i >= 0; i--)
        rxw = (rxw << 1) | (unsigned)(rx[i] & 1);
    for (int i = 22; i >= 11; i--)
        data = (data << 1) | (unsigned)(corrected[i] & 1);
    const unsigned cw = (data << 11) | golay23_parity(data);
    return popcount32(rxw ^ cw);
}

void ambe_frame_decode(const uint8_t frame72[9], AmbeFrame &out)
{
    char fr[4][24];
    memset(fr, 0, sizeof(fr));
    for (unsigned i = 0; i < 36; i++) {
        fr[rW[i]][rX[i]] = (char)get_bit(frame72, 2 * i);
        fr[rY[i]][rZ[i]] = (char)get_bit(frame72, 2 * i + 1);
    }

    /* C0: Golay(24,12) = Golay(23,12) in fr[0][1..23] + overall parity in
     * fr[0][0]. */
    char rx0[23];
    for (int j = 0; j < 23; j++)
        rx0[j] = fr[0][j + 1];
    unsigned par = (unsigned)(fr[0][0] & 1);
    mbe_eccAmbe3600x2450C0(fr);
    unsigned e0 = golay23_distance(rx0, &fr[0][1]);
    /* overall (even) parity over the corrected 23 bit codeword */
    {
        unsigned data = 0;
        for (int i = 23; i >= 12; i--)
            data = (data << 1) | (unsigned)(fr[0][i] & 1);
        const unsigned cw = (data << 11) | golay23_parity(data);
        if ((popcount32(cw) & 1u) != par)
            e0++;
    }

    mbe_demodulateAmbe3600x2450Data(fr);

    char rx1[23];
    for (int j = 0; j < 23; j++)
        rx1[j] = fr[1][j];
    mbe_eccAmbe3600x2450Data(fr, out.ambe_d);
    /* corrected C1 data bits are ambe_d[12..23] (MSB first) */
    char c1[23];
    memset(c1, 0, sizeof(c1));
    for (int j = 0; j < 12; j++)
        c1[22 - j] = out.ambe_d[12 + j];
    const unsigned e1 = golay23_distance(rx1, c1);

    out.errs = (uint8_t)(e0 + e1);
    out.bad  = e0 >= 4;
    memset(out.ambe49, 0, sizeof(out.ambe49));
    for (unsigned i = 0; i < 49; i++)
        put_bit(out.ambe49, i, (unsigned)(out.ambe_d[i] & 1));
}

} // namespace dmrrx
