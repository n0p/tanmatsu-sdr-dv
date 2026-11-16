/*
 * dmr_rx - packet data: rate 3/4 and rate 1 data blocks, block and packet
 * checksums, and the reassembly of the blocks behind a data header into one
 * packet (TS 102 361-1 clauses 8.2 and 9.2, annex B).
 *
 * The trellis coder is written from the specification (state transition
 * table B.7, constellation table B.8 / figure B.20, interleaver B.9); its
 * Viterbi decoder is this project's own.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include "dmr_data.h"

#include <string.h>

#include "dmr_internal.h"

namespace dmrrx {

/* ------------------------------------------------------------------------ */
/* Info bits of a burst: 98 before and 98 after the 68 bits in the middle    */
/* ------------------------------------------------------------------------ */

static inline unsigned info_pos(unsigned k)
{
    return k < 98u ? k : k + 68u;
}

/* ------------------------------------------------------------------------ */
/* Rate 3/4 trellis                                                          */
/* ------------------------------------------------------------------------ */

/* Constellation point sent for [state][tribit]; the tribit becomes the next
 * state. */
static const uint8_t kTrellisPoint[8][8] = {
    {0, 8, 4, 12, 2, 10, 6, 14}, {4, 12, 2, 10, 6, 14, 0, 8}, {1, 9, 5, 13, 3, 11, 7, 15}, {5, 13, 3, 11, 7, 15, 1, 9},
    {3, 11, 7, 15, 1, 9, 5, 13}, {7, 15, 1, 9, 5, 13, 3, 11}, {2, 10, 6, 14, 0, 8, 4, 12}, {6, 14, 0, 8, 4, 12, 2, 10},
};

/* The two 4FSK symbols of a constellation point, as dibits (bit 1, bit 0):
 * 01 = +3, 00 = +1, 10 = -1, 11 = -3. */
static const uint8_t kPointDibits[16][2] = {
    {0, 2}, {2, 2}, {1, 3}, {3, 3}, {3, 2}, {1, 2}, {2, 3}, {0, 3},
    {3, 1}, {1, 1}, {2, 0}, {0, 0}, {0, 1}, {2, 1}, {1, 0}, {3, 0},
};

/* Deviation level of a dibit in steps: +3, +1, -1, -3 -> 3, 2, 1, 0. */
static const uint8_t kDibitLevel[4] = {2, 3, 1, 0};

/* Transmitted dibit i is dibit kInterleave[i] of the encoder output. */
static const uint8_t kInterleave[98] = {
    0,  1,  8,  9,  16, 17, 24, 25, 32, 33, 40, 41, 48, 49, 56, 57, 64, 65, 72, 73, 80, 81, 88, 89, 96,
    97, 2,  3,  10, 11, 18, 19, 26, 27, 34, 35, 42, 43, 50, 51, 58, 59, 66, 67, 74, 75, 82, 83, 90, 91,
    4,  5,  12, 13, 20, 21, 28, 29, 36, 37, 44, 45, 52, 53, 60, 61, 68, 69, 76, 77, 84, 85, 92, 93, 6,
    7,  14, 15, 22, 23, 30, 31, 38, 39, 46, 47, 54, 55, 62, 63, 70, 71, 78, 79, 86, 87, 94, 95,
};

enum { kTrellisSteps = 49 };

static void trellis_dibits(const uint8_t data[18], uint8_t tx[98])
{
    uint8_t d[98];
    unsigned state = 0;
    for (unsigned k = 0; k < kTrellisSteps; k++) {
        unsigned tri = 0; /* the 49th tribit flushes the coder */
        if (k < 48u)
            tri = (get_bit(data, 3 * k) << 2) | (get_bit(data, 3 * k + 1) << 1) | get_bit(data, 3 * k + 2);
        const unsigned pt = kTrellisPoint[state][tri];
        d[2 * k]          = kPointDibits[pt][0];
        d[2 * k + 1]      = kPointDibits[pt][1];
        state             = tri;
    }
    for (unsigned i = 0; i < 98; i++)
        tx[i] = d[kInterleave[i]];
}

void trellis34_encode(const uint8_t data[18], uint8_t *burst)
{
    uint8_t tx[98];
    trellis_dibits(data, tx);
    for (unsigned i = 0; i < 98; i++) {
        put_bit(burst, info_pos(2 * i), (tx[i] >> 1) & 1u);
        put_bit(burst, info_pos(2 * i + 1), tx[i] & 1u);
    }
}

void trellis34_decode(const uint8_t *burst, Trellis &out)
{
    /* received symbol levels, deinterleaved */
    uint8_t rx[98], lev[98];
    for (unsigned i = 0; i < 98; i++) {
        rx[i]                = (uint8_t)((get_bit(burst, info_pos(2 * i)) << 1) | get_bit(burst, info_pos(2 * i + 1)));
        lev[kInterleave[i]] = kDibitLevel[rx[i]];
    }

    /* Viterbi over the 8 states. The bits are hard decisions, but what is
     * likely to have happened to a symbol is a slip to the neighbouring
     * level: the cost of a branch is the squared level distance. */
    enum { kInf = 0x3FFF };
    uint16_t metric[8];
    uint8_t from[kTrellisSteps][8];
    for (unsigned s = 0; s < 8; s++)
        metric[s] = kInf;
    metric[0] = 0;
    for (unsigned k = 0; k < kTrellisSteps; k++) {
        uint8_t cost[16];
        for (unsigned p = 0; p < 16; p++) {
            const int a = (int)lev[2 * k] - (int)kDibitLevel[kPointDibits[p][0]];
            const int b = (int)lev[2 * k + 1] - (int)kDibitLevel[kPointDibits[p][1]];
            cost[p]     = (uint8_t)(a * a + b * b);
        }
        uint16_t next[8];
        const unsigned n_to = k == kTrellisSteps - 1u ? 1u : 8u; /* ends in state 0 */
        for (unsigned to = 0; to < 8; to++) {
            next[to]    = kInf;
            from[k][to] = 0;
            if (to >= n_to)
                continue;
            for (unsigned s = 0; s < 8; s++) {
                if (metric[s] >= kInf)
                    continue;
                const uint16_t m = (uint16_t)(metric[s] + cost[kTrellisPoint[s][to]]);
                if (m < next[to]) {
                    next[to]    = m;
                    from[k][to] = (uint8_t)s;
                }
            }
        }
        memcpy(metric, next, sizeof(metric));
    }

    memset(out.data, 0, sizeof(out.data));
    unsigned state = 0;
    for (int k = kTrellisSteps - 1; k >= 0; k--) {
        if (k < 48) {
            put_bit(out.data, 3u * (unsigned)k, (state >> 2) & 1u);
            put_bit(out.data, 3u * (unsigned)k + 1u, (state >> 1) & 1u);
            put_bit(out.data, 3u * (unsigned)k + 2u, state & 1u);
        }
        state = from[k][state];
    }

    uint8_t re[98];
    trellis_dibits(out.data, re);
    unsigned errs = 0;
    for (unsigned i = 0; i < 98; i++) {
        const unsigned x = rx[i] ^ re[i];
        errs += (x & 1u) + (x >> 1);
    }
    out.errs = (uint8_t)errs;
}

/* ------------------------------------------------------------------------ */
/* Rate 1: 96 data bits, 4 unused bits, 96 data bits                         */
/* ------------------------------------------------------------------------ */

void rate1_decode(const uint8_t *burst, uint8_t data[24])
{
    memset(data, 0, 24);
    for (unsigned i = 0; i < 192; i++)
        put_bit(data, i, get_bit(burst, info_pos(i < 96u ? i : i + 4u)));
}

void rate1_encode(const uint8_t data[24], uint8_t *burst)
{
    for (unsigned i = 0; i < 4; i++)
        put_bit(burst, info_pos(96u + i), 0);
    for (unsigned i = 0; i < 192; i++)
        put_bit(burst, info_pos(i < 96u ? i : i + 4u), get_bit(data, i));
}

unsigned data_block_len(uint8_t t)
{
    switch (t) {
    case DMR_DT_RATE_12_DATA: return 12;
    case DMR_DT_RATE_34_DATA: return 18;
    case DMR_DT_RATE_1_DATA: return 24;
    default: return 0;
    }
}

/* ------------------------------------------------------------------------ */
/* Checksums                                                                 */
/* ------------------------------------------------------------------------ */

uint16_t data_crc9(uint8_t data_type, uint8_t dbsn, const uint8_t *payload, size_t n)
{
    /* x^9 + x^6 + x^4 + x^3 + 1 over the payload and then the serial number,
     * inverted, and masked with the pattern of the data type. */
    unsigned crc = 0;
    for (size_t i = 0; i < n * 8u + 7u; i++) {
        const unsigned bit = i < n * 8u ? get_bit(payload, (unsigned)i) : ((unsigned)dbsn >> (6u - (unsigned)(i - n * 8u))) & 1u;
        const unsigned fb  = ((crc >> 8) & 1u) ^ bit;
        crc                = (crc << 1) & 0x1FFu;
        if (fb)
            crc ^= 0x059u;
    }
    unsigned mask = 0x1FFu;
    if (data_type == DMR_DT_RATE_12_DATA)
        mask = 0x0F0u;
    else if (data_type == DMR_DT_RATE_1_DATA)
        mask = 0x10Fu;
    return (uint16_t)(crc ^ 0x1FFu ^ mask);
}

uint32_t data_crc32(const uint8_t *pdu, size_t n)
{
    /* The octets go into the divider in pairs, the second of a pair first;
     * the remainder is sent least significant octet first. */
    uint32_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        size_t j = i ^ 1u;
        if (j >= n)
            j = i;
        crc ^= (uint32_t)pdu[j] << 24;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : crc << 1;
    }
    return ((crc & 0xFFu) << 24) | ((crc & 0xFF00u) << 8) | ((crc >> 8) & 0xFF00u) | (crc >> 24);
}

bool data_crc32_ok(const uint8_t *pdu, size_t n)
{
    if (n < 5)
        return false;
    const uint8_t *c  = pdu + n - 4;
    const uint32_t rx = ((uint32_t)c[0] << 24) | ((uint32_t)c[1] << 16) | ((uint32_t)c[2] << 8) | c[3];
    return data_crc32(pdu, n - 4) == rx;
}

/* ------------------------------------------------------------------------ */
/* Packet reassembly                                                         */
/* ------------------------------------------------------------------------ */

/* A data header (CRC verified) on this slot. */
void Rx::data_header(Slot &s, const uint8_t *d)
{
    DataAsm &a        = dasm[&s - slot];
    const uint8_t dpf = d[0] & 0x0Fu;

    if (dpf == 15) {
        /* Proprietary header: the second (third ...) header of a packet. It
         * is one of the blocks the first header announced. */
        if (!a.active || a.n != 0) {
            data_flush(s);
            return;
        }
        if (!a.props)
            memcpy(a.prop, d, 10);
        a.props++;
        if (--a.left == 0)
            data_flush(s);
        return;
    }

    data_flush(s);
    unsigned blocks;
    switch (dpf) {
    case 2:  /* unconfirmed data */
    case 3:  /* confirmed data */
        blocks = d[8] & 0x7Fu;
        break;
    case 13: /* short data: defined */
    case 14: /* short data: raw, or status / precoded */
        blocks = (((unsigned)d[0] >> 4) & 3u) << 4 | (d[1] & 0x0Fu);
        break;
    default: /* unified data transport, response packets: not handled */
        return;
    }
    memset(&a, 0, offsetof(DataAsm, blk));
    memcpy(a.hdr, d, 10);
    a.type      = 0xFF;
    a.announced = (uint8_t)blocks;
    a.left      = (uint8_t)blocks;
    a.active    = true;
    if (blocks == 0) {
        if (dpf == 14)
            data_finish(s); /* status / precoded: the header is all there is */
        a.active = false;
    }
}

/* A data header burst that failed its CRC. */
void Rx::data_header_bad(Slot &s)
{
    DataAsm &a = dasm[&s - slot];
    if (!a.active)
        return;
    if (a.n == 0 && a.left > 1) {
        /* most likely a proprietary header: one of the announced blocks */
        a.props++;
        a.prop_lost = true;
        a.left--;
    } else {
        data_flush(s);
    }
}

/* A burst at the place of a data block. t: its data type, 0xFF if the slot
 * type was not readable. w: the burst. */
void Rx::data_block(Slot &s, const uint8_t *w, uint8_t t, const DataEval &de)
{
    DataAsm &a = dasm[&s - slot];
    if (!a.active)
        return;
    if (t == 0xFF)
        t = a.type; /* the blocks of a packet are all of one kind */
    bool ok = t != 0xFF && (a.type == 0xFF || a.type == t);
    uint8_t blk[kDataBlockMax];
    memset(blk, 0, sizeof(blk));
    if (ok) {
        if (t == DMR_DT_RATE_12_DATA) {
            Bptc bp;
            if (de.st.ok && de.have_bptc)
                bp = de.bp;
            else
                bptc_decode(bptc, w, bp);
            memcpy(blk, bp.data, 12);
            ok = bp.errs <= kBptcPlausibleErrs;
        } else if (t == DMR_DT_RATE_34_DATA) {
            Trellis tr;
            trellis34_decode(w, tr);
            memcpy(blk, tr.data, 18);
            ok = tr.errs <= kTrellisPlausibleErrs;
        } else {
            rate1_decode(w, blk);
        }
    }
    if (ok) {
        a.type = t;
        if (a.n < kDataMaxBlocks) {
            memcpy(a.blk[a.n], blk, kDataBlockMax);
            a.have |= (uint64_t)1 << a.n;
        }
    } else {
        a.missing++;
    }
    a.n++;
    if (--a.left == 0)
        data_flush(s);
}

/* Nothing was received where a burst of this slot was due. */
void Rx::data_missing(Slot &s)
{
    DataAsm &a = dasm[&s - slot];
    if (!a.active)
        return;
    a.missing++;
    a.n++;
    if (--a.left == 0)
        data_flush(s);
}

/* The packet is over, complete or not. */
void Rx::data_flush(Slot &s)
{
    DataAsm &a = dasm[&s - slot];
    if (!a.active)
        return;
    a.active = false;
    if (a.n > a.missing)
        data_finish(s);
}

void Rx::data_finish(Slot &s)
{
    DataAsm &a = dasm[&s - slot];
    dmr_event_t ev;
    memset(&ev, 0, sizeof(ev));
    dmr_evt_data_pdu_t &p = ev.u.data_pdu;
    const uint8_t *h      = a.hdr;
    p.is_group            = h[0] >> 7;
    p.response_requested  = (h[0] >> 6) & 1u;
    p.dpf                 = h[0] & 0x0Fu;
    p.sap                 = h[1] >> 4;
    p.dst_id              = ((uint32_t)h[2] << 16) | ((uint32_t)h[3] << 8) | h[4];
    p.src_id              = ((uint32_t)h[5] << 16) | ((uint32_t)h[6] << 8) | h[7];
    p.blocks              = a.announced;
    p.blocks_missing      = (uint8_t)(a.missing + a.left);
    p.block_type          = a.type;
    memcpy(p.header, h, 10);
    if (a.props) {
        p.prop_headers = a.props;
        if (!a.prop_lost) {
            p.mfid = a.prop[1];
            p.sap  = a.prop[0] >> 4;
            memcpy(p.prop_header, a.prop, 10);
        }
    }

    const unsigned bl     = data_block_len(a.type);
    const unsigned stored = a.n < kDataMaxBlocks ? a.n : (unsigned)kDataMaxBlocks;
    const bool all        = a.left == 0 && a.n == stored;
    p.complete            = all && a.missing == 0 && !a.prop_lost;

    /* Confirmed delivery: every block begins with its serial number and a
     * CRC-9. The header says which it is (response requested); the CRC-32
     * over the packet settles it, and failing that the CRC-9s. */
    unsigned crc9_ok = 0, received = 0;
    for (unsigned i = 0; i < stored && bl; i++) {
        if (!((a.have >> i) & 1u))
            continue;
        received++;
        const uint8_t *b   = a.blk[i];
        const uint16_t crc = (uint16_t)(((b[0] & 1u) << 8) | b[1]);
        if (data_crc9(a.type, b[0] >> 1, b + 2, bl - 2) == crc)
            crc9_ok++;
    }
    const bool hint = p.dpf == 3 || (p.dpf != 2 && p.response_requested);
    bool confirmed  = hint;
    size_t len      = 0;
    for (int attempt = 0; attempt < 3 && bl; attempt++) {
        if (attempt == 1)
            confirmed = !hint;
        else if (attempt == 2)
            confirmed = received != 0 && crc9_ok * 2 >= received;
        const unsigned skip = confirmed ? 2 : 0;
        len                 = 0;
        for (unsigned i = 0; i < stored; i++) {
            if ((a.have >> i) & 1u)
                memcpy(pdu_buf + len, a.blk[i] + skip, bl - skip);
            else
                memset(pdu_buf + len, 0, bl - skip);
            len += bl - skip;
        }
        if (attempt == 2)
            break;
        if (p.complete && data_crc32_ok(pdu_buf, len)) {
            p.crc_ok = 1;
            break;
        }
    }
    p.confirmed = confirmed && bl;

    /* the user data: without the CRC-32 and the pad octets before it */
    if (all && len >= 4) {
        len -= 4;
        unsigned pad = 0;
        if (p.dpf == 2 || p.dpf == 3)
            pad = (h[0] & 0x10u) | (h[1] & 0x0Fu);
        else if (p.dpf == 13 || p.dpf == 14)
            pad = h[9] / 8u; /* given in bits */
        if (pad <= len)
            len -= pad;
    }
    p.data = pdu_buf;
    p.len  = (uint16_t)len;
    pdu_interpret(p, pdu_text, sizeof(pdu_text));
    emit(s, ev, DMR_EVT_DATA_PDU);
}

} // namespace dmrrx
