/*
 * dmr_rx host tools - synthetic DMR signal generator. See dmr_gen_lib.h.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include "dmr_gen_lib.h"

#include <math.h>
#include <string.h>

#include "BPTC19696.h"
#include "CRC.h"
#include "DMRDefines.h"
#include "DMREMB.h"
#include "DMREmbeddedData.h"
#include "DMRFullLC.h"
#include "DMRLC.h"
#include "DMRShortLC.h"
#include "DMRSlotType.h"

#include "dmr_data.h"
#include "test_vectors.h"

namespace dmrgen {

/* ---------------------------------------------------------------------- */
/* Burst construction                                                     */
/* ---------------------------------------------------------------------- */

static const unsigned char *sync_pattern(Mode m, bool data)
{
    switch (m) {
    case MODE_BS: return data ? BS_SOURCED_DATA_SYNC : BS_SOURCED_AUDIO_SYNC;
    case MODE_MS: return data ? MS_SOURCED_DATA_SYNC : MS_SOURCED_AUDIO_SYNC;
    case MODE_DM1: return data ? DIRECT_SLOT1_DATA_SYNC : DIRECT_SLOT1_AUDIO_SYNC;
    default: return data ? DIRECT_SLOT2_DATA_SYNC : DIRECT_SLOT2_AUDIO_SYNC;
    }
}

static void put_sync(uint8_t *b, Mode m, bool data)
{
    const unsigned char *s = sync_pattern(m, data);
    for (unsigned i = 0; i < 7; i++)
        b[13 + i] = (uint8_t)((b[13 + i] & ~SYNC_MASK[i]) | s[i]);
}

void SlotScript::finish_data(BurstSpec &b, uint8_t data_type)
{
    CDMRSlotType st;
    st.setColorCode(m_cc);
    st.setDataType(data_type);
    st.getData(b.bits);
    put_sync(b.bits, m_mode, true);
    b.tx        = true;
    b.data_type = data_type;
}

void SlotScript::silence(int n)
{
    for (int i = 0; i < n; i++)
        bursts.push_back(BurstSpec());
}

void SlotScript::idle(int n)
{
    for (int i = 0; i < n; i++) {
        BurstSpec b;
        memcpy(b.bits, kIdleBurst, 33);
        finish_data(b, DT_IDLE);
        bursts.push_back(b);
    }
}

void SlotScript::data_raw(uint8_t data_type, const uint8_t pdu12[12], bool busy)
{
    BurstSpec b;
    CBPTC19696 bptc;
    bptc.encode(pdu12, b.bits);
    finish_data(b, data_type);
    b.busy = busy;
    bursts.push_back(b);
}

static void add_crc(uint8_t pdu[12], const unsigned char mask[2])
{
    /* as MMDVMHost's CDMRCSBK / CDMRDataHeader do it */
    pdu[10] = 0;
    pdu[11] = 0;
    pdu[10] ^= mask[0];
    pdu[11] ^= mask[1];
    CCRC::addCCITT162(pdu, 12);
    pdu[10] ^= mask[0];
    pdu[11] ^= mask[1];
}

void SlotScript::csbk(const uint8_t pdu10[10], int n)
{
    uint8_t p[12];
    memcpy(p, pdu10, 10);
    add_crc(p, CSBK_CRC_MASK);
    for (int i = 0; i < n; i++)
        data_raw(DT_CSBK, p);
}

void SlotScript::data_header(const uint8_t pdu10[10])
{
    uint8_t p[12];
    memcpy(p, pdu10, 10);
    add_crc(p, DATA_HEADER_CRC_MASK);
    data_raw(DT_DATA_HEADER, p);
}

void SlotScript::pi_header(const uint8_t pdu10[10])
{
    uint8_t p[12];
    memcpy(p, pdu10, 10);
    add_crc(p, PI_HEADER_CRC_MASK);
    data_raw(DT_VOICE_PI_HEADER, p);
}

void SlotScript::data_block(uint8_t data_type, const uint8_t *blk)
{
    if (data_type == DT_RATE_12_DATA) {
        data_raw(data_type, blk);
        return;
    }
    BurstSpec b;
    if (data_type == DT_RATE_34_DATA)
        dmrrx::trellis34_encode(blk, b.bits);
    else
        dmrrx::rate1_encode(blk, b.bits);
    finish_data(b, data_type);
    b.busy = true;
    bursts.push_back(b);
}

void SlotScript::burst_raw(const uint8_t bits33[33], int data_type)
{
    BurstSpec b;
    memcpy(b.bits, bits33, 33);
    b.tx        = true;
    b.busy      = true;
    b.data_type = data_type;
    bursts.push_back(b);
}

int SlotScript::packet(const PacketSpec &p)
{
    const size_t before = bursts.size();
    const unsigned bl   = dmrrx::data_block_len(p.data_type);
    const unsigned per  = bl - (p.confirmed ? 2u : 0u);
    const unsigned n_blk = (unsigned)((p.data.size() + 4 + per - 1) / per);
    const unsigned pad   = (unsigned)(n_blk * per - p.data.size() - 4);
    const unsigned total = n_blk + (p.prop_mfid >= 0 ? 1u : 0u);
    const uint8_t dpf    = p.dpf ? p.dpf : (uint8_t)(p.confirmed ? 3 : 2);
    const bool a         = p.response < 0 ? p.confirmed : p.response != 0;

    uint8_t h[10];
    h[0] = (uint8_t)((p.group ? 0x80 : 0) | (a ? 0x40 : 0) | dpf);
    h[1] = (uint8_t)(p.sap << 4);
    h[2] = (uint8_t)(p.dst >> 16);
    h[3] = (uint8_t)(p.dst >> 8);
    h[4] = (uint8_t)p.dst;
    h[5] = (uint8_t)(p.src >> 16);
    h[6] = (uint8_t)(p.src >> 8);
    h[7] = (uint8_t)p.src;
    if (dpf == 13 || dpf == 14) {
        /* short data: appended blocks, (DD format,) S, F, bit padding */
        h[0] |= (uint8_t)(((total >> 4) & 3u) << 4);
        h[1] |= (uint8_t)(total & 0x0Fu);
        h[8] = dpf == 13 ? (uint8_t)((p.dd_format << 2) | 1u) : (uint8_t)0x01;
        h[9] = (uint8_t)(pad * 8u);
    } else {
        /* pad octet count, full message flag + blocks to follow, S / N(S) /
         * fragment sequence number */
        h[0] |= (uint8_t)(pad & 0x10u);
        h[1] |= (uint8_t)(pad & 0x0Fu);
        h[8] = (uint8_t)(0x80u | total);
        h[9] = p.confirmed ? 0x88 : 0x08;
    }
    data_header(h);
    if (p.prop_mfid >= 0) {
        uint8_t ph[10];
        ph[0] = (uint8_t)((p.prop_sap << 4) | 15u);
        ph[1] = (uint8_t)p.prop_mfid;
        memcpy(ph + 2, p.prop_data, 8);
        data_header(ph);
    }

    std::vector<uint8_t> pdu(p.data);
    pdu.resize(pdu.size() + pad, 0);
    const uint32_t crc = dmrrx::data_crc32(pdu.data(), pdu.size());
    for (int i = 3; i >= 0; i--)
        pdu.push_back((uint8_t)(crc >> (8 * i)));
    for (unsigned i = 0; i < n_blk; i++) {
        uint8_t blk[24];
        const uint8_t *d = pdu.data() + (size_t)i * per;
        if (p.confirmed) {
            const uint16_t c9 = dmrrx::data_crc9(p.data_type, (uint8_t)(i & 0x7Fu), d, per);
            blk[0]            = (uint8_t)(((i & 0x7Fu) << 1) | (c9 >> 8));
            blk[1]            = (uint8_t)c9;
            memcpy(blk + 2, d, per);
        } else {
            memcpy(blk, d, per);
        }
        data_block(p.data_type, blk);
    }
    return (int)(bursts.size() - before);
}

void SlotScript::status(uint32_t src, uint32_t dst, bool group, unsigned status10)
{
    const uint8_t h[10] = {(uint8_t)((group ? 0x80 : 0) | 14u),
                           0xA0,
                           (uint8_t)(dst >> 16),
                           (uint8_t)(dst >> 8),
                           (uint8_t)dst,
                           (uint8_t)(src >> 16),
                           (uint8_t)(src >> 8),
                           (uint8_t)src,
                           (uint8_t)((status10 >> 8) & 3u),
                           (uint8_t)status10};
    data_header(h);
}

void SlotScript::call(const CallSpec &c)
{
    uint8_t raw[9];
    raw[0] = c.group ? 0x00 : 0x03;
    raw[1] = c.fid;
    raw[2] = (uint8_t)((c.encrypted ? 0x40 : 0x00) | (c.emergency ? 0x80 : 0x00));
    raw[3] = (uint8_t)(c.dst >> 16);
    raw[4] = (uint8_t)(c.dst >> 8);
    raw[5] = (uint8_t)c.dst;
    raw[6] = (uint8_t)(c.src >> 16);
    raw[7] = (uint8_t)(c.src >> 8);
    raw[8] = (uint8_t)c.src;
    const CDMRLC lc(raw);

    for (int i = 0; i < c.headers; i++) {
        BurstSpec b;
        CDMRFullLC full;
        full.encode(lc, b.bits, DT_VOICE_LC_HEADER);
        finish_data(b, DT_VOICE_LC_HEADER);
        b.busy = true;
        bursts.push_back(b);
    }
    if (c.encrypted) {
        /* algorithm 0x21 (RC4), key 5, MI, destination */
        const uint8_t pi[10] = {0x21,
                                c.fid ? c.fid : (uint8_t)0x10,
                                0x05,
                                0x12,
                                0x34,
                                0x56,
                                0x78,
                                (uint8_t)(c.dst >> 16),
                                (uint8_t)(c.dst >> 8),
                                (uint8_t)c.dst};
        pi_header(pi);
    }

    /* talker alias: header + up to three blocks of 7 octets, UTF-8 format */
    std::vector<CDMRLC> ta;
    if (!c.alias.empty()) {
        std::string a = c.alias.substr(0, 27);
        uint8_t buf[28];
        memset(buf, 0, sizeof(buf));
        buf[0] = (uint8_t)((2u << 6) | ((unsigned)a.size() << 1));
        memcpy(buf + 1, a.data(), a.size());
        const unsigned blocks = (unsigned)((a.size() + 1 + 6) / 7);
        for (unsigned i = 0; i < blocks; i++) {
            uint8_t r[9];
            r[0] = (uint8_t)(0x04 + i);
            r[1] = 0x00;
            memcpy(r + 2, buf + 7 * i, 7);
            ta.push_back(CDMRLC(r));
        }
    }

    unsigned fi = c.first_frame;
    for (int sf = 0; sf < c.superframes; sf++) {
        /* embedded LC of this superframe: call LC and alias blocks in turn */
        CDMREmbeddedData emb;
        if (!ta.empty() && (sf & 1))
            emb.setLC(ta[(unsigned)(sf / 2) % ta.size()]);
        else
            emb.setLC(lc);
        for (unsigned n = 0; n < 6; n++) {
            BurstSpec b;
            b.voice_index = (int)(sent_ambe.size() / 27);
            const uint8_t *f[3];
            for (int k = 0; k < 3; k++) {
                f[k] = kAmbeVectors[fi % kNumAmbeVectors].frame72;
                fi++;
                sent_ambe.insert(sent_ambe.end(), f[k], f[k] + 9);
            }
            /* frame 1: bits 0..71, frame 2: 72..107 + 156..191, frame 3:
             * 192..263 */
            memcpy(b.bits, f[0], 9);
            memcpy(b.bits + 24, f[2], 9);
            for (unsigned i = 0; i < 36; i++) {
                const unsigned v0 = (f[1][i >> 3] >> (7 - (i & 7))) & 1u;
                const unsigned v1 = (f[1][(36 + i) >> 3] >> (7 - ((36 + i) & 7))) & 1u;
                const unsigned p0 = 72 + i, p1 = 156 + i;
                if (v0)
                    b.bits[p0 >> 3] |= (uint8_t)(0x80u >> (p0 & 7));
                if (v1)
                    b.bits[p1 >> 3] |= (uint8_t)(0x80u >> (p1 & 7));
            }
            if (n == 0) {
                put_sync(b.bits, m_mode, false);
            } else {
                const unsigned char lcss = emb.getData(b.bits, (unsigned char)n);
                CDMREMB e;
                e.setColorCode(m_cc);
                e.setPI(c.encrypted);
                e.setLCSS(lcss);
                e.getData(b.bits);
            }
            b.tx   = true;
            b.busy = true;
            bursts.push_back(b);
        }
    }
    if (c.terminator) {
        BurstSpec b;
        CDMRFullLC full;
        full.encode(lc, b.bits, DT_TERMINATOR_WITH_LC);
        finish_data(b, DT_TERMINATOR_WITH_LC);
        b.busy = true;
        bursts.push_back(b);
    }
}

/* ---------------------------------------------------------------------- */
/* CACH: layout of the MMDVM modem firmware (DMRTX.cpp): TACT bits at 0, 4,
 * 8, 12, 14, 18, 22; the 17 payload bits at the remaining positions.      */
/* ---------------------------------------------------------------------- */

void build_cach(bool at, bool tc, unsigned lcss, const uint8_t frag17[17], uint8_t cach[3])
{
    static const uint8_t kPayloadPos[17] = {1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 15, 16, 17, 19, 20, 21, 23};
    const bool ls1 = (lcss & 2u) != 0;
    const bool ls0 = (lcss & 1u) != 0;
    const bool h0  = at ^ tc ^ ls1;
    const bool h1  = tc ^ ls1 ^ ls0;
    const bool h2  = at ^ tc ^ ls0;
    cach[0] = cach[1] = cach[2] = 0;
    cach[0] |= at ? 0x80u : 0x00u;
    cach[0] |= tc ? 0x08u : 0x00u;
    cach[1] |= ls1 ? 0x80u : 0x00u;
    cach[1] |= ls0 ? 0x08u : 0x00u;
    cach[1] |= h0 ? 0x02u : 0x00u;
    cach[2] |= h1 ? 0x20u : 0x00u;
    cach[2] |= h2 ? 0x02u : 0x00u;
    for (unsigned i = 0; i < 17; i++) {
        if (frag17[i])
            cach[kPayloadPos[i] >> 3] |= (uint8_t)(0x80u >> (kPayloadPos[i] & 7));
    }
}

/* ---------------------------------------------------------------------- */
/* Modulator                                                              */
/* ---------------------------------------------------------------------- */

double rrc(double t, double a)
{
    const double pi = 3.14159265358979323846;
    if (fabs(t) < 1e-9)
        return 1.0 - a + 4.0 * a / pi;
    if (fabs(fabs(t) - 1.0 / (4.0 * a)) < 1e-9)
        return a / sqrt(2.0) * ((1.0 + 2.0 / pi) * sin(pi / (4.0 * a)) + (1.0 - 2.0 / pi) * cos(pi / (4.0 * a)));
    const double num = sin(pi * t * (1.0 - a)) + 4.0 * a * t * cos(pi * t * (1.0 + a));
    const double den = pi * t * (1.0 - 16.0 * a * a * t * t);
    return num / den;
}

/* rrc(t, 0.2) from a table (1/256 symbol steps, linear interpolation: the
 * error is below 1e-5 of the peak) */
static const int kSpan = 10;
static const int kOver = 256;
static double rrc_fast(double t)
{
    static std::vector<double> tab;
    if (tab.empty()) {
        tab.resize((size_t)(2 * (kSpan + 2) * kOver + 2));
        for (size_t i = 0; i < tab.size(); i++)
            tab[i] = rrc((double)i / kOver - (kSpan + 2), 0.2);
    }
    const double x = (t + (kSpan + 2)) * kOver;
    if (x < 0.0 || x >= (double)(tab.size() - 1))
        return 0.0;
    const size_t i = (size_t)x;
    const double f = x - (double)i;
    return tab[i] + (tab[i + 1] - tab[i]) * f;
}

static uint32_t xorshift(uint32_t &s)
{
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

static double gauss(uint32_t &s)
{
    const double u1 = ((double)xorshift(s) + 1.0) / 4294967297.0;
    const double u2 = ((double)xorshift(s) + 1.0) / 4294967297.0;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * 3.14159265358979323846 * u2);
}

static void push_bits(std::vector<int8_t> &sym, const uint8_t *bits, unsigned nbits)
{
    /* dibits 01 -> +3, 00 -> +1, 10 -> -1, 11 -> -3 */
    static const int8_t kMap[4] = {+1, +3, -1, -3};
    for (unsigned i = 0; i < nbits; i += 2) {
        const unsigned b1 = (bits[i >> 3] >> (7 - (i & 7))) & 1u;
        const unsigned b0 = (bits[(i + 1) >> 3] >> (7 - ((i + 1) & 7))) & 1u;
        sym.push_back(kMap[(b1 << 1) | b0]);
    }
}

std::vector<float> modulate(Mode mode, const std::vector<BurstSpec> &slot1, const std::vector<BurstSpec> &slot2,
                            const uint8_t *short_lc, const Channel &ch)
{
    /* short LC -> four 17 bit CACH fragments */
    uint8_t frag[4][17];
    memset(frag, 0, sizeof(frag));
    if (short_lc) {
        uint8_t lc[5];
        memcpy(lc, short_lc, 4);
        lc[4] = CCRC::crc8(lc, 4);
        uint8_t enc[9];
        CDMRShortLC slc;
        slc.encode(lc, enc);
        for (unsigned i = 0; i < 68; i++)
            frag[i / 17][i % 17] = (uint8_t)((enc[i >> 3] >> (7 - (i & 7))) & 1u);
    }

    std::vector<int8_t> sym(ch.lead_symbols > 0 ? (size_t)ch.lead_symbols : 0, 0);
    const size_t n = slot1.size() > slot2.size() ? slot1.size() : slot2.size();
    unsigned cach_idx = 0;
    for (size_t i = 0; i < 2 * n; i++) {
        const std::vector<BurstSpec> &sc = (i & 1) ? slot2 : slot1;
        const BurstSpec *b               = (i / 2) < sc.size() ? &sc[i / 2] : nullptr;
        if (mode == MODE_BS && b && b->tx) {
            uint8_t cach[3];
            const unsigned lcss = !short_lc ? 0u : (cach_idx == 0 ? 1u : (cach_idx == 3 ? 2u : 3u));
            build_cach(b->busy, (i & 1) != 0, lcss, frag[cach_idx], cach);
            cach_idx = (cach_idx + 1) & 3u;
            push_bits(sym, cach, 24);
        } else {
            sym.insert(sym.end(), 12, 0);
        }
        if (b && b->tx)
            push_bits(sym, b->bits, 264);
        else
            sym.insert(sym.end(), 132, 0);
    }
    sym.insert(sym.end(), ch.tail_symbols > 0 ? (size_t)ch.tail_symbols : 0, 0);

    /* y(t) = sum a_k h(t - k), evaluated at the receiver's sample instants */
    const double dev   = 2.0 * 3.14159265358979323846 * 1944.0 / 48000.0; /* rad/sample for +-3 */
    const double step  = 1.0 / (10.0 * (1.0 + ch.ppm * 1e-6));            /* symbols per sample  */
    const int span     = kSpan;
    const size_t nsamp = (size_t)((double)sym.size() / step);
    std::vector<float> out(nsamp);
    std::vector<double> clean(nsamp);
    double p_sig = 0.0;
    size_t n_sig = 0;
    for (size_t s = 0; s < nsamp; s++) {
        const double t = (double)s * step;
        const long k0  = (long)floor(t);
        double y       = 0.0;
        for (long k = k0 - span; k <= k0 + span + 1; k++) {
            if (k < 0 || k >= (long)sym.size() || sym[(size_t)k] == 0)
                continue;
            y += sym[(size_t)k] * rrc_fast(t - (double)k);
        }
        y *= dev / 3.0;
        clean[s] = y;
        if (k0 >= 0 && k0 < (long)sym.size() && sym[(size_t)k0] != 0) {
            p_sig += y * y;
            n_sig++;
        }
    }
    const double sigma = (ch.noise && n_sig) ? sqrt(p_sig / (double)n_sig / pow(10.0, ch.snr_db / 10.0)) : 0.0;
    /* the carrier is there for kRamp symbols (0.83 ms of the 1.25 ms guard
     * time on either side of a burst) before the first and after the last
     * symbol of a transmission */
    const long kRamp = 4;
    std::vector<uint8_t> carrier(sym.size(), 0);
    for (long k = 0; k < (long)sym.size(); k++) {
        if (sym[(size_t)k] == 0)
            continue;
        for (long j = k - kRamp; j <= k + kRamp; j++)
            if (j >= 0 && j < (long)sym.size())
                carrier[(size_t)j] = 1;
    }
    uint32_t seed = ch.seed ? ch.seed : 1u;
    for (size_t s = 0; s < nsamp; s++) {
        double y       = clean[s];
        const double t = (double)s * step;
        const long k0  = (long)floor(t);
        const bool gap = k0 < 0 || k0 >= (long)sym.size() || !carrier[(size_t)k0];
        if (ch.noise) {
            /* no carrier: wide band discriminator noise, far above the
             * signal level */
            const double sg = (gap && ch.noise_in_gaps) ? (sigma > dev ? sigma : dev) * 3.0 : sigma;
            y += sg * gauss(seed);
        }
        if (ch.invert)
            y = -y;
        out[s] = (float)(y * ch.gain + ch.dc);
    }
    return out;
}

} // namespace dmrgen
