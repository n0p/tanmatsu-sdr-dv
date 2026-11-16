/*
 * dmr_rx host tests - FEC / CRC blocks.
 *
 * Known vectors are over-the-air PDUs and frames that an independent decoder
 * (dsd-fme) decoded from real recordings (test_vectors.h); on top of that
 * each code is checked for its defining property (minimum distance, error
 * correction capability) and with injected errors.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dmr_gen_lib.h"
#include "dmr_internal.h"

#include "CRC.h"
#include "DMRDefines.h"
#include "DMREMB.h"
#include "DMREmbeddedData.h"
#include "DMRLC.h"
#include "DMRShortLC.h"
#include "DMRSlotType.h"
#include "RS129.h"

#include "test_vectors.h"

#define _MAIN
namespace dsdconst {
#include "dmr_const.h"
}
#undef _MAIN

using namespace dmrrx;

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...)                                                                                              \
    do {                                                                                                              \
        g_checks++;                                                                                                   \
        if (!(cond)) {                                                                                                \
            g_fail++;                                                                                                 \
            printf("FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond);                                                  \
            printf(__VA_ARGS__);                                                                                      \
            printf("\n");                                                                                             \
        }                                                                                                             \
    } while (0)

static uint32_t g_seed = 12345;
static uint32_t rnd()
{
    g_seed ^= g_seed << 13;
    g_seed ^= g_seed >> 17;
    g_seed ^= g_seed << 5;
    return g_seed;
}
static void flip(uint8_t *p, unsigned bit)
{
    p[bit >> 3] ^= (uint8_t)(0x80u >> (bit & 7));
}
static unsigned dist(const uint8_t *a, const uint8_t *b, unsigned n)
{
    unsigned d = 0;
    for (unsigned i = 0; i < n; i++)
        d += popcount8(a[i] ^ b[i]);
    return d;
}

/* ---------------------------------------------------------------------- */

static void test_ambe()
{
    /* real frames: 72 bits off air -> 49 bits as decoded by dsd-fme/mbelib */
    unsigned bad = 0;
    for (unsigned i = 0; i < kNumAmbeVectors; i++) {
        AmbeFrame f;
        ambe_frame_decode(kAmbeVectors[i].frame72, f);
        if (memcmp(f.ambe49, kAmbeVectors[i].ambe49, 7) != 0 || f.errs != 0 || f.bad)
            bad++;
    }
    CHECK(bad == 0, "%u of %u real AMBE frames differ from the dsd-fme decode", bad, kNumAmbeVectors);

    /* the silence frame is a valid codeword (this also pins the sense of the
     * overall parity bit of C0) */
    AmbeFrame sil;
    ambe_frame_decode(kAmbeSilence72, sil);
    CHECK(sil.errs == 0 && !sil.bad, "silence frame errs=%u bad=%d", sil.errs, sil.bad);

    /* positions of the C0 (24) and C1 (23) bits within the 72 bit frame */
    unsigned c0[24], c1[23], n0 = 0, n1 = 0;
    for (unsigned i = 0; i < 36; i++) {
        if (dsdconst::rW[i] == 0)
            c0[n0++] = 2 * i;
        if (dsdconst::rW[i] == 1)
            c1[n1++] = 2 * i;
        if (dsdconst::rY[i] == 0)
            c0[n0++] = 2 * i + 1;
        if (dsdconst::rY[i] == 1)
            c1[n1++] = 2 * i + 1;
    }
    CHECK(n0 == 24 && n1 == 23, "interleave table: %u C0 bits, %u C1 bits", n0, n1);

    /* up to 3 errors in C0 and up to 3 in C1 are corrected and counted */
    unsigned wrong = 0, miscount = 0, trials = 0;
    for (unsigned t = 0; t < 4000; t++) {
        const AmbeVector &v = kAmbeVectors[rnd() % kNumAmbeVectors];
        uint8_t fr[9];
        memcpy(fr, v.frame72, 9);
        const unsigned e0 = rnd() % 4, e1 = rnd() % 4;
        unsigned used0[3] = {99, 99, 99}, used1[3] = {99, 99, 99};
        for (unsigned k = 0; k < e0; k++) {
            unsigned p;
            do
                p = rnd() % 24;
            while (p == used0[0] || p == used0[1] || p == used0[2]);
            used0[k] = p;
            flip(fr, c0[p]);
        }
        for (unsigned k = 0; k < e1; k++) {
            unsigned p;
            do
                p = rnd() % 23;
            while (p == used1[0] || p == used1[1] || p == used1[2]);
            used1[k] = p;
            flip(fr, c1[p]);
        }
        AmbeFrame f;
        ambe_frame_decode(fr, f);
        trials++;
        if (memcmp(f.ambe49, v.ambe49, 7) != 0 || f.bad)
            wrong++;
        if (f.errs != e0 + e1)
            miscount++;
    }
    CHECK(wrong == 0, "%u of %u frames with <= 3+3 bit errors decoded wrong", wrong, trials);
    CHECK(miscount == 0, "%u of %u frames with wrong error count", miscount, trials);

    /* 4 errors in C0 are always detected (extended Golay, d = 8) */
    unsigned undetected = 0;
    for (unsigned t = 0; t < 2000; t++) {
        const AmbeVector &v = kAmbeVectors[rnd() % kNumAmbeVectors];
        uint8_t fr[9];
        memcpy(fr, v.frame72, 9);
        unsigned p[4];
        for (unsigned k = 0; k < 4; k++) {
            bool dup;
            do {
                p[k] = rnd() % 24;
                dup  = false;
                for (unsigned j = 0; j < k; j++)
                    dup = dup || p[j] == p[k];
            } while (dup);
            flip(fr, c0[p[k]]);
        }
        AmbeFrame f;
        ambe_frame_decode(fr, f);
        if (!f.bad)
            undetected++;
    }
    CHECK(undetected == 0, "%u of 2000 frames with 4 errors in C0 not flagged bad", undetected);
    printf("ok   AMBE+2 frame FEC: %u real frames, error injection\n", kNumAmbeVectors);
}

static void test_slot_type()
{
    /* all 256 codewords of the Golay(20,8,7) */
    uint8_t cw[256][33];
    for (unsigned v = 0; v < 256; v++) {
        memset(cw[v], 0, 33);
        CDMRSlotType st;
        st.setColorCode((unsigned char)(v >> 4));
        st.setDataType((unsigned char)(v & 15));
        st.getData(cw[v]);
    }
    unsigned dmin = 99;
    for (unsigned a = 0; a < 256; a++)
        for (unsigned b = a + 1; b < 256; b++) {
            const unsigned d = dist(cw[a], cw[b], 33);
            if (d < dmin)
                dmin = d;
        }
    /* the standard calls it (20,8,7); the code as specified (generator matrix
     * of TS 102 361-1 B.3.1) has minimum distance 8 */
    CHECK(dmin == 8, "Golay(20,8) minimum distance %u", dmin);

    static const unsigned pos[20] = {98, 99, 100, 101, 102, 103, 104, 105, 106, 107,
                                     156, 157, 158, 159, 160, 161, 162, 163, 164, 165};
    unsigned bad = 0, n = 0;
    for (unsigned v = 0; v < 256; v += 5) {
        for (unsigned i = 0; i < 20; i++)
            for (unsigned j = i; j < 20; j++)
                for (unsigned k = j; k < 20; k++) {
                    uint8_t w[33];
                    memcpy(w, cw[v], 33);
                    flip(w, pos[i]);
                    if (j != i)
                        flip(w, pos[j]);
                    if (k != j && j != i)
                        flip(w, pos[k]);
                    const unsigned ne = 1u + (j != i) + (k != j && j != i);
                    const SlotType s  = slot_type_decode(w);
                    n++;
                    if (!s.ok || s.cc != (v >> 4) || s.data_type != (v & 15) || s.errs != ne)
                        bad++;
                }
    }
    CHECK(bad == 0, "%u of %u slot types with 1..3 bit errors not corrected", bad, n);

    /* 4 errors must never be accepted as the original */
    unsigned acc = 0;
    for (unsigned t = 0; t < 5000; t++) {
        const unsigned v = rnd() & 255;
        uint8_t w[33];
        memcpy(w, cw[v], 33);
        unsigned p[4];
        for (unsigned k = 0; k < 4; k++) {
            bool dup;
            do {
                p[k] = rnd() % 20;
                dup  = false;
                for (unsigned j = 0; j < k; j++)
                    dup = dup || p[j] == p[k];
            } while (dup);
            flip(w, pos[p[k]]);
        }
        const SlotType s = slot_type_decode(w);
        if (s.ok && s.cc == (v >> 4) && s.data_type == (v & 15))
            acc++;
    }
    CHECK(acc == 0, "%u slot types with 4 errors accepted as correct", acc);
    printf("ok   slot type Golay(20,8): d_min %u, %u error patterns\n", dmin, n);
}

static void test_emb()
{
    uint8_t cw[128][33];
    for (unsigned v = 0; v < 128; v++) {
        memset(cw[v], 0, 33);
        CDMREMB e;
        e.setColorCode((unsigned char)(v >> 3));
        e.setPI((v & 4) != 0);
        e.setLCSS((unsigned char)(v & 3));
        e.getData(cw[v]);
    }
    unsigned dmin = 99;
    for (unsigned a = 0; a < 128; a++)
        for (unsigned b = a + 1; b < 128; b++) {
            const unsigned d = dist(cw[a], cw[b], 33);
            if (d < dmin)
                dmin = d;
        }
    CHECK(dmin == 6, "QR(16,7) minimum distance %u", dmin);
    static const unsigned pos[16] = {108, 109, 110, 111, 112, 113, 114, 115, 148, 149, 150, 151, 152, 153, 154, 155};
    unsigned bad = 0, n = 0;
    for (unsigned v = 0; v < 128; v++)
        for (unsigned i = 0; i < 16; i++)
            for (unsigned j = i; j < 16; j++) {
                uint8_t w[33];
                memcpy(w, cw[v], 33);
                flip(w, pos[i]);
                if (j != i)
                    flip(w, pos[j]);
                const Emb e = emb_decode(w);
                n++;
                if (!e.ok || e.cc != (v >> 3) || e.pi != ((v >> 2) & 1) || e.lcss != (v & 3) ||
                    e.errs != 1u + (j != i))
                    bad++;
            }
    CHECK(bad == 0, "%u of %u EMBs with 1..2 bit errors not corrected", bad, n);
    printf("ok   EMB QR(16,7,6): d_min %u, %u error patterns\n", dmin, n);
}

static void test_cach()
{
    /* encoder: MMDVM firmware layout (dmr_gen_lib), decoder: specification */
    unsigned bad = 0, n = 0;
    for (unsigned v = 0; v < 16; v++) {
        uint8_t frag[17];
        for (unsigned i = 0; i < 17; i++)
            frag[i] = (uint8_t)(rnd() & 1);
        uint8_t cach[3];
        dmrgen::build_cach((v & 8) != 0, (v & 4) != 0, v & 3, frag, cach);
        uint8_t pl[17];
        Tact t = cach_decode(cach, pl);
        n++;
        if (t.errs != 0 || t.at != ((v >> 3) & 1) || t.tc != ((v >> 2) & 1) || t.lcss != (v & 3) ||
            memcmp(pl, frag, 17) != 0)
            bad++;
        static const unsigned tact_pos[7] = {0, 4, 8, 12, 14, 18, 22};
        for (unsigned i = 0; i < 7; i++) {
            uint8_t c[3];
            memcpy(c, cach, 3);
            flip(c, tact_pos[i]);
            t = cach_decode(c, pl);
            n++;
            if (t.errs != 1 || t.at != ((v >> 3) & 1) || t.tc != ((v >> 2) & 1) || t.lcss != (v & 3) ||
                memcmp(pl, frag, 17) != 0)
                bad++;
        }
    }
    CHECK(bad == 0, "%u of %u CACH / TACT cases wrong", bad, n);
    printf("ok   CACH TACT Hamming(7,4): %u cases\n", n);
}

static void make_burst(uint8_t data_type, uint8_t cc, const uint8_t pdu[12], uint8_t out[33])
{
    memset(out, 0, 33);
    CBPTC19696 bptc;
    bptc.encode(pdu, out);
    CDMRSlotType st;
    st.setColorCode(cc);
    st.setDataType(data_type);
    st.getData(out);
}

static void test_bptc_and_pdus()
{
    CBPTC19696 bptc;

    /* the idle burst every MMDVM sends decodes to the specified idle PDU */
    Bptc o;
    bptc_decode(bptc, kIdleBurst, o);
    CHECK(memcmp(o.data, kIdlePdu, 12) == 0 && o.errs == 0, "idle burst: errs %u", o.errs);

    /* real PDUs: RS(12,9) / CRC-CCITT with the data type's mask */
    struct {
        const char *name;
        uint8_t type;
        const uint8_t *pdu;
    } pdus[] = {{"voice LC header", DMR_DT_VOICE_LC_HEADER, kVlcPdu},
                {"terminator with LC", DMR_DT_TERMINATOR_LC, kTlcPdu},
                {"CSBK", DMR_DT_CSBK, kCsbkPdu},
                {"data header", DMR_DT_DATA_HEADER, kDataHdrPdu},
                {"PI header", DMR_DT_PI_HEADER, kPiPdu}};
    for (auto &p : pdus) {
        uint8_t b[33];
        make_burst(p.type, 7, p.pdu, b);
        DataEval e;
        eval_data(bptc, b, e);
        CHECK(e.st.ok && e.st.cc == 7 && e.st.data_type == p.type && e.have_bptc && e.crc_ok && e.bp.errs == 0 &&
                  memcmp(e.bp.data, p.pdu, 12) == 0,
              "%s: real PDU not accepted (crc_ok %d)", p.name, e.crc_ok);
        /* the same PDU under any other data type must fail its check */
        for (auto &q : pdus) {
            if (q.type == p.type)
                continue;
            make_burst(q.type, 7, p.pdu, b);
            eval_data(bptc, b, e);
            CHECK(!e.crc_ok, "%s accepted as %s", p.name, q.name);
        }
        /* every single bit error in the PDU itself is detected */
        unsigned missed = 0;
        for (unsigned bit = 0; bit < 96; bit++) {
            uint8_t d[12];
            memcpy(d, p.pdu, 12);
            flip(d, bit);
            make_burst(p.type, 7, d, b);
            eval_data(bptc, b, e);
            if (e.crc_ok)
                missed++;
        }
        CHECK(missed == 0, "%s: %u single bit PDU errors not detected", p.name, missed);
    }

    /* channel errors: BPTC(196,96) has d = 9 as a product code; the
     * iterative row / column decoder must fix every pattern of up to 2
     * errors and (measured, printed) nearly all of 3 and 4 */
    static unsigned info_pos[196];
    for (unsigned i = 0; i < 98; i++) {
        info_pos[i]      = i;
        info_pos[98 + i] = 166 + i;
    }
    uint8_t clean[33];
    make_burst(DMR_DT_VOICE_LC_HEADER, 3, kVlcPdu, clean);
    unsigned bad1 = 0, bad2 = 0, n2 = 0;
    for (unsigned i = 0; i < 196; i++) {
        uint8_t b[33];
        memcpy(b, clean, 33);
        flip(b, info_pos[i]);
        bptc_decode(bptc, b, o);
        if (memcmp(o.data, kVlcPdu, 12) != 0 || o.errs != 1)
            bad1++;
        for (unsigned j = i + 1; j < 196; j++) {
            flip(b, info_pos[j]);
            bptc_decode(bptc, b, o);
            n2++;
            if (memcmp(o.data, kVlcPdu, 12) != 0 || o.errs != 2)
                bad2++;
            flip(b, info_pos[j]);
        }
    }
    CHECK(bad1 == 0, "BPTC: %u of 196 single errors not corrected", bad1);
    CHECK(bad2 == 0, "BPTC: %u of %u double errors not corrected", bad2, n2);
    unsigned fail[9] = {0};
    const unsigned trials = 20000;
    for (unsigned ne = 3; ne <= 8; ne++) {
        for (unsigned t = 0; t < trials; t++) {
            uint8_t b[33];
            memcpy(b, clean, 33);
            unsigned p[8];
            for (unsigned k = 0; k < ne; k++) {
                bool dup;
                do {
                    p[k] = rnd() % 196;
                    dup  = false;
                    for (unsigned j = 0; j < k; j++)
                        dup = dup || p[j] == p[k];
                } while (dup);
                flip(b, info_pos[p[k]]);
            }
            DataEval e;
            eval_data(bptc, b, e);
            if (!e.crc_ok || memcmp(e.bp.data, kVlcPdu, 12) != 0)
                fail[ne]++;
            /* a wrong payload must never pass the RS check with the right
             * bytes replaced */
            if (e.crc_ok && memcmp(e.bp.data, kVlcPdu, 12) != 0)
                CHECK(false, "BPTC+RS accepted a wrong LC");
        }
    }
    CHECK(fail[3] == 0, "BPTC: %u of %u triple errors not corrected", fail[3], trials);
    CHECK(fail[4] * 100 < trials, "BPTC: %u of %u quadruple errors not corrected", fail[4], trials);
    printf("ok   BPTC(196,96) + RS(12,9) / CRC-CCITT: 5 real PDUs; random channel errors not corrected: "
           "3: %u, 4: %u, 5: %u, 6: %u, 7: %u, 8: %u of %u each\n",
           fail[3], fail[4], fail[5], fail[6], fail[7], fail[8], trials);

    /* RS(12,9): every single wrong octet is detected */
    unsigned missed = 0;
    for (unsigned pos = 0; pos < 12; pos++)
        for (unsigned x = 1; x < 256; x++) {
            uint8_t d[12];
            memcpy(d, kVlcPdu, 12);
            d[9] ^= 0x96;
            d[10] ^= 0x96;
            d[11] ^= 0x96;
            d[pos] ^= (uint8_t)x;
            if (CRS129::check(d))
                missed++;
        }
    CHECK(missed == 0, "RS(12,9): %u single octet errors not detected", missed);
}

static void test_short_lc()
{
    uint8_t lc[5];
    memcpy(lc, kShortLc, 4);
    CHECK(CCRC::crc8(lc, 4) == kShortLcCrc, "short LC CRC8 %02X, off air %02X", CCRC::crc8(lc, 4), kShortLcCrc);
    lc[4] = kShortLcCrc;
    CDMRShortLC slc;
    uint8_t enc[9];
    slc.encode(lc, enc);
    unsigned bad = 0;
    for (int bit = -1; bit < 68; bit++) {
        uint8_t e[9];
        memcpy(e, enc, 9);
        if (bit >= 0)
            flip(e, (unsigned)bit);
        uint8_t out[5];
        const bool ok = slc.decode(e, out);
        out[0] &= 0x0F;
        if (!ok || memcmp(out, lc, 5) != 0)
            bad++;
    }
    CHECK(bad == 0, "short LC: %u of 69 cases (clean + single errors) wrong", bad);
    printf("ok   short LC: off-air CRC8, Hamming(17,12,3) single errors\n");
}

static void test_embedded_lc()
{
    unsigned bad = 0, n = 0, undetected = 0;
    for (unsigned t = 0; t < 200; t++) {
        uint8_t raw[9];
        for (auto &r : raw)
            r = (uint8_t)rnd();
        raw[0] &= 0x3F;
        const CDMRLC lc(raw);
        CDMREmbeddedData enc;
        enc.setLC(lc);
        uint8_t frag[4][33];
        uint8_t lcss[4];
        for (unsigned k = 0; k < 4; k++) {
            memset(frag[k], 0, 33);
            lcss[k] = enc.getData(frag[k], (unsigned char)(k + 1));
        }
        CHECK(lcss[0] == 1 && lcss[1] == 3 && lcss[2] == 3 && lcss[3] == 2, "embedded LC LCSS sequence");
        /* clean, then one bit error in each of the 128 bits */
        for (int bit = -1; bit < 128; bit += (t < 3 ? 1 : 17)) {
            CDMREmbeddedData dec;
            bool done = false;
            for (unsigned k = 0; k < 4; k++) {
                uint8_t f[33];
                memcpy(f, frag[k], 33);
                if (bit >= 0 && (unsigned)bit / 32 == k)
                    flip(f, 116 + (unsigned)bit % 32);
                done = dec.addData(f, lcss[k]);
            }
            uint8_t o[9];
            n++;
            if (!done || !dec.getRawData(o)) {
                bad++;
                continue;
            }
            if (memcmp(o, raw, 9) != 0)
                bad++;
        }
        /* a burst of errors must not produce a "valid" wrong LC */
        CDMREmbeddedData dec;
        bool done = false;
        for (unsigned k = 0; k < 4; k++) {
            uint8_t f[33];
            memcpy(f, frag[k], 33);
            if (k == 1)
                for (unsigned i = 0; i < 9; i++)
                    flip(f, 116 + (rnd() % 32));
            done = dec.addData(f, lcss[k]);
        }
        uint8_t o[9];
        if (done && dec.getRawData(o) && memcmp(o, raw, 9) != 0)
            undetected++;
    }
    CHECK(bad == 0, "embedded LC: %u of %u cases wrong", bad, n);
    CHECK(undetected <= 2, "embedded LC: %u of 200 corrupted LCs passed the 5 bit checksum", undetected);
    printf("ok   embedded LC: %u clean / single error cases, corrupted accepted %u of 200 (5 bit checksum)\n", n,
           undetected);
}

static void test_sync_words()
{
    /* TS 102 361-1 table 9.2 */
    static const uint64_t kSync[8] = {0x755FD7DF75F7ull, 0xDFF57D75DF5Dull, 0x7F7D5DD57DFDull, 0xD5D7F77FD757ull,
                                      0x5D577F7757FFull, 0xF7FDD5DDFD55ull, 0x7DFFD5F55D5Full, 0xD7557F5FF7F5ull};
    for (unsigned i = 0; i < 8; i++) {
        uint8_t w[33];
        memset(w, 0, 33);
        for (unsigned b = 0; b < 48; b++)
            if ((kSync[i] >> (47 - b)) & 1)
                flip(w, 108 + b);
        const unsigned e = sync_errors(w, (uint8_t)(i / 2), (i & 1) != 0);
        const unsigned o = sync_errors(w, (uint8_t)(i / 2), (i & 1) == 0);
        /* voice and data sync of one family are symbol-wise inverse (+3 <-> -3):
         * one bit of every dibit differs */
        CHECK(e == 0 && o == 24, "sync %u: errors %u, against the other type %u", i, e, o);
    }
    printf("ok   sync words (TS 102 361-1 table 9.2)\n");
}

int main()
{
    test_sync_words();
    test_ambe();
    test_slot_type();
    test_emb();
    test_cach();
    test_bptc_and_pdus();
    test_short_lc();
    test_embedded_lc();
    printf("test_fec: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
