/*
 * dmr_rx host tests - packet data: the rate 3/4 and rate 1 block codings,
 * CRC-9 and CRC-32, the reassembly of packets by the receiver and the
 * interpretation of their content (text messages).
 *
 * The known vectors (trellis coded blocks, block and packet checksums, the
 * three bursts of a text message acknowledgement, text message payloads of
 * Motorola and Hytera radios) are received data from the test suite of
 * ok-dmrlib (https://github.com/OK-DMR/ok-dmrlib, AGPL-3.0); none of its
 * code is used.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

#include "dmr_data.h"
#include "dmr_gen_lib.h"
#include "dmr_internal.h"

using namespace dmrrx;
using namespace dmrgen;

static int g_fail = 0, g_checks = 0;
static bool g_dump = false; /* --dump: print what was received */
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

static uint32_t g_seed = 4711;
static uint32_t rnd()
{
    g_seed ^= g_seed << 13;
    g_seed ^= g_seed >> 17;
    g_seed ^= g_seed << 5;
    return g_seed;
}

typedef std::vector<uint8_t> Bytes;

static Bytes hex(const char *s)
{
    Bytes v;
    unsigned acc = 0, n = 0;
    for (; *s; s++) {
        unsigned d;
        if (*s >= '0' && *s <= '9')
            d = (unsigned)(*s - '0');
        else if (*s >= 'a' && *s <= 'f')
            d = (unsigned)(*s - 'a' + 10);
        else if (*s >= 'A' && *s <= 'F')
            d = (unsigned)(*s - 'A' + 10);
        else
            continue;
        acc = (acc << 4) | d;
        if (++n == 2) {
            v.push_back((uint8_t)acc);
            acc = n = 0;
        }
    }
    return v;
}
static std::string tohex(const uint8_t *d, size_t n)
{
    std::string s;
    char b[4];
    for (size_t i = 0; i < n; i++) {
        snprintf(b, sizeof(b), "%02x", d[i]);
        s += b;
    }
    return s;
}
static std::string tohex(const Bytes &v)
{
    return tohex(v.data(), v.size());
}
static Bytes operator+(const Bytes &a, const Bytes &b)
{
    Bytes r(a);
    r.insert(r.end(), b.begin(), b.end());
    return r;
}

static inline unsigned info_pos(unsigned k)
{
    return k < 98u ? k : k + 68u;
}
/* 196 info bits as a string of '0' / '1' -> burst */
static void bits_to_burst(const char *s, uint8_t burst[33])
{
    memset(burst, 0, 33);
    for (unsigned k = 0; k < 196; k++)
        put_bit(burst, info_pos(k), s[k] == '1');
}
static unsigned info_dist(const uint8_t *a, const uint8_t *b)
{
    unsigned d = 0;
    for (unsigned k = 0; k < 196; k++)
        d += get_bit(a, info_pos(k)) ^ get_bit(b, info_pos(k));
    return d;
}

/* ---------------------------------------------------------------------- */
/* Rate 3/4                                                               */
/* ---------------------------------------------------------------------- */

/* Blocks 0..2 of one confirmed packet as received: 00 01, "AHOJ TADY MAREK"
 * in UTF-16LE, pad, CRC-32. */
static const char *const kTrellisBits[3] = {
    "0010100000101111111000101011010111010010111111110010001011100010011101100010111100111110110100100111001000101110"
    "111110100001001000100010011100101111101100101111001000101001011100100010111101110010",
    "0010010011110110110100100011111111100010001101010010001000010010101011101101001001111111110100100110100011100010"
    "111100100001001011111010111000101111000001111000001000100100011100101111100001110010",
    "0010001100100010001000100010001000100010101011000010110111110010001000100010001000100010000100001101011100100010"
    "001000100010001000100010101001000000100100100010001000100010001000100010001010100110",
};
/* What they carry. The first one is a code word, but not the one that was
 * sent: "O" (4f 00) arrived as 19 80, and only the checksums tell. */
static const char *const kTrellisData[3] = {
    "0062" "00014100480019804a00200054004100",
    "02f2" "4400590020004d004100520045004b00",
    "0538" "000000000000000000000000f486aed8",
};

/* replace the dibit at symbol position sym of the info field */
static void set_dibit(uint8_t *burst, unsigned sym, unsigned v)
{
    put_bit(burst, info_pos(2 * sym), (v >> 1) & 1u);
    put_bit(burst, info_pos(2 * sym + 1), v & 1u);
}
static unsigned get_dibit(const uint8_t *burst, unsigned sym)
{
    return (get_bit(burst, info_pos(2 * sym)) << 1) | get_bit(burst, info_pos(2 * sym + 1));
}
/* dibit <-> deviation level 0..3 (-3, -1, +1, +3) */
static const uint8_t kLevelOf[4]  = {2, 3, 1, 0};
static const uint8_t kDibitOf[4]  = {3, 2, 0, 1};

static void test_trellis()
{
    /* received blocks */
    for (int i = 0; i < 3; i++) {
        uint8_t burst[33], re[33];
        bits_to_burst(kTrellisBits[i], burst);
        Trellis tr;
        trellis34_decode(burst, tr);
        const Bytes want = hex(kTrellisData[i]);
        CHECK(memcmp(tr.data, want.data(), 18) == 0, "vector %d: got %s errs %u", i, tohex(tr.data, 18).c_str(), tr.errs);
        memset(re, 0, sizeof(re));
        trellis34_encode(tr.data, re);
        CHECK(info_dist(re, burst) == tr.errs, "vector %d: errs %u, distance %u", i, tr.errs, info_dist(re, burst));
        CHECK(tr.errs == 0, "vector %d: errs %u", i, tr.errs);
        /* the serial number and CRC-9 of the block agree with the rest */
        const uint16_t crc = (uint16_t)(((tr.data[0] & 1u) << 8) | tr.data[1]);
        const bool crc_ok  = data_crc9(DMR_DT_RATE_34_DATA, tr.data[0] >> 1, tr.data + 2, 16) == crc;
        CHECK(crc_ok == (i != 0) && (tr.data[0] >> 1) == i, "vector %d: crc9 %d", i, crc_ok);
    }
    /* the three blocks are one packet, once the first one is put right */
    {
        Bytes pdu;
        for (int i = 0; i < 3; i++) {
            const Bytes b = hex(kTrellisData[i]);
            pdu.insert(pdu.end(), b.begin() + 2, b.end());
        }
        CHECK(!data_crc32_ok(pdu.data(), pdu.size()), "crc32 over the three blocks as received");
        pdu[6] = 0x4f;
        pdu[7] = 0x00;
        CHECK(data_crc32_ok(pdu.data(), pdu.size()), "crc32 over the three blocks");
        CHECK(data_crc9(DMR_DT_RATE_34_DATA, 0, pdu.data(), 16) == 0x062, "crc9 of the first block as sent");
        uint8_t sent[33] = {0}, got[33];
        Bytes blk        = hex("0062") + Bytes(pdu.begin(), pdu.begin() + 16);
        trellis34_encode(blk.data(), sent);
        bits_to_burst(kTrellisBits[0], got);
        unsigned syms = 0;
        for (unsigned k = 0; k < 98; k++)
            syms += get_bit(sent, info_pos(2 * k)) != get_bit(got, info_pos(2 * k)) ||
                    get_bit(sent, info_pos(2 * k + 1)) != get_bit(got, info_pos(2 * k + 1));
        printf("  trellis: the received code word differs from the sent one in %u bits (%u symbols)\n",
               info_dist(sent, got), syms);
    }

    /* round trip, and the encoder leaves the rest of the burst alone */
    unsigned bad = 0;
    for (int n = 0; n < 2000; n++) {
        uint8_t d[18], burst[33], keep[33];
        for (auto &x : d)
            x = (uint8_t)rnd();
        for (auto &x : burst)
            x = (uint8_t)rnd();
        memcpy(keep, burst, 33);
        trellis34_encode(d, burst);
        Trellis tr;
        trellis34_decode(burst, tr);
        if (memcmp(tr.data, d, 18) != 0 || tr.errs != 0)
            bad++;
        for (unsigned k = 98; k < 166; k++)
            if (get_bit(burst, k) != get_bit(keep, k))
                bad++;
    }
    CHECK(bad == 0, "round trip: %u bad", bad);

    /* What the decoder repairs: symbols that slipped to a neighbouring level
     * (what noise does to 4FSK), and symbols replaced by anything. */
    printf("  trellis: blocks decoded right of 3000 with n wrong symbols\n");
    unsigned right_slip[7] = {0}, right_any[7] = {0};
    unsigned errs_max_right = 0;
    for (unsigned n_err = 1; n_err <= 6; n_err++) {
        for (int kind = 0; kind < 2; kind++) {
            for (int n = 0; n < 3000; n++) {
                uint8_t d[18], burst[33];
                for (auto &x : d)
                    x = (uint8_t)rnd();
                memset(burst, 0, sizeof(burst));
                trellis34_encode(d, burst);
                bool used[98] = {false};
                for (unsigned e = 0; e < n_err; e++) {
                    unsigned sym;
                    do
                        sym = rnd() % 98u;
                    while (used[sym]);
                    used[sym]       = true;
                    const unsigned v = get_dibit(burst, sym);
                    unsigned nv;
                    if (kind == 0) {
                        const unsigned lev = kLevelOf[v];
                        const unsigned nl  = lev == 0 ? 1u : lev == 3 ? 2u : (rnd() & 1u) ? lev + 1u : lev - 1u;
                        nv                 = kDibitOf[nl];
                    } else {
                        nv = (v + 1u + rnd() % 3u) & 3u;
                    }
                    set_dibit(burst, sym, nv);
                }
                Trellis tr;
                trellis34_decode(burst, tr);
                if (memcmp(tr.data, d, 18) == 0) {
                    (kind == 0 ? right_slip : right_any)[n_err]++;
                    if (tr.errs > errs_max_right)
                        errs_max_right = tr.errs;
                }
            }
        }
        printf("    %u: level slips %4u   any %4u\n", n_err, right_slip[n_err], right_any[n_err]);
    }
    CHECK(right_slip[1] == 3000, "single level slips repaired: %u of 3000", right_slip[1]);
    CHECK(right_slip[2] >= 2400, "double level slips repaired: %u of 3000", right_slip[2]);
    CHECK(errs_max_right <= kTrellisPlausibleErrs, "a repaired block had distance %u", errs_max_right);

    /* The code has 52 check bits for 144 of data: there is a code word near
     * any 196 bits, so the distance says little. Noise must mostly be over
     * the limit all the same. */
    unsigned hist[64] = {0}, accepted = 0;
    const int kNoise = 20000;
    for (int n = 0; n < kNoise; n++) {
        uint8_t burst[33];
        for (auto &x : burst)
            x = (uint8_t)rnd();
        Trellis tr;
        trellis34_decode(burst, tr);
        hist[tr.errs < 63 ? tr.errs : 63]++;
        if (tr.errs <= kTrellisPlausibleErrs)
            accepted++;
    }
    unsigned lo = 0, mode = 0;
    while (lo < 63 && hist[lo] == 0)
        lo++;
    for (unsigned i = 0; i < 64; i++)
        if (hist[i] > hist[mode])
            mode = i;
    printf("  trellis: noise within distance");
    for (unsigned i = lo, cum = 0; i <= 16; i++) {
        cum += hist[i];
        printf(" %u: %.2f%%", i, 100.0 * cum / kNoise);
    }
    printf("\n");
    printf("  trellis: noise decodes at distance %u or more (mostly %u); %u of %d within the limit of %d\n", lo, mode,
           accepted, kNoise, (int)kTrellisPlausibleErrs);
    CHECK(accepted * 50u <= (unsigned)kNoise, "noise taken for a block: %u of %d", accepted, kNoise);
}

/* ---------------------------------------------------------------------- */
/* Rate 1, block sizes                                                    */
/* ---------------------------------------------------------------------- */

static void test_rate1()
{
    unsigned bad = 0;
    for (int n = 0; n < 500; n++) {
        uint8_t d[24], out[24], burst[33], keep[33];
        for (auto &x : d)
            x = (uint8_t)rnd();
        for (auto &x : burst)
            x = (uint8_t)rnd();
        memcpy(keep, burst, 33);
        rate1_encode(d, burst);
        rate1_decode(burst, out);
        if (memcmp(d, out, 24) != 0)
            bad++;
        for (unsigned k = 98; k < 166; k++)
            if (get_bit(burst, k) != get_bit(keep, k))
                bad++;
        /* the first 96 data bits are the first 96 info bits, the second 96
         * end at the end of the burst */
        for (unsigned k = 0; k < 96; k++)
            if (get_bit(burst, k) != get_bit(d, k) || get_bit(burst, 264 - 96 + k) != get_bit(d, 96 + k))
                bad++;
    }
    CHECK(bad == 0, "rate 1 round trip: %u bad", bad);
    CHECK(data_block_len(DMR_DT_RATE_12_DATA) == 12 && data_block_len(DMR_DT_RATE_34_DATA) == 18 &&
              data_block_len(DMR_DT_RATE_1_DATA) == 24 && data_block_len(DMR_DT_CSBK) == 0 &&
              data_block_len(DMR_DT_DATA_HEADER) == 0,
          "block lengths");
}

/* ---------------------------------------------------------------------- */
/* Checksums                                                              */
/* ---------------------------------------------------------------------- */

static void test_crc()
{
    static const struct {
        uint8_t type;
        const char *payload;
        uint8_t dbsn;
        uint16_t crc9;
    } k9[] = {
        {DMR_DT_RATE_34_DATA, "47004d00500054002e004a0047004100", 17, 459},
        {DMR_DT_RATE_34_DATA, "2e00570047005400500044002e004a00", 16, 138},
        {DMR_DT_RATE_34_DATA, "0100000101004d004d00470054002e00", 0, 409},
        {DMR_DT_RATE_34_DATA, "4a00470044002e00540047004a002e00", 1, 65},
        {DMR_DT_RATE_34_DATA, "0001410048004f004a000000a197ccb4", 0, 447},
        {DMR_DT_RATE_34_DATA, "000000000000000000000000f486aed8", 2, 312},
        {DMR_DT_RATE_12_DATA, "747ea34235ccc44a774b", 1, 69},
        {DMR_DT_RATE_12_DATA, "af12f44fde4b826afd58", 1, 84},
        {DMR_DT_RATE_1_DATA, "534ce318038bf285569e781f0a26e8ccf56fac97e263", 1, 106},
    };
    for (auto &v : k9) {
        const Bytes p     = hex(v.payload);
        const uint16_t c  = data_crc9(v.type, v.dbsn, p.data(), p.size());
        CHECK(c == v.crc9, "crc9 type %u %s dbsn %u: %u, expected %u", v.type, v.payload, v.dbsn, c, v.crc9);
    }
    /* any single bit of payload or serial number changes it */
    {
        Bytes p = hex("47004d00500054002e004a0047004100");
        unsigned same = 0;
        for (unsigned i = 0; i < 128; i++) {
            p[i >> 3] ^= (uint8_t)(0x80u >> (i & 7));
            same += data_crc9(DMR_DT_RATE_34_DATA, 17, p.data(), 16) == 459;
            p[i >> 3] ^= (uint8_t)(0x80u >> (i & 7));
        }
        for (unsigned i = 0; i < 7; i++)
            same += data_crc9(DMR_DT_RATE_34_DATA, (uint8_t)(17u ^ (1u << i)), p.data(), 16) == 459;
        CHECK(same == 0, "crc9 blind to %u single bit changes", same);
    }

    static const char *const k32[] = {
        "0001410048004f004a000000" "a197ccb4",
        "0001410048004f004a002000540041004400590020004d004100520045004b00" "000000000000000000000000" "f486aed8",
        /* rate 1/2, unconfirmed: compressed UDP header, text message
         * acknowledgement, ten pad octets */
        "d6790062620003bf0007" "00000000000000000000" "025802700ae0250010001e",
    };
    for (int i = 0; i < 2; i++) {
        Bytes p = hex(k32[i]);
        CHECK(data_crc32_ok(p.data(), p.size()), "crc32 vector %d", i);
        const uint32_t c = data_crc32(p.data(), p.size() - 4);
        CHECK(tohex(p.data() + p.size() - 4, 4) == tohex(Bytes{(uint8_t)(c >> 24), (uint8_t)(c >> 16), (uint8_t)(c >> 8), (uint8_t)c}),
              "crc32 value %d: %08x", i, c);
        unsigned pass = 0;
        for (unsigned b = 0; b < p.size() * 8; b++) {
            p[b >> 3] ^= (uint8_t)(0x80u >> (b & 7));
            pass += data_crc32_ok(p.data(), p.size());
            p[b >> 3] ^= (uint8_t)(0x80u >> (b & 7));
        }
        CHECK(pass == 0, "crc32 vector %d blind to %u single bit changes", i, pass);
    }
    CHECK(!data_crc32_ok(hex("00000000").data(), 4), "crc32 of nothing");
}

/* ---------------------------------------------------------------------- */
/* Interpretation                                                         */
/* ---------------------------------------------------------------------- */

struct Seen {
    int kind = -1;
    std::string format, text;
    unsigned sport = 0, dport = 0, status = 0;
};

static Seen interpret(const Bytes &data, unsigned dpf = 2, unsigned sap = 4, unsigned dd = 0, unsigned blocks = 1,
                      size_t cap = kPduTextMax)
{
    static char text[kPduTextMax + 8];
    dmr_evt_data_pdu_t p;
    memset(&p, 0, sizeof(p));
    p.dpf       = (uint8_t)dpf;
    p.sap       = (uint8_t)sap;
    p.blocks    = (uint8_t)blocks;
    p.header[0] = (uint8_t)dpf;
    p.header[1] = (uint8_t)(sap << 4);
    p.header[8] = (uint8_t)(dd << 2);
    if (dpf == 14 && blocks == 0) {
        p.header[8] = (uint8_t)(dd >> 8);
        p.header[9] = (uint8_t)dd;
    }
    p.data = data.data();
    p.len  = (uint16_t)data.size();
    memset(text, 0x55, sizeof(text));
    pdu_interpret(p, text, cap);
    Seen s;
    s.kind   = p.kind;
    s.format = p.format ? p.format : "(null)";
    s.text   = std::string(text, strnlen(text, sizeof(text)));
    s.sport  = p.src_port;
    s.dport  = p.dst_port;
    s.status = p.status;
    /* nothing written behind the buffer */
    for (size_t i = cap; i < sizeof(text); i++)
        if (text[i] != 0x55)
            s.kind = -2;
    if (s.text.size() >= cap || p.text != text)
        s.kind = -2;
    return s;
}

static Bytes utf16(const std::u16string &s, bool be = false)
{
    Bytes v;
    for (char16_t c : s) {
        if (be) {
            v.push_back((uint8_t)(c >> 8));
            v.push_back((uint8_t)c);
        } else {
            v.push_back((uint8_t)c);
            v.push_back((uint8_t)(c >> 8));
        }
    }
    return v;
}
static Bytes udp4(unsigned sport, unsigned dport, const Bytes &pl)
{
    Bytes d            = hex("45000000 12340000 40110000 0c2337fe 0c2337fc");
    const size_t total = 28 + pl.size();
    d[2]               = (uint8_t)(total >> 8);
    d[3]               = (uint8_t)total;
    const size_t ulen  = 8 + pl.size();
    const Bytes u      = {(uint8_t)(sport >> 8), (uint8_t)sport, (uint8_t)(dport >> 8), (uint8_t)dport,
                          (uint8_t)(ulen >> 8),  (uint8_t)ulen,  0,                     0};
    return d + u + pl;
}
/* Motorola text message as the radios send it */
static Bytes tms_text(const std::u16string &s, uint8_t seq = 5)
{
    const Bytes t    = utf16(s);
    const size_t len = 4 + t.size();
    return Bytes{(uint8_t)(len >> 8), (uint8_t)len, 0xA0, 0x00, (uint8_t)(0x80u | seq), 0x04} + t;
}
/* Hytera text message */
static Bytes tmp_text(const std::u16string &s, bool group = false)
{
    const Bytes t    = utf16(s);
    const size_t len = 12 + t.size();
    Bytes d          = Bytes{0x09, 0x00, (uint8_t)(group ? 0xB1 : 0xA1), (uint8_t)(len >> 8), (uint8_t)len} +
              hex("00000007 0a2337fc 0a2337fe") + t;
    unsigned sum = 0;
    for (size_t i = 2; i < d.size(); i++)
        sum += d[i];
    d.push_back((uint8_t)(~sum + 0x33u));
    d.push_back(0x03);
    return d;
}

#define SEEN(s, k, f, t)                                                                                              \
    CHECK((s).kind == (k) && (s).format == (f) && (s).text == (t), "kind %d format \"%s\" text \"%s\"", (s).kind,     \
          (s).format.c_str(), (s).text.c_str())

static void test_interpret()
{
    Seen s;

    /* Motorola: as received (ok-dmrlib), bare and behind the two kinds of
     * UDP header */
    const Bytes ahoj = hex("000DE001019544 6100 6800 6F00 6A00");
    s                = interpret(ahoj, 2, 9);
    SEEN(s, DMR_PDU_TEXT, "Motorola TMS", "ahoj");
    s = interpret(udp4(4007, 4007, ahoj));
    SEEN(s, DMR_PDU_TEXT, "Motorola TMS", "ahoj");
    CHECK(s.sport == 4007 && s.dport == 4007, "ports %u %u", s.sport, s.dport);
    s = interpret(hex("d679 00 00 00 0fa7 0fa7") + ahoj, 2, 3);
    SEEN(s, DMR_PDU_TEXT, "Motorola TMS", "ahoj");
    CHECK(s.sport == 4007 && s.dport == 4007, "ports %u %u", s.sport, s.dport);
    /* pad octets that the header did not announce */
    s = interpret(udp4(4007, 4007, ahoj + hex("0000000000")));
    SEEN(s, DMR_PDU_TEXT, "Motorola TMS", "ahoj");
    for (const char *v : {"00049F009520", "0003BF0007", "00021F00"}) {
        s = interpret(udp4(4007, 4007, hex(v)));
        SEEN(s, DMR_PDU_SERVICE, "Motorola TMS acknowledgement", "");
    }
    s = interpret(udp4(4007, 4007, hex("0003D00001")));
    SEEN(s, DMR_PDU_SERVICE, "Motorola TMS control", "");
    /* the acknowledgement of the recorded bursts: manufacturer port ids */
    s = interpret(hex("d6790062620003bf0007"), 2, 3);
    SEEN(s, DMR_PDU_SERVICE, "Motorola TMS acknowledgement", "");
    /* other alphabets, more than one line, a long text */
    s = interpret(udp4(4007, 4007, tms_text(u"Grüße aus Köln\r\nΑθήνα — Москва €5")));
    SEEN(s, DMR_PDU_TEXT, "Motorola TMS", "Grüße aus Köln\nΑθήνα — Москва €5");
    s = interpret(udp4(4007, 4007, tms_text(u"smile \xD83D\xDE00 !")));
    SEEN(s, DMR_PDU_TEXT, "Motorola TMS", "smile \xF0\x9F\x98\x80 !");
    {
        std::u16string big;
        for (int i = 0; i < 400; i++)
            big += (char16_t)(u'a' + i % 26);
        s = interpret(udp4(4007, 4007, tms_text(big)));
        CHECK(s.kind == DMR_PDU_TEXT && s.text.size() == 400 && s.text[399] == 'a' + 399 % 26, "long text: %zu",
              s.text.size());
        /* a text that does not fit is cut, between characters */
        std::u16string wide;
        for (int i = 0; i < 300; i++)
            wide += u'ж';
        s = interpret(udp4(4007, 4007, tms_text(wide)), 2, 4, 0, 1, 64);
        CHECK(s.kind == DMR_PDU_TEXT && s.text.size() == 62, "cut text: kind %d, %zu bytes", s.kind, s.text.size());
    }

    /* Hytera: as received (ok-dmrlib) */
    const Bytes oliver =
        hex("0980a10022000000010a01b2070a03640e4f004c004900560045005200200054004500530054007a03");
    s = interpret(udp4(5016, 5016, oliver));
    SEEN(s, DMR_PDU_TEXT, "Hytera TMP", "OLIVER TEST");
    CHECK(s.sport == 5016 && s.dport == 5016, "ports %u %u", s.sport, s.dport);
    s = interpret(hex("0001 00 01 01") + oliver, 2, 3);
    SEEN(s, DMR_PDU_TEXT, "Hytera TMP", "OLIVER TEST");
    CHECK(s.sport == 5016 && s.dport == 5016, "ports %u %u", s.sport, s.dport);
    s = interpret(oliver, 2, 9);
    SEEN(s, DMR_PDU_TEXT, "Hytera TMP", "OLIVER TEST");
    s = interpret(udp4(5016, 5016, hex("0980a2000D000000010a01b2070a030000003103")));
    SEEN(s, DMR_PDU_SERVICE, "Hytera TMP acknowledgement", "");
    s = interpret(udp4(5016, 5016, hex("09c0a200120003000000020a01b2070a03000000010203e203")));
    SEEN(s, DMR_PDU_SERVICE, "Hytera TMP acknowledgement", "");
    s = interpret(udp4(5016, 5016, tmp_text(u"Treffen um 18:30 am Tor 2", true)));
    SEEN(s, DMR_PDU_TEXT, "Hytera TMP", "Treffen um 18:30 am Tor 2");
    {
        /* with option field: two octets behind the text */
        Bytes d = tmp_text(u"mit Option");
        d[1]    = 0x40;
        const size_t len = (((size_t)d[3] << 8) | d[4]) + 4; /* option length and option */
        d[3]             = (uint8_t)(len >> 8);
        d[4]             = (uint8_t)len;
        d.insert(d.begin() + 5, {0x00, 0x02});
        d.insert(d.end() - 2, {0xAA, 0xBB});
        s = interpret(udp4(5016, 5016, d));
        SEEN(s, DMR_PDU_TEXT, "Hytera TMP", "mit Option");
    }

    /* the standard's text message port: UTF-16BE */
    s = interpret(hex("0001 00 01 01") + utf16(u"Standard text ΑΒΓ", true), 2, 3);
    SEEN(s, DMR_PDU_TEXT, "text message (UDP 5016)", "Standard text ΑΒΓ");

    /* services known by their port */
    s = interpret(udp4(4001, 4001, hex("0d0822040000000151")));
    SEEN(s, DMR_PDU_SERVICE, "location (LRRP)", "");
    s = interpret(udp4(4005, 4005, hex("000cf02040")));
    SEEN(s, DMR_PDU_SERVICE, "registration (ARS)", "");
    s = interpret(hex("0001 00 02 02 0102030405060708"), 2, 3);
    SEEN(s, DMR_PDU_SERVICE, "location (Hytera)", "");
    {
        Bytes icmp = hex("4500001c 12340000 40010000 0c2337fe 0c2337fc 0800f7ff 00000000");
        s          = interpret(icmp);
        SEEN(s, DMR_PDU_SERVICE, "ICMP", "");
        icmp[9] = 6;
        s       = interpret(icmp);
        SEEN(s, DMR_PDU_UNKNOWN, "TCP", "");
    }

    /* formats nobody announced: text is found where it is */
    s = interpret(udp4(3000, 3000, hex("a1b2c3d4e5f6") + utf16(u"Unknown radio says hello")));
    SEEN(s, DMR_PDU_TEXT, "text? (UTF-16LE)", "Unknown radio says hello");
    s = interpret(hex("a1b2c3d405") + utf16(u"Odd offset, other order", true) + hex("0000ff"), 2, 9);
    SEEN(s, DMR_PDU_TEXT, "text? (UTF-16BE)", "Odd offset, other order");
    s = interpret(udp4(3000, 3000, hex("01") + Bytes{'p', 'l', 'a', 'i', 'n', ' ', 'b', 'y', 't', 'e', 's'} + hex("00")));
    SEEN(s, DMR_PDU_TEXT, "text? (8 bit)", "plain bytes");
    s = interpret(utf16(u"École à côté"), 2, 9);
    SEEN(s, DMR_PDU_TEXT, "text? (UTF-16LE)", "École à côté");

    /* and no text where there is none */
    s = interpret(udp4(3000, 3000, hex("8f3a11c05e77d2096b44a9f1037ce8255d90b6")));
    SEEN(s, DMR_PDU_UNKNOWN, "UDP", "");
    s = interpret(hex("8f3a11c05e77d2096b44a9f1037ce8255d90b6"), 2, 9);
    SEEN(s, DMR_PDU_UNKNOWN, "data", "");
    s = interpret(Bytes(40, 0), 2, 9);
    SEEN(s, DMR_PDU_UNKNOWN, "data", "");
    s = interpret(Bytes(), 2, 9);
    SEEN(s, DMR_PDU_UNKNOWN, "data", "");
    s = interpret(Bytes(), 14, 10, 0x2A5, 0);
    CHECK(s.kind == DMR_PDU_STATUS && s.status == 0x2A5 && s.format == "status" && s.text.empty(), "status: kind %d %u",
          s.kind, s.status);

    /* short data with a defined format */
    s = interpret(hex("0123456789abcdef"), 13, 10, 1);
    SEEN(s, DMR_PDU_TEXT, "short data, BCD", "0123456789*#abc");
    {
        const char *t = "Seven bit";
        Bytes d((strlen(t) * 7 + 7) / 8 + 1, 0);
        for (size_t i = 0; t[i]; i++)
            for (unsigned k = 0; k < 7; k++)
                put_bit(d.data(), (unsigned)(7 * i + k), ((unsigned)t[i] >> (6 - k)) & 1u);
        s = interpret(d, 13, 10, 2);
        SEEN(s, DMR_PDU_TEXT, "short data, 7 bit", "Seven bit");
    }
    s = interpret(hex("47 72 fc df 65 00 00"), 13, 10, 3);
    SEEN(s, DMR_PDU_TEXT, "short data, ISO 8859", "Grüße");
    s = interpret(Bytes{'G', 'r', 0xC3, 0xBC, 0xC3, 0x9F, 'e', ' ', 0xE2, 0x82, 0xAC, 0, 0}, 13, 10, 0x12);
    SEEN(s, DMR_PDU_TEXT, "short data, UTF-8", "Grüße €");
    s = interpret(utf16(u"sechzehn", true) + hex("0000"), 13, 10, 0x13);
    SEEN(s, DMR_PDU_TEXT, "short data, UTF-16", "sechzehn");
    s = interpret(hex("fffe") + utf16(u"mit BOM"), 13, 10, 0x13);
    SEEN(s, DMR_PDU_TEXT, "short data, UTF-16", "mit BOM");
    s = interpret(utf16(u"little"), 13, 10, 0x15);
    SEEN(s, DMR_PDU_TEXT, "short data, UTF-16", "little");
    s = interpret(hex("00000055 00000033 00000032 000020ac 00000000"), 13, 10, 0x16);
    SEEN(s, DMR_PDU_TEXT, "short data, UTF-32", "U32€");
    s = interpret(hex("8f3a11c05e77d2096b44a9f1037ce8255d90b6"), 13, 10, 0);
    SEEN(s, DMR_PDU_UNKNOWN, "data", "");

    /* anything at all: no crash, the text stays in its buffer and is UTF-8;
     * random octets are rarely taken for a message */
    unsigned broken = 0, texts = 0, random = 0;
    const int kFuzz = 60000;
    for (int n = 0; n < kFuzz; n++) {
        Bytes d(rnd() % 120u);
        const unsigned style = rnd() % 4u;
        for (auto &x : d)
            x = style == 0 ? (uint8_t)rnd() : style == 1 ? (uint8_t)(rnd() % 4u ? 0 : rnd()) : (uint8_t)(rnd() & 0x7Fu);
        static const uint8_t kDpf[] = {2, 3, 13, 14};
        /* some start like the known formats */
        if (style == 3 && d.size() > 30) {
            const Bytes pre = (rnd() & 1u) ? hex("4500005012340000401100000c2337fe0c2337fc0fa70fa7003c0000")
                                           : hex("0980a10022000000010a01b2070a03640e");
            memcpy(d.data(), pre.data(), pre.size());
            if (rnd() & 1u)
                d[d.size() - 1] = 0x03;
        }
        const size_t cap = (rnd() & 3u) ? (size_t)kPduTextMax : 1u + rnd() % 40u;
        const unsigned dpf = kDpf[rnd() % 4u];
        s                  = interpret(d, dpf, rnd() % 16u, rnd() % 32u, 1, cap);
        if (s.kind < 0 || (s.kind != DMR_PDU_TEXT && !s.text.empty()) || (s.kind == DMR_PDU_TEXT && s.text.empty()))
            broken++;
        /* well formed UTF-8 */
        for (size_t i = 0; i < s.text.size();) {
            const uint8_t c  = (uint8_t)s.text[i];
            const size_t len = c < 0x80 ? 1 : c < 0xC2 ? 0 : c < 0xE0 ? 2 : c < 0xF0 ? 3 : c < 0xF5 ? 4 : 0;
            if (len == 0 || i + len > s.text.size()) {
                broken++;
                break;
            }
            for (size_t k = 1; k < len; k++)
                if (((uint8_t)s.text[i + k] & 0xC0) != 0x80)
                    broken++;
            i += len;
        }
        /* (short data that says it is text in some coding is text) */
        if (style == 0 && dpf != 13)
            random++;
        if (style == 0 && dpf != 13 && s.kind == DMR_PDU_TEXT) {
            texts++;
            if (g_dump)
                printf("    %s: \"%s\" %s\n", s.format.c_str(), s.text.c_str(), tohex(d).c_str());
        }
    }
    CHECK(broken == 0, "fuzz: %u broken results", broken);
    printf("  interpret: %u of %u random packets taken for text\n", texts, random);
    CHECK(texts * 200u <= random, "random packets taken for text: %u of %u", texts, random);
}

/* ---------------------------------------------------------------------- */
/* The receiver                                                           */
/* ---------------------------------------------------------------------- */

struct Pdu {
    dmr_event_t e;
    Bytes data;
    std::string text, format;
    const dmr_evt_data_pdu_t &p() const { return e.u.data_pdu; }
};
struct Capture {
    std::vector<Pdu> pdu;
    unsigned headers = 0, blocks[16] = {0}, blocks_ok[16] = {0}, events = 0;
};

static void on_event(void *u, const dmr_event_t *e)
{
    Capture *c = (Capture *)u;
    c->events++;
    if (e->type == DMR_EVT_DATA_HEADER)
        c->headers++;
    if (e->type == DMR_EVT_DATA_BURST && e->u.data.slot_type_ok) {
        c->blocks[e->u.data.data_type & 15]++;
        c->blocks_ok[e->u.data.data_type & 15] += e->u.data.payload_ok;
    }
    if (e->type != DMR_EVT_DATA_PDU)
        return;
    Pdu p;
    p.e                   = *e;
    const auto &d         = e->u.data_pdu;
    p.data                = Bytes(d.data, d.data + d.len);
    p.text                = d.text ? d.text : "(null)";
    p.format              = d.format ? d.format : "(null)";
    p.e.u.data_pdu.data   = nullptr;
    p.e.u.data_pdu.text   = nullptr;
    p.e.u.data_pdu.format = nullptr;
    c->pdu.push_back(p);
}

static Capture run(const std::vector<float> &x, int block = 960)
{
    Capture c;
    dmr_rx_config_t cfg;
    dmr_rx_config_default(&cfg);
    cfg.event_cb   = on_event;
    cfg.event_user = &c;
    dmr_rx_t *rx   = dmr_rx_create(&cfg);
    size_t pos     = 0;
    while (pos < x.size()) {
        size_t n = block < 0 ? 1 + rnd() % 3000 : (size_t)block;
        if (n > x.size() - pos)
            n = x.size() - pos;
        dmr_rx_process_fm(rx, x.data() + pos, n);
        pos += n;
    }
    dmr_rx_destroy(rx);
    if (g_dump)
        for (auto &p : c.pdu)
            printf("    TS%u %u>%u dpf%u sap%u type%u blocks %u missing %u complete %u crc %u conf %u kind %u [%s] "
                   "\"%s\" %s\n",
                   p.e.slot, p.p().src_id, p.p().dst_id, p.p().dpf, p.p().sap, p.p().block_type, p.p().blocks,
                   p.p().blocks_missing, p.p().complete, p.p().crc_ok, p.p().confirmed, p.p().kind, p.format.c_str(),
                   p.text.c_str(), tohex(p.data).c_str());
    return c;
}

struct Scenario {
    Mode mode;
    SlotScript s1, s2;
    Scenario(Mode m, uint8_t cc) : mode(m), s1(m, cc), s2(m, cc) {}
    std::vector<float> signal(const Channel &ch = Channel()) const
    {
        static const uint8_t kShortLcTx[4] = {0x01, 0x80, 0x00, 0x86};
        return modulate(mode, s1.bursts, s2.bursts, mode == MODE_BS ? kShortLcTx : nullptr, ch);
    }
    /* both slots as long as the longer one */
    void even()
    {
        while (s2.bursts.size() < s1.bursts.size())
            s2.idle(1);
        while (s1.bursts.size() < s2.bursts.size())
            s1.idle(1);
    }
};

static const char *rate_name(uint8_t t)
{
    return t == DMR_DT_RATE_12_DATA ? "1/2" : t == DMR_DT_RATE_34_DATA ? "3/4" : "1";
}

/* the bursts of a real acknowledgement of a text message (ok-dmrlib): colour
 * code 5, 2308094 -> 2308092, unconfirmed, rate 1/2, SAP 3 */
static const char *const kSmsBurst[3] = {
    "7abc3520240678e3a3436a8b55bdff57d75df5d55ed179b2304122624d0589a7bc",
    "430d22106233407c00b0219a55ddff57d75df5d6f1492a46d43d20c20b8291214b",
    "008a00da01b401400330180015ddff57d75df5d6f104025802700ae0250010001e",
};
static const char *const kSmsPreamble = "55e105fbbde427040a68305294fdff57d75df5dcae42369824097da3bedb329255";

static void test_rx_recorded()
{
    Scenario sc(MODE_BS, 5);
    sc.s1.idle(8);
    for (int i = 0; i < 4; i++)
        sc.s1.burst_raw(hex(kSmsPreamble).data(), DMR_DT_CSBK);
    sc.s1.burst_raw(hex(kSmsBurst[0]).data(), DMR_DT_DATA_HEADER);
    sc.s1.burst_raw(hex(kSmsBurst[1]).data(), DMR_DT_RATE_12_DATA);
    sc.s1.burst_raw(hex(kSmsBurst[2]).data(), DMR_DT_RATE_12_DATA);
    sc.s1.idle(6);
    sc.even();
    const Capture c = run(sc.signal());
    CHECK(c.pdu.size() == 1 && c.headers == 1, "%zu packets, %u headers", c.pdu.size(), c.headers);
    if (c.pdu.size() != 1)
        return;
    const Pdu &p = c.pdu[0];
    CHECK(p.e.slot == 1 && p.e.color_code == 5 && p.p().src_id == 2308094 && p.p().dst_id == 2308092 && !p.p().is_group,
          "TS%u cc %u %u > %u", p.e.slot, p.e.color_code, p.p().src_id, p.p().dst_id);
    CHECK(p.p().dpf == 2 && p.p().sap == 3 && p.p().blocks == 2 && p.p().blocks_missing == 0 &&
              p.p().block_type == DMR_DT_RATE_12_DATA && !p.p().confirmed,
          "dpf %u sap %u blocks %u missing %u type %u confirmed %u", p.p().dpf, p.p().sap, p.p().blocks,
          p.p().blocks_missing, p.p().block_type, p.p().confirmed);
    CHECK(p.p().complete && p.p().crc_ok, "complete %u crc %u", p.p().complete, p.p().crc_ok);
    CHECK(tohex(p.data) == "d6790062620003bf0007", "data %s", tohex(p.data).c_str());
    CHECK(p.p().kind == DMR_PDU_SERVICE && p.format == "Motorola TMS acknowledgement" && p.text.empty(),
          "kind %u format %s", p.p().kind, p.format.c_str());
}

static void test_rx_roundtrip()
{
    static const uint8_t kTypes[3] = {DMR_DT_RATE_12_DATA, DMR_DT_RATE_34_DATA, DMR_DT_RATE_1_DATA};
    const std::u16string msg       = u"Tanmatsu: Prüfung der Textnachricht, Zeile 1\r\nZeile 2 — fertig.";
    const std::string want         = "Tanmatsu: Prüfung der Textnachricht, Zeile 1\nZeile 2 — fertig.";
    const Bytes payload            = udp4(4007, 4007, tms_text(msg));

    for (uint8_t type : kTypes) {
        for (int confirmed = 0; confirmed < 2; confirmed++) {
            Scenario sc(MODE_BS, 1);
            sc.s1.idle(8);
            PacketSpec ps;
            ps.data_type = type;
            ps.confirmed = confirmed != 0;
            ps.group     = confirmed == 0;
            ps.dst       = confirmed ? 2308092 : 9;
            ps.data      = payload;
            const int n  = sc.s1.packet(ps);
            sc.s1.idle(6);
            sc.even();
            const Capture c = run(sc.signal(), -1);
            CHECK(c.pdu.size() == 1, "rate %s confirmed %d: %zu packets", rate_name(type), confirmed, c.pdu.size());
            if (c.pdu.size() != 1)
                continue;
            const Pdu &p = c.pdu[0];
            CHECK(p.e.slot == 1 && p.p().src_id == ps.src && p.p().dst_id == ps.dst && p.p().is_group == ps.group &&
                      p.p().response_requested == confirmed && p.p().dpf == (confirmed ? 3 : 2) && p.p().sap == 4,
                  "rate %s confirmed %d: TS%u %u > %u group %u dpf %u sap %u", rate_name(type), confirmed, p.e.slot,
                  p.p().src_id, p.p().dst_id, p.p().is_group, p.p().dpf, p.p().sap);
            CHECK(p.p().blocks == n - 1 && p.p().blocks_missing == 0 && p.p().complete && p.p().crc_ok &&
                      p.p().confirmed == confirmed && p.p().block_type == type,
                  "rate %s confirmed %d: blocks %u (%d) missing %u complete %u crc %u confirmed %u type %u",
                  rate_name(type), confirmed, p.p().blocks, n - 1, p.p().blocks_missing, p.p().complete, p.p().crc_ok,
                  p.p().confirmed, p.p().block_type);
            CHECK(p.data == payload, "rate %s confirmed %d: data %s", rate_name(type), confirmed, tohex(p.data).c_str());
            CHECK(p.p().kind == DMR_PDU_TEXT && p.format == "Motorola TMS" && p.text == want &&
                      p.p().src_port == 4007 && p.p().dst_port == 4007,
                  "rate %s confirmed %d: kind %u [%s] \"%s\"", rate_name(type), confirmed, p.p().kind, p.format.c_str(),
                  p.text.c_str());
            CHECK(c.blocks[type] == (unsigned)(n - 1) && c.blocks_ok[type] == (unsigned)(n - 1),
                  "rate %s: %u data bursts, %u ok of %d", rate_name(type), c.blocks[type], c.blocks_ok[type], n - 1);
        }
    }

    /* every length from nothing to several blocks: pad octets and CRC come
     * off right */
    unsigned bad = 0, runs = 0;
    for (uint8_t type : kTypes) {
        for (int confirmed = 0; confirmed < 2; confirmed++) {
            Scenario sc(MODE_BS, 1);
            sc.s1.idle(8);
            std::vector<Bytes> sent;
            for (unsigned len = 1; len <= 50; len += (len < 26 ? 1 : 7)) {
                PacketSpec ps;
                ps.data_type = type;
                ps.confirmed = confirmed != 0;
                ps.sap       = 9;
                ps.data.resize(len);
                for (auto &x : ps.data)
                    x = (uint8_t)(0x80u | rnd()); /* nothing that reads as text, no zero at the end */
                sent.push_back(ps.data);
                sc.s1.packet(ps);
                sc.s1.idle(1);
            }
            sc.s1.idle(4);
            sc.even();
            const Capture c = run(sc.signal());
            runs++;
            if (c.pdu.size() != sent.size()) {
                bad++;
                printf("    rate %s confirmed %d: %zu packets of %zu\n", rate_name(type), confirmed, c.pdu.size(),
                       sent.size());
                continue;
            }
            for (size_t i = 0; i < sent.size(); i++)
                if (c.pdu[i].data != sent[i] || !c.pdu[i].p().crc_ok || c.pdu[i].p().confirmed != confirmed)
                    bad++;
        }
    }
    CHECK(bad == 0, "lengths: %u wrong in %u runs", bad, runs);
}

static void test_rx_cases()
{
    const Bytes payload = udp4(5016, 5016, tmp_text(u"Hytera message with more than one block of text in it"));
    const std::string want = "Hytera message with more than one block of text in it";

    /* the header's "response requested" bit is no proof of how the blocks
     * are laid out: the CRC-32 decides */
    for (int confirmed = 0; confirmed < 2; confirmed++) {
        Scenario sc(MODE_BS, 1);
        sc.s1.idle(8);
        PacketSpec ps;
        ps.data_type = DMR_DT_RATE_34_DATA;
        ps.confirmed = confirmed != 0;
        ps.dpf       = 13; /* short data says nothing about it either */
        ps.dd_format = 0;
        ps.response  = !confirmed;
        ps.sap       = 4;
        ps.data      = payload;
        sc.s1.packet(ps);
        sc.s1.idle(6);
        sc.even();
        const Capture c = run(sc.signal());
        CHECK(c.pdu.size() == 1 && c.pdu[0].p().crc_ok && c.pdu[0].p().confirmed == confirmed &&
                  c.pdu[0].data == payload && c.pdu[0].text == want,
              "misleading header, confirmed %d: %zu packets", confirmed, c.pdu.size());
    }

    /* proprietary header between header and blocks */
    {
        Scenario sc(MODE_BS, 1);
        sc.s1.idle(8);
        PacketSpec ps;
        ps.data_type = DMR_DT_RATE_34_DATA;
        ps.confirmed = true;
        ps.sap       = 9;
        ps.prop_mfid = 0x68;
        ps.prop_sap  = 4;
        ps.data      = payload;
        const int n  = sc.s1.packet(ps);
        sc.s1.idle(6);
        sc.even();
        const Capture c = run(sc.signal());
        CHECK(c.pdu.size() == 1, "proprietary header: %zu packets", c.pdu.size());
        if (c.pdu.size() == 1) {
            const Pdu &p = c.pdu[0];
            CHECK(p.p().prop_headers == 1 && p.p().mfid == 0x68 && p.p().sap == 4 && p.p().blocks == n - 1 &&
                      p.p().complete && p.p().crc_ok && p.data == payload && p.text == want &&
                      p.p().prop_header[0] == 0x4F && p.p().prop_header[1] == 0x68,
                  "proprietary header: n %u mfid %02x sap %u blocks %u complete %u crc %u \"%s\"", p.p().prop_headers,
                  p.p().mfid, p.p().sap, p.p().blocks, p.p().complete, p.p().crc_ok, p.text.c_str());
        }
    }

    /* short data, and a status message */
    {
        Scenario sc(MODE_BS, 1);
        sc.s1.idle(8);
        PacketSpec ps;
        ps.data_type = DMR_DT_RATE_12_DATA;
        ps.dpf       = 13;
        ps.dd_format = 0x12;
        ps.sap       = 10;
        ps.group     = true;
        ps.dst       = 262;
        const char *t = "Kurznachricht in UTF-8: äöü";
        ps.data.assign(t, t + strlen(t));
        sc.s1.packet(ps);
        sc.s1.idle(2);
        sc.s1.status(1234567, 7654321, false, 0x155);
        sc.s1.idle(6);
        sc.even();
        const Capture c = run(sc.signal());
        CHECK(c.pdu.size() == 2, "short data + status: %zu packets", c.pdu.size());
        if (c.pdu.size() == 2) {
            const Pdu &p = c.pdu[0], &q = c.pdu[1];
            CHECK(p.p().dpf == 13 && p.p().is_group && p.p().dst_id == 262 && p.p().complete && p.p().crc_ok &&
                      p.p().kind == DMR_PDU_TEXT && p.format == "short data, UTF-8" && p.text == t,
                  "short data: dpf %u complete %u crc %u [%s] \"%s\"", p.p().dpf, p.p().complete, p.p().crc_ok,
                  p.format.c_str(), p.text.c_str());
            CHECK(q.p().kind == DMR_PDU_STATUS && q.p().status == 0x155 && q.p().src_id == 1234567 &&
                      q.p().dst_id == 7654321 && q.p().blocks == 0 && q.data.empty() && q.text.empty(),
                  "status: kind %u status %u %u > %u", q.p().kind, q.p().status, q.p().src_id, q.p().dst_id);
        }
    }

    /* a block that is not received: the packet is reported, as incomplete,
     * with what the other blocks say */
    for (int how = 0; how < 3; how++) {
        for (int confirmed = 0; confirmed < 2; confirmed++) {
            Scenario sc(MODE_BS, 1);
            sc.s1.idle(8);
            PacketSpec ps;
            ps.data_type = DMR_DT_RATE_34_DATA;
            ps.confirmed = confirmed != 0;
            ps.data      = payload;
            const size_t at = sc.s1.bursts.size();
            const int n     = sc.s1.packet(ps);
            BurstSpec &b    = sc.s1.bursts[at + 5]; /* the fifth block: in the middle of the text */
            if (how == 0) {
                b.tx = false;
            } else if (how == 1) {
                for (unsigned k = 0; k < 196; k++) /* the block's bits, not sync and slot type */
                    if (k < 88 || k >= 108)
                        if (rnd() % 3u == 0)
                            b.bits[info_pos(k) >> 3] ^= (uint8_t)(0x80u >> (info_pos(k) & 7));
            } else {
                for (auto &x : b.bits) /* nothing of it is left */
                    x = (uint8_t)rnd();
            }
            sc.s1.idle(6);
            sc.even();
            const Capture c = run(sc.signal());
            CHECK(c.pdu.size() == 1, "lost block %d/%d: %zu packets", how, confirmed, c.pdu.size());
            if (c.pdu.size() != 1)
                continue;
            const Pdu &p = c.pdu[0];
            CHECK(!p.p().complete && !p.p().crc_ok && p.p().blocks == n - 1, "lost block %d/%d: complete %u crc %u",
                  how, confirmed, p.p().complete, p.p().crc_ok);
            /* a block of noise with a readable slot type cannot be told from
             * a received one in an unconfirmed packet */
            if (how != 1)
                CHECK(p.p().blocks_missing == 1, "lost block %d/%d: missing %u", how, confirmed, p.p().blocks_missing);
            CHECK(p.p().confirmed == confirmed, "lost block %d/%d: confirmed %u", how, confirmed, p.p().confirmed);
            /* the text of the blocks before the gap is there (noise in the
             * place of a block of an unconfirmed packet is as good as text) */
            if (how != 1 || confirmed)
                CHECK(p.format == "Hytera TMP" && p.text.rfind("Hytera me", 0) == 0 && p.text.size() < want.size(),
                      "lost block %d/%d: [%s] \"%s\"", how, confirmed, p.format.c_str(), p.text.c_str());
        }
    }

    /* the transmission ends before the last block */
    {
        Scenario sc(MODE_BS, 1);
        sc.s1.idle(8);
        PacketSpec ps;
        ps.data_type = DMR_DT_RATE_12_DATA;
        ps.confirmed = true;
        ps.data      = payload;
        const int n  = sc.s1.packet(ps);
        sc.s1.bursts.resize(sc.s1.bursts.size() - 3);
        sc.s1.idle(6);
        sc.even();
        const Capture c = run(sc.signal());
        CHECK(c.pdu.size() == 1 && !c.pdu[0].p().complete && c.pdu[0].p().blocks == n - 1 &&
                  c.pdu[0].p().blocks_missing == 3 && !c.pdu[0].p().crc_ok,
              "cut short: %zu packets, missing %u", c.pdu.size(), c.pdu.empty() ? 0 : c.pdu[0].p().blocks_missing);
        /* ... or the carrier drops */
        Scenario sd(MODE_BS, 1);
        sd.s1.idle(8);
        sd.s1.packet(ps);
        sd.s1.bursts.resize(sd.s1.bursts.size() - 3);
        sd.even();
        Channel ch;
        ch.tail_symbols = 4800 * 3;
        ch.noise        = true;
        ch.snr_db       = 30;
        const Capture d = run(sd.signal(ch));
        CHECK(d.pdu.size() == 1 && !d.pdu[0].p().complete && d.pdu[0].p().blocks_missing >= 3,
              "carrier lost: %zu packets, missing %u", d.pdu.size(), d.pdu.empty() ? 0 : d.pdu[0].p().blocks_missing);
    }

    /* a header and nothing behind it: no packet */
    {
        Scenario sc(MODE_BS, 1);
        sc.s1.idle(8);
        PacketSpec ps;
        ps.data = payload;
        const int n = sc.s1.packet(ps);
        sc.s1.bursts.resize(sc.s1.bursts.size() - (size_t)(n - 1));
        sc.s1.idle(6);
        sc.even();
        const Capture c = run(sc.signal());
        CHECK(c.pdu.empty() && c.headers == 1, "header alone: %zu packets, %u headers", c.pdu.size(), c.headers);
    }

    /* a voice call starts in the middle of a packet; two headers in a row */
    {
        Scenario sc(MODE_BS, 1);
        sc.s1.idle(8);
        PacketSpec ps;
        ps.data_type = DMR_DT_RATE_34_DATA;
        ps.data      = payload;
        sc.s1.packet(ps);
        sc.s1.bursts.resize(sc.s1.bursts.size() - 2);
        CallSpec call;
        call.superframes = 2;
        sc.s1.call(call);
        sc.s1.idle(2);
        const int n = sc.s1.packet(ps);
        sc.s1.bursts.resize(sc.s1.bursts.size() - (size_t)(n - 1));
        ps.src = 77;
        sc.s1.packet(ps);
        sc.s1.idle(6);
        sc.even();
        const Capture c = run(sc.signal());
        CHECK(c.pdu.size() == 2, "interrupted: %zu packets", c.pdu.size());
        if (c.pdu.size() == 2)
            CHECK(!c.pdu[0].p().complete && c.pdu[0].p().blocks_missing == 2 && c.pdu[1].p().complete &&
                      c.pdu[1].p().crc_ok && c.pdu[1].p().src_id == 77 && c.pdu[1].text == want,
                  "interrupted: missing %u, second complete %u crc %u", c.pdu[0].p().blocks_missing,
                  c.pdu[1].p().complete, c.pdu[1].p().crc_ok);
    }

    /* both timeslots at once, different packets; then the other modes */
    {
        Scenario sc(MODE_BS, 7);
        sc.s1.idle(8);
        sc.s2.idle(9);
        PacketSpec a, b;
        a.data_type = DMR_DT_RATE_34_DATA;
        a.confirmed = true;
        a.data      = udp4(4007, 4007, tms_text(u"message on timeslot one, the longer of the two messages"));
        b.data_type = DMR_DT_RATE_12_DATA;
        b.src       = 111;
        b.dst       = 222;
        b.data      = udp4(4007, 4007, tms_text(u"timeslot two"));
        sc.s1.packet(a);
        sc.s2.packet(b);
        sc.s1.idle(6);
        sc.even();
        const Capture c = run(sc.signal(), -1);
        CHECK(c.pdu.size() == 2, "two slots: %zu packets", c.pdu.size());
        unsigned seen = 0;
        for (auto &p : c.pdu) {
            if (p.e.slot == 1 && p.text == "message on timeslot one, the longer of the two messages" && p.p().crc_ok)
                seen |= 1;
            if (p.e.slot == 2 && p.text == "timeslot two" && p.p().crc_ok && p.p().src_id == 111)
                seen |= 2;
        }
        CHECK(seen == 3, "two slots: seen %u", seen);
    }
    for (Mode m : {MODE_MS, MODE_DM1, MODE_DM2}) {
        Scenario sc(m, 3);
        SlotScript &s = m == MODE_DM2 ? sc.s2 : sc.s1;
        PacketSpec ps;
        ps.data_type = DMR_DT_RATE_34_DATA;
        ps.confirmed = true;
        ps.data      = payload;
        uint8_t pre[10] = {0xBD, 0x00, 0x80, 0x05, 0x23, 0x38, 0x7C, 0x23, 0x38, 0x7E};
        s.csbk(pre, 6);
        s.packet(ps);
        if (m == MODE_DM2)
            sc.s1.silence((int)sc.s2.bursts.size());
        else
            sc.s2.silence((int)sc.s1.bursts.size());
        Channel ch;
        ch.noise  = true;
        ch.snr_db = 25;
        const Capture c = run(sc.signal(ch));
        CHECK(c.pdu.size() == 1 && c.pdu[0].p().complete && c.pdu[0].p().crc_ok && c.pdu[0].text == want,
              "mode %d: %zu packets", (int)m, c.pdu.size());
    }
}

/* What noise does: a packet is either reported right or marked as doubtful,
 * and no signal makes no packets. */
static void test_rx_noise()
{
    const std::string want = "Noise test message 0123456789 abcdefghijklmnopqrstuvwxyz";
    const Bytes payload    = udp4(4007, 4007, tms_text(u"Noise test message 0123456789 abcdefghijklmnopqrstuvwxyz"));
    static const uint8_t kTypes[3] = {DMR_DT_RATE_12_DATA, DMR_DT_RATE_34_DATA, DMR_DT_RATE_1_DATA};
    printf("  packets received right of 12 (rate 1/2, 3/4, 1; unconfirmed / confirmed)\n");
    unsigned lies = 0;
    for (double snr : {20.0, 14.0, 11.0, 9.0, 7.0, 5.0}) {
        printf("    %4.0f dB:", snr);
        for (uint8_t type : kTypes) {
            for (int confirmed = 0; confirmed < 2; confirmed++) {
                unsigned right = 0, reported = 0;
                for (uint32_t seed = 1; seed <= 3; seed++) {
                    Scenario sc(MODE_BS, 1);
                    sc.s1.idle(10);
                    PacketSpec ps;
                    ps.data_type = type;
                    ps.confirmed = confirmed != 0;
                    ps.data      = payload;
                    for (int k = 0; k < 4; k++) {
                        sc.s1.packet(ps);
                        sc.s1.idle(2);
                    }
                    sc.s1.idle(4);
                    sc.even();
                    Channel ch;
                    ch.noise  = true;
                    ch.snr_db = snr;
                    ch.seed   = seed * 131u + type;
                    const Capture c = run(sc.signal(ch));
                    for (auto &p : c.pdu) {
                        reported++;
                        const bool same = p.data == payload && p.text == want;
                        if (p.p().crc_ok && same)
                            right++;
                        if (p.p().crc_ok && !same)
                            lies++;
                        if (p.p().crc_ok != (p.p().complete && same) && p.p().complete)
                            lies += !same && p.p().crc_ok;
                    }
                }
                printf("  %2u", right);
                if (snr >= 14.0)
                    CHECK(right == 12, "rate %s confirmed %d at %.0f dB: %u right, %u reported", rate_name(type),
                          confirmed, snr, right, reported);
            }
        }
        printf("\n");
    }
    CHECK(lies == 0, "%u packets passed the CRC with wrong content", lies);

    /* no signal at all: a minute of discriminator noise */
    {
        Scenario sc(MODE_BS, 1);
        Channel ch;
        ch.noise        = true;
        ch.lead_symbols = 4800 * 60;
        ch.tail_symbols = 0;
        ch.seed         = 99;
        const Capture c = run(sc.signal(ch));
        CHECK(c.pdu.empty(), "noise alone: %zu packets", c.pdu.size());
    }
}

int main(int argc, char **argv)
{
    const char *only = nullptr;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dump") == 0)
            g_dump = true;
        else
            only = argv[i];
    }
    struct {
        const char *name;
        void (*fn)();
    } tests[] = {{"trellis", test_trellis},     {"rate1", test_rate1},         {"crc", test_crc},
                 {"interpret", test_interpret}, {"recorded", test_rx_recorded}, {"roundtrip", test_rx_roundtrip},
                 {"cases", test_rx_cases},      {"noise", test_rx_noise}};
    for (auto &t : tests)
        if (!only || strcmp(t.name, only) == 0)
            t.fn();
    printf("test_data: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
