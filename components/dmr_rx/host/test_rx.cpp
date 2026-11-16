/*
 * dmr_rx host tests - the receiver as a whole, through the public API only,
 * on synthetic signals from dmr_gen_lib (MMDVMHost encoders + analytic 4FSK
 * modulator).
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include <math.h>
#include <new>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

#include "dmr_gen_lib.h"
#include "dmr_rx.h"
#include "test_vectors.h"

using namespace dmrgen;

/* ---------------------------------------------------------------------- */
/* allocation tracking: malloc & co. are wrapped by the linker (see the
 * Makefile), operator new / delete are replaced here                      */
/* ---------------------------------------------------------------------- */

static bool g_track        = false;
static unsigned g_allocs   = 0;
extern "C" {
void *__real_malloc(size_t);
void *__real_calloc(size_t, size_t);
void *__real_realloc(void *, size_t);
void __real_free(void *);
void *__wrap_malloc(size_t n)
{
    if (g_track)
        g_allocs++;
    return __real_malloc(n);
}
void *__wrap_calloc(size_t a, size_t b)
{
    if (g_track)
        g_allocs++;
    return __real_calloc(a, b);
}
void *__wrap_realloc(void *p, size_t n)
{
    if (g_track)
        g_allocs++;
    return __real_realloc(p, n);
}
void __wrap_free(void *p)
{
    if (g_track && p)
        g_allocs++;
    __real_free(p);
}
}
void *operator new(size_t n)
{
    if (g_track)
        g_allocs++;
    void *p = __real_malloc(n ? n : 1);
    if (!p)
        abort();
    return p;
}
void *operator new[](size_t n)
{
    return operator new(n);
}
void operator delete(void *p) noexcept
{
    __real_free(p);
}
void operator delete[](void *p) noexcept
{
    __real_free(p);
}
void operator delete(void *p, size_t) noexcept
{
    __real_free(p);
}
void operator delete[](void *p, size_t) noexcept
{
    __real_free(p);
}

/* ---------------------------------------------------------------------- */

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

typedef std::vector<dmr_event_t> Events;

struct Capture {
    Events ev;
    unsigned pcm_calls[3] = {0, 0, 0};
    unsigned pcm_bad      = 0;
    double pcm_energy     = 0.0;
};

static void on_event(void *u, const dmr_event_t *e)
{
    ((Capture *)u)->ev.push_back(*e);
}
static void on_pcm(void *u, uint8_t slot, const int16_t *pcm, size_t n)
{
    Capture *c = (Capture *)u;
    if (slot < 1 || slot > 2 || n != DMR_RX_PCM_FRAME_SAMPLES) {
        c->pcm_bad++;
        return;
    }
    c->pcm_calls[slot]++;
    for (size_t i = 0; i < n; i++)
        c->pcm_energy += (double)pcm[i] * (double)pcm[i];
}

static uint32_t g_seed = 99;
static uint32_t rnd()
{
    g_seed ^= g_seed << 13;
    g_seed ^= g_seed >> 17;
    g_seed ^= g_seed << 5;
    return g_seed;
}

static bool g_dump = false; /* --dump: print the events of every run */
static std::string canon(const dmr_event_t &e);

/* block: 0 = everything in one call, -1 = random block sizes 1..3000 */
static Capture run(const std::vector<float> &x, int block = 0, dmr_rx_status_t *st = nullptr, uint8_t pcm_mask = 3)
{
    Capture c;
    dmr_rx_config_t cfg;
    dmr_rx_config_default(&cfg);
    cfg.event_cb      = on_event;
    cfg.event_user    = &c;
    cfg.pcm_cb        = on_pcm;
    cfg.pcm_user      = &c;
    cfg.pcm_slot_mask = pcm_mask;
    dmr_rx_t *rx      = dmr_rx_create(&cfg);
    size_t pos        = 0;
    while (pos < x.size()) {
        size_t n = block == 0 ? x.size() : (block < 0 ? 1 + rnd() % 3000 : (size_t)block);
        if (n > x.size() - pos)
            n = x.size() - pos;
        dmr_rx_process_fm(rx, x.data() + pos, n);
        pos += n;
    }
    if (st)
        dmr_rx_get_status(rx, st);
    dmr_rx_destroy(rx);
    if (g_dump) {
        printf("---- run: %zu samples, %zu events\n", x.size(), c.ev.size());
        for (auto &e : c.ev)
            printf("%9llu %s\n", (unsigned long long)e.sample_index, canon(e).c_str());
    }
    return c;
}

static std::string hex(const uint8_t *p, size_t n)
{
    std::string s;
    char b[4];
    for (size_t i = 0; i < n; i++) {
        snprintf(b, sizeof(b), "%02X", p[i]);
        s += b;
    }
    return s;
}

/* everything in an event that does not depend on timing, level or polarity */
static std::string canon(const dmr_event_t &e)
{
    char b[256];
    snprintf(b, sizeof(b), "%-13s ts%u%c sync%u cc%u ", dmr_rx_event_name(e.type), e.slot, e.slot_known ? ' ' : '?',
             e.sync_type, e.color_code);
    std::string s = b;
    switch (e.type) {
    case DMR_EVT_DATA_BURST:
        snprintf(b, sizeof(b), "dt%u st%u pl%u ", e.u.data.data_type, e.u.data.slot_type_ok, e.u.data.payload_ok);
        s += b + hex(e.u.data.payload, e.u.data.payload_len);
        break;
    case DMR_EVT_VOICE_BURST:
        snprintf(b, sizeof(b), "%c enc%u emb%u lcss%u errs%u,%u,%u bad%u,%u,%u ", 'A' + e.u.voice.burst_index,
                 e.u.voice.encrypted, e.u.voice.emb_ok, e.u.voice.lcss, e.u.voice.errs[0], e.u.voice.errs[1],
                 e.u.voice.errs[2], e.u.voice.bad[0], e.u.voice.bad[1], e.u.voice.bad[2]);
        s += b + hex(e.u.voice.frame[0], 27) + " " + hex(e.u.voice.ambe49[0], 21);
        break;
    case DMR_EVT_LC:
        snprintf(b, sizeof(b), "from%u pf%u flco%02X fid%02X svc%02X grp%u enc%u emg%u %u>%u ", e.u.lc.source,
                 e.u.lc.pf, e.u.lc.flco, e.u.lc.fid, e.u.lc.service_options, e.u.lc.is_group, e.u.lc.encrypted,
                 e.u.lc.emergency, e.u.lc.src_id, e.u.lc.dst_id);
        s += b + hex(e.u.lc.raw, 9);
        break;
    case DMR_EVT_CSBK:
        snprintf(b, sizeof(b), "lb%u pf%u op%02X fid%02X mbc%u ids%u %u>%u %s ", e.u.csbk.last_block, e.u.csbk.pf,
                 e.u.csbk.opcode, e.u.csbk.fid, e.u.csbk.is_mbc, e.u.csbk.has_ids, e.u.csbk.src_id, e.u.csbk.dst_id,
                 e.u.csbk.name);
        s += b + hex(e.u.csbk.raw, 10);
        break;
    case DMR_EVT_PI_HEADER:
        snprintf(b, sizeof(b), "alg%02X fid%02X key%u mi%08X dst%u ", e.u.pi.algorithm_id, e.u.pi.fid, e.u.pi.key_id,
                 e.u.pi.mi, e.u.pi.dst_id);
        s += b + hex(e.u.pi.raw, 10);
        break;
    case DMR_EVT_DATA_HEADER:
        snprintf(b, sizeof(b), "grp%u rsp%u dpf%u sap%u btf%u %u>%u ", e.u.data_header.is_group,
                 e.u.data_header.response_requested, e.u.data_header.dpf, e.u.data_header.sap,
                 e.u.data_header.blocks_to_follow, e.u.data_header.src_id, e.u.data_header.dst_id);
        s += b + hex(e.u.data_header.raw, 10);
        break;
    case DMR_EVT_SHORT_LC:
        snprintf(b, sizeof(b), "slco%u %s ", e.u.short_lc.slco, e.u.short_lc.name);
        s += b + hex(e.u.short_lc.data, 3);
        break;
    case DMR_EVT_TALKER_ALIAS:
        snprintf(b, sizeof(b), "complete%u '%s'", e.u.talker_alias.complete, e.u.talker_alias.text);
        s += b;
        break;
    case DMR_EVT_CALL_START:
    case DMR_EVT_CALL_END:
    case DMR_EVT_CALL_UPDATE:
        snprintf(b, sizeof(b), "grp%u enc%u emg%u late%u ids%u %u>%u bursts%u", e.u.call.group, e.u.call.encrypted,
                 e.u.call.emergency, e.u.call.late_entry, e.u.call.ids_valid, e.u.call.src_id, e.u.call.dst_id,
                 e.u.call.voice_bursts);
        s += b;
        break;
    default: break;
    }
    return s;
}

static std::vector<std::string> canon(const Events &ev)
{
    std::vector<std::string> v;
    for (auto &e : ev)
        v.push_back(canon(e));
    return v;
}

/* first difference of two canonical logs, for the failure message */
static std::string first_diff(const std::vector<std::string> &a, const std::vector<std::string> &b)
{
    char h[128];
    for (size_t i = 0; i < a.size() || i < b.size(); i++) {
        const std::string x = i < a.size() ? a[i] : "(end)", y = i < b.size() ? b[i] : "(end)";
        if (x != y) {
            snprintf(h, sizeof(h), "event %zu of %zu/%zu:\n    ", i, a.size(), b.size());
            return h + x + "\n    " + y;
        }
    }
    return "identical";
}

static bool same_events(const Events &a, const Events &b)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); i++)
        if (memcmp(&a[i], &b[i], sizeof(dmr_event_t)) != 0)
            return false;
    return true;
}

static unsigned count(const Events &ev, uint8_t type, int slot = 0)
{
    unsigned n = 0;
    for (auto &e : ev)
        n += e.type == type && (slot == 0 || e.slot == slot);
    return n;
}

static const dmr_event_t *find(const Events &ev, uint8_t type, int slot = 0, unsigned nth = 0)
{
    for (auto &e : ev)
        if (e.type == type && (slot == 0 || e.slot == slot)) {
            if (nth == 0)
                return &e;
            nth--;
        }
    return nullptr;
}

static const dmr_event_t *find_last(const Events &ev, uint8_t type, int slot = 0)
{
    const dmr_event_t *r = nullptr;
    for (auto &e : ev)
        if (e.type == type && (slot == 0 || e.slot == slot))
            r = &e;
    return r;
}

/* ---------------------------------------------------------------------- */
/* Scenarios                                                              */
/* ---------------------------------------------------------------------- */

static const uint8_t kShortLcTx[4] = {0x01, 0x80, 0x00, 0x86};

struct Scenario {
    Mode mode;
    SlotScript s1, s2;
    bool short_lc;
    Scenario(Mode m, uint8_t cc) : mode(m), s1(m, cc), s2(m, cc), short_lc(m == MODE_BS) {}
    std::vector<float> signal(const Channel &ch) const
    {
        return modulate(mode, s1.bursts, s2.bursts, short_lc ? kShortLcTx : nullptr, ch);
    }
    const SlotScript &script(int slot) const { return slot == 1 ? s1 : s2; }
};

/* a repeater: TS1 control / data bursts and a group call with talker alias,
 * TS2 idle and a unit-to-unit call */
static Scenario bs_scenario(uint8_t cc = 1, int superframes = 8)
{
    Scenario sc(MODE_BS, cc);
    sc.s1.idle(8);
    sc.s1.csbk(kCsbkPdu, 2);
    sc.s1.data_header(kDataHdrPdu);
    sc.s1.idle(2);
    CallSpec c1;
    c1.src         = 2345678;
    c1.dst         = 91;
    c1.alias       = "TANMATSU-1 test";
    c1.superframes = superframes;
    sc.s1.call(c1);
    sc.s1.idle(8);
    sc.s2.idle(20);
    CallSpec c2;
    c2.src         = 1001;
    c2.dst         = 2002;
    c2.group       = false;
    c2.superframes = 3;
    c2.first_frame = 7;
    sc.s2.call(c2);
    sc.s2.idle((int)sc.s1.bursts.size() - (int)sc.s2.bursts.size());
    return sc;
}

/* which transmitted burst an event belongs to (by its sample index) */
struct BurstRef {
    int slot;        /* 1 / 2 */
    long index;      /* within the slot's script */
    double offset;   /* samples from the nominal burst centre */
    const BurstSpec *spec;
};

static BurstRef locate(const Scenario &sc, const Channel &ch, uint64_t sample_index)
{
    const double per_sym = 10.0 * (1.0 + ch.ppm * 1e-6);
    const double sym     = (double)sample_index / per_sym - ch.lead_symbols - 78.0; /* CACH 12 + half a burst */
    const long i         = lround(sym / 144.0);
    BurstRef r;
    r.slot   = (int)(i & 1) + 1;
    r.index  = i >> 1;
    r.offset = (sym - (double)i * 144.0) * per_sym;
    const std::vector<BurstSpec> &b = sc.script(r.slot).bursts;
    r.spec                          = (i >= 0 && (size_t)r.index < b.size()) ? &b[(size_t)r.index] : nullptr;
    return r;
}

/* expected 49 bit vector for a transmitted 72 bit frame */
static const uint8_t *ambe49_of(const uint8_t *frame72)
{
    for (unsigned i = 0; i < kNumAmbeVectors; i++)
        if (memcmp(kAmbeVectors[i].frame72, frame72, 9) == 0)
            return kAmbeVectors[i].ambe49;
    return nullptr;
}

struct Score {
    unsigned data_sent = 0, data_ok = 0, data_wrong = 0;
    unsigned voice_sent = 0, voice_rx = 0;       /* bursts */
    unsigned frames_exact = 0, frames_bad = 0, frames_wrong = 0;
    unsigned lc_ok = 0, lc_wrong = 0;
    unsigned stray = 0;                          /* burst events with no transmitted burst */
    double max_offset = 0.0;
};

static bool lc_expected(const Scenario &sc, int slot, const dmr_evt_lc_t &lc)
{
    /* an LC is right if this slot's script contains exactly these 72 bits:
     * check against the ids the scenario uses */
    (void)sc;
    if (lc.flco >= 4 && lc.flco <= 7)
        return slot == 1; /* talker alias blocks, checked through the alias text */
    if (slot == 1)
        return lc.flco == 0 && lc.src_id == 2345678 && lc.dst_id == 91;
    return lc.flco == 3 && lc.src_id == 1001 && lc.dst_id == 2002;
}

static Score score(const Scenario &sc, const Channel &ch, const Events &ev)
{
    Score s;
    for (int slot = 1; slot <= 2; slot++)
        for (auto &b : sc.script(slot).bursts) {
            s.data_sent += b.data_type >= 0;
            s.voice_sent += b.voice_index >= 0;
        }
    for (auto &e : ev) {
        if (e.type == DMR_EVT_LC) {
            if (lc_expected(sc, e.slot, e.u.lc))
                s.lc_ok++;
            else
                s.lc_wrong++;
        }
        if (e.type != DMR_EVT_DATA_BURST && e.type != DMR_EVT_VOICE_BURST)
            continue;
        const BurstRef r = locate(sc, ch, e.sample_index);
        if (!r.spec || !r.spec->tx || r.slot != e.slot) {
            s.stray++;
            continue;
        }
        if (fabs(r.offset) > s.max_offset)
            s.max_offset = fabs(r.offset);
        if (e.type == DMR_EVT_DATA_BURST) {
            if (!e.u.data.slot_type_ok || !e.u.data.payload_ok)
                continue;
            if (e.u.data.data_type == r.spec->data_type)
                s.data_ok++;
            else
                s.data_wrong++;
        } else {
            if (r.spec->voice_index < 0) {
                s.stray++;
                continue;
            }
            s.voice_rx++;
            const uint8_t *sent = &sc.script(r.slot).sent_ambe[27u * (size_t)r.spec->voice_index];
            for (unsigned k = 0; k < 3; k++) {
                const uint8_t *want = ambe49_of(sent + 9 * k);
                if (e.u.voice.bad[k])
                    s.frames_bad++;
                else if (want && memcmp(e.u.voice.ambe49[k], want, 7) == 0)
                    s.frames_exact++;
                else
                    s.frames_wrong++;
            }
        }
    }
    return s;
}

/* ---------------------------------------------------------------------- */
/* Tests                                                                  */
/* ---------------------------------------------------------------------- */

static std::vector<std::string> g_clean_canon;

static void test_clean_bs()
{
    const Scenario sc = bs_scenario();
    Channel ch;
    ch.tail_symbols = 4800; /* long enough to see the loss of sync */
    dmr_rx_status_t st;
    const Capture c  = run(sc.signal(ch), 0, &st);
    const Events &ev = c.ev;
    g_clean_canon    = canon(ev);

    /* sync: both slots acquired once, lost once when the signal ends */
    CHECK(count(ev, DMR_EVT_SYNC_ACQUIRED, 1) == 1 && count(ev, DMR_EVT_SYNC_ACQUIRED, 2) == 1, "acquired %u/%u",
          count(ev, DMR_EVT_SYNC_ACQUIRED, 1), count(ev, DMR_EVT_SYNC_ACQUIRED, 2));
    CHECK(count(ev, DMR_EVT_SYNC_LOST) == 2 && st.sync_loss_count == 2, "sync lost %u", count(ev, DMR_EVT_SYNC_LOST));
    CHECK(ev.size() > 2 && ev[ev.size() - 1].type == DMR_EVT_SYNC_LOST && ev[ev.size() - 2].type == DMR_EVT_SYNC_LOST,
          "sync loss is not the last event");
    CHECK(!st.synced && !st.slot[0].active && !st.slot[1].active, "still synced after the end of the signal");

    /* every transmitted burst is received, on the right slot, with the
     * right content; sample_index is the burst centre */
    const Score s = score(sc, ch, ev);
    CHECK(s.data_ok == s.data_sent && s.data_wrong == 0, "data bursts %u of %u ok, %u wrong", s.data_ok, s.data_sent,
          s.data_wrong);
    CHECK(s.voice_rx == s.voice_sent && s.frames_exact == 3 * s.voice_sent, "voice bursts %u of %u, exact frames %u",
          s.voice_rx, s.voice_sent, s.frames_exact);
    CHECK(s.stray == 0, "%u burst events without a transmitted burst", s.stray);
    CHECK(s.max_offset < 12.0, "sample_index up to %.1f samples from the burst centre", s.max_offset);
    CHECK(s.lc_wrong == 0, "%u wrong LCs", s.lc_wrong);

    unsigned n_evt = 0, bad_hdr = 0, voice_errs = 0, frame_diff = 0, q_bad = 0;
    std::vector<uint8_t> rx_frames[3];
    for (auto &e : ev) {
        n_evt++;
        if (e.type == DMR_EVT_SYNC_LOST)
            continue;
        const bool voice_sync = e.sync_type == DMR_SYNC_BS_VOICE;
        if (e.color_code != 1 || e.inverted || (e.slot != 1 && e.slot != 2) || !e.slot_known ||
            (e.sync_type != DMR_SYNC_BS_DATA && !voice_sync))
            bad_hdr++;
        if (e.type == DMR_EVT_DATA_BURST && (e.sync_type != DMR_SYNC_BS_DATA || e.quality.sync_errs != 0))
            bad_hdr++;
        if (e.type == DMR_EVT_VOICE_BURST) {
            if (!voice_sync)
                bad_hdr++;
            const dmr_evt_voice_burst_t &v = e.u.voice;
            voice_errs += v.errs[0] + v.errs[1] + v.errs[2] + v.bad[0] + v.bad[1] + v.bad[2];
            rx_frames[e.slot].insert(rx_frames[e.slot].end(), v.frame[0], v.frame[0] + 27);
            if ((v.burst_index == 0) != (e.quality.sync_errs == 0) || (v.burst_index > 0 && !v.emb_ok))
                bad_hdr++;
        }
        if ((e.type == DMR_EVT_DATA_BURST || e.type == DMR_EVT_VOICE_BURST) &&
            (e.quality.fec_errs != 0 || e.quality.fec_bits == 0 || e.quality.evm > 0.03f))
            q_bad++;
    }
    CHECK(bad_hdr == 0, "%u of %u events with a wrong header (cc, slot, sync type, polarity)", bad_hdr, n_evt);
    CHECK(voice_errs == 0, "%u AMBE bit errors on a clean signal", voice_errs);
    CHECK(q_bad == 0, "%u bursts with a wrong quality report on a clean signal", q_bad);
    /* the 72 bit frames come out exactly as transmitted, in order */
    for (int slot = 1; slot <= 2; slot++)
        frame_diff += rx_frames[slot] != sc.script(slot).sent_ambe;
    CHECK(frame_diff == 0, "AMBE frames differ from the transmitted ones (%zu/%zu, %zu/%zu bytes)",
          rx_frames[1].size(), sc.s1.sent_ambe.size(), rx_frames[2].size(), sc.s2.sent_ambe.size());

    /* TS1: group call 2345678 -> TG 91 with header, embedded LC, terminator */
    const dmr_event_t *e = find(ev, DMR_EVT_LC, 1);
    CHECK(e && e->u.lc.source == DMR_LC_SRC_VOICE_HEADER && e->u.lc.flco == DMR_FLCO_GROUP_VOICE && e->u.lc.is_group &&
              e->u.lc.fid == 0 && e->u.lc.service_options == 0 && !e->u.lc.encrypted && e->u.lc.src_id == 2345678 &&
              e->u.lc.dst_id == 91,
          "TS1 voice LC header");
    unsigned emb = 0, term = 0, hdr = 0;
    for (auto &x : ev)
        if (x.type == DMR_EVT_LC && x.slot == 1 && x.u.lc.flco == 0) {
            hdr += x.u.lc.source == DMR_LC_SRC_VOICE_HEADER;
            emb += x.u.lc.source == DMR_LC_SRC_EMBEDDED;
            term += x.u.lc.source == DMR_LC_SRC_TERMINATOR;
        }
    CHECK(hdr == 2 && emb == 4 && term == 1, "TS1 LCs: %u headers, %u embedded, %u terminator", hdr, emb, term);
    e = find(ev, DMR_EVT_CALL_START, 1);
    CHECK(e && e->u.call.group && !e->u.call.encrypted && !e->u.call.late_entry && e->u.call.ids_valid &&
              e->u.call.src_id == 2345678 && e->u.call.dst_id == 91,
          "TS1 call start");
    e = find(ev, DMR_EVT_CALL_END, 1);
    CHECK(e && e->u.call.ids_valid && e->u.call.src_id == 2345678 && e->u.call.dst_id == 91 &&
              e->u.call.voice_bursts == 48,
          "TS1 call end (%u voice bursts)", e ? e->u.call.voice_bursts : 0);
    CHECK(count(ev, DMR_EVT_CALL_START) == 2 && count(ev, DMR_EVT_CALL_END) == 2, "calls: %u starts, %u ends",
          count(ev, DMR_EVT_CALL_START), count(ev, DMR_EVT_CALL_END));
    e = find_last(ev, DMR_EVT_TALKER_ALIAS, 1);
    CHECK(e && e->u.talker_alias.complete && strcmp(e->u.talker_alias.text, "TANMATSU-1 test") == 0,
          "talker alias '%s'", e ? e->u.talker_alias.text : "(none)");
    CHECK(count(ev, DMR_EVT_TALKER_ALIAS, 2) == 0, "talker alias on TS2");

    /* TS2: unit to unit call 1001 -> 2002 */
    e = find(ev, DMR_EVT_LC, 2);
    CHECK(e && e->u.lc.flco == DMR_FLCO_UNIT_TO_UNIT && !e->u.lc.is_group && e->u.lc.src_id == 1001 &&
              e->u.lc.dst_id == 2002,
          "TS2 voice LC header");
    e = find(ev, DMR_EVT_CALL_END, 2);
    CHECK(e && !e->u.call.group && e->u.call.src_id == 1001 && e->u.call.dst_id == 2002 &&
              e->u.call.voice_bursts == 18,
          "TS2 call end");

    /* CSBK (preamble 22 -> 21), data header, short LC */
    CHECK(count(ev, DMR_EVT_CSBK) == 2, "%u CSBKs", count(ev, DMR_EVT_CSBK));
    e = find(ev, DMR_EVT_CSBK, 1);
    CHECK(e && e->u.csbk.opcode == 0x3D && e->u.csbk.last_block && e->u.csbk.fid == 0 && !e->u.csbk.is_mbc &&
              memcmp(e->u.csbk.raw, kCsbkPdu, 10) == 0 && e->u.csbk.name && strstr(e->u.csbk.name, "Pre"),
          "CSBK: op %02X name %s", e ? e->u.csbk.opcode : 0, e ? e->u.csbk.name : "");
    CHECK(e && e->u.csbk.has_ids && e->u.csbk.dst_id == 21 && e->u.csbk.src_id == 22, "CSBK ids");
    e = find(ev, DMR_EVT_DATA_HEADER, 1);
    CHECK(e && count(ev, DMR_EVT_DATA_HEADER) == 1 && memcmp(e->u.data_header.raw, kDataHdrPdu, 10) == 0 &&
              e->u.data_header.dst_id == 21 && e->u.data_header.src_id == 22 &&
              e->u.data_header.dpf == (kDataHdrPdu[0] & 0x0F) && e->u.data_header.sap == (kDataHdrPdu[1] >> 4),
          "data header");
    e = find(ev, DMR_EVT_SHORT_LC);
    CHECK(e && e->u.short_lc.slco == 1 && e->u.short_lc.data[0] == 0x80 && e->u.short_lc.data[1] == 0x00 &&
              e->u.short_lc.data[2] == 0x86,
          "short LC");
    CHECK(count(ev, DMR_EVT_PI_HEADER) == 0, "PI header in a clear call");

    /* status */
    CHECK(fabsf(st.clock_ppm) < 3.0f, "clock %.1f ppm", (double)st.clock_ppm);
    CHECK(fabsf(st.deviation - 0.2545f) < 0.01f, "deviation %.4f", (double)st.deviation);
    CHECK(fabsf(st.dc_offset) < 0.012f, "dc %.4f", (double)st.dc_offset);
    CHECK(!st.inverted && st.polarity_locked, "polarity");
    CHECK(st.samples == sc.signal(ch).size(), "sample count");
    CHECK(st.slot[0].voice_bursts == 48 && st.slot[1].voice_bursts == 18 && st.slot[0].data_bursts == 24 &&
              st.slot[1].data_bursts == 54 && st.slot[0].missed_bursts <= 4 && st.slot[0].fec_errs == 0 &&
              st.slot[0].fec_bits > 0,
          "status totals: voice %u/%u data %u/%u missed %u", st.slot[0].voice_bursts, st.slot[1].voice_bursts,
          st.slot[0].data_bursts, st.slot[1].data_bursts, st.slot[0].missed_bursts);
    if (dmr_rx_has_voice()) {
        CHECK(c.pcm_calls[1] == 3 * 48 && c.pcm_calls[2] == 3 * 18 && c.pcm_bad == 0 && c.pcm_energy > 0.0,
              "PCM frames %u/%u", c.pcm_calls[1], c.pcm_calls[2]);
        const Capture c1 = run(sc.signal(ch), 0, nullptr, 2);
        CHECK(c1.pcm_calls[1] == 0 && c1.pcm_calls[2] == 3 * 18, "pcm_slot_mask: %u/%u", c1.pcm_calls[1],
              c1.pcm_calls[2]);
    } else {
        CHECK(c.pcm_calls[1] + c.pcm_calls[2] + c.pcm_bad == 0, "PCM without vocoder");
    }
    printf("ok   clean repeater signal: %zu events, %u data + %u voice bursts all exact, burst centre within %.1f "
           "samples\n",
           ev.size(), s.data_ok, s.voice_rx, s.max_offset);
}

static void test_impairments()
{
    const Scenario sc = bs_scenario();
    struct Case {
        const char *name;
        double gain, dc, ppm;
        bool invert;
    } cases[] = {
        {"gain 0.01", 0.01, 0, 0, false},
        {"gain 30", 30.0, 0, 0, false},
        {"int16 scale (32767 = pi)", 32767.0 / 3.14159265358979, 0, 0, false},
        {"dc +0.3 (118 % of the deviation)", 1.0, 0.3, 0, false},
        {"dc -0.3", 1.0, -0.3, 0, false},
        {"inverted", 1.0, 0, 0, true},
        {"+50 ppm", 1.0, 0, 50, false},
        {"-50 ppm", 1.0, 0, -50, false},
        {"+100 ppm", 1.0, 0, 100, false},
        {"-100 ppm", 1.0, 0, -100, false},
        {"gain 0.05, dc -0.02, inverted, -50 ppm", 0.05, -0.02, -50, true},
        {"gain 8, dc 3, inverted, +50 ppm", 8.0, 3.0, 50, true},
    };
    for (auto &k : cases) {
        Channel ch;
        ch.tail_symbols = 4800;
        ch.gain   = k.gain;
        ch.dc     = k.dc;
        ch.ppm    = k.ppm;
        ch.invert = k.invert;
        dmr_rx_status_t st;
        const Capture c = run(sc.signal(ch), 0, &st);
        const auto cn   = canon(c.ev);
        CHECK(cn == g_clean_canon, "%s: events differ from the clean run, %s", k.name,
              first_diff(g_clean_canon, cn).c_str());
        unsigned pol = 0;
        for (auto &e : c.ev)
            pol += e.inverted != (k.invert ? 1 : 0);
        CHECK(pol == 0 && st.inverted == (k.invert ? 1 : 0) && st.polarity_locked, "%s: polarity (%u events wrong)",
              k.name, pol);
        CHECK(fabs((double)st.clock_ppm - k.ppm) < 4.0, "%s: clock estimate %.1f ppm", k.name, (double)st.clock_ppm);
        CHECK(fabs((double)st.deviation / (0.2545 * k.gain) - 1.0) < 0.04, "%s: deviation estimate %.4f", k.name,
              (double)st.deviation);
        CHECK(fabs((double)st.dc_offset - k.dc) < 0.012 * k.gain + 0.01 * fabs(k.dc), "%s: dc estimate %.4f", k.name,
              (double)st.dc_offset);
        const Score s = score(sc, ch, c.ev);
        CHECK(s.stray == 0 && s.max_offset < 12.0, "%s: burst timing (%u stray, offset %.1f)", k.name, s.stray,
              s.max_offset);
    }
    printf("ok   gain / dc / polarity / clock offset: %zu cases decode to the identical event sequence\n",
           sizeof(cases) / sizeof(cases[0]));
}

static void test_block_sizes()
{
    const Scenario sc = bs_scenario(5, 2);
    Channel ch;
    ch.noise  = true;
    ch.snr_db = 4.0; /* FEC at work, some bursts fail */
    ch.ppm    = -37;
    ch.dc     = 0.04;
    ch.gain   = 0.5;
    ch.seed   = 4;
    const std::vector<float> x = sc.signal(ch);
    dmr_rx_status_t st0, st;
    const Capture ref = run(x, 0, &st0);
    unsigned fec = 0;
    for (auto &e : ref.ev)
        fec += e.quality.fec_errs;
    CHECK(ref.ev.size() > 80 && fec > 20, "reference run too clean / empty (%zu events, %u corrected bits)",
          ref.ev.size(), fec);
    static const int kBlocks[] = {1, 2, 7, 160, 480, 1024, 4096, 48000, -1, -1};
    for (int b : kBlocks) {
        const Capture c = run(x, b, &st);
        CHECK(same_events(ref.ev, c.ev), "block size %d: events differ, %s", b,
              first_diff(canon(ref.ev), canon(c.ev)).c_str());
        CHECK(memcmp(&st, &st0, sizeof(st)) == 0, "block size %d: final status differs", b);
        CHECK(c.pcm_calls[1] == ref.pcm_calls[1] && c.pcm_calls[2] == ref.pcm_calls[2] &&
                  c.pcm_energy == ref.pcm_energy,
              "block size %d: PCM differs", b);
    }

    /* int16 input == the same values as float / 32768 */
    std::vector<int16_t> x16(x.size());
    std::vector<float> xf(x.size());
    for (size_t i = 0; i < x.size(); i++) {
        float v = x[i] * (32767.0f / 3.14159265f);
        v       = v > 32767.0f ? 32767.0f : (v < -32768.0f ? -32768.0f : v);
        x16[i]  = (int16_t)lrintf(v);
        xf[i]   = (float)x16[i] * (1.0f / 32768.0f);
    }
    Capture c16;
    {
        dmr_rx_config_t cfg;
        dmr_rx_config_default(&cfg);
        cfg.event_cb   = on_event;
        cfg.event_user = &c16;
        dmr_rx_t *rx   = dmr_rx_create(&cfg);
        for (size_t pos = 0; pos < x16.size(); pos += 1000)
            dmr_rx_process_fm_s16(rx, x16.data() + pos, x16.size() - pos < 1000 ? x16.size() - pos : 1000);
        dmr_rx_destroy(rx);
    }
    const Capture cf = run(xf, 0);
    CHECK(same_events(c16.ev, cf.ev), "int16 input: events differ from float input, %s",
          first_diff(canon(cf.ev), canon(c16.ev)).c_str());
    CHECK(c16.ev.size() > 80, "int16 input: %zu events", c16.ev.size());
    printf("ok   block sizes 1 .. 48000 and random: bit-identical events (%zu, with %u FEC-corrected bits), status "
           "and PCM; int16 == float\n",
           ref.ev.size(), fec);
}

static void test_snr()
{
    /* SNR is signal power / noise power in 24 kHz (the whole discriminator
     * output band); the matched filter gains about 10 dB on that */
    static const double kSnr[] = {20, 14, 11, 9, 7, 5, 3};
    const unsigned kSeeds      = 4;
    const Scenario sc          = bs_scenario(1, 8);
    printf("     SNR(24k)  data ok        voice bursts   AMBE frames ok   bad inexact  LC ok/wrong  alias  calls\n");
    for (double snr : kSnr) {
        Score t;
        unsigned alias = 0, calls = 0, wrong_ids = 0, stray_evt = 0;
        for (unsigned seed = 1; seed <= kSeeds; seed++) {
            Channel ch;
            ch.noise  = true;
            ch.snr_db = snr;
            ch.seed   = seed * 77;
            ch.ppm    = (seed & 1) ? 20 : -20;
            const Capture c = run(sc.signal(ch));
            const Score s   = score(sc, ch, c.ev);
            t.data_sent += s.data_sent;
            t.data_ok += s.data_ok;
            t.data_wrong += s.data_wrong;
            t.voice_sent += s.voice_sent;
            t.voice_rx += s.voice_rx;
            t.frames_exact += s.frames_exact;
            t.frames_bad += s.frames_bad;
            t.frames_wrong += s.frames_wrong;
            t.lc_ok += s.lc_ok;
            t.lc_wrong += s.lc_wrong;
            t.stray += s.stray;
            const dmr_event_t *e = find_last(c.ev, DMR_EVT_TALKER_ALIAS);
            alias += e && e->u.talker_alias.complete && strcmp(e->u.talker_alias.text, "TANMATSU-1 test") == 0;
            calls += count(c.ev, DMR_EVT_CALL_START);
            for (auto &x : c.ev) {
                if ((x.type == DMR_EVT_CALL_END || x.type == DMR_EVT_CALL_UPDATE || x.type == DMR_EVT_CALL_START) &&
                    x.u.call.ids_valid &&
                    !((x.u.call.src_id == 2345678 && x.u.call.dst_id == 91 && x.u.call.group) ||
                      (x.u.call.src_id == 1001 && x.u.call.dst_id == 2002 && !x.u.call.group)))
                    wrong_ids++;
                if (x.type == DMR_EVT_CSBK && memcmp(x.u.csbk.raw, kCsbkPdu, 10) != 0)
                    stray_evt++;
                if (x.type == DMR_EVT_DATA_HEADER && memcmp(x.u.data_header.raw, kDataHdrPdu, 10) != 0)
                    stray_evt++;
                if (x.type == DMR_EVT_SHORT_LC && (x.u.short_lc.slco != 1 || x.u.short_lc.data[0] != 0x80))
                    stray_evt++;
                if (x.type == DMR_EVT_PI_HEADER)
                    stray_evt++;
            }
        }
        const unsigned frames = 3 * t.voice_sent;
        printf("     %5.0f dB  %4u/%-4u %3.0f%%  %4u/%-4u     %5u/%-5u %3.0f%%  %4u  %4u   %3u/%-3u      %u/%u    "
               "%u/%u\n",
               snr, t.data_ok, t.data_sent, 100.0 * t.data_ok / t.data_sent, t.voice_rx, t.voice_sent, t.frames_exact,
               frames, 100.0 * t.frames_exact / frames, t.frames_bad, t.frames_wrong, t.lc_ok, t.lc_wrong, alias,
               kSeeds, calls, 2 * kSeeds);
        /* never report something that was not transmitted */
        CHECK(t.data_wrong == 0 && wrong_ids == 0 && stray_evt == 0 && t.lc_wrong == 0,
              "%.0f dB: false decodes (data %u, call ids %u, PDUs %u, LC %u)", snr, t.data_wrong, wrong_ids,
              stray_evt, t.lc_wrong);
        if (snr >= 11) {
            CHECK(t.data_ok == t.data_sent && t.voice_rx == t.voice_sent && t.frames_exact == frames,
                  "%.0f dB: not error free", snr);
            CHECK(alias == kSeeds && calls == 2 * kSeeds, "%.0f dB: alias %u calls %u", snr, alias, calls);
        }
        if (snr >= 7) {
            CHECK(t.data_ok * 100 >= t.data_sent * 97, "%.0f dB: data bursts %u/%u", snr, t.data_ok, t.data_sent);
            CHECK(t.voice_rx * 100 >= t.voice_sent * 98, "%.0f dB: voice bursts %u/%u", snr, t.voice_rx,
                  t.voice_sent);
            CHECK(t.frames_exact * 100 >= frames * 93, "%.0f dB: AMBE frames %u/%u", snr, t.frames_exact, frames);
            CHECK(calls == 2 * kSeeds, "%.0f dB: %u calls", snr, calls);
        }
    }
    printf("ok   SNR sweep (%u seeds each, +-20 ppm)\n", kSeeds);
}

static double gauss()
{
    const double u1 = ((double)rnd() + 1.0) / 4294967297.0;
    const double u2 = ((double)rnd() + 1.0) / 4294967297.0;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * 3.14159265358979323846 * u2);
}

static void test_no_signal()
{
    const size_t n = 48000 * 60;
    std::vector<float> x(n);
    struct {
        const char *name;
        int kind;
        double level;
    } cases[] = {{"digital silence", 0, 0},
                 {"weak noise", 1, 0.003},
                 {"noise at the DMR deviation", 1, 0.25},
                 {"un-squelched discriminator noise", 2, 1.2},
                 {"1 kHz tone", 3, 0.2},
                 {"1200 baud square wave (AFSK-like)", 4, 0.25},
                 {"random 4FSK symbols at 4800 baud without sync", 5, 0}};
    for (auto &k : cases) {
        if (k.kind == 5) {
            SlotScript s(MODE_MS, 0);
            for (int i = 0; i < 1000; i++) {
                BurstSpec b;
                b.tx = true;
                for (auto &v : b.bits)
                    v = (uint8_t)rnd();
                s.bursts.push_back(b);
            }
            Channel ch;
            SlotScript s2 = s;
            for (auto &b : s2.bursts)
                for (auto &v : b.bits)
                    v = (uint8_t)rnd();
            x = modulate(MODE_MS, s.bursts, s2.bursts, nullptr, ch);
        } else {
            for (size_t i = 0; i < n; i++) {
                double v = 0.0;
                switch (k.kind) {
                case 1: v = k.level * gauss(); break;
                case 2:
                    v = k.level * gauss();
                    v = v > 3.14159 ? 3.14159 : (v < -3.14159 ? -3.14159 : v);
                    break;
                case 3: v = k.level * sin(2.0 * 3.14159265358979 * 1000.0 * (double)i / 48000.0); break;
                case 4: v = ((i / 40) & 1) ? k.level : -k.level; break;
                default: break;
                }
                x[i] = (float)v;
            }
        }
        dmr_rx_status_t st;
        const Capture c = run(x, 4096, &st);
        CHECK(c.ev.empty() && !st.synced, "%s (%.0f s): %zu events, first: %s", k.name, (double)x.size() / 48000.0,
              c.ev.size(), c.ev.empty() ? "" : canon(c.ev[0]).c_str());
    }
    printf("ok   no DMR signal (7 kinds, 60 s each): no events\n");
}

static void test_modes()
{
    struct {
        Mode mode;
        const char *name;
        uint8_t sync_data, sync_voice;
        int slot;
        bool known;
    } modes[] = {{MODE_MS, "MS sourced", DMR_SYNC_MS_DATA, DMR_SYNC_MS_VOICE, 1, false},
                 {MODE_DM1, "direct mode TS1", DMR_SYNC_DM_TS1_DATA, DMR_SYNC_DM_TS1_VOICE, 1, true},
                 {MODE_DM2, "direct mode TS2", DMR_SYNC_DM_TS2_DATA, DMR_SYNC_DM_TS2_VOICE, 2, true}};
    for (auto &m : modes) {
        for (int inv = 0; inv < 2; inv++) {
            Scenario sc(m.mode, 9);
            CallSpec cs;
            cs.src         = 7654321;
            cs.dst         = 16777215;
            cs.emergency   = true;
            cs.superframes = 3;
            sc.s1.silence(4);
            sc.s1.call(cs);
            sc.s1.silence(4);
            Channel ch;
            ch.noise  = true; /* nothing but noise between the bursts and before the first one */
            ch.snr_db = 20;
            ch.invert = inv != 0;
            const Capture c = run(sc.signal(ch));
            unsigned bad = 0, voice = 0, data = 0;
            std::vector<uint8_t> frames;
            for (auto &e : c.ev) {
                if (e.type == DMR_EVT_SYNC_LOST)
                    continue;
                if (e.slot != m.slot || e.slot_known != (m.known ? 1 : 0) || e.color_code != 9 ||
                    e.inverted != (uint8_t)inv)
                    bad++;
                if (e.type == DMR_EVT_DATA_BURST) {
                    data += e.u.data.payload_ok;
                    bad += e.sync_type != m.sync_data;
                }
                if (e.type == DMR_EVT_VOICE_BURST) {
                    voice++;
                    bad += e.sync_type != m.sync_voice;
                    frames.insert(frames.end(), e.u.voice.frame[0], e.u.voice.frame[0] + 27);
                }
            }
            CHECK(bad == 0, "%s: %u events with wrong slot / sync type / colour code", m.name, bad);
            CHECK(data == 3 && voice == 18 && frames == sc.s1.sent_ambe, "%s: %u data bursts, %u voice bursts",
                  m.name, data, voice);
            const dmr_event_t *e = find(c.ev, DMR_EVT_LC);
            CHECK(e && e->u.lc.source == DMR_LC_SRC_VOICE_HEADER && e->u.lc.emergency &&
                      e->u.lc.service_options == DMR_SVC_EMERGENCY && e->u.lc.src_id == 7654321 &&
                      e->u.lc.dst_id == 16777215,
                  "%s: LC", m.name);
            e = find(c.ev, DMR_EVT_CALL_END);
            CHECK(e && count(c.ev, DMR_EVT_CALL_START) == 1 && count(c.ev, DMR_EVT_CALL_END) == 1 &&
                      e->u.call.emergency && e->u.call.voice_bursts == 18 && e->u.call.src_id == 7654321,
                  "%s: call", m.name);
            CHECK(count(c.ev, DMR_EVT_SHORT_LC) == 0, "%s: short LC without CACH", m.name);
        }
    }
    /* colour codes */
    for (unsigned cc = 0; cc < 16; cc++) {
        Scenario sc(MODE_BS, (uint8_t)cc);
        sc.s1.idle(6);
        CallSpec cs;
        cs.superframes = 1;
        sc.s2.call(cs);
        Channel ch;
        const Capture c = run(sc.signal(ch));
        unsigned bad    = 0;
        for (auto &e : c.ev)
            bad += e.type != DMR_EVT_SYNC_LOST && e.color_code != cc;
        CHECK(bad == 0 && count(c.ev, DMR_EVT_VOICE_BURST, 2) == 6 && count(c.ev, DMR_EVT_DATA_BURST, 1) == 6,
              "colour code %u: %u events wrong", cc, bad);
    }
    printf("ok   MS sourced, direct mode TS1 / TS2 (normal and inverted), colour codes 0..15\n");
}

static void test_encrypted()
{
    for (int mode = 0; mode < 2; mode++) {
        Scenario sc(mode ? MODE_MS : MODE_BS, 3);
        CallSpec cs;
        cs.src       = 111;
        cs.dst       = 222;
        cs.encrypted = true;
        cs.fid       = 0x10;
        cs.superframes = 3;
        SlotScript &s  = sc.s1;
        if (mode)
            s.silence(3);
        else
            s.idle(3);
        s.call(cs);
        if (!mode)
            s.idle(3);
        /* a clear call on the same slot afterwards must be decoded again */
        CallSpec clear;
        clear.src         = 333;
        clear.dst         = 444;
        clear.superframes = 2;
        s.silence(2);
        s.call(clear);
        s.silence(2);
        Channel ch;
        const Capture c = run(sc.signal(ch));
        const dmr_event_t *e = find(c.ev, DMR_EVT_LC);
        CHECK(e && e->u.lc.encrypted && (e->u.lc.service_options & DMR_SVC_PRIVACY) && e->u.lc.fid == 0x10,
              "encrypted call: LC privacy bit");
        e = find(c.ev, DMR_EVT_PI_HEADER);
        CHECK(e && count(c.ev, DMR_EVT_PI_HEADER) == 1 && e->u.pi.algorithm_id == 0x21 && e->u.pi.key_id == 5 &&
                  e->u.pi.mi == 0x12345678 && e->u.pi.dst_id == 222 && e->u.pi.fid == 0x10,
              "PI header");
        e = find(c.ev, DMR_EVT_CALL_START);
        CHECK(e && e->u.call.encrypted && e->u.call.src_id == 111, "encrypted call start");
        e = find(c.ev, DMR_EVT_CALL_END);
        CHECK(e && e->u.call.encrypted && e->u.call.voice_bursts == 18, "encrypted call end");
        unsigned enc = 0, clr = 0;
        for (auto &x : c.ev)
            if (x.type == DMR_EVT_VOICE_BURST) {
                enc += x.u.voice.encrypted;
                clr += !x.u.voice.encrypted;
            }
        CHECK(enc == 18 && clr == 12, "voice bursts flagged encrypted: %u, clear: %u", enc, clr);
        e = find_last(c.ev, DMR_EVT_CALL_END);
        CHECK(e && !e->u.call.encrypted && e->u.call.src_id == 333 && e->u.call.voice_bursts == 12,
              "clear call after the encrypted one");
        /* no audio for the encrypted call, audio for the clear one */
        CHECK(c.pcm_calls[1] + c.pcm_calls[2] == (dmr_rx_has_voice() ? 3u * 12u : 0u) && c.pcm_bad == 0,
              "PCM frames: %u (encrypted call must stay silent)", c.pcm_calls[1] + c.pcm_calls[2]);

        /* late entry into the encrypted call: no header, no PI header; only
         * the PI flag of the EMB and the embedded LC say so */
        Scenario le(mode ? MODE_MS : MODE_BS, 3);
        cs.headers = 0;
        le.s1.silence(2);
        le.s1.call(cs);
        le.s1.silence(2);
        Scenario full(mode ? MODE_MS : MODE_BS, 3);
        cs.headers = 2;
        full.s1.silence(2);
        full.s1.call(cs);
        std::vector<float> x = le.signal(ch);
        /* drop the PI header too (it follows the headers in the script) */
        const Capture c2 = run(x);
        e                = find(c2.ev, DMR_EVT_CALL_END);
        CHECK(e && e->u.call.encrypted && e->u.call.late_entry, "late entry into an encrypted call");
    }
    printf("ok   encrypted calls are flagged (LC, PI header, EMB) and never reach the vocoder\n");
}

static void test_late_entry()
{
    /* the receiver is switched on in the middle of a call */
    for (int mode = 0; mode < 2; mode++) {
        Scenario sc(mode ? MODE_DM1 : MODE_BS, 2);
        CallSpec cs;
        cs.src         = 2780081;
        cs.dst         = 91;
        cs.superframes = 6;
        cs.alias       = "9H5JO Joe";
        sc.s1.call(cs);
        if (!mode)
            sc.s2.idle((int)sc.s1.bursts.size());
        Channel ch;
        ch.lead_symbols        = 0;
        std::vector<float> x   = sc.signal(ch);
        /* start inside burst D of the first superframe */
        const size_t cut = (size_t)(2 + 3) * 2880 + 700;
        x.erase(x.begin(), x.begin() + (long)cut);
        const Capture c  = run(x);
        const dmr_event_t *e = find(c.ev, DMR_EVT_CALL_START, 1);
        CHECK(e && e->u.call.late_entry && !e->u.call.ids_valid, "late entry: call start");
        e = find(c.ev, DMR_EVT_CALL_UPDATE, 1);
        CHECK(e && e->u.call.late_entry && e->u.call.ids_valid && e->u.call.src_id == 2780081 &&
                  e->u.call.dst_id == 91 && e->u.call.group,
              "late entry: ids from the embedded LC");
        /* the ids are known after at most two complete superframes */
        const dmr_event_t *first = find(c.ev, DMR_EVT_VOICE_BURST, 1);
        CHECK(e && first && e->sample_index - first->sample_index <= 15u * 2880u, "late entry: ids after %.2f s",
              e && first ? (double)(e->sample_index - first->sample_index) / 48000.0 : -1.0);
        e = find(c.ev, DMR_EVT_LC, 1);
        CHECK(e && e->u.lc.source == DMR_LC_SRC_EMBEDDED, "late entry: first LC is the embedded one");
        e = find(c.ev, DMR_EVT_CALL_END, 1);
        CHECK(e && e->u.call.ids_valid && e->u.call.src_id == 2780081 && e->u.call.voice_bursts >= 30,
              "late entry: call end with %u bursts", e ? e->u.call.voice_bursts : 0);
        e = find_last(c.ev, DMR_EVT_TALKER_ALIAS, 1);
        CHECK(e && e->u.talker_alias.complete && strcmp(e->u.talker_alias.text, "9H5JO Joe") == 0,
              "late entry: talker alias");
        CHECK(count(c.ev, DMR_EVT_CALL_START) == 1 && count(c.ev, DMR_EVT_CALL_END) == 1, "late entry: one call");
    }

    /* call without terminator: ends by timeout, exactly once */
    Scenario sc(MODE_MS, 1);
    CallSpec cs;
    cs.terminator = false;
    sc.s1.silence(2);
    sc.s1.call(cs);
    sc.s1.silence(30);
    Channel ch;
    ch.noise  = true;
    ch.snr_db = 25;
    const Capture c = run(sc.signal(ch));
    const dmr_event_t *e = find(c.ev, DMR_EVT_CALL_END);
    const dmr_event_t *v = find_last(c.ev, DMR_EVT_VOICE_BURST);
    CHECK(e && v && count(c.ev, DMR_EVT_CALL_END) == 1 && e->u.call.voice_bursts == 24 &&
              e->sample_index - v->sample_index < 48000u,
          "call without terminator: end %.2f s after the last voice burst",
          e && v ? (double)(e->sample_index - v->sample_index) / 48000.0 : -1.0);
    CHECK(count(c.ev, DMR_EVT_SYNC_LOST) == 1, "call without terminator: %u sync losses",
          count(c.ev, DMR_EVT_SYNC_LOST));
    printf("ok   late entry (ids and alias from the embedded LC), call end without terminator\n");
}

static void test_two_calls()
{
    Scenario sc(MODE_BS, 12);
    CallSpec a, b;
    a.src = 1;
    a.dst = 2;
    b.src = 16777214;
    b.dst = 16777215;
    b.first_frame = 11;
    a.superframes = b.superframes = 5;
    sc.s1.idle(3);
    sc.s1.call(a);
    sc.s1.idle(3);
    sc.s2.idle(6);
    sc.s2.call(b);
    Channel ch;
    ch.noise  = true;
    ch.snr_db = 16;
    const Capture c = run(sc.signal(ch));
    std::vector<uint8_t> fr[3];
    for (auto &e : c.ev)
        if (e.type == DMR_EVT_VOICE_BURST)
            fr[e.slot].insert(fr[e.slot].end(), e.u.voice.ambe49[0], e.u.voice.ambe49[0] + 21);
    for (int slot = 1; slot <= 2; slot++) {
        const std::vector<uint8_t> &sent = sc.script(slot).sent_ambe;
        std::vector<uint8_t> want;
        for (size_t i = 0; i < sent.size(); i += 9) {
            const uint8_t *w = ambe49_of(&sent[i]);
            want.insert(want.end(), w, w + 7);
        }
        CHECK(fr[slot] == want, "two calls: TS%d voice parameters differ (%zu/%zu bytes)", slot, fr[slot].size(),
              want.size());
    }
    const dmr_event_t *e1 = find(c.ev, DMR_EVT_CALL_END, 1), *e2 = find(c.ev, DMR_EVT_CALL_END, 2);
    CHECK(e1 && e2 && e1->u.call.src_id == 1 && e1->u.call.dst_id == 2 && e2->u.call.src_id == 16777214 &&
              e2->u.call.dst_id == 16777215 && e1->u.call.voice_bursts == 30 && e2->u.call.voice_bursts == 30,
          "two calls: call ends");
    if (dmr_rx_has_voice())
        CHECK(c.pcm_calls[1] == 90 && c.pcm_calls[2] == 90, "two calls: PCM frames %u/%u", c.pcm_calls[1],
              c.pcm_calls[2]);
    printf("ok   simultaneous calls on both timeslots\n");
}

static void test_discontinuity()
{
    /* samples are lost (USB overrun, retune): the TDMA timing jumps */
    const Scenario sc = bs_scenario(1, 8);
    Channel ch;
    ch.noise  = true;
    ch.snr_db = 18;
    static const size_t kDrop[] = {1, 333, 960, 1440, 2000, 2880 + 721, 48000};
    for (size_t drop : kDrop) {
        std::vector<float> x = sc.signal(ch);
        const size_t at      = x.size() / 2 + 517;
        x.erase(x.begin() + (long)at, x.begin() + (long)(at + drop));
        const Capture c = run(x);
        unsigned voice = 0, data = 0, wrong = 0;
        for (auto &e : c.ev) {
            voice += e.type == DMR_EVT_VOICE_BURST;
            data += e.type == DMR_EVT_DATA_BURST && e.u.data.payload_ok;
            if (e.type == DMR_EVT_LC && !lc_expected(sc, e.slot, e.u.lc))
                wrong++;
        }
        const unsigned lost_max = 6 + (unsigned)(drop / 1440);
        CHECK(voice + data + lost_max >= 144 && wrong == 0,
              "%zu samples dropped: %u voice + %u data bursts of 144, %u wrong LCs", drop, voice, data, wrong);
        const dmr_event_t *e = find_last(c.ev, DMR_EVT_CALL_END, 1);
        CHECK(e && e->u.call.src_id == 2345678 && e->u.call.dst_id == 91, "%zu samples dropped: call end", drop);
    }
    printf("ok   input discontinuities (1 .. 48000 samples dropped mid-call)\n");
}

static void test_reset_and_memory()
{
    const Scenario sc = bs_scenario(1, 2);
    Channel ch;
    ch.noise  = true;
    ch.snr_db = 12;
    ch.dc     = 0.1;
    const std::vector<float> x = sc.signal(ch);
    const Capture ref          = run(x);

    Capture c;
    dmr_rx_config_t cfg;
    dmr_rx_config_default(&cfg);
    cfg.event_cb   = on_event;
    cfg.event_user = &c;
    cfg.pcm_cb     = on_pcm;
    cfg.pcm_user   = &c;
    dmr_rx_t *rx   = dmr_rx_create(&cfg);
    /* stop in the middle of a call, on an inverted signal with another level */
    std::vector<float> y(x.begin(), x.begin() + (long)(x.size() * 2 / 3));
    for (auto &v : y)
        v = -3.0f * v;
    dmr_rx_process_fm(rx, y.data(), y.size());
    CHECK(!c.ev.empty() && c.ev.back().inverted, "reset: first part not decoded");
    dmr_rx_reset(rx);
    dmr_rx_status_t st;
    dmr_rx_get_status(rx, &st);
    CHECK(!st.synced && st.samples == 0 && !st.slot[0].call_active && !st.slot[1].active && !st.polarity_locked &&
              st.slot[0].voice_bursts == 0,
          "reset: status not cleared");
    c = Capture();
    dmr_rx_process_fm(rx, x.data(), x.size());
    CHECK(same_events(ref.ev, c.ev), "reset: events differ from a fresh receiver, %s",
          first_diff(canon(ref.ev), canon(c.ev)).c_str());
    CHECK(c.pcm_energy == ref.pcm_energy, "reset: PCM differs from a fresh receiver");
    dmr_rx_destroy(rx);

    /* fixed polarity setting: the wrong one must not decode, the right one must */
    for (int pol = DMR_POLARITY_NORMAL; pol <= DMR_POLARITY_INVERTED; pol++) {
        Capture cp;
        cfg.event_user = &cp;
        cfg.pcm_user   = &cp;
        cfg.polarity   = (uint8_t)pol;
        rx             = dmr_rx_create(&cfg);
        dmr_rx_process_fm(rx, x.data(), x.size());
        dmr_rx_destroy(rx);
        if (pol == DMR_POLARITY_NORMAL)
            CHECK(same_events(ref.ev, cp.ev), "fixed normal polarity: events differ from auto");
        else
            CHECK(cp.ev.empty(), "fixed inverted polarity on a normal signal: %zu events", cp.ev.size());
    }

    /* memory: one allocation in create (through the configured allocator),
     * none while processing */
    static size_t alloc_size = 0;
    static unsigned alloc_calls = 0, free_calls = 0;
    dmr_rx_config_default(&cfg);
    cfg.alloc = [](size_t n) -> void * {
        alloc_size = n;
        alloc_calls++;
        return __real_malloc(n);
    };
    cfg.dealloc = [](void *p) {
        free_calls++;
        __real_free(p);
    };
    static unsigned n_evt = 0, n_pcm = 0;
    cfg.event_cb = [](void *, const dmr_event_t *) { n_evt++; };
    cfg.pcm_cb   = [](void *, uint8_t, const int16_t *, size_t) { n_pcm++; };
    n_evt = n_pcm = 0;
    g_allocs     = 0;
    g_track      = true;
    rx           = dmr_rx_create(&cfg);
    const unsigned in_create = g_allocs;
    for (size_t pos = 0; pos < x.size(); pos += 777)
        dmr_rx_process_fm(rx, x.data() + pos, x.size() - pos < 777 ? x.size() - pos : 777);
    dmr_rx_reset(rx);
    dmr_rx_process_fm(rx, x.data(), x.size());
    dmr_rx_get_status(rx, &st);
    const unsigned in_process = g_allocs - in_create;
    dmr_rx_destroy(rx);
    g_track = false;
    CHECK(in_create == 0 && alloc_calls == 1 && free_calls == 1 && alloc_size == dmr_rx_state_size(),
          "create: %u allocator calls, %u heap calls besides them", alloc_calls, in_create);
    CHECK(in_process == 0 && g_allocs == 0, "%u heap calls while processing", in_process);
    CHECK(n_evt == 2 * ref.ev.size() && n_pcm == 2 * (ref.pcm_calls[1] + ref.pcm_calls[2]),
          "allocation test did not decode (%u events, %u PCM frames)", n_evt, n_pcm);
    CHECK(dmr_rx_state_size() < 300u * 1024u, "state size %zu", dmr_rx_state_size());

    /* NULL tolerance of the API */
    dmr_rx_process_fm(nullptr, x.data(), 10);
    dmr_rx_reset(nullptr);
    dmr_rx_destroy(nullptr);
    dmr_rx_get_status(nullptr, &st);
    rx = dmr_rx_create(nullptr);
    CHECK(rx != nullptr, "create with default config");
    dmr_rx_process_fm(rx, nullptr, 10);
    dmr_rx_process_fm(rx, x.data(), x.size());
    dmr_rx_get_status(rx, &st);
    CHECK(st.samples == x.size() && st.slot[0].data_bursts > 0, "decode without callbacks: status only");
    dmr_rx_destroy(rx);
    printf("ok   reset == fresh receiver; fixed polarity; state %zu bytes in one allocation, no heap use while "
           "processing (vocoder %s)\n",
           dmr_rx_state_size(), dmr_rx_has_voice() ? "included" : "not built");
}

int main(int argc, char **argv)
{
    bool quick = false;
    const char *only = nullptr;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--quick") == 0)
            quick = true;
        else if (strcmp(argv[i], "--dump") == 0)
            g_dump = true;
        else
            only = argv[i];
    }
    if (only) {
        /* run one test by name (debugging aid) */
        struct {
            const char *name;
            void (*fn)();
        } tests[] = {{"clean", test_clean_bs},     {"impairments", test_impairments}, {"blocks", test_block_sizes},
                     {"modes", test_modes},        {"encrypted", test_encrypted},     {"late", test_late_entry},
                     {"two", test_two_calls},      {"disc", test_discontinuity},      {"reset", test_reset_and_memory},
                     {"snr", test_snr},            {"nosignal", test_no_signal}};
        for (auto &t : tests)
            if (strcmp(t.name, only) == 0)
                t.fn();
        printf("test_rx %s: %d checks, %d failed\n", only, g_checks, g_fail);
        return g_fail ? 1 : 0;
    }
    test_clean_bs();
    test_impairments();
    test_block_sizes();
    test_modes();
    test_encrypted();
    test_late_entry();
    test_two_calls();
    test_discontinuity();
    test_reset_and_memory();
    if (!quick) {
        test_snr();
        test_no_signal();
    }
    printf("test_rx: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
