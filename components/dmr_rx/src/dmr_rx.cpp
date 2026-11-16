/*
 * dmr_rx - TDMA slot tracking, burst / PDU handling and the public API.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include <math.h>
#include <new>
#include <stdlib.h>
#include <string.h>

#include "dmr_internal.h"
#include "dmr_data.h"

#include "CRC.h"
#include "RS129.h"

namespace dmrrx {

/* ------------------------------------------------------------------------ */
/* Tunables                                                                 */
/* ------------------------------------------------------------------------ */

static const float kRho2New       = 0.80f; /* sync quality to open a new slot */
static const tq_t kMatchTol       = 12 * kOne;  /* sync vs. prediction         */
static const tq_t kNearTol        = 300 * kOne; /* overlapping burst           */
static const unsigned kSyncErrMax = 8;  /* direct sync check at predicted time */
static const unsigned kVoiceStrong = 10; /* AMBE Golay errors, 3 frames        */
static const unsigned kVoicePlausible = 12;
static const uint8_t kLossVoice   = 6;  /* consecutive unusable bursts         */
static const uint8_t kLossData    = 4;
static const tq_t kPeriodNominal  = (tq_t)kPeriod * kOne;
static const tq_t kPeriodMaxDev   = kPeriodNominal * 3 / 10000; /* 300 ppm     */

/* burst symbols after the last sync symbol + interpolation/refinement margin */
static const int64_t kDueSamples = (kBurstSyms - 1 - kSyncEndSym) * kSps + 12 + 6;

/* ------------------------------------------------------------------------ */
/* Burst evaluation helpers                                                 */
/* ------------------------------------------------------------------------ */

unsigned sync_errors(const uint8_t *w, uint8_t family, bool data)
{
    const uint8_t *p = kVoiceSync[family];
    const uint8_t x  = data ? 0xAAu : 0x00u;
    unsigned n       = 0;
    n += popcount8((unsigned)(w[13] ^ ((p[0] ^ x) >> 4)) & 0x0Fu);
    for (int i = 0; i < 5; i++) {
        const unsigned v = ((unsigned)((p[i] ^ x) << 4) | (unsigned)((p[i + 1] ^ x) >> 4)) & 0xFFu;
        n += popcount8(w[14 + i] ^ v);
    }
    n += popcount8((unsigned)(w[19] ^ ((p[5] ^ x) << 4)) & 0xF0u);
    return n;
}

static bool type_has_bptc(uint8_t t)
{
    switch (t) {
    case DMR_DT_PI_HEADER:
    case DMR_DT_VOICE_LC_HEADER:
    case DMR_DT_TERMINATOR_LC:
    case DMR_DT_CSBK:
    case DMR_DT_MBC_HEADER:
    case DMR_DT_MBC_CONTINUATION:
    case DMR_DT_DATA_HEADER:
    case DMR_DT_RATE_12_DATA:
    case DMR_DT_IDLE:
    case DMR_DT_UNIFIED_SINGLE:
        return true;
    default:
        return false;
    }
}

/* CRC mask of the data types that carry a CRC-CCITT (0 = none). */
static uint8_t type_crc_mask(uint8_t t)
{
    switch (t) {
    case DMR_DT_PI_HEADER: return 0x69;
    case DMR_DT_CSBK: return 0xA5;
    case DMR_DT_MBC_HEADER: return 0xAA;
    case DMR_DT_DATA_HEADER: return 0xCC;
    default: return 0;
    }
}

void eval_data(CBPTC19696 &bptc, const uint8_t *w, DataEval &e)
{
    e.st        = slot_type_decode(w);
    e.have_bptc = false;
    e.crc_ok    = false;
    e.strong    = false;
    memset(&e.bp, 0, sizeof(e.bp));
    if (!e.st.ok)
        return;
    const uint8_t t = e.st.data_type;
    if (!type_has_bptc(t)) {
        /* rate 3/4 and rate 1 data: nothing but the slot type to go by */
        e.strong = e.st.errs == 0 && t <= DMR_DT_UNIFIED_SINGLE;
        return;
    }
    bptc_decode(bptc, w, e.bp);
    e.have_bptc = true;

    uint8_t d[12];
    memcpy(d, e.bp.data, 12);
    if (t == DMR_DT_VOICE_LC_HEADER || t == DMR_DT_TERMINATOR_LC) {
        const uint8_t m = (t == DMR_DT_VOICE_LC_HEADER) ? 0x96 : 0x99;
        d[9] ^= m;
        d[10] ^= m;
        d[11] ^= m;
        e.crc_ok = CRS129::check(d);
        e.strong = e.crc_ok;
    } else if (type_crc_mask(t) != 0) {
        const uint8_t m = type_crc_mask(t);
        d[10] ^= m;
        d[11] ^= m;
        e.crc_ok = CCRC::checkCCITT162(d, 12);
        e.strong = e.crc_ok;
    } else {
        e.strong = e.st.errs <= 1 && e.bp.errs <= 6;
    }
}

void eval_voice(const uint8_t *w, VoiceEval &e)
{
    memcpy(e.frame[0], w, 9);
    memcpy(e.frame[2], w + 24, 9);
    memset(e.frame[1], 0, 9);
    for (unsigned i = 0; i < 36; i++) {
        put_bit(e.frame[1], i, get_bit(w, 72 + i));
        put_bit(e.frame[1], 36 + i, get_bit(w, 156 + i));
    }
    e.errs = 0;
    for (int f = 0; f < 3; f++) {
        ambe_frame_decode(e.frame[f], e.fr[f]);
        e.errs += e.fr[f].errs;
    }
    e.strong = e.errs <= kVoiceStrong;
}

/* ------------------------------------------------------------------------ */
/* Talker alias (TS 102 361-2 7.1.1.4 / 7.2.18..21)                          */
/* ------------------------------------------------------------------------ */

static size_t utf8_put(char *out, size_t pos, size_t cap, uint32_t c)
{
    if (c < 0x20u || c == 0x7Fu)
        c = ' ';
    if (c < 0x80u) {
        if (pos + 1 >= cap)
            return 0;
        out[pos] = (char)c;
        return 1;
    }
    if (c < 0x800u) {
        if (pos + 2 >= cap)
            return 0;
        out[pos]     = (char)(0xC0u | (c >> 6));
        out[pos + 1] = (char)(0x80u | (c & 0x3Fu));
        return 2;
    }
    if (pos + 3 >= cap)
        return 0;
    out[pos]     = (char)(0xE0u | (c >> 12));
    out[pos + 1] = (char)(0x80u | ((c >> 6) & 0x3Fu));
    out[pos + 2] = (char)(0x80u | (c & 0x3Fu));
    return 3;
}

bool talker_alias_decode(const TalkerAlias &ta, char out[64], uint8_t &complete)
{
    out[0]   = 0;
    complete = 0;
    if (!(ta.have & 1u))
        return false;
    unsigned nblk = 1;
    while (nblk < 4 && (ta.have & (1u << nblk)))
        nblk++;
    const unsigned format = ta.buf[0] >> 6;
    const unsigned len    = (ta.buf[0] >> 1) & 0x1Fu;
    const unsigned avail_bytes = nblk * 7u;
    size_t pos    = 0;
    unsigned done = 0;

    if (format == 0) { /* 7 bit characters, packed right after the length */
        for (unsigned i = 0; i < len; i++) {
            const unsigned b0 = 7u + 7u * i;
            if (b0 + 7u > avail_bytes * 8u)
                break;
            unsigned c = 0;
            for (unsigned k = 0; k < 7; k++)
                c = (c << 1) | get_bit(ta.buf, b0 + k);
            const size_t w = utf8_put(out, pos, 64, c);
            if (!w)
                break;
            pos += w;
            done++;
        }
    } else if (format == 1 || format == 2) { /* ISO 8 bit, UTF-8 */
        for (unsigned i = 0; i < len; i++) {
            if (1u + i >= avail_bytes)
                break;
            const unsigned c = ta.buf[1 + i];
            size_t w;
            if (format == 2 && c >= 0x80u) { /* pass UTF-8 sequences through */
                if (pos + 1 >= 64)
                    break;
                out[pos] = (char)c;
                w        = 1;
            } else {
                w = utf8_put(out, pos, 64, c);
            }
            if (!w)
                break;
            pos += w;
            done++;
        }
        /* do not leave a truncated UTF-8 sequence at the end */
        if (format == 2 && done < len) {
            while (pos > 0 && ((uint8_t)out[pos - 1] & 0xC0u) == 0x80u)
                pos--;
            if (pos > 0 && ((uint8_t)out[pos - 1] & 0x80u))
                pos--;
        }
    } else { /* UTF-16BE */
        for (unsigned i = 0; i < len; i++) {
            if (2u + 2u * i >= avail_bytes)
                break;
            uint32_t c = ((uint32_t)ta.buf[1 + 2 * i] << 8) | ta.buf[2 + 2 * i];
            if (c >= 0xD800u && c <= 0xDFFFu)
                c = '?';
            const size_t w = utf8_put(out, pos, 64, c);
            if (!w)
                break;
            pos += w;
            done++;
        }
    }
    out[pos] = 0;
    complete = (done == len) ? 1 : 0;
    return true;
}

/* ------------------------------------------------------------------------ */
/* Receiver                                                                 */
/* ------------------------------------------------------------------------ */

void Rx::init(const dmr_rx_config_t *c)
{
    if (c)
        cfg = *c;
    else
        dmr_rx_config_default(&cfg);
    dem.init();
    status_gen.store(0, std::memory_order_relaxed);
    memset(status, 0, sizeof(status));
    reset();
}

void Rx::reset()
{
    dem.reset();
    for (int i = 0; i < 2; i++) {
        slot[i] = Slot{};
        emb[i].reset();
        dasm[i].active = false;
        tot[i] = Totals{};
#ifdef DMR_RX_WITH_VOICE
        if (voc[i])
            vocoder_init(voc[i]);
#endif
    }
    period     = kPeriodNominal;
    inverted   = cfg.polarity == DMR_POLARITY_INVERTED;
    pol_locked = cfg.polarity != DMR_POLARITY_AUTO;
    pol_votes  = 0;
    pol_contra = 0;
    slc        = ShortLcAsm{};
    samples_in = 0;
    sync_count = sync_loss_count = 0;
    last_g = last_d = 0.0f;
    q                = dmr_burst_quality_t{};
    cur_sample_index = 0;
    publish_status();
}

void Rx::publish_status()
{
    const uint32_t g    = status_gen.load(std::memory_order_relaxed);
    dmr_rx_status_t &st = status[(g + 1u) & 1u];
    memset(&st, 0, sizeof(st));
    st.inverted        = inverted;
    st.polarity_locked = pol_locked;
    st.dc_offset       = dem.dc_slow + last_d;
    st.deviation       = last_g;
    st.clock_ppm       = (float)(period - kPeriodNominal) * (1.0e6f / (float)kPeriodNominal);
    st.samples         = samples_in;
    st.sync_count      = sync_count;
    st.sync_loss_count = sync_loss_count;
    for (int i = 0; i < 2; i++) {
        dmr_slot_status_t &o = st.slot[i];
        o.color_code         = 0xFF;
        o.last_data_type     = 0xFF;
        o.last_sync_errs     = 0xFF;
        o.voice_bursts       = tot[i].voice_bursts;
        o.data_bursts        = tot[i].data_bursts;
        o.missed_bursts      = tot[i].missed_bursts;
        o.fec_errs           = tot[i].fec_errs;
        o.fec_bits           = tot[i].fec_bits;
    }
    for (int i = 0; i < 2; i++) {
        const Slot &s = slot[i];
        if (!s.active || !s.confirmed)
            continue;
        st.synced            = 1;
        dmr_slot_status_t &o = st.slot[s.label & 1u];
        o.active             = 1;
        o.slot_known         = s.label_known;
        o.state              = s.state;
        o.sync_type          = s.sync_type;
        o.color_code         = s.cc;
        o.last_data_type     = s.last_data_type;
        o.call_active        = s.call.active;
        o.ids_valid          = s.call.ids_valid;
        o.group              = s.call.group;
        o.encrypted          = s.call.encrypted;
        o.emergency          = s.call.emergency;
        o.flco               = s.call.flco;
        o.fid                = s.call.fid;
        o.service_options    = s.call.svc;
        o.last_sync_errs     = s.last_sync_errs;
        o.dst_id             = s.call.dst;
        o.src_id             = s.call.src;
        memcpy(o.talker_alias, s.ta.text, sizeof(o.talker_alias));
        o.ber = s.ber;
        o.evm = s.evm;
        float ql = 1.0f - (s.evm - 0.04f) * (1.0f / 0.26f);
        if (ql > 1.0f)
            ql = 1.0f;
        if (ql < 0.0f)
            ql = 0.0f;
        o.quality = (uint8_t)(ql * 100.0f + 0.5f);
    }
    status_gen.store(g + 1u, std::memory_order_release);
}

void Rx::emit(Slot &s, dmr_event_t &ev, uint8_t type)
{
    if (!cfg.event_cb)
        return;
    ev.type         = type;
    ev.slot         = (uint8_t)(s.label + 1u);
    ev.slot_known   = s.label_known;
    ev.sync_type    = s.sync_type;
    ev.color_code   = s.cc;
    ev.inverted     = inverted;
    ev.sample_index = cur_sample_index;
    ev.quality      = q;
    publish_status();
    cfg.event_cb(cfg.event_user, &ev);
}

/* ------------------------------------------------------------------------ */
/* Slot management                                                          */
/* ------------------------------------------------------------------------ */

void Rx::slot_init(Slot &s, const SyncDet &det)
{
    s          = Slot{};
    s.active   = true;
    s.family   = det.family;
    s.t_next   = det.t_end;
    s.hit      = true;
    s.det      = det;
    s.g        = det.g3;
    s.d        = det.d;
    s.cc       = 0xFF;
    s.last_data_type = 0xFF;
    s.last_sync_errs = 0xFF;
    emb[&s - slot].reset();
    dasm[&s - slot].active = false;
}

void Rx::slot_drop(Slot &s, bool lost)
{
    if (lost && s.confirmed) {
        if (s.call.active)
            call_end(s);
        data_flush(s); /* what there is of a data packet */
        dmr_event_t ev;
        memset(&ev, 0, sizeof(ev));
        s.state = DMR_SLOT_IDLE;
        sync_loss_count++;
        const uint8_t label = s.label;
        const bool known    = s.label_known;
        s.active            = false;
        /* event is emitted after the slot is marked inactive so that the
         * status seen from the callback is consistent */
        Slot tmp        = s;
        tmp.label       = label;
        tmp.label_known = known;
        emit(tmp, ev, DMR_EVT_SYNC_LOST);
    }
    s.active                = false;
    dasm[&s - slot].active = false;
    slc.n                   = 0;
}

/* The other slot turned out to have the label of this one. */
void Rx::relabel_other(Slot &o)
{
    o.label ^= 1u;
    if (o.tc_votes != 0 && (o.tc_votes > 0 ? 1u : 0u) != o.label) {
        /* what it collected so far is void */
        o.tc_votes    = 0;
        o.label_known = false;
    }
    o.tc_contra = 0;
    o.tc_clean  = false;
}

void Rx::assign_label(Slot &s)
{
    Slot &o          = slot[(&s - slot) ^ 1];
    const bool other = o.active && o.confirmed;
    switch (s.family) {
    case kFamDM1:
        s.label       = 0;
        s.label_known = true;
        break;
    case kFamDM2:
        s.label       = 1;
        s.label_known = true;
        break;
    case kFamBS:
        /* one error free TACT, or two corrected ones that agree */
        if (s.tc_votes >= 2 || s.tc_votes <= -2) {
            s.label       = s.tc_votes > 0 ? 1 : 0;
            s.label_known = true;
            break;
        }
        /* fall through */
    default:
        s.label_known = false;
        s.label       = other ? (uint8_t)(o.label ^ 1u) : 0;
        break;
    }
    if (other && o.label == s.label) {
        const int av = s.tc_votes < 0 ? -s.tc_votes : s.tc_votes;
        const int ov = o.tc_votes < 0 ? -o.tc_votes : o.tc_votes;
        /* both claim the same timeslot. The label of the other one stands
         * if its latest TACT was received without error and said so. */
        if (s.label_known && (!o.label_known || (o.family == kFamBS && s.family == kFamBS && (av > ov || !o.tc_clean)))) {
            /* a call never changes its slot number: close it first */
            if (o.call.active)
                call_end(o);
            relabel_other(o);
        } else {
            s.label ^= 1u;
        }
    }
}

void Rx::confirm(Slot &s)
{
    s.confirmed = true;
    s.misses    = 0;
    assign_label(s);
    dmr_event_t ev;
    memset(&ev, 0, sizeof(ev));
    emit(s, ev, DMR_EVT_SYNC_ACQUIRED);
}

void Rx::on_sync(const SyncDet &det)
{
    Slot *near = nullptr;
    for (Slot &s : slot) {
        if (!s.active)
            continue;
        tq_t diff = det.t_end - s.t_next;
        if (diff < 0)
            diff = -diff;
        if (diff <= kMatchTol) {
            if (!s.confirmed && det.rho2 < kRho2New)
                return;
            if (!s.hit || det.rho2 > s.det.rho2) {
                s.hit = true;
                s.det = det;
            }
            return;
        }
        if (diff < kNearTol)
            near = &s;
    }
    if (det.rho2 < kRho2New)
        return;
    if (near) {
        /* overlaps a burst we are tracking: only take over a slot that is
         * not delivering. If it is, remember the sync: should the tracked
         * burst then turn out to be unusable (the sample stream had a gap,
         * or another station started), process_burst() re-times the slot
         * to it straight away. */
        if (near->confirmed && near->misses == 0) {
            if (!near->has_pend || det.rho2 > near->pend.rho2) {
                near->has_pend = true;
                near->pend     = det;
            }
            return;
        }
        slot_drop(*near, true);
        slot_init(*near, det);
        return;
    }
    for (Slot &s : slot) {
        if (!s.active) {
            slot_init(s, det);
            return;
        }
    }
    Slot *v = nullptr;
    for (Slot &s : slot) {
        if (!s.confirmed && (!v || s.unconf_age > v->unconf_age))
            v = &s;
    }
    if (!v) {
        for (Slot &s : slot) {
            if (s.misses >= 2 && (!v || s.misses > v->misses))
                v = &s;
        }
    }
    if (v) {
        slot_drop(*v, true);
        slot_init(*v, det);
    }
}

/* ------------------------------------------------------------------------ */
/* Call state                                                               */
/* ------------------------------------------------------------------------ */

static void fill_call_event(const Call &c, dmr_event_t &ev)
{
    ev.u.call.group        = c.group;
    ev.u.call.encrypted    = c.encrypted;
    ev.u.call.emergency    = c.emergency;
    ev.u.call.late_entry   = c.late_entry;
    ev.u.call.ids_valid    = c.ids_valid;
    ev.u.call.dst_id       = c.dst;
    ev.u.call.src_id       = c.src;
    ev.u.call.voice_bursts = c.voice_bursts;
}

void Rx::call_start(Slot &s, bool late)
{
    s.call.active       = true;
    s.call.late_entry   = late;
    s.call.voice_bursts = 0;
    s.call.cand_n       = 0;
    s.call.data_in_call = 0;
    s.ta                = TalkerAlias{};
    s.have_last_emb     = false;
#ifdef DMR_RX_WITH_VOICE
    if (voc[s.label & 1u])
        vocoder_init(voc[s.label & 1u]);
#endif
    dmr_event_t ev;
    memset(&ev, 0, sizeof(ev));
    fill_call_event(s.call, ev);
    emit(s, ev, DMR_EVT_CALL_START);
}

void Rx::call_end(Slot &s)
{
    dmr_event_t ev;
    memset(&ev, 0, sizeof(ev));
    fill_call_event(s.call, ev);
    s.call          = Call{};
    s.pending_enc   = false;
    s.ta            = TalkerAlias{};
    s.have_last_emb = false;
    emit(s, ev, DMR_EVT_CALL_END);
}

void Rx::call_update(Slot &s)
{
    dmr_event_t ev;
    memset(&ev, 0, sizeof(ev));
    fill_call_event(s.call, ev);
    emit(s, ev, DMR_EVT_CALL_UPDATE);
}

void Rx::handle_talker_alias(Slot &s, uint8_t block, const uint8_t *data7)
{
    if (block == 0 && (s.ta.have & 1u) && memcmp(s.ta.buf, data7, 7) != 0)
        s.ta.have = 0; /* a different alias starts */
    memcpy(s.ta.buf + 7u * block, data7, 7);
    s.ta.have |= (uint8_t)(1u << block);
    char text[64];
    uint8_t complete;
    if (!talker_alias_decode(s.ta, text, complete))
        return;
    if (strcmp(text, s.ta.text) == 0 && complete == s.ta.complete)
        return;
    memcpy(s.ta.text, text, sizeof(text));
    s.ta.complete = complete;
    dmr_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.u.talker_alias.complete = complete;
    memcpy(ev.u.talker_alias.text, text, sizeof(text));
    emit(s, ev, DMR_EVT_TALKER_ALIAS);
}

void Rx::handle_lc(Slot &s, const uint8_t raw[9], uint8_t source)
{
    const uint8_t flco = raw[0] & 0x3Fu;
    const uint8_t fid  = raw[1];
    const uint8_t svc  = raw[2];
    /* Terminator Data LC has no service options: the
     * addresses start one octet earlier and the last octet carries the
     * GI / A / F / S / N(S) flags. */
    const unsigned ao  = (fid == 0 && flco == DMR_FLCO_TERMINATOR_DATA) ? 2u : 3u;
    const uint32_t dst = ((uint32_t)raw[ao] << 16) | ((uint32_t)raw[ao + 1] << 8) | raw[ao + 2];
    const uint32_t src = ((uint32_t)raw[ao + 3] << 16) | ((uint32_t)raw[ao + 4] << 8) | raw[ao + 5];
    /* group / unit-to-unit voice channel user; Motorola Capacity Plus uses
     * FLCO 4 / 7 with FID 0x10 for the same layout */
    const bool cap_plus = fid == 0x10 && (flco == 0x04 || flco == 0x07);
    const bool voice_lc = flco == DMR_FLCO_GROUP_VOICE || flco == DMR_FLCO_UNIT_TO_UNIT || cap_plus;
    const bool group    = flco == DMR_FLCO_GROUP_VOICE || (fid == 0x10 && flco == 0x04);

    bool report = true;
    if (source == DMR_LC_SRC_EMBEDDED) {
        if (s.have_last_emb && memcmp(s.last_emb, raw, 9) == 0)
            report = false;
        memcpy(s.last_emb, raw, 9);
        s.have_last_emb = true;
    }
    if (report) {
        dmr_event_t ev;
        memset(&ev, 0, sizeof(ev));
        dmr_evt_lc_t &l   = ev.u.lc;
        l.source          = source;
        l.pf              = raw[0] >> 7;
        l.flco            = flco;
        l.fid             = fid;
        l.service_options = voice_lc ? svc : 0;
        l.is_group        = group;
        l.encrypted       = voice_lc && (svc & DMR_SVC_PRIVACY) ? 1 : 0;
        l.emergency       = voice_lc && (svc & DMR_SVC_EMERGENCY) ? 1 : 0;
        l.dst_id          = dst;
        l.src_id          = src;
        memcpy(l.raw, raw, 9);
        emit(s, ev, DMR_EVT_LC);
    }

    if (voice_lc) {
        Call &c        = s.call;
        const bool enc = (svc & DMR_SVC_PRIVACY) != 0 || s.pending_enc;
        const bool emg = (svc & DMR_SVC_EMERGENCY) != 0;
        const bool same = c.ids_valid && c.dst == dst && c.src == src;
        auto set = [&]() {
            c.ids_valid = true;
            c.group     = group;
            c.encrypted = enc || c.encrypted;
            c.emergency = emg;
            c.flco      = flco;
            c.fid       = fid;
            c.svc       = svc;
            c.dst       = dst;
            c.src       = src;
            c.cand_n    = 0;
        };
        if (source == DMR_LC_SRC_TERMINATOR) {
            if (c.active && !c.ids_valid)
                set();
            return;
        }
        if (!c.active) {
            c = Call{};
            set();
            call_start(s, source == DMR_LC_SRC_EMBEDDED);
        } else if (!c.ids_valid) {
            set();
            call_update(s);
        } else if (same) {
            c.cand_n = 0;
            if (c.svc != svc || (enc && !c.encrypted)) {
                set();
                call_update(s);
            }
        } else if (source == DMR_LC_SRC_VOICE_HEADER) {
            call_end(s);
            set();
            call_start(s, false);
        } else {
            /* embedded LC with other ids: believe it when seen twice */
            if (c.cand_n > 0 && c.cand_dst == dst && c.cand_src == src)
                c.cand_n++;
            else {
                c.cand_dst = dst;
                c.cand_src = src;
                c.cand_n   = 1;
            }
            if (c.cand_n >= 2) {
                call_end(s);
                set();
                call_start(s, true);
            }
        }
    } else if (fid == 0 && flco >= DMR_FLCO_TALKER_ALIAS0 && flco <= DMR_FLCO_TALKER_ALIAS3) {
        handle_talker_alias(s, (uint8_t)(flco - DMR_FLCO_TALKER_ALIAS0), raw + 2);
    }
}

/* ------------------------------------------------------------------------ */
/* Data bursts                                                              */
/* ------------------------------------------------------------------------ */

static bool csbk_has_ids(uint8_t op, uint8_t fid)
{
    if (fid != 0)
        return false;
    switch (op) {
    case 0x04: case 0x05: case 0x26: case 0x38: case 0x3D:
    case 0x30: case 0x31: case 0x32: case 0x33: case 0x34: case 0x35: case 0x36:
        return true;
    default:
        return false;
    }
}

void Rx::handle_data(Slot &s, const uint8_t *w, const DataEval &de, uint8_t se)
{
    s.sync_type      = (uint8_t)(DMR_SYNC_BS_DATA + 2 * s.family);
    s.last_sync_errs = se;
    s.state          = DMR_SLOT_DATA;
    s.vindex         = 0;
    tot[s.label & 1u].data_bursts++;
    q.sync_errs = se;

    const uint8_t t  = de.st.data_type;
    const uint8_t cm = type_crc_mask(t);
    const bool has_check = cm != 0 || t == DMR_DT_VOICE_LC_HEADER || t == DMR_DT_TERMINATOR_LC;
    bool payload_ok      = false;
    if (de.st.ok) {
        s.cc             = de.st.cc;
        s.last_data_type = t;
        q.fec_errs       = (uint8_t)(q.fec_errs + de.st.errs);
        q.fec_bits       = (uint16_t)(q.fec_bits + 20);
        if (de.have_bptc) {
            payload_ok = has_check ? de.crc_ok : de.bp.errs <= kBptcPlausibleErrs;
            if (payload_ok) {
                q.fec_errs = (uint8_t)(q.fec_errs + de.bp.errs);
                q.fec_bits = (uint16_t)(q.fec_bits + 196);
            }
        }
    }
    {
        dmr_event_t ev;
        memset(&ev, 0, sizeof(ev));
        ev.u.data.data_type    = t;
        ev.u.data.slot_type_ok = de.st.ok;
        if (de.have_bptc) {
            ev.u.data.payload_len = 12;
            memcpy(ev.u.data.payload, de.bp.data, 12);
        } else if (de.st.ok && t == DMR_DT_RATE_34_DATA) {
            Trellis tr;
            trellis34_decode(w, tr);
            payload_ok = tr.errs <= kTrellisPlausibleErrs;
            if (payload_ok) {
                q.fec_errs = (uint8_t)(q.fec_errs + tr.errs);
                q.fec_bits = (uint16_t)(q.fec_bits + 196);
            }
            ev.u.data.payload_len = 18;
            memcpy(ev.u.data.payload, tr.data, 18);
        } else if (de.st.ok && t == DMR_DT_RATE_1_DATA) {
            payload_ok            = true;
            ev.u.data.payload_len = 24;
            rate1_decode(w, ev.u.data.payload);
        }
        ev.u.data.payload_ok = payload_ok;
        emit(s, ev, DMR_EVT_DATA_BURST);
    }

    if (!de.st.ok) {
        if (s.call.active && ++s.call.data_in_call >= 3)
            call_end(s);
        /* within a data packet: a block whose slot type was not readable */
        data_block(s, w, 0xFF, de);
        return;
    }
    if (t != DMR_DT_DATA_HEADER && t != DMR_DT_RATE_12_DATA && t != DMR_DT_RATE_34_DATA && t != DMR_DT_RATE_1_DATA)
        data_flush(s); /* the packet (if there was one) ends here */
    const uint8_t *d = de.bp.data;
    switch (t) {
    case DMR_DT_VOICE_LC_HEADER:
        emb[&s - slot].reset();
        if (de.crc_ok)
            handle_lc(s, d, DMR_LC_SRC_VOICE_HEADER);
        break;
    case DMR_DT_TERMINATOR_LC:
        if (de.crc_ok)
            handle_lc(s, d, DMR_LC_SRC_TERMINATOR);
        if (s.call.active)
            call_end(s);
        s.pending_enc = false;
        break;
    case DMR_DT_PI_HEADER:
        if (de.crc_ok) {
            dmr_event_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.u.pi.algorithm_id = d[0];
            ev.u.pi.fid          = d[1];
            ev.u.pi.key_id       = d[2];
            ev.u.pi.mi  = ((uint32_t)d[3] << 24) | ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 8) | d[6];
            ev.u.pi.dst_id = ((uint32_t)d[7] << 16) | ((uint32_t)d[8] << 8) | d[9];
            memcpy(ev.u.pi.raw, d, 10);
            emit(s, ev, DMR_EVT_PI_HEADER);
            s.pending_enc = true;
            if (s.call.active && !s.call.encrypted) {
                s.call.encrypted = true;
                call_update(s);
            }
        }
        break;
    case DMR_DT_CSBK:
    case DMR_DT_MBC_HEADER:
        if (s.call.active)
            call_end(s);
        if (de.crc_ok) {
            dmr_event_t ev;
            memset(&ev, 0, sizeof(ev));
            dmr_evt_csbk_t &c = ev.u.csbk;
            c.last_block      = d[0] >> 7;
            c.pf              = (d[0] >> 6) & 1u;
            c.opcode          = d[0] & 0x3Fu;
            c.fid             = d[1];
            c.is_mbc          = t == DMR_DT_MBC_HEADER;
            c.has_ids         = csbk_has_ids(c.opcode, c.fid);
            c.dst_id          = ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 8) | d[6];
            c.src_id          = ((uint32_t)d[7] << 16) | ((uint32_t)d[8] << 8) | d[9];
            memcpy(c.raw, d, 10);
            c.name = dmr_rx_csbk_name(c.opcode, c.fid);
            emit(s, ev, DMR_EVT_CSBK);
        }
        break;
    case DMR_DT_DATA_HEADER:
        if (s.call.active)
            call_end(s);
        if (de.crc_ok) {
            dmr_event_t ev;
            memset(&ev, 0, sizeof(ev));
            dmr_evt_data_header_t &h = ev.u.data_header;
            h.is_group           = d[0] >> 7;
            h.response_requested = (d[0] >> 6) & 1u;
            h.dpf                = d[0] & 0x0Fu;
            h.sap                = d[1] >> 4;
            h.dst_id             = ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 8) | d[4];
            h.src_id             = ((uint32_t)d[5] << 16) | ((uint32_t)d[6] << 8) | d[7];
            h.blocks_to_follow   = d[8] & 0x7Fu;
            memcpy(h.raw, d, 10);
            emit(s, ev, DMR_EVT_DATA_HEADER);
            data_header(s, d);
        } else {
            data_header_bad(s);
        }
        break;
    case DMR_DT_RATE_12_DATA:
    case DMR_DT_RATE_34_DATA:
    case DMR_DT_RATE_1_DATA:
        if (s.call.active)
            call_end(s);
        data_block(s, w, t, de);
        break;
    default:
        if (s.call.active)
            call_end(s);
        break;
    }
}

/* ------------------------------------------------------------------------ */
/* Voice bursts                                                             */
/* ------------------------------------------------------------------------ */

bool Rx::handle_voice(Slot &s, const uint8_t *w, const VoiceEval &ve, uint8_t index, bool has_sync, uint8_t se)
{
    CDMREmbeddedData &ed = emb[&s - slot];
    if (s.state != DMR_SLOT_VOICE) {
        s.state = DMR_SLOT_VOICE;
        ed.reset();
    }
    s.vindex = index;
    if (has_sync) {
        s.sync_type      = (uint8_t)(DMR_SYNC_BS_VOICE + 2 * s.family);
        s.last_sync_errs = se;
        q.sync_errs      = se;
    }
    for (int f = 0; f < 3; f++) {
        q.fec_errs = (uint8_t)(q.fec_errs + ve.fr[f].errs);
        q.fec_bits = (uint16_t)(q.fec_bits + 47);
    }

    /* Bursts B..F carry no sync word: whether there is a burst at all can
     * only be judged from the AMBE FEC and the EMB. */
    const bool frames_ok = ve.errs <= kVoicePlausible;
    Emb e;
    memset(&e, 0, sizeof(e));
    uint8_t lcss = 0;
    bool emb_cc_match = false;
    if (index > 0) {
        e = emb_decode(w);
        if (e.ok) {
            emb_cc_match = e.errs == 0 && (s.cc == 0xFF || s.cc == e.cc);
            q.fec_errs   = (uint8_t)(q.fec_errs + e.errs);
            q.fec_bits   = (uint16_t)(q.fec_bits + 16);
        }
    }
    const bool plausible = has_sync || frames_ok || emb_cc_match;
    if (!s.call.active) {
        /* late entry: not on something that is only just a burst */
        if (!(has_sync || ve.strong || emb_cc_match))
            return false;
        s.call           = Call{};
        s.call.encrypted = s.pending_enc;
        call_start(s, true);
    }
    s.call.data_in_call = 0;
    if (plausible) {
        s.call.voice_bursts++;
        tot[s.label & 1u].voice_bursts++;
    }
    if (index > 0 && plausible) {
        if (e.ok) {
            /* a colour code is only taken from an EMB without doubt */
            if (frames_ok ? (e.errs <= 1 || s.cc == 0xFF) : e.errs == 0)
                s.cc = e.cc;
            lcss = e.lcss;
        } else {
            static const uint8_t kInfer[6] = {0, 1, 3, 3, 2, 0};
            lcss = kInfer[index];
        }
        if (lcss != 0 && ed.addData(w, lcss)) {
            uint8_t raw[9];
            if (ed.getRawData(raw))
                handle_lc(s, raw, DMR_LC_SRC_EMBEDDED);
        }
    }

    {
        dmr_event_t ev;
        memset(&ev, 0, sizeof(ev));
        dmr_evt_voice_burst_t &v = ev.u.voice;
        v.burst_index = index;
        v.encrypted   = s.call.encrypted;
        v.emb_ok      = e.ok && plausible;
        v.lcss        = lcss;
        for (int f = 0; f < 3; f++) {
            memcpy(v.frame[f], ve.frame[f], 9);
            memcpy(v.ambe49[f], ve.fr[f].ambe49, 7);
            v.errs[f] = ve.fr[f].errs;
            /* nothing recognisable at the expected position (fade, end of
             * the transmission): the frames are noise, have them concealed */
            v.bad[f]  = ve.fr[f].bad || !plausible;
        }
        emit(s, ev, DMR_EVT_VOICE_BURST);
    }

#ifdef DMR_RX_WITH_VOICE
    {
        const unsigned li = s.label & 1u;
        /* Never run encrypted audio through the vocoder. On late entry wait
         * for the embedded LC (which carries the privacy bit) before the
         * first audio; give up waiting after two superframes. */
        if (cfg.pcm_cb && voc[li] && ((cfg.pcm_slot_mask >> li) & 1u) && s.call.active && !s.call.encrypted &&
            (s.call.ids_valid || s.call.voice_bursts > 12)) {
            int16_t pcm[DMR_RX_PCM_FRAME_SAMPLES];
            for (int f = 0; f < 3; f++) {
                vocoder_decode(voc[li], ve.fr[f].ambe_d, ve.fr[f].errs, ve.fr[f].bad || !plausible, cfg.pcm_gain,
                               pcm);
                cfg.pcm_cb(cfg.pcm_user, (uint8_t)(li + 1u), pcm, DMR_RX_PCM_FRAME_SAMPLES);
            }
        }
    }
#endif

    return plausible;
}

/* ------------------------------------------------------------------------ */
/* CACH: short LC                                                           */
/* ------------------------------------------------------------------------ */

void Rx::short_lc_add(Slot &s, uint8_t lcss, const uint8_t *pl, tq_t t)
{
    if (lcss == 1) {
        slc.n = 0;
    } else if ((lcss == 3 && (slc.n == 1 || slc.n == 2)) || (lcss == 2 && slc.n == 3)) {
        tq_t gap = t - slc.t_last - kPeriodNominal / 2;
        if (gap < 0)
            gap = -gap;
        if (gap > 40 * kOne) {
            slc.n = 0;
            return;
        }
    } else {
        slc.n = 0;
        return;
    }
    memcpy(slc.bits + 17u * slc.n, pl, 17);
    slc.n++;
    slc.t_last = t;
    if (slc.n < 4)
        return;
    slc.n = 0;

    uint8_t in[9], out[5];
    memset(in, 0, sizeof(in));
    for (unsigned i = 0; i < 68; i++)
        put_bit(in, i, slc.bits[i]);
    if (!shortlc.decode(in, out))
        return;
    if (CCRC::crc8(out, 4) != out[4])
        return;
    if (slc.have_last && memcmp(slc.last, out, 4) == 0)
        return;
    memcpy(slc.last, out, 5);
    slc.have_last = true;

    dmr_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.u.short_lc.slco    = out[0] & 0x0Fu;
    ev.u.short_lc.data[0] = out[1];
    ev.u.short_lc.data[1] = out[2];
    ev.u.short_lc.data[2] = out[3];
    ev.u.short_lc.name    = dmr_rx_slco_name(ev.u.short_lc.slco);
    const dmr_burst_quality_t keep = q;
    q.sync_errs = 0xFF;
    q.fec_errs  = 0;
    q.fec_bits  = 0;
    emit(s, ev, DMR_EVT_SHORT_LC);
    q = keep;
}

/* ------------------------------------------------------------------------ */
/* Burst processing                                                         */
/* ------------------------------------------------------------------------ */

void Rx::quality_update(Slot &s, bool miss)
{
    Totals &t = tot[s.label & 1u];
    float ber;
    if (q.fec_bits)
        ber = (float)q.fec_errs / (float)q.fec_bits;
    else
        ber = miss ? 0.1f : s.ber;
    if (!s.q_init) {
        s.ber    = ber;
        s.evm    = q.evm;
        s.q_init = true;
    } else {
        s.ber += (ber - s.ber) * 0.125f;
        s.evm += (q.evm - s.evm) * 0.125f;
    }
    t.fec_errs += q.fec_errs;
    t.fec_bits += q.fec_bits;
    if (miss)
        t.missed_bursts++;
}

void Rx::process_burst(Slot &s)
{
    Burst b;
    const bool hit = s.hit;
    const tq_t t   = hit ? s.det.t_end : s.t_next;
    if (hit)
        dem.slice(t, s.det.family, s.det.sign, s.g, s.d, b);
    else
        dem.slice(t, -1, 0, s.g, s.d, b);
    s.hit          = false;
    const tq_t tau = (tq_t)(b.tau * 65536.0f);

    if (hit && s.det.family != s.family) {
        /* another kind of station took over this timing */
        if (s.confirmed && s.call.active)
            call_end(s);
        s.family   = s.det.family;
        s.tc_votes = 0;
        s.state    = DMR_SLOT_IDLE;
        if (s.confirmed)
            assign_label(s);
    }

    q.sync_errs = 0xFF;
    q.fec_errs  = 0;
    q.fec_bits  = 0;
    q.evm       = b.evm;
    {
        const int64_t centre = (t >> 16) - (kSyncEndSym * kSps - (kBurstSyms * kSps) / 2);
        const int64_t idx    = 2 * centre - (kFirTaps / 2);
        cur_sample_index     = idx > 0 ? (uint64_t)idx : 0;
    }

    uint8_t w[kBurstBytes];
    const uint8_t inv = inverted ? 0xAAu : 0x00u;
    for (int i = 0; i < kBurstBytes; i++)
        w[i] = (uint8_t)(b.bits[i] ^ inv);

    /* Does the burst carry a sync word, and which? */
    bool has_sync = hit;
    bool is_data  = false;
    unsigned se   = 0xFF;
    if (hit) {
        is_data = (s.det.sign > 0) == inverted;
        se      = sync_errors(w, s.family, is_data);
    } else {
        const unsigned ev = sync_errors(w, s.family, false);
        const unsigned ed = sync_errors(w, s.family, true);
        if ((ev < ed ? ev : ed) <= kSyncErrMax) {
            has_sync = true;
            is_data  = ed < ev;
            se       = is_data ? ed : ev;
        }
    }

    DataEval de;
    VoiceEval ve;
    bool have_ve = false;
    bool strong  = false;
    if (has_sync) {
        if (is_data) {
            eval_data(bptc, w, de);
            strong = de.strong;
        } else {
            eval_voice(w, ve);
            have_ve = true;
            strong  = ve.strong;
        }
        if (cfg.polarity == DMR_POLARITY_AUTO) {
            if (strong) {
                pol_contra = 0;
                if (pol_votes < 255)
                    pol_votes++;
                if (pol_votes >= 2)
                    pol_locked = true;
            } else {
                /* the same sync means the other burst type in the other
                 * polarity: test that hypothesis */
                uint8_t w2[kBurstBytes];
                for (int i = 0; i < kBurstBytes; i++)
                    w2[i] = (uint8_t)(w[i] ^ 0xAAu);
                DataEval de2;
                VoiceEval ve2;
                bool alt;
                if (is_data) {
                    eval_voice(w2, ve2);
                    alt = ve2.strong;
                } else {
                    eval_data(bptc, w2, de2);
                    alt = de2.strong;
                }
                if (alt && (!pol_locked || ++pol_contra >= 3)) {
                    inverted   = !inverted;
                    pol_locked = false;
                    pol_votes  = 1;
                    pol_contra = 0;
                    memcpy(w, w2, sizeof(w));
                    is_data = !is_data;
                    if (is_data) {
                        de = de2;
                    } else {
                        ve      = ve2;
                        have_ve = true;
                    }
                    strong = true;
                    se     = sync_errors(w, s.family, is_data);
                }
            }
        }
    }

    /* CACH (repeater downlink only): TACT gives the timeslot number */
    Tact tact = Tact();
    uint8_t cach_pl[17];
    bool have_tact = false;
    if (s.family == kFamBS) {
        const uint8_t pinv = inverted ? 0xAAu : 0x00u;
        uint8_t c[3];
        for (int i = 0; i < 3; i++)
            c[i] = (uint8_t)(b.cach[i] ^ pinv);
        tact      = cach_decode(c, cach_pl);
        have_tact = true;
    }

    /* What is this burst? */
    enum { K_MISS, K_DATA, K_VOICE } kind;
    uint8_t vidx          = 0;
    const bool voice_ctx  = s.confirmed ? (s.state == DMR_SLOT_VOICE) : s.prov_voice;
    if (has_sync) {
        kind = is_data ? K_DATA : K_VOICE;
    } else if (voice_ctx) {
        vidx = (uint8_t)((s.vindex + 1u) % 6u);
        eval_voice(w, ve);
        have_ve = true;
        kind    = K_VOICE;
    } else {
        kind = K_MISS;
    }
    (void)have_ve;

    /* the TACT of something that looks like a burst votes for the timeslot
     * number */
    bool tact_contra = false;
    if (have_tact && (has_sync || (kind == K_VOICE && ve.errs <= kVoicePlausible))) {
        int v = s.tc_votes + (tact.tc ? 1 : -1) * (tact.errs == 0 ? 2 : 1);
        if (v > 8)
            v = 8;
        if (v < -8)
            v = -8;
        s.tc_votes = (int8_t)v;
        s.tc_clean = false;
        if (tact.errs == 0 && s.confirmed) {
            if (s.label == (tact.tc ? 1 : 0)) {
                s.tc_contra = 0;
                s.tc_clean  = true;
            } else {
                tact_contra = true;
            }
        }
    } else {
        have_tact  = false;
        s.tc_clean = false;
    }

    bool plausible = false;
    if (!s.confirmed) {
        bool ok = false;
        if (has_sync) {
            /* Two syncs of the same kind confirm a slot without anything
             * decodable. A voice sync is only due every 6th burst: voice
             * syncs in a row are the data syncs of a signal with the other
             * polarity (the two patterns are each other's inverse), and so
             * is a data sync after them (the first voice burst). */
            const bool first      = s.unconf_syncs == 0;
            const bool cadence_ok = first || (is_data ? !s.prov_voice : (s.prov_voice && s.vindex == 5));
            s.unconf_syncs        = cadence_ok ? (uint8_t)(s.unconf_syncs + 1u) : (uint8_t)1u;
            s.prov_voice          = !is_data;
            s.vindex              = 0;
            ok                    = strong || s.unconf_syncs >= 2;
        } else if (kind == K_VOICE) {
            s.vindex = vidx;
            ok       = ve.strong && vidx != 0;
        }
        if (ok) {
            s.sync_type = (uint8_t)((is_data && has_sync ? DMR_SYNC_BS_DATA : DMR_SYNC_BS_VOICE) + 2 * s.family);
            if (has_sync && is_data && de.st.ok)
                s.cc = de.st.cc; /* known before the SYNC_ACQUIRED event */
            if (has_sync)
                sync_count++;
            confirm(s);
        } else {
            s.unconf_age++;
            if (s.unconf_age > (s.prov_voice ? 7 : 2)) {
                s.active = false;
                return;
            }
        }
    } else if (has_sync) {
        sync_count++;
    }

    if (s.confirmed) {
        if (have_tact) {
            Slot &o = slot[(&s - slot) ^ 1];
            /* Two error free TACTs in a row (of either slot) that contradict
             * the labels: the sample stream had a gap of an odd number of
             * slots. Swap at once. */
            if (tact_contra) {
                const bool other = o.active && o.confirmed && o.family == kFamBS;
                if (s.tc_contra < 2)
                    s.tc_contra++;
                if (s.tc_contra + (other ? o.tc_contra : 0) >= 2) {
                    s.tc_votes = (int8_t)(tact.tc ? 4 : -4);
                    if (other)
                        o.tc_votes = (int8_t)-s.tc_votes;
                }
            }
            /* follow the TACT if it persistently disagrees with the label */
            const uint8_t want = s.tc_votes > 0 ? 1 : 0;
            const int av       = s.tc_votes < 0 ? -s.tc_votes : s.tc_votes;
            if ((av >= 4 && s.label != want) || (av >= 2 && !s.label_known)) {
                s.tc_contra = 0;
                o.tc_contra = 0;
                if (s.label != want) {
                    /* a call never changes its slot number: close it first
                     * (it restarts as a late entry on the next voice burst) */
                    if (s.call.active)
                        call_end(s);
                    if (o.active && o.confirmed && o.label == want) {
                        if (o.call.active)
                            call_end(o);
                        relabel_other(o);
                    }
                }
                s.label       = want;
                s.label_known = true;
            }
        }
        switch (kind) {
        case K_DATA:
            handle_data(s, w, de, (uint8_t)se);
            plausible = true;
            break;
        case K_VOICE:
            data_flush(s);
            plausible = handle_voice(s, w, ve, vidx, has_sync, (uint8_t)se);
            break;
        default:
            data_missing(s); /* within a data packet: a block is lost */
            break;
        }
        if (have_tact && plausible)
            short_lc_add(s, tact.lcss, cach_pl, t);
        if (plausible)
            s.misses = 0;
        else
            s.misses++;
        quality_update(s, !plausible);
    }

    /* timing and levels for the next burst of this slot */
    if (hit) {
        const tq_t tb = t + tau;
        if (s.confirmed && s.periods_since_sync >= 1 && s.periods_since_sync <= 12) {
            const tq_t meas = (tb - s.t_last_sync) / s.periods_since_sync;
            tq_t err        = meas - period;
            if (err > -kOne / 2 && err < kOne / 2) {
                period += err / 8;
                if (period > kPeriodNominal + kPeriodMaxDev)
                    period = kPeriodNominal + kPeriodMaxDev;
                if (period < kPeriodNominal - kPeriodMaxDev)
                    period = kPeriodNominal - kPeriodMaxDev;
            }
        }
        s.t_last_sync        = tb;
        s.periods_since_sync = 0;
        s.t_next             = tb + period;
        s.g                  = b.g;
        s.d                  = b.d;
    } else {
        s.t_next = t + period + ((plausible || has_sync) ? tau / 2 : 0);
        if (plausible || has_sync) {
            s.g += (b.g - s.g) * 0.5f;
            s.d += (b.d - s.d) * 0.5f;
        }
    }
    if (s.periods_since_sync < 1000)
        s.periods_since_sync++;
    if (plausible || has_sync) {
        last_g = s.g;
        last_d = s.d;
    }

    /* A strong sync was seen elsewhere in this burst, and what was received
     * at the predicted position is no burst (or hardly one): the timing
     * jumped (samples were lost) or another station took over. */
    if (s.confirmed && s.has_pend && !has_sync && !(plausible && kind == K_VOICE && ve.strong)) {
        const SyncDet det = s.pend;
        slot_drop(s, true);
        slot_init(s, det);
        return;
    }
    s.has_pend = false;

    if (s.confirmed && s.misses >= (s.state == DMR_SLOT_VOICE ? kLossVoice : kLossData))
        slot_drop(s, true);
}

void Rx::input(float x)
{
    samples_in++;
    if (!dem.push(x))
        return;
    SyncDet det;
    if (dem.search(det))
        on_sync(det);
    const int64_t m = dem.n - 1;
    for (;;) {
        Slot *due = nullptr;
        for (Slot &s : slot) {
            if (s.active && m >= (s.t_next >> 16) + kDueSamples && (!due || s.t_next < due->t_next))
                due = &s;
        }
        if (!due)
            break;
        process_burst(*due);
    }
}

} // namespace dmrrx

/* ------------------------------------------------------------------------ */
/* C API                                                                    */
/* ------------------------------------------------------------------------ */

using namespace dmrrx;

static size_t align_up(size_t v)
{
    return (v + 15u) & ~(size_t)15u;
}

extern "C" {

void dmr_rx_config_default(dmr_rx_config_t *cfg)
{
    if (!cfg)
        return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->polarity      = DMR_POLARITY_AUTO;
    cfg->pcm_slot_mask = 3;
    cfg->pcm_gain      = 1.0f;
}

size_t dmr_rx_state_size(void)
{
    return align_up(sizeof(dmr_rx)) + 2 * align_up(vocoder_size());
}

int dmr_rx_has_voice(void)
{
#ifdef DMR_RX_WITH_VOICE
    return 1;
#else
    return 0;
#endif
}

dmr_rx_t *dmr_rx_create(const dmr_rx_config_t *cfg)
{
    void *mem = (cfg && cfg->alloc) ? cfg->alloc(dmr_rx_state_size()) : malloc(dmr_rx_state_size());
    if (!mem)
        return nullptr;
    dmr_rx *h = new (mem) dmr_rx;
    h->rx.voc[0] = h->rx.voc[1] = nullptr;
#ifdef DMR_RX_WITH_VOICE
    {
        uint8_t *p   = (uint8_t *)mem + align_up(sizeof(dmr_rx));
        h->rx.voc[0] = (Vocoder *)p;
        h->rx.voc[1] = (Vocoder *)(p + align_up(vocoder_size()));
    }
#endif
    h->rx.init(cfg);
    if (h->rx.cfg.pcm_gain <= 0.0f)
        h->rx.cfg.pcm_gain = 1.0f;
    return h;
}

void dmr_rx_destroy(dmr_rx_t *h)
{
    if (!h)
        return;
    void (*dealloc)(void *) = h->rx.cfg.dealloc;
    const bool custom       = h->rx.cfg.alloc != nullptr;
    h->~dmr_rx();
    if (custom) {
        if (dealloc)
            dealloc(h);
    } else {
        free(h);
    }
}

void dmr_rx_reset(dmr_rx_t *h)
{
    if (h)
        h->rx.reset();
}

void dmr_rx_process_fm(dmr_rx_t *h, const float *x, size_t n)
{
    if (!h || !x)
        return;
    Rx &rx = h->rx;
    for (size_t i = 0; i < n; i++)
        rx.input(x[i]);
    rx.publish_status();
}

void dmr_rx_process_fm_s16(dmr_rx_t *h, const int16_t *x, size_t n)
{
    if (!h || !x)
        return;
    Rx &rx = h->rx;
    for (size_t i = 0; i < n; i++)
        rx.input((float)x[i] * (1.0f / 32768.0f));
    rx.publish_status();
}

void dmr_rx_get_status(const dmr_rx_t *h, dmr_rx_status_t *out)
{
    if (!h || !out)
        return;
    const Rx &rx = h->rx;
    for (;;) {
        const uint32_t g1 = rx.status_gen.load(std::memory_order_acquire);
        memcpy(out, &rx.status[g1 & 1u], sizeof(*out));
        const uint32_t g2 = rx.status_gen.load(std::memory_order_acquire);
        /* the writer only touches the other buffer until it has published
         * twice */
        if ((uint32_t)(g2 - g1) < 2u)
            break;
    }
}

void dmr_rx_set_pcm_slot_mask(dmr_rx_t *h, uint8_t mask)
{
    if (h)
        h->rx.cfg.pcm_slot_mask = mask;
}

} // extern "C"
