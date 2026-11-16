/*
 * dmr_rx_file - decode a 48 kHz FM discriminator recording with dmr_rx and
 * print the events.
 *
 * Input: RIFF/WAV (16 bit PCM, first channel is used) or headerless S16LE.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "dmr_rx.h"

namespace {

struct Options {
    const char *path  = nullptr;
    int verbose       = 1; /* 0 summary only, 1 signalling, 2 every burst */
    size_t block      = 960;
    int polarity      = DMR_POLARITY_AUTO;
    const char *pcm   = nullptr;
    const char *ambe  = nullptr;
    unsigned slotmask = 3;
    float gain        = 1.0f;
    bool negate       = false;
};

struct CallRec {
    int slot;
    uint32_t dst, src;
    bool group, enc, late, ids;
    uint32_t bursts;
    double t0, t1;
};

struct Ctx {
    Options opt;
    FILE *pcm  = nullptr;
    FILE *ambe = nullptr;
    size_t pcm_samples = 0;
    uint64_t ev_count[16] = {};
    uint64_t voice[2] = {}, data[2] = {}, voice_frames_bad[2] = {}, voice_errs[2] = {};
    uint64_t data_ok[2] = {};
    std::map<int, uint64_t> cc, dtype[2], csbk, synctype;
    std::map<std::string, uint64_t> lcs;
    std::vector<CallRec> calls;
    CallRec cur[2] = {};
    bool in_call[2] = {};
    std::vector<std::string> short_lcs, aliases;
    uint64_t sync_err_sum = 0, sync_err_n = 0;
};

double tsec(const dmr_event_t *e)
{
    return (double)e->sample_index / 48000.0;
}

void hex(const uint8_t *p, int n, char *out)
{
    for (int i = 0; i < n; i++)
        sprintf(out + 2 * i, "%02X", p[i]);
}

void on_event(void *user, const dmr_event_t *e)
{
    Ctx &c       = *(Ctx *)user;
    const int v  = c.opt.verbose;
    const int si = (e->slot - 1) & 1;
    char pre[96];
    snprintf(pre, sizeof(pre), "%9.3f TS%d%s CC%-2d", tsec(e), e->slot, e->slot_known ? " " : "?",
             e->color_code == 0xFF ? -1 : e->color_code);
    c.ev_count[e->type & 15]++;
    if (e->color_code != 0xFF && (e->type == DMR_EVT_DATA_BURST || e->type == DMR_EVT_VOICE_BURST))
        c.cc[e->color_code]++;
    if (e->quality.sync_errs != 0xFF && (e->type == DMR_EVT_DATA_BURST || e->type == DMR_EVT_VOICE_BURST)) {
        c.sync_err_sum += e->quality.sync_errs;
        c.sync_err_n++;
        c.synctype[e->sync_type]++;
    }
    char hx[64];
    switch (e->type) {
    case DMR_EVT_SYNC_ACQUIRED:
        if (v >= 1)
            printf("%s SYNC_ACQUIRED %s%s\n", pre, dmr_rx_sync_type_name(e->sync_type),
                   e->inverted ? " (inverted)" : "");
        break;
    case DMR_EVT_SYNC_LOST:
        if (v >= 1)
            printf("%s SYNC_LOST\n", pre);
        break;
    case DMR_EVT_DATA_BURST:
        c.data[si]++;
        if (e->u.data.slot_type_ok) {
            c.dtype[si][e->u.data.data_type]++;
            if (e->u.data.payload_ok || e->u.data.payload_len == 0)
                c.data_ok[si]++;
        } else {
            c.dtype[si][-1]++;
        }
        if (v >= 2) {
            hex(e->u.data.payload, e->u.data.payload_len, hx);
            hx[2 * e->u.data.payload_len] = 0;
            printf("%s DATA  %-18s st=%d pl=%d se=%d fec=%d/%d evm=%.3f %s\n", pre,
                   e->u.data.slot_type_ok ? dmr_rx_data_type_name(e->u.data.data_type) : "(slot type bad)",
                   e->u.data.slot_type_ok, e->u.data.payload_ok, e->quality.sync_errs, e->quality.fec_errs,
                   e->quality.fec_bits, (double)e->quality.evm, hx);
        }
        break;
    case DMR_EVT_VOICE_BURST: {
        c.voice[si]++;
        if (c.in_call[si])
            c.cur[si].bursts++;
        for (int f = 0; f < 3; f++) {
            c.voice_errs[si] += e->u.voice.errs[f];
            c.voice_frames_bad[si] += e->u.voice.bad[f];
        }
        if (c.ambe) {
            for (int f = 0; f < 3; f++) {
                uint8_t rec[10];
                rec[0] = (uint8_t)e->slot;
                memcpy(rec + 1, e->u.voice.ambe49[f], 7);
                rec[8] = e->u.voice.errs[f];
                rec[9] = e->u.voice.bad[f];
                fwrite(rec, 1, sizeof(rec), c.ambe);
            }
        }
        if (v >= 2) {
            char a[3][16];
            for (int f = 0; f < 3; f++)
                hex(e->u.voice.ambe49[f], 7, a[f]);
            printf("%s VOICE %c errs=%d,%d,%d bad=%d%d%d emb=%d lcss=%d se=%d evm=%.3f ambe=%s,%s,%s%s\n", pre,
                   'A' + e->u.voice.burst_index, e->u.voice.errs[0], e->u.voice.errs[1], e->u.voice.errs[2],
                   e->u.voice.bad[0], e->u.voice.bad[1], e->u.voice.bad[2], e->u.voice.emb_ok, e->u.voice.lcss,
                   e->quality.sync_errs == 0xFF ? -1 : e->quality.sync_errs, (double)e->quality.evm, a[0], a[1],
                   a[2], e->u.voice.encrypted ? " ENC" : "");
        }
        break;
    }
    case DMR_EVT_LC: {
        static const char *src[] = {"header", "terminator", "embedded"};
        hex(e->u.lc.raw, 9, hx);
        char key[160];
        snprintf(key, sizeof(key), "TS%d FLCO=%02X FID=%02X svc=%02X dst=%u src=%u", e->slot, e->u.lc.flco,
                 e->u.lc.fid, e->u.lc.service_options, e->u.lc.dst_id, e->u.lc.src_id);
        c.lcs[key]++;
        if (v >= 1)
            printf("%s LC    %-10s %s FID=%02X svc=%02X %s dst=%u src=%u%s%s [%s]\n", pre, src[e->u.lc.source % 3],
                   dmr_rx_flco_name(e->u.lc.flco, e->u.lc.fid), e->u.lc.fid, e->u.lc.service_options,
                   e->u.lc.is_group ? "TG" : "ID", e->u.lc.dst_id, e->u.lc.src_id, e->u.lc.encrypted ? " ENC" : "",
                   e->u.lc.emergency ? " EMERGENCY" : "", hx);
        break;
    }
    case DMR_EVT_CSBK:
        c.csbk[(e->u.csbk.fid << 8) | e->u.csbk.opcode]++;
        hex(e->u.csbk.raw, 10, hx);
        if (v >= 1) {
            printf("%s CSBK  %sop=%02X FID=%02X %s", pre, e->u.csbk.is_mbc ? "MBC " : "", e->u.csbk.opcode,
                   e->u.csbk.fid, e->u.csbk.name);
            if (e->u.csbk.has_ids)
                printf(" dst=%u src=%u", e->u.csbk.dst_id, e->u.csbk.src_id);
            printf(" [%s]\n", hx);
        }
        break;
    case DMR_EVT_PI_HEADER:
        if (v >= 1)
            printf("%s PI    alg=%02X FID=%02X key=%02X MI=%08X dst=%u\n", pre, e->u.pi.algorithm_id, e->u.pi.fid,
                   e->u.pi.key_id, e->u.pi.mi, e->u.pi.dst_id);
        break;
    case DMR_EVT_DATA_HEADER:
        hex(e->u.data_header.raw, 10, hx);
        if (v >= 1)
            printf("%s DHDR  %s DPF=%X SAP=%X dst=%u src=%u blocks=%d [%s]\n", pre,
                   e->u.data_header.is_group ? "group" : "indiv", e->u.data_header.dpf, e->u.data_header.sap,
                   e->u.data_header.dst_id, e->u.data_header.src_id, e->u.data_header.blocks_to_follow, hx);
        break;
    case DMR_EVT_SHORT_LC: {
        char s[128];
        snprintf(s, sizeof(s), "SLCO=%X %02X%02X%02X %s", e->u.short_lc.slco, e->u.short_lc.data[0],
                 e->u.short_lc.data[1], e->u.short_lc.data[2], e->u.short_lc.name);
        c.short_lcs.push_back(s);
        if (v >= 1)
            printf("%s SLC   %s\n", pre, s);
        break;
    }
    case DMR_EVT_TALKER_ALIAS:
        if (e->u.talker_alias.complete)
            c.aliases.push_back(e->u.talker_alias.text);
        if (v >= 1)
            printf("%s TA    \"%s\"%s\n", pre, e->u.talker_alias.text, e->u.talker_alias.complete ? "" : " (partial)");
        break;
    case DMR_EVT_CALL_START:
    case DMR_EVT_CALL_UPDATE:
    case DMR_EVT_CALL_END: {
        const dmr_evt_call_t &k = e->u.call;
        if (v >= 1) {
            printf("%s %s", pre,
                   e->type == DMR_EVT_CALL_START ? "CALL_START " : e->type == DMR_EVT_CALL_END ? "CALL_END   "
                                                                                              : "CALL_UPDATE");
            if (k.ids_valid)
                printf(" %s dst=%u src=%u", k.group ? "group" : "private", k.dst_id, k.src_id);
            else
                printf(" (ids unknown)");
            printf("%s%s%s", k.encrypted ? " ENCRYPTED" : "", k.emergency ? " EMERGENCY" : "",
                   k.late_entry ? " late-entry" : "");
            if (e->type == DMR_EVT_CALL_END)
                printf(" voice_bursts=%u", k.voice_bursts);
            printf("\n");
        }
        if (e->type == DMR_EVT_CALL_START) {
            c.in_call[si] = true;
            c.cur[si]     = CallRec{e->slot, k.dst_id, k.src_id, (bool)k.group, (bool)k.encrypted, (bool)k.late_entry,
                                    (bool)k.ids_valid, 0, tsec(e), tsec(e)};
        } else if (c.in_call[si]) {
            CallRec &r = c.cur[si];
            r.dst = k.dst_id, r.src = k.src_id, r.group = k.group, r.enc = k.encrypted, r.ids = k.ids_valid;
            if (e->type == DMR_EVT_CALL_END) {
                r.bursts = k.voice_bursts;
                r.t1     = tsec(e);
                c.calls.push_back(r);
                c.in_call[si] = false;
            }
        }
        break;
    }
    case DMR_EVT_DATA_PDU: {
        const dmr_evt_data_pdu_t &p = e->u.data_pdu;
        if (v >= 1) {
            printf("%s PDU   %u -> %s%u  DPF %u SAP %u %s, %u blocks", pre, p.src_id, p.is_group ? "TG " : "",
                   p.dst_id, p.dpf, p.sap, p.confirmed ? "confirmed" : "unconfirmed", p.blocks);
            if (p.blocks_missing)
                printf(" (%u missing)", p.blocks_missing);
            if (p.blocks)
                printf(", CRC %s", !p.complete ? "not checked" : p.crc_ok ? "ok" : "BAD");
            if (p.prop_headers)
                printf(", MFID 0x%02X", p.mfid);
            if (p.src_port || p.dst_port)
                printf(", UDP %u -> %u", p.src_port, p.dst_port);
            printf(": %s", p.format);
            if (p.kind == DMR_PDU_STATUS)
                printf(" %u", p.status);
            if (p.kind == DMR_PDU_TEXT)
                printf(" \"%s\"", p.text);
            printf("\n");
            if (v >= 2 || p.kind == DMR_PDU_UNKNOWN) {
                printf("%*s", (int)strlen(pre) + 7, "");
                for (unsigned i = 0; i < p.len; i++)
                    printf("%02x", p.data[i]);
                printf("\n");
            }
        }
        break;
    }
    default:
        break;
    }
}

void on_pcm(void *user, uint8_t slot, const int16_t *pcm, size_t n)
{
    Ctx &c = *(Ctx *)user;
    (void)slot;
    if (c.pcm)
        fwrite(pcm, sizeof(int16_t), n, c.pcm);
    c.pcm_samples += n;
}

bool load(const char *path, std::vector<int16_t> &out, unsigned &rate)
{
    FILE *f = fopen(path, "rb");
    if (!f)
        return false;
    std::vector<uint8_t> buf;
    uint8_t tmp[65536];
    size_t n;
    while ((n = fread(tmp, 1, sizeof(tmp), f)) > 0)
        buf.insert(buf.end(), tmp, tmp + n);
    fclose(f);
    size_t off = 0, len = buf.size();
    unsigned ch = 1, bits = 16;
    rate = 48000;
    if (buf.size() > 12 && !memcmp(buf.data(), "RIFF", 4) && !memcmp(buf.data() + 8, "WAVE", 4)) {
        size_t p = 12;
        bool found = false;
        while (p + 8 <= buf.size()) {
            const uint32_t sz = buf[p + 4] | (buf[p + 5] << 8) | (buf[p + 6] << 16) | ((uint32_t)buf[p + 7] << 24);
            if (!memcmp(&buf[p], "fmt ", 4) && p + 24 <= buf.size()) {
                ch   = buf[p + 10] | (buf[p + 11] << 8);
                rate = buf[p + 12] | (buf[p + 13] << 8) | (buf[p + 14] << 16) | ((uint32_t)buf[p + 15] << 24);
                bits = buf[p + 22] | (buf[p + 23] << 8);
            } else if (!memcmp(&buf[p], "data", 4)) {
                off   = p + 8;
                len   = sz;
                if (off + len > buf.size() || sz == 0xFFFFFFFFu)
                    len = buf.size() - off;
                found = true;
                break;
            }
            p += 8 + sz + (sz & 1);
        }
        if (!found || bits != 16 || ch < 1) {
            fprintf(stderr, "%s: unsupported WAV (need 16 bit PCM)\n", path);
            return false;
        }
    }
    const size_t frames = len / (2 * ch);
    out.resize(frames);
    for (size_t i = 0; i < frames; i++) {
        const uint8_t *p = &buf[off + i * 2 * ch];
        out[i]           = (int16_t)(p[0] | (p[1] << 8));
    }
    return true;
}

void usage()
{
    fprintf(stderr,
            "usage: dmr_rx_file [options] recording.wav|.raw\n"
            "  -q            summary only\n"
            "  -v            print every burst\n"
            "  -b N          feed N samples per call (default 960)\n"
            "  -p auto|normal|inverted   polarity (default auto)\n"
            "  -n            negate the input samples (test)\n"
            "  -w FILE       write decoded audio (raw S16LE, 8 kHz mono)\n"
            "  -s MASK       slots to decode audio for (1, 2 or 3; default 3)\n"
            "  -g GAIN       audio gain (default 1.0)\n"
            "  -a FILE       dump AMBE frames (slot, 7 bytes ambe49, errs, bad)\n");
}

} // namespace

int main(int argc, char **argv)
{
    Ctx c;
    Options &o = c.opt;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "-q")
            o.verbose = 0;
        else if (a == "-v")
            o.verbose = 2;
        else if (a == "-n")
            o.negate = true;
        else if (a == "-b" && i + 1 < argc)
            o.block = (size_t)atol(argv[++i]);
        else if (a == "-w" && i + 1 < argc)
            o.pcm = argv[++i];
        else if (a == "-a" && i + 1 < argc)
            o.ambe = argv[++i];
        else if (a == "-s" && i + 1 < argc)
            o.slotmask = (unsigned)atoi(argv[++i]);
        else if (a == "-g" && i + 1 < argc)
            o.gain = (float)atof(argv[++i]);
        else if (a == "-p" && i + 1 < argc) {
            std::string p = argv[++i];
            o.polarity = p == "normal" ? DMR_POLARITY_NORMAL : p == "inverted" ? DMR_POLARITY_INVERTED
                                                                              : DMR_POLARITY_AUTO;
        } else if (a[0] == '-') {
            usage();
            return 2;
        } else
            o.path = argv[i];
    }
    if (!o.path || o.block == 0) {
        usage();
        return 2;
    }
    std::vector<int16_t> x;
    unsigned rate = 0;
    if (!load(o.path, x, rate)) {
        fprintf(stderr, "cannot read %s\n", o.path);
        return 1;
    }
    if (rate != DMR_RX_SAMPLE_RATE)
        fprintf(stderr, "warning: sample rate is %u Hz, dmr_rx expects %d Hz\n", rate, DMR_RX_SAMPLE_RATE);
    if (o.negate)
        for (auto &s : x)
            s = (int16_t)(s == -32768 ? 32767 : -s);
    if (o.pcm)
        c.pcm = fopen(o.pcm, "wb");
    if (o.ambe)
        c.ambe = fopen(o.ambe, "wb");

    dmr_rx_config_t cfg;
    dmr_rx_config_default(&cfg);
    cfg.event_cb      = on_event;
    cfg.event_user    = &c;
    cfg.pcm_cb        = on_pcm;
    cfg.pcm_user      = &c;
    cfg.polarity      = (uint8_t)o.polarity;
    cfg.pcm_slot_mask = (uint8_t)o.slotmask;
    cfg.pcm_gain      = o.gain;
    dmr_rx_t *rx      = dmr_rx_create(&cfg);
    if (!rx)
        return 1;

    const auto t0 = std::chrono::steady_clock::now();
    for (size_t i = 0; i < x.size(); i += o.block)
        dmr_rx_process_fm_s16(rx, &x[i], std::min(o.block, x.size() - i));
    const auto t1    = std::chrono::steady_clock::now();
    const double sec = std::chrono::duration<double>(t1 - t0).count();

    dmr_rx_status_t st;
    dmr_rx_get_status(rx, &st);
    const double dur = (double)x.size() / 48000.0;
    printf("== summary: %s\n", o.path);
    printf("duration %.2f s, %zu samples; processed in %.3f s (%.1f ns/sample, %.0fx realtime)%s\n", dur, x.size(),
           sec, sec * 1e9 / (double)(x.size() ? x.size() : 1), dur / (sec > 0 ? sec : 1),
           dmr_rx_has_voice() ? " [incl. vocoder if audio present]" : "");
    printf("polarity %s%s, clock %+.1f ppm, deviation %.4f, dc %.4f, syncs %u, sync losses %u, mean sync errs %.2f\n",
           st.inverted ? "inverted" : "normal", st.polarity_locked ? " (locked)" : "", (double)st.clock_ppm,
           (double)st.deviation, (double)st.dc_offset, st.sync_count, st.sync_loss_count,
           c.sync_err_n ? (double)c.sync_err_sum / (double)c.sync_err_n : 0.0);
    printf("sync types:");
    for (auto &k : c.synctype)
        printf(" %s=%llu", dmr_rx_sync_type_name((uint8_t)k.first), (unsigned long long)k.second);
    printf("\ncolour codes:");
    for (auto &k : c.cc)
        printf(" CC%d=%llu", k.first, (unsigned long long)k.second);
    printf("\n");
    for (int s = 0; s < 2; s++) {
        printf("TS%d: voice bursts %llu (frames %llu, bad %llu, AMBE FEC errs %llu), data bursts %llu (ok %llu), "
               "missed %u, FEC BER %.4f\n",
               s + 1, (unsigned long long)c.voice[s], (unsigned long long)c.voice[s] * 3,
               (unsigned long long)c.voice_frames_bad[s], (unsigned long long)c.voice_errs[s],
               (unsigned long long)c.data[s], (unsigned long long)c.data_ok[s], st.slot[s].missed_bursts,
               st.slot[s].fec_bits ? (double)st.slot[s].fec_errs / (double)st.slot[s].fec_bits : 0.0);
        if (!c.dtype[s].empty()) {
            printf("     data types:");
            for (auto &k : c.dtype[s])
                printf(" %s=%llu", k.first < 0 ? "(bad slot type)" : dmr_rx_data_type_name((uint8_t)k.first),
                       (unsigned long long)k.second);
            printf("\n");
        }
    }
    for (int s = 0; s < 2; s++)
        if (c.in_call[s]) {
            c.cur[s].t1     = dur;
            c.calls.push_back(c.cur[s]);
        }
    printf("calls: %zu\n", c.calls.size());
    for (auto &k : c.calls) {
        printf("  TS%d %7.2f..%7.2f s  ", k.slot, k.t0, k.t1);
        if (k.ids)
            printf("%s dst=%u src=%u", k.group ? "group" : "private", k.dst, k.src);
        else
            printf("(ids unknown)");
        printf("%s%s bursts=%u\n", k.enc ? " ENCRYPTED" : "", k.late ? " late-entry" : "", k.bursts);
    }
    if (!c.lcs.empty()) {
        printf("link control PDUs:\n");
        for (auto &k : c.lcs)
            printf("  %s  x%llu\n", k.first.c_str(), (unsigned long long)k.second);
    }
    if (!c.csbk.empty()) {
        printf("CSBKs:\n");
        for (auto &k : c.csbk)
            printf("  op=%02X FID=%02X %-50s x%llu\n", k.first & 0xFF, k.first >> 8,
                   dmr_rx_csbk_name((uint8_t)(k.first & 0xFF), (uint8_t)(k.first >> 8)), (unsigned long long)k.second);
    }
    if (!c.short_lcs.empty()) {
        std::map<std::string, int> u;
        for (auto &s : c.short_lcs)
            u[s]++;
        printf("short LC (distinct %zu, changes %zu):\n", u.size(), c.short_lcs.size());
        int n = 0;
        for (auto &k : u) {
            if (n++ >= 12) {
                printf("  ...\n");
                break;
            }
            printf("  %s  x%d\n", k.first.c_str(), k.second);
        }
    }
    for (auto &a : c.aliases)
        printf("talker alias: \"%s\"\n", a.c_str());
    if (c.pcm_samples)
        printf("audio: %zu samples (%.2f s)\n", c.pcm_samples, (double)c.pcm_samples / 8000.0);
    if (c.pcm)
        fclose(c.pcm);
    if (c.ambe)
        fclose(c.ambe);
    dmr_rx_destroy(rx);
    return 0;
}
