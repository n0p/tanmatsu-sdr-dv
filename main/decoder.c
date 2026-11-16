// SPDX-License-Identifier: AGPL-3.0-only
// Glue between the DSP chain and the protocol decoders (components/dmr_rx, components/tetra_rx).
//
// The decoders run inside the DSP task: their callbacks turn events into short text lines for the
// info panel and the console, and DMR speech (8 kHz PCM from the vocoder) is interpolated to the
// 48 kHz audio path.

#include "decoder.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "dmr_rx.h"
#include "dsp.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "tetra_codec.h"
#include "tetra_rx.h"

static const char TAG[] = "decoder";

// ---- Event log -----------------------------------------------------------------------------------

#define STAMP_LEN 7  // "mm:ss  "

static SemaphoreHandle_t lock;
static char              evlog[DEC_LOG_LINES][DEC_LINE_LEN];  // newest first

static void __attribute__((format(printf, 1, 2))) log_event(const char* fmt, ...) {
    char     line[DEC_LINE_LEN];
    unsigned s = (unsigned)(esp_timer_get_time() / 1000000);
    snprintf(line, sizeof(line), "%02u:%02u  ", s / 60 % 100, s % 60);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line + STAMP_LEN, sizeof(line) - STAMP_LEN, fmt, ap);
    va_end(ap);

    xSemaphoreTake(lock, portMAX_DELAY);
    // A repeat of an entry that is still in the log moves that entry to the top with a fresh time
    // stamp instead of adding a line: base stations repeat call set-up and transmit grants every
    // few seconds for late entry, which would push everything else out.
    int at = 0;
    while (at < DEC_LOG_LINES - 1 && strcmp(evlog[at] + STAMP_LEN, line + STAMP_LEN) != 0) at++;
    if (at > 0) memmove(evlog[1], evlog[0], (size_t)at * DEC_LINE_LEN);
    memcpy(evlog[0], line, sizeof(line));
    xSemaphoreGive(lock);
}

static void log_clear(void) {
    xSemaphoreTake(lock, portMAX_DELAY);
    memset(evlog, 0, sizeof(evlog));
    xSemaphoreGive(lock);
}

int dec_get_log(char lines[][DEC_LINE_LEN], int max) {
    if (max > DEC_LOG_LINES) max = DEC_LOG_LINES;
    xSemaphoreTake(lock, portMAX_DELAY);
    int n = 0;
    while (n < max && evlog[n][0]) {
        memcpy(lines[n], evlog[n], DEC_LINE_LEN);
        n++;
    }
    xSemaphoreGive(lock);
    return n;
}

// ---- 8 kHz -> 48 kHz speech interpolator ---------------------------------------------------------

#define UP_FACTOR 6
#define UP_PHASE  12  // taps per output phase
#define UP_TAPS   (UP_FACTOR * UP_PHASE)
#define PCM_MAX   240  // the longest speech frame: TETRA, 30 ms

static float up_h[UP_TAPS];
static float up_x[UP_PHASE - 1 + PCM_MAX];  // history followed by the new frame

static void speech_out(const int16_t* pcm, int n) {
    static int16_t out[PCM_MAX * UP_FACTOR];
    if (n > PCM_MAX) n = PCM_MAX;
    for (int i = 0; i < n; i++) up_x[UP_PHASE - 1 + i] = (float)pcm[i];
    for (int i = 0; i < n; i++) {
        const float* x = &up_x[UP_PHASE - 1 + i];  // newest sample of this output group
        for (int p = 0; p < UP_FACTOR; p++) {
            float acc = 0.0f;
            for (int k = 0; k < UP_PHASE; k++) acc += up_h[p + UP_FACTOR * k] * x[-k];
            acc *= (float)UP_FACTOR;
            if (acc > 32767.0f) acc = 32767.0f;
            if (acc < -32768.0f) acc = -32768.0f;
            out[i * UP_FACTOR + p] = (int16_t)acc;
        }
    }
    memmove(up_x, up_x + n, sizeof(float) * (UP_PHASE - 1));
    cap_feed_s16(TAP_AUDIO, out, n * UP_FACTOR, AUDIO_RATE);
    rec_feed(out, n * UP_FACTOR);
    audio_push(out, n * UP_FACTOR);
}

// ---- DMR -----------------------------------------------------------------------------------------

static dmr_rx_t* dmr;
static volatile int      voice_slot;  // timeslot currently routed to the speaker (one at a time)
static int64_t           voice_last_us;
static volatile uint32_t voice_seen_ms;  // the same moment, for the other tasks

// Which timeslot may use the speaker. A selected slot (g_set.dmr_slot / tetra_slot) is the only one
// that is heard. In automatic mode the slot that started first keeps the speaker until it has been
// quiet for hold_us. Returns false if this slot's speech is to be dropped; sets *start at the
// beginning of a talk spurt (after a pause of spurt_us, or when the speaker changes slot).
static bool voice_gate(int slot, int wanted, int64_t now, int64_t hold_us, int64_t spurt_us, bool* start) {
    bool changed = slot != voice_slot;
    if (wanted) {
        if (slot != wanted) return false;
    } else if (changed && now - voice_last_us < hold_us) {
        return false;
    }
    *start        = changed || now - voice_last_us > spurt_us;
    voice_slot    = slot;
    voice_last_us = now;
    voice_seen_ms = (uint32_t)(now / 1000);
    return true;
}

int dec_speaker_slot(void) {
    uint32_t age = (uint32_t)(esp_timer_get_time() / 1000) - voice_seen_ms;
    return age < 500 ? voice_slot : 0;
}

#define JITTER_MS 80

static void* dec_alloc(size_t size) {
    // Decoder state is touched for every sample: internal RAM if there is room, PSRAM otherwise.
    void* p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (p == NULL) p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p;
}

static void dec_free(void* p) {
    heap_caps_free(p);
}

static void fmt_call(char* out, size_t len, const dmr_evt_call_t* c) {
    if (!c->ids_valid) {
        snprintf(out, len, "ids not yet known");
    } else if (c->group) {
        snprintf(out, len, "TG %u from %u", (unsigned)c->dst_id, (unsigned)c->src_id);
    } else {
        snprintf(out, len, "private %u -> %u", (unsigned)c->src_id, (unsigned)c->dst_id);
    }
}

// A data packet, put together from its header and blocks: a line in the event log; text messages,
// status messages and content this decoder does not know also go to the list of received messages
// (F5) and the log on the card. Registration, location and acknowledgements stay in the event log.
static void dmr_message(unsigned slot, const dmr_evt_data_pdu_t* p) {
    static char body[MSG_TEXT_MAX];  // DSP task only
    char        from[28], to[16];
    snprintf(from, sizeof(from), "%u", (unsigned)p->src_id);
    snprintf(to, sizeof(to), "%s%u", p->is_group ? "TG " : "", (unsigned)p->dst_id);
    // Blocks lost on the way leave holes, and what is left cannot be checked; a complete packet
    // with a wrong CRC-32 has an error somewhere that the block coding did not see.
    const char* flaw = !p->blocks ? NULL : !p->complete ? "incomplete" : !p->crc_ok ? "CRC error" : NULL;

    switch (p->kind) {
        case DMR_PDU_TEXT:
            log_event("TS%u %s %s -> %s: %.40s", slot, p->format, from, to, p->text);
            if (flaw) snprintf(body, sizeof(body), "[%s] %s", flaw, p->text);
            else snprintf(body, sizeof(body), "%s", p->text);
            msglog_put("DMR", false, 0, 0, from, to, "text", body);
            break;
        case DMR_PDU_STATUS:
            log_event("TS%u status %u  %s -> %s", slot, p->status, from, to);
            snprintf(body, sizeof(body), "%u", p->status);
            msglog_put("DMR", false, 0, 0, from, to, "status", body);
            break;
        case DMR_PDU_SERVICE:
            log_event("TS%u %s  %s -> %s%s%s", slot, p->format, from, to, flaw ? ", " : "", flaw ? flaw : "");
            break;
        default: {
            log_event("TS%u %s %s -> %s  SAP %u, %u bytes%s%s", slot, p->format, from, to, p->sap, p->len,
                      flaw ? ", " : "", flaw ? flaw : "");
            if (flaw || !p->len) break;  // nobody can do anything with a damaged packet of unknown content
            int o = snprintf(body, sizeof(body), "%s, SAP %u, ", p->format, p->sap);
            if (p->prop_headers) o += snprintf(body + o, sizeof(body) - (size_t)o, "MFID 0x%02X, ", p->mfid);
            if (p->src_port || p->dst_port) {
                o += snprintf(body + o, sizeof(body) - (size_t)o, "port %u -> %u, ", p->src_port, p->dst_port);
            }
            o += snprintf(body + o, sizeof(body) - (size_t)o, "%u bytes: ", p->len);
            unsigned i = 0;
            for (; i < p->len && o < (int)sizeof(body) - 6; i++) {
                body[o++] = "0123456789ABCDEF"[p->data[i] >> 4];
                body[o++] = "0123456789ABCDEF"[p->data[i] & 15];
            }
            snprintf(body + o, sizeof(body) - (size_t)o, "%s", i < p->len ? "..." : "");
            msglog_put("DMR", false, 0, 0, from, to, "data", body);
            break;
        }
    }
}

static void on_dmr_event(void* user, const dmr_event_t* e) {
    (void)user;
    char call[48];
    switch (e->type) {
        case DMR_EVT_CALL_START:
        case DMR_EVT_CALL_UPDATE:
            // Without the link control (late entry) there is nothing to log yet; the timeslot line
            // already shows the call.
            if (!e->u.call.ids_valid) break;
            fmt_call(call, sizeof(call), &e->u.call);
            log_event("TS%u voice: %s%s%s%s", e->slot, call, e->u.call.encrypted ? "  ENCRYPTED" : "",
                      e->u.call.emergency ? "  EMERGENCY" : "", e->u.call.late_entry ? "  (late entry)" : "");
            break;
        case DMR_EVT_CALL_END:
            if (!e->u.call.ids_valid && e->u.call.voice_bursts < 6) break;
            fmt_call(call, sizeof(call), &e->u.call);
            log_event("TS%u call end: %s, %u.%u s", e->slot, call, (unsigned)(e->u.call.voice_bursts * 6 / 100),
                      (unsigned)(e->u.call.voice_bursts * 6 / 10 % 10));
            break;
        case DMR_EVT_TALKER_ALIAS:
            if (e->u.talker_alias.complete) log_event("TS%u talker alias: %.48s", e->slot, e->u.talker_alias.text);
            break;
        case DMR_EVT_CSBK:
            if (e->u.csbk.has_ids) {
                log_event("TS%u CSBK %.30s  %u -> %u", e->slot, e->u.csbk.name, (unsigned)e->u.csbk.src_id,
                          (unsigned)e->u.csbk.dst_id);
            } else {
                log_event("TS%u CSBK %.40s", e->slot, e->u.csbk.name);
            }
            break;
        case DMR_EVT_DATA_HEADER: {
            // Packets the library puts together are logged when they are over (DMR_EVT_DATA_PDU);
            // a proprietary header is part of one. That leaves the response packets and the like.
            const uint8_t* h   = e->u.data_header.raw;
            unsigned       dpf = e->u.data_header.dpf;
            if (dpf == 14 || dpf == 15) break;
            if ((dpf == 2 || dpf == 3) && e->u.data_header.blocks_to_follow) break;
            if (dpf == 13 && ((h[0] & 0x30u) || (h[1] & 0x0Fu))) break;  // short data with appended blocks
            log_event("TS%u data%s: %u -> %s%u  SAP %u, %u blocks", e->slot, dpf == 1 ? " response" : "",
                      (unsigned)e->u.data_header.src_id, e->u.data_header.is_group ? "TG " : "",
                      (unsigned)e->u.data_header.dst_id, e->u.data_header.sap, e->u.data_header.blocks_to_follow);
            break;
        }
        case DMR_EVT_DATA_PDU:
            dmr_message(e->slot, &e->u.data_pdu);
            break;
        case DMR_EVT_PI_HEADER:
            log_event("TS%u encrypted: algorithm %u key %u", e->slot, e->u.pi.algorithm_id, e->u.pi.key_id);
            break;
        default:
            break;
    }
}

static void on_dmr_pcm(void* user, uint8_t slot, const int16_t* pcm, size_t n) {
    (void)user;
    // Only one timeslot goes to the speaker: the selected one, or the one that started first.
    bool start;
    if (!voice_gate(slot, g_set.dmr_slot, esp_timer_get_time(), 200000, 150000, &start)) return;
    if (start) {
        // Start of a talk spurt. Speech arrives 60 ms at a time, once per voice burst: lead with
        // silence so the output buffer does not run dry between bursts.
        static const int16_t silence[480];
        for (int i = 0; i < JITTER_MS / 10; i++) audio_push(silence, 480);
    }
    speech_out(pcm, (int)n);
}

static void dmr_slot_line(char* out, int slot, const dmr_slot_status_t* s) {
    // Where this slot's speech goes: to the speaker, or nowhere because another slot is selected
    // or has the speaker. Encrypted speech is never decoded and says so by itself.
    int         wanted  = g_set.dmr_slot;
    int         speaker = dec_speaker_slot();
    const char* audio   = "";
    if (speaker == slot) {
        audio = " (audio)";
    } else if (!s->encrypted && ((wanted && wanted != slot) || speaker)) {
        audio = " (muted)";
    }
    if (!s->active) {
        snprintf(out, DEC_LINE_LEN, "TS%d  -", slot);
    } else if (s->call_active) {
        char who[40];
        if (!s->ids_valid) {
            snprintf(who, sizeof(who), "(ids not yet known)");
        } else if (s->group) {
            snprintf(who, sizeof(who), "TG %u < %u", (unsigned)s->dst_id, (unsigned)s->src_id);
        } else {
            snprintf(who, sizeof(who), "%u > %u", (unsigned)s->src_id, (unsigned)s->dst_id);
        }
        snprintf(out, DEC_LINE_LEN, "TS%d  VOICE%s  %s%s%s%s%.24s", slot, audio, who, s->encrypted ? "  ENCRYPTED" : "",
                 s->emergency ? "  EMERGENCY" : "", s->talker_alias[0] ? "  " : "", s->talker_alias);
    } else if (s->state == DMR_SLOT_DATA && s->last_data_type != 0xFF) {
        snprintf(out, DEC_LINE_LEN, "TS%d  %.40s", slot, dmr_rx_data_type_name(s->last_data_type));
    } else {
        snprintf(out, DEC_LINE_LEN, "TS%d  sync", slot);
    }
}

static void dmr_lines(char lines[DEC_LINES][DEC_LINE_LEN]) {
    if (dmr == NULL) {
        snprintf(lines[0], DEC_LINE_LEN, "DMR: decoder could not be started (out of memory)");
        return;
    }
    dmr_rx_status_t st;
    dmr_rx_get_status(dmr, &st);
    char sel[16] = "AUTO";
    if (g_set.dmr_slot) snprintf(sel, sizeof(sel), "TS%d", g_set.dmr_slot);
    if (!st.synced) {
        snprintf(lines[0], DEC_LINE_LEN, "DMR  no sync   audio: %s", sel);
    } else {
        const dmr_slot_status_t* a  = st.slot[0].active ? &st.slot[0] : &st.slot[1];
        float                    ber = st.slot[0].ber > st.slot[1].ber ? st.slot[0].ber : st.slot[1].ber;
        char                     cc[8] = "?";
        if (a->color_code != 0xFF) snprintf(cc, sizeof(cc), "%u", a->color_code);
        const char* link = "repeater";
        if (a->sync_type == DMR_SYNC_MS_VOICE || a->sync_type == DMR_SYNC_MS_DATA) link = "mobile / simplex";
        if (a->sync_type >= DMR_SYNC_DM_TS1_VOICE) link = "direct mode";
        snprintf(lines[0], DEC_LINE_LEN, "DMR  %s  CC %s  BER %.1f%%  clock %+.0f ppm%s   audio: %s", link, cc,
                 (double)(ber * 100.0f), (double)st.clock_ppm, st.inverted ? "  inverted" : "", sel);
    }
    dmr_slot_line(lines[1], 1, &st.slot[0]);
    dmr_slot_line(lines[2], 2, &st.slot[1]);
    char recent[2][DEC_LINE_LEN];
    int  n = dec_get_log(recent, 2);
    for (int i = 0; i < n; i++) memcpy(lines[3 + i], recent[i], DEC_LINE_LEN);
}

// ---- TETRA ---------------------------------------------------------------------------------------

static tetra_rx_t*       tetra;
static tetra_rx_status_t tetra_st;  // snapshot taken in the DSP task (the library does no locking)
static uint32_t          tetra_sig_clear, tetra_sig_enc;  // signalling PDUs since the last reset
static uint32_t          tetra_main_hz;                   // last broadcast info that went to the log
static int               tetra_la = -1;
static int64_t           tetra_tch_us[4] = {-10000000, -10000000, -10000000, -10000000};  // last speech frame per timeslot
static volatile bool     tetra_cell;  // a cell broadcast was decoded since the last burst sync
static volatile uint16_t tetra_mcc, tetra_mnc;  // ... and the network it named
static int               tetra_air_enc = -1;  // what the cell broadcasts: -1 not seen yet, 0 off, 1 on

// Speech of one timeslot comes as 60 ms (two frames) per TDMA frame of 56.67 ms, with nothing in
// frame 18: over a multiframe the output buffer gains 57 ms and then loses it in one go. A talk
// spurt therefore starts with this much silence, so the buffer survives a start right before the gap.
#define TETRA_PREFILL_MS 90
#define TETRA_SPURT_US   400000  // a pause this long ends a talk spurt (the frame 18 gap is 113 ms)

static void fmt_mhz(char* out, size_t len, uint32_t hz) {
    snprintf(out, len, "%u.%04u", (unsigned)(hz / 1000000u), (unsigned)(hz % 1000000u / 100u));
}

bool dec_tetra_plmn(unsigned* mcc, unsigned* mnc) {
    *mcc = tetra_mcc;
    *mnc = tetra_mnc;
    return tetra_cell && g_set.mode == MODE_TETRA && g_stat.status == RADIO_STREAMING;
}

// A short data or status message: to the list of received messages (F5) and the log on the card.
static void tetra_message(const tetra_rx_ev_signal_t* s, const char* dst) {
    const tetra_rx_cmce_t* c = &s->cmce;
    static char            body[MSG_TEXT_MAX];  // DSP task only
    const char*            kind;
    char                   from[28] = "?";
    if ((c->fields & TETRA_RX_CMCE_F_PARTY) && c->party.present == 2) {
        snprintf(from, sizeof(from), "%u-%u-%u", c->party.mcc, c->party.mnc, (unsigned)c->party.ssi);
    } else if ((c->fields & TETRA_RX_CMCE_F_PARTY) && c->party.present) {
        snprintf(from, sizeof(from), "%u", (unsigned)c->party.ssi);
    }
    if (s->pdu_type == TETRA_RX_CMCE_D_STATUS) {
        if (!(c->fields & TETRA_RX_CMCE_F_STATUS)) return;
        kind = "status";
        snprintf(body, sizeof(body), "%u", c->status);
    } else if (c->fields & TETRA_RX_CMCE_F_TEXT) {
        kind = "text";
        snprintf(body, sizeof(body), "%s", c->text);
    } else if ((c->fields & TETRA_RX_CMCE_F_SDS) && c->sds_data) {
        // Not a text this decoder reads: the user data as it is, in hex (the last digit padded with zero bits).
        kind  = "data";
        int o = snprintf(body, sizeof(body), "SDS type %u, ", c->sds_type + 1u);
        if (c->fields & TETRA_RX_CMCE_F_PROTOCOL_ID) o += snprintf(body + o, sizeof(body) - (size_t)o, "protocol 0x%02X, ", c->protocol_id);
        o += snprintf(body + o, sizeof(body) - (size_t)o, "%u bits: ", c->sds_bits);
        for (unsigned i = 0; i < c->sds_bits && o < (int)sizeof(body) - 1; i += 4) {
            unsigned v = 0;
            for (unsigned k = 0; k < 4; k++) v = (v << 1) | ((i + k < c->sds_bits) ? (c->sds_data[i + k] & 1u) : 0u);
            body[o++] = "0123456789ABCDEF"[v];
        }
        body[o] = 0;
    } else {
        return;
    }
    msglog_put("TETRA", tetra_cell, tetra_mcc, tetra_mnc, from, dst, kind, body);
}

static void tetra_signal(const tetra_rx_ev_signal_t* s) {
    if (s->encrypted) {
        tetra_sig_enc++;  // air-interface encrypted: nothing to show but the count
        return;
    }
    if (!s->have_mle) return;
    tetra_sig_clear++;

    char dst[20] = "all";
    if (s->addr.type != TETRA_RX_ADDR_NULL && s->addr.type != TETRA_RX_ADDR_EVENT_LABEL && s->addr.ssi != 0xFFFFFF) {
        snprintf(dst, sizeof(dst), "%u", (unsigned)s->addr.ssi);
    }
    char alloc[32] = "";
    if (s->chan_alloc.present) {
        char tn[5];
        int  k = 0;
        for (int i = 0; i < 4; i++) {
            if (s->chan_alloc.timeslots & (8u >> i)) tn[k++] = (char)('1' + i);
        }
        tn[k] = 0;
        if (s->chan_alloc.dl_freq_hz) {
            char f[16];
            fmt_mhz(f, sizeof(f), s->chan_alloc.dl_freq_hz);
            snprintf(alloc, sizeof(alloc), "  > %s TN%s", f, tn);
        } else {
            snprintf(alloc, sizeof(alloc), "  > carrier %u TN%s", s->chan_alloc.carrier, tn);
        }
    }

    if (!s->have_cmce) {
        // Mobility management and packet data signalling. The MLE broadcasts repeat every few
        // seconds and say nothing new.
        if (s->pdisc == TETRA_RX_PD_MLE) return;
        log_event("%s %.28s to %s%s", tetra_rx_pdisc_name(s->pdisc), s->pdu_name, dst, alloc);
        return;
    }

    const tetra_rx_cmce_t* c        = &s->cmce;
    char                   call[16] = "";
    char                   from[24] = "";
    if (c->fields & TETRA_RX_CMCE_F_CALL_ID) snprintf(call, sizeof(call), " call %u", c->call_id);
    if ((c->fields & TETRA_RX_CMCE_F_PARTY) && c->party.present) {
        snprintf(from, sizeof(from), " from %u", (unsigned)c->party.ssi);
    }
    const char* e2e = ((c->fields & TETRA_RX_CMCE_F_BASIC_SERVICE) && c->e2e_encryption) ? "  E2EE" : "";
    switch (s->pdu_type) {
        case TETRA_RX_CMCE_D_SETUP: {
            static const char* const kind[4] = {"individual", "group", "group (ack)", "broadcast"};
            const char*              what    = (c->fields & TETRA_RX_CMCE_F_BASIC_SERVICE) ? kind[c->comm_type & 3] : "";
            log_event("D-SETUP%s %s%s to %s%s%s", call, what, from, dst, e2e, alloc);
            break;
        }
        case TETRA_RX_CMCE_D_SDS_DATA:
            if (c->fields & TETRA_RX_CMCE_F_TEXT) {
                log_event("SDS%s to %s: %.44s", from, dst, c->text);
            } else {
                log_event("SDS%s to %s  type %u, %u bits", from, dst, c->sds_type + 1u, c->sds_bits);
            }
            tetra_message(s, dst);
            break;
        case TETRA_RX_CMCE_D_STATUS:
            log_event("status %u%s to %s", c->status, from, dst);
            tetra_message(s, dst);
            break;
        case TETRA_RX_CMCE_D_RELEASE:
        case TETRA_RX_CMCE_D_DISCONNECT:
            if (c->fields & TETRA_RX_CMCE_F_DISC_CAUSE) {
                log_event("%.20s%s to %s  cause %u", s->pdu_name, call, dst, c->disconnect_cause);
                break;
            }
            // fall through
        default:
            log_event("%.20s%s%s to %s%s%s", s->pdu_name, call, from, dst, e2e, alloc);
            break;
    }
}

static void on_tetra_event(void* user, const tetra_rx_event_t* e) {
    (void)user;
    switch (e->type) {
        case TETRA_RX_EV_SYNC:
            // The burst search also locks onto noise for a moment now and then, so a lock only
            // counts (and its loss is only reported) once a cell broadcast was decoded with it.
            if (!e->u.sync.locked && tetra_cell) log_event("signal lost");
            tetra_cell = false;
            break;
        case TETRA_RX_EV_BSCH:
            if (e->u.bsch.changed || !tetra_cell) {
                log_event("cell MCC %u MNC %u, colour code %u", e->u.bsch.mcc, e->u.bsch.mnc, e->u.bsch.colour_code);
            }
            tetra_mcc  = e->u.bsch.mcc;
            tetra_mnc  = e->u.bsch.mnc;
            tetra_cell = true;
            break;
        case TETRA_RX_EV_SYSINFO: {
            const tetra_rx_ev_sysinfo_t* si = &e->u.sysinfo;
            tetra_air_enc                   = (si->bs_service_details & TETRA_RX_BSD_AIR_ENCRYPTION) ? 1 : 0;
            if (si->dl_freq_hz != tetra_main_hz || (int)si->location_area != tetra_la) {
                tetra_main_hz = si->dl_freq_hz;
                tetra_la      = si->location_area;
                char f[16];
                fmt_mhz(f, sizeof(f), si->dl_freq_hz);
                log_event("main carrier %s MHz, location area %u, air encryption %s", f, si->location_area,
                          (si->bs_service_details & TETRA_RX_BSD_AIR_ENCRYPTION) ? "on" : "off");
            }
            break;
        }
        case TETRA_RX_EV_SIGNAL:
            tetra_signal(&e->u.signal);
            break;
        case TETRA_RX_EV_TRAFFIC: {
            // One event per traffic slot: two channel-decoded speech frames (or fewer, when half
            // slots were stolen for signalling).
            const tetra_rx_ev_traffic_t* t   = &e->u.traffic;
            const int                    tn  = (e->time.tn - 1) & 3;
            const int64_t                now = esp_timer_get_time();
            // Without signalling for this channel yet (tuned in during a call) the traffic counts
            // as clear only on a cell that broadcasts "no air encryption".
            const bool encrypted = t->enc_state == TETRA_RX_ENC_ENCRYPTED;
            const bool clear     = t->enc_state == TETRA_RX_ENC_CLEAR ||
                                   (t->enc_state == TETRA_RX_ENC_UNKNOWN && tetra_air_enc == 0);
            if (t->n_frames && now - tetra_tch_us[tn] > 2000000) {
                log_event("TN%d speech, %s%s", tn + 1, encrypted ? "encrypted" : clear ? "clear" : "encryption unknown",
                          (clear && !tetra_codec_available()) ? " (no speech codec in this build)" : "");
            }
            if (t->n_frames) tetra_tch_us[tn] = now;
            if (!clear || !tetra_codec_available()) break;

            // Only one timeslot goes to the speaker: the selected one, or the one that started first.
            // There is one speech decoder, so it starts afresh whenever the speaker changes slot.
            bool start;
            if (!voice_gate(tn + 1, g_set.tetra_slot, now, TETRA_SPURT_US, TETRA_SPURT_US, &start)) break;
            if (start) {
                static const int16_t silence[480];
                for (int i = 0; i < TETRA_PREFILL_MS / 10; i++) audio_push(silence, 480);
                tetra_codec_reset();
            }
            if (tetra_cell) rec_note_plmn(tetra_mcc, tetra_mnc);
            for (int f = 0; f < 2; f++) {
                // A stolen, missing or damaged frame is handed over as a bad frame: the decoder
                // then repeats and fades the last good one, and the timing stays intact.
                static int16_t pcm[TETRA_CODEC_FRAME_SAMPLES];
                const bool     bad = f >= t->n_frames || t->bfi[f] || t->stolen[f];
                tetra_codec_decode(t->frame[f], bad, pcm);
                speech_out(pcm, TETRA_CODEC_FRAME_SAMPLES);
            }
            break;
        }
        default:
            break;
    }
}

static const char* tetra_usage(int tn, uint8_t valid, uint8_t usage, uint8_t enc) {
    if (dec_speaker_slot() == tn) return "AUDIO";
    if (!valid) return "?";
    switch (usage) {
        case TETRA_RX_DL_UNALLOCATED:
            return "free";
        case TETRA_RX_DL_ASSIGNED_CCH:
            return "assigned";
        case TETRA_RX_DL_COMMON_CCH:
            return "control";
        case TETRA_RX_DL_RESERVED:
            return "reserved";
        default:
            return enc == TETRA_RX_ENC_ENCRYPTED ? "TRAFFIC (enc)" : "TRAFFIC";
    }
}

static unsigned pct(uint32_t ok, uint32_t fail) {
    return ok + fail ? (unsigned)((uint64_t)ok * 100u / (ok + fail)) : 0;
}

static void tetra_lines(char lines[DEC_LINES][DEC_LINE_LEN]) {
    if (tetra == NULL) {
        snprintf(lines[0], DEC_LINE_LEN, "TETRA: decoder could not be started (out of memory)");
        return;
    }
    tetra_rx_status_t st;
    uint32_t          clear, enc;
    xSemaphoreTake(lock, portMAX_DELAY);
    st    = tetra_st;
    clear = tetra_sig_clear;
    enc   = tetra_sig_enc;
    xSemaphoreGive(lock);
    const bool synced = st.locked && tetra_cell && st.cell_valid;
    char       sel[16] = "AUTO";
    if (g_set.tetra_slot) snprintf(sel, sizeof(sel), "TN%d", g_set.tetra_slot);

    if (!synced) {
        snprintf(lines[0], DEC_LINE_LEN, "TETRA  searching for a downlink carrier");
    } else if (!st.sysinfo_valid) {
        snprintf(lines[0], DEC_LINE_LEN, "TETRA  MCC %u MNC %u  colour code %u", st.mcc, st.mnc, st.colour_code);
    } else {
        char f[16];
        fmt_mhz(f, sizeof(f), st.sysinfo.dl_freq_hz);
        snprintf(lines[0], DEC_LINE_LEN, "TETRA  MCC %u MNC %u  CC %u  LA %u  main %s  air enc %s", st.mcc, st.mnc,
                 st.colour_code, st.sysinfo.location_area, f,
                 (st.sysinfo.bs_service_details & TETRA_RX_BSD_AIR_ENCRYPTION) ? "on" : "off");
    }
    if (synced) {
        uint32_t ok = 0, fail = 0;
        for (int i = TETRA_RX_CH_BSCH; i <= TETRA_RX_CH_STCH; i++) {
            ok   += st.chan[i].ok;
            fail += st.chan[i].fail;
        }
        snprintf(lines[1], DEC_LINE_LEN,
                 "SNR %.0f dB  offset %+.0f Hz  clock %+.0f ppm  CRC %u%%  AACH %u%%  PDUs %u clear, %u enc",
                 (double)st.evm_snr_db, (double)st.freq_offset_hz, (double)st.timing_ppm, pct(ok, fail),
                 pct(st.chan[TETRA_RX_CH_AACH].ok, st.chan[TETRA_RX_CH_AACH].fail), (unsigned)clear, (unsigned)enc);
        snprintf(lines[2], DEC_LINE_LEN, "TN1 %s  TN2 %s  TN3 %s  TN4 %s   audio: %s",
                 tetra_usage(1, st.slot[0].valid, st.slot[0].dl_usage, st.slot[0].enc_state),
                 tetra_usage(2, st.slot[1].valid, st.slot[1].dl_usage, st.slot[1].enc_state),
                 tetra_usage(3, st.slot[2].valid, st.slot[2].dl_usage, st.slot[2].enc_state),
                 tetra_usage(4, st.slot[3].valid, st.slot[3].dl_usage, st.slot[3].enc_state), sel);
    } else {
        snprintf(lines[1], DEC_LINE_LEN, "channel level %.0f dBFS", (double)st.level_dbfs);
        snprintf(lines[2], DEC_LINE_LEN, "audio: %s", sel);
    }
    char recent[2][DEC_LINE_LEN];
    int  n = dec_get_log(recent, 2);
    for (int i = 0; i < n; i++) memcpy(lines[3 + i], recent[i], DEC_LINE_LEN);
}

// ---- Public --------------------------------------------------------------------------------------

void dec_init(void) {
    lock = xSemaphoreCreateMutex();
    // 3.9 kHz cutoff at 48 kHz: passes speech, removes the images of the 8 kHz stream above 4.6 kHz.
    dsp_design_lowpass(up_h, UP_TAPS, 3900.0f / (float)AUDIO_RATE, 50.0f);

    dmr_rx_config_t cfg;
    dmr_rx_config_default(&cfg);
    cfg.event_cb = on_dmr_event;
    cfg.pcm_cb   = on_dmr_pcm;
    cfg.alloc    = dec_alloc;
    cfg.dealloc  = dec_free;
    dmr          = dmr_rx_create(&cfg);
    if (dmr == NULL) ESP_LOGE(TAG, "DMR decoder: out of memory (%u bytes)", (unsigned)dmr_rx_state_size());

    tetra_rx_config_t tcfg = {0};
    tcfg.event_cb          = on_tetra_event;
    tcfg.alloc             = dec_alloc;
    tcfg.free              = dec_free;
    tetra                  = tetra_rx_create(&tcfg);
    if (tetra == NULL) ESP_LOGE(TAG, "TETRA decoder: out of memory");
}

void dec_reset(demod_mode_t mode) {
    (void)mode;
    if (dmr) dmr_rx_reset(dmr);
    if (tetra) tetra_rx_reset(tetra);
    xSemaphoreTake(lock, portMAX_DELAY);
    memset(&tetra_st, 0, sizeof(tetra_st));
    tetra_sig_clear = tetra_sig_enc = 0;
    xSemaphoreGive(lock);
    tetra_main_hz = 0;
    tetra_cell    = false;
    tetra_air_enc = -1;
    for (int i = 0; i < 4; i++) tetra_tch_us[i] = -10000000;
    tetra_la      = -1;
    voice_slot    = 0;
    voice_last_us = 0;
    memset(up_x, 0, sizeof(up_x));
    log_clear();
}

void dec_dmr_feed(const float* fm, int n) {
    if (dmr == NULL || n <= 0) return;
    // A timeslot that is not heard does not need the vocoder either.
    static int applied = -1;
    int        wanted  = g_set.dmr_slot;
    if (wanted != applied) {
        dmr_rx_set_pcm_slot_mask(dmr, wanted ? (uint8_t)(1u << (wanted - 1)) : 3);
        applied = wanted;
    }
    dmr_rx_process_fm(dmr, fm, (size_t)n);
}

void dec_tetra_feed(const cf32_t* iq, int n) {
    if (tetra == NULL || n <= 0) return;
    _Static_assert(sizeof(cf32_t) == 2 * sizeof(float), "cf32_t must be an interleaved I/Q pair");
    tetra_rx_process_cf32(tetra, (const float*)iq, (size_t)n);
    static tetra_rx_status_t st;
    tetra_rx_get_status(tetra, &st);
    xSemaphoreTake(lock, portMAX_DELAY);
    tetra_st = st;
    xSemaphoreGive(lock);
}

int dec_tetra_constellation(cf32_t* iq, int max_symbols) {
    if (tetra == NULL || max_symbols <= 0) return 0;
    return (int)tetra_rx_get_constellation(tetra, (float*)iq, (size_t)max_symbols);
}

void dec_get_lines(demod_mode_t mode, char lines[DEC_LINES][DEC_LINE_LEN]) {
    switch (mode) {
        case MODE_NFM:
            snprintf(lines[0], DEC_LINE_LEN, "Narrow FM, 15 kHz filter");
            break;
        case MODE_AM:
            snprintf(lines[0], DEC_LINE_LEN, "AM, 10 kHz channel");
            break;
        case MODE_LSB:
        case MODE_USB:
            snprintf(lines[0], DEC_LINE_LEN, "%s sideband, audio 0.3 - 2.7 kHz, AGC",
                     mode == MODE_LSB ? "Lower" : "Upper");
            snprintf(lines[1], DEC_LINE_LEN, "The frequency shown is the suppressed carrier; UP / DOWN change the step.");
            break;
        case MODE_WFM:
            snprintf(lines[0], DEC_LINE_LEN, "Broadcast FM (mono)");
            break;
        case MODE_DMR:
            dmr_lines(lines);
            break;
        case MODE_TETRA:
            tetra_lines(lines);
            break;
        default:
            break;
    }
}
