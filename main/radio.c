// SPDX-License-Identifier: AGPL-3.0-only
// RTL-SDR ownership and the receive DSP chain.
//
//   RTL2832U CU8 IQ @ 1.152 MS/s, tuner LO parked fs/4 below the channel
//     -> stage 1: fs/4 downshift + integer FIR, /8 -> 144 kHz   (/4 -> 288 kHz for broadcast FM)
//     -> stage 2: /3 -> 48 kHz (NFM, AM, SSB, DMR)  or  /2 -> 72 kHz (TETRA, 4 samples/symbol)
//     -> channel filter -> demodulator / protocol decoder -> audio
//
// Below 24 MHz the driver switches a dongle that can do it (RTL-SDR Blog V3 and compatibles) to
// direct sampling: the RTL2832 digitises its Q input itself and the tuner, with its gain, is out
// of the path. Nothing changes in the chain: the "LO" is then the RTL2832's own down-converter.
//
// In the SSB modes the hardware is only retuned when the wanted frequency moves more than a few
// kHz away from where it sits; in between, the SSB oscillator takes up the difference. Tuning in
// 10 Hz steps is then exact and free of the gaps and PLL steps of a hardware retune.
//
// One task owns the dongle and runs the whole chain on core 1; the UI lives on core 0.

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "app.h"
#include "bsp/power.h"
#include "decoder.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rtl_sdr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char TAG[] = "radio";

#define READ_BYTES    16384
#define RAW_CARRY_MAX (STAGE1_MAX_TAPS + 8)
#define S1_OUT_MAX    (READ_BYTES / 2 / 4 + 32)
#define CH_OUT_MAX    (S1_OUT_MAX / 2 + 32)

app_settings_t g_set = {
    .freq_hz    = 391187500,
    .mode       = MODE_TETRA,
    .gain_idx   = 16,  // ~30 dB on an R820T; the tuner AGC ("auto") overdrives the ADC next to strong carriers
    .ppm        = 0,
    .squelch_db = SQUELCH_OFF,
    .volume     = 0,
    .zoom       = false,
};
app_status_t g_stat;
snapshot_t   g_snap;
scope_t      g_scope;

const char* const mode_names[MODE_COUNT] = {"NFM", "AM", "LSB", "USB", "WFM", "DMR", "TETRA"};

int mode_from_name(const char* name) {
    for (int i = 0; i < MODE_COUNT; i++) {
        if (strcasecmp(name, mode_names[i]) == 0) return i;
    }
    return -1;
}

// How far the SSB oscillator may tune away from the hardware frequency before that is moved.
#define SSB_SOFT_TUNE_HZ 5000

static esp_rtl_sdr_handle_t sdr;
static volatile bool        evt_disconnected;
static uint32_t             dev_caps;
static bool                 direct_mode;  // the tuner is bypassed at the current frequency
static bool                 rtl_agc_on;
static uint32_t             tuned_freq;   // what the chain demodulates ...
static int32_t              soft_off_hz;  // ... and how far that is from the hardware frequency

// DSP state
static stage1_t s1_narrow, s1_wide;
static cfir_t   f_48k, f_72k, f_chan;
static rfir_t   f_wfm_audio;
static ssb_t    ssb;
static fmdisc_t disc;
static biquad_t aud_lp, aud_hp;
static float    deemph_state, am_carrier;
static bool     squelch_open;

static uint8_t raw_buf[RAW_CARRY_MAX * 2 + READ_BYTES];
static int     raw_carry;  // complex samples carried over as FIR history
static cf32_t  s1_out[S1_OUT_MAX];
static cf32_t  ch_out[CH_OUT_MAX];
static cf32_t  chf_out[CH_OUT_MAX];
static float   fm_out[S1_OUT_MAX];
static float   aud_f[CH_OUT_MAX];
static int16_t aud_i[CH_OUT_MAX];
static int     snap_fill;
static uint8_t  adc_lo = 255, adc_hi = 0;
static uint32_t adc_clip, adc_seen;

static void set_status(radio_status_t st, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_stat.status_text, sizeof(g_stat.status_text), fmt, ap);
    va_end(ap);
    g_stat.status = st;
    ESP_LOGI(TAG, "%s", g_stat.status_text);
}

static void on_event(esp_rtl_sdr_event_t event, const void* payload, void* ctx) {
    (void)ctx;
    switch (event) {
        case ESP_RTL_SDR_EVT_DISCONNECTED:
            evt_disconnected = true;
            break;
        case ESP_RTL_SDR_EVT_ERROR: {
            const esp_rtl_sdr_error_info_t* e = payload;
            if (e) ESP_LOGW(TAG, "driver error %s: %s", esp_rtl_sdr_err_to_name(e->code), e->message);
            break;
        }
        default:
            break;
    }
}

static void dsp_setup(void) {
    // Stage 1 narrow: keep +/-28 kHz clean (zoom spectrum), reject everything that aliases into it.
    stage1_init(&s1_narrow, 40, 8, 72000.0f / (float)RTL_RATE, 60.0f);
    // Stage 1 wide: +/-100 kHz for broadcast FM.
    stage1_init(&s1_wide, 36, 4, 144000.0f / (float)RTL_RATE, 60.0f);
    bool ok  = cfir_init(&f_48k, 30, 3, S1_OUT_MAX);
    ok      &= cfir_init(&f_72k, 44, 2, S1_OUT_MAX);
    ok      &= cfir_init(&f_chan, 64, 1, CH_OUT_MAX);
    ok      &= rfir_init(&f_wfm_audio, 96, 6, S1_OUT_MAX);
    ok      &= ssb_init(&ssb, S1_OUT_MAX / 3 + 16);
    if (!ok) ESP_LOGE(TAG, "out of memory for DSP filters");
    cfir_design(&f_48k, 20000.0f / (float)IF_RATE_NARROW, 60.0f);
    cfir_design(&f_72k, 19000.0f / (float)IF_RATE_NARROW, 60.0f);
    rfir_design(&f_wfm_audio, 14000.0f / (float)IF_RATE_WIDE, 60.0f);
}

// What each mode takes out of the spectrum. The display marks this band, and the channel filter of
// NFM / AM / DMR is designed from it.
void mode_band_hz(demod_mode_t mode, int32_t* lo, int32_t* hi) {
    int32_t half;
    switch (mode) {
        case MODE_AM:
            half = 5000;
            break;
        case MODE_LSB:  // audio 0.3 .. 2.7 kHz below the carrier
            *lo = -2700;
            *hi = -300;
            return;
        case MODE_USB:  // ... and above it
            *lo = 300;
            *hi = 2700;
            return;
        case MODE_WFM:
            half = 100000;
            break;
        case MODE_DMR:
            half = 6250;
            break;
        case MODE_TETRA:  // 25 kHz channel: 18 kHz symbol rate, root raised cosine with 35 % roll-off
            half = 12500;
            break;
        case MODE_NFM:
        default:
            half = 7500;
            break;
    }
    *lo = -half;
    *hi = half;
}

static void dsp_set_mode(demod_mode_t mode) {
    float bw = 7500.0f;  // half width of the channel filter; only NFM, AM and DMR run through it
    if (mode == MODE_NFM || mode == MODE_AM || mode == MODE_DMR) {
        int32_t lo, hi;
        mode_band_hz(mode, &lo, &hi);
        bw = (float)hi;
    }
    switch (mode) {
        case MODE_AM:
            biquad_lowpass(&aud_lp, 4500.0f, AUDIO_RATE, 0.707f);
            biquad_highpass(&aud_hp, 80.0f, AUDIO_RATE, 0.707f);
            break;
        case MODE_DMR:
            break;
        case MODE_NFM:
        default:
            biquad_lowpass(&aud_lp, 3200.0f, AUDIO_RATE, 0.707f);
            biquad_highpass(&aud_hp, 250.0f, AUDIO_RATE, 0.707f);
            break;
    }
    cfir_design(&f_chan, bw / (float)CH_RATE_48K, 60.0f);
    cfir_reset(&f_48k);
    cfir_reset(&f_72k);
    rfir_reset(&f_wfm_audio);
    ssb_reset(&ssb);
    memset(&disc, 0, sizeof(disc));
    deemph_state = 0.0f;
    am_carrier   = 0.0f;
    raw_carry    = 0;
    snap_fill    = 0;
    squelch_open = false;
    dec_reset(mode);
}

static float block_power_db(const cf32_t* x, int n) {
    if (n <= 0) return -200.0f;
    float acc = 0.0f;
    for (int i = 0; i < n; i++) acc += x[i].re * x[i].re + x[i].im * x[i].im;
    return 10.0f * log10f(acc / (float)n + 1e-20f);
}

static void update_squelch(float db) {
    int thr = g_set.squelch_db;
    if (thr <= -120) {
        squelch_open = true;
    } else if (squelch_open) {
        if (db < (float)thr - 2.0f) squelch_open = false;
    } else if (db > (float)thr) {
        squelch_open = true;
    }
    g_stat.squelch_open = squelch_open;
}

static void emit_audio(int n) {
    for (int i = 0; i < n; i++) {
        float v = squelch_open ? aud_f[i] : 0.0f;
        if (v > 1.0f) v = 1.0f;
        if (v < -1.0f) v = -1.0f;
        aud_i[i] = (int16_t)(v * 30000.0f);
    }
    cap_feed_s16(TAP_AUDIO, aud_i, n, AUDIO_RATE);
    if (squelch_open) rec_feed(aud_i, n);  // a recording skips what the squelch mutes
    audio_push(aud_i, n);
}

static void scope_real(const float* x, int n) {
    if (n > SCOPE_LEN) n = SCOPE_LEN;
    memcpy(g_scope.a, x, sizeof(float) * (size_t)n);
    for (int i = n; i < SCOPE_LEN; i++) g_scope.a[i] = 0.0f;
    g_scope.is_iq = false;
    g_scope.seq++;
}

static void scope_iq(const cf32_t* x, int n) {
    if (n > SCOPE_LEN) n = SCOPE_LEN;
    for (int i = 0; i < n; i++) {
        g_scope.a[i] = x[i].re;
        g_scope.b[i] = x[i].im;
    }
    for (int i = n; i < SCOPE_LEN; i++) g_scope.a[i] = g_scope.b[i] = 0.0f;
    g_scope.is_iq = true;
    g_scope.seq++;
}

// Process freshly read raw IQ. new_bytes were appended after the carried history in raw_buf.
static int64_t dec_us;  // time spent inside the protocol decoder since the last statistics update

static void process_block(demod_mode_t mode, int new_bytes) {
    const uint8_t* fresh  = raw_buf + raw_carry * 2;
    int            total  = raw_carry + new_bytes / 2;
    bool           zoom   = g_set.zoom;
    uint32_t       if_rate = (mode == MODE_WFM) ? IF_RATE_WIDE : IF_RATE_NARROW;

    cap_feed_u8(TAP_RAW, fresh, (size_t)new_bytes, RTL_RATE);

    // ADC range, sampled sparsely (every 3rd byte alternates between I and Q).
    for (int i = 0; i < new_bytes; i += 3) {
        uint8_t v = fresh[i];
        if (v < adc_lo) adc_lo = v;
        if (v > adc_hi) adc_hi = v;
        adc_clip += (v == 0 || v == 255);
    }
    adc_seen += (uint32_t)(new_bytes + 2) / 3;

    // Wideband spectrum snapshot straight from the raw stream.
    if (g_snap.req && !zoom && new_bytes >= SNAP_LEN * 2) {
        memcpy(g_snap.raw, fresh, SNAP_LEN * 2);
        g_snap.is_zoom = false;
        g_snap.rate    = RTL_RATE;
        g_snap.freq_hz = tuned_freq;
        g_snap.off_hz  = soft_off_hz;
        g_snap.req     = false;  // before seq: the UI re-arms as soon as it sees the new sequence
        g_snap.seq++;
    }

    int consumed = 0;
    int n1       = stage1_run(mode == MODE_WFM ? &s1_wide : &s1_narrow, raw_buf, total, s1_out, &consumed);
    raw_carry    = total - consumed;
    memmove(raw_buf, raw_buf + consumed * 2, (size_t)raw_carry * 2);

    cap_feed_cf32(TAP_IF, s1_out, n1, if_rate);

    if (g_snap.req && zoom) {
        int take = SNAP_LEN - snap_fill;
        if (take > n1) take = n1;
        memcpy(g_snap.iq + snap_fill, s1_out, sizeof(cf32_t) * (size_t)take);
        snap_fill += take;
        if (snap_fill >= SNAP_LEN) {
            snap_fill      = 0;
            g_snap.is_zoom = true;
            g_snap.rate    = if_rate;
            g_snap.freq_hz = tuned_freq;
            g_snap.off_hz  = soft_off_hz;
            g_snap.req     = false;
            g_snap.seq++;
        }
    } else {
        snap_fill = 0;
    }

    if (mode == MODE_WFM) {
        float db = block_power_db(s1_out, n1);
        g_stat.chan_db = db;
        update_squelch(db);
        for (int i = 0; i < n1; i++) fm_out[i] = fmdisc_run(&disc, s1_out[i]);
        cap_feed_f32(TAP_FM, fm_out, n1, 32767.0f / 3.14159265f, IF_RATE_WIDE);
        scope_real(fm_out, n1);
        int na = rfir_run(&f_wfm_audio, fm_out, n1, aud_f);
        // 75 kHz deviation -> 1.636 rad/sample at 288 kHz; 50 us de-emphasis.
        for (int i = 0; i < na; i++) {
            deemph_state += 0.3408f * (aud_f[i] * (0.6f / 1.636f) - deemph_state);
            aud_f[i]      = deemph_state;
        }
        emit_audio(na);
        return;
    }

    if (mode == MODE_TETRA) {
        int n2 = cfir_run(&f_72k, s1_out, n1, ch_out);
        cap_feed_cf32(TAP_CH, ch_out, n2, CH_RATE_TETRA);
        g_stat.chan_db = block_power_db(ch_out, n2);
        int64_t td = esp_timer_get_time();
        dec_tetra_feed(ch_out, n2);
        dec_us += esp_timer_get_time() - td;
        // The scope shows what the detector decides on: one point per symbol, after the matched
        // filter and the symbol timing, as the phase step from the symbol before. The channel
        // samples themselves (four per symbol, carrier offset still on them) are only ever a blob.
        static cf32_t sym[SCOPE_LEN];
        scope_iq(sym, dec_tetra_constellation(sym, SCOPE_LEN));
        return;
    }

    int n2 = cfir_run(&f_48k, s1_out, n1, ch_out);
    cap_feed_cf32(TAP_CH, ch_out, n2, CH_RATE_48K);

    if (mode_is_ssb(mode)) {
        int na = ssb_run(&ssb, ch_out, n2, aud_f);
        if (na > 0) {
            g_stat.chan_db = ssb.level_db;
            update_squelch(ssb.level_db);
            scope_real(aud_f, na);
            emit_audio(na);
        }
        return;
    }

    int   n3 = cfir_run(&f_chan, ch_out, n2, chf_out);
    float db = block_power_db(chf_out, n3);
    g_stat.chan_db = db;
    update_squelch(db);

    if (mode == MODE_AM) {
        for (int i = 0; i < n3; i++) {
            float mag   = sqrtf(chf_out[i].re * chf_out[i].re + chf_out[i].im * chf_out[i].im);
            am_carrier += (mag - am_carrier) * (1.0f / 2400.0f);
            float v     = (mag - am_carrier) / (am_carrier + 1e-6f);
            v           = biquad_run(&aud_hp, biquad_run(&aud_lp, v));
            aud_f[i]    = v * 0.7f;
        }
        scope_real(aud_f, n3);
        emit_audio(n3);
        return;
    }

    for (int i = 0; i < n3; i++) fm_out[i] = fmdisc_run(&disc, chf_out[i]);
    cap_feed_f32(TAP_FM, fm_out, n3, 32767.0f / 3.14159265f, CH_RATE_48K);
    scope_real(fm_out, n3);

    if (mode == MODE_DMR) {
        int64_t td = esp_timer_get_time();
        dec_dmr_feed(fm_out, n3);
        dec_us += esp_timer_get_time() - td;
        return;
    }

    // NFM: 300 Hz de-emphasis corner, speech band-pass. 2.5 kHz deviation = 0.327 rad/sample.
    for (int i = 0; i < n3; i++) {
        deemph_state += 0.0385f * (fm_out[i] - deemph_state);
        float v       = biquad_run(&aud_hp, biquad_run(&aud_lp, deemph_state));
        aud_f[i]      = v * 6.0f;
    }
    emit_audio(n3);
}

// Direct sampling has no tuner gain; "auto" then stands for the RTL2832's digital AGC.
static void apply_gain(int idx) {
    esp_err_t err;
    if (dev_caps & ESP_RTL_SDR_CAP_RTL_AGC) {
        bool want = direct_mode && idx < 0;
        if (want != rtl_agc_on) {
            err = esp_rtl_sdr_set_rtl_agc(sdr, want);
            if (err == ESP_OK) rtl_agc_on = want;
            else ESP_LOGW(TAG, "RTL AGC %d failed: %s", (int)want, esp_rtl_sdr_err_to_name(err));
        }
    }
    g_stat.rtl_agc = rtl_agc_on;
    if (direct_mode) return;
    if (idx < 0 || idx >= g_stat.gain_count) {
        err = esp_rtl_sdr_set_tuner_gain_mode(sdr, ESP_RTL_SDR_GAIN_MODE_AUTO);
    } else {
        err = esp_rtl_sdr_set_tuner_gain(sdr, g_stat.gains[idx]);
    }
    if (err != ESP_OK) ESP_LOGW(TAG, "gain change failed: %s", esp_rtl_sdr_err_to_name(err));
}

static uint32_t lo_for(uint32_t channel_hz) {
    return channel_hz - RTL_LO_OFFSET;
}

static bool uses_direct_sampling(uint32_t channel_hz) {
    return (dev_caps & ESP_RTL_SDR_CAP_DIRECT_SAMPLING) && lo_for(channel_hz) < DIRECT_SAMPLING_BELOW_HZ;
}

static void status_streaming(void) {
    set_status(RADIO_STREAMING, "%s (%s)", g_stat.device_name, g_stat.profile_name);
}

static void status_cannot_tune(uint32_t hz, esp_err_t err) {
    if (err == ESP_RTL_SDR_ERR_BAD_FREQ) {
        set_status(RADIO_ERROR, "This dongle cannot tune to %u.%03u MHz%s", (unsigned)(hz / 1000000),
                   (unsigned)(hz % 1000000 / 1000),
                   hz < DIRECT_SAMPLING_BELOW_HZ ? " (no direct sampling)" : "");
    } else {
        set_status(RADIO_ERROR, "Tuning to %u.%03u MHz failed: %s", (unsigned)(hz / 1000000),
                   (unsigned)(hz % 1000000 / 1000), esp_rtl_sdr_err_to_name(err));
    }
}

// The ppm setting is the crystal error as rtl_test / gqrx / kalibrate report it (positive = crystal
// fast, signals show up too low). The driver instead adds ppm to the frequency it programs, so the
// sign is flipped here.
static int driver_ppm(int ppm) {
    return -ppm;
}

// started_freq / started_ppm: what the dongle was started with. The settings may have changed while
// it was starting; the loop then retunes right away.
static void stream_loop(uint32_t started_freq, int started_ppm) {
    uint32_t     hw_freq  = started_freq;  // channel frequency the hardware sits on
    uint32_t     cur_freq = started_freq;
    int          cur_ppm  = started_ppm;
    int          cur_gain = g_set.gain_idx;
    demod_mode_t cur_mode = g_set.mode;
    bool         tune_failed = false;
    int64_t      t_stat   = esp_timer_get_time();
    int64_t      busy_us  = 0;
    int          timeouts = 0;

    direct_mode        = uses_direct_sampling(hw_freq);
    rtl_agc_on         = false;
    g_stat.direct      = direct_mode;
    tuned_freq         = cur_freq;
    soft_off_hz        = 0;
    g_stat.soft_off_hz = 0;
    dsp_set_mode(cur_mode);
    ssb_tune(&ssb, cur_mode == MODE_LSB ? -1 : 1, 0.0f);
    apply_gain(cur_gain);

    while (!evt_disconnected) {
        // Apply setting changes between blocks; the driver pauses the bulk stream for EP0 itself.
        uint32_t     f   = g_set.freq_hz;
        int          ppm = g_set.ppm;
        demod_mode_t m   = g_set.mode;
        if (f != cur_freq || ppm != cur_ppm || m != cur_mode) {
            int32_t off  = (int32_t)(f - hw_freq);
            bool    hard = ppm != cur_ppm || tune_failed;
            if (mode_is_ssb(m)) hard |= off > SSB_SOFT_TUNE_HZ || off < -SSB_SOFT_TUNE_HZ;
            else hard |= off != 0;
            if (hard) {
                if (ppm != cur_ppm) esp_rtl_sdr_set_freq_correction(sdr, driver_ppm(ppm));
                esp_err_t err = esp_rtl_sdr_retune_hz(sdr, lo_for(f));
                if (err == ESP_OK) {
                    hw_freq = f;
                    if (tune_failed) status_streaming();
                    tune_failed = false;
                    bool direct = uses_direct_sampling(f);
                    if (direct != direct_mode) {
                        // The tuner was just switched out of or back into the signal path.
                        direct_mode   = direct;
                        g_stat.direct = direct;
                        apply_gain(cur_gain);
                    }
                } else {
                    ESP_LOGW(TAG, "retune %u failed: %s", (unsigned)f, esp_rtl_sdr_err_to_name(err));
                    status_cannot_tune(f, err);
                    tune_failed = true;
                }
                // Drop samples that were still queued from the old frequency.
                size_t got = 0;
                for (int i = 0; i < 64; i++) {
                    if (esp_rtl_sdr_read(sdr, raw_buf, READ_BYTES, 0, &got) != ESP_OK || got == 0) break;
                }
            }
            if (hard || m != cur_mode) dsp_set_mode(m);
            cur_freq           = f;
            cur_ppm            = ppm;
            cur_mode           = m;
            tuned_freq         = f;
            soft_off_hz        = tune_failed ? 0 : (int32_t)(f - hw_freq);
            g_stat.soft_off_hz = soft_off_hz;
            ssb_tune(&ssb, m == MODE_LSB ? -1 : 1, (float)soft_off_hz);
        }
        int gi = g_set.gain_idx;
        if (gi != cur_gain) {
            apply_gain(gi);
            cur_gain = gi;
        }

        size_t    got = 0;
        esp_err_t err = esp_rtl_sdr_read(sdr, raw_buf + raw_carry * 2, READ_BYTES, 100, &got);
        if (err != ESP_OK || got == 0) {
            if (err != ESP_OK && err != ESP_RTL_SDR_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "read failed: %s", esp_rtl_sdr_err_to_name(err));
                break;
            }
            if (++timeouts > 30) {
                ESP_LOGW(TAG, "no samples for 3 s, restarting the stream");
                break;
            }
            continue;
        }
        timeouts = 0;

        int64_t t0 = esp_timer_get_time();
        if (g_cap.state == CAP_ARMED && g_stat.chan_db >= g_cap.trig_db) {
            g_cap.freq_hz = cur_freq;
            g_cap.mode    = cur_mode;
            g_cap.state   = CAP_RECORDING;
        }
        process_block(cur_mode, (int)got);
        g_stat.blocks++;
        int64_t t1 = esp_timer_get_time();
        busy_us   += t1 - t0;

        if (t1 - t_stat >= 1000000) {
            esp_rtl_sdr_metrics_t mt;
            if (esp_rtl_sdr_get_metrics(sdr, &mt) == ESP_OK) {
                g_stat.effective_sps = mt.effective_sps;
                g_stat.drops         = mt.overruns + mt.consumer_drops + mt.ring_overrun_blocks + mt.slot_starve_blocks;
            }
            g_stat.adc_min       = adc_lo;
            g_stat.adc_max       = adc_hi;
            g_stat.clip_permille = adc_seen ? (uint32_t)((uint64_t)adc_clip * 1000 / adc_seen) : 0;
            adc_lo               = 255;
            adc_hi               = 0;
            adc_clip = adc_seen = 0;
            g_stat.cpu_permille = (uint32_t)(busy_us * 1000 / (t1 - t_stat));
            g_stat.dec_permille = (uint32_t)(dec_us * 1000 / (t1 - t_stat));
            dec_us              = 0;
            busy_us             = 0;
            t_stat              = t1;
        }
    }
}

static void usb_reserve_release(void);

static void radio_task(void* arg) {
    (void)arg;
    dsp_setup();

    // The USB-A port is unpowered until the 5 V boost converter is switched on.
    esp_err_t err = bsp_power_set_usb_host_boost_enabled(true);
    if (err != ESP_OK) ESP_LOGW(TAG, "could not enable USB host power: %s", esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(300));

    esp_rtl_sdr_config_t cfg;
    esp_rtl_sdr_config_default(&cfg);
    cfg.event_cb         = on_event;
    cfg.delivery_mode    = ESP_RTL_SDR_DELIVERY_READ;
    cfg.transfer_bytes   = 16384;
    cfg.transfer_count   = 6;
    cfg.usb_task_core_id = 0;
    err                  = esp_rtl_sdr_install(&cfg, &sdr);
    if (err != ESP_OK || sdr == NULL) {
        set_status(RADIO_ERROR, "USB host start failed: %s", esp_rtl_sdr_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "esp_rtl_sdr %s installed", esp_rtl_sdr_get_version_string());

    for (;;) {
        set_status(RADIO_NO_DEVICE, "No RTL-SDR on the USB-A port");
        size_t count = 0;
        while (count == 0) {
            (void)esp_rtl_sdr_refresh_device_list(sdr);
            (void)esp_rtl_sdr_get_device_count(sdr, &count);
            if (count == 0) vTaskDelay(pdMS_TO_TICKS(500));
        }

        set_status(RADIO_STARTING, "Starting RTL-SDR...");
        evt_disconnected = false;
        const uint32_t start_freq = g_set.freq_hz;
        const int      start_ppm  = g_set.ppm;
        (void)esp_rtl_sdr_set_freq_correction(sdr, driver_ppm(start_ppm));
        esp_rtl_sdr_stream_config_t st;
        esp_rtl_sdr_stream_config_default(&st);
        st.preset          = ESP_RTL_SDR_PRESET_CUSTOM_HZ;
        st.frequency_hz    = lo_for(start_freq);
        st.sample_rate_sps = RTL_RATE;
        usb_reserve_release();
        err                = esp_rtl_sdr_start(sdr, &st);
        if (err != ESP_OK) {
            // A frequency this dongle cannot do is no reason to stay down: tune elsewhere and it starts.
            if (err == ESP_RTL_SDR_ERR_BAD_FREQ) status_cannot_tune(start_freq, err);
            else set_status(RADIO_ERROR, "Dongle start failed: %s", esp_rtl_sdr_err_to_name(err));
            for (int i = 0; i < 30 && g_set.freq_hz == start_freq; i++) vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        dev_caps = esp_rtl_sdr_get_device_capabilities(sdr);

        esp_rtl_sdr_device_info_t info;
        if (esp_rtl_sdr_get_device_info(sdr, &info) == ESP_OK) {
            snprintf(g_stat.device_name, sizeof(g_stat.device_name), "%.24s %.30s", info.manufacturer, info.product);
        }
        snprintf(g_stat.profile_name, sizeof(g_stat.profile_name), "%s",
                 esp_rtl_sdr_profile_to_name(esp_rtl_sdr_get_profile(sdr)));
        size_t ngains = 0;
        if (esp_rtl_sdr_get_tuner_gains(sdr, g_stat.gains, MAX_GAINS, &ngains) != ESP_OK) ngains = 0;
        g_stat.gain_count = (int)ngains;
        status_streaming();

        stream_loop(start_freq, start_ppm);

        err = esp_rtl_sdr_stop(sdr, 3000);
        if (err != ESP_OK) ESP_LOGW(TAG, "stop: %s", esp_rtl_sdr_err_to_name(err));
        g_stat.effective_sps = 0;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// The driver needs six 16 kB DMA buffers from internal RAM and only asks for them once the dongle
// is there, long after everything else has taken its share and left the heap in pieces too small
// for them. Their room is set aside before the rest of the app starts and handed back just before
// the driver wants it.
#define USB_RESERVE_COUNT 6
#define USB_RESERVE_BYTES (16384 + 512)
static void* usb_reserve[USB_RESERVE_COUNT];

void radio_reserve_memory(void) {
    for (int i = 0; i < USB_RESERVE_COUNT; i++) {
        usb_reserve[i] = heap_caps_aligned_alloc(64, USB_RESERVE_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (usb_reserve[i] == NULL) ESP_LOGW(TAG, "USB buffer reservation %d failed", i);
    }
}

static void usb_reserve_release(void) {
    for (int i = 0; i < USB_RESERVE_COUNT; i++) {
        heap_caps_free(usb_reserve[i]);
        usb_reserve[i] = NULL;
    }
}

void radio_start(void) {
    xTaskCreatePinnedToCore(radio_task, "radio_dsp", 16384, NULL, 17, NULL, 1);
}
