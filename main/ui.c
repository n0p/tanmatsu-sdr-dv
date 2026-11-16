// SPDX-License-Identifier: AGPL-3.0-only
// Display: spectrum, waterfall, status panel and a small scope / constellation widget.
//
// The pax buffers wrap the MIPI-DSI panel's own two framebuffers, so presenting a frame is a flip.
// The panel is 480x800 RGB888 (bytes B,G,R) mounted rotated: logical pixel (lx,ly) of the 800x480
// landscape view lives at raw index lx*480 + (479-ly). A logical column is therefore one contiguous
// run of memory, bottom pixel first, and everything here draws column by column.
//
// Text goes through pax into small cached label buffers with the same rotated layout; they are only
// re-rendered when their string changes and only pasted into a framebuffer that lacks them.

#include "ui.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "app.h"
#include "bsp/display.h"
#include "decoder.h"
#include "esp_heap_caps.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "pax_fonts.h"
#include "pax_gfx.h"
#include "pax_text.h"

static const char TAG[] = "ui";

#define LW    800
#define LH    480
#define RAW_W 480

#define TOP_H    28
#define SPEC_Y0  28
#define SPEC_Y1  155
#define SPEC_H   (SPEC_Y1 - SPEC_Y0 + 1)
#define SCALE_Y  156
#define SCALE_H  16
#define WF_Y0    172
#define WF_H     150
#define PANEL_Y  322
#define LINE_H   20
#define PANEL_N  (2 + DEC_LINES)
#define PANEL_W  600
#define WID_X    600
#define WID_W    200
#define WID_Y    322
#define WID_H    140
#define HELP_Y   462
#define HELP_H   18

#define COL_BG       0x05080c
#define COL_GRID     0x18222e
#define COL_CENTER   0xff4a4a  // tuned frequency (the carrier, in the SSB modes)
#define COL_CENTER_F 0x8a3030  // ... where it runs through the filled part of the spectrum
#define COL_BAND     0x2a2208  // demodulated band: background,
#define COL_BAND_GR  0x4a3c12  // grid lines,
#define COL_BAND_F   0x7a5c14  // the spectrum inside it,
#define COL_BAND_E   0xffc040  // and its edges,
#define COL_BAND_EF  0xc8962a  // which stay visible under the spectrum line
#define COL_FILL     0x0c3554
#define COL_LINE     0x6fd8ff
#define COL_WID_BG   0x080c12
#define COL_WID_AXIS 0x1c2836
#define COL_WID_MARK 0x4a6480
#define COL_TRACE    0x7dff8a

#define PAX_BAR_BG 0xff101820
#define PAX_PANEL  0xff05080c

typedef struct {
    pax_buf_t buf;
    uint8_t*  px;
    int       x, y, w, h;
    uint8_t   dirty;  // bit n: framebuffer n does not have the current content
    char      key[160];
} label_t;

static pax_buf_t fb[2];
static uint8_t*  fbp[2];
static int       fb_cur;
static size_t    disp_h_res, disp_v_res;
static bool      endian_big;

static label_t lbl_top, lbl_scale, lbl_help, lbl_line[PANEL_N];
static label_t lbl_keys;  // F4: the list of keys, over the spectrum and waterfall; made when first asked for
static label_t lbl_msgs;  // F5: the received messages, in the same place

static cf32_t   fftbuf[DSP_FFT_MAX];
static float    spec_db[LW];
static uint8_t  spec_y[LW];
static bool     tick_col[LW];
static uint8_t* wf;  // [x * WF_H + row], palette indexes
static int      wf_head;
static uint8_t  pal[256][3];
static float    floor_db  = -90.0f;
static bool     floor_set = false;
static float    noise_dbhz;  // noise floor as a density, so that it means the same in both views
static bool     noise_set = false;
static float    snr_db;      // demodulated band over that floor
static float    view_hz_per_px = (float)RTL_RATE / (float)DSP_FFT_MAX;
static uint32_t view_freq;
static int      band_x0 = LW / 2, band_x1 = LW / 2;  // columns of the demodulated band (may lie off screen)

char          ui_entry[20];
volatile bool ui_entry_active;
volatile float g_ui_fps;

// ---- raw drawing ---------------------------------------------------------------------------------

static inline uint8_t* col_ptr(uint8_t* base, int lx, int ly_bottom) {
    return base + ((size_t)lx * RAW_W + (size_t)(RAW_W - 1 - ly_bottom)) * 3;
}

static inline void put(uint8_t* p, uint32_t rgb) {
    p[0] = (uint8_t)rgb;
    p[1] = (uint8_t)(rgb >> 8);
    p[2] = (uint8_t)(rgb >> 16);
}

static void fill_rect(uint8_t* base, int x, int y, int w, int h, uint32_t rgb) {
    for (int lx = x; lx < x + w; lx++) {
        uint8_t* p = col_ptr(base, lx, y + h - 1);
        for (int i = 0; i < h; i++, p += 3) put(p, rgb);
    }
}

static inline void plot(uint8_t* base, int lx, int ly, uint32_t rgb) {
    if ((unsigned)lx >= LW || (unsigned)ly >= LH) return;
    put(col_ptr(base, lx, ly), rgb);
}

// ---- labels --------------------------------------------------------------------------------------

static bool label_init(label_t* l, int x, int y, int w, int h) {
    memset(l, 0, sizeof(*l));
    pax_buf_init(&l->buf, NULL, h, w, PAX_BUF_24_888RGB);
    pax_buf_reversed(&l->buf, endian_big);
    pax_buf_set_orientation(&l->buf, PAX_O_ROT_CW);
    l->px = (uint8_t*)pax_buf_get_pixels(&l->buf);
    l->x  = x;
    l->y  = y;
    l->w  = w;
    l->h  = h;
    l->key[0] = 1;  // never equal to a real string
    l->key[1] = 0;
    return l->px != NULL;
}

// Returns true when the label has to be redrawn for this key.
static bool label_changed(label_t* l, const char* key) {
    if (strncmp(l->key, key, sizeof(l->key) - 1) == 0) return false;
    snprintf(l->key, sizeof(l->key), "%.*s", (int)sizeof(l->key) - 1, key);
    return true;
}

static void label_done(label_t* l) {
    pax_join();
    l->dirty = 3;
}

static void label_paste(label_t* l, int fbi) {
    if (!(l->dirty & (1u << fbi)) || !l->px) return;
    size_t   run = (size_t)l->h * 3;
    uint8_t* src = l->px;
    for (int lx = 0; lx < l->w; lx++, src += run) {
        memcpy(col_ptr(fbp[fbi], l->x + lx, l->y + l->h - 1), src, run);
    }
    l->dirty &= (uint8_t)~(1u << fbi);
}

static void label_text(label_t* l, pax_col_t bg, pax_col_t fg, const pax_font_t* font, float size, float x, float y,
                       const char* text) {
    if (!label_changed(l, text)) return;
    pax_background(&l->buf, bg);
    pax_draw_text(&l->buf, fg, font, size, x, y, text);
    label_done(l);
}

// ---- spectrum ------------------------------------------------------------------------------------

static void build_palette(void) {
    // black -> blue -> cyan -> yellow -> red -> white
    static const struct {
        float   pos;
        uint8_t r, g, b;
    } stops[] = {
        {0.00f, 0, 0, 8},     {0.20f, 0, 16, 96},   {0.42f, 0, 150, 200},
        {0.62f, 230, 220, 40}, {0.82f, 235, 40, 20}, {1.00f, 255, 255, 255},
    };
    for (int i = 0; i < 256; i++) {
        float t = (float)i / 255.0f;
        int   s = 0;
        while (s < 4 && t > stops[s + 1].pos) s++;
        float u   = (t - stops[s].pos) / (stops[s + 1].pos - stops[s].pos);
        pal[i][0] = (uint8_t)((float)stops[s].b + u * (float)(stops[s + 1].b - stops[s].b));
        pal[i][1] = (uint8_t)((float)stops[s].g + u * (float)(stops[s + 1].g - stops[s].g));
        pal[i][2] = (uint8_t)((float)stops[s].r + u * (float)(stops[s + 1].r - stops[s].r));
    }
}

#define SPEC_RANGE_DB 70.0f
#define WF_RANGE_DB   45.0f

static void spectrum_update(void) {
    static uint32_t last_seq = 0;
    static int64_t  last_req = 0;
    int64_t         now      = esp_timer_get_time();
    if (g_snap.seq == last_seq) {
        // Re-arm if a request got lost (zoom toggled mid-request, stream restarted, ...).
        if (now - last_req > 300000) {
            g_snap.req = true;
            last_req   = now;
        }
        return;
    }
    last_seq   = g_snap.seq;
    bool zoom  = g_snap.is_zoom;
    float rate = (float)g_snap.rate;
    view_freq  = g_snap.freq_hz;
    if (zoom) {
        for (int i = 0; i < DSP_FFT_MAX; i++) {
            fftbuf[i].re = g_snap.iq[i].re * dsp_hann[i];
            fftbuf[i].im = g_snap.iq[i].im * dsp_hann[i];
        }
    } else {
        for (int i = 0; i < DSP_FFT_MAX; i++) {
            float w      = dsp_hann[i] * (1.0f / 127.5f);
            fftbuf[i].re = ((float)g_snap.raw[2 * i] - 127.5f) * w;
            fftbuf[i].im = ((float)g_snap.raw[2 * i + 1] - 127.5f) * w;
        }
    }
    g_snap.req = true;  // the snapshot is copied, the DSP task may take the next one
    last_req   = now;

    dsp_fft(fftbuf, DSP_FFT_MAX);
    view_hz_per_px = rate / (float)DSP_FFT_MAX;

    // A full-scale sine reads 0 dB (Hann coherent gain 0.5). In the wideband view the channel is
    // at +fs/4 of the raw stream, in the zoomed view the stream is already centred.
    // In the SSB modes the tuned frequency may sit a few kHz off that position (see radio.c); the
    // view follows it, so the centre line is always the tuned frequency.
    const float norm   = 1.0f / ((float)(DSP_FFT_MAX / 2) * (float)(DSP_FFT_MAX / 2));
    int         center = (zoom ? 0 : DSP_FFT_MAX / 4) + (int)lroundf((float)g_snap.off_hz / view_hz_per_px);
    static uint16_t hist[128];
    memset(hist, 0, sizeof(hist));
    static float row[LW];
    float        band_sum  = 0.0f;  // the demodulated band, as far as it is on screen
    int          band_cols = 0;
    for (int x = 0; x < LW; x++) {
        int   bin = (center + x - LW / 2) & (DSP_FFT_MAX - 1);
        float p   = fftbuf[bin].re * fftbuf[bin].re + fftbuf[bin].im * fftbuf[bin].im;
        if (x >= band_x0 && x <= band_x1) {
            band_sum += p * norm;
            band_cols++;
        }
        float db  = 10.0f * log10f(p * norm + 1e-14f);
        row[x]    = db;
        int h     = (int)(-db);
        if (h < 0) h = 0;
        if (h > 127) h = 127;
        hist[h]++;
    }
    // Noise floor: the level that 15 % of the bins are below.
    int acc = 0, h = 127;
    for (; h > 0; h--) {
        acc += hist[h];
        if (acc >= LW * 15 / 100) break;
    }
    // Signal over noise for the LED. The bins of one spectrum of noise scatter widely: their mean
    // power lies well above the level that 15 % of them are below, 7.9 dB in theory and 6 dB as
    // measured here on an empty channel (window overlap, 1 dB histogram steps). Where the band
    // fills most of the screen there is no noise beside it to measure, and the last value is kept.
    float bin_hz_db = 10.0f * log10f(view_hz_per_px);
    if (band_cols < LW * 6 / 10 || !noise_set) {
        float n = -(float)h + 6.0f - bin_hz_db;
        if (noise_set) noise_dbhz += 0.08f * (n - noise_dbhz);
        else noise_dbhz = n;
    }
    if (band_cols > 0) {
        float s = 10.0f * log10f(band_sum / (float)band_cols + 1e-14f) - bin_hz_db - noise_dbhz;
        if (noise_set) snr_db += 0.15f * (s - snr_db);
        else snr_db = s;
        noise_set = true;
    }

    float target = -(float)h - 8.0f;
    if (!floor_set) {
        floor_db  = target;
        floor_set = true;
        for (int x = 0; x < LW; x++) spec_db[x] = row[x];
    } else {
        floor_db += 0.08f * (target - floor_db);
    }

    wf_head = (wf_head + 1) % WF_H;
    for (int x = 0; x < LW; x++) {
        spec_db[x] += 0.45f * (row[x] - spec_db[x]);
        float v     = (spec_db[x] - floor_db) * ((float)(SPEC_H - 1) / SPEC_RANGE_DB);
        if (v < 0.0f) v = 0.0f;
        if (v > (float)(SPEC_H - 1)) v = (float)(SPEC_H - 1);
        spec_y[x] = (uint8_t)(SPEC_Y1 - (int)v);
        float c   = (row[x] - floor_db - 4.0f) * (255.0f / WF_RANGE_DB);
        if (c < 0.0f) c = 0.0f;
        if (c > 255.0f) c = 255.0f;
        if (wf) wf[x * WF_H + wf_head] = (uint8_t)c;
    }
}

bool ui_snr_db(float* snr) {
    *snr = snr_db;
    return noise_set;
}

// The band the current mode demodulates, in screen columns. The tuned frequency is always column
// LW / 2. Never narrower than five columns, so that it stays a band in the wide view too.
static void band_update(void) {
    int32_t lo, hi;
    mode_band_hz(g_set.mode, &lo, &hi);
    int x0 = LW / 2 + (int)lroundf((float)lo / view_hz_per_px);
    int x1 = LW / 2 + (int)lroundf((float)hi / view_hz_per_px);
    if (x1 - x0 < 4) {
        // Grow away from the carrier line if the band lies to one side of it (SSB).
        if (lo > 0) x1 = x0 + 4;
        else if (hi < 0) x0 = x1 - 4;
        else x0 = LW / 2 - 2, x1 = LW / 2 + 2;
    }
    band_x0 = x0;
    band_x1 = x1;
}

static void draw_spectrum(uint8_t* base) {
    // Horizontal grid every 10 dB.
    static bool grid_row[LH];
    static bool grid_ready = false;
    if (!grid_ready) {
        for (int db = 10; db < (int)SPEC_RANGE_DB; db += 10) {
            grid_row[SPEC_Y1 - (int)((float)db * (float)(SPEC_H - 1) / SPEC_RANGE_DB)] = true;
        }
        grid_ready = true;
    }
    int prev = spec_y[0];
    for (int x = 0; x < LW; x++) {
        int y  = spec_y[x];
        int lo = y < prev ? y : prev;  // top of the line segment
        int hi = y > prev ? y : prev;
        prev   = y;
        // The demodulated band: tinted, with bright edges and a bar along the top. The tuned
        // frequency is a line of its own; in the SSB modes it is the carrier, beside the band.
        bool     in_band = x >= band_x0 && x <= band_x1;
        bool     edge    = x == band_x0 || x == band_x1;
        uint32_t bg      = in_band ? COL_BAND : (tick_col[x] ? COL_GRID : COL_BG);
        uint32_t grid    = in_band ? COL_BAND_GR : COL_GRID;
        uint32_t fill    = in_band ? COL_BAND_F : COL_FILL;
        if (edge) {
            bg = grid = COL_BAND_E;
            fill      = COL_BAND_EF;
        }
        if (x == LW / 2) {
            bg = grid = COL_CENTER;
            fill      = COL_CENTER_F;
        }
        uint8_t* p = col_ptr(base, x, SPEC_Y1);
        for (int ly = SPEC_Y1; ly >= SPEC_Y0; ly--, p += 3) {
            uint32_t c;
            if (ly > hi) {
                c = fill;
            } else if (ly >= lo) {
                c = COL_LINE;
            } else if (in_band && ly < SPEC_Y0 + 3) {
                c = COL_BAND_E;
            } else {
                c = grid_row[ly] ? grid : bg;
            }
            put(p, c);
        }
    }
}

static void draw_waterfall(uint8_t* base) {
    if (!wf) return;
    int oldest = (wf_head + 1) % WF_H;
    for (int x = 0; x < LW; x++) {
        const uint8_t* col = wf + (size_t)x * WF_H;
        uint8_t*       p   = col_ptr(base, x, WF_Y0 + WF_H - 1);
        int            r   = oldest;
        // The newest row is drawn at the top, so the picture scrolls down.
        for (int k = 0; k < WF_H; k++, p += 3) {
            const uint8_t* c = pal[col[r]];
            p[0] = c[0];
            p[1] = c[1];
            p[2] = c[2];
            if (++r == WF_H) r = 0;
        }
    }
    // The edges of the demodulated band, blended in so that the signal under them stays visible.
    for (int e = 0; e < 2; e++) {
        int x = e ? band_x1 : band_x0;
        if (x < 0 || x >= LW) continue;
        uint8_t* p = col_ptr(base, x, WF_Y0 + WF_H - 1);
        for (int k = 0; k < WF_H; k++, p += 3) {
            p[0] = (uint8_t)((p[0] + (COL_BAND_E & 0xff)) >> 1);
            p[1] = (uint8_t)((p[1] + ((COL_BAND_E >> 8) & 0xff)) >> 1);
            p[2] = (uint8_t)((p[2] + ((COL_BAND_E >> 16) & 0xff)) >> 1);
        }
    }
}

static void update_scale(void) {
    char key[64];
    snprintf(key, sizeof(key), "%u/%d/%d/%d", (unsigned)view_freq, (int)(view_hz_per_px * 16.0f), band_x0, band_x1);
    if (!label_changed(&lbl_scale, key)) return;
    pax_background(&lbl_scale.buf, 0xff0a1018);
    memset(tick_col, 0, sizeof(tick_col));
    float span = view_hz_per_px * (float)LW;
    int   tick = span > 300000.0f ? 50000 : (span > 80000.0f ? 12500 : 6250);
    // Ticks sit on absolute multiples of the tick spacing.
    int64_t f_left = (int64_t)view_freq - (int64_t)(span * 0.5f);
    int64_t f      = (f_left / tick + 1) * tick;
    for (; (float)(f - f_left) < span; f += tick) {
        int x = (int)((float)(f - f_left) / view_hz_per_px);
        if (x < 0 || x >= LW) continue;
        tick_col[x] = true;
        pax_draw_line(&lbl_scale.buf, 0xff607080, (float)x + 0.5f, 0, (float)x + 0.5f, 3);
        char txt[20];
        if (tick >= 50000) {
            snprintf(txt, sizeof(txt), "%u.%02u", (unsigned)(f / 1000000), (unsigned)(f % 1000000 / 10000));
        } else {
            snprintf(txt, sizeof(txt), "%u.%05u", (unsigned)(f / 1000000), (unsigned)(f % 1000000 / 10));
        }
        float tw = (float)strlen(txt) * 7.0f;
        float tx = (float)x - tw * 0.5f;
        if (tx < 1.0f || tx + tw > (float)LW - 1.0f) continue;
        pax_draw_text(&lbl_scale.buf, 0xff9fb4c8, pax_font_sky_mono, 9, tx, 5, txt);
    }
    // The demodulated band as a bar, and the tuned frequency.
    int bx0 = band_x0 < 0 ? 0 : band_x0, bx1 = band_x1 >= LW ? LW - 1 : band_x1;
    if (bx1 >= bx0) pax_draw_rect(&lbl_scale.buf, 0xff000000 | COL_BAND_E, (float)bx0, 0, (float)(bx1 - bx0 + 1), 3);
    pax_draw_rect(&lbl_scale.buf, 0xff000000 | COL_CENTER, (float)(LW / 2 - 1), 0, 3, 6);
    label_done(&lbl_scale);
}

// ---- text ----------------------------------------------------------------------------------------

static const char* step_name(uint32_t hz, char* buf, size_t len) {
    if (hz >= 1000000) {
        snprintf(buf, len, "%uM", (unsigned)(hz / 1000000));
    } else if (hz >= 1000 && hz % 1000 == 0) {
        snprintf(buf, len, "%uk", (unsigned)(hz / 1000));
    } else if (hz >= 1000) {
        snprintf(buf, len, "%u.%02uk", (unsigned)(hz / 1000), (unsigned)(hz % 1000 / 10));
    } else {
        snprintf(buf, len, "%uHz", (unsigned)hz);
    }
    return buf;
}

// The battery at the right end of the top bar: a small cell filled to the level, the percentage
// under it, both red when empty, through yellow, to green when full. Without a battery (or a
// reading) the cell is empty and the text "--".
#define BAT_RIGHT 799  // right edge of the cell's cap and of the text
#define BAT_BODY  22
#define BAT_FILL  (BAT_BODY - 4)

static pax_col_t battery_colour(int pct) {
    float t = (float)pct / 100.0f;
    float r = t < 0.5f ? 1.0f : 2.0f - 2.0f * t;
    float g = t < 0.5f ? 2.0f * t : 1.0f;
    return 0xff000040 | ((uint32_t)(0x40 + lroundf(0xbf * r)) << 16) | ((uint32_t)(0x40 + lroundf(0xbf * g)) << 8);
}

static void draw_battery(pax_buf_t* b, int pct) {
    const pax_col_t frame = 0xff8090a0;
    float           x     = BAT_RIGHT - 2 - BAT_BODY;
    pax_draw_rect(b, frame, x, 3, BAT_BODY, 1);
    pax_draw_rect(b, frame, x, 11, BAT_BODY, 1);
    pax_draw_rect(b, frame, x, 3, 1, 9);
    pax_draw_rect(b, frame, x + BAT_BODY - 1, 3, 1, 9);
    pax_draw_rect(b, frame, x + BAT_BODY, 6, 2, 3);
    char txt[16];
    if (pct < 0) {
        snprintf(txt, sizeof(txt), "--");
    } else {
        // Two pixels at least, so that an almost empty battery still shows its red.
        int w = (BAT_FILL * pct + 50) / 100;
        pax_draw_rect(b, battery_colour(pct), x + 2, 5, w < 2 ? 2 : w, 5);
        snprintf(txt, sizeof(txt), "%d%%", pct);
    }
    pax_draw_text(b, pct < 0 ? frame : battery_colour(pct), pax_font_sky_mono, 9, BAT_RIGHT - 7.0f * (float)strlen(txt), 14,
                  txt);
}

static void update_top(void) {
    char     freq[32], gain[24], step[16], vol[16], rec[16], key[160];
    uint32_t f    = g_set.freq_hz;
    int      gi   = g_set.gain_idx;
    int      bat  = g_bat.pct;
    bool     entry = ui_entry_active;
    if (entry) {
        snprintf(freq, sizeof(freq), "> %s_", ui_entry);
    } else {
        snprintf(freq, sizeof(freq), "%u.%06u", (unsigned)(f / 1000000), (unsigned)(f % 1000000));
    }
    if (g_stat.direct) {
        snprintf(gain, sizeof(gain), g_stat.rtl_agc ? "direct AGC" : "direct");
    } else if (gi >= 0 && gi < g_stat.gain_count) {
        snprintf(gain, sizeof(gain), "RF %d.%d dB", g_stat.gains[gi] / 10, g_stat.gains[gi] % 10);
    } else {
        snprintf(gain, sizeof(gain), "RF auto");
    }
    step_name(step_table[g_step_idx], step, sizeof(step));
    if (g_set.volume > 0) {
        snprintf(vol, sizeof(vol), "VOL %d", g_set.volume);
    } else {
        snprintf(vol, sizeof(vol), "MUTED");
    }
    // Recording: the length of the audio in the file, which stands still while there is nothing to record.
    rec_state_t rs = g_rec.state;
    uint32_t    rsec = g_rec.ms / 1000;
    if (rs == REC_IDLE) rec[0] = 0;
    else if (rs == REC_STARTING) snprintf(rec, sizeof(rec), "...");
    else if (rsec >= 3600) snprintf(rec, sizeof(rec), "%uh%02u", (unsigned)(rsec / 3600), (unsigned)(rsec / 60 % 60));
    else snprintf(rec, sizeof(rec), "%u:%02u", (unsigned)(rsec / 60), (unsigned)(rsec % 60));
    snprintf(key, sizeof(key), "%s|%s|%s|%s|%s|%d|%d%s|%d", freq, mode_names[g_set.mode], gain, step, vol, (int)g_set.zoom,
             (int)(rs != REC_IDLE), rec, bat);
    if (!label_changed(&lbl_top, key)) return;

    // The columns are laid out for the widest text of each: a frequency entry of 11 characters
    // ends at 300, "TETRA" at 370, "RF 49.6 dB" at 516, "12.50k" at 564, "narrow" at 611, a
    // recording time at 669, "VOL 100" at 770 and "100%" starts at 771.
    pax_buf_t* b = &lbl_top.buf;
    pax_background(b, PAX_BAR_BG);
    pax_draw_text(b, entry ? 0xffffe060 : 0xffffffff, pax_font_sky_mono, 27, 6, 1, freq);
    if (!entry) pax_draw_text(b, 0xff8090a0, pax_font_sky_mono, 18, 6 + 21 * (float)strlen(freq) + 6, 8, "MHz");
    pax_draw_text(b, 0xff60ff90, pax_font_sky_mono, 18, 300, 5, mode_names[g_set.mode]);
    pax_draw_text(b, 0xffc0d0e0, pax_font_sky_mono, 18, 376, 5, gain);
    pax_draw_text(b, 0xff8090a0, pax_font_sky_mono, 9, 522, 3, "STEP");
    pax_draw_text(b, 0xffc0d0e0, pax_font_sky_mono, 9, 522, 14, step);
    pax_draw_text(b, 0xff8090a0, pax_font_sky_mono, 9, 569, 3, "SPAN");
    pax_draw_text(b, 0xffc0d0e0, pax_font_sky_mono, 9, 569, 14, g_set.zoom ? "narrow" : "wide");
    if (rs != REC_IDLE) {
        pax_draw_circle(b, 0xffff2020, 622, 14, 8);
        pax_draw_text(b, 0xffff6050, pax_font_sky_mono, 9, 634, 3, "REC");
        pax_draw_text(b, 0xffffc0b0, pax_font_sky_mono, 9, 634, 14, rec);
    }
    pax_draw_text(b, g_set.volume > 0 ? 0xffc0d0e0 : 0xffff7060, pax_font_sky_mono, 18, 672, 5, vol);
    draw_battery(b, bat);
    label_done(&lbl_top);
}

#define HELP_KEYS    "</> tune  ^/v step  TAB mode  +/- gain  "
#define HELP_FKEYS   "F2 settings  F3 memories  F4 keys  F5 messages  F6 record  F1 exit"
#define HELP_ANALOG  HELP_KEYS "SPACE squelch  S level  " HELP_FKEYS
#define HELP_DIGITAL HELP_KEYS "SPACE timeslot  " HELP_FKEYS

// Bottom line: the keys, or what the keys do right now (memories, settings, squelch level), or a
// short-lived notice.
void ui_help_text(char* text, size_t len, bool* highlight) {
    mem_ui_t mem = g_mem_ui;
    *highlight   = true;
    if (mem != MEM_UI_IDLE) {
        snprintf(text, len, "MEMORY bank %d   %s   TAB / Fn+TAB bank   F3 or ESC close", g_mem_bank,
                 mem == MEM_UI_SAVE ? "SAVE: 0-9 store the current frequency" : "0-9 load   S save");
    } else if (esp_timer_get_time() < g_notice_until) {
        snprintf(text, len, "%s", g_notice);
    } else if (g_menu_open) {
        snprintf(text, len, "SETTINGS   ^/v select   </> or +/- change   F2 or ESC close");
    } else if (g_help_open) {
        snprintf(text, len, "KEYS   F4 or ESC close   (any other key closes the list and does its job)");
    } else if (g_msgs_open) {
        snprintf(text, len, "MESSAGES   ^/v scroll   F5 or ESC close   (any other key closes the list and does its job)");
    } else if (g_sq_adjust && g_set.squelch_db > -120) {
        snprintf(text, len, "SQUELCH %d dBFS, %s   + / - change the level   SPACE off   S or ESC done (+ / - gain again)",
                 g_set.squelch_db, g_stat.squelch_open ? "open" : "closed");
    } else if (g_sq_adjust) {
        snprintf(text, len, "SQUELCH off   + / - or SPACE switch it on   S or ESC done (+ / - gain again)");
    } else {
        snprintf(text, len, "%s", mode_is_digital(g_set.mode) ? HELP_DIGITAL : HELP_ANALOG);
        *highlight = false;
    }
}

static void update_help(void) {
    char text[160];
    bool highlight;
    ui_help_text(text, sizeof(text), &highlight);
    label_text(&lbl_help, 0xff0a1018, highlight ? 0xffffe060 : 0xff7890a8, pax_font_saira_condensed, 16, 6, 0, text);
}

static void mem_slot_text(int bank, int slot, char* buf, size_t len) {
    uint32_t     hz;
    demod_mode_t mode;
    if (memo_get(bank, slot, &hz, &mode)) {
        snprintf(buf, len, "%d   %u.%06u  %s", slot, (unsigned)(hz / 1000000), (unsigned)(hz % 1000000),
                 mode_names[mode]);
    } else {
        snprintf(buf, len, "%d   -", slot);
    }
}

// The ten memories of the current bank, in place of the decoder lines.
static void panel_memories(void) {
    int bank = g_mem_bank;
    for (int row = 0; row < DEC_LINES; row++) {
        char left[40], right[40], key[96];
        mem_slot_text(bank, row, left, sizeof(left));
        mem_slot_text(bank, row + 5, right, sizeof(right));
        snprintf(key, sizeof(key), "M%d|%s|%s", bank, left, right);
        label_t* l = &lbl_line[2 + row];
        if (!label_changed(l, key)) continue;
        pax_background(&l->buf, PAX_PANEL);
        pax_draw_text(&l->buf, 0xffffe9a0, pax_font_saira_condensed, 18, 6, 0, left);
        pax_draw_text(&l->buf, 0xffffe9a0, pax_font_saira_condensed, 18, 300, 0, right);
        label_done(l);
    }
}

// The settings, in place of the decoder lines: columns of DEC_LINES, the selected one marked.
#define MENU_COLS    3
#define MENU_COL_W   (PANEL_W / MENU_COLS)
#define MENU_VALUE_X 92

static void panel_menu(void) {
    int sel = g_menu_sel;
    for (int row = 0; row < DEC_LINES; row++) {
        const char* label[MENU_COLS];
        char        value[MENU_COLS][32];
        char        key[160];
        int         o = snprintf(key, sizeof(key), "S");
        for (int c = 0; c < MENU_COLS; c++) {
            int item    = row + c * DEC_LINES;
            label[c]    = "";
            value[c][0] = 0;
            if (item < MENU_COUNT) menu_item_text(item, &label[c], value[c], sizeof(value[c]));
            if (o < (int)sizeof(key)) {
                o += snprintf(key + o, sizeof(key) - (size_t)o, "%d|%s|%s|", (int)(sel == item), label[c], value[c]);
            }
        }
        label_t* l = &lbl_line[2 + row];
        if (!label_changed(l, key)) continue;
        pax_background(&l->buf, PAX_PANEL);
        for (int c = 0; c < MENU_COLS; c++) {
            int item = row + c * DEC_LINES;
            if (item >= MENU_COUNT) continue;
            float x  = (float)(c * MENU_COL_W);
            bool  on = sel == item;
            if (on) pax_draw_rect(&l->buf, 0xff1d4a78, x + 2, 0, MENU_COL_W - 4, LINE_H);
            pax_draw_text(&l->buf, on ? 0xffffffff : 0xff8fa4b8, pax_font_saira_condensed, 18, x + 6, 0, label[c]);
            pax_draw_text(&l->buf, on ? 0xffffe060 : 0xffe8eef4, pax_font_saira_condensed, 18, x + MENU_VALUE_X, 0,
                          value[c]);
        }
        label_done(l);
    }
}

static void update_panel(void) {
    static int64_t last      = 0;
    static int     last_view = -1;
    int64_t        now       = esp_timer_get_time();
    // The lower lines show the memories or the settings while those are open. The status lines are
    // redrawn four times a second, and at once when what is shown below them changes.
    bool menu = g_menu_open && g_mem_ui == MEM_UI_IDLE;
    int  mem  = g_mem_ui != MEM_UI_IDLE ? 1 + g_mem_bank : 0;
    int  view = menu ? 100 : mem;
    bool tick = now - last >= 250000 || view != last_view;
    last_view = view;
    if (menu) panel_menu();  // every frame, the keys repeat faster than the tick; unchanged lines cost nothing
    if (!tick) return;
    last = now;

    char           text[DEC_LINE_LEN + 48];
    radio_status_t st = g_stat.status;
    if (st == RADIO_STREAMING) {
        snprintf(text, sizeof(text), "%.40s   %u.%03u MS/s   lost %u   DSP %u%%   %d fps", g_stat.status_text,
                 (unsigned)(g_stat.effective_sps / 1000000), (unsigned)(g_stat.effective_sps % 1000000 / 1000),
                 (unsigned)g_stat.drops, (unsigned)(g_stat.cpu_permille / 10), (int)(g_ui_fps + 0.5f));
    } else {
        snprintf(text, sizeof(text), "%s", g_stat.status_text);
    }
    label_text(&lbl_line[0], PAX_PANEL, st == RADIO_STREAMING ? 0xff8fa4b8 : 0xffffa040, pax_font_saira_condensed, 18,
               6, 0, text);

    char sql[24];
    if (g_set.squelch_db <= -120) {
        snprintf(sql, sizeof(sql), "off");
    } else {
        snprintf(sql, sizeof(sql), "%d dB %s", g_set.squelch_db, g_stat.squelch_open ? "open" : "closed");
    }
    // More than 0.5 % of the raw samples on the rails: the tuner gain is too high for this band.
    bool clipping = g_stat.clip_permille >= 5;
    snprintf(text, sizeof(text), "Signal %.1f dBFS   ADC %u..%u%s   SQL %s   PPM %d", (double)g_stat.chan_db,
             g_stat.adc_min, g_stat.adc_max, clipping ? " CLIP - lower the gain" : "", sql, g_set.ppm);
    label_text(&lbl_line[1], PAX_PANEL, clipping ? 0xffff7060 : 0xffe8eef4, pax_font_saira_condensed, 18, 6, 0, text);

    if (menu) return;
    if (mem) {
        panel_memories();
        return;
    }
    static char dec[DEC_LINES][DEC_LINE_LEN];
    memset(dec, 0, sizeof(dec));
    dec_get_lines(g_set.mode, dec);
    for (int i = 0; i < DEC_LINES; i++) {
        dec[i][DEC_LINE_LEN - 1] = 0;
        label_text(&lbl_line[2 + i], PAX_PANEL, i == 0 ? 0xff7dff8a : 0xffc8e8d0, pax_font_saira_condensed, 18, 6, 0,
                   dec[i]);
    }
}

// ---- key list (F4) -------------------------------------------------------------------------------

typedef struct {
    const char* key;
    const char* text;
} key_row_t;

static const key_row_t KEYS_LEFT[] = {
    {"LEFT / RIGHT", "tune one step, hold to keep going"},
    {"UP / DOWN", "tuning step larger / smaller"},
    {"0-9 .", "type a frequency in MHz, RETURN tunes"},
    {"TAB", "next mode,  Fn+TAB previous mode"},
    {"N A L U W D T", "NFM AM LSB USB WFM DMR TETRA"},
    {"+ / -", "RF gain"},
    {"G", "automatic gain on / off"},
    {"Z", "spectrum span wide / narrow"},
    {"SPACE", "squelch on / off"},
    {"", "in DMR / TETRA: timeslot AUTO, 1, 2, ..."},
    {"S", "squelch level with + / -,  S again: gain"},
    {"volume keys", "audio volume (starts muted)"},
    {"LED A / B", "signal / audio, digital, recording"},
};

static const key_row_t KEYS_RIGHT[] = {
    {"F1", "back to the launcher"},
    {"F2  triangle", "settings: UP / DOWN select,"},
    {"", "LEFT / RIGHT change, F2 closes"},
    {"F3  square", "memories of the current bank (also M):"},
    {"", "0-9 load,  S then 0-9 save,"},
    {"", "TAB / Fn+TAB next / previous bank"},
    {"F4  circle", "this list"},
    {"F5  trilobe", "received messages, UP / DOWN scroll"},
    {"F6  rhombus", "record what is heard to the SD card,"},
    {"", "F6 again stops; format: see settings"},
    {"Fn+F6", "screenshot to the SD card"},
    {"", "all in the folder " REC_DIRNAME},
    {"ESC", "closes settings, memories and lists"},
};

#define KEYS_Y0 SPEC_Y0
#define KEYS_H  (PANEL_Y - SPEC_Y0)

static void keys_column(pax_buf_t* b, float x, float text_x, const key_row_t* rows, int n) {
    for (int i = 0; i < n; i++) {
        float y = 30.0f + (float)(i * LINE_H);
        pax_draw_text(b, 0xffffe060, pax_font_saira_condensed, 18, x, y, rows[i].key);
        pax_draw_text(b, 0xffe8eef4, pax_font_saira_condensed, 18, text_x, y, rows[i].text);
    }
}

// True when the list can be shown.
static bool keys_page(void) {
    static bool ready = false, failed = false;
    if (ready) return true;
    if (failed || !label_init(&lbl_keys, 0, KEYS_Y0, LW, KEYS_H)) {
        if (!failed) app_notice("No memory for the key list");
        failed      = true;
        g_help_open = false;
        return false;
    }
    pax_buf_t* b = &lbl_keys.buf;
    pax_background(b, 0xff0a1018);
    pax_draw_text(b, 0xff60ff90, pax_font_saira_condensed, 18, 10, 4, "KEYS");
    pax_draw_text(b, 0xff8fa4b8, pax_font_saira_condensed, 18, 124, 4, "Tanmatsu SDR " APP_VERSION);
    pax_draw_rect(b, 0xff2a3a4c, 6, 26, LW - 12, 1);
    keys_column(b, 10, 124, KEYS_LEFT, (int)(sizeof(KEYS_LEFT) / sizeof(KEYS_LEFT[0])));
    keys_column(b, 402, 498, KEYS_RIGHT, (int)(sizeof(KEYS_RIGHT) / sizeof(KEYS_RIGHT[0])));
    label_done(&lbl_keys);
    ready = true;
    return true;
}

// ---- received messages (F5) ----------------------------------------------------------------------

#define MSG_ROWS      13   // lines of LINE_H under the heading
#define MSG_MAX_LINES 4    // ... of which one message takes at most this many
#define MSG_TEXT_X    320
#define MSG_RIGHT     (LW - 10)
// The text of a message is drawn in the one font here that has the accented letters of the
// western European languages (Latin-1); the condensed one of the rest of the screen is ASCII only.
#define MSG_FONT      pax_font_saira_regular

// Whether MSG_FONT can draw the character (it leaves out what it does not have, without a trace).
static bool msg_font_has(uint32_t cp) {
    static const uint16_t more[] = {0x131, 0x152, 0x153, 0x2013, 0x2014, 0x2018, 0x2019, 0x201a,
                                    0x201c, 0x201d, 0x201e, 0x2022, 0x2026, 0x20ac, 0x2122};
    if ((cp >= 0x20 && cp <= 0x7e) || (cp >= 0xa0 && cp <= 0xff)) return true;
    for (size_t i = 0; i < sizeof(more) / sizeof(more[0]); i++) {
        if (more[i] == cp) return true;
    }
    return false;
}

// The next line of `text` that fits into `width` pixels, into `out`: broken after a space where
// there is one, anywhere otherwise, never inside a UTF-8 character. Control characters become
// spaces, characters the font does not have question marks. Returns the number of bytes of `text`
// used up.
static size_t wrap_line(const char* text, float width, char* out, size_t out_len) {
    size_t used = 0, len = 0, brk_used = 0, brk_len = 0;
    while (text[used]) {
        size_t        n  = 1;
        unsigned char c0 = (unsigned char)text[used];
        while ((text[used + n] & 0xc0) == 0x80) n++;
        if (len + n >= out_len) break;
        uint32_t cp = c0 < 0x80 ? c0 : (c0 >= 0xf0 ? c0 & 0x07u : (c0 >= 0xe0 ? c0 & 0x0fu : c0 & 0x1fu));
        for (size_t k = 1; k < n; k++) cp = (cp << 6) | ((unsigned char)text[used + k] & 0x3fu);
        size_t add = n;  // bytes that go into the line for it
        if (c0 < 0x20 || c0 == 0x7f) {
            out[len] = ' ';
        } else if (!msg_font_has(cp) || (c0 >= 0x80 && c0 < 0xc0)) {
            out[len] = '?';
            add      = 1;
        } else {
            memcpy(out + len, text + used, n);
        }
        out[len + add] = 0;
        if (len > 0 && pax_text_size(MSG_FONT, 18, out).x > width) {
            if (brk_used) {
                used = brk_used;
                len  = brk_len;
            }
            out[len] = 0;
            return used;
        }
        used += n;
        len  += add;
        if (out[len - 1] == ' ') {
            brk_used = used;
            brk_len  = len - 1;  // the space is not drawn at the end of a line
        }
    }
    out[len] = 0;
    return used;
}

// True when the list can be shown. Redrawn only when what it shows has changed.
static bool msgs_page(void) {
    static bool  ready = false, failed = false;
    static msg_t m;
    if (!ready) {
        if (failed || !label_init(&lbl_msgs, 0, KEYS_Y0, LW, KEYS_H)) {
            if (!failed) app_notice("No memory for the message list");
            failed      = true;
            g_msgs_open = false;
            return false;
        }
        ready = true;
    }
    int  count  = msglog_count();
    int  scroll = g_msgs_scroll;
    char key[160];
    snprintf(key, sizeof(key), "%u|%d|%d|%d|%u|%u|%s", (unsigned)g_msglog.seen, count, scroll, (int)g_set.sds_log,
             (unsigned)g_msglog.written, (unsigned)g_msglog.errors, g_msglog.file);
    if (!label_changed(&lbl_msgs, key)) return true;

    pax_buf_t* b = &lbl_msgs.buf;
    char       line[200];
    pax_background(b, 0xff0a1018);
    pax_draw_text(b, 0xff60ff90, pax_font_saira_condensed, 18, 10, 4, "MESSAGES");
    int o = snprintf(line, sizeof(line), "%u received", (unsigned)g_msglog.seen);
    if (count > 1) o += snprintf(line + o, sizeof(line) - (size_t)o, ", newest first, from no. %d of the last %d", scroll + 1, count);
    if (!g_set.sds_log) {
        snprintf(line + o, sizeof(line) - (size_t)o, "      SD log off (F2 settings)");
    } else if (g_msglog.errors) {
        snprintf(line + o, sizeof(line) - (size_t)o, "      SD log on: %u written, %u NOT written", (unsigned)g_msglog.written,
                 (unsigned)g_msglog.errors);
    } else {
        snprintf(line + o, sizeof(line) - (size_t)o, "      SD log on: %u written  %s", (unsigned)g_msglog.written, g_msglog.file);
    }
    pax_draw_text(b, 0xff8fa4b8, pax_font_saira_condensed, 18, 124, 4, line);
    pax_draw_rect(b, 0xff2a3a4c, 6, 26, LW - 12, 1);

    if (count == 0) {
        pax_draw_text(b, 0xffe8eef4, pax_font_saira_condensed, 18, 10, 30, "No messages received yet.");
        pax_draw_text(b, 0xff8fa4b8, pax_font_saira_condensed, 18, 10, 30 + 2 * LINE_H,
                      "TETRA short data (SDS) and DMR text messages are listed here as they are decoded:");
        pax_draw_text(b, 0xff8fa4b8, pax_font_saira_condensed, 18, 10, 30 + 3 * LINE_H,
                      "time, frequency, sender > receiver and the text, or the data in hex if it is not a text.");
        pax_draw_text(b, 0xff8fa4b8, pax_font_saira_condensed, 18, 10, 30 + 5 * LINE_H,
                      "With \"SDS log\" on in the settings (F2) they are also written to the SD card, in");
        pax_draw_text(b, 0xff8fa4b8, pax_font_saira_condensed, 18, 10, 30 + 6 * LINE_H,
                      REC_DIRNAME "/<network MCC-MNC>/<date>-sds.txt");
        label_done(&lbl_msgs);
        return true;
    }
    int row = 0;
    for (int i = scroll; row < MSG_ROWS && msglog_get(i, &m); i++) {
        float     y = 30.0f + (float)(row * LINE_H);
        time_t    t = (time_t)m.time;
        struct tm tm;
        localtime_r(&t, &tm);
        snprintf(line, sizeof(line), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
        pax_draw_text(b, 0xff8fa4b8, pax_font_saira_condensed, 18, 10, y, line);
        snprintf(line, sizeof(line), "%u.%03u", (unsigned)(m.freq_hz / 1000000), (unsigned)(m.freq_hz % 1000000 / 1000));
        pax_draw_text(b, 0xff8fa4b8, pax_font_saira_condensed, 18, 72, y, line);
        snprintf(line, sizeof(line), "%s > %s", m.from, m.to);
        pax_draw_text(b, 0xffffe060, pax_font_saira_condensed, 18, 132, y, line);
        float x = 132.0f + pax_text_size(pax_font_saira_condensed, 18, line).x + 12.0f;
        if (x < MSG_TEXT_X) x = MSG_TEXT_X;

        // A status is just its number; data that is not a text is shown dimmer than a text.
        static char body[MSG_TEXT_MAX + 16];
        bool        is_text = strcmp(m.kind, "text") == 0;
        if (strcmp(m.kind, "status") == 0) snprintf(body, sizeof(body), "status %s", m.text);
        else snprintf(body, sizeof(body), "%s", m.text);
        const char* p = body;
        for (int n = 0; n < MSG_MAX_LINES && row < MSG_ROWS && (*p || n == 0); n++, row++) {
            size_t used = wrap_line(p, (float)MSG_RIGHT - x, line, sizeof(line) - 4);
            p += used;
            while (*p == ' ') p++;
            if (*p && (n == MSG_MAX_LINES - 1 || row == MSG_ROWS - 1)) {
                // Does not fit: the last characters make room for the dots.
                size_t len = strlen(line);
                for (int cut = 0; cut < 3 && len > 0; cut++) {
                    do len--;
                    while (len > 0 && (line[len] & 0xc0) == 0x80);
                }
                snprintf(line + len, sizeof(line) - len, " ...");
            }
            pax_draw_text(b, is_text ? 0xffe8eef4 : 0xff9fd0e8, MSG_FONT, 18, x, 30.0f + (float)(row * LINE_H), line);
            x = MSG_TEXT_X;
            if (used == 0) {
                row++;
                break;
            }
        }
    }
    label_done(&lbl_msgs);
    return true;
}

// ---- widget --------------------------------------------------------------------------------------

static void draw_widget(uint8_t* base) {
    static float scope_a[SCOPE_LEN], scope_b[SCOPE_LEN];
    static bool  is_iq   = false;
    static float peak    = 0.1f;
    static uint32_t last = 0;
    if (g_scope.seq != last) {
        last = g_scope.seq;
        memcpy(scope_a, g_scope.a, sizeof(scope_a));
        memcpy(scope_b, g_scope.b, sizeof(scope_b));
        is_iq   = g_scope.is_iq;
        float m = 1e-6f;
        for (int i = 0; i < SCOPE_LEN; i++) {
            float v = fabsf(scope_a[i]);
            if (is_iq && fabsf(scope_b[i]) > v) v = fabsf(scope_b[i]);
            if (v > m) m = v;
        }
        peak = m > peak ? m : peak + 0.05f * (m - peak);
    }
    int   cy = WID_Y + WID_H / 2;
    float k  = (float)(WID_H / 2 - 6) / (peak + 1e-9f);
    if (is_iq) {
        // Constellation: the points come with an RMS radius of 1, so the scale is fixed and the
        // four places where a clean pi/4-DQPSK signal lands are marked.
        k = (float)(WID_H / 2 - 6) / 1.5f;
        fill_rect(base, WID_X, WID_Y, WID_W, WID_H, COL_WID_BG);
        int cx = WID_X + WID_W / 2;
        for (int i = 0; i < WID_H; i += 2) plot(base, cx, WID_Y + i, COL_WID_AXIS);
        for (int i = 0; i < WID_W; i += 2) plot(base, WID_X + i, cy, COL_WID_AXIS);
        int d = (int)(0.70710678f * k + 0.5f);
        for (int q = 0; q < 4; q++) {
            int mx = cx + ((q & 1) ? -d : d), my = cy + ((q & 2) ? -d : d);
            for (int i = 4; i <= 7; i++) {
                plot(base, mx - i, my, COL_WID_MARK);
                plot(base, mx + i, my, COL_WID_MARK);
                plot(base, mx, my - i, COL_WID_MARK);
                plot(base, mx, my + i, COL_WID_MARK);
            }
        }
        for (int i = 0; i < SCOPE_LEN; i++) {
            if (scope_a[i] == 0.0f && scope_b[i] == 0.0f) continue;  // unused entry
            int x = cx + (int)(scope_a[i] * k);
            int y = cy - (int)(scope_b[i] * k);
            if (x <= WID_X || x >= WID_X + WID_W - 1 || y <= WID_Y || y >= WID_Y + WID_H - 1) continue;
            plot(base, x, y, COL_TRACE);
            plot(base, x + 1, y, COL_TRACE);
            plot(base, x, y + 1, COL_TRACE);
            plot(base, x + 1, y + 1, COL_TRACE);
        }
        return;
    }
    int prev = cy;
    for (int x = 0; x < WID_W; x++) {
        int y = cy - (int)(scope_a[x] * k);
        if (y < WID_Y) y = WID_Y;
        if (y > WID_Y + WID_H - 1) y = WID_Y + WID_H - 1;
        if (x == 0) prev = y;
        int lo = y < prev ? y : prev;
        int hi = y > prev ? y : prev;
        prev   = y;
        uint8_t* p = col_ptr(base, WID_X + x, WID_Y + WID_H - 1);
        for (int ly = WID_Y + WID_H - 1; ly >= WID_Y; ly--, p += 3) {
            put(p, (ly >= lo && ly <= hi) ? COL_TRACE : (ly == cy ? COL_WID_AXIS : COL_WID_BG));
        }
    }
}

// ---- frame ---------------------------------------------------------------------------------------

void ui_frame(void) {
    static int64_t last_frame = 0;
    static float   acc        = 0.0f;
    static int     count      = 0;

    spectrum_update();
    band_update();
    update_scale();
    update_top();
    update_panel();
    update_help();

    // The key list or the message list takes the place of the spectrum, the scale and the
    // waterfall, which go on being computed underneath.
    static label_t* page_shown = NULL;
    label_t*        page       = g_help_open && keys_page() ? &lbl_keys : (g_msgs_open && msgs_page() ? &lbl_msgs : NULL);
    if (page != page_shown) {
        page_shown      = page;
        if (page) page->dirty = 3;
        lbl_scale.dirty = 3;
    }

    uint8_t* base = fbp[fb_cur];
    if (page) {
        label_paste(page, fb_cur);
    } else {
        draw_spectrum(base);
        draw_waterfall(base);
        label_paste(&lbl_scale, fb_cur);
    }
    draw_widget(base);
    label_paste(&lbl_top, fb_cur);
    label_paste(&lbl_help, fb_cur);
    for (int i = 0; i < PANEL_N; i++) label_paste(&lbl_line[i], fb_cur);

    if (g_shot_req && g_shot_buf) {
        memcpy(g_shot_buf, base, g_shot_size);
        g_shot_req   = false;
        g_shot_ready = true;
    }

    esp_err_t res = bsp_display_blit(0, 0, disp_h_res, disp_v_res, base);
    if (res != ESP_OK) ESP_LOGE(TAG, "blit failed: %s", esp_err_to_name(res));
    fb_cur ^= 1;

    int64_t now = esp_timer_get_time();
    int64_t dt  = now - last_frame;
    last_frame  = now;
    if (dt > 0 && dt < 1000000) {
        acc += 1000000.0f / (float)dt;
        if (++count >= 10) {
            g_ui_fps = acc / (float)count;
            acc      = 0.0f;
            count    = 0;
        }
    }
}

bool ui_init(void) {
    bsp_display_color_format_t color_format = 0;
    bsp_display_endianness_t   endian       = 0;
    esp_err_t res = bsp_display_get_parameters(&disp_h_res, &disp_v_res, &color_format, &endian);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "display parameters: %s", esp_err_to_name(res));
        return false;
    }
    if (disp_h_res != RAW_W || disp_v_res != LW || color_format != BSP_DISPLAY_COLOR_FORMAT_24_888RGB ||
        bsp_display_get_default_rotation() != BSP_DISPLAY_ROTATION_270) {
        ESP_LOGE(TAG, "unsupported display %ux%u format %d", (unsigned)disp_h_res, (unsigned)disp_v_res,
                 (int)color_format);
        return false;
    }
    endian_big = endian == BSP_DISPLAY_ENDIAN_BIG;

    esp_lcd_panel_handle_t panel       = NULL;
    void*                  panel_fb[2] = {NULL, NULL};
    if (bsp_display_get_panel(&panel) != ESP_OK ||
        esp_lcd_dpi_panel_get_frame_buffer(panel, 2, &panel_fb[0], &panel_fb[1]) != ESP_OK) {
        panel_fb[0] = panel_fb[1] = NULL;
        ESP_LOGW(TAG, "panel framebuffers unavailable, falling back to copy blit");
    }
    for (int i = 0; i < 2; i++) {
        pax_buf_init(&fb[i], panel_fb[i], disp_h_res, disp_v_res, PAX_BUF_24_888RGB);
        pax_buf_reversed(&fb[i], endian_big);
        pax_buf_set_orientation(&fb[i], PAX_O_ROT_CW);
        fbp[i] = (uint8_t*)pax_buf_get_pixels(&fb[i]);
        if (!fbp[i]) return false;
        memset(fbp[i], 0, disp_h_res * disp_v_res * 3);
    }

    bool ok  = label_init(&lbl_top, 0, 0, LW, TOP_H);
    ok      &= label_init(&lbl_scale, 0, SCALE_Y, LW, SCALE_H);
    ok      &= label_init(&lbl_help, 0, HELP_Y, LW, HELP_H);
    for (int i = 0; i < PANEL_N; i++) ok &= label_init(&lbl_line[i], 0, PANEL_Y + i * LINE_H, PANEL_W, LINE_H);
    wf = heap_caps_calloc((size_t)LW * WF_H, 1, MALLOC_CAP_SPIRAM);
    if (!ok || !wf) {
        ESP_LOGE(TAG, "out of memory for UI buffers");
        return false;
    }

    build_palette();
    dsp_fft_init();
    for (int x = 0; x < LW; x++) spec_y[x] = SPEC_Y1;
    view_freq = g_set.freq_hz;

    g_snap.req = true;
    return true;
}
