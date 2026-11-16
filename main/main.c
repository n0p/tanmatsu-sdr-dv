// SPDX-License-Identifier: AGPL-3.0-only
// Tanmatsu SDR: RTL-SDR receiver with spectrum / waterfall, NFM / AM / WFM audio and DMR / TETRA
// decoding. Main task: board bring-up, keyboard, settings, UI loop.
//
// Keys
//   LEFT / RIGHT      tune by one step            UP / DOWN   change the step
//   TAB               next mode                   N A L U W D T   NFM / AM / LSB / USB / WFM / DMR / TETRA
//   Fn+TAB            previous mode               0-9 .       enter a frequency in MHz, RETURN tunes
//   + / -             RF gain up / down           G           automatic gain on / off
//   Z                 wide / narrow spectrum      volume keys audio volume (starts muted)
//   SPACE             NFM / AM / LSB / USB / WFM: squelch on / off
//                     DMR / TETRA: timeslot that is heard, AUTO -> 1 -> 2 (-> 3 -> 4) -> AUTO
//   S                 squelch level: + / - change it, S again (or ESC) goes back to + / - = gain
//   F2 (triangle)     settings: UP / DOWN select, LEFT / RIGHT change, F2 or ESC close
//   F3 (square) or M  memories: 0-9 load, S 0-9 save, TAB / Fn+TAB next / previous bank
//   F4 (circle)       all the keys, on screen
//   F5 (trilobe)      the messages received last (TETRA SDS, DMR text), UP / DOWN scroll
//   F6 (rhombus)      record the audio to the SD card, again to stop
//   Fn+F6             screenshot to the SD card
//   F1                back to the launcher
// LEFT / RIGHT and the volume keys repeat while they are held.

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app.h"
#include "bsp/audio.h"
#include "bsp/device.h"
#include "bsp/display.h"
#include "bsp/input.h"
#include "bsp/led.h"
#include "bsp/power.h"
#include "decoder.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "recfmt.h"
#include "ui.h"

static const char TAG[] = "main";

const uint32_t step_table[] = {10, 100, 1000, 5000, 6250, 12500, 25000, 100000, 1000000};
const int      step_count   = sizeof(step_table) / sizeof(step_table[0]);
volatile int   g_step_idx   = 5;

volatile mem_ui_t g_mem_ui;
volatile int      g_mem_bank;
volatile bool     g_menu_open;
volatile int      g_menu_sel;
volatile bool     g_sq_adjust;
volatile bool     g_help_open;
volatile bool     g_msgs_open;
volatile int      g_msgs_scroll;
char              g_notice[96];
volatile int64_t  g_notice_until;
static int64_t    mem_ui_until;

#define MEM_UI_TIMEOUT_US 8000000
#define NOTICE_US         3000000

typedef struct {
    int      nav_key;
    uint32_t mods;
    char     ascii;
    int8_t   edge;  // 0: pressed and released, 1: pressed, -1: released
} inject_t;

static QueueHandle_t input_queue;
static QueueHandle_t inject_queue;
static volatile bool settings_dirty;
static int64_t       settings_dirty_at;
static int           last_manual_gain = -1;
static int           squelch_last     = SQUELCH_OFF;  // level to come back to when the squelch is switched on

// ---- settings ------------------------------------------------------------------------------------

void settings_mark_dirty(void) {
    settings_dirty_at = esp_timer_get_time();
    settings_dirty    = true;
}

static void settings_load(void) {
    nvs_handle_t h;
    if (nvs_open("sdr", NVS_READONLY, &h) != ESP_OK) return;
    uint32_t u32;
    int32_t  i32;
    char     name[12];
    size_t   len = sizeof(name);
    if (nvs_get_u32(h, "freq", &u32) == ESP_OK && u32 >= FREQ_MIN_HZ && u32 <= FREQ_MAX_HZ) g_set.freq_hz = u32;
    // The mode is stored by name. "mode" is the number that versions before the SSB modes wrote.
    static const demod_mode_t old_modes[] = {MODE_NFM, MODE_AM, MODE_WFM, MODE_DMR, MODE_TETRA};
    if (nvs_get_str(h, "modename", name, &len) == ESP_OK) {
        if (mode_from_name(name) >= 0) g_set.mode = (demod_mode_t)mode_from_name(name);
    } else if (nvs_get_i32(h, "mode", &i32) == ESP_OK && i32 >= 0 && i32 < 5) {
        g_set.mode = old_modes[i32];
    }
    if (nvs_get_i32(h, "gain", &i32) == ESP_OK && i32 >= GAIN_AUTO && i32 < MAX_GAINS) g_set.gain_idx = i32;
    if (nvs_get_i32(h, "ppm", &i32) == ESP_OK && i32 >= -200 && i32 <= 200) g_set.ppm = i32;
    if (nvs_get_i32(h, "sq", &i32) == ESP_OK && i32 >= -200 && i32 <= 0) g_set.squelch_db = i32;
    if (nvs_get_i32(h, "zoom", &i32) == ESP_OK) g_set.zoom = i32 != 0;
    if (nvs_get_u32(h, "stephz", &u32) == ESP_OK) {
        for (int i = 0; i < step_count; i++) {
            if (step_table[i] == u32) g_step_idx = i;
        }
    }
    if (nvs_get_i32(h, "bank", &i32) == ESP_OK && i32 >= 0 && i32 < MEM_BANKS) g_mem_bank = i32;
    if (nvs_get_i32(h, "lastgain", &i32) == ESP_OK && i32 >= 0 && i32 < MAX_GAINS) last_manual_gain = i32;
    if (nvs_get_i32(h, "sqlast", &i32) == ESP_OK && i32 >= -100 && i32 <= 0) squelch_last = i32;
    if (nvs_get_i32(h, "dmrts", &i32) == ESP_OK && i32 >= 0 && i32 <= DMR_SLOTS) g_set.dmr_slot = i32;
    if (nvs_get_i32(h, "tetrats", &i32) == ESP_OK && i32 >= 0 && i32 <= TETRA_SLOTS) g_set.tetra_slot = i32;
    if (nvs_get_i32(h, "recfmt", &i32) == ESP_OK && i32 >= 0 && i32 < REC_FMT_COUNT) g_set.rec_format = i32;
    if (nvs_get_i32(h, "sdslog", &i32) == ESP_OK) g_set.sds_log = i32 != 0;
    nvs_close(h);
    // The volume is deliberately not stored: the app always starts muted.
}

static void settings_save(void) {
    nvs_handle_t h;
    settings_dirty = false;
    if (nvs_open("sdr", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, "freq", g_set.freq_hz);
    nvs_set_str(h, "modename", mode_names[g_set.mode]);
    nvs_set_i32(h, "gain", g_set.gain_idx);
    nvs_set_i32(h, "ppm", g_set.ppm);
    nvs_set_i32(h, "sq", g_set.squelch_db);
    nvs_set_i32(h, "zoom", g_set.zoom ? 1 : 0);
    nvs_set_u32(h, "stephz", step_table[g_step_idx]);
    nvs_set_i32(h, "bank", g_mem_bank);
    nvs_set_i32(h, "lastgain", last_manual_gain);
    nvs_set_i32(h, "sqlast", squelch_last);
    nvs_set_i32(h, "dmrts", g_set.dmr_slot);
    nvs_set_i32(h, "tetrats", g_set.tetra_slot);
    nvs_set_i32(h, "recfmt", g_set.rec_format);
    nvs_set_i32(h, "sdslog", g_set.sds_log ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
}

// ---- actions -------------------------------------------------------------------------------------

void app_set_freq(uint32_t hz) {
    if (hz < FREQ_MIN_HZ) hz = FREQ_MIN_HZ;
    if (hz > FREQ_MAX_HZ) hz = FREQ_MAX_HZ;
    g_set.freq_hz = hz;
    settings_mark_dirty();
}

void app_set_mode(demod_mode_t mode) {
    if (mode < 0 || mode >= MODE_COUNT) return;
    g_set.mode = mode;
    settings_mark_dirty();
}

void app_set_gain_idx(int idx) {
    int n = g_stat.gain_count;
    if (idx < 0 || n <= 0) {
        idx = GAIN_AUTO;
    } else if (idx >= n) {
        idx = n - 1;
    }
    if (idx >= 0) last_manual_gain = idx;
    g_set.gain_idx = idx;
    settings_mark_dirty();
}

void app_set_volume(int volume) {
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    g_set.volume = volume;
    audio_apply_volume();
}

void app_exit_to_launcher(void) {
    g_set.volume = 0;
    audio_apply_volume();
    rec_stop_wait();  // a recording is closed properly before the restart,
    msglog_stop_wait();  // the message log written,
    shot_stop_wait();    // a screenshot finished
    leds_off();
    if (settings_dirty) settings_save();
    bsp_power_set_usb_host_boost_enabled(false);
    bsp_device_restart_to_launcher();
}

void app_inject_key(int nav_key, uint32_t modifiers, char ascii) {
    inject_t ev = {nav_key, modifiers, ascii, 0};
    if (inject_queue) xQueueSend(inject_queue, &ev, 0);
}

void app_inject_key_edge(int nav_key, bool down) {
    inject_t ev = {nav_key, 0, 0, down ? 1 : -1};
    if (inject_queue) xQueueSend(inject_queue, &ev, 0);
}

void app_notice(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_notice, sizeof(g_notice), fmt, ap);
    va_end(ap);
    g_notice_until = esp_timer_get_time() + NOTICE_US;
}

// dir 0: back to manual gain where it was left. From automatic gain, + and - start there too.
static void gain_step(int dir) {
    int n = g_stat.gain_count;
    if (n <= 0) {
        app_notice("No RF gain steps: no dongle running");
        return;
    }
    if (g_stat.direct && dir != 0) {
        app_notice("Direct sampling below 24 MHz: the tuner and its gain are bypassed (G: RTL AGC on / off)");
        return;
    }
    int idx = g_set.gain_idx;
    if (idx < 0) idx = last_manual_gain >= 0 ? last_manual_gain : n / 2;
    idx += dir;
    if (idx < 0) idx = 0;
    app_set_gain_idx(idx);
}

// ---- squelch and timeslot (SPACE) ----------------------------------------------------------------

static void squelch_set(int db) {
    g_set.squelch_db = db;
    if (db > -120) squelch_last = db;
    settings_mark_dirty();
}

#define SQUELCH_MIN_DB (-100)

static void squelch_on(void) {
    int level = squelch_last;
    if (level <= -120) {
        // Never set: a little above what the channel carries right now, which is right when the
        // key is pressed on a quiet channel. S with + / - moves it from there.
        float db = g_stat.chan_db;
        level    = (db > -110.0f && db < -10.0f) ? (int)db + 6 : -40;
    }
    if (level < SQUELCH_MIN_DB) level = SQUELCH_MIN_DB;
    if (level > 0) level = 0;
    squelch_set(level);
    app_notice("Squelch on: opens above %d dBFS   (S then + / - change the level, SPACE switches it off)", level);
}

static void squelch_toggle(void) {
    if (g_set.squelch_db > -120) {
        squelch_set(SQUELCH_OFF);
        app_notice("Squelch off");
    } else {
        squelch_on();
    }
}

// One step of the level. From off, a step in either direction switches the squelch on where it was.
static void squelch_step(int dir) {
    if (g_set.squelch_db <= -120) {
        squelch_on();
        g_notice_until = 0;
        return;
    }
    int level = g_set.squelch_db + dir;
    if (level < SQUELCH_MIN_DB) level = SQUELCH_MIN_DB;
    if (level > 0) level = 0;
    squelch_set(level);
}

// S: + and - belong to the squelch level until S is pressed again.
static void squelch_adjust_set(bool on) {
    if (on && mode_is_digital(g_set.mode)) {
        app_notice("DMR / TETRA have no squelch   (SPACE selects the timeslot that is heard)");
        return;
    }
    if (on == g_sq_adjust) return;
    g_sq_adjust = on;
    if (on) {
        if (g_set.squelch_db <= -120) squelch_on();
        g_notice_until = 0;  // the prompt of the mode takes the line
    } else if (g_set.squelch_db > -120) {
        app_notice("Squelch: opens above %d dBFS   (+ / - change the gain again)", g_set.squelch_db);
    } else {
        app_notice("Squelch off   (+ / - change the gain again)");
    }
}

// AUTO -> 1 -> 2 (-> 3 -> 4) -> AUTO
static void slot_next(void) {
    bool        dmr  = g_set.mode == MODE_DMR;
    int         n    = dmr ? DMR_SLOTS : TETRA_SLOTS;
    int         slot = ((dmr ? g_set.dmr_slot : g_set.tetra_slot) + 1) % (n + 1);
    const char* name = dmr ? "TS" : "TN";
    if (dmr) g_set.dmr_slot = slot;
    else g_set.tetra_slot = slot;
    settings_mark_dirty();
    if (slot) {
        app_notice("Audio: timeslot %s%d only   (SPACE: %s)", name, slot, slot < n ? "next timeslot" : "AUTO");
    } else {
        app_notice("Audio: AUTO, the first timeslot with speech   (SPACE: %s1 only)", name);
    }
}

// ---- memories ------------------------------------------------------------------------------------

static void mem_ui_set(mem_ui_t st) {
    g_mem_ui     = st;
    mem_ui_until = esp_timer_get_time() + MEM_UI_TIMEOUT_US;
    if (st != MEM_UI_IDLE) {
        g_notice_until = 0;
        g_menu_open    = false;
        g_sq_adjust    = false;
    }
}

static void mem_step_bank(int dir) {
    g_mem_bank = (g_mem_bank + MEM_BANKS + dir) % MEM_BANKS;
    settings_mark_dirty();
    mem_ui_set(g_mem_ui);
}

static void mem_digit(int slot) {
    int          bank = g_mem_bank;
    mem_ui_t     what = g_mem_ui;
    uint32_t     hz   = g_set.freq_hz;
    demod_mode_t mode = g_set.mode;
    mem_ui_set(MEM_UI_IDLE);
    if (what == MEM_UI_SAVE) {
        bool ok = memo_set(bank, slot, hz, mode);
        if (ok) {
            app_notice("Saved %u.%06u MHz %s to bank %d memory %d", (unsigned)(hz / 1000000), (unsigned)(hz % 1000000),
                       mode_names[mode], bank, slot);
        } else {
            app_notice("Bank %d memory %d set, but not written to the file: %s", bank, slot, memo_status());
        }
        return;
    }
    if (!memo_get(bank, slot, &hz, &mode)) {
        app_notice("Bank %d memory %d is empty", bank, slot);
        return;
    }
    app_set_mode(mode);
    app_set_freq(hz);
    app_notice("Bank %d memory %d: %u.%06u MHz %s", bank, slot, (unsigned)(hz / 1000000), (unsigned)(hz % 1000000),
               mode_names[mode]);
}

// A character while a memory sequence is open. Anything unexpected closes it.
static void mem_char(char c) {
    if (c >= '0' && c <= '9') {
        mem_digit(c - '0');
    } else if ((c == 's' || c == 'S') && g_mem_ui == MEM_UI_LOAD) {
        mem_ui_set(MEM_UI_SAVE);
    } else {
        mem_ui_set(MEM_UI_IDLE);
    }
}

// ---- settings menu (F2) --------------------------------------------------------------------------

static const char* slot_name(int slot, const char* prefix, char* buf, size_t len) {
    if (slot) snprintf(buf, len, "%s%d only", prefix, slot);
    else snprintf(buf, len, "AUTO");
    return buf;
}

void menu_item_text(int item, const char** label, char* value, size_t len) {
    int gi = g_set.gain_idx;
    value[0] = 0;
    switch (item) {
        case MENU_PPM:
            *label = "Freq. corr.";
            snprintf(value, len, "%+d ppm", g_set.ppm);
            break;
        case MENU_GAIN:
            *label = "RF gain";
            if (g_stat.direct) snprintf(value, len, "direct");
            else if (gi >= 0 && gi < g_stat.gain_count) snprintf(value, len, "%d.%d dB", g_stat.gains[gi] / 10, g_stat.gains[gi] % 10);
            else snprintf(value, len, "AUTO");
            break;
        case MENU_SQUELCH:
            *label = "Squelch";
            if (g_set.squelch_db <= -120) snprintf(value, len, "off");
            else snprintf(value, len, "%d dBFS", g_set.squelch_db);
            break;
        case MENU_STEP: {
            uint32_t hz = step_table[g_step_idx];
            *label      = "Step";
            if (hz >= 1000000) snprintf(value, len, "%u MHz", (unsigned)(hz / 1000000));
            else if (hz >= 1000) snprintf(value, len, "%u.%02u kHz", (unsigned)(hz / 1000), (unsigned)(hz % 1000 / 10));
            else snprintf(value, len, "%u Hz", (unsigned)hz);
            break;
        }
        case MENU_SPAN:
            *label = "Span";
            snprintf(value, len, "%s", g_set.zoom ? "narrow" : "wide");
            break;
        case MENU_MODE:
            *label = "Mode";
            snprintf(value, len, "%s", mode_names[g_set.mode]);
            break;
        case MENU_DMR_SLOT:
            *label = "DMR slot";
            slot_name(g_set.dmr_slot, "TS", value, len);
            break;
        case MENU_TETRA_SLOT:
            *label = "TETRA slot";
            slot_name(g_set.tetra_slot, "TN", value, len);
            break;
        case MENU_BANK:
            *label = "Mem. bank";
            snprintf(value, len, "%d", g_mem_bank);
            break;
        case MENU_REC_FORMAT:
            *label = "Rec. format";
            snprintf(value, len, "%s", rec_format_name((rec_format_t)g_set.rec_format));
            break;
        case MENU_SDS_LOG:
            *label = "SDS log";
            snprintf(value, len, "%s", g_set.sds_log ? "to SD card" : "off");
            break;
        default:
            *label = "";
            break;
    }
}

static int wrap(int v, int n) {
    return ((v % n) + n) % n;
}

static void menu_change(int item, int dir) {
    switch (item) {
        case MENU_PPM: {
            int ppm = g_set.ppm + dir;
            if (ppm < -200) ppm = -200;
            if (ppm > 200) ppm = 200;
            g_set.ppm = ppm;  // the radio task applies it and retunes
            break;
        }
        case MENU_GAIN:
            // AUTO sits below the lowest step.
            if (g_stat.gain_count <= 0 || g_stat.direct) gain_step(dir);  // says why not
            else if (g_set.gain_idx < 0 && dir > 0) app_set_gain_idx(0);
            else if (g_set.gain_idx >= 0) app_set_gain_idx(g_set.gain_idx + dir);
            break;
        case MENU_SQUELCH:
            // Off sits below the lowest level.
            if (g_set.squelch_db > -120 && g_set.squelch_db + dir < SQUELCH_MIN_DB) {
                squelch_last = SQUELCH_OFF;  // no level to come back to: the next "on" starts from the channel
                squelch_set(SQUELCH_OFF);
            }
            else if (g_set.squelch_db > -120 || dir > 0) squelch_step(dir);
            break;
        case MENU_STEP: {
            int idx = g_step_idx + dir;
            if (idx >= 0 && idx < step_count) g_step_idx = idx;
            break;
        }
        case MENU_SPAN:
            g_set.zoom = !g_set.zoom;
            break;
        case MENU_MODE:
            g_set.mode = (demod_mode_t)wrap((int)g_set.mode + dir, MODE_COUNT);
            break;
        case MENU_DMR_SLOT:
            g_set.dmr_slot = wrap(g_set.dmr_slot + dir, DMR_SLOTS + 1);
            break;
        case MENU_TETRA_SLOT:
            g_set.tetra_slot = wrap(g_set.tetra_slot + dir, TETRA_SLOTS + 1);
            break;
        case MENU_BANK:
            g_mem_bank = wrap(g_mem_bank + dir, MEM_BANKS);
            break;
        case MENU_REC_FORMAT:
            g_set.rec_format = wrap(g_set.rec_format + dir, REC_FMT_COUNT);
            if (g_rec.state != REC_IDLE) app_notice("The running recording keeps its format; the next one is %s", rec_format_name((rec_format_t)g_set.rec_format));
            break;
        case MENU_SDS_LOG:
            g_set.sds_log = !g_set.sds_log;
            if (g_set.sds_log) app_notice("Messages are written to the SD card: %s/<network>/<date>-sds.txt", REC_DIRNAME);
            else app_notice("Messages are not written to the SD card   (F5 still lists them)");
            break;
        default:
            break;
    }
    settings_mark_dirty();
}

static void help_set(bool open) {
    g_help_open = open;
    if (open) {
        g_msgs_open    = false;
        g_mem_ui       = MEM_UI_IDLE;
        g_menu_open    = false;
        g_notice_until = 0;
    }
}

// F5: the list of received messages, in the place of the spectrum like the key list.
static void msgs_set(bool open) {
    g_msgs_open = open;
    if (open) {
        g_msgs_scroll  = 0;
        g_help_open    = false;
        g_mem_ui       = MEM_UI_IDLE;
        g_menu_open    = false;
        g_notice_until = 0;
    }
}

static void msgs_scroll(int dir) {
    int n = msglog_count();
    int v = g_msgs_scroll + dir;
    if (v > n - 1) v = n - 1;
    if (v < 0) v = 0;
    g_msgs_scroll = v;
}

static void menu_set(bool open) {
    g_menu_open = open;
    if (open) {
        g_mem_ui        = MEM_UI_IDLE;
        g_sq_adjust     = false;
        ui_entry_active = false;
        ui_entry[0]     = 0;
        g_notice_until  = 0;
    }
}

static void entry_commit(void) {
    ui_entry_active = false;
    if (ui_entry[0] == 0) return;
    double mhz = strtod(ui_entry, NULL);
    if (mhz >= (double)FREQ_MIN_HZ / 1e6 && mhz <= (double)FREQ_MAX_HZ / 1e6) {
        app_set_freq((uint32_t)(mhz * 1e6 + 0.5));
    }
    ui_entry[0] = 0;
}

static void tune_step(int dir) {
    uint32_t step = step_table[g_step_idx];
    uint32_t f    = g_set.freq_hz;
    app_set_freq(dir > 0 ? f + step : f - step);
}

static void on_char(char c) {
    g_help_open = false;  // the key list goes away with the next key, which does what it always does
    g_msgs_open = false;  // ... and so does the message list
    if (g_mem_ui != MEM_UI_IDLE) {
        mem_char(c);
        return;
    }
    if (g_menu_open) {
        if (c == '+' || c == '=' || c == '-' || c == '_') {
            menu_change(g_menu_sel, (c == '+' || c == '=') ? 1 : -1);
            return;
        }
        menu_set(false);  // and the key does what it always does
    }
    if ((c >= '0' && c <= '9') || c == '.') {
        if (!ui_entry_active) {
            ui_entry_active = true;
            ui_entry[0]     = 0;
        }
        size_t len = strlen(ui_entry);
        if (len < 11 && !(c == '.' && strchr(ui_entry, '.'))) {
            ui_entry[len]     = c;
            ui_entry[len + 1] = 0;
        }
        return;
    }
    // Any other key drops a frequency that was being typed and does what it always does: an entry
    // started by a stray digit must not make the rest of the keyboard look dead.
    ui_entry_active = false;
    ui_entry[0]     = 0;
    switch (c) {
        case 'm':
        case 'M':
            mem_ui_set(MEM_UI_LOAD);
            break;
        case 'l':
        case 'L':
            app_set_mode(MODE_LSB);
            break;
        case 'u':
        case 'U':
            app_set_mode(MODE_USB);
            break;
        case 'n':
        case 'N':
            app_set_mode(MODE_NFM);
            break;
        case 'a':
        case 'A':
            app_set_mode(MODE_AM);
            break;
        case 'w':
        case 'W':
            app_set_mode(MODE_WFM);
            break;
        case 'd':
        case 'D':
            app_set_mode(MODE_DMR);
            break;
        case 't':
        case 'T':
            app_set_mode(MODE_TETRA);
            break;
        case '+':
        case '=':
            if (g_sq_adjust) squelch_step(1);
            else gain_step(1);
            break;
        case '-':
        case '_':
            if (g_sq_adjust) squelch_step(-1);
            else gain_step(-1);
            break;
        case 'g':
        case 'G':
            if (g_set.gain_idx >= 0) app_set_gain_idx(GAIN_AUTO);
            else gain_step(0);
            break;
        case 'z':
        case 'Z':
            g_set.zoom = !g_set.zoom;
            settings_mark_dirty();
            break;
        case ' ':
            if (mode_is_digital(g_set.mode)) slot_next();
            else squelch_toggle();
            break;
        case 's':
        case 'S':
            squelch_adjust_set(!g_sq_adjust);
            break;
        default:
            break;
    }
}

static int     held_key   = 0;  // LEFT / RIGHT and the volume keys repeat while they are held
static int64_t held_since = 0, held_last = 0;

#define REPEAT_AFTER_US  400000
#define REPEAT_TUNE_US   80000
#define REPEAT_VOLUME_US 150000  // 0 to 100 in three seconds

static void hold_start(int key) {
    held_key   = key;
    held_since = held_last = esp_timer_get_time();
}

// LEFT / RIGHT, pressed or repeating: the selected setting while the menu is open, else the frequency.
static void left_right(int key) {
    int dir = key == BSP_INPUT_NAVIGATION_KEY_RIGHT ? 1 : -1;
    if (g_menu_open) menu_change(g_menu_sel, dir);
    else tune_step(dir);
}

static void volume_step(int key) {
    app_set_volume(g_set.volume + (key == BSP_INPUT_NAVIGATION_KEY_VOLUME_UP ? 5 : -5));
}

static void on_nav(int key, uint32_t mods, bool down) {
    if (!down) {
        if (key == held_key) held_key = 0;
        return;
    }
    // The three parts of the space bar also arrive as one ' ' character, which is what is used.
    if (key == BSP_INPUT_NAVIGATION_KEY_SPACE_L || key == BSP_INPUT_NAVIGATION_KEY_SPACE_M ||
        key == BSP_INPUT_NAVIGATION_KEY_SPACE_R) {
        return;
    }
    // Fn (or SHIFT) turns TAB around.
    int tab_dir = (mods & (BSP_INPUT_MODIFIER_FUNCTION | BSP_INPUT_MODIFIER_SHIFT)) ? -1 : 1;
    if (g_help_open) {
        switch (key) {
            case BSP_INPUT_NAVIGATION_KEY_F4:
            case BSP_INPUT_NAVIGATION_KEY_ESC:
            case BSP_INPUT_NAVIGATION_KEY_BACKSPACE:
            case BSP_INPUT_NAVIGATION_KEY_RETURN:
                help_set(false);
                return;
            case BSP_INPUT_NAVIGATION_KEY_F1:
            case BSP_INPUT_NAVIGATION_KEY_F6:
            case BSP_INPUT_NAVIGATION_KEY_VOLUME_UP:
            case BSP_INPUT_NAVIGATION_KEY_VOLUME_DOWN:
                break;  // work as always, the list stays
            default:
                help_set(false);  // and the key does what it normally does
                break;
        }
    }
    if (g_msgs_open) {
        switch (key) {
            case BSP_INPUT_NAVIGATION_KEY_UP:
                msgs_scroll(-1);
                return;
            case BSP_INPUT_NAVIGATION_KEY_DOWN:
                msgs_scroll(1);
                return;
            case BSP_INPUT_NAVIGATION_KEY_F5:
            case BSP_INPUT_NAVIGATION_KEY_ESC:
            case BSP_INPUT_NAVIGATION_KEY_BACKSPACE:
            case BSP_INPUT_NAVIGATION_KEY_RETURN:
                msgs_set(false);
                return;
            case BSP_INPUT_NAVIGATION_KEY_F1:
            case BSP_INPUT_NAVIGATION_KEY_F6:
            case BSP_INPUT_NAVIGATION_KEY_VOLUME_UP:
            case BSP_INPUT_NAVIGATION_KEY_VOLUME_DOWN:
                break;  // work as always, the list stays
            default:
                msgs_set(false);  // and the key does what it normally does
                break;
        }
    }
    if (g_mem_ui != MEM_UI_IDLE) {
        switch (key) {
            case BSP_INPUT_NAVIGATION_KEY_TAB:
                mem_step_bank(tab_dir);
                return;
            case BSP_INPUT_NAVIGATION_KEY_ESC:
            case BSP_INPUT_NAVIGATION_KEY_BACKSPACE:
            case BSP_INPUT_NAVIGATION_KEY_RETURN:
            case BSP_INPUT_NAVIGATION_KEY_F3:
                mem_ui_set(MEM_UI_IDLE);
                return;
            case BSP_INPUT_NAVIGATION_KEY_F1:
            case BSP_INPUT_NAVIGATION_KEY_F6:
            case BSP_INPUT_NAVIGATION_KEY_VOLUME_UP:
            case BSP_INPUT_NAVIGATION_KEY_VOLUME_DOWN:
                break;  // work as always, the sequence stays open
            default:
                mem_ui_set(MEM_UI_IDLE);  // and the key does what it normally does
                break;
        }
    }
    if (g_menu_open) {
        switch (key) {
            case BSP_INPUT_NAVIGATION_KEY_UP:
                g_menu_sel = wrap(g_menu_sel - 1, MENU_COUNT);
                return;
            case BSP_INPUT_NAVIGATION_KEY_DOWN:
                g_menu_sel = wrap(g_menu_sel + 1, MENU_COUNT);
                return;
            case BSP_INPUT_NAVIGATION_KEY_F2:
            case BSP_INPUT_NAVIGATION_KEY_ESC:
            case BSP_INPUT_NAVIGATION_KEY_BACKSPACE:
            case BSP_INPUT_NAVIGATION_KEY_RETURN:
                menu_set(false);
                return;
            default:
                break;  // LEFT / RIGHT change the setting; the other keys work as always, the menu stays
        }
    }
    switch (key) {
        case BSP_INPUT_NAVIGATION_KEY_F1:
            app_exit_to_launcher();
            break;
        case BSP_INPUT_NAVIGATION_KEY_F2:
            menu_set(true);
            break;
        case BSP_INPUT_NAVIGATION_KEY_F3:
            mem_ui_set(MEM_UI_LOAD);
            break;
        case BSP_INPUT_NAVIGATION_KEY_F4:
            help_set(true);
            break;
        case BSP_INPUT_NAVIGATION_KEY_F5:
            msgs_set(true);
            break;
        case BSP_INPUT_NAVIGATION_KEY_F6:
            if (mods & BSP_INPUT_MODIFIER_FUNCTION) shot_save();
            else rec_toggle();
            break;
        case BSP_INPUT_NAVIGATION_KEY_LEFT:
        case BSP_INPUT_NAVIGATION_KEY_RIGHT:
            left_right(key);
            hold_start(key);
            break;
        case BSP_INPUT_NAVIGATION_KEY_UP:
            if (g_step_idx < step_count - 1) g_step_idx = g_step_idx + 1;
            settings_mark_dirty();
            break;
        case BSP_INPUT_NAVIGATION_KEY_DOWN:
            if (g_step_idx > 0) g_step_idx = g_step_idx - 1;
            settings_mark_dirty();
            break;
        case BSP_INPUT_NAVIGATION_KEY_TAB:
            app_set_mode((demod_mode_t)wrap((int)g_set.mode + tab_dir, MODE_COUNT));
            break;
        case BSP_INPUT_NAVIGATION_KEY_RETURN:
            if (ui_entry_active) entry_commit();
            else squelch_adjust_set(false);
            break;
        case BSP_INPUT_NAVIGATION_KEY_ESC:
            if (!ui_entry_active) squelch_adjust_set(false);
            ui_entry_active = false;
            ui_entry[0]     = 0;
            break;
        case BSP_INPUT_NAVIGATION_KEY_BACKSPACE:
            if (ui_entry_active) {
                size_t len = strlen(ui_entry);
                if (len > 0) ui_entry[len - 1] = 0;
                else ui_entry_active = false;
            }
            break;
        case BSP_INPUT_NAVIGATION_KEY_VOLUME_UP:
        case BSP_INPUT_NAVIGATION_KEY_VOLUME_DOWN:
            volume_step(key);
            hold_start(key);
            break;
        default:
            break;
    }
}

static void handle_input(void) {
    bsp_input_event_t event;
    while (xQueueReceive(input_queue, &event, 0) == pdTRUE) {
        if (event.type == INPUT_EVENT_TYPE_NAVIGATION) {
            on_nav((int)event.args_navigation.key, event.args_navigation.modifiers, event.args_navigation.state);
        } else if (event.type == INPUT_EVENT_TYPE_KEYBOARD) {
            // Control characters also arrive as navigation events; only take printable ones here.
            char c = event.args_keyboard.ascii;
            if (c >= 0x20 && c < 0x7f) on_char(c);
        } else if (event.type == INPUT_EVENT_TYPE_ACTION && event.args_action.type == BSP_INPUT_ACTION_TYPE_AUDIO_JACK) {
            audio_apply_volume();  // speaker amplifier off with headphones, on without
        }
    }
    inject_t inj;
    while (xQueueReceive(inject_queue, &inj, 0) == pdTRUE) {
        if (inj.nav_key) {
            if (inj.edge >= 0) on_nav(inj.nav_key, inj.mods, true);
            if (inj.edge <= 0) on_nav(inj.nav_key, inj.mods, false);
        } else if (inj.ascii >= 0x20 && inj.ascii < 0x7f) {
            on_char(inj.ascii);
        }
    }
    if (g_mem_ui != MEM_UI_IDLE && esp_timer_get_time() > mem_ui_until) g_mem_ui = MEM_UI_IDLE;
    if (g_sq_adjust && mode_is_digital(g_set.mode)) g_sq_adjust = false;  // nothing left to adjust
    if (held_key) {
        int64_t now    = esp_timer_get_time();
        bool    volume = held_key == BSP_INPUT_NAVIGATION_KEY_VOLUME_UP || held_key == BSP_INPUT_NAVIGATION_KEY_VOLUME_DOWN;
        if (now - held_since > REPEAT_AFTER_US && now - held_last > (volume ? REPEAT_VOLUME_US : REPEAT_TUNE_US)) {
            held_last = now;
            if (volume) volume_step(held_key);
            else left_right(held_key);
        }
    }
}

void app_main(void) {
    gpio_install_isr_service(0);

    esp_err_t res = nvs_flash_init();
    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        res = nvs_flash_erase();
        if (res == ESP_OK) res = nvs_flash_init();
    }
    if (res != ESP_OK) ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(res));

    const bsp_configuration_t bsp_configuration = {
        .display =
            {
                .requested_color_format = BSP_DISPLAY_COLOR_FORMAT_24_888RGB,
                .num_fbs                = 2,
            },
    };
    res = bsp_device_initialize(&bsp_configuration);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "BSP init failed: %s", esp_err_to_name(res));
        return;
    }

    radio_reserve_memory();

    // Mute before anything else: the BSP has just re-initialised the codec.
    audio_init();

    // The receiver does not use WiFi / BLE; keep the C6 radio off.
    bsp_power_set_radio_state(BSP_POWER_RADIO_STATE_OFF);
    leds_init();

    settings_load();
    memo_init();
    rec_init();
    msglog_init();
    shot_init();
    inject_queue = xQueueCreate(32, sizeof(inject_t));
    if (bsp_input_get_queue(&input_queue) != ESP_OK || input_queue == NULL || inject_queue == NULL) {
        ESP_LOGE(TAG, "no input queue");
        return;
    }
    console_start();

    if (!ui_init()) {
        ESP_LOGE(TAG, "UI init failed");
        vTaskDelay(pdMS_TO_TICKS(2000));
        bsp_device_restart_to_launcher();
        return;
    }
    dec_init();
    radio_start();
    ESP_LOGI(TAG, "Tanmatsu SDR %s started", APP_VERSION);

    const int64_t frame_us = 33000;
    for (;;) {
        int64_t t0 = esp_timer_get_time();
        handle_input();
        ui_frame();
        leds_update();
        if (settings_dirty && esp_timer_get_time() - settings_dirty_at > 3000000) settings_save();
        int64_t spare = frame_us - (esp_timer_get_time() - t0);
        vTaskDelay(pdMS_TO_TICKS(spare > 2000 ? (TickType_t)(spare / 1000) : 2));
    }
}
