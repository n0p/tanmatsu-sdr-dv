// SPDX-License-Identifier: AGPL-3.0-only
// Line-based remote console on the ESP32-P4 USB-Serial-JTAG port. It exists for development and
// testing: remote control, status, sample captures and screenshots (see tools/tsdr.py).
//
// Replies are lines starting with '#'. Binary payloads are sent as base64 between
// "#<KIND> BEGIN ... bytes=N" and "#<KIND> END crc32=XXXXXXXX".

#include <dirent.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "app.h"
#include "bsp/input.h"
#include "decoder.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "recfmt.h"
#include "ui.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void con_write(const char* data, size_t len) {
    while (len > 0) {
        int n = usb_serial_jtag_write_bytes(data, len, pdMS_TO_TICKS(500));
        if (n <= 0) return;  // host is not listening
        data += n;
        len  -= (size_t)n;
    }
}

static void con_printf(const char* fmt, ...) {
    char    line[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n >= (int)sizeof(line)) n = sizeof(line) - 1;
    con_write(line, (size_t)n);
}

static void dump_base64(const char* kind, const char* header, const uint8_t* buf, size_t len) {
    static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    static char       line[4096 + 2];
    esp_log_level_set("*", ESP_LOG_NONE);
    vTaskDelay(pdMS_TO_TICKS(20));
    con_printf("#%s BEGIN %s bytes=%u\n", kind, header, (unsigned)len);
    uint32_t crc = esp_rom_crc32_le(0, buf, len);
    size_t   pos = 0;
    while (pos < len) {
        size_t chunk = len - pos;
        if (chunk > 3072) chunk = 3072;
        size_t o = 0;
        size_t i = 0;
        for (; i + 2 < chunk; i += 3) {
            uint32_t v = ((uint32_t)buf[pos + i] << 16) | ((uint32_t)buf[pos + i + 1] << 8) | buf[pos + i + 2];
            line[o++]  = b64[(v >> 18) & 63];
            line[o++]  = b64[(v >> 12) & 63];
            line[o++]  = b64[(v >> 6) & 63];
            line[o++]  = b64[v & 63];
        }
        if (i < chunk) {
            uint32_t v = (uint32_t)buf[pos + i] << 16;
            if (i + 1 < chunk) v |= (uint32_t)buf[pos + i + 1] << 8;
            line[o++] = b64[(v >> 18) & 63];
            line[o++] = b64[(v >> 12) & 63];
            line[o++] = (i + 1 < chunk) ? b64[(v >> 6) & 63] : '=';
            line[o++] = '=';
        }
        line[o++] = '\n';
        con_write(line, o);
        pos += chunk;
    }
    con_printf("#%s END crc32=%08x\n", kind, (unsigned)crc);
    esp_log_level_set("*", ESP_LOG_INFO);
}

static const char* status_name(radio_status_t s) {
    switch (s) {
        case RADIO_NO_DEVICE:
            return "nodev";
        case RADIO_STARTING:
            return "starting";
        case RADIO_STREAMING:
            return "streaming";
        default:
            return "error";
    }
}

static void cmd_stat(void) {
    int gi = g_set.gain_idx;
    con_printf(
        "#STAT ver=%s status=%s freq=%u mode=%s gain_idx=%d gain=%d ppm=%d sq=%d vol=%d zoom=%d step=%d sps=%u "
        "drops=%u cpu=%u dec=%u chan_db=%.1f adc=%u..%u clip=%u blocks=%u sql_open=%d fps=%.1f heap_int=%u heap_blk=%u heap_psram=%u\n",
        APP_VERSION, status_name(g_stat.status), (unsigned)g_set.freq_hz, mode_names[g_set.mode], gi,
        (gi >= 0 && gi < g_stat.gain_count) ? g_stat.gains[gi] : -1, g_set.ppm, g_set.squelch_db, g_set.volume,
        (int)g_set.zoom, g_step_idx, (unsigned)g_stat.effective_sps, (unsigned)g_stat.drops,
        (unsigned)g_stat.cpu_permille, (unsigned)g_stat.dec_permille, (double)g_stat.chan_db, g_stat.adc_min, g_stat.adc_max,
        (unsigned)g_stat.clip_permille, (unsigned)g_stat.blocks, (int)g_stat.squelch_open, (double)g_ui_fps,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    con_printf("#TEXT %s\n", g_stat.status_text);
    con_printf("#TUNE step_hz=%u direct=%d rtl_agc=%d soft_off=%d bank=%d mem_ui=%d ts_dmr=%d ts_tetra=%d speaker_ts=%d "
               "notice=%s\n",
               (unsigned)step_table[g_step_idx], (int)g_stat.direct, (int)g_stat.rtl_agc, (int)g_stat.soft_off_hz,
               g_mem_bank, (int)g_mem_ui, g_set.dmr_slot, g_set.tetra_slot, dec_speaker_slot(),
               esp_timer_get_time() < g_notice_until ? g_notice : "");
    con_printf("#LED a=%06x b=%06x snr=%.1f setting=%d full=%d sends=%u err=%d\n", (unsigned)g_led.a, (unsigned)g_led.b,
               (double)g_led.snr_db, g_led.setting, g_led.full, (unsigned)g_led.sends, g_led.err);
    char help[160];
    bool prompt;
    ui_help_text(help, sizeof(help), &prompt);
    con_printf("#UI menu=%d sel=%d sq_adjust=%d help=%d msgs=%d scroll=%d prompt=%d line=%s\n", (int)g_menu_open,
               g_menu_sel, (int)g_sq_adjust, (int)g_help_open, (int)g_msgs_open, g_msgs_scroll, (int)prompt, help);
    con_printf("#MSG log=%d seen=%u kept=%d dup=%u written=%u errors=%u card=%d file=%s\n", (int)g_set.sds_log,
               (unsigned)g_msglog.seen, msglog_count(), (unsigned)g_msglog.dup, (unsigned)g_msglog.written,
               (unsigned)g_msglog.errors, (int)g_msglog.card, g_msglog.file);
    time_t    now = time(NULL);
    struct tm tm;
    char      stamp[24];
    localtime_r(&now, &tm);
    strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm);
    con_printf("#REC state=%d fmt=%d ms=%u bytes=%u dropped=%u enc_us_max=%u enc_us_avg=%u cx=%d stack_free=%u now=%s "
               "name=%s\n",
               (int)g_rec.state, g_set.rec_format, (unsigned)g_rec.ms, (unsigned)g_rec.bytes, (unsigned)g_rec.dropped,
               (unsigned)g_rec.enc_us_max, (unsigned)g_rec.enc_us_avg, recw_opus_complexity, (unsigned)g_rec.stack_free,
               stamp, g_rec.name);
    audio_stats_t au;
    audio_get_stats(&au);
    con_printf("#AUDIO i2s=%d init_err=%d codec_pct=%d amp=%d headphones=%d vol_err=%d amp_err=%d chunks=%u "
               "from_ring=%u write_err=%u last_err=%d underruns=%u gaps=%u overflows=%u fill=%u low=%u\n",
               (int)au.i2s_ok, au.init_err, au.codec_pct, (int)au.amplifier, (int)au.headphones, au.vol_err, au.amp_err,
               (unsigned)au.chunks, (unsigned)au.chunks_audio, (unsigned)au.write_errors, au.last_write_err,
               (unsigned)au.underruns, (unsigned)au.gaps, (unsigned)au.overflows, (unsigned)au.fill,
               (unsigned)au.low_water);
    char dec[DEC_LINES][DEC_LINE_LEN];
    memset(dec, 0, sizeof(dec));
    dec_get_lines(g_set.mode, dec);
    for (int i = 0; i < DEC_LINES; i++) {
        if (dec[i][0]) con_printf("#DEC%d %s\n", i, dec[i]);
    }
}

static void cmd_gains(void) {
    char line[400];
    int  o = snprintf(line, sizeof(line), "#GAINS n=%d:", g_stat.gain_count);
    for (int i = 0; i < g_stat.gain_count && o < (int)sizeof(line) - 8; i++) {
        o += snprintf(line + o, sizeof(line) - (size_t)o, " %d", g_stat.gains[i]);
    }
    con_printf("%s\n", line);
}

static void cmd_cap(char* args) {
    char* tap_s  = strtok(args, " ");
    char* ms_s   = strtok(NULL, " ");
    char* opt    = strtok(NULL, " ");
    char* opt_v  = strtok(NULL, " ");
    char* wait_s = strtok(NULL, " ");
    if (!tap_s || !ms_s) {
        con_printf("#ERR usage: cap <raw|if|ch|fm|audio> <ms> [trig <dBFS> [wait_s]]\n");
        return;
    }
    int tap = -1;
    for (int i = 0; i < TAP_COUNT; i++) {
        if (strcmp(tap_s, tap_names[i]) == 0) tap = i;
    }
    if (tap < 0) {
        con_printf("#ERR unknown tap\n");
        return;
    }
    float trig   = -1000.0f;
    int   wait_s_ = 5;
    if (opt && opt_v && strcmp(opt, "trig") == 0) {
        trig    = strtof(opt_v, NULL);
        wait_s_ = wait_s ? atoi(wait_s) : 60;
    }
    uint32_t ms = (uint32_t)atoi(ms_s);
    if (!cap_arm((tap_t)tap, ms, trig)) {
        con_printf("#ERR capture buffer allocation failed\n");
        return;
    }
    con_printf("#CAP ARMED tap=%s ms=%u trig=%.1f\n", tap_names[tap], (unsigned)ms, (double)trig);
    int budget = (wait_s_ * 1000 + (int)ms + 2000) / 20;
    while (g_cap.state != CAP_DONE && budget-- > 0) vTaskDelay(pdMS_TO_TICKS(20));
    if (g_cap.state != CAP_DONE) {
        if (g_cap.state != CAP_RECORDING || g_cap.fill == 0) {
            con_printf("#ERR capture timed out (state %d, %u of %u bytes)\n", (int)g_cap.state, (unsigned)g_cap.fill,
                       (unsigned)g_cap.size);
            cap_release();
            return;
        }
        // The tap stopped delivering (decoded speech ends with the call): hand over what there is.
        g_cap.state = CAP_DONE;
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    char hdr[160];
    snprintf(hdr, sizeof(hdr), "tap=%s rate=%u fmt=%s freq=%u mode=%s", tap_names[tap], (unsigned)g_cap.rate,
             cap_format((tap_t)tap), (unsigned)g_cap.freq_hz, mode_names[g_cap.mode]);
    dump_base64("CAP", hdr, g_cap.buf, g_cap.fill);
    cap_release();
}

static void cmd_shot(void) {
    if (!shot_grab()) {
        con_printf("#ERR no screenshot (no buffer, or the display loop did not deliver a frame)\n");
        return;
    }
    dump_base64("SHOT", "w=480 h=800 fmt=bgr888 rot=270", g_shot_buf, g_shot_size);
    shot_release();
}

// "msgs": the list that F5 shows, newest first.
static void cmd_msgs(void) {
    static msg_t m;
    int          n = msglog_count();
    con_printf("#MSGS n=%d\n", n);
    for (int i = 0; i < n && msglog_get(i, &m); i++) {
        time_t    t = (time_t)m.time;
        struct tm tm;
        char      stamp[24];
        localtime_r(&t, &tm);
        strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm);
        con_printf("#MSG%d %s %u %s plmn=%d/%u-%u card=%d %s > %s %s: %.300s\n", i, stamp, (unsigned)m.freq_hz, m.sys,
                   (int)m.plmn, m.mcc, m.mnc, (int)m.to_card, m.from, m.to, m.kind, m.text);
    }
}

// "sdstest <mcc> <mnc> <text>": a message as if it had been received, to try the list and the log
// without waiting for one on the air. mcc 0 = from a cell whose network is not known.
static void cmd_sdstest(char* args) {
    char* mcc_s = strtok(args, " ");
    char* mnc_s = strtok(NULL, " ");
    char* text  = strtok(NULL, "");
    if (!mcc_s || !mnc_s || !text) {
        con_printf("#ERR usage: sdstest <mcc> <mnc> <text>\n");
        return;
    }
    unsigned mcc = (unsigned)atoi(mcc_s), mnc = (unsigned)atoi(mnc_s);
    // "\n" in the text is a line feed, to see what the log makes of one.
    for (char* q = text; (q = strstr(q, "\\n")) != NULL;) {
        *q = '\n';
        memmove(q + 1, q + 2, strlen(q + 2) + 1);
    }
    msglog_put("TEST", mcc != 0, mcc, mnc, "1234567", "7654321", "text", text);
    con_printf("#OK sdstest seen=%u\n", (unsigned)g_msglog.seen);
}

static void cmd_key(const char* arg) {
    static const struct {
        const char* name;
        int         key;
    } keys[] = {
        {"left", BSP_INPUT_NAVIGATION_KEY_LEFT},     {"right", BSP_INPUT_NAVIGATION_KEY_RIGHT},
        {"up", BSP_INPUT_NAVIGATION_KEY_UP},         {"down", BSP_INPUT_NAVIGATION_KEY_DOWN},
        {"esc", BSP_INPUT_NAVIGATION_KEY_ESC},       {"return", BSP_INPUT_NAVIGATION_KEY_RETURN},
        {"tab", BSP_INPUT_NAVIGATION_KEY_TAB},       {"backspace", BSP_INPUT_NAVIGATION_KEY_BACKSPACE},
        {"f2", BSP_INPUT_NAVIGATION_KEY_F2},         {"f3", BSP_INPUT_NAVIGATION_KEY_F3},
        {"f4", BSP_INPUT_NAVIGATION_KEY_F4},         {"f5", BSP_INPUT_NAVIGATION_KEY_F5},
        {"f6", BSP_INPUT_NAVIGATION_KEY_F6},
    };
    // "fn-tab", "shift-tab": the key with a modifier held.
    uint32_t    mods = 0;
    const char* name = arg;
    if (strncmp(name, "fn-", 3) == 0) {
        mods  = BSP_INPUT_MODIFIER_FUNCTION;
        name += 3;
    } else if (strncmp(name, "shift-", 6) == 0) {
        mods  = BSP_INPUT_MODIFIER_SHIFT_L;
        name += 6;
    }
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
        if (strcmp(name, keys[i].name) == 0) {
            app_inject_key(keys[i].key, mods, 0);
            con_printf("#OK key %s\n", arg);
            return;
        }
    }
    if (strcmp(arg, "space") == 0) {
        app_inject_key(0, 0, ' ');
        con_printf("#OK key space\n");
        return;
    }
    // Anything else is typed as text. Volume keys are deliberately not injectable.
    for (const char* p = arg; *p; p++) app_inject_key(0, 0, *p);
    con_printf("#OK typed %s\n", arg);
}

// "hold left 1500": LEFT or RIGHT kept down for that many milliseconds, to see the key repeat.
static void cmd_hold(const char* arg) {
    int key = strncmp(arg, "left", 4) == 0 ? BSP_INPUT_NAVIGATION_KEY_LEFT
            : strncmp(arg, "right", 5) == 0 ? BSP_INPUT_NAVIGATION_KEY_RIGHT : 0;
    const char* sp = strchr(arg, ' ');
    int         ms = sp ? atoi(sp + 1) : 0;
    if (!key || ms < 1 || ms > 5000) {
        con_printf("#ERR hold left|right <1..5000 ms>\n");
        return;
    }
    app_inject_key_edge(key, true);
    vTaskDelay(pdMS_TO_TICKS(ms));
    app_inject_key_edge(key, false);
    con_printf("#OK hold %s\n", arg);
}

// ---- recordings on the SD card ---------------------------------------------------------------------

static void cmd_menu(void) {
    for (int i = 0; i < MENU_COUNT; i++) {
        const char* label = "";
        char        value[32];
        menu_item_text(i, &label, value, sizeof(value));
        con_printf("#MENU %d%s %s = %s\n", i, (g_menu_open && g_menu_sel == i) ? "*" : "", label, value);
    }
}

// A name inside the app's folder on the card, or inside one of the folders in it; nothing else.
static bool rec_path(const char* name, char* path, size_t len) {
    const char* slash = strchr(name, '/');
    if (!*name || strchr(name, '\\') || name[0] == '.' || name[0] == '/' || strstr(name, "..") ||
        (slash && (slash[1] == 0 || slash[1] == '.' || strchr(slash + 1, '/')))) {
        con_printf("#ERR usage: <file name in %s, as \"ls\" prints it>\n", REC_DIR);
        return false;
    }
    snprintf(path, len, "%s/%s", REC_DIR, name);
    return true;
}

// The files in `sub` of the app's folder ("" = the folder itself), with the names "get" and "rm" take.
static int ls_dir(const char* sub, int* dirs) {
    char dpath[96];
    snprintf(dpath, sizeof(dpath), "%s%s%s", REC_DIR, *sub ? "/" : "", sub);
    DIR* dir = opendir(dpath);
    if (!dir) return -1;
    int            n = 0;
    struct dirent* e;
    while ((e = readdir(dir)) != NULL) {
        char        path[400], rel[300];
        struct stat st;
        if (e->d_name[0] == '.') continue;
        snprintf(rel, sizeof(rel), "%s%s%s", sub, *sub ? "/" : "", e->d_name);
        snprintf(path, sizeof(path), "%s/%s", REC_DIR, rel);
        if (stat(path, &st) != 0) continue;
        if (S_ISREG(st.st_mode)) {
            con_printf("#F %u %s\n", (unsigned)st.st_size, rel);
            n++;
        } else if (S_ISDIR(st.st_mode) && !*sub && dirs) {
            con_printf("#D %s\n", rel);
            (*dirs)++;
        }
    }
    closedir(dir);
    return n;
}

static void cmd_ls(void) {
    char why[64];
    if (!rec_sd_acquire(why, sizeof(why))) {
        con_printf("#ERR %s\n", why);
        return;
    }
    int dirs = 0;
    int n    = ls_dir("", &dirs);
    if (n >= 0 && dirs) {
        // One level down: the network folders. The names are read first, the directory is not kept
        // open while another one is being read.
        static char names[16][16];
        int         count = 0;
        char        dpath[96];
        snprintf(dpath, sizeof(dpath), "%s", REC_DIR);
        DIR* dir = opendir(dpath);
        if (dir) {
            struct dirent* e;
            while (count < 16 && (e = readdir(dir)) != NULL) {
                char        path[400];
                struct stat st;
                if (e->d_name[0] == '.' || strlen(e->d_name) >= sizeof(names[0])) continue;
                snprintf(path, sizeof(path), "%s/%s", REC_DIR, e->d_name);
                if (stat(path, &st) == 0 && S_ISDIR(st.st_mode)) snprintf(names[count++], sizeof(names[0]), "%s", e->d_name);
            }
            closedir(dir);
        }
        for (int i = 0; i < count; i++) {
            int m = ls_dir(names[i], NULL);
            if (m > 0) n += m;
        }
    }
    struct stat st;
    con_printf("#LS dir=%s files=%d dirs=%d%s%s\n", REC_DIR, n < 0 ? 0 : n, dirs, n < 0 ? " (no such directory)" : "",
               stat(REC_DIR_OLD, &st) == 0 ? " (old folder " REC_DIR_OLD " also exists)" : "");
    rec_sd_release();
}

static void cmd_get(const char* name) {
    char path[320], why[64];
    if (!rec_path(name, path, sizeof(path))) return;
    if (!rec_sd_acquire(why, sizeof(why))) {
        con_printf("#ERR %s\n", why);
        return;
    }
    struct stat st;
    FILE*       f   = NULL;
    uint8_t*    buf = NULL;
    if (stat(path, &st) != 0 || (f = fopen(path, "rb")) == NULL) {
        con_printf("#ERR no such file\n");
    } else if (st.st_size > 8 * 1024 * 1024 || (buf = heap_caps_malloc((size_t)st.st_size + 1, MALLOC_CAP_SPIRAM)) == NULL) {
        con_printf("#ERR file too large for the console (%u bytes)\n", (unsigned)st.st_size);
    } else {
        size_t got = fread(buf, 1, (size_t)st.st_size, f);
        char   hdr[96];
        snprintf(hdr, sizeof(hdr), "name=%s", name);
        dump_base64("FILE", hdr, buf, got);
    }
    if (f) fclose(f);
    heap_caps_free(buf);
    rec_sd_release();
}

static void cmd_rm(const char* name) {
    char path[320], why[64];
    if (!rec_path(name, path, sizeof(path))) return;
    if (!rec_sd_acquire(why, sizeof(why))) {
        con_printf("#ERR %s\n", why);
        return;
    }
    struct stat st;
    if (g_rec.state != REC_IDLE && strcmp(name, g_rec.name) == 0) con_printf("#ERR that file is being recorded\n");
    else if (stat(path, &st) != 0) con_printf("#ERR no such file\n");
    else if (S_ISDIR(st.st_mode) ? rmdir(path) == 0 : unlink(path) == 0) con_printf("#OK removed %s\n", name);
    else con_printf("#ERR cannot remove %s%s\n", name, S_ISDIR(st.st_mode) ? " (a folder must be empty)" : "");
    rec_sd_release();
}

static void handle_line(char* line) {
    while (*line == ' ') line++;
    char* args = strchr(line, ' ');
    if (args) {
        *args++ = 0;
        while (*args == ' ') args++;
    } else {
        args = line + strlen(line);
    }

    if (strcmp(line, "stat") == 0) {
        cmd_stat();
    } else if (strcmp(line, "gains") == 0) {
        cmd_gains();
    } else if (strcmp(line, "freq") == 0 && *args) {
        app_set_freq((uint32_t)strtoul(args, NULL, 10));
        con_printf("#OK freq=%u\n", (unsigned)g_set.freq_hz);
    } else if (strcmp(line, "mode") == 0 && *args) {
        int m = mode_from_name(args);
        if (m < 0) {
            con_printf("#ERR unknown mode\n");
            return;
        }
        app_set_mode((demod_mode_t)m);
        con_printf("#OK mode=%s\n", mode_names[m]);
    } else if (strcmp(line, "gain") == 0 && *args) {
        // "gain auto" or "gain <index into the gain ladder>"
        app_set_gain_idx(strcmp(args, "auto") == 0 ? GAIN_AUTO : atoi(args));
        con_printf("#OK gain_idx=%d\n", g_set.gain_idx);
    } else if (strcmp(line, "ppm") == 0 && *args) {
        int ppm = atoi(args);
        if (ppm < -200) ppm = -200;
        if (ppm > 200) ppm = 200;
        g_set.ppm = ppm;
        settings_mark_dirty();
        con_printf("#OK ppm=%d\n", ppm);
    } else if (strcmp(line, "sq") == 0 && *args) {
        g_set.squelch_db = atoi(args);
        settings_mark_dirty();
        con_printf("#OK sq=%d\n", g_set.squelch_db);
    } else if (strcmp(line, "zoom") == 0 && *args) {
        g_set.zoom = atoi(args) != 0;
        settings_mark_dirty();
        con_printf("#OK zoom=%d\n", (int)g_set.zoom);
    } else if (strcmp(line, "vol") == 0 && *args) {
        app_set_volume(atoi(args));
        con_printf("#OK vol=%d\n", g_set.volume);
    } else if (strcmp(line, "hold") == 0 && *args) {
        cmd_hold(args);
    } else if (strcmp(line, "key") == 0 && *args) {
        cmd_key(args);
    } else if (strcmp(line, "cap") == 0) {
        cmd_cap(args);
    } else if (strcmp(line, "declog") == 0) {
        static char hist[DEC_LOG_LINES][DEC_LINE_LEN];
        int         n = dec_get_log(hist, DEC_LOG_LINES);
        con_printf("#DECLOG n=%d\n", n);
        for (int i = n - 1; i >= 0; i--) con_printf("#EV %s\n", hist[i]);
    } else if (strcmp(line, "mem") == 0) {
        con_printf("#MEM bank=%d file=%s status=%s\n", g_mem_bank, MEM_FILE, memo_status());
        for (int b = 0; b < MEM_BANKS; b++) {
            for (int s = 0; s < MEM_SLOTS; s++) {
                uint32_t     hz;
                demod_mode_t mode;
                if (memo_get(b, s, &hz, &mode)) con_printf("#M %d %d %u %s\n", b, s, (unsigned)hz, mode_names[mode]);
            }
        }
    } else if (strcmp(line, "shot") == 0) {
        cmd_shot();
    } else if (strcmp(line, "menu") == 0) {
        cmd_menu();
    } else if (strcmp(line, "recfmt") == 0 && *args) {
        int fmt = atoi(args);
        if (fmt >= 0 && fmt < REC_FMT_COUNT) {
            g_set.rec_format = fmt;
            settings_mark_dirty();
        }
        con_printf("#OK recfmt=%d %s\n", g_set.rec_format, rec_format_name((rec_format_t)g_set.rec_format));
    } else if (strcmp(line, "reccx") == 0 && *args) {
        // Encoder effort of the next Opus recording, to measure what it costs. Not saved.
        int cx = atoi(args);
        if (cx >= 0 && cx <= 10) recw_opus_complexity = cx;
        con_printf("#OK reccx=%d\n", recw_opus_complexity);
    } else if (strcmp(line, "sdslog") == 0 && *args) {
        g_set.sds_log = atoi(args) != 0;
        settings_mark_dirty();
        con_printf("#OK sdslog=%d\n", (int)g_set.sds_log);
    } else if (strcmp(line, "sdstest") == 0) {
        cmd_sdstest(args);
    } else if (strcmp(line, "msgs") == 0) {
        cmd_msgs();
    } else if (strcmp(line, "ls") == 0) {
        cmd_ls();
    } else if (strcmp(line, "get") == 0) {
        cmd_get(args);
    } else if (strcmp(line, "rm") == 0) {
        cmd_rm(args);
    } else if (strcmp(line, "exit") == 0) {
        con_printf("#OK exit\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        app_exit_to_launcher();
    } else if (*line) {
        con_printf("#ERR unknown command '%s' (stat declog msgs mem menu gains freq mode gain ppm sq zoom vol key hold cap shot recfmt sdslog sdstest ls get rm exit)\n", line);
    }
}

static void console_task(void* arg) {
    (void)arg;
    static char line[160];
    size_t      len = 0;
    for (;;) {
        char c;
        int  n = usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(100));
        if (n <= 0) continue;
        if (c == '\r' || c == '\n') {
            line[len] = 0;
            if (len > 0) handle_line(line);
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = c;
        }
    }
}

void console_start(void) {
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size                  = 16384;
    cfg.rx_buffer_size                  = 1024;
    if (usb_serial_jtag_driver_install(&cfg) != ESP_OK) {
        ESP_LOGW("console", "USB-Serial-JTAG driver install failed, console disabled");
        return;
    }
    usb_serial_jtag_vfs_use_driver();
    xTaskCreatePinnedToCore(console_task, "console", 8192, NULL, 3, NULL, 0);
}
