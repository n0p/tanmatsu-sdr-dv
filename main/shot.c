// SPDX-License-Identifier: AGPL-3.0-only
// Screenshots. Fn+F6 writes what is on the display to a PNG file on the SD card,
//
//   /sd/tanmatsu-sdr-dv/yyyymmdd-hhmmss-ffffff-mode.png      800 x 480, 24 bit
//
// and the development console fetches the same frame copy over USB ("shot").
//
// The display loop copies a finished frame into a buffer when asked (ui.c); a low-priority task
// turns it the right way up, compresses it with the deflate coder in the ESP32-P4's ROM (miniz) and
// writes the file. That takes a second or two, during which the receiver carries on.

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <ctype.h>
#include "app.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "miniz.h"  // esp_rom: the functions are in the ROM

static const char TAG[] = "shot";

#define RAW_W      480  // the panel as it is wired: 800 lines of 480 pixels, B G R,
#define RAW_H      800  // what the app shows at (x, y) is pixel 479 - y of line x
#define PNG_W      RAW_H
#define PNG_H      RAW_W
#define TASK_STACK (12 * 1024)

volatile bool g_shot_req;
volatile bool g_shot_ready;
uint8_t*      g_shot_buf;
size_t        g_shot_size;

static SemaphoreHandle_t buf_lock;  // one user of the buffer at a time: this task or the console
static TaskHandle_t      task;
static volatile bool     busy;

bool shot_grab(void) {
    if (!g_shot_buf || !buf_lock) return false;
    if (xSemaphoreTake(buf_lock, pdMS_TO_TICKS(8000)) != pdTRUE) return false;
    g_shot_ready = false;
    g_shot_req   = true;
    for (int i = 0; i < 100 && !g_shot_ready; i++) vTaskDelay(pdMS_TO_TICKS(20));
    if (!g_shot_ready) {
        g_shot_req = false;
        xSemaphoreGive(buf_lock);
        return false;
    }
    return true;
}

void shot_release(void) {
    g_shot_ready = false;
    xSemaphoreGive(buf_lock);
}

// ---- PNG -------------------------------------------------------------------------------------------

static void be32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static bool chunk(FILE* f, const char type[4], const void* data, size_t len) {
    uint8_t head[8], tail[4];
    be32(head, (uint32_t)len);
    memcpy(head + 4, type, 4);
    uint32_t crc = esp_rom_crc32_le(0, head + 4, 4);
    if (len) crc = esp_rom_crc32_le(crc, data, (uint32_t)len);
    be32(tail, crc);
    return fwrite(head, 1, 8, f) == 8 && (len == 0 || fwrite(data, 1, len, f) == len) && fwrite(tail, 1, 4, f) == 4;
}

// The compressor hands over its output a buffer at a time: each becomes one IDAT chunk.
static mz_bool put_idat(const void* buf, int len, void* user) {
    return len <= 0 || chunk((FILE*)user, "IDAT", buf, (size_t)len) ? MZ_TRUE : MZ_FALSE;
}

static bool write_png(FILE* f, const uint8_t* raw) {
    static const uint8_t sig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
    uint8_t              ihdr[13];
    be32(ihdr, PNG_W);
    be32(ihdr + 4, PNG_H);
    ihdr[8]  = 8;  // bits per sample
    ihdr[9]  = 2;  // RGB
    ihdr[10] = 0;
    ihdr[11] = 0;
    ihdr[12] = 0;
    if (fwrite(sig, 1, 8, f) != 8 || !chunk(f, "IHDR", ihdr, sizeof(ihdr))) return false;

    tdefl_compressor* comp = heap_caps_malloc(sizeof(tdefl_compressor), MALLOC_CAP_SPIRAM);
    uint8_t*          row  = heap_caps_malloc(1 + PNG_W * 3, MALLOC_CAP_SPIRAM);
    bool              ok   = comp && row;
    // A few probes only: the picture is mostly flat colour and long runs, which the first probe finds.
    if (ok) ok = tdefl_init(comp, put_idat, f, TDEFL_WRITE_ZLIB_HEADER | TDEFL_GREEDY_PARSING_FLAG | 8) == TDEFL_STATUS_OKAY;
    for (int y = 0; ok && y < PNG_H; y++) {
        const uint8_t* src = raw + (size_t)(RAW_W - 1 - y) * 3;
        uint8_t*       dst = row;
        *dst++             = 0;  // filter: none
        for (int x = 0; x < PNG_W; x++, src += RAW_W * 3) {
            *dst++ = src[2];
            *dst++ = src[1];
            *dst++ = src[0];
        }
        tdefl_status st = tdefl_compress_buffer(comp, row, 1 + PNG_W * 3, y == PNG_H - 1 ? TDEFL_FINISH : TDEFL_NO_FLUSH);
        ok              = st == (y == PNG_H - 1 ? TDEFL_STATUS_DONE : TDEFL_STATUS_OKAY);
    }
    heap_caps_free(row);
    heap_caps_free(comp);
    return ok && chunk(f, "IEND", NULL, 0);
}

static void save(void) {
    // The frame first, as it is now: the notices below must not be in the picture.
    uint32_t     hz   = g_set.freq_hz;
    const char*  mode = mode_names[g_set.mode];
    if (!shot_grab()) {
        app_notice("No screenshot: the display did not deliver a frame");
        return;
    }
    app_notice("Screenshot: writing to the SD card ...");
    char why[64];
    if (!rec_sd_acquire(why, sizeof(why))) {
        app_notice("No screenshot: %s", why);
        shot_release();
        return;
    }
    if (!rec_make_dir(NULL)) {
        app_notice("No screenshot: cannot create the folder %s on the card (write protected?)", REC_DIRNAME);
        goto out;
    }
    char   low[8], name[48], path[112];
    size_t i;
    for (i = 0; mode[i] && i < sizeof(low) - 1; i++) low[i] = (char)tolower((unsigned char)mode[i]);
    low[i] = 0;
    for (;;) {
        time_t      now = time(NULL);
        struct tm   tm;
        struct stat st;
        localtime_r(&now, &tm);
        snprintf(name, sizeof(name), "%04d%02d%02d-%02d%02d%02d-%06u-%s.png", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                 tm.tm_hour, tm.tm_min, tm.tm_sec, (unsigned)((hz + 500) / 1000), low);
        snprintf(path, sizeof(path), "%s/%s", REC_DIR, name);
        if (stat(path, &st) != 0) break;
        vTaskDelay(pdMS_TO_TICKS(200));  // two within the same second
    }
    FILE* f = fopen(path, "wb");
    if (!f) {
        app_notice("No screenshot: cannot create the file (errno %d: card full or write protected?)", errno);
        goto out;
    }
    bool ok   = write_png(f, g_shot_buf);
    long size = ftell(f);
    ok        = fclose(f) == 0 && ok;
    if (ok) {
        app_notice("Screenshot saved: %s   %u KB", name, (unsigned)((size + 1023) / 1024));
    } else {
        unlink(path);
        app_notice("No screenshot: write error (card full or removed?)");
        ESP_LOGW(TAG, "writing %s failed", path);
    }
out:
    rec_sd_release();
    shot_release();
}

static void shot_task(void* arg) {
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        save();
        ulTaskNotifyTake(pdTRUE, 0);  // a key press during the save is not another screenshot
        busy = false;
    }
}

void shot_save(void) {
    if (!task || !g_shot_buf) {
        app_notice("No screenshot: out of memory");
        return;
    }
    if (busy) return;
    busy = true;
    xTaskNotifyGive(task);
}

void shot_stop_wait(void) {
    for (int i = 0; i < 800 && busy; i++) vTaskDelay(pdMS_TO_TICKS(10));
}

void shot_init(void) {
    buf_lock    = xSemaphoreCreateMutex();
    g_shot_size = (size_t)RAW_W * RAW_H * 3;
    g_shot_buf  = heap_caps_malloc(g_shot_size, MALLOC_CAP_SPIRAM);  // 1.15 MB
    if (!buf_lock || !g_shot_buf) {
        ESP_LOGE(TAG, "no memory for screenshots");
        return;
    }
    // At the priority of the display loop, which it shares core 0 with: the picture is compressed
    // in the time the loop leaves.
    if (xTaskCreatePinnedToCoreWithCaps(shot_task, "shot", TASK_STACK, NULL, 1, &task, 0, MALLOC_CAP_SPIRAM) != pdPASS) {
        task = NULL;
        ESP_LOGE(TAG, "screenshot task not created");
    }
}
