// SPDX-License-Identifier: AGPL-3.0-only
// Audio recorder: writes what the receiver demodulates to a file on the SD card.
//
//   /sd/tanmatsu-sdr-dv/yyyymmdd-hhmmss-ffffff-mode.opus   (or .wav)
//   /sd/tanmatsu-sdr-dv/<MCC-MNC>/...                         TETRA, once the network is known
//
// with the local time and the frequency (kHz) and mode at the moment the recording was started.
// A TETRA recording goes into the folder of the network it is made on: the one the receiver is
// locked to when the recording starts, or else the one the first recorded speech comes from (the
// file is then moved there when it is closed).
// The DSP task hands its 48 kHz audio to a ring in PSRAM; a low-priority task on the other core
// encodes it (recfmt.c) and writes the file. Only audio that is actually there is recorded: the
// analogue modes while the squelch is open, DMR / TETRA while there is speech, so a recording left
// running holds the transmissions back to back and not the hours in between.
//
// The card is mounted when a recording starts and unmounted when it ends, so it can be taken out
// at any time in between; while recording the file is brought to a playable state every few
// seconds in case the power goes or the card is pulled.

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "app.h"
#include "bsp/input.h"
#include "bsp/rtc.h"
#include "decoder.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "recfmt.h"
#include "sd_pwr_ctrl.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "sdmmc_cmd.h"

static const char TAG[] = "rec";

#define RING_SAMPLES  (1u << 18)  // 5.4 s at 48 kHz: rides out a slow card
#define TASK_STACK    (64 * 1024)  // libopus keeps its work arrays on the stack
#define CHECKPOINT_US 5000000
#define IO_BUF_BYTES  8192
#define SD_LDO_CHAN   4  // Tanmatsu: the card is powered from the P4's LDO 4, on SDMMC slot 0

rec_status_t g_rec;

static int16_t*          ring;
static volatile uint32_t ring_w, ring_r;  // free-running sample counts
static TaskHandle_t      task;
static volatile bool     want_on;

// ---- SD card ---------------------------------------------------------------------------------------

static SemaphoreHandle_t    sd_lock;
static int                  sd_users;
static sdmmc_card_t*        sd_card;
static sd_pwr_ctrl_handle_t sd_pwr;
static uint8_t*             sd_dma_buf;

// Off and on again, with the bus lines held low while it is off so the card is not fed through
// them: a card left in an unknown state by whatever used it before the restart does not answer.
static void sd_power_cycle(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = BIT64(GPIO_NUM_39) | BIT64(GPIO_NUM_40) | BIT64(GPIO_NUM_41) | BIT64(GPIO_NUM_42) |
                        BIT64(GPIO_NUM_43) | BIT64(GPIO_NUM_44),
        .mode         = GPIO_MODE_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    for (int pin = GPIO_NUM_39; pin <= GPIO_NUM_44; pin++) gpio_set_level(pin, 0);
    sd_pwr_ctrl_set_io_voltage(sd_pwr, 0);
    vTaskDelay(pdMS_TO_TICKS(150));
    cfg.mode = GPIO_MODE_INPUT;
    gpio_config(&cfg);
    sd_pwr_ctrl_set_io_voltage(sd_pwr, 3300);
    vTaskDelay(pdMS_TO_TICKS(150));
}

static bool sd_mount(char* why, size_t why_len) {
    bool inserted = false;
    if (bsp_input_read_action(BSP_INPUT_ACTION_TYPE_SD_CARD, &inserted) == ESP_OK && !inserted) {
        snprintf(why, why_len, "no SD card");
        return false;
    }
    if (!sd_pwr) {
        sd_pwr_ctrl_ldo_config_t ldo = {.ldo_chan_id = SD_LDO_CHAN};
        esp_err_t                err = sd_pwr_ctrl_new_on_chip_ldo(&ldo, &sd_pwr);
        if (err != ESP_OK) {
            sd_pwr = NULL;
            snprintf(why, why_len, "SD card power: %s", esp_err_to_name(err));
            return false;
        }
    }
    if (!sd_dma_buf) sd_dma_buf = heap_caps_malloc(512 * 4, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!sd_dma_buf) {
        snprintf(why, why_len, "SD card: out of memory");
        return false;
    }
    sd_power_cycle();

    sdmmc_host_t host       = SDMMC_HOST_DEFAULT();
    host.slot               = SDMMC_HOST_SLOT_0;
    host.max_freq_khz       = SDMMC_FREQ_HIGHSPEED;
    host.pwr_ctrl_handle    = sd_pwr;
    host.dma_aligned_buffer = sd_dma_buf;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk                 = GPIO_NUM_43;
    slot.cmd                 = GPIO_NUM_44;
    slot.d0                  = GPIO_NUM_39;
    slot.d1                  = GPIO_NUM_40;
    slot.d2                  = GPIO_NUM_41;
    slot.d3                  = GPIO_NUM_42;
    slot.width               = 4;
    const esp_vfs_fat_sdmmc_mount_config_t cfg = {
        .format_if_mount_failed = false,  // never: it is the user's card
        .max_files              = 5,  // a recording, the message log, a screenshot, the console
        .allocation_unit_size   = 16 * 1024,
    };
    esp_err_t err = esp_vfs_fat_sdmmc_mount(REC_MOUNT, &host, &slot, &cfg, &sd_card);
    if (err != ESP_OK) {
        sd_card = NULL;
        // ESP_FAIL: the card answers but has no file system this build can read (FAT16 / FAT32).
        if (err == ESP_FAIL) snprintf(why, why_len, "SD card: no FAT file system (exFAT is not supported)");
        else snprintf(why, why_len, "SD card does not answer (%s)", esp_err_to_name(err));
        ESP_LOGW(TAG, "mount failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool rec_sd_acquire(char* why, size_t why_len) {
    if (!sd_lock) {
        snprintf(why, why_len, "recorder not started");
        return false;
    }
    xSemaphoreTake(sd_lock, portMAX_DELAY);
    bool ok = sd_users > 0 || sd_mount(why, why_len);
    if (ok) sd_users++;
    xSemaphoreGive(sd_lock);
    return ok;
}

void rec_sd_release(void) {
    xSemaphoreTake(sd_lock, portMAX_DELAY);
    if (sd_users > 0 && --sd_users == 0 && sd_card) {
        esp_vfs_fat_sdcard_unmount(REC_MOUNT, sd_card);
        sd_card = NULL;
    }
    xSemaphoreGive(sd_lock);
}

// ---- Folders ---------------------------------------------------------------------------------------

void rec_plmn_dir(char* out, size_t len, unsigned mcc, unsigned mnc) {
    snprintf(out, len, "%03u-%04u", mcc, mnc);
}

bool rec_make_dir(const char* sub) {
    struct stat st;
    bool        ok = true;
    xSemaphoreTake(sd_lock, portMAX_DELAY);  // the recorder, the message log and screenshots all come here
    if (stat(REC_DIR, &st) != 0) {
        // The folder the first versions of the app wrote to is taken over, with what is in it.
        if (stat(REC_DIR_OLD, &st) == 0 && rename(REC_DIR_OLD, REC_DIR) == 0) {
            ESP_LOGI(TAG, "renamed %s to %s", REC_DIR_OLD, REC_DIR);
        } else {
            ok = mkdir(REC_DIR, 0775) == 0;
        }
    }
    if (ok && sub && *sub) {
        char dir[64];
        snprintf(dir, sizeof(dir), "%s/%s", REC_DIR, sub);
        ok = stat(dir, &st) == 0 || mkdir(dir, 0775) == 0;
    }
    xSemaphoreGive(sd_lock);
    return ok;
}

// ---- Recording -------------------------------------------------------------------------------------

static FILE*         file;
static char*         io_buf;
static rec_writer_t* writer;
static char          path[112];
static bool          failed;
static int64_t       checkpoint_at;
static char          rec_sub[16];  // network folder the file was created in, "" = none
static volatile bool rec_tetra;    // started in TETRA mode: the file belongs in a network folder
static volatile bool plmn_noted;   // ... of this network, where the first recorded speech came from
static volatile uint16_t noted_mcc, noted_mnc;

void rec_note_plmn(unsigned mcc, unsigned mnc) {
    if (g_rec.state != REC_ON || !rec_tetra || plmn_noted) return;
    noted_mcc  = (uint16_t)mcc;
    noted_mnc  = (uint16_t)mnc;
    plmn_noted = true;
}

// From the DSP task. Whatever does not fit is dropped and counted: the receiver never waits for the card.
void rec_feed(const int16_t* pcm, int n) {
    if (g_rec.state != REC_ON || n <= 0) return;
    uint32_t w    = ring_w;
    uint32_t room = RING_SAMPLES - (w - ring_r);
    if ((uint32_t)n > room) {
        g_rec.dropped += (uint32_t)n - room;
        n              = (int)room;
    }
    uint32_t at    = w & (RING_SAMPLES - 1);
    uint32_t first = RING_SAMPLES - at < (uint32_t)n ? RING_SAMPLES - at : (uint32_t)n;
    memcpy(ring + at, pcm, first * sizeof(int16_t));
    memcpy(ring, pcm + first, ((uint32_t)n - first) * sizeof(int16_t));
    ring_w = w + (uint32_t)n;
}

static void drain(void) {
    static int16_t chunk[960];
    while (!failed) {
        uint32_t avail = ring_w - ring_r;
        if (avail == 0) break;
        uint32_t n  = avail < 960 ? avail : 960;
        uint32_t at = ring_r & (RING_SAMPLES - 1);
        for (uint32_t i = 0; i < n; i++) chunk[i] = ring[(at + i) & (RING_SAMPLES - 1)];
        ring_r += n;
        if (!recw_write(writer, chunk, (int)n)) failed = true;
        uint32_t us = recw_enc_us_max(writer);
        if (us > g_rec.enc_us_max) g_rec.enc_us_max = us;
    }
    g_rec.ms         = recw_ms(writer);
    g_rec.enc_us_avg = recw_enc_us_avg(writer);
    g_rec.stack_free = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    if (!failed && esp_timer_get_time() - checkpoint_at > CHECKPOINT_US) {
        if (!recw_checkpoint(writer) || fsync(fileno(file)) != 0) failed = true;
        long pos = ftell(file);
        if (pos > 0) g_rec.bytes = (uint32_t)pos;
        checkpoint_at = esp_timer_get_time();
    }
}

static void start(void) {
    char why[64];
    g_rec.state = REC_STARTING;
    if (!rec_sd_acquire(why, sizeof(why))) {
        app_notice("Not recording: %s", why);
        goto fail;
    }
    struct stat st;
    unsigned    mcc, mnc;
    rec_tetra  = g_set.mode == MODE_TETRA;
    plmn_noted = false;
    rec_sub[0] = 0;
    if (rec_tetra && dec_tetra_plmn(&mcc, &mnc)) rec_plmn_dir(rec_sub, sizeof(rec_sub), mcc, mnc);
    if (!rec_make_dir(rec_sub)) {
        app_notice("Not recording: cannot create the folder %s on the card (write protected?)", REC_DIRNAME);
        goto fail_mounted;
    }

    rec_format_t fmt  = (rec_format_t)g_set.rec_format;
    uint32_t     hz   = g_set.freq_hz;
    const char*  mode = mode_names[g_set.mode];
    char         low[8];
    size_t       i;
    for (i = 0; mode[i] && i < sizeof(low) - 1; i++) low[i] = (char)tolower((unsigned char)mode[i]);
    low[i] = 0;
    struct tm tm;
    for (;;) {
        time_t now = time(NULL);
        localtime_r(&now, &tm);
        snprintf(g_rec.name, sizeof(g_rec.name), "%s%s%04d%02d%02d-%02d%02d%02d-%06u-%s.%s", rec_sub, rec_sub[0] ? "/" : "",
                 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec,
                 (unsigned)((hz + 500) / 1000), low, rec_format_ext(fmt));
        snprintf(path, sizeof(path), "%s/%s", REC_DIR, g_rec.name);
        if (stat(path, &st) != 0) break;
        vTaskDelay(pdMS_TO_TICKS(200));  // stopped and started again within the same second
    }
    file = fopen(path, "wb");
    if (!file) {
        app_notice("Not recording: cannot create the file (errno %d: card full or write protected?)", errno);
        goto fail_mounted;
    }
    if (io_buf) setvbuf(file, io_buf, _IOFBF, IO_BUF_BYTES);

    char title[64], date[40];
    snprintf(title, sizeof(title), "TITLE=%u.%06u MHz %s", (unsigned)(hz / 1000000), (unsigned)(hz % 1000000), mode);
    strftime(date, sizeof(date), "DATE=%Y-%m-%dT%H:%M:%S", &tm);
    const char* tags[] = {title, date, "ENCODER=Tanmatsu SDR " APP_VERSION};
    writer             = recw_open(file, fmt, tags, 3, esp_random());
    if (!writer) {
        app_notice("Not recording: cannot start the %s file", rec_format_name(fmt));
        fclose(file);
        file = NULL;
        unlink(path);
        goto fail_mounted;
    }
    failed           = false;
    g_rec.ms         = 0;
    g_rec.bytes      = 0;
    g_rec.dropped    = 0;
    g_rec.enc_us_max = 0;
    g_rec.enc_us_avg = 0;
    g_rec.started_us = esp_timer_get_time();
    checkpoint_at    = g_rec.started_us;
    ring_r           = ring_w;
    g_rec.state      = REC_ON;
    app_notice("Recording to SD: %s   (F6 stops)", g_rec.name);
    return;

fail_mounted:
    rec_sd_release();
fail:
    want_on     = false;
    g_rec.state = REC_IDLE;
}

static void finish(void) {
    g_rec.state = REC_STOPPING;  // rec_feed() stops adding; take what is in the ring
    drain();
    uint32_t ms = recw_ms(writer);
    bool     ok = recw_close(writer) && !failed;
    writer      = NULL;
    long size   = ftell(file);
    ok          = fclose(file) == 0 && ok;
    file        = NULL;
    g_rec.ms    = ms;
    if (size > 0) g_rec.bytes = (uint32_t)size;
    if (!ok) {
        app_notice("Recording %s ended with a write error (card full or removed?)", g_rec.name);
    } else if (ms == 0) {
        unlink(path);
        if (rec_sub[0]) {
            char dir[64];
            snprintf(dir, sizeof(dir), "%s/%s", REC_DIR, rec_sub);
            rmdir(dir);  // goes only if this file was all there was in it
        }
        app_notice("Recording stopped: there was no audio, no file kept");
    } else {
        if (!rec_sub[0] && rec_tetra && plmn_noted) {
            // Started before the network was known: the file moves to its folder now.
            char sub[16], name[sizeof(g_rec.name)], to[sizeof(path)];
            rec_plmn_dir(sub, sizeof(sub), noted_mcc, noted_mnc);
            snprintf(name, sizeof(name), "%s/%s", sub, g_rec.name);
            snprintf(to, sizeof(to), "%s/%s", REC_DIR, name);
            if (rec_make_dir(sub) && rename(path, to) == 0) snprintf(g_rec.name, sizeof(g_rec.name), "%s", name);
        }
        app_notice("Saved %s   %u:%02u, %u KB", g_rec.name, (unsigned)(ms / 60000), (unsigned)(ms / 1000 % 60),
                   (unsigned)((size + 1023) / 1024));
    }
    rec_sd_release();
    want_on     = false;
    g_rec.state = REC_IDLE;
}

static void rec_task(void* arg) {
    (void)arg;
    for (;;) {
        if (g_rec.state == REC_IDLE) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if (want_on) start();
            continue;
        }
        drain();
        if (!want_on || failed) finish();
        else vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void rec_toggle(void) {
    if (!task) {
        app_notice("Recorder not available: out of memory");
        return;
    }
    want_on = !want_on;
    if (want_on) app_notice("Recording: opening the SD card ...");
    xTaskNotifyGive(task);
}

void rec_stop_wait(void) {
    if (!task) return;
    want_on = false;
    for (int i = 0; i < 300 && g_rec.state != REC_IDLE; i++) vTaskDelay(pdMS_TO_TICKS(10));
}

void rec_init(void) {
    // Local time for the file names: the clock comes from the RTC, the time zone is the one set
    // in the launcher.
    bsp_rtc_update_time();
    nvs_handle_t h;
    if (nvs_open("system", NVS_READONLY, &h) == ESP_OK) {
        char   tz[64];
        size_t len = sizeof(tz);
        if (nvs_get_str(h, "tz", tz, &len) == ESP_OK && tz[0]) {
            setenv("TZ", tz, 1);
            tzset();
        }
        nvs_close(h);
    }

    sd_lock = xSemaphoreCreateMutex();
    ring    = heap_caps_malloc(RING_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    io_buf  = heap_caps_malloc(IO_BUF_BYTES, MALLOC_CAP_SPIRAM);
    if (!sd_lock || !ring) {
        ESP_LOGE(TAG, "no memory for the recorder");
        return;
    }
    // Core 0, above the display loop and below everything that makes sound. Core 1 is not an option:
    // next to the DSP task and the USB transfers an Opus frame took 30 ms there, slower than real time.
    if (xTaskCreatePinnedToCoreWithCaps(rec_task, "rec", TASK_STACK, NULL, 2, &task, 0, MALLOC_CAP_SPIRAM) != pdPASS) {
        task = NULL;
        ESP_LOGE(TAG, "recorder task not created");
    }
}
