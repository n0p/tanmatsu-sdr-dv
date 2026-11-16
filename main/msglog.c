// SPDX-License-Identifier: AGPL-3.0-only
// Received messages: TETRA short data (SDS) and status messages, DMR text and status messages.
//
// The decoder hands every one to msglog_put(). The last MSG_KEEP are kept in memory for the list
// on the screen (F5). With the "SDS log" setting on each one is also appended to a text file on
// the SD card, one line per message, the fields separated by tabs:
//
//   /sd/tanmatsu-sdr-dv/<MCC-MNC>/yyyymmdd-sds.txt     the network of the cell it was received from
//   /sd/tanmatsu-sdr-dv/yyyymmdd-sds.txt               if that is not known
//
//   date  time  MHz  system  network  from  to  kind  content
//
// A message that the network repeats within a few seconds is taken once.
//
// The file is written by a low-priority task, never by the decoder. It is opened, appended to and
// closed for every batch of messages, and the card is let go a few seconds after the last one, so
// it can be taken out in between like with the recorder.

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include "app.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char TAG[] = "msglog";

#define TASK_STACK   (12 * 1024)
#define DUP_WINDOW_S 10
#define DUP_LOOKBACK 8
#define CARD_IDLE_MS 3000
#define RETRY_US     30000000  // after the card could not be used: leave it alone for this long

msglog_status_t g_msglog;

static msg_t*            ring;     // message number n is in ring[n % MSG_KEEP]
static uint32_t          put_seq;  // messages taken so far
static uint32_t          done_seq;  // ... of which the writer has dealt with
static SemaphoreHandle_t lock;
static TaskHandle_t      task;
static volatile bool     stopping;
static volatile bool     busy;

static void copy(char* dst, size_t len, const char* src) {
    snprintf(dst, len, "%s", src ? src : "");
}

void msglog_put(const char* sys, bool plmn, unsigned mcc, unsigned mnc, const char* from, const char* to,
                const char* kind, const char* text) {
    if (!ring) return;
    time_t now = time(NULL);
    xSemaphoreTake(lock, portMAX_DELAY);
    for (uint32_t back = 1; back <= DUP_LOOKBACK && back <= put_seq && back <= MSG_KEEP; back++) {
        const msg_t* m = &ring[(put_seq - back) % MSG_KEEP];
        if (now - m->time > DUP_WINDOW_S) break;
        if (strncmp(m->text, text, sizeof(m->text) - 1) == 0 && strncmp(m->from, from, sizeof(m->from) - 1) == 0 &&
            strncmp(m->to, to, sizeof(m->to) - 1) == 0 && strncmp(m->kind, kind, sizeof(m->kind) - 1) == 0 &&
            strncmp(m->sys, sys, sizeof(m->sys) - 1) == 0) {
            g_msglog.dup++;
            xSemaphoreGive(lock);
            return;
        }
    }
    msg_t* m   = &ring[put_seq % MSG_KEEP];
    m->time    = now;
    m->freq_hz = g_set.freq_hz;
    m->plmn    = plmn;
    m->mcc     = (uint16_t)mcc;
    m->mnc     = (uint16_t)mnc;
    m->to_card = g_set.sds_log;
    copy(m->sys, sizeof(m->sys), sys);
    copy(m->from, sizeof(m->from), from);
    copy(m->to, sizeof(m->to), to);
    copy(m->kind, sizeof(m->kind), kind);
    copy(m->text, sizeof(m->text), text);
    put_seq++;
    g_msglog.seen = put_seq;
    xSemaphoreGive(lock);
    if (task) xTaskNotifyGive(task);
}

int msglog_count(void) {
    uint32_t n = put_seq;
    return n < MSG_KEEP ? (int)n : MSG_KEEP;
}

bool msglog_get(int back, msg_t* out) {
    if (!ring || back < 0) return false;
    bool ok = false;
    xSemaphoreTake(lock, portMAX_DELAY);
    if ((uint32_t)back < put_seq && back < MSG_KEEP) {
        *out = ring[(put_seq - 1 - (uint32_t)back) % MSG_KEEP];
        ok   = true;
    }
    xSemaphoreGive(lock);
    return ok;
}

// ---- the file --------------------------------------------------------------------------------------

// A field of the line: tabs, line ends and backslashes written as \t \n \r \\, any other control
// character as \xNN, so that a line is always one message and a tab always a field boundary.
static void put_field(FILE* f, const char* s) {
    for (; *s; s++) {
        unsigned char c = (unsigned char)*s;
        if (c == '\t') fputs("\\t", f);
        else if (c == '\n') fputs("\\n", f);
        else if (c == '\r') fputs("\\r", f);
        else if (c == '\\') fputs("\\\\", f);
        else if (c < 0x20 || c == 0x7f) fprintf(f, "\\x%02X", c);
        else fputc(c, f);
    }
}

// With the card acquired.
static bool write_msg(const msg_t* m, char* why, size_t why_len) {
    char sub[16] = "", net[16] = "-";
    if (m->plmn) {
        rec_plmn_dir(sub, sizeof(sub), m->mcc, m->mnc);
        snprintf(net, sizeof(net), "%s", sub);
    }
    if (!rec_make_dir(sub)) {
        snprintf(why, why_len, "cannot create the folder %s on the card (write protected?)", REC_DIRNAME);
        return false;
    }
    time_t    t = (time_t)m->time;
    struct tm tm;
    localtime_r(&t, &tm);
    char name[64], path[112];
    snprintf(name, sizeof(name), "%s%s%04d%02d%02d-sds.txt", sub, sub[0] ? "/" : "", tm.tm_year + 1900, tm.tm_mon + 1,
             tm.tm_mday);
    snprintf(path, sizeof(path), "%s/%s", REC_DIR, name);
    struct stat st;
    bool        is_new = stat(path, &st) != 0 || st.st_size == 0;
    FILE*       f      = fopen(path, "a");
    if (!f) {
        snprintf(why, why_len, "cannot open %s (errno %d: card full or write protected?)", name, errno);
        return false;
    }
    if (is_new) fputs("# date\ttime\tMHz\tsystem\tnetwork\tfrom\tto\tkind\tcontent\n", f);
    fprintf(f, "%04d-%02d-%02d\t%02d:%02d:%02d\t%u.%06u\t", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
            tm.tm_min, tm.tm_sec, (unsigned)(m->freq_hz / 1000000), (unsigned)(m->freq_hz % 1000000));
    put_field(f, m->sys);
    fprintf(f, "\t%s\t", net);
    put_field(f, m->from);
    fputc('\t', f);
    put_field(f, m->to);
    fputc('\t', f);
    put_field(f, m->kind);
    fputc('\t', f);
    put_field(f, m->text);
    fputc('\n', f);
    bool ok = !ferror(f);
    ok      = fclose(f) == 0 && ok;
    if (!ok) {
        snprintf(why, why_len, "write error on %s (card full or removed?)", name);
        return false;
    }
    snprintf(g_msglog.file, sizeof(g_msglog.file), "%s", name);
    return true;
}

static void log_task(void* arg) {
    (void)arg;
    static msg_t m;
    bool         have_card = false;
    bool         told      = false;  // the user knows that the log is not being written
    int64_t      retry_at  = 0;
    for (;;) {
        bool pending;
        xSemaphoreTake(lock, portMAX_DELAY);
        if (put_seq - done_seq > MSG_KEEP) {
            // Messages came faster than the card took them, and the oldest are gone from the ring.
            g_msglog.errors += put_seq - done_seq - MSG_KEEP;
            done_seq         = put_seq - MSG_KEEP;
        }
        pending = done_seq != put_seq;
        if (pending) m = ring[done_seq % MSG_KEEP];
        xSemaphoreGive(lock);

        if (!pending) {
            busy = false;
            // Nothing to do: wait for the next message, and let go of the card if none comes soon.
            if (ulTaskNotifyTake(pdTRUE, have_card && !stopping ? pdMS_TO_TICKS(CARD_IDLE_MS) : portMAX_DELAY) == 0 ||
                stopping) {
                if (have_card) {
                    rec_sd_release();
                    have_card      = false;
                    g_msglog.card  = false;
                }
            }
            continue;
        }
        busy = true;
        if (m.to_card) {
            char why[96] = "";
            bool ok      = false;
            if (!have_card && esp_timer_get_time() >= retry_at) {
                have_card     = rec_sd_acquire(why, sizeof(why));
                g_msglog.card = have_card;
            } else if (!have_card) {
                snprintf(why, sizeof(why), "card not usable a moment ago");
            }
            if (have_card) ok = write_msg(&m, why, sizeof(why));
            if (ok) {
                g_msglog.written++;
                told = false;
            } else {
                g_msglog.errors++;
                ESP_LOGW(TAG, "message not logged: %s", why);
                if (!told) app_notice("SDS log: %s", why);
                told = true;
                if (have_card) {
                    rec_sd_release();
                    have_card     = false;
                    g_msglog.card = false;
                }
                if (esp_timer_get_time() >= retry_at) retry_at = esp_timer_get_time() + RETRY_US;
            }
        }
        xSemaphoreTake(lock, portMAX_DELAY);
        done_seq++;
        xSemaphoreGive(lock);
    }
}

void msglog_stop_wait(void) {
    if (!task) return;
    stopping = true;
    xTaskNotifyGive(task);
    for (int i = 0; i < 300 && (busy || g_msglog.card || done_seq != put_seq); i++) vTaskDelay(pdMS_TO_TICKS(10));
}

void msglog_init(void) {
    lock = xSemaphoreCreateMutex();
    ring = heap_caps_calloc(MSG_KEEP, sizeof(msg_t), MALLOC_CAP_SPIRAM);
    if (!lock || !ring) {
        ring = NULL;
        ESP_LOGE(TAG, "no memory for the message list");
        return;
    }
    if (xTaskCreatePinnedToCoreWithCaps(log_task, "msglog", TASK_STACK, NULL, 2, &task, 0, MALLOC_CAP_SPIRAM) != pdPASS) {
        task = NULL;
        ESP_LOGE(TAG, "message log task not created: the list works, the file does not");
    }
}
