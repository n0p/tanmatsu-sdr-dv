// SPDX-License-Identifier: AGPL-3.0-only
// Sample capture taps: the DSP task copies one chosen stream into a PSRAM buffer, the console task
// dumps it to the host. This is how recordings for offline decoder work are taken off the device.

#include <string.h>
#include "app.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

capture_t         g_cap;
const char* const tap_names[TAP_COUNT] = {"raw", "if", "ch", "fm", "audio"};

const char* cap_format(tap_t tap) {
    switch (tap) {
        case TAP_RAW:
            return "cu8";
        case TAP_IF:
        case TAP_CH:
            return "cs16";
        default:
            return "s16";
    }
}

static uint32_t tap_rate(tap_t tap, demod_mode_t mode) {
    switch (tap) {
        case TAP_RAW:
            return RTL_RATE;
        case TAP_IF:
            return mode == MODE_WFM ? IF_RATE_WIDE : IF_RATE_NARROW;
        case TAP_CH:
            return mode == MODE_TETRA ? CH_RATE_TETRA : CH_RATE_48K;
        case TAP_FM:
            return mode == MODE_WFM ? IF_RATE_WIDE : CH_RATE_48K;
        default:
            return AUDIO_RATE;
    }
}

static size_t tap_frame_bytes(tap_t tap) {
    switch (tap) {
        case TAP_RAW:
            return 2;
        case TAP_IF:
        case TAP_CH:
            return 4;
        default:
            return 2;
    }
}

void cap_release(void) {
    g_cap.state = CAP_IDLE;
    // Let a feed call that is in flight on the DSP task finish before the buffer goes away.
    vTaskDelay(pdMS_TO_TICKS(40));
    if (g_cap.buf) heap_caps_free(g_cap.buf);
    g_cap.buf  = NULL;
    g_cap.size = 0;
    g_cap.fill = 0;
}

bool cap_arm(tap_t tap, uint32_t ms, float trig_db) {
    cap_release();
    uint32_t rate  = tap_rate(tap, g_set.mode);
    size_t   bytes = (size_t)((uint64_t)rate * ms / 1000u) * tap_frame_bytes(tap);
    if (bytes == 0 || bytes > 24u * 1024u * 1024u) return false;
    g_cap.buf = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (!g_cap.buf) return false;
    g_cap.size    = bytes;
    g_cap.fill    = 0;
    g_cap.tap     = tap;
    g_cap.rate    = rate;
    g_cap.trig_db = trig_db;
    g_cap.freq_hz = g_set.freq_hz;
    g_cap.mode    = g_set.mode;
    g_cap.state   = CAP_ARMED;
    return true;
}

static inline bool cap_wants(tap_t tap) {
    return g_cap.state == CAP_RECORDING && g_cap.tap == tap;
}

static void cap_advance(size_t n, uint32_t rate) {
    g_cap.rate  = rate;
    g_cap.fill += n;
    if (g_cap.fill >= g_cap.size) g_cap.state = CAP_DONE;
}

void cap_feed_u8(tap_t tap, const uint8_t* data, size_t bytes, uint32_t rate) {
    if (!cap_wants(tap)) return;
    size_t room = g_cap.size - g_cap.fill;
    if (bytes > room) bytes = room;
    memcpy(g_cap.buf + g_cap.fill, data, bytes);
    cap_advance(bytes, rate);
}

static inline int16_t sat16(float v) {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

void cap_feed_cf32(tap_t tap, const cf32_t* data, int n, uint32_t rate) {
    if (!cap_wants(tap)) return;
    size_t room = (g_cap.size - g_cap.fill) / 4;
    if ((size_t)n > room) n = (int)room;
    int16_t* dst = (int16_t*)(void*)(g_cap.buf + g_cap.fill);
    for (int i = 0; i < n; i++) {
        dst[2 * i]     = sat16(data[i].re * 32767.0f);
        dst[2 * i + 1] = sat16(data[i].im * 32767.0f);
    }
    cap_advance((size_t)n * 4, rate);
}

void cap_feed_f32(tap_t tap, const float* data, int n, float scale, uint32_t rate) {
    if (!cap_wants(tap)) return;
    size_t room = (g_cap.size - g_cap.fill) / 2;
    if ((size_t)n > room) n = (int)room;
    int16_t* dst = (int16_t*)(void*)(g_cap.buf + g_cap.fill);
    for (int i = 0; i < n; i++) dst[i] = sat16(data[i] * scale);
    cap_advance((size_t)n * 2, rate);
}

void cap_feed_s16(tap_t tap, const int16_t* data, int n, uint32_t rate) {
    if (!cap_wants(tap)) return;
    size_t room = (g_cap.size - g_cap.fill) / 2;
    if ((size_t)n > room) n = (int)room;
    memcpy(g_cap.buf + g_cap.fill, data, (size_t)n * 2);
    cap_advance((size_t)n * 2, rate);
}
