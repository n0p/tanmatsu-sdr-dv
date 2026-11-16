// SPDX-License-Identifier: AGPL-3.0-only
// Audio output through the ES8156 codec. The app is silent until the user raises the volume: the
// codec volume is at its minimum and the speaker amplifier is switched off at start, and only zeros
// are sent to the codec while the volume setting is 0.

#include <string.h>
#include "app.h"
#include "bsp/audio.h"
#include "bsp/input.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char TAG[] = "audio";

#define RING_LEN   16384  // samples, power of two (341 ms at 48 kHz)
#define RING_MASK  (RING_LEN - 1)
#define CHUNK      480   // 10 ms
#define START_FILL 2400  // buffer 50 ms before starting playback

static int16_t*          ring;  // in PSRAM: internal RAM is needed for the USB transfer buffers
static volatile uint32_t ring_wr, ring_rd;
static i2s_chan_handle_t i2s;
static volatile bool     audible;
static audio_stats_t     stats;
static volatile bool     playing;
static volatile bool     starved;     // an underrun happened and nothing was pushed since
static volatile int64_t  starved_us;
static volatile uint32_t low_water = UINT32_MAX;

void audio_push(const int16_t* mono, int n) {
    if (ring == NULL) return;
    uint32_t wr   = ring_wr;
    uint32_t room = RING_LEN - (wr - ring_rd);
    // Diagnostics: how close the buffer came to running dry while playing, and whether it did in
    // the middle of a stream (new samples arriving right after an underrun) rather than at its end.
    if (playing && RING_LEN - room < low_water) low_water = RING_LEN - room;
    if (starved) {
        if (esp_timer_get_time() - starved_us < 120000) stats.gaps++;
        starved = false;
    }
    if ((uint32_t)n > room) {  // overflow: drop the tail
        n = (int)room;
        stats.overflows++;
    }
    for (int i = 0; i < n; i++) ring[(wr + (uint32_t)i) & RING_MASK] = mono[i];
    ring_wr = wr + (uint32_t)n;
}

// The volume register of the ES8156 is in 0.5 dB steps and bsp_audio_set_volume() maps 0..100 % to
// 0..180 of it linearly, i.e. to -95.5 .. -5.5 dB: everything below about 60 % is inaudible on the
// speaker. The app's 1..100 therefore covers the top of that range, 60.4 .. 100 % (-41 .. -5.5 dB).
static float codec_percent(int volume) {
    return 60.0f + 0.4f * (float)volume;
}

void audio_apply_volume(void) {
    int v = g_set.volume;
    if (v <= 0) {
        audible = false;
        bsp_audio_set_volume(0.0f);
        bsp_audio_set_amplifier(false);
        stats.amplifier = false;
        stats.codec_pct = 0;
    } else {
        if (v > 100) v = 100;
        // The speaker amplifier stays off while headphones are plugged in.
        bool headphones = false;
        if (bsp_input_read_action(BSP_INPUT_ACTION_TYPE_AUDIO_JACK, &headphones) != ESP_OK) headphones = false;
        stats.headphones = headphones;
        stats.vol_err    = bsp_audio_set_volume(codec_percent(v));
        stats.amp_err    = bsp_audio_set_amplifier(!headphones);
        stats.amplifier  = !headphones;
        stats.codec_pct  = (int)codec_percent(v);
        if (stats.vol_err != ESP_OK) ESP_LOGW(TAG, "set volume failed: %s", esp_err_to_name(stats.vol_err));
        if (stats.amp_err != ESP_OK) ESP_LOGW(TAG, "amplifier failed: %s", esp_err_to_name(stats.amp_err));
        audible = true;
    }
}

void audio_get_stats(audio_stats_t* out) {
    *out            = stats;
    out->fill       = ring_wr - ring_rd;
    out->low_water  = low_water == UINT32_MAX ? out->fill : low_water;
    low_water       = UINT32_MAX;
}

// The I2S stream runs all the time, also while muted, so the path to the codec is the same in both
// states (and its counters can be checked without making a sound). Muted means: codec volume at its
// minimum, amplifier off, and zeros instead of the samples.
static void audio_task(void* arg) {
    (void)arg;
    static int16_t stereo[CHUNK * 2];
    for (;;) {
        if (i2s == NULL || ring == NULL) {
            ring_rd = ring_wr;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        uint32_t avail = ring_wr - ring_rd;
        if (!playing && avail >= START_FILL) playing = true;
        if (playing && avail < CHUNK) {
            playing = false;  // underrun: silence until START_FILL is buffered again
            stats.underruns++;
            starved_us = esp_timer_get_time();
            starved    = true;
        }
        if (playing) {
            uint32_t rd = ring_rd;
            for (int i = 0; i < CHUNK; i++) {
                int16_t s         = audible ? ring[(rd + (uint32_t)i) & RING_MASK] : 0;
                stereo[2 * i]     = s;
                stereo[2 * i + 1] = s;
            }
            ring_rd = rd + CHUNK;
            stats.chunks_audio++;
        } else {
            memset(stereo, 0, sizeof(stereo));
        }
        size_t    written = 0;
        esp_err_t err     = i2s_channel_write(i2s, stereo, sizeof(stereo), &written, pdMS_TO_TICKS(200));
        if (err != ESP_OK || written != sizeof(stereo)) {
            stats.write_errors++;
            stats.last_write_err = err;
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            stats.chunks++;
        }
    }
}

void audio_init(void) {
    // Silence first, before anything else touches the codec.
    bsp_audio_set_volume(0.0f);
    bsp_audio_set_amplifier(false);
    audible = false;
    ring    = heap_caps_calloc(RING_LEN, sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (ring == NULL) ring = heap_caps_calloc(RING_LEN, sizeof(int16_t), MALLOC_CAP_DEFAULT);
    if (ring == NULL) ESP_LOGE(TAG, "no memory for the audio buffer");

    esp_err_t err = bsp_audio_get_i2s_handle(&i2s);
    if (err != ESP_OK || i2s == NULL) {
        ESP_LOGW(TAG, "no I2S handle: %s", esp_err_to_name(err));
        i2s = NULL;
        stats.init_err = err != ESP_OK ? err : ESP_ERR_INVALID_STATE;
    } else {
        // The BSP starts the channel at 44.1 kHz; the clock can only be changed while it is disabled.
        (void)i2s_channel_disable(i2s);
        err = bsp_audio_set_rate(AUDIO_RATE);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "set rate failed: %s", esp_err_to_name(err));
            stats.init_err = err;
        }
        err = i2s_channel_enable(i2s);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2S enable failed: %s", esp_err_to_name(err));
            stats.init_err = err;
        }
        stats.i2s_ok = true;
    }
    xTaskCreatePinnedToCore(audio_task, "audio", 4096, NULL, 12, NULL, 0);
    audio_apply_volume();
}
