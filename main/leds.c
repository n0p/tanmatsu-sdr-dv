// SPDX-License-Identifier: AGPL-3.0-only
// The two LEDs that belong to the app, A and B. The other four (power, radio, messages, power
// button) stay with the keyboard controller, which shows the battery and charging state on them:
// that is its "automatic" mode, in which only the colours of A and B are taken from the app.
//
//   A  signal in the tuned channel over the noise floor: off below 3 dB, then red (weak) through
//      yellow to green (30 dB and more)
//   B  one colour channel per state, so that they mix:
//        green  analogue audio is playing: the squelch is open or switched off
//        blue   digital audio is playing: DMR / TETRA speech is being decoded to the speaker
//        red    a recording is running            (yellow: recording analogue audio, magenta: digital)
//      "Playing" is what the receiver puts out, whatever the volume: muted, B still shows it.
//
// Brightness: the controller scales all six LEDs by the brightness set in the launcher (Settings,
// NVS "system" / "led.brightness"), which is left alone because the power LED hangs on it too.
// A and B get what is left to bring them to that setting or to 10 %, whichever is less.
//
// Called from the display loop; the LEDs are looked at four times a second and written (one I2C
// message of 19 bytes) only when a colour has changed.

#include <math.h>
#include <string.h>
#include "app.h"
#include "bsp/led.h"
#include "decoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "ui.h"

static const char TAG[] = "leds";

#define LED_A          4
#define LED_B          5
#define LIMIT_PCT      10
#define UPDATE_US      250000
#define SNR_OFF_DB     3.0f   // A lights 1 dB above this and goes out 1 dB below
#define SNR_FULL_DB    30.0f

led_status_t g_led;

static int     full;  // colour value that gives the brightness limit, 0..255
static bool    a_lit;
static int     a_step;  // 0 red .. 8 green
static int64_t last_update;

// What the controller multiplies the colours with, in percent. The register is what counts; the
// launcher's setting is the fallback, and full brightness the assumption if neither can be read
// (which gives the dimmest result).
static int brightness_setting(void) {
    uint8_t pct = 100;
    if (bsp_led_get_brightness(&pct) == ESP_OK) return pct;
    nvs_handle_t h;
    if (nvs_open("system", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_u8(h, "led.brightness", &pct) != ESP_OK || pct > 100) pct = 100;
        nvs_close(h);
    }
    return pct;
}

static void write_leds(uint32_t a, uint32_t b) {
    if (a == g_led.a && b == g_led.b && g_led.sends) return;
    g_led.a = a;
    g_led.b = b;
    // The first four entries are not used by the controller in automatic mode; they stay zero.
    bsp_led_set_pixel(LED_A, a);
    bsp_led_set_pixel(LED_B, b);
    esp_err_t err = bsp_led_send();
    g_led.sends++;
    if (err != ESP_OK && g_led.err == ESP_OK) ESP_LOGW(TAG, "LED write failed: %s", esp_err_to_name(err));
    g_led.err = err;
}

void leds_init(void) {
    int pct       = brightness_setting();
    g_led.setting = pct;
    // The percentage is rounded down from a 0..255 register, hence the + 1: never above the limit.
    full       = pct < LIMIT_PCT ? 255 : 255 * LIMIT_PCT / (pct + 1);
    g_led.full = full;
    bsp_led_set_mode(true);
    for (int i = 0; i < 6; i++) bsp_led_set_pixel(i, 0);
    write_leds(0, 0);
}

void leds_off(void) {
    write_leds(0, 0);
}

static uint32_t rgb(int r, int g, int b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

void leds_update(void) {
    int64_t now = esp_timer_get_time();
    if (now - last_update < UPDATE_US) return;
    last_update = now;

    // A: signal over noise.
    uint32_t a   = 0;
    float    snr = 0.0f;
    if (g_stat.status == RADIO_STREAMING && ui_snr_db(&snr)) {
        a_lit = snr > SNR_OFF_DB + (a_lit ? -1.0f : 1.0f);
        if (a_lit) {
            float t = (snr - SNR_OFF_DB) / (SNR_FULL_DB - SNR_OFF_DB);
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            // Red -> yellow -> green in eight steps, and a step is only left by three quarters of
            // one: a steady signal is a steady colour, and nothing is written.
            if (fabsf(t * 8.0f - (float)a_step) > 0.75f) a_step = (int)lroundf(t * 8.0f);
            t = (float)a_step / 8.0f;
            int r = (int)lroundf((float)full * (t < 0.5f ? 1.0f : 2.0f - 2.0f * t));
            int g = (int)lroundf((float)full * (t < 0.5f ? 2.0f * t : 1.0f));
            a     = rgb(r, g, 0);
        }
    } else {
        a_lit = false;
    }
    g_led.snr_db = snr;

    // B: recording, analogue audio, digital voice.
    bool digital = mode_is_digital(g_set.mode);
    bool rec     = g_rec.state != REC_IDLE;
    bool sql     = !digital && g_stat.status == RADIO_STREAMING && (g_set.squelch_db <= -120 || g_stat.squelch_open);
    bool voice   = digital && g_stat.status == RADIO_STREAMING && dec_speaker_slot() != 0;
    write_leds(a, rgb(rec ? full : 0, sql ? full : 0, voice ? full : 0));
}
