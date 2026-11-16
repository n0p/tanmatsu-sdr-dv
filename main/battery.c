// SPDX-License-Identifier: AGPL-3.0-only
// The battery level shown in the top bar.
//
// The BSP derives the percentage from the battery voltage (four short I2C reads from the keyboard
// controller's PMIC registers), so it moves with the load: it dips while the dongle and the speaker
// draw current and reads high while the charger is on. The reading is therefore taken every five
// seconds and smoothed over about twenty.
//
// Called from the display loop, like the LEDs.

#include "app.h"
#include "bsp/power.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char TAG[] = "battery";

#define POLL_US 5000000
#define SMOOTH  0.25f  // share of a new reading that goes into the level shown

battery_status_t g_bat = {.pct = -1};

void battery_update(void) {
    static int64_t last_poll;
    static bool    polled;
    static bool    have;   // level holds a reading
    static float   level;  // smoothed percentage

    int64_t now = esp_timer_get_time();
    if (polled && now - last_poll < POLL_US) return;
    polled    = true;
    last_poll = now;

    bsp_power_battery_information_t info = {0};
    esp_err_t                       err  = bsp_power_get_battery_information(&info);
    if (err != ESP_OK && g_bat.err == ESP_OK) ESP_LOGW(TAG, "battery read failed: %s", esp_err_to_name(err));
    g_bat.err = err;
    if (err != ESP_OK || !info.battery_available) {
        have          = false;
        g_bat.pct     = -1;
        g_bat.present = false;
        return;
    }

    float pct = (float)info.remaining_percentage;
    level     = have ? level + SMOOTH * (pct - level) : pct;
    have      = true;
    int shown      = (int)(level + 0.5f);
    g_bat.pct      = shown < 0 ? 0 : (shown > 100 ? 100 : shown);
    g_bat.mv       = info.voltage;
    g_bat.charging = info.battery_charging;
    g_bat.usb      = info.power_supply_available;
    g_bat.present  = true;
}
