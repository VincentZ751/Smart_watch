#include "watch_apps.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ir_transceiver.h"

static const char *TAG = "WATCH_APPS";

// Shared state
static ir_signal_t saved_ir = {0};
static imu_euler_t latest_euler = {0};
static imu_calib_t latest_calib = {0};
static sleep_request_cb_t on_sleep_request = NULL;

// ============================================================================
// APP 1: WATCHFACE (HOME)
// ============================================================================
static uint32_t wf_accum_ms = 0;

static void wf_draw(void)
{
    int64_t sec = esp_timer_get_time() / 1000000;
    int hrs = (sec / 3600) % 24;
    int min = (sec / 60) % 60;
    int s   = sec % 60;
    ESP_LOGI(TAG, "[WATCHFACE] Uptime %02d:%02d:%02d | Hold RIGHT: Back", hrs, min, s);
}

static void wf_enter(void)
{
    wf_accum_ms = 0;
    wf_draw();
}

static void wf_tick(uint32_t dt_ms, const imu_euler_t *euler, const imu_calib_t *calib)
{
    wf_accum_ms += dt_ms;
    if (wf_accum_ms >= 1000) {
        wf_accum_ms = 0;
        wf_draw();
    }
}

static const watch_app_t app_watchface = {
    .name          = "Watchface",
    .on_enter      = wf_enter,
    .on_exit       = NULL,
    .on_button     = NULL,
    .on_double_tap = NULL,
    .on_tick       = wf_tick,
    .on_draw       = wf_draw,
};

// ============================================================================
// APP 2: IR REMOTE
// ============================================================================
static void ir_app_draw(void)
{
    ESP_LOGI(TAG, "[IR REMOTE] Saved: %u symbols | LEFT: Record | RIGHT/Double-Tap: Blast | Hold RIGHT: Back",
             (unsigned)saved_ir.num_symbols);
}

static void ir_app_blast(void)
{
    if (saved_ir.num_symbols == 0) {
        ESP_LOGW(TAG, "[IR REMOTE] No signal saved! Press LEFT to record first.");
        return;
    }

    ESP_LOGI(TAG, "[IR REMOTE] >>> Blasting %u symbols (3x)!", (unsigned)saved_ir.num_symbols);
    for (int i = 0; i < 3; i++) {
        if (ir_transmit(&saved_ir) != ESP_OK) {
            ESP_LOGE(TAG, "[IR REMOTE] TX failed on burst %d", i + 1);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

static void ir_app_button(btn_event_t evt)
{
    if (evt.id == BTN_LEFT && evt.type == BTN_EVENT_SHORT) {
        ESP_LOGI(TAG, "[IR REMOTE] Listening (5 sec)... Point remote & press button!");
        if (ir_receive(&saved_ir, 5000) == ESP_OK) {
            ESP_LOGI(TAG, "[IR REMOTE] Captured %u symbols! (%uus / %uus)",
                     (unsigned)saved_ir.num_symbols,
                     saved_ir.symbols[0].duration0,
                     saved_ir.symbols[0].duration1);
        } else {
            ESP_LOGW(TAG, "[IR REMOTE] Timed out (no signal).");
        }
        ir_app_draw();
    } else if (evt.id == BTN_RIGHT && evt.type == BTN_EVENT_SHORT) {
        ir_app_blast();
    }
}

static const watch_app_t app_ir_remote = {
    .name          = "IR Remote",
    .on_enter      = ir_app_draw,
    .on_exit       = NULL,
    .on_button     = ir_app_button,
    .on_double_tap = ir_app_blast,
    .on_tick       = NULL,
    .on_draw       = ir_app_draw,
};

// ============================================================================
// APP 3: COMPASS / IMU
// ============================================================================
static uint32_t imu_accum_ms = 0;

static void compass_draw(void)
{
    ESP_LOGI(TAG, "[COMPASS] H:%5.1f° R:%5.1f° P:%5.1f° | Calib S:%d G:%d A:%d M:%d",
             latest_euler.heading, latest_euler.roll, latest_euler.pitch,
             latest_calib.sys, latest_calib.gyro, latest_calib.accel, latest_calib.mag);
}

static void compass_enter(void)
{
    imu_accum_ms = 0;
    compass_draw();
}

static void compass_tick(uint32_t dt_ms, const imu_euler_t *euler, const imu_calib_t *calib)
{
    latest_euler = *euler;
    latest_calib = *calib;

    imu_accum_ms += dt_ms;
    if (imu_accum_ms >= 500) {
        imu_accum_ms = 0;
        compass_draw();
    }
}

static const watch_app_t app_compass = {
    .name          = "Compass",
    .on_enter      = compass_enter,
    .on_exit       = NULL,
    .on_button     = NULL,
    .on_double_tap = NULL,
    .on_tick       = compass_tick,
    .on_draw       = compass_draw,
};

// ============================================================================
// CAROUSEL, SLEEP OVERLAY & APP MANAGER
// ============================================================================
static const watch_app_t *apps[] = {
    &app_watchface,
    &app_ir_remote,
    &app_compass,
};

#define NUM_APPS (sizeof(apps) / sizeof(apps[0]))
#define SLEEP_CONFIRM_WINDOW_US  2000000 // 2 seconds between 1st and 2nd tap

static int current_idx = 0;
static bool in_app = false;
static bool sleep_overlay_active = false;
static int sleep_tap_count = 0;
static int64_t sleep_first_tap_us = 0;

static void draw_carousel(void)
{
    int left_idx  = (current_idx - 1 + NUM_APPS) % NUM_APPS;
    int right_idx = (current_idx + 1) % NUM_APPS;

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "  < %s  |  [ %s ]  |  %s >",
             apps[left_idx]->name,
             apps[current_idx]->name,
             apps[right_idx]->name);
    ESP_LOGI(TAG, "  LEFT/RIGHT: Scroll | Hold RIGHT: Open | Hold LEFT: Sleep");
    ESP_LOGI(TAG, "=========================================");
}

static void draw_sleep_overlay(void)
{
    ESP_LOGW(TAG, "=========================================");
    ESP_LOGW(TAG, "         POWER OFF / SLEEP?");
    ESP_LOGW(TAG, "  Double-tap LEFT to confirm | RIGHT to cancel");
    ESP_LOGW(TAG, "=========================================");
}

static void redraw_current_screen(void)
{
    if (in_app && apps[current_idx]->on_draw) {
        apps[current_idx]->on_draw();
    } else {
        draw_carousel();
    }
}

void app_mgr_init(sleep_request_cb_t sleep_cb)
{
    on_sleep_request = sleep_cb;
    current_idx = 0;
    in_app = false;
    sleep_overlay_active = false;
    sleep_tap_count = 0;
    draw_carousel();
}

void app_mgr_handle_button(btn_event_t evt)
{
    // Ignore the unused 3rd button if still wired on breadboard
    if (evt.id != BTN_LEFT && evt.id != BTN_RIGHT) {
        return;
    }

    // --- 1. APPLE-STYLE SLEEP CONFIRMATION OVERLAY ---
    if (sleep_overlay_active) {
        if (evt.id == BTN_LEFT && evt.type == BTN_EVENT_SHORT) {
            int64_t now_us = esp_timer_get_time();
            if (sleep_tap_count == 1 && (now_us - sleep_first_tap_us) <= SLEEP_CONFIRM_WINDOW_US) {
                ESP_LOGW(TAG, "[SLEEP?] Confirmed!");
                sleep_overlay_active = false;
                if (on_sleep_request) {
                    on_sleep_request("User Confirmed Sleep");
                }
            } else {
                sleep_tap_count = 1;
                sleep_first_tap_us = now_us;
                ESP_LOGW(TAG, "[SLEEP?] Tap LEFT once more within 2s to shut down...");
            }
        } else {
            ESP_LOGI(TAG, "[SLEEP?] Cancelled.");
            sleep_overlay_active = false;
            sleep_tap_count = 0;
            redraw_current_screen();
        }
        return;
    }

    // --- 2. GLOBAL LONG-LEFT -> OPEN SLEEP OVERLAY ---
    if (evt.id == BTN_LEFT && evt.type == BTN_EVENT_LONG) {
        sleep_overlay_active = true;
        sleep_tap_count = 0;
        draw_sleep_overlay();
        return;
    }

    // --- 3. CAROUSEL MODE ---
    if (!in_app) {
        if (evt.id == BTN_LEFT && evt.type == BTN_EVENT_SHORT) {
            current_idx = (current_idx - 1 + NUM_APPS) % NUM_APPS;
            draw_carousel();
        } else if (evt.id == BTN_RIGHT && evt.type == BTN_EVENT_SHORT) {
            current_idx = (current_idx + 1) % NUM_APPS;
            draw_carousel();
        } else if (evt.id == BTN_RIGHT && evt.type == BTN_EVENT_LONG) {
            in_app = true;
            ESP_LOGI(TAG, ">>> Opening [%s]", apps[current_idx]->name);
            if (apps[current_idx]->on_enter) {
                apps[current_idx]->on_enter();
            }
        }
        return;
    }

    // --- 4. ACTIVE APP MODE ---
    if (evt.id == BTN_RIGHT && evt.type == BTN_EVENT_LONG) {
        if (apps[current_idx]->on_exit) {
            apps[current_idx]->on_exit();
        }
        in_app = false;
        draw_carousel();
    } else if (apps[current_idx]->on_button) {
        apps[current_idx]->on_button(evt);
    }
}

void app_mgr_handle_double_tap(void)
{
    if (!sleep_overlay_active && in_app && apps[current_idx]->on_double_tap) {
        apps[current_idx]->on_double_tap();
    }
}

void app_mgr_tick(uint32_t dt_ms, const imu_euler_t *euler, const imu_calib_t *calib)
{
    if (!sleep_overlay_active && in_app && apps[current_idx]->on_tick) {
        apps[current_idx]->on_tick(dt_ms, euler, calib);
    }
}