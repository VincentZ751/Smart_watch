#include "watch_apps.h"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ir_transceiver.h"
#include "ble_time_sync.h"

static const char *TAG = "WATCH_APPS";

// Shared state
static imu_euler_t latest_euler = {0};
static imu_calib_t latest_calib = {0};
static sleep_request_cb_t on_sleep_request = NULL;

// ============================================================================
// APP 1: WATCHFACE (REAL RTC CLOCK + 3-BUTTON TIME SET)
// ============================================================================
typedef enum {
    WF_MODE_NORMAL,
    WF_MODE_EDIT_HOUR,
    WF_MODE_EDIT_MIN
} wf_mode_t;

static wf_mode_t wf_mode = WF_MODE_NORMAL;
static bool wf_use_24h = false;
static uint32_t wf_accum_ms = 0;

static void rtc_ensure_valid_time(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    // Seed a default time only on cold boot (1970). Deep Sleep keeps the RTC ticking.
    if (tv.tv_sec < 1700000000) {
        struct tm t = {
            .tm_year = 2026 - 1900,
            .tm_mon  = 10,   // November (0-indexed)
            .tm_mday = 23,
            .tm_hour = 12,
            .tm_min  = 0,
            .tm_sec  = 0,
            .tm_isdst = -1
        };
        tv.tv_sec = mktime(&t);
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);
    }
}

static void wf_adjust_time(int delta_hours, int delta_mins)
{
    time_t now;
    struct tm t;
    time(&now);
    localtime_r(&now, &t);

    if (delta_hours) {
        t.tm_hour = (t.tm_hour + delta_hours + 24) % 24;
    }
    if (delta_mins) {
        t.tm_min = (t.tm_min + delta_mins + 60) % 60;
        t.tm_sec = 0;
    }
    t.tm_isdst = -1;

    struct timeval tv = { .tv_sec = mktime(&t), .tv_usec = 0 };
    settimeofday(&tv, NULL);
}

static void wf_draw(void)
{
    time_t now;
    struct tm t;
    time(&now);
    localtime_r(&now, &t);

    char date_str[24];
    strftime(date_str, sizeof(date_str), "%a %b %d, %Y", &t);

    int disp_hr = t.tm_hour;
    const char *ampm = "";
    if (!wf_use_24h) {
        ampm = (disp_hr >= 12) ? " PM" : " AM";
        disp_hr = disp_hr % 12;
        if (disp_hr == 0) disp_hr = 12;
    }

    if (wf_mode == WF_MODE_EDIT_HOUR) {
        ESP_LOGW(TAG, "[WATCHFACE - SET HOUR] [%02d]:%02d:%02d%s | UP:+1 | DOWN:-1 | SELECT:Next",
                 disp_hr, t.tm_min, t.tm_sec, ampm);
    } else if (wf_mode == WF_MODE_EDIT_MIN) {
        ESP_LOGW(TAG, "[WATCHFACE - SET MIN ]  %02d:[%02d]:%02d%s | UP:+1 | DOWN:-1 | SELECT:Save",
                 disp_hr, t.tm_min, t.tm_sec, ampm);
    } else {
        ESP_LOGI(TAG, "[WATCHFACE] %02d:%02d:%02d%s | %s | UP:Set | DOWN:BLE Sync | Hold SEL:Back",
                 disp_hr, t.tm_min, t.tm_sec, ampm, date_str);
    }
}

static void wf_enter(void)
{
    wf_mode = WF_MODE_NORMAL;
    wf_accum_ms = 0;
    wf_draw();
}

static void wf_button(btn_event_t evt)
{
    if (evt.type != BTN_EVENT_SHORT) return;

    if (wf_mode == WF_MODE_NORMAL) {
        if (evt.id == BTN_UP) {
            wf_mode = WF_MODE_EDIT_HOUR;
            wf_draw();
        } else if (evt.id == BTN_DOWN) {
            if (ble_time_sync_is_active()) {
                ble_time_sync_stop();
                ESP_LOGI(TAG, "[WATCHFACE] BLE Time Sync stopped.");
            } else {
                ble_time_sync_start();
                ESP_LOGI(TAG, "[WATCHFACE] BLE Time Sync started...");
            }
        }
    } else if (wf_mode == WF_MODE_EDIT_HOUR) {
        if (evt.id == BTN_UP) {
            wf_adjust_time(1, 0);
            wf_draw();
        } else if (evt.id == BTN_DOWN) {
            wf_adjust_time(-1, 0);
            wf_draw();
        } else if (evt.id == BTN_SELECT) {
            wf_mode = WF_MODE_EDIT_MIN;
            wf_draw();
        }
    } else if (wf_mode == WF_MODE_EDIT_MIN) {
        if (evt.id == BTN_UP) {
            wf_adjust_time(0, 1);
            wf_draw();
        } else if (evt.id == BTN_DOWN) {
            wf_adjust_time(0, -1);
            wf_draw();
        } else if (evt.id == BTN_SELECT) {
            wf_mode = WF_MODE_NORMAL;
            ESP_LOGI(TAG, "[WATCHFACE] Time saved!");
            wf_draw();
        }
    }
}

static void wf_tick(uint32_t dt_ms, const imu_euler_t *euler, const imu_calib_t *calib)
{
    if (wf_mode != WF_MODE_NORMAL) return;

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
    .on_button     = wf_button,
    .on_double_tap = NULL,
    .on_tick       = wf_tick,
    .on_draw       = wf_draw,
};

// ============================================================================
// APP 2: IR REMOTE (TV-B-GONE DEFAULT + 4 NVS CUSTOM SLOTS)
// ============================================================================
// Flip these two if your ir_transceiver.c TX carrier is active-LOW (0 = LED ON)
#define IR_MARK_LEVEL   1
#define IR_SPACE_LEVEL  0

#define NUM_CUSTOM_IR_SLOTS  4
#define NUM_IR_MODES         (1 + NUM_CUSTOM_IR_SLOTS) // 0 = TV-B-Gone, 1..4 = Custom 1..4

static ir_signal_t custom_ir[NUM_CUSTOM_IR_SLOTS] = {0};
static int ir_slot_idx = 0; // Default to 0 (Generic TVs)

static inline void ir_set_sym(ir_signal_t *sig, size_t idx, uint16_t mark_us, uint16_t space_us)
{
    sig->symbols[idx].level0    = IR_MARK_LEVEL;
    sig->symbols[idx].duration0 = mark_us;
    sig->symbols[idx].level1    = IR_SPACE_LEVEL;
    sig->symbols[idx].duration1 = space_us;
}

// LG (NEC 32-bit, MSB-first): Power = 0x20DF10EF
static void ir_build_nec(ir_signal_t *sig, uint32_t code)
{
    size_t i = 0;
    ir_set_sym(sig, i++, 9000, 4500); // Header
    for (int bit = 31; bit >= 0; bit--) {
        bool one = (code >> bit) & 1U;
        ir_set_sym(sig, i++, 560, one ? 1690 : 560);
    }
    ir_set_sym(sig, i++, 560, 0);     // Stop bit
    sig->num_symbols = i;
}

// Samsung (Samsung32, MSB-first): Power = 0xE0E040BF
static void ir_build_samsung(ir_signal_t *sig, uint32_t code)
{
    size_t i = 0;
    ir_set_sym(sig, i++, 4500, 4500); // Header
    for (int bit = 31; bit >= 0; bit--) {
        bool one = (code >> bit) & 1U;
        ir_set_sym(sig, i++, 560, one ? 1690 : 560);
    }
    ir_set_sym(sig, i++, 560, 0);     // Stop bit
    sig->num_symbols = i;
}

// Sony (SIRC 12-bit, MSB-first): Power = 0xA90
static void ir_build_sony12(ir_signal_t *sig, uint16_t code)
{
    size_t i = 0;
    ir_set_sym(sig, i++, 2400, 600);  // Header
    for (int bit = 11; bit >= 0; bit--) {
        bool one = (code >> bit) & 1U;
        ir_set_sym(sig, i++, one ? 1200 : 600, 600);
    }
    sig->num_symbols = i;
}

static void ir_nvs_load_all(void)
{
    nvs_handle_t h;
    if (nvs_open("ir_slots", NVS_READONLY, &h) != ESP_OK) return;

    for (int i = 0; i < NUM_CUSTOM_IR_SLOTS; i++) {
        char key[12];
        snprintf(key, sizeof(key), "slot_%d", i);
        size_t len = sizeof(ir_signal_t);
        if (nvs_get_blob(h, key, &custom_ir[i], &len) == ESP_OK) {
            ESP_LOGI(TAG, "[IR NVS] Loaded Custom %d (%u symbols)", i + 1, (unsigned)custom_ir[i].num_symbols);
        }
    }
    nvs_close(h);
}

static void ir_nvs_save_slot(int slot)
{
    nvs_handle_t h;
    if (nvs_open("ir_slots", NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "[IR NVS] Failed to open NVS for write");
        return;
    }

    char key[12];
    snprintf(key, sizeof(key), "slot_%d", slot);
    esp_err_t err = nvs_set_blob(h, key, &custom_ir[slot], sizeof(ir_signal_t));
    if (err == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "[IR NVS] Saved Custom %d to flash!", slot + 1);
    } else {
        ESP_LOGE(TAG, "[IR NVS] Save failed: %s", esp_err_to_name(err));
    }
    nvs_close(h);
}

static void ir_app_draw(void)
{
    if (ir_slot_idx == 0) {
        ESP_LOGI(TAG, "[IR REMOTE] [ TV-B-Gone: LG + Samsung + Sony ] | SEL:Blast | UP/DN:Slot | Hold SEL:Back");
    } else {
        int c = ir_slot_idx - 1;
        ESP_LOGI(TAG, "[IR REMOTE] [ Custom %d (%u syms) ] | SEL:Blast | Hold UP:Record | UP/DN:Slot | Hold SEL:Back",
                 ir_slot_idx, (unsigned)custom_ir[c].num_symbols);
    }
}

static void ir_app_enter(void)
{
    ir_slot_idx = 0; // Always default to Generic TVs
    ir_app_draw();
}

static void ir_app_blast(void)
{
    if (ir_slot_idx == 0) {
        ir_signal_t tmp = {0};

        ESP_LOGI(TAG, "[IR REMOTE] >>> Blasting LG Power (0x20DF10EF)...");
        ir_build_nec(&tmp, 0x20DF10EF);
        ir_transmit(&tmp);
        vTaskDelay(pdMS_TO_TICKS(60));

        ESP_LOGI(TAG, "[IR REMOTE] >>> Blasting Samsung Power (0xE0E040BF)...");
        ir_build_samsung(&tmp, 0xE0E040BF);
        ir_transmit(&tmp);
        vTaskDelay(pdMS_TO_TICKS(60));

        ESP_LOGI(TAG, "[IR REMOTE] >>> Blasting Sony Power (0xA90 x3)...");
        ir_build_sony12(&tmp, 0xA90);
        for (int i = 0; i < 3; i++) {
            ir_transmit(&tmp);
            vTaskDelay(pdMS_TO_TICKS(25));
        }
        return;
    }

    int c = ir_slot_idx - 1;
    if (custom_ir[c].num_symbols == 0) {
        ESP_LOGW(TAG, "[IR REMOTE] Custom %d is empty! Hold UP to record first.", ir_slot_idx);
        return;
    }

    ESP_LOGI(TAG, "[IR REMOTE] >>> Blasting Custom %d (%u symbols, 3x)!",
             ir_slot_idx, (unsigned)custom_ir[c].num_symbols);
    for (int i = 0; i < 3; i++) {
        if (ir_transmit(&custom_ir[c]) != ESP_OK) {
            ESP_LOGE(TAG, "[IR REMOTE] TX failed on burst %d", i + 1);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

static void ir_app_button(btn_event_t evt)
{
    if (evt.type == BTN_EVENT_SHORT) {
        if (evt.id == BTN_UP) {
            ir_slot_idx = (ir_slot_idx - 1 + NUM_IR_MODES) % NUM_IR_MODES;
            ir_app_draw();
        } else if (evt.id == BTN_DOWN) {
            ir_slot_idx = (ir_slot_idx + 1) % NUM_IR_MODES;
            ir_app_draw();
        } else if (evt.id == BTN_SELECT) {
            ir_app_blast();
        }
    } else if (evt.type == BTN_EVENT_LONG && evt.id == BTN_UP) {
        if (ir_slot_idx == 0) {
            ESP_LOGW(TAG, "[IR REMOTE] Press UP/DOWN to pick Custom 1-4 before recording!");
            return;
        }
        int c = ir_slot_idx - 1;
        ESP_LOGI(TAG, "[IR REMOTE] Recording Custom %d (5 sec)... Point remote & press button!", ir_slot_idx);
        if (ir_receive(&custom_ir[c], 5000) == ESP_OK) {
            ESP_LOGI(TAG, "[IR REMOTE] Captured %u symbols!", (unsigned)custom_ir[c].num_symbols);
            ir_nvs_save_slot(c);
        } else {
            ESP_LOGW(TAG, "[IR REMOTE] Timed out (no signal).");
        }
        ir_app_draw();
    }
}

static const watch_app_t app_ir_remote = {
    .name          = "IR Remote",
    .on_enter      = ir_app_enter,
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
    ESP_LOGI(TAG, "[COMPASS] H:%5.1f° R:%5.1f° P:%5.1f° | Calib S:%d G:%d A:%d M:%d | Hold SEL:Back",
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
// APP 4: MEDIA & CALL REMOTE (BLE HID)
// ============================================================================
static void media_draw(void)
{
    ESP_LOGI(TAG, "[MEDIA/CALL] %s | SEL:Play/Call | UP/DN:Vol | Hold UP:Skip | Hold DN:Prev | Hold SEL:Back",
             ble_hid_is_connected() ? "CONNECTED" : "CONNECTING...");
}

static void media_enter(void)
{
    ble_hid_mode_start();
    media_draw();
}

static void media_exit(void)
{
    ble_hid_mode_stop();
}

static void media_button(btn_event_t evt)
{
    if (evt.type == BTN_EVENT_SHORT) {
        if (evt.id == BTN_SELECT) {
            ESP_LOGI(TAG, "[MEDIA/CALL] >>> Play/Pause (or Answer/End Call)");
            ble_hid_send_key(BLE_HID_PLAY_PAUSE);
            ble_hid_send_key(BLE_HID_CALL_ANSWER_END);
        } else if (evt.id == BTN_UP) {
            ESP_LOGI(TAG, "[MEDIA/CALL] >>> Volume Up");
            ble_hid_send_key(BLE_HID_VOL_UP);
        } else if (evt.id == BTN_DOWN) {
            ESP_LOGI(TAG, "[MEDIA/CALL] >>> Volume Down");
            ble_hid_send_key(BLE_HID_VOL_DOWN);
        }
    } else if (evt.type == BTN_EVENT_LONG) {
        if (evt.id == BTN_UP) {
            ESP_LOGI(TAG, "[MEDIA/CALL] >>> Next Track");
            ble_hid_send_key(BLE_HID_NEXT_TRACK);
        } else if (evt.id == BTN_DOWN) {
            ESP_LOGI(TAG, "[MEDIA/CALL] >>> Previous Track");
            ble_hid_send_key(BLE_HID_PREV_TRACK);
        }
    }
}

static const watch_app_t app_media = {
    .name          = "Media/Call",
    .on_enter      = media_enter,
    .on_exit       = media_exit,
    .on_button     = media_button,
    .on_double_tap = NULL,
    .on_tick       = NULL,
    .on_draw       = media_draw,
};

// ============================================================================
// CAROUSEL, SLEEP OVERLAY & APP MANAGER
// ============================================================================
static const watch_app_t *apps[] = {
    &app_watchface,
    &app_ir_remote,
    &app_compass,
    &app_media,
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
    ESP_LOGI(TAG, "  UP: Left | DOWN: Right | SELECT: Open | Hold SELECT: Sleep");
    ESP_LOGI(TAG, "=========================================");
}

static void draw_sleep_overlay(void)
{
    ESP_LOGW(TAG, "=========================================");
    ESP_LOGW(TAG, "         POWER OFF / SLEEP?");
    ESP_LOGW(TAG, "  Double-tap SELECT to confirm | UP/DOWN to cancel");
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
    rtc_ensure_valid_time();
    ir_nvs_load_all();
    on_sleep_request = sleep_cb;
    current_idx = 0;
    in_app = false;
    sleep_overlay_active = false;
    sleep_tap_count = 0;
    draw_carousel();
}

void app_mgr_handle_button(btn_event_t evt)
{
    // --- 1. APPLE-STYLE SLEEP CONFIRMATION OVERLAY ---
    if (sleep_overlay_active) {
        if (evt.id == BTN_SELECT && evt.type == BTN_EVENT_SHORT) {
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
                ESP_LOGW(TAG, "[SLEEP?] Tap SELECT once more within 2s to shut down...");
            }
        } else {
            ESP_LOGI(TAG, "[SLEEP?] Cancelled.");
            sleep_overlay_active = false;
            sleep_tap_count = 0;
            redraw_current_screen();
        }
        return;
    }

    // --- 2. CAROUSEL MODE ---
    if (!in_app) {
        if (evt.id == BTN_SELECT && evt.type == BTN_EVENT_LONG) {
            sleep_overlay_active = true;
            sleep_tap_count = 0;
            draw_sleep_overlay();
        } else if (evt.id == BTN_UP && evt.type == BTN_EVENT_SHORT) {
            current_idx = (current_idx - 1 + NUM_APPS) % NUM_APPS;
            draw_carousel();
        } else if (evt.id == BTN_DOWN && evt.type == BTN_EVENT_SHORT) {
            current_idx = (current_idx + 1) % NUM_APPS;
            draw_carousel();
        } else if (evt.id == BTN_SELECT && evt.type == BTN_EVENT_SHORT) {
            in_app = true;
            ESP_LOGI(TAG, ">>> Opening [%s]", apps[current_idx]->name);
            if (apps[current_idx]->on_enter) {
                apps[current_idx]->on_enter();
            }
        }
        return;
    }

    // --- 3. ACTIVE APP MODE ---
    // Hold SELECT exits back to the carousel
    if (evt.id == BTN_SELECT && evt.type == BTN_EVENT_LONG) {
        if (apps[current_idx]->on_exit) {
            apps[current_idx]->on_exit();
        }
        in_app = false;
        draw_carousel();
        return;
    }

    // Everything else (including Short SELECT) goes straight to the active app
    if (apps[current_idx]->on_button) {
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