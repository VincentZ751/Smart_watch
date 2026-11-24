#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "nvs.h"

typedef enum {
    BLE_HID_PLAY_PAUSE,
    BLE_HID_NEXT_TRACK,
    BLE_HID_PREV_TRACK,
    BLE_HID_VOL_UP,
    BLE_HID_VOL_DOWN,
    BLE_HID_CALL_ANSWER_END,
    BLE_HID_CALL_MUTE
} ble_hid_key_t;

esp_err_t ble_time_sync_init(void);

// One-shot time sync (connects -> reads time -> disconnects)
esp_err_t ble_time_sync_start(void);
void ble_time_sync_stop(void);
bool ble_time_sync_is_active(void);

// Persistent HID mode (stays connected for media & call controls)
esp_err_t ble_hid_mode_start(void);
void ble_hid_mode_stop(void);
bool ble_hid_is_connected(void);
esp_err_t ble_hid_send_key(ble_hid_key_t key);