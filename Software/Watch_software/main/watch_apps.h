#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "buttons.h"
#include "imu_bno055.h"

typedef void (*sleep_request_cb_t)(const char *reason);

typedef struct {
    const char *name;
    void (*on_enter)(void);
    void (*on_exit)(void);
    void (*on_button)(btn_event_t evt);
    void (*on_double_tap)(void);
    void (*on_tick)(uint32_t dt_ms, const imu_euler_t *euler, const imu_calib_t *calib);
    void (*on_draw)(void);
} watch_app_t;

void app_mgr_init(sleep_request_cb_t sleep_cb);
void app_mgr_handle_button(btn_event_t evt);
void app_mgr_handle_double_tap(void);
void app_mgr_tick(uint32_t dt_ms, const imu_euler_t *euler, const imu_calib_t *calib);