#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    float heading; // Degrees (0 to 360)
    float roll;    // Degrees (-90 to 90)
    float pitch;   // Degrees (-180 to 180)
} imu_euler_t;

typedef struct {
    float x; // m/s^2 (gravity removed)
    float y;
    float z;
} imu_vec3_t;

typedef struct {
    uint8_t sys;   // 0 (uncalibrated) to 3 (fully calibrated)
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} imu_calib_t;

esp_err_t imu_init(gpio_num_t sda_pin, gpio_num_t scl_pin, gpio_num_t pwr_pin);
esp_err_t imu_read_euler(imu_euler_t *out_euler);
esp_err_t imu_read_linear_accel(imu_vec3_t *out_accel);
esp_err_t imu_read_calib(imu_calib_t *out_calib);
esp_err_t imu_sleep(void);