#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "board_pins.h"
#include "buttons.h"
#include "power_mgr.h"
#include "imu_bno055.h"
#include "ir_transceiver.h"
#include "watch_apps.h"

static const char *TAG = "WATCH_MAIN";

RTC_DATA_ATTR static int boot_count = 0;

#define INACTIVITY_TIMEOUT_MS   180000  // 3 minutes
#define LOOP_PERIOD_MS          20      // 50 Hz loop
#define ANGLE_CHECK_MS          100     // 10 Hz orientation check

// Double-tap tuning
#define TAP_THRESHOLD_MS2       6.5f
#define TAP_QUIET_MS2           2.0f
#define TAP_MAX_DURATION_MS     80
#define TAP_MIN_GAP_MS          100
#define TAP_MAX_GAP_MS          450

typedef enum {
    TAP_IDLE,
    TAP_1_ACTIVE,
    TAP_WAIT_2
} tap_state_t;

static void blink_led(int ms)
{
    gpio_set_level(PIN_LED_BUZZER, 0);
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(PIN_LED_BUZZER, 1);
}

static void go_to_sleep(const char *reason)
{
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "ENTERING SLEEP (%s)", reason);
    ESP_LOGI(TAG, "=========================================");

    imu_sleep();

    gpio_set_level(PIN_LED_BUZZER, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(PIN_LED_BUZZER, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(PIN_LED_BUZZER, 0);

    ESP_LOGI(TAG, "ESP32-C6 -> DEEP SLEEP. Press LEFT (GPIO1) to wake.");
    vTaskDelay(pdMS_TO_TICKS(100));
    power_mgr_enter_deep_sleep(PIN_BTN_SELECT);
}

void app_main(void)
{
    boot_count++;

    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << PIN_LED_BUZZER),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_cfg);
    gpio_set_level(PIN_LED_BUZZER, 1);

    uint32_t causes = esp_sleep_get_wakeup_causes();
    ESP_LOGI(TAG, "WATCH AWAKE! Boot #%d | Wakeup: %s",
             boot_count,
             (causes & BIT(ESP_SLEEP_WAKEUP_EXT1)) ? "LEFT Button (from Deep Sleep)" : "Power-On / Reset");

    for (int i = 0; i < 3; i++) {
        blink_led(60);
        vTaskDelay(pdMS_TO_TICKS(60));
    }

    buttons_init(PIN_BTN_SELECT, PIN_BTN_UP, PIN_BTN_DOWN);

    if (imu_init(PIN_I2C_SDA, PIN_I2C_SCL, PIN_I2C_PWR) != ESP_OK) {
        ESP_LOGE(TAG, "IMU init FAILED!");
        while (1) {
            blink_led(150);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
    }

    ESP_ERROR_CHECK(ir_init(PIN_IR_TX, PIN_IR_RX));

    // Launch 2-button Carousel UI & pass sleep callback
    app_mgr_init(go_to_sleep);

    btn_event_t evt;
    imu_euler_t euler = {0};
    imu_vec3_t accel = {0};
    imu_calib_t calib = {0};

    TickType_t last_activity    = xTaskGetTickCount();
    TickType_t last_tick        = xTaskGetTickCount();
    TickType_t last_angle_check = xTaskGetTickCount();
    TickType_t tap_start_tick   = 0;

    tap_state_t tap_state = TAP_IDLE;
    bool in_viewing_angle = false;

    while (1) {
        if (buttons_get_event(&evt, pdMS_TO_TICKS(LOOP_PERIOD_MS))) {
            blink_led(50);
            app_mgr_handle_button(evt);
            last_activity = xTaskGetTickCount();
            continue;
        }

        TickType_t now = xTaskGetTickCount();
        uint32_t dt_ms = (now - last_tick) * portTICK_PERIOD_MS;
        last_tick = now;

        // --- 1. Orientation & Calibration Read (10 Hz) ---
        if (((now - last_angle_check) * portTICK_PERIOD_MS) >= ANGLE_CHECK_MS) {
            last_angle_check = now;
            if (imu_read_euler(&euler) == ESP_OK) {
                in_viewing_angle = (fabsf(euler.roll) < 35.0f && fabsf(euler.pitch) < 35.0f);
            }
            imu_read_calib(&calib);
        }

        // --- 2. Double-Tap Detection (50 Hz, only when facing up) ---
        if (!in_viewing_angle) {
            tap_state = TAP_IDLE;
        } else if (imu_read_linear_accel(&accel) == ESP_OK) {
            float mag = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
            uint32_t tap_dt = (now - tap_start_tick) * portTICK_PERIOD_MS;

            switch (tap_state) {
                case TAP_IDLE:
                    if (mag >= TAP_THRESHOLD_MS2) {
                        tap_state = TAP_1_ACTIVE;
                        tap_start_tick = now;
                    }
                    break;

                case TAP_1_ACTIVE:
                    if (mag < TAP_QUIET_MS2) {
                        tap_state = TAP_WAIT_2;
                    } else if (tap_dt > TAP_MAX_DURATION_MS) {
                        tap_state = TAP_IDLE;
                    }
                    break;

                case TAP_WAIT_2:
                    if (tap_dt > TAP_MAX_GAP_MS) {
                        tap_state = TAP_IDLE;
                    } else if (tap_dt >= TAP_MIN_GAP_MS && mag >= TAP_THRESHOLD_MS2) {
                        tap_state = TAP_IDLE;
                        last_activity = now;
                        blink_led(60);
                        app_mgr_handle_double_tap();
                        vTaskDelay(pdMS_TO_TICKS(150));
                    }
                    break;
            }
        }

        // --- 3. Tick Active App ---
        app_mgr_tick(dt_ms, &euler, &calib);

        // --- 4. 3-Minute Inactivity Timeout ---
        if (((now - last_activity) * portTICK_PERIOD_MS) >= INACTIVITY_TIMEOUT_MS) {
            go_to_sleep("3-min Inactivity Timeout");
        }
    }
}