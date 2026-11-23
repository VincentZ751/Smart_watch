#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "board_pins.h"
#include "buttons.h"
#include "power_mgr.h"
#include "display_st7789.h"

static const char *TAG = "WATCH_MAIN";

RTC_DATA_ATTR static int boot_count = 0;

#define INACTIVITY_TIMEOUT_MS 60000  // 1 minute

static const uint16_t test_colors[] = {
    COLOR_BLUE, COLOR_RED, COLOR_GREEN, COLOR_CYAN, COLOR_YELLOW, COLOR_WHITE
};
#define NUM_COLORS (sizeof(test_colors) / sizeof(test_colors[0]))

void app_main(void)
{
    boot_count++;

    // Turn LED ON while awake
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << PIN_LED_BUZZER),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_cfg);
    gpio_set_level(PIN_LED_BUZZER, 1);

    // Initialize display and fill initial color BEFORE turning on backlight
    display_pins_t lcd_pins = {
        .sck  = PIN_LCD_SCK,
        .mosi = PIN_LCD_MOSI,
        .cs   = PIN_LCD_CS,
        .dc   = PIN_LCD_DC,
        .rst  = PIN_LCD_RST,
        .bl   = PIN_LCD_BL,
    };
    display_init(&lcd_pins);

    int color_idx = 0;
    display_fill_color(test_colors[color_idx]);
    display_set_backlight(true);

    buttons_init(PIN_BTN_SELECT, PIN_BTN_UP, PIN_BTN_DOWN);

    btn_event_t evt;
    ESP_LOGI(TAG, "Awake! Auto-sleep in 60s, or long-press SELECT to sleep now.");

    while (1) {
        if (buttons_get_event(&evt, pdMS_TO_TICKS(INACTIVITY_TIMEOUT_MS))) {
            if (evt.id == BTN_SELECT && evt.type == BTN_EVENT_LONG) {
                gpio_set_level(PIN_LED_BUZZER, 0);
                display_sleep();
                power_mgr_enter_deep_sleep(PIN_BTN_SELECT);
            } else if (evt.id == BTN_UP) {
                color_idx = (color_idx + 1) % NUM_COLORS;
                display_fill_color(test_colors[color_idx]);

                // Brief blink to confirm press
                gpio_set_level(PIN_LED_BUZZER, 0);
                vTaskDelay(pdMS_TO_TICKS(60));
                gpio_set_level(PIN_LED_BUZZER, 1);
            } else if (evt.id == BTN_DOWN) {
                color_idx = (color_idx - 1 + NUM_COLORS) % NUM_COLORS;
                display_fill_color(test_colors[color_idx]);

                // Brief blink to confirm press
                gpio_set_level(PIN_LED_BUZZER, 0);
                vTaskDelay(pdMS_TO_TICKS(60));
                gpio_set_level(PIN_LED_BUZZER, 1);
            }
        } else {
            // 1-minute inactivity timeout
            gpio_set_level(PIN_LED_BUZZER, 0);
            display_sleep();
            power_mgr_enter_deep_sleep(PIN_BTN_SELECT);
        }
    }
}