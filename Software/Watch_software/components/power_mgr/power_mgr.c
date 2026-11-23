#include "power_mgr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "POWER_MGR";

uint32_t power_mgr_get_wakeup_causes(void)
{
    return esp_sleep_get_wakeup_causes();
}

void power_mgr_enter_deep_sleep(gpio_num_t wake_pin)
{
    ESP_LOGI(TAG, "Preparing for Deep Sleep. Release button to sleep...");

    // Wait until the wake button is released (HIGH) so it doesn't wake up immediately
    while (gpio_get_level(wake_pin) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Settle release bounce

    // Arm EXT1 wake-up on active-LOW (external 3k pull-up keeps it HIGH while idle)
    esp_sleep_enable_ext1_wakeup_io(1ULL << wake_pin, ESP_EXT1_WAKEUP_ANY_LOW);

    ESP_LOGI(TAG, "Entering Deep Sleep now.");
    vTaskDelay(pdMS_TO_TICKS(50)); // Let UART/USB log finish flushing

    esp_deep_sleep_start();
}