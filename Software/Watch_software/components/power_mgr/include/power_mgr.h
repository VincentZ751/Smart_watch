#pragma once

#include "driver/gpio.h"
#include "esp_sleep.h"

uint32_t power_mgr_get_wakeup_causes(void);
void power_mgr_enter_deep_sleep(gpio_num_t wake_pin);