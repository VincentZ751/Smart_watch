#pragma once

#include <stdbool.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

typedef enum {
    BTN_SELECT = 0,
    BTN_UP,
    BTN_DOWN,
    BTN_COUNT
} btn_id_t;

typedef enum {
    BTN_EVENT_SHORT = 0,
    BTN_EVENT_LONG
} btn_event_type_t;

typedef struct {
    btn_id_t id;
    btn_event_type_t type;
} btn_event_t;

void buttons_init(gpio_num_t select_pin, gpio_num_t up_pin, gpio_num_t down_pin);
bool buttons_get_event(btn_event_t *out_event, TickType_t wait_ticks);