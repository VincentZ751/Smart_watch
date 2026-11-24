#pragma once

#include <stddef.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/rmt_types.h"
#include "esp_err.h"

#define IR_MAX_SYMBOLS  128

typedef struct {
    rmt_symbol_word_t symbols[IR_MAX_SYMBOLS];
    size_t num_symbols;
} ir_signal_t;

esp_err_t ir_init(gpio_num_t tx_pin, gpio_num_t rx_pin);

// Blocks up to timeout_ms waiting for an IR remote press.
// Returns ESP_OK if a signal was captured, ESP_ERR_TIMEOUT otherwise.
esp_err_t ir_receive(ir_signal_t *out_signal, uint32_t timeout_ms);

// Blasts a previously captured signal out at 38 kHz.
esp_err_t ir_transmit(const ir_signal_t *signal);