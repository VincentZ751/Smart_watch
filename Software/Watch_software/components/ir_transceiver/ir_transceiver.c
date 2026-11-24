#include "ir_transceiver.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"

static const char *TAG = "IR_RMT";

#define IR_RESOLUTION_HZ    1000000 // 1 MHz = 1 us per tick
#define IR_CARRIER_HZ       40000   // 40 kHz carrier for Sony SIRC

static rmt_channel_handle_t tx_chan = NULL;
static rmt_channel_handle_t rx_chan = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;
static QueueHandle_t rx_queue = NULL;

static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t channel,
                                     const rmt_rx_done_event_data_t *edata,
                                     void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t q = (QueueHandle_t)user_data;
    xQueueSendFromISR(q, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

esp_err_t ir_init(gpio_num_t tx_pin, gpio_num_t rx_pin)
{
    // 1. Setup TX Channel (40 kHz modulated emitter)
    rmt_tx_channel_config_t tx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = tx_pin,
        .mem_block_symbols = 64,
        .resolution_hz = IR_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &tx_chan));

    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33f,
        .frequency_hz = IR_CARRIER_HZ,
        .flags.polarity_active_low = false,
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_chan, &carrier_cfg));

    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &copy_encoder));
    ESP_ERROR_CHECK(rmt_enable(tx_chan));

    // 2. Setup RX Channel (demodulated active-low receiver)
    rmt_rx_channel_config_t rx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = rx_pin,
        .mem_block_symbols = 64,
        .resolution_hz = IR_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rx_chan));

    rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_cb,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, rx_queue));
    ESP_ERROR_CHECK(rmt_enable(rx_chan));

    ESP_LOGI(TAG, "IR RMT initialized (TX: GPIO%d @ %dHz, RX: GPIO%d)", tx_pin, IR_CARRIER_HZ, rx_pin);
    return ESP_OK;
}

esp_err_t ir_receive(ir_signal_t *out_signal, uint32_t timeout_ms)
{
    if (!out_signal) return ESP_ERR_INVALID_ARG;

    rmt_rx_done_event_data_t rx_data;
    xQueueReset(rx_queue);

    rmt_receive_config_t recv_cfg = {
        .signal_range_min_ns = 1250,      // Filter out glitches < 1.25 us
        .signal_range_max_ns = 12000000,  // End frame after 12 ms of silence
    };

    esp_err_t err = rmt_receive(rx_chan, out_signal->symbols,
                                sizeof(out_signal->symbols), &recv_cfg);
    if (err != ESP_OK) return err;

    if (xQueueReceive(rx_queue, &rx_data, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        out_signal->num_symbols = rx_data.num_symbols;

        // Invert active-low receiver symbols to active-high TX symbols
        for (size_t i = 0; i < out_signal->num_symbols; i++) {
            out_signal->symbols[i].level0 = !out_signal->symbols[i].level0;
            out_signal->symbols[i].level1 = !out_signal->symbols[i].level1;
        }
        return ESP_OK;
    }

    rmt_disable(rx_chan);
    rmt_enable(rx_chan);
    return ESP_ERR_TIMEOUT;
}

esp_err_t ir_transmit(const ir_signal_t *signal)
{
    if (!signal || signal->num_symbols == 0) return ESP_ERR_INVALID_ARG;

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };

    esp_err_t err = rmt_transmit(tx_chan, copy_encoder, signal->symbols,
                                 signal->num_symbols * sizeof(rmt_symbol_word_t),
                                 &tx_cfg);
    if (err != ESP_OK) return err;

    return rmt_tx_wait_all_done(tx_chan, portMAX_DELAY);
}