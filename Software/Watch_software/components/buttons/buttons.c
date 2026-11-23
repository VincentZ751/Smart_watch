#include "buttons.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define DEBOUNCE_US     50000  // 50 ms ISR rate-limit
#define CONFIRM_MS      20     // 20 ms press confirmation
#define POLL_MS         10     // Check interval only while finger is on button
#define LONG_PRESS_MS   600    // Hold duration for a long press

static QueueHandle_t btn_queue = NULL;
static int64_t last_press_time[BTN_COUNT] = {0};
static gpio_num_t btn_pins[BTN_COUNT];

static void IRAM_ATTR button_isr_handler(void *arg)
{
    btn_id_t btn = (btn_id_t)(uintptr_t)arg;
    int64_t now = esp_timer_get_time();

    if ((now - last_press_time[btn]) > DEBOUNCE_US) {
        last_press_time[btn] = now;
        xQueueSendFromISR(btn_queue, &btn, NULL);
    }
}

void buttons_init(gpio_num_t select_pin, gpio_num_t up_pin, gpio_num_t down_pin)
{
    btn_pins[BTN_SELECT] = select_pin;
    btn_pins[BTN_UP]     = up_pin;
    btn_pins[BTN_DOWN]   = down_pin;

    btn_queue = xQueueCreate(10, sizeof(btn_id_t));

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << select_pin) | (1ULL << up_pin) | (1ULL << down_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,     // Using external 3k pull-ups
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&cfg);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(select_pin, button_isr_handler, (void *)(uintptr_t)BTN_SELECT);
    gpio_isr_handler_add(up_pin,     button_isr_handler, (void *)(uintptr_t)BTN_UP);
    gpio_isr_handler_add(down_pin,   button_isr_handler, (void *)(uintptr_t)BTN_DOWN);
}

bool buttons_get_event(btn_event_t *out_event, TickType_t wait_ticks)
{
    TickType_t start_tick = xTaskGetTickCount();

    while (1) {
        TickType_t remaining = wait_ticks;
        if (wait_ticks != portMAX_DELAY) {
            TickType_t elapsed = xTaskGetTickCount() - start_tick;
            if (elapsed >= wait_ticks) {
                return false; // Actual timeout
            }
            remaining = wait_ticks - elapsed;
        }

        btn_id_t btn;
        if (xQueueReceive(btn_queue, &btn, remaining) != pdTRUE) {
            return false; // Actual timeout
        }

        // Wait for mechanical bounce to settle
        vTaskDelay(pdMS_TO_TICKS(CONFIRM_MS));
        if (gpio_get_level(btn_pins[btn]) != 0) {
            continue; // Noise or release bounce -> silently keep waiting
        }

        // Button is genuinely held: track duration until release or threshold
        int held_ms = CONFIRM_MS;
        while (gpio_get_level(btn_pins[btn]) == 0) {
            if (held_ms >= LONG_PRESS_MS) {
                out_event->id = btn;
                out_event->type = BTN_EVENT_LONG;
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            held_ms += POLL_MS;
        }

        // Released before LONG_PRESS_MS
        out_event->id = btn;
        out_event->type = BTN_EVENT_SHORT;
        return true;
    }
}