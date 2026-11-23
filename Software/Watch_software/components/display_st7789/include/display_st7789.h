#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"

#define LCD_WIDTH   240
#define LCD_HEIGHT  240

// Common RGB565 colors (byte-swapped for SPI)
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_CYAN    0x07FF
#define COLOR_YELLOW  0xFFE0

typedef struct {
    gpio_num_t sck;
    gpio_num_t mosi;
    gpio_num_t cs;
    gpio_num_t dc;
    gpio_num_t rst;
    gpio_num_t bl;
} display_pins_t;

void display_init(const display_pins_t *pins);
void display_set_backlight(bool on);
void display_fill_color(uint16_t color_rgb565);
void display_sleep(void);