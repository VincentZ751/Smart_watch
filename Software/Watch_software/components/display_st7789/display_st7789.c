#include "display_st7789.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "DISPLAY";

#define LCD_SPI_HOST        SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ  (10 * 1000 * 1000) // 10 MHz (safe for breadboard jumpers)
#define CHUNK_LINES         20

static esp_lcd_panel_handle_t panel_handle = NULL;
static gpio_num_t bl_pin = GPIO_NUM_NC;

void display_init(const display_pins_t *pins)
{
    bl_pin = pins->bl;

    // 1. Configure backlight pin (start OFF so we don't flash random RAM noise)
    if (bl_pin != GPIO_NUM_NC) {
        gpio_config_t bl_cfg = {
            .pin_bit_mask = (1ULL << bl_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&bl_cfg);
        gpio_set_level(bl_pin, 0);
    }

    // 2. Initialize SPI bus
    spi_bus_config_t buscfg = {
        .sclk_io_num = pins->sck,
        .mosi_io_num = pins->mosi,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * CHUNK_LINES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 3. Attach LCD to SPI bus
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = pins->dc,
        .cs_gpio_num = pins->cs,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_config, &io_handle));

    // 4. Install ST7789 panel driver
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = pins->rst,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    // 5. Reset, init, and turn on display
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true)); // ST7789 IPS panels usually need invert=true
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "ST7789 initialized.");
}

void display_set_backlight(bool on)
{
    if (bl_pin != GPIO_NUM_NC) {
        gpio_set_level(bl_pin, on ? 1 : 0);
    }
}

void display_fill_color(uint16_t color_rgb565)
{
    static uint16_t *buf = NULL;
    size_t chunk_pixels = LCD_WIDTH * CHUNK_LINES;

    if (!buf) {
        buf = heap_caps_malloc(chunk_pixels * sizeof(uint16_t), MALLOC_CAP_DMA);
        if (!buf) {
            ESP_LOGE(TAG, "Failed to allocate DMA buffer");
            return;
        }
    }

    uint16_t swapped = (color_rgb565 >> 8) | (color_rgb565 << 8);
    for (size_t i = 0; i < chunk_pixels; i++) {
        buf[i] = swapped;
    }

    for (int y = 0; y < LCD_HEIGHT; y += CHUNK_LINES) {
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, LCD_WIDTH, y + CHUNK_LINES, buf);
    }
}

void display_sleep(void)
{
    display_set_backlight(false);
    if (panel_handle) {
        esp_lcd_panel_disp_on_off(panel_handle, false);
    }
}