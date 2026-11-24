#include "imu_bno055.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "IMU_BNO055";

#define BNO055_ADDR         0x28 // Default address (0x29 if ADR pin is HIGH)
#define BNO055_REG_CHIP_ID  0x00
#define BNO055_REG_EULER_H  0x1A // 6 bytes: Heading, Roll, Pitch (LSB/MSB)
#define BNO055_REG_OPR_MODE 0x3D

#define BNO055_CHIP_ID_VAL  0xA0
#define BNO055_MODE_NDOF    0x0C
#define BNO055_REG_LIA_X    0x28 // 6 bytes: Linear Acceleration X, Y, Z
#define BNO055_REG_CALIB    0x35 // 1 byte: SYS[7:6], GYR[5:4], ACC[3:2], MAG[1:0]

#define BNO055_REG_PWR_MODE 0x3E

#define BNO055_MODE_CONFIG  0x00
#define BNO055_PWR_NORMAL   0x00
#define BNO055_PWR_SUSPEND  0x02

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;

static esp_err_t bno_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, 100);
}

static esp_err_t bno_write_byte(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev_handle, buf, sizeof(buf), 100);
}

esp_err_t imu_init(gpio_num_t sda_pin, gpio_num_t scl_pin, gpio_num_t pwr_pin)
{
    // 1. Enable Feather's onboard I2C power / pull-ups
    if (pwr_pin != GPIO_NUM_NC) {
        gpio_config_t pwr_cfg = {
            .pin_bit_mask = (1ULL << pwr_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&pwr_cfg);
        gpio_set_level(pwr_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(50)); // Let rail stabilize
    }

    // 2. Initialize I2C Master Bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    // 3. Attach BNO055 device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO055_ADDR,
        .scl_speed_hz = 100000, // 100 kHz for BNO055 clock stretching
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    // BNO055 needs ~650ms after cold power-on before responding
    vTaskDelay(pdMS_TO_TICKS(700));

    // 4. Verify Chip ID
    uint8_t chip_id = 0;
    esp_err_t err = bno_read(BNO055_REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK || chip_id != BNO055_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "BNO055 not found! err=%s, chip_id=0x%02X (expected 0xA0)",
                 esp_err_to_name(err), chip_id);
        return ESP_FAIL;
    }

    // 5. Switch to CONFIG -> NORMAL power -> NDOF fusion mode
    ESP_ERROR_CHECK(bno_write_byte(BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG));
    vTaskDelay(pdMS_TO_TICKS(25));

    ESP_ERROR_CHECK(bno_write_byte(BNO055_REG_PWR_MODE, BNO055_PWR_NORMAL));
    vTaskDelay(pdMS_TO_TICKS(10));

    err = bno_write_byte(BNO055_REG_OPR_MODE, BNO055_MODE_NDOF);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set NDOF mode");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(25));

    ESP_LOGI(TAG, "BNO055 AWAKE (Chip ID: 0x%02X, Power: NORMAL, Mode: NDOF)", chip_id);
    return ESP_OK;
}

esp_err_t imu_read_euler(imu_euler_t *out_euler)
{
    uint8_t raw[6] = {0};
    esp_err_t err = bno_read(BNO055_REG_EULER_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    int16_t h = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t r = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t p = (int16_t)((raw[5] << 8) | raw[4]);

    // BNO055 datasheet: 1 degree = 16 LSB
    out_euler->heading = h / 16.0f;
    out_euler->roll    = r / 16.0f;
    out_euler->pitch   = p / 16.0f;

    return ESP_OK;
}
esp_err_t imu_read_linear_accel(imu_vec3_t *out_accel)
{
    uint8_t raw[6] = {0};
    esp_err_t err = bno_read(BNO055_REG_LIA_X, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]);

    // BNO055 datasheet: 1 m/s^2 = 100 LSB
    out_accel->x = x / 100.0f;
    out_accel->y = y / 100.0f;
    out_accel->z = z / 100.0f;

    return ESP_OK;
}

esp_err_t imu_read_calib(imu_calib_t *out_calib)
{
    uint8_t val = 0;
    esp_err_t err = bno_read(BNO055_REG_CALIB, &val, 1);
    if (err != ESP_OK) {
        return err;
    }

    out_calib->sys   = (val >> 6) & 0x03;
    out_calib->gyro  = (val >> 4) & 0x03;
    out_calib->accel = (val >> 2) & 0x03;
    out_calib->mag   = val & 0x03;

    return ESP_OK;
}
esp_err_t imu_sleep(void)
{
    // Must enter CONFIG mode before changing power mode
    esp_err_t err = bno_write_byte(BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter CONFIG mode before sleep");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(25));

    err = bno_write_byte(BNO055_REG_PWR_MODE, BNO055_PWR_SUSPEND);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BNO055 Suspend mode");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "BNO055 ASLEEP (Power: SUSPEND ~40uA)");
    return ESP_OK;
}