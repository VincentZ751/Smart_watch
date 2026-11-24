#pragma once
#include "driver/gpio.h"


// For buttons
#define PIN_LED_BUZZER  GPIO_NUM_5   // A3 / IO5
#define PIN_BTN_SELECT  GPIO_NUM_1   // A0
#define PIN_BTN_UP      GPIO_NUM_15  // IO15
#define PIN_BTN_DOWN    GPIO_NUM_14  // IO1


// ST7789 Display (SPI2)
#define PIN_LCD_SCK     GPIO_NUM_21
#define PIN_LCD_MOSI    GPIO_NUM_22 
#define PIN_LCD_CS      GPIO_NUM_7
#define PIN_LCD_DC      GPIO_NUM_6
#define PIN_LCD_BL      GPIO_NUM_0
#define PIN_LCD_RST     GPIO_NUM_NC  // No hardware reset pin


// I2C Bus (BNO055 IMU)
#define PIN_I2C_SCL     GPIO_NUM_18
#define PIN_I2C_SDA     GPIO_NUM_19
#define PIN_I2C_PWR     GPIO_NUM_20  // Feather onboard I2C pull-up / STEMMA QT power
#define PIN_IMU_INT     GPIO_NUM_4   // A1 (LP GPIO - can wake from deep sleep)


// IR transmitter/reciever
#define PIN_IR_RX   GPIO_NUM_3  // A4 (IRin - 38kHz Receiver)
#define PIN_IR_TX   GPIO_NUM_2  // A5 (IRout - 940nm Emitter)