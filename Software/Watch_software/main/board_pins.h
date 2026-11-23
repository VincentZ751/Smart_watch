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