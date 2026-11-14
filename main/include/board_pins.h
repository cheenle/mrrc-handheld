/*
 * Board pins — coolkit-xz-02 == nologo/xingzhi-cube-1.54tft-wifi hardware
 *
 * Recovered from the stock firmware (v1.7.6) and cross-checked against
 * upstream 78/xiaozhi-esp32 main/boards/nologo/xingzhi-cube-1.54tft-wifi.
 * DO NOT "improve" these: every one of them was verified on this exact PCB.
 */
#pragma once

#include <driver/gpio.h>

/* ── LCD: ST7789, SPI, 240x240, needs invert_color ───────────────── */
#define PIN_LCD_SCLK    GPIO_NUM_9
#define PIN_LCD_MOSI    GPIO_NUM_10
#define PIN_LCD_DC      GPIO_NUM_8
#define PIN_LCD_CS      GPIO_NUM_14
#define PIN_LCD_RST     GPIO_NUM_18
#define PIN_LCD_BL      GPIO_NUM_13

#define LCD_H_RES       240
#define LCD_V_RES       240
#define LCD_INVERT      1       /* esp_lcd_panel_invert_color(panel, true) */
#define LCD_RGB_ORDER_RGB 1     /* RGB element order (not BGR) */

/* ── Audio: two simplex I2S channels, no codec chip, no MCLK ─────── */
#define PIN_SPK_BCLK    GPIO_NUM_15
#define PIN_SPK_LRCK    GPIO_NUM_16
#define PIN_SPK_DOUT    GPIO_NUM_7

#define PIN_MIC_SCK     GPIO_NUM_5
#define PIN_MIC_WS      GPIO_NUM_4
#define PIN_MIC_DIN     GPIO_NUM_6

/* ── Buttons (all active-low, internal pull-up) ──────────────────── */
#define PIN_BTN_BOOT    GPIO_NUM_0      /* strapping pin: long-press = PTT */
#define PIN_BTN_VOL_UP  GPIO_NUM_40
#define PIN_BTN_VOL_DN  GPIO_NUM_39

/* ── Power ───────────────────────────────────────────────────────── */
#define PIN_CHARGE_DET  GPIO_NUM_38     /* 1 = charging */
#define PIN_BATT_ADC    GPIO_NUM_17     /* ADC2 channel 6 */
#define PIN_PWR_HOLD    GPIO_NUM_21     /* deep-sleep hold (not used yet) */
