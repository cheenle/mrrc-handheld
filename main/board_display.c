#include "board.h"
#include "board_pins.h"

#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

static const char *TAG = "disp";

#define CHUNK_ROWS 16

static esp_lcd_panel_handle_t    s_panel;
static esp_lcd_panel_io_handle_t s_io;
static uint16_t                 *s_fb;      /* 240x240 RGB565, PSRAM */
static uint16_t                 *s_dma;     /* CHUNK_ROWS lines, internal RAM */

void *board_psram_alloc(size_t bytes)
{
    void *p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) {
        ESP_LOGW(TAG, "PSRAM alloc %u failed, falling back to internal",
                 (unsigned)bytes);
        p = heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
    }
    return p;
}

uint16_t *board_display_fb(void)
{
    return s_fb;
}

static void backlight_pwm_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    ledc_channel_config_t c = {
        .gpio_num   = PIN_LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 1023,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

void board_display_backlight(int percent)
{
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                  (uint32_t)(1023 * percent / 100));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void board_display_flush(int x, int y, int w, int h)
{
    if (!s_panel || !s_fb) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_H_RES) w = LCD_H_RES - x;
    if (y + h > LCD_V_RES) h = LCD_V_RES - y;
    if (w <= 0 || h <= 0) return;

    for (int row = 0; row < h; row += CHUNK_ROWS) {
        int rows = (h - row > CHUNK_ROWS) ? CHUNK_ROWS : (h - row);
        /* Pack the stripe (the fb is full-width, so copy line by line). */
        for (int r = 0; r < rows; r++) {
            memcpy(s_dma + r * w,
                   s_fb + (y + row + r) * LCD_H_RES + x,
                   (size_t)w * sizeof(uint16_t));
        }
        esp_lcd_panel_draw_bitmap(s_panel, x, y + row, x + w, y + row + rows,
                                  s_dma);
    }
}

esp_err_t board_display_init(void)
{
    backlight_pwm_init();
    board_display_backlight(80);

    spi_bus_config_t buscfg = {
        .mosi_io_num     = PIN_LCD_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = PIN_LCD_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_H_RES * CHUNK_ROWS * (int)sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num       = PIN_LCD_DC,
        .cs_gpio_num       = PIN_LCD_CS,
        .pclk_hz           = 24 * 1000 * 1000,   /* 40 MHz corrupted rows on this flex */
        .lcd_cmd_bits      = 8,
        .lcd_param_bits    = 8,
        .spi_mode          = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST,
                                             &io_cfg, &s_io));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(s_io, &panel_cfg, &s_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
#if LCD_INVERT
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
#endif
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, false, false));
    /* VSCRSADD (0x37) takes TWO parameter bytes.  Sending one desynchronises
     * the command stream and every later command lands shifted — that is what
     * turned ordinary ghosting into a garbled screen. */
    uint8_t scroll[2] = { 0x00, 0x00 };
    esp_lcd_panel_io_tx_param(s_io, 0x37, scroll, 2);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    size_t fb_bytes = LCD_H_RES * LCD_V_RES * sizeof(uint16_t);
    s_fb = board_psram_alloc(fb_bytes);
    s_dma = heap_caps_malloc(LCD_H_RES * CHUNK_ROWS * sizeof(uint16_t),
                             MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_fb || !s_dma) {
        ESP_LOGE(TAG, "framebuffer allocation failed (fb=%p dma=%p)", s_fb, s_dma);
        return ESP_ERR_NO_MEM;
    }
    memset(s_fb, 0, fb_bytes);
    ESP_LOGI(TAG, "ST7789 240x240 up, free heap=%u, min=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size());
    return ESP_OK;
}
