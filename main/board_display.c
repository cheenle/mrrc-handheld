#include "board.h"
#include "board_pins.h"

#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"

static const char *TAG = "disp";

#define CHUNK_ROWS 16

static spi_device_handle_t s_spi;
static uint16_t           *s_fb;      /* 240x240 RGB565, PSRAM */
/* Two ping-pong line buffers.  The SPI colour write is DMA-free (polling),
 * but we still double-buffer so the next chunk's pack never races the
 * controller's own line latch. */
static uint16_t           *s_dma[2];
static int                 s_dma_cur;

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

/* ── ST7789 low-level (polling SPI, no DMA) ──────────────────────── */
/* The stock esp_lcd_panel_io_spi driver queues the colour DMA and waits on
 * portMAX_DELAY for completion.  On this board the GDMA eventually wedges
 * after minutes of streaming and the UI task blocks forever in
 * spi_device_get_trans_result(); the only robust fix is to stop using DMA for
 * the display entirely.  Polling the SPI FIFO cannot stall: the master always
 * finishes clocking out the bits it was given. */

static void lcd_cmd(uint8_t cmd)
{
    gpio_set_level(PIN_LCD_DC, 0);
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    spi_device_polling_transmit(s_spi, &t);
}

static void lcd_data(const uint8_t *data, size_t len)
{
    gpio_set_level(PIN_LCD_DC, 1);
    spi_transaction_t t = { .length = len * 8, .tx_buffer = data };
    spi_device_polling_transmit(s_spi, &t);
}

static void lcd_flush_region(int x0, int y0, int w, int h, const uint16_t *px)
{
    uint8_t b[4];
    lcd_cmd(0x2A);                       /* CASET */
    b[0] = (x0 >> 8) & 0xFF; b[1] = x0 & 0xFF;
    b[2] = ((x0 + w - 1) >> 8) & 0xFF; b[3] = (x0 + w - 1) & 0xFF;
    lcd_data(b, 4);
    lcd_cmd(0x2B);                       /* RASET */
    b[0] = (y0 >> 8) & 0xFF; b[1] = y0 & 0xFF;
    b[2] = ((y0 + h - 1) >> 8) & 0xFF; b[3] = (y0 + h - 1) & 0xFF;
    lcd_data(b, 4);
    lcd_cmd(0x2C);                       /* RAMWR */
    lcd_data((const uint8_t *)px, (size_t)w * h * 2);
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
    if (!s_spi || !s_fb) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_H_RES) w = LCD_H_RES - x;
    if (y + h > LCD_V_RES) h = LCD_V_RES - y;
    if (w <= 0 || h <= 0) return;

    for (int row = 0; row < h; row += CHUNK_ROWS) {
        int rows = (h - row > CHUNK_ROWS) ? CHUNK_ROWS : (h - row);
        uint16_t *buf = s_dma[s_dma_cur & 1];
        s_dma_cur++;
        for (int r = 0; r < rows; r++) {
            memcpy(buf + r * w,
                   s_fb + (y + row + r) * LCD_H_RES + x,
                   (size_t)w * sizeof(uint16_t));
        }
        lcd_flush_region(x, y + row, w, rows, buf);
    }
}

esp_err_t board_display_init(void)
{
    backlight_pwm_init();
    board_display_backlight(80);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LCD_DC) | (1ULL << PIN_LCD_RST),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    /* Hardware reset: RST is active-low on this flex. */
    gpio_set_level(PIN_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    spi_bus_config_t buscfg = {
        .mosi_io_num     = PIN_LCD_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = PIN_LCD_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_H_RES * CHUNK_ROWS * (int)sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 24 * 1000 * 1000,   /* 40 MHz corrupted rows on this flex */
        .mode           = 0,
        .spics_io_num   = PIN_LCD_CS,
        .queue_size     = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi));

    /* ST7789 init, byte-for-byte what esp_lcd_panel_st7789 did. */
    lcd_cmd(0x01);                       /* SWRESET */
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x11);                       /* SLPOUT */
    vTaskDelay(pdMS_TO_TICKS(100));
    lcd_cmd(0x36);                       /* MADCTL: RGB, no mirror/swap */
    { uint8_t d = 0x00; lcd_data(&d, 1); }
    lcd_cmd(0x3A);                       /* COLMOD: 16-bit */
    { uint8_t d = 0x55; lcd_data(&d, 1); }
    lcd_cmd(0xB0);                       /* RAMCTRL: big endian */
    { uint8_t d[2] = { 0x00, 0xF0 }; lcd_data(d, 2); }
#if LCD_INVERT
    lcd_cmd(0x21);                       /* INVON */
#endif
    /* VSCRSADD (0x37) takes TWO parameter bytes.  Sending one desynchronises
     * the command stream and every later command lands shifted. */
    { uint8_t d[2] = { 0x00, 0x00 }; lcd_cmd(0x37); lcd_data(d, 2); }
    lcd_cmd(0x29);                       /* DISPON */

    size_t fb_bytes = LCD_H_RES * LCD_V_RES * sizeof(uint16_t);
    size_t dma_bytes = LCD_H_RES * CHUNK_ROWS * sizeof(uint16_t);
    s_fb = board_psram_alloc(fb_bytes);
    s_dma[0] = heap_caps_malloc(dma_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    s_dma[1] = heap_caps_malloc(dma_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_fb || !s_dma[0] || !s_dma[1]) {
        ESP_LOGE(TAG, "framebuffer allocation failed (fb=%p dma0=%p dma1=%p)",
                 s_fb, s_dma[0], s_dma[1]);
        return ESP_ERR_NO_MEM;
    }
    memset(s_fb, 0, fb_bytes);
    ESP_LOGI(TAG, "ST7789 240x240 up (polling SPI), free heap=%u, min=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size());
    return ESP_OK;
}
