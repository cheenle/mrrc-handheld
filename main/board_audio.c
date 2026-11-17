/*
 * Audio — replication of the factory (xiaozhi v1.7.6) configuration,
 * read straight out of the OEM firmware's GPIO-matrix registers over JTAG:
 *
 *   I2S0 : BCLK gpio15  WS gpio16  SD_out gpio7      (amplifier)
 *   I2S1 : BCK_out gpio5  WS_out gpio4  SD_in <- gpio6  (microphone)
 *   gpio21 : held high through the RTC-IO mux (power hold)
 *
 * The two buses are independent: the amp runs at 48 kHz (Opus decode rate),
 * the microphone at 16 kHz, which is what the stock firmware uses.
 *
 * The microphone path is exactly the factory one: one 32-bit I2S word per
 * sample, arithmetic shift right by 12, nothing else.
 */
#include "board.h"
#include "board_pins.h"

#include <string.h>

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/rtc_io.h"

static const char *TAG = "audio";

#define SPK_RATE      48000
#define MIC_RATE      16000
#define MIC_FRAME     320            /* 20 ms @ 16 kHz */
#define MIC_RING      8
#define I2S_BITS      I2S_DATA_BIT_WIDTH_32BIT

static i2s_chan_handle_t s_tx;       /* I2S0 speaker */
static i2s_chan_handle_t s_rx;       /* I2S1 microphone */

/* The ring + scratch are plain app buffers (the I2S DMA buffers live in the
 * driver's own internal-RAM descriptors), so they can live in PSRAM and stop
 * eating the scarcest resource.  aligned(4) is required by .ext_ram.bss. */
static EXT_RAM_BSS_ATTR int16_t  s_mic_ring[MIC_RING][MIC_FRAME] __attribute__((aligned(4)));
static volatile uint32_t s_mic_w, s_mic_r;
static volatile int32_t  s_peak_in;
static EXT_RAM_BSS_ATTR int32_t  s_spk_scratch[1152] __attribute__((aligned(4)));

/* ── speaker ─────────────────────────────────────────────────────── */
size_t board_spk_write(const int16_t *src, size_t samples)
{
    if (!s_tx || !src) return 0;
    if (samples > sizeof(s_spk_scratch) / sizeof(s_spk_scratch[0])) {
        samples = sizeof(s_spk_scratch) / sizeof(s_spk_scratch[0]);
    }
    for (size_t i = 0; i < samples; i++) {
        s_spk_scratch[i] = (int32_t)src[i] << 16;
    }
    size_t written = 0;
    if (i2s_channel_write(s_tx, s_spk_scratch, samples * sizeof(int32_t), &written,
                          1000) != ESP_OK) {
        return 0;
    }
    return written / sizeof(int32_t);
}

/* ── microphone ring ─────────────────────────────────────────────── */
size_t board_mic_take(int16_t *dst, size_t samples)
{
    if (!dst) return 0;
    if (samples > MIC_FRAME) samples = MIC_FRAME;
    if (s_mic_w == s_mic_r) {
        memset(dst, 0, samples * sizeof(int16_t));
        return 0;
    }
    memcpy(dst, s_mic_ring[s_mic_r % MIC_RING], samples * sizeof(int16_t));
    s_mic_r++;
    return samples;
}

size_t board_mic_read(int16_t *dst, size_t samples)
{
    return board_mic_take(dst, samples);
}

int32_t board_mic_raw_peak(void)
{
    int32_t p = s_peak_in;
    s_peak_in = 0;
    return p;
}

/* ── pump ────────────────────────────────────────────────────────── */
static void mic_pump_task(void *arg)
{
    /* 2 words per sample (both slots are transferred) */
    int32_t *raw = heap_caps_malloc(MIC_FRAME * 2 * sizeof(int32_t),
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!raw) {
        ESP_LOGE(TAG, "mic pump: no memory");
        vTaskDelete(NULL);
        return;
    }

    int32_t peak = 0;

    while (1) {
        if (!s_rx) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

        /* Factory behaviour (NoAudioCodec::Read): ONE 32-bit word = one
         * sample.  My earlier "two words per sample" reading was an artefact
         * of several tasks reading the same handle; de-interleaving halved the
         * rate and aliased the voice band away. */
        size_t got = 0;
        if (i2s_channel_read(s_rx, raw, MIC_FRAME * sizeof(int32_t), &got,
                             1000) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        size_t frames = got / sizeof(int32_t);
        if (!frames) continue;

        int16_t *dst = s_mic_ring[s_mic_w % MIC_RING];
        for (size_t i = 0; i < frames && i < MIC_FRAME; i++) {
            /* Factory conversion: one 32-bit word == one sample, >>12.
             * No gain, no filter, no AGC — the stock firmware does the same
             * and relies on its noise suppressor, not on level tricks. */
            int32_t v = raw[i] >> 12;
            int32_t a = v < 0 ? -v : v;
            if (a > peak) peak = a;
            if (v > 32767) v = 32767; else if (v < -32767) v = -32767;
            dst[i] = (int16_t)v;
        }
        s_peak_in = peak;
        peak = 0;

        if (s_mic_w - s_mic_r >= MIC_RING) s_mic_r++;
        s_mic_w++;
    }
}

esp_err_t board_audio_init(void)
{
    /* Power hold exactly like the factory firmware: RTC-IO mux, level 1.
     * (The digital GPIO registers read back as 0 because the pad belongs to
     * the RTC IO mux, which is also why my earlier "it did not work" check was
     * wrong.) */
    rtc_gpio_hold_dis(PIN_PWR_HOLD);
    rtc_gpio_init(PIN_PWR_HOLD);
    rtc_gpio_set_direction(PIN_PWR_HOLD, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(PIN_PWR_HOLD, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    /* ── amplifier: I2S0, TX only, 48 kHz ── */
    i2s_chan_config_t tx_cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    tx_cc.auto_clear_after_cb = true;
    tx_cc.dma_desc_num  = 12;      /* ~120 ms of DMA buffering */
    tx_cc.dma_frame_num = 480;
    ESP_ERROR_CHECK(i2s_new_channel(&tx_cc, &s_tx, NULL));

    i2s_std_config_t tx_cfg = {
        .clk_cfg  = { .sample_rate_hz = SPK_RATE, .clk_src = I2S_CLK_SRC_DEFAULT,
                      .mclk_multiple = I2S_MCLK_MULTIPLE_256 },
        .slot_cfg = { .data_bit_width = I2S_BITS,
                      .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                      .slot_mode = I2S_SLOT_MODE_MONO,
                      .slot_mask = I2S_STD_SLOT_LEFT,
                      .ws_width = I2S_BITS, .ws_pol = false, .bit_shift = true,
                      .left_align = true, .big_endian = false,
                      .bit_order_lsb = false },
        .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = PIN_SPK_BCLK,
                      .ws = PIN_SPK_LRCK, .dout = PIN_SPK_DOUT,
                      .din = I2S_GPIO_UNUSED },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx, &tx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx));

    /* ── microphone: I2S1, RX only, 16 kHz, its own clock pair 5/4 ── */
    i2s_chan_config_t rx_cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    rx_cc.dma_desc_num  = 6;
    rx_cc.dma_frame_num = 240;
    ESP_ERROR_CHECK(i2s_new_channel(&rx_cc, NULL, &s_rx));

    i2s_std_config_t rx_cfg = {
        .clk_cfg  = { .sample_rate_hz = MIC_RATE, .clk_src = I2S_CLK_SRC_DEFAULT,
                      .mclk_multiple = I2S_MCLK_MULTIPLE_256 },
        .slot_cfg = { .data_bit_width = I2S_BITS,
                      .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                      .slot_mode = I2S_SLOT_MODE_MONO,
                      .slot_mask = I2S_STD_SLOT_LEFT,
                      .ws_width = I2S_BITS, .ws_pol = false, .bit_shift = true,
                      .left_align = true, .big_endian = false,
                      .bit_order_lsb = false },
        .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = PIN_MIC_SCK,
                      .ws = PIN_MIC_WS, .dout = I2S_GPIO_UNUSED,
                      .din = PIN_MIC_DIN },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx, &rx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx));

    xTaskCreatePinnedToCore(mic_pump_task, "mic", 4096, NULL, 5, NULL, 1);
    ESP_LOGI(TAG, "amp I2S0 %d Hz (bclk%d ws%d dout%d), mic I2S1 %d Hz "
             "(bclk%d ws%d din%d)", SPK_RATE, PIN_SPK_BCLK, PIN_SPK_LRCK,
             PIN_SPK_DOUT, MIC_RATE, PIN_MIC_SCK, PIN_MIC_WS, PIN_MIC_DIN);
    return ESP_OK;
}


