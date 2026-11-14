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
 * Mic conditioning (HP 300 Hz -> LP 3 kHz -> AGC -> limiter) lives in the pump
 * because this MEMS mic has a large DC offset and very low AC sensitivity.
 */
#include "board.h"
#include "board_pins.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/rtc_io.h"

static const char *TAG = "audio";

static void acoustic_selftest_task(void *arg);

#define SPK_RATE      48000
#define MIC_RATE      16000
#define MIC_FRAME     320            /* 20 ms @ 16 kHz */
#define MIC_RING      8
#define I2S_BITS      I2S_DATA_BIT_WIDTH_32BIT

static i2s_chan_handle_t s_tx;       /* I2S0 speaker */
static i2s_chan_handle_t s_rx;       /* I2S1 microphone */

static int16_t           s_mic_ring[MIC_RING][MIC_FRAME];
static volatile uint32_t s_mic_w, s_mic_r;
static volatile int32_t  s_peak_in;
static volatile bool     s_dump_req;
volatile bool g_mic_loopback = false;   /* DEV: bypass Opus, hear the raw capture */
static int32_t           s_spk_scratch[1152];

/* ── mic conditioning @16 kHz ──────────────────────────────────────
 * Measured on this unit: the mic has a DC offset of ~11 % of full scale and a
 * huge 1-7 Hz drift, while speech only contributes ~2 % AC.  A one-pole
 * high-pass (6 dB/oct) cannot remove that drift, so use real biquads:
 *   2nd-order Butterworth HP @350 Hz -> 2nd-order LP @3 kHz -> AGC -> limiter
 * (RBJ coefficients, fs = 16000)
 */
typedef struct { float b0, b1, b2, a1, a2, x1, x2, y1, y2; } biquad_t;

static float bq(biquad_t *q, float x)
{
    float y = q->b0 * x + q->b1 * q->x1 + q->b2 * q->x2
            - q->a1 * q->y1 - q->a2 * q->y2;
    q->x2 = q->x1; q->x1 = x;
    q->y2 = q->y1; q->y1 = y;
    return y;
}

static biquad_t s_hp = { 0.90731f, -1.81462f, 0.90731f, -1.80603f, 0.82327f,
                         0, 0, 0, 0 };                 /* HP 350 Hz, Q=0.707 */
static biquad_t s_lp = { 0.18670f,  0.37340f, 0.18670f, -0.46294f, 0.20977f,
                         0, 0, 0, 0 };                 /* LP 3 kHz,  Q=0.707 */

static float agc_env;
static float agc_gain = 1.0f;

#define AGC_TARGET      0.12f        /* 12 % of full scale */
#define AGC_MAX_GAIN    64.0f        /* +36 dB */
#define AGC_GATE        0.004f       /* ~130 counts; below this hold gain */

static int16_t mic_condition(int32_t x_in)
{
    float x = (float)x_in;
    float y = bq(&s_lp, bq(&s_hp, x));

    float a = y < 0 ? -y : y;
    /* envelope: 5 ms attack, 300 ms release at 16 kHz */
    agc_env += (a - agc_env) * ((a > agc_env) ? 0.0125f : 0.000208f);

    if (agc_env > AGC_GATE * 32768.0f) {
        float want = (AGC_TARGET * 32768.0f) / agc_env;
        if (want > AGC_MAX_GAIN) want = AGC_MAX_GAIN;
        if (want < 1.0f) want = 1.0f;
        agc_gain += (want - agc_gain) * 0.0156f;         /* ~80 ms */
    }

    float out = y * agc_gain;
    /* soft limit above 0.7 FS */
    const float knee = 23000.0f;
    if (out > knee)       out = knee + (out - knee) * 8700.0f / (out - knee + 9000.0f);
    else if (out < -knee) out = -(knee + (-out - knee) * 8700.0f / (-out - knee + 9000.0f));
    if (out > 31744.0f) out = 31744.0f;
    if (out < -31744.0f) out = -31744.0f;
    return (int16_t)out;
}

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

void board_mic_dump_once(void)
{
    s_dump_req = true;
}

/* ── statistics ──────────────────────────────────────────────────── */
static int64_t st_sum_raw, st_sum_ssb, st_n, st_frames;
static int32_t st_pk_raw, st_pk_ssb;
static int64_t st_last_log;

static void mic_stats_tick(void)
{
    int64_t now = esp_timer_get_time() / 1000;
    if (now - st_last_log < 2000) return;
    st_last_log = now;

    int rms_raw = st_n ? (int)sqrt((double)st_sum_raw / st_n) : 0;
    int rms_ssb = st_n ? (int)sqrt((double)st_sum_ssb / st_n) : 0;
    ESP_LOGI(TAG, "mic: frames=%ld/s rms=%d ssb=%d pk_raw=%ld pk_ssb=%ld",
             (long)st_frames, rms_raw, rms_ssb, (long)st_pk_raw, (long)st_pk_ssb);
    st_sum_raw = st_sum_ssb = st_n = 0;
    st_pk_raw = st_pk_ssb = 0;
    st_frames = 0;
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
    static int16_t acc[16000];
    static int acc_n;
    static int32_t acc32[4000];
    static int acc32_n;

    while (1) {
        if (!s_rx) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

        /* The RX DMA hands over two 32-bit words per sample in mono mode
         * (both slots are transferred).  Read 2N words and keep one of each
         * pair, otherwise 10 ms of audio gets encoded as 20 ms — an octave of
         * pitch shift plus aliasing. */
        size_t got = 0;
        if (i2s_channel_read(s_rx, raw, MIC_FRAME * 2 * sizeof(int32_t), &got,
                             1000) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        size_t words = got / sizeof(int32_t);
        size_t frames = words / 2;
        st_frames += (int64_t)frames;
        if (!frames) continue;

        int16_t *dst = s_mic_ring[s_mic_w % MIC_RING];
        static int16_t raw_keep[MIC_FRAME];
        static int32_t raw32[MIC_FRAME];
        for (size_t i = 0; i < frames && i < MIC_FRAME; i++) raw32[i] = raw[i * 2];
        for (size_t i = 0; i < frames && i < MIC_FRAME; i++) {
            raw[i] = raw[i * 2];          /* de-interleave: keep the left slot */
            /* The mic presents ~20 significant bits left-justified in the
             * 32-bit slot; the factory firmware converts with >>12, and the
             * conditioning chain below expects that int16 scale. */
            int32_t x = raw[i] >> 12;
            if (x > 32767) x = 32767;
            if (x < -32767) x = -32767;
            int32_t a = x < 0 ? -x : x;
            if (a > peak) peak = a;
            st_sum_raw += (int64_t)a * a;
            if (a > st_pk_raw) st_pk_raw = a;

            int16_t y = mic_condition(x);
            raw_keep[i] = (int16_t)x;
            st_sum_ssb += (int64_t)y * y;
            int32_t ay = y < 0 ? -y : y;
            if (ay > st_pk_ssb) st_pk_ssb = ay;
            st_n++;
            dst[i] = y;

            /* keep the raw sample for the periodic dump */
            raw[i] = x;
        }
        s_peak_in = peak;
        peak = 0;

        if (g_mic_loopback) {
            static int16_t up[MIC_FRAME * 3];
            for (int i = 0; i < MIC_FRAME; i++) {
                up[i * 3] = up[i * 3 + 1] = up[i * 3 + 2] = dst[i];
            }
            board_spk_write(up, MIC_FRAME * 3);
        }
        mic_stats_tick();

        if (s_mic_w - s_mic_r >= MIC_RING) s_mic_r++;
        s_mic_w++;

        /* on-demand dump (PTT key-up), never periodic: printing 16k hex values
         * blocks the pump and drops frames */
        if (s_dump_req) {
            for (size_t i = 0; i < frames && acc32_n < 4000; i++)
                acc32[acc32_n++] = raw32[i];
        }
        if (acc32_n >= 4000) {
            acc32_n = 0;
            s_dump_req = false;
            printf("\n===MIC32-4000===\n");
            for (int i = 0; i < 4000; i++) {
                printf("%08lx%s", (unsigned long)(uint32_t)acc32[i],
                       (i % 20 == 19) ? "\n" : " ");
            }
            printf("===MIC32-END===\n");
            fflush(stdout);
        }
    }
}

/* ── init ────────────────────────────────────────────────────────── */
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

    xTaskCreatePinnedToCore(mic_pump_task, "mic", 6144, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(acoustic_selftest_task, "acoustic", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "amp I2S0 %d Hz (bclk%d ws%d dout%d), mic I2S1 %d Hz "
             "(bclk%d ws%d din%d)", SPK_RATE, PIN_SPK_BCLK, PIN_SPK_LRCK,
             PIN_SPK_DOUT, MIC_RATE, PIN_MIC_SCK, PIN_MIC_WS, PIN_MIC_DIN);
    return ESP_OK;
}


/* Acoustic self-test: play a 1 kHz tone on the speaker, then dump the raw mic
 * words.  If the mic is picking up sound at all, the dump will contain a 1 kHz
 * peak; if it does not, the microphone is not hearing anything. */
static void acoustic_selftest_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(6000));

    static int16_t tone[4800];
    for (int i = 0; i < 4800; i++) {
        tone[i] = (int16_t)(14000 * sinf(2.0f * 3.14159265f * 1000.0f * i / 48000.0f));
    }
    for (int r = 0; r < 3; r++) board_spk_write(tone, 4800);   /* 0.3 s of tone */
    vTaskDelay(pdMS_TO_TICKS(50));

    s_dump_req = true;                 /* capture starts now */
    vTaskDelay(pdMS_TO_TICKS(600));
    ESP_LOGW(TAG, "acoustic self-test: tone played, raw mic dumped");
    vTaskDelete(NULL);
}
