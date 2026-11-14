#include "board.h"
#include "board_pins.h"

#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include <math.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"

static const char *TAG = "audio";

/* Speaker always runs at the Opus decode rate: 48 kHz mono.
 * The mic runs at CONFIG_MRRC_MIC_RATE (16 kHz by default) — Opus is clocked
 * at 48 kHz internally, so a 16 kHz encode still decodes to 48 kHz on the
 * server.  Raw-PCM uplink mode forces the mic to 48 kHz. */
#define SPK_RATE 48000

#if !defined(CONFIG_MRRC_MIC_RATE)
#define CONFIG_MRRC_MIC_RATE 16000
#endif

#ifdef CONFIG_MRRC_AUDIO_PCM
#define MIC_RATE 48000
#else
#define MIC_RATE CONFIG_MRRC_MIC_RATE
#endif

/* 32-bit slots, mono, left — byte-for-byte what the stock xiaozhi firmware
 * uses for this board's amp.  Changing the slot width breaks the amp. */
#define I2S_BITS I2S_DATA_BIT_WIDTH_32BIT

static i2s_chan_handle_t s_tx;
static i2s_chan_handle_t s_rx;

static int32_t s_scratch[1152];   /* 60 ms @ 48k, guarded by single caller */

size_t board_spk_write(const int16_t *src, size_t samples)
{
    if (!s_tx || !src) return 0;
    if (samples > sizeof(s_scratch) / sizeof(s_scratch[0])) {
        samples = sizeof(s_scratch) / sizeof(s_scratch[0]);
    }
    for (size_t i = 0; i < samples; i++) {
        s_scratch[i] = (int32_t)src[i] << 16;
    }
    size_t written = 0;
    esp_err_t err = i2s_channel_write(s_tx, s_scratch,
                                      samples * sizeof(int32_t), &written, 1000);
    if (err != ESP_OK) return 0;
    return written / sizeof(int32_t);
}

/* Raw I2S words of the last mic frame, for level/alignment diagnostics. */
int32_t g_mic_raw_min, g_mic_raw_max;
int32_t g_mic_raw_head[8];
int32_t g_mic_l_min, g_mic_l_max, g_mic_r_min, g_mic_r_max;
int32_t g_mic_r_head[4];
int64_t g_mic_raw_sum;
int     g_mic_raw_n;

size_t board_mic_read(int16_t *dst, size_t samples)
{
    if (!s_rx || !dst) return 0;
    if (samples > sizeof(s_scratch) / sizeof(s_scratch[0])) {
        samples = sizeof(s_scratch) / sizeof(s_scratch[0]);
    }
    size_t got = 0;
    esp_err_t err = i2s_channel_read(s_rx, s_scratch,
                                     samples * sizeof(int32_t), &got, 1000);
    if (err != ESP_OK) return 0;
    size_t frames = got / sizeof(int32_t) / 2;   /* stereo probe */
    int32_t lmin = INT32_MAX, lmax = INT32_MIN, rmin = INT32_MAX, rmax = INT32_MIN;
    int64_t rsum = 0;
    for (size_t i = 0; i < frames; i++) {
        int32_t l = s_scratch[i * 2], r = s_scratch[i * 2 + 1];
        if (l < lmin) lmin = l;
        if (l > lmax) lmax = l;
        if (r < rmin) rmin = r;
        if (r > rmax) rmax = r;
        rsum += l;
    }
    g_mic_l_min = lmin; g_mic_l_max = lmax;
    g_mic_r_min = rmin; g_mic_r_max = rmax;
    g_mic_raw_min = lmin; g_mic_raw_max = lmax;
    g_mic_raw_sum = rsum;
    g_mic_raw_n = (int)frames;
    for (int i = 0; i < 4 && i < (int)frames; i++) {
        g_mic_raw_head[i] = s_scratch[i * 2];
        g_mic_r_head[i]  = s_scratch[i * 2 + 1];
    }
    size_t n = frames;
    static int32_t dc;
    for (size_t i = 0; i < n; i++) {
        int32_t raw = s_scratch[i * 2];
        dc += (raw - dc) >> 10;          /* ~1 s high-pass */
        s_scratch[i] = raw - dc;
    }
    for (size_t i = 0; i < n; i++) {
        /* The stock xiaozhi firmware for this exact board uses >>12, not >>16:
         * this MEMS mic presents ~20 significant bits left-justified in the
         * 32-bit slot.  >>16 threw away 24 dB and SSB came out with no
         * modulation at all. */
        int32_t v = s_scratch[i] >> 12;
        if (v > INT16_MAX) v = INT16_MAX;
        if (v < -INT16_MAX) v = -INT16_MAX;
        dst[i] = (int16_t)v;
    }
    return n;
}

/* ── Microphone pump ────────────────────────────────────────────────
 * The stock firmware reads the mic continuously (its AFE pipeline never
 * stops).  We used to read only while keyed, which left the I2S RX DMA ring
 * untouched for minutes at a time — the channel desyncs and hands back stale,
 * DC-dominated data.  A permanent 20 ms pump keeps the stream healthy and
 * gives the TX path a live buffer. */
#define MIC_FRAME_IN  960        /* 20 ms @ 48 kHz, from the shared bus */
#define MIC_FRAME     320        /* 20 ms @ 16 kHz after 3:1 decimation */
#define MIC_RING  8

volatile bool g_mic_dump_req;
static int16_t  s_mic_ring[MIC_RING][MIC_FRAME];
static volatile uint32_t s_mic_w, s_mic_r;
static volatile int32_t s_mic_peak_l, s_mic_peak_r;
static volatile bool s_mic_log_pending;

size_t board_mic_take(int16_t *dst, size_t samples)
{
    if (samples > MIC_FRAME) samples = MIC_FRAME;
    if (s_mic_w == s_mic_r) return 0;          /* underrun */
    uint32_t i = s_mic_r % MIC_RING;
    for (size_t k = 0; k < samples; k++) dst[k] = s_mic_ring[i][k];
    s_mic_r++;
    return samples;
}

bool board_mic_take_probe(int32_t *lmin, int32_t *lmax, int32_t *rmin, int32_t *rmax)
{
    if (!s_mic_log_pending) return false;
    s_mic_log_pending = false;
    *lmin = 0; *lmax = s_mic_peak_l; *rmin = 0; *rmax = s_mic_peak_r;
    return true;
}

/* ── Mic conditioning ─────────────────────────────────────────────
 * Measured on this board: the MEMS mic has a big DC offset and ~0.01 % AC
 * sensitivity, and its noise floor is broadband.  Feeding it raw gave the
 * radio nothing to modulate (99.98 % of the energy was under 300 Hz); feeding
 * it with bare AGC put 66 % of the energy *above* 2.7 kHz (amplified hiss) and
 * clipped 70 % of the samples.  So: band-limit to the SSB passband first,
 * then AGC, then a soft limiter.
 *
 *   HP 300 Hz  ->  LP 3 kHz  ->  AGC (+24 dB max, gated)  ->  soft clip
 */
static int32_t hp_in, hp_out;         /* one-pole HP @300 Hz  */
static int32_t lp_out;                /* one-pole LP @3 kHz   */
static int32_t agc_env;
static int32_t agc_gain_q8 = 256;

#define HP_A_Q25600  24632            /* 0.9622 * 25600, 300 Hz @48k */
#define LP_A_Q256       72            /* 0.2820 * 256, 3 kHz @48k */
#define AGC_TARGET     6500           /* ~20 % of full scale */
#define AGC_MAX_GAIN_Q8 (256 * 16)    /* +24 dB */
#define AGC_GATE       120            /* below this: hold gain, don't pump hiss */

static int16_t mic_condition(int32_t x)
{
    /* high-pass: y = a*(y1 + x - x1) */
    int32_t y = (HP_A_Q25600 * (hp_out + x - hp_in)) / 25600;
    hp_in = x;
    hp_out = y;

    /* low-pass: y += a*(x - y) */
    lp_out += ((y - lp_out) * LP_A_Q256) / 256;
    y = lp_out;

    int32_t a = y < 0 ? -y : y;
    if (a > agc_env) agc_env += (a - agc_env) / 240;       /* 5 ms attack @48k */
    else             agc_env -= (agc_env - a) / 14400;     /* 300 ms release */

    if (agc_env > AGC_GATE) {
        int32_t want = AGC_TARGET * 256 / agc_env;
        if (want > AGC_MAX_GAIN_Q8) want = AGC_MAX_GAIN_Q8;
        if (want < 256) want = 256;
        agc_gain_q8 += (want - agc_gain_q8) / 64;
    }

    int32_t out = (y * agc_gain_q8) / 256;

    /* soft limiter: linear to 0.7 FS, then compresses to 0.97 FS */
    const int32_t knee = 23000;
    if (out > knee || out < -knee) {
        int32_t sign = out < 0 ? -1 : 1;
        int32_t over = (out < 0 ? -out : out) - knee;
        out = sign * (knee + over * 8700 / (over + 9000));
    }
    if (out > 31744) out = 31744;
    if (out < -31744) out = -31744;
    return (int16_t)out;
}

static void mic_pump_task(void *arg)
{
    int32_t *raw = heap_caps_malloc(MIC_FRAME_IN * sizeof(int32_t),
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!raw) { vTaskDelete(NULL); return; }
    int64_t last_probe = 0;
    int32_t pk_l = 0, pk_r = 0;
    static int32_t dc;
    static int16_t prev[MIC_FRAME];

    while (1) {
        size_t got = 0;
        if (i2s_channel_read(s_rx, raw, MIC_FRAME_IN * sizeof(int32_t), &got,
                             1000) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        size_t frames = got / sizeof(int32_t);
        if (!frames) continue;

        int16_t *dst = s_mic_ring[s_mic_w % MIC_RING];
        for (size_t i = 0; i < frames && i < MIC_FRAME_IN; i++) {
            int32_t l = raw[i];
            int32_t a = l < 0 ? -l : l;
            if (a > pk_l) pk_l = a;
            raw[i] = mic_condition(l);
        }
        /* 48 kHz -> 16 kHz: average of 3 (cheap anti-alias, fine for voice) */
        for (int i = 0; i < MIC_FRAME; i++) {
            size_t b = (size_t)i * 3;
            int32_t s3 = 0;
            for (size_t k = 0; k < 3 && b + k < frames; k++) s3 += raw[b + k];
            dst[i] = (int16_t)(s3 / 3);
        }
        for (int i = 0; i < MIC_FRAME; i++) prev[i] = dst[i];
        if (s_mic_w - s_mic_r >= MIC_RING) s_mic_r++;   /* drop oldest */
        s_mic_w++;

        /* One-shot raw dump: 16000 int16 samples (1 s @16 k) as hex, so the
         * actual microphone signal can be inspected on the host. */
        {
            static bool dumped;
            static int16_t acc_buf[16000];
            static int acc_n;
            if (g_mic_dump_req && !dumped) {
                for (size_t i = 0; i < frames && acc_n < 16000; i++) {
                    /* raw, pre-DSP: what the microphone actually delivers */
                    int32_t v = raw[i * 2] >> 12;
                    if (v > 32767) v = 32767;
                    if (v < -32767) v = -32767;
                    acc_buf[acc_n++] = (int16_t)v;
                }
                if (acc_n >= 16000) {
                    dumped = true;
                    g_mic_dump_req = false;
                    printf("\n===MICDUMP-16000===\n");
                    for (int i = 0; i < 16000; i++) {
                        printf("%04x%s", (unsigned)(uint16_t)acc_buf[i],
                               (i % 32 == 31) ? "\n" : " ");
                    }
                    printf("===MICDUMP-END===\n");
                    fflush(stdout);
                }
            }
        }

        int64_t now = esp_timer_get_time() / 1000;
        if (now - last_probe > 5000) {
            last_probe = now;
            s_mic_peak_l = pk_l;
            s_mic_peak_r = pk_r;
            s_mic_log_pending = true;
            pk_l = pk_r = 0;
        }
    }
}

__attribute__((unused)) static esp_err_t spk_init(void);
__attribute__((unused)) static esp_err_t mic_init_legacy(void);
__attribute__((unused)) /* Brute force: which GPIO, when driven high, powers the microphone?
 * Reads the mic on the amp's clock pair while toggling candidates. */
static void mic_power_sweep(void)
{
    static const int cand[] = { 11, 12, 17, 21, 41, 42, 47, 48 };
    int32_t *buf = heap_caps_malloc(512 * sizeof(int32_t),
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buf) return;

    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    cc.dma_desc_num = 6; cc.dma_frame_num = 240;
    i2s_chan_handle_t rx = NULL;
    if (i2s_new_channel(&cc, NULL, &rx) != ESP_OK) { free(buf); return; }
    i2s_std_config_t cfg = {
        .clk_cfg = { .sample_rate_hz = 48000, .clk_src = I2S_CLK_SRC_DEFAULT,
                     .mclk_multiple = I2S_MCLK_MULTIPLE_256 },
        .slot_cfg = { .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
                      .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                      .slot_mode = I2S_SLOT_MODE_MONO, .slot_mask = I2S_STD_SLOT_LEFT,
                      .ws_width = I2S_DATA_BIT_WIDTH_32BIT, .ws_pol = false,
                      .bit_shift = true, .left_align = true, .big_endian = false,
                      .bit_order_lsb = false },
        .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = PIN_SPK_BCLK,
                      .ws = PIN_SPK_LRCK, .dout = I2S_GPIO_UNUSED,
                      .din = PIN_MIC_DIN },
    };
    if (i2s_channel_init_std_mode(rx, &cfg) != ESP_OK) { i2s_del_channel(rx); free(buf); return; }
    i2s_channel_enable(rx);

    ESP_LOGW(TAG, "--- microphone power sweep ---");
    for (size_t i = 0; i < sizeof cand / sizeof cand[0]; i++) {
        gpio_num_t g = (gpio_num_t)cand[i];
        gpio_config_t io = { .pin_bit_mask = 1ULL << g, .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE };
        gpio_config(&io);
        gpio_set_level(g, 1);
        vTaskDelay(pdMS_TO_TICKS(60));

        size_t got = 0;
        i2s_channel_read(rx, buf, 512 * sizeof(int32_t), &got, 200);
        int n = (int)(got / sizeof(int32_t));
        int32_t peak = 0; int64_t sum = 0, dsum = 0; int32_t x1 = 0, x2 = 0;
        for (int k = 0; k < n; k++) {
            int32_t x = buf[k];
            int32_t a = x < 0 ? -x : x;
            if (a > peak) peak = a;
            sum += a;
            int32_t d = x - 2 * x1 + x2; if (d < 0) d = -d;
            dsum += d; x2 = x1; x1 = x;
        }
        ESP_LOGW(TAG, "gpio%-2d high -> peak=%8ld AC/DC=%3d%% head=%08lx %08lx",
                 cand[i], (long)peak, (int)(sum ? dsum * 100 / sum : 0),
                 (unsigned long)(n > 0 ? buf[0] : 0),
                 (unsigned long)(n > 1 ? buf[1] : 0));

        gpio_set_level(g, 0);
        gpio_reset_pin(g);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGW(TAG, "--- power sweep done ---");

    i2s_channel_disable(rx);
    i2s_del_channel(rx);
    free(buf);
}

/* Which GPIO actually carries microphone data, and on which clock pair?
 * One boot-time sweep, no user interaction. */
static void mic_pin_sweep(void)
{
    static const int clocks[2][2] = { {15, 16}, {4, 5} };   /* bclk, ws */
    static const gpio_num_t dins[4] = { GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_4, GPIO_NUM_5 };
    int32_t *buf = heap_caps_malloc(512 * sizeof(int32_t),
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buf) return;

    ESP_LOGW(TAG, "--- microphone pin sweep (bclk/ws x din) ---");
    for (int c = 0; c < 2; c++) {
        for (int d = 0; d < 4; d++) {
            if (dins[d] == clocks[c][0] || dins[d] == clocks[c][1]) continue;
            i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
            cc.dma_desc_num = 6; cc.dma_frame_num = 240;
            i2s_chan_handle_t rx = NULL;
            if (i2s_new_channel(&cc, NULL, &rx) != ESP_OK) continue;
            i2s_std_config_t cfg = {
                .clk_cfg = { .sample_rate_hz = 48000, .clk_src = I2S_CLK_SRC_DEFAULT,
                             .mclk_multiple = I2S_MCLK_MULTIPLE_256 },
                .slot_cfg = { .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
                              .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                              .slot_mode = I2S_SLOT_MODE_MONO,
                              .slot_mask = I2S_STD_SLOT_LEFT,
                              .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
                              .ws_pol = false, .bit_shift = true, .left_align = true,
                              .big_endian = false, .bit_order_lsb = false },
                .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = clocks[c][0],
                              .ws = clocks[c][1], .dout = I2S_GPIO_UNUSED,
                              .din = dins[d] },
            };
            if (i2s_channel_init_std_mode(rx, &cfg) == ESP_OK &&
                i2s_channel_enable(rx) == ESP_OK) {
                size_t got = 0;
                i2s_channel_read(rx, buf, 512 * sizeof(int32_t), &got, 250);
                int n = (int)(got / sizeof(int32_t));
                int32_t peak = 0; int64_t sum = 0, dsum = 0;
                int32_t x1 = 0, x2 = 0;
                for (int i = 0; i < n; i++) {
                    int32_t x = buf[i];
                    int32_t a = x < 0 ? -x : x;
                    if (a > peak) peak = a;
                    sum += a;
                    int32_t dd = x - 2 * x1 + x2; if (dd < 0) dd = -dd;
                    dsum += dd; x2 = x1; x1 = x;
                }
                ESP_LOGW(TAG, "bclk=%d ws=%d din=%d  n=%3d peak=%8ld AC/DC=%3d%%",
                         clocks[c][0], clocks[c][1], (int)dins[d], n, (long)peak,
                         (int)(sum ? dsum * 100 / sum : 0));
                i2s_channel_disable(rx);
            }
            i2s_del_channel(rx);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    ESP_LOGW(TAG, "--- pin sweep done ---");
    free(buf);
}

static void mic_probe_shared_bus(void);

/* Brute-force RX configuration sweep: which slot width / mask / rate actually
 * carries the microphone?  Prints one line per variant with the AC/DC ratio,
 * which is ~0 for a floating line and large for real audio. */
static void board_mic_sweep(void)
{
    static const struct {
        const char *name;
        i2s_port_t port;
        i2s_clock_src_t clk;
        i2s_data_bit_width_t bits;
        i2s_slot_mode_t mode;
        i2s_std_slot_mask_t mask;
        int rate;
        bool slot32;
    } variants[] = {
        {"i2s1 L 16k",          I2S_NUM_1, I2S_CLK_SRC_DEFAULT, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT,  16000, false},
        {"i2s1 L 16k slot32",   I2S_NUM_1, I2S_CLK_SRC_DEFAULT, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT,  16000, true},
        {"i2s1 L 16k xtal",     I2S_NUM_1, I2S_CLK_SRC_XTAL,    I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT,  16000, false},
        {"i2s1 R 16k",          I2S_NUM_1, I2S_CLK_SRC_DEFAULT, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_RIGHT, 16000, false},
        {"i2s0 L 16k",          I2S_NUM_0, I2S_CLK_SRC_DEFAULT, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT,  16000, false},
        {"i2s0 L 48k",          I2S_NUM_0, I2S_CLK_SRC_DEFAULT, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT,  48000, false},
        {"i2s1 L 8k",           I2S_NUM_1, I2S_CLK_SRC_DEFAULT, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT,   8000, false},
    };
    int32_t *buf = heap_caps_malloc(512 * sizeof(int32_t),
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buf) return;

    ESP_LOGW(TAG, "--- microphone configuration sweep ---");
    for (size_t v = 0; v < sizeof variants / sizeof variants[0]; v++) {
        i2s_channel_disable(s_rx);
        i2s_del_channel(s_rx);
        s_rx = NULL;

        i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(variants[v].port, I2S_ROLE_MASTER);
        cc.dma_desc_num = 6;
        cc.dma_frame_num = 240;
        if (i2s_new_channel(&cc, NULL, &s_rx) != ESP_OK) continue;

        i2s_std_config_t c = {
            .clk_cfg  = { .sample_rate_hz = (uint32_t)variants[v].rate,
                          .clk_src = variants[v].clk,
                          .mclk_multiple = I2S_MCLK_MULTIPLE_256 },
            .slot_cfg = { .data_bit_width = variants[v].bits,
                          .slot_bit_width = variants[v].slot32
                              ? I2S_SLOT_BIT_WIDTH_32BIT : I2S_SLOT_BIT_WIDTH_AUTO,
                          .slot_mode = variants[v].mode,
                          .slot_mask = variants[v].mask,
                          .ws_width = variants[v].bits,
                          .ws_pol = false,
                          .bit_shift = true,
                          .left_align = true,
                          .big_endian = false,
                          .bit_order_lsb = false },
            .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = PIN_MIC_SCK,
                          .ws = PIN_MIC_WS, .dout = I2S_GPIO_UNUSED,
                          .din = PIN_MIC_DIN },
        };
        if (i2s_channel_init_std_mode(s_rx, &c) != ESP_OK ||
            i2s_channel_enable(s_rx) != ESP_OK) {
            ESP_LOGW(TAG, "%-16s init failed", variants[v].name);
            continue;
        }

        size_t got = 0;
        i2s_channel_read(s_rx, buf, 512 * sizeof(int32_t), &got, 200);
        int n = (int)(got / sizeof(int32_t));
        int shift = (variants[v].bits == I2S_DATA_BIT_WIDTH_16BIT) ? 0 : 12;
        int32_t peak = 0; int64_t sum = 0, dsum = 0;
        int32_t x1 = 0, x2 = 0;
        for (int i = 0; i < n; i++) {
            int32_t x = buf[i] >> shift;
            int32_t a = x < 0 ? -x : x;
            if (a > peak) peak = a;
            sum += a;
            /* 2nd difference = strong high-pass: ~0 for DC/ramp, big for audio */
            int32_t d = x - 2 * x1 + x2;
            if (d < 0) d = -d;
            dsum += d;
            x2 = x1; x1 = x;
        }
        int ac = (int)((sum && n) ? (dsum * 100 / sum) : 0);
        ESP_LOGW(TAG, "%-16s n=%3d peak=%6ld AC/DC=%3d%%", variants[v].name, n,
                 (long)peak, ac);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    ESP_LOGW(TAG, "--- sweep done ---");
    free(buf);
}

esp_err_t board_audio_init(void)
{
    /* GPIO21 is this board's power-hold pin: the stock firmware drives it high
     * at boot and only pulls it low (with a latch) when shutting down.  Without
     * it the microphone rail never comes up and the I2S data line just drifts —
     * which reads as "100 % of the energy below 300 Hz, 0 % in the voice band". */
    /* The RTC-IO route silently failed here (rtc_gpio_set_level(1) left the pin
     * at 0), so the board's power-hold rail never came up and the microphone
     * stayed unpowered — its data line read as a slow drift instead of audio.
     * Drive it with the plain digital GPIO mux instead and verify. */
    rtc_gpio_hold_dis(PIN_PWR_HOLD);
    rtc_gpio_deinit(PIN_PWR_HOLD);
    gpio_config_t ph = {
        .pin_bit_mask = 1ULL << PIN_PWR_HOLD,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ph));
    gpio_set_level(PIN_PWR_HOLD, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGW(TAG, "PWR_HOLD gpio21 = %d (must be 1)",
             (int)gpio_get_level(PIN_PWR_HOLD));


    /* One full-duplex bus: the amplifier AND the microphone share BCLK 15 /
     * WS 16, with the mic data on GPIO6.  Doing this as two simplex buses (the
     * upstream xingzhi wiring) leaves the mic unclocked — its data pin then
     * reads as pure drift instead of audio. */
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    cc.auto_clear_after_cb = true;
    cc.dma_desc_num  = 8;
    cc.dma_frame_num = 240;
    ESP_ERROR_CHECK(i2s_new_channel(&cc, &s_tx, &s_rx));

    i2s_std_config_t bus = {
        .clk_cfg  = { .sample_rate_hz = 48000, .clk_src = I2S_CLK_SRC_DEFAULT,
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
                      .din = PIN_MIC_DIN },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx, &bus));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx, &bus));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx));
    ESP_LOGI(TAG, "full-duplex bus 48k: amp on gpio%d, mic on gpio%d",
             PIN_SPK_DOUT, PIN_MIC_DIN);

    /* RX path self-test: play a tone on the amp (GPIO7 carries it) and read
     * that same pin back on I2S1 — in MONO and in STEREO.  All-zero means the
     * receiver itself is broken, which would explain every "silent mic" so far. */
    {
        static int16_t tone[2400];
        for (int i = 0; i < 2400; i++) {
            tone[i] = (int16_t)(9000 * sinf(2.0f * 3.14159265f * 1200.0f * i / 48000.0f));
        }
        for (int r = 0; r < 3; r++) board_spk_write(tone, 2400);

        int32_t *buf = heap_caps_malloc(1024 * sizeof(int32_t),
                                        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        for (int st = 0; st < 2 && buf; st++) {
            i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
            cc.dma_desc_num = 6; cc.dma_frame_num = 240;
            i2s_chan_handle_t rx = NULL;
            if (i2s_new_channel(&cc, NULL, &rx) != ESP_OK) continue;
            i2s_std_config_t c = {
                .clk_cfg = { .sample_rate_hz = 48000, .clk_src = I2S_CLK_SRC_DEFAULT,
                             .mclk_multiple = I2S_MCLK_MULTIPLE_256 },
                .slot_cfg = { .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
                              .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                              .slot_mode = st ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO,
                              .slot_mask = I2S_STD_SLOT_LEFT,
                              .ws_width = I2S_DATA_BIT_WIDTH_32BIT, .ws_pol = false,
                              .bit_shift = true, .left_align = true,
                              .big_endian = false, .bit_order_lsb = false },
                .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = PIN_SPK_BCLK,
                              .ws = PIN_SPK_LRCK, .dout = I2S_GPIO_UNUSED,
                              .din = PIN_SPK_DOUT },
            };
            if (i2s_channel_init_std_mode(rx, &c) == ESP_OK &&
                i2s_channel_enable(rx) == ESP_OK) {
                size_t got = 0;
                i2s_channel_read(rx, buf, 1024 * sizeof(int32_t), &got, 300);
                int n = (int)(got / sizeof(int32_t));
                int32_t pk = 0;
                for (int k = 0; k < n; k++) { int32_t a = buf[k] < 0 ? -buf[k] : buf[k]; if (a > pk) pk = a; }
                ESP_LOGW(TAG, "RX self-test %s (din=gpio7, tone): n=%d peak=%ld head=%08lx %08lx",
                         st ? "STEREO" : "MONO", n, (long)pk,
                         (unsigned long)(n > 0 ? buf[0] : 0),
                         (unsigned long)(n > 1 ? buf[1] : 0));
                i2s_channel_disable(rx);
            }
            i2s_del_channel(rx);
        }
        free(buf);
    }

    xTaskCreatePinnedToCore(mic_pump_task, "mic", 4096, NULL, 7, NULL, 1);
    ESP_LOGI(TAG, "audio ready (mic 48k -> 16k decimated)");
    return ESP_OK;
}

/* ── Speaker (I2S0, TX only) ─────────────────────────────────────── */
static esp_err_t spk_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear_after_cb = true;   /* silence instead of repeat */
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.dma_frame_num = 240;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SPK_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_BITS, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_SPK_BCLK,
            .ws   = PIN_SPK_LRCK,
            .dout = PIN_SPK_DOUT,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx));
    return ESP_OK;
}

/* Probe: is the microphone wired to the *amplifier's* I2S bus (BCLK 15 /
 * WS 16 / DIN 6) instead of its own GPIO4/5?  Cheap test: borrow I2S0. */
/* Brute force: which GPIO, when driven high, powers the microphone?
 * Reads the mic on the amp's clock pair while toggling candidates. */
static void mic_probe_shared_bus(void)
{
    int32_t *buf = heap_caps_malloc(512 * sizeof(int32_t),
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buf) return;

    i2s_channel_disable(s_tx);
    i2s_del_channel(s_tx);
    s_tx = NULL;

    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    cc.dma_desc_num = 6;
    cc.dma_frame_num = 240;
    i2s_chan_handle_t rx = NULL;
    if (i2s_new_channel(&cc, NULL, &rx) == ESP_OK) {
        i2s_std_config_t c = {
            .clk_cfg  = { .sample_rate_hz = 48000, .clk_src = I2S_CLK_SRC_DEFAULT,
                          .mclk_multiple = I2S_MCLK_MULTIPLE_256 },
            .slot_cfg = { .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
                          .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                          .slot_mode = I2S_SLOT_MODE_MONO,
                          .slot_mask = I2S_STD_SLOT_LEFT,
                          .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
                          .ws_pol = false, .bit_shift = true, .left_align = true,
                          .big_endian = false, .bit_order_lsb = false },
            .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = PIN_SPK_BCLK,
                          .ws = PIN_SPK_LRCK, .dout = I2S_GPIO_UNUSED,
                          .din = PIN_MIC_DIN },
        };
        if (i2s_channel_init_std_mode(rx, &c) == ESP_OK &&
            i2s_channel_enable(rx) == ESP_OK) {
            size_t got = 0;
            i2s_channel_read(rx, buf, 512 * sizeof(int32_t), &got, 200);
            int n = (int)(got / sizeof(int32_t));
            int32_t peak = 0; int64_t sum = 0, dsum = 0, dcu = 0;
            int32_t x1 = 0, x2 = 0;
            for (int i = 0; i < n; i++) {
                int32_t x = buf[i];
                int32_t a = x < 0 ? -x : x;
                if (a > peak) peak = a;
                sum += a;
                int32_t d = x - 2 * x1 + x2; if (d < 0) d = -d;
                dsum += d;
                x2 = x1; x1 = x;
            }
            dcu = sum;
            ESP_LOGW(TAG, "SHARED-BUS mic probe (bclk15/ws16/din6): n=%d peak=%ld "
                     "AC/DC=%d%%", n, (long)peak,
                     (int)(dcu ? dsum * 100 / dcu : 0));
        } else {
            ESP_LOGW(TAG, "SHARED-BUS mic probe: init/enable failed");
        }
        i2s_channel_disable(rx);
        i2s_del_channel(rx);
    }
    free(buf);
    spk_init();                 /* restore the amplifier */
}

/* ── Microphone on its own I2S1 bus (the upstream wiring) ────────── */
static esp_err_t mic_init_legacy(void)
{
    i2s_chan_config_t mic_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    mic_cfg.dma_desc_num  = 6;
    mic_cfg.dma_frame_num = 240;
    ESP_ERROR_CHECK(i2s_new_channel(&mic_cfg, NULL, &s_rx));

    i2s_std_config_t mic_std = {
        .clk_cfg  = {
            .sample_rate_hz = MIC_RATE,
            .clk_src        = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
        },
        /* Field-for-field copy of the stock firmware's slot config.  With the
         * IDF default (left_align=false) this mic produced bit-misaligned
         * garbage: ~22 M DC with 1 % AC, i.e. no modulation at all. */
        .slot_cfg = {
            .data_bit_width = I2S_BITS,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode      = I2S_SLOT_MODE_STEREO,   /* probe: read L+R */
            .slot_mask      = I2S_STD_SLOT_BOTH,
            .ws_width       = I2S_BITS,
            .ws_pol         = false,
            .bit_shift      = true,
            .left_align     = true,
            .big_endian     = false,
            .bit_order_lsb  = false,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_MIC_SCK,
            .ws   = PIN_MIC_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = PIN_MIC_DIN,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx, &mic_std));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx));

    /* Audible proof that the amplifier rail is alive. */
    {
        static int16_t tone[2400];
        for (int i = 0; i < 2400; i++) {
            tone[i] = (int16_t)(9000 * sinf(2.0f * 3.14159265f * 1200.0f * i / 48000.0f));
        }
        for (int r = 0; r < 8; r++) board_spk_write(tone, 2400);   /* ~0.4 s */
        ESP_LOGW(TAG, "played 1.2 kHz test tone (amp/power check)");
    }

    board_mic_sweep();
    return ESP_OK;
}
