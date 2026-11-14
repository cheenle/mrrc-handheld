/*
 * MRRC Handheld — application shell.
 *
 * Control model (as requested):
 *   VOL+ / VOL-          frequency ±1 kHz  (FREQ mode)
 *                        AF gain ±step     (VOL mode)
 *   BOOT long press      PTT  (key on after the long-press threshold)
 *   BOOT double click    toggle FREQ <-> VOL
 *
 * The PTT state machine owns the "never leave the radio keyed" rule:
 * frames stop first, then "s:", then ptt:false — and a 180 s watchdog.
 */
#include "app.h"
#include "board.h"
#include "mrrc.h"
#include "ui.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "board_pins.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "app";

app_shared_t g_app;
static SemaphoreHandle_t s_mtx;

#define PTT_MAX_MS        180000     /* design §5 failsafe */
#define TX_HB_PERIOD_MS   1500       /* design §5 heartbeat */
#define PTT_TAIL_MS       200        /* RELEASING -> IDLE */

#ifndef CONFIG_MRRC_FREQ_STEP_HZ
#define CONFIG_MRRC_FREQ_STEP_HZ 1000
#endif
#ifndef CONFIG_MRRC_AF_STEP
#define CONFIG_MRRC_AF_STEP 6
#endif
#ifndef CONFIG_MRRC_FREQ_MIN_HZ
#define CONFIG_MRRC_FREQ_MIN_HZ 100000
#endif
#ifndef CONFIG_MRRC_FREQ_MAX_HZ
#define CONFIG_MRRC_FREQ_MAX_HZ 54000000
#endif

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

void app_lock(void)
{
    if (s_mtx) xSemaphoreTake(s_mtx, portMAX_DELAY);
}

void app_unlock(void)
{
    if (s_mtx) xSemaphoreGive(s_mtx);
}

/* ── callbacks used by the network layer ─────────────────────────── */

void app_on_state_changed(void)
{
    app_lock();
    g_app.st_seq++;
    app_unlock();
}

void app_on_spectrum(const uint8_t *bins, int n)
{
    if (!bins || n <= 0) return;
    if (n > MRRC_BINS) n = MRRC_BINS;
    app_lock();
    memcpy(g_app.spec, bins, (size_t)n);
    g_app.spec_seq++;
    app_unlock();
}

void app_show_toast(const char *msg)
{
    if (!msg) return;
    app_lock();
    snprintf(g_app.toast, sizeof g_app.toast, "%s", msg);
    g_app.toast_seq++;
    g_app.toast_until_ms = now_ms() + 3000;
    app_unlock();
    ESP_LOGW(TAG, "toast: %s", msg);
}

/* ── PTT ─────────────────────────────────────────────────────────── */

static void ptt_begin(void)
{
    app_lock();
    if (g_app.ptt != PTT_IDLE) {
        app_unlock();
        return;
    }
    g_app.ptt = PTT_KEYING;
    g_app.ptt_since_ms = now_ms();
    app_unlock();

    mrrc_send_ptt(true);
    mrrc_send_txhb();          /* capability + liveness, design §5 */
    mrrc_tx_set_active(true);  /* TX task may now push frames */
    ESP_LOGI(TAG, "PTT down");
}

static void ptt_end(void)
{
    app_lock();
    if (g_app.ptt != PTT_KEYING) {
        app_unlock();
        return;
    }
    g_app.ptt = PTT_RELEASING;
    g_app.ptt_since_ms = now_ms();
    app_unlock();

    mrrc_tx_set_active(false); /* 1. stop mic frames  */
    mrrc_tx_stop_text();       /* 2. "s:" — drain, don't chop the tail */
    mrrc_send_ptt(false);      /* 3. unkey */
    ESP_LOGI(TAG, "PTT up");
}

/* ── knobs ───────────────────────────────────────────────────────── */

static void toggle_ui_mode(void)
{
    app_lock();
    g_app.ui_mode = (g_app.ui_mode == UI_MODE_FREQ) ? UI_MODE_VOL : UI_MODE_FREQ;
    ui_mode_t m = g_app.ui_mode;
    app_unlock();
    app_show_toast(m == UI_MODE_VOL ? "VOLUME" : "FREQUENCY");
}

#define SPK_VOL_STEP 5

static int nvs_get_spk_vol(void)
{
    nvs_handle_t h;
    int v = CONFIG_MRRC_SPK_VOLUME;
    if (nvs_open("mrrc", NVS_READONLY, &h) == ESP_OK) {
        int32_t t;
        if (nvs_get_i32(h, "spk_vol", &t) == ESP_OK && t >= 0 && t <= 100) {
            v = (int)t;
        }
        nvs_close(h);
    }
    return v;
}

static void nvs_put_spk_vol(int v)
{
    nvs_handle_t h;
    if (nvs_open("mrrc", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "spk_vol", v);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void step(int dir)
{
    app_lock();
    ui_mode_t mode = g_app.ui_mode;
    int32_t freq = g_app.st.freq_hz;
    int vol = g_app.spk_vol;
    app_unlock();

    if (mode == UI_MODE_FREQ) {
        if (freq <= 0) {
            /* No fullState yet: clamping 0 to the lower bound would teleport
             * the radio to 100 kHz the moment a key is touched. */
            app_show_toast("NO VFO YET");
            return;
        }
        int32_t nf = freq + (int32_t)dir * CONFIG_MRRC_FREQ_STEP_HZ;
        if (nf < CONFIG_MRRC_FREQ_MIN_HZ) nf = CONFIG_MRRC_FREQ_MIN_HZ;
        if (nf > CONFIG_MRRC_FREQ_MAX_HZ) nf = CONFIG_MRRC_FREQ_MAX_HZ;
        app_lock();
        g_app.st.freq_hz = nf;
        app_unlock();
        app_on_state_changed();
        mrrc_set_freq(nf);
    } else {
        /* Terminal speaker volume — a local software gain on the I2S output.
         * The radio's AF gain is a different thing and stays untouched. */
        int nv = vol + dir * SPK_VOL_STEP;
        if (nv < 0) nv = 0;
        if (nv > 100) nv = 100;
        app_lock();
        g_app.spk_vol = nv;
        g_app.spk_vol_dirty = true;
        app_unlock();
        app_on_state_changed();
    }
}

/* ── main ────────────────────────────────────────────────────────── */

void app_main(void)
{
    s_mtx = xSemaphoreCreateMutex();
    memset(&g_app, 0, sizeof g_app);
    g_app.st.battery_pct = -1;
    g_app.st.af_gain = 128;
    g_app.ui_mode = UI_MODE_FREQ;
    ESP_ERROR_CHECK(nvs_flash_init());
    g_app.spk_vol = nvs_get_spk_vol();
    g_app.ptt = PTT_IDLE;
    strcpy(g_app.st.mode_name, "---");
    strcpy(g_app.st.band_name, "--");

    /* Why did we reboot last time?  Panic / brownout / watchdog all land here. */
    {
        static const char *rs[] = {
            "UNKNOWN", "POWERON", "EXT", "SW", "PANIC", "INT_WDT",
            "TASK_WDT", "WDT", "DEEPSLEEP", "BROWNOUT", "SDIO",
        };
        esp_reset_reason_t r = esp_reset_reason();
        ESP_LOGW(TAG, "reset reason: %s (%d)",
                 (r >= 0 && r < (int)(sizeof rs / sizeof rs[0])) ? rs[r] : "?", r);
    }
    ESP_LOGI(TAG, "MRRC handheld booting on coolkit-xz-02 / ESP32-S3, heap=%u",
             (unsigned)esp_get_free_heap_size());

    ESP_ERROR_CHECK(board_display_init());
    ESP_ERROR_CHECK(board_audio_init());
    board_input_init();
    ui_start();
    mrrc_net_start();

    app_show_toast("MRRC BOOT");

    int64_t last_batt = 0;
    int64_t last_hb_log = 0;

    int dump_stage = 0;
    int hold_ms = 0;

    while (1) {
        int64_t t = now_ms();

        /* Debug hook: one automatic dump (15 s in), plus VOL+ + VOL- held
         * together for 1.5 s.  Lets the UI be inspected without a camera. */
#ifdef CONFIG_MRRC_DEV_SELFTEST
        if (dump_stage == 0 && t > 12000) { dump_stage = 1; ui_dump_fb(); }
        else if (dump_stage == 1 && t > 26000) { dump_stage = 2; ui_dump_fb(); }

        /* DEV: reproducible PTT self-test (keys the radio briefly).
         *   45 s  ptt:true + txhb, no mic frames  (dead carrier, 2 s)
         *   47 s  mic frames start
         *   50 s  frames stop, "s:", ptt:false                          */
        if (dump_stage == 2 && t > 45000 && t < 47000 && g_app.ptt == PTT_IDLE) {
            ptt_begin();
            ESP_LOGW(TAG, "SELFTEST: PTT keyed (no audio)");
        } else if (dump_stage == 2 && t > 47200 && t < 70000 && g_app.ptt == PTT_KEYING) {
            mrrc_tx_set_active(true);
        } else if (dump_stage == 2 && t > 70000 && g_app.ptt == PTT_KEYING) {
            ESP_LOGW(TAG, "SELFTEST: PTT release");
            ptt_end();
        }
#endif
        if (gpio_get_level(PIN_BTN_VOL_UP) == 0 &&
            gpio_get_level(PIN_BTN_VOL_DN) == 0) {
            hold_ms += 20;
            if (hold_ms == 1520) ui_dump_fb();
        } else {
            hold_ms = 0;
        }

        /* Persist the volume a second after the last click, not per click. */
        app_lock();
        bool vol_dirty = g_app.spk_vol_dirty;
        int vol_now = g_app.spk_vol;
        g_app.spk_vol_dirty = false;
        app_unlock();
        if (vol_dirty) {
            static int64_t vol_saved_ms;
            vol_saved_ms = t;
            vTaskDelay(pdMS_TO_TICKS(1000));
            app_lock();
            int vol_final = g_app.spk_vol;
            g_app.spk_vol_dirty = false;
            app_unlock();
            nvs_put_spk_vol(vol_final);
            ESP_LOGI(TAG, "spk volume %d%% saved", vol_final);
            (void)vol_now;
        }

        /* Battery + charge state (ADC2 is shared with Wi-Fi: it may fail). */
        if (t - last_batt >= 20000) {
            last_batt = t;
            int pct = -1;
            bool charging = false;
            board_power_status(&pct, &charging);
            app_lock();
            g_app.st.battery_pct = pct;
            g_app.st.charging = charging;
            app_unlock();
            app_on_state_changed();
        }

        input_event_t ev;
        while ((ev = board_input_get_event()) != INPUT_NONE) {
            switch (ev) {
            case INPUT_BOOT_LONG:   ptt_begin();     break;
            case INPUT_BOOT_RELEASE: ptt_end();      break;
            case INPUT_BOOT_DOUBLE: toggle_ui_mode(); break;
            case INPUT_VOL_UP:      step(+1);        break;
            case INPUT_VOL_DOWN:    step(-1);        break;
            default: break;
            }
        }

        /* Heartbeat: a headless box is undebuggable without this. */
        if (t - last_hb_log >= 10000) {
            last_hb_log = t;
            app_lock();
            int32_t f = g_app.st.freq_hz;
            int sm = g_app.st.s_meter;
            bool wu = g_app.wifi_up, cu = g_app.ctrl_up;
            int rssi = g_app.rssi;
            int32_t seq = (int32_t)g_app.spec_seq;
            app_unlock();
            uint32_t arx = 0, atx = 0, aerr = 0, atb = 0;
            mrrc_get_stats(&arx, &atx, &aerr, &atb);
            ESP_LOGI(TAG, "hb: wifi=%d ctrl=%d rssi=%d freq=%ld s=%d spec=%ld "
                     "audio(rx=%lu tx=%lu err=%lu txB=%lu) drops(wifi=%lu ws=%lu) "
                     "heap=%u min=%u", wu, cu, rssi, (long)f, sm, (long)seq,
                     (unsigned long)arx, (unsigned long)atx,
                     (unsigned long)aerr, (unsigned long)atb,
                     (unsigned long)mrrc_wifi_disconnects(),
                     (unsigned long)mrrc_ws_disconnects(),
                     (unsigned)esp_get_free_heap_size(),
                     (unsigned)esp_get_minimum_free_heap_size());
        }

        app_lock();
        ptt_state_t ps = g_app.ptt;
        int64_t since = g_app.ptt_since_ms;
        int tx_now = g_app.st.tx_status;
        app_unlock();

        if (ps == PTT_KEYING) {
            if (mrrc_take_force_rx()) {
                app_show_toast("SERVER UNKEYED");
                ptt_end();
            } else if (tx_now == 0 && t - since > 8000) {
                /* tx_status arrives 3-5 s late from the server poll; only give
                 * up when the radio clearly never keyed. */
                app_show_toast("NO TX");
                ptt_end();
            }
        }

        if (ps == PTT_KEYING) {
            if (t - since > PTT_MAX_MS) {
                ESP_LOGW(TAG, "PTT failsafe: %d s reached", PTT_MAX_MS / 1000);
                ptt_end();
                app_show_toast("TX TIMEOUT");
            }
        } else if (ps == PTT_RELEASING && t - since > PTT_TAIL_MS) {
            app_lock();
            g_app.ptt = PTT_IDLE;
            app_unlock();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
