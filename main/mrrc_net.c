/*
 * MRRC client: Wi-Fi + login + 4 WebSockets (/WSradio /WSspectrum
 * /WSaudioRX /WSaudioTX) + audio decode/encode.
 *
 * Wire contract (verified against server.py):
 *   /WSradio    JSON: fullState / stateUpdate / value / error
 *   /WSspectrum binary: 0x01 + wf1(850) [+ wf2(850)] — we declare "mid"
 *   /WSaudioRX  binary: 1-byte tag (0x00 PCM / 0x01 Opus) + payload, 48 kHz mono
 *   /WSaudioTX  binary: 1-byte tag + payload; text "s:" = stop capturing
 *   auth        POST /api/auth/login {"password":...} -> {"token":...}
 *               then "Authorization: Bearer <token>" on every WS handshake
 */
#include "mrrc.h"
#include "app.h"
#include "board.h"

#include "board_pins.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_websocket_client.h"
#ifdef CONFIG_MRRC_TLS
#include "esp_crt_bundle.h"
#endif
#include "nvs_flash.h"
#include "cJSON.h"

#if CONFIG_MRRC_AUDIO_OPUS
#include "opus.h"
#endif

static const char *TAG = "mrrc";

#ifndef CONFIG_MRRC_UI_FPS
#define CONFIG_MRRC_UI_FPS 12
#endif
#if !defined(CONFIG_MRRC_MIC_RATE)
#define CONFIG_MRRC_MIC_RATE 16000
#endif

#define WS_TASK_STACK   7168
#define WS_BUF_SIZE     4096
#define FRAME_MAX       8192      /* /WSradio fullState measured 5509 B */
#define RB_BYTES        49152     /* ~6 s of 64 kbps Opus jitter buffer */

/* ── connection state ────────────────────────────────────────────── */

typedef struct {
    esp_websocket_client_handle_t handle;
    const char *path;
    volatile bool up;
    uint8_t *acc;
    size_t   acc_len;
    const char *name;
} ws_ctx_t;

static ws_ctx_t s_ctrl = { .path = "/WSradio",    .name = "ctrl" };
static ws_ctx_t s_spec = { .path = "/WSspectrum", .name = "spec" };
static ws_ctx_t s_rx   = { .path = "/WSaudioRX",  .name = "audioRX" };
static ws_ctx_t s_tx   = { .path = "/WSaudioTX",  .name = "audioTX" };

static volatile bool s_want_ctrl_hello;
static volatile bool s_want_spec_caps;

static char s_token[96];
static char s_hdr[160];
static volatile bool s_wifi_up;
static volatile bool s_tx_active;
static volatile uint32_t s_st_rx_frames, s_st_tx_frames, s_st_rx_err, s_st_tx_bytes;
static volatile uint32_t s_wifi_dc, s_ws_dc;
static volatile bool s_force_rx;

bool mrrc_take_force_rx(void)
{
    bool f = s_force_rx;
    s_force_rx = false;
    return f;
}

uint32_t mrrc_wifi_disconnects(void) { return s_wifi_dc; }
uint32_t mrrc_ws_disconnects(void) { return s_ws_dc; }

void mrrc_get_stats(uint32_t *rx, uint32_t *tx, uint32_t *err, uint32_t *txb)
{
    if (rx)  *rx  = s_st_rx_frames;
    if (tx)  *tx  = s_st_tx_frames;
    if (err) *err = s_st_rx_err;
    if (txb) *txb = s_st_tx_bytes;
}
static int s_wifi_retry;

/* ── RX ring buffer (WS task -> audio task) ──────────────────────── */

static uint8_t          *s_rb;
static size_t            s_rb_r, s_rb_w, s_rb_used;
static SemaphoreHandle_t s_rb_mtx;
static SemaphoreHandle_t s_rb_data;

static bool rb_put(const uint8_t *d, size_t n)
{
    if (n + 2 > RB_BYTES) return false;
    bool ok = false;
    xSemaphoreTake(s_rb_mtx, portMAX_DELAY);
    if (RB_BYTES - s_rb_used >= n + 2) {
        uint8_t hdr[2] = { (uint8_t)(n & 0xFF), (uint8_t)(n >> 8) };
        for (int i = 0; i < 2; i++) {
            s_rb[s_rb_w] = hdr[i];
            s_rb_w = (s_rb_w + 1) % RB_BYTES;
        }
        for (size_t i = 0; i < n; i++) {
            s_rb[s_rb_w] = d[i];
            s_rb_w = (s_rb_w + 1) % RB_BYTES;
        }
        s_rb_used += n + 2;
        ok = true;
    }
    xSemaphoreGive(s_rb_mtx);
    if (ok) xSemaphoreGive(s_rb_data);
    return ok;
}

static bool rb_get_byte(uint8_t *b)
{
    if (!s_rb_used) return false;
    *b = s_rb[s_rb_r];
    s_rb_r = (s_rb_r + 1) % RB_BYTES;
    s_rb_used--;
    return true;
}

/* Blocking frame read: returns payload length, 0 on timeout. */
static size_t rb_get_frame(uint8_t *out, size_t max, TickType_t wait)
{
    if (xSemaphoreTake(s_rb_data, wait) != pdTRUE) return 0;
    size_t len = 0;
    xSemaphoreTake(s_rb_mtx, portMAX_DELAY);
    uint8_t lo = 0, hi = 0;
    if (rb_get_byte(&lo) && rb_get_byte(&hi)) {
        len = (size_t)lo | ((size_t)hi << 8);
        if (len > max) {                       /* corrupted: drain it */
            for (size_t i = 0; i < len; i++) { uint8_t t; rb_get_byte(&t); }
            len = 0;
        } else {
            for (size_t i = 0; i < len; i++) rb_get_byte(&out[i]);
        }
    }
    xSemaphoreGive(s_rb_mtx);
    /* Anything left? Re-arm the semaphore so the next call does not block. */
    if (s_rb_used > 2) xSemaphoreGive(s_rb_data);
    return len;
}

/* ── helpers ─────────────────────────────────────────────────────── */

bool mrrc_ctrl_up(void) { return s_ctrl.up; }
radio_state_t *mrrc_state_ptr(void) { return &g_app.st; }
bool mrrc_tx_ready(void) { return s_tx.up; }

void mrrc_tx_set_active(bool on) { s_tx_active = on; }

static void ws_send_text(ws_ctx_t *c, const char *text)
{
    if (!c->handle || !c->up) return;
    esp_websocket_client_send_text(c->handle, text, (int)strlen(text),
                                   pdMS_TO_TICKS(2000));
}

void mrrc_tx_frame(const uint8_t *tagged, size_t len)
{
    if (!s_tx.handle || !s_tx.up) return;
    esp_websocket_client_send_bin(s_tx.handle, (const char *)tagged, (int)len,
                                  pdMS_TO_TICKS(2000));
}

void mrrc_tx_stop_text(void)
{
    ws_send_text(&s_tx, "s:");
}

void mrrc_send_ptt(bool on)
{
    char b[64];
    snprintf(b, sizeof b, "{\"type\":\"set\",\"field\":\"ptt\",\"value\":%s}",
             on ? "true" : "false");
    ws_send_text(&s_ctrl, b);
    ESP_LOGI(TAG, "PTT %s", on ? "ON" : "OFF");
}

void mrrc_send_txhb(void)
{
    ws_send_text(&s_ctrl, "{\"type\":\"txhb\"}");
}

void mrrc_set_freq(int32_t hz)
{
    ESP_LOGW(TAG, "TX freq set -> %ld Hz (from a VOL key press)", (long)hz);
    char b[80];
    snprintf(b, sizeof b, "{\"type\":\"set\",\"field\":\"freq\",\"value\":%ld}",
             (long)hz);
    ws_send_text(&s_ctrl, b);
}

void mrrc_set_af_gain(int v)
{
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    char b[80];
    snprintf(b, sizeof b, "{\"type\":\"set\",\"field\":\"af_gain\",\"value\":%d}", v);
    ws_send_text(&s_ctrl, b);
}

/* ── server state → app_shared ───────────────────────────────────── */

/* The server pushes s_meter at ~20 Hz.  Redrawing the whole static area for
 * that would burn SPI bandwidth, so changes are bucketed: frequency/mode/TX
 * redraws the top half, meters redraw the 18 px strip only. */
#define M_NONE 0
#define M_MAIN 1
#define M_METER 2
static int s_changed;

static void apply_field(const char *k, cJSON *v)
{
    radio_state_t *st = &g_app.st;
    if (!strcmp(k, "vfo_a_freq") && cJSON_IsNumber(v)) {
        st->vfo_a_hz = (int32_t)v->valuedouble;
        s_changed |= M_MAIN;
    } else if (!strcmp(k, "vfo_b_freq") && cJSON_IsNumber(v)) {
        st->vfo_b_hz = (int32_t)v->valuedouble;
        s_changed |= M_MAIN;
    } else if (!strcmp(k, "active_freq") && cJSON_IsNumber(v)) {
        st->freq_hz = (int32_t)v->valuedouble;
        s_changed |= M_MAIN;
    } else if (!strcmp(k, "active_vfo") && cJSON_IsString(v)) {
        st->vfo = v->valuestring[0] ? v->valuestring[0] : 'A';
        s_changed |= M_MAIN;
    } else if ((!strcmp(k, "mode_display") || !strcmp(k, "mode_name")) &&
               cJSON_IsString(v)) {
        snprintf(st->mode_name, sizeof st->mode_name, "%s", v->valuestring);
        s_changed |= M_MAIN;
    } else if ((!strcmp(k, "band_name") || !strcmp(k, "band")) && cJSON_IsString(v)) {
        snprintf(st->band_name, sizeof st->band_name, "%s", v->valuestring);
        s_changed |= M_MAIN;
    } else if (!strcmp(k, "mode") && cJSON_IsNumber(v)) {
        st->mode_num = (int)v->valuedouble;
        s_changed |= M_MAIN;
    } else if (!strcmp(k, "s_meter") && cJSON_IsNumber(v)) {
        st->s_meter = (int)v->valuedouble;
        s_changed |= M_METER;
    } else if (!strcmp(k, "s_unit") && cJSON_IsString(v)) {
        snprintf(st->s_unit, sizeof st->s_unit, "%s", v->valuestring);
        s_changed |= M_METER;
    } else if (!strcmp(k, "s_meter_dbm") && cJSON_IsNumber(v)) {
        st->s_dbm = (float)v->valuedouble;
        s_changed |= M_METER;
    } else if (!strcmp(k, "af_gain") && cJSON_IsNumber(v)) {
        st->af_gain = (int)v->valuedouble;
        s_changed |= M_METER;
    } else if (!strcmp(k, "filter_hz") && cJSON_IsNumber(v)) {
        st->filter_hz = (int)v->valuedouble;
        s_changed |= M_MAIN;
    } else if (!strcmp(k, "swr_ratio") && cJSON_IsNumber(v)) {
        st->swr = (float)v->valuedouble;
        s_changed |= M_METER;
    } else if (!strcmp(k, "alc_pct") && cJSON_IsNumber(v)) {
        st->alc_pct = (float)v->valuedouble;
        s_changed |= M_METER;
    } else if (!strcmp(k, "power_watts") && cJSON_IsNumber(v)) {
        st->power_w = (int)v->valuedouble;
        s_changed |= M_METER;
    } else if (!strcmp(k, "tx_status") && cJSON_IsNumber(v)) {
        st->tx_status = (int)v->valuedouble;
        s_changed |= M_MAIN | M_METER;
    } else if (!strcmp(k, "is_transmitting") && cJSON_IsBool(v)) {
        st->tx_status = cJSON_IsTrue(v) ? 1 : 0;
        s_changed |= M_MAIN | M_METER;
    } else if (!strcmp(k, "scope_span") && cJSON_IsNumber(v)) {
        st->scope_span = (int)v->valuedouble;
    } else if (!strcmp(k, "serial_connected") && cJSON_IsBool(v)) {
        st->radio_online = cJSON_IsTrue(v);
        s_changed |= M_MAIN;
    }
}

static void apply_object(cJSON *obj)
{
    if (!cJSON_IsObject(obj)) return;
    for (cJSON *it = obj->child; it; it = it->next) {
        if (it->string) apply_field(it->string, it);
    }
    /* Reconcile the active frequency from the VFO pair when the server only
     * sent the VFO fields (stateUpdate is dirty-field based). */
    radio_state_t *st = &g_app.st;
    if (st->freq_hz == 0) {
        st->freq_hz = (st->vfo == 'B') ? st->vfo_b_hz : st->vfo_a_hz;
    }
}

static void handle_ctrl_text(const char *payload, int len)
{
    cJSON *root = cJSON_ParseWithLength(payload, (size_t)len);
    if (!root) {
        ESP_LOGW(TAG, "bad JSON: %.*s", len > 60 ? 60 : len, payload);
        return;
    }
    char err_msg[48] = {0};
    bool changed = false;

    app_lock();   /* apply_field() writes g_app.st directly */

    const cJSON *type = cJSON_GetObjectItem(root, "type");
    const char *t = cJSON_IsString(type) ? type->valuestring : "";

    if (!strcmp(t, "fullState")) {
        apply_object(cJSON_GetObjectItem(root, "data"));
        changed = true;
    } else if (!strcmp(t, "stateUpdate")) {
        cJSON *fields = cJSON_GetObjectItem(root, "fields");
        apply_object(fields);
        /* Only the VFO pair may arrive: recompute the active frequency. */
        radio_state_t *st = &g_app.st;
        int32_t want = (st->vfo == 'B') ? st->vfo_b_hz : st->vfo_a_hz;
        if (want && cJSON_GetObjectItem(fields, "active_freq") == NULL) {
            st->freq_hz = want;
        }
        changed = true;
    } else if (!strcmp(t, "value")) {
        const cJSON *f = cJSON_GetObjectItem(root, "field");
        if (cJSON_IsString(f)) apply_field(f->valuestring,
                                           cJSON_GetObjectItem(root, "value"));
        changed = true;
    } else if (!strcmp(t, "error")) {
        const cJSON *m = cJSON_GetObjectItem(root, "message");
        if (cJSON_IsString(m)) {
            snprintf(err_msg, sizeof err_msg, "%s", m->valuestring);
        }
        ESP_LOGW(TAG, "server error: %s", err_msg);
        if (strstr(err_msg, "PTT released") || strstr(err_msg, "FT8") ||
            strstr(err_msg, "CQ call")) {
            s_force_rx = true;      /* the server already unkeyed us */
        }
    }

    int bucket = s_changed;
    s_changed = M_NONE;
    app_unlock();
    if (bucket) {
        app_lock();
        if (bucket & M_MAIN)  g_app.st_seq++;
        if (bucket & M_METER) g_app.meter_seq++;
        app_unlock();
    }
    (void)changed;
    if (err_msg[0]) app_show_toast(err_msg);   /* takes the lock itself */
    cJSON_Delete(root);
}

/* ── WebSocket event plumbing ────────────────────────────────────── */

static void ws_common_event(ws_ctx_t *c, int32_t id, void *event_data)
{
    switch (id) {
    case WEBSOCKET_EVENT_CONNECTED:
        c->up = true;
        c->acc_len = 0;
        ESP_LOGI(TAG, "%s connected", c->name);
        if (c == &s_ctrl) {
            s_want_ctrl_hello = true;   /* sent from net_task, not from here */
        } else if (c == &s_spec) {
            s_want_spec_caps = true;
        }
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
    case WEBSOCKET_EVENT_CLOSED:
        if (c->up) {
            s_ws_dc++;
            ESP_LOGW(TAG, "%s disconnected (total ws drops %u)", c->name,
                     (unsigned)s_ws_dc);
        }
        c->up = false;
        c->acc_len = 0;
        break;

    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGW(TAG, "%s transport error", c->name);
        c->up = false;
        break;

    case WEBSOCKET_EVENT_DATA: {
        esp_websocket_event_data_t *d = (esp_websocket_event_data_t *)event_data;
        if (!d) break;
        if (d->op_code == 0x08 || d->op_code == 0x09 || d->op_code == 0x0A) break;
        if (d->payload_len <= 0) break;

        if ((size_t)d->payload_len > FRAME_MAX) {
            ESP_LOGW(TAG, "%s frame too big (%d)", c->name, d->payload_len);
            break;
        }
        if (d->payload_offset == 0) c->acc_len = 0;
        memcpy(c->acc + d->payload_offset, d->data_ptr, d->data_len);
        if (d->payload_offset + d->data_len < d->payload_len) break;  /* more */
        c->acc_len = (size_t)d->payload_len;

        if (c == &s_ctrl) {
            handle_ctrl_text((const char *)c->acc, (int)c->acc_len);
        } else if (c == &s_spec) {
            if (c->acc_len >= 1 + MRRC_BINS) {
                app_on_spectrum(c->acc + 1, MRRC_BINS);   /* skip version byte */
            }
        } else if (c == &s_rx) {
            if (!rb_put(c->acc, c->acc_len)) {
                static uint32_t drops;
                if ((drops++ % 100) == 0) {
                    ESP_LOGW(TAG, "RX audio buffer full, dropped (n=%u)",
                             (unsigned)drops);
                }
            }
        }
        break;
    }
    default:
        break;
    }
}

static void ws_event_handler(void *arg, esp_event_base_t base, int32_t id,
                             void *event_data)
{
    ws_ctx_t *c = (ws_ctx_t *)arg;
    if (!c) return;
    ws_common_event(c, id, event_data);
}

/* ── Wi-Fi ───────────────────────────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id,
                               void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_up = false;
        s_wifi_dc++;
        ESP_LOGW(TAG, "wifi disconnected #%u", (unsigned)s_wifi_dc);
        if (++s_wifi_retry < CONFIG_MRRC_WIFI_MAX_RETRY) {
            ESP_LOGW(TAG, "wifi disconnected, retry %d", s_wifi_retry);
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "wifi gave up, rebooting");
            esp_restart();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got ip " IPSTR, IP2STR(&ev->ip_info.ip));
        s_wifi_up = true;
        s_wifi_retry = 0;
    }
}

static void wifi_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wc = {0};
    snprintf((char *)wc.sta.ssid, sizeof wc.sta.ssid, "%s", CONFIG_MRRC_WIFI_SSID);
    snprintf((char *)wc.sta.password, sizeof wc.sta.password, "%s",
             CONFIG_MRRC_WIFI_PASSWORD);
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wc.sta.pmf_cfg.capable = true;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi STA -> %s", CONFIG_MRRC_WIFI_SSID);
}

/* ── login ───────────────────────────────────────────────────────── */

static bool mrrc_login(void)
{
    char url[160];
    snprintf(url, sizeof url, "%s://%s:%d/api/auth/login",
#if CONFIG_MRRC_TLS
             "https",
#else
             "http",
#endif
             CONFIG_MRRC_SERVER_HOST, CONFIG_MRRC_SERVER_PORT);

    char body[128];
    snprintf(body, sizeof body, "{\"password\":\"%s\"}", CONFIG_MRRC_SERVER_PASSWORD);

    esp_http_client_config_t cfg = {
        .url        = url,
        .method     = HTTP_METHOD_POST,
        .timeout_ms = 8000,
    };
#ifdef CONFIG_MRRC_TLS
#ifdef CONFIG_MRRC_TLS_INSECURE
    /* The box serves a self-signed certificate (issuer == subject), so chain
     * verification cannot succeed.  Requires CONFIG_ESP_TLS_INSECURE +
     * CONFIG_ESP_TLS_SKIP_SERVER_CERT_VERIFY in sdkconfig. */
    cfg.skip_cert_common_name_check = true;
#else
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
#endif
#endif
    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    if (!cli) return false;
    esp_http_client_set_header(cli, "Content-Type", "application/json");
    esp_http_client_set_post_field(cli, body, (int)strlen(body));

    char resp[384] = {0};
    int code = -1;
    if (esp_http_client_open(cli, (int)strlen(body)) == ESP_OK) {
        esp_http_client_write(cli, body, (int)strlen(body));
        esp_http_client_fetch_headers(cli);
        int n = esp_http_client_read(cli, resp, sizeof(resp) - 1);
        if (n > 0) resp[n] = 0;
        code = esp_http_client_get_status_code(cli);
    }
    esp_http_client_close(cli);
    esp_http_client_cleanup(cli);

    if (code != 200) {
        ESP_LOGE(TAG, "login failed (HTTP %d) %s", code, resp);
        return false;
    }
    cJSON *root = cJSON_Parse(resp);
    if (!root) return false;
    cJSON *tok = cJSON_GetObjectItem(root, "token");
    bool ok = cJSON_IsString(tok) && tok->valuestring[0];
    if (ok) {
        snprintf(s_token, sizeof s_token, "%s", tok->valuestring);
        snprintf(s_hdr, sizeof s_hdr, "Authorization: Bearer %s\r\n", s_token);
        ESP_LOGI(TAG, "logged in (token %c%c%c…)", s_token[0], s_token[1],
                 s_token[2]);
    }
    cJSON_Delete(root);
    return ok;
}

/* ── WebSocket lifecycle ─────────────────────────────────────────── */

static void ws_stop(ws_ctx_t *c)
{
    if (c->handle) {
        esp_websocket_client_stop(c->handle);
        esp_websocket_client_destroy(c->handle);
        c->handle = NULL;
    }
    c->up = false;
}

static void ws_start(ws_ctx_t *c)
{
    if (c->handle) return;
    char uri[160];
    snprintf(uri, sizeof uri, "%s://%s:%d%s",
#if CONFIG_MRRC_TLS
             "wss",
#else
             "ws",
#endif
             CONFIG_MRRC_SERVER_HOST, CONFIG_MRRC_SERVER_PORT, c->path);

    if (!c->acc) {
        c->acc = board_psram_alloc(FRAME_MAX);
        if (!c->acc) return;
    }

    esp_websocket_client_config_t cfg = {
        .uri                  = uri,
        .headers              = s_hdr,
        .buffer_size          = WS_BUF_SIZE,
        .task_stack           = WS_TASK_STACK,
        .reconnect_timeout_ms = 3000,
        .network_timeout_ms   = 10000,
        .ping_interval_sec    = 15,
        .pingpong_timeout_sec = 25,
    };
#ifdef CONFIG_MRRC_TLS
#ifdef CONFIG_MRRC_TLS_INSECURE
    cfg.skip_cert_common_name_check = true;
#else
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
#endif
#endif

    c->handle = esp_websocket_client_init(&cfg);
    if (!c->handle) {
        ESP_LOGE(TAG, "%s init failed", c->name);
        return;
    }
    esp_websocket_register_events(c->handle, WEBSOCKET_EVENT_ANY,
                                  ws_event_handler, c);
    esp_websocket_client_start(c->handle);
    ESP_LOGI(TAG, "%s -> %s", c->name, uri);
}

/* ── audio tasks ─────────────────────────────────────────────────── */

static int s16_apply_gain(int16_t s, int gain_x100)
{
    int32_t v = (int32_t)s * gain_x100 / 100;
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (int)v;
}

/*
 * ONE task owns both Opus directions.
 *
 * Why: ESP32-S3's GCC silently turns `_Thread_local` into a plain global (no
 * .tbss section is emitted), so micro-opus's "thread-safe pseudostack" is not
 * actually per-thread.  Two tasks calling into libopus corrupt
 * global_stack/scratch_ptr and panic with "Cache disabled but cached memory
 * region accessed".  Serialising them costs nothing: PTT is half duplex, so
 * decode and encode never have to run at the same time.
 */
static void audio_task(void *arg)
{
    static uint8_t frame[FRAME_MAX];
    static int16_t pcm[5760];
    static int16_t mic[1152];
    static uint8_t out[1600];

#if CONFIG_MRRC_AUDIO_PCM
    const int tx_rate = 48000;
#else
    const int tx_rate = 48000;      /* encode at 48 kHz: same path as the browser,
                                     * which is known-good on this server */
#endif
    const int tx_frame = tx_rate / 50;               /* 20 ms */
    const int mic_frame = 320;                       /* 20 ms @ 16 kHz from the mic */
    /* Local speaker volume, changed live by the VOL knob (never the radio's
     * AF gain — that one stays a server-side setting). */
    int spk_gain = g_app.spk_vol > 0 ? g_app.spk_vol : CONFIG_MRRC_SPK_VOLUME;
    const int mic_gain = CONFIG_MRRC_MIC_GAIN;

#if CONFIG_MRRC_AUDIO_OPUS
    int err = 0;
    OpusDecoder *dec = opus_decoder_create(48000, 1, &err);
    if (err != OPUS_OK) {
        ESP_LOGE(TAG, "opus decoder create failed (%d)", err);
        dec = NULL;
    }
    err = 0;
    OpusEncoder *enc = opus_encoder_create(tx_rate, 1, OPUS_APPLICATION_VOIP, &err);
    if (err != OPUS_OK) {
        ESP_LOGE(TAG, "opus encoder create failed (%d)", err);
        enc = NULL;
    } else {
        opus_encoder_ctl(enc, OPUS_SET_BITRATE(CONFIG_MRRC_TX_BITRATE > 20000 ? CONFIG_MRRC_TX_BITRATE : 24000));
        opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(3));
        opus_encoder_ctl(enc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
        opus_encoder_ctl(enc, OPUS_SET_INBAND_FEC(1));
        opus_encoder_ctl(enc, OPUS_SET_PACKET_LOSS_PERC(10));
    }
    ESP_LOGI(TAG, "audio up: spk 48k, tx %d Hz, %d samples/20ms, opus %s",
             tx_rate, tx_frame, enc ? "on" : "FAILED");
#else
    ESP_LOGI(TAG, "audio up: spk 48k, tx %d Hz raw PCM (tag 0x00)", tx_rate);
#endif

    bool tx_prev = false;

    while (1) {
        bool tx = s_tx_active && s_tx.up;

        /* ── transmit ───────────────────────────────────────────── */
        if (tx) {
            if (!tx_prev) {
                tx_prev = true;
                /* The server keys the radio (a CAT write) and then reopens its
                 * TX stream, clearing the queue — do not race that. */
                vTaskDelay(pdMS_TO_TICKS(200));
                { extern void board_mic_dump_once(void); board_mic_dump_once(); }
                board_mic_read(mic, (size_t)tx_frame);   /* drop pre-key audio */
                board_mic_read(mic, (size_t)tx_frame);
                while (rb_get_frame(frame, sizeof frame, 0) > 0) { }  /* drop stale RX */
            }
            size_t got = board_mic_take(mic, (size_t)tx_frame);
            if (got < (size_t)tx_frame) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
            int peak = 0;
            for (int i = 0; i < tx_frame; i++) {
                int a = mic[i] < 0 ? -mic[i] : mic[i];
                if (a > peak) peak = a;
                mic[i] = (int16_t)s16_apply_gain(mic[i], mic_gain);
            }
            {
                static int n, acc;
                if (peak > acc) acc = peak;
                if (++n >= 50) {       /* ~1 s while keyed */
                    extern radio_state_t *mrrc_state_ptr(void);
                    extern int32_t board_mic_raw_peak(void);
                    radio_state_t *rst = mrrc_state_ptr();
                    ESP_LOGI(TAG, "tx mic peak %d%% raw %ld pwr=%dW alc=%d%% "
                             "txst=%d", acc * 100 / 32768,
                             (long)board_mic_raw_peak(), rst->power_w,
                             (int)(rst->alc_pct + 0.5f), rst->tx_status);
                    n = 0;
                    acc = 0;
                }
            }
#if CONFIG_MRRC_AUDIO_OPUS
            if (enc) {
                int bytes = opus_encode(enc, mic, tx_frame, out + 1,
                                        (opus_int32)(sizeof(out) - 1));
                if (bytes > 0) {
                    out[0] = 0x01;
                    mrrc_tx_frame(out, (size_t)bytes + 1);
                    s_st_tx_frames++;
                    s_st_tx_bytes += (uint32_t)bytes + 1;
                }
            }
#else
            out[0] = 0x00;
            memcpy(out + 1, mic, (size_t)tx_frame * 2);
            mrrc_tx_frame(out, (size_t)tx_frame * 2 + 1);
            s_st_tx_frames++;
            s_st_tx_bytes += (uint32_t)tx_frame * 2 + 1;
#endif
            continue;
        }

        if (tx_prev) {                 /* TX -> RX: the buffer holds seconds of
                                        * audio that is already in the past */
            tx_prev = false;
            while (rb_get_frame(frame, sizeof frame, 0) > 0) { }
            continue;
        }

        /* ── receive ────────────────────────────────────────────── */
        spk_gain = g_app.spk_vol > 0 ? g_app.spk_vol : CONFIG_MRRC_SPK_VOLUME;

        static bool primed;
        if (!primed) {
            vTaskDelay(pdMS_TO_TICKS(150));      /* let a jitter backlog build */
            primed = true;
        }
        size_t len = rb_get_frame(frame, sizeof frame, pdMS_TO_TICKS(30));
        if (len < 2) {
            primed = false;                      /* underrun: re-prime next time */
            continue;
        }
        uint8_t tag = frame[0];
        const uint8_t *payload = frame + 1;
        size_t plen = len - 1;

        if (tag == 0x00) {
            size_t n = plen / 2;
            if (n > 5760) n = 5760;
            memcpy(pcm, payload, n * 2);
            for (size_t i = 0; i < n; i++) {
                pcm[i] = (int16_t)s16_apply_gain(pcm[i], spk_gain);
            }
            board_spk_write(pcm, n);
            s_st_rx_frames++;
        }
#if CONFIG_MRRC_AUDIO_OPUS
        else if (tag == 0x01 && dec) {
            int samples = opus_decode(dec, payload, (opus_int32)plen, pcm, 5760, 0);
            if (samples > 0) {
                for (int i = 0; i < samples; i++) {
                    pcm[i] = (int16_t)s16_apply_gain(pcm[i], spk_gain);
                }
                board_spk_write(pcm, (size_t)samples);
                s_st_rx_frames++;
            } else {
                s_st_rx_err++;
                static uint32_t bad;
                if ((bad++ % 50) == 0) ESP_LOGW(TAG, "opus decode err %d", samples);
            }
        }
#endif
        else {
            static uint32_t unk;
            if ((unk++ % 100) == 0) {
                ESP_LOGW(TAG, "RX frame tag 0x%02x len %u unsupported", tag,
                         (unsigned)plen);
            }
        }
    }
}

/* TX-phase heartbeat.  This MUST NOT live in the application loop: while
 * Opus is encoding at 50 fps on core 0 the app task is starved for seconds and
 * the server's liveness gate (5 s) force-releases PTT with
 * "Link lost while transmitting".  A dedicated high-priority task always runs. */
static void tx_hb_task(void *arg)
{
    int64_t last = 0;
    while (1) {
        bool keying = (g_app.ptt == PTT_KEYING);
        int64_t now = esp_timer_get_time() / 1000;
        if (keying && now - last >= 1500) {
            last = now;
            mrrc_send_txhb();
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ── supervisor ──────────────────────────────────────────────────── */

static void net_task(void *arg)
{
    wifi_start();
    int64_t last_rssi_ms = 0;
    int64_t ctrl_down_since = 0;

    while (1) {
        int64_t now = esp_timer_get_time() / 1000;

        g_app.wifi_up = s_wifi_up;
        g_app.ctrl_up = s_ctrl.up;

        if (s_wifi_up && now - last_rssi_ms > 5000) {
            last_rssi_ms = now;
            wifi_ap_record_t ap;
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                g_app.rssi = ap.rssi;
            }
        }

        if (!s_wifi_up) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!s_token[0]) {
            if (!mrrc_login()) {
                vTaskDelay(pdMS_TO_TICKS(10000));
                continue;
            }
        }

        /* Stagger the handshakes: four simultaneous mbedtls
         * handshakes spike peak heap and trigger spurious cert failures. */
        ws_start(&s_ctrl);
        vTaskDelay(pdMS_TO_TICKS(250));
        ws_start(&s_spec);
        vTaskDelay(pdMS_TO_TICKS(250));
        ws_start(&s_rx);
        vTaskDelay(pdMS_TO_TICKS(250));
        ws_start(&s_tx);

        if (s_ctrl.up && s_want_ctrl_hello) {
            s_want_ctrl_hello = false;
            mrrc_send_txhb();           /* declare TX heartbeat capability */
        }
        if (s_spec.up && s_want_spec_caps) {
            s_want_spec_caps = false;
            char b[80];
            snprintf(b, sizeof b, "{\"type\":\"spectrumCaps\",\"profile\":\"%s\"}",
                     CONFIG_MRRC_SPECTRUM_PROFILE);
            ws_send_text(&s_spec, b);
            ESP_LOGI(TAG, "spectrum profile '%s' declared",
                     CONFIG_MRRC_SPECTRUM_PROFILE);
        }

        if (s_ctrl.up) {
            ctrl_down_since = 0;
        } else if (ctrl_down_since == 0) {
            ctrl_down_since = now;
        } else if (now - ctrl_down_since > 15000) {
            /* The socket layer gave up (bad/expired token, wrong URL).
             * Throw everything away and re-authenticate. */
            ESP_LOGW(TAG, "control socket down 15 s — re-authenticating");
            ws_stop(&s_ctrl);
            ws_stop(&s_spec);
            ws_stop(&s_rx);
            ws_stop(&s_tx);
            s_token[0] = 0;
            ctrl_down_since = 0;
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ── init ────────────────────────────────────────────────────────── */

void mrrc_net_start(void)
{
    s_rb = board_psram_alloc(RB_BYTES);
    s_rb_mtx = xSemaphoreCreateMutex();
    s_rb_data = xSemaphoreCreateCounting(1, 0);
    if (!s_rb || !s_rb_mtx || !s_rb_data) {
        ESP_LOGE(TAG, "ring buffer allocation failed");
        return;
    }
    xTaskCreatePinnedToCore(audio_task, "audio", 12288, NULL, 8, NULL, 1);
    /* Above the audio task so the heartbeat survives Opus load. */
    xTaskCreatePinnedToCore(tx_hb_task, "txhb", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(net_task, "net", 8192, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "server %s:%d, mic %d Hz, opus %s",
             CONFIG_MRRC_SERVER_HOST, CONFIG_MRRC_SERVER_PORT,
             CONFIG_MRRC_MIC_RATE,
#ifdef CONFIG_MRRC_AUDIO_OPUS
             "on");
#else
             "off (raw PCM)");
#endif
}
