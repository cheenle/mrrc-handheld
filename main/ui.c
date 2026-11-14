/*
 * UI — ICOM-handheld-style layout on a 240x240 RGB565 framebuffer.
 * No LVGL: fixed layout, known dirty regions, hand-drawn chrome + a baked
 * anti-aliased font (tools/gen_font.py).
 *
 *   y   0.. 21  chrome:  [LSB]              VFO A  87%  wifi
 *   y  40.. 80  frequency  "7.079.51"  (30 px numerals, centred)
 *   y  82.. 94  info:    S9+20        FIL 2.4K     SWR 1.1
 *   y  96..107  scope label + span
 *   y 108..164  spectrum (filled, blue gradient, bright trace, centre mark)
 *   y 166..206  waterfall (40 rows)
 *   y 208..219  frequency scale ticks
 *   y 222..238  chips:   [FREQ]  [RX]  [87%]
 */
#include "ui.h"
#include "app.h"
#include "board.h"
#include "ui_font.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ui";

#define FB_W 240
#define FB_H 240

/* ── palette ─────────────────────────────────────────────────────── */
#define C_BG       0x0000
#define C_CHROME   0x0882       /* navy chrome */
#define C_CHROME_D 0x0420
#define C_BADGE    0x1ACD       /* accent blue (LSB badge) */
#define C_TEXT     0xFFFF
#define C_DIM      0x9CF3
#define C_DIMMER   0x4208
#define C_CYAN     0x07FF
#define C_AMBER    0xFD20
#define C_RED      0xF800
#define C_GREEN    0x07E0
#define C_BLUEHI   0x2D7F

/* ── layout ─────────────────────────────────────────────────────── */
#define Y_HDR       0
#define H_HDR      22
#define H_BADGE    15            /* badge lives INSIDE the header now */
#define Y_FREQ     36            /* em-box top; numerals land ~45..75 */
#define Y_INFO     80
#define Y_LBL      95
#define Y_PLOT    108
#define H_PLOT     56
#define Y_WF      166
#define H_WF       40
#define Y_SCALE   208
#define Y_CHIPS   222
#define H_CHIPS    16

static uint16_t *s_fb;
static uint8_t  *s_wf;           /* [H_WF][FB_W] ring, PSRAM */
static int       s_wf_head;
static uint16_t  s_heat[64];

/* ── primitives ──────────────────────────────────────────────────── */

static void fill(int x, int y, int w, int h, uint16_t c)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > FB_W) w = FB_W - x;
    if (y + h > FB_H) h = FB_H - y;
    if (w <= 0 || h <= 0) return;
    for (int r = 0; r < h; r++) {
        uint16_t *line = s_fb + (y + r) * FB_W + x;
        for (int i = 0; i < w; i++) line[i] = c;
    }
}

static void rrect(int x, int y, int w, int h, uint16_t c)
{
    if (w < 3 || h < 3) { fill(x, y, w, h, c); return; }
    fill(x + 1, y, w - 2, h, c);
    fill(x, y + 1, w, h - 2, c);
}

static void txt(const font_t *f, int x, int y, const char *s, uint16_t c)
{
    font_draw(f, s_fb, FB_W, FB_H, x, y, s, c);
}

static void txt_c(const font_t *f, int y, const char *s, uint16_t c)
{
    txt(f, (FB_W - font_width(f, s)) / 2, y, s, c);
}

static void txt_r(const font_t *f, int x_right, int y, const char *s, uint16_t c)
{
    txt(f, x_right - font_width(f, s), y, s, c);
}

/* ── spectrum helpers ────────────────────────────────────────────── */

static void heat_init(void)
{
    for (int i = 0; i < 64; i++) {
        float t = i / 63.0f;
        int r, g, b;
        if (t < 0.30f)      { r = 0;   g = 4;   b = 16 + (int)(74 * t / 0.30f); }
        else if (t < 0.55f) { r = 0;   g = 30 + (int)(200 * (t - 0.30f) / 0.25f); b = 90 + (int)(165 * (t - 0.30f) / 0.25f); }
        else if (t < 0.80f) { r = 0;   g = 220; b = (int)(255 * (0.80f - t) / 0.25f); }
        else                { r = (int)(255 * (t - 0.80f) / 0.20f); g = 255; b = 60; }
        s_heat[i] = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
    }
}

static void bins_to_cols(const uint8_t *bins, uint8_t *cols)
{
    for (int x = 0; x < FB_W; x++) {
        int a = (x * MRRC_BINS) / FB_W;
        int b = ((x + 1) * MRRC_BINS) / FB_W;
        if (b <= a) b = a + 1;
        uint8_t m = 0;
        for (int i = a; i < b && i < MRRC_BINS; i++) {
            if (bins[i] > m) m = bins[i];
        }
        cols[x] = m;
    }
}

/* The FT-710 scope bins arrive with a tall noise floor (typically 120-150 of
 * 255).  Dividing by the peak alone turns the whole band into one solid block,
 * so subtract a tracked floor first: the trace then sits on the baseline and
 * only real carriers climb. */
static int s_peak = 96;
static int s_floor = 40;

static void cols_normalize(const uint8_t *cols, uint8_t *idx)
{
    int mx = 1, mn = 255;
    for (int x = 0; x < FB_W; x++) {
        if (cols[x] > mx) mx = cols[x];
        if (cols[x] < mn) mn = cols[x];
    }

    if (mx > s_peak) s_peak = mx;                  /* fast attack */
    else s_peak -= (s_peak - mx) / 5;              /* slow release */
    if (s_peak < 24) s_peak = 24;

    if (mn > s_floor) s_floor += (mn - s_floor + 3) / 4;   /* rise slowly */
    else s_floor -= (s_floor - mn) / 4;                    /* fall slowly */
    if (s_floor < 0) s_floor = 0;

    int range = s_peak - s_floor;
    if (range < 24) range = 24;

    for (int x = 0; x < FB_W; x++) {
        /* map to 4..63 so the noise floor stays a visible dark blue instead
         * of pure black — a black waterfall reads as "no data" */
        int v = ((int)cols[x] - s_floor) * 59 / range + 4;
        idx[x] = (uint8_t)(v < 4 ? 4 : (v > 63 ? 63 : v));
    }
}

static uint16_t dim565(uint16_t c, int pct)
{
    int r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, b = c & 0x1F;
    r = r * pct / 100;
    g = g * pct / 100;
    b = b * pct / 100;
    return (uint16_t)((r << 11) | (g << 5) | b);
}

/* ── chrome ──────────────────────────────────────────────────────── */

static void draw_wifi_icon(int x, int y, int rssi, uint16_t c)
{
    int bars = rssi > -55 ? 4 : rssi > -67 ? 3 : rssi > -75 ? 2 : 1;
    for (int i = 0; i < 4; i++) {
        int h = 2 + i * 2;
        fill(x + i * 3, y + 7 - h, 2, h, i < bars ? c : C_DIMMER);
    }
}

static void draw_header(const radio_state_t *st, const app_shared_t *a)
{
    fill(0, Y_HDR, FB_W, H_HDR, C_CHROME);
    fill(0, Y_HDR + H_HDR - 1, FB_W, 1, C_BADGE);       /* accent rule */

    /* mode badge, inside the chrome strip */
    const char *mode = st->mode_name[0] ? st->mode_name : "---";
    int mw = font_width(&font_mid, mode) + 12;
    rrect(4, 3, mw, H_BADGE, C_BADGE);
    txt(&font_mid, 4 + 6, 4, mode, C_TEXT);

    /* right cluster: VFO, battery, wifi — no logo, no band name */
    int x = FB_W - 4;
    draw_wifi_icon(x - 13, 7, a->rssi, a->wifi_up ? C_GREEN : C_DIMMER);
    x -= 18;

    if (st->battery_pct >= 0) {
        char b[12];
        snprintf(b, sizeof b, "%d%%", st->battery_pct);
        x -= font_width(&font_small, b);
        txt(&font_small, x, 4, b, st->charging ? C_GREEN : C_DIM);
        x -= 6;
    }

    char vfo[8];
    snprintf(vfo, sizeof vfo, "VFO %c", st->vfo ? st->vfo : 'A');
    x -= font_width(&font_tiny, vfo);
    txt(&font_tiny, x, 7, vfo, C_DIM);
}

static void draw_frequency(const radio_state_t *st)
{
    int32_t hz = st->freq_hz;
    if (hz < 0) hz = 0;
    char b[24];
    snprintf(b, sizeof b, "%ld.%03ld.%02ld",
             (long)(hz / 1000000), (long)((hz / 1000) % 1000), (long)((hz % 1000) / 10));

    uint16_t c = (st->tx_status > 0) ? C_RED : C_TEXT;
    txt(&font_big, (FB_W - font_width(&font_big, b)) / 2, Y_FREQ, b, c);
}

static void draw_info(const radio_state_t *st, const app_shared_t *a)
{
    fill(0, Y_INFO, FB_W, Y_LBL - Y_INFO - 1, C_BG);

    char l[16];
    if (st->tx_status > 0) {
        snprintf(l, sizeof l, "SWR %.1f", st->swr);
        txt(&font_small, 6, Y_INFO, l, st->swr > 2.0f ? C_RED : C_TEXT);
        snprintf(l, sizeof l, "%dW", st->power_w);
        txt_c(&font_small, Y_INFO, l, C_AMBER);
        snprintf(l, sizeof l, "ALC %d%%", (int)(st->alc_pct + 0.5f));
        txt_r(&font_small, FB_W - 6, Y_INFO, l, C_CYAN);
        return;
    }

    {
        char u[16];
        if (st->s_unit[0] == '+') snprintf(u, sizeof u, "S9%s", st->s_unit);
        else snprintf(u, sizeof u, "%s", st->s_unit[0] ? st->s_unit : "-");
        txt(&font_small, 6, Y_INFO, u, C_AMBER);
    }

    if (st->filter_hz > 0) {
        snprintf(l, sizeof l, "FIL %d.%dK", st->filter_hz / 1000,
                 (st->filter_hz % 1000) / 100);
        txt_c(&font_small, Y_INFO, l, C_DIM);
    }

    char r[16];
    if (a->ui_mode == UI_MODE_VOL) {
        snprintf(r, sizeof r, "SPK %d%%", a->spk_vol);
        txt_r(&font_small, FB_W - 6, Y_INFO, r, C_CYAN);
    } else {
        /* the radio's own AF gain — read-only here, it is a server setting */
        snprintf(r, sizeof r, "AF %d", st->af_gain);
        txt_r(&font_small, FB_W - 6, Y_INFO, r, C_DIM);
    }
}

/* ── scope ───────────────────────────────────────────────────────── */

static const int32_t k_spans_hz[10] = {
    1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000
};

static int32_t span_hz(const radio_state_t *st)
{
    int i = st->scope_span;
    if (i < 0 || i > 9) i = 6;          /* default 100 kHz */
    return k_spans_hz[i];
}

static void fmt_mhz(char *out, size_t n, int32_t hz)
{
    if (hz < 0) hz = 0;
    snprintf(out, n, "%ld.%03ld", (long)(hz / 1000000), (long)((hz / 1000) % 1000));
}

static void draw_scope_label(const radio_state_t *st)
{
    fill(0, Y_LBL, FB_W, Y_PLOT - Y_LBL, C_BG);
    txt(&font_tiny, 6, Y_LBL + 1, "SPECTRUM SCOPE", C_DIM);
    char b[16];
    int32_t sp = span_hz(st);
    if (sp >= 1000000) snprintf(b, sizeof b, "%ldM", (long)(sp / 1000000));
    else snprintf(b, sizeof b, "%ldK", (long)(sp / 1000));
    txt_r(&font_tiny, FB_W - 6, Y_LBL + 1, b, C_DIM);
}

static void draw_plot(const uint8_t *idx, bool tx)
{
    for (int x = 0; x < FB_W; x++) {
        int h = (int)idx[x] * (H_PLOT - 2) / 63;
        uint16_t c = dim565(s_heat[idx[x]], 55);
        uint16_t *base = s_fb + (Y_PLOT + H_PLOT - 1) * FB_W + x;
        for (int i = 0; i < H_PLOT; i++) base[-i * FB_W] = (i < h) ? c : C_BG;
        if (h > 0) {
            base[-h * FB_W] = tx ? C_RED : C_TEXT;          /* the trace */
            if (h > 1) base[-(h - 1) * FB_W] = tx ? C_RED : C_CYAN;
        }
    }
    /* centre marker (the FT-710 scope is centre-mode) */
    for (int y = Y_PLOT; y < Y_PLOT + H_PLOT; y += 4) {
        s_fb[y * FB_W + FB_W / 2] = C_DIMMER;
    }
    fill(0, Y_PLOT + H_PLOT, FB_W, 1, C_CHROME_D);
}

static void draw_waterfall(const uint8_t *idx)
{
    s_wf_head = (s_wf_head + 1) % H_WF;
    memcpy(s_wf + s_wf_head * FB_W, idx, FB_W);

    for (int r = 0; r < H_WF; r++) {
        int k = (s_wf_head - r + H_WF) % H_WF;
        const uint8_t *src = s_wf + k * FB_W;
        uint16_t *dst = s_fb + (Y_WF + r) * FB_W;
        for (int x = 0; x < FB_W; x++) dst[x] = s_heat[src[x]];
    }
}

static void draw_scale(const radio_state_t *st)
{
    fill(0, Y_SCALE, FB_W, Y_CHIPS - Y_SCALE, C_BG);
    int32_t half = span_hz(st) / 2;
    char b[16];

    fmt_mhz(b, sizeof b, st->freq_hz - half);
    txt(&font_tiny, 4, Y_SCALE, b, C_DIMMER);

    fmt_mhz(b, sizeof b, st->freq_hz);
    txt_c(&font_tiny, Y_SCALE, b, C_DIM);

    fmt_mhz(b, sizeof b, st->freq_hz + half);
    txt_r(&font_tiny, FB_W - 4, Y_SCALE, b, C_DIMMER);
}

static void draw_chips(const radio_state_t *st, const app_shared_t *a, int64_t now_ms)
{
    fill(0, Y_CHIPS, FB_W, FB_H - Y_CHIPS, C_BG);

    /* 1) tuning mode */
    const char *m = (a->ui_mode == UI_MODE_VOL) ? "VOL" : "FREQ";
    uint16_t mc = (a->ui_mode == UI_MODE_VOL) ? C_CYAN : C_AMBER;
    int w = font_width(&font_small, m) + 14;
    rrect(4, Y_CHIPS, w, H_CHIPS, C_CHROME);
    txt(&font_small, 4 + 7, Y_CHIPS + 2, m, mc);

    /* 2) TX / RX */
    char t[16];
    uint16_t tc = C_GREEN;
    if (st->tx_status > 0) {
        int s = (int)((now_ms - a->ptt_since_ms) / 1000);
        if (s < 0) s = 0;
        snprintf(t, sizeof t, "TX %d", s);
        tc = C_RED;
    } else {
        snprintf(t, sizeof t, "%s", "RX");
    }
    int tw = font_width(&font_small, t) + 14;
    rrect((FB_W - tw) / 2, Y_CHIPS, tw, H_CHIPS, st->tx_status > 0 ? C_RED : C_CHROME);
    txt(&font_small, (FB_W - tw) / 2 + 7, Y_CHIPS + 2, t,
        st->tx_status > 0 ? C_TEXT : C_GREEN);

    /* 3) link */
    char l[12];
    snprintf(l, sizeof l, "%s", a->ctrl_up ? "LINK" : "noSRV");
    int lw = font_width(&font_small, l) + 14;
    rrect(FB_W - lw - 4, Y_CHIPS, lw, H_CHIPS, C_CHROME);
    txt(&font_small, FB_W - lw + 3, Y_CHIPS + 2, l,
        a->ctrl_up ? C_GREEN : C_RED);
    (void)tc;
}

/* ── toast ───────────────────────────────────────────────────────── */

static void draw_toast(const char *msg)
{
    if (!msg[0]) return;
    int w = font_width(&font_small, msg) + 10;
    int x = (FB_W - w) / 2;
    if (x < 0) { x = 0; w = FB_W; }
    fill(x, Y_LBL, w, 13, C_RED);
    txt(&font_small, x + 5, Y_LBL + 1, msg, C_TEXT);
}

/* ── task ────────────────────────────────────────────────────────── */

#define UI_FPS (CONFIG_MRRC_UI_FPS > 0 ? CONFIG_MRRC_UI_FPS : 12)

static void ui_task(void *arg)
{
    uint32_t last_st = 0xFFFFFFFF, last_spec = 0xFFFFFFFF, last_toast = 0;
    uint32_t last_meter = 0xFFFFFFFF;
    int64_t last_frame_us = 0, last_meter_us = 0, last_tx_redraw_ms = 0;
    int64_t last_full_ms = 0;
    int64_t toast_until_ms = 0;
    char toast[sizeof g_app.toast] = {0};
    radio_state_t st;

    memset(&st, 0, sizeof st);
    st.battery_pct = -1;
    strcpy(st.mode_name, "---");
    strcpy(st.band_name, "--");

    fill(0, 0, FB_W, FB_H, C_BG);
    board_display_flush(0, 0, FB_W, FB_H);

    while (1) {
        app_shared_t snap;
        app_lock();
        snap = g_app;
        app_unlock();

        int64_t now_us = esp_timer_get_time();
        int64_t now_ms = now_us / 1000;
        bool dirty = false;

        /* A full repaint every 10 s is a cheap safety net: it guarantees the
         * screen can never be left showing a stale value forever. */
        if (snap.st_seq != last_st || now_ms - last_full_ms >= 10000) {
            last_st = snap.st_seq;
            last_full_ms = now_ms;
            st = snap.st;
            draw_header(&st, &snap);
            fill(0, Y_FREQ, FB_W, Y_INFO - Y_FREQ, C_BG);
            draw_frequency(&st);
            draw_info(&st, &snap);
            draw_chips(&st, &snap, now_ms);
            board_display_flush(0, 0, FB_W, Y_LBL);
            board_display_flush(0, Y_CHIPS, FB_W, FB_H - Y_CHIPS);
            dirty = true;
        } else if (snap.meter_seq != last_meter && now_us - last_meter_us >= 60000) {
            last_meter = snap.meter_seq;
            last_meter_us = now_us;
            st = snap.st;          /* the meter lives in the fresh snapshot */
            draw_info(&st, &snap);
            board_display_flush(0, Y_INFO, FB_W, Y_LBL - Y_INFO);
        } else if (st.tx_status > 0 && now_ms - last_tx_redraw_ms >= 1000) {
            last_tx_redraw_ms = now_ms;
            draw_chips(&st, &snap, now_ms);
            board_display_flush(0, Y_CHIPS, FB_W, FB_H - Y_CHIPS);
        }

        if (snap.toast_seq != last_toast) {
            last_toast = snap.toast_seq;
            strncpy(toast, snap.toast, sizeof toast - 1);
            toast[sizeof toast - 1] = 0;
            toast_until_ms = now_ms + 3000;
            dirty = true;
        }
        if (toast[0] && now_ms > toast_until_ms) {
            toast[0] = 0;
            dirty = true;
        }

        if (snap.spec_seq != last_spec) {
            int64_t gap = 1000000 / UI_FPS;
            if (now_us - last_frame_us >= gap) {
                last_spec = snap.spec_seq;
                last_frame_us = now_us;
                uint8_t cols[FB_W], idx[FB_W];
                bins_to_cols(snap.spec, cols);
                cols_normalize(cols, idx);
                draw_scope_label(&st);
                draw_plot(idx, st.tx_status > 0);
                draw_waterfall(idx);
                draw_scale(&st);
                if (toast[0]) draw_toast(toast);
                board_display_flush(0, Y_LBL, FB_W, Y_CHIPS - Y_LBL);
            }
        } else if (dirty && toast[0]) {
            draw_toast(toast);
            board_display_flush(0, Y_LBL, FB_W, 13);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ui_start(void)
{
    s_fb = board_display_fb();
    s_wf = board_psram_alloc(H_WF * FB_W);
    if (!s_fb || !s_wf) {
        ESP_LOGE(TAG, "UI buffers missing (fb=%p wf=%p)", s_fb, s_wf);
        return;
    }
    memset(s_wf, 0, H_WF * FB_W);
    heat_init();
    xTaskCreatePinnedToCore(ui_task, "ui", 6144, NULL, 4, NULL, 1);
    ESP_LOGI(TAG, "UI up: ICOM layout, scope %dx%d, waterfall %d rows",
             FB_W, H_PLOT, H_WF);
}
