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

#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ui";

/* UI-task liveness.  Bumped every ui_task loop iteration; the network
 * supervisor watches it and restarts the board if the UI task wedges inside
 * an SPI/lcd call (the one thing the task watchdog is not subscribed to). */
volatile uint32_t g_ui_heartbeat;
/* TEMP-DIAG: which blocking region the ui task is in when it wedges. */
volatile int g_ui_phase;

#define FB_W 240
#define FB_H 240

/* ── palette ─────────────────────────────────────────────────────────
 * Frequency readout is white; every other label/chrome is the web UI's amber
 * accent, so the handheld and the browser look like one product.  The scope
 * colours are NOT from this table: they come from the "Jet" palette below,
 * which is a port of static/ft710_ui.js WF_PALETTES.jet. */
#define C_BG       0x0000
#define C_PANEL    0x18E3       /* #1e1e1e dark chrome, as the web settings panel */
#define C_PANEL_D  0x0861
/* Top and bottom strips are one bar of ONE colour from edge to edge — the
 * separate badge/chip boxes read as coloured blobs on this panel. */
#define C_BAR      0x2965       /* #2a2a2a */
#define C_ORANGE   0xFD20       /* #ffa500 — web accent, solid */
/* "Semi-transparent orange": RGB565 has no alpha, so a dim orange over black is
 * what the eye reads as a translucent field behind the text.  Text on top of it
 * is white — orange-on-black at 8-10 px was unreadable. */
#define C_FIELD    0x39E7       /* #3a3a3a — text field background */
#define C_FIELD_D  0x2104       /* #212121 — dimmer field */
#define C_ORANGE_D 0xB901       /* bright orange, for small accents only */
#define C_ORANGE_X 0x4200       /* ~25 % orange, barely-there marks */
#define C_TEXT     0xFFFF       /* white — every glyph */
#define C_RED      0xF800
#define C_GREEN    0x07E0
#define C_BLUE     0x3C1E       /* #3b82f6 — the spectrum trace: far more legible
                                 * against the dark plot than cyan-on-black */

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

/* "Jet" — ported verbatim from the web client (static/ft710_ui.js,
 * WF_PALETTES.jet).  64 steps is what the 6-bit colour index can address.
 * v=0 is the palette's dark blue floor, not black: a black waterfall reads as
 * "no data". */
static void heat_init(void)
{
    for (int i = 0; i < 64; i++) {
        float v = i / 63.0f;
        int r, g, b;
        if (v < 0.125f) {
            float u = v / 0.125f;
            r = 0; g = 0; b = (int)(128 + u * 127);
        } else if (v < 0.375f) {
            float u = (v - 0.125f) / 0.25f;
            r = 0; g = (int)(u * 255); b = 255;
        } else if (v < 0.625f) {
            float u = (v - 0.375f) / 0.25f;
            r = (int)(u * 255); g = 255; b = (int)(255 * (1 - u));
        } else if (v < 0.875f) {
            float u = (v - 0.625f) / 0.25f;
            r = 255; g = (int)(255 * (1 - u)); b = 0;
        } else {
            float u = (v - 0.875f) / 0.125f;
            r = (int)(255 * (1 - u * 0.5f)); g = 0; b = 0;
        }
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

/* The web client maps the raw 0..255 bins through a fixed floor/ceiling from
 * its scope settings (Floor 71 / Ceil 233 by default, both adjustable there).
 * Doing the same here means the handheld and the browser show the same picture:
 * no adaptive AGC on the scope data, which is what made the trace jump when a
 * single carrier appeared. */
#define SCOPE_FLOOR CONFIG_MRRC_SCOPE_FLOOR
#define SCOPE_CEIL  CONFIG_MRRC_SCOPE_CEIL

static void cols_to_idx(const uint8_t *cols, uint8_t *idx)
{
    int range = SCOPE_CEIL - SCOPE_FLOOR;
    if (range < 1) range = 1;
    for (int x = 0; x < FB_W; x++) {
        int v = ((int)cols[x] - SCOPE_FLOOR) * 63 / range;
        idx[x] = (uint8_t)(v < 0 ? 0 : (v > 63 ? 63 : v));
    }
}

/* Web FFT plot: 30 % new data per frame, then a 2x amplitude boost. */
#define FFT_ALPHA 0.30f
#define FFT_BOOST 2.0f
static float s_fft[FB_W];
static bool  s_fft_primed;

static void fft_update(const uint8_t *cols)
{
    for (int x = 0; x < FB_W; x++) {
        if (!s_fft_primed) s_fft[x] = (float)cols[x];
        else s_fft[x] = s_fft[x] * (1.0f - FFT_ALPHA) + (float)cols[x] * FFT_ALPHA;
    }
    s_fft_primed = true;
}


/* ── chrome ──────────────────────────────────────────────────────── */

static void draw_wifi_icon(int x, int y, int rssi, uint16_t c)
{
    int bars = rssi > -55 ? 4 : rssi > -67 ? 3 : rssi > -75 ? 2 : 1;
    for (int i = 0; i < 4; i++) {
        int h = 2 + i * 2;
        fill(x + i * 3, y + 7 - h, 2, h, i < bars ? c : C_ORANGE_X);
    }
}

static void draw_header(const radio_state_t *st, const app_shared_t *a)
{
    /* one bar, one colour, white glyphs — the mode sits on the same fill */
    fill(0, Y_HDR, FB_W, H_HDR, C_BAR);
    fill(0, Y_HDR + H_HDR - 1, FB_W, 1, C_ORANGE);       /* accent rule */

    const char *mode = st->mode_name[0] ? st->mode_name : "---";
    txt(&font_small, 6, 3, mode, C_TEXT);

    int x = FB_W - 5;
    draw_wifi_icon(x - 12, 7, a->rssi, a->wifi_up ? C_GREEN : C_PANEL_D);
    x -= 17;

    if (st->battery_pct >= 0) {
        char b[12];
        snprintf(b, sizeof b, "%d%%", st->battery_pct);
        x -= font_width(&font_small, b);
        txt(&font_small, x, 3, b, st->charging ? C_GREEN : C_TEXT);
        x -= 8;
    }

    char vfo[8];
    snprintf(vfo, sizeof vfo, "VFO %c", st->vfo ? st->vfo : 'A');
    x -= font_width(&font_small, vfo);
    txt(&font_small, x, 3, vfo, C_TEXT);
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

/* A field behind a run of text: translucent-orange background, white glyphs.
 * Everything in the info strip is drawn this way so it stays readable in
 * daylight at 10 px. */
static void field_txt(const font_t *f, int x, int y, const char *s,
                      uint16_t fg, uint16_t bg)
{
    int w = font_width(f, s);
    fill(x - 2, y - 1, w + 4, f->ascent >= 12 ? 14 : 12, bg);
    txt(f, x, y, s, fg);
}

static void field_txt_c(const font_t *f, int y, const char *s,
                        uint16_t fg, uint16_t bg)
{
    field_txt(f, (FB_W - font_width(f, s)) / 2, y, s, fg, bg);
}

static void field_txt_r(const font_t *f, int x_right, int y, const char *s,
                        uint16_t fg, uint16_t bg)
{
    field_txt(f, x_right - font_width(f, s), y, s, fg, bg);
}

static void draw_info(const radio_state_t *st, const app_shared_t *a)
{
    fill(0, Y_INFO, FB_W, Y_LBL - Y_INFO - 1, C_BG);

    char l[16];
    if (st->tx_status > 0) {
        snprintf(l, sizeof l, "SWR %.1f", st->swr);
        field_txt(&font_small, 6, Y_INFO, l, C_TEXT,
                  st->swr > 2.0f ? C_RED : C_FIELD);
        snprintf(l, sizeof l, "%dW", st->power_w);
        field_txt_c(&font_small, Y_INFO, l, C_TEXT, C_FIELD);
        snprintf(l, sizeof l, "ALC %d%%", (int)(st->alc_pct + 0.5f));
        field_txt_r(&font_small, FB_W - 6, Y_INFO, l, C_TEXT, C_FIELD);
        return;
    }

    {
        char u[16];
        if (st->s_unit[0] == '+') snprintf(u, sizeof u, "S9%s", st->s_unit);
        else snprintf(u, sizeof u, "%s", st->s_unit[0] ? st->s_unit : "-");
        field_txt(&font_small, 6, Y_INFO, u, C_TEXT, C_FIELD);
    }

    if (st->filter_hz > 0) {
        snprintf(l, sizeof l, "FIL %d.%dK", st->filter_hz / 1000,
                 (st->filter_hz % 1000) / 100);
        field_txt_c(&font_small, Y_INFO, l, C_TEXT, C_FIELD_D);
    }

    char r[16];
    if (a->ui_mode == UI_MODE_VOL) {
        snprintf(r, sizeof r, "SPK %d%%", a->spk_vol);
        field_txt_r(&font_small, FB_W - 6, Y_INFO, r, C_TEXT, C_FIELD);
    } else {
        /* the radio's own AF gain — read-only here, it is a server setting */
        snprintf(r, sizeof r, "AF %d", st->af_gain);
        field_txt_r(&font_small, FB_W - 6, Y_INFO, r, C_TEXT, C_FIELD_D);
    }
}

/* ── scope ───────────────────────────────────────────────────────── */

static const int32_t k_spans_hz[10] = {
    1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000
};

static int32_t span_hz(const radio_state_t *st)
{
    int i = st->scope_span;
    if (i < 0 || i > 9) i = CONFIG_MRRC_SCOPE_SPAN;
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
    fill(4, Y_LBL + 1, 96, 10, C_FIELD_D);
    txt(&font_tiny, 6, Y_LBL + 2, "SPECTRUM SCOPE", C_TEXT);
    char b[16];
    int32_t sp = span_hz(st);
    if (sp >= 1000000) snprintf(b, sizeof b, "%ldM", (long)(sp / 1000000));
    else snprintf(b, sizeof b, "%ldK", (long)(sp / 1000));
    int bw = font_width(&font_tiny, b);
    fill(FB_W - bw - 8, Y_LBL + 1, bw + 4, 10, C_FIELD);
    txt(&font_tiny, FB_W - bw - 6, Y_LBL + 2, b, C_TEXT);
}

/* The trace is a *line* plot over the raw floor/ceil mapping, exactly like the
 * web FFT canvas: 30 % EMA smoothing, 2x amplitude boost, 1 px line, cyan.
 * Filling the area under the line (what this used to do) made every noise bump
 * look like a signal and hid the real shape.  The trace is blue: it is the one
 * colour that stays legible against both the dark plot and the Jet waterfall. */
static void draw_plot(bool tx)
{
    int range = SCOPE_CEIL - SCOPE_FLOOR;
    if (range < 1) range = 1;
    uint16_t tc = tx ? C_RED : C_BLUE;

    /* Wipe the plot area first. */
    for (int x = 0; x < FB_W; x++) {
        uint16_t *col = s_fb + Y_PLOT * FB_W + x;
        for (int i = 0; i < H_PLOT; i++) col[i * FB_W] = C_BG;
    }

    /* One continuous line: each column's point is joined to the previous one
     * with a vertical run, so the trace is a solid stroke rather than a row of
     * isolated dots.  A second pixel underneath thickens it slightly, which is
     * what makes it legible on this panel. */
    int yprev = -1;
    for (int x = 0; x < FB_W; x++) {
        float mapped = (s_fft[x] - (float)SCOPE_FLOOR) / (float)range * FFT_BOOST;
        if (mapped < 0.0f) mapped = 0.0f;
        if (mapped > 1.0f) mapped = 1.0f;
        int y = Y_PLOT + (H_PLOT - 1) - (int)(mapped * (H_PLOT - 1));

        if (yprev < 0) {
            s_fb[y * FB_W + x] = tc;
            if (y + 1 < Y_PLOT + H_PLOT) s_fb[(y + 1) * FB_W + x] = tc;
        } else {
            int a = (y < yprev) ? y : yprev;
            int b = (y < yprev) ? yprev : y;
            for (int yy = a; yy <= b; yy++) s_fb[yy * FB_W + x] = tc;
            if (b + 1 < Y_PLOT + H_PLOT) s_fb[(b + 1) * FB_W + x] = tc;
        }
        yprev = y;
    }

    /* centre marker (the FT-710 runs CENTER mode: VFO == scope centre) */
    for (int y = Y_PLOT; y < Y_PLOT + H_PLOT; y += 4) {
        uint16_t *p = &s_fb[y * FB_W + FB_W / 2];
        if (*p == C_BG) *p = C_PANEL_D;
    }
    fill(0, Y_PLOT + H_PLOT, FB_W, 1, C_PANEL_D);
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
    field_txt(&font_tiny, 4, Y_SCALE, b, C_TEXT, C_FIELD_D);

    fmt_mhz(b, sizeof b, st->freq_hz);
    field_txt_c(&font_tiny, Y_SCALE, b, C_TEXT, C_FIELD);

    fmt_mhz(b, sizeof b, st->freq_hz + half);
    field_txt_r(&font_tiny, FB_W - 4, Y_SCALE, b, C_TEXT, C_FIELD_D);
}

static void draw_chips(const radio_state_t *st, const app_shared_t *a, int64_t now_ms)
{
    /* one bar, one colour, left-to-right: mode | TX/RX | link */
    fill(0, Y_CHIPS, FB_W, FB_H - Y_CHIPS, C_BAR);

    const char *m = (a->ui_mode == UI_MODE_VOL) ? "VOL" : "FREQ";
    txt(&font_small, 6, Y_CHIPS, m, C_TEXT);

    char t[16];
    if (st->tx_status > 0) {
        int sec = (int)((now_ms - a->ptt_since_ms) / 1000);
        if (sec < 0) sec = 0;
        snprintf(t, sizeof t, "TX %d", sec);
        txt_c(&font_small, Y_CHIPS, t, C_RED);
    } else {
        txt_c(&font_small, Y_CHIPS, "RX", C_TEXT);
    }

    const char *l = a->ctrl_up ? "LINK" : "noSRV";
    txt_r(&font_small, FB_W - 6, Y_CHIPS, l, a->ctrl_up ? C_TEXT : C_RED);
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
        g_ui_phase = 1; /* app_lock */
        app_lock();
        snap = g_app;
        app_unlock();

        int64_t now_us = esp_timer_get_time();
        int64_t now_ms = now_us / 1000;
        bool dirty = false;

        g_ui_heartbeat++;
        g_ui_phase = 2; /* body */

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
            g_ui_phase = 3; /* spectrum draw */

            int64_t gap = 1000000 / UI_FPS;
            if (now_us - last_frame_us >= gap) {
                last_spec = snap.spec_seq;
                last_frame_us = now_us;
                uint8_t cols[FB_W], idx[FB_W];
                bins_to_cols(snap.spec, cols);
                cols_to_idx(cols, idx);          /* waterfall: floor/ceil, no boost */
                fft_update(cols);                /* trace: 30 % EMA */
                draw_scope_label(&st);
                draw_plot(st.tx_status > 0);
                draw_waterfall(idx);
                draw_scale(&st);
                if (toast[0]) draw_toast(toast);
                board_display_flush(0, Y_LBL, FB_W, Y_CHIPS - Y_LBL);
            }
        } else if (dirty && toast[0]) {
            draw_toast(toast);
            board_display_flush(0, Y_LBL, FB_W, 13);
        }

        g_ui_phase = 0;
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
