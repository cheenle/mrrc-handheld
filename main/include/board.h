#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* ── Display ─────────────────────────────────────────────────────── */
esp_err_t board_display_init(void);
uint16_t *board_display_fb(void);
/* Blocking blit of an fb rectangle (x,y,w,h) to the panel. */
void board_display_flush(int x, int y, int w, int h);
void board_display_backlight(int percent);
void *board_psram_alloc(size_t bytes);

/* ── Audio ───────────────────────────────────────────────────────── */
esp_err_t board_audio_init(void);
/* Both return the number of *samples* actually transferred. */
size_t board_mic_read(int16_t *dst, size_t samples);      /* mono @ CONFIG_MRRC_MIC_RATE */
size_t board_mic_take(int16_t *dst, size_t samples);      /* from the live pump */
bool   board_mic_take_probe(int32_t *lmin, int32_t *lmax, int32_t *rmin, int32_t *rmax);
size_t board_spk_write(const int16_t *src, size_t samples);/* mono @ 48000 */

/* ── Input ───────────────────────────────────────────────────────── */
typedef enum {
    INPUT_NONE = 0,
    INPUT_BOOT_LONG,      /* long press crossed: begin PTT */
    INPUT_BOOT_RELEASE,   /* released after INPUT_BOOT_LONG: end PTT */
    INPUT_BOOT_DOUBLE,    /* two short clicks: toggle FREQ/VOL */
    INPUT_VOL_UP,
    INPUT_VOL_DOWN,
} input_event_t;

void board_input_init(void);
input_event_t board_input_get_event(void);   /* non-blocking, any task */

/* ── Power ───────────────────────────────────────────────────────── */
void board_power_status(int *battery_percent, bool *charging);

/* Audio diagnostics */
size_t board_mic_take(int16_t *dst, size_t samples);
int32_t board_mic_raw_peak(void);
