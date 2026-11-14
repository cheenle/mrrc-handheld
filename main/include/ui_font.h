#pragma once

#include <stdint.h>

/* 8-bit alpha glyph baked from DejaVu by tools/gen_font.py. */
typedef struct {
    uint8_t  w, h;      /* glyph box */
    int8_t   bx, by;    /* bearing from the pen origin (top of the em box) */
    uint8_t  adv;       /* pen advance */
    uint32_t off;       /* offset into the alpha pool */
} glyph_t;

typedef struct {
    const glyph_t *glyphs;
    const uint8_t *pool;
    uint8_t first, last, ascent;
} font_t;

extern const font_t font_big;     /* . 0-9, 30 px tall  — the frequency */
extern const font_t font_mid;     /* printable, 15 px   — badge values */
extern const font_t font_small;   /* printable, 10 px   — labels */
extern const font_t font_tiny;    /* printable, 8 px    — ticks */

/* Draw at (x, y_top) where y_top is the top of the em box. */
void font_draw(const font_t *f, uint16_t *fb, int fb_w, int fb_h,
               int x, int y_top, const char *s, uint16_t color);
int  font_width(const font_t *f, const char *s);
