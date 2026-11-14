#include "ui_font.h"

#include <string.h>
#include <stdbool.h>

int font_width(const font_t *f, const char *s)
{
    int w = 0;
    for (; *s; s++) {
        unsigned c = (unsigned char)*s;
        if (c < f->first || c > f->last) c = f->first;   /* no '?' in subsets */
        w += f->glyphs[c - f->first].adv;
    }
    return w;
}

/* Coverage -> opacity curve.  A linear blend of 8-bit coverage makes 12-15 px
 * text look washed out on this panel (the ST7789 gamma lifts the dark end), so
 * the low end of the alpha ramp is pushed up: edges stay solid, not grey. */
static uint8_t s_gamma[256];
static bool    s_gamma_ready;

static void gamma_init(void)
{
    for (int i = 0; i < 256; i++) {
        float a = i / 255.0f;
        float g = a < 0.5f ? 2.0f * a * a : 1.0f - 2.0f * (1.0f - a) * (1.0f - a);
        s_gamma[i] = (uint8_t)(g * 255.0f + 0.5f);
    }
    s_gamma_ready = true;
}

void font_draw(const font_t *f, uint16_t *fb, int fb_w, int fb_h,
               int x, int y_top, const char *s, uint16_t color)
{
    if (!s_gamma_ready) gamma_init();
    int cr = (color >> 11) & 0x1F;
    int cg = (color >> 5) & 0x3F;
    int cb = color & 0x1F;

    for (; *s; s++) {
        unsigned c = (unsigned char)*s;
        if (c < f->first || c > f->last) c = f->first;   /* no '?' in subsets */
        const glyph_t *g = &f->glyphs[c - f->first];
        const uint8_t *src = f->pool + g->off;

        int gx = x + g->bx;
        int gy = y_top + g->by;
        for (int row = 0; row < g->h; row++) {
            int py = gy + row;
            if (py < 0 || py >= fb_h) { src += g->w; continue; }
            uint16_t *dst = fb + py * fb_w;
            for (int col = 0; col < g->w; col++) {
                int a = s_gamma[src[col]];
                if (!a) continue;
                int px = gx + col;
                if (px < 0 || px >= fb_w) continue;
                if (a >= 250) {
                    dst[px] = color;
                } else {
                    uint16_t d = dst[px];
                    int dr = (d >> 11) & 0x1F, dg = (d >> 5) & 0x3F, db = d & 0x1F;
                    int r = (cr * a + dr * (255 - a)) / 255;
                    int gg = (cg * a + dg * (255 - a)) / 255;
                    int b = (cb * a + db * (255 - a)) / 255;
                    dst[px] = (uint16_t)((r << 11) | (gg << 5) | b);
                }
            }
            src += g->w;
        }
        x += g->adv;
    }
}
