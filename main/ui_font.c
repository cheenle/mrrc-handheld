#include "ui_font.h"

#include <string.h>

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

void font_draw(const font_t *f, uint16_t *fb, int fb_w, int fb_h,
               int x, int y_top, const char *s, uint16_t color)
{
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
                int a = src[col];
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
