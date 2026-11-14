/* Host renderer: compiles the real main/ui.c and dumps a PPM.
 *   gcc -I tools/host -I main/include -I main -DCONFIG_MRRC_UI_FPS=12 \
 *       tools/host/render.c main/ui_font.c main/ui_fonts.c -o /tmp/render -lm
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ui.c"          /* same TU → the static drawing helpers are usable */

app_shared_t g_app;

void app_lock(void) {}
void app_unlock(void) {}
uint16_t *board_display_fb(void) { return s_fb; }
void board_display_flush(int x, int y, int w, int h)
{ (void)x; (void)y; (void)w; (void)h; }
void board_display_backlight(int p) { (void)p; }
void *board_psram_alloc(size_t n) { return calloc(1, n); }
int64_t esp_timer_get_time(void) { return 1000000; }
void host_task_stub(void *fn, const char *name, unsigned stack, void *arg,
                    unsigned prio, void *handle, int core)
{ (void)fn; (void)name; (void)stack; (void)arg; (void)prio; (void)handle; (void)core; }

/* synthetic scope: a noise floor plus a few carriers */
static void synth_bins(uint8_t *bins, int frame)
{
    for (int i = 0; i < MRRC_BINS; i++) {
        float v = 82.0f + 12.0f * (float)rand() / RAND_MAX;   /* measured on this FT-710 */
        /* carriers */
        int peaks[4] = { 300, 430, 470, 620 };
        int amps[4] = { 90, 140, 70, 110 };
        for (int p = 0; p < 4; p++) {
            float d = (float)(i - peaks[p]) / 9.0f;
            v += amps[p] * expf(-d * d);
        }
        /* a slow moving emphasis so the waterfall shows structure */
        v += 35.0f * expf(-powf((i - (250 + frame * 12)) / 40.0f, 2));
        if (v > 255) v = 255;
        bins[i] = (uint8_t)v;
    }
}

int main(int argc, char **argv)
{
    int tx = (argc > 1 && !strcmp(argv[1], "tx"));
    int vol_mode = (argc > 2 && !strcmp(argv[2], "vol"));

    s_fb = calloc(FB_W * FB_H, sizeof(uint16_t));
    s_wf = calloc(H_WF * FB_W, 1);
    heat_init();

    radio_state_t st;
    memset(&st, 0, sizeof st);
    st.freq_hz = 7079510;
    st.vfo_a_hz = 7079510;
    st.vfo = 'A';
    snprintf(st.mode_name, sizeof st.mode_name, "LSB");
    snprintf(st.band_name, sizeof st.band_name, "40m");
    st.s_meter = 160;
    snprintf(st.s_unit, sizeof st.s_unit, "S9+10");
    st.filter_hz = 2400;
    st.af_gain = 128;
    st.scope_span = 6;              /* 100 kHz */
    st.battery_pct = 87;
    st.tx_status = tx ? 1 : 0;
    st.swr = 1.2f;
    st.alc_pct = 35;
    st.power_w = 100;

    app_shared_t a;
    memset(&a, 0, sizeof a);
    a.ui_mode = vol_mode ? UI_MODE_VOL : UI_MODE_FREQ;
    a.spk_vol = 70;
    a.ctrl_up = true;
    a.wifi_up = true;
    a.rssi = -55;
    a.ptt_since_ms = 0;

    /* prime the waterfall: 40 frames of history */
    for (int f = H_WF - 1; f >= 0; f--) {
        uint8_t bins[MRRC_BINS], cols[FB_W], idx[FB_W];
        synth_bins(bins, f);
        bins_to_cols(bins, cols);
        cols_to_idx(cols, idx);
        fft_update(cols);
        s_wf_head = (s_wf_head + 1) % H_WF;
        memcpy(s_wf + s_wf_head * FB_W, idx, FB_W);
    }

    /* one full compose, exactly like ui_task does */
    uint8_t bins[MRRC_BINS], cols[FB_W], idx[FB_W];
    synth_bins(bins, 18);
    bins_to_cols(bins, cols);
    cols_to_idx(cols, idx);
    fft_update(cols);

    fill(0, 0, FB_W, FB_H, C_BG);
    draw_header(&st, &a);
    fill(0, Y_FREQ, FB_W, Y_INFO - Y_FREQ, C_BG);
    draw_frequency(&st);
    draw_info(&st, &a);
    draw_chips(&st, &a, 3300);
    draw_scope_label(&st);
    draw_plot(tx);
    draw_waterfall(idx);
    draw_scale(&st);

    FILE *fp = fopen("/tmp/ui.ppm", "wb");
    fprintf(fp, "P6\n%d %d\n255\n", FB_W, FB_H);
    for (int i = 0; i < FB_W * FB_H; i++) {
        uint16_t c = s_fb[i];
        unsigned char rgb[3] = {
            (unsigned char)(((c >> 11) & 0x1F) * 255 / 31),
            (unsigned char)(((c >> 5) & 0x3F) * 255 / 63),
            (unsigned char)((c & 0x1F) * 255 / 31),
        };
        fwrite(rgb, 1, 3, fp);
    }
    fclose(fp);
    printf("/tmp/ui.ppm written\n");
    return 0;
}
