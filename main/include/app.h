#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MRRC_BINS 850

typedef enum {
    UI_MODE_FREQ = 0,
    UI_MODE_VOL  = 1,
} ui_mode_t;

typedef enum {
    PTT_IDLE = 0,
    PTT_KEYING,
    PTT_RELEASING,
    PTT_FAILSAFE,
} ptt_state_t;

/* Everything the UI draws, mirrored from the server's RadioState. */
typedef struct {
    int32_t freq_hz;        /* active VFO */
    int32_t vfo_a_hz;
    int32_t vfo_b_hz;
    char    vfo;            /* 'A' / 'B' */

    char    mode_name[12];
    char    band_name[8];
    int     mode_num;

    int     s_meter;        /* raw 0..255 */
    char    s_unit[12];     /* "S9+20" */
    float   s_dbm;

    int     af_gain;        /* 0..255 */
    int     filter_hz;
    float   swr;
    float   alc_pct;
    int     power_w;

    int     tx_status;      /* 0=RX 1=TX 2=TUNE */
    int     scope_span;     /* FT-710 span index 0..9 (Hz table in ui.c) */
    bool    radio_online;   /* server says the serial link is up */

    int     battery_pct;    /* local, -1 = unknown */
    bool    charging;
} radio_state_t;

/* Application-wide shared block, guarded by app_lock()/app_unlock(). */
typedef struct {
    radio_state_t st;
    uint32_t      st_seq;       /* frequency / mode / band / TX changed */
    uint32_t      meter_seq;    /* S-meter / AF / SWR / power changed */

    uint8_t       spec[MRRC_BINS];
    uint32_t      spec_seq;

    char          toast[48];
    uint32_t      toast_seq;
    int64_t       toast_until_ms;

    int           spk_vol;      /* terminal speaker volume 0..100 (local!) */
    bool          spk_vol_dirty;
    ui_mode_t     ui_mode;
    ptt_state_t   ptt;
    int64_t       ptt_since_ms;

    bool          wifi_up;
    bool          ctrl_up;
    int           rssi;
} app_shared_t;

extern app_shared_t g_app;

void app_lock(void);
void app_unlock(void);

/* Called from the network layer when fresh data arrives. */
void app_on_state_changed(void);
void app_on_spectrum(const uint8_t *bins, int n);
void app_show_toast(const char *msg);
