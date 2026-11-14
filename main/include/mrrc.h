#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Wi-Fi + 4 WebSockets + auth.  Starts a supervisor task; never blocks. */
void mrrc_net_start(void);

/* Control-plane commands ({"type":"set","field":...,"value":...}). */
void mrrc_set_freq(int32_t hz);
void mrrc_set_af_gain(int v);
void mrrc_send_ptt(bool on);
void mrrc_send_txhb(void);

/* TX mic path. */
bool mrrc_tx_ready(void);                                   /* /WSaudioTX open */
void mrrc_tx_set_active(bool on);                           /* PTT gate for the mic task */
void mrrc_tx_frame(const uint8_t *tagged, size_t len);      /* 1-byte tag + payload */
void mrrc_tx_stop_text(void);                               /* "s:" */

bool mrrc_ctrl_up(void);

/* Audio path counters (rx frames played, tx frames sent, decode errors). */

/* True once if the server force-released our PTT (liveness/FT8/CQ). */
bool mrrc_take_force_rx(void);
