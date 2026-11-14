#pragma once

/* Renders the whole UI on top of board_display and app_shared. */
void ui_start(void);

/* Debug: hex-PPM dump of the framebuffer to the console. */
void ui_dump_fb(void);
