/* AetherVNA firmware: display and input. SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdint.h>
void ui_background(void);
void ui_redraw(void);
void ui_command(int argc, char **argv);
void sd_command(int argc, char **argv);   /* sd.c */

/* display state and config, for store.c */
#define UI_STATE_MAX 256
#define UI_CFG_MAX 64
int  ui_state_size(void);
void ui_state_get(uint8_t *b);
void ui_state_put(const uint8_t *b);
int  ui_cfg_get(uint8_t *b);                 /* returns its length */
void ui_cfg_put(const uint8_t *b, int n);
void ui_redraw_full(void);
void ui_render_screen(void);
int  sd_save_screen(int autoname, char *name);
/* store.c */
int  store_valid(int n);
int  store_save(int n);
int  store_recall(int n);
int  config_save(void);
int  store_erase_all(void);
void store_boot(void);
void store_command(int argc, char **argv);
/* sd.c */
int  sd_save_touchstone(int two_port, int autoname, char *name);   /* name: 13 bytes out */
int  sd_save_cal(int autoname, char *name);
int  sd_load_cal(const char *name);
int  sd_list(const char *ext, char names[][13], int max);
