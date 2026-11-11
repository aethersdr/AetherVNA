/* LiteVNAAI firmware: display UI (stub for engine bring-up).
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later */
#include "hw.h"
#include "vna.h"
#include "ui.h"
void ui_background(void) { }
void ui_redraw(void) { }
void ui_command(int argc, char **argv) { (void)argc; (void)argv; con_puts("error: display not built yet\r\n"); }
float vbat_volts(void) { return 0; }
