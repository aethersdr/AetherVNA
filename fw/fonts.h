/* AetherVNA firmware: anti-aliased fonts (generated: host/mkfont.py).
 * SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdint.h>
typedef struct {
    uint8_t h, first, last;
    const uint8_t *w;                      /* advance width per glyph */
    const uint16_t *offs;                  /* byte offset of each glyph in data */
    const uint8_t *data;                   /* 4-bit coverage, row by row, 2 px per byte */
} font_t;
extern const font_t font_s, font_m, font_l;
