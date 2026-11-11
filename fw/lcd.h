/* AetherVNA firmware: display panel, text, screen capture, touch controller.
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdint.h>
#include "fonts.h"

#define W 480
#define H 320

/* No framebuffer: drawing goes through adc_buf, between captures. */
#define BUF ((uint16_t *)adc_buf)
#define BUF_PX (4 * (SEG_N + 300))

extern uint8_t lcd_ready, madctl, inverted;
void lcd_init(void);
void lcd_off(void);
void lcd_cmd(uint8_t c, const uint8_t *d, int n);
void lcd_fill(int x, int y, int w, int h, uint16_t c);
void lcd_blit(int x, int y, int w, int h, const uint16_t *px);
void lcd_capture(int on);
void lcd_capture_sink(void (*w)(int, int, int, int), void (*r)(uint16_t, uint32_t));   /* panel muted; 0 ends */
void lcd_test_card(void);

int  text(int x, int y, const char *s, uint16_t fg, uint16_t bg, int sc);   /* returns x after */
int  text_w(const char *s, int sc);
int  ftext(int x, int y, const char *s, const font_t *f, uint16_t fg, uint16_t bg, int xmax);   /* anti-aliased, clipped at xmax */
int  ftext_w(const char *s, const font_t *f);

char *put_s(char *p, const char *s);
char *put_u(char *p, uint64_t v, int min_digits);
char *put_fx(char *p, float v, int dec);
char *put_sfx(char *p, float v, int dec);
char *put_freq(char *p, uint64_t f);

/* SPI1, shared by the panel, the touch controller and the microSD slot */
extern uint8_t spi_ready;
void spi_init(void);
void spi_speed(int slow);                    /* 1: 375 kHz, 0: 12 MHz */
uint8_t spi_xfer(uint8_t b);

uint16_t tsc_read(uint8_t cmd);              /* TSC2046 command byte, 12-bit result */
uint16_t vbat_counts(void);
