/* AetherVNA firmware: display panel, text, screen capture, touch controller.
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Written from the MIPI DCS command set (the ST7796S/ILI9488 family the
 * panel answered to in the probe tests), the TSC2046 datasheet (the HR2046
 * follows it) and the board map in NOTES.md.
 *
 * The panel is write-only (MISO is not driven by it), so nothing here can
 * check what the screen shows: the test card labels itself (corners, colour
 * bars, a 480x320 and a 320x240 border) for a person to confirm, and
 * `lcd madctl` / `lcd inv` change orientation and inversion at run time.
 *
 * No framebuffer (RAM): the graph is drawn in strips of 32 columns through
 * adc_buf, between captures, never during one. Waits on SPI flags do not feed
 * the watchdog, so a stuck bus resets the chip instead of hanging the console. */
#include "hw.h"
#include "vna.h"
#include "lcd.h"

extern const uint8_t x5x7_bits[];
#define FONT_FIRST 0x16
#define FONT_LAST  0x7E

/* ---------- SPI1 (PB3 SCK, PB4 MISO, PB5 MOSI; remapped) ---------- */
uint8_t spi_ready;
void spi_init(void)
{
    pin_cfg(P_LCD_SCK, 0xB); pin_cfg(P_LCD_MOSI, 0xB);   /* alternate function push-pull */
    pin_input(P_TOUCH_MISO, 1);
    SPI_CTRL1 = 0;
    SPI_CTRL2 = 0;
    SPI_CTRL1 = (1u << 9) | (1u << 8) | (1u << 2) | (1u << 3);   /* SSM, SSI, master, /4 = 12 MHz */
    SPI_CTRL1 |= 1u << 6;
    spi_ready = 1;
}
static inline void spi_tx(uint8_t b) { while (!(SPI_STS & 2)) ; SPI_DT = b; }
static void spi_idle(void) { while (!(SPI_STS & 2)) ; while (SPI_STS & 0x80) ; }
void spi_speed(int slow)
{
    spi_idle();
    SPI_CTRL1 &= ~(1u << 6);
    SPI_CTRL1 = (SPI_CTRL1 & ~(7u << 3)) | ((slow ? 6u : 1u) << 3);   /* /128 (375 kHz) or /4 */
    SPI_CTRL1 |= 1u << 6;
}
uint8_t spi_xfer(uint8_t b)
{
    while (SPI_STS & 1) (void)SPI_DT;
    while (!(SPI_STS & 2)) ;
    SPI_DT = b;
    while (!(SPI_STS & 1)) ;
    return (uint8_t)SPI_DT;
}

/* ---------- panel ---------- */
uint8_t lcd_ready, madctl = 0x28, inverted;   /* MV + BGR: landscape, red is red */
void lcd_cmd(uint8_t c, const uint8_t *d, int n)
{
    spi_idle();
    pin_set(P_LCD_CS, 0); pin_set(P_LCD_DC, 0);
    spi_tx(c); spi_idle();
    pin_set(P_LCD_DC, 1);
    for (int i = 0; i < n; i++) spi_tx(d[i]);
    spi_idle();
    pin_set(P_LCD_CS, 1);
}
/* ---------- screen capture: everything drawn is also sent to the console ----------
 * The panel cannot be read back, so `lcd capture` redraws the screen with
 * this on: each window as "W x0 y0 x1 y1", then its pixels in drawing order
 * as runs "cccc*n" (RGB565 hex, count), then "E". host/screenshot.py turns it
 * into a PNG. */
static uint8_t cap, mute;
static uint16_t cap_c; static uint32_t cap_n; static int cap_k;
static void (*sink_win)(int x0, int y0, int x1, int y1);   /* capture to somewhere other than the console */
static void (*sink_run)(uint16_t c, uint32_t n);
static void cap_flush(void)
{
    if (!cap_n) return;
    if (sink_run) sink_run(cap_c, cap_n);
    else {
        con_hex(cap_c, 4); con_putc('*'); con_udec(cap_n); con_putc(' ');
        if (++cap_k == 12) { cap_k = 0; con_puts("\r\n"); }
    }
    cap_n = 0;
}
static void cap_px(uint16_t c, uint32_t n)
{
    if (cap_n && c != cap_c) cap_flush();
    cap_c = c; cap_n += n;
}
void lcd_capture(int on)
{
    if (!on) { cap_flush(); cap = 0; return; }
    cap = 1; cap_n = 0; cap_k = 0;
}
/* Capture into callbacks with the panel left alone (it already shows the screen). */
void lcd_capture_sink(void (*w)(int, int, int, int), void (*r)(uint16_t, uint32_t))
{
    if (!w) { cap_flush(); cap = 0; mute = 0; sink_win = 0; sink_run = 0; return; }
    sink_win = w; sink_run = r; cap = 1; mute = 1; cap_n = 0;
}
static void lcd_window(int x0, int y0, int x1, int y1)
{
    if (cap) {
        cap_flush(); cap_k = 0;
        if (sink_win) sink_win(x0, y0, x1, y1);
        else {
            con_puts("W "); con_udec(x0); con_putc(' '); con_udec(y0); con_putc(' ');
            con_udec(x1); con_putc(' '); con_udec(y1); con_puts("\r\n");
        }
    }
    if (mute) return;
    uint8_t ca[4] = { x0 >> 8, x0, x1 >> 8, x1 }, ra[4] = { y0 >> 8, y0, y1 >> 8, y1 };
    lcd_cmd(0x2A, ca, 4); lcd_cmd(0x2B, ra, 4);
    spi_idle();
    pin_set(P_LCD_CS, 0); pin_set(P_LCD_DC, 0);
    spi_tx(0x2C); spi_idle();
    pin_set(P_LCD_DC, 1);                            /* pixels follow; CS stays low */
}
static void lcd_end(void)
{
    if (!mute) { spi_idle(); pin_set(P_LCD_CS, 1); }
    if (cap) { cap_flush(); cap_k = 0; if (!sink_win) con_puts("\r\nE\r\n"); }
}
static void lcd_push(const uint16_t *px, int n)
{
    if (!mute) for (int i = 0; i < n; i++) { spi_tx(px[i] >> 8); spi_tx(px[i] & 0xFF); }
    if (cap) for (int i = 0; i < n; i++) cap_px(px[i], 1);
}
void lcd_blit(int x, int y, int w, int h, const uint16_t *px)
{
    if (w <= 0 || h <= 0) return;
    lcd_window(x, y, x + w - 1, y + h - 1); lcd_push(px, w * h); lcd_end();
}
void lcd_fill(int x, int y, int w, int h, uint16_t c)
{
    if (w <= 0 || h <= 0) return;
    lcd_window(x, y, x + w - 1, y + h - 1);
    if (!mute) for (int i = 0; i < w * h; i++) { spi_tx(c >> 8); spi_tx(c & 0xFF); if ((i & 4095) == 4095) service(); }
    if (cap) cap_px(c, (uint32_t)w * h);
    lcd_end();
}
static void wait_ms(uint32_t ms) { uint32_t t0 = ms_ticks; while (ms_ticks - t0 < ms) service(); }   /* USB keeps running */
void lcd_init(void)
{
    if (!spi_ready) spi_init();
    spi_speed(0);
    pin_set(P_TOUCH_CS, 1); pin_set(P_LCD_CS, 1);
    lcd_cmd(0x01, 0, 0); wait_ms(150);                /* software reset */
    lcd_cmd(0x11, 0, 0); wait_ms(150);                /* sleep out */
    uint8_t v = 0x55; lcd_cmd(0x3A, &v, 1);          /* 16 bits per pixel */
    lcd_cmd(0x36, &madctl, 1);
    lcd_cmd(inverted ? 0x21 : 0x20, 0, 0);
    lcd_cmd(0x29, 0, 0);                             /* display on */
    lcd_ready = 1;
}

/* ---------- text: 5x7 font, any scale, rendered into adc_buf ---------- */
static int glyph_w(char c) { if (c < FONT_FIRST || c > FONT_LAST) c = '?'; return 8 - (x5x7_bits[(c - FONT_FIRST) * 7] & 7); }
int text_w(const char *s, int sc) { int w = 0; for (; *s; s++) w += (glyph_w(*s) + 1) * sc; return w; }
/* Draws s at (x, y) on its background; returns the x after it. */
int text(int x, int y, const char *s, uint16_t fg, uint16_t bg, int sc)
{
    while (*s) {
        int w = 0, n = 0;                            /* as many characters as fit the buffer */
        while (s[n] && (w + (glyph_w(s[n]) + 1) * sc) * 8 * sc <= BUF_PX / 2) { w += (glyph_w(s[n]) + 1) * sc; n++; }
        if (x + w > W) w = W - x;
        if (w <= 0) return x;
        int hh = 8 * sc, cx = 0;
        for (int i = 0; i < w * hh; i++) BUF[i] = bg;
        for (int k = 0; k < n; k++) {
            char c = s[k]; if (c < FONT_FIRST || c > FONT_LAST) c = '?';
            const uint8_t *g = &x5x7_bits[(c - FONT_FIRST) * 7];
            int gw = glyph_w(c);
            for (int r = 0; r < 7; r++) {
                uint8_t bits = r ? g[r] : (gw == 8 ? g[0] : (g[0] & ~7));
                for (int col = 0; col < gw; col++) if (bits & (0x80 >> col))
                    for (int dy = 0; dy < sc; dy++) for (int dx = 0; dx < sc; dx++) {
                        int px = cx + col * sc + dx, py = r * sc + dy;
                        if (px < w) BUF[py * w + px] = fg;
                    }
            }
            cx += (gw + 1) * sc;
        }
        lcd_blit(x, y, w, hh, BUF);
        x += w; s += n;
        service();
    }
    return x;
}

/* ---------- small formatting, no printf ---------- */
char *put_s(char *p, const char *s) { while (*s) *p++ = *s++; *p = 0; return p; }
char *put_u(char *p, uint64_t v, int min_digits)
{
    char b[21]; int n = 0;
    do b[n++] = '0' + v % 10; while ((v /= 10) || n < min_digits);
    while (n) *p++ = b[--n];
    *p = 0; return p;
}
char *put_fx(char *p, float v, int dec)        /* fixed point, dec <= 6 */
{
    if (v < 0) { *p++ = '-'; v = -v; }
    uint32_t sc = 1; for (int i = 0; i < dec; i++) sc *= 10;
    uint64_t q = (uint64_t)(v * sc + 0.5f);
    p = put_u(p, q / sc, 1);
    if (dec) { *p++ = '.'; p = put_u(p, q % sc, dec); }
    return p;
}
char *put_sfx(char *p, float v, int dec) { if (v >= 0) *p++ = '+'; return put_fx(p, v, dec); }
char *put_freq(char *p, uint64_t f)            /* Hz, kHz, MHz or GHz */
{
    if (f >= 1000000000ull) { p = put_u(p, f / 1000000000ull, 1); *p++ = '.'; p = put_u(p, f % 1000000000ull / 1000, 6); return put_s(p, " GHz"); }
    if (f >= 1000000ull)    { p = put_u(p, f / 1000000ull, 1); *p++ = '.'; p = put_u(p, f % 1000000ull, 6); return put_s(p, " MHz"); }
    if (f >= 1000ull)       { p = put_u(p, f / 1000ull, 1); *p++ = '.'; p = put_u(p, f % 1000ull, 3); return put_s(p, " kHz"); }
    p = put_u(p, f, 1); return put_s(p, " Hz");
}

/* ---------- touch controller (HR2046 / TSC2046), CS PB7 ---------- */
uint16_t tsc_read(uint8_t cmd)
{
    if (!spi_ready) spi_init();
    spi_speed(1);
    pin_set(P_LCD_CS, 1); pin_set(P_TOUCH_CS, 0);
    spi_xfer(cmd);
    uint8_t a = spi_xfer(0), b = spi_xfer(0);
    pin_set(P_TOUCH_CS, 1);
    spi_speed(0);
    return ((a << 8 | b) >> 3) & 0xFFF;
}
/* The touch controller's two temperature diodes (TSC2046 TEMP0/TEMP1,
 * single-ended), 16 conversions each, summed: a second temperature proxy,
 * away from the MCU. Their difference tracks temperature without the
 * supply's own drift. */
void tsc_temps(uint32_t *t0, uint32_t *t1)
{
    uint32_t a = 0, b = 0;
    for (int k = 0; k < 16; k++) { a += tsc_read(0x84); b += tsc_read(0xF4); }
    *t0 = a; *t1 = b;
}
uint16_t vbat_counts(void) { uint32_t s = 0; for (int i = 0; i < 4; i++) s += tsc_read(0xA4); return s / 4; }
/* VBAT input divides by 4, single-ended against the supply. Scale set so
 * that 1299 counts reads 4.07 V, as stock reported in the same state (NOTES.md). */
int16_t vbat_offset_mv;                              /* EXPERT > VBAT OFFSET */
float vbat_volts(void) { return vbat_counts() * (4.07f / 1299.0f) + vbat_offset_mv * 0.001f; }

/* ---------- test card ---------- */
void lcd_test_card(void)
{
    lcd_fill(0, 0, W, H, 0x0000);
    static const struct { uint16_t c; const char *n; } bars[] = {
        { 0xF800, "RED" }, { 0x07E0, "GREEN" }, { 0x001F, "BLUE" }, { 0xFFFF, "WHITE" }, { 0xFFE0, "YELLOW" }, { 0x07FF, "CYAN" } };
    for (int i = 0; i < 6; i++) {
        int x = 40 + i * 68;
        lcd_fill(x, 110, 60, 90, bars[i].c);
        text(x, 206, bars[i].n, 0xFFFF, 0x0000, 1);
    }
    lcd_fill(0, 0, W, 2, 0xFFFF); lcd_fill(0, H - 2, W, 2, 0xFFFF);      /* 480x320 edge, white */
    lcd_fill(0, 0, 2, H, 0xFFFF); lcd_fill(W - 2, 0, 2, H, 0xFFFF);
    lcd_fill(0, 239, 320, 1, 0xF81F); lcd_fill(319, 0, 1, 240, 0xF81F);  /* 320x240 corner, magenta */
    text(6, 6, "TOP LEFT", 0xFFFF, 0x0000, 2);
    text(W - 6 - text_w("TOP RIGHT", 2), 6, "TOP RIGHT", 0xFFFF, 0x0000, 2);
    text(6, H - 22, "BOTTOM LEFT", 0xFFFF, 0x0000, 2);
    text(W - 6 - text_w("BOTTOM RIGHT", 2), H - 22, "BOTTOM RIGHT", 0xFFFF, 0x0000, 2);
    text(130, 40, "AetherVNA test card", 0xFFE0, 0x0000, 2);
    static const char hx[] = "0123456789ABCDEF";
    char b[64], *p = put_s(b, "madctl 0x");
    *p++ = hx[madctl >> 4]; *p++ = hx[madctl & 15]; *p = 0;
    put_s(p, inverted ? "  inv 1" : "  inv 0");
    text(130, 66, b, 0xFFFF, 0x0000, 2);
    text(40, 236, "Magenta lines: the 320x240 corner. White: the 480x320 edge.", 0xC618, 0x0000, 1);
    text(40, 250, "Text should read left to right, top line at the top.", 0xC618, 0x0000, 1);
}

void lcd_off(void)
{
    if (lcd_ready) lcd_cmd(0x28, 0, 0);
    pin_set(P_BACKLIGHT, 0);
    lcd_ready = 0;
}

/* ---------- anti-aliased text (fonts.c) ---------- */
static int fglyph(const font_t *f, char c) { return (c < f->first || c > f->last) ? '?' - f->first : c - f->first; }
int ftext_w(const char *s, const font_t *f) { int w = 0; for (; *s; s++) w += f->w[fglyph(f, *s)]; return w; }
/* Draws s at (x, y), stopping before xmax; returns the x after the last glyph. */
int ftext(int x, int y, const char *s, const font_t *f, uint16_t fg, uint16_t bg, int xmax)
{
    uint16_t pal[16];
    int fr = fg >> 11, fgg = (fg >> 5) & 63, fb = fg & 31, br = bg >> 11, bgg = (bg >> 5) & 63, bb = bg & 31;
    for (int a = 0; a < 16; a++)
        pal[a] = (uint16_t)((((fr * a + br * (15 - a) + 7) / 15) << 11) | (((fgg * a + bgg * (15 - a) + 7) / 15) << 5) | ((fb * a + bb * (15 - a) + 7) / 15));
    int h = f->h, maxw = (BUF_PX / 2) / h;              /* the lower half of the buffer: the time domain keeps data above */
    if (xmax > W) xmax = W;
    while (*s && x < 0) { x += f->w[fglyph(f, *s)]; s++; }   /* clip on the left by whole glyphs */
    while (*s) {
        int w = 0, n = 0;
        while (s[n]) {
            int gw = f->w[fglyph(f, s[n])];
            if (w + gw > maxw || x + w + gw > xmax) break;
            w += gw; n++;
        }
        if (!n) break;
        int cx = 0;
        for (int k = 0; k < n; k++) {
            int g = fglyph(f, s[k]), gw = f->w[g];
            const uint8_t *d = f->data + f->offs[g];
            for (int r = 0, p = 0; r < h; r++)
                for (int c = 0; c < gw; c++, p++) {
                    uint8_t b = d[p >> 1];
                    BUF[r * w + cx + c] = pal[(p & 1) ? (b & 15) : (b >> 4)];
                }
            cx += gw;
        }
        lcd_blit(x, y, w, h, BUF);
        x += w; s += n;
        if (s[0] && x + f->w[fglyph(f, s[0])] > xmax) break;
        service();
    }
    return x;
}
