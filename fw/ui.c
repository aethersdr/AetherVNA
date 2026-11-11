/* LiteVNAAI firmware: display, touch controller, jog and the on-screen UI.
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
#include "ui.h"

#define W 480
#define H 320
extern const uint8_t x5x7_bits[];
#define FONT_FIRST 0x16
#define FONT_LAST  0x7E

/* ---------- SPI1 (PB3 SCK, PB4 MISO, PB5 MOSI; remapped) ---------- */
static uint8_t spi_ready;
static void spi_init(void)
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
static void spi_speed(int slow)
{
    spi_idle();
    SPI_CTRL1 &= ~(1u << 6);
    SPI_CTRL1 = (SPI_CTRL1 & ~(7u << 3)) | ((slow ? 6u : 1u) << 3);   /* /128 (375 kHz) or /4 */
    SPI_CTRL1 |= 1u << 6;
}
static uint8_t spi_xfer(uint8_t b)
{
    while (SPI_STS & 1) (void)SPI_DT;
    while (!(SPI_STS & 2)) ;
    SPI_DT = b;
    while (!(SPI_STS & 1)) ;
    return (uint8_t)SPI_DT;
}

/* ---------- panel ---------- */
static uint8_t lcd_ready, madctl = 0x28, inverted;   /* MV + BGR: landscape, red is red */
static void lcd_cmd(uint8_t c, const uint8_t *d, int n)
{
    spi_idle();
    pin_set(P_LCD_CS, 0); pin_set(P_LCD_DC, 0);
    spi_tx(c); spi_idle();
    pin_set(P_LCD_DC, 1);
    for (int i = 0; i < n; i++) spi_tx(d[i]);
    spi_idle();
    pin_set(P_LCD_CS, 1);
}
static void lcd_window(int x0, int y0, int x1, int y1)
{
    uint8_t ca[4] = { x0 >> 8, x0, x1 >> 8, x1 }, ra[4] = { y0 >> 8, y0, y1 >> 8, y1 };
    lcd_cmd(0x2A, ca, 4); lcd_cmd(0x2B, ra, 4);
    spi_idle();
    pin_set(P_LCD_CS, 0); pin_set(P_LCD_DC, 0);
    spi_tx(0x2C); spi_idle();
    pin_set(P_LCD_DC, 1);                            /* pixels follow; CS stays low */
}
static void lcd_end(void) { spi_idle(); pin_set(P_LCD_CS, 1); }
static void lcd_push(const uint16_t *px, int n)
{
    for (int i = 0; i < n; i++) { spi_tx(px[i] >> 8); spi_tx(px[i] & 0xFF); }
}
static void lcd_fill(int x, int y, int w, int h, uint16_t c)
{
    if (w <= 0 || h <= 0) return;
    lcd_window(x, y, x + w - 1, y + h - 1);
    for (int i = 0; i < w * h; i++) { spi_tx(c >> 8); spi_tx(c & 0xFF); if ((i & 4095) == 4095) service(); }
    lcd_end();
}
static void wait_ms(uint32_t ms) { uint32_t t0 = ms_ticks; while (ms_ticks - t0 < ms) service(); }   /* USB keeps running */
static void lcd_init(void)
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
#define BUF ((uint16_t *)adc_buf)
#define BUF_PX (4 * (SEG_N + 300))
static int glyph_w(char c) { if (c < FONT_FIRST || c > FONT_LAST) c = '?'; return 8 - (x5x7_bits[(c - FONT_FIRST) * 7] & 7); }
static int text_w(const char *s, int sc) { int w = 0; for (; *s; s++) w += (glyph_w(*s) + 1) * sc; return w; }
/* Draws s at (x, y) on its background; returns the x after it. */
static int text(int x, int y, const char *s, uint16_t fg, uint16_t bg, int sc)
{
    while (*s) {
        int w = 0, n = 0;                            /* as many characters as fit the buffer */
        while (s[n] && (w + (glyph_w(s[n]) + 1) * sc) * 8 * sc <= BUF_PX) { w += (glyph_w(s[n]) + 1) * sc; n++; }
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
        lcd_window(x, y, x + w - 1, y + hh - 1); lcd_push(BUF, w * hh); lcd_end();
        x += w; s += n;
        service();
    }
    return x;
}

/* ---------- small formatting, no printf ---------- */
static char *put_s(char *p, const char *s) { while (*s) *p++ = *s++; *p = 0; return p; }
static char *put_u(char *p, uint64_t v, int min_digits)
{
    char b[21]; int n = 0;
    do b[n++] = '0' + v % 10; while ((v /= 10) || n < min_digits);
    while (n) *p++ = b[--n];
    *p = 0; return p;
}
static char *put_fx(char *p, float v, int dec)        /* fixed point, dec <= 6 */
{
    if (v < 0) { *p++ = '-'; v = -v; }
    uint32_t sc = 1; for (int i = 0; i < dec; i++) sc *= 10;
    uint64_t q = (uint64_t)(v * sc + 0.5f);
    p = put_u(p, q / sc, 1);
    if (dec) { *p++ = '.'; p = put_u(p, q % sc, dec); }
    return p;
}
static char *put_sfx(char *p, float v, int dec) { if (v >= 0) *p++ = '+'; return put_fx(p, v, dec); }
static char *put_freq(char *p, uint64_t f)            /* Hz, kHz, MHz or GHz */
{
    if (f >= 1000000000ull) { p = put_u(p, f / 1000000000ull, 1); *p++ = '.'; p = put_u(p, f % 1000000000ull / 1000, 6); return put_s(p, " GHz"); }
    if (f >= 1000000ull)    { p = put_u(p, f / 1000000ull, 1); *p++ = '.'; p = put_u(p, f % 1000000ull, 6); return put_s(p, " MHz"); }
    if (f >= 1000ull)       { p = put_u(p, f / 1000ull, 1); *p++ = '.'; p = put_u(p, f % 1000ull, 3); return put_s(p, " kHz"); }
    p = put_u(p, f, 1); return put_s(p, " Hz");
}

/* ---------- touch controller (HR2046 / TSC2046), CS PB7 ---------- */
static uint16_t tsc_read(uint8_t cmd)
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
static uint16_t vbat_counts(void) { uint32_t s = 0; for (int i = 0; i < 4; i++) s += tsc_read(0xA4); return s / 4; }
/* VBAT input divides by 4, single-ended against the supply. Scale set so
 * that 1299 counts reads 4.07 V, as stock reported in the same state (NOTES.md). */
float vbat_volts(void) { return vbat_counts() * (4.07f / 1299.0f); }

/* ---------- test card ---------- */
static void test_card(void)
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
    text(130, 40, "LiteVNAAI test card", 0xFFE0, 0x0000, 2);
    static const char hx[] = "0123456789ABCDEF";
    char b[64], *p = put_s(b, "madctl 0x");
    *p++ = hx[madctl >> 4]; *p++ = hx[madctl & 15]; *p = 0;
    put_s(p, inverted ? "  inv 1" : "  inv 0");
    text(130, 66, b, 0xFFFF, 0x0000, 2);
    text(40, 236, "Magenta lines: the 320x240 corner. White: the 480x320 edge.", 0xC618, 0x0000, 1);
    text(40, 250, "Text should read left to right, top line at the top.", 0xC618, 0x0000, 1);
}

/* ---------- graph: log magnitude, +10 to -70 dB ---------- */
#define GX0 36
#define GX1 476                                      /* exclusive */
#define GW  (GX1 - GX0)
#define GY0 18
#define DIV_PX 32
#define NDIV 8
#define GY1 (GY0 + NDIV * DIV_PX)                    /* exclusive */
#define TOP_DB 10
#define DB_DIV 10
#define C_BG 0x0000
#define C_GRID 0x31A6
#define C_S11 0xFFE0
#define C_S21 0x07FF
#define C_MARK 0xFFFF
#define C_TEXT 0xFFFF
#define C_DIM 0x8410

static uint8_t ui_on, dirty, have_data, boot_done;
#define UI_BOOT_MS 1500                              /* after USB has enumerated */
static int marker;                                   /* point index */
static uint32_t drawn_count;

static void point_vals(int i, cpx *s11, cpx *s21)
{
    *s11 = pts[i].s11; *s21 = pts[i].s21;
    if (cal_on && cal_valid()) cal_apply(i, s11, s21);
}
static float point_db(int i, int which)
{
    cpx a, b; point_vals(i, &a, &b);
    return c_db(which ? b : a);
}
static int y_of_db(float db)
{
    float y = GY0 + (TOP_DB - db) * (DIV_PX / (float)DB_DIV);
    if (y < GY0) y = GY0;
    if (y > GY1 - 1) y = GY1 - 1;
    return (int)(y + 0.5f);
}
static int y_at(int col, int which)                  /* col 0..GW-1 */
{
    int n = sweep_cfg.points;
    if (n < 2) return y_of_db(point_db(0, which));
    float pos = (float)col * (n - 1) / (GW - 1);
    int i = (int)pos; if (i >= n - 1) i = n - 2;
    float fr = pos - i;
    return y_of_db(point_db(i, which) * (1 - fr) + point_db(i + 1, which) * fr);
}
static int marker_col(void)
{
    int n = sweep_cfg.points;
    return n < 2 ? 0 : (marker * (GW - 1) + (n - 1) / 2) / (n - 1);
}
static void draw_graph(void)
{
    int mc = marker_col(), hh = GY1 - GY0;
    int prev[2] = { -1, -1 };
    for (int xs = 0; xs < GW; xs += 32) {
        int w = GW - xs < 32 ? GW - xs : 32;
        for (int r = 0; r < hh; r++) {
            uint16_t c = (r % DIV_PX == 0 || r == hh - 1) ? C_GRID : C_BG;
            for (int k = 0; k < w; k++) BUF[r * w + k] = c;
        }
        for (int k = 0; k < w; k++) {
            int col = xs + k;
            if (col % 44 == 0 || col == GW - 1) for (int r = 0; r < hh; r++) BUF[r * w + k] = C_GRID;
            if (col == mc) for (int r = 0; r < hh; r += 2) BUF[r * w + k] = C_MARK;
            if (!have_data) continue;
            for (int t = 1; t >= 0; t--) {
                int y = y_at(col, t) - GY0, a = prev[t] < 0 ? y : prev[t];
                int lo = a < y ? a : y, hi = a < y ? y : a;
                for (int r = lo; r <= hi; r++) BUF[r * w + k] = t ? C_S21 : C_S11;
                prev[t] = y;
            }
        }
        lcd_window(GX0 + xs, GY0, GX0 + xs + w - 1, GY1 - 1); lcd_push(BUF, w * hh); lcd_end();
        service();
    }
}
static void draw_axes(void)
{
    for (int d = 0; d <= NDIV; d++) {
        int db = TOP_DB - d * DB_DIV;
        char b[8], *p = b;
        *p++ = db > 0 ? '+' : db < 0 ? '-' : ' ';
        put_u(p, (uint64_t)(db < 0 ? -db : db), 1);
        text(GX0 - 4 - text_w(b, 1), GY0 + d * DIV_PX - 3, b, C_DIM, C_BG, 1);
    }
}
static void draw_status(void)
{
    char b[112], *p = b;
    p = put_freq(p, sweep_cfg.start); p = put_s(p, " - "); p = put_freq(p, sweep_cfg.stop);
    p = put_s(p, "  "); p = put_u(p, sweep_cfg.points, 1); p = put_s(p, " pt  CAL ");
    p = put_s(p, cal_on && cal_valid() ? "on" : cal_have ? "partial" : "none");
    p = put_s(p, "  "); p = put_fx(p, vbat_volts(), 2); p = put_s(p, " V  #"); put_u(p, sweep_count, 1);
    int x = text(2, 4, b, C_TEXT, C_BG, 1);
    lcd_fill(x, 4, GX1 - 66 - x, 8, C_BG);
    x = text(GX1 - 62, 4, "S11", C_S11, C_BG, 1);
    text(x + 10, 4, "S21", C_S21, C_BG, 1);
}
static void draw_marker(void)
{
    char b[112], *p;
    if (marker >= sweep_cfg.points) marker = sweep_cfg.points - 1;
    cpx s11, s21; point_vals(marker, &s11, &s21);
    float g = c_abs(s11), swr = g < 0.999f ? (1 + g) / (1 - g) : 999;
    uint8_t fl = pts[marker].flags;
    p = put_s(b, "M "); p = put_freq(p, pts[marker].f_tuned ? pts[marker].f_tuned : sweep_freq(marker));
    put_s(p, fl & PF_RANGE ? "  range" : fl & PF_CLIP ? "  CLIP" : fl & PF_LOWREF ? "  low ref" : "");
    int x = text(4, GY1 + 6, b, C_TEXT, C_BG, 2);
    lcd_fill(x, GY1 + 6, W - x, 16, C_BG);
    p = put_s(b, "S11 "); p = put_sfx(p, c_db(s11), 2); p = put_s(p, " dB "); p = put_sfx(p, c_deg(s11), 1);
    p = put_s(p, " deg  SWR "); put_fx(p, swr, 3);
    x = text(4, GY1 + 28, b, C_S11, C_BG, 1);
    lcd_fill(x, GY1 + 28, W / 2 + 14 - x, 8, C_BG);
    p = put_s(b, "S21 "); p = put_sfx(p, c_db(s21), 3); p = put_s(p, " dB "); p = put_sfx(p, c_deg(s21), 2); put_s(p, " deg");
    x = text(W / 2 + 14, GY1 + 28, b, C_S21, C_BG, 1);
    lcd_fill(x, GY1 + 28, W - x, 8, C_BG);
}
static void ui_full(void)
{
    lcd_fill(0, 0, W, H, C_BG);
    draw_axes(); draw_status(); draw_graph(); draw_marker();
}

/* ---------- jog: left PB1 (ladder to 0 V, read by the ADC), right PB13 (low) ---------- */
static uint32_t jog_t, jog_next;
static int jog_dir_last;
static void jog_poll(void)
{
    if (ms_ticks - jog_t < 20) return;
    jog_t = ms_ticks;
    int dir = !pin_read(P_JOG_RIGHT) ? 1 : jog_left() ? -1 : 0;
    if (!dir) { jog_dir_last = 0; return; }
    int step = 0;
    if (dir != jog_dir_last) { step = 1; jog_next = ms_ticks + 400; }
    else if ((int32_t)(ms_ticks - jog_next) >= 0) { step = sweep_cfg.points > 100 ? 4 : 1; jog_next = ms_ticks + 60; }
    jog_dir_last = dir;
    if (step) {
        marker += dir * step;
        if (marker < 0) marker = 0;
        if (marker >= sweep_cfg.points) marker = sweep_cfg.points - 1;
        dirty |= 2;
    }
}

/* ---------- entry points ---------- */
void ui_redraw(void) { have_data = 1; dirty |= 1; }
static void ui_start(int sweep)
{
    if (!lcd_ready) lcd_init();
    pin_set(P_BACKLIGHT, 1);
    ui_on = 1; ui_full(); dirty = 0; drawn_count = sweep_count;
    if (sweep) sweep_running = 1;
}
void ui_background(void)
{
    if (!boot_done && ms_ticks > UI_BOOT_MS) { boot_done = 1; ui_start(1); }   /* standalone: screen and sweep at power-on */
    if (!ui_on || !lcd_ready) return;
    jog_poll();
    if (sweep_count != drawn_count) { drawn_count = sweep_count; have_data = 1; dirty |= 1; }
    if (!dirty) return;
    if (dirty & 1) draw_status();
    draw_graph(); draw_marker();
    dirty = 0;
}

static int streq(const char *a, const char *b) { while (*a && *a == *b) a++, b++; return *a == *b; }
static int hexval(const char *s, uint32_t *v)
{
    uint32_t x = 0; int n = 0;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) s += 2;
    for (; *s; s++, n++) {
        int d = *s >= '0' && *s <= '9' ? *s - '0' : *s >= 'a' && *s <= 'f' ? *s - 'a' + 10 : *s >= 'A' && *s <= 'F' ? *s - 'A' + 10 : -1;
        if (d < 0 || n >= 8) return 0;
        x = x * 16 + d;
    }
    *v = x; return n > 0;
}
void ui_command(int argc, char **argv)
{
    const char *c = argv[0];
    if (streq(c, "backlight")) {
        if (argc > 1) pin_set(P_BACKLIGHT, argv[1][0] == '1');
        con_puts(GPIO_ODT(P_BACKLIGHT >> 4) >> (P_BACKLIGHT & 15) & 1 ? "1\r\n" : "0\r\n");
        return;
    }
    if (streq(c, "touch")) {
        uint16_t x = tsc_read(0xD0), y = tsc_read(0x90), z1 = tsc_read(0xB0), z2 = tsc_read(0xC0), vb = vbat_counts();
        con_puts("x "); con_udec(x); con_puts(" y "); con_udec(y); con_puts(" z1 "); con_udec(z1); con_puts(" z2 "); con_udec(z2);
        con_puts(" pen "); con_puts(pin_read(P_TOUCH_IRQ) ? "up" : "down");
        con_puts(" vbat "); con_udec(vb); con_puts(" counts\r\n");
        return;
    }
    /* lcd ... */
    const char *a = argc > 1 ? argv[1] : "";
    boot_done = 1;                                   /* the host has taken over the display */
    if (streq(a, "init") || streq(a, "test")) {
        if (streq(a, "init") || !lcd_ready) lcd_init();
        pin_set(P_BACKLIGHT, 1);
        ui_on = 0; test_card(); con_puts("ok\r\n");
    } else if (streq(a, "ui")) {
        ui_start(!(argc > 2 && streq(argv[2], "nosweep")));
        con_puts("ok\r\n");
    } else if (streq(a, "off")) {
        ui_on = 0;
        if (lcd_ready) lcd_cmd(0x28, 0, 0);
        pin_set(P_BACKLIGHT, 0);
        lcd_ready = 0;
        con_puts("ok\r\n");
    } else if (streq(a, "madctl") && argc > 2) {
        uint32_t v; if (!hexval(argv[2], &v) || v > 0xFF) { con_puts("usage: lcd madctl <hex>\r\n"); return; }
        madctl = (uint8_t)v;
        if (lcd_ready) { lcd_cmd(0x36, &madctl, 1); if (ui_on) ui_full(); else test_card(); }
        con_puts("ok\r\n");
    } else if (streq(a, "inv") && argc > 2) {
        inverted = argv[2][0] == '1';
        if (lcd_ready) lcd_cmd(inverted ? 0x21 : 0x20, 0, 0);
        con_puts("ok\r\n");
    } else if (streq(a, "marker") && argc > 2) {
        int v = 0; for (const char *s = argv[2]; *s >= '0' && *s <= '9'; s++) v = v * 10 + (*s - '0');
        marker = v < sweep_cfg.points ? v : sweep_cfg.points - 1; dirty |= 2;
        con_puts("ok\r\n");
    } else if (streq(a, "status") || !*a) {
        con_puts("lcd "); con_puts(lcd_ready ? "on" : "off"); con_puts(" ui "); con_puts(ui_on ? "on" : "off");
        con_puts(" madctl 0x"); con_hex(madctl, 2); con_puts(" inv "); con_udec(inverted);
        con_puts(" marker "); con_udec(marker); con_puts(" sweeping "); con_udec(sweep_running); con_puts("\r\n");
    } else con_puts("usage: lcd [init|test|ui [nosweep]|off|madctl <hex>|inv 0|1|marker <i>|status]\r\n");
}
