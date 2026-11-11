/* AetherVNA firmware: the on-screen UI: traces, Smith chart, markers, menus,
 * touch and jog input, and their console mirror.
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Layout (480x320): two legend rows at the top (one entry per trace, its
 * value at the active marker), a 440x256 graph (10x8 divisions) with the
 * Smith chart centred on it, three readout lines below. The menu is a
 * column of buttons on the right, over the graph; numbers and band presets
 * are entered on full-screen grids.
 *
 * Input: touch (tap) and the jog. The jog's push is not readable (NOTES.md,
 * stage 8), so with the menu closed left/right move the marker and a double
 * click right opens the menu; with a menu or grid open a click moves the
 * highlight, holding right selects and holding left goes back.
 *
 * Everything here is also reachable from the console (`trace`, `marker`,
 * `ui`, `lcd capture`), so a host can drive and check the screen.
 *
 * Nothing is stored per point: values are computed from pts[] (with the
 * calibration applied) as each 32-column strip of the graph is drawn
 * through adc_buf. */
#include "hw.h"
#include "vna.h"
#include "ui.h"
#include "lcd.h"

/* ---------- geometry and colours ---------- */
#define GX0 34
#define GX1 474                                      /* exclusive */
#define GW  (GX1 - GX0)
#define GY0 24
#define DIV_PX 32
#define NDIV 8
#define GY1 (GY0 + NDIV * DIV_PX)                    /* exclusive */
#define XDIV_PX 44
#define SCX (GX0 + GW / 2)                           /* Smith centre and radius */
#define SCY (GY0 + NDIV * DIV_PX / 2)
#define SR  127
#define MX  384                                      /* menu panel: x MX..W-1 */
#define MROW 35
#define RY0 (GY1 + 4)                                /* readout lines */
#define RDY 11

#define C_BG    0x0000
#define C_GRID  0x31A6
#define C_SMITH 0x4A69
#define C_TEXT  0xFFFF
#define C_DIM   0x8410
#define C_MARK  0xFFFF
#define C_MARK2 0xA514
#define C_BTN   0x2104
#define C_BTNHI 0x041F
#define C_EDGE  0x6B4D
static const uint16_t tcolor[4] = { 0xFFE0, 0x07FF, 0xF81F, 0x07E0 };

static inline float fsqrt(float x) { float r; __asm__("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x)); return r; }

/* ---------- traces ---------- */
enum { F_LOGMAG, F_PHASE, F_SWR, F_SMITH, F_LINEAR, F_R, F_X, F_Z, F_N };
static const char *const fmt_name[F_N] = { "LOGMAG", "PHASE", "SWR", "SMITH", "LINEAR", "R", "X", "|Z|" };
static const char *const fmt_cmd[F_N]  = { "logmag", "phase", "swr", "smith", "linear", "r", "x", "z" };
static const char *const fmt_unit[F_N] = { "dB", "deg", "", "", "", "ohm", "ohm", "ohm" };
static const float fmt_scale[F_N]      = { 10, 90, 1, 1, 0.125f, 25, 50, 25 };
static const float fmt_ref[F_N]        = { 0, 0, 1, 0, 0, 0, 0, 0 };
static const uint8_t fmt_refpos[F_N]   = { 7, 4, 0, 0, 0, 0, 4, 0 };
typedef struct { uint8_t on, ch, fmt, refpos; float scale, ref; } trace_t;
static trace_t tr[4] = {
    { 1, 0, F_LOGMAG, 7, 10, 0 },                    /* S11 log magnitude */
    { 1, 1, F_LOGMAG, 7, 10, 0 },                    /* S21 log magnitude */
    { 1, 0, F_SMITH, 0, 1, 0 },                      /* S11 Smith chart */
    { 1, 1, F_PHASE, 4, 90, 0 },                     /* S21 phase */
};
static int tr_act;

typedef struct { uint8_t on; uint16_t idx; } marker_t;
static marker_t mk[4] = { { 1, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
static int mk_act;

/* ---------- UI state ---------- */
enum { MODE_MAIN, MODE_GRID, MODE_TCAL };
enum { D_LEGEND = 1, D_GRAPH = 2, D_READOUT = 4, D_STATUS = 8, D_MENU = 16, D_FULL = 32 };
static uint8_t ui_on, have_data, boot_done, mode;
static uint16_t dirty;
#define UI_BOOT_MS 1500                              /* after USB has enumerated */
static uint32_t drawn_count;
static char msg[80]; static uint32_t msg_until;

static void say(const char *s, uint32_t ms)
{
    int i = 0; while (s[i] && i < (int)sizeof msg - 1) { msg[i] = s[i]; i++; }
    msg[i] = 0; msg_until = ms_ticks + ms; dirty |= D_STATUS;
}

/* ---------- values ---------- */
static void point_vals(int i, cpx *s11, cpx *s21)
{
    *s11 = pts[i].s11; *s21 = pts[i].s21;
    if (cal_on && cal_valid()) cal_apply(i, s11, s21);
}
static cpx point_g(int i, int ch) { cpx a, b; point_vals(i, &a, &b); return ch ? b : a; }
static cpx g_to_z(cpx g)                             /* 50 ohm reference */
{
    cpx d = c_sub(c_mk(1, 0), g);
    if (c_abs2(d) < 1e-12f) return c_mk(1e9f, 0);
    cpx z = c_div(c_add(c_mk(1, 0), g), d);
    return c_mk(50 * z.re, 50 * z.im);
}
static float fmt_val(int fmt, cpx g)
{
    switch (fmt) {
    case F_LOGMAG: return c_db(g);
    case F_PHASE:  return c_deg(g);
    case F_SWR:    { float m = c_abs(g); return m < 0.999f ? (1 + m) / (1 - m) : 999; }
    case F_SMITH:
    case F_LINEAR: return c_abs(g);
    case F_R:      return g_to_z(g).re;
    case F_X:      return g_to_z(g).im;
    default:       return c_abs(g_to_z(g));
    }
}
static float trace_val(int t, int i) { return fmt_val(tr[t].fmt, point_g(i, tr[t].ch)); }
static float y_of(int t, float v)                    /* graph row, 0 at top; may be off the graph */
{
    return (NDIV - tr[t].refpos) * DIV_PX - (v - tr[t].ref) / tr[t].scale * DIV_PX;
}
static uint64_t point_freq(int i) { return pts[i].f_tuned ? pts[i].f_tuned : sweep_freq(i); }
static int col_of_point(int i)
{
    int n = sweep_cfg.points;
    return n < 2 ? 0 : (i * (GW - 1) + (n - 1) / 2) / (n - 1);
}
static void clamp_markers(void)
{
    for (int m = 0; m < 4; m++) if (mk[m].idx >= sweep_cfg.points) mk[m].idx = sweep_cfg.points - 1;
}
static int any_rect(void) { for (int t = 0; t < 4; t++) if (tr[t].on && tr[t].fmt != F_SMITH) return 1; return 0; }
static int any_smith(void) { for (int t = 0; t < 4; t++) if (tr[t].on && tr[t].fmt == F_SMITH) return 1; return 0; }

/* ---------- text helpers ---------- */
static char *put_eng(char *p, float v, const char *unit)   /* SI prefix, 4 significant digits */
{
    static const char pre[] = { 'p', 'n', 'u', 'm', 0, 'k', 'M', 'G' };
    float a = v < 0 ? -v : v; int e = 4;
    if (a > 0) {
        while (a < 1 && e > 0) { a *= 1000; v *= 1000; e--; }
        while (a >= 1000 && e < 7) { a /= 1000; v /= 1000; e++; }
    }
    p = put_fx(p, v, a < 10 ? 3 : a < 100 ? 2 : 1);
    *p++ = ' '; if (pre[e]) *p++ = pre[e];
    return put_s(p, unit);
}
/* Text that stops at xmax (the menu's edge when it is open), the rest up to xmax cleared. */
static void line_text(int x, int y, const char *s, uint16_t fg, int xmax)
{
    char b[100]; int n = 0, w = 0;
    while (s[n] && n < (int)sizeof b - 1) {
        char one[2] = { s[n], 0 };
        int cw = text_w(one, 1);
        if (x + w + cw > xmax) break;
        b[n] = s[n]; w += cw; n++;
    }
    b[n] = 0;
    int e = n ? text(x, y, b, fg, C_BG, 1) : x;
    if (e < xmax) lcd_fill(e, y, xmax - e, 8, C_BG);
}

/* ---------- menus ---------- */
static uint8_t menu_open, menu_id, menu_sel;
enum { MN_MAIN, MN_DISPLAY, MN_FORMAT, MN_MARKER, MN_SWEEP, MN_CAL, MN_N };
enum {
    A_NONE, A_CLOSE, A_BACK,
    A_SUB_DISPLAY, A_SUB_MARKER, A_SUB_SWEEP, A_SUB_CAL, A_SUB_FORMAT,
    A_PAUSE, A_TOUCHCAL,
    A_TRSEL, A_TRON, A_TRCH, A_SCALE, A_REF, A_AUTO,
    A_FMT0,                                          /* .. A_FMT0 + F_N - 1 */
    A_MKSEL = A_FMT0 + F_N, A_MKON, A_MINSWR, A_MAX, A_MIN, A_MKCENTER, A_MKALLOFF,
    A_START, A_STOP, A_CENTER, A_SPAN, A_POINTS, A_BANDS,
    A_COPEN, A_CSHORT, A_CLOAD, A_CTHRU, A_CDONE, A_CONOFF, A_CRESET,
};
static const uint8_t menus[MN_N][10] = {
    [MN_MAIN]    = { A_SUB_DISPLAY, A_SUB_MARKER, A_SUB_SWEEP, A_SUB_CAL, A_PAUSE, A_TOUCHCAL, A_CLOSE },
    [MN_DISPLAY] = { A_TRSEL, A_TRON, A_TRCH, A_SUB_FORMAT, A_SCALE, A_REF, A_AUTO, A_BACK },
    [MN_FORMAT]  = { A_FMT0, A_FMT0 + 1, A_FMT0 + 2, A_FMT0 + 3, A_FMT0 + 4, A_FMT0 + 5, A_FMT0 + 6, A_FMT0 + 7, A_BACK },
    [MN_MARKER]  = { A_MKSEL, A_MKON, A_MINSWR, A_MAX, A_MIN, A_MKCENTER, A_MKALLOFF, A_BACK },
    [MN_SWEEP]   = { A_START, A_STOP, A_CENTER, A_SPAN, A_POINTS, A_BANDS, A_PAUSE, A_BACK },
    [MN_CAL]     = { A_COPEN, A_CSHORT, A_CLOAD, A_CTHRU, A_CDONE, A_CONOFF, A_CRESET, A_BACK },
};
static const char *const menu_name[MN_N] = { "main", "display", "format", "marker", "sweep", "cal" };
static int menu_len(int id) { int n = 0; while (n < 10 && menus[id][n]) n++; return n; }
static uint32_t reset_armed;                         /* CAL RESET asks twice */

static void item_label(int a, char *b)
{
    static const char *const cal_n[] = { "OPEN", "SHORT", "LOAD", "THRU" };
    char *p = b; *p = 0;
    switch (a) {
    case A_CLOSE: put_s(p, "CLOSE"); break;
    case A_BACK: put_s(p, "< BACK"); break;
    case A_SUB_DISPLAY: put_s(p, "DISPLAY"); break;
    case A_SUB_MARKER: put_s(p, "MARKER"); break;
    case A_SUB_SWEEP: put_s(p, "SWEEP"); break;
    case A_SUB_CAL: put_s(p, "CAL"); break;
    case A_SUB_FORMAT: p = put_s(p, "FMT "); put_s(p, fmt_name[tr[tr_act].fmt]); break;
    case A_PAUSE: put_s(p, sweep_running ? "PAUSE" : "RESUME"); break;
    case A_TOUCHCAL: put_s(p, "TOUCH CAL"); break;
    case A_TRSEL: p = put_s(p, "TRACE "); put_u(p, tr_act + 1, 1); break;
    case A_TRON: put_s(p, tr[tr_act].on ? "SHOWN" : "HIDDEN"); break;
    case A_TRCH: put_s(p, tr[tr_act].ch ? "S21" : "S11"); break;
    case A_SCALE: put_s(p, "SCALE"); break;
    case A_REF: put_s(p, "REF"); break;
    case A_AUTO: put_s(p, "AUTO"); break;
    case A_MKSEL: p = put_s(p, "MARKER "); put_u(p, mk_act + 1, 1); break;
    case A_MKON: put_s(p, mk[mk_act].on ? "SHOWN" : "HIDDEN"); break;
    case A_MINSWR: put_s(p, "MIN SWR"); break;
    case A_MAX: put_s(p, "MAX"); break;
    case A_MIN: put_s(p, "MIN"); break;
    case A_MKCENTER: put_s(p, "MK>CTR"); break;
    case A_MKALLOFF: put_s(p, "ALL OFF"); break;
    case A_START: put_s(p, "START"); break;
    case A_STOP: put_s(p, "STOP"); break;
    case A_CENTER: put_s(p, "CENTER"); break;
    case A_SPAN: put_s(p, "SPAN"); break;
    case A_POINTS: p = put_s(p, "PTS "); put_u(p, sweep_cfg.points, 1); break;
    case A_BANDS: put_s(p, "BANDS"); break;
    case A_COPEN: case A_CSHORT: case A_CLOAD: case A_CTHRU:
        p = put_s(p, cal_n[a - A_COPEN]); if (cal_have & (1 << (a - A_COPEN))) put_s(p, " *"); break;
    case A_CDONE: put_s(p, "DONE"); break;
    case A_CONOFF: put_s(p, cal_on && cal_valid() ? "CAL ON" : "CAL OFF"); break;
    case A_CRESET: put_s(p, reset_armed && ms_ticks - reset_armed < 3000 ? "SURE?" : "RESET"); break;
    default:
        if (a >= A_FMT0 && a < A_FMT0 + F_N) { p = put_s(p, fmt_name[a - A_FMT0]); if (tr[tr_act].fmt == a - A_FMT0) put_s(p, " *"); }
    }
}
static void draw_menu(void)
{
    int n = menu_len(menu_id);
    for (int r = 0; r * MROW < H; r++) {
        int y = r * MROW, h = y + MROW > H ? H - y : MROW;
        if (r >= n) { lcd_fill(MX, y, W - MX, h, C_BG); continue; }
        char b[24]; item_label(menus[menu_id][r], b);
        uint16_t bg = r == menu_sel ? C_BTNHI : C_BTN;
        lcd_fill(MX, y, W - MX, 1, C_EDGE);
        lcd_fill(MX, y + 1, 1, MROW - 2, C_EDGE);
        lcd_fill(MX + 1, y + 1, W - MX - 1, MROW - 2, bg);
        lcd_fill(MX, y + MROW - 1, W - MX, 1, C_BG);
        int sc = text_w(b, 2) <= W - MX - 6 ? 2 : 1;
        text(MX + 4, y + (MROW - 8 * sc) / 2, b, C_TEXT, bg, sc);
    }
}
static void menu_show(int id) { menu_open = 1; menu_id = id; menu_sel = 0; dirty |= D_MENU; }
static void menu_close(void) { if (menu_open) { menu_open = 0; dirty |= D_FULL; } }

/* ---------- full-screen grids: keypad and band presets ---------- */
enum { G_KEYPAD, G_BANDS };
static uint8_t grid_kind, grid_sel, kp_target;
static char kb[20];
static const char *const kp_keys[20] = {
    "7", "8", "9", "BS", "CANCEL",
    "4", "5", "6", "-", "",
    "1", "2", "3", "G", "M",
    "0", ".", "k", "ENTER", "" };
static const struct { const char *n; uint32_t a, b; } bands[] = {
    { "160m", 1800000, 2000000 }, { "80m", 3500000, 4000000 }, { "60m", 5330500, 5406500 }, { "40m", 7000000, 7300000 },
    { "30m", 10100000, 10150000 }, { "20m", 14000000, 14350000 }, { "17m", 18068000, 18168000 }, { "15m", 21000000, 21450000 },
    { "12m", 24890000, 24990000 }, { "10m", 28000000, 29700000 }, { "6m", 50000000, 54000000 }, { "HF 0-55", 50000, 55050000 },
    { "CANCEL", 0, 0 } };
#define NBANDS ((int)(sizeof bands / sizeof bands[0]))
static int grid_cols(void) { return grid_kind == G_KEYPAD ? 5 : 4; }
static int grid_cells(void) { return grid_kind == G_KEYPAD ? 20 : 16; }
static const char *grid_label(int k)
{
    if (grid_kind == G_KEYPAD) return kp_keys[k];
    return k < NBANDS ? bands[k].n : "";
}
static const char *target_name(void)
{
    switch (kp_target) {
    case A_START: return "START (Hz; k M G)";
    case A_STOP: return "STOP (Hz; k M G)";
    case A_CENTER: return "CENTER (Hz; k M G)";
    case A_SPAN: return "SPAN (Hz; k M G)";
    case A_SCALE: return "SCALE PER DIV";
    default: return "REFERENCE VALUE";
    }
}
static void draw_grid(void)
{
    int cols = grid_cols(), cw = W / cols, ch = (H - 64) / 4;
    lcd_fill(0, 0, W, 64, C_BG);
    text(8, 6, grid_kind == G_KEYPAD ? target_name() : "BAND PRESETS (keeps the point count)", C_DIM, C_BG, 2);
    if (grid_kind == G_KEYPAD) { char b[24]; char *p = put_s(b, kb); put_s(p, "_"); text(8, 32, b, C_TEXT, C_BG, 3); }
    for (int k = 0; k < grid_cells(); k++) {
        int x = (k % cols) * cw, y = 64 + (k / cols) * ch;
        const char *l = grid_label(k);
        if (!*l) { lcd_fill(x, y, cw, ch, C_BG); continue; }
        uint16_t bg = k == grid_sel ? C_BTNHI : C_BTN;
        lcd_fill(x, y, cw, ch, C_EDGE);
        lcd_fill(x + 1, y + 1, cw - 2, ch - 2, bg);
        int sc = text_w(l, 3) <= cw - 8 ? 3 : 2;
        text(x + (cw - text_w(l, sc)) / 2, y + (ch - 8 * sc) / 2, l, C_TEXT, bg, sc);
    }
}
static void grid_show(int kind, int target)
{
    mode = MODE_GRID; grid_kind = kind; kp_target = target; grid_sel = 0; kb[0] = 0;
    dirty |= D_FULL;
}
static void grid_leave(void) { mode = MODE_MAIN; dirty |= D_FULL; }

/* ---------- actions ---------- */
static int set_sweep_ui(uint64_t a, uint64_t b, int n)
{
    if (a < FREQ_MIN) a = FREQ_MIN;
    if (b > FREQ_MAX) b = FREQ_MAX;
    if (b < a || n < 1 || n > POINTS_MAX) { say("invalid sweep", 3000); return 0; }
    sweep_cfg.start = a; sweep_cfg.stop = b; sweep_cfg.points = n;
    vna_restart(); clamp_markers();
    dirty |= D_FULL;
    return 1;
}
/* "12.5" times a multiplier, in integer Hz (a float loses Hz above 16 MHz). */
static int parse_hz(const char *s, uint64_t mult, uint64_t *out)
{
    uint64_t ip = 0, fp = 0, fd = 1; int dot = 0, any = 0;
    for (; *s; s++) {
        if (*s == '.') { if (dot) return 0; dot = 1; continue; }
        if (*s < '0' || *s > '9') return 0;
        any = 1;
        if (dot) { if (fd < 1000000000ull) { fp = fp * 10 + (*s - '0'); fd *= 10; } }
        else ip = ip * 10 + (*s - '0');
    }
    if (!any) return 0;
    *out = ip * mult + fp * mult / fd;
    return 1;
}
static int parse_num(const char *s, float mult, float *out)
{
    int neg = 0; if (*s == '-') { neg = 1; s++; }
    uint64_t ip = 0, fp = 0, fd = 1; int dot = 0, any = 0;
    for (; *s; s++) {
        if (*s == '.') { if (dot) return 0; dot = 1; continue; }
        if (*s < '0' || *s > '9') return 0;
        any = 1;
        if (dot) { if (fd < 1000000ull) { fp = fp * 10 + (*s - '0'); fd *= 10; } }
        else ip = ip * 10 + (*s - '0');
    }
    if (!any) return 0;
    float v = ((float)ip + (float)fp / (float)fd) * mult;
    *out = neg ? -v : v;
    return 1;
}
/* Scale from 1-2-5 steps so the trace spans the graph; reference at the top line. */
static void autoscale(int t)
{
    if (tr[t].fmt == F_SMITH || !have_data) return;
    float lo = 1e30f, hi = -1e30f;
    for (int i = 0; i < sweep_cfg.points; i++) { float v = trace_val(t, i); if (v < lo) lo = v; if (v > hi) hi = v; }
    float need = (hi - lo) / NDIV, s = 1e-4f;
    if (need <= 0) need = 1e-4f;
    for (;;) {                                       /* the smallest step whose 8 divisions hold the span */
        float k = 0;
        for (int j = 0; j < 3; j++) {
            float c = s * (j == 0 ? 1 : j == 1 ? 2 : 5);
            float top = hi / c; int n = (int)top; if ((float)n < top) n++;   /* ceiling */
            if (n * c - NDIV * c <= lo) { k = c; break; }
        }
        if (k > 0) { s = k; break; }
        s *= 10;
        if (s > 1e9f) break;
    }
    float top = hi / s; int n = (int)top; if ((float)n < top) n++;
    tr[t].scale = s; tr[t].refpos = NDIV; tr[t].ref = n * s;
}
static void keypad_enter(uint64_t mult)
{
    uint64_t hz; float v;
    uint64_t s = sweep_cfg.start, e = sweep_cfg.stop, c = (s + e) / 2, sp = e - s;
    switch (kp_target) {
    case A_START: if (parse_hz(kb, mult, &hz)) set_sweep_ui(hz, e < hz ? hz : e, sweep_cfg.points); break;
    case A_STOP:  if (parse_hz(kb, mult, &hz)) set_sweep_ui(s > hz ? hz : s, hz, sweep_cfg.points); break;
    case A_CENTER: if (parse_hz(kb, mult, &hz)) set_sweep_ui(hz > sp / 2 ? hz - sp / 2 : 0, hz + sp / 2, sweep_cfg.points); break;
    case A_SPAN:  if (parse_hz(kb, mult, &hz)) set_sweep_ui(c > hz / 2 ? c - hz / 2 : 0, c + hz / 2, sweep_cfg.points); break;
    case A_SCALE: if (parse_num(kb, (float)mult, &v) && v > 0) tr[tr_act].scale = v; break;
    case A_REF:   if (parse_num(kb, (float)mult, &v)) tr[tr_act].ref = v; break;
    }
    grid_leave();
}
static void grid_press(int k)
{
    if (k < 0 || k >= grid_cells() || !*grid_label(k)) return;
    grid_sel = k;
    if (grid_kind == G_BANDS) {
        if (k < NBANDS - 1) set_sweep_ui(bands[k].a, bands[k].b, sweep_cfg.points);
        grid_leave(); return;
    }
    const char *l = kp_keys[k]; int n = 0; while (kb[n]) n++;
    if (l[0] >= '0' && l[0] <= '9' && !l[1]) { if (n < 15) { kb[n] = l[0]; kb[n + 1] = 0; } }
    else if (l[0] == '.' && !l[1]) { if (n < 15) { kb[n] = '.'; kb[n + 1] = 0; } }
    else if (l[0] == '-' && !l[1]) { if (n == 0) { kb[0] = '-'; kb[1] = 0; } }
    else if (l[0] == 'B') { if (n) kb[n - 1] = 0; }
    else if (l[0] == 'C') { grid_leave(); return; }
    else if (l[0] == 'G') { keypad_enter(1000000000ull); return; }
    else if (l[0] == 'M') { keypad_enter(1000000ull); return; }
    else if (l[0] == 'k') { keypad_enter(1000ull); return; }
    else if (l[0] == 'E') { keypad_enter(1); return; }
    dirty |= D_MENU;
}
static int marker_search(int what)                   /* 0 min SWR (S11), 1 max, 2 min (active trace) */
{
    int best = 0; float bv = 0;
    for (int i = 0; i < sweep_cfg.points; i++) {
        float v = what == 0 ? c_abs(point_g(i, 0)) : trace_val(tr_act, i);
        if (i == 0 || (what == 1 ? v > bv : v < bv)) { bv = v; best = i; }
    }
    return best;
}
static void touch_cal_start(void);
static void draw_status(void);
static void cal_step(int k)
{
    static const char *const nm[] = { "OPEN", "SHORT", "LOAD", "THRU" };
    char b[48]; char *p = put_s(b, "measuring "); put_s(p, nm[k]); say(b, 60000);
    draw_status();
    uint8_t was = sweep_running; sweep_running = 0;
    int r = cal_measure(k);
    sweep_running = was;
    p = put_s(b, nm[k]); put_s(p, r ? " failed" : " done");
    say(b, 4000);
}
static void act(int a)
{
    switch (a) {
    case A_CLOSE: menu_close(); return;
    case A_BACK: if (menu_id == MN_FORMAT) menu_show(MN_DISPLAY); else if (menu_id == MN_MAIN) menu_close(); else menu_show(MN_MAIN); return;
    case A_SUB_DISPLAY: menu_show(MN_DISPLAY); return;
    case A_SUB_MARKER: menu_show(MN_MARKER); return;
    case A_SUB_SWEEP: menu_show(MN_SWEEP); return;
    case A_SUB_CAL: menu_show(MN_CAL); return;
    case A_SUB_FORMAT: menu_show(MN_FORMAT); return;
    case A_PAUSE: sweep_running = !sweep_running; if (!sweep_running) synth_all_off(); dirty |= D_MENU | D_STATUS; return;
    case A_TOUCHCAL: menu_open = 0; touch_cal_start(); return;
    case A_TRSEL: tr_act = (tr_act + 1) & 3; dirty |= D_MENU | D_LEGEND | D_GRAPH; return;
    case A_TRON: tr[tr_act].on ^= 1; dirty |= D_MENU | D_LEGEND | D_GRAPH; return;
    case A_TRCH: tr[tr_act].ch ^= 1; dirty |= D_MENU | D_LEGEND | D_GRAPH; return;
    case A_SCALE: case A_REF: menu_open = 0; grid_show(G_KEYPAD, a); return;
    case A_AUTO: autoscale(tr_act); dirty |= D_LEGEND | D_GRAPH; return;
    case A_MKSEL: mk_act = (mk_act + 1) & 3; mk[mk_act].on = 1; dirty |= D_MENU | D_GRAPH | D_LEGEND | D_READOUT; return;
    case A_MKON: mk[mk_act].on ^= 1; dirty |= D_MENU | D_GRAPH | D_LEGEND | D_READOUT; return;
    case A_MINSWR: case A_MAX: case A_MIN:
        mk[mk_act].idx = marker_search(a == A_MINSWR ? 0 : a == A_MAX ? 1 : 2); mk[mk_act].on = 1;
        dirty |= D_GRAPH | D_LEGEND | D_READOUT; return;
    case A_MKCENTER: {
        uint64_t f = point_freq(mk[mk_act].idx), sp = sweep_cfg.stop - sweep_cfg.start;
        if (set_sweep_ui(f > sp / 2 ? f - sp / 2 : 0, f + sp / 2, sweep_cfg.points)) mk[mk_act].idx = sweep_cfg.points / 2;
        return; }
    case A_MKALLOFF: for (int m = 0; m < 4; m++) mk[m].on = 0; dirty |= D_MENU | D_GRAPH | D_LEGEND | D_READOUT; return;
    case A_START: case A_STOP: case A_CENTER: case A_SPAN: menu_open = 0; grid_show(G_KEYPAD, a); return;
    case A_POINTS: {
        static const uint16_t pp[] = { 51, 101, 201, 401 };
        int k = 0; while (k < 4 && pp[k] <= sweep_cfg.points) k++;
        set_sweep_ui(sweep_cfg.start, sweep_cfg.stop, pp[k & 3]); return; }
    case A_BANDS: menu_open = 0; grid_show(G_BANDS, 0); return;
    case A_COPEN: case A_CSHORT: case A_CLOAD: case A_CTHRU: cal_step(a - A_COPEN); dirty |= D_MENU; return;
    case A_CDONE: {
        int r = cal_solve();
        if (!r) cal_on = 1;
        say(r == -1 ? "need open, short, load and thru" : r ? "singular standards" : "calibration on", 4000);
        dirty |= D_MENU | D_GRAPH | D_LEGEND | D_READOUT; return; }
    case A_CONOFF:
        if (cal_valid()) cal_on = !cal_on; else say("no calibration for this sweep", 3000);
        dirty |= D_MENU | D_GRAPH | D_LEGEND | D_READOUT; return;
    case A_CRESET:
        if (reset_armed && ms_ticks - reset_armed < 3000) { cal_reset(); reset_armed = 0; say("calibration cleared", 3000); }
        else reset_armed = ms_ticks ? ms_ticks : 1;
        dirty |= D_MENU | D_GRAPH | D_LEGEND | D_READOUT; return;
    default:
        if (a >= A_FMT0 && a < A_FMT0 + F_N) {
            int f = a - A_FMT0; trace_t *t = &tr[tr_act];
            t->fmt = f; t->scale = fmt_scale[f]; t->ref = fmt_ref[f]; t->refpos = fmt_refpos[f]; t->on = 1;
            menu_show(MN_DISPLAY); dirty |= D_FULL;
        }
    }
}
static void menu_select(int r)
{
    if (r < 0 || r >= menu_len(menu_id)) return;
    menu_sel = r; dirty |= D_MENU;
    act(menus[menu_id][r]);
}

/* ---------- graph ---------- */
/* Smith grid: the unit circle, resistance circles r = 0.2, 0.5, 1, 2, 5
 * (centre r/(1+r), radius 1/(1+r)) and reactance arcs x = +/-0.2 .. 5
 * (centre (1, 1/x), radius 1/|x|, inside the unit circle), in pixels. Each
 * is drawn by solving for its crossings with every column and every row of
 * the strip, so it has no gaps whatever its slope. */
typedef struct { float cx, cy, r; uint8_t clip; } circ_t;
static circ_t sg[16]; static int sg_n;
static void smith_init(void)
{
    static const float v[] = { 0.2f, 0.5f, 1, 2, 5 };
    sg_n = 0;
    sg[sg_n++] = (circ_t){ 0, 0, SR, 0 };
    for (int k = 0; k < 5; k++) sg[sg_n++] = (circ_t){ SR * v[k] / (1 + v[k]), 0, SR / (1 + v[k]), 1 };
    for (int k = 0; k < 5; k++) {
        sg[sg_n++] = (circ_t){ SR, -SR / v[k], SR / v[k], 1 };
        sg[sg_n++] = (circ_t){ SR, SR / v[k], SR / v[k], 1 };
    }
}
static inline void plot(int x, int y, int x0, int w, uint16_t c)   /* screen coordinates, clipped to the strip */
{
    if (x < x0 || x >= x0 + w || y < GY0 || y >= GY1) return;
    BUF[(y - GY0) * w + (x - x0)] = c;
}
static void smith_grid(int x0, int w)
{
    float lim = (float)SR * SR + SR;
    for (int k = 0; k < sg_n; k++) {
        const circ_t *c = &sg[k];
        float ccx = SCX + c->cx, ccy = SCY + c->cy, r2 = c->r * c->r;
        for (int x = x0; x < x0 + w; x++) {
            float dx = x - ccx; if (dx * dx > r2) continue;
            float s = fsqrt(r2 - dx * dx);
            for (int sgn = -1; sgn <= 1; sgn += 2) {
                float yf = ccy + sgn * s; if (yf < GY0 - 1 || yf > GY1) continue;
                int y = (int)(yf + 0.5f);
                float ux = x - SCX, uy = y - SCY;
                if (!c->clip || ux * ux + uy * uy <= lim) plot(x, y, x0, w, C_SMITH);
            }
        }
        for (int y = GY0; y < GY1; y++) {
            float dy = y - ccy; if (dy * dy > r2) continue;
            float s = fsqrt(r2 - dy * dy);
            for (int sgn = -1; sgn <= 1; sgn += 2) {
                float xf = ccx + sgn * s; if (xf < x0 - 1 || xf > x0 + w) continue;
                int x = (int)(xf + 0.5f);
                float ux = x - SCX, uy = y - SCY;
                if (!c->clip || ux * ux + uy * uy <= lim) plot(x, y, x0, w, C_SMITH);
            }
        }
    }
    for (int x = SCX - SR; x <= SCX + SR; x++) plot(x, SCY, x0, w, C_SMITH);   /* real axis */
}
static void smith_xy(cpx g, int *x, int *y)
{
    float m = c_abs2(g);
    if (m > 1.21f) { float s = 1.1f / fsqrt(m); g.re *= s; g.im *= s; }
    float fx = SCX + g.re * SR, fy = SCY - g.im * SR;
    *x = (int)(fx + 0.5f); *y = (int)(fy + 0.5f);
}
static int iabs(int v) { return v < 0 ? -v : v; }
static void seg(int xa, int ya, int xb, int yb, int x0, int w, uint16_t c)
{
    if ((xa < x0 && xb < x0) || (xa >= x0 + w && xb >= x0 + w)) return;
    int dx = xb - xa, dy = yb - ya, n = iabs(dx) > iabs(dy) ? iabs(dx) : iabs(dy);
    if (!n) { plot(xa, ya, x0, w, c); return; }
    for (int k = 0; k <= n; k++) {
        int x = xa + (2 * dx * k + (dx >= 0 ? n : -n)) / (2 * n), y = ya + (2 * dy * k + (dy >= 0 ? n : -n)) / (2 * n);
        plot(x, y, x0, w, c);
    }
}
static void smith_trace(int t, int x0, int w)
{
    int px = 0, py = 0;
    for (int i = 0; i < sweep_cfg.points; i++) {
        int x, y; smith_xy(point_g(i, tr[t].ch), &x, &y);
        if (i) seg(px, py, x, y, x0, w, tcolor[t]);
        px = x; py = y;
    }
    for (int m = 0; m < 4; m++) if (mk[m].on) {
        int x, y; smith_xy(point_g(mk[m].idx, tr[t].ch), &x, &y);
        uint16_t c = m == mk_act ? C_MARK : C_MARK2;
        for (int d = -4; d <= 4; d++) { plot(x + d, y, x0, w, c); plot(x, y + d, x0, w, c); }
    }
}
static float col_val(int t, int col)
{
    int n = sweep_cfg.points;
    if (n < 2) return trace_val(t, 0);
    float pos = (float)col * (n - 1) / (GW - 1);
    int i = (int)pos; if (i >= n - 1) i = n - 2;
    float fr = pos - i;
    if (tr[t].fmt == F_PHASE) return trace_val(t, fr < 0.5f ? i : i + 1);   /* no interpolation across a wrap */
    return trace_val(t, i) * (1 - fr) + trace_val(t, i + 1) * fr;
}
static void draw_graph(void)
{
    int hh = GY1 - GY0, xend = menu_open ? MX : GX1;
    int prev[4] = { -1, -1, -1, -1 };
    int rect = any_rect(), smith = any_smith();
    int mcol[4]; for (int m = 0; m < 4; m++) mcol[m] = mk[m].on ? col_of_point(mk[m].idx) : -100;
    for (int xs = 0; GX0 + xs < xend; xs += 32) {
        int x0 = GX0 + xs, w = xend - x0 < 32 ? xend - x0 : 32;
        for (int r = 0; r < hh; r++) {
            uint16_t c = rect && (r % DIV_PX == 0 || r == hh - 1) ? C_GRID : C_BG;
            for (int k = 0; k < w; k++) BUF[r * w + k] = c;
        }
        if (rect) for (int k = 0; k < w; k++) {
            int col = xs + k;
            if (col % XDIV_PX == 0 || col == GW - 1) for (int r = 0; r < hh; r++) BUF[r * w + k] = C_GRID;
        }
        if (smith) smith_grid(x0, w);
        if (have_data) {
            for (int t = 3; t >= 0; t--) if (tr[t].on && tr[t].fmt == F_SMITH) smith_trace(t, x0, w);
            for (int t = 3; t >= 0; t--) {
                if (!tr[t].on || tr[t].fmt == F_SMITH) continue;
                for (int k = 0; k < w; k++) {
                    int col = xs + k;
                    float yf = y_of(t, col_val(t, col));
                    int y = yf < 0 ? 0 : yf > hh - 1 ? hh - 1 : (int)(yf + 0.5f);
                    int a = prev[t] < 0 ? y : prev[t];
                    if (tr[t].fmt == F_PHASE && iabs(a - y) > hh / 2) a = y;
                    int lo = a < y ? a : y, hi = a < y ? y : a;
                    for (int r = lo; r <= hi; r++) BUF[r * w + k] = tcolor[t];
                    prev[t] = y;
                    for (int m = 0; m < 4; m++) {                 /* marker: a small triangle above the point */
                        int d = iabs(col - mcol[m]); if (d > 3) continue;
                        uint16_t c = m == mk_act ? C_MARK : C_MARK2;
                        for (int r = y - 9 + 2 * d; r <= y - 3; r++) if (r >= 0 && r < hh) BUF[r * w + k] = c;
                    }
                }
            }
        }
        lcd_blit(x0, GY0, w, hh, BUF);
        service();
    }
}
static void draw_axes(void)
{
    int t = -1;
    if (tr[tr_act].on && tr[tr_act].fmt != F_SMITH) t = tr_act;
    else for (int k = 0; k < 4; k++) if (tr[k].on && tr[k].fmt != F_SMITH) { t = k; break; }
    lcd_fill(0, GY0 - 4, GX0 - 1, GY1 - GY0 + 8, C_BG);
    if (t < 0) return;
    for (int d = 0; d <= NDIV; d++) {
        float v = tr[t].ref + (NDIV - d - tr[t].refpos) * tr[t].scale;
        char b[16]; float a = v < 0 ? -v : v;
        put_fx(b, v, a >= 100 || tr[t].scale >= 5 ? 0 : a >= 10 || tr[t].scale >= 0.5f ? 1 : 2);
        text(GX0 - 3 - text_w(b, 1), GY0 + d * DIV_PX - 3, b, tcolor[t], C_BG, 1);
    }
}

/* ---------- legend, readout, status ---------- */
static int active_idx(void) { return mk[mk_act].on ? mk[mk_act].idx : -1; }
static char *put_val(char *p, int fmt, float v)
{
    switch (fmt) {
    case F_LOGMAG: return put_sfx(p, v, 3);
    case F_PHASE: return put_sfx(p, v, 2);
    case F_SWR: return put_fx(p, v, 3);
    case F_SMITH: case F_LINEAR: return put_fx(p, v, 4);
    default: return put_sfx(p, v, 2);
    }
}
static char *put_scale(char *p, float s)
{
    float a = s < 0 ? -s : s;
    return put_fx(p, s, a >= 10 ? 0 : a >= 1 ? 1 : a >= 0.1f ? 2 : 4);
}
static void draw_legend(void)
{
    int idx = active_idx();
    for (int t = 0; t < 4; t++) {
        char b[64], *p = b;
        *p++ = t == tr_act ? '>' : ' '; p = put_u(p, t + 1, 1); *p++ = ' ';
        if (!tr[t].on) put_s(p, "hidden");
        else {
            p = put_s(p, tr[t].ch ? "S21 " : "S11 "); p = put_s(p, fmt_name[tr[t].fmt]); *p++ = ' ';
            if (tr[t].fmt != F_SMITH) { p = put_scale(p, tr[t].scale); p = put_s(p, "/ "); }
            if (idx >= 0 && have_data) {
                if (tr[t].fmt == F_SMITH) {
                    cpx z = g_to_z(point_g(idx, tr[t].ch));
                    p = put_fx(p, z.re, 1); *p++ = z.im < 0 ? '-' : '+'; *p++ = 'j'; put_fx(p, z.im < 0 ? -z.im : z.im, 1);
                } else { p = put_val(p, tr[t].fmt, trace_val(t, idx)); *p++ = ' '; put_s(p, fmt_unit[tr[t].fmt]); }
            }
        }
        line_text((t & 1) * 192 + 2, 2 + (t >> 1) * 11, b, tr[t].on ? tcolor[t] : C_DIM, (t & 1) * 192 + 190);
    }
}
static void draw_readout(void)
{
    int xm = menu_open ? MX : W, idx = active_idx();
    char b[128], *p = b;
    if (idx < 0 || !have_data) { line_text(2, RY0, "no active marker", C_DIM, xm); line_text(2, RY0 + RDY, "", C_DIM, xm); return; }
    uint64_t f = point_freq(idx);
    cpx s11, s21; point_vals(idx, &s11, &s21);
    cpx z = g_to_z(s11);
    float gm = c_abs(s11), swr = gm < 0.999f ? (1 + gm) / (1 - gm) : 999, w = 6.2831853f * (float)f;
    p = put_s(p, "M"); p = put_u(p, mk_act + 1, 1); *p++ = ' '; p = put_freq(p, f);
    p = put_s(p, "  Z "); p = put_fx(p, z.re, 2); *p++ = z.im < 0 ? '-' : '+'; *p++ = 'j'; p = put_fx(p, z.im < 0 ? -z.im : z.im, 2);
    p = put_s(p, " ohm  SWR "); p = put_fx(p, swr, 3); p = put_s(p, "  ");
    if (z.im >= 0) p = put_eng(p, z.im / w, "H"); else p = put_eng(p, -1 / (w * z.im), "F");
    uint8_t fl = pts[idx].flags;
    put_s(p, fl & PF_RANGE ? "  range" : fl & PF_CLIP ? "  CLIP" : fl & PF_LOWREF ? "  low ref" : "");
    line_text(2, RY0, b, C_TEXT, xm);
    p = put_s(b, "S11 "); p = put_sfx(p, c_db(s11), 2); p = put_s(p, " dB "); p = put_sfx(p, c_deg(s11), 1);
    p = put_s(p, " deg   S21 "); p = put_sfx(p, c_db(s21), 3); p = put_s(p, " dB "); p = put_sfx(p, c_deg(s21), 2); p = put_s(p, " deg");
    for (int m = 0; m < 4; m++) if (mk[m].on && m != mk_act) { p = put_s(p, "  M"); p = put_u(p, m + 1, 1); *p++ = ' '; p = put_freq(p, point_freq(mk[m].idx)); }
    line_text(2, RY0 + RDY, b, C_DIM, xm);
}
static uint8_t touch_ok;
static void draw_status(void)
{
    int xm = menu_open ? MX : W;
    if (msg[0] && (int32_t)(msg_until - ms_ticks) > 0) { line_text(2, RY0 + 2 * RDY, msg, 0xFFE0, xm); return; }
    msg[0] = 0;
    char b[112], *p = b;
    p = put_freq(p, sweep_cfg.start); p = put_s(p, " - "); p = put_freq(p, sweep_cfg.stop);
    p = put_s(p, "  "); p = put_u(p, sweep_cfg.points, 1); p = put_s(p, " pt  CAL ");
    p = put_s(p, cal_on && cal_valid() ? "on" : cal_have ? "partial" : "none");
    p = put_s(p, sweep_running ? "  #" : "  PAUSED #"); p = put_u(p, sweep_count, 1);
    p = put_s(p, "  "); p = put_fx(p, vbat_volts(), 2); p = put_s(p, " V");
    if (!touch_ok) put_s(p, "  touch: uncalibrated");
    line_text(2, RY0 + 2 * RDY, b, C_DIM, xm);
}

/* ---------- touch ---------- */
/* Screen = a * raw + b on each axis; `swap` when the panel's x is the
 * controller's Y channel. Unset until TOUCH CAL (or `touch map`), and taps
 * are ignored until then. */
static float tm_ax, tm_bx, tm_ay, tm_by; static uint8_t tm_swap;
static uint8_t pen, tcal_step; static uint16_t tcal_raw[3][2];
static const int16_t tcal_pt[3][2] = { { 40, 40 }, { 440, 40 }, { 40, 280 } };
static int touch_raw(uint16_t *x, uint16_t *y)
{
    uint32_t sx = 0, sy = 0; uint16_t lo = 4095, hi = 0;
    if (tsc_read(0xB0) < 40) return 0;               /* Z1: not pressed firmly */
    for (int k = 0; k < 4; k++) {
        uint16_t a = tsc_read(0xD0), b = tsc_read(0x90);
        sx += a; sy += b; if (a < lo) lo = a; if (a > hi) hi = a;
    }
    if (hi - lo > 80) return 0;                      /* still settling */
    *x = sx / 4; *y = sy / 4;
    return 1;
}
static void draw_cross(int x, int y)
{
    lcd_fill(0, 0, W, H, C_BG);
    lcd_fill(x - 12, y, 25, 1, C_TEXT); lcd_fill(x, y - 12, 1, 25, C_TEXT);
    text(110, 150, "TOUCH CAL: tap the centre of the cross", C_TEXT, C_BG, 1);
    text(110, 164, "(hold the jog left to cancel)", C_DIM, C_BG, 1);
}
static void touch_cal_start(void) { mode = MODE_TCAL; tcal_step = 0; draw_cross(tcal_pt[0][0], tcal_pt[0][1]); }
static void touch_cal_point(uint16_t rx, uint16_t ry)
{
    tcal_raw[tcal_step][0] = rx; tcal_raw[tcal_step][1] = ry;
    if (++tcal_step < 3) { draw_cross(tcal_pt[tcal_step][0], tcal_pt[tcal_step][1]); return; }
    /* point 0 -> 1 moves screen x only: the raw channel that changed more is x */
    int dxa = iabs(tcal_raw[1][0] - tcal_raw[0][0]), dxb = iabs(tcal_raw[1][1] - tcal_raw[0][1]);
    uint8_t swap = dxb > dxa;
    int cx = swap, cy = !swap;
    int d1 = tcal_raw[1][cx] - tcal_raw[0][cx], d2 = tcal_raw[2][cy] - tcal_raw[0][cy];
    mode = MODE_MAIN; dirty |= D_FULL;
    if (iabs(d1) < 200 || iabs(d2) < 200) { say("touch cal failed: taps too close together", 4000); return; }
    tm_swap = swap;
    tm_ax = (float)(tcal_pt[1][0] - tcal_pt[0][0]) / d1; tm_bx = tcal_pt[0][0] - tm_ax * tcal_raw[0][cx];
    tm_ay = (float)(tcal_pt[2][1] - tcal_pt[0][1]) / d2; tm_by = tcal_pt[0][1] - tm_ay * tcal_raw[0][cy];
    touch_ok = 1; say("touch calibrated", 4000);
}
static void tap(int x, int y)
{
    if (mode == MODE_GRID) {
        int cols = grid_cols(), cw = W / cols, ch = (H - 64) / 4;
        if (y >= 64) grid_press((x / cw) + ((y - 64) / ch) * cols);
        return;
    }
    if (mode != MODE_MAIN) return;
    if (menu_open) { if (x >= MX) menu_select(y / MROW); else menu_close(); return; }
    menu_show(MN_MAIN);
}
static uint32_t touch_t;
static void touch_poll(void)
{
    if (ms_ticks - touch_t < 20) return;
    touch_t = ms_ticks;
    if (pin_read(P_TOUCH_IRQ)) { pen = 0; return; }   /* pen up */
    if (pen) return;                                 /* one action per touch */
    uint16_t rx, ry;
    if (!touch_raw(&rx, &ry)) return;
    pen = 1;
    if (mode == MODE_TCAL) { touch_cal_point(rx, ry); return; }
    if (!touch_ok) { say("touch uncalibrated: double click jog right, then TOUCH CAL", 5000); return; }
    float a = tm_swap ? ry : rx, b = tm_swap ? rx : ry;
    int x = (int)(tm_ax * a + tm_bx), y = (int)(tm_ay * b + tm_by);
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    tap(x, y);
}

/* ---------- jog ---------- */
/* With the menu closed, a press moves the marker at once and repeats while
 * held; a second right press within 350 ms of a quick right click opens the
 * menu (and takes back the click's step). With a menu or grid open, a
 * release under 700 ms is a click (move), 700 ms held is a hold (right
 * selects, left goes back). */
static uint32_t jog_t, jog_down, jog_rep, last_r_click;
static int jog_dir, jog_held_fired, jog_consumed;
static void marker_step(int d)
{
    marker_t *m = &mk[mk_act];
    m->on = 1;
    int i = m->idx + d;
    if (i < 0) i = 0;
    if (i >= sweep_cfg.points) i = sweep_cfg.points - 1;
    m->idx = i; dirty |= D_GRAPH | D_LEGEND | D_READOUT;
}
static void nav_move(int d)
{
    if (mode == MODE_GRID) {
        int n = grid_cells(), k = grid_sel;
        do { k = (k + d + n) % n; } while (!*grid_label(k));
        grid_sel = k; dirty |= D_MENU;
    } else if (menu_open) {
        int n = menu_len(menu_id); menu_sel = (menu_sel + d + n) % n; dirty |= D_MENU;
    }
}
static void nav_hold(int d)
{
    if (mode == MODE_TCAL) { if (d < 0) { mode = MODE_MAIN; dirty |= D_FULL; say("touch cal cancelled", 3000); } return; }
    if (mode == MODE_GRID) { if (d > 0) grid_press(grid_sel); else grid_leave(); return; }
    if (menu_open) { if (d > 0) menu_select(menu_sel); else act(A_BACK); }
}
static void jog_event(int dir, int edge)             /* edge: 1 press, 2 still held, 0 release */
{
    int nav = mode != MODE_MAIN || menu_open;
    if (edge == 1) {
        jog_dir = dir; jog_down = ms_ticks; jog_held_fired = 0; jog_consumed = 0;
        if (!nav) {
            if (dir > 0 && last_r_click && ms_ticks - last_r_click < 350) {
                marker_step(-1); menu_show(MN_MAIN); jog_consumed = 1; last_r_click = 0;
            } else { marker_step(dir); jog_rep = ms_ticks + 400; }
        }
        return;
    }
    if (edge == 2) {
        if (nav) { if (!jog_held_fired && ms_ticks - jog_down >= 700) { jog_held_fired = 1; nav_hold(jog_dir); } }
        else if (jog_consumed != 1 && (int32_t)(ms_ticks - jog_rep) >= 0) {
            marker_step(jog_dir * (sweep_cfg.points > 100 ? 4 : 1)); jog_rep = ms_ticks + 60; jog_consumed = 2;
        }
        return;
    }
    if (nav) { if (!jog_held_fired && !jog_consumed) nav_move(jog_dir); }
    else last_r_click = (jog_dir > 0 && !jog_consumed && ms_ticks - jog_down < 300) ? (ms_ticks ? ms_ticks : 1) : 0;
    jog_dir = 0;
}
static void jog_poll(void)
{
    if (ms_ticks - jog_t < 20) return;
    jog_t = ms_ticks;
    int dir = !pin_read(P_JOG_RIGHT) ? 1 : jog_left() ? -1 : 0;
    if (dir != jog_dir) {
        if (jog_dir) jog_event(jog_dir, 0);
        if (dir) jog_event(dir, 1);
    } else if (dir) jog_event(dir, 2);
}

/* ---------- drawing and entry points ---------- */
static void render(void)
{
    uint16_t d = dirty; dirty = 0;
    if (mode == MODE_TCAL) return;
    if (mode == MODE_GRID) { if (d & (D_FULL | D_MENU)) draw_grid(); return; }
    if (d & D_FULL) { lcd_fill(0, 0, W, H, C_BG); d |= D_LEGEND | D_GRAPH | D_READOUT | D_STATUS | D_MENU; }
    if (d & D_LEGEND) draw_legend();
    if (d & D_GRAPH) { draw_axes(); draw_graph(); }
    if (d & D_READOUT) draw_readout();
    if (d & D_STATUS) draw_status();
    if (menu_open && (d & D_MENU)) draw_menu();
}
void ui_redraw(void) { have_data = 1; dirty |= D_GRAPH | D_LEGEND | D_READOUT | D_STATUS; }
static void ui_start(int sweep)
{
    if (!lcd_ready) lcd_init();
    if (!sg_n) smith_init();
    pin_set(P_BACKLIGHT, 1);
    ui_on = 1; mode = MODE_MAIN; clamp_markers();
    dirty = D_FULL; render(); drawn_count = sweep_count;
    if (sweep) sweep_running = 1;
}
void ui_background(void)
{
    if (!boot_done && ms_ticks > UI_BOOT_MS) { boot_done = 1; ui_start(1); }   /* standalone: screen and sweep at power-on */
    if (!ui_on || !lcd_ready) return;
    jog_poll(); touch_poll();
    if (sweep_count != drawn_count) { drawn_count = sweep_count; have_data = 1; dirty |= D_GRAPH | D_LEGEND | D_READOUT | D_STATUS; }
    if (msg[0] && (int32_t)(msg_until - ms_ticks) <= 0) dirty |= D_STATUS;
    if (reset_armed && ms_ticks - reset_armed >= 3000) { reset_armed = 0; if (menu_open) dirty |= D_MENU; }
    if (dirty) render();
}

/* ---------- console ---------- */
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
static int decval(const char *s, int *v)
{
    int x = 0, n = 0, neg = *s == '-'; if (neg) s++;
    for (; *s; s++, n++) { if (*s < '0' || *s > '9' || n > 6) return 0; x = x * 10 + (*s - '0'); }
    *v = neg ? -x : x; return n > 0;
}
static void put_f(float v, int dec) { if (v < 0) { con_putc('-'); v = -v; } con_float(v, dec); }
static void print_trace(int t)
{
    con_udec(t + 1); con_puts(tr[t].on ? " on " : " off "); con_puts(tr[t].ch ? "s21 " : "s11 "); con_puts(fmt_cmd[tr[t].fmt]);
    con_puts(" scale "); put_f(tr[t].scale, 4); con_puts(" ref "); put_f(tr[t].ref, 4); con_puts(" refpos "); con_udec(tr[t].refpos);
    if (t == tr_act) con_puts(" active");
    con_puts("\r\n");
}
static void print_marker(int m)
{
    con_udec(m + 1); con_puts(mk[m].on ? " on " : " off "); con_udec(mk[m].idx); con_putc(' ');
    char b[24]; put_u(b, point_freq(mk[m].idx), 1); con_puts(b);
    if (m == mk_act) con_puts(" active");
    con_puts("\r\n");
}
static void cmd_trace(int argc, char **argv)
{
    if (argc == 1) { for (int t = 0; t < 4; t++) print_trace(t); return; }
    int n; if (!decval(argv[1], &n) || n < 1 || n > 4) goto usage;
    trace_t *t = &tr[n - 1];
    for (int k = 2; k < argc; k++) {
        const char *a = argv[k]; float v; int d;
        if (streq(a, "on")) t->on = 1;
        else if (streq(a, "off")) t->on = 0;
        else if (streq(a, "s11")) t->ch = 0;
        else if (streq(a, "s21")) t->ch = 1;
        else if (streq(a, "active")) tr_act = n - 1;
        else if (streq(a, "auto")) autoscale(n - 1);
        else if (streq(a, "scale") && k + 1 < argc && parse_num(argv[k + 1], 1, &v) && v > 0) { t->scale = v; k++; }
        else if (streq(a, "ref") && k + 1 < argc && parse_num(argv[k + 1], 1, &v)) { t->ref = v; k++; }
        else if (streq(a, "refpos") && k + 1 < argc && decval(argv[k + 1], &d) && d >= 0 && d <= NDIV) { t->refpos = d; k++; }
        else {
            int f = -1; for (int j = 0; j < F_N; j++) if (streq(a, fmt_cmd[j])) f = j;
            if (f < 0) goto usage;
            t->fmt = f; t->scale = fmt_scale[f]; t->ref = fmt_ref[f]; t->refpos = fmt_refpos[f]; t->on = 1;
        }
    }
    dirty |= D_FULL; print_trace(n - 1); return;
usage:
    con_puts("usage: trace [1-4 [on|off|s11|s21|logmag|phase|swr|smith|linear|r|x|z|scale <v>|ref <v>|refpos <0-8>|auto|active]...]\r\n");
}
static void cmd_marker(int argc, char **argv)
{
    if (argc == 1) { for (int m = 0; m < 4; m++) print_marker(m); return; }
    int n; if (!decval(argv[1], &n) || n < 1 || n > 4) goto usage;
    marker_t *m = &mk[n - 1];
    for (int k = 2; k < argc; k++) {
        const char *a = argv[k]; int v;
        if (streq(a, "on")) m->on = 1;
        else if (streq(a, "off")) m->on = 0;
        else if (streq(a, "active")) { mk_act = n - 1; m->on = 1; }
        else if (streq(a, "minswr")) { m->idx = marker_search(0); m->on = 1; }
        else if (streq(a, "max")) { m->idx = marker_search(1); m->on = 1; }
        else if (streq(a, "min")) { m->idx = marker_search(2); m->on = 1; }
        else if (decval(a, &v) && v >= 0 && v < sweep_cfg.points) { m->idx = v; m->on = 1; }
        else goto usage;
    }
    dirty |= D_GRAPH | D_LEGEND | D_READOUT; print_marker(n - 1); return;
usage:
    con_puts("usage: marker [1-4 [on|off|active|<point index>|minswr|max|min]...]\r\n");
}
static void print_ui(void)
{
    con_puts("mode "); con_puts(mode == MODE_GRID ? (grid_kind == G_KEYPAD ? "keypad" : "bands") : mode == MODE_TCAL ? "touchcal" : "main");
    con_puts(" menu "); con_puts(menu_open ? menu_name[menu_id] : "closed");
    if (mode == MODE_GRID) { con_puts(" selected \""); con_puts(grid_label(grid_sel)); con_putc('"'); }
    else if (menu_open) { char b[24]; item_label(menus[menu_id][menu_sel], b); con_puts(" selected \""); con_puts(b); con_putc('"'); }
    if (mode == MODE_GRID && grid_kind == G_KEYPAD) { con_puts(" entry \""); con_puts(kb); con_puts("\" for "); con_puts(target_name()); }
    con_puts(" touch "); con_puts(touch_ok ? "calibrated" : "uncalibrated");
    if (msg[0] && (int32_t)(msg_until - ms_ticks) > 0) { con_puts(" message \""); con_puts(msg); con_putc('"'); }
    con_puts("\r\n");
    if (menu_open && mode == MODE_MAIN) for (int r = 0; r < menu_len(menu_id); r++) {
        char b[24]; item_label(menus[menu_id][r], b);
        con_puts("  "); con_udec(r); con_putc(' '); con_puts(b); con_puts(r == menu_sel ? " <\r\n" : "\r\n");
    }
}
static void cmd_ui(int argc, char **argv)
{
    const char *a = argc > 1 ? argv[1] : "";
    if (!*a || streq(a, "state")) { print_ui(); return; }
    if (!ui_on || !lcd_ready) { con_puts("error: the screen is off (lcd ui)\r\n"); return; }
    int x, y;
    if (streq(a, "tap") && argc == 4 && decval(argv[2], &x) && decval(argv[3], &y) && x >= 0 && x < W && y >= 0 && y < H) tap(x, y);
    else if (streq(a, "jog") && argc == 3) {
        const char *j = argv[2];
        int d = j[0] == 'r' || j[0] == 'R' ? 1 : j[0] == 'l' || j[0] == 'L' ? -1 : 0;
        if (!d) { con_puts("usage: ui jog l|r|L|R|rr (capitals hold)\r\n"); return; }
        int hold = j[0] == 'L' || j[0] == 'R', reps = streq(j, "rr") ? 2 : 1;
        for (int k = 0; k < reps; k++) {
            jog_event(d, 1);
            if (hold) { uint32_t t0 = ms_ticks; while (ms_ticks - t0 < 750) service(); jog_event(d, 2); }
            if (jog_dir) jog_event(d, 0);
        }
    }
    else if (streq(a, "redraw")) dirty |= D_FULL;
    else { con_puts("usage: ui [state|tap <x> <y>|jog l|r|L|R|rr|redraw]\r\n"); return; }
    if (dirty) render();
    print_ui();
}
void ui_command(int argc, char **argv)
{
    const char *c = argv[0];
    if (streq(c, "trace")) { cmd_trace(argc, argv); if (ui_on && lcd_ready && dirty) render(); return; }
    if (streq(c, "marker")) { cmd_marker(argc, argv); if (ui_on && lcd_ready && dirty) render(); return; }
    if (streq(c, "ui")) { cmd_ui(argc, argv); return; }
    if (streq(c, "backlight")) {
        if (argc > 1) pin_set(P_BACKLIGHT, argv[1][0] == '1');
        con_puts(GPIO_ODT(P_BACKLIGHT >> 4) >> (P_BACKLIGHT & 15) & 1 ? "1\r\n" : "0\r\n");
        return;
    }
    if (streq(c, "touch")) {
        const char *a = argc > 1 ? argv[1] : "";
        if (streq(a, "cal")) {
            if (!ui_on || !lcd_ready) { con_puts("error: the screen is off (lcd ui)\r\n"); return; }
            menu_open = 0; touch_cal_start(); con_puts("ok: tap the three crosses on the screen\r\n"); return;
        }
        if (streq(a, "map")) {
            if (argc == 7) {
                float v[4]; int s;
                for (int k = 0; k < 4; k++) if (!parse_num(argv[2 + k], 1, &v[k])) { con_puts("usage: touch map [ax bx ay by swap]\r\n"); return; }
                if (!decval(argv[6], &s)) { con_puts("usage: touch map [ax bx ay by swap]\r\n"); return; }
                tm_ax = v[0]; tm_bx = v[1]; tm_ay = v[2]; tm_by = v[3]; tm_swap = s != 0; touch_ok = 1;
            } else if (argc != 2) { con_puts("usage: touch map [ax bx ay by swap]\r\n"); return; }
            if (!touch_ok) { con_puts("uncalibrated\r\n"); return; }
            con_puts("ax "); put_f(tm_ax, 6); con_puts(" bx "); put_f(tm_bx, 2); con_puts(" ay "); put_f(tm_ay, 6);
            con_puts(" by "); put_f(tm_by, 2); con_puts(" swap "); con_udec(tm_swap); con_puts("\r\n");
            return;
        }
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
        ui_on = 0; lcd_test_card(); con_puts("ok\r\n");
    } else if (streq(a, "ui")) {
        ui_start(!(argc > 2 && streq(argv[2], "nosweep")));
        con_puts("ok\r\n");
    } else if (streq(a, "off")) {
        ui_on = 0; lcd_off();
        con_puts("ok\r\n");
    } else if (streq(a, "capture")) {
        if (!lcd_ready) { con_puts("error: the screen is off\r\n"); return; }
        con_puts("capture 480 320\r\n");
        lcd_capture(1);
        if (!ui_on) lcd_test_card();
        else if (mode == MODE_TCAL) draw_cross(tcal_pt[tcal_step][0], tcal_pt[tcal_step][1]);
        else { dirty = D_FULL; render(); }
        lcd_capture(0);
        con_puts("end\r\n");
    } else if (streq(a, "madctl") && argc > 2) {
        uint32_t v; if (!hexval(argv[2], &v) || v > 0xFF) { con_puts("usage: lcd madctl <hex>\r\n"); return; }
        madctl = (uint8_t)v;
        if (lcd_ready) { lcd_cmd(0x36, &madctl, 1); if (ui_on) { dirty = D_FULL; render(); } else lcd_test_card(); }
        con_puts("ok\r\n");
    } else if (streq(a, "inv") && argc > 2) {
        inverted = argv[2][0] == '1';
        if (lcd_ready) lcd_cmd(inverted ? 0x21 : 0x20, 0, 0);
        con_puts("ok\r\n");
    } else if (streq(a, "marker") && argc > 2) {        /* kept from stage 8: the active marker's point */
        int v; if (!decval(argv[2], &v) || v < 0) v = 0;
        mk[mk_act].idx = v < sweep_cfg.points ? v : sweep_cfg.points - 1; mk[mk_act].on = 1;
        dirty |= D_GRAPH | D_LEGEND | D_READOUT; if (ui_on && lcd_ready) render();
        con_puts("ok\r\n");
    } else if (streq(a, "status") || !*a) {
        con_puts("lcd "); con_puts(lcd_ready ? "on" : "off"); con_puts(" ui "); con_puts(ui_on ? "on" : "off");
        con_puts(" madctl 0x"); con_hex(madctl, 2); con_puts(" inv "); con_udec(inverted);
        con_puts(" marker "); con_udec(mk[mk_act].idx); con_puts(" sweeping "); con_udec(sweep_running); con_puts("\r\n");
    } else con_puts("usage: lcd [init|test|ui [nosweep]|off|capture|madctl <hex>|inv 0|1|marker <i>|status]\r\n");
}
