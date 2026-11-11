/* AetherVNA firmware: the on-screen UI: traces, Smith and polar charts,
 * markers, the menu tree, touch and jog input, and their console mirror.
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The menu tree follows the LiteVNA's published menu map (its names and
 * layout, so the unit is used the same way); the code is our own.
 *
 * Layout (480x320): two legend rows at the top (one entry per trace, its
 * value at the active marker), a 440x256 graph (10x8 divisions) with the
 * Smith or polar chart centred on it, three readout lines below. The menu
 * is a column of buttons on the right, over the graph; numbers and band
 * presets are entered on full-screen grids.
 *
 * Input: touch (tap) and the jog. The jog's push is not readable (NOTES.md,
 * stage 8), so with the menu closed left/right move the marker and a double
 * click right opens the menu; with a menu or grid open a click moves the
 * highlight, holding right selects and holding left goes back.
 *
 * Everything here is also reachable from the console (`trace`, `marker`,
 * `display`, `ui`, `lcd capture`), so a host can drive and check the screen.
 *
 * Nothing is stored per point: values are computed from pts[] (calibration,
 * electrical delay, S21 offset and port impedance applied) as each
 * 32-column strip of the graph is drawn through adc_buf. */
#include "hw.h"
#include "vna.h"
#include "ui.h"
#include "lcd.h"

/* ---------- geometry and colours ---------- */
#define GX0 40
#define GX1 480                                      /* exclusive */
#define GW  (GX1 - GX0)
#define GY0 26
#define DIV_PX 32
#define NDIV 8
#define GY1 (GY0 + NDIV * DIV_PX)                    /* exclusive */
#define XDIV_PX 44
#define SCX (GX0 + GW / 2)                           /* Smith centre and radius */
#define SCY (GY0 + NDIV * DIV_PX / 2)
#define SR  127
#define MX  384                                      /* menu panel: x MX..W-1 */
#define MROW 32
#define MROWS 10
#define RY0 (GY1 + 1)                                /* readout lines */
#define RDY 12

#define C_BG    0x0000
#define C_GRID  0x31A6
#define C_SMITH 0x4A69
#define C_TEXT  0xFFFF
#define C_DIM   0x8410
#define C_OFF   0x4208
#define C_MARK  0xFFFF
#define C_MARK2 0xA514
#define C_BTN   0x2104
#define C_BTNHI 0x041F
#define C_EDGE  0x6B4D
static const uint16_t tcolor[4] = { 0xFFE0, 0x07FF, 0xF81F, 0x07E0 };
#define OHM "\x7f"
#define DEG "\x80"
#define MU  "\x81"
#define DELTA "\x82"

static inline float fsqrt(float x) { float r; __asm__("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x)); return r; }
static int iabs(int v) { return v < 0 ? -v : v; }
#define PI_F 3.14159265f
static void sincos_f(float x, float *s, float *c)  /* |error| < 2e-6 after range reduction */
{
    float k = (float)(int)(x * (1 / (2 * PI_F)) + (x >= 0 ? 0.5f : -0.5f));
    x -= k * 2 * PI_F;                               /* -pi..pi */
    int neg = 0; float y = x;
    if (y > PI_F / 2) y = PI_F - y, neg = 1;
    else if (y < -PI_F / 2) y = -PI_F - y, neg = 1;
    float y2 = y * y;
    *s = y * (1 - y2 / 6 * (1 - y2 / 20 * (1 - y2 / 42 * (1 - y2 / 72))));
    float cc = 1 - y2 / 2 * (1 - y2 / 12 * (1 - y2 / 30 * (1 - y2 / 56)));
    *c = neg ? -cc : cc;
}

/* ---------- formats ---------- */
enum { U_DB, U_DEG, U_NONE, U_OHM, U_S, U_F, U_H, U_SEC };
enum { K_RECT, K_SMITH, K_POLAR };
enum {
    F_LOGMAG, F_PHASE, F_DELAY, F_SMITH, F_SWR, F_R, F_X, F_Z, F_POLAR, F_LINEAR, F_REAL, F_IMAG,
    F_Q, F_G, F_B, F_Y, F_ZPHASE, F_SERC, F_SERL, F_PARR, F_PARX, F_PARC, F_PARL,
    F_S21SR, F_S21SX, F_S21HR, F_S21HX, F_S21Q, F_N
};
typedef struct { const char *name, *cmd; uint8_t unit, kind, refpos; float scale, ref; const char *menu; } fmt_t;   /* menu: shorter label, if needed */
static const fmt_t fmts[F_N] = {
    [F_LOGMAG] = { "LOGMAG", "logmag", U_DB, K_RECT, 7, 10, 0 },
    [F_PHASE]  = { "PHASE", "phase", U_DEG, K_RECT, 4, 90, 0 },
    [F_DELAY]  = { "DELAY", "delay", U_SEC, K_RECT, 4, 1e-8f, 0 },
    [F_SMITH]  = { "SMITH", "smith", U_NONE, K_SMITH, 0, 1, 0 },
    [F_SWR]    = { "SWR", "swr", U_NONE, K_RECT, 0, 1, 1 },
    [F_R]      = { "RESISTANCE", "r", U_OHM, K_RECT, 0, 25, 0, "RESIST R" },
    [F_X]      = { "REACTANCE", "x", U_OHM, K_RECT, 4, 50, 0, "REACT X" },
    [F_Z]      = { "|Z|", "z", U_OHM, K_RECT, 0, 25, 0 },
    [F_POLAR]  = { "POLAR", "polar", U_NONE, K_POLAR, 0, 1, 0 },
    [F_LINEAR] = { "LINEAR", "linear", U_NONE, K_RECT, 0, 0.125f, 0 },
    [F_REAL]   = { "REAL", "real", U_NONE, K_RECT, 4, 0.25f, 0 },
    [F_IMAG]   = { "IMAG", "imag", U_NONE, K_RECT, 4, 0.25f, 0 },
    [F_Q]      = { "Q FACTOR", "q", U_NONE, K_RECT, 0, 10, 0 },
    [F_G]      = { "CONDUCTANCE", "g", U_S, K_RECT, 0, 0.005f, 0, "CONDUCT G" },
    [F_B]      = { "SUSCEPTANCE", "b", U_S, K_RECT, 4, 0.01f, 0, "SUSCEPT B" },
    [F_Y]      = { "|Y|", "y", U_S, K_RECT, 0, 0.005f, 0 },
    [F_ZPHASE] = { "Z PHASE", "zphase", U_DEG, K_RECT, 4, 45, 0 },
    [F_SERC]   = { "SERIES C", "serc", U_F, K_RECT, 0, 1e-10f, 0 },
    [F_SERL]   = { "SERIES L", "serl", U_H, K_RECT, 0, 1e-7f, 0 },
    [F_PARR]   = { "PARALLEL R", "parr", U_OHM, K_RECT, 0, 25, 0 },
    [F_PARX]   = { "PARALLEL X", "parx", U_OHM, K_RECT, 4, 50, 0 },
    [F_PARC]   = { "PARALLEL C", "parc", U_F, K_RECT, 0, 1e-10f, 0 },
    [F_PARL]   = { "PARALLEL L", "parl", U_H, K_RECT, 0, 1e-6f, 0 },
    [F_S21SR]  = { "SERIES R", "s21sr", U_OHM, K_RECT, 0, 25, 0 },
    [F_S21SX]  = { "SERIES X", "s21sx", U_OHM, K_RECT, 4, 50, 0 },
    [F_S21HR]  = { "SHUNT R", "s21hr", U_OHM, K_RECT, 0, 25, 0 },
    [F_S21HX]  = { "SHUNT X", "s21hx", U_OHM, K_RECT, 4, 50, 0 },
    [F_S21Q]   = { "Q FACTOR", "s21q", U_NONE, K_RECT, 0, 10, 0 },
};
static const char *const unit_txt[] = { "dB", DEG, "", OHM, "S", "F", "H", "s" };
/* Smith-chart marker readouts: S11 and S21 sets, as in the menu map. */
static const char *const smode_s11[] = { "LIN", "LOG", "Re+Im", "R+jX", "R+L/C", "G+jB", "G+L/C", "Rp+jXp", "Rp+L/C" };
static const char *const smode_s21[] = { "LIN", "LOG", "Re+Im", "R+jX SHUNT", "R+jX SERIES" };
static const char *const smode_s21_menu[] = { "LIN", "LOG", "Re+Im", "SHUNT Z", "SERIES Z" };
static const char *const smode_cmd[] = { "lin", "log", "reim", "rjx", "rlc", "gjb", "glc", "rpjxp", "rplc" };

typedef struct { uint8_t on, ch, fmt, refpos, smode; float scale, ref; } trace_t;
static trace_t tr[4] = {
    { 1, 0, F_LOGMAG, 7, 3, 10, 0 },                 /* S11 log magnitude */
    { 1, 1, F_LOGMAG, 7, 1, 10, 0 },                 /* S21 log magnitude */
    { 1, 0, F_SMITH, 0, 3, 1, 0 },                   /* S11 Smith chart, R+jX */
    { 1, 1, F_PHASE, 4, 1, 90, 0 },                  /* S21 phase */
};
static int tr_act;

#define NMK 8
typedef struct { uint8_t on; uint16_t idx; } marker_t;
static marker_t mk[NMK] = { { 1, 0 } };
static int mk_act;
static uint8_t mk_delta, mk_tracking, search_min;  /* delta: values relative to marker 1 */
static uint32_t jog_step;                          /* Hz per jog click, 0 = one point */

/* Stored traces A-D: a rectangular trace's values at the time of storing. */
typedef struct { uint8_t on, fmt, ch, src, refpos; uint16_t n; float scale, ref; float v[POINTS_MAX]; } mem_t;
static mem_t mem[4];
static float edelay, s21_offset, port_z = 50;      /* s, dB, ohm */
enum { MS_OFF, MS_LC, MS_CABLE, MS_SHUNTLC, MS_SERIESL, MS_XTAL, MS_N };
static uint8_t meas_mode;
static float meas_rl = 50, meas_vf = 0.66f;
static const char *const meas_name[MS_N] = { "off", "lc", "cable", "shuntlc", "seriesl", "xtal" };

static uint8_t grid_values = 1, dot_grid, flipped;

/* ---------- UI state ---------- */
enum { MODE_MAIN, MODE_GRID, MODE_TCAL, MODE_TTEST, MODE_INFO };
enum { D_LEGEND = 1, D_GRAPH = 2, D_READOUT = 4, D_STATUS = 8, D_MENU = 16, D_FULL = 32 };
static uint8_t ui_on, have_data, boot_done, mode;
static uint16_t dirty;
#define UI_BOOT_MS 1500                              /* after USB has enumerated */
static uint32_t drawn_count;
static char msg[80]; static uint32_t msg_until;
static uint32_t boot_pending;                        /* BOOTLOAD waits for the jog to be released */

static void say(const char *s, uint32_t ms)
{
    int i = 0; while (s[i] && i < (int)sizeof msg - 1) { msg[i] = s[i]; i++; }
    msg[i] = 0; msg_until = ms_ticks + ms; dirty |= D_STATUS;
}

/* ---------- values ---------- */
static uint64_t point_freq(int i) { return pts[i].f_tuned ? pts[i].f_tuned : sweep_freq(i); }
static void point_vals(int i, cpx *s11, cpx *s21)
{
    *s11 = pts[i].s11; *s21 = pts[i].s21;
    if (cal_on && cal_valid()) cal_apply(i, s11, s21);
    if (edelay != 0) {                               /* remove a delay of edelay seconds */
        float s, c; sincos_f(2 * PI_F * (float)point_freq(i) * edelay, &s, &c);
        cpx r = c_mk(c, s);
        *s11 = c_mul(*s11, r); *s21 = c_mul(*s21, r);
    }
    if (s21_offset != 0) {
        float k = 1; float e = s21_offset / 20;     /* 10^e by exp2 */
        float l2 = e * 3.3219281f; int n = (int)l2; if ((float)n > l2) n--;
        float fr = (l2 - n) * 0.69314718f;
        float ex = 1 + fr * (1 + fr / 2 * (1 + fr / 3 * (1 + fr / 4 * (1 + fr / 5))));
        for (; n > 0; n--) k *= 2;
        for (; n < 0; n++) k *= 0.5f;
        k *= ex; s21->re *= k; s21->im *= k;
    }
    if (port_z != 50) {                              /* renormalise S11 to port_z */
        cpx d = c_sub(c_mk(1, 0), *s11);
        if (c_abs2(d) > 1e-12f) {
            cpx z = c_div(c_add(c_mk(1, 0), *s11), d); z.re *= 50; z.im *= 50;
            *s11 = c_div(c_sub(z, c_mk(port_z, 0)), c_add(z, c_mk(port_z, 0)));
        }
    }
}
static cpx point_g(int i, int ch) { cpx a, b; point_vals(i, &a, &b); return ch ? b : a; }
static cpx g_to_z(cpx g)                             /* at the port impedance */
{
    cpx d = c_sub(c_mk(1, 0), g);
    if (c_abs2(d) < 1e-12f) return c_mk(1e9f, 0);
    cpx z = c_div(c_add(c_mk(1, 0), g), d);
    return c_mk(port_z * z.re, port_z * z.im);
}
static cpx s21_series_z(cpx t)                       /* a series element between the ports */
{
    if (c_abs2(t) < 1e-12f) return c_mk(1e9f, 0);
    cpx z = c_div(c_sub(c_mk(1, 0), t), t); return c_mk(100 * z.re, 100 * z.im);
}
static cpx s21_shunt_z(cpx t)                        /* a shunt element across the line */
{
    cpx d = c_sub(c_mk(1, 0), t);
    if (c_abs2(d) < 1e-12f) return c_mk(1e9f, 0);
    cpx z = c_div(t, d); return c_mk(25 * z.re, 25 * z.im);
}
static float c_argr(cpx a) { return c_deg(a) * (PI_F / 180); }
static float group_delay(int i, int ch)
{
    int n = sweep_cfg.points; if (n < 2) return 0;
    int a = i > 0 ? i - 1 : i, b = i < n - 1 ? i + 1 : i;
    cpx ga = point_g(a, ch), gb = point_g(b, ch);
    float dphi = c_argr(c_mul(gb, c_mk(ga.re, -ga.im)));
    float df = (float)(point_freq(b) - point_freq(a));
    return df > 0 ? -dphi / (2 * PI_F * df) : 0;
}
static float fmt_val(int fmt, int ch, int i)
{
    cpx g = point_g(i, ch);
    float w = 2 * PI_F * (float)point_freq(i);
    cpx z;
    switch (fmt) {
    case F_LOGMAG: return c_db(g);
    case F_PHASE:  return c_deg(g);
    case F_DELAY:  return group_delay(i, ch);
    case F_SWR:    { float m = c_abs(g); return m < 0.999f ? (1 + m) / (1 - m) : 999; }
    case F_SMITH: case F_POLAR: case F_LINEAR: return c_abs(g);
    case F_REAL:   return g.re;
    case F_IMAG:   return g.im;
    case F_S21SR: case F_S21SX: case F_S21Q: z = s21_series_z(g); break;
    case F_S21HR: case F_S21HX: z = s21_shunt_z(g); break;
    default:       z = g_to_z(g); break;
    }
    float m2 = c_abs2(z);
    switch (fmt) {
    case F_R: case F_S21SR: case F_S21HR: return z.re;
    case F_X: case F_S21SX: case F_S21HX: return z.im;
    case F_Z: return fsqrt(m2);
    case F_Q: case F_S21Q: return z.re != 0 ? (z.im < 0 ? -z.im : z.im) / (z.re < 0 ? -z.re : z.re) : 1e6f;
    case F_G: return m2 > 0 ? z.re / m2 : 0;
    case F_B: return m2 > 0 ? -z.im / m2 : 0;
    case F_Y: return m2 > 0 ? 1 / fsqrt(m2) : 0;
    case F_ZPHASE: return c_deg(z);
    case F_SERC: return z.im != 0 ? -1 / (w * z.im) : 0;
    case F_SERL: return z.im / w;
    case F_PARR: return z.re != 0 ? m2 / z.re : 1e9f;
    case F_PARX: return z.im != 0 ? m2 / z.im : 1e9f;
    case F_PARC: return m2 != 0 ? -z.im / (w * m2) : 0;
    case F_PARL: return z.im != 0 ? m2 / (w * z.im) : 0;
    }
    return 0;
}
static float trace_val(int t, int i) { return fmt_val(tr[t].fmt, tr[t].ch, i); }
static int fkind(int t) { return fmts[tr[t].fmt].kind; }
static float y_of(int t, float v)                    /* graph row, 0 at top; may be off the graph */
{
    return (NDIV - tr[t].refpos) * DIV_PX - (v - tr[t].ref) / tr[t].scale * DIV_PX;
}
static int col_of_point(int i)
{
    int n = sweep_cfg.points;
    return n < 2 ? 0 : (i * (GW - 1) + (n - 1) / 2) / (n - 1);
}
static void clamp_markers(void)
{
    for (int m = 0; m < NMK; m++) if (mk[m].idx >= sweep_cfg.points) mk[m].idx = sweep_cfg.points - 1;
}
static int any_kind(int k) { for (int t = 0; t < 4; t++) if (tr[t].on && fkind(t) == k) return 1; return 0; }

/* ---------- number formatting ---------- */
static char *put_eng(char *p, float v, const char *unit)   /* SI prefix, 4 significant digits */
{
    static const char *const pre[] = { "f", "p", "n", MU, "m", "", "k", "M", "G" };
    float a = v < 0 ? -v : v; int e = 5;
    if (a > 0) {
        while (a < 1 && e > 0) { a *= 1000; v *= 1000; e--; }
        while (a >= 1000 && e < 8) { a /= 1000; v /= 1000; e++; }
    }
    p = put_fx(p, v, a < 10 ? 3 : a < 100 ? 2 : 1);
    *p++ = ' '; p = put_s(p, pre[e]);
    return put_s(p, unit);
}
static char *put_short_eng(char *p, float v)       /* axis labels: 3 digits and a prefix */
{
    static const char *const pre[] = { "f", "p", "n", MU, "m", "", "k", "M", "G" };
    float a = v < 0 ? -v : v; int e = 5;
    if (a > 0) {
        while (a < 1 && e > 0) { a *= 1000; v *= 1000; e--; }
        while (a >= 1000 && e < 8) { a /= 1000; v /= 1000; e++; }
    }
    float fr = a - (float)(int)(a + 0.5f); if (fr < 0) fr = -fr;
    p = put_fx(p, v, a < 10 && fr > 0.05f ? 1 : 0);
    return put_s(p, pre[e]);
}
static char *put_val(char *p, int fmt, float v)    /* a value with its unit */
{
    int u = fmts[fmt].unit;
    switch (u) {
    case U_DB: p = put_sfx(p, v, 2); return put_s(p, " dB");
    case U_DEG: p = put_sfx(p, v, 1); return put_s(p, DEG);
    case U_NONE: return put_fx(p, v, fmt == F_SWR ? 3 : 4);
    case U_OHM: p = put_sfx(p, v, 2); *p++ = ' '; return put_s(p, OHM);
    default: return put_eng(p, v, unit_txt[u]);
    }
}
static char *put_scale(char *p, int fmt, float s)
{
    int u = fmts[fmt].unit;
    if (u >= U_S) return put_eng(p, s, unit_txt[u]);
    float a = s < 0 ? -s : s;
    p = put_fx(p, s, a >= 10 ? 0 : a >= 1 ? 1 : a >= 0.1f ? 2 : 4);
    return put_s(p, unit_txt[u]);
}
static char *put_rx(char *p, cpx z, int dec)        /* "R+jX" */
{
    p = put_fx(p, z.re, dec); *p++ = z.im < 0 ? '-' : '+'; *p++ = 'j'; p = put_fx(p, z.im < 0 ? -z.im : z.im, dec);
    return p;
}
static char *put_lc(char *p, float x, float w)      /* reactance as the equivalent L or C */
{
    return x >= 0 ? put_eng(p, x / w, "H") : put_eng(p, -1 / (w * x), "F");
}
static char *put_smith(char *p, int t, int i)       /* the Smith readout chosen for trace t */
{
    cpx g = point_g(i, tr[t].ch); float w = 2 * PI_F * (float)point_freq(i);
    int m = tr[t].smode;
    if (m == 0) { p = put_fx(p, c_abs(g), 4); *p++ = ' '; p = put_sfx(p, c_deg(g), 1); return put_s(p, DEG); }
    if (m == 1) { p = put_sfx(p, c_db(g), 2); p = put_s(p, " dB "); p = put_sfx(p, c_deg(g), 1); return put_s(p, DEG); }
    if (m == 2) { p = put_sfx(p, g.re, 4); *p++ = g.im < 0 ? '-' : '+'; *p++ = 'j'; return put_fx(p, g.im < 0 ? -g.im : g.im, 4); }
    cpx z;
    if (tr[t].ch) z = m == 3 ? s21_shunt_z(g) : s21_series_z(g);
    else z = g_to_z(g);
    if (tr[t].ch || m == 3) { p = put_rx(p, z, 2); *p++ = ' '; return put_s(p, OHM); }
    float m2 = c_abs2(z);
    if (m == 4) { p = put_fx(p, z.re, 2); p = put_s(p, OHM " "); return put_lc(p, z.im, w); }
    if (m == 5 || m == 6) {
        cpx y = m2 > 0 ? c_mk(z.re / m2, -z.im / m2) : c_mk(0, 0);
        if (m == 5) { p = put_eng(p, y.re, "S"); *p++ = ' '; *p++ = y.im < 0 ? '-' : '+'; *p++ = 'j'; return put_eng(p, y.im < 0 ? -y.im : y.im, "S"); }
        p = put_eng(p, y.re, "S"); *p++ = ' ';
        return y.im != 0 ? put_lc(p, -1 / y.im, w) : put_s(p, "-");
    }
    float rp = z.re != 0 ? m2 / z.re : 1e9f, xp = z.im != 0 ? m2 / z.im : 1e9f;
    if (m == 7) { p = put_rx(p, c_mk(rp, xp), 1); *p++ = ' '; return put_s(p, OHM); }
    p = put_fx(p, rp, 1); p = put_s(p, OHM " "); return put_lc(p, xp, w);
}
/* Text that stops at xmax (the menu's edge when it is open), the rest up to xmax cleared. */
static void line_text(int x, int y, const char *s, uint16_t fg, int xmax)
{
    int e = *s ? ftext(x, y, s, &font_s, fg, C_BG, xmax) : x;
    if (e < xmax) lcd_fill(e, y, xmax - e, font_s.h, C_BG);
}

/* ---------- menus ---------- */
enum {
    MN_HOME, MN_DISPLAY, MN_TRACE, MN_F11, MN_F11B, MN_F11C, MN_S11SM, MN_F21, MN_F21B, MN_S21SM, MN_SCALE, MN_AVG,
    MN_MARKER, MN_SELMK, MN_OPS, MN_STIM, MN_CFGSW, MN_CALMENU, MN_CAL, MN_CONFIG, MN_EXPERT, MN_SAVE, MN_RECALL, MN_SD, MN_SDLOAD, MN_MEAS, MN_N
};
enum {
    A_NONE, A_BACK,
    /* submenus: A_SUB + menu id */
    A_SUB = 2, A_SUB_END = A_SUB + MN_N,
    A_PAUSE = A_SUB_END, A_RECALL, A_MEASURE, A_SDCARD,
    A_TRACE0, A_TRACE3 = A_TRACE0 + 3, A_STORE0, A_STORE3 = A_STORE0 + 3,
    A_F0, A_FEND = A_F0 + F_N,                       /* set format (and channel by menu) */
    A_SM0 = A_FEND, A_SMEND = A_SM0 + 9,             /* Smith readout */
    A_CHANNEL = A_SMEND, A_SCALEDIV, A_REFVAL, A_REFPOS, A_EDELAY, A_S21OFF, A_GRIDVAL, A_DOTGRID, A_AUTO,
    A_TRANSFORM, A_AVG0, A_AVGEND = A_AVG0 + 8, A_PORTZ = A_AVGEND,
    A_MK0, A_MKEND = A_MK0 + NMK, A_MKALLOFF = A_MKEND, A_DELTA, A_SEARCH, A_SLEFT, A_SRIGHT, A_TRACKING,
    A_OSTART, A_OSTOP, A_OCENTER, A_OSPAN, A_OEDELAY,
    A_START, A_STOP, A_CENTER, A_SPAN, A_CW, A_JOGSTEP, A_POINTS, A_TXPOWER, A_BANDS,
    A_COPEN, A_CSHORT, A_CLOAD, A_CISOLN, A_CTHRU, A_CDONE, A_CDONERAM, A_CSTD,
    A_CSAVE, A_CRANGE, A_CRESET, A_CRESETALL, A_CAPPLY, A_CENH,
    A_TOUCHCAL, A_TOUCHTEST, A_SAVECFG, A_VERSION, A_BRIGHT, A_FLIP, A_CLEARCFG, A_BOOTLOAD,
    A_SAVE0, A_SAVEEND = A_SAVE0 + 7, A_RCL0 = A_SAVEEND, A_RCLEND = A_RCL0 + 7,
    A_SDS1P = A_RCLEND, A_SDS2P, A_SDSCREEN, A_SDSCAL, A_SDAUTO, A_FILE0, A_FILEEND = A_FILE0 + 8,
    A_MS0 = A_FILEEND, A_MSEND = A_MS0 + 6, A_MSRL = A_MSEND, A_MSVF,
    A_N
};
#define SUB(m) (A_SUB + (m))
static uint8_t menus[MN_N][MROWS] = {                /* MN_SDLOAD is filled from the card */
    [MN_HOME]    = { SUB(MN_DISPLAY), SUB(MN_MARKER), SUB(MN_STIM), SUB(MN_CALMENU), SUB(MN_RECALL), SUB(MN_MEAS), SUB(MN_SD), SUB(MN_CONFIG), A_PAUSE },
    [MN_DISPLAY] = { SUB(MN_TRACE), SUB(MN_F11), SUB(MN_F21), A_CHANNEL, SUB(MN_SCALE), A_TRANSFORM, SUB(MN_AVG), A_PORTZ, A_BACK },
    [MN_TRACE]   = { A_TRACE0, A_TRACE0 + 1, A_TRACE0 + 2, A_TRACE0 + 3, A_STORE0, A_STORE0 + 1, A_STORE0 + 2, A_STORE0 + 3, A_BACK },
    [MN_F11]     = { A_F0 + F_LOGMAG, A_F0 + F_PHASE, A_F0 + F_DELAY, SUB(MN_S11SM), A_F0 + F_SWR, A_F0 + F_R, A_F0 + F_X, A_F0 + F_Z, SUB(MN_F11B), A_BACK },
    [MN_F11B]    = { A_F0 + F_POLAR, A_F0 + F_LINEAR, A_F0 + F_REAL, A_F0 + F_IMAG, A_F0 + F_Q, A_F0 + F_G, A_F0 + F_B, A_F0 + F_Y, SUB(MN_F11C), A_BACK },
    [MN_F11C]    = { A_F0 + F_ZPHASE, A_F0 + F_SERC, A_F0 + F_SERL, A_F0 + F_PARR, A_F0 + F_PARX, A_F0 + F_PARC, A_F0 + F_PARL, A_BACK },
    [MN_S11SM]   = { A_SM0, A_SM0 + 1, A_SM0 + 2, A_SM0 + 3, A_SM0 + 4, A_SM0 + 5, A_SM0 + 6, A_SM0 + 7, A_SM0 + 8, A_BACK },
    [MN_F21]     = { A_F0 + F_LOGMAG, A_F0 + F_PHASE, A_F0 + F_DELAY, SUB(MN_S21SM), A_F0 + F_POLAR, A_F0 + F_LINEAR, A_F0 + F_REAL, A_F0 + F_IMAG, SUB(MN_F21B), A_BACK },
    [MN_F21B]    = { A_F0 + F_S21SR, A_F0 + F_S21SX, A_F0 + F_S21HR, A_F0 + F_S21HX, A_F0 + F_S21Q, A_BACK },
    [MN_S21SM]   = { A_SM0, A_SM0 + 1, A_SM0 + 2, A_SM0 + 3, A_SM0 + 4, A_BACK },
    [MN_SCALE]   = { A_SCALEDIV, A_REFVAL, A_REFPOS, A_EDELAY, A_S21OFF, A_GRIDVAL, A_DOTGRID, A_AUTO, A_BACK },
    [MN_AVG]     = { A_AVG0, A_AVG0 + 1, A_AVG0 + 2, A_AVG0 + 3, A_AVG0 + 4, A_AVG0 + 5, A_AVG0 + 6, A_AVG0 + 7, A_BACK },
    [MN_MARKER]  = { SUB(MN_SELMK), A_SEARCH, A_SLEFT, A_SRIGHT, SUB(MN_OPS), A_TRACKING, A_BACK },
    [MN_SELMK]   = { A_MK0, A_MK0 + 1, A_MK0 + 2, A_MK0 + 3, A_MK0 + 4, A_MK0 + 5, A_MK0 + 6, A_MK0 + 7, A_MKALLOFF, A_BACK },
    [MN_OPS]     = { A_OSTART, A_OSTOP, A_OCENTER, A_OSPAN, A_OEDELAY, A_BACK },
    [MN_STIM]    = { A_START, A_STOP, A_CENTER, A_SPAN, A_CW, A_JOGSTEP, SUB(MN_CFGSW), A_BANDS, A_BACK },
    [MN_CFGSW]   = { A_POINTS, A_TXPOWER, A_BACK },
    [MN_CALMENU] = { SUB(MN_CAL), SUB(MN_SAVE), A_CRANGE, A_CRESET, A_CRESETALL, A_CAPPLY, A_CENH, A_BACK },
    [MN_CAL]     = { A_COPEN, A_CSHORT, A_CLOAD, A_CISOLN, A_CTHRU, A_CDONE, A_CDONERAM, A_CSTD, A_BACK },
    [MN_CONFIG]  = { A_TOUCHCAL, A_TOUCHTEST, SUB(MN_EXPERT), A_SAVECFG, A_VERSION, A_BRIGHT, A_BACK },
    [MN_EXPERT]  = { A_FLIP, A_CLEARCFG, A_BOOTLOAD, A_BACK },
    [MN_SAVE]    = { A_SAVE0, A_SAVE0 + 1, A_SAVE0 + 2, A_SAVE0 + 3, A_SAVE0 + 4, A_SAVE0 + 5, A_SAVE0 + 6, A_BACK },
    [MN_RECALL]  = { A_RCL0, A_RCL0 + 1, A_RCL0 + 2, A_RCL0 + 3, A_RCL0 + 4, A_RCL0 + 5, A_RCL0 + 6, A_BACK },
    [MN_SD]      = { A_SDS1P, A_SDS2P, A_SDSCREEN, A_SDSCAL, A_SDAUTO, SUB(MN_SDLOAD), A_BACK },
    [MN_SDLOAD]  = { A_FILE0, A_FILE0 + 1, A_FILE0 + 2, A_FILE0 + 3, A_FILE0 + 4, A_FILE0 + 5, A_FILE0 + 6, A_FILE0 + 7, A_BACK },
    [MN_MEAS]    = { A_MS0, A_MS0 + 1, A_MS0 + 2, A_MS0 + 3, A_MS0 + 4, A_MS0 + 5, A_MSRL, A_MSVF, A_BACK },
};
static const uint8_t menu_parent[MN_N] = {
    [MN_HOME] = MN_HOME, [MN_DISPLAY] = MN_HOME, [MN_TRACE] = MN_DISPLAY, [MN_F11] = MN_DISPLAY, [MN_F11B] = MN_F11,
    [MN_F11C] = MN_F11B, [MN_S11SM] = MN_F11, [MN_F21] = MN_DISPLAY, [MN_F21B] = MN_F21, [MN_S21SM] = MN_F21,
    [MN_SCALE] = MN_DISPLAY, [MN_AVG] = MN_DISPLAY, [MN_MARKER] = MN_HOME, [MN_SELMK] = MN_MARKER, [MN_OPS] = MN_MARKER,
    [MN_STIM] = MN_HOME, [MN_CFGSW] = MN_STIM, [MN_CALMENU] = MN_HOME, [MN_CAL] = MN_CALMENU, [MN_CONFIG] = MN_HOME,
    [MN_EXPERT] = MN_CONFIG, [MN_SAVE] = MN_CALMENU, [MN_RECALL] = MN_HOME, [MN_SD] = MN_HOME, [MN_SDLOAD] = MN_SD, [MN_MEAS] = MN_HOME,
};
static const char *const menu_name[MN_N] = {
    "home", "display", "trace", "format_s11", "format_s11_more", "format_s11_more2", "s11_smith", "format_s21",
    "format_s21_more", "s21_smith", "scale", "avg", "marker", "select_marker", "operations", "stimulus", "cfg_sweep",
    "calibrate", "calibrate_standards", "config", "expert", "save", "recall", "sd_card", "sd_load", "measure" };
static const uint8_t avg_vals[8] = { AVG_AUTO, 1, 2, 4, 8, 16, 32, 64 };
static uint8_t menu_open, menu_id, menu_sel;
static char sd_files[8][13]; static int sd_nfiles; static uint8_t sd_autoname = 1;
static int menu_len(int id) { int n = 0; while (n < MROWS && menus[id][n]) n++; return n; }
static uint32_t confirm_t; static uint8_t confirm_a;  /* actions that ask twice */
static int confirming(int a) { return confirm_a == a && ms_ticks - confirm_t < 3000; }

enum { CK_NONE, CK_OFF, CK_ON, CK_DIM };            /* item decoration */
static int item_label(int a, char *b)
{
    static const char *const sub_n[MN_N] = {
        "HOME", "DISPLAY", "TRACE", "FORMAT S11", ">MORE", ">MORE", "SMITH", "FORMAT S21", ">MORE", "SMITH",
        "SCALE", "AVG", "MARKER", "SELECT MK", "OPERATIONS", "STIMULUS", "CFG SWEEP", "CALIBRATE", "CALIBRATE",
        "CONFIG", "EXPERT", "SAVE", "RECALL", "SD CARD", "LOAD", "MEASURE" };
    char *p = b; *p = 0;
    if (a == A_BACK) { put_s(p, "< BACK"); return CK_NONE; }
    if (a >= A_SUB && a < A_SUB_END) { put_s(p, sub_n[a - A_SUB]); return CK_NONE; }
    if (a >= A_STORE0 && a <= A_STORE3) {
        int k = a - A_STORE0; p = put_s(p, "STORE "); *p++ = (char)('A' + k); *p = 0;
        return mem[k].on ? CK_ON : CK_OFF;
    }
    if (a >= A_TRACE0 && a <= A_TRACE3) {
        int t = a - A_TRACE0; p = put_s(p, "TRACE "); p = put_u(p, t + 1, 1); if (t == tr_act) put_s(p, " <");
        return tr[t].on ? CK_ON : CK_OFF;
    }
    if (a >= A_F0 && a < A_FEND) {
        int f = a - A_F0; put_s(p, fmts[f].menu ? fmts[f].menu : fmts[f].name);
        int ch = menu_id == MN_F21 || menu_id == MN_F21B;
        return tr[tr_act].fmt == f && tr[tr_act].ch == ch ? CK_ON : CK_NONE;
    }
    if (a >= A_SM0 && a < A_SMEND) {
        int m = a - A_SM0; int s21 = menu_id == MN_S21SM;
        put_s(p, s21 ? smode_s21_menu[m] : smode_s11[m]);
        return tr[tr_act].fmt == F_SMITH && tr[tr_act].ch == s21 && tr[tr_act].smode == m ? CK_ON : CK_NONE;
    }
    if (a >= A_AVG0 && a < A_AVGEND) {
        int k = a - A_AVG0;
        if (!k) put_s(p, "AUTO"); else if (k == 1) put_s(p, "NONE"); else { p = put_u(p, avg_vals[k], 1); put_s(p, "x"); }
        return sweep_cfg.avg == avg_vals[k] ? CK_ON : CK_NONE;
    }
    if (a >= A_FILE0 && a < A_FILEEND) {
        int k = a - A_FILE0;
        if (k >= sd_nfiles) { put_s(p, "-"); return CK_DIM; }
        for (int j = 0; sd_files[k][j] && sd_files[k][j] != '.'; j++) *p++ = sd_files[k][j];
        *p = 0; return CK_NONE;
    }
    if (a >= A_MS0 && a < A_MSEND) {
        static const char *const n[] = { "OFF", "L/C MATCH", "CABLE S11", "SHUNT LC", "SERIES L", "XTAL" };
        put_s(p, n[a - A_MS0]); return meas_mode == a - A_MS0 ? CK_ON : CK_OFF;
    }
    switch (a) {
    case A_MSRL: p = put_s(p, "Rl "); p = put_fx(p, meas_rl, meas_rl == (int)meas_rl ? 0 : 1); put_s(p, OHM); return CK_NONE;
    case A_MSVF: p = put_s(p, "VF "); put_fx(p, meas_vf, 2); return CK_NONE;
    case A_SDS1P: put_s(p, "SAVE S1P"); return CK_NONE;
    case A_SDS2P: put_s(p, "SAVE S2P"); return CK_NONE;
    case A_SDSCREEN: put_s(p, "SAVE SCREEN"); return CK_NONE;
    case A_SDSCAL: put_s(p, "SAVE CAL"); return CK_NONE;
    case A_SDAUTO: put_s(p, "AUTONAME"); return sd_autoname ? CK_ON : CK_OFF;
    }
    if (a >= A_SAVE0 && a < A_RCLEND) {
        int n = a >= A_RCL0 ? a - A_RCL0 : a - A_SAVE0;
        p = put_s(p, a >= A_RCL0 ? "RECALL " : "SAVE "); put_u(p, n, 1);
        if (a >= A_RCL0 && !store_valid(n)) return CK_DIM;
        return store_valid(n) ? CK_ON : CK_OFF;
    }
    if (a >= A_MK0 && a < A_MKEND) {
        int m = a - A_MK0; p = put_s(p, "MARKER "); p = put_u(p, m + 1, 1); if (m == mk_act) put_s(p, " <");
        return mk[m].on ? CK_ON : CK_OFF;
    }
    switch (a) {
    case A_PAUSE: put_s(p, "PAUSE"); return sweep_running ? CK_OFF : CK_ON;
    case A_CHANNEL: put_s(p, tr[tr_act].ch ? "CHAN S21" : "CHAN S11"); return CK_NONE;
    case A_SCALEDIV: put_s(p, "SCALE/DIV"); return CK_NONE;
    case A_REFVAL: put_s(p, "REF VALUE"); return CK_NONE;
    case A_REFPOS: put_s(p, "REF POS"); return CK_NONE;
    case A_EDELAY: put_s(p, "E-DELAY"); return CK_NONE;
    case A_S21OFF: put_s(p, "S21 OFFSET"); return CK_NONE;
    case A_GRIDVAL: put_s(p, "GRID VALS"); return grid_values ? CK_ON : CK_OFF;
    case A_DOTGRID: put_s(p, "DOT GRID"); return dot_grid ? CK_ON : CK_OFF;
    case A_AUTO: put_s(p, "AUTOSCALE"); return CK_NONE;
    case A_TRANSFORM: put_s(p, "TRANSFORM"); return CK_DIM;
    case A_PORTZ: p = put_s(p, "PORT-Z "); p = put_fx(p, port_z, port_z == (int)port_z ? 0 : 1); put_s(p, OHM); return CK_NONE;
    case A_MKALLOFF: put_s(p, "ALL OFF"); return CK_NONE;
    case A_DELTA: put_s(p, "DELTA"); return mk_delta ? CK_ON : CK_OFF;
    case A_SEARCH: put_s(p, search_min ? "SEARCH MIN" : "SEARCH MAX"); return CK_NONE;
    case A_SLEFT: put_s(p, "< SEARCH"); return CK_NONE;
    case A_SRIGHT: put_s(p, "SEARCH >"); return CK_NONE;
    case A_TRACKING: put_s(p, "TRACKING"); return mk_tracking ? CK_ON : CK_OFF;
    case A_OSTART: put_s(p, ">START"); return CK_NONE;
    case A_OSTOP: put_s(p, ">STOP"); return CK_NONE;
    case A_OCENTER: put_s(p, ">CENTER"); return CK_NONE;
    case A_OSPAN: put_s(p, ">SPAN"); return CK_NONE;
    case A_OEDELAY: put_s(p, ">E-DELAY"); return CK_NONE;
    case A_START: put_s(p, "START"); return CK_NONE;
    case A_STOP: put_s(p, "STOP"); return CK_NONE;
    case A_CENTER: put_s(p, "CENTER"); return CK_NONE;
    case A_SPAN: put_s(p, "SPAN"); return CK_NONE;
    case A_CW: put_s(p, "CW FREQ"); return CK_NONE;
    case A_JOGSTEP: put_s(p, "JOG STEP"); return CK_NONE;
    case A_POINTS: p = put_s(p, "POINTS "); put_u(p, sweep_cfg.points, 1); return CK_NONE;
    case A_TXPOWER: put_s(p, "TX POWER"); return CK_DIM;
    case A_BANDS: put_s(p, "HF BANDS"); return CK_NONE;
    case A_COPEN: case A_CSHORT: case A_CLOAD: case A_CTHRU: {
        static const char *const n[] = { "OPEN", "SHORT", "LOAD", "THRU" };
        int k = a == A_CTHRU ? CAL_THRU : a - A_COPEN; put_s(p, n[k]);
        return cal_have & (1 << k) ? CK_ON : CK_OFF; }
    case A_CISOLN: put_s(p, "ISOLN"); return cal_have & CAL_ISO_BIT ? CK_ON : CK_OFF;
    case A_CDONE: put_s(p, "DONE"); return CK_NONE;
    case A_CDONERAM: put_s(p, "DONE IN RAM"); return CK_NONE;
    case A_CSTD: put_s(p, "CAL KIT"); return CK_DIM;
    case A_CSAVE: put_s(p, "SAVE"); return CK_DIM;
    case A_CRANGE: put_s(p, "RESET RANGE"); return CK_NONE;
    case A_CRESET: put_s(p, confirming(a) ? "SURE?" : "RESET"); return CK_NONE;
    case A_CRESETALL: put_s(p, confirming(a) ? "SURE?" : "RESET ALL"); return CK_NONE;
    case A_CAPPLY: put_s(p, "APPLY"); return cal_on && cal_valid() ? CK_ON : CK_OFF;
    case A_CENH: put_s(p, "ENH RESP"); return cal_enhanced ? CK_ON : CK_OFF;
    case A_TOUCHCAL: put_s(p, "TOUCH CAL"); return CK_NONE;
    case A_TOUCHTEST: put_s(p, "TOUCH TEST"); return CK_NONE;
    case A_SAVECFG: put_s(p, "SAVE CONFIG"); return CK_NONE;
    case A_VERSION: put_s(p, "VERSION"); return CK_NONE;
    case A_BRIGHT: put_s(p, "BRIGHTNESS"); return CK_DIM;
    case A_FLIP: put_s(p, "FLIP"); return flipped ? CK_ON : CK_OFF;
    case A_CLEARCFG: put_s(p, confirming(a) ? "SURE?" : "CLEAR CFG"); return CK_NONE;
    case A_BOOTLOAD: put_s(p, confirming(a) ? "SURE?" : ">BOOTLOAD"); return CK_NONE;
    }
    return CK_NONE;
}
static void draw_menu(void)
{
    int n = menu_len(menu_id);
    for (int r = 0; r < MROWS; r++) {
        int y = r * MROW;
        if (r >= n) { lcd_fill(MX, y, W - MX, MROW, C_BG); continue; }
        char b[28]; int ck = item_label(menus[menu_id][r], b);
        uint16_t bg = r == menu_sel ? C_BTNHI : C_BTN, fg = ck == CK_DIM ? C_OFF : C_TEXT;
        lcd_fill(MX, y, W - MX, 1, C_EDGE);
        lcd_fill(MX, y + 1, 1, MROW - 2, C_EDGE);
        lcd_fill(MX + 1, y + 1, W - MX - 1, MROW - 2, bg);
        lcd_fill(MX, y + MROW - 1, W - MX, 1, C_BG);
        int x = MX + 5;
        if (ck == CK_ON || ck == CK_OFF) {           /* a check box, as on the stock menus */
            int by = y + (MROW - 9) / 2;
            lcd_fill(x, by, 9, 9, fg);
            lcd_fill(x + 1, by + 1, 7, 7, bg);
            if (ck == CK_ON) lcd_fill(x + 2, by + 2, 5, 5, fg);
            x += 13;
        }
        ftext(x, y + (MROW - font_s.h) / 2, b, &font_s, fg, bg, W - 2);   /* one size for every item (maintainer's choice) */
    }
}
static void menu_show(int id) { if (!menu_open) dirty |= D_LEGEND; menu_open = 1; menu_id = id; menu_sel = 0; dirty |= D_MENU; }
static void menu_close(void) { if (menu_open) { menu_open = 0; dirty |= D_FULL; } }

/* ---------- full-screen grids: keypad and band presets ---------- */
enum { G_KEYPAD, G_BANDS };
static uint8_t grid_kind, grid_sel, kp_target;
static char kb[20];
#define KP_COLS 6
static const char *const kp_keys[24] = {
    "7", "8", "9", "G", "M", "k",
    "4", "5", "6", MU, "n", "p",
    "1", "2", "3", "-", "BS", "CANCEL",
    "0", ".", "ENTER", "", "", "" };
static const struct { const char *n; uint32_t a, b; } bands[] = {
    { "160m", 1800000, 2000000 }, { "80m", 3500000, 4000000 }, { "60m", 5330500, 5406500 }, { "40m", 7000000, 7300000 },
    { "30m", 10100000, 10150000 }, { "20m", 14000000, 14350000 }, { "17m", 18068000, 18168000 }, { "15m", 21000000, 21450000 },
    { "12m", 24890000, 24990000 }, { "10m", 28000000, 29700000 }, { "6m", 50000000, 54000000 }, { "HF 0-55", 50000, 55050000 },
    { "CANCEL", 0, 0 } };
#define NBANDS ((int)(sizeof bands / sizeof bands[0]))
static int grid_cols(void) { return grid_kind == G_KEYPAD ? KP_COLS : 4; }
static int grid_cells(void) { return grid_kind == G_KEYPAD ? 24 : 16; }
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
    case A_CW: return "CW FREQUENCY (Hz; k M G)";
    case A_JOGSTEP: return "JOG STEP (Hz; 0 = one point)";
    case A_POINTS: return "SWEEP POINTS (1-401)";
    case A_SCALEDIV: return "SCALE PER DIVISION";
    case A_REFVAL: return "REFERENCE VALUE";
    case A_REFPOS: return "REFERENCE POSITION (0-8 divisions)";
    case A_EDELAY: return "ELECTRICAL DELAY (s; n p " MU ")";
    case A_S21OFF: return "S21 OFFSET (dB)";
    case A_MSRL: return "MEASURE: TERMINATION Rl (" OHM ")";
    case A_MSVF: return "VELOCITY FACTOR (0.1-1)";
    default: return "PORT IMPEDANCE (" OHM ")";
    }
}
static void draw_grid(void)
{
    int cols = grid_cols(), cw = W / cols, ch = (H - 64) / 4;
    lcd_fill(0, 0, W, 64, C_BG);
    ftext(8, 6, grid_kind == G_KEYPAD ? target_name() : "HF BAND PRESETS (keeps the point count)", &font_m, C_DIM, C_BG, W);
    if (grid_kind == G_KEYPAD) { char b[24]; char *p = put_s(b, kb); put_s(p, "_"); ftext(8, 32, b, &font_l, C_TEXT, C_BG, W); }
    for (int k = 0; k < grid_cells(); k++) {
        int x = (k % cols) * cw, y = 64 + (k / cols) * ch;
        const char *l = grid_label(k);
        if (!*l) { lcd_fill(x, y, cw, ch, C_BG); continue; }
        uint16_t bg = k == grid_sel ? C_BTNHI : C_BTN;
        lcd_fill(x, y, cw, ch, C_EDGE);
        lcd_fill(x + 1, y + 1, cw - 2, ch - 2, bg);
        const font_t *f = ftext_w(l, &font_l) <= cw - 8 ? &font_l : &font_m;
        if (ftext_w(l, f) > cw - 8) f = &font_s;
        ftext(x + (cw - ftext_w(l, f)) / 2, y + (ch - f->h) / 2, l, f, C_TEXT, bg, x + cw - 1);
    }
}
static void grid_show(int kind, int target)
{
    menu_open = 0; mode = MODE_GRID; grid_kind = kind; kp_target = target; grid_sel = 0; kb[0] = 0;
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
    uint64_t ip = 0, fp = 0, fd = 1; int dot = 0, any = 0, e = 0, eneg = 0;
    for (; *s; s++) {
        if (*s == 'e' || *s == 'E') {                /* console: 1e-9 */
            s++; if (*s == '-') { eneg = 1; s++; } else if (*s == '+') s++;
            if (!*s) return 0;
            for (; *s; s++) { if (*s < '0' || *s > '9') return 0; e = e * 10 + (*s - '0'); }
            break;
        }
        if (*s == '.') { if (dot) return 0; dot = 1; continue; }
        if (*s < '0' || *s > '9') return 0;
        any = 1;
        if (dot) { if (fd < 1000000ull) { fp = fp * 10 + (*s - '0'); fd *= 10; } }
        else ip = ip * 10 + (*s - '0');
    }
    if (!any) return 0;
    float v = ((float)ip + (float)fp / (float)fd) * mult;
    for (; e > 0; e--) v = eneg ? v / 10 : v * 10;
    *out = neg ? -v : v;
    return 1;
}
/* Scale from 1-2-5 steps so the trace spans the graph; reference at the top line. */
static void autoscale(int t)
{
    if (fkind(t) != K_RECT || !have_data) return;
    float lo = 1e30f, hi = -1e30f;
    for (int i = 0; i < sweep_cfg.points; i++) { float v = trace_val(t, i); if (v < lo) lo = v; if (v > hi) hi = v; }
    float s = 1e-15f;
    while (s < 1e12f) {
        float k = 0;
        for (int j = 0; j < 3; j++) {
            float c = s * (j == 0 ? 1 : j == 1 ? 2 : 5);
            float top = hi / c; int n = (int)top; if ((float)n < top) n++;   /* ceiling */
            if (n * c - NDIV * c <= lo) { k = c; break; }
        }
        if (k > 0) { s = k; break; }
        s *= 10;
    }
    float top = hi / s; int n = (int)top; if ((float)n < top) n++;
    tr[t].scale = s; tr[t].refpos = NDIV; tr[t].ref = n * s;
}
static void keypad_enter(float fmult, uint64_t hmult)
{
    uint64_t hz; float v;
    uint64_t s = sweep_cfg.start, e = sweep_cfg.stop, c = (s + e) / 2, sp = e - s;
    int ok = 1;
    switch (kp_target) {
    case A_START: if ((ok = parse_hz(kb, hmult, &hz))) set_sweep_ui(hz, e < hz ? hz : e, sweep_cfg.points); break;
    case A_STOP:  if ((ok = parse_hz(kb, hmult, &hz))) set_sweep_ui(s > hz ? hz : s, hz, sweep_cfg.points); break;
    case A_CENTER: if ((ok = parse_hz(kb, hmult, &hz))) set_sweep_ui(hz > sp / 2 ? hz - sp / 2 : 0, hz + sp / 2, sweep_cfg.points); break;
    case A_SPAN:  if ((ok = parse_hz(kb, hmult, &hz))) set_sweep_ui(c > hz / 2 ? c - hz / 2 : 0, c + hz / 2, sweep_cfg.points); break;
    case A_CW:    if ((ok = parse_hz(kb, hmult, &hz))) set_sweep_ui(hz, hz, sweep_cfg.points); break;
    case A_JOGSTEP: if ((ok = parse_hz(kb, hmult, &hz))) jog_step = hz > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)hz; break;
    case A_POINTS: if ((ok = parse_hz(kb, 1, &hz) && hz >= 1 && hz <= POINTS_MAX)) set_sweep_ui(s, e, (int)hz); break;
    case A_SCALEDIV: if ((ok = parse_num(kb, fmult, &v) && v > 0)) tr[tr_act].scale = v; break;
    case A_REFVAL: if ((ok = parse_num(kb, fmult, &v))) tr[tr_act].ref = v; break;
    case A_REFPOS: if ((ok = parse_num(kb, 1, &v) && v >= 0 && v <= NDIV)) tr[tr_act].refpos = (uint8_t)(v + 0.5f); break;
    case A_EDELAY: if ((ok = parse_num(kb, fmult, &v))) edelay = v; break;
    case A_S21OFF: if ((ok = parse_num(kb, fmult, &v))) s21_offset = v; break;
    case A_PORTZ: if ((ok = parse_num(kb, fmult, &v) && v > 0)) port_z = v; break;
    case A_MSRL: if ((ok = parse_num(kb, fmult, &v) && v > 0)) meas_rl = v; break;
    case A_MSVF: if ((ok = parse_num(kb, fmult, &v) && v >= 0.1f && v <= 1)) meas_vf = v; break;
    }
    if (!ok) say("not a valid value", 3000);
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
    if (l[0] >= '0' && l[0] <= '9') { if (n < 15) { kb[n] = l[0]; kb[n + 1] = 0; } }
    else if (l[0] == '.') { if (n < 15) { kb[n] = '.'; kb[n + 1] = 0; } }
    else if (l[0] == '-') { if (n == 0) { kb[0] = '-'; kb[1] = 0; } }
    else if (l[0] == 'B') { if (n) kb[n - 1] = 0; }
    else if (l[0] == 'C') { grid_leave(); return; }
    else if (l[0] == 'G') { keypad_enter(1e9f, 1000000000ull); return; }
    else if (l[0] == 'M') { keypad_enter(1e6f, 1000000ull); return; }
    else if (l[0] == 'k') { keypad_enter(1e3f, 1000ull); return; }
    else if (l[0] == MU[0]) { keypad_enter(1e-6f, 1); return; }
    else if (l[0] == 'n') { keypad_enter(1e-9f, 1); return; }
    else if (l[0] == 'p') { keypad_enter(1e-12f, 1); return; }
    else if (l[0] == 'E') { keypad_enter(1, 1); return; }
    dirty |= D_MENU;
}
static float search_val(int i) { return fkind(tr_act) == K_RECT ? trace_val(tr_act, i) : c_abs(point_g(i, tr[tr_act].ch)); }
static int marker_search(int want_min)               /* global max or min of the active trace */
{
    int best = 0; float bv = 0;
    for (int i = 0; i < sweep_cfg.points; i++) {
        float v = search_val(i);
        if (i == 0 || (want_min ? v < bv : v > bv)) { bv = v; best = i; }
    }
    return best;
}
static int marker_search_dir(int from, int dir, int want_min)   /* next peak (or valley) left or right */
{
    int n = sweep_cfg.points;
    for (int i = from + dir; i > 0 && i < n - 1; i += dir) {
        float a = search_val(i - 1), v = search_val(i), b = search_val(i + 1);
        if (want_min ? (v <= a && v <= b && (v < a || v < b)) : (v >= a && v >= b && (v > a || v > b))) return i;
    }
    return from;
}
static void touch_cal_start(void);
static void render(void);
static void draw_status(void);
static void info_show(void);
static void ttest_show(void);
static void cal_step(int k)
{
    static const char *const nm[] = { "OPEN", "SHORT", "LOAD", "THRU", "ISOLN" };
    char b[48]; char *p = put_s(b, "measuring "); put_s(p, nm[k]); say(b, 60000);
    draw_status();
    uint8_t was = sweep_running; sweep_running = 0;
    int r = cal_measure(k);
    sweep_running = was;
    p = put_s(b, nm[k]); put_s(p, r ? " failed" : " done");
    say(b, 4000);
}
static void reset_display(void)
{
    static const trace_t d[4] = { { 1, 0, F_LOGMAG, 7, 3, 10, 0 }, { 1, 1, F_LOGMAG, 7, 1, 10, 0 }, { 1, 0, F_SMITH, 0, 3, 1, 0 }, { 1, 1, F_PHASE, 4, 1, 90, 0 } };
    for (int t = 0; t < 4; t++) tr[t] = d[t];
    for (int m = 0; m < NMK; m++) { mk[m].on = m == 0; mk[m].idx = 0; }
    tr_act = 0; mk_act = 0; mk_delta = mk_tracking = search_min = 0; jog_step = 0;
    edelay = 0; s21_offset = 0; port_z = 50; grid_values = 1; dot_grid = 0;
}
static void set_format(int f, int ch)
{
    trace_t *t = &tr[tr_act];
    if (t->fmt != f || t->ch != ch) { t->scale = fmts[f].scale; t->ref = fmts[f].ref; t->refpos = fmts[f].refpos; }
    t->fmt = f; t->ch = ch; t->on = 1;
    dirty |= D_FULL;
}
static void mem_store(int k, int t)
{
    mem_t *m = &mem[k];
    m->fmt = tr[t].fmt; m->ch = tr[t].ch; m->src = (uint8_t)t; m->scale = tr[t].scale; m->ref = tr[t].ref; m->refpos = tr[t].refpos;
    m->n = sweep_cfg.points;
    for (int i = 0; i < m->n; i++) { m->v[i] = trace_val(t, i); if ((i & 31) == 0) service(); }
    m->on = 1;
}
static void act(int a)
{
    if (a >= A_SUB && a < A_SUB_END && a != SUB(MN_SDLOAD)) { menu_show(a - A_SUB); return; }
    if (a >= A_TRACE0 && a <= A_TRACE3) {
        int t = a - A_TRACE0;
        if (t == tr_act) tr[t].on = !tr[t].on; else { tr[t].on = 1; tr_act = t; }
        dirty |= D_MENU | D_LEGEND | D_GRAPH; return;
    }
    if (a >= A_STORE0 && a <= A_STORE3) {
        int k = a - A_STORE0;
        if (mem[k].on) { mem[k].on = 0; dirty |= D_MENU | D_GRAPH; return; }
        if (!tr[tr_act].on || fkind(tr_act) != K_RECT || !have_data) { say("store: the active trace must be shown in a rectangular format", 3000); return; }
        mem_store(k, tr_act); dirty |= D_MENU | D_GRAPH; return;
    }
    if (a >= A_F0 && a < A_FEND) { set_format(a - A_F0, menu_id == MN_F21 || menu_id == MN_F21B); dirty |= D_MENU; return; }
    if (a >= A_SM0 && a < A_SMEND) {
        set_format(F_SMITH, menu_id == MN_S21SM); tr[tr_act].smode = a - A_SM0; dirty |= D_MENU; return;
    }
    if (a >= A_AVG0 && a < A_AVGEND) { sweep_cfg.avg = avg_vals[a - A_AVG0]; dirty |= D_MENU; return; }
    if (a == SUB(MN_SDLOAD)) {
        say("reading the card...", 10000); draw_status();
        sd_nfiles = sd_list("CAL", sd_files, 8);
        if (sd_nfiles < 0) { sd_nfiles = 0; say("no card, or not FAT32", 3000); return; }
        for (int k = 0; k < MROWS; k++) menus[MN_SDLOAD][k] = k < sd_nfiles ? A_FILE0 + k : k == sd_nfiles ? A_BACK : 0;
        say(sd_nfiles ? "choose a calibration" : "no .CAL files on the card", 3000);
        menu_show(MN_SDLOAD); return;
    }
    if (a >= A_FILE0 && a < A_FILEEND) {
        int k = a - A_FILE0; if (k >= sd_nfiles) return;
        int r = sd_load_cal(sd_files[k]);
        say(r ? "could not load that file" : "calibration and its sweep loaded", 3000);
        if (!r) menu_close();
        return;
    }
    if (a == A_SDSCREEN) {                           /* the screen without the menu over it */
        char name[13], b[48];
        menu_open = 0; dirty = D_FULL; render();
        say("writing the screen to the card...", 20000); draw_status();
        msg[0] = 0; draw_status();                   /* not in the picture */
        int r = sd_save_screen(sd_autoname, name);
        if (r) say(r == -5 ? "no room on the card" : "could not write to the card", 3000);
        else { char *q = put_s(b, "saved "); put_s(q, name); say(b, 4000); }
        dirty |= D_FULL; return;
    }
    if (a == A_SDS1P || a == A_SDS2P || a == A_SDSCAL) {
        char name[13], b[48]; say("writing to the card...", 10000); draw_status();
        int r = a == A_SDSCAL ? sd_save_cal(sd_autoname, name) : sd_save_touchstone(a == A_SDS2P, sd_autoname, name);
        if (r == -3) say("no calibration to save", 3000);
        else if (r) say("could not write to the card", 3000);
        else { char *q = put_s(b, "saved "); put_s(q, name); say(b, 4000); }
        return;
    }
    if (a == A_SDAUTO) { sd_autoname ^= 1; dirty |= D_MENU; return; }
    if (a >= A_MS0 && a < A_MSEND) { meas_mode = (uint8_t)(a - A_MS0); dirty |= D_MENU | D_GRAPH; return; }
    if (a == A_MSRL || a == A_MSVF) { grid_show(G_KEYPAD, a); return; }
    if (a >= A_SAVE0 && a < A_SAVEEND) {
        int n = a - A_SAVE0; say("saving...", 10000); draw_status();
        say(store_save(n) ? "error: flash write failed" : "saved", 3000); dirty |= D_MENU; return;
    }
    if (a >= A_RCL0 && a < A_RCLEND) {
        int n = a - A_RCL0;
        if (store_recall(n)) { say("that slot is empty", 3000); return; }
        say("recalled", 3000); menu_close(); return;
    }
    if (a >= A_MK0 && a < A_MKEND) {
        int m = a - A_MK0;
        if (m == mk_act) mk[m].on = !mk[m].on; else { mk[m].on = 1; mk_act = m; }
        dirty |= D_MENU | D_GRAPH | D_LEGEND | D_READOUT; return;
    }
    uint64_t f = point_freq(mk[mk_act].idx), s = sweep_cfg.start, e = sweep_cfg.stop;
    switch (a) {
    case A_BACK: if (menu_id == MN_HOME) menu_close(); else menu_show(menu_parent[menu_id]); return;
    case A_PAUSE: sweep_running = !sweep_running; if (!sweep_running) synth_all_off(); dirty |= D_MENU | D_STATUS; return;
    case A_TRANSFORM: case A_CSTD: case A_BRIGHT: case A_TXPOWER:
        say("not built yet: see the plan in NOTES.md (stage 12)", 3000); return;
    case A_CHANNEL: tr[tr_act].ch ^= 1; if (tr[tr_act].fmt >= F_S21SR && !tr[tr_act].ch) set_format(F_LOGMAG, 0); dirty |= D_FULL; return;
    case A_SCALEDIV: case A_REFVAL: case A_REFPOS: case A_EDELAY: case A_S21OFF: case A_PORTZ:
    case A_START: case A_STOP: case A_CENTER: case A_SPAN: case A_CW: case A_JOGSTEP: case A_POINTS:
        grid_show(G_KEYPAD, a); return;
    case A_GRIDVAL: grid_values ^= 1; dirty |= D_MENU | D_GRAPH; return;
    case A_DOTGRID: dot_grid ^= 1; dirty |= D_MENU | D_GRAPH; return;
    case A_AUTO: autoscale(tr_act); dirty |= D_LEGEND | D_GRAPH; return;
    case A_MKALLOFF: for (int m = 0; m < NMK; m++) mk[m].on = 0; dirty |= D_MENU | D_GRAPH | D_LEGEND | D_READOUT; return;
    case A_DELTA: mk_delta ^= 1; dirty |= D_MENU | D_LEGEND | D_READOUT; return;
    case A_SEARCH:
        if (mk[mk_act].on) search_min ^= 1;
        mk[mk_act].idx = marker_search(search_min); mk[mk_act].on = 1;
        dirty |= D_MENU | D_GRAPH | D_LEGEND | D_READOUT; return;
    case A_SLEFT: case A_SRIGHT:
        mk[mk_act].idx = marker_search_dir(mk[mk_act].idx, a == A_SLEFT ? -1 : 1, search_min); mk[mk_act].on = 1;
        dirty |= D_GRAPH | D_LEGEND | D_READOUT; return;
    case A_TRACKING: mk_tracking ^= 1; dirty |= D_MENU; return;
    case A_OSTART: set_sweep_ui(f, e > f ? e : f, sweep_cfg.points); return;
    case A_OSTOP: set_sweep_ui(s < f ? s : f, f, sweep_cfg.points); return;
    case A_OCENTER: { uint64_t sp = e - s; if (set_sweep_ui(f > sp / 2 ? f - sp / 2 : 0, f + sp / 2, sweep_cfg.points)) mk[mk_act].idx = sweep_cfg.points / 2; return; }
    case A_OSPAN: {
        if (mk_act == 0 || !mk[0].on) { say(">SPAN: needs marker 1 and another active marker", 3000); return; }
        uint64_t g = point_freq(mk[0].idx);
        set_sweep_ui(g < f ? g : f, g < f ? f : g, sweep_cfg.points); return; }
    case A_OEDELAY: edelay += group_delay(mk[mk_act].idx, tr[tr_act].ch); dirty |= D_FULL; return;
    case A_BANDS: grid_show(G_BANDS, 0); return;
    case A_COPEN: cal_step(CAL_OPEN); dirty |= D_MENU; return;
    case A_CSHORT: cal_step(CAL_SHORT); dirty |= D_MENU; return;
    case A_CLOAD: cal_step(CAL_LOAD); dirty |= D_MENU; return;
    case A_CTHRU: cal_step(CAL_THRU); dirty |= D_MENU; return;
    case A_CISOLN: cal_step(CAL_ISOLN); dirty |= D_MENU; return;
    case A_CDONE: case A_CDONERAM: {
        int r = cal_solve();
        say(r == -1 ? "need open, short, load and thru" : r ? "singular standards" : a == A_CDONE ? "calibration on: choose a slot to save it" : "calibration on", 4000);
        if (!r) menu_show(a == A_CDONE ? MN_SAVE : MN_CALMENU);
        dirty |= D_MENU | D_GRAPH | D_LEGEND | D_READOUT; return; }
    case A_CRANGE: {
        uint64_t cs, ce; int cn;
        if (cal_range(&cs, &ce, &cn) < 0) { say("no calibration", 3000); return; }
        set_sweep_ui(cs, ce, cn); return; }
    case A_SAVECFG: say(config_save() ? "error: flash write failed" : "config saved", 3000); return;
    case A_CRESET: case A_CRESETALL: case A_BOOTLOAD: case A_CLEARCFG:
        if (!confirming(a)) { confirm_a = a; confirm_t = ms_ticks; dirty |= D_MENU; return; }
        confirm_a = 0;
        if (a == A_CLEARCFG) { say(store_erase_all() ? "error: erase failed" : "saved settings and calibrations erased", 4000); dirty |= D_MENU; return; }
        if (a == A_BOOTLOAD) { boot_pending = ms_ticks ? ms_ticks : 1; say("release the jog: entering the bootloader", 10000); return; }
        cal_reset();
        if (a == A_CRESETALL) reset_display();
        say(a == A_CRESET ? "calibration cleared" : "calibration and display reset", 3000);
        dirty |= D_FULL; return;
    case A_CAPPLY: if (cal_valid()) cal_on = !cal_on; else say("no calibration for this sweep", 3000); dirty |= D_MENU | D_GRAPH | D_LEGEND | D_READOUT; return;
    case A_CENH: cal_enhanced ^= 1; dirty |= D_MENU | D_GRAPH | D_LEGEND | D_READOUT; return;
    case A_TOUCHCAL: menu_open = 0; touch_cal_start(); return;
    case A_TOUCHTEST: menu_open = 0; ttest_show(); return;
    case A_VERSION: menu_open = 0; info_show(); return;
    case A_FLIP: flipped ^= 1; madctl ^= 0xC0; lcd_cmd(0x36, &madctl, 1); dirty |= D_FULL; return;
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
 * (centre (1, 1/x), radius 1/|x|, inside the unit circle). Polar grid:
 * |G| = 0.2 .. 1 circles and radials every 30 degrees. Each circle is drawn
 * by solving for its crossings with every column and row of the strip, so
 * it has no gaps whatever its slope. */
typedef struct { float cx, cy, r; uint8_t clip; } circ_t;
static circ_t sg[16], pg[5]; static int sg_n;
static void grids_init(void)
{
    static const float v[] = { 0.2f, 0.5f, 1, 2, 5 };
    sg_n = 0;
    sg[sg_n++] = (circ_t){ 0, 0, SR, 0 };
    for (int k = 0; k < 5; k++) sg[sg_n++] = (circ_t){ SR * v[k] / (1 + v[k]), 0, SR / (1 + v[k]), 1 };
    for (int k = 0; k < 5; k++) {
        sg[sg_n++] = (circ_t){ SR, -SR / v[k], SR / v[k], 1 };
        sg[sg_n++] = (circ_t){ SR, SR / v[k], SR / v[k], 1 };
    }
    for (int k = 0; k < 5; k++) pg[k] = (circ_t){ 0, 0, SR * (k + 1) / 5.0f, 0 };
}
static inline void plot(int x, int y, int x0, int w, uint16_t c)   /* screen coordinates, clipped to the strip */
{
    if (x < x0 || x >= x0 + w || y < GY0 || y >= GY1) return;
    BUF[(y - GY0) * w + (x - x0)] = c;
}
static void circles(const circ_t *cs, int n, int x0, int w)
{
    float lim = (float)SR * SR + SR;
    for (int k = 0; k < n; k++) {
        const circ_t *c = &cs[k];
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
}
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
static void smith_grid(int x0, int w)
{
    circles(sg, sg_n, x0, w);
    for (int x = SCX - SR; x <= SCX + SR; x++) plot(x, SCY, x0, w, C_SMITH);   /* real axis */
}
static void polar_grid(int x0, int w)
{
    static const float cs[6][2] = { { 1, 0 }, { 0.8660254f, 0.5f }, { 0.5f, 0.8660254f }, { 0, 1 }, { -0.5f, 0.8660254f }, { -0.8660254f, 0.5f } };
    circles(pg, 5, x0, w);
    for (int k = 0; k < 6; k++) {
        int dx = (int)(cs[k][0] * SR), dy = (int)(cs[k][1] * SR);
        seg(SCX - dx, SCY + dy, SCX + dx, SCY - dy, x0, w, C_SMITH);
    }
}
static void chart_xy(cpx g, int *x, int *y)
{
    float m = c_abs2(g);
    if (m > 1.21f) { float s = 1.1f / fsqrt(m); g.re *= s; g.im *= s; }
    float fx = SCX + g.re * SR, fy = SCY - g.im * SR;
    *x = (int)(fx + 0.5f); *y = (int)(fy + 0.5f);
}
static void chart_trace(int t, int x0, int w)
{
    int px = 0, py = 0;
    for (int i = 0; i < sweep_cfg.points; i++) {
        int x, y; chart_xy(point_g(i, tr[t].ch), &x, &y);
        if (i) seg(px, py, x, y, x0, w, tcolor[t]);
        px = x; py = y;
    }
    for (int m = 0; m < NMK; m++) if (mk[m].on) {
        int x, y; chart_xy(point_g(mk[m].idx, tr[t].ch), &x, &y);
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
    int f = tr[t].fmt;
    if (f == F_PHASE || f == F_ZPHASE) return trace_val(t, fr < 0.5f ? i : i + 1);   /* no interpolation across a wrap */
    return trace_val(t, i) * (1 - fr) + trace_val(t, i + 1) * fr;
}
static void draw_graph(void)
{
    int hh = GY1 - GY0, xend = menu_open ? MX : GX1;
    int prev[4] = { -1, -1, -1, -1 }, prevm[4] = { -1, -1, -1, -1 };
    int rect = any_kind(K_RECT), smith = any_kind(K_SMITH), polar = any_kind(K_POLAR);
    int mcol[NMK]; for (int m = 0; m < NMK; m++) mcol[m] = mk[m].on ? col_of_point(mk[m].idx) : -100;
    for (int xs = 0; GX0 + xs < xend; xs += 32) {
        int x0 = GX0 + xs, w = xend - x0 < 32 ? xend - x0 : 32;
        for (int r = 0; r < hh; r++) {
            int line = rect && (r % DIV_PX == 0 || r == hh - 1);
            for (int k = 0; k < w; k++) BUF[r * w + k] = line && (!dot_grid || ((xs + k) & 3) == 0) ? C_GRID : C_BG;
        }
        if (rect) for (int k = 0; k < w; k++) {
            int col = xs + k;
            if (col % XDIV_PX == 0 || col == GW - 1) for (int r = 0; r < hh; r++) if (!dot_grid || (r & 3) == 0) BUF[r * w + k] = C_GRID;
        }
        if (smith) smith_grid(x0, w);
        if (polar) polar_grid(x0, w);
        if (have_data) {
            for (int t = 3; t >= 0; t--) if (tr[t].on && fkind(t) != K_RECT) chart_trace(t, x0, w);
            for (int k = 0; k < 4; k++) {                 /* stored traces, under the live ones */
                mem_t *m = &mem[k]; if (!m->on || m->n < 2) continue;
                const trace_t *src = &tr[m->src];
                int same = src->fmt == m->fmt;
                float sc = same ? src->scale : m->scale, rf = same ? src->ref : m->ref; int rp = same ? src->refpos : m->refpos;
                uint16_t c = (uint16_t)((tcolor[m->src] >> 1) & 0x7BEF);
                for (int kk = 0; kk < w; kk++) {
                    int col = xs + kk;
                    float pos = (float)col * (m->n - 1) / (GW - 1); int i = (int)pos; if (i >= m->n - 1) i = m->n - 2;
                    float fr = pos - i, v = m->v[i] * (1 - fr) + m->v[i + 1] * fr;
                    float yf = (NDIV - rp) * DIV_PX - (v - rf) / sc * DIV_PX;
                    int y = yf < 0 ? 0 : yf > hh - 1 ? hh - 1 : (int)(yf + 0.5f);
                    int a2 = prevm[k] < 0 ? y : prevm[k];
                    if ((m->fmt == F_PHASE || m->fmt == F_ZPHASE) && iabs(a2 - y) > hh / 2) a2 = y;
                    int lo = a2 < y ? a2 : y, hi = a2 < y ? y : a2;
                    for (int r = lo; r <= hi; r++) BUF[r * w + kk] = c;
                    prevm[k] = y;
                }
            }
            for (int t = 3; t >= 0; t--) {
                if (!tr[t].on || fkind(t) != K_RECT) continue;
                int wraps = tr[t].fmt == F_PHASE || tr[t].fmt == F_ZPHASE;
                for (int k = 0; k < w; k++) {
                    int col = xs + k;
                    float yf = y_of(t, col_val(t, col));
                    int y = yf < 0 ? 0 : yf > hh - 1 ? hh - 1 : (int)(yf + 0.5f);
                    int a = prev[t] < 0 ? y : prev[t];
                    if (wraps && iabs(a - y) > hh / 2) a = y;
                    int lo = a < y ? a : y, hi = a < y ? y : a;
                    for (int r = lo; r <= hi; r++) BUF[r * w + k] = tcolor[t];
                    prev[t] = y;
                    for (int m = 0; m < NMK; m++) {           /* marker: a small triangle above the point */
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
    if (tr[tr_act].on && fkind(tr_act) == K_RECT) t = tr_act;
    else for (int k = 0; k < 4; k++) if (tr[k].on && fkind(k) == K_RECT) { t = k; break; }
    lcd_fill(0, GY0, GX0 - 1, GY1 - GY0, C_BG);
    if (t < 0 || !grid_values) return;
    int eng = fmts[tr[t].fmt].unit >= U_S;
    for (int d = 0; d <= NDIV; d++) {
        float v = tr[t].ref + (NDIV - d - tr[t].refpos) * tr[t].scale;
        char b[16]; float a = v < 0 ? -v : v;
        if (a < tr[t].scale * 1e-3f) v = a = 0;
        if (eng) put_short_eng(b, v);
        else put_fx(b, v, a >= 100 || tr[t].scale >= 5 ? 0 : a >= 10 || tr[t].scale >= 0.5f ? 1 : 2);
        int y = GY0 + d * DIV_PX - font_s.h / 2;
        if (y < GY0) y = GY0;
        if (y > GY1 - font_s.h) y = GY1 - font_s.h;
        ftext(GX0 - 3 - ftext_w(b, &font_s), y, b, &font_s, tcolor[t], C_BG, GX0 - 1);
    }
}

/* ---------- legend, readout, status ---------- */
static int active_idx(void) { return mk[mk_act].on ? mk[mk_act].idx : -1; }
static void draw_legend(void)
{
    int idx = active_idx(), dref = mk_delta && mk_act != 0 && mk[0].on ? mk[0].idx : -1;
    for (int t = 0; t < 4; t++) {
        char b[80], *p = b;
        *p++ = t == tr_act ? '>' : ' '; p = put_u(p, t + 1, 1); *p++ = ' ';
        if (!tr[t].on) put_s(p, "hidden");
        else {
            p = put_s(p, tr[t].ch ? "S21 " : "S11 ");
            if (fkind(t) == K_SMITH) { p = put_s(p, "SMITH "); p = put_s(p, (tr[t].ch ? smode_s21 : smode_s11)[tr[t].smode]); *p++ = ' '; }
            else { p = put_s(p, fmts[tr[t].fmt].name); *p++ = ' '; if (fkind(t) == K_RECT) { p = put_scale(p, tr[t].fmt, tr[t].scale); p = put_s(p, "/ "); } }
            if (idx >= 0 && have_data) {
                if (fkind(t) == K_SMITH) put_smith(p, t, idx);
                else {
                    float v = trace_val(t, idx);
                    if (dref >= 0) { p = put_s(p, DELTA); v -= trace_val(t, dref); }
                    put_val(p, tr[t].fmt, v);
                }
            }
        }
        int cw = (menu_open ? MX : W) / 2;
        line_text((t & 1) * cw + 2, 1 + (t >> 1) * 12, b, tr[t].on ? tcolor[t] : C_DIM, (t & 1) * cw + cw - 2);
    }
}
static void draw_readout(void)
{
    int xm = menu_open ? MX : W, idx = active_idx();
    char b[160], *p = b;
    if (idx < 0 || !have_data) { line_text(2, RY0, "no active marker", C_DIM, xm); line_text(2, RY0 + RDY, "", C_DIM, xm); return; }
    uint64_t f = point_freq(idx);
    cpx s11, s21; point_vals(idx, &s11, &s21);
    cpx z = g_to_z(s11);
    float gm = c_abs(s11), swr = gm < 0.999f ? (1 + gm) / (1 - gm) : 999, w = 2 * PI_F * (float)f;
    p = put_s(p, "M"); p = put_u(p, mk_act + 1, 1); *p++ = ' '; p = put_freq(p, f);
    if (mk_delta && mk_act != 0 && mk[0].on) {
        uint64_t g = point_freq(mk[0].idx); p = put_s(p, " (" DELTA " ");
        if (f >= g) p = put_freq(p, f - g); else { *p++ = '-'; p = put_freq(p, g - f); }
        *p++ = ')';
    }
    p = put_s(p, "  Z "); p = put_rx(p, z, 2); p = put_s(p, " " OHM "  SWR "); p = put_fx(p, swr, 3); p = put_s(p, "  ");
    p = put_lc(p, z.im, w);
    uint8_t fl = pts[idx].flags;
    put_s(p, fl & PF_RANGE ? "  range" : fl & PF_CLIP ? "  CLIP" : fl & PF_LOWREF ? "  low ref" : "");
    line_text(2, RY0, b, C_TEXT, xm);
    p = put_s(b, "S11 "); p = put_sfx(p, c_db(s11), 2); p = put_s(p, " dB "); p = put_sfx(p, c_deg(s11), 1);
    p = put_s(p, DEG "   S21 "); p = put_sfx(p, c_db(s21), 3); p = put_s(p, " dB "); p = put_sfx(p, c_deg(s21), 2); p = put_s(p, DEG);
    for (int m = 0; m < NMK; m++) if (mk[m].on && m != mk_act && p - b < 120) { p = put_s(p, "  M"); p = put_u(p, m + 1, 1); *p++ = ' '; p = put_freq(p, point_freq(mk[m].idx)); }
    line_text(2, RY0 + RDY, b, C_DIM, xm);
}

/* ---------- MEASURE modes ----------
 * Our own derivations from circuit models, on the calibrated data. Rl is
 * the termination the S21 modes assume (both ports); port Z for L/C match.
 *   S21 of a series element Z between Rl ports:  S21 = 2Rl / (2Rl + Z)
 *   S21 of a shunt element Z across the line:   S21 = 2Z / (2Z + Rl) */
#define C_LIGHT 299792458.0f
static cpx z_series21(cpx t) { if (c_abs2(t) < 1e-12f) return c_mk(1e9f, 0); cpx z = c_div(c_sub(c_mk(1, 0), t), t); return c_mk(2 * meas_rl * z.re, 2 * meas_rl * z.im); }
static cpx z_shunt21(cpx t) { cpx d = c_sub(c_mk(1, 0), t); if (c_abs2(d) < 1e-12f) return c_mk(1e9f, 0); cpx z = c_div(t, d); return c_mk(meas_rl / 2 * z.re, meas_rl / 2 * z.im); }
static float w_of(int i) { return 2 * PI_F * (float)point_freq(i); }
static char *put_x_as_lc(char *p, const char *what, float x, float w)   /* "series L 1.23 uH" */
{
    p = put_s(p, what); p = put_s(p, x >= 0 ? " L " : " C ");
    return x >= 0 ? put_eng(p, x / w, "H") : put_eng(p, -1 / (w * x), "F");
}
static char *put_b_as_lc(char *p, float b, float w)                    /* shunt susceptance */
{
    p = put_s(p, b >= 0 ? "shunt C " : "shunt L ");
    return b >= 0 ? put_eng(p, b / w, "F") : put_eng(p, -1 / (w * b), "H");
}
/* parabola through three samples: the offset (-1..1) of the extremum from the middle one */
static float vertex(float a, float b, float c) { float d = a - 2 * b + c; return d != 0 ? 0.5f * (a - c) / d : 0; }
static float freq_at(int i, float off)
{
    int n = sweep_cfg.points; if (n < 2) return (float)point_freq(i);
    float step = (float)(sweep_cfg.stop - sweep_cfg.start) / (n - 1);
    return (float)point_freq(i) + off * step;
}
static int meas_lines(char ln[][96])
{
    int n = sweep_cfg.points, k = 0, idx = active_idx() < 0 ? 0 : active_idx();
    char *p;
    if (!have_data || n < 3) { put_s(ln[0], "MEASURE: no sweep yet"); return 1; }
    switch (meas_mode) {
    case MS_LC: {
        cpx z = g_to_z(point_g(idx, 0)); float R = z.re, X = z.im, Z0 = port_z, w = w_of(idx);
        p = put_s(ln[k], "L/C MATCH at "); p = put_freq(p, point_freq(idx)); p = put_s(p, ": load "); p = put_rx(p, z, 1);
        p = put_s(p, " " OHM " to "); p = put_fx(p, Z0, 0); put_s(p, " " OHM); k++;
        if (R <= 0) { put_s(ln[k++], "no match: the load's resistance is not positive"); return k; }
        float m2 = R * R + X * X, G = R / m2, BL = -X / m2;
        if (G * Z0 < 1) for (int sg = -1; sg <= 1; sg += 2) {               /* shunt at the load, series to the source */
            float bb = sg * fsqrt(G / Z0 - G * G), B = bb - BL, Xs = bb * Z0 / G;
            p = put_s(ln[k], "at load "); p = put_b_as_lc(p, B, w); p = put_s(p, ", then "); put_x_as_lc(p, "series", Xs, w); k++;
        }
        if (R < Z0) for (int sg = -1; sg <= 1; sg += 2) {                   /* series at the load, shunt to the source */
            float xx = sg * fsqrt(R * (Z0 - R)), Xs = xx - X, B = xx / (R * Z0);
            p = put_x_as_lc(ln[k], "at load series", Xs, w); p = put_s(p, ", then "); put_b_as_lc(p, B, w); k++;
        }
        if (k == 1) put_s(ln[k++], "already matched");
        return k;
    }
    case MS_CABLE: {
        /* open (or shorted) at the far end: one-way delay from the unwrapped S11 phase slope;
         * lambda/4 where the unwrapped phase reaches -180 (open) or 0 (short, starting at +180) */
        float first = c_deg(point_g(0, 0)), prev = first;
        int open = first > -90 && first < 90;
        if (!open && first < 0) first += 360;
        float target = open ? -180 : 0, unwrap = 0, fq = 0;
        for (int i = 1; i < n; i++) {
            float ph = c_deg(point_g(i, 0)), d = ph - prev;
            if (d > 180) d -= 360; else if (d < -180) d += 360;
            float before = first + unwrap, after = before + d;
            if (!fq && before > target && after <= target)
                fq = (float)point_freq(i - 1) + (float)(point_freq(i) - point_freq(i - 1)) * (before - target) / (before - after);
            unwrap += d; prev = ph;
            if ((i & 31) == 0) service();
        }
        float df = (float)(point_freq(n - 1) - point_freq(0));
        float tau = fq > 0 ? 1 / (4 * fq)             /* exact for any Z0; the phase slope is biased when Z0 is not the port's */
                  : df > 0 ? -unwrap / 360.0f / df / 2 : 0;
        p = put_s(ln[k], open ? "CABLE (open at the far end): " : "CABLE (shorted at the far end): ");
        if (fq > 0) { p = put_s(p, "lambda/4 at "); put_freq(p, (uint64_t)(fq + 0.5f)); } else put_s(p, "lambda/4 above the sweep (delay from the phase slope)");
        k++;
        p = put_s(ln[k], "delay "); p = put_eng(p, tau, "s"); p = put_s(p, " one way, length "); p = put_fx(p, C_LIGHT * meas_vf * tau, 3);
        p = put_s(p, " m at VF "); put_fx(p, meas_vf, 2); k++;
        float fq_use = fq > 0 ? fq : (tau > 0 ? 1 / (4 * tau) : 0);
        if (fq_use > 0) {                            /* Z0 from the stub's reactance near lambda/8 */
            int best = 0; float bd = 1e30f;
            for (int i = 0; i < n; i++) { float d = (float)point_freq(i) - fq_use / 2; if (d < 0) d = -d; if (d < bd) { bd = d; best = i; } }
            float s_, c_; sincos_f(PI_F / 2 * (float)point_freq(best) / fq_use, &s_, &c_);
            cpx zz = g_to_z(point_g(best, 0));
            float z0 = open ? (c_ != 0 ? -zz.im * s_ / c_ : 0) : (s_ != 0 ? zz.im * c_ / s_ : 0);
            p = put_s(ln[k], "Z0 "); p = put_fx(p, z0, 1); p = put_s(p, " " OHM " (at "); p = put_freq(p, point_freq(best)); put_s(p, ")"); k++;
        }
        if (fq > 0) {
            int best = 0; float bd = 1e30f;
            for (int i = 0; i < n; i++) { float d = (float)point_freq(i) - fq; if (d < 0) d = -d; if (d < bd) { bd = d; best = i; } }
            p = put_s(ln[k], "loss "); p = put_fx(p, -c_db(point_g(best, 0)) / 2, 3); put_s(p, " dB one way at lambda/4"); k++;
        }
        return k;
    }
    case MS_SHUNTLC: {
        int i0 = 0; float m0 = 1e30f;
        for (int i = 0; i < n; i++) { float m = c_abs2(point_g(i, 1)); if (m < m0) { m0 = m; i0 = i; } }
        if (i0 == 0 || i0 == n - 1) { put_s(ln[k++], "SHUNT LC: the notch is at the edge of the sweep"); return k; }
        float off = vertex(c_abs(point_g(i0 - 1, 1)), c_abs(point_g(i0, 1)), c_abs(point_g(i0 + 1, 1)));
        int d = n / 20 > 1 ? n / 20 : 1, a = i0 - d < 0 ? 0 : i0 - d, b = i0 + d > n - 1 ? n - 1 : i0 + d;
        float w1 = w_of(a), w2 = w_of(b), x1 = z_shunt21(point_g(a, 1)).im, x2 = z_shunt21(point_g(b, 1)).im;
        /* X = wL - 1/(wC) at two frequencies */
        float det = -w1 / w2 + w2 / w1, L = (-x1 / w2 + x2 / w1) / det, iC = (w1 * x2 - w2 * x1) / det;
        float R = z_shunt21(point_g(i0, 1)).re, f0 = L > 0 && iC > 0 ? fsqrt(iC / L) / (2 * PI_F) : freq_at(i0, off), w0 = 2 * PI_F * f0;
        p = put_s(ln[k], "SHUNT LC: f0 "); p = put_freq(p, (uint64_t)(f0 + 0.5f)); p = put_s(p, ", R "); p = put_fx(p, R, 3); put_s(p, " " OHM); k++;
        p = put_s(ln[k], "L "); p = put_eng(p, L, "H"); p = put_s(p, ", C "); p = put_eng(p, iC != 0 ? 1 / iC : 0, "F");
        p = put_s(p, ", Q "); put_fx(p, R > 0 ? w0 * L / R : 0, 1); k++;
        return k;
    }
    case MS_SERIESL: {
        cpx z = z_series21(point_g(idx, 1)); float w = w_of(idx);
        p = put_s(ln[k], "SERIES L at "); p = put_freq(p, point_freq(idx)); p = put_s(p, ": L "); p = put_eng(p, z.im / w, "H");
        p = put_s(p, ", R "); p = put_fx(p, z.re, 3); p = put_s(p, " " OHM ", Q "); put_fx(p, z.re > 0 ? z.im / z.re : 0, 1); k++;
        for (int i = 1; i < n; i++) {
            float xa = z_series21(point_g(i - 1, 1)).im, xb = z_series21(point_g(i, 1)).im;
            if (xa > 0 && xb <= 0) {
                float f = (float)point_freq(i - 1) + (float)(point_freq(i) - point_freq(i - 1)) * xa / (xa - xb);
                p = put_s(ln[k], "self-resonance "); put_freq(p, (uint64_t)(f + 0.5f)); k++; break;
            }
        }
        return k;
    }
    case MS_XTAL: {
        int is = 0; float ms = -1;
        for (int i = 0; i < n; i++) { float m = c_abs2(point_g(i, 1)); if (m > ms) { ms = m; is = i; } }
        int ip = -1; float mp = 1e30f;
        for (int i = is + 1; i < n; i++) { float m = c_abs2(point_g(i, 1)); if (m < mp) { mp = m; ip = i; } }
        if (is == 0 || is >= n - 1 || ip <= is || ip >= n - 1) { put_s(ln[k++], "XTAL: sweep across both resonances (fs and fp)"); return k; }
        float fs = freq_at(is, vertex(c_abs(point_g(is - 1, 1)), c_abs(point_g(is, 1)), c_abs(point_g(is + 1, 1))));
        float fp = freq_at(ip, vertex(c_abs(point_g(ip - 1, 1)), c_abs(point_g(ip, 1)), c_abs(point_g(ip + 1, 1))));
        float Rm = z_series21(point_g(is, 1)).re;
        /* C0 from the admittance at the sweep start, below fs: B = wC0 [1 + r / (1 - (f/fs)^2)], r = Cm/C0 */
        float r = (fp / fs) * (fp / fs) - 1, f1 = (float)point_freq(0), w1 = 2 * PI_F * f1;
        cpx z1 = z_series21(point_g(0, 1)); float m2 = c_abs2(z1), B1 = m2 > 0 ? -z1.im / m2 : 0;
        float q = 1 - (f1 / fs) * (f1 / fs), C0 = q != 0 ? B1 / (w1 * (1 + r / q)) : 0, Cm = r * C0;
        float ws = 2 * PI_F * fs, Lm = Cm > 0 ? 1 / (ws * ws * Cm) : 0;
        p = put_s(ln[k], "XTAL: fs "); p = put_freq(p, (uint64_t)(fs + 0.5f)); p = put_s(p, ", fp "); put_freq(p, (uint64_t)(fp + 0.5f)); k++;
        p = put_s(ln[k], "Rm "); p = put_fx(p, Rm, 2); p = put_s(p, " " OHM ", Lm "); p = put_eng(p, Lm, "H");
        p = put_s(p, ", Cm "); p = put_eng(p, Cm, "F"); k++;
        p = put_s(ln[k], "C0 "); p = put_eng(p, C0, "F"); p = put_s(p, ", Q "); put_fx(p, Rm > 0 ? ws * Lm / Rm : 0, 0); k++;
        return k;
    }
    }
    return 0;
}
static void draw_measure(void)
{
    if (meas_mode == MS_OFF) return;
    char ln[6][96]; int k = meas_lines(ln);
    int xm = menu_open ? MX : GX1, y = GY1 - k * RDY - 4;
    lcd_fill(GX0 + 2, y - 2, xm - GX0 - 4, k * RDY + 4, C_BTN);
    for (int i = 0; i < k; i++) {
        int e = ftext(GX0 + 6, y + i * RDY, ln[i], &font_s, 0xFFE0, C_BTN, xm - 4);
        if (e < xm - 2) lcd_fill(e, y + i * RDY, xm - 2 - e, font_s.h, C_BTN);
    }
}
static uint8_t touch_ok = 1;
static void draw_status(void)
{
    int xm = menu_open ? MX : W;
    if (msg[0] && (int32_t)(msg_until - ms_ticks) > 0) { line_text(2, RY0 + 2 * RDY, msg, 0xFFE0, xm); return; }
    msg[0] = 0;
    char b[128], *p = b;
    p = put_freq(p, sweep_cfg.start); p = put_s(p, " - "); p = put_freq(p, sweep_cfg.stop);
    p = put_s(p, "  "); p = put_u(p, sweep_cfg.points, 1); p = put_s(p, " pt  CAL ");
    p = put_s(p, cal_on && cal_valid() ? "on" : cal_have ? "partial" : "none");
    if (edelay != 0) { p = put_s(p, "  ED "); p = put_eng(p, edelay, "s"); }
    if (port_z != 50) { p = put_s(p, "  Z0 "); p = put_fx(p, port_z, 1); }
    p = put_s(p, sweep_running ? "  #" : "  PAUSED #"); p = put_u(p, sweep_count, 1);
    p = put_s(p, "  "); p = put_fx(p, vbat_volts(), 2); p = put_s(p, " V");
    if (!touch_ok) put_s(p, "  touch: uncalibrated");
    line_text(2, RY0 + 2 * RDY, b, C_DIM, xm);
}

/* ---------- version and touch test screens ---------- */
static void info_show(void)
{
    mode = MODE_INFO;
    lcd_fill(0, 0, W, H, C_BG);
    char b[96], *p;
    ftext(16, 12, "AetherVNA", &font_l, C_TEXT, C_BG, W);
    ftext(16, 44, "firmware " GIT_VERSION ", built " __DATE__ " " __TIME__, &font_m, C_TEXT, C_BG, W);
    ftext(16, 68, "github.com/aethersdr/AetherVNA  (GPL-3.0-or-later)", &font_m, C_DIM, C_BG, W);
    ftext(16, 92, "Board: LiteVNA 64 (AT32F403A, 2x MAX2871, MS5351M, 24 MHz TCXO)", &font_s, C_DIM, C_BG, W);
    static const char hx[] = "0123456789abcdef";
    p = put_s(b, "MCU uid ");
    for (int w = 2; w >= 0; w--) for (int k = 7; k >= 0; k--) *p++ = hx[(UID(w) >> (4 * k)) & 15];
    *p = 0; ftext(16, 110, b, &font_s, C_DIM, C_BG, W);
    p = put_s(b, "Battery "); p = put_fx(p, vbat_volts(), 2); p = put_s(p, " V   sweeps "); put_u(p, sweep_count, 1);
    ftext(16, 128, b, &font_s, C_DIM, C_BG, W);
    ftext(16, 146, "Text: Noto Sans (SIL Open Font License 1.1)", &font_s, C_DIM, C_BG, W);
    ftext(16, 290, "tap or hold the jog left to return", &font_m, C_DIM, C_BG, W);
}
static void ttest_show(void)
{
    mode = MODE_TTEST;
    lcd_fill(0, 0, W, H, C_BG);
    ftext(16, 12, "TOUCH TEST: draw on the screen", &font_m, C_TEXT, C_BG, W);
    lcd_fill(W - 90, 0, 90, 40, C_BTN); ftext(W - 70, 11, "EXIT", &font_m, C_TEXT, C_BTN, W);
}

/* ---------- touch ---------- */
/* Screen = a * raw + b on each axis; `swap` when the panel's x is the
 * controller's Y channel (it is on this board). The default is the bench
 * unit's TOUCH CAL of 2026-11-11; TOUCH CAL or `touch map` replaces it. */
static float tm_ax = -0.131839f, tm_bx = 519.89f, tm_ay = 0.094118f, tm_by = -35.39f; static uint8_t tm_swap = 1;
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
static int touch_xy(uint16_t rx, uint16_t ry, int *x, int *y)
{
    float a = tm_swap ? ry : rx, b = tm_swap ? rx : ry;
    *x = (int)(tm_ax * a + tm_bx); *y = (int)(tm_ay * b + tm_by);
    if (flipped) { *x = W - 1 - *x; *y = H - 1 - *y; }
    return *x >= 0 && *x < W && *y >= 0 && *y < H;
}
static void draw_cross(int x, int y)
{
    lcd_fill(0, 0, W, H, C_BG);
    lcd_fill(x - 12, y, 25, 1, C_TEXT); lcd_fill(x, y - 12, 1, 25, C_TEXT);
    ftext(100, 144, "TOUCH CAL: tap the centre of the cross", &font_m, C_TEXT, C_BG, W);
    ftext(100, 166, "(hold the jog left to cancel)", &font_s, C_DIM, C_BG, W);
}
static void touch_cal_start(void)
{
    if (flipped) { flipped = 0; madctl ^= 0xC0; lcd_cmd(0x36, &madctl, 1); }   /* calibrate the unflipped panel */
    mode = MODE_TCAL; tcal_step = 0; draw_cross(tcal_pt[0][0], tcal_pt[0][1]);
}
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
    if (mode == MODE_INFO) { mode = MODE_MAIN; dirty |= D_FULL; return; }
    if (mode == MODE_TTEST) { if (x >= W - 90 && y < 40) { mode = MODE_MAIN; dirty |= D_FULL; } return; }
    if (mode == MODE_GRID) {
        int cols = grid_cols(), cw = W / cols, ch = (H - 64) / 4;
        if (y >= 64) grid_press((x / cw) + ((y - 64) / ch) * cols);
        return;
    }
    if (mode != MODE_MAIN) return;
    if (menu_open) { if (x >= MX) menu_select(y / MROW); else menu_close(); return; }
    menu_show(MN_HOME);
}
static uint32_t touch_t;
static void touch_poll(void)
{
    if (ms_ticks - touch_t < 20) return;
    touch_t = ms_ticks;
    if (pin_read(P_TOUCH_IRQ)) { pen = 0; return; }   /* pen up */
    if (pen && mode != MODE_TTEST) return;           /* one action per touch (the test draws while held) */
    uint16_t rx, ry;
    if (!touch_raw(&rx, &ry)) return;
    int first = !pen; pen = 1;
    if (mode == MODE_TCAL) { touch_cal_point(rx, ry); return; }
    if (!touch_ok) { say("touch uncalibrated: double click jog right, CONFIG, TOUCH CAL", 5000); return; }
    int x, y;
    if (!touch_xy(rx, ry, &x, &y)) return;
    if (mode == MODE_TTEST && !(first && x >= W - 90 && y < 40)) { if (y >= 42) lcd_fill(x - 1, y - 1, 3, 3, 0xFFE0); return; }
    tap(x, y);
}

/* ---------- jog ---------- */
/* With the menu closed, a press moves the marker at once (by JOG STEP, or a
 * point) and repeats while held; a second right press within 350 ms of a
 * quick right click opens the menu (and takes back the click's step). With
 * a menu or grid open, a release under 700 ms is a click (move), 700 ms
 * held is a hold (right selects, left goes back). */
static uint32_t jog_t, jog_down, jog_rep, last_r_click;
static int jog_dir, jog_held_fired, jog_consumed;
static void marker_step(int d)
{
    marker_t *m = &mk[mk_act];
    m->on = 1;
    int n = sweep_cfg.points, i = m->idx + d;
    if (jog_step && n > 1) {                         /* the first point at least JOG STEP away */
        int dir = d > 0 ? 1 : -1;
        uint64_t f = point_freq(m->idx), st = (uint64_t)jog_step * (uint64_t)(d > 0 ? d : -d);
        i = m->idx + dir;
        while (i > 0 && i < n - 1 && (dir > 0 ? point_freq(i) < f + st : point_freq(i) + st > f)) i += dir;
    }
    if (i < 0) i = 0;
    if (i >= n) i = n - 1;
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
    if (mode == MODE_INFO || mode == MODE_TTEST) { if (d < 0) { mode = MODE_MAIN; dirty |= D_FULL; } return; }
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
                marker_step(-1); menu_show(MN_HOME); jog_consumed = 1; last_r_click = 0;
            } else { marker_step(dir); jog_rep = ms_ticks + 400; }
        }
        return;
    }
    if (edge == 2) {
        if (nav) { if (!jog_held_fired && ms_ticks - jog_down >= 700) { jog_held_fired = 1; nav_hold(jog_dir); } }
        else if (jog_consumed != 1 && (int32_t)(ms_ticks - jog_rep) >= 0) {
            marker_step(jog_dir * (sweep_cfg.points > 100 && !jog_step ? 4 : 1)); jog_rep = ms_ticks + 60; jog_consumed = 2;
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

/* ---------- state for store.c ---------- */
typedef struct {
    trace_t tr[4]; marker_t mk[NMK];
    uint8_t tr_act, mk_act, mk_delta, mk_tracking, search_min, grid_values, dot_grid, avg;
    uint32_t jog_step; float edelay, s21_offset, port_z;
} ui_state_t;
typedef struct { float ax, bx, ay, by; uint8_t swap, flipped, touch_ok, pad; } ui_cfg_t;
int ui_state_size(void) { return sizeof(ui_state_t); }
static void bcopy_(void *d, const void *s, int n) { uint8_t *a = d; const uint8_t *b = s; while (n--) *a++ = *b++; }
void ui_state_get(uint8_t *b)
{
    ui_state_t u;
    bcopy_(u.tr, tr, sizeof tr); bcopy_(u.mk, mk, sizeof mk);
    u.tr_act = tr_act; u.mk_act = mk_act; u.mk_delta = mk_delta; u.mk_tracking = mk_tracking; u.search_min = search_min;
    u.grid_values = grid_values; u.dot_grid = dot_grid; u.avg = sweep_cfg.avg;
    u.jog_step = jog_step; u.edelay = edelay; u.s21_offset = s21_offset; u.port_z = port_z;
    bcopy_(b, &u, sizeof u);
}
void ui_state_put(const uint8_t *b)
{
    ui_state_t u; bcopy_(&u, b, sizeof u);
    for (int t = 0; t < 4; t++) if (u.tr[t].fmt >= F_N || !(u.tr[t].scale > 0)) return;   /* not ours: keep the defaults */
    bcopy_(tr, u.tr, sizeof tr); bcopy_(mk, u.mk, sizeof mk);
    tr_act = u.tr_act & 3; mk_act = u.mk_act < NMK ? u.mk_act : 0; mk_delta = u.mk_delta; mk_tracking = u.mk_tracking;
    search_min = u.search_min; grid_values = u.grid_values; dot_grid = u.dot_grid;
    jog_step = u.jog_step; edelay = u.edelay; s21_offset = u.s21_offset; port_z = u.port_z > 0 ? u.port_z : 50;
    clamp_markers();
}
int ui_cfg_get(uint8_t *b)
{
    ui_cfg_t c = { tm_ax, tm_bx, tm_ay, tm_by, tm_swap, flipped, touch_ok, 0 };
    bcopy_(b, &c, sizeof c); return sizeof c;
}
void ui_cfg_put(const uint8_t *b, int n)
{
    if (n != (int)sizeof(ui_cfg_t)) return;
    ui_cfg_t c; bcopy_(&c, b, sizeof c);
    tm_ax = c.ax; tm_bx = c.bx; tm_ay = c.ay; tm_by = c.by; tm_swap = c.swap; touch_ok = c.touch_ok;
    if (c.flipped != flipped) { flipped = c.flipped; madctl ^= 0xC0; if (lcd_ready) lcd_cmd(0x36, &madctl, 1); }
}
void ui_redraw_full(void) { dirty |= D_FULL; }
void ui_render_screen(void)                          /* the whole screen as it stands, for a capture */
{
    if (!ui_on) lcd_test_card();
    else if (mode == MODE_TCAL) draw_cross(tcal_pt[tcal_step][0], tcal_pt[tcal_step][1]);
    else if (mode == MODE_INFO) info_show();
    else if (mode == MODE_TTEST) ttest_show();
    else { dirty = D_FULL; render(); }
}

/* ---------- drawing and entry points ---------- */
static void render(void)
{
    uint16_t d = dirty; dirty = 0;
    if (mode == MODE_TCAL || mode == MODE_INFO || mode == MODE_TTEST) return;
    if (mode == MODE_GRID) { if (d & (D_FULL | D_MENU)) draw_grid(); return; }
    if (d & D_FULL) { lcd_fill(0, 0, W, H, C_BG); d |= D_LEGEND | D_GRAPH | D_READOUT | D_STATUS | D_MENU; }
    if (d & D_LEGEND) draw_legend();
    if (d & D_GRAPH) { draw_axes(); draw_graph(); draw_measure(); }
    if (d & D_READOUT) draw_readout();
    if (d & D_STATUS) draw_status();
    if (menu_open && (d & D_MENU)) draw_menu();
}
void ui_redraw(void) { have_data = 1; dirty |= D_GRAPH | D_LEGEND | D_READOUT | D_STATUS; }
static void ui_start(int sweep)
{
    if (!lcd_ready) lcd_init();
    if (!sg_n) grids_init();
    pin_set(P_BACKLIGHT, 1);
    ui_on = 1; mode = MODE_MAIN; clamp_markers();
    dirty = D_FULL; render(); drawn_count = sweep_count;
    if (sweep) sweep_running = 1;
}
void ui_background(void)
{
    if (!boot_done && ms_ticks > UI_BOOT_MS) { boot_done = 1; ui_start(1); }   /* standalone: screen and sweep at power-on */
    if (boot_pending) {                              /* never reset with the jog held: ROM bootloader hazard */
        if (jog_pressed()) boot_pending = ms_ticks ? ms_ticks : 1;
        else if (ms_ticks - boot_pending > 500) enter_bootloader();
    }
    if (!ui_on || !lcd_ready) return;
    jog_poll(); touch_poll();
    if (sweep_count != drawn_count) {
        drawn_count = sweep_count; have_data = 1;
        if (mk_tracking) { mk[mk_act].idx = marker_search(search_min); mk[mk_act].on = 1; }
        dirty |= D_GRAPH | D_LEGEND | D_READOUT | D_STATUS;
    }
    if (msg[0] && (int32_t)(msg_until - ms_ticks) <= 0) dirty |= D_STATUS;
    if (confirm_a && ms_ticks - confirm_t >= 3000) { confirm_a = 0; if (menu_open) dirty |= D_MENU; }
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
    for (; *s; s++, n++) { if (*s < '0' || *s > '9' || n > 9) return 0; x = x * 10 + (*s - '0'); }
    *v = neg ? -x : x; return n > 0;
}
static void put_f(float v, int dec) { if (v < 0) { con_putc('-'); v = -v; } con_float(v, dec); }
static void put_g(float v)                            /* console: a float in e-notation when small */
{
    float a = v < 0 ? -v : v;
    if (a != 0 && (a < 0.001f || a >= 1e7f)) {
        int e = 0; while (a >= 10) { a /= 10; e++; } while (a < 1) { a *= 10; e--; }
        if (a >= 9.99995f) { a /= 10; e++; }
        if (v < 0) con_putc('-');
        con_float(a, 4); con_putc('e'); con_dec(e);
    } else put_f(v, 4);
}
static void print_trace(int t)
{
    con_udec(t + 1); con_puts(tr[t].on ? " on " : " off "); con_puts(tr[t].ch ? "s21 " : "s11 "); con_puts(fmts[tr[t].fmt].cmd);
    if (fkind(t) == K_SMITH) { con_puts(" readout "); con_puts(tr[t].ch && tr[t].smode >= 3 ? (tr[t].smode == 3 ? "shunt" : "series") : smode_cmd[tr[t].smode]); }
    con_puts(" scale "); put_g(tr[t].scale); con_puts(" ref "); put_g(tr[t].ref); con_puts(" refpos "); con_udec(tr[t].refpos);
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
    if (argc == 1) {
        for (int t = 0; t < 4; t++) print_trace(t);
        for (int k = 0; k < 4; k++) if (mem[k].on) { con_puts("stored "); con_putc((char)('A' + k)); con_puts(" from trace "); con_udec(mem[k].src + 1); con_putc(' '); con_puts(fmts[mem[k].fmt].cmd); con_puts(mem[k].ch ? " s21 " : " s11 "); con_udec(mem[k].n); con_puts(" points\r\n"); }
        return;
    }
    if (argc == 3 && streq(argv[1], "clear") && argv[2][0] >= 'A' && argv[2][0] <= 'D') { mem[argv[2][0] - 'A'].on = 0; dirty |= D_GRAPH; con_puts("ok\r\n"); return; }
    int n; if (!decval(argv[1], &n) || n < 1 || n > 4) goto usage;
    if (argc == 4 && streq(argv[2], "store") && argv[3][0] >= 'A' && argv[3][0] <= 'D') {
        if (!tr[n - 1].on || fkind(n - 1) != K_RECT || !have_data) { con_puts("error: the trace must be shown in a rectangular format\r\n"); return; }
        mem_store(argv[3][0] - 'A', n - 1); dirty |= D_GRAPH; con_puts("ok\r\n"); return;
    }
    int keep = tr_act; tr_act = n - 1;
    trace_t *t = &tr[n - 1];
    for (int k = 2; k < argc; k++) {
        const char *a = argv[k]; float v; int d;
        if (streq(a, "on")) t->on = 1;
        else if (streq(a, "off")) t->on = 0;
        else if (streq(a, "s11")) t->ch = 0;
        else if (streq(a, "s21")) t->ch = 1;
        else if (streq(a, "active")) keep = n - 1;
        else if (streq(a, "auto")) autoscale(n - 1);
        else if (streq(a, "scale") && k + 1 < argc && parse_num(argv[k + 1], 1, &v) && v > 0) { t->scale = v; k++; }
        else if (streq(a, "ref") && k + 1 < argc && parse_num(argv[k + 1], 1, &v)) { t->ref = v; k++; }
        else if (streq(a, "refpos") && k + 1 < argc && decval(argv[k + 1], &d) && d >= 0 && d <= NDIV) { t->refpos = d; k++; }
        else if (streq(a, "readout") && k + 1 < argc) {
            int m = -1, lim = t->ch ? 3 : 9;
            for (int j = 0; j < lim; j++) if (streq(argv[k + 1], smode_cmd[j])) m = j;
            if (t->ch && streq(argv[k + 1], "shunt")) m = 3;
            if (t->ch && streq(argv[k + 1], "series")) m = 4;
            if (m < 0) { tr_act = keep; goto usage; }
            t->smode = m; k++;
        }
        else {
            int f = -1; for (int j = 0; j < F_N; j++) if (streq(a, fmts[j].cmd)) f = j;
            if (f < 0) { tr_act = keep; goto usage; }
            set_format(f, f >= F_S21SR ? 1 : t->ch);
        }
    }
    tr_act = keep;
    dirty |= D_FULL; print_trace(n - 1); return;
usage:
    con_puts("usage: trace [1-4 [on|off|s11|s21|<format>|readout <mode>|scale <v>|ref <v>|refpos <0-8>|auto|active]...] | trace <1-4> store A-D | trace clear A-D\r\n"
             "  formats: logmag phase delay smith swr r x z polar linear real imag q g b y zphase serc serl parr parx parc parl\r\n"
             "           s21sr s21sx s21hr s21hx s21q; smith readouts: lin log reim rjx rlc gjb glc rpjxp rplc (s21: lin log reim shunt series)\r\n");
}
static void cmd_marker(int argc, char **argv)
{
    if (argc == 1) {
        for (int m = 0; m < NMK; m++) print_marker(m);
        con_puts("delta "); con_udec(mk_delta); con_puts(" tracking "); con_udec(mk_tracking);
        con_puts(" search "); con_puts(search_min ? "min" : "max"); con_puts(" jogstep "); con_udec(jog_step); con_puts("\r\n");
        return;
    }
    int v;
    if (streq(argv[1], "delta") && argc == 3) { mk_delta = argv[2][0] == '1'; dirty |= D_LEGEND | D_READOUT; con_puts("ok\r\n"); return; }
    if (streq(argv[1], "tracking") && argc == 3) { mk_tracking = argv[2][0] == '1'; con_puts("ok\r\n"); return; }
    if (streq(argv[1], "jogstep") && argc == 3 && decval(argv[2], &v) && v >= 0) { jog_step = (uint32_t)v; con_puts("ok\r\n"); return; }
    int n; if (!decval(argv[1], &n) || n < 1 || n > NMK) goto usage;
    marker_t *m = &mk[n - 1];
    for (int k = 2; k < argc; k++) {
        const char *a = argv[k];
        if (streq(a, "on")) m->on = 1;
        else if (streq(a, "off")) m->on = 0;
        else if (streq(a, "active")) { mk_act = n - 1; m->on = 1; }
        else if (streq(a, "max")) { m->idx = marker_search(0); m->on = 1; }
        else if (streq(a, "min")) { m->idx = marker_search(1); m->on = 1; }
        else if (streq(a, "minswr")) {
            int best = 0; float bv = 0;
            for (int i = 0; i < sweep_cfg.points; i++) { float g = c_abs(point_g(i, 0)); if (!i || g < bv) { bv = g; best = i; } }
            m->idx = best; m->on = 1;
        }
        else if (streq(a, "left") || streq(a, "right")) { m->idx = marker_search_dir(m->idx, a[0] == 'l' ? -1 : 1, search_min); m->on = 1; }
        else if (decval(a, &v) && v >= 0 && v < sweep_cfg.points) { m->idx = v; m->on = 1; }
        else goto usage;
    }
    dirty |= D_GRAPH | D_LEGEND | D_READOUT; print_marker(n - 1); return;
usage:
    con_puts("usage: marker [1-8 [on|off|active|<point index>|max|min|minswr|left|right]...] | marker delta|tracking 0|1 | marker jogstep <Hz>\r\n");
}
static void cmd_display(int argc, char **argv)
{
    float v;
    if (argc == 3 && streq(argv[1], "edelay") && parse_num(argv[2], 1, &v)) edelay = v;
    else if (argc == 3 && streq(argv[1], "s21offset") && parse_num(argv[2], 1, &v)) s21_offset = v;
    else if (argc == 3 && streq(argv[1], "portz") && parse_num(argv[2], 1, &v) && v > 0) port_z = v;
    else if (argc == 3 && streq(argv[1], "gridvalues")) grid_values = argv[2][0] == '1';
    else if (argc == 3 && streq(argv[1], "dotgrid")) dot_grid = argv[2][0] == '1';
    else if (argc == 3 && streq(argv[1], "flip")) { if ((argv[2][0] == '1') != flipped) { flipped ^= 1; madctl ^= 0xC0; if (lcd_ready) lcd_cmd(0x36, &madctl, 1); } }
    else if (argc != 1) { con_puts("usage: display [edelay <s>|s21offset <dB>|portz <ohm>|gridvalues 0|1|dotgrid 0|1|flip 0|1]\r\n"); return; }
    con_puts("edelay "); put_g(edelay); con_puts(" s21offset "); put_f(s21_offset, 3); con_puts(" portz "); put_f(port_z, 2);
    con_puts(" gridvalues "); con_udec(grid_values); con_puts(" dotgrid "); con_udec(dot_grid); con_puts(" flip "); con_udec(flipped); con_puts("\r\n");
    if (argc != 1) dirty |= D_FULL;
}
static void print_ui(void)
{
    static const char *const mn[] = { "main", "grid", "touchcal", "touchtest", "version" };
    con_puts("mode "); con_puts(mode == MODE_GRID ? (grid_kind == G_KEYPAD ? "keypad" : "bands") : mn[mode]);
    con_puts(" menu "); con_puts(menu_open ? menu_name[menu_id] : "closed");
    if (mode == MODE_GRID) { con_puts(" selected \""); con_puts(grid_label(grid_sel)); con_putc('"'); }
    else if (menu_open) { char b[28]; item_label(menus[menu_id][menu_sel], b); con_puts(" selected \""); con_puts(b); con_putc('"'); }
    if (mode == MODE_GRID && grid_kind == G_KEYPAD) { con_puts(" entry \""); con_puts(kb); con_puts("\" for "); con_puts(target_name()); }
    con_puts(" touch "); con_puts(touch_ok ? "calibrated" : "uncalibrated");
    if (msg[0] && (int32_t)(msg_until - ms_ticks) > 0) { con_puts(" message \""); con_puts(msg); con_putc('"'); }
    con_puts("\r\n");
    if (menu_open && mode == MODE_MAIN) for (int r = 0; r < menu_len(menu_id); r++) {
        char b[28]; int ck = item_label(menus[menu_id][r], b);
        con_puts("  "); con_udec(r); con_putc(' '); con_puts(b);
        con_puts(ck == CK_ON ? " [x]" : ck == CK_OFF ? " [ ]" : ck == CK_DIM ? " (not built yet)" : "");
        con_puts(r == menu_sel ? " <\r\n" : "\r\n");
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
    else if (streq(a, "menu") && argc == 3) {             /* open a menu by name, for scripts */
        int id = -1; for (int k = 0; k < MN_N; k++) if (streq(argv[2], menu_name[k])) id = k;
        if (id < 0) { con_puts("error: no such menu\r\n"); return; }
        mode = MODE_MAIN; menu_show(id);
    }
    else if (streq(a, "select") && argc == 3 && decval(argv[2], &x)) { if (menu_open && mode == MODE_MAIN) menu_select(x); }
    else if (streq(a, "redraw")) dirty |= D_FULL;
    else if (streq(a, "labels")) {                   /* every menu label against its button, at the menu font */
        int keep = menu_id, over = 0, total = 0;
        for (int id = 0; id < MN_N; id++) {
            menu_id = id;
            for (int r = 0; r < menu_len(id); r++) {
                char b[28]; int ck = item_label(menus[id][r], b);
                int room = W - 2 - (MX + 5) - ((ck == CK_ON || ck == CK_OFF) ? 13 : 0), w = ftext_w(b, &font_s);
                total++;
                if (w > room) { over++; con_puts(menu_name[id]); con_puts(": \""); con_puts(b); con_puts("\" "); con_udec(w); con_puts(" px > "); con_udec(room); con_puts("\r\n"); }
            }
        }
        menu_id = keep;
        con_udec(total); con_puts(" labels, "); con_udec(over); con_puts(" too wide\r\n");
        return;
    }
    else { con_puts("usage: ui [state|tap <x> <y>|jog l|r|L|R|rr|menu <name>|select <row>|labels|redraw]\r\n"); return; }
    if (dirty) render();
    print_ui();
}
void ui_command(int argc, char **argv)
{
    const char *c = argv[0];
    if (streq(c, "trace")) { cmd_trace(argc, argv); if (ui_on && lcd_ready && dirty) render(); return; }
    if (streq(c, "marker")) { cmd_marker(argc, argv); if (ui_on && lcd_ready && dirty) render(); return; }
    if (streq(c, "display")) { cmd_display(argc, argv); if (ui_on && lcd_ready && dirty) render(); return; }
    if (streq(c, "ui")) { cmd_ui(argc, argv); return; }
    if (streq(c, "measure")) {                       /* measure [mode] | rl <ohm> | vf <0-1>: prints the results */
        float v;
        if (argc == 3 && streq(argv[1], "rl") && parse_num(argv[2], 1, &v) && v > 0) meas_rl = v;
        else if (argc == 3 && streq(argv[1], "vf") && parse_num(argv[2], 1, &v) && v >= 0.1f && v <= 1) meas_vf = v;
        else if (argc == 2) {
            int m = -1; for (int k = 0; k < MS_N; k++) if (streq(argv[1], meas_name[k])) m = k;
            if (m < 0) { con_puts("usage: measure [off|lc|cable|shuntlc|seriesl|xtal] | measure rl <ohm> | measure vf <0.1-1>\r\n"); return; }
            meas_mode = (uint8_t)m;
        } else if (argc != 1) { con_puts("usage: measure [off|lc|cable|shuntlc|seriesl|xtal] | measure rl <ohm> | measure vf <0.1-1>\r\n"); return; }
        con_puts("mode "); con_puts(meas_name[meas_mode]); con_puts(" rl "); put_f(meas_rl, 2); con_puts(" vf "); put_f(meas_vf, 3); con_puts("\r\n");
        if (meas_mode != MS_OFF) {
            char ln[6][96]; int k = meas_lines(ln);
            for (int i = 0; i < k; i++) {                /* the screen's symbols in ASCII */
                for (const char *q = ln[i]; *q; q++) { char ch = *q; if (ch == OHM[0]) con_puts("ohm"); else if (ch == MU[0]) con_putc('u'); else if (ch == DEG[0]) con_puts("deg"); else con_putc(ch); }
                con_puts("\r\n");
            }
        }
        dirty |= D_GRAPH; if (ui_on && lcd_ready) render(); return;
    }
    if (streq(c, "point")) {                         /* diagnostic: point <i> <s11re> <s11im> <s21re> <s21im> sets a raw point (pause first) */
        int i; float v[4];
        if (argc != 6 || !decval(argv[1], &i) || i < 0 || i >= sweep_cfg.points) { con_puts("usage: point <index> <s11re> <s11im> <s21re> <s21im>\r\n"); return; }
        for (int k = 0; k < 4; k++) if (!parse_num(argv[2 + k], 1, &v[k])) { con_puts("usage: point <index> <s11re> <s11im> <s21re> <s21im>\r\n"); return; }
        pts[i].s11 = c_mk(v[0], v[1]); pts[i].s21 = c_mk(v[2], v[3]); pts[i].f_tuned = sweep_freq(i); pts[i].flags = 0;
        have_data = 1; con_puts("ok\r\n"); return;
    }
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
        ui_render_screen();
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
