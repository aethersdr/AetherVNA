/* LiteVNAAI firmware: text console.
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Command names, arguments and output formats of the measurement commands
 * follow NanoVNA-D (github.com/DiSlord/NanoVNA-D, main.c: scan, data,
 * frequencies, sweep, cal, version, info, pause, resume), so tools written for
 * it can drive this one. Everything is plain text, one fact per line. */
#include "hw.h"
#include "vna.h"
#include "ui.h"

#ifndef GIT_VERSION
#define GIT_VERSION "unknown"
#endif

static int argc_; static char *argv_[12];
static void split(char *line)
{
    argc_ = 0;
    while (*line && argc_ < 12) {
        while (*line == ' ') *line++ = 0;
        if (!*line) break;
        argv_[argc_++] = line;
        while (*line && *line != ' ') line++;
    }
}
static int streq(const char *a, const char *b) { while (*a && *a == *b) a++, b++; return *a == *b; }
static int parse_u64(const char *s, uint64_t *v)
{   /* digits with optional k, M, G suffix */
    uint64_t x = 0; int any = 0;
    while (*s >= '0' && *s <= '9') { x = x * 10 + (*s++ - '0'); any = 1; }
    uint64_t frac = 0, div = 1;
    if (*s == '.') { s++; while (*s >= '0' && *s <= '9') { if (div < 1000000000ull) { frac = frac * 10 + (*s - '0'); div *= 10; } s++; any = 1; } }
    uint64_t mul = 1;
    if (*s == 'k' || *s == 'K') { mul = 1000; s++; } else if (*s == 'M') { mul = 1000000; s++; } else if (*s == 'G') { mul = 1000000000; s++; }
    if (!any || *s) return 0;
    *v = x * mul + frac * mul / div;
    return 1;
}
static int parse_f(const char *s, float *v)
{   /* [-]digits[.digits][e[-]digits] */
    int neg = 0, any = 0, e = 0, eneg = 0; double x = 0, sc = 1;
    if (*s == '-' || *s == '+') neg = *s++ == '-';
    while (*s >= '0' && *s <= '9') { x = x * 10 + (*s++ - '0'); any = 1; }
    if (*s == '.') { s++; while (*s >= '0' && *s <= '9') { sc *= 0.1; x += (*s++ - '0') * sc; any = 1; } }
    if (any && (*s == 'e' || *s == 'E')) { s++; if (*s == '-' || *s == '+') eneg = *s++ == '-'; if (!(*s >= '0' && *s <= '9')) return 0;
        while (*s >= '0' && *s <= '9') e = e * 10 + (*s++ - '0'); }
    if (!any || *s || e > 60) return 0;
    while (e--) x = eneg ? x * 0.1 : x * 10;
    *v = (float)(neg ? -x : x);
    return 1;
}
static void put_u64(uint64_t v) { char b[21]; int n = 0; do b[n++] = '0' + v % 10; while (v /= 10); while (n) con_putc(b[--n]); }
static void nl(void) { con_puts("\r\n"); }
static void put_e(float v)
{   /* %e-like, 9 significant digits are more than the data hold; 6 decimals of mantissa */
    if (v == 0) { con_puts("0.000000000"); return; }
    if (v < 0) { con_putc('-'); v = -v; }
    int e = 0;
    while (v >= 10) { v /= 10; e++; }
    while (v < 1) { v *= 10; e--; }
    con_float(v, 9); con_putc('e'); if (e < 0) { con_putc('-'); e = -e; } else con_putc('+');
    if (e < 10) con_putc('0');
    con_udec(e);
}
static void put_cpx(cpx z) { put_e(z.re); con_putc(' '); put_e(z.im); }

static void out_point(int i, int mask)
{
    cpx s11 = pts[i].s11, s21 = pts[i].s21;
    if (cal_on && cal_valid() && !(mask & 8)) cal_apply(i, &s11, &s21);
    int sp = 0;
    if (mask & 1) { put_u64(sweep_freq(i)); sp = 1; }
    if (mask & 2) { if (sp) con_putc(' '); put_cpx(s11); sp = 1; }
    if (mask & 4) { if (sp) con_putc(' '); put_cpx(s21); }
    nl();
}

static int set_sweep(uint64_t start, uint64_t stop, int points)
{
    if (start < FREQ_MIN || stop > FREQ_MAX || stop < start) { con_puts("frequency range is invalid\r\n"); return 0; }
    if (points < 1 || points > POINTS_MAX) { con_puts("sweep points exceeds range 401\r\n"); return 0; }
    sweep_cfg.start = start; sweep_cfg.stop = stop; sweep_cfg.points = points;
    vna_restart();
    return 1;
}

static void cmd_info(void)
{
    con_puts("Board: LiteVNA 64 (AT32F403A, MAX2871 x2, MS5351M, 24 MHz TCXO)\r\n");
    con_puts("Firmware: LiteVNAAI " GIT_VERSION " built " __DATE__ " " __TIME__ "\r\n");
    con_puts("Source: github.com/aethersdr/LiteVNAAI (GPL-3.0-or-later)\r\n");
    con_puts("MCU: idcode 0x"); con_hex(DBG_IDCODE, 8); con_puts(" flash "); con_udec(F_SIZE); con_puts(" KB uid ");
    con_hex(UID(2), 8); con_hex(UID(1), 8); con_hex(UID(0), 8); nl();
    con_puts("Clock: "); con_puts(clock_on_tcxo() ? "TCXO (HEXT, PLL x2)" : "internal (HICK)"); nl();
    con_puts("Range: 50000 6300000000 Hz, points 1..401, IF 12000 Hz\r\n");
    con_puts("Calibration: "); con_puts(cal_on && cal_valid() ? "applied" : (cal_have ? "partial" : "none")); nl();
    con_puts("Sweeps: "); con_udec(sweep_count); con_puts(sweep_running ? " (running)" : " (paused)"); nl();
}

static void cmd_cal(void)
{
    static const char *const names[] = { "open", "short", "load", "thru" };
    if (argc_ == 1) {
        for (int k = 0; k < CAL_N; k++) if (cal_have & (1 << k)) { con_puts(names[k]); con_putc(' '); }
        if (cal_valid()) con_puts(cal_on ? "applied" : "off");
        nl(); return;
    }
    for (int k = 0; k < CAL_N; k++) if (streq(argv_[1], names[k])) {
        uint8_t was = sweep_running; sweep_running = 0;
        int r = cal_measure(k);
        sweep_running = was;
        con_puts(r ? "error: measurement failed\r\n" : "ok\r\n"); return;
    }
    if (streq(argv_[1], "get") && argc_ == 3) {   /* cal get <standard|terms>: one line per point */
        int k = -1; for (int j = 0; j < CAL_N; j++) if (streq(argv_[2], names[j])) k = j;
        cpx a, b, t[6];
        for (int i = 0; i < sweep_cfg.points; i++) {
            if (k >= 0) { if (cal_std_get(k, i, &a, &b) < 0) { con_puts("error: not measured\r\n"); return; } put_cpx(a); con_putc(' '); put_cpx(b); }
            else if (streq(argv_[2], "terms")) { if (cal_terms_get(i, t) < 0) { con_puts("error: not solved\r\n"); return; }
                for (int j = 0; j < 6; j++) { if (j) con_putc(' '); put_cpx(t[j]); } }
            else break;
            nl(); service();
        }
        if (k < 0 && !streq(argv_[2], "terms")) con_puts("usage: cal get <open|short|load|thru|terms>\r\n");
        return;
    }
    if (streq(argv_[1], "put") && argc_ == 8) {   /* cal put <standard> <i> s11re s11im s21re s21im */
        int k = -1; for (int j = 0; j < CAL_N; j++) if (streq(argv_[2], names[j])) k = j;
        uint64_t i; float v[4];
        if (k < 0 || !parse_u64(argv_[3], &i) || !parse_f(argv_[4], &v[0]) || !parse_f(argv_[5], &v[1]) || !parse_f(argv_[6], &v[2]) || !parse_f(argv_[7], &v[3])
            || cal_std_put(k, (int)i, c_mk(v[0], v[1]), c_mk(v[2], v[3])) < 0) { con_puts("error: cal put <standard> <index> s11re s11im s21re s21im\r\n"); return; }
        con_puts("ok\r\n"); return;
    }
    if (streq(argv_[1], "done")) { int r = cal_solve(); con_puts(r == -1 ? "error: need open, short, load and thru\r\n" : r ? "error: singular standards\r\n" : "ok\r\n"); return; }
    if (streq(argv_[1], "on")) { if (cal_valid()) { cal_on = 1; con_puts("ok\r\n"); } else con_puts("error: no calibration for this sweep\r\n"); return; }
    if (streq(argv_[1], "off")) { cal_on = 0; con_puts("ok\r\n"); return; }
    if (streq(argv_[1], "reset")) { cal_reset(); con_puts("ok\r\n"); return; }
    con_puts("usage: cal [open|short|load|thru|done|on|off|reset|get <std|terms>|put <std> <i> 4 values]\r\n");
}

/* raw: one capture at the current tuning, the samples and the device's own
 * DFT, so the host can check the detector (regression test). */
static void cmd_raw(void)
{
    uint64_t f, tuned;
    if (argc_ < 2 || !parse_u64(argv_[1], &f)) { con_puts("usage: raw <freq>\r\n"); return; }
    if (!clock_on_tcxo() && !clock_use_tcxo()) { con_puts("error: no TCXO clock\r\n"); return; }
    sweep_running = 0;
    if (tune(f, &tuned) < 0) { con_puts("error: frequency out of range\r\n"); return; }
    vna_adc_init();
    int n; uint8_t clip; cpx z[4], zi[4];
    capture3(sweep_cfg.settle, adc_buf, &n, &clip, zi);
    dft3(adc_buf, sweep_cfg.settle, z);
    con_puts("tuned "); put_u64(tuned); con_puts(" settle "); con_udec(sweep_cfg.settle); con_puts(" n "); con_udec(n);
    con_puts(" clip "); con_udec(clip); con_puts(" cycles "); con_udec(capture_cycles); con_puts(" expected "); con_udec((uint32_t)(n - 1) * 82u); nl();
    for (int k = 0; k < (sweep_cfg.bracket ? 4 : 3); k++) { con_puts(k == 0 ? "ref " : k == 1 ? "port1 " : k == 2 ? "port2 " : "ref2 "); put_cpx(z[k]); con_puts(" inline "); put_cpx(zi[k]); nl(); }
    for (int i = 0; i < n; i++) { con_hex(adc_buf[i], 3); if ((i & 31) == 31) nl(); }
    nl(); con_puts("end\r\n");
}

static void cmd_help(void)
{
    con_puts("measurement: scan <start> <stop> [points] [outmask] | sweep [start stop [points]] | data [0|1] |\r\n"
             "  frequencies | cal [open|short|load|thru|done|on|off|reset] | cal get <std|terms> |\r\n"
             "  cal put <std> <i> <s11re> <s11im> <s21re> <s21im> | pause | resume | flags |\r\n"
             "  settle [samples] | tsettle [us] | avg [auto|n] | gain [auto|0..3] | bracket [0|1] | retune [0|1]\r\n"
             "instrument: version | info | vbat | clk [tcxo] | backlight [0|1] | touch |\r\n"
             "  lcd [init|test|ui [nosweep]|off|madctl <hex>|inv 0|1|marker <i>|status] |\r\n"
             "  bootloader | reset\r\n"
             "diagnostics: raw <freq> | tune <freq> | ms r <reg> <n> | ms w <reg> <v>... | mx <le pin> <hex> | pin <code> <0|1|r>\r\n"
             "frequencies take k, M, G suffixes; outmask: 1 freq, 2 S11, 4 S21, 8 uncalibrated\r\n");
}

void console_line(char *line)
{
    split(line);
    if (!argc_) return;
    const char *c = argv_[0];
    if (streq(c, "help")) cmd_help();
    else if (streq(c, "version")) { con_puts("LiteVNAAI " GIT_VERSION " " __DATE__ " " __TIME__ "\r\n"); }
    else if (streq(c, "info")) cmd_info();
    else if (streq(c, "sweep")) {
        if (argc_ == 1) { put_u64(sweep_cfg.start); con_putc(' '); put_u64(sweep_cfg.stop); con_putc(' '); con_udec(sweep_cfg.points); nl(); return; }
        uint64_t a, b = sweep_cfg.stop, p = sweep_cfg.points;
        if (!parse_u64(argv_[1], &a) || (argc_ > 2 && !parse_u64(argv_[2], &b)) || (argc_ > 3 && !parse_u64(argv_[3], &p))) {
            con_puts("usage: sweep {start(Hz)} [stop(Hz)] [points]\r\n"); return; }
        set_sweep(a, b, (int)p);
    }
    else if (streq(c, "scan")) {
        uint64_t a, b, p = sweep_cfg.points, m = 0;
        if (argc_ < 3 || argc_ > 5 || !parse_u64(argv_[1], &a) || !parse_u64(argv_[2], &b) ||
            (argc_ > 3 && !parse_u64(argv_[3], &p)) || (argc_ > 4 && !parse_u64(argv_[4], &m))) {
            con_puts("usage: scan {start(Hz)} {stop(Hz)} [points] [outmask]\r\n"); return; }
        if (!set_sweep(a, b, (int)p)) return;
        uint8_t was = sweep_running; sweep_running = 0;
        if (sweep_once() < 0) { con_puts("error: no TCXO clock\r\n"); sweep_running = was; return; }
        sweep_running = was;
        if (m) for (int i = 0; i < sweep_cfg.points; i++) out_point(i, (int)m);
        ui_redraw();
    }
    else if (streq(c, "data")) {
        int sel = argc_ > 1 ? argv_[1][0] - '0' : 0;
        if (sel < 0 || sel > 1) { con_puts("usage: data [array]\r\n"); return; }
        for (int i = 0; i < sweep_cfg.points; i++) out_point(i, sel ? 4 : 2);
    }
    else if (streq(c, "frequencies")) { for (int i = 0; i < sweep_cfg.points; i++) { put_u64(sweep_freq(i)); nl(); } }
    else if (streq(c, "flags")) {
        for (int i = 0; i < sweep_cfg.points; i++) {
            put_u64(pts[i].f_tuned); con_putc(' '); con_float(pts[i].ref, 1); con_putc(' ');
            con_puts(pts[i].flags & PF_RANGE ? "range" : pts[i].flags & PF_CLIP ? "clip" : pts[i].flags & PF_LOWREF ? "lowref" : "ok"); nl();
        }
    }
    else if (streq(c, "cal")) cmd_cal();
    else if (streq(c, "pause")) sweep_running = 0;
    else if (streq(c, "resume")) sweep_running = 1;
    else if (streq(c, "settle")) {
        if (argc_ > 1) { uint64_t v; if (parse_u64(argv_[1], &v) && v <= 300) sweep_cfg.settle = (uint16_t)v; else { con_puts("usage: settle [0..300]\r\n"); return; } }
        con_udec(sweep_cfg.settle); nl();
    }
    else if (streq(c, "avg")) {
        if (argc_ > 1) { uint64_t v; if (streq(argv_[1], "auto")) sweep_cfg.avg = AVG_AUTO;
            else if (parse_u64(argv_[1], &v) && v >= 1 && v <= 64) sweep_cfg.avg = (uint8_t)v; else { con_puts("usage: avg [auto|1..64]\r\n"); return; } }
        if (sweep_cfg.avg == AVG_AUTO) con_puts("auto"); else con_udec(sweep_cfg.avg);
        nl();
    }
    else if (streq(c, "gain")) {
        if (argc_ == 1) { if (sweep_cfg.gain == GAIN_AUTO) con_puts("auto"); else con_udec(sweep_cfg.gain); nl(); return; }
        if (streq(argv_[1], "auto")) sweep_cfg.gain = GAIN_AUTO;
        else if (argv_[1][0] >= '0' && argv_[1][0] <= '3' && !argv_[1][1]) sweep_cfg.gain = argv_[1][0] - '0';
        else { con_puts("usage: gain [auto|0..3]  (bit0 x3, bit1 x1.9, 3 = x8)\r\n"); return; }
        con_puts("ok\r\n");
    }
    else if (streq(c, "retune")) {
        if (argc_ > 1) sweep_cfg.retune = argv_[1][0] == '1';
        con_puts(sweep_cfg.retune ? "1\r\n" : "0\r\n");
    }
    else if (streq(c, "bracket")) {
        if (argc_ > 1) sweep_cfg.bracket = argv_[1][0] == '1';
        con_puts(sweep_cfg.bracket ? "1\r\n" : "0\r\n");
    }
    else if (streq(c, "tsettle")) {
        extern uint32_t tune_settle_us;
        if (argc_ > 1) { uint64_t v; if (parse_u64(argv_[1], &v) && v <= 100000) tune_settle_us = (uint32_t)v; else { con_puts("usage: tsettle [0..100000 us]\r\n"); return; } }
        con_udec(tune_settle_us); con_puts(" us\r\n");
    }
    else if (streq(c, "jog")) {   /* ladder level (PB1, ADC ch9), digital reads, right (PB13) */
        uint32_t sum = 0, ones = 0;
        for (int i = 0; i < 16; i++) sum += adc_single(9);
        for (int i = 0; i < 1000; i++) ones += pin_read(P_JOG_LADDER);
        con_puts("ladder "); con_udec(sum / 16); con_puts(" counts ("); con_udec(sum / 16 * 3300 / 4095); con_puts(" mV) digital-high ");
        con_udec(ones); con_puts("/1000 right "); con_puts(pin_read(P_JOG_RIGHT) ? "up" : "down");
        con_puts(" pressed "); con_udec(jog_pressed()); nl();
    }
    else if (streq(c, "vbat")) { con_udec((uint32_t)(vbat_volts() * 1000 + 0.5f)); con_puts(" mV\r\n"); }
    else if (streq(c, "clk")) {
        if (argc_ > 1 && streq(argv_[1], "tcxo")) con_puts(clock_use_tcxo() ? "ok\r\n" : "error: no clock on the HEXT pins\r\n");
        else { con_puts(clock_on_tcxo() ? "tcxo\r\n" : "internal\r\n"); }
    }
    else if (streq(c, "tune")) {
        uint64_t f, t;
        if (argc_ < 2 || !parse_u64(argv_[1], &f)) { con_puts("usage: tune <freq>\r\n"); return; }
        sweep_running = 0;
        if (tune(f, &t) < 0) con_puts("error: frequency out of range\r\n"); else { put_u64(t); nl(); }
    }
    else if (streq(c, "raw")) cmd_raw();
    else if (streq(c, "ms") && argc_ >= 4) {
        uint64_t reg, v;
        if (!parse_u64(argv_[2], &reg) || reg > 255) { con_puts("usage: ms r <reg> <n> | ms w <reg> <v>...\r\n"); return; }
        if (streq(argv_[1], "r") && parse_u64(argv_[3], &v) && v >= 1 && v <= 16) {
            uint8_t b[16]; if (!ms_read((uint8_t)reg, b, (int)v)) { con_puts("error: no ACK\r\n"); return; }
            for (int i = 0; i < (int)v; i++) { con_puts("0x"); con_hex(b[i], 2); con_putc(i + 1 < (int)v ? ' ' : '\r'); } con_putc('\n');
        } else if (streq(argv_[1], "w")) {
            uint8_t b[9]; int n = 0;
            for (int i = 3; i < argc_ && n < 9; i++) { if (!parse_u64(argv_[i], &v) || v > 255) { con_puts("error: bad value\r\n"); return; } b[n++] = (uint8_t)v; }
            con_puts(ms_write((uint8_t)reg, b, n) ? "ok\r\n" : "error: no ACK\r\n");
        } else con_puts("usage: ms r <reg> <n> | ms w <reg> <v>...\r\n");
    }
    else if (streq(c, "mx") && argc_ == 3) {
        uint64_t le; uint32_t w = 0; const char *q = argv_[2];
        if (!parse_u64(argv_[1], &le) || (le != P_LE_STIM && le != P_LE_LO)) { con_puts("usage: mx <1|4> <hex word>\r\n"); return; }
        while (*q) { char ch = *q++; int d = ch >= '0' && ch <= '9' ? ch - '0' : ch >= 'a' && ch <= 'f' ? ch - 'a' + 10 : -1; if (d < 0) { con_puts("error: hex\r\n"); return; } w = (w << 4) | d; }
        mx_write((uint8_t)le, w); con_puts("ok\r\n");
    }
    else if (streq(c, "pin") && argc_ == 3) {
        uint64_t pp; if (!parse_u64(argv_[1], &pp) || pp >= 64 || (pp >= 0x0B && pp <= 0x0E)) { con_puts("usage: pin <code> <0|1|r> (USB and SWD refused)\r\n"); return; }
        if (argv_[2][0] == 'r') pin_input((uint8_t)pp, 0); else pin_out((uint8_t)pp, argv_[2][0] == '1');
        con_puts("ok\r\n");
    }
    else if (streq(c, "lcd") || streq(c, "backlight") || streq(c, "touch")) ui_command(argc_, argv_);
    else if (streq(c, "bootloader") || streq(c, "reset")) {
        if (jog_pressed()) { con_puts("error: the jog is pressed; a reset now would start the ROM bootloader\r\n"); return; }
        if (streq(c, "bootloader")) { con_puts("entering bootloader\r\n"); enter_bootloader(); }
        con_flush(); SCB_AIRCR = 0x05FA0004;
    }
    else { con_puts(c); con_puts("?\r\n"); }
}
