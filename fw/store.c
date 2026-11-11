/* AetherVNA firmware: saved settings and calibrations in flash.
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Written from the AT32F403A reference manual (flash memory controller:
 * unlock keys, sector erase, halfword programming, status flags).
 *
 * Layout, all in bank 1 above the image (link.ld refuses an image that
 * reaches 0x08050000), never the bootloader (0x08000000-0x08003FFF), never
 * the option bytes:
 *   0x08050000 + n * 0x6000   slot n (0-6), 24 KB: sweep, display, calibration
 *   0x0807F000                config, 4 KB: touch map, display flip
 * Each record is erased, written, then given its header (magic, length,
 * CRC32) last, so an interrupted save reads as empty. Slot 0 is recalled at
 * power-on, as on the stock firmware. */
#include "hw.h"
#include "vna.h"
#include "ui.h"

#define STORE_BASE 0x08050000u
#define STORE_END  0x08080000u
#define SLOT_SIZE  0x6000u
#define NSLOTS     7
#define CFG_ADDR   0x0807F000u
#define CFG_SIZE   0x1000u
#define SECTOR     2048u
#define MAGIC_SLOT 0x31534E41u                       /* "ANS1" */
#define MAGIC_CFG  0x31434E41u                       /* "ANC1" */

#define FL_UNLOCK REG(0x40022004)
#define FL_STS    REG(0x4002200C)
#define FL_CTRL   REG(0x40022010)
#define FL_ADDR   REG(0x40022014)

typedef struct { uint32_t magic, len, crc; uint16_t cfg_size, ui_size; } hdr_t;

static int fl_wait(void)
{
    for (uint32_t t = 0; FL_STS & 1; t++) if (t > 50000000u) return -1;     /* OBF */
    uint32_t s = FL_STS;
    FL_STS = 0x34;                                   /* clear PRGMERR, EPPERR, ODF */
    return (s & 0x14) ? -1 : 0;
}
static void fl_unlock(void) { if (FL_CTRL & 0x80) { FL_UNLOCK = 0x45670123u; FL_UNLOCK = 0xCDEF89ABu; } }
static void fl_lock(void) { FL_CTRL |= 0x80; }
static int fl_erase(uint32_t addr, uint32_t len)
{
    if (addr < STORE_BASE || addr + len > STORE_END || (addr & (SECTOR - 1))) return -1;   /* nothing else, ever */
    fl_unlock();
    int e = 0;
    for (uint32_t a = addr; a < addr + len && !e; a += SECTOR) {
        if (fl_wait()) { e = -1; break; }
        FL_CTRL |= 2; FL_ADDR = a; FL_CTRL |= 0x40;  /* SECERS, ERSTR */
        e = fl_wait();
        FL_CTRL &= ~2u;
        for (uint32_t k = 0; k < SECTOR && !e; k += 4) if (*(volatile uint32_t *)(a + k) != 0xFFFFFFFFu) e = -2;
        service();
    }
    fl_lock();
    return e;
}
/* Streaming writer: bytes into halfwords, with a running CRC32. */
typedef struct { uint32_t addr, end, crc; uint16_t half; uint8_t odd, err; } wr_t;
static uint32_t crc_byte(uint32_t c, uint8_t b) { c ^= b; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & -(c & 1)); return c; }
static void prog16(wr_t *w, uint16_t v)
{
    if (w->err) return;
    if (w->addr + 2 > w->end || w->addr < STORE_BASE) { w->err = 1; return; }
    if (fl_wait()) { w->err = 1; return; }
    FL_CTRL |= 1;
    *(volatile uint16_t *)w->addr = v;
    if (fl_wait()) w->err = 1;
    FL_CTRL &= ~1u;
    if (*(volatile uint16_t *)w->addr != v) w->err = 1;
    w->addr += 2;
    if (!(w->addr & 1023)) service();
}
static void wbytes(wr_t *w, const void *p, uint32_t n)
{
    const uint8_t *b = p;
    for (uint32_t i = 0; i < n; i++) {
        w->crc = crc_byte(w->crc, b[i]);
        if (!w->odd) { w->half = b[i]; w->odd = 1; }
        else { prog16(w, (uint16_t)(w->half | b[i] << 8)); w->odd = 0; }
    }
}
static void wflush(wr_t *w) { if (w->odd) { prog16(w, w->half); w->odd = 0; } }
static uint32_t crc_mem(const uint8_t *p, uint32_t n) { uint32_t c = 0xFFFFFFFFu; while (n--) c = crc_byte(c, *p++); return ~c; }
static int put_header(uint32_t at, uint32_t magic, uint32_t len, uint32_t crc)
{
    hdr_t h = { magic, len, crc, (uint16_t)sizeof(sweep_cfg_t), (uint16_t)ui_state_size() };
    wr_t w = { at, at + sizeof h, 0, 0, 0, 0 };
    fl_unlock(); wbytes(&w, &h, sizeof h); wflush(&w); fl_lock();
    return w.err ? -1 : 0;
}
static const hdr_t *valid(uint32_t at, uint32_t magic, uint32_t max)
{
    const hdr_t *h = (const hdr_t *)at;
    if (h->magic != magic || h->len > max - sizeof *h) return 0;
    if (h->cfg_size != sizeof(sweep_cfg_t) || h->ui_size != ui_state_size()) return 0;   /* saved by another layout */
    if (crc_mem((const uint8_t *)(at + sizeof *h), h->len) != h->crc) return 0;
    return h;
}

/* ---------- slots ---------- */
static uint32_t slot_addr(int n) { return STORE_BASE + (uint32_t)n * SLOT_SIZE; }
int store_valid(int n) { return n >= 0 && n < NSLOTS && valid(slot_addr(n), MAGIC_SLOT, SLOT_SIZE) != 0; }
int store_save(int n)
{
    if (n < 0 || n >= NSLOTS) return -1;
    uint32_t at = slot_addr(n);
    if (fl_erase(at, SLOT_SIZE)) return -2;
    wr_t w = { at + sizeof(hdr_t), at + SLOT_SIZE, 0xFFFFFFFFu, 0, 0, 0 };
    fl_unlock();
    wbytes(&w, &sweep_cfg, sizeof sweep_cfg);
    uint8_t ui[UI_STATE_MAX]; ui_state_get(ui); wbytes(&w, ui, ui_state_size());
    /* calibration: the standards measured for this sweep (re-solved on recall) */
    uint64_t cs, ce; int cn;
    uint8_t have = 0;
    if (cal_range(&cs, &ce, &cn) == 0 && cs == sweep_cfg.start && ce == sweep_cfg.stop && cn == sweep_cfg.points) have = cal_have;
    float tmv = 0; uint8_t has_t = have && cal_temp_get(&tmv) == 0;
    uint8_t c[4] = { have, cal_on, cal_enhanced, has_t };   /* byte 3: 1 = the calibration's temperature (float, mV) follows */
    wbytes(&w, c, 4);
    if (has_t) wbytes(&w, &tmv, sizeof tmv);
    for (int k = 0; k <= CAL_ISOLN; k++) if (have & (1u << k))
        for (int i = 0; i < sweep_cfg.points; i++) {
            cpx a, b; cal_std_get(k, i, &a, &b);
            if (k != CAL_ISOLN) wbytes(&w, &a, sizeof a);
            if (k >= CAL_LOAD) wbytes(&w, &b, sizeof b);   /* load: isolation; thru: S21; isoln: S21 */
        }
    wflush(&w);
    fl_lock();
    if (w.err) return -3;
    uint32_t len = w.addr - (at + sizeof(hdr_t));
    return put_header(at, MAGIC_SLOT, len, crc_mem((const uint8_t *)(at + sizeof(hdr_t)), len));
}
int store_recall(int n)
{
    if (!store_valid(n)) return -1;
    const uint8_t *p = (const uint8_t *)(slot_addr(n) + sizeof(hdr_t));
    uint8_t *d = (uint8_t *)&sweep_cfg;
    for (unsigned i = 0; i < sizeof sweep_cfg; i++) d[i] = *p++;
    vna_restart();
    ui_state_put(p); p += ui_state_size();
    uint8_t have = p[0], on = p[1], enh = p[2], has_t = p[3] == 1; p += 4;
    float tmv = 0; if (has_t) { uint8_t *t = (uint8_t *)&tmv; for (unsigned j = 0; j < sizeof tmv; j++) t[j] = *p++; }
    cal_reset(); cal_enhanced = enh;
    for (int k = 0; k <= CAL_ISOLN; k++) if (have & (1u << k))
        for (int i = 0; i < sweep_cfg.points; i++) {
            cpx a = c_mk(0, 0), b = c_mk(0, 0);
            const uint8_t *q;
            if (k != CAL_ISOLN) { q = p; uint8_t *t = (uint8_t *)&a; for (unsigned j = 0; j < sizeof a; j++) t[j] = q[j]; p += sizeof a; }
            if (k >= CAL_LOAD) { q = p; uint8_t *t = (uint8_t *)&b; for (unsigned j = 0; j < sizeof b; j++) t[j] = q[j]; p += sizeof b; }
            cal_std_put(k, i, a, b);
        }
    if (has_t) cal_temp_set(tmv);
    if ((have & 15) == 15 && cal_solve() == 0) cal_on = on;
    ui_redraw_full();
    return 0;
}

/* ---------- config ---------- */
int config_save(void)
{
    if (fl_erase(CFG_ADDR, CFG_SIZE)) return -2;
    uint8_t c[UI_CFG_MAX]; int n = ui_cfg_get(c);
    wr_t w = { CFG_ADDR + sizeof(hdr_t), CFG_ADDR + CFG_SIZE, 0xFFFFFFFFu, 0, 0, 0 };
    fl_unlock(); wbytes(&w, c, (uint32_t)n); wflush(&w); fl_lock();
    if (w.err) return -3;
    return put_header(CFG_ADDR, MAGIC_CFG, (uint32_t)n, crc_mem((const uint8_t *)(CFG_ADDR + sizeof(hdr_t)), (uint32_t)n));
}
int store_erase_all(void) { return fl_erase(STORE_BASE, STORE_END - STORE_BASE); }
void store_boot(void)
{
    const hdr_t *h = valid(CFG_ADDR, MAGIC_CFG, CFG_SIZE);
    if (h) ui_cfg_put((const uint8_t *)(CFG_ADDR + sizeof(hdr_t)), (int)h->len);
    if (store_valid(0)) store_recall(0);
}

/* ---------- console ---------- */
static void put_u64s(char *b, uint64_t v) { char t[21]; int n = 0; do t[n++] = '0' + v % 10; while (v /= 10); while (n) *b++ = t[--n]; *b = 0; }
static void slot_line(int n)
{
    con_udec(n);
    if (!store_valid(n)) { con_puts(" empty\r\n"); return; }
    const uint8_t *p = (const uint8_t *)(slot_addr(n) + sizeof(hdr_t));
    sweep_cfg_t s; uint8_t *d = (uint8_t *)&s; for (unsigned i = 0; i < sizeof s; i++) d[i] = p[i];
    uint8_t have = p[sizeof s + ui_state_size()];
    char b[24];
    con_puts(" sweep "); put_u64s(b, s.start); con_puts(b); con_putc(' '); put_u64s(b, s.stop); con_puts(b);
    con_putc(' '); con_udec(s.points); con_puts((have & 15) == 15 ? " calibrated" : have ? " partial calibration" : " no calibration");
    con_puts("\r\n");
}
static int streq(const char *a, const char *b) { while (*a && *a == *b) a++, b++; return *a == *b; }
void store_command(int argc, char **argv)
{
    const char *c = argv[0];
    int n = argc > 1 && argv[1][0] >= '0' && argv[1][0] <= '9' && !argv[1][1] ? argv[1][0] - '0' : -1;
    if (streq(c, "save")) {
        if (n < 0 || n >= NSLOTS) { con_puts("usage: save <0-6>\r\n"); return; }
        int r = store_save(n); con_puts(r ? "error: flash write failed\r\n" : "ok\r\n"); return;
    }
    if (streq(c, "recall")) {
        if (n < 0 || n >= NSLOTS) { con_puts("usage: recall <0-6>\r\n"); return; }
        con_puts(store_recall(n) ? "error: slot empty\r\n" : "ok\r\n"); return;
    }
    /* config */
    const char *a = argc > 1 ? argv[1] : "";
    if (streq(a, "save")) { con_puts(config_save() ? "error: flash write failed\r\n" : "ok\r\n"); return; }
    if (streq(a, "erase")) { con_puts(store_erase_all() ? "error: erase failed\r\n" : "ok: settings, calibrations and config erased\r\n"); return; }
    if (*a) { con_puts("usage: config [save|erase] (with no argument: list the slots)\r\n"); return; }
    con_puts("config "); con_puts(valid(CFG_ADDR, MAGIC_CFG, CFG_SIZE) ? "saved\r\n" : "default\r\n");
    for (int k = 0; k < NSLOTS; k++) slot_line(k);
}
