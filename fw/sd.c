/* AetherVNA firmware: microSD card in SPI mode, and a minimal FAT32.
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Written from the SD Association's Physical Layer Simplified Specification
 * (SPI mode: CMD0, CMD8, ACMD41, CMD58, CMD9/10, CMD16/17/24; SDSC cards are
 * byte-addressed, SDHC/SDXC block-addressed) and Microsoft's FAT32 File
 * System Specification (MBR, BPB, FAT, 8.3 directory entries, FSInfo).
 *
 * The slot shares SPI1 (PB3 SCK, PB4 MISO, PB5 MOSI) with the panel and the
 * touch controller; its chip select is PB9 (found by `sd probe`, NOTES.md). Card
 * work runs inside console commands, so nothing draws on the bus meanwhile,
 * and two 512-byte sector buffers are borrowed from adc_buf (no capture runs
 * during a command either).
 *
 * FAT32 support: the first partition (or an unpartitioned volume), the root
 * directory, 8.3 names, reading by cluster chain, and creating or replacing
 * one small root-directory file (both FATs and FSInfo kept consistent). */
#include "hw.h"
#include "vna.h"
#include "lcd.h"

static uint8_t sd_cs = 0x19;                         /* PB9 (stage 13); `sd probe` finds it again */
static uint8_t sd_ok, sd_hc;                         /* initialised; block addressing */
#define SEC  ((uint8_t *)adc_buf)                    /* sector buffer */
#define SEC2 (((uint8_t *)adc_buf) + 512)            /* FAT sector buffer */

/* ---------- SPI-mode commands ---------- */
static void deselect(void) { if (sd_cs != 0xFF) pin_set(sd_cs, 1); spi_xfer(0xFF); }
static void select_(void) { pin_set(P_LCD_CS, 1); pin_set(P_TOUCH_CS, 1); pin_set(sd_cs, 0); spi_xfer(0xFF); }
static uint8_t crc7(const uint8_t *d, int n)
{
    uint8_t c = 0;
    for (int i = 0; i < n; i++) for (int b = 7; b >= 0; b--) {
        uint8_t bit = ((d[i] >> b) & 1) ^ ((c >> 6) & 1);
        c = (uint8_t)((c << 1) & 0x7F); if (bit) c ^= 0x09;
    }
    return (uint8_t)((c << 1) | 1);
}
static uint8_t cmd(uint8_t idx, uint32_t arg)       /* R1, or 0xFF on timeout; CS left low */
{
    uint8_t f[6] = { (uint8_t)(0x40 | idx), (uint8_t)(arg >> 24), (uint8_t)(arg >> 16), (uint8_t)(arg >> 8), (uint8_t)arg, 0 };
    f[5] = crc7(f, 5);
    for (int t = 0; t < 50000 && spi_xfer(0xFF) != 0xFF; t++) ;   /* not busy */
    for (int i = 0; i < 6; i++) spi_xfer(f[i]);
    uint8_t r = 0xFF;
    for (int i = 0; i < 10 && (r = spi_xfer(0xFF)) & 0x80; i++) ;
    return r;
}
static uint8_t acmd(uint8_t idx, uint32_t arg) { cmd(55, 0); return cmd(idx, arg); }
static int wait_token(void)
{
    uint32_t t0 = ms_ticks; uint8_t b;
    while ((b = spi_xfer(0xFF)) == 0xFF) { if (ms_ticks - t0 > 200) return -1; service(); }
    return b == 0xFE ? 0 : -2;
}
static int read_data(uint8_t *buf, int n)
{
    if (wait_token()) return -1;
    for (int i = 0; i < n; i++) buf[i] = spi_xfer(0xFF);
    spi_xfer(0xFF); spi_xfer(0xFF);                  /* CRC16, not checked */
    return 0;
}

/* Probe: each unassigned pin as the chip select, CMD0 after 80 idle clocks.
 * The pins return to floating inputs, as at reset. */
static const uint8_t cand[] = { 0x09, 0x0A, 0x10, 0x12, 0x19, 0x1C, 0x2D };   /* PA9 PA10 PB0 PB2 PB9 PB12 PC13 */
static const char *const cand_n[] = { "PA9", "PA10", "PB0", "PB2", "PB9", "PB12", "PC13" };
static void sd_probe(void)
{
    if (!spi_ready) spi_init();
    pin_set(P_LCD_CS, 1); pin_set(P_TOUCH_CS, 1);
    spi_speed(1);
    int found = -1;
    for (unsigned k = 0; k < sizeof cand; k++) {
        uint8_t p = cand[k];
        pin_out(p, 1); delay_us(1000);
        for (int i = 0; i < 10; i++) spi_xfer(0xFF);
        pin_set(p, 0); spi_xfer(0xFF);
        uint8_t f[6] = { 0x40, 0, 0, 0, 0, 0x95 }, r = 0xFF;
        for (int i = 0; i < 6; i++) spi_xfer(f[i]);
        for (int i = 0; i < 10 && (r = spi_xfer(0xFF)) & 0x80; i++) ;
        pin_set(p, 1); spi_xfer(0xFF);
        pin_input(p, 0);
        con_puts(cand_n[k]); con_puts(": R1 0x"); con_hex(r, 2);
        con_puts(r == 0x01 ? "  <- a card answered: chip select\r\n" : "\r\n");
        if (r == 0x01 && found < 0) found = (int)k;
    }
    spi_speed(0);
    if (found >= 0) { sd_cs = cand[found]; sd_ok = 0; con_puts("sd cs "); con_puts(cand_n[found]); con_puts("\r\n"); }
    else con_puts("no card answered (is one inserted?)\r\n");
}
static int sd_init(void)
{
    sd_ok = 0;
    if (sd_cs == 0xFF) { con_puts("error: chip select unknown (sd probe)\r\n"); return -1; }
    if (!spi_ready) spi_init();
    pin_set(P_LCD_CS, 1); pin_set(P_TOUCH_CS, 1);
    pin_out(sd_cs, 1);
    spi_speed(1);
    for (int i = 0; i < 10; i++) spi_xfer(0xFF);
    select_();
    uint8_t r = cmd(0, 0);
    if (r != 0x01) { deselect(); spi_speed(0); con_puts("error: CMD0 R1 0x"); con_hex(r, 2); con_puts("\r\n"); return -1; }
    int v2 = 0; uint8_t r7[4];
    r = cmd(8, 0x1AA);
    if (!(r & 0x04)) { for (int i = 0; i < 4; i++) r7[i] = spi_xfer(0xFF); v2 = 1; if (r7[3] != 0xAA) { deselect(); spi_speed(0); con_puts("error: CMD8 echo\r\n"); return -1; } }
    uint32_t t0 = ms_ticks;
    do { r = acmd(41, v2 ? 0x40000000u : 0); service(); } while (r == 0x01 && ms_ticks - t0 < 1500);
    if (r != 0x00) { deselect(); spi_speed(0); con_puts("error: ACMD41 R1 0x"); con_hex(r, 2); con_puts("\r\n"); return -1; }
    uint8_t ocr[4] = { 0 };
    if (v2) { r = cmd(58, 0); for (int i = 0; i < 4; i++) ocr[i] = spi_xfer(0xFF); }
    sd_hc = v2 && (ocr[0] & 0x40);
    if (!sd_hc) cmd(16, 512);
    deselect();
    spi_speed(0);                                    /* 12 MHz from here */
    sd_ok = 1;
    return 0;
}
static int sd_read(uint32_t lba, uint8_t *buf)
{
    select_();
    uint8_t r = cmd(17, sd_hc ? lba : lba * 512);
    int e = r ? -1 : read_data(buf, 512);
    deselect();
    return e;
}
static int sd_write(uint32_t lba, const uint8_t *buf)
{
    select_();
    uint8_t r = cmd(24, sd_hc ? lba : lba * 512);
    if (r) { deselect(); return -1; }
    spi_xfer(0xFF); spi_xfer(0xFE);
    for (int i = 0; i < 512; i++) spi_xfer(buf[i]);
    spi_xfer(0xFF); spi_xfer(0xFF);
    uint8_t d = spi_xfer(0xFF);
    uint32_t t0 = ms_ticks;
    while (spi_xfer(0xFF) == 0x00) { if (ms_ticks - t0 > 500) { deselect(); return -3; } service(); }   /* busy */
    deselect();
    return (d & 0x1F) == 0x05 ? 0 : -2;
}
static void sd_info(void)
{
    uint8_t b[16];
    select_();
    if (cmd(10, 0) == 0 && read_data(b, 16) == 0) {
        con_puts("CID: maker 0x"); con_hex(b[0], 2); con_puts(" oem "); con_putc(b[1]); con_putc(b[2]);
        con_puts(" name "); for (int i = 3; i < 8; i++) con_putc(b[i]);
        con_puts(" rev "); con_udec(b[8] >> 4); con_putc('.'); con_udec(b[8] & 15);
        con_puts(" serial 0x"); con_hex((uint32_t)b[9] << 24 | b[10] << 16 | b[11] << 8 | b[12], 8); con_puts("\r\n");
    }
    if (cmd(9, 0) == 0 && read_data(b, 16) == 0) {
        uint64_t bytes;
        if (b[0] >> 6) { uint32_t cs = ((uint32_t)(b[7] & 0x3F) << 16) | b[8] << 8 | b[9]; bytes = (uint64_t)(cs + 1) * 512 * 1024; }
        else {
            uint32_t rbl = b[5] & 15, cs = ((uint32_t)(b[6] & 3) << 10) | b[7] << 2 | b[8] >> 6, mult = ((b[9] & 3) << 1) | b[10] >> 7;
            bytes = (uint64_t)(cs + 1) << (mult + 2 + rbl);
        }
        con_puts("CSD: v"); con_udec((b[0] >> 6) + 1); con_puts(".0, "); con_udec((uint32_t)(bytes >> 20)); con_puts(" MiB\r\n");
    }
    deselect();
}

/* ---------- FAT32 ---------- */
static uint32_t part_lba, fat_lba, fat_sz, data_lba, root_clus, fsinfo_lba, n_clus;
static uint8_t spc, n_fats, mounted;
static uint32_t rd32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static void wr32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static void wr16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static uint32_t clus_lba(uint32_t c) { return data_lba + (c - 2) * spc; }
static int bpb_ok(const uint8_t *b) { return rd16(b + 11) == 512 && b[13] && rd16(b + 14) && b[16] && rd32(b + 36) && b[510] == 0x55 && b[511] == 0xAA; }
static int fat_mount(int verbose)
{
    mounted = 0;
    if (!sd_ok && sd_init()) return -1;
    if (sd_read(0, SEC)) { con_puts("error: reading sector 0\r\n"); return -1; }
    if (SEC[510] != 0x55 || SEC[511] != 0xAA) { con_puts("error: no boot signature in sector 0\r\n"); return -1; }
    part_lba = 0;
    if (!bpb_ok(SEC)) {                              /* an MBR: the first FAT32 partition */
        const uint8_t *e = SEC + 0x1BE;
        if (e[4] != 0x0B && e[4] != 0x0C) { con_puts("error: partition 1 is not FAT32 (type 0x"); con_hex(e[4], 2); con_puts(")\r\n"); return -1; }
        part_lba = rd32(e + 8);
        if (verbose) { con_puts("MBR: partition 1 type 0x"); con_hex(e[4], 2); con_puts(" start "); con_udec(part_lba); con_puts(" sectors "); con_udec(rd32(e + 12)); con_puts("\r\n"); }
        if (sd_read(part_lba, SEC) || !bpb_ok(SEC)) { con_puts("error: no FAT32 boot sector at the partition\r\n"); return -1; }
    }
    spc = SEC[13]; n_fats = SEC[16]; fat_sz = rd32(SEC + 36); root_clus = rd32(SEC + 44);
    fat_lba = part_lba + rd16(SEC + 14);
    fsinfo_lba = part_lba + rd16(SEC + 48);
    data_lba = fat_lba + n_fats * fat_sz;
    uint32_t tot = rd32(SEC + 32);
    n_clus = (tot - (data_lba - part_lba)) / spc;
    if (verbose) {
        con_puts("FAT32: "); con_udec(spc * 512); con_puts("-byte clusters, "); con_udec(n_clus); con_puts(" clusters, ");
        con_udec(n_fats); con_puts(" FATs of "); con_udec(fat_sz); con_puts(" sectors, root cluster "); con_udec(root_clus);
        con_puts(", label \""); for (int i = 71; i < 82; i++) con_putc(SEC[i]); con_puts("\"\r\n");
    }
    mounted = 1;
    return 0;
}
static uint32_t fat_cached = 0xFFFFFFFF;
static uint32_t fat_get(uint32_t c)
{
    uint32_t s = fat_lba + c / 128;
    if (s != fat_cached) { if (sd_read(s, SEC2)) return 0x0FFFFFFF; fat_cached = s; }
    return rd32(SEC2 + (c % 128) * 4) & 0x0FFFFFFF;
}
static int fat_set(uint32_t c, uint32_t v)           /* in every FAT copy */
{
    for (int k = 0; k < n_fats; k++) {
        uint32_t s = fat_lba + k * fat_sz + c / 128;
        if (sd_read(s, SEC2)) return -1;
        uint8_t *p = SEC2 + (c % 128) * 4;
        wr32(p, (rd32(p) & 0xF0000000u) | (v & 0x0FFFFFFF));
        if (sd_write(s, SEC2)) return -1;
    }
    fat_cached = 0xFFFFFFFF;
    return 0;
}
static void name83(const char *n, char out[11])
{
    int i = 0, k = 0;
    for (; k < 11; k++) out[k] = ' ';
    for (k = 0; n[i] && n[i] != '.' && k < 8; i++) out[k++] = (n[i] >= 'a' && n[i] <= 'z') ? n[i] - 32 : n[i];
    while (n[i] && n[i] != '.') i++;
    if (n[i] == '.') for (i++, k = 8; n[i] && k < 11; i++) out[k++] = (n[i] >= 'a' && n[i] <= 'z') ? n[i] - 32 : n[i];
}
typedef struct { uint32_t lba; uint16_t off; uint32_t clus, size; uint8_t attr; } dirent_t;
/* Finds name in the root (or, with name 0, the first free entry); -1 none. */
static int dir_find(const char *name, dirent_t *d)
{
    char n11[11]; if (name) name83(name, n11);
    for (uint32_t c = root_clus; c >= 2 && c < 0x0FFFFFF8; c = fat_get(c)) {
        for (int s = 0; s < spc; s++) {
            uint32_t lba = clus_lba(c) + s;
            if (sd_read(lba, SEC)) return -2;
            for (int o = 0; o < 512; o += 32) {
                uint8_t *e = SEC + o;
                if (!name) { if (e[0] == 0 || e[0] == 0xE5) { d->lba = lba; d->off = o; return 0; } continue; }
                if (e[0] == 0) return -1;
                if (e[0] == 0xE5 || e[11] == 0x0F || (e[11] & 0x08)) continue;
                int eq = 1; for (int k = 0; k < 11; k++) if (e[k] != (uint8_t)n11[k]) eq = 0;
                if (eq) { d->lba = lba; d->off = o; d->attr = e[11]; d->size = rd32(e + 28); d->clus = (uint32_t)rd16(e + 20) << 16 | rd16(e + 26); return 0; }
            }
        }
    }
    return -1;
}
static void fat_ls(void)
{
    uint32_t used = 0;
    for (uint32_t c = root_clus; c >= 2 && c < 0x0FFFFFF8; c = fat_get(c)) {
        for (int s = 0; s < spc; s++) {
            if (sd_read(clus_lba(c) + s, SEC)) { con_puts("error: read\r\n"); return; }
            for (int o = 0; o < 512; o += 32) {
                uint8_t *e = SEC + o;
                if (e[0] == 0) goto done;
                if (e[0] == 0xE5 || e[11] == 0x0F) continue;
                for (int k = 0; k < ((e[11] & 0x08) ? 11 : 8); k++) if (e[k] != ' ' || (e[11] & 0x08)) con_putc(e[k]);
                if (!(e[11] & 0x08) && e[8] != ' ') { con_putc('.'); for (int k = 8; k < 11; k++) if (e[k] != ' ') con_putc(e[k]); }
                if (e[11] & 0x08) con_puts("  (volume label)");
                else if (e[11] & 0x10) con_puts("  <dir>");
                else { con_puts("  "); con_udec(rd32(e + 28)); con_puts(" bytes, cluster "); con_udec((uint32_t)rd16(e + 20) << 16 | rd16(e + 26)); }
                con_puts("\r\n");
            }
        }
    }
done:
    for (uint32_t c = 2; c < n_clus + 2; c++) { if (fat_get(c)) used++; if ((c & 1023) == 0) service(); }
    con_udec(used); con_putc('/'); con_udec(n_clus); con_puts(" clusters in use\r\n");
}
/* Reads a file, handing each sector to fn with its file offset; -1 on error. */
typedef int (*sec_fn)(const uint8_t *b, uint32_t off, int n, void *ctx);
static int fat_read(const dirent_t *d, sec_fn fn, void *ctx)
{
    uint32_t off = 0, c = d->clus;
    while (off < d->size) {
        if (c < 2 || c >= 0x0FFFFFF8) return -1;
        for (int s = 0; s < spc && off < d->size; s++) {
            if (sd_read(clus_lba(c) + s, SEC)) return -1;
            int n = d->size - off < 512 ? (int)(d->size - off) : 512;
            if (fn(SEC, off, n, ctx)) return -2;
            off += n;
        }
        c = fat_get(c);
        service();
    }
    return 0;
}
static uint32_t crc32_upd(uint32_t c, const uint8_t *b, int n)
{
    for (int i = 0; i < n; i++) { c ^= b[i]; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & -(c & 1)); }
    return c;
}
typedef struct { uint32_t crc, bad_at, n_bad; } pat_t;
static int pat_check(const uint8_t *b, uint32_t off, int n, void *ctx)
{
    pat_t *p = ctx;
    for (int k = 0; k < n; k++) {
        uint32_t i = off + k;
        if (b[k] != (uint8_t)(i ^ ((i >> 9) * 37))) { if (!p->n_bad) p->bad_at = i; p->n_bad++; }
    }
    p->crc = crc32_upd(p->crc, b, n);
    return 0;
}
typedef struct { uint32_t crc; char line[48]; int ln; } txt_t;
static int txt_take(const uint8_t *b, uint32_t off, int n, void *ctx)
{
    (void)off; txt_t *t = ctx;
    for (int k = 0; k < n && t->ln >= 0; k++) {
        if (b[k] == '\r' || b[k] == '\n') { t->line[t->ln] = 0; t->ln = -1; }
        else if (t->ln < (int)sizeof t->line - 1) t->line[t->ln++] = b[k];
    }
    t->crc = crc32_upd(t->crc, b, n);
    return 0;
}
static void sd_verify(void)
{
    dirent_t d;
    if (dir_find("VERIFY.TXT", &d)) { con_puts("VERIFY.TXT: not found\r\n"); return; }
    txt_t t = { 0xFFFFFFFF, { 0 }, 0 };
    if (fat_read(&d, txt_take, &t)) { con_puts("VERIFY.TXT: read error\r\n"); return; }
    if (t.ln > 0) t.line[t.ln] = 0;
    const char *want = "AETHERVNA SD TEST v1"; int eq = 1;
    for (int k = 0; want[k] || t.line[k]; k++) if (want[k] != t.line[k]) { eq = 0; break; }
    con_puts("VERIFY.TXT: "); con_udec(d.size); con_puts(" bytes, CRC32 0x"); con_hex(~t.crc, 8);
    con_puts(", first line \""); con_puts(t.line); con_puts(eq ? "\": ok\r\n" : "\": MISMATCH\r\n");
    if (dir_find("PATTERN.BIN", &d)) { con_puts("PATTERN.BIN: not found\r\n"); return; }
    pat_t p = { 0xFFFFFFFF, 0, 0 };
    uint32_t t0 = ms_ticks;
    int r = fat_read(&d, pat_check, &p);
    uint32_t ms = ms_ticks - t0;
    con_puts("PATTERN.BIN: "); con_udec(d.size); con_puts(" bytes in "); con_udec(ms); con_puts(" ms");
    if (r) { con_puts(", read error\r\n"); return; }
    con_puts(", CRC32 0x"); con_hex(~p.crc, 8); con_puts(~p.crc == 0x81CB8188u ? " (expected)" : " (expected 0x81cb8188)");
    if (p.n_bad) { con_puts(", "); con_udec(p.n_bad); con_puts(" bad bytes, first at offset "); con_udec(p.bad_at); con_puts("\r\n"); }
    else con_puts(", every byte matches the formula: ok\r\n");
}
static void fsinfo_take(uint32_t c)                  /* one cluster allocated: free count and hint */
{
    if (sd_read(fsinfo_lba, SEC2) || rd32(SEC2) != 0x41615252 || rd32(SEC2 + 484) != 0x61417272) return;
    uint32_t fc = rd32(SEC2 + 488);
    if (fc != 0xFFFFFFFF && fc) wr32(SEC2 + 488, fc - 1);
    wr32(SEC2 + 492, c + 1);
    sd_write(fsinfo_lba, SEC2);
    fat_cached = 0xFFFFFFFF;
}
/* Creates (or overwrites) a root-directory file of at most one sector. */
static int fat_write_small(const char *name, const char *text)
{
    int n = 0; while (text[n]) n++;
    if (n > 512) return -1;
    dirent_t d; uint32_t clus;
    int exists = dir_find(name, &d) == 0;
    if (exists) clus = d.clus;
    else {
        clus = 0;
        for (uint32_t c = 3; c < n_clus + 2; c++) { if (!fat_get(c)) { clus = c; break; } if ((c & 1023) == 0) service(); }
        if (!clus) { con_puts("error: no free cluster\r\n"); return -1; }
    }
    for (int k = 0; k < 512; k++) SEC[k] = k < n ? (uint8_t)text[k] : 0;
    if (sd_write(clus_lba(clus), SEC)) { con_puts("error: data write\r\n"); return -1; }
    if (!exists) {
        if (fat_set(clus, 0x0FFFFFFF)) { con_puts("error: FAT write\r\n"); return -1; }
        fsinfo_take(clus);
        if (dir_find(0, &d)) { con_puts("error: root directory full\r\n"); return -1; }
    }
    if (sd_read(d.lba, SEC)) return -1;
    uint8_t *e = SEC + d.off;
    if (!exists) {
        char n11[11]; name83(name, n11);
        for (int k = 0; k < 32; k++) e[k] = 0;
        for (int k = 0; k < 11; k++) e[k] = (uint8_t)n11[k];
        e[11] = 0x20;                                /* archive */
        uint16_t date = (uint16_t)(((2026 - 1980) << 9) | (11 << 5) | 11), time = 12 << 11;   /* no clock: the date of this code */
        wr16(e + 14, time); wr16(e + 16, date); wr16(e + 18, date); wr16(e + 22, time); wr16(e + 24, date);
        wr16(e + 20, (uint16_t)(clus >> 16)); wr16(e + 26, (uint16_t)clus);
    }
    wr32(e + 28, (uint32_t)n);
    if (sd_write(d.lba, SEC)) { con_puts("error: directory write\r\n"); return -1; }
    con_puts(exists ? "rewrote " : "created "); con_puts(name); con_puts(", "); con_udec(n); con_puts(" bytes, cluster "); con_udec(clus); con_puts("\r\n");
    return 0;
}

/* ---------- console ---------- */
static int streq(const char *a, const char *b) { while (*a && *a == *b) a++, b++; return *a == *b; }
/* Rewrites one sector of a cluster with PATTERN.BIN's formula (repair aid). */
static int pat_sector(uint32_t clus, int sec, uint32_t off)
{
    for (int k = 0; k < 512; k++) { uint32_t i = off + k; SEC[k] = (uint8_t)(i ^ ((i >> 9) * 37)); }
    return sd_write(clus_lba(clus) + sec, SEC);
}
static int parse_u(const char *s, uint32_t *v)
{
    uint32_t x = 0; int n = 0;
    if (s[0] == '0' && s[1] == 'x') { s += 2; for (; *s; s++, n++) { int d = *s >= '0' && *s <= '9' ? *s - '0' : *s >= 'a' && *s <= 'f' ? *s - 'a' + 10 : -1; if (d < 0) return 0; x = x * 16 + d; } }
    else for (; *s; s++, n++) { if (*s < '0' || *s > '9') return 0; x = x * 10 + (*s - '0'); }
    *v = x; return n > 0;
}
void sd_command(int argc, char **argv)
{
    const char *a = argc > 1 ? argv[1] : "";
    fat_cached = 0xFFFFFFFF;                         /* the buffers are shared with the display and captures: never trust them across commands */
    if (streq(a, "probe")) { sd_probe(); return; }
    if (streq(a, "cs") && argc == 3) {
        for (unsigned k = 0; k < sizeof cand; k++) if (streq(argv[2], cand_n[k])) { sd_cs = cand[k]; sd_ok = 0; con_puts("ok\r\n"); return; }
        con_puts("usage: sd cs PA9|PA10|PB0|PB2|PB9|PB12|PC13\r\n"); return;
    }
    if (streq(a, "init")) {
        if (sd_init()) return;
        con_puts("card: "); con_puts(sd_hc ? "SDHC/SDXC (block addressed)\r\n" : "SDSC (byte addressed)\r\n");
        sd_info(); return;
    }
    if (streq(a, "mount")) { fat_mount(1); return; }
    if (!mounted && (streq(a, "ls") || streq(a, "verify") || streq(a, "write") || streq(a, "fat") || streq(a, "patsec") || streq(a, "unlink") || streq(a, "fsinfo")) && fat_mount(0)) return;
    uint32_t u1, u2, u3;
    if (streq(a, "fat") && (argc == 3 || argc == 4) && parse_u(argv[2], &u1)) {   /* sd fat <cluster> [value] */
        if (argc == 4) { if (!parse_u(argv[3], &u2) || fat_set(u1, u2)) { con_puts("error\r\n"); return; } }
        uint32_t lo = u1, hi = argc == 3 ? u1 : u1;
        for (uint32_t c = lo; c <= hi; c++) { con_udec(c); con_puts(" -> 0x"); con_hex(fat_get(c), 8); con_puts("\r\n"); }
        return;
    }
    if (streq(a, "fatrange") && argc == 4 && parse_u(argv[2], &u1) && parse_u(argv[3], &u2)) {   /* sd fatrange <first> <last>: chain breaks only */
        if (!mounted && fat_mount(0)) return;
        uint32_t breaks = 0;
        for (uint32_t c = u1; c <= u2; c++) { uint32_t v = fat_get(c); if (v != c + 1) { con_udec(c); con_puts(" -> 0x"); con_hex(v, 8); con_puts("\r\n"); breaks++; } }
        con_udec(breaks); con_puts(" entries not pointing to the next cluster\r\n"); return;
    }
    if (streq(a, "patsec") && argc == 5 && parse_u(argv[2], &u1) && parse_u(argv[3], &u2) && parse_u(argv[4], &u3)) {   /* sd patsec <cluster> <sector> <file offset> */
        con_puts(pat_sector(u1, (int)u2, u3) ? "error: write\r\n" : "ok\r\n"); return;
    }
    if (streq(a, "unlink") && argc == 3) {           /* mark a root entry deleted; its clusters are left alone */
        dirent_t d; if (dir_find(argv[2], &d)) { con_puts("error: not found\r\n"); return; }
        if (sd_read(d.lba, SEC)) { con_puts("error: read\r\n"); return; }
        SEC[d.off] = 0xE5;
        con_puts(sd_write(d.lba, SEC) ? "error: write\r\n" : "ok\r\n"); return;
    }
    if (streq(a, "fsinfo")) {                        /* sd fsinfo [free next] */
        if (sd_read(fsinfo_lba, SEC2) || rd32(SEC2) != 0x41615252) { con_puts("error: no FSInfo\r\n"); return; }
        if (argc == 4 && parse_u(argv[2], &u1) && parse_u(argv[3], &u2)) { wr32(SEC2 + 488, u1); wr32(SEC2 + 492, u2); if (sd_write(fsinfo_lba, SEC2)) { con_puts("error: write\r\n"); return; } }
        con_puts("free 0x"); con_hex(rd32(SEC2 + 488), 8); con_puts(" ("); con_udec(rd32(SEC2 + 488)); con_puts(") next 0x"); con_hex(rd32(SEC2 + 492), 8); con_puts("\r\n");
        return;
    }
    if (streq(a, "ls")) { fat_ls(); return; }
    if (streq(a, "verify")) { sd_verify(); return; }
    if (streq(a, "write")) { fat_write_small("WRITE.TXT", "AETHERVNA WRITE OK"); return; }
    if (!*a || streq(a, "status")) {
        con_puts("cs "); if (sd_cs == 0xFF) con_puts("unknown"); else for (unsigned k = 0; k < sizeof cand; k++) if (cand[k] == sd_cs) con_puts(cand_n[k]);
        con_puts(sd_ok ? (sd_hc ? " card SDHC" : " card SDSC") : " card not initialised"); con_puts(mounted ? " mounted\r\n" : "\r\n");
        return;
    }
    con_puts("usage: sd [status|probe|cs <pin>|init|mount|ls|verify|write]\r\n");
}
