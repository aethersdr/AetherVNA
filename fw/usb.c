/* AetherVNA firmware: USB CDC-ACM console (polled), from the probe firmware.
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later
 * Written from the AT32F403A reference manual (USBFS) and the USB 2.0 / CDC 1.2
 * specifications. A received line is held until the main loop takes it, so a
 * long command can keep servicing USB without commands nesting. */
#include "hw.h"
#define USB        0x40005C00
#define USB_EPT(n)  REG(USB + 4 * (n))
#define USB_CTRL    REG(USB + 0x40)
#define USB_INTSTS  REG(USB + 0x44)
#define USB_DEVADDR REG(USB + 0x4C)
#define USB_BUFTBL  REG(USB + 0x50)
#define USB_CFG     REG(USB + 0x60)
#define PMA_BASE   0x40006000
#define PMA16(off) (*(volatile uint32_t *)(PMA_BASE + 2 * (off)))
/* ---- USB CDC-ACM device (polled) ---- */
enum { EP0_SIZE = 64, DATA_SIZE = 64 };
static const uint16_t BUF_EP0_TX = 0x40, BUF_EP0_RX = 0x80, BUF_EP1_TX = 0xC0,
                      BUF_EP2_RX = 0x100, BUF_EP3_TX = 0x140;

static const uint8_t dev_desc[18] = {
    18, 1, 0x00, 0x02, 0x02, 0x00, 0x00, EP0_SIZE,
    0x09, 0x12, 0x01, 0x00,                            /* VID 0x1209 (pid.codes), PID 0x0001 (test) */
    0x00, 0x01, 1, 2, 3, 1,
};
static const uint8_t cfg_desc[67] = {
    9, 2, 67, 0, 2, 1, 0, 0x80, 50,
    9, 4, 0, 0, 1, 0x02, 0x02, 0x01, 0,                /* interface 0: CDC control */
    5, 0x24, 0x00, 0x10, 0x01,                         /* header */
    5, 0x24, 0x01, 0x00, 1,                            /* call management */
    4, 0x24, 0x02, 0x02,                               /* ACM: line coding */
    5, 0x24, 0x06, 0, 1,                               /* union */
    7, 5, 0x83, 0x03, 8, 0, 255,                       /* EP3 IN interrupt */
    9, 4, 1, 0, 2, 0x0A, 0, 0, 0,                      /* interface 1: data */
    7, 5, 0x02, 0x02, DATA_SIZE, 0, 0,                 /* EP2 OUT bulk */
    7, 5, 0x81, 0x02, DATA_SIZE, 0, 0,                 /* EP1 IN bulk */
};
static const char *const strings[] = { "AetherSDR", "AetherVNA" };

static uint8_t line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };
static const uint8_t *ep0_data; static uint16_t ep0_left; static uint8_t ep0_zlp;
static uint8_t pending_addr, configured, setup_req, ep0_buf[64];

static void pma_write(uint16_t off, const uint8_t *src, uint16_t n)
{
    for (uint16_t i = 0; i < n; i += 2)
        PMA16(off + i) = src[i] | (i + 1 < n ? src[i + 1] << 8 : 0);
}
static void pma_read(uint16_t off, uint8_t *dst, uint16_t n)
{
    for (uint16_t i = 0; i < n; i += 2) {
        uint16_t w = PMA16(off + i);
        dst[i] = w; if (i + 1 < n) dst[i + 1] = w >> 8;
    }
}
#define BT(ep, k) PMA16(8 * (ep) + 2 * (k))           /* TXADDR, TXLEN, RXADDR, RXLEN */

/* Endpoint register writes: rc_w0 bits written 1, toggle bits written 0
 * unless they are the ones being changed. */
#define EP_CTR_RX  0x8000u
#define EP_CTR_TX  0x0080u
#define EP_SETUP   0x0800u
#define EP_KEEP    0x070Fu                             /* type, kind, address */
static void ep_set_tx(int ep, uint16_t stat)
{
    uint16_t r = USB_EPT(ep);
    USB_EPT(ep) = (r & EP_KEEP) | EP_CTR_RX | EP_CTR_TX | ((r ^ stat) & 0x0030);
}
static void ep_set_rx(int ep, uint16_t stat)
{
    uint16_t r = USB_EPT(ep);
    USB_EPT(ep) = (r & EP_KEEP) | EP_CTR_RX | EP_CTR_TX | ((r ^ stat) & 0x3000);
}
static void ep_clear(int ep, uint16_t bit)
{
    uint16_t r = USB_EPT(ep);
    USB_EPT(ep) = (r & EP_KEEP) | ((EP_CTR_RX | EP_CTR_TX) & ~bit);
}
#define TX_STALL 0x0010
#define TX_NAK   0x0020
#define TX_VALID 0x0030
#define RX_STALL 0x1000
#define RX_VALID 0x3000

static void ep_init(int ep, uint16_t type_addr, uint16_t tx_off, uint16_t rx_off, uint16_t rx_cnt)
{
    BT(ep, 0) = tx_off; BT(ep, 1) = 0; BT(ep, 2) = rx_off; BT(ep, 3) = rx_cnt;
    uint16_t r = USB_EPT(ep);
    /* write type/address, clear DTOG bits (toggle them if set), stat to NAK/disabled */
    USB_EPT(ep) = type_addr | EP_CTR_RX | EP_CTR_TX | (r & 0x4040);
}

static void ep0_send_next(void)
{
    uint16_t n = ep0_left > EP0_SIZE ? EP0_SIZE : ep0_left;
    pma_write(BUF_EP0_TX, ep0_data, n);
    BT(0, 1) = n;
    ep0_data += n; ep0_left -= n;
    if (n < EP0_SIZE) ep0_zlp = 0;
    ep_set_tx(0, TX_VALID);
}
static void ep0_send(const uint8_t *d, uint16_t n, uint16_t wlen)
{
    if (n > wlen) n = wlen;
    ep0_data = d; ep0_left = n; ep0_zlp = (n % EP0_SIZE == 0 && n < wlen);
    ep0_send_next();
}
static void ep0_stall(void) { ep_set_tx(0, TX_STALL); ep_set_rx(0, RX_STALL); }

static uint8_t str_buf[2 + 2 * 32];
static const uint8_t *string_desc(int idx, uint16_t *len)
{
    static const char hex[] = "0123456789ABCDEF";
    char tmp[25]; const char *s;
    if (idx == 0) { str_buf[0] = 4; str_buf[1] = 3; str_buf[2] = 0x09; str_buf[3] = 0x04; *len = 4; return str_buf; }
    if (idx == 3) {                                    /* serial: the chip's unique ID */
        for (int w = 0; w < 3; w++) for (int b = 0; b < 8; b++) tmp[8 * w + b] = hex[(UID(2 - w) >> (28 - 4 * b)) & 15];
        tmp[24] = 0; s = tmp;
    } else if (idx <= 2) s = strings[idx - 1]; else return 0;
    int n = 0; while (s[n] && n < 32) { str_buf[2 + 2 * n] = s[n]; str_buf[3 + 2 * n] = 0; n++; }
    str_buf[0] = 2 + 2 * n; str_buf[1] = 3; *len = str_buf[0];
    return str_buf;
}

static uint8_t tx_ring[1024];   /* 4 KB before stage 17; output just waits on the host a little more */ static uint16_t tx_head, tx_tail; static uint8_t tx_busy, tx_last_full;

static void configure_data_eps(void)
{
    ep_init(1, 0x0001, BUF_EP1_TX, 0, 0);              /* bulk IN */
    ep_set_tx(1, TX_NAK); tx_busy = 0; tx_last_full = 0;
    ep_init(2, 0x0002, 0, BUF_EP2_RX, 0x8400);         /* bulk OUT, 64 bytes */
    ep_set_rx(2, RX_VALID);
    ep_init(3, 0x0603, BUF_EP3_TX, 0, 0);              /* interrupt IN */
    ep_set_tx(3, TX_NAK);
    configured = 1;
}

static void handle_setup(void)
{
    uint8_t s[8]; pma_read(BUF_EP0_RX, s, 8);
    uint8_t type = s[0], req = s[1];
    uint16_t value = s[2] | s[3] << 8, wlen = s[6] | s[7] << 8;
    setup_req = 0;
    if (type == 0x80 && req == 6) {                    /* GET_DESCRIPTOR */
        uint16_t len;
        switch (value >> 8) {
        case 1: ep0_send(dev_desc, sizeof dev_desc, wlen); return;
        case 2: ep0_send(cfg_desc, sizeof cfg_desc, wlen); return;
        case 3: { const uint8_t *d = string_desc(value & 0xFF, &len); if (d) { ep0_send(d, len, wlen); return; } break; }
        }
    } else if (type == 0x00 && req == 5) {             /* SET_ADDRESS: applied after the status stage */
        pending_addr = value & 0x7F; ep0_send(0, 0, 0); return;
    } else if (type == 0x00 && req == 9) {             /* SET_CONFIGURATION */
        if (value) configure_data_eps();
        ep0_send(0, 0, 0); return;
    } else if (type == 0x80 && req == 8) {             /* GET_CONFIGURATION */
        static uint8_t c; c = configured; ep0_send(&c, 1, wlen); return;
    } else if (type == 0x80 && req == 0) {             /* GET_STATUS */
        static const uint8_t z[2]; ep0_send(z, 2, wlen); return;
    } else if (type == 0x21 && req == 0x20) {          /* SET_LINE_CODING: data stage follows */
        setup_req = 0x20; ep_set_rx(0, RX_VALID); return;
    } else if (type == 0xA1 && req == 0x21) {          /* GET_LINE_CODING */
        ep0_send(line_coding, 7, wlen); return;
    } else if (type == 0x21 && (req == 0x22 || req == 0x23)) {  /* SET_CONTROL_LINE_STATE, SEND_BREAK */
        ep0_send(0, 0, 0); return;
    } else if (type == 0x01 && req == 11) {            /* SET_INTERFACE */
        ep0_send(0, 0, 0); return;
    }
    ep0_stall();
}

static void usb_reset(void)
{
    USB_BUFTBL = 0;
    ep_init(0, 0x0200, BUF_EP0_TX, BUF_EP0_RX, 0x8400);  /* control, 64 bytes */
    ep_set_rx(0, RX_VALID); ep_set_tx(0, TX_NAK);
    for (int i = 1; i < 8; i++) USB_EPT(i) = (USB_EPT(i) & 0x7070);  /* disable (toggle stat bits to 0) */
    USB_DEVADDR = 0x80;
    configured = 0; pending_addr = 0;
}

volatile uint8_t usb_host_seen;
static char rx_line[160]; static uint16_t rx_len; static volatile uint8_t line_ready;
static void tx_kick(void);

void con_putc(char c)
{
    uint16_t n = (tx_head + 1) % sizeof tx_ring;
    for (uint32_t spin = 0; n == tx_tail && configured && spin < 4000000; spin++) { wdt_feed(); usb_poll(); }
    if (n == tx_tail) return;
    tx_ring[tx_head] = c; tx_head = n;
}
void con_write(const char *s, int n) { while (n--) con_putc(*s++); }
void con_puts(const char *s) { while (*s) con_putc(*s++); }
void con_hex(uint32_t v, int digits) { static const char h[] = "0123456789abcdef"; for (int i = digits - 1; i >= 0; i--) con_putc(h[(v >> (4 * i)) & 15]); }
void con_udec(uint32_t v) { char b[11]; int n = 0; do b[n++] = '0' + v % 10; while (v /= 10); while (n) con_putc(b[--n]); }
void con_dec(int32_t v) { if (v < 0) { con_putc('-'); con_udec((uint32_t)-v); } else con_udec(v); }
void con_float(float v, int decimals)
{
    if (v != v) { con_puts("nan"); return; }
    if (v < 0) { con_putc('-'); v = -v; }
    if (v > 4e9f) { con_puts("inf"); return; }
    float scale = 1; for (int i = 0; i < decimals; i++) scale *= 10;
    uint64_t q = (uint64_t)(v * scale + 0.5f);
    uint64_t ip = q / (uint64_t)scale, fp = q % (uint64_t)scale;
    con_udec((uint32_t)ip);
    if (decimals) { con_putc('.'); char b[12]; for (int i = decimals - 1; i >= 0; i--) { b[i] = '0' + fp % 10; fp /= 10; } con_write(b, decimals); }
}
void con_flush(void) { for (uint32_t t = 0; (tx_head != tx_tail || tx_busy) && configured && t < 4000000; t++) { wdt_feed(); usb_poll(); } }
int usb_configured(void) { return configured; }
void usb_poll(void)
{
    tx_kick();
    uint16_t ists = USB_INTSTS;
    if (ists & 0x0400) { USB_INTSTS = (uint16_t)~0x0400; usb_reset(); usb_host_seen = 1; return; }   /* bus reset: a host is there */
    if (ists & 0x0800) USB_INTSTS = (uint16_t)~0x0800;                            /* suspend: ignore */
    if (ists & 0x1000) USB_INTSTS = (uint16_t)~0x1000;                            /* wakeup */
    if (ists & 0x0200) USB_INTSTS = (uint16_t)~0x0200;                            /* SOF */
    if (ists & 0x0100) USB_INTSTS = (uint16_t)~0x0100;                            /* expected SOF missing */
    if (ists & 0x2000) USB_INTSTS = (uint16_t)~0x2000;                            /* error */
    if (ists & 0x4000) USB_INTSTS = (uint16_t)~0x4000;                            /* buffer overrun */
    if (!(ists & 0x8000)) return;
    int ep = ists & 0x0F;
    uint16_t r = USB_EPT(ep);
    if (ep == 0) {
        if (r & EP_CTR_TX) {
            ep_clear(0, EP_CTR_TX);
            if (pending_addr) { USB_DEVADDR = 0x80 | pending_addr; pending_addr = 0; }
            if (ep0_left || ep0_zlp) ep0_send_next();
            else ep_set_rx(0, RX_VALID);
        }
        if (r & EP_CTR_RX) {
            ep_clear(0, EP_CTR_RX);
            if (r & EP_SETUP) handle_setup();
            else if (setup_req == 0x20) {
                pma_read(BUF_EP0_RX, ep0_buf, 7);
                for (int i = 0; i < 7; i++) line_coding[i] = ep0_buf[i];
                setup_req = 0; ep0_send(0, 0, 0);
            } else ep_set_rx(0, RX_VALID);             /* status OUT of an IN transfer */
        }
    } else if (ep == 1 && (r & EP_CTR_TX)) {
        ep_clear(1, EP_CTR_TX); tx_busy = 0;
    } else if (ep == 2 && (r & EP_CTR_RX)) {
        ep_clear(2, EP_CTR_RX);
        uint16_t n = BT(2, 3) & 0x3FF; uint8_t buf[DATA_SIZE];
        pma_read(BUF_EP2_RX, buf, n);
        for (uint16_t i = 0; i < n; i++) {
            char c = buf[i];
            if (line_ready) break;                       /* only while the previous line runs: dropped */
            if (c == '\r' || c == '\n') { if (rx_len) { rx_line[rx_len] = 0; line_ready = 1; } }
            else if (rx_len < sizeof rx_line - 1) rx_line[rx_len++] = c;
        }
        if (!line_ready) ep_set_rx(2, RX_VALID);         /* else NAK until the line has run */
    } else if (ep == 3 && (r & EP_CTR_TX)) {
        ep_clear(3, EP_CTR_TX);
    }
}

/* A transfer that ends on a full packet needs a zero-length packet after it:
 * the host's read (cdc-acm queues 128-byte reads) otherwise stays open
 * waiting for more, and the tail of a reply - the prompt - is held there
 * until the next reply arrives. */
static void tx_kick(void)
{
    if (!configured || tx_busy) return;
    if (tx_head == tx_tail) {
        if (tx_last_full) { tx_last_full = 0; BT(1, 1) = 0; tx_busy = 1; ep_set_tx(1, TX_VALID); }
        return;
    }
    uint8_t chunk[DATA_SIZE]; uint16_t n = 0;
    while (n < DATA_SIZE && tx_tail != tx_head) { chunk[n++] = tx_ring[tx_tail]; tx_tail = (tx_tail + 1) % sizeof tx_ring; }
    pma_write(BUF_EP1_TX, chunk, n); BT(1, 1) = n;
    tx_last_full = (n == DATA_SIZE);
    tx_busy = 1; ep_set_tx(1, TX_VALID);
}

/* Called from the main loop only: run a received line, then accept the next. */
void usb_take_line(void)
{
    if (!line_ready) return;
    console_line(rx_line);
    con_puts("ch> ");
    rx_len = 0; line_ready = 0;
    ep_set_rx(2, RX_VALID);
}

void usb_init(void)
{
    CRM_APB1EN |= 1u << 23;
    CRM_APB2EN |= 1u << 22;
    USB_CFG = 0x0002;                                   /* D+ pull-up off: host sees a disconnect */
    USB_CTRL = 0x0001;
    for (int i = 0; i < 20; i++) delay_us(1000);
    USB_CTRL = 0x0000;
    USB_INTSTS = 0;
    usb_reset();
    USB_CFG = 0x0000;
    ACC_CTRL1 = (1u << 8) | (1u << 1) | 1u;
}
