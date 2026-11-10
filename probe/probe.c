/*
 * LiteVNA probe firmware, stage 1: our own code runs, USB enumerates, and
 * every pin is classified as an input. Only i2cscan drives pins, and only
 * open-drain low on PB10/PB11 (approved by the maintainer).
 *
 * Copyright (C) 2026 the AetherSDR project
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Written from Artery's AT32F403A/407 Reference Manual (RM V2.06) and the USB
 * 2.0 / CDC 1.2 specifications. It drives no GPIO: every peripheral is reset to
 * its power-on state (pins are inputs), the core runs from the internal 48 MHz
 * oscillator (HICK), and USB uses the chip's internal D+ pull-up, so nothing on
 * the board is touched. A USB CDC-ACM console answers:
 *
 *   id          chip ID, flash size, unique ID, flash access protection
 *   pins        each GPIO read floating, with pull-up and with pull-down
 *   activity    which pins change while all are floating inputs
 *   adc         idle voltage on the analog-capable pins (PA0-PA7, PB0, PB1)
 *   i2cscan     I2C address scan on PB10/PB11 (open-drain only, read probes)
 *   bootloader  ask the bootloader to stay in update mode, then reset
 *   reset       software reset
 *   help
 *
 * Zeenko's bootloader below 0x08004000 and the option bytes are never written.
 */
#include <stdint.h>

#define REG(a) (*(volatile uint32_t *)(a))

/* Cortex-M4 */
#define SCB_VTOR   REG(0xE000ED08)
#define SCB_AIRCR  REG(0xE000ED0C)
#define SYST_CSR   REG(0xE000E010)
#define NVIC_ICER(n) REG(0xE000E180 + 4 * (n))
#define NVIC_ICPR(n) REG(0xE000E280 + 4 * (n))
#define DBG_IDCODE REG(0xE0042000)

/* AT32F403A (RM V2.06) */
#define CRM        0x40021000
#define CRM_CTRL    REG(CRM + 0x00)
#define CRM_CFG     REG(CRM + 0x04)
#define CRM_CLKINT  REG(CRM + 0x08)
#define CRM_APB2RST REG(CRM + 0x0C)
#define CRM_APB1RST REG(CRM + 0x10)
#define CRM_APB2EN  REG(CRM + 0x18)
#define CRM_APB1EN  REG(CRM + 0x1C)
#define CRM_AHBRST  REG(CRM + 0x28)
#define CRM_MISC1   REG(CRM + 0x30)
#define CRM_MISC3   REG(CRM + 0x54)
#define FLASH_USD   REG(0x40022000 + 0x1C)
#define WDT_CMD     REG(0x40003000)
#define ACC        0x40015800
#define ACC_CTRL1   REG(ACC + 0x04)
#define F_SIZE      (*(volatile uint16_t *)0x1FFFF7E0)
#define UID(n)      REG(0x1FFFF7E8 + 4 * (n))

#define USB        0x40005C00
#define USB_EPT(n)  REG(USB + 4 * (n))
#define USB_CTRL    REG(USB + 0x40)
#define USB_INTSTS  REG(USB + 0x44)
#define USB_DEVADDR REG(USB + 0x4C)
#define USB_BUFTBL  REG(USB + 0x50)
#define USB_CFG     REG(USB + 0x60)
#define PMA_BASE   0x40006000           /* 512-byte buffer, 16 bits per 32-bit word */
#define PMA16(off) (*(volatile uint32_t *)(PMA_BASE + 2 * (off)))

/* ---- startup ---- */
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int main(void);
void reset_handler(void);
static void default_handler(void) { for (;;) WDT_CMD = 0xAAAA; }

__attribute__((section(".vectors"), used))
static void (*const vectors[16 + 68])(void) = {
    (void (*)(void))&_estack, reset_handler,
    [2 ... 16 + 67] = default_handler,
};

static void start(void);
__attribute__((naked, noreturn)) void reset_handler(void)
{
    /* assume nothing about the handover: interrupts off, our own stack */
    __asm volatile("cpsid i\n movw r0, #:lower16:_estack\n movt r0, #:upper16:_estack\n msr msp, r0\n b start\n");
}

__attribute__((noreturn, used)) static void start(void)
{
    SCB_VTOR = 0x08004000;
    for (uint32_t *s = &_sidata, *d = &_sdata; d < &_edata;) *d++ = *s++;
    for (uint32_t *d = &_sbss; d < &_ebss;) *d++ = 0;
    main();
    for (;;) WDT_CMD = 0xAAAA;
}

/* freestanding: the compiler may emit calls to these */
void *memset(void *d, int c, unsigned n) { uint8_t *p = d; while (n--) *p++ = c; return d; }
void *memcpy(void *d, const void *s, unsigned n) { uint8_t *p = d; const uint8_t *q = s; while (n--) *p++ = *q++; return d; }

/* ---- clocks and peripherals to a known state ---- */
static void delay(uint32_t n) { while (n--) { WDT_CMD = 0xAAAA; __asm volatile("nop"); } }

static void system_init(void)
{
    SYST_CSR = 0;
    for (int i = 0; i < 8; i++) { NVIC_ICER(i) = 0xFFFFFFFF; NVIC_ICPR(i) = 0xFFFFFFFF; }
    WDT_CMD = 0xAAAA;                                  /* reload, in case the bootloader started it */

    CRM_CTRL |= 1;                                     /* HICK on */
    while (!(CRM_CTRL & 2)) ;
    CRM_CFG &= ~3u;                                    /* system clock from HICK */
    while (CRM_CFG & 0xC) ;
    CRM_CFG = 0;                                       /* no prescalers, no USB/PLL settings */
    CRM_CTRL &= ~(1u << 24);                           /* PLL off */
    CRM_CLKINT = 0x00BF0000;                           /* clear and disable clock interrupts */

    CRM_APB2RST = 0xFFFFFFFF; CRM_APB2RST = 0;         /* every peripheral to power-on state: */
    CRM_APB1RST = 0xFFFFFFFF; CRM_APB1RST = 0;         /* GPIO pins are inputs again          */
    CRM_AHBRST  = 0xFFFFFFFF; CRM_AHBRST  = 0;

    CRM_MISC1 |= 1u << 25;                             /* HICKDIV: HICK runs at 48 MHz */
    CRM_MISC3 |= (1u << 9) | (1u << 8);                /* 48 MHz HICK to the system clock and to USB */
    delay(1000);
}

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
static const char *const strings[] = { "AetherSDR", "LiteVNA probe" };

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

static void configure_data_eps(void)
{
    ep_init(1, 0x0001, BUF_EP1_TX, 0, 0);              /* bulk IN */
    ep_set_tx(1, TX_NAK);
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

/* console buffers */
static uint8_t rx_line[96]; static uint16_t rx_len;
static uint8_t tx_ring[4096]; static uint16_t tx_head, tx_tail; static uint8_t tx_busy;
static void usb_poll(void);
static void tx_kick(void);

static void con_putc(char c)
{
    uint16_t n = (tx_head + 1) % sizeof tx_ring;
    for (uint32_t spin = 0; n == tx_tail && configured && spin < 2000000; spin++) {
        WDT_CMD = 0xAAAA; usb_poll(); tx_kick();       /* full: drain to the host */
    }
    if (n == tx_tail) return;
    tx_ring[tx_head] = c; tx_head = n;
}
static void con_puts(const char *s) { while (*s) con_putc(*s++); }
static void con_hex(uint32_t v, int digits)
{
    static const char hex[] = "0123456789abcdef";
    for (int i = digits - 1; i >= 0; i--) con_putc(hex[(v >> (4 * i)) & 15]);
}
static void con_dec(uint32_t v)
{
    char b[11]; int n = 0;
    do b[n++] = '0' + v % 10; while (v /= 10);
    while (n) con_putc(b[--n]);
}

static void tx_kick(void)
{
    if (!configured || tx_busy || tx_head == tx_tail) return;
    uint8_t chunk[DATA_SIZE]; uint16_t n = 0;
    while (n < DATA_SIZE && tx_tail != tx_head) { chunk[n++] = tx_ring[tx_tail]; tx_tail = (tx_tail + 1) % sizeof tx_ring; }
    pma_write(BUF_EP1_TX, chunk, n); BT(1, 1) = n;
    tx_busy = 1; ep_set_tx(1, TX_VALID);
}

static void command(char *line);

static void usb_poll(void)
{
    uint16_t ists = USB_INTSTS;
    if (ists & 0x0400) { USB_INTSTS = (uint16_t)~0x0400; usb_reset(); return; }   /* reset */
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
            if (c == '\r' || c == '\n') {
                if (rx_len) { rx_line[rx_len] = 0; command((char *)rx_line); rx_len = 0; }
                else con_puts("> ");
            } else if (rx_len < sizeof rx_line - 1) rx_line[rx_len++] = c;
        }
        ep_set_rx(2, RX_VALID);
    } else if (ep == 3 && (r & EP_CTR_TX)) {
        ep_clear(3, EP_CTR_TX);
    }
}

/* ---- stage 1: pins, inputs only ---- */
#define GPIO(p)     (0x40010800 + 0x400 * (p))         /* A=0 .. D=3 */
#define GPIO_CFG(p, n) REG(GPIO(p) + ((n) < 8 ? 0x00 : 0x04))
#define GPIO_IDT(p) REG(GPIO(p) + 0x08)
#define GPIO_ODT(p) REG(GPIO(p) + 0x0C)
#define IOMUX_REMAP REG(0x40010000 + 0x04)
#define ADC1       0x40012400
#define ADC_STS     REG(ADC1 + 0x00)
#define ADC_CTRL2   REG(ADC1 + 0x08)
#define ADC_SPT1    REG(ADC1 + 0x0C)
#define ADC_SPT2    REG(ADC1 + 0x10)
#define ADC_OSQ1    REG(ADC1 + 0x2C)
#define ADC_OSQ3    REG(ADC1 + 0x34)
#define ADC_ODT     REG(ADC1 + 0x4C)
#define DEMCR       REG(0xE000EDFC)
#define DWT_CTRL    REG(0xE0001000)
#define DWT_CYCCNT  REG(0xE0001004)

/* LQFP48 GPIOs, less PA11/PA12 (USB) and PA13/PA14 (SWD). */
static const uint8_t pin_list[] = {
    0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0F,
    0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,
    0x2D,0x2E,0x2F, 0x30,0x31,
};
#define NPINS (sizeof pin_list)

static void pin_name(uint8_t pp) { con_putc('P'); con_putc('A' + (pp >> 4)); con_dec(pp & 15); }
static void pin_cfg(uint8_t pp, uint32_t cfg)          /* cfg: 4 floating, 8 pull, 0 analog */
{
    int p = pp >> 4, n = pp & 15, sh = 4 * (n & 7);
    GPIO_CFG(p, n) = (GPIO_CFG(p, n) & ~(0xFu << sh)) | (cfg << sh);
}
static void pin_pull(uint8_t pp, int up)
{
    int p = pp >> 4, n = pp & 15;
    GPIO_ODT(p) = up ? GPIO_ODT(p) | (1u << n) : GPIO_ODT(p) & ~(1u << n);
    pin_cfg(pp, 8);
}
static int pin_read(uint8_t pp) { return (GPIO_IDT(pp >> 4) >> (pp & 15)) & 1; }
static void pin_release(uint8_t pp) { pin_cfg(pp, 4); GPIO_ODT(pp >> 4) &= ~(1u << (pp & 15)); }

static void gpio_init(void)
{
    CRM_APB2EN |= 1u | (0xFu << 2) | (1u << 9);        /* IOMUX, GPIOA-D, ADC1 clocks */
    IOMUX_REMAP = (IOMUX_REMAP & ~(7u << 24)) | (2u << 24);   /* JTAG off, SWD kept: PA15/PB3/PB4 are GPIO */
    for (unsigned i = 0; i < NPINS; i++) pin_release(pin_list[i]);
    DEMCR |= 1u << 24; DWT_CYCCNT = 0; DWT_CTRL |= 1;
}

static void cmd_pins(void)
{
    con_puts("pin float up down class   (internal pulls ~70 kOhm, 2 ms settle)\r\n");
    for (unsigned i = 0; i < NPINS; i++) {
        uint8_t pp = pin_list[i];
        pin_release(pp); delay(30000); int f = pin_read(pp);
        pin_pull(pp, 1); delay(30000); int u = pin_read(pp);
        pin_pull(pp, 0); delay(30000); int d = pin_read(pp);
        pin_release(pp);
        pin_name(pp); con_puts(" "); con_putc('0' + f); con_puts(" "); con_putc('0' + u); con_puts(" "); con_putc('0' + d);
        con_puts(u == d ? (u ? " held-high" : " held-low") : (u ? " floating" : " contrary"));
        con_puts("\r\n");
    }
}

static void cmd_activity(void)
{
    enum { N = 200000 };
    uint32_t last[4], cnt[4][16] = {{0}};
    for (int p = 0; p < 4; p++) last[p] = GPIO_IDT(p);
    uint32_t t0 = DWT_CYCCNT;
    for (int k = 0; k < N; k++) {
        for (int p = 0; p < 4; p++) {
            uint32_t v = GPIO_IDT(p), x = v ^ last[p]; last[p] = v;
            while (x) { int b = __builtin_ctz(x); cnt[p][b]++; x &= x - 1; }
        }
        if ((k & 1023) == 0) WDT_CMD = 0xAAAA;
    }
    uint32_t cycles = DWT_CYCCNT - t0;
    con_puts("window_us "); con_dec(cycles / 48); con_puts("  samples "); con_dec(N);
    con_puts("  sample_ns "); con_dec(cycles / (N / 1000) * 1000 / 48 / 1000); con_puts("\r\n");
    int any = 0;
    for (unsigned i = 0; i < NPINS; i++) {
        uint8_t pp = pin_list[i]; uint32_t c = cnt[pp >> 4][pp & 15];
        if (!c) continue;
        any = 1; pin_name(pp); con_puts(" edges "); con_dec(c);
        con_puts(" (~"); con_dec((uint32_t)((uint64_t)c * 24000000ull / cycles)); con_puts(" Hz if a clock)\r\n");
    }
    if (!any) con_puts("no pin changed\r\n");
}

static uint32_t adc_read(int ch)
{
    ADC_OSQ1 = 0; ADC_OSQ3 = ch;
    uint32_t sum = 0;
    for (int k = 0; k < 16; k++) {
        ADC_STS = 0;
        ADC_CTRL2 |= 1u << 22;                          /* software trigger */
        for (uint32_t t = 0; !(ADC_STS & 2) && t < 100000; t++) ;
        sum += ADC_ODT & 0xFFF;
    }
    return sum / 16;
}

static void cmd_adc(void)
{
    static const uint8_t apins[] = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07, 0x10,0x11 };
    ADC_SPT1 = 0x00FFFFFF; ADC_SPT2 = 0x3FFFFFFF;       /* longest sample time everywhere */
    ADC_CTRL2 = (7u << 17) | (1u << 20) | (1u << 23) | 1u;   /* software trigger, VINTRV on, ADC on */
    delay(5000);
    ADC_CTRL2 |= 1u << 3; for (uint32_t t = 0; (ADC_CTRL2 & 8) && t < 100000; t++) ;
    ADC_CTRL2 |= 1u << 2; for (uint32_t t = 0; (ADC_CTRL2 & 4) && t < 100000; t++) ;
    uint32_t vref = adc_read(17);
    uint32_t vdda = vref ? 1200u * 4095u / vref : 0;     /* VINTRV typ 1.20 V */
    con_puts("vintrv_raw "); con_dec(vref); con_puts("  vdda_mv "); con_dec(vdda); con_puts("\r\n");
    con_puts("pin ch mv_after_pullup mv_after_pulldown  (same: driven; different: floating)\r\n");
    for (unsigned i = 0; i < sizeof apins; i++) {
        uint8_t pp = apins[i]; int ch = i < 8 ? (int)i : 8 + (int)(i - 8);
        uint32_t mv[2];
        for (int up = 1; up >= 0; up--) {
            pin_pull(pp, up); delay(30000);            /* charge the pin through the internal pull */
            pin_cfg(pp, 0); delay(300);                /* analog, then read soon after */
            mv[1 - up] = adc_read(ch) * vdda / 4095;
        }
        pin_release(pp);
        pin_name(pp); con_puts(" ch"); con_dec(ch); con_puts(" "); con_dec(mv[0]); con_puts(" "); con_dec(mv[1]);
        int diff = (int)mv[0] - (int)mv[1];
        con_puts(diff > 200 || diff < -200 ? " floating\r\n" : " driven\r\n");
    }
    ADC_CTRL2 = 0;
}

/* ---- stage 2a: I2C address scan on PB10/PB11, open-drain only ---- */
/* Open-drain output: writing 1 releases the line (the board's pull-up takes it
 * high), writing 0 pulls it low. The pins are never driven high. Each address
 * gets a read probe that is NACKed at once, so no register is written. */
static uint8_t i2c_scl, i2c_sda;
static void od_set(uint8_t pp, int high)
{
    int p = pp >> 4, n = pp & 15;
    if (high) GPIO_ODT(p) |= 1u << n; else GPIO_ODT(p) &= ~(1u << n);
}
static void od_mode(uint8_t pp) { od_set(pp, 1); pin_cfg(pp, 0x6); }   /* output 2 MHz, open-drain, released */
#define I2C_HALF() delay(60)                             /* ~100 kHz */
static int scl_high(void)
{
    od_set(i2c_scl, 1);
    for (int t = 0; t < 2000; t++) { if (pin_read(i2c_scl)) return 1; delay(10); }
    return 0;                                            /* held low: clock stretching or stuck */
}
static void i2c_start(void) { od_set(i2c_sda, 1); scl_high(); I2C_HALF(); od_set(i2c_sda, 0); I2C_HALF(); od_set(i2c_scl, 0); I2C_HALF(); }
static void i2c_stop(void)  { od_set(i2c_sda, 0); I2C_HALF(); scl_high(); I2C_HALF(); od_set(i2c_sda, 1); I2C_HALF(); }
static int i2c_bit_out(int b)                            /* returns the bit read back on SCL high */
{
    od_set(i2c_sda, b); I2C_HALF();
    scl_high(); I2C_HALF();
    int r = pin_read(i2c_sda);
    od_set(i2c_scl, 0); I2C_HALF();
    return r;
}
static int i2c_probe_read(uint8_t addr)                 /* 1 if ACKed */
{
    i2c_start();
    for (int i = 7; i >= 0; i--) i2c_bit_out(((addr << 1) | 1) >> i & 1);
    int ack = !i2c_bit_out(1);                           /* release SDA, sample ACK */
    if (ack) { for (int i = 0; i < 8; i++) i2c_bit_out(1); i2c_bit_out(1); }  /* read a byte, NACK it */
    i2c_stop();
    return ack;
}
static void i2c_scan_one(uint8_t scl, uint8_t sda)
{
    i2c_scl = scl; i2c_sda = sda;
    od_mode(scl); od_mode(sda); delay(3000);
    con_puts("SCL="); pin_name(scl); con_puts(" SDA="); pin_name(sda);
    if (!pin_read(sda) || !pin_read(scl)) {
        con_puts(": a line is low at idle; clocking 9 times to free it\r\n");
        for (int i = 0; i < 9; i++) { od_set(scl, 0); I2C_HALF(); scl_high(); I2C_HALF(); }
        i2c_stop();
        if (!pin_read(sda) || !pin_read(scl)) { con_puts("  still low, skipped\r\n"); goto out; }
    }
    con_puts(": answering");
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        WDT_CMD = 0xAAAA;
        if (i2c_probe_read(a)) { con_puts(" 0x"); con_hex(a, 2); found++; }
    }
    if (!found) con_puts(" none");
    con_puts("\r\n");
out:
    pin_release(scl); pin_release(sda);
}
static void cmd_i2cscan(void)
{
    i2c_scan_one(0x1A, 0x1B);                            /* PB10 SCL, PB11 SDA: the I2C2 default */
    i2c_scan_one(0x1B, 0x1A);                            /* and swapped */
}

static int streq(const char *a, const char *b) { while (*a && *a == *b) a++, b++; return *a == *b; }

static void command(char *line)
{
    if (streq(line, "id")) {
        uint32_t usd = FLASH_USD;
        con_puts("idcode 0x"); con_hex(DBG_IDCODE, 8);
        con_puts("\r\nflash_kb "); con_dec(F_SIZE);
        con_puts("\r\nuid "); con_hex(UID(2), 8); con_hex(UID(1), 8); con_hex(UID(0), 8);
        con_puts("\r\nseries 0x"); con_hex(UID(2) >> 24, 2);
        con_puts("\r\naccess_protection "); con_puts(usd & 2 ? "on" : "off");
        con_puts("\r\nusd 0x"); con_hex(usd, 8);
        con_puts("\r\nprobe stage1 " __DATE__ " " __TIME__ "\r\n");
    } else if (streq(line, "bootloader")) {
        con_puts("asking the bootloader for update mode; jog-at-power-on if this comes back\r\n");
        for (int i = 0; i < 20000; i++) usb_poll(), tx_kick();
        REG(0x20000000 + 48 * 1024 - 4) = 0xDEADBABE;    /* the open V2 bootloader's location */
        REG(0x20000000 + 96 * 1024 - 4) = 0xDEADBABE;
        USB_CTRL = 0x0003;                               /* detach before reset */
        SCB_AIRCR = 0x05FA0004;
    } else if (streq(line, "reset")) {
        USB_CTRL = 0x0003;
        SCB_AIRCR = 0x05FA0004;
    } else if (streq(line, "pins")) {
        cmd_pins();
    } else if (streq(line, "activity")) {
        cmd_activity();
    } else if (streq(line, "adc")) {
        cmd_adc();
    } else if (streq(line, "i2cscan")) {
        cmd_i2cscan();
    } else if (streq(line, "help")) {
        con_puts("id | pins | activity | adc | i2cscan | bootloader | reset | help\r\n");
    } else {
        con_puts("error: unknown command; help lists them\r\n");
    }
    con_puts("> ");
}

int main(void)
{
    system_init();
    gpio_init();
    CRM_APB1EN |= 1u << 23;                            /* USB clock */
    CRM_APB2EN |= 1u << 22;                            /* ACC clock */
    USB_CFG = 0x0002;                                  /* D+ pull-up off: host sees a disconnect */
    USB_CTRL = 0x0001;                                 /* PHY on, held in reset */
    delay(200000);                                     /* well over the host's disconnect detect time */
    USB_CTRL = 0x0000;
    USB_INTSTS = 0;
    usb_reset();
    USB_CFG = 0x0000;                                  /* D+ pull-up on: connect */
    ACC_CTRL1 = (1u << 8) | (1u << 1) | 1u;            /* trim HICK to USB SOF */
    for (;;) {
        WDT_CMD = 0xAAAA;
        usb_poll();
        tx_kick();
    }
}
