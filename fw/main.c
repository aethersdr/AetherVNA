/* LiteVNAAI firmware: start-up, clocks, watchdog, boot guard, main loop.
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Runs at 0x08004000 behind Zeenko's bootloader, which is never written.
 * Safety, in order of what runs first:
 *  - The watchdog starts at reset (about 2 s), so any hang reboots the chip.
 *  - Boot guard: after two watchdog resets in a row the firmware writes the
 *    bootloader magic and resets, so Zeenko's bootloader stays in update mode
 *    and a broken image can be replaced from the host without hands.
 *  - The USB console comes up on the internal oscillator before anything
 *    else; the TCXO, synthesizers and display start on demand.
 */
#include "hw.h"
#include "vna.h"
#include "ui.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int main(void);
void reset_handler(void);

/* Faults must NOT feed the watchdog: let it reset the chip. */
static void fault_handler(void) { for (;;) ; }

__attribute__((section(".vectors"), used))
static void (*const vectors[16 + 68])(void) = {
    (void (*)(void))&_estack, reset_handler,
    [2 ... 16 + 67] = fault_handler,
};

/* Survives resets (not cleared by start-up): boot-guard counter. */
__attribute__((section(".noinit"))) static volatile uint32_t guard_magic, guard_count;
#define GUARD_MAGIC 0x4C564E41u          /* "LVNA" */
#define BOOTLOAD_MAGIC 0xDEADBABEu

static void start(void);
__attribute__((naked, noreturn)) void reset_handler(void)
{
    __asm volatile("cpsid i\n movw r0, #:lower16:_estack\n movt r0, #:upper16:_estack\n msr msp, r0\n b start\n");
}

static void watchdog_start(void)
{
    WDT_CMD = 0x5555;                    /* unlock DIV/RLD */
    WDT_DIV = 4;                         /* LICK (~40 kHz) / 64 -> 0.625 kHz */
    WDT_RLD = 1250;                      /* ~2 s */
    WDT_CMD = 0xAAAA;
    WDT_CMD = 0xCCCC;                    /* start */
}
void wdt_feed(void) { WDT_CMD = 0xAAAA; }

static void boot_guard(void)
{
    uint32_t sts = CRM_CTRLSTS;
    int wdt_reset = (sts >> 29) & 1;     /* WDTRSTF */
    CRM_CTRLSTS = sts | (1u << 24);      /* clear reset flags */
    if (guard_magic != GUARD_MAGIC) { guard_magic = GUARD_MAGIC; guard_count = 0; }
    guard_count = wdt_reset ? guard_count + 1 : 0;
    if (guard_count >= 2) {              /* repeated hangs: hand over to the bootloader */
        guard_count = 0;
        REG(0x20000000 + 48 * 1024 - 4) = BOOTLOAD_MAGIC;
        REG(0x20000000 + 96 * 1024 - 4) = BOOTLOAD_MAGIC;
        SCB_AIRCR = 0x05FA0004;
        for (;;) ;
    }
}

__attribute__((noreturn, used)) static void start(void)
{
    SCB_VTOR = 0x08004000;
    watchdog_start();
    for (uint32_t *s = &_sidata, *d = &_sdata; d < &_edata;) *d++ = *s++;
    for (uint32_t *d = &_sbss; d < &_ebss;) *d++ = 0;
    SCB_CPACR |= 0xFu << 20;             /* FPU on */
    __asm volatile("dsb\n isb");
    boot_guard();
    main();
    for (;;) ;
}

void *memset(void *d, int c, unsigned n) { uint8_t *p = d; while (n--) *p++ = c; return d; }
void *memcpy(void *d, const void *s, unsigned n) { uint8_t *p = d; const uint8_t *q = s; while (n--) *p++ = *q++; return d; }

/* ---- clocks ---- */
static uint32_t core_hz = 48000000;
void delay_cycles(uint32_t n) { uint32_t t0 = DWT_CYCCNT; while (DWT_CYCCNT - t0 < n) ; }
void delay_us(uint32_t us)
{
    while (us > 10000) { delay_cycles(core_hz / 100); us -= 10000; wdt_feed(); }
    delay_cycles(us * (core_hz / 1000000));
}

static void clocks_init(void)
{
    SYST_CSR = 0;
    for (int i = 0; i < 8; i++) { NVIC_ICER(i) = 0xFFFFFFFF; NVIC_ICPR(i) = 0xFFFFFFFF; }
    CRM_CTRL |= 1;
    while (!(CRM_CTRL & 2)) ;
    CRM_CFG &= ~3u;
    while (CRM_CFG & 0xC) ;
    CRM_CFG = 0;
    CRM_CTRL &= ~(1u << 24);
    CRM_CLKINT = 0x00BF0000;
    CRM_APB2RST = 0xFFFFFFFF; CRM_APB2RST = 0;
    CRM_APB1RST = 0xFFFFFFFF & ~(1u << 11); CRM_APB1RST = 0;   /* not the watchdog's bus bit */
    CRM_AHBRST  = 0xFFFFFFFF; CRM_AHBRST  = 0;
    CRM_MISC1 |= 1u << 25;               /* HICK 48 MHz */
    CRM_MISC3 |= (1u << 9) | (1u << 8);  /* to SCLK and USB */
    DEMCR |= 1u << 24; DWT_CYCCNT = 0; DWT_CTRL |= 1;
}

int clock_on_tcxo(void) { return ((CRM_CFG >> 2) & 3) == 2; }

/* Core from the 24 MHz TCXO on the HEXT pins through the PLL x2 (48 MHz), so
 * ADC sampling is coherent with the synthesizers. Never touches a running PLL. */
int clock_use_tcxo(void)
{
    if (clock_on_tcxo()) return 1;
    CRM_CTRL |= 1u << 16;
    for (uint32_t t = 0; !(CRM_CTRL & (1u << 17)); t++) { if (t > 2000000) { CRM_CTRL &= ~(1u << 16); return 0; } wdt_feed(); }
    CRM_CTRL &= ~(1u << 24);
    while (CRM_CTRL & (1u << 25)) ;
    uint32_t cfg = CRM_CFG & ~((3u << 29) | (0xFu << 18) | (1u << 17) | (1u << 16) | (1u << 31));
    CRM_CFG = cfg | (1u << 16);
    CRM_CTRL |= 1u << 24;
    for (uint32_t t = 0; !(CRM_CTRL & (1u << 25)); t++) { if (t > 2000000) { CRM_CTRL &= ~(1u << 24); return 0; } wdt_feed(); }
    CRM_CFG = (CRM_CFG & ~3u) | 2u;
    while (((CRM_CFG >> 2) & 3) != 2) ;
    return 1;
}

/* ---- jog safety: a reset with the jog held starts Artery's ROM bootloader ---- */
/* The ladder (PB1) idles at 2.5-2.8 V, only ~0.3 V above the digital-high
 * threshold, so it is read with the ADC: left pulls it to 0 V. */
int jog_left(void) { return adc_single(9) < 1000; }          /* below ~0.8 V */
int jog_pressed(void)
{
    if (!pin_read(P_JOG_RIGHT)) return 1;
    return jog_left();
}

void enter_bootloader(void)
{
    con_flush();
    REG(0x20000000 + 48 * 1024 - 4) = BOOTLOAD_MAGIC;
    REG(0x20000000 + 96 * 1024 - 4) = BOOTLOAD_MAGIC;
    REG(USB_BASE_CTRL) = 0x0003;
    SCB_AIRCR = 0x05FA0004;
    for (;;) ;
}

/* ---- main loop ---- */
volatile uint32_t ms_ticks;
static uint32_t last_cyc;
static void tick(void)
{
    uint32_t now = DWT_CYCCNT, per = core_hz / 1000;
    while (now - last_cyc >= per) { last_cyc += per; ms_ticks++; }
}

void service(void)
{
    wdt_feed();
    tick();
    usb_poll();
}

int main(void)
{
    clocks_init();
    gpio_init();
    usb_init();
    vna_init();
    last_cyc = DWT_CYCCNT;
    int was_configured = 0;
    for (;;) {
        service();
        if (usb_configured()) was_configured = 1;
        else if (!was_configured && usb_host_seen && ms_ticks > 10000) enter_bootloader();   /* host present, no console: hand over */
        usb_take_line();
        vna_background();               /* continuous sweep when running */
        ui_background();
    }
}
