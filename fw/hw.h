/* AetherVNA firmware: hardware definitions and shared declarations.
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Register addresses from Artery's AT32F403A/407 Reference Manual (RM V2.06).
 * Board wiring as measured with the probe firmware (see NOTES.md). */
#pragma once
#include <stdint.h>

#define REG(a)   (*(volatile uint32_t *)(a))
#define REG16(a) (*(volatile uint16_t *)(a))

/* Cortex-M4 */
#define SCB_VTOR    REG(0xE000ED08)
#define SCB_AIRCR   REG(0xE000ED0C)
#define SCB_CPACR   REG(0xE000ED88)
#define SYST_CSR    REG(0xE000E010)
#define NVIC_ICER(n) REG(0xE000E180 + 4 * (n))
#define NVIC_ICPR(n) REG(0xE000E280 + 4 * (n))
#define DBG_IDCODE  REG(0xE0042000)
#define DEMCR       REG(0xE000EDFC)
#define DWT_CTRL    REG(0xE0001000)
#define DWT_CYCCNT  REG(0xE0001004)

/* AT32F403A */
#define CRM         0x40021000
#define CRM_CTRL    REG(CRM + 0x00)
#define CRM_CFG     REG(CRM + 0x04)
#define CRM_CLKINT  REG(CRM + 0x08)
#define CRM_APB2RST REG(CRM + 0x0C)
#define CRM_APB1RST REG(CRM + 0x10)
#define CRM_APB2EN  REG(CRM + 0x18)
#define CRM_APB1EN  REG(CRM + 0x1C)
#define CRM_CTRLSTS REG(CRM + 0x24)
#define CRM_AHBRST  REG(CRM + 0x28)
#define CRM_MISC1   REG(CRM + 0x30)
#define CRM_MISC3   REG(CRM + 0x54)
#define FLASH_USD   REG(0x40022000 + 0x1C)
#define WDT_CMD     REG(0x40003000)
#define WDT_DIV     REG(0x40003004)
#define WDT_RLD     REG(0x40003008)
#define WDT_STS     REG(0x4000300C)
#define ACC_CTRL1   REG(0x40015800 + 0x04)
#define F_SIZE      REG16(0x1FFFF7E0)
#define UID(n)      REG(0x1FFFF7E8 + 4 * (n))
#define IOMUX_REMAP REG(0x40010000 + 0x04)
#define GPIO(p)     (0x40010800 + 0x400 * (p))
#define GPIO_CFG(p, n) REG(GPIO(p) + ((n) < 8 ? 0x00 : 0x04))
#define GPIO_IDT(p) REG(GPIO(p) + 0x08)
#define GPIO_ODT(p) REG(GPIO(p) + 0x0C)
#define GPIO_SCR(p) REG(GPIO(p) + 0x10)
#define GPIO_CLR(p) REG(GPIO(p) + 0x14)
#define ADC1        0x40012400
#define ADC_STS     REG(ADC1 + 0x00)
#define ADC_CTRL1   REG(ADC1 + 0x04)
#define ADC_CTRL2   REG(ADC1 + 0x08)
#define ADC_SPT1    REG(ADC1 + 0x0C)
#define ADC_SPT2    REG(ADC1 + 0x10)
#define ADC_OSQ1    REG(ADC1 + 0x2C)
#define ADC_OSQ3    REG(ADC1 + 0x34)
#define ADC_ODT     REG(ADC1 + 0x4C)
#define USB_BASE_CTRL (0x40005C00 + 0x40)
#define SPI1        0x40013000
#define SPI_CTRL1   REG(SPI1 + 0x00)
#define SPI_CTRL2   REG(SPI1 + 0x04)
#define SPI_STS     REG(SPI1 + 0x08)
#define SPI_DT      REG(SPI1 + 0x0C)

/* Pins: port * 16 + pin */
enum {
    P_ADC_IF = 0x00,   /* PA0: IF, ADC ch0 */
    P_LE_STIM = 0x01,  /* PA1: MAX2871 #1 (stimulus) LE */
    P_SYN_CLK = 0x02,  /* PA2: MAX2871 CLK (shared) */
    P_SYN_DATA = 0x03, /* PA3: MAX2871 DATA (shared) */
    P_LE_LO = 0x04,    /* PA4: MAX2871 #2 (LO) LE */
    P_BACKLIGHT = 0x05,/* PA5: backlight, high = on */
    P_STIM_PORT = 0x06,/* PA6: low = stimulus to port 1, high = to the reference path */
    P_RX_SEL = 0x07,   /* PA7: low = port 1 (or reference with PA6 high), high = port 2 */
    P_BAND = 0x08,     /* PA8: low = MS5351M stimulus, high = MAX2871 stimulus */
    P_LCD_CS = 0x0F,   /* PA15 */
    P_JOG_LADDER = 0x11, /* PB1: jog left (analog ladder, ADC ch9) */
    P_LCD_SCK = 0x13, P_TOUCH_MISO = 0x14, P_LCD_MOSI = 0x15, /* PB3/PB4/PB5: SPI1 remapped */
    P_LCD_DC = 0x16,   /* PB6 */
    P_TOUCH_CS = 0x17, /* PB7 */
    P_TOUCH_IRQ = 0x18,/* PB8 */
    P_I2C_SCL = 0x1A, P_I2C_SDA = 0x1B, /* PB10/PB11: MS5351M at 0x60 */
    P_JOG_RIGHT = 0x1D,/* PB13: jog right, low when pressed */
    P_GAIN_A = 0x1E, P_GAIN_B = 0x1F,   /* PB14/PB15: IF gain, low = less */
};

/* main.c */
extern volatile uint32_t ms_ticks;
void delay_cycles(uint32_t n);
void delay_us(uint32_t us);
void wdt_feed(void);
int  clock_on_tcxo(void);
int  clock_use_tcxo(void);
void enter_bootloader(void);
int  jog_pressed(void);
int  jog_left(void);
void service(void);        /* USB and console, called from long loops */

/* gpio.c */
void pin_cfg(uint8_t pp, uint32_t cfg);
void pin_out(uint8_t pp, int level);    /* push-pull output, 2 MHz */
void pin_od(uint8_t pp);                /* open-drain output, released */
void pin_set(uint8_t pp, int level);
int  pin_read(uint8_t pp);
void pin_input(uint8_t pp, int pull);   /* pull: -1 down, 0 none, 1 up */
void gpio_init(void);

/* usb.c */
void usb_init(void);
void usb_poll(void);
int  usb_configured(void);
extern volatile uint8_t usb_host_seen;
void con_putc(char c);
void con_puts(const char *s);
void con_write(const char *s, int n);
void con_dec(int32_t v);
void con_udec(uint32_t v);
void con_hex(uint32_t v, int digits);
void con_float(float v, int decimals);
void con_flush(void);
void usb_take_line(void);

/* console.c */
void console_line(char *line);
