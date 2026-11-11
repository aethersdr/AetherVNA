/* LiteVNAAI firmware: GPIO. Copyright (C) 2026 the AetherSDR project.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include "hw.h"

void pin_cfg(uint8_t pp, uint32_t cfg)
{
    int p = pp >> 4, n = pp & 15, sh = 4 * (n & 7);
    GPIO_CFG(p, n) = (GPIO_CFG(p, n) & ~(0xFu << sh)) | (cfg << sh);
}
void pin_set(uint8_t pp, int level)
{
    if (level) GPIO_SCR(pp >> 4) = 1u << (pp & 15); else GPIO_CLR(pp >> 4) = 1u << (pp & 15);
}
void pin_out(uint8_t pp, int level) { pin_set(pp, level); pin_cfg(pp, 0x2); }
void pin_od(uint8_t pp) { pin_set(pp, 1); pin_cfg(pp, 0x6); }
int  pin_read(uint8_t pp) { return (GPIO_IDT(pp >> 4) >> (pp & 15)) & 1; }
void pin_input(uint8_t pp, int pull)
{
    if (!pull) { pin_cfg(pp, 0x4); return; }
    pin_set(pp, pull > 0); pin_cfg(pp, 0x8);
}

/* Every pin to a defined state. Outputs only where the board needs them. */
void gpio_init(void)
{
    CRM_APB2EN |= 1u | (0xFu << 2) | (1u << 9) | (1u << 12);   /* IOMUX, GPIOA-D, ADC1, SPI1 */
    IOMUX_REMAP = (IOMUX_REMAP & ~(7u << 24)) | (2u << 24) | 1u; /* JTAG off (SWD kept), SPI1 on PB3/PB4/PB5 */
    pin_out(P_LE_STIM, 0); pin_out(P_LE_LO, 0);
    pin_out(P_SYN_CLK, 0); pin_out(P_SYN_DATA, 0);
    pin_out(P_STIM_PORT, 1);                 /* stimulus to the reference path until a measurement */
    pin_out(P_RX_SEL, 0);
    pin_out(P_BAND, 0);
    pin_out(P_GAIN_A, 0); pin_out(P_GAIN_B, 0);
    pin_out(P_BACKLIGHT, 0);
    pin_out(P_LCD_CS, 1); pin_out(P_LCD_DC, 1); pin_out(P_TOUCH_CS, 1);
    pin_input(P_TOUCH_IRQ, 1);
    pin_input(P_JOG_RIGHT, 1);
    pin_input(P_JOG_LADDER, 0);
    pin_od(P_I2C_SCL); pin_od(P_I2C_SDA);
    pin_cfg(P_ADC_IF, 0);                    /* analog */
}
