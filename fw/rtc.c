/* AetherVNA firmware: the real-time clock on the 32.768 kHz crystal (LEXT).
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Written from the AT32F403A reference manual (battery-powered domain:
 * PWC_CTRL BPWEN, CRM_BPDC LEXTEN/LEXTSTBL/RTCSEL/RTCEN; RTC: a 32-bit
 * second counter with a 20-bit divider, written in configuration mode once
 * the previous write has finished). The probe found crystal pins on
 * PC14/PC15 (NOTES.md, stage 1).
 *
 * The counter holds seconds since 2000-01-01 00:00; backup register 1 says
 * whether a time has been set. The battery-powered domain survives resets;
 * whether it survives power-off depends on what feeds the chip's VBAT pin. */
#include "hw.h"
#include "ui.h"

#define CRM_APB1EN_ REG(0x4002101Cu)
#define CRM_BPDC    REG(0x40021020u)
#define PWC_CTRL    REG(0x40007000u)
#define BPR_DT1     REG(0x40006C04u)                 /* backup data register 1 (16 bits) */
#define RTC_CTRLL   REG(0x40002804u)
#define RTC_DIVH    REG(0x40002808u)
#define RTC_DIVL    REG(0x4000280Cu)
#define RTC_CNTH    REG(0x40002818u)
#define RTC_CNTL    REG(0x4000281Cu)
#define RTC_SET_MARK 0xA55Au

static int8_t rtc_state = -1;                        /* -1 not tried, 0 no crystal, 1 running */

static int wait_bit(volatile uint32_t *r, uint32_t bit, uint32_t ms)
{
    uint32_t t0 = ms_ticks;
    while (!(*r & bit)) { if (ms_ticks - t0 > ms) return -1; service(); }
    return 0;
}
int rtc_start(void)
{
    if (rtc_state >= 0) return rtc_state;
    CRM_APB1EN_ |= (1u << 28) | (1u << 27);          /* power control, backup registers */
    PWC_CTRL |= 1u << 8;                             /* writes to the battery-powered domain allowed */
    if (!(CRM_BPDC & (1u << 15))) {                  /* RTC not running yet: crystal on, RTC on it */
        CRM_BPDC |= 1u;
        if (wait_bit(&CRM_BPDC, 1u << 1, 3000)) { rtc_state = 0; return 0; }
        CRM_BPDC = (CRM_BPDC & ~(3u << 8)) | (1u << 8) | (1u << 15);
        if (wait_bit(&RTC_CTRLL, 1u << 5, 500)) { rtc_state = 0; return 0; }
        RTC_CTRLL |= 1u << 4;                        /* configuration mode */
        RTC_DIVH = 0; RTC_DIVL = 32767;              /* 1 Hz */
        RTC_CTRLL &= ~(1u << 4);
        if (wait_bit(&RTC_CTRLL, 1u << 5, 500)) { rtc_state = 0; return 0; }
    } else if (!(CRM_BPDC & (1u << 1))) { rtc_state = 0; return 0; }
    RTC_CTRLL &= ~(1u << 3);                         /* wait for the registers to resynchronise */
    if (wait_bit(&RTC_CTRLL, 1u << 3, 500)) { rtc_state = 0; return 0; }
    rtc_state = 1;
    return 1;
}
int rtc_is_set(void) { return rtc_start() == 1 && (BPR_DT1 & 0xFFFF) == RTC_SET_MARK; }
uint32_t rtc_get(void)
{
    if (rtc_start() != 1) return 0;
    uint32_t h, l;
    do { h = RTC_CNTH & 0xFFFF; l = RTC_CNTL & 0xFFFF; } while (h != (RTC_CNTH & 0xFFFF));
    return h << 16 | l;
}
int rtc_set(uint32_t s)
{
    if (rtc_start() != 1) return -1;
    if (wait_bit(&RTC_CTRLL, 1u << 5, 500)) return -1;
    RTC_CTRLL |= 1u << 4;
    RTC_CNTH = s >> 16; RTC_CNTL = s & 0xFFFF;
    RTC_CTRLL &= ~(1u << 4);
    if (wait_bit(&RTC_CTRLL, 1u << 5, 500)) return -1;
    BPR_DT1 = RTC_SET_MARK;
    return 0;
}

/* ---------- calendar: seconds since 2000-01-01 <-> date and time ---------- */
static int leap(int y) { return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0; }
static const uint8_t mdays[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
uint32_t rtc_from_date(int y, int mo, int d, int h, int mi, int s)
{
    uint32_t days = 0;
    for (int k = 2000; k < y; k++) days += leap(k) ? 366 : 365;
    for (int k = 1; k < mo; k++) days += mdays[k - 1] + (k == 2 && leap(y));
    days += d - 1;
    return days * 86400u + h * 3600u + mi * 60u + s;
}
void rtc_to_date(uint32_t t, int *y, int *mo, int *d, int *h, int *mi, int *s)
{
    uint32_t days = t / 86400u, r = t % 86400u;
    *h = r / 3600; *mi = r / 60 % 60; *s = r % 60;
    int yy = 2000; while (days >= (uint32_t)(leap(yy) ? 366 : 365)) { days -= leap(yy) ? 366 : 365; yy++; }
    int m = 1; while (days >= (uint32_t)(mdays[m - 1] + (m == 2 && leap(yy)))) { days -= mdays[m - 1] + (m == 2 && leap(yy)); m++; }
    *y = yy; *mo = m; *d = (int)days + 1;
}
/* For FAT directory entries: the date and time now, or 2026-11-11 12:00 if the clock is not set. */
void rtc_fat(uint16_t *date, uint16_t *time)
{
    int y = 2026, mo = 11, d = 11, h = 12, mi = 0, s = 0;
    if (rtc_is_set()) rtc_to_date(rtc_get(), &y, &mo, &d, &h, &mi, &s);
    *date = (uint16_t)(((y - 1980) << 9) | (mo << 5) | d);
    *time = (uint16_t)((h << 11) | (mi << 5) | (s / 2));
}
