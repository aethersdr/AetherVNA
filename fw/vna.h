/* AetherVNA firmware: synthesizers, measurement, sweep and calibration.
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdint.h>

#define FREQ_MIN     50000u
#define FREQ_MAX     6300000000ull
#define BAND_SPLIT   100000000u          /* MS5351M below, MAX2871 pair from here */
#define POINTS_MAX   401
#define IF_HZ        12000u
#define SEG_N        2000                /* 41 IF cycles at 24 MHz / 41 */
#define IF_STEP      41                  /* (41 * i) mod 2000 is the IF phase index */

typedef struct { float re, im; } cpx;

typedef struct {
    uint64_t start, stop;
    uint16_t points;
    uint16_t settle;                     /* samples discarded after each switch */
    uint8_t  avg;                        /* captures per point, or AVG_AUTO (by frequency) */
    uint8_t  gain;                       /* IF gain: GAIN_AUTO or a fixed code 0..3 */
    uint8_t  bracket;                    /* 1: second reference after port 2, interpolated */
    uint8_t  retune;                     /* 1: retune before every averaged capture */
    uint8_t  txpower;                    /* stimulus level 0-3: MS5351M 2/4/6/8 mA, MAX2871 -4/-1/+2/+5 dBm */
    uint32_t split;                      /* Hz: MS5351M below, MAX2871 from here (50-112.5 MHz; 100 MHz default) */
} sweep_cfg_t;
#define SPLIT_MIN 50000000u
#define SPLIT_MAX 112500000u                 /* the MS5351M's top with even dividers >= 8 on a 900 MHz PLL */
extern int32_t tcxo_ppb;                 /* TCXO error: words are computed for f / (1 + ppb), frequencies reported as tuned */
#define GAIN_AUTO 0xFF
#define AVG_AUTO 0
int  avg_for(uint64_t f);
int  gain_for(uint64_t f);               /* code: bit0 PB14 (x3), bit1 PB15 (x1.9); both x8 */

/* Per point: raw ratios (port / reference), the frequency actually tuned, flags. */
enum { PF_CLIP = 1, PF_LOWREF = 2, PF_RANGE = 4 };
typedef struct {
    cpx s11, s21;
    uint64_t f_tuned;                    /* (after the floats: 32 bytes a point, not 40) */
    float ref;                           /* |reference| in ADC counts */
    uint8_t flags;
} point_t;

extern sweep_cfg_t sweep_cfg;
extern point_t pts[POINTS_MAX];
extern volatile uint8_t sweep_running;  /* continuous sweeping for the display */
extern uint32_t sweep_count;

void vna_init(void);
void vna_background(void);
void vna_restart(void);
void vna_set_txpower(int p);             /* 0-3; the synthesizers are reprogrammed on the next tune */

uint64_t sweep_freq(int i);
int  tune(uint64_t f, uint64_t *tuned);
extern uint8_t hb_used;                  /* the MAX2871s ran since power-on */
extern uint32_t hb_last_ms;              /* ms_ticks when they last ran */
int  vna_band(void);                     /* -1 off, 0 low, 1 high */  /* 0 ok, <0 out of range */
void synth_all_off(void);
int  measure_point(int i, uint64_t f);   /* tunes, captures, stores pts[i] */
int  sweep_once(void);                   /* returns points measured */

/* low level, for diagnostics */
int  ms_write(uint8_t reg, const uint8_t *v, int n);
int  ms_read(uint8_t reg, uint8_t *v, int n);
void mx_write(uint8_t le, uint32_t w);
float mcu_sensor_mv(void);
int  capture3(int settle, uint16_t *buf, int *n_out, uint8_t *clip, cpx out[4]);
void dft3(const uint16_t *buf, int settle, cpx out[4]);
extern uint16_t adc_buf[];
extern uint32_t capture_cycles, capture_seq;          /* of the last capture, first to last sample */
float vbat_volts(void);
void vna_adc_init(void);
uint16_t adc_single(int ch);              /* e.g. 9 = PB1, the jog ladder */

/* calibration (one-path SOLT; ideal standards) */
enum { CAL_OPEN, CAL_SHORT, CAL_LOAD, CAL_THRU, CAL_N, CAL_ISOLN = CAL_N };
#define CAL_ISO_BIT (1u << CAL_ISOLN)            /* isolation measured on its own (both ports terminated) */
extern uint8_t cal_enhanced;
typedef struct { float c[4], od, l[4], sd, lr, ll, ld, td; } calkit_t;   /* fF.., ps, pH.., ps, ohm, pH, ps, ps */
extern calkit_t calkit;              /* used by cal_solve; all zero = ideal */             /* enhanced response (port 1 match) in S21, default 1 */
extern uint8_t cal_have;                 /* bit per standard measured */
extern uint8_t cal_on;                   /* apply when valid */
int  cal_valid(void);
int  cal_measure(int which);
int  cal_solve(void);
void cal_reset(void);
int  cal_range(uint64_t *start, uint64_t *stop, int *points);   /* the sweep the standards were measured on */
void cal_apply(int i, cpx *s11, cpx *s21);
int  cal_std_get(int which, int i, cpx *s11, cpx *s21);   /* raw standard as measured */
int  cal_std_put(int which, int i, cpx s11, cpx s21);     /* load one (for the current sweep) */
int  cal_terms_get(int i, cpx t[6]);                      /* e00 e11 de e30 e22 e10e32 */

/* complex helpers */
static inline cpx c_mk(float r, float i) { cpx z = { r, i }; return z; }
static inline cpx c_add(cpx a, cpx b) { return c_mk(a.re + b.re, a.im + b.im); }
static inline cpx c_sub(cpx a, cpx b) { return c_mk(a.re - b.re, a.im - b.im); }
static inline cpx c_mul(cpx a, cpx b) { return c_mk(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re); }
static inline cpx c_div(cpx a, cpx b) { float d = b.re * b.re + b.im * b.im; return c_mk((a.re * b.re + a.im * b.im) / d, (a.im * b.re - a.re * b.im) / d); }
static inline float c_abs2(cpx a) { return a.re * a.re + a.im * a.im; }
float c_abs(cpx a);
float c_db(cpx a);
float c_deg(cpx a);
