/* LiteVNAAI firmware: synthesizers, coherent capture, sweep and calibration.
 * Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Written from: Silicon Labs AN619 (Si5351 registers, which the MS5351M
 * follows), the MAX2871 datasheet (19-6604), the AT32F403A reference manual,
 * and the board map measured with the probe firmware (NOTES.md).
 *
 * Every frequency word is computed in integers so that the LO is exactly
 * IF_HZ above the stimulus in units of the shared 24 MHz TCXO; with the MCU
 * also clocked from the TCXO, the IF is exactly 41 cycles per 2000 ADC samples
 * (ADC at 24 MHz / 41) and the detector needs no frequency estimate. */
#include "hw.h"
#include "vna.h"

sweep_cfg_t sweep_cfg = { 50000ull, 55050000ull, 101, 200, AVG_AUTO, GAIN_AUTO, 0, 1 }   /* HF: never wakes the MAX2871s */;
point_t pts[POINTS_MAX];
volatile uint8_t sweep_running;
uint32_t sweep_count;

/* ---------- maths ---------- */
float c_abs(cpx a)
{
    float x = c_abs2(a);
    if (x <= 0) return 0;
    float r = x > 1 ? x : 1;                       /* Newton square root */
    for (int i = 0; i < 30; i++) r = 0.5f * (r + x / r);
    return r;
}
static float ln_f(float x)                         /* natural log, x > 0 */
{
    int e = 0;
    while (x > 2) { x *= 0.5f; e++; }
    while (x < 1) { x *= 2; e--; }
    float y = (x - 1) / (x + 1), y2 = y * y, s = 0, t = y;
    for (int k = 1; k < 30; k += 2) { s += t / k; t *= y2; }
    return 2 * s + e * 0.69314718f;
}
float c_db(cpx a) { float x = c_abs2(a); return x > 0 ? 10.0f * ln_f(x) / 2.30258509f : -999.0f; }
static float atan_01(float r)                      /* 0 <= r <= 1, error ~1e-7 */
{
    const float t15 = 0.26794919f, s3 = 1.7320508f;  /* tan(15 deg), sqrt(3) */
    float off = 0;
    if (r > t15) { r = (r * s3 - 1) / (s3 + r); off = 0.52359878f; }
    float r2 = r * r;
    return off + r * (1 - r2 * (1.0f / 3 - r2 * (1.0f / 5 - r2 * (1.0f / 7 - r2 * (1.0f / 9 - r2 / 11)))));
}
float c_deg(cpx a)
{
    float x = a.re, y = a.im, ax = x < 0 ? -x : x, ay = y < 0 ? -y : y, ang;
    if (ax == 0 && ay == 0) return 0;
    ang = ax >= ay ? atan_01(ay / ax) : 1.5707963f - atan_01(ax / ay);
    if (x < 0) ang = 3.14159265f - ang;
    if (y < 0) ang = -ang;
    return ang * 57.2957795f;
}

/* cos(2 pi k / 2000), built once by rotation in double precision */
static float cos_t[SEG_N];
static void table_init(void)
{
    double c = 1, s = 0, cd = 0.99999506520185821, sd = 0.0031415874858795635;  /* cos/sin(2 pi / 2000) */
    for (int k = 0; k < SEG_N; k++) {
        cos_t[k] = (float)c;
        double nc = c * cd - s * sd, ns = s * cd + c * sd; c = nc; s = ns;
        if ((k & 255) == 255) { double m = c * c + s * s; m = (3 - m) * 0.5; c *= m; s *= m; }  /* renormalise */
    }
}

/* ---------- MS5351M on I2C2 pins (bit-banged, open drain) ---------- */
#define I2C_HALF() delay_cycles(60)                /* ~400 kHz at 48 MHz */
static void od(uint8_t pp, int high) { pin_set(pp, high); }
static void scl_high(void) { od(P_I2C_SCL, 1); for (int t = 0; t < 1000 && !pin_read(P_I2C_SCL); t++) delay_cycles(48); }
static void i2c_start(void) { od(P_I2C_SDA, 1); scl_high(); I2C_HALF(); od(P_I2C_SDA, 0); I2C_HALF(); od(P_I2C_SCL, 0); I2C_HALF(); }
static void i2c_stop(void)  { od(P_I2C_SDA, 0); I2C_HALF(); scl_high(); I2C_HALF(); od(P_I2C_SDA, 1); I2C_HALF(); }
static int i2c_bit(int b) { od(P_I2C_SDA, b); I2C_HALF(); scl_high(); I2C_HALF(); int r = pin_read(P_I2C_SDA); od(P_I2C_SCL, 0); I2C_HALF(); return r; }
static int i2c_byte(uint8_t b) { for (int i = 7; i >= 0; i--) i2c_bit(b >> i & 1); return !i2c_bit(1); }
static uint8_t i2c_rbyte(int ack) { uint8_t v = 0; for (int i = 0; i < 8; i++) v = (v << 1) | i2c_bit(1); i2c_bit(!ack); return v; }

int ms_write(uint8_t reg, const uint8_t *v, int n)
{
    i2c_start();
    int ok = i2c_byte(0x60 << 1) && i2c_byte(reg);
    for (int i = 0; ok && i < n; i++) ok = i2c_byte(v[i]);
    i2c_stop();
    return ok;
}
int ms_read(uint8_t reg, uint8_t *v, int n)
{
    i2c_start();
    if (!i2c_byte(0x60 << 1) || !i2c_byte(reg)) { i2c_stop(); return 0; }
    i2c_start();
    if (!i2c_byte((0x60 << 1) | 1)) { i2c_stop(); return 0; }
    for (int i = 0; i < n; i++) v[i] = i2c_rbyte(i < n - 1);
    i2c_stop();
    return 1;
}
static int ms_w1(uint8_t reg, uint8_t v) { return ms_write(reg, &v, 1); }

/* AN619 parameter words: a + b/c */
static void ms_params(uint8_t *o, uint32_t a, uint32_t b, uint32_t c, uint8_t rdiv_log2)
{
    uint32_t f = (uint32_t)(((uint64_t)128 * b) / c);
    uint32_t p1 = 128 * a + f - 512, p2 = 128 * b - c * f, p3 = c;
    o[0] = p3 >> 8; o[1] = p3; o[2] = (rdiv_log2 << 4) | ((p1 >> 16) & 3); o[3] = p1 >> 8; o[4] = p1;
    o[5] = ((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F); o[6] = p2 >> 8; o[7] = p2;
}

#define MS_C 1000000u
static uint32_t ms_last_d, ms_last_r;
static void ms_outputs_off(void) { ms_w1(3, 0xFF); uint8_t off[3] = { 0x80, 0x80, 0x80 }; ms_write(16, off, 3); ms_last_d = 0; }
static void ms_outputs_on(void)
{
    uint8_t ctl[3] = { 0x6C, 0x80, 0x4C };          /* CLK0: int MS, PLL B, own MS, 2 mA (LO); CLK1 off; CLK2: PLL A (stimulus) */
    ms_write(16, ctl, 3);
    ms_w1(3, 0xFA);                                 /* CLK0 and CLK2 enabled */
}

/* Stimulus f on CLK2 (PLL A), LO f + IF on CLK0 (PLL B). Returns 0 or -1. */
static int ms_tune(uint64_t f, uint64_t *tuned)
{
    uint32_t r = 1, d = 0, rl = 0;
    for (rl = 0, r = 1; rl <= 7; rl++, r <<= 1) {
        d = (uint32_t)(900000000ull / (f * r)); d &= ~1u;
        if (d > 2048) continue;
        if (d >= 8 && f * r * d >= 600000000ull) break;
    }
    if (rl > 7 || d < 8) return -1;
    uint64_t u = (f * r * d + 12) / 24;             /* PLL multiplier in units of 1e-6 */
    uint64_t ulo = u + 500ull * d * r;               /* exactly IF_HZ above, in TCXO units */
    uint8_t w[8];
    ms_params(w, (uint32_t)(u / MS_C), (uint32_t)(u % MS_C), MS_C, 0);   ms_write(26, w, 8);
    ms_params(w, (uint32_t)(ulo / MS_C), (uint32_t)(ulo % MS_C), MS_C, 0); ms_write(34, w, 8);
    if (d != ms_last_d || r != ms_last_r) {
        ms_params(w, d, 0, 1, rl); ms_write(42, w, 8);   /* MS0 (LO) */
        ms_write(58, w, 8);                              /* MS2 (stimulus) */
        ms_last_d = d; ms_last_r = r;
    }
    ms_w1(177, 0xA0);                                /* reset both PLLs */
    *tuned = (u * 24 + (uint64_t)r * d / 2) / ((uint64_t)r * d);
    return 0;
}

/* ---------- MAX2871 pair: CLK PA2, DATA PA3, LE PA1 (stimulus) / PA4 (LO) ---------- */
void mx_write(uint8_t le, uint32_t w)
{
    pin_set(le, 0); pin_set(P_SYN_CLK, 0);
    for (int i = 31; i >= 0; i--) {
        pin_set(P_SYN_DATA, w >> i & 1); delay_cycles(4);
        pin_set(P_SYN_CLK, 1); delay_cycles(4); pin_set(P_SYN_CLK, 0);
    }
    delay_cycles(4); pin_set(le, 1); delay_cycles(4); pin_set(le, 0);
}
#define MX_M 4000u
static uint32_t mx_r4(uint32_t diva, int on)
{   /* reserved 011, BS = 480 (VAS clock 50 kHz), FB = fundamental, DIVA, RFA_EN, APWR = 0 */
    return (3u << 29) | (1u << 24) | (1u << 23) | (diva << 20) | (0xE0u << 12) | ((on ? 1u : 0u) << 5) | 4u;
}
static const uint32_t MX_R2 = (1u << 14) | (7u << 9) | (1u << 6) | 2u;   /* R = 1, CP = 7, PDP + */
static const uint32_t MX_R1 = (1u << 29) | (1u << 15) | (MX_M << 3) | 1u; /* CPL = 01, P = 1, M */
static uint32_t mx_last_diva = 99;
static void mx_off(void)
{
    for (int k = 0; k < 2; k++) {
        uint8_t le = k ? P_LE_LO : P_LE_STIM;
        mx_write(le, mx_r4(0, 0));
        mx_write(le, MX_R2 | (1u << 5));            /* SHDN */
    }
    mx_last_diva = 99;
}
static void mx_init(void)
{
    for (int k = 0; k < 2; k++) {
        uint8_t le = k ? P_LE_LO : P_LE_STIM;
        mx_write(le, 0x00400005u); mx_write(le, mx_r4(0, 1)); mx_write(le, 0x0000000Bu);
        mx_write(le, MX_R2); mx_write(le, MX_R1);
    }
    mx_last_diva = 99;
}
static int mx_tune(uint64_t f, uint64_t *tuned)
{
    uint32_t diva = 0;
    while (diva < 7 && (f << diva) < 3000000000ull) diva++;
    uint64_t vco = f << diva;
    uint64_t u = (vco + 3000) / 6000;               /* VCO in 6 kHz units (24 MHz / M) */
    uint64_t ulo = u + (2ull << diva);              /* exactly IF_HZ above at the output */
    if (diva != mx_last_diva) {
        mx_write(P_LE_STIM, mx_r4(diva, 1)); mx_write(P_LE_LO, mx_r4(diva, 1));
        mx_last_diva = diva;
    }
    mx_write(P_LE_STIM, (uint32_t)((u / MX_M) << 15) | (uint32_t)((u % MX_M) << 3));
    mx_write(P_LE_LO, (uint32_t)((ulo / MX_M) << 15) | (uint32_t)((ulo % MX_M) << 3));
    *tuned = (u * 6000) >> diva;
    return 0;
}

/* ---------- band and tuning ---------- */
static int band = -1;                               /* 0 low (MS5351M), 1 high (MAX2871) */
int vna_band(void) { return band; }
void synth_all_off(void) { if (band == 1) hb_last_ms = ms_ticks; ms_outputs_off(); mx_off(); band = -1; }
uint32_t tune_settle_us = 0;                        /* extra delay after each retune (console: tsettle) */
/* Measured on the bench unit (NOTES.md): after its PLL reset the MS5351M needs
 * ~3.5 ms to repeat and ~6 ms to settle within 0.001 dB; the MAX2871 pair
 * 1 ms for 0.01 deg; a band change 20 ms. */
#define SETTLE_LOW_US  6000                 /* MS5351M: repeat <0.01 dB from 3.5 ms; 6 ms leaves <0.001 dB mean bias */
#define SETTLE_HIGH_US 1000                /* 300 us left 0.04 deg mean phase bias, 1 ms 0.01 */
#define SETTLE_BAND_US 20000               /* 10 ms left 0.0012 dB on the first point */
/* IF gain by frequency, measured with the thru (NOTES.md): keeps the largest of
 * the three IF segments near 1100-1400 counts, against clipping near 2000.
 * The ratio does not change with gain (within 0.03 dB), so calibration holds. */
/* Captures per point (each after a fresh retune by default), where the noise
 * is: measured sd of S21 on the thru at one capture is 0.001-0.0015 dB below
 * 1.5 GHz, 0.0033 dB at 1.5-3 GHz and 0.0057 dB above; x2 and x4 bring the
 * upper bands to about 0.002 dB. MAX2871 dividers 2-4 (187.5 MHz-1.5 GHz)
 * get x2 for the near-integer synthesizer states, which only a retune
 * re-draws (NOTES.md). */
int avg_for(uint64_t f)
{
    if (sweep_cfg.avg != AVG_AUTO) return sweep_cfg.avg;
    if (f < 187500000ull) return 1;
    if (f < 3000000000ull) return 2;
    return 4;
}
int gain_for(uint64_t f)
{
    if (sweep_cfg.gain != GAIN_AUTO) return sweep_cfg.gain & 3;
    if (f < BAND_SPLIT) return 0;                   /* x1 */
    if (f < 2800000000ull) return 2;                /* x1.9 */
    if (f < 5300000000ull) return 1;                /* x3 */
    return 3;                                       /* x8 */
}
static void gain_set(int g) { pin_set(P_GAIN_A, g & 1); pin_set(P_GAIN_B, g >> 1 & 1); }
/* The MAX2871 pair heats the board: after 3 min of high-band sweeping the raw
 * low band reads up to 0.048 dB high below 1.5 MHz, recovering over 10 min or
 * more (NOTES.md, stage 10). Tools read how long it has been off from `info`. */
uint8_t hb_used;
uint32_t hb_last_ms;
int tune(uint64_t f, uint64_t *tuned)
{
    if (f < FREQ_MIN || f > FREQ_MAX) return -1;
    int b = f >= BAND_SPLIT;
    if (b) { hb_used = 1; hb_last_ms = ms_ticks; }
    if (b != band) {
        if (b) { ms_outputs_off(); mx_init(); pin_set(P_BAND, 1); }
        else   { mx_off(); ms_outputs_on(); pin_set(P_BAND, 0); }
        if (!b && band == 1) hb_last_ms = ms_ticks;   /* the moment it shut down */
        band = b;
        delay_us(SETTLE_BAND_US);
    }
    gain_set(gain_for(f));
    int r = b ? mx_tune(f, tuned) : ms_tune(f, tuned);
    delay_us((b ? SETTLE_HIGH_US : SETTLE_LOW_US) + tune_settle_us);
    return r;
}

/* ---------- coherent 3-segment capture ---------- */
uint16_t adc_buf[4 * (SEG_N + 300)];
static uint8_t adc_up;
void vna_adc_init(void);
static void adc_init(void)
{
    ADC_SPT1 = 3u * 0x00249249u; ADC_SPT2 = 3u * 0x09249249u;   /* 28.5 cycles: 24 MHz / 41 */
    ADC_CTRL2 = (7u << 17) | (1u << 20) | 1u;
    delay_us(100);
    ADC_CTRL2 |= 1u << 3; for (uint32_t t = 0; (ADC_CTRL2 & 8) && t < 1000000; t++) ;
    ADC_CTRL2 |= 1u << 2; for (uint32_t t = 0; (ADC_CTRL2 & 4) && t < 1000000; t++) ;
    ADC_OSQ1 = 0; ADC_OSQ3 = 0;                     /* PA0, channel 0 */
    adc_up = 1;
}
void vna_adc_init(void) { adc_init(); }

/* The MCU's internal temperature sensor, ADC1 channel 16, as a proxy for the
 * board's temperature (the low band drifts with it). Enabled only for the
 * reading so captures run as before, sampled for 239.5 ADC cycles, 64
 * conversions averaged. Returned as the sensor's voltage in mV for VDDA =
 * 3.3 V: its changes track temperature at a few mV per degree; the
 * datasheet's absolute figures were not to hand, so no degrees are claimed. */
float mcu_sensor_mv(void)
{
    if (!adc_up) adc_init();
    uint32_t spt = ADC_SPT1;
    ADC_SPT1 = (spt & ~(7u << 18)) | (7u << 18);
    ADC_CTRL2 |= 1u << 23;
    delay_us(100);
    uint32_t s = 0;
    for (int i = 0; i < 64; i++) s += adc_single(16);
    ADC_CTRL2 &= ~(1u << 23);
    ADC_SPT1 = spt;
    return s * (3300.0f / 4096.0f / 64.0f);
}
/* One conversion of another channel between captures (capture3 sets channel 0 again). */
uint16_t adc_single(int ch)
{
    if (!adc_up) { adc_init(); }
    ADC_CTRL2 &= ~(1u << 1);
    ADC_OSQ3 = (uint32_t)ch;
    (void)ADC_ODT;
    ADC_CTRL2 |= 1u << 22;
    for (uint32_t t = 0; !(ADC_STS & 2) && t < 100000; t++) ;
    uint16_t v = ADC_ODT & 0xFFF;
    ADC_OSQ3 = 0;
    return v;
}
/* Reference (PA6 high, PA7 low), port 1 (PA6 low, PA7 low), port 2 (PA6 low, PA7 high),
 * and with bracketing the reference again. Pins switch between conversions at
 * fixed sample indices.
 *
 * The Hann-windowed DFT at the IF (see dft3) runs inside the loop, in the time
 * the ADC takes for each conversion (82 core cycles; this work is about 20),
 * so a capture costs only its samples. The nominal mid-scale 2048 is taken
 * off instead of the segment mean: a periodic Hann window has no response to
 * DC at the IF bin, so the result is the same (checked against dft3 by `raw`).
 * The samples are still stored for `raw`. */
uint32_t capture_cycles;
int capture3(int settle, uint16_t *buf, int *n_out, uint8_t *clip, cpx out[4])
{
    static const uint8_t st[4][2] = { {1, 0}, {0, 0}, {0, 1}, {1, 0} };
    int nseg = sweep_cfg.bracket ? 4 : 3, seg = SEG_N + settle, total = nseg * seg;
    if (settle < 0 || settle > 300) return -1;
    pin_set(P_STIM_PORT, st[0][0]); pin_set(P_RX_SEL, st[0][1]);
    ADC_OSQ3 = 0;
    ADC_CTRL2 |= 1u << 1; ADC_CTRL2 |= 1u << 22;     /* continuous, start */
    uint8_t c = 0;
    int k = 0, j = 0;                                /* segment, sample within it */
    uint32_t ph = 0, phs = 1500;                     /* IF phase index of sample i, and minus 90 deg */
    float re = 0, im = 0;
    uint32_t t0 = 0;
    for (int i = 0; i < total; i++, j++) {
        if (j == seg) {
            out[k] = c_mk(re * (2.0f / 1000.0f), im * (2.0f / 1000.0f));   /* window sum 1000 */
            re = im = 0; j = 0; k++;
            pin_set(P_STIM_PORT, st[k][0]); pin_set(P_RX_SEL, st[k][1]);
        }
        while (!(ADC_STS & 2)) ;
        uint16_t v = ADC_ODT & 0xFFF;
        if (!i) t0 = DWT_CYCCNT;
        buf[i] = v;
        if (j >= settle) {
            int n = j - settle;
            if (v == 0 || v == 0xFFF) c = 1;
            float x = ((int)v - 2048) * (0.5f - 0.5f * cos_t[n]);
            re += x * cos_t[ph];
            im -= x * cos_t[phs];                    /* sin(t) = cos(t - pi/2) */
        }
        ph += IF_STEP; if (ph >= SEG_N) ph -= SEG_N;
        phs += IF_STEP; if (phs >= SEG_N) phs -= SEG_N;
    }
    capture_cycles = DWT_CYCCNT - t0;                /* first to last sample: (total - 1) * 82 if none missed */
    out[k] = c_mk(re * (2.0f / 1000.0f), im * (2.0f / 1000.0f));
    ADC_CTRL2 &= ~(1u << 1);
    while (ADC_CTRL2 & (1u << 22)) ;
    (void)ADC_ODT;
    pin_set(P_STIM_PORT, 1); pin_set(P_RX_SEL, 0);
    *n_out = total; *clip = c;
    return 0;
}

/* Hann-windowed DFT of each segment at exactly 41 cycles per 2000 samples,
 * phase referred to sample 0 of the capture. Scaled to the IF amplitude.
 * The reference for the inline DFT in capture3 (`raw` prints both). */
void dft3(const uint16_t *buf, int settle, cpx out[4])
{
    int seg = SEG_N + settle;
    for (int k = 0; k < (sweep_cfg.bracket ? 4 : 3); k++) {
        int s0 = k * seg + settle;
        float mean = 0;
        for (int n = 0; n < SEG_N; n++) mean += buf[s0 + n];
        mean /= SEG_N;
        float re = 0, im = 0;
        uint32_t ph = (uint32_t)(((uint64_t)IF_STEP * (uint32_t)s0) % SEG_N);
        for (int n = 0; n < SEG_N; n++) {
            float x = (buf[s0 + n] - mean) * (0.5f - 0.5f * cos_t[n]);
            re += x * cos_t[ph];
            im -= x * cos_t[(ph + 1500) % SEG_N];   /* sin(t) = cos(t - pi/2) */
            ph += IF_STEP; if (ph >= SEG_N) ph -= SEG_N;
        }
        out[k] = c_mk(re * 2.0f / 1000.0f, im * 2.0f / 1000.0f);   /* sum of the window = 1000 */
    }
}

/* ---------- sweep ---------- */
uint64_t sweep_freq(int i)
{
    sweep_cfg_t *c = &sweep_cfg;
    if (c->points < 2) return c->start;
    return c->start + (c->stop - c->start) * (uint64_t)i / (c->points - 1);
}

int measure_point(int i, uint64_t f)
{
    point_t *p = &pts[i];
    uint64_t tuned = 0;
    p->flags = 0;
    if (tune(f, &tuned) < 0) { p->flags = PF_RANGE; p->f_tuned = f; p->s11 = p->s21 = c_mk(0, 0); p->ref = 0; return -1; }
    cpx a11 = c_mk(0, 0), a21 = c_mk(0, 0); float aref = 0; uint8_t fl = 0;
    int avg = avg_for(f);
    for (int k = 0; k < avg; k++) {
        int n; uint8_t clip; cpx z[4];
        if (k && sweep_cfg.retune) tune(f, &tuned);   /* a fresh synthesizer state per capture */
        capture3(sweep_cfg.settle, adc_buf, &n, &clip, z);
        float r2 = c_abs2(z[0]);
        if (clip) fl |= PF_CLIP;
        if (r2 < 400.0f) { fl |= PF_LOWREF; r2 = r2 > 0 ? r2 : 1; }   /* below 20 counts */
        cpx r1 = z[0], r2c = z[0];
        if (sweep_cfg.bracket) {                              /* reference interpolated to each port's time */
            r1  = c_add(c_mk(z[0].re * (2.0f / 3), z[0].im * (2.0f / 3)), c_mk(z[3].re / 3, z[3].im / 3));
            r2c = c_add(c_mk(z[0].re / 3, z[0].im / 3), c_mk(z[3].re * (2.0f / 3), z[3].im * (2.0f / 3)));
        }
        a11 = c_add(a11, c_div(z[1], r1)); a21 = c_add(a21, c_div(z[2], r2c));
        aref += c_abs(z[0]);
        service();
    }
    p->s11 = c_mk(a11.re / avg, a11.im / avg); p->s21 = c_mk(a21.re / avg, a21.im / avg);
    p->ref = aref / avg; p->f_tuned = tuned; p->flags = fl;
    return 0;
}

int sweep_once(void)
{
    if (!clock_on_tcxo() && !clock_use_tcxo()) return -1;
    adc_init();
    int n = sweep_cfg.points;
    for (int i = 0; i < n; i++) { measure_point(i, sweep_freq(i)); service(); }
    synth_all_off();                     /* idle cool: nothing runs between host sweeps */
    sweep_count++;
    return n;
}

/* ---------- calibration: one-path SOLT, ideal standards ---------- */
uint8_t cal_have, cal_on;
static sweep_cfg_t cal_cfg;
static cpx std_open[POINTS_MAX], std_short[POINTS_MAX], std_load[POINTS_MAX], std_iso[POINTS_MAX], std_t11[POINTS_MAX], std_t21[POINTS_MAX];
typedef struct { cpx e00, e11, de, e30, e22, e10e32; } terms_t;
static terms_t terms[POINTS_MAX];
static uint8_t terms_ok;

static int same_cfg(const sweep_cfg_t *a, const sweep_cfg_t *b)
{
    return a->start == b->start && a->stop == b->stop && a->points == b->points && a->gain == b->gain && a->bracket == b->bracket;   /* averaging changes noise, not the expected value */
}
int cal_valid(void)
{
    return terms_ok && same_cfg(&cal_cfg, &sweep_cfg);
}
void cal_reset(void) { cal_have = 0; terms_ok = 0; cal_on = 0; }
int cal_measure(int which)
{
    if (which < 0 || which >= CAL_N) return -1;
    if (cal_have && !same_cfg(&cal_cfg, &sweep_cfg)) cal_have = 0;
    cal_cfg = sweep_cfg; terms_ok = 0;
    if (sweep_once() < 0) return -2;
    for (int i = 0; i < sweep_cfg.points; i++) {
        switch (which) {
        case CAL_OPEN:  std_open[i] = pts[i].s11; break;
        case CAL_SHORT: std_short[i] = pts[i].s11; break;
        case CAL_LOAD:  std_load[i] = pts[i].s11; std_iso[i] = pts[i].s21; break;
        case CAL_THRU:  std_t11[i] = pts[i].s11; std_t21[i] = pts[i].s21; break;
        }
    }
    cal_have |= 1u << which;
    return 0;
}
/* e00 + (G*Gm)*e11 - G*de = Gm for G = +1, -1, 0 (open, short, load) */
static int solve3(cpx a[3][4], cpx x[3])
{
    for (int c = 0; c < 3; c++) {
        int p = c;
        for (int r = c + 1; r < 3; r++) if (c_abs2(a[r][c]) > c_abs2(a[p][c])) p = r;
        if (c_abs2(a[p][c]) < 1e-30f) return -1;
        for (int k = 0; k < 4; k++) { cpx t = a[c][k]; a[c][k] = a[p][k]; a[p][k] = t; }
        for (int r = 0; r < 3; r++) if (r != c) {
            cpx f = c_div(a[r][c], a[c][c]);
            for (int k = 0; k < 4; k++) a[r][k] = c_sub(a[r][k], c_mul(f, a[c][k]));
        }
    }
    for (int i = 0; i < 3; i++) x[i] = c_div(a[i][3], a[i][i]);
    return 0;
}
static cpx corr11(cpx gm, const terms_t *t) { return c_div(c_sub(gm, t->e00), c_sub(c_mul(gm, t->e11), t->de)); }
int cal_solve(void)
{
    if ((cal_have & 15) != 15) return -1;
    for (int i = 0; i < cal_cfg.points; i++) {
        cpx mo = std_open[i], ms = std_short[i], ml = std_load[i];
        cpx a[3][4] = {
            { c_mk(1, 0), mo, c_mk(-1, 0), mo },
            { c_mk(1, 0), c_mk(-ms.re, -ms.im), c_mk(1, 0), ms },
            { c_mk(1, 0), c_mk(0, 0), c_mk(0, 0), ml },
        };
        cpx x[3];
        if (solve3(a, x) < 0) return -2;
        terms_t *t = &terms[i];
        t->e00 = x[0]; t->e11 = x[1]; t->de = x[2];
        t->e30 = std_iso[i];
        t->e22 = corr11(std_t11[i], t);
        t->e10e32 = c_mul(c_sub(std_t21[i], t->e30), c_sub(c_mk(1, 0), c_mul(t->e11, t->e22)));
        service();
    }
    terms_ok = 1; cal_on = 1;
    return 0;
}
void cal_apply(int i, cpx *s11, cpx *s21)
{
    const terms_t *t = &terms[i];
    cpx g = corr11(*s11, t);
    *s21 = c_mul(c_div(c_sub(*s21, t->e30), t->e10e32), c_sub(c_mk(1, 0), c_mul(t->e11, g)));
    *s11 = g;
}

int cal_std_get(int which, int i, cpx *s11, cpx *s21)
{
    if (which < 0 || which >= CAL_N || !(cal_have & (1u << which)) || i < 0 || i >= cal_cfg.points) return -1;
    switch (which) {
    case CAL_OPEN:  *s11 = std_open[i]; *s21 = c_mk(0, 0); break;
    case CAL_SHORT: *s11 = std_short[i]; *s21 = c_mk(0, 0); break;
    case CAL_LOAD:  *s11 = std_load[i]; *s21 = std_iso[i]; break;
    default:        *s11 = std_t11[i]; *s21 = std_t21[i]; break;
    }
    return 0;
}
/* Loading point 0 of a standard starts it afresh for the current sweep, so a
 * host can restore a calibration it saved; "cal done" then solves as usual. */
int cal_std_put(int which, int i, cpx s11, cpx s21)
{
    if (which < 0 || which >= CAL_N || i < 0 || i >= sweep_cfg.points) return -1;
    if (cal_have && !same_cfg(&cal_cfg, &sweep_cfg)) cal_have = 0;
    cal_cfg = sweep_cfg; terms_ok = 0;
    switch (which) {
    case CAL_OPEN:  std_open[i] = s11; break;
    case CAL_SHORT: std_short[i] = s11; break;
    case CAL_LOAD:  std_load[i] = s11; std_iso[i] = s21; break;
    default:        std_t11[i] = s11; std_t21[i] = s21; break;
    }
    if (i == sweep_cfg.points - 1) cal_have |= 1u << which;   /* complete once the last point is in */
    return 0;
}
int cal_terms_get(int i, cpx t[6])
{
    if (!terms_ok || i < 0 || i >= cal_cfg.points) return -1;
    const terms_t *e = &terms[i];
    t[0] = e->e00; t[1] = e->e11; t[2] = e->de; t[3] = e->e30; t[4] = e->e22; t[5] = e->e10e32;
    return 0;
}

/* ---------- background ---------- */
static int bg_i;
void vna_restart(void) { bg_i = 0; }   /* the background sweep starts over (new sweep settings) */
void vna_init(void)
{
    table_init();
    pin_out(P_GAIN_A, 0); pin_out(P_GAIN_B, 0);
    synth_all_off();
}
void vna_background(void)
{
    if (!sweep_running) { bg_i = 0; return; }
    if (!clock_on_tcxo() && !clock_use_tcxo()) { sweep_running = 0; return; }
    if (bg_i == 0) adc_init();
    measure_point(bg_i, sweep_freq(bg_i));
    if (++bg_i >= sweep_cfg.points) { bg_i = 0; sweep_count++; }
}
