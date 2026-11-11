"""LiteVNA first-sweep prototype: host-side control of the probe firmware.

Copyright (C) 2026 the AetherSDR project. SPDX-License-Identifier: GPL-3.0-or-later

Prototype only (needs numpy). Drives the probe console: MS5351M words (AN619),
MAX2871 words (datasheet 19-6604), pin states, and the meas3 capture, and turns
each capture into raw S11/S21 (port / reference). The on-device sweep replaces
this. Calibration math is the AetherSDR bench's SOLT (calibrate.py, found through
the LITEVNA_CAL_LIB environment variable).
"""
import os, sys, time, glob, select, tty
def probe(cmd, t=10):
    port = sorted(glob.glob("/dev/serial/by-id/usb-AetherSDR_LiteVNA_probe_*-if00"))[0]
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY); tty.setraw(fd)
    os.write(fd, (cmd + "\r").encode()); out, end = b"", time.time() + t
    while time.time() < end and not out.endswith(b"> "):
        if select.select([fd], [], [], 0.1)[0]: out += os.read(fd, 1 << 16)
    os.close(fd); return out.decode(errors="replace").strip("> \r\n")
def p123(x):                      # AN619 integer: P1 = 128*x - 512, P2 = 0, P3 = 1
    p1 = 128 * x - 512
    return [0, 1, (p1 >> 16) & 3, (p1 >> 8) & 255, p1 & 255, 0, 0, 0]
def ms_setup(a, divs):
    """PLLA = fref*a; CLKn = PLLA/divs[n] (integer); outputs not in divs powered down."""
    assert all(probe("msw 3 255") == "ok" for _ in [0])              # all outputs off while configuring
    assert probe("msw 26 " + " ".join(map(str, p123(a)))) == "ok"   # PLLA
    for n in range(3):
        if n in divs:
            b = p123(divs[n]); assert probe("msw %d %s" % (42 + 8 * n, " ".join(map(str, b)))) == "ok"
            assert probe("msw %d %d" % (16 + n, 0x4F)) == "ok"       # int mode, PLLA, own MS, 8 mA
        else:
            assert probe("msw %d %d" % (16 + n, 0x80)) == "ok"       # powered down
    assert probe("msw 177 160") == "ok"                              # reset PLLA and PLLB
    mask = 0xFF & ~sum(1 << n for n in divs)
    assert probe("msw 3 %d" % mask) == "ok"
def ms_off():
    probe("msw 3 255"); probe("msw 16 128 128 128")
import numpy as np
def frac(a, b, c):                 # AN619 fractional a + b/c
    f = (128 * b) // c
    p1, p2, p3 = 128 * a + f - 512, 128 * b - c * f, c
    return [(p3 >> 8) & 255, p3 & 255, (p1 >> 16) & 3, (p1 >> 8) & 255, p1 & 255, ((p3 >> 12) & 0xF0) | ((p2 >> 16) & 15), (p2 >> 8) & 255, p2 & 255]
def capture(n=8192):
    out = probe("adc0 %d" % n, 60); lines = out.splitlines()
    rate = int(lines[0].split()[1])
    v = np.array([int(l[i:i+3], 16) for l in lines[1:] if l and l != "end" for i in range(0, len(l), 3)], float)
    return rate, v
def spectrum(rate, v, fmax=200e3):
    w = np.hanning(len(v)); s = np.abs(np.fft.rfft((v - v.mean()) * w)) / (w.sum() / 2)
    f = np.fft.rfftfreq(len(v), 1 / rate); m = f <= fmax
    return f[m], s[m]
def top(rate, v, k=3):
    f, s = spectrum(rate, v); i = np.argsort(s)[::-1][:k]
    return ", ".join("%.2f kHz %.1f counts" % (f[j] / 1e3, s[j]) for j in sorted(i))
def setup_rx(lo_clk, stim=True, lo=True, b=1, c=25):
    """PLLA 768 MHz: CLK2 = 9.600 MHz stimulus. PLLB 24*(32+b/c): LO on lo_clk /80."""
    probe("msw 3 255")
    assert probe("msw 26 " + " ".join(map(str, p123(32)))) == "ok"
    assert probe("msw 34 " + " ".join(map(str, frac(32, b, c)))) == "ok"
    for n in range(3):
        if n == 2 and stim:
            assert probe("msw 58 " + " ".join(map(str, p123(80)))) == "ok"; probe("msw 18 %d" % 0x4F)
        elif n == lo_clk and lo:
            assert probe("msw %d %s" % (42 + 8 * n, " ".join(map(str, p123(80))))) == "ok"; probe("msw %d %d" % (16 + n, 0x6F))  # PLLB
        else:
            probe("msw %d 128" % (16 + n))
    probe("msw 177 160")
    on = (4 if stim else 0) | ((1 << lo_clk) if lo else 0)
    probe("msw 3 %d" % (0xFF & ~on))
def ms_clk0(a, b, c, d, drive=3):
    """CLK0 = 24 MHz * (a + b/c) / d on PLL B; CLK1/CLK2 off."""
    probe("msw 3 255")
    assert probe("msw 34 " + " ".join(map(str, frac(a, b, c)))) == "ok"
    assert probe("msw 42 " + " ".join(map(str, p123(d)))) == "ok"
    probe("msw 16 %d" % (0x6C | drive)); probe("msw 17 128"); probe("msw 18 128")
    probe("msw 177 160"); probe("msw 3 254")
def iq_at(rate, v, f0):
    n = np.arange(len(v)); return abs(np.sum((v - v.mean()) * np.exp(-2j * np.pi * f0 * n / rate)) * 2 / len(v))
def tones(rate, v, k=4, fmax=800e3):
    f, s = spectrum(rate, v, fmax); m = f > 1e3; f, s = f[m], s[m]
    i = np.argsort(s)[::-1][:k]
    return ", ".join("%.2f kHz %.1f" % (f[j] / 1e3, s[j]) for j in sorted(i))
def mx_set(freq_hz, diva=4, out_a=True, out_b=False, apwr=3):
    """MAX2871 (datasheet 19-6604): fREF 24 MHz, R=1, fPFD 24 MHz; fVCO = f * 2**diva, N + F/M."""
    fvco = freq_hz * (1 << diva); n = int(fvco // 24e6); M = 4000; F = round((fvco / 24e6 - n) * M)
    if F == M: n, F = n + 1, 0
    r5 = 0x00400005
    r4 = (0b011 << 29) | (1 << 24) | (1 << 23) | (diva << 20) | (0xE0 << 12) | (out_b << 8) | (out_a << 5) | (apwr << 3) | (apwr << 6) | 4
    r3 = 0x0000000B
    r2 = (0 << 26) | (1 << 14) | (7 << 9) | (1 << 6) | 2          # R=1, CP=7, PDP positive
    r1 = (1 << 29) | (1 << 15) | (M << 3) | 1                        # CPL=01, P=1, M
    r0 = (n << 15) | (F << 3)                                        # frac mode
    words = " ".join("%08x" % w for w in (r5, r4, r3, r2, r1, r0))
    probe("mx " + words); time.sleep(0.03); probe("mx " + words)
    return 24e6 * (n + F / M) / (1 << diva)
def mx_words(freq_hz, diva=4, out_a=True, out_b=False, apwr=3):
    fvco = freq_hz * (1 << diva); n = int(fvco // 24e6); M = 4000; F = round((fvco / 24e6 - n) * M)
    if F == M: n, F = n + 1, 0
    r = [0x00400005,
         (0b011 << 29) | (1 << 24) | (1 << 23) | (diva << 20) | (0xE0 << 12) | (out_b << 8) | (out_a << 5) | (apwr << 3) | (apwr << 6) | 4,
         0x0000000B, (1 << 14) | (7 << 9) | (1 << 6) | 2, (1 << 29) | (1 << 15) | (M << 3) | 1, (n << 15) | (F << 3)]
    return " ".join("%08x" % w for w in r), 24e6 * (n + F / M) / (1 << diva)
def mx_on(le, freq_hz, **kw):
    w, f = mx_words(freq_hz, **kw)
    cmd = ("mx " if le == 1 else "mxl %d " % le) + w
    probe(cmd); time.sleep(0.03); probe(cmd); return f
def capture2(n=4096):
    out = probe("adc2 %d" % n, 60); lines = out.splitlines()
    hdr = lines[0].split(); rate = int(hdr[1]); lag = int(hdr[3]) * 1e-9
    v = np.array([int(l[i:i+3], 16) for l in lines[1:] if l and l != "end" for i in range(0, len(l), 3)], float)
    return rate, lag, v[0::2], v[1::2]
def iqz(rate, v, f0, t0=0.0):
    v = v[8:]; n = np.arange(len(v)) + 8
    return np.sum((v - v.mean()) * np.exp(-2j * np.pi * f0 * (n / rate + t0))) * 2 / len(v)
def meas3(n=2048, settle=200, f_if=12000.0):
    out = probe("meas3 %d %d" % (n, settle), 60); lines = out.splitlines()
    h = lines[0].split(); rate = int(h[1]); seg = int(h[3]); settle = int(h[5])
    v = np.array([int(l[i:i+3], 16) for l in lines[1:] if l and l != "end" for i in range(0, len(l), 3)], float)
    idx = [np.arange(k * seg + settle, (k + 1) * seg) for k in range(3)]
    r = v[idx[0]] - v[idx[0]].mean()
    def mag(fc): return abs(np.sum(r * np.exp(-2j * np.pi * fc * idx[0])))
    fc = f_if / rate; step = 0.01 * fc                       # cycles per sample: golden-section refine
    lo, hi = fc - 5 * step, fc + 5 * step
    for _ in range(60):
        m1, m2 = lo + (hi - lo) * 0.382, lo + (hi - lo) * 0.618
        if mag(m1) < mag(m2): lo = m1
        else: hi = m2
    fc = (lo + hi) / 2
    z = [np.sum((v[i] - v[i].mean()) * np.exp(-2j * np.pi * fc * i)) * 2 / len(i) for i in idx]
    return {"ref": z[0], "p1": z[1], "p2": z[2], "f_if_hz": fc * rate}
from fractions import Fraction
def ms_lf(f, f_if=12000, drive=0):
    """MS5351M: CLK2 = f (PLL A), CLK0 = f + IF (PLL B), same even divider d; VCO 600-900 MHz."""
    d = int(900e6 // f); d -= d % 2
    if d < 6 or d > 2048: raise ValueError("f out of the MS5351M range used here")
    def pll(target):
        x = Fraction(int(round(target * d)), 24_000_000).limit_denominator(1048575)
        a = x.numerator // x.denominator; return a, x.numerator - a * x.denominator, x.denominator
    pa, pb = pll(f), pll(f + f_if)
    probe("msw 3 255")
    assert probe("msw 26 " + " ".join(map(str, frac(*pa)))) == "ok"
    assert probe("msw 34 " + " ".join(map(str, frac(*pb)))) == "ok"
    assert probe("msw 42 " + " ".join(map(str, p123(d)))) == "ok"
    assert probe("msw 58 " + " ".join(map(str, p123(d)))) == "ok"
    probe("msw 16 %d 128 %d" % (0x6C | drive, 0x4C | drive))      # CLK0: PLL B LO; CLK1 off; CLK2: PLL A stimulus
    probe("msw 177 160"); probe("msw 3 250")
def mx_diva(f):
    k = 0
    while f * (1 << k) < 3e9 and k < 7: k += 1
    return k
def point(f, f_if=12000):
    if f < 100e6:
        probe("pin 8 0"); ms_lf(f, f_if)
    else:
        probe("pin 8 1"); k = mx_diva(f)
        mx_on(1, f, diva=k, apwr=0); mx_on(4, f + f_if, diva=k, apwr=0)
    m = meas3(f_if=f_if)
    return m["p1"] / m["ref"], m["p2"] / m["ref"], abs(m["ref"])
sys.path.insert(0, os.environ.get("LITEVNA_CAL_LIB", "."))  # SOLT math (calibrate.py, litevna.py)
import json as _json
FREQS = [float(x) for x in np.unique(np.round(np.logspace(6, 9, 61) / 6000) * 6000) if abs(x - 100e6) > 1e5]
CALDIR = "ourcal"
def sweep(avg=2):
    probe("pin 30 0"); probe("pin 31 0")
    s11, s21 = [], []
    for f in FREQS:
        a = b = 0
        for _ in range(avg):
            x, y, _r = point(f); a += x; b += y
        s11.append(a / avg); s21.append(b / avg)
    return s11, s21
def save_std(name):
    os.makedirs(CALDIR, exist_ok=True)
    s11, s21 = sweep()
    _json.dump({"freq_hz": FREQS, "s11": [[z.real, z.imag] for z in s11], "s21": [[z.real, z.imag] for z in s21]}, open("%s/%s.json" % (CALDIR, name), "w"))
    return s11, s21
