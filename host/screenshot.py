#!/usr/bin/env python3
"""Save the AetherVNA screen as a PNG, from `lcd capture` over the console.

The panel cannot be read back, so the firmware redraws the screen and sends
every window it draws ("W x0 y0 x1 y1", then "cccc*n" runs of RGB565 pixels in
drawing order, then "E"). This rebuilds the image from that. Standard
library only.

    python3 host/screenshot.py screen.png [--port /dev/serial/by-id/...]

SPDX-License-Identifier: GPL-3.0-or-later"""
import argparse, glob, os, select, struct, sys, time, tty, zlib

PROMPT = b"ch> "


def find_port():
    p = sorted(glob.glob("/dev/serial/by-id/usb-AetherSDR_*VNA*-if00"))
    if not p:
        sys.exit("no AetherVNA console")
    return p[0]


def capture(port, timeout=60.0):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    try:
        while select.select([fd], [], [], 0.1)[0] and os.read(fd, 65536):
            pass
        os.write(fd, b"lcd capture\r")
        out, end = b"", time.time() + timeout
        while not out.endswith(PROMPT):
            if time.time() > end:
                sys.exit("no prompt after %.0f s (%d bytes)" % (timeout, len(out)))
            if select.select([fd], [], [], 0.2)[0]:
                out += os.read(fd, 65536)
        return out.decode(errors="replace")
    finally:
        os.close(fd)


def decode(text):
    if "capture 480 320" not in text:
        sys.exit("unexpected reply: %r" % text[:200])
    w, h = 480, 320
    img = [0] * (w * h)
    win, pos = None, 0
    for line in text.replace("\r", "").split("\n"):
        line = line.strip()
        if not line or line == "end" or line.startswith("capture") or line == "ch>":
            continue
        if line.startswith("W "):
            x0, y0, x1, y1 = map(int, line[2:].split())
            win, pos = (x0, y0, x1 - x0 + 1, y1 - y0 + 1), 0
            continue
        if line == "E":
            win = None
            continue
        if win is None:
            continue
        x0, y0, ww, wh = win
        for tok in line.split():
            c, n = tok.split("*")
            c, n = int(c, 16), int(n)
            for _ in range(n):
                if pos < ww * wh:
                    x, y = x0 + pos % ww, y0 + pos // ww
                    if 0 <= x < w and 0 <= y < h:
                        img[y * w + x] = c
                pos += 1
    return w, h, img


def png(path, w, h, img):
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        for c in img[y * w:(y + 1) * w]:
            r, g, b = (c >> 11) & 31, (c >> 5) & 63, c & 31
            raw += bytes(((r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)))

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    data = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) \
        + chunk(b"IDAT", zlib.compress(bytes(raw), 9)) + chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(data)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("out")
    ap.add_argument("--port")
    a = ap.parse_args()
    t = time.time()
    text = capture(a.port or find_port())
    w, h, img = decode(text)
    png(a.out, w, h, img)
    print("%s: %dx%d, %d bytes of capture in %.1f s" % (a.out, w, h, len(text), time.time() - t))


if __name__ == "__main__":
    main()
