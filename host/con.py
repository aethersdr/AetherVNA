#!/usr/bin/env python3
"""Send one command to the LiteVNAAI console and print the reply.
SPDX-License-Identifier: GPL-3.0-or-later"""
import glob, os, select, sys, time, tty
def port():
    p = sorted(glob.glob("/dev/serial/by-id/usb-AetherSDR_LiteVNA*-if00"))
    if not p: sys.exit("no LiteVNAAI console")
    return p[0]
def cmd(line, timeout=30.0):
    fd = os.open(port(), os.O_RDWR | os.O_NOCTTY); tty.setraw(fd)
    try:
        os.write(fd, (line + "\r").encode())
        out, end = b"", time.time() + timeout
        while time.time() < end and not (out.endswith(b"ch> ") or out.endswith(b"> ")):
            if select.select([fd], [], [], 0.1)[0]:
                try: out += os.read(fd, 1 << 16)
                except OSError: break
        return out.decode(errors="replace").replace("ch> ", "").rstrip()
    finally:
        os.close(fd)
if __name__ == "__main__":
    t = float(os.environ.get("T", "60"))
    print(cmd(" ".join(sys.argv[1:]), t))
