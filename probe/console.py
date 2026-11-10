#!/usr/bin/env python3
"""Send one command to the LiteVNA probe console and print the reply.
SPDX-License-Identifier: GPL-3.0-or-later"""
import glob, os, select, sys, time, tty
port = sorted(glob.glob("/dev/serial/by-id/usb-AetherSDR_LiteVNA_probe_*-if00"))[0]
fd = os.open(port, os.O_RDWR | os.O_NOCTTY); tty.setraw(fd)
os.write(fd, (" ".join(sys.argv[1:]) + "\r").encode())
out, end = b"", time.time() + 2
while time.time() < end and not out.endswith(b"> "):
    if select.select([fd], [], [], 0.1)[0]:
        try: out += os.read(fd, 4096)
        except OSError: break
os.close(fd)
print(out.decode(errors="replace").rstrip("> ").rstrip())
