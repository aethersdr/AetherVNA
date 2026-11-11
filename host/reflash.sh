#!/bin/sh
# Rebuild and reflash the AetherVNA firmware through its own bootloader command.
# SPDX-License-Identifier: GPL-3.0-or-later
# FLASH_PY: a flash tool speaking Zeenko's V2 DFU protocol (path to its .py).
set -e
here=$(dirname "$0")
: "${FLASH_PY:?set FLASH_PY to a V2-protocol flash tool}"
make -C "$here/../fw" >/dev/null
python3 "$here/con.py" bootloader
sleep 4
python3 "$FLASH_PY" --no-wait "$here/../fw/aethervna.bin" | tail -1
sleep 5
python3 "$here/con.py" version
