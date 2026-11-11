AetherVNA: open, API-driven firmware for the LiteVNA
====================================================

*Named LiteVNAAI until 2026-11-11; the old repository URL redirects here.*

**Work in progress.** An independent, clean-room firmware for the LiteVNA 64
vector network analyzer, maintained by the [AetherSDR](https://github.com/aethersdr/AetherSDR)
project. It is not affiliated with Zeenko, the LiteVNA's maker, or with the
authors of its stock firmware.

## Hardware required

**AetherVNA runs only on a LiteVNA, model LiteVNA 64** (Zeenko), and has
been tested on one unit, hardware revision 2 (AT32F403A, MS5351M, two
MAX2871, 24 MHz TCXO, 480x320 display). It is not for NanoVNA V2 or other
LiteVNA models: the pin map, synthesizers and front end were measured on
this board (NOTES.md) and are not checked against any other. It installs
behind the LiteVNA 64's own bootloader, which it never writes, so the stock
firmware can be put back at any time.

## Why

AetherSDR's RF test bench is driven by software agents, so every instrument has
to give everything (readings, settings, identity) through an API. The
LiteVNA's stock firmware is closed and shows some of that only on its screen.
This project writes open firmware for the same hardware that does not.

## How

- Written from public sources only: Artery's AT32F403A reference manual and
  datasheet, the synthesizer and switch datasheets, Zeenko's published USB
  protocol, public teardowns, and the licensed parts of the open
  [NanoVNA V2 firmware](https://github.com/nanovna-v2/NanoVNA2-firmware).
  Nothing is taken from the stock firmware image.
- The hardware is learned with our own probe firmware, which runs behind the
  unit's existing bootloader and never writes it, so the stock firmware can be
  restored at any time.

See [`NOTES.md`](NOTES.md) for decisions, sources, findings and recovery.

## Status

| Stage | State |
|---|---|
| 0: our firmware runs, USB console, chip identity, return to bootloader | Done on a LiteVNA 64 |
| 1: classify every pin, inputs only | Done (see NOTES.md) |
| 2a: I2C scan (open-drain): MS5351M at 0x60 on PB10/PB11 | Done |
| 2b: MS5351M registers read at power-on (reference present, PLLs lock) | Done |
| 2c: touch, display (write-only), backlight PA5, jog mapped; MAX2871 needs an RF method | Done |
| 2d/2e: MAX2871 on CLK PA2, DATA PA3, LE PA1; 24 MHz reference; stimulus switches PA6, PA8 | Done |
| 3: low band: CLK2 stimulus, CLK0 LO, 24 MHz reference, PA7 channel select, PB14/PB15 IF gain; IF seen at PA0 | Done |
| 4: high band: second MAX2871 (LE PA4) is the LO; measurement chain mapped | Done |
| 5: first calibrated sweep from our own firmware, 1 MHz-1 GHz (thru +/-0.09 dB LF, +/-0.24 dB HF) | Done |
| 6: repeatability: TCXO-clocked MCU, coherent sampling, Hann window; calibrated thru +/-0.006 dB, short +/-0.0046 dB (stock: +/-0.015 / +/-0.009) | Done |
| 7: firmware proper (`fw/`): sweep, console API and SOLT calibration on the device, 50 kHz-6.3 GHz; 101 points 1 MHz-1 GHz in 1.9 s repeat within 0.005 dB | Done |
| 8: display UI (test card, S11/S21 log-magnitude traces, jog marker, status), starts at power-on | Done; seen working by the maintainer |
| 9: settling, averaging and detector tuned for accuracy: 101 points 1 MHz-1 GHz in 2.6 s within 0.003 dB, 401 points to 6.3 GHz in 15.5 s within 0.03 dB | Done |
| 10: HF focus: default sweep 50 kHz-55 MHz, synthesizers off between host sweeps, `info` reports how long the high band has been off (its heat moves HF readings) | Done |
| 11: first full calibration and checks on the HF plan; USB replies ending on a full packet end with a zero-length packet (a scan could hang) | Done |
| 12: stock-style UI: four traces (log mag, phase, SWR, Smith, linear, R, X, \|Z\|), Smith chart over the grid, four markers with search, menus (display, marker, sweep with keypad and HF band presets, on-device calibration), touch and jog input, all mirrored on the console; `lcd capture` and `host/screenshot.py` for checking the screen over USB | Done: driven over the console, touch checked by hand |
| 13: microSD slot found (SPI1, chip select PB9); SPI-mode driver and FAT32 read/write checked on a test card | Done |
| 14: save/recall slots 0-6 (settings, display, calibration) and config (touch map) in flash; slot 0 at power-on | Done |
| 15: SD CARD menu: S1P/S2P Touchstone, BMP screenshots, calibration save and LOAD | Done |
| Later: the remaining stock menus (NOTES.md, stage 12), open/short level check above 1 GHz | Planned |

## Build and flash

```sh
make -C fw                         # arm-none-eabi-gcc; fw/aethervna.bin
# unit in its bootloader: `bootloader` on our console, or by hand (NOTES.md, Recovery)
# then write fw/aethervna.bin at 0x08004000 with a V2-protocol flasher;
# host/reflash.sh does both (set FLASH_PY to the flasher)
```

`probe/` is the hardware-discovery firmware the board map came from.

## Console API

USB CDC serial (`/dev/serial/by-id/usb-AetherSDR_AetherVNA_*`), plain text,
`ch> ` prompt, one fact per line; `help` lists everything. The measurement
commands follow NanoVNA-D so existing tools can drive it:

| Command | Does |
|---|---|
| `scan <start> <stop> [points] [outmask]` | one sweep; outmask 1 freq, 2 S11, 4 S21, 8 uncalibrated |
| `sweep`, `data [0\|1]`, `frequencies`, `pause`, `resume` | as NanoVNA-D |
| `cal open\|short\|load\|thru\|done\|on\|off\|reset` | one-path SOLT on the device (RAM only for now) |
| `cal get <std\|terms>`, `cal put <std> <i> <4 values>` | read standards and error terms; restore a saved calibration |
| `flags` | per point: tuned frequency, reference level, ok/clip/lowref/range |
| `gain [auto\|0..3]`, `avg [auto\|n]`, `retune [0\|1]`, `bracket [0\|1]`, `settle`, `tsettle` | measurement settings |
| `info`, `version`, `vbat`, `temp`, `jog`, `touch` | identity, state, battery, MCU temperature sensor (mV), inputs |
| `trace [1-4 [on\|off\|s11\|s21\|<format>\|scale <v>\|ref <v>\|refpos <d>\|auto\|active]]` | the screen's traces; formats `logmag phase swr smith linear r x z` |
| `marker [1-4 [on\|off\|active\|<index>\|minswr\|max\|min]]` | the screen's markers |
| `ui [state\|tap <x> <y>\|jog l\|r\|L\|R\|rr\|redraw]` | menu state; taps and jog clicks/holds as if by hand |
| `touch [cal\|map [ax bx ay by swap]]` | raw reading, on-screen touch calibration, the mapping |
| `lcd [init\|test\|ui\|off\|capture\|madctl <hex>\|inv 0\|1\|marker <i>\|status]`, `backlight` | display; `capture` streams the redrawn screen (`host/screenshot.py` makes a PNG) |
| `save <0-6>`, `recall <0-6>`, `config [save\|erase]` | settings and calibrations in flash (slot 0 loads at power-on); `config erase` before restoring stock |
| `sd status\|probe\|cs\|init\|mount\|ls\|verify\|write`, `sd save s1p\|s2p\|cal\|bmp [fixed]`, `sd load\|cat\|hex\|rm <file>` | microSD (SPI1, chip select PB9), FAT32, Touchstone and calibration files; `fat`, `fatrange`, `patsec`, `unlink`, `fsinfo` for repair |
| `raw <freq>`, `tune`, `ms`, `mx`, `pin`, `clk` | diagnostics |
| `bootloader`, `reset` | refused while the jog is pressed |

On the device: tap the screen (or double-click the jog right) for the menu;
with a menu open, jog clicks move the highlight, holding right selects and
holding left goes back. Touch works from power-on with a mapping measured
on the bench unit; TOUCH CAL (main menu) redoes it, until power-off.

With the screen running, host-driven sweeps are slower (it redraws between
commands); `lcd off` or `pause` gives the full speed.

## Licence

GPL-3.0-or-later ([`LICENSE`](LICENSE)). The on-screen font is rendered from
Noto Sans, Copyright 2022 The Noto Project Authors, under the SIL Open Font
License 1.1 ([`fonts/OFL.txt`](fonts/OFL.txt)); `fw/fonts.c` is generated by
`host/mkfont.py`.
