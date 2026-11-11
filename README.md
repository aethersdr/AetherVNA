LiteVNAAI: open, API-driven firmware for the LiteVNA
====================================================

**Work in progress.** An independent, clean-room firmware for the LiteVNA 64
vector network analyzer, maintained by the [AetherSDR](https://github.com/aethersdr/AetherSDR)
project. It is not affiliated with Zeenko, the LiteVNA's maker, or with the
authors of its stock firmware.

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
| 8: display UI (test card, S11/S21 log-magnitude traces, jog marker, status), starts at power-on | Built; orientation and colours not yet confirmed by eye |
| Later: saved settings and calibration in flash, touch input, Smith chart, open/short level check above 1 GHz | Planned |

## Build and flash

```sh
make -C fw                         # arm-none-eabi-gcc; fw/litevnaai.bin
# unit in its bootloader: `bootloader` on our console, or by hand (NOTES.md, Recovery)
# then write fw/litevnaai.bin at 0x08004000 with a V2-protocol flasher;
# host/reflash.sh does both (set FLASH_PY to the flasher)
```

`probe/` is the hardware-discovery firmware the board map came from.

## Console API

USB CDC serial (`/dev/serial/by-id/usb-AetherSDR_LiteVNAAI_*`), plain text,
`ch> ` prompt, one fact per line; `help` lists everything. The measurement
commands follow NanoVNA-D so existing tools can drive it:

| Command | Does |
|---|---|
| `scan <start> <stop> [points] [outmask]` | one sweep; outmask 1 freq, 2 S11, 4 S21, 8 uncalibrated |
| `sweep`, `data [0\|1]`, `frequencies`, `pause`, `resume` | as NanoVNA-D |
| `cal open\|short\|load\|thru\|done\|on\|off\|reset` | one-path SOLT on the device (RAM only for now) |
| `cal get <std\|terms>`, `cal put <std> <i> <4 values>` | read standards and error terms; restore a saved calibration |
| `flags` | per point: tuned frequency, reference level, ok/clip/lowref/range |
| `gain [auto\|0..3]`, `avg [n]`, `bracket [0\|1]`, `settle`, `tsettle` | measurement settings |
| `info`, `version`, `vbat`, `jog`, `touch` | identity, state, battery, inputs |
| `lcd [init\|test\|ui\|off\|madctl <hex>\|inv 0\|1\|marker <i>\|status]`, `backlight` | display |
| `raw <freq>`, `tune`, `ms`, `mx`, `pin`, `clk` | diagnostics |
| `bootloader`, `reset` | refused while the jog is pressed |

With the screen running, host-driven sweeps are slower (it redraws between
commands); `lcd off` or `pause` gives the full speed.

## Licence

GPL-3.0-or-later ([`LICENSE`](LICENSE)).
