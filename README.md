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
| Later: synthesizers, RF switches, ADC, sweeps, calibration, console API | Planned |

## Build and flash

```sh
make -C probe                      # arm-none-eabi-gcc
# unit in its bootloader: power on with the jog held, or `bootloader` on the probe console
# then write probe/probe.bin at 0x08004000 with a V2-protocol flasher (NOTES.md, Recovery)
```

## Licence

GPL-3.0-or-later ([`LICENSE`](LICENSE)).
