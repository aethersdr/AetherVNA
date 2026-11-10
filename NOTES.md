# LiteVNA AI firmware port: working notes

Goal: API-driven, open firmware for the bench LiteVNA 64, so every reading,
setting and identity comes over USB (the stock firmware shows its build only on
screen). Clean-room: written from public datasheets and reference manuals, our
own probe firmware, and the licensed parts of the open NanoVNA V2 firmware.

## Decisions (maintainer, 2026-11-10)

- **Port only.** No source request to Zeenko or DiSlord.
- **V2 firmware as the model, core rewritten.** The open NanoVNA V2 firmware
  (github.com/nanovna-v2/NanoVNA2-firmware, d9c768b) is read as a reference.
  Its licensed files may be used: `main2.cpp` and `bootloader/bootloader.cpp`
  (LGPL-3.0-or-later), `ui.cpp`, `ili9341.cpp` and the fonts (GPL), `fft.*` and
  `xpt2046.*` (MIT). The other 45 files, including the measurement core,
  synthesizer and board code, carry no licence and are **not copied**; nor is
  `mculib` (no licence). Synthesizer and switch programming come from the chip
  datasheets.
- **Hardware discovery only through our own probe firmware**, inputs-only first.
  Not opening the case, not observing the stock firmware's signals, not reading
  register state the closed bootloader or firmware leave behind, never
  disassembling the stock image (Principle IV).
- After each session the stock image is restored (`~/bench-firmware/litevna/`)
  and the thru check rerun: stock v1.4.10 read 0 dB ±0.014 dB to 3 GHz after
  calibration; that is the acceptance bar for the port.

## Hardware (public sources)

From Kerry Wong's teardown of a LiteVNA 62, board "LiteVNA 62-0.3"
(kerrywong.com, 2022-03-22; the 64 differs in display and battery):

| Part | Marking | Role |
|---|---|---|
| MCU | Artery AT32F403ACGT7 (LQFP48) | Cortex-M4F, 1 MB flash |
| Synthesizer | MAX2871 ("2871E", U17) | Above ~140 MHz |
| Clock generator | MS5351M | Below ~140 MHz (Si5351-compatible) |
| RF switch | Maxscend MXD8641 (SP4T) | Path selection (one mixer, switched) |
| Op-amp | Gainsil GS8722 (U16) | IF / ADC driver |
| Touch | HR2046 (U20) | TSC2046/XPT2046-compatible |
| Edge header | UART TX/RX, GND, VCC, 5V, SWDIO, SWCLK, GND, NRST, 3V, BOOT0 | Debug and ROM-bootloader access |

Zeenko's *LiteVNA User Guide* documents the USB protocol (V2 register map,
FIFO format, DFU registers) and says the unit uses one mixer with internal RF
switching for S11 and S21.

## Facts measured with our probe (stage 0, 2026-11-10)

- Runs at 0x08004000 behind Zeenko's bootloader, from the 48 MHz HICK with ACC
  trimming to USB SOF; the internal D+ pull-up enumerates with no GPIO.
- `id`: IDCODE 0x70050346, 1024 KB flash, UID 07e1a417 88650000 0020a179,
  series 0x07 (AT32F403A), **flash access protection off**, USD 0x03fffffc.
- **Bootloader entry from our firmware works:** writing 0xDEADBABE at the top
  of 48 KB or 96 KB of RAM (both are written) and resetting leaves Zeenko's
  bootloader in update mode, as in the open V2 bootloader. Hands-only entry
  (power on with the jog held) also works. So the loop flash → test →
  `bootloader` → flash stock runs from the bench host.

## Recovery

1. Our firmware's `bootloader` command, or power on with the jog held.
2. Write the stock image (`liteVNA64-260713.bin`) at 0x08004000 with the bench's V2-protocol flash tool, which follows Zeenko's documented DFU registers.
3. Zeenko's bootloader (0x08000000–0x08003FFF) and the option bytes are never
   written by anything here. Artery's ROM bootloader (BOOT0) is not used.

## Next

- Stage 1 (inputs only): classify every pin by reading it with pull-up and
  pull-down (driven, pulled, floating), and the ADC channels' idle levels.
  Results go to the maintainer before any pin is driven.
- Then, with approval: I2C scan for the MS5351 (address 0x60), SPI to the
  MAX2871 and the display, the RF switch lines.
