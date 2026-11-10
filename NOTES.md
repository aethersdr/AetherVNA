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
- **External chips are read only from their own power-on state.** The
  MS5351M, MAX2871 and others keep their registers across an MCU reset, so
  after stock has run they hold stock's configuration. Before reading one, the
  unit is power-cycled with our probe installed, so stock never runs first.
  (A first MS5351M read on 2026-11-10 was taken after stock had run; it was
  discarded unrecorded.)
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

## Stage 1 results: every pin as an input (2026-11-10)

Probe stage 1 (`pins`, `activity`, `adc`) on the bench LiteVNA 64. Nothing was
driven: pins were only floating, pulled (internal ~70 kOhm) or analog inputs.
PA11/PA12 (USB) and PA13/PA14 (SWD) were left alone; JTAG was released so
PA15/PB3/PB4 read as GPIO.

**Digital (read floating, with pull-up, with pull-down):**

| Pins | Result | Reading (hypothesis, to be tested) |
|---|---|---|
| PB10, PB11 | held high | Pull-ups: I2C2 SCL/SDA, likely the MS5351M bus |
| PB8 | held high | Pull-up: an interrupt line (the V2 used PB8 for the touch controller's PENIRQ) |
| PA0 | held high (1.5 V, see ADC) | Analog: biased signal into ADC ch0 |
| PA5, PB2 | held low | PB2 is BOOT1; PA5 tied low |
| PC14, PC15 | held low | 32 kHz crystal pins (LEXT) |
| PD0, PD1 | read 0 | Main crystal pins; not GPIO without remap, not interpreted |
| all others | follow the pull | Floating at rest: driven only when the firmware drives them (chip selects, SPI, switch controls, LCD) |

**Activity:** with every pin a floating input, no pin changed in 337 ms
(200,000 samples, 1.7 us apart): nothing on the board clocks or signals the MCU
unprompted.

**Analog (VDDA 3.02 V from VINTRV; each pin charged high, then low, before the
reading):**

| Pin | After pull-up / pull-down | Reading |
|---|---|---|
| PA0 (ch0) | 1606 / 1460 mV | Driven near VDDA/2: op-amp (GS8722) output biased for the ADC; ch0 is the V2's receive channel |
| PA5 (ch5) | 0 / 0 mV | Tied to ground |
| PB0 (ch8) | 1465 / 1738 mV | Held near 1.6 V through a resistance (a divider) |
| PB1 (ch9) | 2908 / 1931 mV | Weakly held near 2 V: consistent with a high-resistance divider on the 3.97 V battery |
| PA1-PA4, PA6, PA7 | follow the charge | Floating or high impedance at rest |

## Stage 2a: I2C scan (approved, 2026-11-10)

`i2cscan`: software I2C on PB10/PB11, open-drain only (lines pulled low or
released, never driven high), ~100 kHz, an address-only read probe per address,
NACKed at once, so no register is written.

| Wiring | Answering |
|---|---|
| SCL = PB10, SDA = PB11 (I2C2's own pins) | **0x60**: the MS5351M's address |
| swapped | none |

So the MS5351M is on I2C2, SCL PB10, SDA PB11, with board pull-ups; nothing
else is on that bus. Both lines read pulled-high again afterwards.

## Stage 2b: MS5351M registers at power-on (approved, read-only, 2026-11-10)

`ms5351`: read-only reads from 0x60 (a one-byte pointer write, then a repeated
start), open-drain only. Read after a power cycle with the probe installed, so
the chip was at its own power-on state. Register meanings from Silicon Labs
AN619 (Si5351), which the MS5351M follows.

| Register | Value | Meaning |
|---|---|---|
| 0 status | 0x11 | SYS_INIT 0 (ready), LOL_A 0 and LOL_B 0 (both PLLs locked), LOS 1 (nothing on CLKIN: normal with a crystal/TCXO reference), REVID 1 |
| 1 sticky | 0xF8 | |
| 2 mask | 0x03 | |
| 3 output disable | 0x00 | all outputs enabled (power-on default) |
| 16-18 CLK0-2 control | 0x00 0x00 0x00 | powered up, reference as source (default) |
| 19-23 CLK3-7 control | 0x80 x5 | powered down: the MS5351M has three outputs |
| 183 crystal load | 0xD2 | AN619 default (10 pF) |
| 187 fanout | 0x02 | |

So the MS5351M's reference is present and both PLLs lock at power-on. With
its outputs passing the reference through, `activity` still saw no MCU pin
change: CLK0-2 go to the RF side, not to the MCU. `pins` matched stage 1.

## Recovery

1. Our firmware's `bootloader` command, or power on with the jog held.
2. Write the stock image (`liteVNA64-260713.bin`) at 0x08004000 with the bench's V2-protocol flash tool, which follows Zeenko's documented DFU registers.
3. Zeenko's bootloader (0x08000000–0x08003FFF) and the option bytes are never
   written by anything here. Artery's ROM bootloader (BOOT0) is not used.

## Next

- Stage 1 (inputs only): done, results above.
- Stage 2a (I2C scan): done, MS5351M at 0x60 on PB10/PB11.
- Next, each with approval: reading the MS5351M's status register; finding
  the MAX2871's SPI lines (it has a readback through MUXOUT); the display and
  touch controller; the RF switch lines.
