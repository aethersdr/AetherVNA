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

## Stage 2c: buses found by the chips' own responses (approved, 2026-11-10)

Pins were driven for these tests (maintainer: "do them all"). Pin map so far:

| Function | Pins | How it was established |
|---|---|---|
| Touch controller HR2046 | SCK PB3, MISO PB4, MOSI PB5, CS PB7 (IRQ PB8, pulled up) | `touch`: TEMP0 789 and TEMP1 967 counts (~0.60 V and ~0.78 V, TEMP1 > TEMP0 as the TSC2046 datasheet says), VBAT 1299 counts (~4.1 V through its /4 input; stock reported 4.07 V), X/Z1 0 with nothing touching |
| Display | SCK PB3, MOSI PB5 (shared with touch), CS PA15, D/C PB6; no read-back (ID reads return 0xFF: MISO not driven by the display) | `lcdred`: MIPI-DCS reset, sleep-out, 16-bit pixels, display-on and a solid fill lit the screen. Sent 0xF800 (red in RGB565) shows **blue**: the controller defaults to BGR order |
| Backlight | **PA5, active high** (board pull-down holds it off) | screen dark with every other candidate driven high or low; lit with PA5 high alone |
| MS5351M | I2C2: SCL PB10, SDA PB11, address 0x60 | stage 2a/2b |
| Jog | LEFT pulls **PB1** (analog, ~2.1 V idle) to 0 V; RIGHT drives **PB13** low; IN changed no GPIO | `buttons` and `jog` with the maintainer pressing |
| Receive ADC | PA0 (ch0), biased near VDDA/2 | stage 1 |

**MAX2871 not found by response.** `synthsearch` tried every (CLK, DATA, LE)
assignment over 28 candidate pins (all GPIOs except PA0, PA11-PA14), writing
register 2 (SHDN set, MUX = D_VDD then D_GND) and register 5 (LD pin high
then low), and watched every pin: none followed. The MAX2871's serial
interface works in shutdown (datasheet), so the likely reading is that
neither its MUX nor its LD pin reaches the MCU, as in the open V2 design
(which waits fixed times instead of reading lock). Finding its bus needs an RF
method: program a candidate wiring for a known output and look for the carrier
with a spectrum analyzer.

**Incidents:**
- With PB10/PB11 among the `synthsearch` candidates, the bit patterns reached
  the MS5351M as I2C writes: its output-disable register went from 0x00 to
  0x07. Harmless (a power cycle restores defaults, stock reprograms it), but an
  unintended write. I2C pins are excluded from future searches.
- **Holding the jog during a reset starts Artery's ROM bootloader**
  (USB 2e3c:df11, "AT32 Bootloader DFU"): the maintainer was pressing the jog
  (left first) when a flash rebooted the unit. Nothing was sent to it; a full
  power cycle (USB unplugged, switch off) returned to our probe. The ROM
  bootloader can erase flash including Zeenko's bootloader: never send it
  commands, and keep hands off the jog during reboots. Holding the jog IN
  while powering on still gives Zeenko's bootloader.

## Stage 2d: MAX2871 by its RF output, first attempt (approved, 2026-11-10)

Setup: LiteVNA port 1 to the tinySA Ultra+ RF input, tinySA at 10 dB
attenuation (the strongest possible source is the MAX2871's +5 dBm), scans
150-420 MHz (450 points, 850 kHz RBW). `synthrf k` clocks the datasheet
start-up sequence (registers 5..0 twice, int-N, R=1, N=100, output A /16 at
+5 dBm) on candidate pair k as (CLK, DATA), pulsing every other candidate high
together as LE and leaving them high (CE, RFOUT_EN if MCU-driven).

- **Carriers appear:** 24 dB over baseline at 172.85 MHz (-56.7 dBm) after
  pair 20 (CLK PA2, DATA PA3) in the search; 30.5 dB at 401.96 MHz
  (-50.5 dBm) after pair 21 (CLK PA2, DATA PA4) in a replay. So our pin
  activity can wake the MAX2871 and its output reaches port 1.
- **Not attributable:** neither pair reproduces it on its own (`rfprog` with
  the same pins and N = 90, 100, 110, 120 shows nothing above the 5-8 dB scan
  noise), and replaying pairs 13-20 did not reproduce the first hit. The
  method is confounded: the "LE group" includes every non-(CLK, DATA)
  candidate, so the real CLK and DATA, if among them, are toggled too and
  shift partial words into the chip; each result depends on earlier tests.
- **Next design:** start from a known chip state (power cycle with the probe
  installed), hold every candidate static, and change one role at a time; or
  record the exact pin history around a hit and bisect it. Find the reference
  frequency first so a correct wiring gives a locked, predictable carrier
  (N x fREF / 16) instead of a free-running VCO.

## Stage 2e: MAX2871 bus found, reference 24 MHz, stimulus switches (2026-11-10)

Clean method (after the confounded attempt in 2d): only the three tested pins
move; every other candidate is held static.

- The carrier in 2d came from programming, not from pin levels: with no words
  sent, no combination of static levels (all high, all low, PA2/PA3/PA4 low)
  produced it. Once loaded, the chip kept it, which is why every later test
  showed it.
- **LE found by turning the carrier off:** with the carrier on, the "off"
  words (register 4 with RFA_EN = 0, register 2 with SHDN = 1) were sent
  through each LE candidate in turn; only **PA1** switched it off. Off via
  LE = PA4 left it on; off via DATA = PA4 left it on; via DATA = PA3 it went off.
- **MAX2871 bus: CLK PA2, DATA PA3, LE PA1.** CE and RFOUT_EN are not MCU pins:
  the carrier appears with every pin but PA1-PA3 floating.
- **Reference 24.000 MHz:** int-N, R = 1, output A / 16: N = 150 gives
  225.0000 MHz and N = 200 gives 300.0000 MHz (tinySA, 3 kHz RBW), so the PLL
  locks and fREF = 24.000 MHz. Below N = 120 the VCO stays at its lower limit
  (172.78 MHz x 16 = 2.765 GHz).
- **Stimulus path (225 MHz tone at port 1, tinySA at 10 dB attenuation):**
  with every other pin floating the tone is -55.3 dBm. **PA6 low: -5.3 dBm**
  (+50 dB: the synthesizer is routed to port 1). **PA8 low: -79.3 dBm**
  (-25 dB). No other candidate changes it, low or high. Receive-side switch
  lines need the receive chain to find.

## Recovery

1. Our firmware's `bootloader` command, or power on with the jog held.
2. Write the stock image (`liteVNA64-260713.bin`) at 0x08004000 with the bench's V2-protocol flash tool, which follows Zeenko's documented DFU registers.
3. Zeenko's bootloader (0x08000000–0x08003FFF) and the option bytes are never
   written by anything here. Artery's ROM bootloader (BOOT0) is not used.

## Next

- Stage 1 (inputs only): done, results above.
- Stage 2a (I2C scan): done, MS5351M at 0x60 on PB10/PB11.
- Stage 2c: touch, display, backlight and jog mapped; MAX2871 not found by
  MUX/LD response.
- Stage 2e: MAX2871 on PA2/PA3/PA1, 24 MHz reference, PA6/PA8 steer the
  stimulus. Next: the MS5351M's role (low-band stimulus or LO), the receive
  path into PA0 and its switch lines, then sweeps.
