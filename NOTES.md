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

## Stage 3: MS5351M role and the receive path, low band (2026-11-10)

`msw` writes MS5351M registers (AN619 map: integer and fractional PLL and
multisynth words); `adc0 n` captures PA0 at 1.714 MS/s (the first sample is the
converter's start-up value and is dropped). MAX2871 off throughout.

**Stimulus (tinySA on port 1):** PLL A = 24 MHz x 32, CLK0/1/2 = /64, /72, /80.
Only **CLK2** reaches port 1: 9.5999 MHz at -0.5 dBm (1 kHz RBW), so the
**MS5351M reference is 24.000 MHz** (same TCXO as the MAX2871). It needs **PA6
low** (path to port 1; PA6 high: -91 dBm) and **PA8 low** (source select;
PA8 high: -68.5 dBm, and high selects the MAX2871). A released pin keeps its
last level on its own capacitance, which first made PA8 alone look sufficient.

**Receiver:** stimulus CLK2 = 9.600 MHz (PLL A), candidate LO 9.612 MHz on
PLL B (24 x (32 + 1/25), /80): a 12 kHz IF at PA0.

| Condition | IF at PA0 |
|---|---|
| LO = **CLK0**, PA7 low | ~410 counts (port 1 on the tinySA's 50 ohm) |
| LO = CLK1, PA7 low | 6 counts (leakage) |
| PA7 not low, any other single pin | noise (~0.2 counts) |

**CLK0 is the receiver LO.** Channel select **PA7: low = port 1 reflection,
high = port 2 transmission**:

| Port setup | PA7 low (port 1) | PA7 high (port 2) |
|---|---|---|
| port 1 to the tinySA (matched) | 418 counts | - |
| port 1 open | 2576 counts (clipping) | 0.3 counts |
| thru (port 2 cable on port 1) | 319 counts | 2584 counts (clipping, 0..4095) |

**IF gain: PB14, PB15** (the V2's baseband-gain pair): low cuts the IF (PB14
-12.5 dB, PB15 -8.6 dB; both -18 dB). The transmission channel still clips
with both low at full MS5351M drive (8 mA): drive strength or gain must be
chosen per band.

No other pin (PA4, PA9, PA10, PB0, PB2, PB9, PB12, PC13-PC15) changes the
stimulus or the IF in this band; no separate reference channel was found. They
may act only with the MAX2871 (high band), which is next: its LO arrangement
(CLK0 harmonics or otherwise) and the remaining pins.

## Stage 4: high band, a second MAX2871 for the LO (2026-11-10)

New commands: `mx <hex>...` writes raw 32-bit words to the MAX2871 on CLK
PA2 / DATA PA3 / LE PA1; `mxl <le> <hex>...` the same with another LE pin.
Fractional words from the datasheet (fREF 24 MHz, R = 1, fPFD 24 MHz, M = 4000,
band-select divider 480 so the VAS clock stays under 50 kHz). Thru connected.

- **MS5351M harmonics are not the high-band LO.** With the stimulus MAX2871 at
  300.000 MHz, CLK0 or CLK1 at 150.006 / 100.004 / 75.003 MHz (2nd/3rd/4th
  harmonic 12 kHz away) gives at most ~7 counts of 12 kHz IF (CLK0, 3rd).
  Nor is the stimulus MAX2871 an LO for MS5351M harmonics (CLK2 at 100 MHz,
  MAX2871 at 300.012 MHz on output A, B or both, PA8 low or high): nothing.
- **A second MAX2871 is the LO, LE = PA4**, sharing CLK PA2 and DATA PA3. With
  the stimulus at 300.000 MHz (LE PA1), the LO at 300.012 / 300.020 / 299.970
  MHz written via LE PA4 gives a 12 / 20 / 30 kHz IF on the port 2 channel at
  full scale (clipping); the LO shut down via PA4, or the stimulus shut down
  via PA1, removes it; a shutdown word via LE PA9 changes nothing. (The
  teardown photographed one 2871E outside the cans; this one is presumably
  under a can.) Like the open V2 design: two synthesizers, stimulus and LO.
- **Levels:** both MAX2871s at APWR 0 and both gain pins low: 538 counts on the
  port 2 channel (thru), 1.4 counts on port 1 (terminated by the thru).
- **A 16.35 kHz tone** (~20 counts, harmonic at 32.7 kHz) appears on the port 1
  channel whenever PA8 is high, also with both MAX2871s shut down: a board
  spur, probably a switching regulator. IF frequencies should avoid it.
- **No other pin matters:** PA9, PA10, PB0, PB2, PB9, PB12, PC13, PC14, PC15
  change neither channel in either band (the microSD slot is the likely owner of
  some; not checked).

### The measurement chain

| Function | Control |
|---|---|
| Reference | 24.000 MHz TCXO, shared by the MS5351M and both MAX2871s |
| Low-band stimulus | MS5351M CLK2 (PA8 low) |
| Low-band LO | MS5351M CLK0 |
| High-band stimulus | MAX2871 #1, LE PA1 (PA8 high) |
| High-band LO | MAX2871 #2, LE PA4 |
| MAX2871 bus | CLK PA2, DATA PA3 (shared) |
| Stimulus to port 1 | PA6 low |
| Receiver channel | PA7: low = port 1 (reflection), high = port 2 (transmission) |
| IF gain | PB14, PB15 (low = less gain) |
| IF | PA0, ADC channel 0 (biased at VDDA/2); 1.714 MS/s measured |

## Stage 5: first sweep from our own firmware (2026-11-10)

**The reference channel is a switch state:** PA6 high (stimulus cut from port 1)
with PA7 low routes the stimulus into an internal reference path (1361 counts
at 9.6 MHz with the thru; PA6 high + PA7 high gives nothing). PB0 (ADC ch8)
carries the same IF as PA0 at a fixed ratio and phase (1.039 at 12.8 deg on
the port 2 channel), so it is a second tap of the same path, not a reference.

**Phase coherence:** stimulus and LO start at a random relative phase after
each retune, so only port / reference ratios are repeatable, and only if the
three are related in time. `meas3 <n> <settle>` captures reference, port 1
and port 2 as one continuous PA0 run (1.714 MS/s), switching PA6/PA7 at fixed
sample indices; the host fits the IF frequency from the reference segment
(the MCU clock is ~0.1% off the TCXO: the IF reads ~11 985 Hz for 12 000 Hz)
and reads all three phases against sample zero. Across three retunes the raw
ratios then repeat within ~0.3% in magnitude and a few degrees in phase.

**Sweep (host prototype, `host/prototype.py`):** below 100 MHz the MS5351M
(CLK2 stimulus on PLL A, CLK0 LO on PLL B, same even divider, fractional PLLs
for an exact 12 kHz offset, 2 mA drive); from 100 MHz the MAX2871 pair (output
divider keeping the VCO at 3-6 GHz, output power 0); IF gain pins low. 14
points 1 MHz-1 GHz in 1.6 s; raw S21 continuous across the 100 MHz crossover.

**Calibrated with our own sweeps:** open, short, load and thru (60 log-spaced
points, 1 MHz-1 GHz, 2 averages) solved with the bench's SOLT math. Checks:

| Check | Result | Stock firmware, same caps |
|---|---|---|
| Fresh short, corrected | 0 dB +/- 0.33 dB, 180 +/- 4.2 deg | +/- 0.009 dB, +/- 0.06 deg |
| Fresh thru, corrected, 1-100 MHz | within +/- 0.09 dB | within +/- 0.015 dB |
| Fresh thru, corrected, 100 MHz-1 GHz | within +/- 0.24 dB | |
| Fresh thru phase | within +/- 7 deg | within +/- 0.07 deg |

It measures correctly, end to end, on our firmware. Repeatability is the gap.
Likely causes, to work on next: ~1.2 ms of integration per segment (14 IF
cycles) and 2 averages; 200-sample settling after each switch; the MCU timed
from its own oscillator rather than the 24 MHz TCXO; the low-band port 2
segment near clipping. Then: the sweep loop on the device, and the console API.

## Stage 6: repeatability (2026-11-10)

Each cause measured on its own (S21 on the thru, std over 8 captures), then
fixed:

| Step | 10 MHz | 300 MHz |
|---|---|---|
| Stage 5 method (HICK clock, 1.714 MS/s, fitted IF) | 0.0038 dB, 3.2 deg | 0.11 dB, 2.9 deg |
| Uniform sampling: ADC sample time 28.5 cycles, exactly 24 MHz / 41 = 585.4 kS/s (the 1.7 MS/s polling loop missed conversions) | 0.0081 dB, 1.1 deg | 0.025 dB, 1.0 deg |
| **MCU from the 24 MHz TCXO** (`clk hext`: HEXT, PLL x2 = 48 MHz; USB stays on HICK48 + ACC). The IF then fits at 12 000.02 +/- 0.13 Hz: sampling is coherent with the synthesizers | 0.029 dB, 0.48 deg | 0.018 dB, 0.44 deg |
| Exact IF instead of a fit | 0.020 dB, 0.14 deg | 0.021 dB, 0.12 deg |
| **Hann window** (the 16.35 kHz spur and the switching edges leak into a rectangular window) | **0.0013 dB, 0.003 deg** | **0.0015 dB, 0.015 deg** |

**Retuning** then still added ~0.03 dB at 0.8-1 GHz, with the reference and
port 2 amplitudes moving in opposite directions: an extra 12 kHz signal of
unrelated phase. The other band's synthesizers were still running (the
MS5351M's CLK2/CLK0 pair sits 12 kHz apart, as does the MAX2871 pair), and
their leakage made a second IF. **Switching the other band off at each band
change** (MS5351M outputs off in the high band; both MAX2871s shut down in the
low band) brings retune scatter to 0.001-0.002 dB, 0.02-0.07 deg (high band)
and 0.0005-0.001 dB, under 0.01 deg (low band). The MAX2871's low-spur modes
made no difference.

**Final calibration** (60 points, 1 MHz-1 GHz, 1 capture per point, 7 s per
sweep; standards in stock's order, thru last, checks without re-mating):

| Check | Our firmware | Stock (same caps) |
|---|---|---|
| Fresh short, corrected | 0 dB +/- 0.0046 dB, 180 +/- 0.062 deg | +/- 0.009 dB, +/- 0.06 deg |
| Fresh thru, 1-100 MHz | -0.0032..+0.0054 dB | +/- 0.015 dB |
| Fresh thru, 100 MHz-1 GHz | -0.0060..+0.0026 dB | (+/- 0.007 dB to 200 MHz) |
| Fresh thru phase | -0.054..+0.159 deg | +/- 0.07 deg |
| Sweep-to-sweep S21 (raw, thru) | 0.005 dB, 0.12 deg | |

Measuring the thru first and checking it after the caps adds the SMA
re-mating error (0.05-0.08 dB near 1 GHz); the checks above avoid it, as
stock's procedure does.

## Stage 7: firmware on the device, full range (2026-11-10)

`fw/` is the LiteVNAAI firmware proper (not the probe): the sweep runs on the
device, a NanoVNA-D-compatible text console drives it, and it tunes
50 kHz-6.3 GHz. Flashed through its own `bootloader` command
(`host/reflash.sh`), hands-free; thru connected for every number below.

**Safety built in:** watchdog from the first instruction (about 2 s); after
two watchdog resets in a row the firmware hands over to Zeenko's bootloader,
so a broken image can be replaced from the host. If a USB host is seen but
the console is not configured within 10 s, it also hands over. `bootloader`
and `reset` refuse while the jog reads pressed (Artery's ROM bootloader hazard).

**Settling, measured** (sweep-to-sweep, thru, 4 sweeps):

| Band | Settle after retune | Result |
|---|---|---|
| MS5351M (< 100 MHz) | 1.5 ms | 0.097 dB worst |
| | 3.5 ms | 0.008 dB |
| | 6.5 ms / 16.5 ms | 0.006 / 0.007 dB (no further gain) |
| MAX2871 (>= 100 MHz) | 300 us; +1 ms or +5 ms extra changed nothing | noise-limited, see below |

Chosen: 4 ms low band, 300 us high band, 10 ms at a band change.

**Range, raw thru levels** (`flags`, IF gain x1): reference 1557 counts at
50 kHz, 651 at 1 MHz, about 200 at 1-3 GHz, 40-50 at 5.5-6.3 GHz. Every point
from 50 kHz to 6.3 GHz tunes and reads without clip or low-reference flags;
6.0-6.3 GHz is beyond the MAX2871's 6 GHz specification and works on this
unit. Tuned frequency error is at most 3 kHz (6 kHz VCO steps at divider 1),
reported per point by `flags`.

**IF gain** (PB14 x3, PB15 x1.9, both x8): the S21 ratio is the same at every
gain within 0.03 dB wherever nothing clips, so a fixed gain per frequency
keeps a calibration valid. Clipping starts near 2000 counts. Table
(`gain auto`): x1 below 100 MHz, x1.9 to 2.8 GHz, x3 to 5.3 GHz, x8 above,
keeping the largest segment (port 2 on the thru) at about 1100-1400 counts.
Validated with the thru only; an open/short `flags` pass above 1 GHz is still
to do. Extra gain did not improve repeatability: the top band is not ADC-noise
limited.

**Repeatability, final defaults** (5 sweeps, deviation from the mean, S21 thru):

| Sweep | Time | Max | Median sd | Phase sd (median) |
|---|---|---|---|---|
| 1 MHz-1 GHz, 101 pts | 1.88 s | 0.0042 dB | 0.0009 dB | 0.025 deg |
| 50 kHz-100 MHz (in a 401-pt full sweep) | | 0.0043 dB | 0.0011 dB | 0.008 deg |
| 0.1-1.5 GHz | | 0.0053 dB | 0.0010 dB | 0.039 deg |
| 1.5-3 GHz | | 0.0146 dB | 0.0022 dB | 0.108 deg |
| 3-5 GHz | | 0.0136 dB | 0.0032 dB | 0.204 deg |
| 5-6.3 GHz | | 0.0520 dB | 0.0065 dB | 0.273 deg |
| 50 kHz-6.3 GHz, 401 pts | 7.18 s | 0.052 dB | 0.0027 dB | |

Above 3 GHz the scatter is synthesizer phase noise: averaging reduces it as
random noise would, extra settling does not, and a second reference capture
after port 2 (`bracket 1`, interpolated) cuts the median by about 25 % for
30 % more time, the same as averaging. `bracket` stays off by default.

**Known limitation, MAX2871 near-integer fractions:** at output dividers 2-4
(about 190 MHz-1.5 GHz) a stimulus fraction word near zero (F = 0, and
intermittently small values such as 4, 8, 16, 32, 64 or -4, -16 of M = 4000)
lands in one of several states at each retune: 0.01-0.02 dB sd that does not
average down within a tuning and persists until the next retune. 300, 600,
612 and 1200 MHz are examples; 301 and 601 MHz are clean. The sigma-delta
low-spur modes (R2 SDN = 10, 11) do not change it. A 1-2 MHz grid like the
101-point sweep above misses most of them. Open: integer mode for F = 0, or
re-tuning with a different modulus at those points.

**Frequency axis:** raw thru group delay -1.13 ns, constant from 0.5 to
6.3 GHz (band means -1.08 to -1.17 ns) with no phase step at the divider
boundaries (0.75, 1.5, 3 GHz); negative because the raw ratio's reference path
is longer than the thru.

**Calibration maths on the device:** the host's stage-6 standards (60 points)
loaded with `cal put`, solved on the device, read back with `cal get terms`:
every error term matches the host's within 6e-5 relative (float precision);
the device's correction of a live sweep matches the host formula within 2e-6.
A full SOLT check with real standards needs the maintainer's caps.

**Console additions:** `gain [auto|0..3]`, `bracket [0|1]`,
`cal get <open|short|load|thru|terms>`, `cal put <std> <i> s11re s11im s21re s21im`
(a host can save and restore a calibration), `flags` (tuned frequency,
reference level, status per point), `raw <freq>` (samples and the device DFT).

## Stage 8: display UI (2026-11-10)

Hardware SPI1 (remapped PB3/PB4/PB5) at 12 MHz for the panel and 375 kHz for
the HR2046. Eyes-free proof that the bus works: `vbat` 1323 counts through
hardware SPI (the probe's bit-banged read was 1299), `touch` y 4095 / z2 4085
with nothing touching. Battery scale set so 1299 counts reads 4.07 V, the
value stock showed for that state (a 3.3 V reference would read 0.25 V high).

The panel cannot be read back, so what it shows is **not confirmed by eye**.
Defaults: MIPI-DCS reset, sleep-out, 16-bit pixels, MADCTL 0x28 (row/column
exchange for landscape, BGR so red is red), inversion off, 480x320. `lcd test`
draws a self-describing card (corner labels, labelled colour bars, a white
480x320 edge and a magenta 320x240 corner); `lcd madctl <hex>` and
`lcd inv 0|1` change it at run time.

UI (`lcd ui`, and at power-on 1.5 s after start so USB enumerates first):
status line (range, points, calibration, battery, sweep count), log
magnitude +10 to -70 dB with S11 yellow and S21 cyan, a marker moved by the
jog with frequency, S11 dB/phase/SWR and S21 dB/phase, and point flags. No
framebuffer: drawn in 32-column strips through the ADC buffer, between
points. Repeatability with the UI drawing is unchanged (101 points,
1 MHz-1 GHz: 0.005 dB max, 0.0009 dB median sd).

**Jog ladder:** PB1 floats at rest (0.97-2.8 V over 30 reads with no pull, so
a digital read was unsafe for the reset guard). The internal pull-up holds it
at 2.48-2.85 V; left still pulls it to 0 V; it is read with the ADC
(pressed below 1000 counts). The jog push (in) changes no pin we can read and
may be wired to BOOT0, which would explain the ROM bootloader hazard: the
reset guard cannot see it.

## Recovery

**End every probe session with a real power cycle** (USB unplugged, switch
off), not just a reflash. The MAX2871 and MS5351M keep their registers until
power is removed, and stock does not rewrite all of them: after the MAX2871
tests, restored stock read the thru -3.35 dB / -28.9 deg off above 100 MHz
(raw S21, a step at its 100 MHz crossover, nothing below). A power cycle
brought the raw thru back to within 0.008 dB / 0.07 deg of the calibration-time
recording (2026-11-10).

1. Our firmware's `bootloader` command, or power on with the jog held.
2. Write the stock image (`liteVNA64-260713.bin`) at 0x08004000 with the bench's V2-protocol flash tool, which follows Zeenko's documented DFU registers.
3. Zeenko's bootloader (0x08000000–0x08003FFF) and the option bytes are never
   written by anything here. Artery's ROM bootloader (BOOT0) is not used.

## Next

- Stage 1 (inputs only): done, results above.
- Stage 2a (I2C scan): done, MS5351M at 0x60 on PB10/PB11.
- Stage 2c: touch, display, backlight and jog mapped; MAX2871 not found by
  MUX/LD response.
- Stage 6: repeatability matches or beats stock (thru +/-0.006 dB, short
  +/-0.0046 dB, 1 MHz-1 GHz).
- Stage 7: sweep, console and calibration on the device, 50 kHz-6.3 GHz.
- Stage 8: display UI built, starts at power-on; to be confirmed by eye.
  Next: saved settings and calibration in flash, touch input, an open/short
  level check above 1 GHz, the near-integer MAX2871 states.
