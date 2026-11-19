# Designing a PCB for direct EC

## What is different without a front end

- A conventional board puts a fixed capacitor on each sense line (1 nF in Tom Smalley's guide), much larger than a key. Here there is none. The line's own stray capacitance takes its place, and it is tens of picofarads at most.
- Reading is roughly VDD × C_key / (everything on the line). So everything on the line is part of the gain: trace, pads, the ADC pin, every other key, a hand.
- Three consequences that the rest of the guide follows up: pin mapping decides the matrix shape, keys on a line load each other, and copper near a sense line costs signal as well as shielding it.
- The sense line floats at high impedance. It cannot be probed with a scope (a 10× probe is as big as the line). All measurement is through firmware.

## Pin mapping

### Sense lines need ADC pins

- Table per controller: how many ADC inputs the chip has, and how many a XIAO Plus module brings out.
  - nRF52840: 8 AIN on the chip. XIAO Plus exposes AIN0–5 on D0–D5; AIN6 is an LED, AIN7 the battery divider.
  - RP2040: 4 (GPIO26–29), D0–D3 on the XIAO.
  - SAMD21: 11 on the XIAO's front pads (D0–D10). The 16 back pads have no ADC, so they can only be strobes.
- Driver limits: 8 sense lines, 16 strobes, 80 cells.
- The ADC pin count caps the number of sense lines, not the number of keys. It decides which axis is sensed. With 4 inputs a 40-key board senses rows; with 11 it can sense columns or half-rows.
- Strobes can be any GPIO.

### Pins to avoid or free up

- A sense pin must carry nothing else. On nRF52840 the pin's output driver is disconnected while its analog input is selected, so it cannot double as anything.
- Dev-board defaults claim most analog pins. On the XIAO SAMD21 every front pad but three belongs to a default peripheral (DAC, I2C, SPI, UART); all have to be disabled in the overlay.
- nRF52840 NFC pins (P0.09, P0.10) work as strobes with `nfct-pins-as-gpios`.
- LED pins: fine as strobes once the board's `leds` node is dropped.
- Module footprints: check pad order against the module's own pinout by position. The XIAO Plus extra pads are numbered differently in Seeed's symbol and in common KiCad footprints.
- On-module parts in series with a pad count as part of the line. XIAO RP2040 Plus routes D3 through an analog switch shared with the battery divider; that line reads about a fifth lower.
- One sense pad can be noisier than the rest because of the module (D0 on the XIAO SAMD21, next to USB and a floating VREFA pin, about 2×). Put the least critical line there, or leave margin.

### Picking a topology for the controller

- Design the matrix for the one controller the board will use, and spend its ADC inputs. With SAMD21's eleven, split each row into two half-row sense lines; with four inputs, sense whole rows.
- Sparse matrices: when a strobe only crosses some sense lines (as with half-rows), `sense-strobe-masks` skips the empty cells (SAMD21 and RP2040 only).
- Caroline, the board the numbers in this guide come from, is a technology demonstrator and does both: a solder jumper per row joins the two halves (4 lines of ten, nRF52840 and RP2040) or gives the left half its own ADC pad (8 lines of five, SAMD21). Mention it only to explain where the five-versus-ten comparison comes from. Not a recommendation; a normal design does not need the jumpers.

## Keys per sense line

- Every pressed key adds its capacitance to the line, so every other key on that line reads lower. No conventional guide covers this because the 1 nF capacitor hides it.
- Measured on Caroline, which can be jumpered for either line length on the same PCB:
  - 5 keys per line: each held key takes 15–20 % off the others; all five held leaves 41–65 %. An unpressed key sits about 75 counts below its baseline with the other four down.
  - 10 keys per line: 5–9 % per held key; ten held leaves 55–60 %.
- The trade: a shorter line gives a larger lone press and a bigger drop per held key. A longer line gives the opposite. Cleaner routing (less stray capacitance) makes the per-key drop worse, not better.
- `load-comp-scale` removes it in firmware: with K fitted, a key reads within about 3 % from one held to ten. Cost is noise multiplied by the same factor, up to 1.5–1.8 on a fully held line.
- So ten per line is workable with compensation. Without it, keep lines to 4–5 keys or set thresholds for the worst case.
- Which keys share a line matters. Rows as sense lines put the whole home row on one line. Keep habitually held keys (modifiers, layer keys) off lines that carry fast alphas where the layout allows.
- A hand over an unshielded line loads it the same way a pressed key does, and firmware cannot count hands.
- The idle pass on nRF52840 is not weakened by a larger matrix: a press reads the same with all strobes raised as in the per-key pass (210 against 227 on a ten-key line).

## Routing the sense lines

- What carries over from conventional EC, by link only: keep strobe and other digital traces away from sense traces, cross at right angles and as rarely as possible.
- Ground under the sense traces is fine. Smalley's guide asks for pour around the lines and a ground plane on the other layer, and the same works here: Caroline has unbroken pour under its sense lines, ten keys per line, and reads 227–353 counts per key.
  - In principle that ground adds to what the key divides against (roughly 0.4 pF/cm for a 0.25 mm trace on 1.6 mm FR4, calculated), so a keepout under the trace should buy some signal. An earlier test board had a 1 mm keepout; the gain was never measured. It also cuts both ways: less stray capacitance means a bigger drop per held key.
  - So treat a keepout as optional. The plane keeps the line's capacitance set by the board, not by what is near it.
- Side pour has to be the same on every line. On a test board with no ground under the traces, a sense trace along the board edge with pour on one side only was 4–6× noisier than its neighbours on battery and 13–17× on a noisy charger, with no measurable difference in capacitance. If the plane underneath is removed, side pour is the whole shield.
- Keep pour around the pads themselves as existing guides describe.
- Trace length: 156–175 mm sense lines work. Put the controller central if the layout allows.
- Nothing in series or parallel: no resistors, ESD parts, test points with long stubs, or connectors on a sense line.
- Untested: floating metal screws next to sense traces and pads may add noise on a poor charger.
- Two layers are enough.

## Routing the strobes

- Driven push-pull and never floated, so their own capacitance is charged by the driver and barely shows (a bodge wire on one strobe added 2 counts).
- They have fast edges and are the source of coupling into sense lines, so they do get a ground reference.
- Strobe-to-sense trace coupling with no pad involved shows up as a fixed offset per cell, removed by the baseline.
- SAMD21 wants active-low strobes. That is a devicetree flag, not a board change.

## Power supply and noise

- On battery the lines are quiet: idle noise about 0.5–3 counts against presses of 200–400.
- Over USB the supply sets the noise, not the chip. A poor charger gave 20–40× the battery figure on all three controllers, with excursions of 50–66 counts. No ghost keys at sensible thresholds, but the margin gets thin once compensation multiplies it.
- A mains-earthed host is about 3.5–4× a laptop on battery.
- So a USB-only board should leave more threshold margin than a battery board, and the per-controller estimator choice matters there.

## Battery sensing on nRF52840

- The driver owns the SAADC, so the divider has to be read by the module's own battery node. It needs its own AIN, which comes out of the eight.
- XIAO nRF52840: divider on AIN7, enabled by P0.14, 510 k / 1 M.
- Decide this at schematic time; it is one fewer sense line.

## Bringing a board up

- Keep USB reachable on the first prototype; the `ec` shell is the only instrument.
- Order of work: `ec rest` to check every cell scans, `ec fit` for `load-comp-scale`, `ec calibrate` to find the weakest key, then set thresholds (about 40–45 % of the weakest compensated key).
- A spring seated badly can read less than half of a good one and look fine by eye.
- Expect an assembled key to read about 40 % under a bare collapsed spring.

## Further reading

- Tom Smalley, custom-topre-guide: the conventional circuit, plate hole sizes, calibration. Its routing section is four sentences; it has no clearances, via or pad dimensions.
