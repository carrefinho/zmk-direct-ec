# Designing a PCB for direct EC

This guide covers the schematic and PCB layout differences between a conventional electrostatic-capacitive (EC) keyboard and a direct-EC board driven by `zmk-direct-ec`. Measured numbers below come from Caroline, a 36-key unibody split tested across nRF52840, SAMD21, and RP2040.

Because of ADC pin-count limits, sense-line capacitance, and the 80-cell driver cap, direct EC works best on smaller keyboards—such as 40% boards, splits, and macropads—rather than full-size layouts.

## What changes without an analog front end

A conventional EC board puts a fixed integrating capacitor (such as 1 nF in Tom Smalley's guide) on each sense line—much larger than any single key's capacitance. Direct EC removes that capacitor, so the sense line's own stray capacitance (tens of picofarads at most) takes its place.

Roughly speaking, a key's reading is proportional to:

```
VDD × C_key / C_line_total
```

Everything connected to the sense line—trace capacitance, key pads, the ADC pin, other keys on the line, and even a nearby hand—is part of `C_line_total` and directly affects signal amplitude.

Two practical consequences follow from this:

- Sense lines float at high impedance. You cannot probe them with an oscilloscope (a 10× probe adds as much capacitance as the entire line). All bring-up and measurement happens through the firmware's `ec` shell over USB.
- Pin count, line length, and surrounding copper matter more than on a conventional EC board, as detailed below.

## Choosing a controller and mapping pins

### Sense lines require ADC pins

The number of available ADC pins caps how many sense lines you can have (not how many keys). That usually dictates which axis of your matrix is sensed: with 4 ADC pins, a 40-key board senses rows; with 8–11 ADC pins, it can sense columns or half-rows. Strobes can use any plain GPIO.

Driver limits are 8 sense lines, 16 strobes, and 80 cells per board.

| Controller | Chip ADC inputs | Exposed on XIAO Plus | Notes |
|---|---|---|---|
| nRF52840 | 8 (AIN0–7) | AIN0–5 on D0–D5 | AIN6 is an LED; AIN7 is the battery divider. |
| RP2040 | 4 (GPIO26–29) | D0–D3 | D3 passes through an analog switch shared with the battery divider (~20% lower signal). |
| SAMD21 | 11+ | 11 front pads (D0–D10) | The 16 back pads have no ADC and can only be strobes. |

Design the matrix for the controller you plan to use and spend its available ADC pins. On SAMD21, splitting each row into two half-row sense lines gives a stronger signal than 4 long rows; the firmware automatically skips empty cells in a sparse matrix. (Caroline uses solder jumpers to test both 4×10 and 8×5 on one PCB, which is where the 5-vs-10-key comparisons below come from—a normal design does not need jumpers.)

### Pins to watch out for

- Dedicated sense pins: A sense pin must not be shared with any other circuit. On nRF52840, the pin's output driver is disconnected while its analog input is active.
- Default peripherals: Dev-board definitions claim most analog pins by default (e.g., on XIAO SAMD21, all but three front pads belong to DAC, I2C, SPI, or UART). They work fine as sense or strobe pins once disabled in the ZMK overlay.
- NFC and LED pins: nRF52840 NFC pins (P0.09, P0.10) and board LED pins work fine as strobes once freed in devicetree.
- Module footprint numbering: Double-check pad order by physical position against the module's datasheet. XIAO Plus extra pads are numbered differently across Seeed's symbol and common KiCad footprints.
- Series components on modules: Anything in series with an ADC pad becomes part of the sense line. On the XIAO RP2040 Plus, D3's shared analog switch reduced that line's signal by about a fifth.
- Noisy module pads: Some pads sit next to noisy rails or clocks (e.g., D0 on XIAO SAMD21 sits next to USB and a floating VREFA pin and measured ~2× noisier than other pads). Assign the least critical line there or leave extra threshold margin.

## Keys per sense line

Every pressed key adds capacitance to its sense line, which reduces the reading of every other key on that same line. Conventional EC guides rarely mention this because the 1 nF integrating capacitor masks it.

Measured on Caroline across the same PCB:

- 5 keys per line: Each held key drops the others by 15–20%. With all five held, readings drop to 41–65% of normal, and an unpressed key sits ~75 counts below baseline.
- 10 keys per line: Each held key drops the others by 5–9%. With all ten held, readings drop to 55–60% of normal.

This creates a trade-off: shorter lines (or cleaner routing with less stray capacitance) give a stronger single-key signal, but a larger percentage drop per held key. Longer lines give the opposite.

The firmware's load compensation (`load-comp-scale`) corrects this to within ~3% from one key held to ten, at the cost of multiplying noise by the same factor (up to 1.5–1.8× on a fully held line). With compensation enabled, 10 keys per line is completely workable. Without it, stick to 4–5 keys per line or leave large threshold margins.

A few layout-level tips:

- Modifier placement: If you sense rows, the whole home row shares one line. Where your layout allows, keep habitually held keys (modifiers, layer keys) off lines that carry fast alphas.
- Hand loading: A hand hovering over an unshielded sense line loads it just like a pressed key, and firmware cannot compensate for that.
- nRF52840 idle wake: Larger matrices do not weaken the idle tripwire pass—on Caroline's 10-key line, a press read 210 in the all-strobes idle pass vs. 227 in the per-key pass.

## PCB layout

Two layers are enough. Standard EC layout rules still apply: keep strobes and other digital traces away from sense traces, and cross them at right angles as rarely as possible.

### Routing sense lines

- Ground plane underneath is fine. Like conventional EC guides recommend, Caroline uses an unbroken ground pour under its sense lines (10 keys per line) and still reads 227–353 counts per key. While a keepout under the trace reduces stray capacitance slightly (~0.4 pF/cm calculated for a 0.25 mm trace on 1.6 mm FR4), it also increases the per-key load drop and makes the line more sensitive to nearby hands. Treat a ground keepout as optional.
- Keep side pour symmetric. On a test board without a ground plane underneath, a sense trace running along the board edge with pour on only one side was 4–6× noisier on battery and 13–17× noisier on a poor USB charger than its neighbors. If you omit the plane underneath, symmetric side pour is your only shield.
- Pad pour: Keep ground pour around the key pads themselves as conventional EC guides describe.
- Trace length: Sense traces of 156–175 mm worked without issue. Place the controller centrally if your layout allows.
- Nothing in series or parallel: Do not place resistors, ESD diodes, connectors, or long test-point stubs on a sense line.
- Hardware near traces: Floating metal screws next to sense traces or pads are untested and may couple noise on a poor USB charger.

### Routing strobes

- Strobes are driven push-pull and never float, so their own trace capacitance has almost no effect (a bodge wire on a strobe changed readings by only 2 counts).
- Give strobes a clean ground reference, since their fast edges are the main source of unwanted coupling into sense lines.
- Fixed strobe-to-sense trace coupling (outside of key pads) shows up as a constant offset per cell and is subtracted out by the power-on baseline.
- SAMD21 requires active-low strobes, which is handled purely in devicetree—no PCB change is needed.

## Power supply, noise, and battery sensing

- Battery power is quiet: Idle noise on battery measured 0.5–3 counts against presses of 200–400 counts.
- USB supply dictates noise: Over USB, the power supply dominates noise across all three controllers. A mains-earthed host was ~3.5–4× noisier than a laptop on battery, and a cheap power-strip charger was 20–40× noisier (excursions of 50–66 counts). USB-only boards should leave more threshold margin than battery boards.
- nRF52840 battery divider: Because the driver owns the SAADC, battery sensing uses the module's `zmk,direct-ec-battery` node and requires its own AIN pin (out of the 8 available). On XIAO nRF52840 this is AIN7 (enabled by P0.14, 510 kΩ / 1 MΩ). Account for this at schematic time, as it consumes one potential sense line.

## Prototype bring-up tips

- Keep USB accessible on your first prototype, since the `ec` shell over USB serial is your only measurement tool.
- Expect ~40% lower signal once assembled: A bare collapsed spring reads much higher than a full plate/dome/housing stack.
- Check spring seating: A poorly seated spring can look fine by eye while reading less than half as strong as its neighbors.

Once your prototype is built, follow the [Shield Guide](shield-guide.md) to bring it up (`ec rest` → `ec fit` → `ec calibrate` → `ec export` into the overlay; each key presses at `actuation-percent`, default 40, of its own swing).

## Further reading

- [Tom Smalley's custom-topre-guide](https://github.com/tomsmalley/custom-topre-guide) — covers conventional EC circuits, plate cutouts, and mechanical assembly.
