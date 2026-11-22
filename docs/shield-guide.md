# Writing a ZMK shield for direct EC

This guide walks through configuring a ZMK shield for a new direct-EC keyboard, using Caroline as a worked example. Caroline is a 36-key unibody split on a Seeed XIAO Plus socket with one shield covering nRF52840, SAMD21, and RP2040. Its config lives in [caroline-zmk-module](https://github.com/carrefinho/caroline-zmk-module) under `boards/shields/caroline/`.

If you are building or tuning an existing keyboard rather than creating a new shield, see the [User & Tuning Guide](user-guide.md).

## File layout

Follow ZMK's [new shield guide](https://zmk.dev/docs/hardware-integration/new-shield) for the basic shield folder, Kconfig files, and overlay, and add `zmk-direct-ec` to `config/west.yml` as shown in the [README](../README.md).

Because Caroline supports three controllers, its shield overlay only holds the physical layout. Chip-specific parts—the kscan node, matrix transform, pin cleanup, and `.conf`—live in per-board files under `boards/` (the same `boards/<board>.overlay` pattern described on ZMK's [underglow page](https://zmk.dev/docs/hardware-integration/lighting/underglow)). If your board targets a single controller, put everything in the shield overlay.

## Step 1: free the pins

Because the driver controls the ADC registers directly, Zephyr's ADC driver must be disabled, along with any default peripheral that claims a sense or strobe pin. Leaving a peripheral enabled will cause it to fight the driver over the pin and corrupt readings.

What Caroline disables:

| Controller | Disabled |
|---|---|
| nRF52840 | `&adc`, `&vbatt`, `&uart0`, `&i2c1`, `&spi2` |
| SAMD21 | `&adc`, `&dac0`, `&sercom0`, `&sercom2`, `&sercom4`, `leds` node |
| RP2040 | `&adc`, `&i2c1`, `&spi0`, `&uart0`, `leds` node |

Check your board's `.dts` against your pin list. Common pitfalls from Caroline:

- XIAO Plus modules build on the non-Plus board files. The extra pads aren't in the `xiao_d` connector map, so address them as raw GPIOs (`&gpio1 3`, `&porta 18`). Some pads that are LEDs on the plain board are broken out on the Plus, so disable the `leds` node to prevent it from driving a strobe.
- NFC pins on nRF52840. Caroline uses P0.09 and P0.10 as strobes, which requires `nfct-pins-as-gpios` in `&uicr`. A fresh chip resets once on first boot while writing this setting.
- Shared analog switches. On the XIAO RP2040 Plus, D3 reaches GPIO29 through an analog switch shared with the battery divider. A `gpio-hog` on GPIO24 holds the switch on the D3 side.

The `ec` shell does not use the board's console UART—the `direct-ec-shell` snippet creates its own USB serial port—so you can safely disable the console UART if you need its pins.

## Step 2: sense lines

Sense lines are the matrix lines wired to ADC pins. List each one twice in the same order: once in `ain-channels` as the chip's ADC channel number, and once in `col-gpios` as a GPIO. The driver verifies that the two lists match.

The `ain-channels` value depends on the controller:

| Chip | `ain-channels` value | Caroline |
|---|---|---|
| nRF52840 | AIN index | `<0 1 4 5>` for D0–D3 |
| SAMD21 | MUXPOS | `<0 4 18 19 6 5 7 3>` for D0–D3, D10–D7 |
| RP2040 | AINSEL (0 to 3) | `<0 1 2 3>` for GPIO26–29 |

Verify these against the chip datasheet rather than the module's silkscreen names; on SAMD21 in particular, MUXPOS values do not match pad numbers.

Keep your naming consistent in comments: Caroline's schematic calls sense lines "rows" and strobes "columns", while the driver calls sense lines `col-gpios`.

## Step 3: strobes

List strobe pins as plain GPIO outputs in `strobe-gpios` in your desired numbering order (up to 16 strobes).

- SAMD21: Declare strobes `GPIO_ACTIVE_LOW` and set `bias-pull-up`. The SAMD21 backend primes the sense line with a pull-up to avoid a low-voltage ADC glitch, so strobes must idle high and step down.
- nRF52840 and RP2040: Use `GPIO_ACTIVE_HIGH` and leave bias at its default.

Driver limits: up to 8 sense lines, 16 strobes, 80 cells, and one `zmk,kscan-direct-ec` node per build.

## Step 4: transform and layout

See ZMK's [layout reference](https://zmk.dev/docs/config/layout) and [physical layouts page](https://zmk.dev/docs/hardware-integration/physical-layouts) for general usage. Two details are specific to direct EC:

1. Coordinate order: The driver reports keys as `(strobe, sense)`. In your matrix transform, `rows` is the number of strobes, `columns` is the number of sense lines, and each entry is `RC(strobe, sense)`.

   Caroline's nRF52840 transform (4 sense lines, 10 strobes):

   ```dts
   ec_transform: ec_transform {
       compatible = "zmk,matrix-transform";
       columns = <4>;
       rows = <10>;
       map = <
           RC(0,0) RC(1,0) ... RC(9,0)
           ...
           RC(0,3)  RC(3,3) RC(4,3) RC(5,3) RC(6,3)  RC(9,3)
       >;
   };
   ```

   The driver automatically skips any `(strobe, sense)` pair not present in `map` (reporting 0). On Caroline's SAMD21 build, this skips 44 of 80 cells, which saves significant scan time. Note that unmapped cells are never scanned and will not appear in `ec status`. If you need to scan cells outside the transform, override this with `sense-strobe-masks` (see `dts/bindings/kscan/zmk,kscan-direct-ec.yaml`).

2. Input device binding: Because the driver is a ZMK input device rather than a legacy kscan, attach it to the physical layout's `input` property:

   ```dts
   ec_layout: ec_layout {
       compatible = "zmk,physical-layout";
       transform = <&ec_transform>;
       input = <&ec_kscan>;
       keys = < ... >;
   };
   ```

   If `input` is omitted, ZMK falls back to a `zmk,matrix-input` chosen node (see the [binding](https://github.com/zmkfirmware/zmk/blob/main/app/dts/bindings/zmk%2Cphysical-layout.yaml)). Do not add a `kscan` property or `zmk,kscan` / `zmk,matrix-transform` chosen nodes, as they conflict with the input path.

## Step 5: first flash

Start with a bench config: a single-layer keymap where every key outputs a distinct character, loose initial thresholds (e.g., `200` / `140`), and the `ec` shell enabled via `-S direct-ec-shell` (or `snippet: direct-ec-shell` in `build.yaml`).

Add `CONFIG_INPUT=y` to your `.conf` (required for both bench and release builds).

Flash the board (with no keys held at power-on), open the shell's USB serial port, and verify the matrix:

1. Run `ec rest` (see the [User & Tuning Guide](user-guide.md)) to confirm every cell scans cleanly.
2. Run `ec status` while pressing keys and verify that:
   - Pressing a key moves only that key, well above idle noise.
   - Bringing a finger near a pad moves all keys on that sense line together. If the wrong keys move together, check `ain-channels` / `col-gpios` ordering or board jumpers.

## Step 6: scan timing

The scan pass must finish within `poll-interval-ms`. Pass time depends on the controller, cell count, and `CONFIG_ZMK_KSCAN_DIRECT_EC_NAVG` (samples per key per pass, default 4).

Caroline's timings (36 keys):

| Controller | NAVG | Filter | Pass time | `poll-interval-ms` |
|---|---|---|---|---|
| nRF52840 | 4 | median | ~1.5 ms | 4 ms |
| SAMD21 | 3 | median | 3.9 ms | 5 ms |
| RP2040 | 4 | mean | 1.8 ms | 4 ms |

- SAMD21: At 5 µs per conversion and 4 conversions per cell, `NAVG=4` took 5.1 ms and overran a 5 ms interval. Dropping to `NAVG=3` keeps a true median while fitting comfortably in 5 ms.
- RP2040: Set `CONFIG_ZMK_KSCAN_DIRECT_EC_MEAN=y`. Its ADC does not produce outlier samples, and a plain mean was quieter than a median on noisy USB supplies.
- Settle and prime (SAMD21 / RP2040): On Caroline, `settle-us = <0>` read identically to `200` and made the pass 5× faster; try `0` first and verify with `ec status`. Keep `prime-conversions = <2>` unless you test otherwise (dropping to `1` on SAMD21 shifted one cell's resting level by 59 counts).

## Step 7: load compensation and calibration

Always measure on the fully assembled board (plate, domes, and housings). A bare PCB reads very differently—Caroline's SAMD21 build lost ~40% of its signal going from a bare spring to the full assembly.

### Load compensation (`ec fit`)

Every held key adds capacitance to its sense line and lowers the readings of other keys on that line. On Caroline's 5-key SAMD21 lines, each held key dropped the others by 15–20%; on 10-key lines, the drop was 5–9% per held key. Without compensation, fast rolls can miss actuations.

`load-comp-scale` (K) corrects this in firmware. To measure it:

1. Run `ec fit`. It highlights the keys on one sense line.
2. Press at least four of those keys one after another (as many as is comfortable), hold them all down, then release. Consistent finger pressure matters more than holding every key on a long line, as pressure variation can shift a key's estimate by up to 20%.
3. Repeat for as many sense lines as you want. The command ends automatically 10 s after the last key or once every line is done.

`ec fit` computes K as the median across lines and applies it until reset. The usable range is broad: Caroline's hand-tuned values were `3600` (nRF52840), `3850` (SAMD21), and `4000` (RP2040), and `ec fit` later produced `3830` and `3864` on nRF52840 and SAMD21. Refit K if the PCB or controller changes.

### Per-key swing and thresholds

Run `ec calibrate` after `ec fit` (since K also scales single-key readings). When it finishes, the driver applies the new swings until reset so you can type on the result immediately, and flags any marginal key whose press point is under 3× its own noise from `ec rest`.

Each key presses at `actuation-percent` (default 40) of its own swing and releases at two-thirds of that. Test values live with `ec threshold 35%`. `press-threshold` and `release-threshold` only cover keys without a swing (so a new board types before calibration); set them around 40% of the weakest key's swing with release at two-thirds, and test with `ec threshold <press> <release>`.

Caroline's uncompensated swing spread (before `load-comp-scale`):

| Controller | Weakest | Median | Strongest |
|---|---|---|---|
| nRF52840 (10 keys/line) | 227 | 304 | 353 |
| SAMD21 (5 keys/line) | 284 | 391 | 506 |
| RP2040 (10 keys/line) | 165 | 262 | 327 |

With compensation enabled, SAMD21's weakest key rose from 284 to 346. Swings are measured compensated, and press points are computed from them.

Caroline's final values:

| Controller | Press points at 40% | Fallback press / release | `load-comp-scale` |
|---|---|---|---|
| nRF52840 | +74 to +168 | 110 / 70 | 3600 |
| SAMD21 | +164 to +278 | 150 / 100 | 3850 |
| RP2040 | +88 to +130 | 100 / 65 | 4000 |

Run `ec export` to print `actuation-percent`, the fallback thresholds, `load-comp-scale`, and the `swing` table as devicetree lines, and paste them into your overlay (keeping the measured numbers in a comment makes future retuning easier). A key's swing changes if it is reseated or rebuilt, so re-calibrate after mechanical changes.

### Check on a noisy power supply

Test USB-powered boards on the worst USB charger or hub you have. On a cheap power-strip brick, Caroline's SAMD21 idle noise jumped 7–15× (excursions up to ±93). At a press threshold of 150 it still had no ghost presses, but margin dropped to 1.8×. If you see ghost keys on a noisy supply, raise `actuation-percent` or `NAVG`. Measure noise on the supply the board will actually use: Caroline's nRF52840 read 16 peak-to-peak on a MacBook running on battery and 24 with its charger plugged in.

## Step 8: nRF52840 power settings

Only nRF52840 supports the low-power idle scan. When idle, it raises all strobes at once and checks each sense line in one pass, waking the full scan whenever a line crosses `CONFIG_ZMK_KSCAN_DIRECT_EC_TRIPWIRE_THRESHOLD`.

- Tuning the tripwire: With the idle scan running, check `ec status` to see how far each key moves its sense line and what the quiet maximum is. On Caroline, presses moved 210–315 and the quiet max was +14, so a threshold of `70` sits at ~1/3 of the weakest press and 5× the quiet ceiling.
- USB-only nRF52840 boards: Set `CONFIG_ZMK_KSCAN_DIRECT_EC_TRIPWIRE=n`.
- Deep sleep: Leave `CONFIG_ZMK_SLEEP` off (ZMK's [default](https://zmk.dev/docs/config/power)). EC keys cannot wake the chip from deep sleep, and the idle scan already provides the low-power state.
- Battery sensing: Because the driver owns the SAADC, replace `zmk,battery-voltage-divider` with `zmk,direct-ec-battery` and point the `zmk,battery` chosen node at it:

  ```dts
  ec_battery: ec_battery {
      compatible = "zmk,direct-ec-battery";
      ain-channel = <7>;
      power-gpios = <&gpio0 14 (GPIO_OPEN_DRAIN | GPIO_ACTIVE_LOW)>;
      output-ohms = <510000>;
      full-ohms = <1510000>;
  };
  ```

  The scan thread reads the divider between idle passes, so the battery percentage does not update while a key is held down.

Caroline measured 42 µA idle and ~0.6 mA typing over BLE. Reaching that also required standard nRF low-power hygiene: console and logging off, plus suspending the XIAO's unused QSPI flash with [zmk-feature-xiao-ble-qspi-suspend](https://github.com/carrefinho/zmk-feature-xiao-ble-qspi-suspend).

## Step 9: release build

Once the numbers are in your overlay, rebuild without `-S direct-ec-shell` before adding your full keymap. On SAMD21, the shell snippet plus Caroline's full keymap overflowed RAM by 2.7 KB.

## Checklist

- [ ] `&adc` and every default peripheral on a used pin disabled
- [ ] `ain-channels` and `col-gpios` in the same order, using the chip's ADC channel numbers
- [ ] SAMD21: strobes `GPIO_ACTIVE_LOW` and `bias-pull-up` set
- [ ] Transform has `rows` = strobes, `columns` = sense lines, entries `RC(strobe, sense)`
- [ ] Driver attached to physical layout's `input` property; no `kscan` property or chosen node
- [ ] Scan pass fits inside `poll-interval-ms`
- [ ] Swings and `load-comp-scale` measured on the assembled board, no marginal keys
- [ ] Tested on a noisy USB power supply
- [ ] nRF52840: tripwire threshold set, `CONFIG_ZMK_SLEEP=n`, `zmk,direct-ec-battery` configured
