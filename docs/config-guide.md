# Configuring a keyboard for direct EC

This guide walks through writing a ZMK shield for a direct-EC board. It uses Caroline as the worked example. Caroline is a 36-key split-in-one board on a Seeed XIAO Plus socket, and one shield covers three controllers: nRF52840, SAMD21 and RP2040. Its config lives in [caroline-zmk-module](https://github.com/carrefinho/caroline-zmk-module), under `boards/shields/caroline/`.

The README covers what each property does. This guide is about the order you do things in and the traps along the way.

## How the files split up

For the shield basics, follow ZMK's [new shield guide](https://zmk.dev/docs/hardware-integration/new-shield). That covers the folder, the Kconfig files and the overlay. Add this module to `config/west.yml` as shown in the README.

Caroline takes three different controllers, so its shield overlay only holds the physical layout. Everything that depends on the chip lives in per-board files under `boards/`: the kscan node, the transform, the pin cleanup and the `.conf`. That is the same `boards/<board>.overlay` setup ZMK's [underglow page](https://zmk.dev/docs/hardware-integration/lighting/underglow) describes. If your board only ever takes one controller, put everything in the shield overlay.

## Step 1: free the pins

The driver talks to the ADC registers itself, so Zephyr's ADC driver has to go. So does any peripheral the board file turns on by default on a pin you sense or strobe. If you skip this, the peripheral and the driver fight over the pin and the readings make no sense.

What Caroline turns off:

| Controller | Disabled |
|---|---|
| nRF52840 | `&adc`, `&vbatt`, `&uart0`, `&i2c1`, `&spi2` |
| SAMD21 | `&adc`, `&dac0`, `&sercom0`, `&sercom2`, `&sercom4`, the `leds` node |
| RP2040 | `&adc`, `&i2c1`, `&spi0`, `&uart0`, the `leds` node |

Go through your board's `.dts` and check every peripheral against your pin list. A few that caught Caroline:

- **XIAO Plus modules build on the non-Plus board files.** The extra pads aren't in the `xiao_d` connector map, so address them as raw GPIOs (`&gpio1 3`, `&porta 18`). Some pads that are LEDs on the plain board are pads on the Plus, so the `leds` node has to be disabled or something will drive a strobe.
- **NFC pins on nRF52840.** Caroline strobes on P0.09 and P0.10. They need `nfct-pins-as-gpios` in `&uicr`. A fresh chip resets once on first boot while it writes this.
- **Shared analog switches.** On the XIAO RP2040 Plus, D3 reaches GPIO29 through an analog switch that also connects the battery divider. A `gpio-hog` on GPIO24 holds it on the D3 side.

The `ecd` shell doesn't need the board's console UART. The `ec-direct-shell` snippet adds its own USB serial port, so you can disable that UART if you need its pins.

## Step 2: sense lines

Sense lines are the matrix lines wired to ADC pins. Each one is listed twice in the same order: once in `ain-channels` as the converter's channel number, once in `col-gpios` as a GPIO. The driver checks that the two lists agree.

The channel number is a different thing on each chip:

| Chip | `ain-channels` value | Caroline |
|---|---|---|
| nRF52840 | AIN index | `<0 1 4 5>` for D0–D3 |
| SAMD21 | MUXPOS | `<0 4 18 19 6 5 7 3>` for D0–D3, D10–D7 |
| RP2040 | AINSEL, 0 to 3 | `<0 1 2 3>` for GPIO26–29 |

Look these up in the chip's datasheet, not the module's pin names. On SAMD21 especially, the MUXPOS value has nothing to do with the pad number.

Watch the naming. Caroline's schematic calls its sense lines rows and its strobes columns. The driver calls sense lines `col-gpios`. Pick one convention for your comments and stick to it, because this gets confusing fast.

### How many keys per sense line

Each key on a sense line adds to that line's capacitance, which shrinks every other key's reading on the line. Fewer keys per line means stronger signal. Caroline has jumpers (JP1–JP4) that either join each row into one line of ten keys or split it into two lines of five:

- nRF52840 and RP2040 only have four usable ADC inputs on the module, so they run four lines of ten (A–C).
- SAMD21 has ADC on all eleven front pads, so it runs eight lines of five (C–B).

If your chip has spare ADC pins, splitting lines is worth it.

## Step 3: strobes

Strobes are plain GPIO outputs in `strobe-gpios`, in the order you want them numbered. Up to 16.

Polarity matters on SAMD21. That backend primes the sense line with a pull-up to dodge a converter glitch, so strobes have to idle high and step down. Declare them `GPIO_ACTIVE_LOW` and set `bias-pull-up`. On nRF52840 and RP2040, use `GPIO_ACTIVE_HIGH` and leave the bias at its default.

The driver tops out at 8 sense lines, 16 strobes and 80 cells, and you can only have one `ec_kscan` node per build.

## Step 4: transform and layout

For how transforms and physical layouts work in general, see ZMK's [layout reference](https://zmk.dev/docs/config/layout) and [physical layouts page](https://zmk.dev/docs/hardware-integration/physical-layouts). Two things differ here.

The driver reports each key at (strobe, sense line). So in the matrix transform, `rows` is the number of strobes and `columns` is the number of sense lines, and each entry is `RC(strobe, sense)`.

Caroline's nRF52840 transform, four sense lines and ten strobes:

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

On the SAMD21 build the left half of each row is its own sense line (4 to 7), so the map jumps between line numbers mid-row. Get this wrong and keys come out on the wrong half, which is easy to spot with the default keymap.

The driver also uses the transform to decide which spots to scan. Any (strobe, sense line) pair the map doesn't list has no key, so the driver skips it and reports 0 there. On Caroline's SAMD21 build that skips 44 of 80 spots, which matters on a chip that converts one spot at a time. You don't set anything for this. The flip side: a key missing from the map is never scanned, so it won't show up in `ecd status` either. If you need to scan spots outside the map, `sense-strobe-masks` in the binding overrides it.

The driver is an input device, not a kscan, so it goes in the physical layout's `input` property instead of `kscan`:

```dts
ec_layout: ec_layout {
    compatible = "zmk,physical-layout";
    transform = <&ec_transform>;
    input = <&ec_kscan>;
    keys = < ... >;
};
```

ZMK's docs only show `kscan`. `input` is in the [binding](https://github.com/zmkfirmware/zmk/blob/main/app/dts/bindings/zmk%2Cphysical-layout.yaml), and if you leave it out, ZMK falls back to a `zmk,matrix-input` chosen node. Don't add a `kscan` property or a `zmk,kscan` or `zmk,matrix-transform` chosen node. Those belong to the regular kscan path and will conflict.

## Step 5: first flash

Start with a bench config: a one-layer keymap where every key types something distinct, thresholds set loose, and the `ecd` shell. Build with `-S ec-direct-shell`, or `snippet: ec-direct-shell` in `build.yaml`. Add to the `.conf`:

```
CONFIG_INPUT=y
```

`CONFIG_INPUT=y` stays in the release build too. The driver depends on it and won't build without it.

Thresholds from another board are fine to boot with. Caroline's first images used 200 / 140 copied from a bench rig.

**No key may be held at power-on.** The board takes each key's released level when it starts.

Flash, open the shell's serial port (the board shows a second one with the snippet), and run `ecd rest` with your hands off. It prints each key's resting level and noise, and flags keys that look open, held, shorted or much noisier than the rest. A whole strobe or sense line that fails together is named once, with its keys, so you can go straight to that pin.

Then run `ecd status` while you hold keys, and check that:

- Pressing a key moves only that key, and by a lot more than the idle wobble.
- A finger near a pad moves one sense line's keys together. If a group of keys moves together that shouldn't share a line, your `ain-channels` and `col-gpios` are out of step, or a jumper is wrong.

## Step 6: timing

`poll-interval-ms` sets the scan rate. The pass has to finish inside it, and the time a pass takes depends on the chip, the number of cells and `CONFIG_ZMK_KSCAN_EC_DIRECT_NAVG` (samples per key per pass).

Caroline's numbers, 36 keys:

| Controller | NAVG | Combine | Pass time | Interval |
|---|---|---|---|---|
| nRF52840 | 4 | median | ~1.5 ms | 4 ms |
| SAMD21 | 3 | median | 3.9 ms | 5 ms |
| RP2040 | 4 | mean | 1.8 ms | 4 ms |

SAMD21 is the tight one. A conversion takes 5 µs and each cell needs four, so NAVG 4 at 36 cells ran 5.1 ms and missed a 5 ms period. Caroline drops to NAVG 3, which is still a real median.

On RP2040, use `CONFIG_ZMK_KSCAN_EC_DIRECT_MEAN=y`. Its converter doesn't throw stray samples, and the mean was quieter than the median on a noisy USB supply.

For SAMD21 and RP2040, Caroline found `settle-us = <0>` read the same as 200 and made the pass five times faster. Try 0 first and compare `ecd status` against a longer settle. Both run `prime-conversions = <2>`. Dropping to one prime on SAMD21 shifted one cell's resting level by 59 counts, so keep two unless you've checked.

## Step 7: thresholds and load compensation

Do this on the fully assembled board, with plate, domes and housings. The bare PCB reads very differently. Caroline's SAMD21 build lost about 40 % of its signal going from a bare spring to the full stack.

### Load compensation

Every held key makes the other keys on its sense line read lower. On Caroline's SAMD21 build, each held key took 15–20 % off the rest of its five-key line. With all five down, Q fell from 284 to 137, and an unpressed key read about −75. On the ten-key lines it was 5–9 % per key. So a threshold that works for single presses can miss keys during fast rolls.

`load-comp-scale` corrects this, and `ecd fit` measures it. It shows the keys on one sense line. Press at least four of them one after another, as many as is comfortable, keep them all down, then let go. Holding every key on a long line doesn't make K any better. Pressing each key about as hard as the last does, since one finger's pressure moves that key's estimate by up to 20 %. Each key gives an estimate, and it moves on to the next line. Do as many lines as you like: it stops 10 s after the last key, or when every line is done. K is the median of all the estimates, applied until reset.

The good range is broad. Caroline's hand-tuned values were 3600 (nRF52840), 3850 (SAMD21) and 4000 (RP2040), each a bit under 4095. `ecd fit` later gave 3830 and 3864 on the nRF52840 and SAMD21 boards. K belongs to the board and controller, so refit it if either changes.

### Swing per key

Run `ecd calibrate` after `ecd fit`, because K changes even a lone key's reading. It asks you to press every key once, all the way down, and prints each key's swing on the board. It lists the weakest keys, keys you didn't press, and keys that need a deeper press than the rest. If keys next to a held one dipped or rose a lot, it tells you K is off and to run `ecd fit` again.

You want the weakest key. Caroline's spread before compensation:

| Controller | Weakest | Median | Strongest |
|---|---|---|---|
| nRF52840, 10 keys a line | 227 | 304 | 353 |
| SAMD21, 5 keys a line | 284 | 391 | 506 |
| RP2040, 10 keys a line | 165 | 262 | 327 |

With compensation on, the SAMD21's weakest went from 284 to 346. The compensated swings are what you set thresholds against.

### Picking thresholds

`ecd rest` gives the idle noise. Then:

- Press threshold about 40–45 % of the weakest compensated key.
- Release threshold about two thirds of the press threshold.
- Both well clear of idle noise, including with a full line held, since compensation scales noise up too (about 1.5× on Caroline).

Try them live with `ecd threshold <press> <release>`. Caroline's final values:

| Controller | Press | Release | `load-comp-scale` |
|---|---|---|---|
| nRF52840 | 110 | 70 | 3600 |
| SAMD21 | 150 | 100 | 3850 |
| RP2040 | 100 | 65 | 4000 |

Nothing set from the shell is saved. `ecd export` prints the thresholds, `load-comp-scale` and the per-key table as devicetree lines. Copy them into the overlay, with a comment saying where they came from. Caroline's overlays keep the measurements next to the values, which helps when you come back to retune.

### Check on a bad power supply

USB-powered boards should be checked on the worst charger you have. On a cheap power-strip brick, Caroline's SAMD21 idle noise went up 7–15×, with swings up to ±93. It still had no ghost presses at 150, but the margin was down to 1.8×. If you see ghost keys there, raise the thresholds or the NAVG.

## Step 8: nRF52840 power settings

Only nRF52840 has the low-power idle scan. When nothing is pressed, it raises every strobe at once and checks each sense line in one go. When a line moves past `CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE_THRESHOLD`, the full scan runs.

To set the threshold, press each key with the idle scan running and note how far its line moves, then watch the quiet maximum in `ecd status`. Caroline's presses moved 210–315 and the quiet maximum was +14, so 70 sits at a third of the weakest press and five times the quiet ceiling.

Leave `CONFIG_ZMK_SLEEP` off, which is ZMK's [default](https://zmk.dev/docs/config/power). An EC key can't wake the chip from deep sleep, and the idle scan is already the low-power state.

For battery level, use `zmk,ec-direct-battery` in place of the board's divider and point `zmk,battery` at it. The README has the node.

Caroline measured 42 µA idle and about 0.6 mA typing with BLE connected. Getting there also took the usual nRF low-power steps: console and logging off, and the XIAO's unused QSPI flash put to sleep with [zmk-feature-xiao-ble-qspi-suspend](https://github.com/carrefinho/zmk-feature-xiao-ble-qspi-suspend). Caroline's `.conf` has the list.

Set `CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE=n` if the board is always on USB.

## Step 9: the release build

Once the numbers are in the overlay, build without the `ec-direct-shell` snippet before you add your real keymap. On SAMD21, Caroline's bench shell plus its full keymap ran out of RAM by 2.7 kB.

## Checklist

- [ ] `&adc` disabled, and every default peripheral on a used pin disabled
- [ ] `ain-channels` and `col-gpios` in the same order, with the chip's own channel numbers
- [ ] SAMD21: strobes `GPIO_ACTIVE_LOW` and `bias-pull-up`
- [ ] Transform `rows` = strobes, `columns` = sense lines, entries `RC(strobe, sense)`
- [ ] Driver in the physical layout's `input`, no `kscan` property or chosen node
- [ ] Pass time fits inside `poll-interval-ms`
- [ ] Thresholds and `load-comp-scale` measured on the assembled board
- [ ] Checked on a noisy USB supply
- [ ] nRF52840: tripwire threshold set, `CONFIG_ZMK_SLEEP=n`, battery node swapped
