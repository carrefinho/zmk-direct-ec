# zmk-direct-ec

A ZMK module for electrostatic-capacitive (EC) keyboards without an analog front end.

Conventional EC boards place an analog mux, op-amp, and integrating capacitor between the matrix and the microcontroller. Direct EC omits those components: each sense line connects directly to an ADC pin, and each strobe line connects to a GPIO.

Each key couples its strobe to its sense line through a small capacitance that increases when pressed. Because the sense line is left floating, its resting voltage drifts over time. To cancel that drift, the driver samples the sense line with the strobe at rest, toggles the strobe, and samples again immediately. The difference between the two samples is the key's reading.

Because a full press produces only a few hundred ADC counts, timing between the strobe edge and the ADC sample is tight. The driver accesses the ADC registers directly rather than using Zephyr's ADC API, so Zephyr's ADC driver must be disabled on the board.

## Documentation

- Building or tuning an existing board? See the [User & Tuning Guide](docs/user-guide.md) for testing your build and adjusting actuation thresholds.
- Designing a PCB? Read the [PCB Design Guide](docs/design-guide.md) for pin selection, matrix topology, routing, and noise considerations.
- Writing a new ZMK shield? Follow the [Shield Guide](docs/shield-guide.md) for step-by-step devicetree setup, Kconfig options, load compensation, and nRF52840 power tuning.
- Curious how it works or porting a new MCU? Read the [Architecture & Porting Guide](docs/architecture.md).

## Supported controllers

The scan loop, thresholds, and calibration are shared; each chip has its own ADC backend.

| Controller | Max sense lines | ADC scan | Low-power idle | Notes |
|---|---|---|---|---|
| nRF52840 | 8 (4–6 on XIAO) | All channels in one pass | Yes (tripwire) | ~40 µA idle / ~0.6 mA typing on a 36-key BLE board (vs. 1.5 mA continuous). Requires `zmk,direct-ec-battery` instead of Zephyr's battery divider. |
| SAMD21 | 8 (11 ADC pins on XIAO) | One channel at a time (~5 µs/conv) | No (USB) | Set `bias-pull-up` and `GPIO_ACTIVE_LOW` strobes to avoid a low-voltage ADC glitch. Keep default median filter. 32 KB RAM. |
| RP2040 | 4 (GPIO26–29) | One channel at a time (~2 µs/conv) | No (USB) | No outlier samples; plain mean (`CONFIG_ZMK_KSCAN_DIRECT_EC_MEAN=y`) gives lower noise than median on noisy USB supplies. |

On nRF52840, an idle board stops scanning individual keys, raises all strobes at once, and takes a single reading per sense line. When any line crosses the tripwire threshold, the full per-key scan runs immediately. This same check runs between keystrokes and while keys are held, so the full scan only runs when something changes.

On SAMD21 and RP2040, the driver scans cell by cell: it briefly primes the sense line through the pad's internal pull resistor, releases it, samples, toggles the strobe, and samples again.

## Quick start

Add the module to your `west.yml`:

```yaml
manifest:
  remotes:
    - name: carrefinho
      url-base: https://github.com/carrefinho
  projects:
    - name: zmk-direct-ec
      remote: carrefinho
      revision: main
```

Enable `CONFIG_INPUT=y` in your shield's `.conf`, disable `&adc`, and add a `zmk,kscan-direct-ec` node to your overlay:

```dts
&adc { status = "disabled"; };

/ {
    ec_kscan: ec_kscan {
        compatible = "zmk,kscan-direct-ec";

        ain-channels = <0 1 4 5>;
        col-gpios = <&gpio0 2 GPIO_ACTIVE_HIGH>, <&gpio0 3 GPIO_ACTIVE_HIGH>,
                    <&gpio0 28 GPIO_ACTIVE_HIGH>, <&gpio0 29 GPIO_ACTIVE_HIGH>;

        strobe-gpios = <&gpio1 3 GPIO_ACTIVE_HIGH>, <&gpio1 5 GPIO_ACTIVE_HIGH>;

        press-threshold = <110>;
        release-threshold = <70>;
        poll-interval-ms = <4>;

        /* From `ec export`: one entry per cell, strobes x sense lines. */
        swing = <
            305  304  301  356
            187  398  296    0
        >;
    };
};
```

Key wiring rules:

- Sense pins are listed twice in the same order: once in `ain-channels` (AIN index on nRF52840, MUXPOS on SAMD21, AINSEL 0–3 on RP2040) and once in `col-gpios`.
- Each key with a `swing` entry actuates at `actuation-percent` (default 40) of its own swing and releases at two-thirds of that. `press-threshold` and `release-threshold` are fallbacks for keys with a swing of 0 (such as an uncalibrated board).
- The driver is a ZMK input device reporting `(strobe, sense)`. Reference it from your `zmk,physical-layout` node's `input` property (with a matrix transform where `rows` = strobes and `columns` = sense lines). Do not set a `kscan` property or a `zmk,kscan` chosen node.
- Limits: up to 8 sense lines, 16 strobes, 80 cells, and one `zmk,kscan-direct-ec` node per build.

See the [Shield Guide](docs/shield-guide.md) for full setup details, Kconfig options (`drivers/kscan/Kconfig`), devicetree properties (`dts/bindings/kscan/zmk,kscan-direct-ec.yaml`), and the nRF52840 battery node.

## Measuring a board

Signal levels depend on the PCB, domes, and plate, so thresholds must be measured on the assembled board. Build a measurement image with the `direct-ec-shell` snippet (`-S direct-ec-shell` on `west build`, or `snippet: direct-ec-shell` in `build.yaml`) to expose the `ec` commands over USB serial.

Run these in order when bringing up a new design:

| Command | Action | Output |
|---|---|---|
| `ec rest` | Hands off | Each key's resting level and noise; flags open, held, shorted, or noisy keys |
| `ec fit` | Hold keys on each line as prompted, then release | `load-comp-scale` value (applied until reset) |
| `ec calibrate` | Press every key all the way down | Per-key swing (applied until reset); flags weak, missed, or marginal keys |
| `ec export` | None | Ready-to-paste devicetree lines for `actuation-percent`, fallback thresholds, `load-comp-scale`, and `swing` table |

If you are checking a build of an existing design, `ec rest` followed by `ec calibrate` is usually enough. While a command runs, keystrokes are suppressed so they do not type into your terminal. Nothing set from the shell is persisted; copy the final values into your overlay and `.conf` and rebuild without the shell snippet for daily use.

## Limitations

- Power-on zeroing: Resting levels are captured at boot, so no key may be held while the board powers on. (`ec zero` re-captures them in the shell.)
- No stored calibration yet: `actuation-percent`, `swing`, fallback thresholds, and `load-comp-scale` are compiled into devicetree rather than saved in flash.
- No deep sleep on nRF52840: Leave `CONFIG_ZMK_SLEEP` disabled. An EC key cannot wake the chip from deep sleep, and the idle tripwire tier already brings idle draw down to tens of microamps.

## License

MIT
