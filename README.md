# zmk-ec-direct

A ZMK module for electrostatic-capacitive keyboards that have no analog front end.

A conventional EC board puts an analog mux, an op-amp and an integrating capacitor between the matrix and the microcontroller. Direct EC leaves all of that out. Each sense line of the matrix is wired straight to an ADC pin and each strobe line to a GPIO, and the microcontroller's own ADC does the rest.

It works because a key couples its strobe to its sense line through a small capacitance that grows as the key goes down. The sense line is left floating. The driver samples it once with the strobe at rest, moves the strobe, and samples it again right away. The step between the two samples is the key's reading, and it is larger when the key is pressed. Taking a fresh reference before every edge is what makes a floating line usable, since its resting level drifts by more than a press over time.

The signal is small, a few hundred ADC counts for a full press, so the timing between the strobe edge and the sample matters a great deal. For that reason the driver does not use Zephyr's ADC API. It drives the converter's registers directly, and Zephyr's ADC driver has to be disabled on the board.

## Supported controllers

The module runs on nRF52840, SAMD21 and RP2040. The scan loop, thresholds and calibration are shared, and each chip has its own backend for the part that touches the ADC. The backends differ enough that it is worth knowing which one you are on.

On nRF52840 the SAADC converts all sense lines in one go, so a pass is one scan with the strobes at rest, then one scan per strobe, with extra reference scans in between (every second strobe by default). The sense lines are never driven. This is the only backend with the low-power tier. An idle board stops scanning individual keys, raises every strobe at once, and takes a single reading per sense line. That is enough to tell that something is pressed somewhere, and when a line trips, the full per-key pass runs immediately and reports the press. The same cheap pass also runs between keystrokes while typing and while keys are held, so the full pass only runs when something changes. On a 36-key board with BLE connected this measured about 40 µA idle and about 0.6 mA while typing, against 1.5 mA for scanning continuously. Because the driver owns the SAADC, ZMK's normal battery driver cannot be used; the module has its own battery node, described below.

On SAMD21 and RP2040 the ADC converts one channel at a time, so the pass goes cell by cell: the sense line is briefly pulled through the pad's internal pull resistor to bring it to a known level, released, converted, the strobe is moved, and it is converted again. These two scan continuously with no idle tier, so they are meant for boards that run from USB.

SAMD21 has a quirk. Some conversions of a high-impedance line read about 75 counts low, and it happens more often the lower the line sits. Priming with the pull-up instead of the pull-down keeps the line near the top of the range, where it does not happen. So on SAMD21 set `bias-pull-up` and declare the strobes `GPIO_ACTIVE_LOW`, and keep the default median. A conversion takes about 5 µs, which limits how many cells and samples fit in a scan period, and the chip has 32 KB of RAM.

RP2040 has only four ADC inputs, so a board is limited to four sense lines. Its converter is fast, 2 µs a conversion, and does not throw stray samples, so a plain mean of the samples gives lower noise than a median, especially on a noisy USB supply.

## Configuration

Add the module to your `west.yml`:

```yaml
manifest:
  remotes:
    - name: carrefinho
      url-base: https://github.com/carrefinho
  projects:
    - name: zmk-ec-direct
      remote: carrefinho
      revision: main
```

Then describe the matrix in the shield's overlay. The sense pins are listed twice, once as ADC channel numbers and once as GPIOs in the same order, because devicetree cannot express both roles in one property. The channel number is the AIN index on nRF52840, the MUXPOS value on SAMD21 and the AINSEL value (0 to 3) on RP2040.

```dts
&adc { status = "disabled"; };

/ {
    ec_kscan: ec_kscan {
        compatible = "zmk,kscan-ec-direct";

        ain-channels = <0 1 4 5>;
        col-gpios = <&gpio0 2 GPIO_ACTIVE_HIGH>, <&gpio0 3 GPIO_ACTIVE_HIGH>,
                    <&gpio0 28 GPIO_ACTIVE_HIGH>, <&gpio0 29 GPIO_ACTIVE_HIGH>;

        strobe-gpios = <&gpio1 3 GPIO_ACTIVE_HIGH>, <&gpio1 5 GPIO_ACTIVE_HIGH>;

        press-threshold = <110>;
        release-threshold = <70>;
        poll-interval-ms = <4>;
    };
};
```

The driver is an input device that reports keys at (strobe, sense line). Reference it from the `input` property of your `zmk,physical-layout` node, with a matrix transform whose `rows` is the number of strobes and `columns` the number of sense lines. Do not also set a `kscan` property or a `zmk,kscan` chosen node. A matrix can have up to 8 sense lines, 16 strobes and 80 cells, and a build can have one such node.

`press-threshold` and `release-threshold` are in ADC counts above each key's released level. `load-comp-scale` corrects for the fact that every held key adds capacitance to its sense line and makes the other keys on that line read lower; it is off by default and worth fitting on boards with many keys per line. The remaining properties (`bias-us`, `prime-conversions`, `settle-us`, `bias-pull-up`) only apply to SAMD21 and RP2040. The driver skips any cell the matrix transform doesn't list; `sense-strobe-masks` overrides that, and is rarely needed. All of them are documented in `dts/bindings/kscan/zmk,kscan-ec-direct.yaml`.

The Kconfig options are in `drivers/kscan/Kconfig`. The ones most boards will touch are `CONFIG_ZMK_KSCAN_EC_DIRECT_NAVG`, the number of samples taken per key per pass (default 4), the choice of how those samples are combined (median, trimmed mean or mean), and on nRF52840 `CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE_THRESHOLD`, which sets how far a sense line has to move to wake the per-key scan. Set `CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE=n` for an nRF52840 board that is always on USB.

For battery level on nRF52840, add a `zmk,ec-direct-battery` node in place of `zmk,battery-voltage-divider` and point the `zmk,battery` chosen node at it. It takes the same divider description:

```dts
ec_battery: ec_battery {
    compatible = "zmk,ec-direct-battery";
    ain-channel = <7>;
    power-gpios = <&gpio0 14 (GPIO_OPEN_DRAIN | GPIO_ACTIVE_LOW)>;
    output-ohms = <510000>;
    full-ohms = <1510000>;
};
```

The scan thread reads the divider between idle passes, so the value does not update while a key is held down.

## Finding the thresholds

Signal levels depend on the PCB, the domes and the plate, so thresholds have to be measured on the actual board. Build with `CONFIG_SHELL=y` and a console (USB CDC ACM works) to get the `ecd` commands.

`ecd status` prints every key's released baseline and its current reading relative to that baseline. Run it with nothing pressed to see the idle noise, then with each key held to find the weakest one. A press threshold around 40 % of the weakest key, with the release threshold at about two thirds of that, is a reasonable starting point. `ecd threshold <press> <release>` and `ecd comp <K>` apply values to the running board so you can try them, and `ecd calibrate` takes the released baselines again. On nRF52840, `ecd status` also shows what each sense line reads in the idle pass and the largest reading seen while quiet; the tripwire threshold should sit several times above that quiet maximum and well below what the weakest key adds.

Nothing set from the shell is stored. Once the numbers are right, put them in the overlay and the shield's `.conf`.

## Limitations

The released baselines are taken at power-on, so no key may be held while the board starts. There is no stored calibration yet, and the thresholds are one pair of values for the whole board rather than per key.

On nRF52840, leave `CONFIG_ZMK_SLEEP` off. An EC key cannot wake the chip from deep sleep, and the idle tier already brings the scan cost down to tens of microamps.

## License

MIT
