# zmk-direct-ec

A ZMK module for electrostatic-capacitive (EC) keyboards without an analog front end.

Conventional EC boards measure the change in capacitance per key through an analog mux and op-amp before the MCU's ADC input, with an additional integrating capacitor. Direct EC does not use any of those components: each sense line connects directly to an ADC input pin, and strobe lines connect to any ordinary GPIO.

This means an ultra-minimal bill of materials on the PCB side, even cheaper to build than a conventional mechanical board (no diodes or sockets), while also enabling very low power wireless operation that was previously hard to achieve on EC keyboards.

## Getting Started

- Building or calibrating an existing board
  - See the [User & Tuning Guide](docs/user-guide.md) on how to test and calibrate your build and to adjust actuation thresholds.
- Designing a PCB with Direct EC
  - Read the [PCB Design Guide](docs/design-guide.md) on choosing an MCU, selecting pins, and routing considerations.
- Writing a ZMK shield for a new PCB
  - Follow the [Shield Guide](docs/shield-guide.md) for step-by-step devicetree setup, Kconfig options, and explainers on Direct EC specific quirks.
- Curious how it works or porting to a new MCU
  - Read the [Architecture & Porting Guide](docs/architecture.md), which includes my methodology for bringing up new chips.

## Supported MCUs

- nRF52840: up to 8 sense lines, wired or BLE.
  - Features a low-power tripwire scan (~40 µA idle / ~0.6 mA typing on a 36-key vs. ~1.5 mA continuous) that raises all strobes at once to check each sense line in one pass, waking the full scan immediately when a line moves. The tripwire operates at 30 Hz, so it still feels responsive and doesn't introduce excessive first-key wake-up delay.
- SAMD21: up to 8 sense lines, wired (USB).
- RP2040: up to 4 sense lines, wired (USB).

## Usage

Add `zmk-direct-ec` to your `west.yml`:

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

See the [Shield Guide](docs/shield-guide.md) on how to set up the devicetree overlay, Kconfig options (`drivers/kscan/Kconfig`), and devicetree binding details (`dts/bindings/kscan/zmk,kscan-direct-ec.yaml`).

## Limitations

- Power-on zeroing: Resting levels are captured at boot, so make sure you don't accidentally press a key when powering on the board (`ec zero` in the shell re-captures them).
- No stored calibration yet: `actuation-percent`, `swing`, fallback thresholds, and `load-comp-scale` are set in devicetree at compile time.
- No deep sleep on nRF52840: Leave `CONFIG_ZMK_SLEEP` disabled. It's not possible to wake up the chip with an EC matrix. Since tripwire already enables sufficiently low power consumption, just embrace and enjoy the always-on BLE experience!

## License

MIT
