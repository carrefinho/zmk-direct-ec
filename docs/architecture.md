# Architecture and porting guide

This document explains how `zmk-direct-ec` works under the hood, how each controller backend acquires samples, and the bench methodology for bringing up a new MCU.

## Sensing physics and signal processing

### Floating-line differential charge-share

Each key sits at the intersection of a strobe line (push-pull GPIO) and a sense line (ADC input). With no external integrating capacitor, the sense line's own stray capacitance (`C_tot`, tens of picofarads) forms a capacitive divider with the key (`C_key`):

```
delta ≈ VDD × C_key / (C_tot + C_s)
```

where `C_s` is the ADC's sample-and-hold capacitance.

Because the sense line floats at high impedance, its resting voltage drifts over time by more than a keypress. Every measurement is therefore differential: take a reference sample with the strobe at rest, toggle the strobe, and take a test sample immediately. Subtracting the reference cancels slow drift, leaving only the capacitive step caused by the strobe edge.

### Filtering and debouncing

Each scan pass takes `CONFIG_ZMK_KSCAN_DIRECT_EC_NAVG` readings per cell (default 4) and reduces them with the configured estimator:

- Median (default): rejects single-sample outliers outright. Required on SAMD21.
- Trimmed mean: drops the highest and lowest sample and averages the rest (equivalent to median below NAVG 4).
- Mean: lowest noise when the ADC never produces outliers and supply noise is narrowband, such as RP2040 over USB.

`CONFIG_ZMK_KSCAN_DIRECT_EC_CONFIRM_PASSES` (default 2) requires a new press or release state to hold across consecutive passes before reporting an input event, preventing chatter when a key is held right at its threshold.

### Sense-line load compensation

When a key is held down, its increased capacitance adds to `C_tot` for the entire sense line, reducing the reading of every other key on that line by 5–20% per held key.

When `load-comp-scale` (`K`) is non-zero, the driver scales every reading on a sense line after each pass by:

```
scale = (K - sum(baselines)) / (K - sum(readings))
```

where `K` (in ADC counts) represents what a key would read if it were the line's entire capacitance. This requires no knowledge of which keys are currently pressed, restores released keys to their baseline, and keeps pressed keys within ~3% from one held key to ten. The trade-off is that noise is multiplied by the same factor (up to ~1.5–1.8× when a full line is held).

### Per-key actuation thresholds

Because mechanical seating and trace capacitance vary across a board, a single global count threshold leaves strong keys actuating near the top of their travel and weak keys near the bottom.

When a cell has a non-zero `swing` entry (measured compensated by `ec calibrate`), the driver computes its press point as `swing × actuation-percent / 100` (default 40%) and its release point at two-thirds of that. Cells with `swing = 0` fall back to `press-threshold` and `release-threshold`, allowing an uncalibrated board to type immediately.

## Shared scan thread and backend split

The scan loop, filtering, load compensation, hysteresis, and ZMK input reporting live in `drivers/kscan/kscan_direct_ec.c`. The scan runs in a dedicated Zephyr thread (rather than a timer ISR) so it does not block Bluetooth/MPSL interrupts while waiting on ADC conversions.

Hardware acquisition is split into two families:

- `backend_nrf.c` — multi-channel SAADC scan mode plus the low-power tripwire state machine (nRF52840).
- `backend_pair.c` — generic per-cell prime/release/sample loop shared by single-channel ADCs (`backend_samd21.c`, `backend_rp2040.c`).

## nRF52840 backend (`backend_nrf.c`)

### Per-key pass

The nRF52840 SAADC converts all sense lines in a single scan sequence (0.6 V internal reference, 1/6 gain for 3.6 V full scale, 3 µs acquisition time). Sense lines are configured as analog inputs once and never driven; repeated SAADC sampling self-biases the floating node.

Because sampling a pressed key leaves its sense line slightly depressed for the next few scans, a single reference scan at the start of a 10-strobe pass is not enough. The driver interleaves fresh reference scans every `CONFIG_ZMK_KSCAN_DIRECT_EC_REF_INTERVAL` strobes (default 2, so every strobe is adjacent to a reference).

The SAADC EasyDMA buffer is static rather than stack-allocated to avoid nRF52840 anomaly 246 (which otherwise leaves a ~400 µA current floor between passes). The driver arms the SAADC once per pass and waits on a latched event rather than counting `DONE` events, avoiding stalls under BLE preemption.

### Low-power tripwire state machine

Continuous per-key scanning draws ~1.5 mA on a 36-key board. When `CONFIG_ZMK_KSCAN_DIRECT_EC_TRIPWIRE` is enabled, the driver uses an all-strobes-at-once tripwire pass—taking one ref/test pair per sense line back-to-back—to gate the full per-key scan:

1. Idle (`TRIPWIRE_PERIOD_MS`, default 33 ms): checks all sense lines at 30 Hz. Any line crossing `TRIPWIRE_THRESHOLD` immediately triggers a full per-key pass in the same slot, so the first keypress is not lost.
2. Warm (`TW_WARM_HOLD_MS`, default 10 s): after a confirmed key, the tripwire runs at the active poll rate (`TW_WARM_PERIOD_MS`, default 4 ms) with a tighter threshold (`TW_WARM_THRESHOLD`, default 40). Between keystrokes in a typing burst, the full per-key pass only runs when a line actually moves.
3. Hold watch (`TW_HOLD_WATCH`): once the set of held keys is steady for `TW_HOLD_SETTLE_PASSES` passes, the driver switches back to the tripwire pass against a baseline of the held state, watching for a rise (`TW_HOLD_EDGE`) or drop (`TW_HOLD_RELEASE_EDGE`) with a periodic backstop (`TW_HOLD_BACKSTOP_MS`).
4. Dormant (`TW_DORMANT_AFTER_MS`, default 30 min): slows the idle tripwire period to `TW_DORMANT_PERIOD_MS` (default 50 ms).

Taking each sense line's ref/test pair back-to-back (interleaved) rather than converting all references then all tests eliminates SAADC input-mux switching glitches on floating nodes, which otherwise dominate quiet positive excursions on mains-earthed hosts.

The tripwire baseline slowly tracks quiet passes (`TRIPWIRE_TRACK_SHIFT`, default 8) to absorb drift without following tripped passes.

### Battery sensing (`battery_nrf.c`)

Because `backend_nrf.c` owns the SAADC, Zephyr's `zmk,battery-voltage-divider` cannot run alongside it. Instead, `zmk,direct-ec-battery` borrows the SAADC inside the scan thread right after a tripwire pass (every `BATTERY_INTERVAL_S`, default 30 s) and caches the result for ZMK's sensor API.

## Pair-pass backends (`backend_pair.c`)

Parts without a multi-channel scan-mode ADC convert one cell at a time:

```
bias_on(col)          // engage weak pull (down, or up with bias-pull-up)
wait bias-us
prime x convert(col)  // throwaway conversions to bring node to a known level
bias_off(col)         // release pull BEFORE ref so the step doesn't bleed away
ref  = convert(col)
strobe row active     // high, or low for active-low strobes
test = convert(col)   // immediately after strobe edge
strobe row inactive
diff = test - ref     // negated for active-low strobes so presses stay positive
wait settle-us
```

A new SoC backend only needs to implement `struct ec_pair_ops` (`init`, `bias_on`, `bias_off`, `convert`, plus an optional fused `cell` op when per-step call overhead dominates conversion time).

### SAMD21 (`backend_samd21.c`)

- Pull in analog mode: the internal pull does not survive `PMUXEN=1`, but the ADC still converts with `PMUXEN=0`, where the pull remains active.
- Low-voltage glitch: on rev D silicon, high-impedance conversions occasionally read ~75 counts low, with probability rising as the input voltage drops (16% near 500 counts, 0% above 3600). Priming with `bias-pull-up` parks the node near VDD and pairs with `GPIO_ACTIVE_LOW` (falling-edge) strobes so presses stay in the glitch-free region.
- Timing: conversions take ~5 µs each, so SAMD21 implements the fused `cell` op with direct port register writes to avoid per-step GPIO overhead.

### RP2040 (`backend_rp2040.c`)

- Clean bias path: internal pulls work in all mux states, conversions take a fixed 2 µs, and no single-shot glitches were observed across thousands of primed pairs.
- Estimator: because there are no outlier samples, `CONFIG_ZMK_KSCAN_DIRECT_EC_MEAN=y` gives lower noise than median on noisy USB supplies.
- Channel limit: only 4 ADC inputs (GPIO26–29), limiting boards to 4 sense lines.

## Methodology for bringing up a new MCU

Nine MCUs across six vendors (nRF52840, nRF54L15, SAMD21, RP2040, STM32G431, STM32G474, CH32V303, CH32X035, MCXA366) were benched on silicon during development, and none failed on analog grounds. Datasheet sample-and-hold capacitance (`C_s`) did not reliably predict behavior—even CH32X035 (21 pF S/H) worked cleanly once its bias recipe was found.

Rather than screening chips from datasheets, pick an MCU for its ADC pin count, USB/BLE, RAM/flash, and cost, then spend one bench session establishing its recipe:

1. Check the bias path: test whether the pad's internal pull-down/pull-up survives while the ADC is connected (true on STM32G4, MCXA366, RP2040, and SAMD21 with `PMUXEN=0`), or whether the pin must be briefly driven/pulled digitally and released to analog mode before each pair (CH32V303).
2. Check first-conversion and S/H carryover: compare 0, 1, and 2 throwaway `prime-conversions` (or a grounded dummy channel read). On several parts, the first conversion after idle arrives with stale S/H charge and doubles noise or shifts the baseline.
3. Sweep the input voltage range on a high-Z node: look for one-sided outlier glitches (like SAMD21's low-voltage dip). If glitches cluster near ground, switch to pull-up priming with active-low (falling-edge) strobes.
4. Check multi-channel mux kicks: if using a block pass across multiple sense lines, verify whether converting other channels inside a node's ref→test window injects tail glitches (observed on nRF52840, STM32G474, and MCXA366). If so, take ref/test pairs back-to-back per sense line.
5. Test on both battery and a noisy USB charger: USB supplies dominate noise (3.5–4× on a mains-earthed host, up to 20–40× on a cheap switching brick). Verify that your estimator choice (median vs. mean) and `NAVG` keep ample margin on a bad supply.
6. Implement `struct ec_pair_ops`: if the chip uses a single-channel ADC, plug it into `backend_pair.c` (adding the fused `cell` op if per-step overhead threatens `poll-interval-ms`).
