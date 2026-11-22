# User & tuning guide

This guide is for people building or using an existing direct-EC keyboard. If you are designing a new PCB or writing a ZMK shield from scratch, see the [PCB Design Guide](design-guide.md) and [Shield Guide](shield-guide.md).

## Power-on zeroing

Resting levels are captured at boot. **Do not hold any key while plugging in or powering on the keyboard.** If a key was held at boot, power-cycle the board (or run `ec zero` in the shell).

## Checking a new build

To verify that every key, dome, and spring is working, build a temporary test image with the `direct-ec-shell` snippet:

- Local `west build`: add `-S direct-ec-shell`
- GitHub Actions (`build.yaml`): add `snippet: direct-ec-shell`

Flash the board and open the shell's USB serial port (the snippet adds a second serial port alongside normal typing). While an `ec` command runs, keystrokes are suppressed so they do not type into your terminal.

### 1. Check resting state (`ec rest`)

Keep your hands off the board and run:

```
ec rest
```

It prints each key's resting level and noise, flagging keys that look open, held, shorted, or unusually noisy. If an entire row or column fails together, it is reported as a group.

### 2. Check every key's swing (`ec calibrate`)

Run:

```
ec calibrate
```

1. Press every key all the way down. Once per key is enough, though a few presses per key gives a steadier number (the command uses the median of the last four presses, ignoring light touches under half the peak).
2. Wait 5 seconds after the last key; the command finishes automatically, prints each key's swing, and applies the new swings live until reset so you can test typing right away.

The output flags weak keys, keys you missed, and marginal keys (any key whose press point is under 3× its own noise from `ec rest`). A poorly seated spring can look fine by eye while reading less than half as strong as its neighbors—if a single key reads unusually low, reseat its spring and dome and re-run `ec calibrate`.

## Adjusting actuation or swapping domes

Each key with a calibrated `swing` actuates at `actuation-percent` (default 40) of its own swing and releases at two-thirds of that. At 40, a slow press trips partway through the dome's collapse. 30 is roughly the lowest practical setting: a hard lean on a 28 g dome reached 16% of its swing, and the release point at 30 is 20%.

You can test a different actuation depth live before rebuilding:

```
ec threshold 35%
```

Shell changes are not saved across reboots. Once you like the feel, override `actuation-percent` in your `zmk-config` board/shield `.overlay`:

```dts
&ec_kscan {
    actuation-percent = <35>;
};
```

If you swap domes or reseat keys, a key's swing will change—re-run `ec calibrate` and `ec export` to update the `swing` table in your overlay.

## Rebuild for daily use

When you are done testing, remove `direct-ec-shell` from your build and reflash. The shell image is only for measurement and uses extra RAM (on SAMD21, the shell plus a full keymap can run out of RAM).

## Troubleshooting

- Ghost presses on a specific USB charger or hub: USB power supplies vary widely in noise. A noisy charger can raise idle noise 7–15×. If you only see ghost keys on one supply, try a different cable/port or raise `actuation-percent` in your overlay.
- One key feels much weaker than the rest: Reseat its spring and dome, then re-check with `ec calibrate`.
- Board stops registering a key after boot: Make sure no key was held at power-on and power-cycle the board.
