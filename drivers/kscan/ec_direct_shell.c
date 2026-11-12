/*
 * Shell for zmk,kscan-ec-direct: look at the readings, re-take the baselines
 * and try thresholds on a running board.
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 *
 * Nothing set here is saved. Once the numbers are right, put them in the
 * devicetree node.
 */

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <zmk_ec_direct/ec_direct.h>

#define DT_DRV_COMPAT zmk_kscan_ec_direct

static const struct device *const dev = DEVICE_DT_INST_GET(0);

static const char *const rate_name[] = {"idle", "warm", "dormant"};

static void print_tripwire(const struct shell *sh)
{
	struct zmk_ec_direct_tripwire tw;

	zmk_ec_direct_get_tripwire(dev, &tw);
	if (!tw.enabled) {
		return;
	}
	shell_print(sh, "tier %s, tripwire %s every %u ms, trips over %u",
		    tw.tier == ZMK_EC_DIRECT_TIER_TRIPWIRE ? "tripwire" : "active",
		    rate_name[tw.rate], tw.period_ms, tw.threshold);
	shell_print(sh, "trips %u, false %u, gate misses %u, hold edges %u, backstops %u",
		    tw.trips, tw.false_trips, tw.gate_misses, tw.hold_edges, tw.hold_backstops);
	shell_print(sh, "quiet max %+d on line %u, min %+d on line %u", tw.max_pos,
		    tw.max_pos_col, tw.max_neg, tw.max_neg_col);
	shell_fprintf(sh, SHELL_NORMAL, "tripwire lines:");
	for (uint8_t c = 0; c < tw.cols; c++) {
		shell_fprintf(sh, SHELL_NORMAL, "  %d (%+d)", tw.baseline[c],
			      tw.value[c] - tw.baseline[c]);
	}
	shell_print(sh, "");
}

/* One row per strobe, one cell per sense line: baseline, then the reading
 * relative to it, then * if the key is reported pressed. */
static void print_grid(const struct shell *sh)
{
	uint8_t rows, cols;

	zmk_ec_direct_get_size(dev, &rows, &cols);
	for (uint8_t r = 0; r < rows; r++) {
		shell_fprintf(sh, SHELL_NORMAL, "strobe %2u |", r);
		for (uint8_t c = 0; c < cols; c++) {
			struct zmk_ec_direct_cell cell;

			if (zmk_ec_direct_get_cell(dev, r, c, &cell)) {
				continue;
			}
			shell_fprintf(sh, SHELL_NORMAL, " %5d %+5d%c |", cell.baseline,
				      cell.value - cell.baseline, cell.pressed ? '*' : ' ');
		}
		shell_print(sh, "");
	}
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t press, release;
	uint8_t rows, cols, stage;
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	zmk_ec_direct_get_init_status(dev, &stage, &rc);
	if (!device_is_ready(dev)) {
		shell_error(sh, "driver failed to start: init stage %u, rc %d", stage, rc);
		return -ENODEV;
	}
	/* The grid is not scanned while the tripwire holds. */
	rc = zmk_ec_direct_force_pass(dev);
	if (rc) {
		shell_warn(sh, "no fresh pass (%d), showing the last one", rc);
	}
	zmk_ec_direct_get_size(dev, &rows, &cols);
	zmk_ec_direct_get_thresholds(dev, &press, &release);
	shell_print(sh, "%u strobes x %u sense lines, press over %u, release under %u, comp %u",
		    rows, cols, press, release, zmk_ec_direct_get_load_comp(dev));
	print_tripwire(sh);
	print_grid(sh);
	return 0;
}

static int cmd_calibrate(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	rc = zmk_ec_direct_calibrate(dev);
	if (rc) {
		shell_error(sh, "calibration did not finish (%d)", rc);
		return rc;
	}
	shell_print(sh, "released baselines re-taken");
	return 0;
}

static int cmd_threshold(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t press, release;

	if (argc == 3) {
		press = (uint16_t)strtoul(argv[1], NULL, 0);
		release = (uint16_t)strtoul(argv[2], NULL, 0);
		if (release >= press) {
			shell_error(sh, "release must be below press");
			return -EINVAL;
		}
		zmk_ec_direct_set_thresholds(dev, press, release);
	} else if (argc != 1) {
		shell_error(sh, "threshold <press> <release>");
		return -EINVAL;
	}
	zmk_ec_direct_get_thresholds(dev, &press, &release);
	shell_print(sh, "press over %u, release under %u", press, release);
	return 0;
}

static int cmd_comp(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 2) {
		zmk_ec_direct_set_load_comp(dev, (uint16_t)strtoul(argv[1], NULL, 0));
	}
	shell_print(sh, "load-comp-scale %u", zmk_ec_direct_get_load_comp(dev));
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_ecd,
	SHELL_CMD_ARG(status, NULL,
		      "Every cell's baseline and reading (* = pressed), and the tripwire state",
		      cmd_status, 1, 0),
	SHELL_CMD_ARG(calibrate, NULL, "Re-take the released baselines. Every key must be up",
		      cmd_calibrate, 1, 0),
	SHELL_CMD_ARG(threshold, NULL, "Show, or set until reset: threshold <press> <release>",
		      cmd_threshold, 1, 2),
	SHELL_CMD_ARG(comp, NULL,
		      "Show, or set until reset, the load compensation scale: comp <K>, 0 = off",
		      cmd_comp, 1, 1),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ecd, &sub_ecd, "EC matrix", NULL);
