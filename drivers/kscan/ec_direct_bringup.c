/*
 * Bring-up commands for zmk,kscan-ec-direct: what a board designer runs once
 * to characterise a design, and what a builder runs to check an assembly.
 *
 *   ecd check      hands off: resting level and noise of every key
 *   ecd comp fit   a row of held keys: the load compensation scale
 *   ecd survey     one press per key: every key's swing
 *   ecd report     all of it, ready to paste
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 *
 * Every command puts the driver in bench mode, so the keys being pressed are
 * not typed into the terminal, and every command ends by itself: a key on the
 * board under test cannot reach the shell to stop it.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include <zmk_ec_direct/ec_direct.h>

#include "ec_direct_priv.h"
#include "ec_direct_shell.h"

/* Passes per figure in `check`: about a second at 250 Hz. */
#define CHECK_PASSES 256
/* Passes that set how far a quiet key wanders, before survey and comp fit. */
#define QUIET_PASSES 64
/* A key counts as down above 4x that, inside these bounds. */
#define DETECT_MIN 30
#define DETECT_MAX 120
/* Survey works in blocks of passes and keeps each block's lowest reading, so
 * a swing is a level the key held for a whole block and not one high sample. */
#define BLOCK        4
#define PRESS_BLOCKS 3
/* comp fit: passes a set of held keys has to stay the same before it is read. */
#define STABLE_PASSES 25
/* A key has to drop by this much between its two states to give a K. */
#define FIT_MIN_DROP 8

#define SURVEY_IDLE_MS 20000
#define FIT_FIRST_MS   30000
#define FIT_IDLE_MS    10000
#define FIT_CAP_MS     180000

/* Keymap positions, from the chosen layout's transform: entry p is
 * RC(strobe, sense line) of position p. */
#if DT_HAS_CHOSEN(zmk_physical_layout) &&                                                          \
	DT_NODE_HAS_PROP(DT_CHOSEN(zmk_physical_layout), transform)
static const uint16_t pos_map[] = DT_PROP(DT_PHANDLE(DT_CHOSEN(zmk_physical_layout), transform), map);
#define POS_COUNT ARRAY_SIZE(pos_map)
#else
static const uint16_t pos_map[1];
#define POS_COUNT 0
#endif

/* One pass. Between passes it doubles as sort scratch. */
static int16_t snap[EC_MAX_CELLS];

/* Working arrays. Only one command runs at a time, so they share. */
static union {
	struct {
		int32_t sum[EC_MAX_CELLS];
		int16_t lo[EC_MAX_CELLS];
		int16_t hi[EC_MAX_CELLS];
	} check;
	struct {
		int16_t base[EC_MAX_CELLS];
		int16_t lo[EC_MAX_CELLS];   /* lowest reading of the current block */
		int16_t peak[EC_MAX_CELLS]; /* highest block of the current press */
		uint8_t blocks[EC_MAX_CELLS];
	} survey;
	struct {
		int16_t base[EC_MAX_CELLS];
		int16_t v1[EC_MAX_CELLS];  /* reading when the key first went down */
		uint16_t s1[EC_MAX_CELLS]; /* its line's sum then */
		int16_t v2[EC_MAX_CELLS];  /* reading with the most keys held */
		uint16_t est[EC_MAX_CELLS];
	} fit;
} w;

/* What the commands found, kept for `report`. */
static struct {
	uint8_t rows, cols;
	uint16_t keys;
	uint16_t mask[EC_MAX_COLS]; /* per sense line, bit r = strobe r is a key */
	bool have_check, have_survey;
	int16_t rest[EC_MAX_CELLS];
	int16_t swing[EC_MAX_CELLS]; /* 0 = not pressed in the survey */
	uint16_t pp_median, pp_worst;
	uint16_t sd10_median, sd10_worst;
	uint16_t fit_k, fit_keys;
} res;

#define CELL(r, c)   ((size_t)(r) * res.cols + (c))
#define IS_KEY(r, c) ((res.mask[c] & BIT(r)) != 0)
#define FOR_KEYS(r, c)                                                                             \
	for (uint8_t r = 0; r < res.rows; r++)                                                     \
		for (uint8_t c = 0; c < res.cols; c++)                                             \
			if (IS_KEY(r, c))

/* --- helpers --------------------------------------------------------- */

static int position(uint8_t r, uint8_t c)
{
	for (size_t p = 0; p < POS_COUNT; p++) {
		if (pos_map[p] == ((r << 8) | c)) {
			return p;
		}
	}
	return -1;
}

/* One at a time: the result is a static buffer. */
static const char *label(size_t i)
{
	static char buf[40];
	uint8_t r = i / res.cols, c = i % res.cols;
	int p = position(r, c);

	if (p >= 0) {
		snprintk(buf, sizeof(buf), "key %d (strobe %u, line %u)", p, r, c);
	} else {
		snprintk(buf, sizeof(buf), "strobe %u, line %u", r, c);
	}
	return buf;
}

static int pass(const struct shell *sh, const struct device *dev)
{
	int rc = zmk_ec_direct_read_pass(dev, snap, K_MSEC(2500));

	if (rc) {
		shell_error(sh, "no pass from the scan thread (%d)", rc);
	}
	return rc;
}

/*
 * Wait until nothing has moved for a moment. The Enter that started the
 * command may be on this board and still on its way up.
 */
static int settle(const struct shell *sh, const struct device *dev)
{
	int64_t t0 = k_uptime_get();
	int still = 0;
	int rc = pass(sh, dev);

	if (rc) {
		return rc;
	}
	memcpy(w.survey.lo, snap, sizeof(snap));
	while (still < 100 && k_uptime_get() - t0 < 5000) {
		rc = pass(sh, dev);
		if (rc) {
			return rc;
		}
		still++;
		FOR_KEYS(r, c) {
			size_t i = CELL(r, c);

			if (abs(snap[i] - w.survey.lo[i]) > 25) {
				still = 0;
			}
		}
		if (!still) {
			memcpy(w.survey.lo, snap, sizeof(snap));
		}
	}
	if (still < 100) {
		shell_warn(sh, "the readings never settled; going on anyway");
	}
	return 0;
}

static int begin(const struct shell *sh, const struct device *dev)
{
	if (!device_is_ready(dev)) {
		shell_error(sh, "driver failed to start, see `ecd status`");
		return -ENODEV;
	}
	zmk_ec_direct_get_size(dev, &res.rows, &res.cols);
	res.keys = 0;
	for (uint8_t c = 0; c < res.cols; c++) {
		res.mask[c] = 0;
		for (uint8_t r = 0; r < res.rows; r++) {
			struct zmk_ec_direct_cell cell;

			zmk_ec_direct_get_cell(dev, r, c, &cell);
			if (cell.scanned) {
				res.mask[c] |= BIT(r);
				res.keys++;
			}
		}
	}
	if (!res.keys) {
		shell_error(sh, "no keys in the matrix");
		return -ENODEV;
	}
	zmk_ec_direct_bench(dev, true);
	return 0;
}

static void end(const struct device *dev)
{
	zmk_ec_direct_bench(dev, false);
}

/* Fresh released baselines into `base`, and how far a quiet key wanders. */
static int rebase(const struct shell *sh, const struct device *dev, int16_t *base,
		  int16_t *detect)
{
	int16_t worst = 0;
	int rc = zmk_ec_direct_calibrate(dev);

	if (rc) {
		shell_error(sh, "baselines were not re-taken (%d)", rc);
		return rc;
	}
	FOR_KEYS(r, c) {
		struct zmk_ec_direct_cell cell;

		zmk_ec_direct_get_cell(dev, r, c, &cell);
		base[CELL(r, c)] = cell.baseline;
	}
	for (int n = 0; n < QUIET_PASSES; n++) {
		rc = pass(sh, dev);
		if (rc) {
			return rc;
		}
		FOR_KEYS(r, c) {
			size_t i = CELL(r, c);

			worst = MAX(worst, abs(snap[i] - base[i]));
		}
	}
	*detect = CLAMP(4 * worst, DETECT_MIN, DETECT_MAX);
	if (4 * worst > DETECT_MAX) {
		shell_warn(sh, "noisy here: quiet keys move by %d. A battery or a quieter "
			       "supply gives better numbers", worst);
	}
	return 0;
}

static int cmp_i16(const void *a, const void *b)
{
	return *(const int16_t *)a - *(const int16_t *)b;
}

/* Median of v over the keys, skipping zeros if asked. Uses `snap`. */
static int16_t median(const int16_t *v, bool skip_zero, uint16_t *count)
{
	uint16_t n = 0;

	FOR_KEYS(r, c) {
		int16_t x = v[CELL(r, c)];

		if (x || !skip_zero) {
			snap[n++] = x;
		}
	}
	if (count) {
		*count = n;
	}
	if (!n) {
		return 0;
	}
	qsort(snap, n, sizeof(snap[0]), cmp_i16);
	return snap[n / 2];
}

static uint32_t isqrt(uint32_t v)
{
	uint32_t r = 0, bit = 1UL << 30;

	while (bit > v) {
		bit >>= 2;
	}
	while (bit) {
		if (v >= r + bit) {
			v -= r + bit;
			r = (r >> 1) + bit;
		} else {
			r >>= 1;
		}
		bit >>= 2;
	}
	return r;
}

/* One row per strobe, one column per sense line. */
static void print_grid(const struct shell *sh, const int16_t *v, bool skip_zero)
{
	for (uint8_t r = 0; r < res.rows; r++) {
		shell_fprintf(sh, SHELL_NORMAL, "strobe %2u |", r);
		for (uint8_t c = 0; c < res.cols; c++) {
			int16_t x = v[CELL(r, c)];

			if (!IS_KEY(r, c)) {
				shell_fprintf(sh, SHELL_NORMAL, "     .");
			} else if (!x && skip_zero) {
				shell_fprintf(sh, SHELL_NORMAL, "    --");
			} else {
				shell_fprintf(sh, SHELL_NORMAL, " %5d", x);
			}
		}
		shell_print(sh, "");
	}
}

/* The same as a devicetree array; cells that are not keys are 0. */
static void print_array(const struct shell *sh, const char *name, const int16_t *v)
{
	shell_print(sh, "%s = <", name);
	for (uint8_t r = 0; r < res.rows; r++) {
		shell_fprintf(sh, SHELL_NORMAL, "   ");
		for (uint8_t c = 0; c < res.cols; c++) {
			shell_fprintf(sh, SHELL_NORMAL, " %4d", IS_KEY(r, c) ? v[CELL(r, c)] : 0);
		}
		shell_print(sh, "");
	}
	shell_print(sh, ">;");
}

/* --- check ----------------------------------------------------------- */

/* Name the cells in `bad` (per sense line, bit r = strobe r): a whole sense
 * line or strobe once, the rest one by one. */
static void print_bad(const struct shell *sh, const uint16_t *bad, const char *what)
{
	uint16_t whole_strobes = 0;

	for (uint8_t r = 0; r < res.rows; r++) {
		bool all = true, any = false;

		for (uint8_t c = 0; c < res.cols; c++) {
			if (IS_KEY(r, c)) {
				any = true;
				all &= (bad[c] & BIT(r)) != 0;
			}
		}
		if (any && all) {
			whole_strobes |= BIT(r);
			shell_warn(sh, "strobe %u: every key %s. Check that pin", r, what);
		}
	}
	for (uint8_t c = 0; c < res.cols; c++) {
		if (bad[c] && bad[c] == res.mask[c]) {
			shell_warn(sh, "sense line %u: every key %s. Check that pin", c, what);
			continue;
		}
		for (uint8_t r = 0; r < res.rows; r++) {
			if ((bad[c] & BIT(r)) && !(whole_strobes & BIT(r))) {
				shell_warn(sh, "%s %s", label(CELL(r, c)), what);
			}
		}
	}
}

int ecd_cmd_check(const struct shell *sh, const struct device *dev)
{
	uint16_t low[EC_MAX_COLS] = {0}, high[EC_MAX_COLS] = {0}, noisy[EC_MAX_COLS] = {0};
	uint16_t press, release;
	int16_t rest_median, moved = 0;
	size_t worst_i = 0, moved_i = 0;
	bool have_worst = false;
	int rc = begin(sh, dev);

	if (rc) {
		return rc;
	}
	shell_print(sh, "Hands off the keys for a few seconds.");
	rc = settle(sh, dev);
	if (rc) {
		goto out;
	}

	memset(w.check.sum, 0, sizeof(w.check.sum));
	FOR_KEYS(r, c) {
		w.check.lo[CELL(r, c)] = INT16_MAX;
		w.check.hi[CELL(r, c)] = INT16_MIN;
	}
	for (int n = 0; n < CHECK_PASSES; n++) {
		rc = pass(sh, dev);
		if (rc) {
			goto out;
		}
		FOR_KEYS(r, c) {
			size_t i = CELL(r, c);

			w.check.sum[i] += snap[i];
			w.check.lo[i] = MIN(w.check.lo[i], snap[i]);
			w.check.hi[i] = MAX(w.check.hi[i], snap[i]);
		}
	}
	/* `hi` becomes the peak-to-peak, `sum` is reused for the squares. */
	FOR_KEYS(r, c) {
		size_t i = CELL(r, c);

		res.rest[i] = w.check.sum[i] / CHECK_PASSES;
		w.check.hi[i] -= w.check.lo[i];
		w.check.sum[i] = 0;
	}
	for (int n = 0; n < CHECK_PASSES; n++) {
		rc = pass(sh, dev);
		if (rc) {
			goto out;
		}
		FOR_KEYS(r, c) {
			size_t i = CELL(r, c);
			int32_t d = snap[i] - res.rest[i];

			w.check.sum[i] += d * d;
		}
	}
	/* `lo` becomes the standard deviation, in tenths. */
	FOR_KEYS(r, c) {
		size_t i = CELL(r, c);
		uint64_t var100 = (uint64_t)(uint32_t)w.check.sum[i] * 100 / CHECK_PASSES;

		w.check.lo[i] = isqrt(MIN(var100, UINT32_MAX));
		if (!have_worst || w.check.hi[i] > w.check.hi[worst_i]) {
			worst_i = i;
			have_worst = true;
		}
	}
	res.have_check = true;
	res.pp_worst = w.check.hi[worst_i];
	res.sd10_worst = w.check.lo[worst_i];

	shell_print(sh, "Resting level, one row per strobe, one column per sense line:");
	print_grid(sh, res.rest, false);
	shell_print(sh, "Noise, peak to peak over %u passes:", CHECK_PASSES);
	print_grid(sh, w.check.hi, false);

	/* The medians sort in `snap`, so nothing reads a pass after this. */
	res.pp_median = median(w.check.hi, false, NULL);
	res.sd10_median = median(w.check.lo, false, NULL);
	rest_median = median(res.rest, false, NULL);

	FOR_KEYS(r, c) {
		size_t i = CELL(r, c);
		struct zmk_ec_direct_cell cell;

		zmk_ec_direct_get_cell(dev, r, c, &cell);
		if (abs(res.rest[i] - cell.baseline) > abs(moved)) {
			moved = res.rest[i] - cell.baseline;
			moved_i = i;
		}
		if (rest_median > 0 && res.rest[i] * 4 < rest_median) {
			low[c] |= BIT(r);
		} else if (rest_median > 0 && res.rest[i] > rest_median * 4) {
			high[c] |= BIT(r);
		}
		if (w.check.hi[i] > 3 * res.pp_median && w.check.hi[i] > 15) {
			noisy[c] |= BIT(r);
		}
	}

	zmk_ec_direct_get_thresholds(dev, &press, &release);
	shell_print(sh, "%u keys. Resting level: median %d.", res.keys, rest_median);
	shell_print(sh, "Noise: median %u peak to peak (sd %u.%u), worst %u (sd %u.%u) on %s.",
		    res.pp_median, res.sd10_median / 10, res.sd10_median % 10, res.pp_worst,
		    res.sd10_worst / 10, res.sd10_worst % 10, label(worst_i));
	shell_print(sh, "Release point %u, press point %u.", release, press);
	shell_print(sh, "Largest move since the baselines were taken: %+d on %s.", moved,
		    label(moved_i));
	print_bad(sh, low, "rests far below the others: open, or no pad coupling");
	print_bad(sh, high, "rests far above the others: held down, or shorted");
	print_bad(sh, noisy, "is much noisier than the others");
out:
	end(dev);
	return rc;
}

/* --- survey ---------------------------------------------------------- */

static void survey_summary(const struct shell *sh, const struct device *dev, int16_t sag,
			   size_t sag_i)
{
	uint16_t press, release, seen, shown = 0, over = 0;
	int16_t med, lo = INT16_MAX, hi = 0, last = 0;
	size_t last_i = 0;

	shell_print(sh, "Swing, one row per strobe, one column per sense line:");
	print_grid(sh, res.swing, true);
	med = median(res.swing, true, &seen);
	shell_print(sh, "%u of %u keys pressed.", seen, res.keys);
	if (!seen) {
		return;
	}
	zmk_ec_direct_get_thresholds(dev, &press, &release);
	FOR_KEYS(r, c) {
		int16_t s = res.swing[CELL(r, c)];

		if (!s) {
			shell_warn(sh, "not pressed: %s", label(CELL(r, c)));
			continue;
		}
		lo = MIN(lo, s);
		hi = MAX(hi, s);
		over += press * 100 > s * 40;
	}
	shell_print(sh, "Swing: min %d, median %d, max %d.", lo, med, hi);

	/* Weakest first: each round takes the smallest swing after the last. */
	shell_print(sh, "Weakest:");
	while (shown < MIN(seen, 5)) {
		int16_t best = INT16_MAX;
		size_t best_i = 0;

		FOR_KEYS(r, c) {
			size_t i = CELL(r, c);
			int16_t s = res.swing[i];
			bool after = !shown || s > last || (s == last && i > last_i);

			if (s && after && s < best) {
				best = s;
				best_i = i;
			}
		}
		last = best;
		last_i = best_i;
		shown++;
		shell_print(sh, "  %4d  %s%s", best, label(best_i),
			    best * 2 < med ? "  <- under half the median, check its dome" : "");
	}
	shell_print(sh, "Press point %u is %u to %u %% of a key's swing; above 40 %% on %u.",
		    press, press * 100 / hi, press * 100 / lo, over);
	shell_print(sh, "Lowest an unpressed key read: %+d on %s.", sag, label(sag_i));
	shell_print(sh, "`ecd report` prints this as devicetree.");
}

int ecd_cmd_survey(const struct shell *sh, const struct device *dev)
{
	uint16_t seen[EC_MAX_COLS] = {0};
	uint16_t count = 0;
	int16_t detect, sag = 0;
	size_t sag_i = 0;
	int64_t last_ms;
	int blk = 0;
	int rc = begin(sh, dev);

	if (rc) {
		return rc;
	}
	if (!zmk_ec_direct_get_load_comp(dev)) {
		shell_warn(sh, "load-comp-scale is 0. With several keys on a sense line, run "
			       "`ecd comp fit` first");
	}
	shell_print(sh, "Hands off the keys for a moment.");
	rc = settle(sh, dev);
	if (!rc) {
		rc = rebase(sh, dev, w.survey.base, &detect);
	}
	if (rc) {
		goto out;
	}
	shell_print(sh, "Now press each of the %u keys once, all the way down, for about half a "
			"second.", res.keys);
	shell_print(sh, "A key counts above %+d. Ends when all are done, or after %u s with no "
			"key.", detect, SURVEY_IDLE_MS / 1000);

	memset(res.swing, 0, sizeof(res.swing));
	memset(w.survey.blocks, 0, sizeof(w.survey.blocks));
	FOR_KEYS(r, c) {
		w.survey.lo[CELL(r, c)] = INT16_MAX;
	}
	res.have_survey = true;
	last_ms = k_uptime_get();

	while (1) {
		bool down = false;

		rc = pass(sh, dev);
		if (rc) {
			goto out;
		}
		FOR_KEYS(r, c) {
			size_t i = CELL(r, c);

			w.survey.lo[i] = MIN(w.survey.lo[i], snap[i] - w.survey.base[i]);
		}
		if (++blk < BLOCK) {
			continue;
		}
		blk = 0;
		FOR_KEYS(r, c) {
			size_t i = CELL(r, c);
			int16_t level = w.survey.lo[i];

			w.survey.lo[i] = INT16_MAX;
			if (level > detect) {
				if (!w.survey.blocks[i]) {
					w.survey.peak[i] = 0;
					last_ms = k_uptime_get();
				}
				if (w.survey.blocks[i] < UINT8_MAX) {
					w.survey.blocks[i]++;
				}
				w.survey.peak[i] = MAX(w.survey.peak[i], level);
			} else if (level <= detect / 2 && w.survey.blocks[i]) {
				/* Released. Too short a press may not have bottomed. */
				if (w.survey.blocks[i] >= PRESS_BLOCKS) {
					if (!(seen[c] & BIT(r))) {
						seen[c] |= BIT(r);
						count++;
					}
					res.swing[i] = MAX(res.swing[i], w.survey.peak[i]);
					shell_print(sh, "%3u/%u  %4d  %s", count, res.keys,
						    res.swing[i], label(i));
				}
				w.survey.blocks[i] = 0;
				last_ms = k_uptime_get();
			}
			if (w.survey.blocks[i]) {
				down = true;
			} else if (level < sag) {
				sag = level;
				sag_i = i;
			}
		}
		if (count == res.keys && !down) {
			break;
		}
		if (k_uptime_get() - last_ms > SURVEY_IDLE_MS) {
			shell_print(sh, "No key for %u s, stopping.", SURVEY_IDLE_MS / 1000);
			break;
		}
	}
	survey_summary(sh, dev, sag, sag_i);
out:
	end(dev);
	return rc;
}

/* --- comp fit -------------------------------------------------------- */

/*
 * The compensation holds reading / (K - S) constant for a key held at the
 * same depth, S being the sum of its sense line's readings. So one key read
 * in two states of the line, (v1, S1) and (v2, S2), gives
 *
 *   K = (v1 * S2 - v2 * S1) / (v1 - v2)
 *
 * A single key's answer moves by 20 % with how hard the finger pushes in each
 * state, so a line gives one estimate per key and the median is the answer.
 */
static void fit_line(const struct shell *sh, uint8_t c, uint16_t held, uint16_t s2)
{
	int16_t k[EC_MAX_ROWS];
	uint8_t n = 0;

	for (uint8_t r = 0; r < res.rows; r++) {
		size_t i = CELL(r, c);
		int32_t v1 = w.fit.v1[i], v2 = w.fit.v2[i];
		int64_t est;

		if (!(held & BIT(r)) || v1 - v2 < FIT_MIN_DROP) {
			continue;
		}
		est = ((int64_t)v1 * s2 - (int64_t)v2 * w.fit.s1[i]) / (v1 - v2);
		/* K under the line's own sum would turn the correction over. */
		if (est <= s2 || est > INT16_MAX) {
			continue;
		}
		w.fit.est[i] = est;
		k[n++] = est;
	}
	if (!n) {
		shell_print(sh, "line %u: nothing to go on. Hold the first keys down while "
				"adding the rest", c);
		return;
	}
	qsort(k, n, sizeof(k[0]), cmp_i16);
	shell_print(sh, "line %u: %u keys held, K %d to %d, median %d from %u of them", c,
		    __builtin_popcount(held), k[0], k[n - 1], k[n / 2], n);
}

int ecd_cmd_comp_fit(const struct shell *sh, const struct device *dev)
{
	uint16_t prev[EC_MAX_COLS] = {0}, held[EC_MAX_COLS] = {0}, s2[EC_MAX_COLS] = {0};
	uint8_t stable[EC_MAX_COLS] = {0}, most[EC_MAX_COLS] = {0};
	uint16_t k_old = zmk_ec_direct_get_load_comp(dev);
	uint16_t n = 0;
	int16_t detect, k_new;
	int64_t t0, last_ms;
	int rc = begin(sh, dev);

	if (rc) {
		return rc;
	}
	/* The fit works on readings nothing has corrected yet. */
	zmk_ec_direct_set_load_comp(dev, 0);
	shell_print(sh, "Hands off the keys for a moment.");
	rc = settle(sh, dev);
	if (!rc) {
		rc = rebase(sh, dev, w.fit.base, &detect);
	}
	if (rc) {
		goto out;
	}
	shell_print(sh, "Pick a sense line. Press its keys one after another and keep them all "
			"down, five or more, then let go.");
	shell_print(sh, "Repeat on other lines if you like. Ends %u s after the last one.",
		    FIT_IDLE_MS / 1000);

	memset(&w.fit.v1, 0, sizeof(w.fit.v1));
	memset(&w.fit.est, 0, sizeof(w.fit.est));
	t0 = last_ms = k_uptime_get();

	while (1) {
		bool any = false, have = false;
		int64_t now;

		rc = pass(sh, dev);
		if (rc) {
			goto out;
		}
		for (uint8_t c = 0; c < res.cols; c++) {
			uint16_t down = 0;
			int32_t sum = 0;

			for (uint8_t r = 0; r < res.rows; r++) {
				size_t i = CELL(r, c);

				if (!IS_KEY(r, c)) {
					continue;
				}
				sum += snap[i];
				if (snap[i] - w.fit.base[i] > detect) {
					down |= BIT(r);
				}
			}
			any |= down != 0;
			if (down != prev[c]) {
				prev[c] = down;
				stable[c] = 0;
				continue;
			}
			/* Read a set of held keys once, when it has stopped changing. */
			if (stable[c] == UINT8_MAX || ++stable[c] != STABLE_PASSES) {
				continue;
			}
			sum = CLAMP(sum, 0, UINT16_MAX);
			if (down) {
				bool deepest = __builtin_popcount(down) >= most[c];

				for (uint8_t r = 0; r < res.rows; r++) {
					size_t i = CELL(r, c);

					if (!(down & BIT(r))) {
						continue;
					}
					if (!w.fit.v1[i]) {
						w.fit.v1[i] = snap[i];
						w.fit.s1[i] = sum;
					}
					if (deepest) {
						w.fit.v2[i] = snap[i];
					}
				}
				if (deepest) {
					most[c] = __builtin_popcount(down);
					held[c] = down;
					s2[c] = sum;
				}
			} else if (most[c]) {
				if (most[c] >= 2) {
					fit_line(sh, c, held[c], s2[c]);
				}
				most[c] = 0;
				for (uint8_t r = 0; r < res.rows; r++) {
					w.fit.v1[CELL(r, c)] = 0;
				}
			}
		}
		FOR_KEYS(r, c) {
			have |= w.fit.est[CELL(r, c)] != 0;
		}
		now = k_uptime_get();
		if (any) {
			last_ms = now;
		}
		if (now - last_ms > (have ? FIT_IDLE_MS : FIT_FIRST_MS) || now - t0 > FIT_CAP_MS) {
			break;
		}
	}

	/* est is uint16 but holds values that passed the INT16_MAX test. */
	k_new = median((const int16_t *)w.fit.est, true, &n);
	if (!n) {
		shell_error(sh, "No estimate. load-comp-scale stays at %u.", k_old);
		rc = -ENODATA;
		goto out;
	}
	res.fit_k = k_old = k_new;
	res.fit_keys = n;
	shell_print(sh, "K = %d, the median of %u keys (%d to %d).", k_new, n, snap[0],
		    snap[n - 1]);
	if (n < 5) {
		shell_warn(sh, "Five or more keys give a steadier number");
	}
	shell_print(sh, "load-comp-scale = <%d>;  applied until reset. Run `ecd survey` next.",
		    k_new);
out:
	zmk_ec_direct_set_load_comp(dev, k_old);
	end(dev);
	return rc;
}

/* --- report ---------------------------------------------------------- */

int ecd_cmd_report(const struct shell *sh, const struct device *dev)
{
	struct zmk_ec_direct_tripwire tw;
	uint16_t press, release, seen = 0;
	int16_t med;

	if (!res.have_check && !res.have_survey) {
		shell_print(sh, "Nothing measured yet. Run `ecd check`, `ecd comp fit` and "
				"`ecd survey` first.");
		return 0;
	}
	zmk_ec_direct_get_thresholds(dev, &press, &release);
	zmk_ec_direct_get_tripwire(dev, &tw);

	shell_print(sh, "/* %s, %u strobes x %u sense lines, %u keys, %u samples per key */",
		    CONFIG_BOARD, res.rows, res.cols, res.keys, NAVG);
	shell_print(sh, "press-threshold = <%u>;", press);
	shell_print(sh, "release-threshold = <%u>;", release);
	shell_print(sh, "load-comp-scale = <%u>;%s", zmk_ec_direct_get_load_comp(dev),
		    res.fit_keys ? "" : "  /* not fitted in this session */");
	shell_print(sh, "");
	shell_print(sh, "/* Per-key table, one row per strobe, one column per sense line.");
	shell_print(sh, " * The driver does not read these yet. */");
	if (res.have_check) {
		print_array(sh, "resting", res.rest);
	}
	if (res.have_survey) {
		print_array(sh, "swing", res.swing);
	}
	shell_print(sh, "");
	if (res.fit_keys) {
		shell_print(sh, "K %u from %u keys.", res.fit_k, res.fit_keys);
	}
	if (res.have_check) {
		shell_print(sh, "Noise: median %u peak to peak (sd %u.%u), worst %u (sd %u.%u).",
			    res.pp_median, res.sd10_median / 10, res.sd10_median % 10,
			    res.pp_worst, res.sd10_worst / 10, res.sd10_worst % 10);
	}
	if (res.have_survey) {
		med = median(res.swing, true, &seen);
		if (seen) {
			shell_print(sh, "Swing: %u of %u keys, min %d, median %d, max %d.", seen,
				    res.keys, snap[0], med, snap[seen - 1]);
			shell_print(sh, "Press point %u is %u to %u %% of a key's swing.", press,
				    press * 100 / snap[seen - 1], press * 100 / snap[0]);
		}
	}
	if (tw.enabled) {
		shell_print(sh, "Tripwire: trips over %u, quiet maximum %+d, %u false trips.",
			    tw.threshold, tw.max_pos, tw.false_trips);
	}
	return 0;
}
