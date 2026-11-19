/*
 * Bring-up commands for zmk,kscan-ec-direct: what a board designer runs once
 * to characterise a design, and what a builder runs to check an assembly.
 *
 *   ecd rest        hands off: resting level and noise of every key
 *   ecd fit         lines of held keys: the load compensation scale
 *   ecd calibrate   one press per key: every key's swing
 *   ecd export      all of it, ready to paste
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

#include <zmk/matrix_transform.h>
#include <zmk/physical_layouts.h>

#include <zmk_ec_direct/ec_direct.h>

#include "ec_direct_priv.h"
#include "ec_direct_shell.h"

/* Passes per figure in `rest`: about a second at 250 Hz. */
#define REST_PASSES 256
/* Passes that set how far a quiet key wanders, before calibrate and fit. */
#define QUIET_PASSES 64
/* A key counts as down above 4x that, inside these bounds. */
#define DETECT_MIN 30
#define DETECT_MAX 120
/* Calibrate works in blocks of passes and keeps each block's lowest reading,
 * so a swing is a level the key held for a whole block and not one high
 * sample. */
#define BLOCK        4
#define PRESS_BLOCKS 3
/* fit: passes a set of held keys has to stay the same before it is read. */
#define STABLE_PASSES 25
/* A key has to drop by this much between its two states to give a K. */
#define FIT_MIN_DROP 8
/* Estimates kept across every roll of a fit. */
#define FIT_MAX_EST (2 * EC_MAX_CELLS)

#define CALIBRATE_IDLE_MS 20000
#define FIT_FIRST_MS      30000
#define FIT_IDLE_MS       10000
#define FIT_CAP_MS        180000

/* Drawing the board: characters per key width, the field a key's text gets,
 * and the widest line drawn before falling back to the strobe grid. */
#define DRAW_U       6
#define DRAW_FIELD   5
#define DRAW_WIDTH   96
#define DRAW_MAX_POS 128
#define NO_CELL      0xFF

/* One pass. Between passes it doubles as sort scratch. */
static int16_t snap[EC_MAX_CELLS];

/* Working arrays. Only one command runs at a time, so they share. */
static union {
	struct {
		int32_t sum[EC_MAX_CELLS];
		int16_t lo[EC_MAX_CELLS];
		int16_t hi[EC_MAX_CELLS];
	} rest;
	struct {
		int16_t base[EC_MAX_CELLS];
		int16_t lo[EC_MAX_CELLS];   /* lowest reading of the current block */
		int16_t peak[EC_MAX_CELLS]; /* highest block of the current press */
		uint8_t blocks[EC_MAX_CELLS];
	} cal;
	struct {
		int16_t base[EC_MAX_CELLS];
		int16_t v1[EC_MAX_CELLS];  /* reading when the key first went down */
		uint16_t s1[EC_MAX_CELLS]; /* its line's sum then */
		int16_t v2[EC_MAX_CELLS];  /* reading with the most keys held */
		int16_t est[FIT_MAX_EST];  /* every estimate from every roll */
	} fit;
} w;

/* What the commands found, kept for `export`. */
static struct {
	uint8_t rows, cols;
	uint16_t keys;
	uint16_t mask[EC_MAX_COLS]; /* per sense line, bit r = strobe r is a key */
	bool have_rest, have_cal;
	int16_t rest[EC_MAX_CELLS];
	int16_t swing[EC_MAX_CELLS]; /* 0 = not pressed in calibrate */
	uint16_t pp_median, pp_worst;
	uint16_t sd10_median, sd10_worst;
	uint16_t fit_k, fit_n;
	/* Unpressed keys while another key on their line was held. */
	int16_t dip, rise;
	size_t dip_i, rise_i;
} res;

/* The layout the board is using now, and the cell at each of its positions. */
static const struct zmk_physical_layout *layout;
static uint8_t pos_cell[DRAW_MAX_POS];

#define CELL(r, c)   ((size_t)(r) * res.cols + (c))
#define IS_KEY(r, c) ((res.mask[c] & BIT(r)) != 0)
#define FOR_KEYS(r, c)                                                                             \
	for (uint8_t r = 0; r < res.rows; r++)                                                     \
		for (uint8_t c = 0; c < res.cols; c++)                                             \
			if (IS_KEY(r, c))

/* --- helpers --------------------------------------------------------- */

static int position(uint8_t r, uint8_t c)
{
	int32_t p;

	if (!layout) {
		return -1;
	}
	p = zmk_matrix_transform_row_column_to_position(layout->matrix_transform, r, c);
	return p < 0 ? -1 : p;
}

static void load_layout(void)
{
	const struct zmk_physical_layout *const *list;
	size_t n = zmk_physical_layouts_get_list(&list);
	int sel = zmk_physical_layouts_get_selected();

	layout = sel >= 0 && (size_t)sel < n ? list[sel] : NULL;
	memset(pos_cell, NO_CELL, sizeof(pos_cell));
	FOR_KEYS(r, c) {
		int p = position(r, c);

		if (p >= 0 && p < DRAW_MAX_POS) {
			pos_cell[p] = CELL(r, c);
		}
	}
}

/* One at a time: the result is a static buffer. */
static const char *label(size_t i)
{
	static char buf[24];
	uint8_t r = i / res.cols, c = i % res.cols;
	int p = position(r, c);

	if (p >= 0) {
		snprintk(buf, sizeof(buf), "key %d", p);
	} else {
		snprintk(buf, sizeof(buf), "strobe %u line %u", r, c);
	}
	return buf;
}

/* "keys 4, 14, 24" for the cells in `set` (per sense line, bit r = strobe r),
 * in keymap order. No line break. */
static void print_keys(const struct shell *sh, enum shell_vt100_color color, bool cap,
		       const uint16_t *set)
{
	uint16_t n = 0, done = 0;

	FOR_KEYS(r, c) {
		n += (set[c] & BIT(r)) != 0;
	}
	shell_fprintf(sh, color, "%s%s", cap ? "K" : "k", n == 1 ? "ey" : "eys");
	for (int p = 0; p < DRAW_MAX_POS; p++) {
		uint8_t i = pos_cell[p];

		if (i != NO_CELL && (set[i % res.cols] & BIT(i / res.cols))) {
			shell_fprintf(sh, color, "%s %d", done++ ? "," : "", p);
		}
	}
	FOR_KEYS(r, c) {
		int p = position(r, c);

		if ((set[c] & BIT(r)) && (p < 0 || p >= DRAW_MAX_POS)) {
			shell_fprintf(sh, color, "%s strobe %u line %u", done++ ? "," : "", r, c);
		}
	}
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
	memcpy(w.cal.lo, snap, sizeof(snap));
	while (still < 100 && k_uptime_get() - t0 < 5000) {
		rc = pass(sh, dev);
		if (rc) {
			return rc;
		}
		still++;
		FOR_KEYS(r, c) {
			size_t i = CELL(r, c);

			if (abs(snap[i] - w.cal.lo[i]) > 25) {
				still = 0;
			}
		}
		if (!still) {
			memcpy(w.cal.lo, snap, sizeof(snap));
		}
	}
	if (still < 100) {
		shell_warn(sh, "The readings never settled. Going on anyway.");
	}
	return 0;
}

static int begin(const struct shell *sh, const struct device *dev)
{
	if (!device_is_ready(dev)) {
		shell_error(sh, "The driver failed to start. See `ecd status`.");
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
		shell_error(sh, "No keys in the matrix.");
		return -ENODEV;
	}
	load_layout();
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
		shell_error(sh, "The resting levels were not taken (%d).", rc);
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
		shell_warn(sh, "Noisy here: quiet keys move by %d. A battery or a quieter "
			       "supply gives better numbers.", worst);
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

/* --- drawing the board ----------------------------------------------- */

/*
 * A table shows a value per key (`vals`, with 0 as "--" if `skip_zero`).
 * Without `vals` it marks keys: the ones in `sel` (per sense line, bit r =
 * strobe r) show their keymap number in brackets, the others a dot on the
 * board or their number in the grid.
 */
struct view {
	const int16_t *vals;
	bool skip_zero;
	const uint16_t *sel;
};

static void key_text(const struct view *v, size_t i, bool grid, char *out)
{
	uint8_t r = i / res.cols, c = i % res.cols;
	int p = position(r, c);
	char t[16];
	size_t len;

	if (v->vals) {
		if (!v->vals[i] && v->skip_zero) {
			strcpy(t, "--");
		} else {
			snprintk(t, sizeof(t), "%d", v->vals[i]);
		}
	} else if (v->sel[c] & BIT(r)) {
		snprintk(t, sizeof(t), p >= 0 ? "[%d]" : "[*]", p);
	} else if (grid && p >= 0) {
		snprintk(t, sizeof(t), "%d", p);
	} else {
		strcpy(t, ".");
	}
	/* Right-aligned by hand: the nano printf has no `*` width. Anything
	 * wider than the field keeps its first characters. */
	len = MIN(strlen(t), DRAW_FIELD);
	memset(out, ' ', DRAW_FIELD);
	memcpy(out + DRAW_FIELD - len, t, len);
	out[DRAW_FIELD] = '\0';
}

/* One row per strobe, one column per sense line. */
static void print_grid(const struct shell *sh, const struct view *v)
{
	char t[DRAW_FIELD + 1];

	for (uint8_t r = 0; r < res.rows; r++) {
		shell_fprintf(sh, SHELL_NORMAL, "strobe %2u |", r);
		for (uint8_t c = 0; c < res.cols; c++) {
			if (IS_KEY(r, c)) {
				key_text(v, CELL(r, c), true, t);
			} else {
				memset(t, ' ', DRAW_FIELD);
				t[DRAW_FIELD] = '\0';
			}
			shell_fprintf(sh, SHELL_NORMAL, " %s", t);
		}
		shell_print(sh, "");
	}
}

static int rdiv(int32_t a, int32_t b)
{
	return a >= 0 ? (a + b / 2) / b : -((-a + b / 2) / b);
}

/* Half a key per line, so a column stagger shows as steps. */
static int key_line(const struct zmk_key_physical_attrs *k)
{
	return rdiv(k->y, 50);
}

static int key_col(const struct zmk_key_physical_attrs *k)
{
	return rdiv((int32_t)k->x * DRAW_U, 100);
}

static int key_width(const struct zmk_key_physical_attrs *k)
{
	return MAX(rdiv((int32_t)k->width * DRAW_U, 100), DRAW_U);
}

static size_t drawn_keys(void)
{
	return layout ? MIN(layout->keys_len, DRAW_MAX_POS) : 0;
}

/* The board can be drawn: the layout has key positions and fits the width. */
static bool can_draw(void)
{
	bool any = false;

	for (size_t p = 0; p < drawn_keys(); p++) {
		const struct zmk_key_physical_attrs *k = &layout->keys[p];

		if (pos_cell[p] == NO_CELL) {
			continue;
		}
		if (key_col(k) < 0 || key_col(k) + key_width(k) > DRAW_WIDTH) {
			return false;
		}
		any = true;
	}
	return any;
}

/*
 * Built one terminal line at a time: a whole picture would not fit next to
 * everything else on a 32 kB part. Rotation is ignored, and a key that would
 * overlap the one to its left moves right.
 */
static void draw_board(const struct shell *sh, const struct view *v)
{
	char buf[DRAW_WIDTH + 1];
	int top = INT16_MAX, bottom = INT16_MIN;

	for (size_t p = 0; p < drawn_keys(); p++) {
		if (pos_cell[p] != NO_CELL) {
			top = MIN(top, key_line(&layout->keys[p]));
			bottom = MAX(bottom, key_line(&layout->keys[p]));
		}
	}
	for (int line = top; line <= bottom; line++) {
		int end = 0, last_x = INT16_MIN, len = 0;
		size_t last_p = 0;
		bool first = true;

		memset(buf, ' ', sizeof(buf));
		/* Left to right: each round takes the next key by x, then position. */
		while (1) {
			const struct zmk_key_physical_attrs *k;
			size_t best = SIZE_MAX;
			int start;

			for (size_t p = 0; p < drawn_keys(); p++) {
				int x = layout->keys[p].x;

				if (pos_cell[p] == NO_CELL || key_line(&layout->keys[p]) != line) {
					continue;
				}
				if (!first && (x < last_x || (x == last_x && p <= last_p))) {
					continue;
				}
				if (best == SIZE_MAX || x < layout->keys[best].x) {
					best = p;
				}
			}
			if (best == SIZE_MAX) {
				break;
			}
			k = &layout->keys[best];
			first = false;
			last_x = k->x;
			last_p = best;
			start = MAX(key_col(k), end) + (key_width(k) - DRAW_U) / 2;
			if (start + DRAW_FIELD > DRAW_WIDTH) {
				break;
			}
			key_text(v, pos_cell[best], false, &buf[start]);
			buf[start + DRAW_FIELD] = ' '; /* key_text ends with a NUL */
			end = MAX(key_col(k), end) + key_width(k);
			len = start + DRAW_FIELD;
		}
		if (len) {
			buf[len] = '\0';
			shell_print(sh, "%s", buf);
		}
	}
}

/* The board when it can be drawn, the strobe grid otherwise. Keys the
 * drawing has no place for are listed under it. */
static void show(const struct shell *sh, const struct view *v)
{
	char t[DRAW_FIELD + 1];

	if (!can_draw()) {
		print_grid(sh, v);
		return;
	}
	draw_board(sh, v);
	FOR_KEYS(r, c) {
		int p = position(r, c);

		if (p >= 0 && (size_t)p < drawn_keys()) {
			continue;
		}
		if (!v->vals && !(v->sel[c] & BIT(r))) {
			continue;
		}
		key_text(v, CELL(r, c), true, t);
		shell_print(sh, "Not in the drawing: strobe %u line %u: %s", r, c, t);
	}
}

static void show_values(const struct shell *sh, const int16_t *vals, bool skip_zero)
{
	const struct view v = {.vals = vals, .skip_zero = skip_zero};

	show(sh, &v);
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

/* --- rest ------------------------------------------------------------ */

/* Warn about the cells in `bad` (per sense line, bit r = strobe r): a whole
 * strobe or sense line once, with its keys and pin, the rest one by one. */
static bool print_bad(const struct shell *sh, const uint16_t *bad, const char *what)
{
	uint16_t whole_strobes = 0, one[EC_MAX_COLS];
	bool any_bad = false;

	for (uint8_t r = 0; r < res.rows; r++) {
		bool all = true, any = false;

		memset(one, 0, sizeof(one));
		for (uint8_t c = 0; c < res.cols; c++) {
			if (IS_KEY(r, c)) {
				any = true;
				all &= (bad[c] & BIT(r)) != 0;
				one[c] = BIT(r);
			}
		}
		if (any && all) {
			whole_strobes |= BIT(r);
			shell_fprintf(sh, SHELL_WARNING, "Strobe %u (", r);
			print_keys(sh, SHELL_WARNING, false, one);
			shell_fprintf(sh, SHELL_WARNING, "): every key %s. Check that pin.\n",
				      what);
			any_bad = true;
		}
	}
	for (uint8_t c = 0; c < res.cols; c++) {
		if (bad[c] && bad[c] == res.mask[c]) {
			memset(one, 0, sizeof(one));
			one[c] = res.mask[c];
			shell_fprintf(sh, SHELL_WARNING, "Sense line %u (", c);
			print_keys(sh, SHELL_WARNING, false, one);
			shell_fprintf(sh, SHELL_WARNING, "): every key %s. Check that pin.\n",
				      what);
			any_bad = true;
			continue;
		}
		for (uint8_t r = 0; r < res.rows; r++) {
			if ((bad[c] & BIT(r)) && !(whole_strobes & BIT(r))) {
				shell_warn(sh, "%s %s.", label(CELL(r, c)), what);
				any_bad = true;
			}
		}
	}
	return any_bad;
}

int ecd_cmd_rest(const struct shell *sh, const struct device *dev)
{
	uint16_t low[EC_MAX_COLS] = {0}, high[EC_MAX_COLS] = {0}, noisy[EC_MAX_COLS] = {0};
	uint16_t press, release;
	int16_t rest_median, moved = 0;
	size_t worst_i = 0, moved_i = 0;
	bool have_worst = false, stuck;
	int rc = begin(sh, dev);

	if (rc) {
		return rc;
	}
	shell_print(sh, "Hands off the keys for a few seconds.");
	rc = settle(sh, dev);
	if (rc) {
		goto out;
	}

	memset(w.rest.sum, 0, sizeof(w.rest.sum));
	FOR_KEYS(r, c) {
		w.rest.lo[CELL(r, c)] = INT16_MAX;
		w.rest.hi[CELL(r, c)] = INT16_MIN;
	}
	for (int n = 0; n < REST_PASSES; n++) {
		rc = pass(sh, dev);
		if (rc) {
			goto out;
		}
		FOR_KEYS(r, c) {
			size_t i = CELL(r, c);

			w.rest.sum[i] += snap[i];
			w.rest.lo[i] = MIN(w.rest.lo[i], snap[i]);
			w.rest.hi[i] = MAX(w.rest.hi[i], snap[i]);
		}
	}
	/* `hi` becomes the peak-to-peak, `sum` is reused for the squares. */
	FOR_KEYS(r, c) {
		size_t i = CELL(r, c);

		res.rest[i] = w.rest.sum[i] / REST_PASSES;
		w.rest.hi[i] -= w.rest.lo[i];
		w.rest.sum[i] = 0;
	}
	for (int n = 0; n < REST_PASSES; n++) {
		rc = pass(sh, dev);
		if (rc) {
			goto out;
		}
		FOR_KEYS(r, c) {
			size_t i = CELL(r, c);
			int32_t d = snap[i] - res.rest[i];

			w.rest.sum[i] += d * d;
		}
	}
	/* `lo` becomes the standard deviation, in tenths. */
	FOR_KEYS(r, c) {
		size_t i = CELL(r, c);
		uint64_t var100 = (uint64_t)(uint32_t)w.rest.sum[i] * 100 / REST_PASSES;

		w.rest.lo[i] = isqrt(MIN(var100, UINT32_MAX));
		if (!have_worst || w.rest.hi[i] > w.rest.hi[worst_i]) {
			worst_i = i;
			have_worst = true;
		}
	}
	res.have_rest = true;
	res.pp_worst = w.rest.hi[worst_i];
	res.sd10_worst = w.rest.lo[worst_i];

	shell_print(sh, "Resting level:");
	show_values(sh, res.rest, false);
	shell_print(sh, "Noise, peak to peak:");
	show_values(sh, w.rest.hi, false);

	/* The medians sort in `snap`, so nothing reads a pass after this. */
	res.pp_median = median(w.rest.hi, false, NULL);
	res.sd10_median = median(w.rest.lo, false, NULL);
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
		if (w.rest.hi[i] > 3 * res.pp_median && w.rest.hi[i] > 15) {
			noisy[c] |= BIT(r);
		}
	}

	shell_print(sh, "%u keys. Resting level: median %d. Noise: median %u peak to peak, "
			"worst %u on %s.", res.keys, rest_median, res.pp_median, res.pp_worst,
		    label(worst_i));
	stuck = print_bad(sh, low, "rests far below the others: open, or no pad coupling");
	stuck |= print_bad(sh, high, "rests far above the others: held down, or shorted");
	print_bad(sh, noisy, "is much noisier than the others");

	/* Re-zeroing over a stuck key would hide it, so only hint when none is. */
	zmk_ec_direct_get_thresholds(dev, &press, &release);
	if (!stuck && abs(moved) > release / 2) {
		shell_print(sh, "Resting levels have moved since they were last taken (%+d on %s). "
				"`ecd zero` takes them again.", moved, label(moved_i));
	}
out:
	end(dev);
	return rc;
}

/* --- calibrate ------------------------------------------------------- */

static void cal_summary(const struct shell *sh, const struct device *dev, int16_t detect)
{
	uint16_t missing[EC_MAX_COLS] = {0}, deep[EC_MAX_COLS] = {0};
	uint16_t press, release, seen, shown = 0;
	uint8_t lines = 0, weak_lines = 0;
	int16_t med, last = 0;
	size_t last_i = 0;
	bool any_missing = false, any_deep = false;

	shell_print(sh, "Swing per key:");
	show_values(sh, res.swing, true);
	med = median(res.swing, true, &seen);
	if (!seen) {
		shell_warn(sh, "No key was pressed.");
		return;
	}
	zmk_ec_direct_get_thresholds(dev, &press, &release);
	FOR_KEYS(r, c) {
		int16_t s = res.swing[CELL(r, c)];

		if (!s) {
			missing[c] |= BIT(r);
			any_missing = true;
		} else if (press * 100 > s * 40) {
			deep[c] |= BIT(r);
			any_deep = true;
		}
	}
	if (any_missing) {
		shell_fprintf(sh, SHELL_WARNING, "Not pressed: ");
		print_keys(sh, SHELL_WARNING, false, missing);
		shell_fprintf(sh, SHELL_WARNING, ". None read above %+d. If you did press them, "
						 "check their domes.\n", detect);
	}

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

	/* A whole weak sense line is its pin or routing, not its domes, so it
	 * is named once. Caroline's RP2040 has one behind an analog switch.
	 * Only when other lines are fine: if all are weak, it's the threshold. */
	for (uint8_t c = 0; c < res.cols; c++) {
		lines += res.mask[c] != 0;
		weak_lines += __builtin_popcount(res.mask[c]) >= 2 && deep[c] == res.mask[c];
	}
	for (uint8_t c = 0; weak_lines < lines && c < res.cols; c++) {
		uint16_t one[EC_MAX_COLS] = {0};

		if (__builtin_popcount(res.mask[c]) < 2 || deep[c] != res.mask[c]) {
			continue;
		}
		one[c] = deep[c];
		deep[c] = 0;
		shell_fprintf(sh, SHELL_WARNING, "Sense line %u (", c);
		print_keys(sh, SHELL_WARNING, false, one);
		shell_fprintf(sh, SHELL_WARNING, ") reads weaker than the others: the press point "
						 "is over 40 %% of every key's travel. Check that "
						 "line's pin and routing, or lower `press-threshold`.\n");
	}
	any_deep = false;
	for (uint8_t c = 0; c < res.cols; c++) {
		any_deep |= deep[c] != 0;
	}
	if (any_deep) {
		print_keys(sh, SHELL_WARNING, true, deep);
		shell_fprintf(sh, SHELL_WARNING, " need a deeper press than the rest: the press "
						 "point is over 40 %% of their travel. Check their "
						 "domes, or lower `press-threshold`.\n");
	}

	/* A wrong K shows on the keys next to a held one. With K at 0 the
	 * warning before the presses has already said so. */
	if (zmk_ec_direct_get_load_comp(dev)) {
		if (-res.dip > press / 2) {
			shell_warn(sh, "Unpressed keys dipped to %+d (%s) while others on their "
				       "line were held, so K is too high. Run `ecd fit` again.",
				   res.dip, label(res.dip_i));
		}
		if (res.rise > press / 2) {
			shell_warn(sh, "Unpressed keys rose to %+d (%s) while others on their line "
				       "were held, so K is too low. Run `ecd fit` again.",
				   res.rise, label(res.rise_i));
		}
	}
	shell_print(sh, "`ecd export` prints this as devicetree.");
}

int ecd_cmd_calibrate(const struct shell *sh, const struct device *dev)
{
	uint16_t seen[EC_MAX_COLS] = {0};
	uint16_t count = 0;
	int16_t detect;
	int64_t last_ms;
	int blk = 0;
	int rc = begin(sh, dev);

	if (rc) {
		return rc;
	}
	if (!zmk_ec_direct_get_load_comp(dev)) {
		shell_warn(sh, "Load compensation is off. With several keys on a sense line, run "
			       "`ecd fit` first.");
	}
	shell_print(sh, "Hands off the keys for a moment.");
	rc = settle(sh, dev);
	if (!rc) {
		rc = rebase(sh, dev, w.cal.base, &detect);
	}
	if (rc) {
		goto out;
	}
	shell_print(sh, "Now press each of the %u keys once, all the way down, for about half a "
			"second.", res.keys);
	shell_print(sh, "It ends when every key is done, or after %u s with no key.",
		    CALIBRATE_IDLE_MS / 1000);

	memset(res.swing, 0, sizeof(res.swing));
	memset(w.cal.blocks, 0, sizeof(w.cal.blocks));
	FOR_KEYS(r, c) {
		w.cal.lo[CELL(r, c)] = INT16_MAX;
	}
	res.have_cal = true;
	res.dip = res.rise = 0;
	res.dip_i = res.rise_i = 0;
	last_ms = k_uptime_get();

	while (1) {
		uint16_t held[EC_MAX_COLS] = {0};
		bool down = false;

		rc = pass(sh, dev);
		if (rc) {
			goto out;
		}
		FOR_KEYS(r, c) {
			size_t i = CELL(r, c);

			w.cal.lo[i] = MIN(w.cal.lo[i], snap[i] - w.cal.base[i]);
		}
		if (++blk < BLOCK) {
			continue;
		}
		blk = 0;
		FOR_KEYS(r, c) {
			size_t i = CELL(r, c);
			int16_t level = w.cal.lo[i];

			if (level > detect) {
				if (!w.cal.blocks[i]) {
					w.cal.peak[i] = 0;
					last_ms = k_uptime_get();
				}
				if (w.cal.blocks[i] < UINT8_MAX) {
					w.cal.blocks[i]++;
				}
				w.cal.peak[i] = MAX(w.cal.peak[i], level);
			} else if (level <= detect / 2 && w.cal.blocks[i]) {
				/* Released. Too short a press may not have bottomed. */
				if (w.cal.blocks[i] >= PRESS_BLOCKS) {
					int16_t peak = w.cal.peak[i];

					if (!(seen[c] & BIT(r))) {
						seen[c] |= BIT(r);
						count++;
						shell_print(sh, "%3u/%u  %4d  %s", count, res.keys,
							    peak, label(i));
					} else if (peak > res.swing[i]) {
						shell_print(sh, "        %4d  %s (again)", peak,
							    label(i));
					} else {
						/* `label` first: one buffer per print. */
						shell_print(sh, "        %4d  %s (again, keeping %d)",
							    peak, label(i), res.swing[i]);
					}
					res.swing[i] = MAX(res.swing[i], peak);
				}
				w.cal.blocks[i] = 0;
				last_ms = k_uptime_get();
			}
			if (w.cal.blocks[i]) {
				held[c] |= BIT(r);
				down = true;
			}
		}
		/*
		 * Keys left alone on a line where another is held: K at work. A
		 * wrong K moves all of them together, while one key on its way
		 * down moves alone. So a line counts as risen by its lowest
		 * unpressed key and as dipped by its highest.
		 */
		for (uint8_t c = 0; c < res.cols; c++) {
			int16_t lo = INT16_MAX, hi = INT16_MIN;
			size_t lo_i = 0, hi_i = 0;

			for (uint8_t r = 0; held[c] && r < res.rows; r++) {
				size_t i = CELL(r, c);

				if (!IS_KEY(r, c) || (held[c] & BIT(r))) {
					continue;
				}
				if (w.cal.lo[i] < lo) {
					lo = w.cal.lo[i];
					lo_i = i;
				}
				if (w.cal.lo[i] > hi) {
					hi = w.cal.lo[i];
					hi_i = i;
				}
			}
			if (lo == INT16_MAX) {
				continue;
			}
			if (hi < res.dip) {
				res.dip = hi;
				res.dip_i = hi_i;
			}
			if (lo > res.rise) {
				res.rise = lo;
				res.rise_i = lo_i;
			}
		}
		FOR_KEYS(r, c) {
			w.cal.lo[CELL(r, c)] = INT16_MAX;
		}
		if (count == res.keys && !down) {
			break;
		}
		if (k_uptime_get() - last_ms > CALIBRATE_IDLE_MS) {
			shell_print(sh, "No key for %u s, stopping.", CALIBRATE_IDLE_MS / 1000);
			break;
		}
	}
	cal_summary(sh, dev, detect);
out:
	end(dev);
	return rc;
}

/* --- fit ------------------------------------------------------------- */

/* Lines with two or more keys: one key alone cannot give a K. */
static bool fittable(uint8_t c)
{
	return __builtin_popcount(res.mask[c]) >= 2;
}

/* Keys to ask for on a long line. Holding every key of it is awkward and
 * doesn't tighten K; fewer than this leaves too small a drop per key. */
#define FIT_ASK_KEYS 4

/* Show which keys to roll next: the first fittable line after `from` that
 * has not been rolled. False when every line has. */
static bool suggest(const struct shell *sh, uint8_t from, uint16_t rolled, bool first)
{
	uint16_t sel[EC_MAX_COLS] = {0};
	const struct view v = {.sel = sel};

	for (uint8_t k = 0; k < res.cols; k++) {
		uint8_t c = (from + k) % res.cols;

		if (fittable(c) && !(rolled & BIT(c))) {
			sel[c] = res.mask[c];
			if (__builtin_popcount(res.mask[c]) > FIT_ASK_KEYS) {
				shell_print(sh, "%s at least %u of these, as many as is comfortable, "
						"one after another. Keep them all down, then let go:",
					    first ? "Press" : "Next, press", FIT_ASK_KEYS);
			} else {
				shell_print(sh, "%s all of these one after another. Keep them all "
						"down, then let go:", first ? "Press" : "Next, press");
			}
			show(sh, &v);
			print_keys(sh, SHELL_NORMAL, true, sel);
			shell_print(sh, ".");
			return true;
		}
	}
	return false;
}

/*
 * The compensation holds reading / (K - S) constant for a key held at the
 * same depth, S being the sum of its sense line's readings. So one key read
 * in two states of the line, (v1, S1) and (v2, S2), gives
 *
 *   K = (v1 * S2 - v2 * S1) / (v1 - v2)
 *
 * A single key's answer moves by 20 % with how hard the finger pushes in each
 * state, so every key of every roll gives one estimate and the median of all
 * of them is the answer.
 */
static void fit_line(const struct shell *sh, uint8_t c, uint16_t held, uint16_t s2,
		     uint16_t *n_est)
{
	uint16_t none[EC_MAX_COLS] = {0};
	int16_t k[EC_MAX_ROWS];
	uint8_t n = 0;

	for (uint8_t r = 0; r < res.rows; r++) {
		size_t i = CELL(r, c);
		int32_t v1 = w.fit.v1[i], v2 = w.fit.v2[i];
		int64_t est = 0;

		if (!(held & BIT(r))) {
			continue;
		}
		/* The last key down was first read with the line at its fullest,
		 * so it has only one state and never gives a K. */
		if (w.fit.s1[i] == s2) {
			continue;
		}
		if (v1 - v2 >= FIT_MIN_DROP) {
			est = ((int64_t)v1 * s2 - (int64_t)v2 * w.fit.s1[i]) / (v1 - v2);
		}
		/* K under the line's own sum would turn the correction over. */
		if (est <= s2 || est > INT16_MAX) {
			none[c] |= BIT(r);
			continue;
		}
		k[n++] = est;
		if (*n_est < FIT_MAX_EST) {
			w.fit.est[(*n_est)++] = est;
		}
	}
	if (!n) {
		shell_print(sh, "Nothing to go on from that line. Keep the first keys down while "
				"adding the rest.");
		return;
	}
	qsort(k, n, sizeof(k[0]), cmp_i16);
	shell_print(sh, "%u keys held: K %d to %d, median %d.", __builtin_popcount(held), k[0],
		    k[n - 1], k[n / 2]);
	if (none[c]) {
		shell_fprintf(sh, SHELL_WARNING, "No estimate from ");
		print_keys(sh, SHELL_WARNING, false, none);
		shell_fprintf(sh, SHELL_WARNING, ".\n");
	}
}

int ecd_cmd_fit(const struct shell *sh, const struct device *dev)
{
	uint16_t prev[EC_MAX_COLS] = {0}, held[EC_MAX_COLS] = {0}, s2[EC_MAX_COLS] = {0};
	uint8_t stable[EC_MAX_COLS] = {0}, most[EC_MAX_COLS] = {0};
	uint16_t k_old = zmk_ec_direct_get_load_comp(dev);
	uint16_t n = 0, rolled = 0;
	int16_t detect, k_new;
	int64_t t0, last_ms;
	bool all_done = false;
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
	if (!suggest(sh, 0, 0, true)) {
		shell_error(sh, "No sense line has two keys, so there is nothing to fit.");
		rc = -ENODATA;
		goto out;
	}
	shell_print(sh, "Press each key about as hard as the last. Any line works, and you can "
			"stop early: it ends %u s after the last key.", FIT_IDLE_MS / 1000);

	memset(&w.fit.v1, 0, sizeof(w.fit.v1));
	t0 = last_ms = k_uptime_get();

	while (!all_done) {
		bool any = false;
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
					fit_line(sh, c, held[c], s2[c], &n);
					rolled |= BIT(c);
					all_done = !suggest(sh, c + 1, rolled, false);
				}
				most[c] = 0;
				for (uint8_t r = 0; r < res.rows; r++) {
					w.fit.v1[CELL(r, c)] = 0;
				}
			}
		}
		now = k_uptime_get();
		if (any) {
			last_ms = now;
		}
		if (now - last_ms > (n ? FIT_IDLE_MS : FIT_FIRST_MS) || now - t0 > FIT_CAP_MS) {
			break;
		}
	}
	if (all_done) {
		shell_print(sh, "That was every line.");
	}

	if (!n) {
		shell_error(sh, "No estimate. load-comp-scale stays at %u.", k_old);
		rc = -ENODATA;
		goto out;
	}
	qsort(w.fit.est, n, sizeof(w.fit.est[0]), cmp_i16);
	k_new = w.fit.est[n / 2];
	res.fit_k = k_old = k_new;
	res.fit_n = n;
	shell_print(sh, "K = %d, the median of %u estimates (%d to %d).", k_new, n, w.fit.est[0],
		    w.fit.est[n - 1]);
	if (n < 5) {
		shell_warn(sh, "Only %u estimates. Another line gives a steadier K.", n);
	}
	shell_print(sh, "load-comp-scale = <%d>;  applied until reset. Run `ecd calibrate` next.",
		    k_new);
out:
	zmk_ec_direct_set_load_comp(dev, k_old);
	end(dev);
	return rc;
}

/* --- export ---------------------------------------------------------- */

int ecd_cmd_export(const struct shell *sh, const struct device *dev)
{
	struct zmk_ec_direct_tripwire tw;
	uint16_t press, release, seen = 0;
	int16_t med;

	if (!res.have_rest && !res.have_cal) {
		shell_print(sh, "Nothing measured yet. Run `ecd rest`, `ecd fit` and "
				"`ecd calibrate` first.");
		return 0;
	}
	zmk_ec_direct_get_thresholds(dev, &press, &release);
	zmk_ec_direct_get_tripwire(dev, &tw);
	/* `label` needs the layout; begin() is not called here. */
	load_layout();

	shell_print(sh, "/* %s, %u strobes x %u sense lines, %u keys, %u samples per key */",
		    CONFIG_BOARD, res.rows, res.cols, res.keys, NAVG);
	shell_print(sh, "press-threshold = <%u>;", press);
	shell_print(sh, "release-threshold = <%u>;", release);
	shell_print(sh, "load-comp-scale = <%u>;%s", zmk_ec_direct_get_load_comp(dev),
		    res.fit_n ? "" : "  /* not fitted in this session */");
	shell_print(sh, "");
	shell_print(sh, "/* Per-key table, one row per strobe, one column per sense line.");
	shell_print(sh, " * The driver does not read these yet. */");
	if (res.have_rest) {
		print_array(sh, "resting", res.rest);
	}
	if (res.have_cal) {
		median(res.swing, true, &seen);
		if (seen) {
			print_array(sh, "swing", res.swing);
		}
	}
	shell_print(sh, "");
	if (res.fit_n) {
		shell_print(sh, "K %u from %u estimates.", res.fit_k, res.fit_n);
	}
	if (res.have_rest) {
		shell_print(sh, "Noise: median %u peak to peak (sd %u.%u), worst %u (sd %u.%u).",
			    res.pp_median, res.sd10_median / 10, res.sd10_median % 10,
			    res.pp_worst, res.sd10_worst / 10, res.sd10_worst % 10);
	}
	if (res.have_cal) {
		med = median(res.swing, true, &seen);
		if (seen) {
			shell_print(sh, "Swing: %u of %u keys, min %d, median %d, max %d.", seen,
				    res.keys, snap[0], med, snap[seen - 1]);
			shell_print(sh, "Press point %u is %u to %u %% of a key's swing.", press,
				    press * 100 / snap[seen - 1], press * 100 / snap[0]);
			/* `label` has one buffer, so one call per print. */
			shell_fprintf(sh, SHELL_NORMAL, "Unpressed keys on a line with a held one: ");
			if (res.dip) {
				shell_fprintf(sh, SHELL_NORMAL, "dipped to %+d (%s)", res.dip,
					      label(res.dip_i));
			} else {
				shell_fprintf(sh, SHELL_NORMAL, "didn't dip");
			}
			if (res.rise) {
				shell_fprintf(sh, SHELL_NORMAL, ", rose to %+d (%s)", res.rise,
					      label(res.rise_i));
			} else {
				shell_fprintf(sh, SHELL_NORMAL, ", didn't rise");
			}
			shell_print(sh, ".");
		}
	}
	if (tw.enabled) {
#if defined(CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE_THRESHOLD)
		shell_fprintf(sh, SHELL_NORMAL, "Tripwire: trips over %u idle",
			      CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE_THRESHOLD);
#endif
#if defined(CONFIG_ZMK_KSCAN_EC_DIRECT_TW_WARM_THRESHOLD)
		shell_fprintf(sh, SHELL_NORMAL, ", %u just after a key",
			      CONFIG_ZMK_KSCAN_EC_DIRECT_TW_WARM_THRESHOLD);
#endif
		shell_print(sh, ". Quiet maximum %+d, %u false trips.", tw.max_pos,
			    tw.false_trips);
	}
	return 0;
}
