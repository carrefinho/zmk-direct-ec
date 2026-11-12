/*
 * EC matrix with no analog front end, read directly by the SoC ADC.
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 *
 * Each sense line goes straight to an ADC input and floats. A key couples its
 * strobe to its sense line, so a strobe edge moves the line by an amount that
 * grows as the key goes down. Every reading is a pair: the line sampled with
 * the strobe at rest (reference), then again just after the edge (test). The
 * difference is the reading.
 *
 * Why it is built this way:
 *
 *  - Mutual, not self-capacitance. A strobe held at a fixed level looks the
 *    same to the sense line whether it is high or low, so unselected strobes
 *    do not need to be tri-stated. Only the strobe that moves is seen.
 *
 *  - A pair per reading. The raw level of a floating line drifts by more than
 *    a press over a session. The reference taken just before the edge cancels
 *    that.
 *
 *  - The converter is driven directly, not through Zephyr's ADC API. The
 *    time between the strobe edge and the sample is where the signal lives,
 *    and adc_read() puts about a millisecond there.
 *
 *  - NAVG samples per cell, reduced by a median by default. One wild sample
 *    costs nothing that way.
 *
 *  - Two tiers on nRF52840. The per-key pass costs about a milliamp and a
 *    keyboard is idle nearly all the time, so an idle board drops to a cheap
 *    tripwire pass that only answers "is anything down". See the tripwire
 *    block below.
 */

#define DT_DRV_COMPAT zmk_kscan_ec_direct

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk_ec_direct/ec_direct.h>

#include "ec_direct_priv.h"

LOG_MODULE_REGISTER(kscan_ec_direct, CONFIG_INPUT_LOG_LEVEL);

/* --- acquisition ----------------------------------------------------- */

/* Static, not on the scan thread's stack: rows*cols*NAVG int16 would overflow
 * a 1 KB stack at realistic matrix sizes. */
static int16_t ec_samples[EC_MAX_CELLS][NAVG];

/* Insertion sort over one cell's NAVG samples. For this few elements it is
 * several times faster than qsort through a compare callback on a
 * Cortex-M0+. */
static inline void sort_navg(int16_t *v)
{
	for (int i = 1; i < NAVG; i++) {
		int16_t x = v[i];
		int j = i;

		while (j > 0 && v[j - 1] > x) {
			v[j] = v[j - 1];
			j--;
		}
		v[j] = x;
	}
}

/* One cell's NAVG samples to one reading. */
static inline int32_t reduce_navg(int16_t *v)
{
	if (IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_MEAN)) {
		int32_t s = 0;

		for (uint8_t k = 0; k < NAVG; k++) {
			s += v[k];
		}
		return s / NAVG;
	}
	sort_navg(v);
	if (IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TRIMMED_MEAN) && NAVG >= 4) {
		int32_t s = 0;

		for (uint8_t k = 1; k < NAVG - 1; k++) {
			s += v[k];
		}
		return s / (NAVG - 2);
	}
	return v[NAVG / 2];
}

static void read_pass(const struct ec_direct_config *cfg, int32_t *out)
{
	static int16_t diff[EC_MAX_CELLS];
	size_t n_cells = (size_t)cfg->rows * cfg->cols;

	if (ec_backend.pass_begin) {
		ec_backend.pass_begin(cfg);
	}
	for (int k = 0; k < NAVG; k++) {
		ec_backend.sample_pass(cfg, diff);
		for (size_t i = 0; i < n_cells; i++) {
			ec_samples[i][k] = diff[i];
		}
	}
	if (ec_backend.pass_end) {
		ec_backend.pass_end(cfg);
	}
	for (uint8_t r = 0; r < cfg->rows; r++) {
		for (uint8_t c = 0; c < cfg->cols; c++) {
			size_t i = r * cfg->cols + c;

			/* A cell the pass never acquires (sense-strobe-masks) is
			 * all zeros. Do not spend a sort on it. */
			if (cfg->col_strobe_mask && !(cfg->col_strobe_mask[c] & BIT(r))) {
				out[i] = 0;
				continue;
			}
			out[i] = reduce_navg(ec_samples[i]);
		}
	}
}

/*
 * Sense-line loading compensation (load-comp-scale).
 *
 * Every reading on a sense line shares one denominator, the line's total
 * capacitance, and each pressed key adds to it. So with other keys held a
 * key reads low (5-20 % per held key, depending on how many keys share the
 * line) and a released key reads below its baseline.
 *
 * The line's own readings say how much it has grown. A reading is
 * K * c_i / D, with K the count a key would give if it were the whole line.
 * Summing the line, pressed against released:
 *
 *   D / D0 = (K - S0) / (K - S)      S = sum of readings, S0 = sum of baselines
 *
 * Multiplying every reading on the line by that ratio puts it back where it
 * would be with nothing else held: released keys return to their baseline
 * and a pressed key reads the same whatever its neighbours do. It needs no
 * idea of which keys are pressed and costs one divide per line. Noise is
 * multiplied by the same ratio.
 *
 * K is fitted, not derived: hold all but one key of a line and pick the K
 * that returns the released one to its baseline.
 */
static void load_comp(const struct ec_direct_config *cfg, struct ec_direct_data *data)
{
	int32_t k = data->load_comp_scale;

	if (!k) {
		return;
	}
	for (uint8_t c = 0; c < cfg->cols; c++) {
		int32_t s0 = 0, s = 0, den, gain;

		for (uint8_t r = 0; r < cfg->rows; r++) {
			s0 += data->baseline[r * cfg->cols + c];
			s += data->value[r * cfg->cols + c];
		}
		/* A line cannot plausibly grow past 4x. Past that the sum is
		 * garbage (a fault, a rail) and the gain must not run away. */
		den = MAX(k - s, (k - s0) / 4);
		if (den <= 0) {
			continue;
		}
		gain = ((k - s0) * 1024) / den;
		for (uint8_t r = 0; r < cfg->rows; r++) {
			data->value[r * cfg->cols + c] =
				data->value[r * cfg->cols + c] * gain / 1024;
		}
	}
}

/*
 * The loop sleeps to an absolute deadline, so the scan period is
 * poll-interval-ms and not that plus the pass time. If a pass runs long the
 * deadline rolls forward a whole slot instead of running passes back to back
 * to catch up.
 */
static int64_t period_next;

static void period_reset(void)
{
	period_next = k_uptime_ticks();
}

static void period_sleep(uint32_t period_ms)
{
	int64_t period_ticks = k_ms_to_ticks_ceil64(period_ms);
	int64_t now = k_uptime_ticks();

	period_next += period_ticks;
	if (period_next <= now) {
		period_next = now + period_ticks;
	}
	k_sleep(K_TIMEOUT_ABS_TICKS(period_next));
}

/* --- tripwire tier --------------------------------------------------- */

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE)

/*
 * The idle cost is the battery life, so an idle board does not scan keys. The
 * tripwire pass raises every strobe together and takes one pair per sense
 * line: two conversions per line instead of a whole matrix times NAVG. A line
 * that reads over its baseline by the threshold sends the driver to ACTIVE
 * with no sleep in between, so the press that tripped the wire is the one the
 * next per-key pass reports.
 *
 * The same pass runs at three rates, picked from how long ago a key was last
 * confirmed down:
 *
 *   warm     TW_WARM_PERIOD_MS, for TW_WARM_HOLD_MS after a key. Here it is a
 *            gate for the per-key pass during typing: the per-key pass runs
 *            only in a slot where a line crosses TW_WARM_THRESHOLD.
 *   idle     TRIPWIRE_PERIOD_MS, until TW_DORMANT_AFTER_MS.
 *   dormant  TW_DORMANT_PERIOD_MS after that.
 *
 * One baseline serves all three.
 */

#define TW_THRESHOLD     CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE_THRESHOLD
#define TW_IDLE_PERIOD   CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE_PERIOD_MS
#define TW_CAL_PASSES    CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE_CAL_PASSES
#define TW_TRACK_SHIFT   CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE_TRACK_SHIFT
#define TW_SETTLE_PASSES CONFIG_ZMK_KSCAN_EC_DIRECT_TW_HOLD_SETTLE_PASSES

static int16_t tw_cal[EC_MAX_COLS][TW_CAL_PASSES];

static int cmp_i16(const void *a, const void *b)
{
	return *(const int16_t *)a - *(const int16_t *)b;
}

/*
 * The tripwire baseline is a different quantity from the per-key ones: with
 * all strobes moving together it sits on a pedestal that grows with the
 * number of strobes. It is the median of a few passes taken at the tripwire's
 * own rate, because a floating line remembers the interval since the pass
 * before and reads differently back to back.
 */
static void tripwire_calibrate(const struct ec_direct_config *cfg, struct ec_direct_data *data)
{
	int32_t one[EC_MAX_COLS];

	period_reset();
	/* The first pass is a warm-up and is not kept. */
	for (int k = -1; k < TW_CAL_PASSES; k++) {
		ec_backend.tripwire_pass(cfg, one);
		if (k >= 0) {
			for (uint8_t c = 0; c < cfg->cols; c++) {
				tw_cal[c][k] = (int16_t)one[c];
			}
		}
		period_sleep(data->tw_period_ms);
	}
	for (uint8_t c = 0; c < cfg->cols; c++) {
		qsort(tw_cal[c], TW_CAL_PASSES, sizeof(int16_t), cmp_i16);
		data->tw_baseline[c] = tw_cal[c][TW_CAL_PASSES / 2];
		data->tw_base_q[c] = data->tw_baseline[c] << TW_TRACK_SHIFT;
	}
}

static void tw_marks_reset(struct ec_direct_data *data)
{
	data->tw_max_pos = 0;
	data->tw_max_pos_col = 0;
	data->tw_max_neg = 0;
	data->tw_max_neg_col = 0;
}

/* Pick the tripwire rate. Called before every tripwire pass; a change takes
 * effect at that pass's sleep. */
static void tw_schedule(struct ec_direct_data *data)
{
	int64_t since = k_uptime_get() - data->last_key_ms;
	uint8_t rate = since < CONFIG_ZMK_KSCAN_EC_DIRECT_TW_WARM_HOLD_MS
			       ? ZMK_EC_DIRECT_RATE_WARM
			       : since < CONFIG_ZMK_KSCAN_EC_DIRECT_TW_DORMANT_AFTER_MS
					 ? ZMK_EC_DIRECT_RATE_IDLE
					 : ZMK_EC_DIRECT_RATE_DORMANT;

	if (rate == data->tw_rate) {
		return;
	}
	data->tw_rate = rate;
	switch (rate) {
	case ZMK_EC_DIRECT_RATE_WARM:
		data->tw_period_ms = CONFIG_ZMK_KSCAN_EC_DIRECT_TW_WARM_PERIOD_MS;
		data->tw_gate_threshold = CONFIG_ZMK_KSCAN_EC_DIRECT_TW_WARM_THRESHOLD;
		break;
	case ZMK_EC_DIRECT_RATE_DORMANT:
		data->tw_period_ms = CONFIG_ZMK_KSCAN_EC_DIRECT_TW_DORMANT_PERIOD_MS;
		data->tw_gate_threshold = 0;
		break;
	default:
		data->tw_period_ms = TW_IDLE_PERIOD;
		data->tw_gate_threshold = 0;
		break;
	}
	LOG_DBG("tripwire rate %u: %u ms", rate, data->tw_period_ms);
}

/* The per-key hold after the last key: short while warm, since the gate
 * catches the next press at the same rate, the full hold otherwise. */
static int64_t active_hold_ms(struct ec_direct_data *data)
{
	if (k_uptime_get() - data->last_key_ms < CONFIG_ZMK_KSCAN_EC_DIRECT_TW_WARM_HOLD_MS) {
		return CONFIG_ZMK_KSCAN_EC_DIRECT_TW_WARM_ACTIVE_HOLD_MS;
	}
	return CONFIG_ZMK_KSCAN_EC_DIRECT_ACTIVE_HOLD_MS;
}

static void tier_set(struct ec_direct_data *data, enum zmk_ec_direct_tier tier)
{
	if (data->tier == tier) {
		return;
	}
	data->tier = tier;
	period_reset(); /* the two tiers have different periods */
	LOG_DBG("tier -> %s", tier == ZMK_EC_DIRECT_TIER_TRIPWIRE ? "tripwire" : "active");
}

/*
 * One tripwire slot. Returns true if a line tripped and the tier is now
 * ACTIVE; the caller then runs the per-key pass with no sleep in between.
 */
static bool tripwire_tick(const struct ec_direct_config *cfg, struct ec_direct_data *data)
{
	bool trip = false;
	int32_t pass_pos = 0, pass_neg = 0;
	uint8_t pass_pos_c = 0, pass_neg_c = 0;

	tw_schedule(data);
	ec_backend.tripwire_pass(cfg, data->tw_value);

	for (uint8_t c = 0; c < cfg->cols; c++) {
		int32_t d = data->tw_value[c] - data->tw_baseline[c];
		int32_t thr = data->tw_gate_threshold ? data->tw_gate_threshold : TW_THRESHOLD;

		if (d > pass_pos) {
			pass_pos = d;
			pass_pos_c = c;
		}
		if (d < pass_neg) {
			pass_neg = d;
			pass_neg_c = c;
		}
		if (!trip && d > thr) {
			trip = true;
			data->trips++;
			data->last_active_ms = k_uptime_get();
			data->stint_from_trip = true;
			data->stint_press = false;
			LOG_DBG("trip on line %u, %+d", c, d);
			tier_set(data, ZMK_EC_DIRECT_TIER_ACTIVE);
		} else if (!trip && TW_TRACK_SHIFT > 0) {
			/*
			 * Slow follow of the baseline. It is calibrated once, and
			 * whatever was near the board at that moment is in it: a
			 * hand resting on the case at boot would otherwise leave
			 * a line offset for good. A real press trips long before
			 * this can follow it, and a pass that tripped is skipped.
			 */
			data->tw_base_q[c] += d;
			data->tw_baseline[c] = data->tw_base_q[c] >> TW_TRACK_SHIFT;
		}
	}
	if (trip) {
		return true;
	}
	/* Quiet passes only: a pass that tripped is a press, not an excursion. */
	if (pass_pos > data->tw_max_pos) {
		data->tw_max_pos = pass_pos;
		data->tw_max_pos_col = pass_pos_c;
	}
	if (pass_neg < data->tw_max_neg) {
		data->tw_max_neg = pass_neg;
		data->tw_max_neg_col = pass_neg_c;
	}
	/* Quiet pass, converter disarmed: the one place the battery is read, so
	 * it never lands next to a press. */
	ec_battery_tick();
	period_sleep(data->tw_period_ms);
	return false;
}

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TW_HOLD_WATCH)
/*
 * One slot of the hold watch. Returns true if the slot is done, false if the
 * per-key pass has to run now (an edge, or the backstop).
 */
static bool hold_watch_tick(const struct ec_direct_config *cfg, struct ec_direct_data *data)
{
	bool edge = false;

	ec_backend.tripwire_pass(cfg, data->tw_value);
	for (uint8_t c = 0; c < cfg->cols; c++) {
		int32_t d = data->tw_value[c] - data->hold_base[c];

		/*
		 * Up is a new press (+EDGE) or the finger settling deeper, which
		 * the baseline follows. Down is only ever a release, so the
		 * baseline never follows it: a slow roll-off accumulates until
		 * it crosses -RELEASE_EDGE instead of being tracked away and
		 * left to the backstop.
		 */
		if (d > CONFIG_ZMK_KSCAN_EC_DIRECT_TW_HOLD_EDGE ||
		    d < -CONFIG_ZMK_KSCAN_EC_DIRECT_TW_HOLD_RELEASE_EDGE) {
			edge = true;
		} else if (d > 0) {
			data->hold_base[c] += d / (1 << CONFIG_ZMK_KSCAN_EC_DIRECT_TW_HOLD_TRACK_SHIFT);
		}
	}
	if (!edge && k_uptime_get() - data->hold_last_full_ms <
			     CONFIG_ZMK_KSCAN_EC_DIRECT_TW_HOLD_BACKSTOP_MS) {
		period_sleep(cfg->poll_interval_ms);
		return true;
	}
	if (edge) {
		data->hold_edges++;
		data->hold_stable = 0;
	} else {
		/* The set was already steady. One unchanged pass is enough to
		 * resume, not SETTLE_PASSES of them. */
		data->hold_backstops++;
		data->hold_stable = TW_SETTLE_PASSES - 1;
	}
	data->hold_watch = false;
	return false;
}

/* After a per-key pass with a key down: start the watch once the pressed set
 * has been steady for long enough. */
static void hold_watch_arm(const struct ec_direct_config *cfg, struct ec_direct_data *data)
{
	bool pending = data->hold_changed;

	for (size_t i = 0; !pending && i < (size_t)cfg->rows * cfg->cols; i++) {
		/* A pressed key reading under its press point is on its way out.
		 * A watch seeded now would take the half-released level as the
		 * hold and miss the rest of the drop until the backstop. */
		pending = data->confirm[i] != 0 ||
			  (data->pressed[i] &&
			   data->value[i] - data->baseline[i] < data->press_threshold);
	}
	data->hold_changed = false;
	if (pending) {
		data->hold_stable = 0;
	} else if (++data->hold_stable >= TW_SETTLE_PASSES) {
		/* Steady: seed the held baseline from one tripwire pass and let
		 * it watch from the next slot on. */
		ec_backend.tripwire_pass(cfg, data->tw_value);
		memcpy(data->hold_base, data->tw_value, cfg->cols * sizeof(data->hold_base[0]));
		data->hold_last_full_ms = data->last_active_ms;
		data->hold_watch = true;
	}
}
#endif /* TW_HOLD_WATCH */

/*
 * After a per-key pass: note activity, or drop back to the tripwire once the
 * board has been quiet for the hold. Returns true if the tier changed, in
 * which case the caller skips its sleep.
 */
static bool tripwire_after_pass(const struct ec_direct_config *cfg, struct ec_direct_data *data)
{
	bool any = false;

	for (size_t i = 0; i < (size_t)cfg->rows * cfg->cols; i++) {
		any |= data->pressed[i];
	}
	if (any) {
		data->last_active_ms = k_uptime_get();
		data->last_key_ms = data->last_active_ms;
		data->stint_press = true;
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TW_HOLD_WATCH)
		hold_watch_arm(cfg, data);
#endif
		return false;
	}
	if (k_uptime_get() - data->last_active_ms < active_hold_ms(data)) {
		return false;
	}
	/*
	 * A trip the per-key pass never confirmed. While warm that is a gate
	 * miss: noise, or a press still on its way down that the next gate
	 * pass sees again, and it cost one per-key pass. Otherwise it is a
	 * false trip and cost the whole hold, so its rate is what says whether
	 * the threshold is right.
	 */
	if (data->stint_from_trip && !data->stint_press) {
		if (data->tw_rate == ZMK_EC_DIRECT_RATE_WARM) {
			data->gate_misses++;
		} else {
			data->false_trips++;
		}
	}
	data->stint_from_trip = false;
	tw_marks_reset(data);
	tier_set(data, ZMK_EC_DIRECT_TIER_TRIPWIRE);
	return true;
}

#endif /* CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE */

/* --- scan ------------------------------------------------------------ */

/* Compare every cell against its thresholds and report the changes. */
static void report_keys(const struct device *dev)
{
	const struct ec_direct_config *cfg = dev->config;
	struct ec_direct_data *data = dev->data;

	for (uint8_t r = 0; r < cfg->rows; r++) {
		for (uint8_t c = 0; c < cfg->cols; c++) {
			size_t i = r * cfg->cols + c;
			int32_t delta = data->value[i] - data->baseline[i];
			bool now = data->pressed[i];

			/* hysteresis: separate press and release thresholds */
			if (!now && delta > data->press_threshold) {
				now = true;
			} else if (now && delta < data->release_threshold) {
				now = false;
			}

			if (now == data->pressed[i]) {
				data->confirm[i] = 0;
				continue;
			}
			if (++data->confirm[i] < CONFIG_ZMK_KSCAN_EC_DIRECT_CONFIRM_PASSES) {
				/* Seen the flip, not yet for long enough. */
				continue;
			}
			data->pressed[i] = now;
			data->confirm[i] = 0;
			data->hold_changed = true;
			input_report_abs(dev, INPUT_ABS_X, c, false, K_FOREVER);
			input_report_abs(dev, INPUT_ABS_Y, r, false, K_FOREVER);
			input_report_key(dev, INPUT_BTN_TOUCH, now, true, K_FOREVER);
		}
	}
}

static void ec_direct_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct ec_direct_config *cfg = dev->config;
	struct ec_direct_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	ec_backend.thread_start(cfg);

	/* Warm-up: the first conversions after a cold enable settle over a few
	 * passes on every part tried so far. Nothing is kept. */
	for (int i = 0; i < 8; i++) {
		read_pass(cfg, data->value);
	}
	data->recalibrate = true;
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE)
	/* Boot into ACTIVE and let the quiet timer drop it. */
	data->tier = ZMK_EC_DIRECT_TIER_ACTIVE;
	data->last_active_ms = k_uptime_get();
	data->last_key_ms = data->last_active_ms;
	data->tw_period_ms = TW_IDLE_PERIOD;
	data->tw_rate = ZMK_EC_DIRECT_RATE_IDLE;
#endif
	period_reset();

	while (1) {
		if (data->recalibrate) {
			read_pass(cfg, data->baseline);
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE)
			tripwire_calibrate(cfg, data);
			tw_marks_reset(data);
#endif
			data->recalibrate = false;
			LOG_INF("calibrated %ux%u released baselines", cfg->rows, cfg->cols);
		}
		if (data->force_pass) {
			/* refresh the grid without disturbing the tier */
			read_pass(cfg, data->value);
			load_comp(cfg, data);
			data->force_pass = false;
		}

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE)
		if (data->tier == ZMK_EC_DIRECT_TIER_TRIPWIRE && !tripwire_tick(cfg, data)) {
			continue;
		}
#endif
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TW_HOLD_WATCH)
		if (data->hold_watch && hold_watch_tick(cfg, data)) {
			continue;
		}
#endif
		read_pass(cfg, data->value);
		load_comp(cfg, data);
		report_keys(dev);

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE)
		if (tripwire_after_pass(cfg, data)) {
			continue;
		}
#endif
		period_sleep(cfg->poll_interval_ms);
	}
}

/* --- API for the shell ----------------------------------------------- */

static int wait_flag_clear(const bool *flag, int ms)
{
	for (int i = 0; i < ms / 10 && *flag; i++) {
		k_msleep(10);
	}
	return *flag ? -ETIMEDOUT : 0;
}

int zmk_ec_direct_calibrate(const struct device *dev)
{
	struct ec_direct_data *data = dev->data;

	data->recalibrate = true;
	/* Tripwire calibration takes CAL_PASSES at the tripwire's rate. */
	return wait_flag_clear(&data->recalibrate, 5000);
}

int zmk_ec_direct_force_pass(const struct device *dev)
{
	struct ec_direct_data *data = dev->data;

	data->force_pass = true;
	/* The thread picks it up at its next slot; the dormant tripwire has the
	 * longest one. */
	return wait_flag_clear(&data->force_pass, 2000);
}

int zmk_ec_direct_get_cell(const struct device *dev, uint8_t row, uint8_t col,
			   struct zmk_ec_direct_cell *out)
{
	const struct ec_direct_config *cfg = dev->config;
	struct ec_direct_data *data = dev->data;
	size_t i = row * cfg->cols + col;

	if (row >= cfg->rows || col >= cfg->cols) {
		return -EINVAL;
	}
	out->value = data->value[i];
	out->baseline = data->baseline[i];
	out->pressed = data->pressed[i];
	return 0;
}

void zmk_ec_direct_get_size(const struct device *dev, uint8_t *rows, uint8_t *cols)
{
	const struct ec_direct_config *cfg = dev->config;

	*rows = cfg->rows;
	*cols = cfg->cols;
}

void zmk_ec_direct_set_load_comp(const struct device *dev, uint16_t scale)
{
	struct ec_direct_data *data = dev->data;

	data->load_comp_scale = scale;
}

uint16_t zmk_ec_direct_get_load_comp(const struct device *dev)
{
	const struct ec_direct_data *data = dev->data;

	return data->load_comp_scale;
}

void zmk_ec_direct_set_thresholds(const struct device *dev, uint16_t press, uint16_t release)
{
	struct ec_direct_data *data = dev->data;

	data->press_threshold = press;
	data->release_threshold = release;
}

void zmk_ec_direct_get_thresholds(const struct device *dev, uint16_t *press, uint16_t *release)
{
	struct ec_direct_data *data = dev->data;

	*press = data->press_threshold;
	*release = data->release_threshold;
}

void zmk_ec_direct_get_init_status(const struct device *dev, uint8_t *stage, int *rc)
{
	struct ec_direct_data *data = dev->data;

	*stage = data->init_stage;
	*rc = data->init_rc;
}

void zmk_ec_direct_get_tripwire(const struct device *dev, struct zmk_ec_direct_tripwire *out)
{
	const struct ec_direct_config *cfg = dev->config;
	struct ec_direct_data *data = dev->data;

	out->enabled = IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE);
	out->tier = data->tier;
	out->rate = data->tw_rate;
	out->period_ms = data->tw_period_ms;
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE)
	out->threshold = data->tw_gate_threshold ? data->tw_gate_threshold : TW_THRESHOLD;
#else
	out->threshold = 0;
#endif
	out->trips = data->trips;
	out->false_trips = data->false_trips;
	out->gate_misses = data->gate_misses;
	out->hold_edges = data->hold_edges;
	out->hold_backstops = data->hold_backstops;
	out->max_pos = data->tw_max_pos;
	out->max_pos_col = data->tw_max_pos_col;
	out->max_neg = data->tw_max_neg;
	out->max_neg_col = data->tw_max_neg_col;
	out->cols = cfg->cols;
	out->baseline = data->tw_baseline;
	out->value = data->tw_value;
}

static int ec_direct_init(const struct device *dev)
{
	const struct ec_direct_config *cfg = dev->config;
	struct ec_direct_data *data = dev->data;
	int rc;

	data->dev = dev;
	data->init_stage = ZMK_EC_DIRECT_INIT_STROBES;

	for (uint8_t r = 0; r < cfg->rows; r++) {
		if (!gpio_is_ready_dt(&cfg->strobes[r])) {
			LOG_ERR("strobe %u not ready", r);
			data->init_rc = -ENODEV;
			return data->init_rc;
		}
		rc = gpio_pin_configure_dt(&cfg->strobes[r], GPIO_OUTPUT_INACTIVE);
		if (rc) {
			LOG_ERR("strobe %u configure failed (%d)", r, rc);
			data->init_rc = rc;
			return rc;
		}
	}

	rc = ec_backend.init(cfg, &data->init_stage);
	if (rc) {
		LOG_ERR("backend init failed (%d) at stage %u", rc, data->init_stage);
		data->init_rc = rc;
		return rc;
	}

	data->init_stage = ZMK_EC_DIRECT_INIT_THREAD;
	k_thread_create(&data->thread, data->stack, K_KERNEL_STACK_SIZEOF(data->stack),
			ec_direct_thread, (void *)dev, NULL, NULL,
			CONFIG_ZMK_KSCAN_EC_DIRECT_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&data->thread, "ec_direct");
	data->init_stage = ZMK_EC_DIRECT_INIT_READY;
	data->init_rc = 0;

	return 0;
}

/* --- instantiation --------------------------------------------------- */

#define STROBE_SPEC(idx, n) GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), strobe_gpios, idx),

#define COL_AIN(idx, n) DT_INST_PROP_BY_IDX(n, ain_channels, idx),

#define COL_STROBE_MASK(idx, n) DT_INST_PROP_BY_IDX(n, sense_strobe_masks, idx),

#define EC_STROBE_MASK_DEFINE(n)                                                                   \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, sense_strobe_masks) == DT_INST_PROP_LEN(n, ain_channels), \
		     "sense-strobe-masks needs one entry per sense line");                         \
	static const uint16_t ec_smask_##n[] = {                                                   \
		LISTIFY(DT_INST_PROP_LEN(n, sense_strobe_masks), COL_STROBE_MASK, (), n)};

/*
 * Without sense-strobe-masks, the masks come from the matrix transforms of
 * every physical layout whose input is this node (or, with no `input`, the
 * zmk,matrix-input chosen). Each map entry is RC(strobe, sense) =
 * (strobe << 8) + sense, so it sets bit `strobe` in its sense line's mask.
 * All of it folds to constants.
 */
#define EC_MAP_BIT(node, prop, idx, s)                                                             \
	| ((((DT_PROP_BY_IDX(node, prop, idx)) & 0xFF) == (s))                                     \
		   ? BIT((DT_PROP_BY_IDX(node, prop, idx)) >> 8)                                   \
		   : 0)

#define EC_LAYOUT_MAP_BITS(node, s)                                                                \
	(0 DT_FOREACH_PROP_ELEM_VARGS(DT_PHANDLE(node, transform), map, EC_MAP_BIT, s))

#define EC_LAYOUT_BITS(layout_node, inst_node, s)                                                  \
	| (COND_CODE_1(DT_NODE_HAS_PROP(layout_node, input),                                       \
		       (DT_SAME_NODE(DT_PHANDLE(layout_node, input), inst_node)),                  \
		       (COND_CODE_1(DT_HAS_CHOSEN(zmk_matrix_input),                                \
				    (DT_SAME_NODE(DT_CHOSEN(zmk_matrix_input), inst_node)), (0))))   \
		   ? EC_LAYOUT_MAP_BITS(layout_node, s)                                            \
		   : 0)

#define EC_DERIVED_EXPR(s, n)                                                                      \
	(0 DT_FOREACH_STATUS_OKAY_VARGS(zmk_physical_layout, EC_LAYOUT_BITS, DT_DRV_INST(n), s))

#define EC_DERIVED_MASK(s, n) EC_DERIVED_EXPR(s, n),

#define EC_DERIVED_MASK_DEFINE(n)                                                                  \
	static const uint16_t ec_smask_##n[] __maybe_unused = {                                    \
		LISTIFY(DT_INST_PROP_LEN(n, ain_channels), EC_DERIVED_MASK, (), n)};

/* No layout points at this node (a bench shield, say): scan every cell. */
#define EC_DERIVED_ANY(s, n) | EC_DERIVED_EXPR(s, n)
#define EC_MASK_PTR(n)                                                                             \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, sense_strobe_masks), (ec_smask_##n),                  \
		    (((0 LISTIFY(DT_INST_PROP_LEN(n, ain_channels), EC_DERIVED_ANY, (), n)) != 0)  \
			     ? ec_smask_##n                                                        \
			     : NULL))

#define EC_CELLS(n) (DT_INST_PROP_LEN(n, strobe_gpios) * DT_INST_PROP_LEN(n, ain_channels))

#define EC_DIRECT_INST(n)                                                                          \
	static const struct gpio_dt_spec ec_strobes_##n[] = {                                      \
		LISTIFY(DT_INST_PROP_LEN(n, strobe_gpios), STROBE_SPEC, (), n)};                    \
	static const uint8_t ec_ain_##n[] = {                                                      \
		LISTIFY(DT_INST_PROP_LEN(n, ain_channels), COL_AIN, (), n)};                         \
	EC_BACKEND_INST(n)                                                                         \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, sense_strobe_masks), (EC_STROBE_MASK_DEFINE(n)),      \
		    (EC_DERIVED_MASK_DEFINE(n)))                                                   \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, col_gpios) == DT_INST_PROP_LEN(n, ain_channels),          \
		     "col-gpios and ain-channels must list the same pins, same order");            \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, ain_channels) <= EC_MAX_COLS,                             \
		     "more sense lines than EC_MAX_COLS");                                         \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, strobe_gpios) <= EC_MAX_ROWS,                             \
		     "more strobes than EC_MAX_ROWS");                                             \
	BUILD_ASSERT(EC_CELLS(n) <= EC_MAX_CELLS, "more cells than EC_MAX_CELLS");                 \
	static int32_t ec_base_##n[EC_CELLS(n)];                                                   \
	static int32_t ec_val_##n[EC_CELLS(n)];                                                    \
	static bool ec_pressed_##n[EC_CELLS(n)];                                                   \
	static uint8_t ec_confirm_##n[EC_CELLS(n)];                                                \
	static struct ec_direct_data ec_data_##n = {                                               \
		.baseline = ec_base_##n,                                                           \
		.value = ec_val_##n,                                                               \
		.pressed = ec_pressed_##n,                                                         \
		.confirm = ec_confirm_##n,                                                         \
		.press_threshold = DT_INST_PROP(n, press_threshold),                               \
		.release_threshold = DT_INST_PROP(n, release_threshold),                           \
		.load_comp_scale = DT_INST_PROP(n, load_comp_scale),                               \
	};                                                                                         \
	static const struct ec_direct_config ec_cfg_##n = {                                        \
		.strobes = ec_strobes_##n,                                                         \
		.hw = &ec_hw_##n,                                                                  \
		.col_ain = ec_ain_##n,                                                             \
		.rows = DT_INST_PROP_LEN(n, strobe_gpios),                                         \
		.cols = DT_INST_PROP_LEN(n, ain_channels),                                         \
		.poll_interval_ms = DT_INST_PROP(n, poll_interval_ms),                             \
		.bias_us = DT_INST_PROP(n, bias_us),                                               \
		.prime = DT_INST_PROP(n, prime_conversions),                                       \
		.settle_us = DT_INST_PROP(n, settle_us),                                           \
		.bias_pull_up = DT_INST_PROP(n, bias_pull_up),                                     \
		.col_strobe_mask = EC_MASK_PTR(n),                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, ec_direct_init, NULL, &ec_data_##n, &ec_cfg_##n, POST_KERNEL,     \
			      CONFIG_INPUT_INIT_PRIORITY, NULL);

/* The sample buffers and the scan deadline are shared, so one instance. */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "one zmk,kscan-ec-direct node");

DT_INST_FOREACH_STATUS_OKAY(EC_DIRECT_INST)
