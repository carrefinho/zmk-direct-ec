/*
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>

enum zmk_direct_ec_init_stage {
	ZMK_DIRECT_EC_INIT_NONE = 0,
	ZMK_DIRECT_EC_INIT_STROBES,
	ZMK_DIRECT_EC_INIT_ADC,
	ZMK_DIRECT_EC_INIT_PIN_MAP,
	ZMK_DIRECT_EC_INIT_CHANNELS,
	ZMK_DIRECT_EC_INIT_THREAD,
	ZMK_DIRECT_EC_INIT_READY,
};

/** Last initialization stage reached and its return code. */
void zmk_direct_ec_get_init_status(const struct device *dev, uint8_t *stage, int *rc);

/** Strobes (rows) and sense lines (cols) of the scanned grid. */
void zmk_direct_ec_get_size(const struct device *dev, uint8_t *rows, uint8_t *cols);

struct zmk_direct_ec_cell {
	int32_t value;    /* latest reading, after load compensation */
	int32_t baseline; /* released level from the last calibration */
	bool pressed;
	bool scanned; /* false for a cell no layout lists; it reads 0 */
	int16_t swing;          /* 0 = none, the shared counts apply */
	uint16_t press_point;   /* counts above baseline, in force now */
	uint16_t release_point;
};

/** Snapshot one cell. @p row is the strobe, @p col the sense line. */
int zmk_direct_ec_get_cell(const struct device *dev, uint8_t row, uint8_t col,
			   struct zmk_direct_ec_cell *out);

/** Re-take the released baselines and wait for it. Every key must be up. */
int zmk_direct_ec_calibrate(const struct device *dev);

/**
 * Run one per-key pass now and wait for it. While the tripwire is holding the
 * grid is not scanned, so a reader wanting fresh values asks for this first.
 */
int zmk_direct_ec_force_pass(const struct device *dev);

/*
 * A key with a swing presses at actuation-percent of it and releases at two
 * thirds of that. A key without one uses the shared counts below. Every
 * setter lasts until reset and returns once the scan uses the new points.
 */
int zmk_direct_ec_set_thresholds(const struct device *dev, uint16_t press, uint16_t release);
void zmk_direct_ec_get_thresholds(const struct device *dev, uint16_t *press, uint16_t *release);
int zmk_direct_ec_set_actuation(const struct device *dev, uint8_t percent);
uint8_t zmk_direct_ec_get_actuation(const struct device *dev);
/** Replace every swing: rows * cols entries, strobe-major, 0 = none. */
int zmk_direct_ec_set_swings(const struct device *dev, const int16_t *swing);

/*
 * Bench mode, for the bring-up commands (CONFIG_ZMK_KSCAN_DIRECT_EC_BRINGUP).
 * While it is on, every slot runs the per-key pass and no key is reported, so
 * pressing keys on the board under test does not type into the terminal.
 * Keys held when it starts are released to the host, and keys still held when
 * it ends are reported pressed.
 */
void zmk_direct_ec_bench(const struct device *dev, bool on);

/**
 * Copy the next per-key pass, after load compensation, to @p out (rows * cols
 * entries, strobe-major). Bench mode only: outside it the per-key pass may not
 * run at all.
 */
int zmk_direct_ec_read_pass(const struct device *dev, int16_t *out, k_timeout_t timeout);

/** The binding's load-comp-scale; 0 turns compensation off. */
void zmk_direct_ec_set_load_comp(const struct device *dev, uint16_t scale);
uint16_t zmk_direct_ec_get_load_comp(const struct device *dev);

/*
 * Scan tiers. ACTIVE is the per-key pass at poll-interval-ms. TRIPWIRE raises
 * every strobe at once and reads one value per sense line, which only says
 * whether something is down on that line.
 */
enum zmk_direct_ec_tier {
	ZMK_DIRECT_EC_TIER_ACTIVE = 0,
	ZMK_DIRECT_EC_TIER_TRIPWIRE,
};

enum zmk_direct_ec_tripwire_rate {
	ZMK_DIRECT_EC_RATE_IDLE = 0,
	ZMK_DIRECT_EC_RATE_WARM,
	ZMK_DIRECT_EC_RATE_DORMANT,
};

struct zmk_direct_ec_tripwire {
	bool enabled;
	enum zmk_direct_ec_tier tier;
	enum zmk_direct_ec_tripwire_rate rate;
	uint16_t period_ms;      /* the tripwire's current period */
	uint16_t threshold;      /* the threshold in force at that rate */
	uint32_t trips;
	uint32_t false_trips;    /* idle trips no per-key pass confirmed */
	uint32_t gate_misses;    /* warm gate crossings no per-key pass confirmed */
	uint32_t hold_edges;     /* hold watch: per-key passes run for an edge */
	uint32_t hold_backstops; /* ...and for the backstop timer */
	/*
	 * Largest quiet readings since the tier was last entered, per sign.
	 * Only max_pos is a margin: the trip test is `delta > threshold`, so a
	 * negative excursion cannot trip however large it is.
	 */
	int32_t max_pos;
	uint8_t max_pos_col;
	int32_t max_neg;
	uint8_t max_neg_col;
	uint8_t cols;
	const int32_t *baseline; /* cols entries, released tripwire level */
	const int32_t *value;    /* cols entries, last tripwire pass */
};

/** Snapshot the tier state. */
void zmk_direct_ec_get_tripwire(const struct device *dev, struct zmk_direct_ec_tripwire *out);
