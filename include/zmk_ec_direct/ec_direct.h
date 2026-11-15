/*
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>

enum zmk_ec_direct_init_stage {
	ZMK_EC_DIRECT_INIT_NONE = 0,
	ZMK_EC_DIRECT_INIT_STROBES,
	ZMK_EC_DIRECT_INIT_ADC,
	ZMK_EC_DIRECT_INIT_PIN_MAP,
	ZMK_EC_DIRECT_INIT_CHANNELS,
	ZMK_EC_DIRECT_INIT_THREAD,
	ZMK_EC_DIRECT_INIT_READY,
};

/** Last initialization stage reached and its return code. */
void zmk_ec_direct_get_init_status(const struct device *dev, uint8_t *stage, int *rc);

/** Strobes (rows) and sense lines (cols) of the scanned grid. */
void zmk_ec_direct_get_size(const struct device *dev, uint8_t *rows, uint8_t *cols);

struct zmk_ec_direct_cell {
	int32_t value;    /* latest reading, after load compensation */
	int32_t baseline; /* released level from the last calibration */
	bool pressed;
	bool scanned; /* false for a cell no layout lists; it reads 0 */
};

/** Snapshot one cell. @p row is the strobe, @p col the sense line. */
int zmk_ec_direct_get_cell(const struct device *dev, uint8_t row, uint8_t col,
			   struct zmk_ec_direct_cell *out);

/** Re-take the released baselines and wait for it. Every key must be up. */
int zmk_ec_direct_calibrate(const struct device *dev);

/**
 * Run one per-key pass now and wait for it. While the tripwire is holding the
 * grid is not scanned, so a reader wanting fresh values asks for this first.
 */
int zmk_ec_direct_force_pass(const struct device *dev);

void zmk_ec_direct_set_thresholds(const struct device *dev, uint16_t press, uint16_t release);
void zmk_ec_direct_get_thresholds(const struct device *dev, uint16_t *press, uint16_t *release);

/*
 * Bench mode, for the bring-up commands (CONFIG_ZMK_KSCAN_EC_DIRECT_BRINGUP).
 * While it is on, every slot runs the per-key pass and no key is reported, so
 * pressing keys on the board under test does not type into the terminal.
 * Keys held when it starts are released to the host, and keys still held when
 * it ends are reported pressed.
 */
void zmk_ec_direct_bench(const struct device *dev, bool on);

/**
 * Copy the next per-key pass, after load compensation, to @p out (rows * cols
 * entries, strobe-major). Bench mode only: outside it the per-key pass may not
 * run at all.
 */
int zmk_ec_direct_read_pass(const struct device *dev, int16_t *out, k_timeout_t timeout);

/** The binding's load-comp-scale; 0 turns compensation off. */
void zmk_ec_direct_set_load_comp(const struct device *dev, uint16_t scale);
uint16_t zmk_ec_direct_get_load_comp(const struct device *dev);

/*
 * Scan tiers. ACTIVE is the per-key pass at poll-interval-ms. TRIPWIRE raises
 * every strobe at once and reads one value per sense line, which only says
 * whether something is down on that line.
 */
enum zmk_ec_direct_tier {
	ZMK_EC_DIRECT_TIER_ACTIVE = 0,
	ZMK_EC_DIRECT_TIER_TRIPWIRE,
};

enum zmk_ec_direct_tripwire_rate {
	ZMK_EC_DIRECT_RATE_IDLE = 0,
	ZMK_EC_DIRECT_RATE_WARM,
	ZMK_EC_DIRECT_RATE_DORMANT,
};

struct zmk_ec_direct_tripwire {
	bool enabled;
	enum zmk_ec_direct_tier tier;
	enum zmk_ec_direct_tripwire_rate rate;
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
void zmk_ec_direct_get_tripwire(const struct device *dev, struct zmk_ec_direct_tripwire *out);
