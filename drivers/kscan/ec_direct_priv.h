/*
 * Shared driver state and the backend interface for zmk,kscan-ec-direct.
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 *
 * The scan loop, tiers, hysteresis, calibration and shell are SoC-agnostic and
 * live in kscan_ec_direct.c. Everything that touches an ADC or a pad register
 * sits behind `ec_backend`, one implementation per SoC:
 *
 *   backend_nrf.c      nRF52840 via nrfx SAADC, scan-mode block pass
 *   backend_pair.c     generic per-cell pair pass over a small SoC layer:
 *     backend_samd21.c   SAMD21, register-direct ADC and PORT
 *     backend_rp2040.c   RP2040, register-direct ADC and PADS_BANK0
 *
 * A backend supplies one sample of the whole matrix per sample_pass call
 * (test minus reference per cell, press positive). The core reduces NAVG of
 * them to one reading per cell and does the rest.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <zmk_ec_direct/ec_direct.h>

/* Sense lines are bounded by ADC channels, strobes by GPIOs, and the scratch
 * buffers by cells. */
#define EC_MAX_COLS  8
#define EC_MAX_ROWS  16
#define EC_MAX_CELLS 80
#define NAVG         CONFIG_ZMK_KSCAN_EC_DIRECT_NAVG

struct ec_backend_cfg; /* defined by the backend header included below */

struct ec_direct_config {
	const struct gpio_dt_spec *strobes;
	const struct ec_backend_cfg *hw; /* per-SoC pin data, see the backend header */
	const uint8_t *col_ain;          /* converter channel per sense line (DT order) */
	uint8_t rows;                    /* strobes */
	uint8_t cols;                    /* sense lines */
	uint16_t poll_interval_ms;
	/* Pair-pass tuning (SAMD21, RP2040); ignored by the nRF block pass. */
	uint16_t bias_us;
	uint8_t prime;
	uint16_t settle_us;
	bool bias_pull_up;
	/* Pair pass: per sense line, the strobes that cross it (bit r = strobe
	 * r). NULL scans every cell. See sense-strobe-masks in the binding. */
	const uint16_t *col_strobe_mask;
};

struct ec_direct_data {
	const struct device *dev;
	uint8_t init_stage;
	int init_rc;
	int32_t *baseline; /* rows*cols, released level */
	int32_t *value;    /* rows*cols, latest reading */
	bool *pressed;     /* rows*cols */
	uint8_t *confirm;  /* rows*cols, consecutive passes wanting a flip */
	uint16_t press_threshold; /* live-tunable from the shell */
	uint16_t release_threshold;
	uint16_t load_comp_scale; /* 0 = off, see load_comp() */
	/* Requests from other threads. The scan thread owns the converter, so
	 * they are flags it picks up rather than calls. */
	bool recalibrate;
	bool force_pass;
	/* Bench mode, see zmk_ec_direct_bench(). */
	bool bench;
	bool bench_applied; /* the host has been told about it */
	int16_t *snap;      /* where the next per-key pass is copied, or NULL */
	struct k_sem snap_done;

	/* Tripwire tier. Unused when it is compiled out. */
	enum zmk_ec_direct_tier tier;
	int32_t tw_baseline[EC_MAX_COLS]; /* all-strobes-high released level */
	int32_t tw_base_q[EC_MAX_COLS];   /* the same, scaled, for the slow tracker */
	int32_t tw_value[EC_MAX_COLS];    /* latest tripwire reading */
	uint32_t trips;
	uint32_t false_trips;
	uint32_t gate_misses;
	bool stint_from_trip; /* this ACTIVE stint was entered by a trip */
	bool stint_press;     /* ...and a key was confirmed during it */
	int32_t tw_max_pos;
	uint8_t tw_max_pos_col;
	int32_t tw_max_neg;
	uint8_t tw_max_neg_col;
	int64_t last_active_ms; /* last trip, or last pass with a key down */
	/* Last pass with a key confirmed down. A trip alone does not move it, so
	 * an unconfirmed gate crossing never extends the warm hold. */
	int64_t last_key_ms;
	uint16_t tw_period_ms;
	uint8_t tw_rate;
	int32_t tw_gate_threshold; /* warm gate threshold, 0 = the idle threshold */

	/* Hold watch: while the pressed set is steady, the slot runs the
	 * tripwire pass against hold_base and the per-key pass only on an edge
	 * or at the backstop. */
	bool hold_watch;
	bool hold_changed;   /* a cell flipped on this per-key pass */
	uint8_t hold_stable; /* per-key passes in a row with nothing pending */
	int32_t hold_base[EC_MAX_COLS];
	int64_t hold_last_full_ms;
	uint32_t hold_edges;
	uint32_t hold_backstops;

	struct k_thread thread;
	K_KERNEL_STACK_MEMBER(stack, CONFIG_ZMK_KSCAN_EC_DIRECT_THREAD_STACK_SIZE);
};

struct ec_backend {
	/* Device init: claim the converter, check the pin map, configure
	 * channels. `stage` is advanced through the ZMK_EC_DIRECT_INIT_* values
	 * so a failure can be located from the shell. */
	int (*init)(const struct ec_direct_config *cfg, uint8_t *stage);
	/* Called once on the scan thread before the first pass. The thread owns
	 * the converter from here on. */
	void (*thread_start)(const struct ec_direct_config *cfg);
	/* Optional, either may be NULL: called around the NAVG sample_pass
	 * calls of one pass, for a backend that sets up once for all of them. */
	void (*pass_begin)(const struct ec_direct_config *cfg);
	void (*pass_end)(const struct ec_direct_config *cfg);
	/* One sample of every cell: diff[r * cols + c] = test - ref. */
	void (*sample_pass)(const struct ec_direct_config *cfg, int16_t *diff);
	/* Required with CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE. One reading per
	 * sense line with every strobe raised together: out[c] = test - ref. A
	 * failed pass writes zeros, which cannot trip. */
	void (*tripwire_pass)(const struct ec_direct_config *cfg, int32_t *out);
};

extern const struct ec_backend ec_backend;

/* battery_nrf.c: called on the scan thread after a quiet tripwire pass, with
 * the converter disarmed. Takes a reading when one is due. */
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_BATTERY)
void ec_battery_tick(void);
#else
static inline void ec_battery_tick(void)
{
}
#endif

#if defined(CONFIG_SOC_SERIES_NRF52X)
#include "backend_nrf.h"
#elif defined(CONFIG_SOC_SERIES_SAMD21) || defined(CONFIG_SOC_SERIES_RP2040)
#include "backend_pair.h"
#else
#error "zmk,kscan-ec-direct has no backend for this SoC"
#endif
