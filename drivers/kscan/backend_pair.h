/*
 * Generic per-column pair pass for zmk,kscan-ec-direct.
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 *
 * The acquisition for parts without a scan-mode converter, one cell at a
 * time:
 *
 *   bias_on(col)             weak pull engaged (down, or up with bias-pull-up),
 *                            pin stays on the ADC
 *   wait bias-us
 *   prime x convert(col)     throwaway; drags the node to a known point
 *   bias_off(col)
 *   ref  = convert(col)
 *   strobe row active       (high; or LOW for an active-low strobe, whose
 *                            idle is high -- the falling-edge form that goes
 *                            with a pull-up prime)
 *   test = convert(col)      no delay: the edge and the sample are adjacent
 *   strobe row inactive
 *   diff = test - ref        (ref - test for a falling edge: press stays +)
 *   wait settle-us
 *
 * The pull is released BEFORE ref: holding it through the pair bleeds the
 * injected step within microseconds.
 *
 * A SoC supplies the four operations below plus init; the pass, the NAVG
 * loop and everything above it are shared.
 */

#pragma once

#include <zephyr/devicetree.h>

#include "ec_direct_priv.h"

/*
 * Column pin as (port register base, pin index), derived from col-gpios.
 * The GPIO controller's `reg` is the port base on SAM0; the backend casts it
 * to its own port type.
 */
struct ec_backend_cfg {
	const uintptr_t *col_port;
	const uint8_t *col_pin;
	/* Strobe pins the same way, for a SoC `cell` that writes the port
	 * register itself instead of going through the GPIO API. */
	const uintptr_t *strobe_port;
	const uint8_t *strobe_pin;
};

#define EC_COL_PORT(idx, n) DT_REG_ADDR(DT_GPIO_CTLR_BY_IDX(DT_DRV_INST(n), col_gpios, idx)),
#define EC_COL_PIN(idx, n)  DT_GPIO_PIN_BY_IDX(DT_DRV_INST(n), col_gpios, idx),
#define EC_STROBE_PORT(idx, n)                                                                     \
	DT_REG_ADDR(DT_GPIO_CTLR_BY_IDX(DT_DRV_INST(n), strobe_gpios, idx)),
#define EC_STROBE_PIN(idx, n) DT_GPIO_PIN_BY_IDX(DT_DRV_INST(n), strobe_gpios, idx),

#define EC_BACKEND_INST(n)                                                                         \
	static const uintptr_t ec_colport_##n[] = {                                                \
		LISTIFY(DT_INST_PROP_LEN(n, col_gpios), EC_COL_PORT, (), n)};                      \
	static const uint8_t ec_colpin_##n[] = {                                                   \
		LISTIFY(DT_INST_PROP_LEN(n, col_gpios), EC_COL_PIN, (), n)};                       \
	static const uintptr_t ec_strobeport_##n[] = {                                             \
		LISTIFY(DT_INST_PROP_LEN(n, strobe_gpios), EC_STROBE_PORT, (), n)};                \
	static const uint8_t ec_strobepin_##n[] = {                                                \
		LISTIFY(DT_INST_PROP_LEN(n, strobe_gpios), EC_STROBE_PIN, (), n)};                 \
	static const struct ec_backend_cfg ec_hw_##n = {                                           \
		.col_port = ec_colport_##n,                                                        \
		.col_pin = ec_colpin_##n,                                                          \
		.strobe_port = ec_strobeport_##n,                                                  \
		.strobe_pin = ec_strobepin_##n,                                                    \
	};

struct ec_pair_ops {
	/* Claim the converter, validate cfg->col_ain against the pins, put every
	 * column pin in its sensing state. Advances *stage through the
	 * ZMK_EC_DIRECT_INIT_* values. */
	int (*init)(const struct ec_direct_config *cfg, uint8_t *stage);
	/* Engage / release the column's weak pull (down, or up when
	 * cfg->bias_pull_up). The pin stays connected to the converter
	 * throughout; only the pull changes. init must reject bias_pull_up on a
	 * part whose pull-up does not survive the analog state. */
	void (*bias_on)(const struct ec_direct_config *cfg, uint8_t col);
	void (*bias_off)(const struct ec_direct_config *cfg, uint8_t col);
	/* One single-shot conversion of the column's channel. Bounded wait;
	 * -ETIMEDOUT if the converter never signals completion. */
	int (*convert)(const struct ec_direct_config *cfg, uint8_t col, int16_t *out);
	/*
	 * Optional. One whole cell -- bias, primes, release, ref, strobe edge,
	 * test, strobe back -- in a single call, for a part where the generic
	 * sequence's per-step calls cost more than the conversions (SAMD21:
	 * ~49 us per cell against 20 us of conversion). Called with interrupts
	 * locked; must honour bias_us, prime, bias_pull_up and the strobe's
	 * polarity. NULL -> the pass runs the steps through the ops above.
	 */
	int (*cell)(const struct ec_direct_config *cfg, uint8_t col, uint8_t row, int16_t *ref,
		    int16_t *test);
};

extern const struct ec_pair_ops ec_pair_ops;
