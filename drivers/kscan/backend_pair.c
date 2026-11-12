/*
 * Generic per-column pair pass for zmk,kscan-ec-direct.
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include "backend_pair.h"

LOG_MODULE_DECLARE(kscan_ec_direct, CONFIG_INPUT_LOG_LEVEL);

static uint32_t convert_timeouts;

static int pair_init(const struct ec_direct_config *cfg, uint8_t *stage)
{
	return ec_pair_ops.init(cfg, stage);
}

static void pair_thread_start(const struct ec_direct_config *cfg)
{
	ARG_UNUSED(cfg);
}

/*
 * Column-major on purpose: the converter's channel changes once per column
 * rather than once per cell, and each key's whole pair (prime, ref, test)
 * stays on one channel.
 *
 * Each cell runs with interrupts off. The pair is ~15 us on SAMD21; an
 * interrupt landing between ref and test would widen the window the
 * floating node integrates noise over, which is the dominant idle term.
 */
static void pair_sample_pass(const struct ec_direct_config *cfg, int16_t *diff)
{
	bool timed_out = false;

	for (uint8_t c = 0; c < cfg->cols; c++) {
		for (uint8_t r = 0; r < cfg->rows; r++) {
			int16_t junk, ref = 0, test = 0;
			int rc = 0;
			/* An active-low strobe idles high and the edge is a fall, so
			 * the press moves the node DOWN; flip the sign so a press is
			 * positive to the core. Used with bias-pull-up, where a rising
			 * edge would clip against VDD. */
			bool falling = cfg->strobes[r].dt_flags & GPIO_ACTIVE_LOW;
			unsigned int key;

			/* No key can exist where this strobe never crosses this
			 * sense line; skipping it is pure scan time back. */
			if (cfg->col_strobe_mask && !(cfg->col_strobe_mask[c] & BIT(r))) {
				diff[r * cfg->cols + c] = 0;
				continue;
			}
			key = irq_lock();

			if (ec_pair_ops.cell) {
				rc = ec_pair_ops.cell(cfg, c, r, &ref, &test);
				irq_unlock(key);
				goto reduce;
			}
			ec_pair_ops.bias_on(cfg, c);
			if (cfg->bias_us) {
				k_busy_wait(cfg->bias_us);
			}
			for (uint8_t p = 0; p < cfg->prime && !rc; p++) {
				rc = ec_pair_ops.convert(cfg, c, &junk);
			}
			ec_pair_ops.bias_off(cfg, c);
			if (!rc) {
				rc = ec_pair_ops.convert(cfg, c, &ref);
			}
			gpio_pin_set_dt(&cfg->strobes[r], 1);
			if (!rc) {
				rc = ec_pair_ops.convert(cfg, c, &test);
			}
			gpio_pin_set_dt(&cfg->strobes[r], 0);
			irq_unlock(key);
reduce:

			if (rc) {
				timed_out = true;
				diff[r * cfg->cols + c] = 0;
			} else {
				diff[r * cfg->cols + c] = falling ? ref - test : test - ref;
			}
			if (cfg->settle_us) {
				k_busy_wait(cfg->settle_us);
			}
		}
	}
	if (timed_out && convert_timeouts++ < 4) {
		LOG_ERR("conversion timed out (%u so far); cells read 0", convert_timeouts);
	}
}

const struct ec_backend ec_backend = {
	.init = pair_init,
	.thread_start = pair_thread_start,
	.sample_pass = pair_sample_pass,
};
