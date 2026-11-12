/*
 * RP2040 pair-pass operations for zmk,kscan-ec-direct.
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 *
 * Register-direct ADC and PADS_BANK0. Zephyr's ADC driver must stay disabled:
 * it would own the converter and put its own pad configuration on the sense
 * pins.
 *
 * Measured on a single key: press
 * ~466 counts, primed pull-down pair sd 0.63 at NAVG 8 and 1.4 at NAVG 1, no
 * single-shot glitch in 6144 primed pairs, carryover <= 0.23 %.
 *
 * Two RP2040 facts shape this file:
 *  - One conversion is 96 clk_adc cycles = 2 us at 48 MHz, fixed. There is
 *    no sample-time register, so the only bias knob is prime-then-release.
 *  - Four external inputs, AINSEL 0..3 = GPIO26..29, and AINSEL 4 is the
 *    temperature sensor, the only stiff internal source.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <hardware/address_mapped.h>
#include <hardware/structs/adc.h>
#include <hardware/structs/clocks.h>
#include <hardware/structs/pads_bank0.h>
#include <hardware/structs/resets.h>
#include <hardware/structs/sio.h>

#include "backend_pair.h"

LOG_MODULE_DECLARE(kscan_ec_direct, CONFIG_INPUT_LOG_LEVEL);

#define AIN_FIRST_GPIO 26
#define AIN_EXTERNAL   4        /* AINSEL 0..3 are pads */
#define AIN_TEMP       4
#define AIN_NONE       0xff

/* A conversion is ~250 core cycles; anything reaching these bounds is broken
 * hardware, not a slow converter. */
#define CONV_SPINS  20000U
#define SETUP_SPINS 100000U

static uint8_t cur_ain = AIN_NONE;

static inline io_rw_32 *col_pad(const struct ec_direct_config *cfg, uint8_t col)
{
	return &pads_bank0_hw->io[cfg->hw->col_pin[col]];
}

/*
 * The pad stays in the datasheet's ADC state (IE=0, OD=1, FUNCSEL untouched
 * at NULL); only PDE/PUE move. The pull acts in every mux and input-buffer
 * state on this part, so there is no PMUXEN-style trap.
 * Atomic set/clear aliases, so nothing else in the pad register is rewritten.
 */
static void rp_bias_on(const struct ec_direct_config *cfg, uint8_t col)
{
	hw_set_bits(col_pad(cfg, col), cfg->bias_pull_up ? PADS_BANK0_GPIO0_PUE_BITS
							 : PADS_BANK0_GPIO0_PDE_BITS);
}

static void rp_bias_off(const struct ec_direct_config *cfg, uint8_t col)
{
	hw_clear_bits(col_pad(cfg, col), PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS);
}

static inline void adc_select(uint8_t ain)
{
	if (ain != cur_ain) {
		hw_write_masked(&adc_hw->cs, (uint32_t)ain << ADC_CS_AINSEL_LSB,
				ADC_CS_AINSEL_BITS);
		cur_ain = ain;
	}
}

static inline int adc_conv(int16_t *out)
{
	uint32_t n = CONV_SPINS;

	hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
	while (!(adc_hw->cs & ADC_CS_READY_BITS) && --n) {
	}
	if (!n) {
		return -ETIMEDOUT;
	}
	*out = (int16_t)adc_hw->result;
	return 0;
}

static int rp_convert(const struct ec_direct_config *cfg, uint8_t col, int16_t *out)
{
	adc_select(cfg->col_ain[col]);
	return adc_conv(out);
}

/*
 * The whole cell in one call, as on SAMD21. Four single-shot conversions are
 * ~12.5 us here (2.0 us each plus the trigger-and-poll), and through the ops
 * and the GPIO API a cell took 15.1 us. Same sequence as
 * the generic pass, with the strobe written straight to SIO.
 *
 * The pad register is written whole: the sense pad's only other state is OD
 * alone (see init), so this is the pull on and off without the alias write.
 * All strobes are in bank 0, the only one SIO's set/clr registers reach.
 */
__attribute__((optimize("O2")))
static int rp_cell(const struct ec_direct_config *cfg, uint8_t col, uint8_t row, int16_t *ref,
		   int16_t *test)
{
	io_rw_32 *pad = col_pad(cfg, col);
	uint32_t sbit = BIT(cfg->hw->strobe_pin[row]);
	bool falling = cfg->strobes[row].dt_flags & GPIO_ACTIVE_LOW;
	int16_t junk;
	int rc = 0;

	adc_select(cfg->col_ain[col]);

	*pad = PADS_BANK0_GPIO0_OD_BITS |
	       (cfg->bias_pull_up ? PADS_BANK0_GPIO0_PUE_BITS : PADS_BANK0_GPIO0_PDE_BITS);
	if (cfg->bias_us) {
		k_busy_wait(cfg->bias_us);
	}
	for (uint8_t p = 0; p < cfg->prime && !rc; p++) {
		rc = adc_conv(&junk);
	}
	*pad = PADS_BANK0_GPIO0_OD_BITS;
	if (!rc) {
		rc = adc_conv(ref);
	}
	if (falling) {
		sio_hw->gpio_clr = sbit;
	} else {
		sio_hw->gpio_set = sbit;
	}
	if (!rc) {
		rc = adc_conv(test);
	}
	if (falling) {
		sio_hw->gpio_set = sbit;
	} else {
		sio_hw->gpio_clr = sbit;
	}
	return rc;
}

static int adc_setup(void)
{
	const uint32_t auxsrc = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB
				<< CLOCKS_CLK_ADC_CTRL_AUXSRC_LSB;
	clock_hw_t *clk = &clocks_hw->clk[clk_adc];
	uint32_t n;

	/*
	 * clk_adc = pll_usb 48 MHz / 1. Zephyr's clock driver normally sets this
	 * already; only touch it if it differs, and stop the clock before
	 * switching the aux mux, which is not glitchless.
	 */
	if ((clk->ctrl & (CLOCKS_CLK_ADC_CTRL_ENABLE_BITS | CLOCKS_CLK_ADC_CTRL_AUXSRC_BITS)) !=
		    (CLOCKS_CLK_ADC_CTRL_ENABLE_BITS | auxsrc) ||
	    clk->div != (1u << CLOCKS_CLK_ADC_DIV_INT_LSB)) {
		hw_clear_bits(&clk->ctrl, CLOCKS_CLK_ADC_CTRL_ENABLE_BITS);
		k_busy_wait(1);
		clk->div = 1u << CLOCKS_CLK_ADC_DIV_INT_LSB;
		hw_write_masked(&clk->ctrl, auxsrc, CLOCKS_CLK_ADC_CTRL_AUXSRC_BITS);
		hw_set_bits(&clk->ctrl, CLOCKS_CLK_ADC_CTRL_ENABLE_BITS);
	}

	hw_set_bits(&resets_hw->reset, RESETS_RESET_ADC_BITS);
	hw_clear_bits(&resets_hw->reset, RESETS_RESET_ADC_BITS);
	n = SETUP_SPINS;
	while (!(resets_hw->reset_done & RESETS_RESET_ADC_BITS) && --n) {
	}
	if (!n) {
		return -ETIMEDOUT;
	}

	/* Single-shot, FIFO off, no pacing. The temperature sensor stays
	 * powered, as it was for every measurement behind this backend. */
	adc_hw->cs = ADC_CS_EN_BITS | ADC_CS_TS_EN_BITS;
	adc_hw->div = 0;
	adc_hw->fcs = 0;
	n = SETUP_SPINS;
	while (!(adc_hw->cs & ADC_CS_READY_BITS) && --n) {
	}
	if (!n) {
		return -ETIMEDOUT;
	}
	cur_ain = AIN_NONE;
	return 0;
}

static int rp_init(const struct ec_direct_config *cfg, uint8_t *stage)
{
	const uintptr_t bank0 = DT_REG_ADDR(DT_NODELABEL(gpio0));
	int rc;

	*stage = ZMK_EC_DIRECT_INIT_PIN_MAP;
	for (uint8_t c = 0; c < cfg->cols; c++) {
		uint8_t ain = cfg->col_ain[c];
		uint8_t pin = cfg->hw->col_pin[c];

		if (ain >= AIN_EXTERNAL) {
			LOG_ERR("col %u: AINSEL %u is not an external input", c, ain);
			return -EINVAL;
		}
		if (cfg->hw->col_port[c] != bank0 || pin != AIN_FIRST_GPIO + ain) {
			LOG_ERR("col %u: AINSEL %u is GPIO%u but col-gpios is GPIO%u", c, ain,
				AIN_FIRST_GPIO + ain, pin);
			return -ENODEV;
		}
		LOG_INF("col %u: GPIO%u = AINSEL %u", c, pin, ain);
	}

	*stage = ZMK_EC_DIRECT_INIT_ADC;
	rc = adc_setup();
	if (rc) {
		LOG_ERR("ADC setup failed (%d): reset or READY never completed", rc);
		return rc;
	}

	*stage = ZMK_EC_DIRECT_INIT_CHANNELS;
	for (uint8_t c = 0; c < cfg->cols; c++) {
		/* The datasheet's ADC pad state and the only one these pads
		 * leave: input buffer off, output disabled, no pull. FUNCSEL is
		 * NULL out of reset and nothing here claims the pin. */
		*col_pad(cfg, c) = PADS_BANK0_GPIO0_OD_BITS;
	}

	/* The first conversion after enable is not trusted. */
	int16_t junk;

	adc_select(cfg->col_ain[0]);
	return adc_conv(&junk);
}

const struct ec_pair_ops ec_pair_ops = {
	.init = rp_init,
	.bias_on = rp_bias_on,
	.bias_off = rp_bias_off,
	.convert = rp_convert,
	.cell = rp_cell,
};
