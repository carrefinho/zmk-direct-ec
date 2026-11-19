/*
 * SAMD21 pair-pass operations for zmk,kscan-direct-ec.
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 *
 * Register-direct ADC and PORT. Zephyr's ADC driver must stay disabled: it
 * would own the converter, and its blocking adc_read() puts a millisecond
 * between the strobe edge and the sample.
 *
 * Every SYNCBUSY wait is bounded. A missed sync is the silent failure mode on
 * this part -- the register write is dropped, nothing reports it, and the
 * numbers stay plausible.
 *
 * Measured on a single key: ~5.5 us per
 * conversion, press ~510 counts. With bias-pull-up and active-low strobes
 * (the shield's setting) idle sd 1.3-1.5 at NAVG 8 and 3.0-3.3 at NAVG 1 with
 * no outliers; with the pull-down prime 1.6-1.9 at NAVG 8 behind a one-sided
 * ~-75-count tail that makes NAVG 1 unusable (sd 23). A constant no-edge
 * offset (~+150 pull-down, ~-160 pull-up before the sign flip) is absorbed by
 * the calibrated baseline.
 */

#include <zephyr/logging/log.h>

#include <soc.h>

#include "backend_pair.h"

LOG_MODULE_DECLARE(kscan_direct_ec, CONFIG_INPUT_LOG_LEVEL);

/*
 * MUXPOS -> pad, SAMD21G18A. ain-channels carries MUXPOS, col-gpios the same
 * pin as a GPIO, and init checks they agree: a mismatch converts a different
 * pad than it biases, which reads as a dead column rather than as an error.
 * Entries 8-15 (PB00-PB07) do not exist on the G package; they are listed so
 * the index arithmetic stays honest and are rejected by the port check.
 */
#define AIN_NONE 0xff
static const uint8_t ain_pin[20] = {
	2, 3, 8, 9, 4, 5, 6, 7,             /* PA02 PA03 PB08 PB09 PA04..PA07 */
	0, 1, 2, 3, 4, 5, 6, 7,             /* PB00..PB07 */
	8, 9, 10, 11,                       /* PA08..PA11 */
};
/* 0 = PORTA, 1 = PORTB for the entry above. */
static const uint8_t ain_grp[20] = {
	0, 0, 1, 1, 0, 0, 0, 0,
	1, 1, 1, 1, 1, 1, 1, 1,
	0, 0, 0, 0,
};

/* Loop bounds, not timeouts in any real sense: a conversion is 5.5 us and a
 * sync a few cycles, so anything that reaches these is broken hardware. */
#define SYNC_SPINS 100000U
#define CONV_SPINS 20000U

static bool adc_ok;             /* cleared by any bounded wait that expired */
static uint8_t cur_muxpos = AIN_NONE;

static inline PortGroup *col_group(const struct direct_ec_config *cfg, uint8_t col)
{
	return (PortGroup *)cfg->hw->col_port[col];
}

static inline bool adc_sync(void)
{
	uint32_t n = SYNC_SPINS;

	while (ADC->STATUS.bit.SYNCBUSY && --n) {
	}
	if (!n) {
		adc_ok = false;
	}
	return n != 0;
}

static inline bool gclk_sync(void)
{
	uint32_t n = SYNC_SPINS;

	while (GCLK->STATUS.bit.SYNCBUSY && --n) {
	}
	return n != 0;
}

/*
 * The pad stays exactly as init left it: DIR=0, PMUXEN=0, INEN=0. Only PULLEN
 * moves; OUT selects the direction while DIR=0 (Table 23-2) and is rewritten
 * every time because nothing else guarantees it across a reset path.
 *
 * PMUXEN must never be set on a sense pad. Under peripheral function B the
 * pull is lost, and with PMUXEN=0 the ADC converts the pad perfectly well --
 * so "connect the pin to the ADC" is a no-op here and there is no pinctrl.
 */
static void samd_bias_on(const struct direct_ec_config *cfg, uint8_t col)
{
	PortGroup *g = col_group(cfg, col);
	uint8_t pin = cfg->hw->col_pin[col];

	if (cfg->bias_pull_up) {
		g->OUTSET.reg = BIT(pin);
	} else {
		g->OUTCLR.reg = BIT(pin);
	}
	g->PINCFG[pin].bit.PULLEN = 1;
}

static void samd_bias_off(const struct direct_ec_config *cfg, uint8_t col)
{
	PortGroup *g = col_group(cfg, col);

	g->PINCFG[cfg->hw->col_pin[col]].bit.PULLEN = 0;
}

/*
 * Single-shot, never free-running. On the rev D die a fixed charge packet is
 * drawn from a high-Z node in a random fraction of single-shot conversions
 * (reads ~75 counts low on the electrode); free-running has no discrete event
 * but is sd ~25 continuous at every release phase, so it is no better. The
 * event's probability falls with the node's level -- 16 % near 500 counts,
 * 1 % at 2870, none above 3600 -- which is
 * why the shield primes with the pull-UP (bias-pull-up + active-low strobes):
 * the node then sits at ~3900 / ~3200 and the event does not occur. Under a
 * pull-DOWN prime it is ~10 % per conversion and only the core's median over
 * NAVG 8 hides it, so TRIMMED_MEAN must stay off in that configuration.
 * Nearest errata item is DS80000760 1.4.1, but that is a linearity item scoped
 * to 125 C and its free-run workaround does not apply; mechanism unproven.
 */
static ALWAYS_INLINE int conv_raw(int16_t *out)
{
	uint32_t n = CONV_SPINS;

	ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
	ADC->SWTRIG.reg = ADC_SWTRIG_START;
	while (!(ADC->INTFLAG.reg & ADC_INTFLAG_RESRDY) && --n) {
	}
	if (!n) {
		adc_ok = false;
		return -ETIMEDOUT;
	}
	*out = (int16_t)ADC->RESULT.reg;
	return 0;
}

/* The MUXPOS write needs a sync; skipping it when the channel has not
 * changed is most of why the pass is column-major. */
static ALWAYS_INLINE int select_col(const struct direct_ec_config *cfg, uint8_t col)
{
	uint8_t ain = cfg->col_ain[col];

	if (ain != cur_muxpos) {
		ADC->INPUTCTRL.bit.MUXPOS = ain;
		if (!adc_sync()) {
			return -ETIMEDOUT;
		}
		cur_muxpos = ain;
	}
	return 0;
}

static int samd_convert(const struct direct_ec_config *cfg, uint8_t col, int16_t *out)
{
	int rc = select_col(cfg, col);

	return rc ? rc : conv_raw(out);
}

/*
 * The whole cell in one call. A single-shot conversion is a fixed 10 CLK_ADC
 * cycles, 5.0 us at 2 MHz (the clock source's speed does not change it), so
 * four of them are 20 us -- but run step by step through the ops and the GPIO
 * API a cell took ~49 us. Same sequence as the
 * generic pass, with the pull and the strobe written straight to PORT.
 *
 * PINCFG is written whole: the sense pad's only other state is 0 (see init),
 * so this is PULLEN on and off without a read-modify-write.
 */
__attribute__((optimize("O2")))
static int samd_cell(const struct direct_ec_config *cfg, uint8_t col, uint8_t row, int16_t *ref,
		     int16_t *test)
{
	PortGroup *g = col_group(cfg, col);
	uint8_t pin = cfg->hw->col_pin[col];
	PortGroup *sg = (PortGroup *)cfg->hw->strobe_port[row];
	uint32_t sbit = BIT(cfg->hw->strobe_pin[row]);
	bool falling = cfg->strobes[row].dt_flags & GPIO_ACTIVE_LOW;
	int16_t junk;
	int rc = select_col(cfg, col);

	if (rc) {
		return rc;
	}

	if (cfg->bias_pull_up) {
		g->OUTSET.reg = BIT(pin);
	} else {
		g->OUTCLR.reg = BIT(pin);
	}
	g->PINCFG[pin].reg = PORT_PINCFG_PULLEN;
	if (cfg->bias_us) {
		k_busy_wait(cfg->bias_us);
	}
	for (uint8_t p = 0; p < cfg->prime && !rc; p++) {
		rc = conv_raw(&junk);
	}
	g->PINCFG[pin].reg = 0;
	if (!rc) {
		rc = conv_raw(ref);
	}
	if (falling) {
		sg->OUTCLR.reg = sbit;
	} else {
		sg->OUTSET.reg = sbit;
	}
	if (!rc) {
		rc = conv_raw(test);
	}
	if (falling) {
		sg->OUTSET.reg = sbit;
	} else {
		sg->OUTCLR.reg = sbit;
	}
	return rc;
}

static int adc_setup(void)
{
	uint32_t lin, bias, n;

	adc_ok = true;
	cur_muxpos = AIN_NONE;

	PM->APBCMASK.reg |= PM_APBCMASK_ADC;

	/*
	 * Build GCLK generator 3 here. Zephyr's SoC init only builds it when
	 * CONFIG_ADC_SAM0=y, and the Zephyr ADC driver is disabled on this
	 * board -- without a clock the SWRST below never completes and the
	 * bounded wait is all that stops it spinning forever.
	 * DFLL48M / 6 = 8 MHz, crystal-accurate through XOSC32K.
	 */
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(3) | GCLK_GENDIV_DIV(6);
	if (!gclk_sync()) {
		return -ETIMEDOUT;
	}
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(3) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_GENEN;
	if (!gclk_sync()) {
		return -ETIMEDOUT;
	}
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID_ADC;
	if (!gclk_sync()) {
		return -ETIMEDOUT;
	}

	ADC->CTRLA.bit.ENABLE = 0;
	if (!adc_sync()) {
		return -ETIMEDOUT;
	}
	ADC->CTRLA.bit.SWRST = 1;
	if (!adc_sync()) {
		return -ETIMEDOUT;
	}
	n = SYNC_SPINS;
	while (ADC->CTRLA.bit.SWRST && --n) {
	}
	if (!n) {
		adc_ok = false;
		return -ETIMEDOUT;
	}

	/* Factory linearity and bias trim. SWRST clears CALIB, and without it
	 * the converter is out of spec. LINEARITY_0 is bits 4:0, so
	 * LINEARITY_1 shifts by 5. */
	lin = (*(uint32_t *)ADC_FUSES_LINEARITY_0_ADDR & ADC_FUSES_LINEARITY_0_Msk) >>
	      ADC_FUSES_LINEARITY_0_Pos;
	lin |= ((*(uint32_t *)ADC_FUSES_LINEARITY_1_ADDR & ADC_FUSES_LINEARITY_1_Msk) >>
		ADC_FUSES_LINEARITY_1_Pos) << 5;
	bias = (*(uint32_t *)ADC_FUSES_BIASCAL_ADDR & ADC_FUSES_BIASCAL_Msk) >>
	       ADC_FUSES_BIASCAL_Pos;
	ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(lin);

	/* 8 MHz / 4 = 2 MHz CLK_ADC, 12-bit, single-shot. */
	ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_12BIT;
	if (!adc_sync()) {
		return -ETIMEDOUT;
	}
	ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0);

	/*
	 * INTVCC1 with GAIN_DIV2 puts full scale at VDDANA, and no REFCOMP.
	 * The 1X references were measured to make the errata glitch worse;
	 * keep this combination.
	 */
	ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;
	ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXNEG_GND |
			     ADC_INPUTCTRL_MUXPOS(0);
	if (!adc_sync()) {
		return -ETIMEDOUT;
	}

	ADC->EVCTRL.reg = 0;                    /* no event-triggered starts */
	ADC->INTFLAG.reg = ADC_INTFLAG_MASK;
	ADC->CTRLA.bit.ENABLE = 1;
	if (!adc_sync()) {
		return -ETIMEDOUT;
	}

	/* First conversion after enable is not trustworthy; throw it away. */
	n = CONV_SPINS;
	ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
	ADC->SWTRIG.reg = ADC_SWTRIG_START;
	while (!ADC->INTFLAG.bit.RESRDY && --n) {
	}
	if (!n) {
		adc_ok = false;
		return -ETIMEDOUT;
	}
	(void)ADC->RESULT.reg;

	return adc_ok ? 0 : -EIO;
}

static int samd_init(const struct direct_ec_config *cfg, uint8_t *stage)
{
	uintptr_t porta = (uintptr_t)&PORT->Group[0];
	int rc;

	*stage = ZMK_DIRECT_EC_INIT_ADC;
	rc = adc_setup();
	if (rc) {
		LOG_ERR("ADC setup failed (%d): a sync or conversion never completed", rc);
		return rc;
	}

	*stage = ZMK_DIRECT_EC_INIT_PIN_MAP;
	for (uint8_t c = 0; c < cfg->cols; c++) {
		uint8_t ain = cfg->col_ain[c];
		uintptr_t base = cfg->hw->col_port[c];
		uint8_t pin = cfg->hw->col_pin[c];
		uint8_t grp = (base - porta) / sizeof(PortGroup);

		if (ain >= ARRAY_SIZE(ain_pin)) {
			LOG_ERR("col %u: MUXPOS %u is not an external input", c, ain);
			return -EINVAL;
		}
		if (grp > 1 || ain_grp[ain] != grp || ain_pin[ain] != pin) {
			LOG_ERR("col %u: MUXPOS %u is P%c%02u but col-gpios is P%c%02u", c, ain,
				'A' + ain_grp[ain], ain_pin[ain], 'A' + grp, pin);
			return -ENODEV;
		}
		LOG_INF("col %u: P%c%02u = MUXPOS %u", c, 'A' + grp, pin, ain);
	}

	*stage = ZMK_DIRECT_EC_INIT_CHANNELS;
	for (uint8_t c = 0; c < cfg->cols; c++) {
		PortGroup *g = col_group(cfg, c);
		uint8_t pin = cfg->hw->col_pin[c];

		/* The sensing state, and the only state these pads ever leave:
		 * input, no peripheral mux, input buffer off, no pull. From here
		 * bias_on/off touch PULLEN and nothing else. A pad must never be
		 * output and pulled at once -- the output driver silently
		 * disables the pull. */
		g->DIRCLR.reg = BIT(pin);
		g->OUTCLR.reg = BIT(pin);
		g->PINCFG[pin].reg = 0;
	}

	return 0;
}

const struct ec_pair_ops ec_pair_ops = {
	.init = samd_init,
	.bias_on = samd_bias_on,
	.bias_off = samd_bias_off,
	.convert = samd_convert,
	.cell = samd_cell,
};
