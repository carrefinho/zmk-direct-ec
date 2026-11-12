/*
 * nRF52840 backend for zmk,kscan-ec-direct, through nrfx.
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 *
 * The sense lines are never driven. On this part the SAADC disconnects a
 * pin's GPIO output driver while its AIN is selected, and the line does not
 * need precharging anyway: repeated sampling biases the floating node through
 * the sample-and-hold, and a bare strobe edge gives the full signal.
 *
 * Per-key pass: scan mode, every sense line converted per SAMPLE, one arm for
 * all NAVG samples. Tripwire pass: one line at a time, its reference and test
 * back to back.
 */

#include <zephyr/logging/log.h>

#include "backend_nrf.h"

LOG_MODULE_DECLARE(kscan_ec_direct, CONFIG_INPUT_LOG_LEVEL);

static uint32_t col_chan_mask;

/*
 * nrfy, not the raw HAL: it puts the barriers around every task and event.
 *
 * The enable in arm and the disable in disarm are a pair. Without the enable,
 * START never yields STARTED and the pass stalls. The disable after every
 * pass is the largest power lever in the driver: an enabled SAADC draws on
 * the order of a milliamp between passes.
 */
void ec_saadc_arm(int16_t *buf, uint16_t cnt)
{
	nrfy_saadc_buffer_t d = {.p_buffer = buf, .length = cnt};

	nrfy_saadc_enable(NRF_SAADC);
	nrfy_saadc_buffer_set(NRF_SAADC, &d, true, true);
}

/* The SAMPLE that fills the buffer: waits for END. */
void ec_saadc_convert_wait(int16_t *buf, uint16_t cnt)
{
	nrfy_saadc_buffer_t d = {.p_buffer = buf, .length = cnt};

	nrfy_saadc_sample_start(NRF_SAADC, &d);
}

/*
 * Wait for n per-channel DONE events, the only per-conversion completion
 * signal there is mid-buffer: BUSY clears before the conversion lands and
 * AMOUNT only updates after END. Bounded, because moving a strobe during a
 * conversion and then waiting forever is a dead keyboard at full current.
 */
bool ec_saadc_wait_done(uint8_t n)
{
	for (uint8_t i = 0; i < n; i++) {
		uint32_t g = 0;

		nrf_barrier_r();
		while (!nrf_saadc_event_check(NRF_SAADC, NRF_SAADC_EVENT_DONE)) {
			if (++g > 100000u) {
				return false;
			}
			nrf_barrier_r();
		}
		nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_DONE);
	}
	return true;
}

void ec_saadc_disarm(void)
{
	nrfy_saadc_abort(NRF_SAADC, NULL);
	nrfy_saadc_disable(NRF_SAADC);
}

/* --- per-key pass ---------------------------------------------------- */

/*
 * Reference scans are interleaved with the test scans. One reference for the
 * whole pass is not enough once a board has more than a few strobes: the
 * floating line moves between the reference and a test taken many scans
 * later, and idle noise climbs strobe by strobe.
 *
 * The pass is R, then groups of REF_INTERVAL test scans each followed by
 * another R, so every group has a reference on both sides:
 *
 *   interval 1    R T R T R ... T R        2N+1 scans
 *   interval 2    R T T R T T R ...        every test still touches a reference
 *   interval N    R T T ... T R            N+2 scans
 *
 * A strobe goes up for its test and comes back down, so the line is at rest
 * again afterwards and a reference serves the tests on either side of it.
 * Each test takes its reference from the two around its group: a straight
 * line between them (REF_INTERPOLATE) or the nearer one.
 *
 * Idle noise alone favours few references and interpolation. A pressed key
 * does not: sampling the line while it is high takes charge out of it, so
 * after the strobe drops the line sits about 10 % of the press low and
 * recovers over the next few scans. A reference far from the test does not
 * see that step, and the following strobes on the line read low.
 * Interpolating smears the step across the group instead of cancelling it.
 * Interval 2 with the nearest reference keeps every test beside one that has
 * seen the same step, which is why it is the default.
 */
#define REF_INTERVAL CONFIG_ZMK_KSCAN_EC_DIRECT_REF_INTERVAL
#define REF_INTERP   IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_REF_INTERPOLATE)

/* All NAVG samples of a pass land back to back in this buffer under one arm.
 * Static, never the stack: an EasyDMA buffer on a thread stack keeps the
 * converter's static current up (nRF52840 anomaly 246). */
#define EC_PASS_LEN (EC_MAX_COLS * (2 * EC_MAX_ROWS + 1))
static int16_t ec_pass_buf[EC_PASS_LEN * NAVG];
static uint8_t pass_n; /* strobes per group */
static uint8_t pass_k; /* sample index within the pass */

static inline uint16_t pass_total(const struct ec_direct_config *cfg)
{
	return cfg->cols * (cfg->rows + DIV_ROUND_UP(cfg->rows, pass_n) + 1);
}

/*
 * Waiting for one scan (every sense line converted once).
 *
 * DONE fires per channel and there is no per-scan event. Counting DONEs loses
 * one whenever an interrupt keeps the thread away for longer than a
 * conversion: two events merge, the count comes up short, and the wait spins
 * out its whole bound.
 *
 * So the pass waits for one latched event instead: the last channel's LIMITH,
 * with its high limit at the bottom of the range so every result trips it.
 * The SAADC interrupt stays disabled in the NVIC. Nothing is connected to it.
 */
static uint8_t scan_last_ch;

static void scan_wait_setup(const struct ec_direct_config *cfg)
{
	scan_last_ch = cfg->cols - 1;
	nrf_saadc_channel_limits_set(NRF_SAADC, scan_last_ch, INT16_MIN, INT16_MIN);
	nrf_saadc_event_clear(NRF_SAADC,
			      nrf_saadc_limit_event_get(scan_last_ch, NRF_SAADC_LIMIT_HIGH));
	nrf_saadc_int_enable(NRF_SAADC,
			     nrf_saadc_limit_int_get(scan_last_ch, NRF_SAADC_LIMIT_HIGH));
	NVIC_ClearPendingIRQ(SAADC_IRQn);
}

static void scan_wait_teardown(void)
{
	nrf_saadc_int_disable(NRF_SAADC,
			      nrf_saadc_limit_int_get(scan_last_ch, NRF_SAADC_LIMIT_HIGH));
	NVIC_ClearPendingIRQ(SAADC_IRQn);
}

static bool scan_wait(void)
{
	const nrf_saadc_event_t ev = nrf_saadc_limit_event_get(scan_last_ch, NRF_SAADC_LIMIT_HIGH);
	uint32_t g = 0;

	while (!nrf_saadc_event_check(NRF_SAADC, ev)) {
		if (++g > 100000u) {
			return false;
		}
	}
	nrf_saadc_event_clear(NRF_SAADC, ev);
	nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_DONE);
	NVIC_ClearPendingIRQ(SAADC_IRQn);
	return true;
}

/* The scan that fills the buffer: END follows its last conversion. */
static void scan_wait_end(void)
{
	uint32_t g = 0;

	scan_wait();
	while (!nrf_saadc_event_check(NRF_SAADC, NRF_SAADC_EVENT_END)) {
		if (++g > 100000u) {
			break;
		}
	}
	nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_END);
	nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_DONE);
	nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_RESULTDONE);
}

static void nrf_pass_begin(const struct ec_direct_config *cfg)
{
	pass_n = MIN(REF_INTERVAL, cfg->rows);
	pass_k = 0;
	nrfx_saadc_simple_mode_set(col_chan_mask, NRF_SAADC_RESOLUTION_12BIT,
				   NRF_SAADC_OVERSAMPLE_DISABLED, NULL);
	scan_wait_setup(cfg);
	ec_saadc_arm(ec_pass_buf, pass_total(cfg) * NAVG);
	nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_DONE);
	for (uint8_t i = 0; i < cfg->rows; i++) {
		gpio_pin_set_dt(&cfg->strobes[i], 0);
	}
}

static void nrf_pass_end(const struct ec_direct_config *cfg)
{
	ARG_UNUSED(cfg);
	ec_saadc_disarm();
	scan_wait_teardown();
}

static void nrf_sample_pass(const struct ec_direct_config *cfg, int16_t *diff)
{
	const uint8_t n = pass_n;
	const uint8_t groups = DIV_ROUND_UP(cfg->rows, n);
	const int16_t *p = ec_pass_buf + pass_k * pass_total(cfg);
	/* only the last sample's closing reference fills the buffer */
	const bool fills = ++pass_k == NAVG;
	uint8_t r = 0;

	nrfy_saadc_sample_start(NRF_SAADC, NULL);
	scan_wait();

	for (uint8_t g = 0; g < groups; g++) {
		for (uint8_t j = 0; j < n && r < cfg->rows; j++, r++) {
			/* Sampled while the strobe is still high. Dropping it
			 * first would cancel the step on a floating line. */
			gpio_pin_set_dt(&cfg->strobes[r], 1);
			nrfy_saadc_sample_start(NRF_SAADC, NULL);
			scan_wait();
			gpio_pin_set_dt(&cfg->strobes[r], 0);
		}
		nrfy_saadc_sample_start(NRF_SAADC, NULL);
		if (fills && g + 1 == groups) {
			scan_wait_end();
		} else {
			scan_wait();
		}
	}

	r = 0;
	for (uint8_t g = 0; g < groups; g++) {
		const uint8_t cnt = MIN(n, cfg->rows - r);
		const int16_t *ra = p, *rb = p + cfg->cols * (cnt + 1);

		for (uint8_t j = 0; j < cnt; j++, r++) {
			const int16_t *t = p + cfg->cols * (j + 1);

			for (uint8_t c = 0; c < cfg->cols; c++) {
				int32_t ref;

				if (REF_INTERP) {
					ref = ra[c] + ((int32_t)(rb[c] - ra[c]) * (j + 1)) / (cnt + 1);
				} else {
					ref = (2 * (j + 1) <= cnt + 1) ? ra[c] : rb[c];
				}
				diff[r * cfg->cols + c] = t[c] - ref;
			}
		}
		p = rb;
	}
}

/* --- tripwire pass --------------------------------------------------- */

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE)

/*
 * All strobes in one write per port: OUTSET/OUTCLR are port-wide masked
 * registers. Falls back to a per-pin loop if a strobe is active-low (OUTSET
 * is polarity-blind) or the strobes span more ports than the table holds.
 */
struct port_group {
	NRF_GPIO_Type *port;
	uint32_t mask;
};

static struct port_group strobe_ports[2];
static uint8_t n_strobe_ports;
static bool strobe_masked;

static void strobes_init(const struct ec_direct_config *cfg)
{
	n_strobe_ports = 0;
	strobe_masked = false;

	for (uint8_t r = 0; r < cfg->rows; r++) {
		uint32_t pin = cfg->hw->strobe_psel[r];
		NRF_GPIO_Type *port = nrf_gpio_pin_port_decode(&pin);
		uint8_t g;

		if (cfg->strobes[r].dt_flags & GPIO_ACTIVE_LOW) {
			return;
		}
		for (g = 0; g < n_strobe_ports; g++) {
			if (strobe_ports[g].port == port) {
				break;
			}
		}
		if (g == n_strobe_ports) {
			if (n_strobe_ports == ARRAY_SIZE(strobe_ports)) {
				return;
			}
			strobe_ports[n_strobe_ports].port = port;
			strobe_ports[n_strobe_ports].mask = 0;
			n_strobe_ports++;
		}
		strobe_ports[g].mask |= BIT(pin);
	}
	strobe_masked = true;
}

static void all_strobes(const struct ec_direct_config *cfg, int level)
{
	if (!strobe_masked) {
		for (uint8_t r = 0; r < cfg->rows; r++) {
			gpio_pin_set_dt(&cfg->strobes[r], level);
		}
		return;
	}
	for (uint8_t g = 0; g < n_strobe_ports; g++) {
		if (level) {
			strobe_ports[g].port->OUTSET = strobe_ports[g].mask;
		} else {
			strobe_ports[g].port->OUTCLR = strobe_ports[g].mask;
		}
	}
}

/*
 * Each line's reference and test are taken back to back, one line at a time.
 *
 * Converting every line as the reference and then every line as the test
 * would be fewer steps, but it puts conversions on other lines between a
 * line's own pair. Switching the SAADC input next to a floating line throws
 * occasional large glitches, and this pass takes one sample with no median
 * to hide them. With only one channel in the scan there is nothing in the
 * gap.
 *
 * The converter is set up and started once, in single-channel mode on line 0.
 * Between lines only the channels' PSELP registers change: the previous
 * line's input is disconnected and the next one's connected. Every SAMPLE
 * then converts exactly one channel and the buffer fills as
 * ref0 test0 ref1 test1 ...
 *
 * The PSELP values are read back from what nrfx programs rather than
 * computed, so the input encoding stays nrfx's business.
 */
static int16_t tw_buf[2 * EC_MAX_COLS]; /* static, see ec_pass_buf */
static uint32_t tw_psel_on[EC_MAX_COLS], tw_psel_off;
static bool tw_psel_known;

static bool tw_psel_learn(const struct ec_direct_config *cfg)
{
	if (nrfx_saadc_simple_mode_set(col_chan_mask, NRF_SAADC_RESOLUTION_12BIT,
				       NRF_SAADC_OVERSAMPLE_DISABLED, NULL) != NRFX_SUCCESS) {
		return false;
	}
	for (uint8_t c = 0; c < cfg->cols; c++) {
		tw_psel_on[c] = NRF_SAADC->CH[c].PSELP;
	}
	if (nrfx_saadc_simple_mode_set(BIT(0), NRF_SAADC_RESOLUTION_12BIT,
				       NRF_SAADC_OVERSAMPLE_DISABLED, NULL) != NRFX_SUCCESS) {
		return false;
	}
	/* any channel that is not line 0 is now in its disconnected state */
	tw_psel_off = NRF_SAADC->CH[cfg->cols > 1 ? 1 : EC_MAX_COLS - 1].PSELP;
	tw_psel_known = true;
	return true;
}

static void nrf_tripwire_pass(const struct ec_direct_config *cfg, int32_t *out)
{
	const uint16_t total = 2 * cfg->cols;

	/*
	 * The mode set runs every pass and has to. Most of its time is the
	 * SAADC power cycle nrfx does for anomaly 212, and a converter that
	 * skips it after a scan-mode pass reads hundreds of counts off one
	 * that did not, so the two could not share a baseline.
	 */
	if ((!tw_psel_known && !tw_psel_learn(cfg)) ||
	    nrfx_saadc_simple_mode_set(BIT(0), NRF_SAADC_RESOLUTION_12BIT,
				       NRF_SAADC_OVERSAMPLE_DISABLED, NULL) != NRFX_SUCCESS) {
		/* Zero, not stale: a zero reads as a large negative delta and
		 * cannot trip. A failed pass must not wake the per-key scan. */
		for (uint8_t c = 0; c < cfg->cols; c++) {
			out[c] = 0;
		}
		return;
	}

	ec_saadc_arm(tw_buf, total);
	nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_DONE);

	for (uint8_t c = 0; c < cfg->cols; c++) {
		if (c) {
			NRF_SAADC->CH[c - 1].PSELP = tw_psel_off;
			NRF_SAADC->CH[c].PSELP = tw_psel_on[c];
			nrf_barrier_w();
		}

		all_strobes(cfg, 0);
		nrfy_saadc_sample_start(NRF_SAADC, NULL); /* reference */
		ec_saadc_wait_done(1);

		all_strobes(cfg, 1);
		if (c + 1 == cfg->cols) {
			/* the last test fills the buffer: waits END */
			ec_saadc_convert_wait(tw_buf, total);
		} else {
			nrfy_saadc_sample_start(NRF_SAADC, NULL);
			ec_saadc_wait_done(1);
		}
		all_strobes(cfg, 0);
	}
	ec_saadc_disarm();

	for (uint8_t c = 0; c < cfg->cols; c++) {
		out[c] = tw_buf[2 * c + 1] - tw_buf[2 * c];
	}
}

#endif /* CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE */

/* --- init ------------------------------------------------------------ */

static void nrf_thread_start(const struct ec_direct_config *cfg)
{
	col_chan_mask = 0;
	for (uint8_t c = 0; c < cfg->cols; c++) {
		col_chan_mask |= BIT(c);
		/* Input buffer disconnected, no pull: the pad is the SAADC's. */
		nrf_gpio_cfg(cfg->hw->col_psel[c], NRF_GPIO_PIN_DIR_INPUT,
			     NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1,
			     NRF_GPIO_PIN_NOSENSE);
	}
	/* nrfx_saadc_init enabled the SAADC line in the NVIC with nothing
	 * connected to it. The scan wait wants it pending, never taken. */
	NVIC_DisableIRQ(SAADC_IRQn);
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE)
	strobes_init(cfg);
	if (!strobe_masked) {
		LOG_WRN("tripwire strobes fall back to per-pin writes");
	}
#endif
}

static int nrf_init(const struct ec_direct_config *cfg, uint8_t *stage)
{
	/* Fixed AIN map: AINn <-> P0.<02,03,04,05,28,29,30,31>[n] */
	static const uint8_t ain_pin[8] = {2, 3, 4, 5, 28, 29, 30, 31};
	int rc;

	*stage = ZMK_EC_DIRECT_INIT_ADC;
	rc = nrfx_saadc_init(6);
	if (rc != NRFX_SUCCESS) {
		LOG_ERR("nrfx_saadc_init failed (%d)", rc);
		return -ENODEV;
	}

	*stage = ZMK_EC_DIRECT_INIT_PIN_MAP;
	for (uint8_t c = 0; c < cfg->cols; c++) {
		if (cfg->col_ain[c] >= ARRAY_SIZE(ain_pin)) {
			LOG_ERR("col %u: invalid AIN%u", c, cfg->col_ain[c]);
			return -EINVAL;
		}
		uint32_t want = ain_pin[cfg->col_ain[c]];

		if (cfg->hw->col_psel[c] != want) {
			LOG_ERR("col %u: ain-channels AIN%u is P0.%02u but col-gpios is P%u.%02u", c,
				cfg->col_ain[c], want, cfg->hw->col_psel[c] / 32,
				cfg->hw->col_psel[c] % 32);
			return -ENODEV;
		}
	}

	*stage = ZMK_EC_DIRECT_INIT_CHANNELS;
	for (uint8_t c = 0; c < cfg->cols; c++) {
		nrfx_saadc_channel_t ch = NRFX_SAADC_DEFAULT_CHANNEL_SE(EC_AIN(cfg->col_ain[c]), c);

		ch.channel_config.gain = EC_GAIN;
		ch.channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
		ch.channel_config.acq_time = EC_ACQ_TIME;
		rc = nrfx_saadc_channel_config(&ch);
		if (rc != NRFX_SUCCESS) {
			LOG_ERR("col %u channel config failed (%d)", c, rc);
			return -ENODEV;
		}
	}
	return 0;
}

const struct ec_backend ec_backend = {
	.init = nrf_init,
	.thread_start = nrf_thread_start,
	.pass_begin = nrf_pass_begin,
	.pass_end = nrf_pass_end,
	.sample_pass = nrf_sample_pass,
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_DIRECT_TRIPWIRE)
	.tripwire_pass = nrf_tripwire_pass,
#endif
};
