/*
 * Battery level for boards where zmk,kscan-ec-direct owns the nRF52 SAADC.
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 *
 * The scan thread takes the reading itself (ec_battery_tick, right after a
 * tripwire pass) and this file's sensor device only hands out the cached
 * value. Why not a second driver sharing the converter behind a lock: the
 * scan drives nrfx directly with the SAADC interrupt off in the NVIC, a lock
 * gives no say over when a conversion lands next to a floating sense line,
 * and a stock adc_read holds the converter across a context switch.
 *
 * The divider is switched on and read in the same tick, so it must have no
 * capacitor on its midpoint (the XIAO nRF52840's does not). 338 k of source impedance wants the 40 us
 * acquisition, as in the stock driver.
 */

#define DT_DRV_COMPAT zmk_ec_direct_battery

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "ec_direct_priv.h"

LOG_MODULE_DECLARE(kscan_ec_direct, CONFIG_INPUT_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "one zmk,ec-direct-battery node");

/* The last SAADC channel, clear of the sense lines on channels 0..cols-1. */
#define BAT_CH      (SAADC_CH_NUM - 1)
#define BAT_SAMPLES 4

BUILD_ASSERT(DT_PROP_LEN(DT_INST(0, zmk_kscan_ec_direct), ain_channels) <= BAT_CH,
	     "the battery channel collides with a sense line");

struct ec_battery_config {
	uint8_t ain;
	struct gpio_dt_spec power;
	uint32_t output_ohms;
	uint32_t full_ohms;
};

static const struct ec_battery_config bat_cfg = {
	.ain = DT_INST_PROP(0, ain_channel),
	.power = GPIO_DT_SPEC_INST_GET_OR(0, power_gpios, {0}),
	.output_ohms = DT_INST_PROP(0, output_ohms),
	.full_ohms = DT_INST_PROP(0, full_ohms),
};

static int16_t bat_buf[BAT_SAMPLES];    /* static, never the stack (anomaly 246) */
static bool bat_ch_ready;
static bool bat_dev_ready;
static int64_t bat_next_ms;
static atomic_t bat_mv;                 /* 0 until the first reading */
static K_SEM_DEFINE(bat_first, 0, 1);

/* Scan thread only, with the converter disarmed. */
static int bat_read_mv(void)
{
	int32_t sum = 0;

	if (!bat_ch_ready) {
		nrfx_saadc_channel_t ch = NRFX_SAADC_DEFAULT_CHANNEL_SE(EC_AIN(bat_cfg.ain), BAT_CH);

		ch.channel_config.gain = NRF_SAADC_GAIN1_6;
		ch.channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
		ch.channel_config.acq_time = NRF_SAADC_ACQTIME_40US;
		if (nrfx_saadc_channel_config(&ch) != NRFX_SUCCESS) {
			return -EIO;
		}
		bat_ch_ready = true;
	}

	if (bat_cfg.power.port) {
		gpio_pin_set_dt(&bat_cfg.power, 1);
	}

	/* The scan passes run their own mode set, which disconnects this
	 * channel again; this one disconnects the sense lines. */
	nrfx_saadc_simple_mode_set(BIT(BAT_CH), NRF_SAADC_RESOLUTION_12BIT,
				   NRF_SAADC_OVERSAMPLE_DISABLED, NULL);
	ec_saadc_arm(bat_buf, BAT_SAMPLES);
	nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_DONE);
	for (int i = 0; i < BAT_SAMPLES - 1; i++) {
		nrfy_saadc_sample_start(NRF_SAADC, NULL);
		ec_saadc_wait_done(1);
	}
	ec_saadc_convert_wait(bat_buf, BAT_SAMPLES);
	ec_saadc_disarm();

	if (bat_cfg.power.port) {
		gpio_pin_set_dt(&bat_cfg.power, 0);
	}

	for (int i = 0; i < BAT_SAMPLES; i++) {
		sum += MAX(bat_buf[i], 0);
	}
	/* gain 1/6 on the 0.6 V reference: 3.6 V over 4096 codes */
	int64_t node_mv = (int64_t)sum * 3600 / (4096 * BAT_SAMPLES);

	return (int)(node_mv * bat_cfg.full_ohms / bat_cfg.output_ohms);
}

void ec_battery_tick(void)
{
	int64_t now = k_uptime_get();

	if (!bat_dev_ready || now < bat_next_ms) {
		return;
	}
	bat_next_ms = now + CONFIG_ZMK_KSCAN_EC_DIRECT_BATTERY_INTERVAL_S * MSEC_PER_SEC;

	int mv = bat_read_mv();

	if (mv <= 0) {
		LOG_WRN("battery read failed (%d)", mv);
		return;
	}
	if (atomic_set(&bat_mv, mv) == 0) {
		k_sem_give(&bat_first);
	}
	LOG_DBG("battery %d mV", mv);
}

/* ZMK's linear Li-ion curve (battery_common.c), so the percentage matches
 * what the stock divider would report. */
static uint8_t mv_to_pct(int mv)
{
	if (mv >= 4200) {
		return 100;
	}
	if (mv <= 3450) {
		return 0;
	}
	return mv * 2 / 15 - 459;
}

static int bat_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	ARG_UNUSED(dev);

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_GAUGE_VOLTAGE &&
	    chan != SENSOR_CHAN_VOLTAGE && chan != SENSOR_CHAN_GAUGE_STATE_OF_CHARGE) {
		return -ENOTSUP;
	}
	/* ZMK asks once at boot, before the scan thread has calibrated and
	 * dropped to the tripwire. Waiting here (on ZMK's low-priority work
	 * queue) beats reporting 0 % until the next interval. */
	if (atomic_get(&bat_mv) == 0 && k_sem_take(&bat_first, K_SECONDS(5)) != 0) {
		return -EAGAIN;
	}
	return 0;
}

static int bat_channel_get(const struct device *dev, enum sensor_channel chan,
			   struct sensor_value *val)
{
	const int mv = atomic_get(&bat_mv);

	ARG_UNUSED(dev);

	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
	case SENSOR_CHAN_VOLTAGE:
		val->val1 = mv / 1000;
		val->val2 = (mv % 1000) * 1000;
		return 0;
	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		val->val1 = mv_to_pct(mv);
		val->val2 = 0;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static const struct sensor_driver_api bat_api = {
	.sample_fetch = bat_sample_fetch,
	.channel_get = bat_channel_get,
};

static int bat_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	if (bat_cfg.power.port) {
		if (!gpio_is_ready_dt(&bat_cfg.power)) {
			return -ENODEV;
		}
		int rc = gpio_pin_configure_dt(&bat_cfg.power, GPIO_OUTPUT_INACTIVE);

		if (rc) {
			return rc;
		}
	}
	bat_dev_ready = true;
	return 0;
}

DEVICE_DT_INST_DEFINE(0, bat_init, NULL, NULL, NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		      &bat_api);
