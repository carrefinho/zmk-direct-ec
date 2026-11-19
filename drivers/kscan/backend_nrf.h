/*
 * nRF52840 backend internals for zmk,kscan-direct-ec.
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <hal/nrf_gpio.h>
#include <soc_nrf_common.h>
#include <nrfx_saadc.h>
#if defined(__has_include) && __has_include(<helpers/nrfx_analog_common.h>)
#define EC_NRFX_ANALOG_INPUT_API 1
#include <helpers/nrfx_analog_common.h>
#else
#define EC_NRFX_ANALOG_INPUT_API 0
#endif
#include <haly/nrfy_saadc.h>

#include "direct_ec_priv.h"

/*
 * Newer nrfx routes .pin_p through the generic analog layer
 * (nrfx_analog_input_t, a 0-based AIN index). Older nrfx takes the
 * PSELP-encoded nrf_saadc_input_t directly, where AINn is n+1. The devicetree
 * property always carries the plain AIN index.
 */
#if EC_NRFX_ANALOG_INPUT_API
#define EC_AIN(i) ((nrfx_analog_input_t)(i))
#else
#define EC_AIN(i) ((nrf_saadc_input_t)((i) + 1))
#endif

/* 0.6 V internal reference at gain 1/6: 3.6 V full scale. */
#define EC_GAIN     NRF_SAADC_GAIN1_6
/* The shortest there is. The line floats, so the sample is a charge share
 * that settles in nanoseconds and a longer acquisition buys nothing. */
#define EC_ACQ_TIME NRF_SAADC_ACQTIME_3US

struct ec_backend_cfg {
	const uint32_t *col_psel;    /* absolute pin number per sense line */
	const uint32_t *strobe_psel; /* same per strobe, for the tripwire's masked write */
};

/* ain-channels gives the AIN index for the SAADC. col-gpios gives the same
 * pin as a GPIO, which is what lets init check the two agree. */
#define EC_COL_PSEL(idx, n)    NRF_DT_GPIOS_TO_PSEL_BY_IDX(DT_DRV_INST(n), col_gpios, idx),
#define EC_STROBE_PSEL(idx, n) NRF_DT_GPIOS_TO_PSEL_BY_IDX(DT_DRV_INST(n), strobe_gpios, idx),

#define EC_BACKEND_INST(n)                                                                         \
	static const uint32_t ec_psel_##n[] = {                                                    \
		LISTIFY(DT_INST_PROP_LEN(n, col_gpios), EC_COL_PSEL, (), n)};                       \
	static const uint32_t ec_spsel_##n[] = {                                                   \
		LISTIFY(DT_INST_PROP_LEN(n, strobe_gpios), EC_STROBE_PSEL, (), n)};                 \
	static const struct ec_backend_cfg ec_hw_##n = {                                           \
		.col_psel = ec_psel_##n,                                                           \
		.strobe_psel = ec_spsel_##n,                                                       \
	};

/* Shared with battery_nrf.c, which borrows the converter between passes. */
void ec_saadc_arm(int16_t *buf, uint16_t cnt);
void ec_saadc_convert_wait(int16_t *buf, uint16_t cnt);
bool ec_saadc_wait_done(uint8_t n);
void ec_saadc_disarm(void);
