/*
 * Bring-up commands, direct_ec_bringup.c. The `ec` command table is in
 * direct_ec_shell.c.
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <zephyr/device.h>
#include <zephyr/shell/shell.h>

int ec_cmd_rest(const struct shell *sh, const struct device *dev);
int ec_cmd_fit(const struct shell *sh, const struct device *dev);
int ec_cmd_calibrate(const struct shell *sh, const struct device *dev);
int ec_cmd_export(const struct shell *sh, const struct device *dev);
