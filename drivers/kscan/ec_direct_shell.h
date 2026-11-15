/*
 * Bring-up commands, ec_direct_bringup.c. The `ecd` command table is in
 * ec_direct_shell.c.
 *
 * Copyright (c) 2026 carrefinho
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <zephyr/device.h>
#include <zephyr/shell/shell.h>

int ecd_cmd_check(const struct shell *sh, const struct device *dev);
int ecd_cmd_survey(const struct shell *sh, const struct device *dev);
int ecd_cmd_comp_fit(const struct shell *sh, const struct device *dev);
int ecd_cmd_report(const struct shell *sh, const struct device *dev);
