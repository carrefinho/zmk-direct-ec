# Copyright (c) 2026 carrefinho
# SPDX-License-Identifier: MIT

if(CONFIG_ZMK_KSCAN_EC_DIRECT)
  zephyr_library_named(zmk_ec_direct)
  zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)
  zephyr_library_sources(kscan_ec_direct.c)
  zephyr_library_sources_ifdef(CONFIG_SOC_SERIES_NRF52X backend_nrf.c)
  zephyr_library_sources_ifdef(CONFIG_SOC_SERIES_SAMD21 backend_pair.c backend_samd21.c)
  zephyr_library_sources_ifdef(CONFIG_SOC_SERIES_RP2040 backend_pair.c backend_rp2040.c)
  zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_EC_DIRECT_SHELL ec_direct_shell.c)
  zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_EC_DIRECT_BRINGUP ec_direct_bringup.c)
  # The bring-up commands draw the board from ZMK's physical layout.
  if(CONFIG_ZMK_KSCAN_EC_DIRECT_BRINGUP)
    zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)
  endif()
  zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_EC_DIRECT_BATTERY battery_nrf.c)
endif()
