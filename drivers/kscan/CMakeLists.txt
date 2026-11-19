# Copyright (c) 2026 carrefinho
# SPDX-License-Identifier: MIT

if(CONFIG_ZMK_KSCAN_DIRECT_EC)
  zephyr_library_named(zmk_direct_ec)
  zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)
  zephyr_library_sources(kscan_direct_ec.c)
  zephyr_library_sources_ifdef(CONFIG_SOC_SERIES_NRF52X backend_nrf.c)
  zephyr_library_sources_ifdef(CONFIG_SOC_SERIES_SAMD21 backend_pair.c backend_samd21.c)
  zephyr_library_sources_ifdef(CONFIG_SOC_SERIES_RP2040 backend_pair.c backend_rp2040.c)
  zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_DIRECT_EC_SHELL direct_ec_shell.c)
  zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_DIRECT_EC_BRINGUP direct_ec_bringup.c)
  # The bring-up commands draw the board from ZMK's physical layout.
  if(CONFIG_ZMK_KSCAN_DIRECT_EC_BRINGUP)
    zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)
  endif()
  zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_DIRECT_EC_BATTERY battery_nrf.c)
endif()
