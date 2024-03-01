# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_MSPM0L2228)
    zephyr_compile_definitions(-D__MSPM0L2228__)
endif()
zephyr_sources(soc.c)
