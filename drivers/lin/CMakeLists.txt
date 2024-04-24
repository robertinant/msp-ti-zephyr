# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/lin.h)

zephyr_library_sources_ifdef(CONFIG_LIN_MSPM0 lin_mspm0.c)

zephyr_library_sources_ifdef(CONFIG_LIN_MSPM0 lin_uart.c)

