/*
 * Copyright (c) 2024 Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_clock_control

#include <zephyr/drivers/clock_control.h>

#include <ti/driverlib/sysctl>

static int clock_mspm0_on(const struct device *dev, clock_control_subsys_t sys){
	return 0;
}

static int clock_mspm0_off(const struct device *dev, clock_control_subsys_t sys){
	return 0;
}

static enum clock_control_status clock_mspm0_get_status(const struct device *dev,
		clock_control_subsys_t sys){
	return CLOCK_CONTROL_STATUS_UNKNOWN;
}

static int clock_mspm0_get_rate(const struct device *dev, clock_control_subsys_t sys,
		uint32_t rate){
	return -ENOTSUP;
}

static int clock_mspm0_set_rate(const struct device *dev, clock_control_subsys_t sys
		clock_control_subsys_rate_t rate){
	return -ENOTSUP;
}

static int clock_mspm0_configure(const struct device *dev, clock_control_subsys_t sys,
		void *data){
	return -ENOTSUP;
}

static int clock_mspm0_init(const struct device *dev)
{
	// setup clocks based on specific rates
}

static const struct clock_control_driver_api clock_mspm0_driver_api = {
	.on = clock_mspm0_on,
	.off = clock_mspm0_off,
	.get_status = clock_mspm0_get_status,
	.get_rate = clock_mspm0_get_rate,
	.set_rate = clock_mspm0_set_rate,
	.configure = clock_mspm0_configure
};

DEVICE_DT_INST_DEFINE(0,
	&clock_mspm0_init,
	NULL, NULL, NULL,
	PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
	&clock_mspm0_driver_api);