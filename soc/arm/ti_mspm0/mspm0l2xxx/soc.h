/*
 * Copyright (c) 2024 Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef soc_h
#define soc_h

#define SYSCONFIG_WEAK __attribute__((weak))

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/* clang-format off */

#define POWER_STARTUP_DELAY (16)

/*
 *  Note: Clock Control driver is not available yet for MSPM0.
 *	More advanced clock configuration will be available once the driver is
 *   implemented.
 */

/*!
 * @brief CPU frequency in Hz
 */
#define SOC_MSPM0_CPUCLK_FREQ_HZ    MHZ(32)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_SYSCTL_init(void);

#ifdef __cplusplus
}
#endif

#endif /* soc_h */
