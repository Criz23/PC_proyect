/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.common_arm"
#endif

#ifndef __GIC_PRIO_BITS

#endif /* __GIC_PRIO_BITS. */

                 /* defined(SDK_DELAY_USE_DWT) && defined(DWT) */
/* Use software loop. */

/* Cortex-M0 has a smaller instruction set, SUBS isn't supported in thumb-16 mode reported from __GNUC__ compiler,
 * use SUB and CMP here for compatibility */
static void DelayLoop(uint32_t count)
{
    __ASM volatile("    MOV    R0, %0" : : "r"(count));
    __ASM volatile(
        "loop:                          \n"

        "    SUB    R0, R0, #1          \n"

        "    CMP    R0, #0              \n"

        "    BNE    loop                \n"
        :
        :
        : "r0");
}


/*!
 * @brief Delay at least for some time.
 *  Please note that, if not uses DWT, this API will use while loop for delay, different run-time environments have
 *  effect on the delay time. If precise delay is needed, please enable DWT delay. The two parmeters delayTime_us and
 *  coreClock_Hz have limitation. For example, in the platform with 1GHz coreClock_Hz, the delayTime_us only supports
 *  up to 4294967 in current code. If long time delay is needed, please implement a new delay function.
 *
 * @param delayTime_us  Delay time in unit of microsecond.
 * @param coreClock_Hz  Core clock frequency with Hz.
 */
void SDK_DelayAtLeastUs(uint32_t delayTime_us, uint32_t coreClock_Hz)
{
    uint64_t count;

    if (delayTime_us > 0U)
    {
        count = USEC_TO_COUNT(delayTime_us, coreClock_Hz);

        assert(count <= UINT32_MAX);


        /* Divide value may be different in various environment to ensure delay is precise.
         * Every loop count includes three instructions, due to Cortex-M7 sometimes executes
         * two instructions in one period, through test here set divide 1.5. Other M cores use
         * divide 4. By the way, divide 1.5 or 4 could let the count lose precision, but it does
         * not matter because other instructions outside while loop is enough to fill the time.
         */

        count = count / 4U;
        DelayLoop((uint32_t)count);
	 /* defined(SDK_DELAY_USE_DWT) && defined(DWT) */
    }
}
