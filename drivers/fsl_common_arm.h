/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_COMMON_ARM_H_
#define _FSL_COMMON_ARM_H_

/*
 * For CMSIS pack RTE.
 * CMSIS pack RTE generates "RTC_Components.h" which contains the statements
 * of the related <RTE_Components_h> element for all selected software components.
 */

/*!
 * @addtogroup ksdk_common
 * @{
 */

/*! @name Atomic modification
 *
 * These macros are used for atomic access, such as read-modify-write
 * to the peripheral registers.
 *
 * - SDK_ATOMIC_LOCAL_ADD
 * - SDK_ATOMIC_LOCAL_SET
 * - SDK_ATOMIC_LOCAL_CLEAR
 * - SDK_ATOMIC_LOCAL_TOGGLE
 * - SDK_ATOMIC_LOCAL_CLEAR_AND_SET
 *
 * Take SDK_ATOMIC_LOCAL_CLEAR_AND_SET as an example: the parameter @c addr
 * means the address of the peripheral register or variable you want to modify
 * atomically, the parameter @c clearBits is the bits to clear, the parameter
 * @c setBits it the bits to set.
 * For example, to set a 32-bit register bit1:bit0 to 0b10, use like this:
 *
 * @code
   volatile uint32_t * reg = (volatile uint32_t *)REG_ADDR;

   SDK_ATOMIC_LOCAL_CLEAR_AND_SET(reg, 0x03, 0x02);
   @endcode
 *
 * In this example, the register bit1:bit0 are cleared and bit1 is set, as a result,
 * register bit1:bit0 = 0b10.
 *
 * @note For the platforms don't support exclusive load and store, these macros
 * disable the global interrupt to pretect the modification.
 *
 * @note These macros only guarantee the local processor atomic operations. For
 * the multi-processor devices, use hardware semaphore such as SEMA42 to
 * guarantee exclusive access if necessary.
 *
 * @{
 */

/* If the LDREX and STREX are supported, use them. */
#define _SDK_ATOMIC_LOCAL_OPS_1BYTE(addr, val, ops) \
    do                                              \
    {                                               \
        (val) = __LDREXB(addr);                     \
        (ops);                                      \
    } while (0UL != __STREXB((val), (addr)))

#define _SDK_ATOMIC_LOCAL_OPS_2BYTE(addr, val, ops) \
    do                                              \
    {                                               \
        (val) = __LDREXH(addr);                     \
        (ops);                                      \
    } while (0UL != __STREXH((val), (addr)))

#define _SDK_ATOMIC_LOCAL_OPS_4BYTE(addr, val, ops) \
    do                                              \
    {                                               \
        (val) = __LDREXW(addr);                     \
        (ops);                                      \
    } while (0UL != __STREXW((val), (addr)))

static inline void _SDK_AtomicLocalAdd1Byte(volatile uint8_t *addr, uint8_t val)
{
    uint8_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_1BYTE(addr, s_val, s_val += val);
}

static inline void _SDK_AtomicLocalAdd2Byte(volatile uint16_t *addr, uint16_t val)
{
    uint16_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_2BYTE(addr, s_val, s_val += val);
}

static inline void _SDK_AtomicLocalAdd4Byte(volatile uint32_t *addr, uint32_t val)
{
    uint32_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_4BYTE(addr, s_val, s_val += val);
}

static inline void _SDK_AtomicLocalSub1Byte(volatile uint8_t *addr, uint8_t val)
{
    uint8_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_1BYTE(addr, s_val, s_val -= val);
}

static inline void _SDK_AtomicLocalSub2Byte(volatile uint16_t *addr, uint16_t val)
{
    uint16_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_2BYTE(addr, s_val, s_val -= val);
}

static inline void _SDK_AtomicLocalSub4Byte(volatile uint32_t *addr, uint32_t val)
{
    uint32_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_4BYTE(addr, s_val, s_val -= val);
}

static inline void _SDK_AtomicLocalSet1Byte(volatile uint8_t *addr, uint8_t bits)
{
    uint8_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_1BYTE(addr, s_val, s_val |= bits);
}

static inline void _SDK_AtomicLocalSet2Byte(volatile uint16_t *addr, uint16_t bits)
{
    uint16_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_2BYTE(addr, s_val, s_val |= bits);
}

static inline void _SDK_AtomicLocalSet4Byte(volatile uint32_t *addr, uint32_t bits)
{
    uint32_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_4BYTE(addr, s_val, s_val |= bits);
}

static inline void _SDK_AtomicLocalClear1Byte(volatile uint8_t *addr, uint8_t bits)
{
    uint8_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_1BYTE(addr, s_val, s_val &= ~bits);
}

static inline void _SDK_AtomicLocalClear2Byte(volatile uint16_t *addr, uint16_t bits)
{
    uint16_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_2BYTE(addr, s_val, s_val &= ~bits);
}

static inline void _SDK_AtomicLocalClear4Byte(volatile uint32_t *addr, uint32_t bits)
{
    uint32_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_4BYTE(addr, s_val, s_val &= ~bits);
}

static inline void _SDK_AtomicLocalToggle1Byte(volatile uint8_t *addr, uint8_t bits)
{
    uint8_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_1BYTE(addr, s_val, s_val ^= bits);
}

static inline void _SDK_AtomicLocalToggle2Byte(volatile uint16_t *addr, uint16_t bits)
{
    uint16_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_2BYTE(addr, s_val, s_val ^= bits);
}

static inline void _SDK_AtomicLocalToggle4Byte(volatile uint32_t *addr, uint32_t bits)
{
    uint32_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_4BYTE(addr, s_val, s_val ^= bits);
}

static inline void _SDK_AtomicLocalClearAndSet1Byte(volatile uint8_t *addr, uint8_t clearBits, uint8_t setBits)
{
    uint8_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_1BYTE(addr, s_val, s_val = (s_val & ~clearBits) | setBits);
}

static inline void _SDK_AtomicLocalClearAndSet2Byte(volatile uint16_t *addr, uint16_t clearBits, uint16_t setBits)
{
    uint16_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_2BYTE(addr, s_val, s_val = (s_val & ~clearBits) | setBits);
}

static inline void _SDK_AtomicLocalClearAndSet4Byte(volatile uint32_t *addr, uint32_t clearBits, uint32_t setBits)
{
    uint32_t s_val;

    _SDK_ATOMIC_LOCAL_OPS_4BYTE(addr, s_val, s_val = (s_val & ~clearBits) | setBits);
}

#define SDK_ATOMIC_LOCAL_ADD(addr, val)                                                                                        \
    ((1UL == sizeof(*(addr))) ?                                                                                                \
         _SDK_AtomicLocalAdd1Byte((volatile uint8_t *)(volatile void *)(addr), (uint8_t)(val)) :                               \
         ((2UL == sizeof(*(addr))) ? _SDK_AtomicLocalAdd2Byte((volatile uint16_t *)(volatile void *)(addr), (uint16_t)(val)) : \
                                     _SDK_AtomicLocalAdd4Byte((volatile uint32_t *)(volatile void *)(addr), (uint32_t)(val))))

#define SDK_ATOMIC_LOCAL_SET(addr, bits)                                                                                        \
    ((1UL == sizeof(*(addr))) ?                                                                                                 \
         _SDK_AtomicLocalSet1Byte((volatile uint8_t *)(volatile void *)(addr), (uint8_t)(bits)) :                               \
         ((2UL == sizeof(*(addr))) ? _SDK_AtomicLocalSet2Byte((volatile uint16_t *)(volatile void *)(addr), (uint16_t)(bits)) : \
                                     _SDK_AtomicLocalSet4Byte((volatile uint32_t *)(volatile void *)(addr), (uint32_t)(bits))))

#define SDK_ATOMIC_LOCAL_CLEAR(addr, bits)                                                                 \
    ((1UL == sizeof(*(addr))) ?                                                                            \
         _SDK_AtomicLocalClear1Byte((volatile uint8_t *)(volatile void *)(addr), (uint8_t)(bits)) :        \
         ((2UL == sizeof(*(addr))) ?                                                                       \
              _SDK_AtomicLocalClear2Byte((volatile uint16_t *)(volatile void *)(addr), (uint16_t)(bits)) : \
              _SDK_AtomicLocalClear4Byte((volatile uint32_t *)(volatile void *)(addr), (uint32_t)(bits))))

#define SDK_ATOMIC_LOCAL_TOGGLE(addr, bits)                                                                 \
    ((1UL == sizeof(*(addr))) ?                                                                             \
         _SDK_AtomicLocalToggle1Byte((volatile uint8_t *)(volatile void *)(addr), (uint8_t)(bits)) :        \
         ((2UL == sizeof(*(addr))) ?                                                                        \
              _SDK_AtomicLocalToggle2Byte((volatile uint16_t *)(volatile void *)(addr), (uint16_t)(bits)) : \
              _SDK_AtomicLocalToggle4Byte((volatile uint32_t *)(volatile void *)(addr), (uint32_t)(bits))))

#define SDK_ATOMIC_LOCAL_CLEAR_AND_SET(addr, clearBits, setBits)                                                                           \
    ((1UL == sizeof(*(addr))) ?                                                                                                            \
         _SDK_AtomicLocalClearAndSet1Byte((volatile uint8_t *)(volatile void *)(addr), (uint8_t)(clearBits), (uint8_t)(setBits)) :         \
         ((2UL == sizeof(*(addr))) ?                                                                                                       \
              _SDK_AtomicLocalClearAndSet2Byte((volatile uint16_t *)(volatile void *)(addr), (uint16_t)(clearBits), (uint16_t)(setBits)) : \
              _SDK_AtomicLocalClearAndSet4Byte((volatile uint32_t *)(volatile void *)(addr), (uint32_t)(clearBits), (uint32_t)(setBits))))
/* @} */

/*! @name Timer utilities */
/* @{ */
/*! Macro to convert a microsecond period to raw count value */
#define USEC_TO_COUNT(us, clockFreqInHz) (uint64_t)(((uint64_t)(us) * (clockFreqInHz)) / 1000000U)
/*! Macro to convert a raw count value to microsecond */
#define COUNT_TO_USEC(count, clockFreqInHz) (uint64_t)((uint64_t)(count)*1000000U / (clockFreqInHz))

/*! Macro to convert a millisecond period to raw count value */
#define MSEC_TO_COUNT(ms, clockFreqInHz) (uint64_t)((uint64_t)(ms) * (clockFreqInHz) / 1000U)
/*! Macro to convert a raw count value to millisecond */
#define COUNT_TO_MSEC(count, clockFreqInHz) (uint64_t)((uint64_t)(count)*1000U / (clockFreqInHz))
/* @} */

/*! @name ISR exit barrier
 * @{
 *
 * ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
 * exception return operation might vector to incorrect interrupt.
 * For Cortex-M7, if core speed much faster than peripheral register write speed,
 * the peripheral interrupt flags may be still set after exiting ISR, this results to
 * the same error similar with errata 83869.
 */
#define SDK_ISR_EXIT_BARRIER __DSB()

/* @} */

/*! @name Alignment variable definition macros */
/* @{ */
/*! Macro to define a variable with alignbytes alignment */
#define SDK_ALIGN(var, alignbytes) var __attribute__((aligned(alignbytes)))
/*! Macro to define a variable with L1 d-cache line size alignment */
#if defined(FSL_FEATURE_L1DCACHE_LINESIZE_BYTE)
#define SDK_L1DCACHE_ALIGN(var) SDK_ALIGN(var, FSL_FEATURE_L1DCACHE_LINESIZE_BYTE)
#endif
/*! Macro to define a variable with L2 cache line size alignment */

/*! Macro to change a value to a given size aligned value */
#define SDK_SIZEALIGN(var, alignbytes) \
    ((unsigned int)((var) + ((alignbytes)-1U)) & (unsigned int)(~(unsigned int)((alignbytes)-1U)))
/* @} */

/*! @name Non-cacheable region definition macros */
/* For initialized non-zero non-cacheable variables, please using "AT_NONCACHEABLE_SECTION_INIT(var) ={xx};" or
 * "AT_NONCACHEABLE_SECTION_ALIGN_INIT(var) ={xx};" in your projects to define them, for zero-inited non-cacheable
 * variables, please using "AT_NONCACHEABLE_SECTION(var);" or "AT_NONCACHEABLE_SECTION_ALIGN(var);" to define them,
 * these zero-inited variables will be initialized to zero in system startup.
 */
/* @{ */

#if ((!(defined(FSL_FEATURE_HAS_NO_NONCACHEABLE_SECTION) && FSL_FEATURE_HAS_NO_NONCACHEABLE_SECTION)) && \
     defined(FSL_FEATURE_L1ICACHE_LINESIZE_BYTE))
/* For GCC, when the non-cacheable section is required, please define "__STARTUP_INITIALIZE_NONCACHEDATA"
 * in your projects to make sure the non-cacheable section variables will be initialized in system startup.
 */
#define AT_NONCACHEABLE_SECTION_INIT(var) __attribute__((section("NonCacheable.init"))) var
#define AT_NONCACHEABLE_SECTION_ALIGN_INIT(var, alignbytes) \
    __attribute__((section("NonCacheable.init"))) var __attribute__((aligned(alignbytes)))
#define AT_NONCACHEABLE_SECTION(var) __attribute__((section("NonCacheable,\"aw\",%nobits @"))) var
#define AT_NONCACHEABLE_SECTION_ALIGN(var, alignbytes) \
    __attribute__((section("NonCacheable,\"aw\",%nobits @"))) var __attribute__((aligned(alignbytes)))

#endif

/* @} */

/*!
 * @name Time sensitive region
 * @{
 */
#define AT_QUICKACCESS_SECTION_CODE(func) __attribute__((section("CodeQuickAccess"), __noinline__)) func
#define AT_QUICKACCESS_SECTION_DATA(var)  __attribute__((section("DataQuickAccess"))) var
#define AT_QUICKACCESS_SECTION_DATA_ALIGN(var, alignbytes) \
    __attribute__((section("DataQuickAccess"))) var __attribute__((aligned(alignbytes)))
/*! @name Ram Function */
#define RAMFUNCTION_SECTION_CODE(func) __attribute__((section("RamFunction"))) func

/*
 * The fsl_clock.h is included here because it needs MAKE_VERSION/MAKE_STATUS/status_t
 * defined in previous of this file.
 */
#include "fsl_clock.h"

/*
 * Chip level peripheral reset API, for MCUs that implement peripheral reset control external to a peripheral
 */
/*!
 * @brief Enable specific interrupt.
 *
 * Enable LEVEL1 interrupt. For some devices, there might be multiple interrupt
 * levels. For example, there are NVIC and intmux. Here the interrupts connected
 * to NVIC are the LEVEL1 interrupts, because they are routed to the core directly.
 * The interrupts connected to intmux are the LEVEL2 interrupts, they are routed
 * to NVIC first then routed to core.
 *
 * This function only enables the LEVEL1 interrupts. The number of LEVEL1 interrupts
 * is indicated by the feature macro FSL_FEATURE_NUMBER_OF_LEVEL1_INT_VECTORS.
 *
 * @param interrupt The IRQ number.
 * @retval kStatus_Success Interrupt enabled successfully
 * @retval kStatus_Fail Failed to enable the interrupt
 */
static inline status_t EnableIRQ(IRQn_Type interrupt)
{
    status_t status = kStatus_Success;

    if (NotAvail_IRQn == interrupt)
    {
        status = kStatus_Fail;
    }
    else
    {
        NVIC_EnableIRQ(interrupt);
    }

    return status;
}

/*!
 * @brief Disable specific interrupt.
 *
 * Disable LEVEL1 interrupt. For some devices, there might be multiple interrupt
 * levels. For example, there are NVIC and intmux. Here the interrupts connected
 * to NVIC are the LEVEL1 interrupts, because they are routed to the core directly.
 * The interrupts connected to intmux are the LEVEL2 interrupts, they are routed
 * to NVIC first then routed to core.
 *
 * This function only disables the LEVEL1 interrupts. The number of LEVEL1 interrupts
 * is indicated by the feature macro FSL_FEATURE_NUMBER_OF_LEVEL1_INT_VECTORS.
 *
 * @param interrupt The IRQ number.
 * @retval kStatus_Success Interrupt disabled successfully
 * @retval kStatus_Fail Failed to disable the interrupt
 */
static inline status_t DisableIRQ(IRQn_Type interrupt)
{
    status_t status = kStatus_Success;

    if (NotAvail_IRQn == interrupt)
    {
        status = kStatus_Fail;
    }
    else
    {
        NVIC_DisableIRQ(interrupt);
    }

    return status;
}

/*!
 * @brief Disable the global IRQ
 *
 * Disable the global interrupt and return the current primask register. User is required to provided the primask
 * register for the EnableGlobalIRQ().
 *
 * @return Current primask value.
 */
static inline uint32_t DisableGlobalIRQ(void)
{
    uint32_t regPrimask = __get_PRIMASK();

    __disable_irq();

    return regPrimask;
}

/*!
 * @brief Enable the global IRQ
 *
 * Set the primask register with the provided primask value but not just enable the primask. The idea is for the
 * convenience of integration of RTOS. some RTOS get its own management mechanism of primask. User is required to
 * use the EnableGlobalIRQ() and DisableGlobalIRQ() in pair.
 *
 * @param primask value of primask register to be restored. The primask value is supposed to be provided by the
 * DisableGlobalIRQ().
 */
static inline void EnableGlobalIRQ(uint32_t primask)
{
    __set_PRIMASK(primask);
}
#endif /* _FSL_COMMON_ARM_H_ */
