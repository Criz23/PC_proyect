/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Can.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.flexcan"
#endif
/* According to CiA doc 1301 v1.0.0, specified data/nominal phase sample point postion for CAN FD at 80 MHz. */
#define IDEAL_DATA_SP_1  (800U)
#define IDEAL_DATA_SP_2  (750U)
#define IDEAL_DATA_SP_3  (700U)
#define IDEAL_DATA_SP_4  (625U)
#define IDEAL_NOMINAL_SP (800U)

/* According to CiA doc 301 v4.2.0 and previous version. */
#define IDEAL_SP_LOW  (750U)
#define IDEAL_SP_MID  (800U)
#define IDEAL_SP_HIGH (875U)

#define IDEAL_SP_FACTOR (1000U)

/* Define the max value of bit timing segments when use different timing register. */
#define MAX_PROPSEG           (CAN_CTRL1_PROPSEG_MASK >> CAN_CTRL1_PROPSEG_SHIFT)
#define MAX_PSEG1             (CAN_CTRL1_PSEG1_MASK >> CAN_CTRL1_PSEG1_SHIFT)
#define MAX_PSEG2             (CAN_CTRL1_PSEG2_MASK >> CAN_CTRL1_PSEG2_SHIFT)
#define MAX_RJW               (CAN_CTRL1_RJW_MASK >> CAN_CTRL1_RJW_SHIFT)
#define MAX_PRESDIV           (CAN_CTRL1_PRESDIV_MASK >> CAN_CTRL1_PRESDIV_SHIFT)
#define CTRL1_MAX_TIME_QUANTA (1U + MAX_PROPSEG + 1U + MAX_PSEG1 + 1U + MAX_PSEG2 + 1U)
#define CTRL1_MIN_TIME_QUANTA (8U)

#define MAX_EPROPSEG        (CAN_CBT_EPROPSEG_MASK >> CAN_CBT_EPROPSEG_SHIFT)
#define MAX_EPSEG1          (CAN_CBT_EPSEG1_MASK >> CAN_CBT_EPSEG1_SHIFT)
#define MAX_EPSEG2          (CAN_CBT_EPSEG2_MASK >> CAN_CBT_EPSEG2_SHIFT)
#define MAX_ERJW            (CAN_CBT_ERJW_MASK >> CAN_CBT_ERJW_SHIFT)
#define MAX_EPRESDIV        (CAN_CBT_EPRESDIV_MASK >> CAN_CBT_EPRESDIV_SHIFT)
#define CBT_MAX_TIME_QUANTA (1U + MAX_EPROPSEG + 1U + MAX_EPSEG1 + 1U + MAX_EPSEG2 + 1U)
#define CBT_MIN_TIME_QUANTA (8U)

#define MAX_FPROPSEG          (CAN_FDCBT_FPROPSEG_MASK >> CAN_FDCBT_FPROPSEG_SHIFT)
#define MAX_FPSEG1            (CAN_FDCBT_FPSEG1_MASK >> CAN_FDCBT_FPSEG1_SHIFT)
#define MAX_FPSEG2            (CAN_FDCBT_FPSEG2_MASK >> CAN_FDCBT_FPSEG2_SHIFT)
#define MAX_FRJW              (CAN_FDCBT_FRJW_MASK >> CAN_FDCBT_FRJW_SHIFT)
#define MAX_FPRESDIV          (CAN_FDCBT_FPRESDIV_MASK >> CAN_FDCBT_FPRESDIV_SHIFT)
#define FDCBT_MAX_TIME_QUANTA (1U + MAX_FPROPSEG + 0U + MAX_FPSEG1 + 1U + MAX_FPSEG2 + 1U)
#define FDCBT_MIN_TIME_QUANTA (5U)

#define MAX_TDCOFF ((uint32_t)CAN_FDCTRL_TDCOFF_MASK >> CAN_FDCTRL_TDCOFF_SHIFT)

#define MAX_NTSEG1            (CAN_ENCBT_NTSEG1_MASK >> CAN_ENCBT_NTSEG1_SHIFT)
#define MAX_NTSEG2            (CAN_ENCBT_NTSEG2_MASK >> CAN_ENCBT_NTSEG2_SHIFT)
#define MAX_NRJW              (CAN_ENCBT_NRJW_MASK >> CAN_ENCBT_NRJW_SHIFT)
#define MAX_ENPRESDIV         (CAN_EPRS_ENPRESDIV_MASK >> CAN_EPRS_ENPRESDIV_SHIFT)
#define ENCBT_MAX_TIME_QUANTA (1U + MAX_NTSEG1 + 1U + MAX_NTSEG2 + 1U)
#define ENCBT_MIN_TIME_QUANTA (8U)

#define MAX_DTSEG1            (CAN_EDCBT_DTSEG1_MASK >> CAN_EDCBT_DTSEG1_SHIFT)
#define MAX_DTSEG2            (CAN_EDCBT_DTSEG2_MASK >> CAN_EDCBT_DTSEG2_SHIFT)
#define MAX_DRJW              (CAN_EDCBT_DRJW_MASK >> CAN_EDCBT_DRJW_SHIFT)
#define MAX_EDPRESDIV         (CAN_EPRS_EDPRESDIV_MASK >> CAN_EPRS_EDPRESDIV_SHIFT)
#define EDCBT_MAX_TIME_QUANTA (1U + MAX_DTSEG1 + 1U + MAX_DTSEG2 + 1U)
#define EDCBT_MIN_TIME_QUANTA (5U)

#define MAX_ETDCOFF ((uint32_t)CAN_ETDC_ETDCOFF_MASK >> CAN_ETDC_ETDCOFF_SHIFT)

/* TSEG1 corresponds to the sum of xPROPSEG and xPSEG1, TSEG2 corresponds to the xPSEG2 value. */
#define MIN_TIME_SEGMENT1 (2U)
#define MIN_TIME_SEGMENT2 (2U)

/* Define maximum CAN and CAN FD bit rate supported by FLEXCAN. */
#define MAX_CANFD_BITRATE (8000000U)
#define MAX_CAN_BITRATE   (1000000U)
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
#ifndef CAN_CLOCK_CHECK_NO_AFFECTS
/* If no define such MACRO, it mean that the CAN in current device have no clock affect issue. */
#define CAN_CLOCK_CHECK_NO_AFFECTS (true)
#endif /* CAN_CLOCK_CHECK_NO_AFFECTS */
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*! @brief FlexCAN Internal State. */
enum _flexcan_state
{
    kFLEXCAN_StateIdle     = 0x0, /*!< MB/RxFIFO idle.*/
    kFLEXCAN_StateRxData   = 0x1, /*!< MB receiving.*/
    kFLEXCAN_StateRxRemote = 0x2, /*!< MB receiving remote reply.*/
    kFLEXCAN_StateTxData   = 0x3, /*!< MB transmitting.*/
    kFLEXCAN_StateTxRemote = 0x4, /*!< MB transmitting remote request.*/
    kFLEXCAN_StateRxFifo   = 0x5, /*!< RxFIFO receiving.*/
};

/*! @brief FlexCAN message buffer CODE for Rx buffers. */
enum _flexcan_mb_code_rx
{
    kFLEXCAN_RxMbInactive = 0x0, /*!< MB is not active.*/
    kFLEXCAN_RxMbFull     = 0x2, /*!< MB is full.*/
    kFLEXCAN_RxMbEmpty    = 0x4, /*!< MB is active and empty.*/
    kFLEXCAN_RxMbOverrun  = 0x6, /*!< MB is overwritten into a full buffer.*/
    kFLEXCAN_RxMbBusy     = 0x8, /*!< FlexCAN is updating the contents of the MB, The CPU must not access the MB.*/
    kFLEXCAN_RxMbRanswer  = 0xA, /*!< A frame was configured to recognize a Remote Request Frame and transmit a
                                      Response Frame in return.*/
    kFLEXCAN_RxMbNotUsed = 0xF,  /*!< Not used.*/
};

/*! @brief FlexCAN message buffer CODE FOR Tx buffers. */
enum _flexcan_mb_code_tx
{
    kFLEXCAN_TxMbInactive     = 0x8, /*!< MB is not active.*/
    kFLEXCAN_TxMbAbort        = 0x9, /*!< MB is aborted.*/
    kFLEXCAN_TxMbDataOrRemote = 0xC, /*!< MB is a TX Data Frame(when MB RTR = 0) or MB is a TX Remote Request
                                          Frame (when MB RTR = 1).*/
    kFLEXCAN_TxMbTanswer = 0xE,      /*!< MB is a TX Response Request Frame from an incoming Remote Request Frame.*/
    kFLEXCAN_TxMbNotUsed = 0xF,      /*!< Not used.*/
};

/* Typedef for interrupt handler. */
typedef void (*flexcan_isr_t)(CAN_Type *base, flexcan_handle_t *handle);

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#if !defined(NDEBUG)
/*!
 * @brief Check if Message Buffer is occupied by Rx FIFO.
 *
 * This function check if Message Buffer is occupied by Rx FIFO.
 *
 * @param base FlexCAN peripheral base address.
 * @param mbIdx The FlexCAN Message Buffer index.
 * @return TRUE if the index MB is occupied by Rx FIFO, FALSE if the index MB not occupied by Rx FIFO.
 */
static bool FLEXCAN_IsMbOccupied(CAN_Type *base, uint8_t mbIdx);
#endif
/*!
 * @brief Check if Message Buffer interrupt is enabled.
 *
 * This function check if Message Buffer interrupt is enabled.
 *
 * @param base FlexCAN peripheral base address.
 * @param mbIdx The FlexCAN Message Buffer index.
 *
 * @return TRUE if the index MB interrupt mask enabled, FALSE if the index MB interrupt mask disabled.
 */
static bool FLEXCAN_IsMbIntEnabled(CAN_Type *base, uint8_t mbIdx);

/*!
 * @brief Reset the FlexCAN Instance.
 *
 * Restores the FlexCAN module to reset state, notice that this function
 * will set all the registers to reset state so the FlexCAN module can not work
 * after calling this API.
 *
 * @param base FlexCAN peripheral base address.
 */
static void FLEXCAN_Reset(CAN_Type *base);

/*!
 * @brief Set bit rate of FlexCAN classical CAN frame or CAN FD frame nominal phase.
 *
 * This function set the bit rate of classical CAN frame or CAN FD frame nominal phase base on the value of the
 * parameter passed in. Users need to ensure that the timing segment values (phaseSeg1, phaseSeg2 and propSeg) match the
 * clock and bit rate, if not match, the final output bit rate may not equal the bitRate_Bps value. Suggest use
 * FLEXCAN_CalculateImprovedTimingValues() to get timing configuration.
 *
 * @param base FlexCAN peripheral base address.
 * @param sourceClock_Hz Source Clock in Hz.
 * @param bitRate_Bps Bit rate in Bps.
 * @param timingConfig FlexCAN timingConfig.
 */
static void FLEXCAN_SetBitRate(CAN_Type *base,
                               uint32_t sourceClock_Hz,
                               uint32_t bitRate_Bps,
                               flexcan_timing_config_t timingConfig);

/*!
 * @brief Calculates the segment values for a single bit time for classical CAN.
 *
 * This function use to calculates the Classical CAN segment values which will be set in CTRL1/CBT/ENCBT register.
 *
 * @param base FlexCAN peripheral base address.
 * @param tqNum Number of time quantas per bit, range in 8 ~ 25 when use CTRL1, range in 8 ~ 129 when use CBT, range in
 *             8 ~ 385 when use ENCBT. param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 */
static void FLEXCAN_GetSegments(CAN_Type *base,
                                uint32_t bitRate,
                                uint32_t tqNum,
                                flexcan_timing_config_t *pTimingConfig);
/*!
 * @brief Check unhandle interrupt events
 *
 * @param base FlexCAN peripheral base address.
 * @return TRUE if unhandled interrupt action exist, FALSE if no unhandlered interrupt action exist.
 */
static bool FLEXCAN_CheckUnhandleInterruptEvents(CAN_Type *base);

/*!
 * @brief Sub Handler Data Trasfered Events
 *
 * @param base FlexCAN peripheral base address.
 * @param handle FlexCAN handle pointer.
 * @param pResult Pointer to the Handle result.
 *
 * @return the status after handle each data transfered event.
 */
static status_t FLEXCAN_SubHandlerForDataTransfered(CAN_Type *base, flexcan_handle_t *handle, uint32_t *pResult);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Array of FlexCAN peripheral base address. */
static CAN_Type *const s_flexcanBases[] = CAN_BASE_PTRS;

/* Array of FlexCAN IRQ number. */
static const IRQn_Type s_flexcanRxWarningIRQ[] = CAN_Rx_Warning_IRQS;
static const IRQn_Type s_flexcanTxWarningIRQ[] = CAN_Tx_Warning_IRQS;
static const IRQn_Type s_flexcanWakeUpIRQ[]    = CAN_Wake_Up_IRQS;
static const IRQn_Type s_flexcanErrorIRQ[]     = CAN_Error_IRQS;
static const IRQn_Type s_flexcanBusOffIRQ[]    = CAN_Bus_Off_IRQS;
static const IRQn_Type s_flexcanMbIRQ[]        = CAN_ORed_Message_buffer_IRQS;

/* Array of FlexCAN handle. */
static flexcan_handle_t *s_flexcanHandle[ARRAY_SIZE(s_flexcanBases)];

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/* Array of FlexCAN clock name. */
static const clock_ip_name_t s_flexcanClock[] = FLEXCAN_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/* FlexCAN ISR for transactional APIs. */
static flexcan_isr_t s_flexcanIsr;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * brief Get the FlexCAN instance from peripheral base address.
 *
 * param base FlexCAN peripheral base address.
 * return FlexCAN instance.
 */
uint32_t FLEXCAN_GetInstance(CAN_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_flexcanBases); instance++)
    {
        if (s_flexcanBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_flexcanBases));

    return instance;
}

/*!
 * brief Enter FlexCAN Freeze Mode.
 *
 * This function makes the FlexCAN work under Freeze Mode.
 *
 * param base FlexCAN peripheral base address.
 */
void FLEXCAN_EnterFreezeMode(CAN_Type *base)
{
    /* Set Freeze, Halt bits. */
    base->MCR |= CAN_MCR_FRZ_MASK;
    base->MCR |= CAN_MCR_HALT_MASK;
    while (0U == (base->MCR & CAN_MCR_FRZACK_MASK))
    {
    }
}

/*!
 * brief Exit FlexCAN Freeze Mode.
 *
 * This function makes the FlexCAN leave Freeze Mode.
 *
 * param base FlexCAN peripheral base address.
 */
void FLEXCAN_ExitFreezeMode(CAN_Type *base)
{
    /* Clear Freeze, Halt bits. */
    base->MCR &= ~CAN_MCR_HALT_MASK;
    base->MCR &= ~CAN_MCR_FRZ_MASK;

    /* Wait until the FlexCAN Module exit freeze mode. */
    while (0U != (base->MCR & CAN_MCR_FRZACK_MASK))
    {
    }
}

#if !defined(NDEBUG)
/*!
 * brief Check if Message Buffer is occupied by Rx FIFO.
 *
 * This function check if Message Buffer is occupied by Rx FIFO.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN Message Buffer index.
 * return TRUE if the index MB is occupied by Rx FIFO, FALSE if the index MB not occupied by Rx FIFO.
 */
static bool FLEXCAN_IsMbOccupied(CAN_Type *base, uint8_t mbIdx)
{
    uint8_t lastOccupiedMb;
    bool fgRet;

    /* Is Rx FIFO enabled? */
    if (0U != (base->MCR & CAN_MCR_RFEN_MASK))
    {
        /* Get RFFN value. */
        lastOccupiedMb = (uint8_t)((base->CTRL2 & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        /* Calculate the number of last Message Buffer occupied by Rx FIFO. */
        lastOccupiedMb = ((lastOccupiedMb + 1U) * 2U) + 5U;
        fgRet = (mbIdx <= lastOccupiedMb);
    }
    else
    {
        {
            fgRet = false;
        }
    }

    return fgRet;
}
#endif
/*!
 * brief Check if Message Buffer interrupt is enabled.
 *
 * This function check if Message Buffer interrupt is enabled.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN Message Buffer index.
 *
 * return TRUE if the index MB interrupt mask enabled, FALSE if the index MB interrupt mask disabled.
 */
static bool FLEXCAN_IsMbIntEnabled(CAN_Type *base, uint8_t mbIdx)
{
    /* Assertion. */
    assert(mbIdx < (uint8_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base));

    uint32_t flag = 1U;
    bool fgRet    = false;
    {
        fgRet = (0U != (base->IMASK1 & (flag << mbIdx)));
    }

    return fgRet;
}

/*!
 * brief Reset the FlexCAN Instance.
 *
 * Restores the FlexCAN module to reset state, notice that this function
 * will set all the registers to reset state so the FlexCAN module can not work
 * after calling this API.
 *
 * param base FlexCAN peripheral base address.
 */
static void FLEXCAN_Reset(CAN_Type *base)
{
    /* The module must should be first exit from low power
     * mode, and then soft reset can be applied.
     */
    assert(0U == (base->MCR & CAN_MCR_MDIS_MASK));

    uint8_t i;
    /* Wait until FlexCAN exit from any Low Power Mode. */
    while (0U != (base->MCR & CAN_MCR_LPMACK_MASK))
    {
    }

    /* Assert Soft Reset Signal. */
    base->MCR |= CAN_MCR_SOFTRST_MASK;
    /* Wait until FlexCAN reset completes. */
    while (0U != (base->MCR & CAN_MCR_SOFTRST_MASK))
    {
    }

/* Reset MCR register. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_GLITCH_FILTER) && FSL_FEATURE_FLEXCAN_HAS_GLITCH_FILTER)
    base->MCR |= CAN_MCR_WRNEN_MASK | CAN_MCR_WAKSRC_MASK |
                 CAN_MCR_MAXMB((uint32_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base) - 1U);
#endif

    /* Reset CTRL1 and CTRL2 register, default to eanble SMP feature which enable three sample point to determine the
     * received bit's value of the. */
    base->CTRL1 = CAN_CTRL1_SMP_MASK;
    base->CTRL2 = CAN_CTRL2_TASD(0x16) | CAN_CTRL2_RRS_MASK | CAN_CTRL2_EACEN_MASK;
    /* Only need clean all Message Buffer memory. */
    (void)memset((void *)&base->MB[0], 0, sizeof(base->MB));

    /* Clean all individual Rx Mask of Message Buffers. */
    for (i = 0; i < (uint32_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base); i++)
    {
        base->RXIMR[i] = 0x3FFFFFFF;
    }

    /* Clean Global Mask of Message Buffers. */
    base->RXMGMASK = 0x3FFFFFFF;
    /* Clean Global Mask of Message Buffer 14. */
    base->RX14MASK = 0x3FFFFFFF;
    /* Clean Global Mask of Message Buffer 15. */
    base->RX15MASK = 0x3FFFFFFF;
    /* Clean Global Mask of Rx FIFO. */
    base->RXFGMASK = 0x3FFFFFFF;
}

/*!
 * brief Set bit rate of FlexCAN classical CAN frame or CAN FD frame nominal phase.
 *
 * This function set the bit rate of classical CAN frame or CAN FD frame nominal phase base on the value of the
 * parameter passed in. Users need to ensure that the timing segment values (phaseSeg1, phaseSeg2 and propSeg) match the
 * clock and bit rate, if not match, the final output bit rate may not equal the bitRate_Bps value. Suggest use
 * FLEXCAN_CalculateImprovedTimingValues() to get timing configuration.
 *
 * param base FlexCAN peripheral base address.
 * param sourceClock_Hz Source Clock in Hz.
 * param bitRate_Bps Bit rate in Bps.
 * param timingConfig FlexCAN timingConfig.
 */
static void FLEXCAN_SetBitRate(CAN_Type *base,
                               uint32_t sourceClock_Hz,
                               uint32_t bitRate_Bps,
                               flexcan_timing_config_t timingConfig)
{
    /* FlexCAN classical CAN frame or CAN FD frame nominal phase timing setting formula:
     * quantum = 1 + (phaseSeg1 + 1) + (phaseSeg2 + 1) + (propSeg + 1);
     */
    uint32_t quantum = (1U + ((uint32_t)timingConfig.phaseSeg1 + 1U) + ((uint32_t)timingConfig.phaseSeg2 + 1U) +
                        ((uint32_t)timingConfig.propSeg + 1U));

    /* Assertion: Desired bit rate is too high. */
    assert(bitRate_Bps <= 1000000U);
    /* Assertion: Source clock should greater than or equal to bit rate * quantum. */
    assert((bitRate_Bps * quantum) <= sourceClock_Hz);
    /* Assertion: Desired bit rate is too low, the bit rate * quantum * max prescaler divider value should greater than
       or equal to source clock. */
    assert((bitRate_Bps * quantum * MAX_PRESDIV) >= sourceClock_Hz);
    if (quantum < (MIN_TIME_SEGMENT1 + MIN_TIME_SEGMENT2 + 1U))
    {
        /* No valid timing configuration. */
        timingConfig.preDivider = 0U;
    }
    else
    {
        timingConfig.preDivider = (uint16_t)((sourceClock_Hz / (bitRate_Bps * quantum)) - 1U);
    }

    /* Update actual timing characteristic. */
    FLEXCAN_SetTimingConfig(base, (const flexcan_timing_config_t *)(uint32_t)&timingConfig);
}
/*!
 * brief Initializes a FlexCAN instance.
 *
 * This function initializes the FlexCAN module with user-defined settings.
 * This example shows how to set up the Can_ConfigType parameters and how
 * to call the FLEXCAN_Init function by passing in these parameters.
 *  code
 *   Can_ConfigType flexcanConfig;
 *   flexcanConfig.clkSrc               = kFLEXCAN_ClkSrc0;
 *   flexcanConfig.bitRate              = 1000000U;
 *   flexcanConfig.maxMbNum             = 16;
 *   flexcanConfig.enableLoopBack       = false;
 *   flexcanConfig.enableSelfWakeup     = false;
 *   flexcanConfig.enableIndividMask    = false;
 *   flexcanConfig.disableSelfReception = false;
 *   flexcanConfig.enableListenOnlyMode = false;
 *   flexcanConfig.enableDoze           = false;
 *   flexcanConfig.timingConfig         = timingConfig;
 *   FLEXCAN_Init(CAN0, &flexcanConfig, 40000000UL);
 *   endcode
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the user-defined configuration structure.
 * param sourceClock_Hz FlexCAN Protocol Engine clock source frequency in Hz.
 */
void FLEXCAN_Init(CAN_Type *base, const Can_ConfigType *pConfig, uint32_t sourceClock_Hz)
{
    /* Assertion. */
    assert(NULL != pConfig);
    assert((pConfig->maxMbNum > 0U) &&
           (pConfig->maxMbNum <= (uint8_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base)));
    assert(pConfig->bitRate > 0U);

    uint32_t mcrTemp;
    uint32_t ctrl1Temp;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    uint32_t instance;
#endif

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    instance = FLEXCAN_GetInstance(base);
    /* Enable FlexCAN clock. */
    (void)CLOCK_EnableClock(s_flexcanClock[instance]);
    /*
     * Check the CAN clock in this device whether affected by Other clock gate
     * If it affected, we'd better to change other clock source,
     * If user insist on using that clock source, user need open these gate at same time,
     * In this scene, User need to care the power consumption.
     */
    assert(CAN_CLOCK_CHECK_NO_AFFECTS);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if defined(CAN_CTRL1_CLKSRC_MASK)
    {
        /* Disable FlexCAN Module. */
        FLEXCAN_Enable(base, false);

        /* Protocol-Engine clock source selection, This bit must be set
         * when FlexCAN Module in Disable Mode.
         */
        base->CTRL1 = (kFLEXCAN_ClkSrc0 == pConfig->clkSrc) ? (base->CTRL1 & ~CAN_CTRL1_CLKSRC_MASK) :
                                                              (base->CTRL1 | CAN_CTRL1_CLKSRC_MASK);
    }
#endif /* CAN_CTRL1_CLKSRC_MASK */

    /* Enable FlexCAN Module for configuration. */
    FLEXCAN_Enable(base, true);

    /* Reset to known status. */
    FLEXCAN_Reset(base);
    /* Save current CTRL1 value and enable to enter Freeze mode(enabled by default). */
    ctrl1Temp = base->CTRL1;

    /* Save current MCR value and enable to enter Freeze mode(enabled by default). */
    mcrTemp = base->MCR;

    /* Enable Loop Back Mode? */
    ctrl1Temp = (pConfig->enableLoopBack) ? (ctrl1Temp | CAN_CTRL1_LPB_MASK) : (ctrl1Temp & ~CAN_CTRL1_LPB_MASK);

    /* Enable Timer Sync? */
    ctrl1Temp = (pConfig->enableTimerSync) ? (ctrl1Temp | CAN_CTRL1_TSYN_MASK) : (ctrl1Temp & ~CAN_CTRL1_TSYN_MASK);

    /* Enable Listen Only Mode? */
    ctrl1Temp = (pConfig->enableListenOnlyMode) ? ctrl1Temp | CAN_CTRL1_LOM_MASK : ctrl1Temp & ~CAN_CTRL1_LOM_MASK;

#if !(defined(FSL_FEATURE_FLEXCAN_HAS_NO_SUPV_SUPPORT) && FSL_FEATURE_FLEXCAN_HAS_NO_SUPV_SUPPORT)
    /* Enable Supervisor Mode? */
    mcrTemp = (pConfig->enableSupervisorMode) ? mcrTemp | CAN_MCR_SUPV_MASK : mcrTemp & ~CAN_MCR_SUPV_MASK;
#endif

    /* Set the maximum number of Message Buffers */
    mcrTemp = (mcrTemp & ~CAN_MCR_MAXMB_MASK) | CAN_MCR_MAXMB((uint32_t)pConfig->maxMbNum - 1U);

    /* Enable Self Wake Up Mode and configure the wake up source. */
    mcrTemp = (pConfig->enableSelfWakeup) ? (mcrTemp | CAN_MCR_SLFWAK_MASK) : (mcrTemp & ~CAN_MCR_SLFWAK_MASK);
    mcrTemp = (kFLEXCAN_WakeupSrcFiltered == pConfig->wakeupSrc) ? (mcrTemp | CAN_MCR_WAKSRC_MASK) :
                                                                   (mcrTemp & ~CAN_MCR_WAKSRC_MASK);
    /* Enable Individual Rx Masking and Queue feature? */
    mcrTemp = (pConfig->enableIndividMask) ? (mcrTemp | CAN_MCR_IRMQ_MASK) : (mcrTemp & ~CAN_MCR_IRMQ_MASK);

    /* Disable Self Reception? */
    mcrTemp = (pConfig->disableSelfReception) ? mcrTemp | CAN_MCR_SRXDIS_MASK : mcrTemp & ~CAN_MCR_SRXDIS_MASK;
    /* Write back CTRL1 Configuration to register. */
    base->CTRL1 = ctrl1Temp;

    /* Write back MCR Configuration to register. */
    base->MCR = mcrTemp;

    /* Bit Rate Configuration.*/
    FLEXCAN_SetBitRate(base, sourceClock_Hz, pConfig->bitRate, pConfig->timingConfig);
}
/*!
 * brief De-initializes a FlexCAN instance.
 *
 * This function disables the FlexCAN module clock and sets all register values
 * to the reset value.
 *
 * param base FlexCAN peripheral base address.
 */
void FLEXCAN_Deinit(CAN_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    uint32_t instance;
#endif
    /* Reset all Register Contents. */
    FLEXCAN_Reset(base);

    /* Disable FlexCAN module. */
    FLEXCAN_Enable(base, false);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    instance = FLEXCAN_GetInstance(base);
    /* Disable FlexCAN clock. */
    (void)CLOCK_DisableClock(s_flexcanClock[instance]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

/*!
 * brief Gets the default configuration structure.
 *
 * This function initializes the FlexCAN configuration structure to default values. The default
 * values are as follows.
 *   flexcanConfig->clkSrc                               = kFLEXCAN_ClkSrc0;
 *   flexcanConfig->bitRate                              = 1000000U;
 *   flexcanConfig->bitRateFD                            = 2000000U;
 *   flexcanConfig->maxMbNum                             = 16;
 *   flexcanConfig->enableLoopBack                       = false;
 *   flexcanConfig->enableSelfWakeup                     = false;
 *   flexcanConfig->enableIndividMask                    = false;
 *   flexcanConfig->disableSelfReception                 = false;
 *   flexcanConfig->enableListenOnlyMode                 = false;
 *   flexcanConfig->enableDoze                           = false;
 *   flexcanConfig->enablePretendedeNetworking           = false;
 *   flexcanConfig->enableMemoryErrorControl             = true;
 *   flexcanConfig->enableNonCorrectableErrorEnterFreeze = true;
 *   flexcanConfig->enableTransceiverDelayMeasure        = true;
 *   flexcanConfig.timingConfig                          = timingConfig;
 *
 * param pConfig Pointer to the FlexCAN configuration structure.
 */
void FLEXCAN_GetDefaultConfig(Can_ConfigType *pConfig)
{
    /* Assertion. */
    assert(NULL != pConfig);

    /* Initializes the configure structure to zero. */
    (void)memset(pConfig, 0, sizeof(*pConfig));

    /* Initialize FlexCAN Module config struct with default value. */
    pConfig->clkSrc  = kFLEXCAN_ClkSrc0;
    pConfig->bitRate = 1000000U;
    pConfig->maxMbNum             = 16;
    pConfig->enableLoopBack       = false;
    pConfig->enableTimerSync      = true;
    pConfig->enableSelfWakeup     = false;
    pConfig->wakeupSrc            = kFLEXCAN_WakeupSrcUnfiltered;
    pConfig->enableIndividMask    = false;
    pConfig->disableSelfReception = false;
    pConfig->enableListenOnlyMode = false;
#if !(defined(FSL_FEATURE_FLEXCAN_HAS_NO_SUPV_SUPPORT) && FSL_FEATURE_FLEXCAN_HAS_NO_SUPV_SUPPORT)
    pConfig->enableSupervisorMode = true;
#endif
    /* Default protocol timing configuration, nominal bit time quantum is 10 (80% SP), data bit time quantum is 5
     * (60%). Suggest use FLEXCAN_CalculateImprovedTimingValues/FLEXCAN_FDCalculateImprovedTimingValues to get the
     * improved timing configuration.*/
    pConfig->timingConfig.phaseSeg1  = 1;
    pConfig->timingConfig.phaseSeg2  = 1;
    pConfig->timingConfig.propSeg    = 4;
    pConfig->timingConfig.rJumpwidth = 1;
}

/*!
 * brief Sets the FlexCAN classical protocol timing characteristic.
 *
 * This function gives user settings to classical CAN or CAN FD nominal phase timing characteristic.
 * The function is for an experienced user. For less experienced users, call the FLEXCAN_GetDefaultConfig()
 * and get the default timing characteristicsthe, then call FLEXCAN_Init() and fill the
 * bit rate field.
 *
 * note Calling FLEXCAN_SetTimingConfig() overrides the bit rate set
 * in FLEXCAN_Init().
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the timing configuration structure.
 */
void FLEXCAN_SetTimingConfig(CAN_Type *base, const flexcan_timing_config_t *pConfig)
{
    /* Assertion. */
    assert(NULL != pConfig);

    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);
    /* Cleaning previous Timing Setting. */
    base->CTRL1 &= ~(CAN_CTRL1_PRESDIV_MASK | CAN_CTRL1_RJW_MASK | CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PSEG2_MASK |
                     CAN_CTRL1_PROPSEG_MASK);

    /* Updating Timing Setting according to configuration structure. */
    base->CTRL1 |= (CAN_CTRL1_PRESDIV(pConfig->preDivider) | CAN_CTRL1_RJW(pConfig->rJumpwidth) |
                    CAN_CTRL1_PSEG1(pConfig->phaseSeg1) | CAN_CTRL1_PSEG2(pConfig->phaseSeg2) |
                    CAN_CTRL1_PROPSEG(pConfig->propSeg));

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}
/*!
 * brief Sets the FlexCAN receive message buffer global mask.
 *
 * This function sets the global mask for the FlexCAN message buffer in a matching process.
 * The configuration is only effective when the Rx individual mask is disabled in the FLEXCAN_Init().
 *
 * param base FlexCAN peripheral base address.
 * param mask Rx Message Buffer Global Mask value.
 */
void FLEXCAN_SetRxMbGlobalMask(CAN_Type *base, uint32_t mask)
{
    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);

    /* Setting Rx Message Buffer Global Mask value. */
    base->RXMGMASK = mask;
    base->RX14MASK = mask;
    base->RX15MASK = mask;

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}

/*!
 * brief Sets the FlexCAN receive FIFO global mask.
 *
 * This function sets the global mask for FlexCAN FIFO in a matching process.
 *
 * param base FlexCAN peripheral base address.
 * param mask Rx Fifo Global Mask value.
 */
void FLEXCAN_SetRxFifoGlobalMask(CAN_Type *base, uint32_t mask)
{
    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);

    /* Setting Rx FIFO Global Mask value. */
    base->RXFGMASK = mask;

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}

/*!
 * brief Sets the FlexCAN receive individual mask.
 *
 * This function sets the individual mask for the FlexCAN matching process.
 * The configuration is only effective when the Rx individual mask is enabled in the FLEXCAN_Init().
 * If the Rx FIFO is disabled, the individual mask is applied to the corresponding Message Buffer.
 * If the Rx FIFO is enabled, the individual mask for Rx FIFO occupied Message Buffer is applied to
 * the Rx Filter with the same index. Note that only the first 32
 * individual masks can be used as the Rx FIFO filter mask.
 *
 * param base FlexCAN peripheral base address.
 * param maskIdx The Index of individual Mask.
 * param mask Rx Individual Mask value.
 */
void FLEXCAN_SetRxIndividualMask(CAN_Type *base, uint8_t maskIdx, uint32_t mask)
{
    assert(maskIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));

    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);

    /* Setting Rx Individual Mask value. */
    base->RXIMR[maskIdx] = mask;

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}

/*!
 * brief Configures a FlexCAN transmit message buffer.
 *
 * This function aborts the previous transmission, cleans the Message Buffer, and
 * configures it as a Transmit Message Buffer.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The Message Buffer index.
 * param enable Enable/disable Tx Message Buffer.
 *               - true: Enable Tx Message Buffer.
 *               - false: Disable Tx Message Buffer.
 */
void FLEXCAN_SetTxMbConfig(CAN_Type *base, uint8_t mbIdx, bool enable)
{
    /* Assertion. */
    assert(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    assert(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    /* Inactivate Message Buffer. */
    if (enable)
    {
        base->MB[mbIdx].CS = CAN_CS_CODE(kFLEXCAN_TxMbInactive);
    }
    else
    {
        base->MB[mbIdx].CS = 0;
    }

    /* Clean Message Buffer content. */
    base->MB[mbIdx].ID    = 0x0;
    base->MB[mbIdx].WORD0 = 0x0;
    base->MB[mbIdx].WORD1 = 0x0;
}

/*!
 * brief Calculates the segment values for a single bit time for classical CAN.
 *
 * This function use to calculates the Classical CAN segment values which will be set in CTRL1/CBT/ENCBT register.
 *
 * param bitRate The classical CAN bit rate in bps.
 * param base FlexCAN peripheral base address.
 * param tqNum Number of time quantas per bit, range in 8 ~ 25 when use CTRL1, range in 8 ~ 129 when use CBT, range in
 *             8 ~ 385 when use ENCBT. param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 */
static void FLEXCAN_GetSegments(CAN_Type *base,
                                uint32_t bitRate,
                                uint32_t tqNum,
                                flexcan_timing_config_t *pTimingConfig)
{
    uint32_t ideal_sp;
    uint32_t seg1Max, seg2Max, proSegMax, sjwMAX;
    uint32_t seg1Temp;
    /* Maximum value allowed in CTRL1 register. */
    seg1Max   = MAX_PSEG1 + 1U;
    proSegMax = MAX_PROPSEG + 1U;
    seg2Max   = MAX_PSEG2 + 1U;
    sjwMAX    = MAX_RJW + 1U;

    /* Try to find the ideal sample point, according to CiA 301 doc.*/
    if (bitRate == 1000000U)
    {
        ideal_sp = IDEAL_SP_LOW;
    }
    else if (bitRate >= 800000U)
    {
        ideal_sp = IDEAL_SP_MID;
    }
    else
    {
        ideal_sp = IDEAL_SP_HIGH;
    }
    /* Calculates phaseSeg2. */
    pTimingConfig->phaseSeg2 = (uint8_t)(tqNum - (tqNum * ideal_sp) / (uint32_t)IDEAL_SP_FACTOR);
    if (pTimingConfig->phaseSeg2 < MIN_TIME_SEGMENT2)
    {
        pTimingConfig->phaseSeg2 = MIN_TIME_SEGMENT2;
    }
    else if (pTimingConfig->phaseSeg2 > seg2Max)
    {
        pTimingConfig->phaseSeg2 = (uint8_t)seg2Max;
    }
    else
    {
        ; /* Intentional empty */
    }

    /* Calculates phaseSeg1 and propSeg and try to make phaseSeg1 equal to phaseSeg2. */
    if ((tqNum - pTimingConfig->phaseSeg2 - 1U) > (seg1Max + proSegMax))
    {
        seg1Temp                 = seg1Max + proSegMax;
        pTimingConfig->phaseSeg2 = (uint8_t)(tqNum - 1U - seg1Temp);
    }
    else
    {
        seg1Temp = tqNum - pTimingConfig->phaseSeg2 - 1U;
    }
    if (seg1Temp > (pTimingConfig->phaseSeg2 + proSegMax))
    {
        pTimingConfig->propSeg   = (uint8_t)proSegMax;
        pTimingConfig->phaseSeg1 = (uint8_t)(seg1Temp - proSegMax);
    }
    else if (seg1Temp > pTimingConfig->phaseSeg2)
    {
        pTimingConfig->propSeg   = (uint8_t)(seg1Temp - pTimingConfig->phaseSeg2);
        pTimingConfig->phaseSeg1 = pTimingConfig->phaseSeg2;
    }
    else
    {
        pTimingConfig->propSeg   = 1U;
        pTimingConfig->phaseSeg1 = pTimingConfig->phaseSeg2 - 1U;
    }

    /* rJumpwidth (sjw) is the minimum value of phaseSeg1 and phaseSeg2. */
    pTimingConfig->rJumpwidth =
        (pTimingConfig->phaseSeg1 > pTimingConfig->phaseSeg2) ? pTimingConfig->phaseSeg2 : pTimingConfig->phaseSeg1;
    if (pTimingConfig->rJumpwidth > sjwMAX)
    {
        pTimingConfig->rJumpwidth = (uint8_t)sjwMAX;
    }

    pTimingConfig->phaseSeg1 -= 1U;
    pTimingConfig->phaseSeg2 -= 1U;
    pTimingConfig->propSeg -= 1U;
    pTimingConfig->rJumpwidth -= 1U;
}

/*!
 * brief Calculates the improved timing values by specific bit Rates for classical CAN.
 *
 * This function use to calculates the Classical CAN timing values according to the given bit rate. The Calculated
 * timing values will be set in CTRL1/CBT/ENCBT register. The calculation is based on the recommendation of the CiA 301
 * v4.2.0 and previous version document.
 *
 * param base FlexCAN peripheral base address.
 * param bitRate  The classical CAN speed in bps defined by user, should be less than or equal to 1Mbps.
 * param sourceClock_Hz The Source clock frequency in Hz.
 * param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 *
 * return TRUE if timing configuration found, FALSE if failed to find configuration.
 */
bool FLEXCAN_CalculateImprovedTimingValues(CAN_Type *base,
                                           uint32_t bitRate,
                                           uint32_t sourceClock_Hz,
                                           flexcan_timing_config_t *pTimingConfig)
{
    /* Observe bit rate maximums. */
    assert(bitRate <= MAX_CAN_BITRATE);

    uint32_t clk;
    uint32_t tqNum, tqMin, pdivMAX;
    uint32_t spTemp                    = 1000U;
    flexcan_timing_config_t configTemp = {0};
    bool fgRet                         = false;
    /*  Auto Improved Protocal timing for CTRL1. */
    tqNum   = CTRL1_MAX_TIME_QUANTA;
    tqMin   = CTRL1_MIN_TIME_QUANTA;
    pdivMAX = MAX_PRESDIV;
    do
    {
        clk = bitRate * tqNum;
        if (clk > sourceClock_Hz)
        {
            continue; /* tqNum too large, clk has been exceed sourceClock_Hz. */
        }

        if ((sourceClock_Hz / clk * clk) != sourceClock_Hz)
        {
            continue; /* Non-supporting: the frequency of clock source is not divisible by target bit rate, the user
                      should change a divisible bit rate. */
        }

        configTemp.preDivider = (uint16_t)(sourceClock_Hz / clk) - 1U;
        if (configTemp.preDivider > pdivMAX)
        {
            break; /* The frequency of source clock is too large or the bit rate is too small, the pre-divider could
                      not handle it. */
        }

        /* Calculates the best timing configuration under current tqNum. */
        FLEXCAN_GetSegments(base, bitRate, tqNum, &configTemp);
        /* Determine whether the calculated timing configuration can get the optimal sampling point. */
        if (((((uint32_t)configTemp.phaseSeg2 + 1U) * 1000U) / tqNum) < spTemp)
        {
            spTemp                    = (((uint32_t)configTemp.phaseSeg2 + 1U) * 1000U) / tqNum;
            pTimingConfig->preDivider = configTemp.preDivider;
            pTimingConfig->rJumpwidth = configTemp.rJumpwidth;
            pTimingConfig->phaseSeg1  = configTemp.phaseSeg1;
            pTimingConfig->phaseSeg2  = configTemp.phaseSeg2;
            pTimingConfig->propSeg    = configTemp.propSeg;
        }
        fgRet = true;
    } while (--tqNum >= tqMin);

    return fgRet;
}
/*!
 * brief Configures a FlexCAN Receive Message Buffer.
 *
 * This function cleans a FlexCAN build-in Message Buffer and configures it
 * as a Receive Message Buffer.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The Message Buffer index.
 * param pRxMbConfig Pointer to the FlexCAN Message Buffer configuration structure.
 * param enable Enable/disable Rx Message Buffer.
 *               - true: Enable Rx Message Buffer.
 *               - false: Disable Rx Message Buffer.
 */
void FLEXCAN_SetRxMbConfig(CAN_Type *base, uint8_t mbIdx, const flexcan_rx_mb_config_t *pRxMbConfig, bool enable)
{
    /* Assertion. */
    assert(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
    assert(((NULL != pRxMbConfig) || (false == enable)));
#if !defined(NDEBUG)
    assert(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    uint32_t cs_temp = 0;

    /* Inactivate Message Buffer. */
    base->MB[mbIdx].CS = 0;

    /* Clean Message Buffer content. */
    base->MB[mbIdx].ID    = 0x0;
    base->MB[mbIdx].WORD0 = 0x0;
    base->MB[mbIdx].WORD1 = 0x0;

    if (enable)
    {
        /* Setup Message Buffer ID. */
        base->MB[mbIdx].ID = pRxMbConfig->id;

        /* Setup Message Buffer format. */
        if (kFLEXCAN_FrameFormatExtend == pRxMbConfig->format)
        {
            cs_temp |= CAN_CS_IDE_MASK;
        }

        /* Setup Message Buffer type. */
        if (kFLEXCAN_FrameTypeRemote == pRxMbConfig->type)
        {
            cs_temp |= CAN_CS_RTR_MASK;
        }

        /* Activate Rx Message Buffer. */
        cs_temp |= CAN_CS_CODE(kFLEXCAN_RxMbEmpty);
        base->MB[mbIdx].CS = cs_temp;
    }
}
/*!
 * brief Configures the FlexCAN Legacy Rx FIFO.
 *
 * This function configures the FlexCAN Rx FIFO with given configuration.
 * note Legacy Rx FIFO only can receive classic CAN message.
 *
 * param base FlexCAN peripheral base address.
 * param pRxFifoConfig Pointer to the FlexCAN Legacy Rx FIFO configuration structure. Can be NULL when enable parameter
 *                      is false.
 * param enable Enable/disable Legacy Rx FIFO.
 *              - true: Enable Legacy Rx FIFO.
 *              - false: Disable Legacy Rx FIFO.
 */
void FLEXCAN_SetRxFifoConfig(CAN_Type *base, const flexcan_rx_fifo_config_t *pRxFifoConfig, bool enable)
{
    /* Assertion. */
    assert((NULL != pRxFifoConfig) || (false == enable));

    volatile uint32_t *mbAddr;
    uint8_t i, j, k, rffn = 0, numMbOccupy;
    uint32_t setup_mb = 0;

    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);

    if (enable)
    {
        assert(pRxFifoConfig->idFilterNum <= 128U);
        /* Get the setup_mb value. */
        setup_mb = (uint8_t)((base->MCR & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT);
        setup_mb = (setup_mb < (uint32_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base)) ?
                       setup_mb :
                       (uint32_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base);

        /* Determine RFFN value. */
        for (i = 0; i <= 0xFU; i++)
        {
            if ((8U * (i + 1U)) >= pRxFifoConfig->idFilterNum)
            {
                rffn = i;
                assert(((setup_mb - 8U) - (2U * rffn)) > 0U);

                base->CTRL2 = (base->CTRL2 & ~CAN_CTRL2_RFFN_MASK) | CAN_CTRL2_RFFN(rffn);
                break;
            }
        }

        /* caculate the Number of Mailboxes occupied by RX Legacy FIFO and the filter. */
        numMbOccupy = 6U + (rffn + 1U) * 2U;

        /* Copy ID filter table to Message Buffer Region (Fix MISRA_C-2012 Rule 18.1). */
        j = 0U;
        for (i = 6U; i < numMbOccupy; i++)
        {
            /* Get address for current mail box.  */
            mbAddr = &(base->MB[i].CS);

            /* One Mail box contain 4U DWORD registers. */
            for (k = 0; k < 4U; k++)
            {
                /* Fill all valid filter in the mail box occupied by filter.
                 * Disable unused Rx FIFO Filter, the other rest of register in the last Mail box occupied by fiter set
                 * as 0xffffffff.
                 */
                mbAddr[k] = (j < pRxFifoConfig->idFilterNum) ? (pRxFifoConfig->idFilterTable[j]) : 0xFFFFFFFFU;

                /* Try to fill next filter in current Mail Box.  */
                j++;
            }
        }

        /* Setup ID Fitlter Type. */
        switch (pRxFifoConfig->idFilterType)
        {
            case kFLEXCAN_RxFifoFilterTypeA:
                base->MCR = (base->MCR & ~CAN_MCR_IDAM_MASK) | CAN_MCR_IDAM(0x0);
                break;
            case kFLEXCAN_RxFifoFilterTypeB:
                base->MCR = (base->MCR & ~CAN_MCR_IDAM_MASK) | CAN_MCR_IDAM(0x1);
                break;
            case kFLEXCAN_RxFifoFilterTypeC:
                base->MCR = (base->MCR & ~CAN_MCR_IDAM_MASK) | CAN_MCR_IDAM(0x2);
                break;
            case kFLEXCAN_RxFifoFilterTypeD:
                /* All frames rejected. */
                base->MCR = (base->MCR & ~CAN_MCR_IDAM_MASK) | CAN_MCR_IDAM(0x3);
                break;
            default:
                /* All the cases have been listed above, the default clause should not be reached. */
                assert(false);
                break;
        }

        /* Setting Message Reception Priority. */
        base->CTRL2 = (pRxFifoConfig->priority == kFLEXCAN_RxFifoPrioHigh) ? (base->CTRL2 & ~CAN_CTRL2_MRP_MASK) :
                                                                             (base->CTRL2 | CAN_CTRL2_MRP_MASK);

        /* Enable Rx Message FIFO. */
        base->MCR |= CAN_MCR_RFEN_MASK;
    }
    else
    {
        rffn = (uint8_t)((base->CTRL2 & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        /* caculate the Number of Mailboxes occupied by RX Legacy FIFO and the filter. */
        numMbOccupy = 6U + (rffn + 1U) * 2U;

        /* Disable Rx Message FIFO. */
        base->MCR &= ~CAN_MCR_RFEN_MASK;

        /* Clean MB0 ~ MB5 and all MB occupied by ID filters (Fix MISRA_C-2012 Rule 18.1). */

        for (i = 0; i < numMbOccupy; i++)
        {
            FLEXCAN_SetRxMbConfig(base, i, NULL, false);
        }
    }

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}
/*!
 * brief Writes a FlexCAN Message to the Transmit Message Buffer.
 *
 * This function writes a CAN Message to the specified Transmit Message Buffer
 * and changes the Message Buffer state to start CAN Message transmit. After
 * that the function returns immediately.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN Message Buffer index.
 * param pTxFrame Pointer to CAN message frame to be sent.
 * retval kStatus_Success - Write Tx Message Buffer Successfully.
 * retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t FLEXCAN_WriteTxMb(CAN_Type *base, uint8_t mbIdx, const flexcan_frame_t *pTxFrame)
{
    /* Assertion. */
    assert(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
    assert(NULL != pTxFrame);
    assert(pTxFrame->length <= 8U);
#if !defined(NDEBUG)
    assert(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    uint32_t cs_temp = 0;
    status_t status;
    /* Check if Message Buffer is available. */
    if (CAN_CS_CODE(kFLEXCAN_TxMbDataOrRemote) != (base->MB[mbIdx].CS & CAN_CS_CODE_MASK))
    {
        /* Inactive Tx Message Buffer. */
        base->MB[mbIdx].CS = (base->MB[mbIdx].CS & ~CAN_CS_CODE_MASK) | CAN_CS_CODE(kFLEXCAN_TxMbInactive);

        /* Fill Message ID field. */
        base->MB[mbIdx].ID = pTxFrame->id;

        /* Fill Message Format field. */
        if ((uint32_t)kFLEXCAN_FrameFormatExtend == pTxFrame->format)
        {
            cs_temp |= CAN_CS_SRR_MASK | CAN_CS_IDE_MASK;
        }

        /* Fill Message Type field. */
        if ((uint32_t)kFLEXCAN_FrameTypeRemote == pTxFrame->type)
        {
            cs_temp |= CAN_CS_RTR_MASK;
        }

        cs_temp |= CAN_CS_CODE(kFLEXCAN_TxMbDataOrRemote) | CAN_CS_DLC(pTxFrame->length);

        /* Load Message Payload. */
        base->MB[mbIdx].WORD0 = pTxFrame->dataWord0;
        base->MB[mbIdx].WORD1 = pTxFrame->dataWord1;

        /* Activate Tx Message Buffer. */
        base->MB[mbIdx].CS = cs_temp;
        status = kStatus_Success;
    }
    else
    {
        /* Tx Message Buffer is activated, return immediately. */
        status = kStatus_Fail;
    }

    return status;
}
/*!
 * brief Reads a FlexCAN Message from Receive Message Buffer.
 *
 * This function reads a CAN message from a specified Receive Message Buffer.
 * The function fills a receive CAN message frame structure with
 * just received data and activates the Message Buffer again.
 * The function returns immediately.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN Message Buffer index.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * retval kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 * retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t FLEXCAN_ReadRxMb(CAN_Type *base, uint8_t mbIdx, flexcan_frame_t *pRxFrame)
{
    /* Assertion. */
    assert(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
    assert(NULL != pRxFrame);
#if !defined(NDEBUG)
    assert(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    uint32_t cs_temp;
    uint32_t rx_code;
    status_t status;

    /* Read CS field of Rx Message Buffer to lock Message Buffer. */
    cs_temp = base->MB[mbIdx].CS;
    /* Get Rx Message Buffer Code field. */
    rx_code = (cs_temp & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT;

    /* Check to see if Rx Message Buffer is full. */
    if (((uint32_t)kFLEXCAN_RxMbFull == rx_code) || ((uint32_t)kFLEXCAN_RxMbOverrun == rx_code))
    {
        /* Store Message ID. */
        pRxFrame->id = base->MB[mbIdx].ID & (CAN_ID_EXT_MASK | CAN_ID_STD_MASK);

        /* Get the message ID and format. */
        pRxFrame->format = (cs_temp & CAN_CS_IDE_MASK) != 0U ? (uint8_t)kFLEXCAN_FrameFormatExtend :
                                                               (uint8_t)kFLEXCAN_FrameFormatStandard;

        /* Get the message type. */
        pRxFrame->type =
            (cs_temp & CAN_CS_RTR_MASK) != 0U ? (uint8_t)kFLEXCAN_FrameTypeRemote : (uint8_t)kFLEXCAN_FrameTypeData;

        /* Get the message length. */
        pRxFrame->length = (uint8_t)((cs_temp & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT);

        /* Get the time stamp. */
        pRxFrame->timestamp = (uint16_t)((cs_temp & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

        /* Store Message Payload. */
        pRxFrame->dataWord0 = base->MB[mbIdx].WORD0;
        pRxFrame->dataWord1 = base->MB[mbIdx].WORD1;

        /* Read free-running timer to unlock Rx Message Buffer. */
        (void)base->TIMER;

        if ((uint32_t)kFLEXCAN_RxMbFull == rx_code)
        {
            status = kStatus_Success;
        }
        else
        {
            status = kStatus_FLEXCAN_RxOverflow;
        }
    }
    else
    {
        /* Read free-running timer to unlock Rx Message Buffer. */
        (void)base->TIMER;

        status = kStatus_Fail;
    }

    return status;
}
/*!
 * brief Reads a FlexCAN Message from Legacy Rx FIFO.
 *
 * This function reads a CAN message from the FlexCAN Legacy Rx FIFO.
 *
 * param base FlexCAN peripheral base address.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * retval kStatus_Success - Read Message from Rx FIFO successfully.
 * retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t FLEXCAN_ReadRxFifo(CAN_Type *base, flexcan_frame_t *pRxFrame)
{
    /* Assertion. */
    assert(NULL != pRxFrame);

    uint32_t cs_temp;
    status_t status;

    /* Check if Legacy Rx FIFO is Enabled. */
    if (0U != (base->MCR & CAN_MCR_RFEN_MASK))
    {
        /* Read CS field of Rx Message Buffer to lock Message Buffer. */
        cs_temp = base->MB[0].CS;

        /* Read data from Rx FIFO output port. */
        /* Store Message ID. */
        pRxFrame->id = base->MB[0].ID & (CAN_ID_EXT_MASK | CAN_ID_STD_MASK);

        /* Get the message ID and format. */
        pRxFrame->format = (cs_temp & CAN_CS_IDE_MASK) != 0U ? (uint8_t)kFLEXCAN_FrameFormatExtend :
                                                               (uint8_t)kFLEXCAN_FrameFormatStandard;

        /* Get the message type. */
        pRxFrame->type =
            (cs_temp & CAN_CS_RTR_MASK) != 0U ? (uint8_t)kFLEXCAN_FrameTypeRemote : (uint8_t)kFLEXCAN_FrameTypeData;

        /* Get the message length. */
        pRxFrame->length = (uint8_t)((cs_temp & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT);

        /* Get the time stamp. */
        pRxFrame->timestamp = (uint16_t)((cs_temp & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

        /* Store Message Payload. */
        pRxFrame->dataWord0 = base->MB[0].WORD0;
        pRxFrame->dataWord1 = base->MB[0].WORD1;

        /* Store ID Filter Hit Index. */
        pRxFrame->idhit = (uint16_t)(base->RXFIR & CAN_RXFIR_IDHIT_MASK);

        /* Read free-running timer to unlock Rx Message Buffer. */
        (void)base->TIMER;

        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}
/*!
 * brief Performs a polling send transaction on the CAN bus.
 *
 * note  A transfer handle does not need to be created  before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 * param pTxFrame Pointer to CAN message frame to be sent.
 * retval kStatus_Success - Write Tx Message Buffer Successfully.
 * retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t FLEXCAN_TransferSendBlocking(CAN_Type *base, uint8_t mbIdx, flexcan_frame_t *pTxFrame)
{
    status_t status;

    /* Write Tx Message Buffer to initiate a data sending. */
    if (kStatus_Success == FLEXCAN_WriteTxMb(base, mbIdx, (const flexcan_frame_t *)(uint32_t)pTxFrame))
    {
/* Wait until CAN Message send out. */
        uint32_t u32flag = 1;
        while (0U == FLEXCAN_GetMbStatusFlags(base, u32flag << mbIdx))
        {
        }

/* Clean Tx Message Buffer Flag. */
        FLEXCAN_ClearMbStatusFlags(base, u32flag << mbIdx);
        /*After TX MB tranfered success, update the Timestamp from MB[mbIdx].CS register*/
        pTxFrame->timestamp = (uint16_t)((base->MB[mbIdx].CS & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

/*!
 * brief Performs a polling receive transaction on the CAN bus.
 *
 * note  A transfer handle does not need to be created  before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * retval kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 * retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t FLEXCAN_TransferReceiveBlocking(CAN_Type *base, uint8_t mbIdx, flexcan_frame_t *pRxFrame)
{
/* Wait until Rx Message Buffer non-empty. */
    uint32_t u32flag = 1;
    while (0U == FLEXCAN_GetMbStatusFlags(base, u32flag << mbIdx))
    {
    }

/* Clean Rx Message Buffer Flag. */
    FLEXCAN_ClearMbStatusFlags(base, u32flag << mbIdx);

    /* Read Received CAN Message. */
    return FLEXCAN_ReadRxMb(base, mbIdx, pRxFrame);
}
/*!
 * brief Performs a polling receive transaction from Legacy Rx FIFO on the CAN bus.
 *
 * note  A transfer handle does not need to be created before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * retval kStatus_Success - Read Message from Rx FIFO successfully.
 * retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t FLEXCAN_TransferReceiveFifoBlocking(CAN_Type *base, flexcan_frame_t *pRxFrame)
{
    status_t rxFifoStatus;

    /* Wait until Legacy Rx FIFO non-empty. */
    while (0U == FLEXCAN_GetMbStatusFlags(base, (uint32_t)kFLEXCAN_RxFifoFrameAvlFlag))
    {
    }

    /* Read data from Legacy Rx FIFO. */
    rxFifoStatus = FLEXCAN_ReadRxFifo(base, pRxFrame);

    /* Clean Rx Fifo available flag. */
    FLEXCAN_ClearMbStatusFlags(base, (uint32_t)kFLEXCAN_RxFifoFrameAvlFlag);

    return rxFifoStatus;
}
/*!
 * brief Initializes the FlexCAN handle.
 *
 * This function initializes the FlexCAN handle, which can be used for other FlexCAN
 * transactional APIs. Usually, for a specified FlexCAN instance,
 * call this API once to get the initialized handle.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param callback The callback function.
 * param userData The parameter of the callback function.
 */
void FLEXCAN_TransferCreateHandle(CAN_Type *base,
                                  flexcan_handle_t *handle,
                                  flexcan_transfer_callback_t callback,
                                  void *userData)
{
    assert(NULL != handle);

    uint8_t instance;

    /* Clean FlexCAN transfer handle. */
    (void)memset(handle, 0, sizeof(*handle));

    /* Get instance from peripheral base address. */
    instance = (uint8_t)FLEXCAN_GetInstance(base);

    /* Save the context in global variables to support the double weak mechanism. */
    s_flexcanHandle[instance] = handle;

    /* Register Callback function. */
    handle->callback = callback;
    handle->userData = userData;

    s_flexcanIsr = FLEXCAN_TransferHandleIRQ;

    /* We Enable Error & Status interrupt here, because this interrupt just
     * report current status of FlexCAN module through Callback function.
     * It is insignificance without a available callback function.
     */
    if (handle->callback != NULL)
    {
        FLEXCAN_EnableInterrupts(
            base, (uint32_t)kFLEXCAN_BusOffInterruptEnable | (uint32_t)kFLEXCAN_ErrorInterruptEnable |
                      (uint32_t)kFLEXCAN_RxWarningInterruptEnable | (uint32_t)kFLEXCAN_TxWarningInterruptEnable |
                      (uint32_t)kFLEXCAN_WakeUpInterruptEnable
        );
    }
    else
    {
        FLEXCAN_DisableInterrupts(
            base, (uint32_t)kFLEXCAN_BusOffInterruptEnable | (uint32_t)kFLEXCAN_ErrorInterruptEnable |
                      (uint32_t)kFLEXCAN_RxWarningInterruptEnable | (uint32_t)kFLEXCAN_TxWarningInterruptEnable |
                      (uint32_t)kFLEXCAN_WakeUpInterruptEnable
        );
    }

    /* Enable interrupts in NVIC. */
    (void)EnableIRQ((IRQn_Type)(s_flexcanRxWarningIRQ[instance]));
    (void)EnableIRQ((IRQn_Type)(s_flexcanTxWarningIRQ[instance]));
    (void)EnableIRQ((IRQn_Type)(s_flexcanWakeUpIRQ[instance]));
    (void)EnableIRQ((IRQn_Type)(s_flexcanErrorIRQ[instance]));
    (void)EnableIRQ((IRQn_Type)(s_flexcanBusOffIRQ[instance]));
    (void)EnableIRQ((IRQn_Type)(s_flexcanMbIRQ[instance]));
}

/*!
 * brief Sends a message using IRQ.
 *
 * This function sends a message using IRQ. This is a non-blocking function, which returns
 * right away. When messages have been sent out, the send callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pMbXfer FlexCAN Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
 * retval kStatus_Success        Start Tx Message Buffer sending process successfully.
 * retval kStatus_Fail           Write Tx Message Buffer failed.
 * retval kStatus_FLEXCAN_TxBusy Tx Message Buffer is in use.
 */
status_t FLEXCAN_TransferSendNonBlocking(CAN_Type *base, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer)
{
    /* Assertion. */
    assert(NULL != handle);
    assert(NULL != pMbXfer);
    assert(pMbXfer->mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    assert(!FLEXCAN_IsMbOccupied(base, pMbXfer->mbIdx));
#endif

    status_t status;

    /* Check if Message Buffer is idle. */
    if ((uint8_t)kFLEXCAN_StateIdle == handle->mbState[pMbXfer->mbIdx])
    {
        /* Distinguish transmit type. */
        if ((uint32_t)kFLEXCAN_FrameTypeRemote == pMbXfer->frame->type)
        {
            handle->mbState[pMbXfer->mbIdx] = (uint8_t)kFLEXCAN_StateTxRemote;
        }
        else
        {
            handle->mbState[pMbXfer->mbIdx] = (uint8_t)kFLEXCAN_StateTxData;
        }

        if (kStatus_Success ==
            FLEXCAN_WriteTxMb(base, pMbXfer->mbIdx, (const flexcan_frame_t *)(uint32_t)pMbXfer->frame))
        {
/* Enable Message Buffer Interrupt. */
            uint32_t u32mask = 1;
            FLEXCAN_EnableMbInterrupts(base, u32mask << pMbXfer->mbIdx);
            status = kStatus_Success;
        }
        else
        {
            handle->mbState[pMbXfer->mbIdx] = (uint8_t)kFLEXCAN_StateIdle;
            status                          = kStatus_Fail;
        }
    }
    else
    {
        status = kStatus_FLEXCAN_TxBusy;
    }

    return status;
}

/*!
 * brief Receives a message using IRQ.
 *
 * This function receives a message using IRQ. This is non-blocking function, which returns
 * right away. When the message has been received, the receive callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pMbXfer FlexCAN Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
 * retval kStatus_Success        - Start Rx Message Buffer receiving process successfully.
 * retval kStatus_FLEXCAN_RxBusy - Rx Message Buffer is in use.
 */
status_t FLEXCAN_TransferReceiveNonBlocking(CAN_Type *base, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer)
{
    status_t status;

    /* Assertion. */
    assert(NULL != handle);
    assert(NULL != pMbXfer);
    assert(pMbXfer->mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    assert(!FLEXCAN_IsMbOccupied(base, pMbXfer->mbIdx));
#endif

    /* Check if Message Buffer is idle. */
    if ((uint8_t)kFLEXCAN_StateIdle == handle->mbState[pMbXfer->mbIdx])
    {
        handle->mbState[pMbXfer->mbIdx] = (uint8_t)kFLEXCAN_StateRxData;

        /* Register Message Buffer. */
        handle->mbFrameBuf[pMbXfer->mbIdx] = pMbXfer->frame;

/* Enable Message Buffer Interrupt. */
        uint32_t u32mask = 1;
        FLEXCAN_EnableMbInterrupts(base, u32mask << pMbXfer->mbIdx);
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_FLEXCAN_RxBusy;
    }

    return status;
}
/*!
 * brief Receives a message from Legacy Rx FIFO using IRQ.
 *
 * This function receives a message using IRQ. This is a non-blocking function, which returns
 * right away. When all messages have been received, the receive callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pFifoXfer FlexCAN Rx FIFO transfer structure. See the ref flexcan_fifo_transfer_t.
 * retval kStatus_Success            - Start Rx FIFO receiving process successfully.
 * retval kStatus_FLEXCAN_RxFifoBusy - Rx FIFO is currently in use.
 */
status_t FLEXCAN_TransferReceiveFifoNonBlocking(CAN_Type *base,
                                                flexcan_handle_t *handle,
                                                flexcan_fifo_transfer_t *pFifoXfer)
{
    /* Assertion. */
    assert(NULL != handle);
    assert(NULL != pFifoXfer);

    status_t status;

    /* Check if Message Buffer is idle. */
    if ((uint8_t)kFLEXCAN_StateIdle == handle->rxFifoState)
    {
        handle->rxFifoState = (uint8_t)kFLEXCAN_StateRxFifo;

        /* Register Message Buffer. */
        handle->rxFifoFrameBuf = pFifoXfer->frame;

        /* Enable Message Buffer Interrupt. */
        FLEXCAN_EnableMbInterrupts(base, (uint32_t)kFLEXCAN_RxFifoOverflowFlag | (uint32_t)kFLEXCAN_RxFifoWarningFlag |
                                             (uint32_t)kFLEXCAN_RxFifoFrameAvlFlag);

        status = kStatus_Success;
    }
    else
    {
        status = kStatus_FLEXCAN_RxFifoBusy;
    }

    return status;
}
/*!
 * brief Aborts the interrupt driven message send process.
 *
 * This function aborts the interrupt driven message send process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 */
void FLEXCAN_TransferAbortSend(CAN_Type *base, flexcan_handle_t *handle, uint8_t mbIdx)
{
    uint16_t timestamp;

    /* Assertion. */
    assert(NULL != handle);
    assert(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    assert(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

/* Disable Message Buffer Interrupt. */
    uint32_t u32mask = 1;
    FLEXCAN_DisableMbInterrupts(base, u32mask << mbIdx);

    /* Update the TX frame 's time stamp by MB[mbIdx].cs. */
    timestamp                = (uint16_t)((base->MB[mbIdx].CS & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);
    handle->timestamp[mbIdx] = timestamp;

    /* Clean Message Buffer. */
    FLEXCAN_SetTxMbConfig(base, mbIdx, true);

    handle->mbState[mbIdx] = (uint8_t)kFLEXCAN_StateIdle;
}
/*!
 * brief Aborts the interrupt driven message receive process.
 *
 * This function aborts the interrupt driven message receive process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 */
void FLEXCAN_TransferAbortReceive(CAN_Type *base, flexcan_handle_t *handle, uint8_t mbIdx)
{
    /* Assertion. */
    assert(NULL != handle);
    assert(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    assert(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif
    uint32_t u32mask = 1;
    FLEXCAN_DisableMbInterrupts(base, (u32mask << mbIdx));

    /* Un-register handle. */
    handle->mbFrameBuf[mbIdx] = NULL;
    handle->mbState[mbIdx]    = (uint8_t)kFLEXCAN_StateIdle;
}

/*!
 * brief Aborts the interrupt driven message receive from Legacy Rx FIFO process.
 *
 * This function aborts the interrupt driven message receive from Legacy Rx FIFO process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 */
void FLEXCAN_TransferAbortReceiveFifo(CAN_Type *base, flexcan_handle_t *handle)
{
    /* Assertion. */
    assert(NULL != handle);

    /* Check if Rx FIFO is enabled. */
    if (0U != (base->MCR & CAN_MCR_RFEN_MASK))
    {
        /* Disable Rx Message FIFO Interrupts. */
        FLEXCAN_DisableMbInterrupts(base, (uint32_t)kFLEXCAN_RxFifoOverflowFlag | (uint32_t)kFLEXCAN_RxFifoWarningFlag |
                                              (uint32_t)kFLEXCAN_RxFifoFrameAvlFlag);

        /* Un-register handle. */
        handle->rxFifoFrameBuf = NULL;
    }

    handle->rxFifoState = (uint8_t)kFLEXCAN_StateIdle;
}
/*!
 * brief Gets the detail index of Mailbox's Timestamp by handle.
 *
 * Then function can only be used when calling non-blocking Data transfer (TX/RX) API,
 * After TX/RX data transfer done (User can get the status by handler's callback function),
 * we can get the detail index of Mailbox's timestamp by handle,
 * Detail non-blocking data transfer API (TX/RX) contain.
 *   -FLEXCAN_TransferSendNonBlocking
 *   -FLEXCAN_TransferFDSendNonBlocking
 *   -FLEXCAN_TransferReceiveNonBlocking
 *   -FLEXCAN_TransferFDReceiveNonBlocking
 *   -FLEXCAN_TransferReceiveFifoNonBlocking
 *
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN FD Message Buffer index.
 * return the index of mailbox 's timestamp stored in the handle.
 *
 */
uint32_t FLEXCAN_GetTimeStamp(flexcan_handle_t *handle, uint8_t mbIdx)
{
    /* Assertion. */
    assert(NULL != handle);

    return (uint32_t)(handle->timestamp[mbIdx]);
}

/*!
 * brief Check unhandle interrupt events
 *
 * param base FlexCAN peripheral base address.
 * return TRUE if unhandled interrupt action exist, FALSE if no unhandlered interrupt action exist.
 */
static bool FLEXCAN_CheckUnhandleInterruptEvents(CAN_Type *base)
{
    uint64_t tempmask;
    uint64_t tempflag;
    bool fgRet = false;

    /* Checking exist error or status flag. */
    if (0U == (FLEXCAN_GetStatusFlags(base) & (FLEXCAN_ERROR_AND_STATUS_INIT_FLAG | FLEXCAN_WAKE_UP_FLAG)))
    {
        tempmask = (uint64_t)base->IMASK1;
        tempflag = (uint64_t)base->IFLAG1;
        fgRet = (0U != (tempmask & tempflag));
    }
    else
    {
        fgRet = true;
    }

    return fgRet;
}

/*!
 * brief Sub Handler Data Trasfered Events
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pResult Pointer to the Handle result.
 *
 * return the status after handle each data transfered event.
 */
static status_t FLEXCAN_SubHandlerForDataTransfered(CAN_Type *base, flexcan_handle_t *handle, uint32_t *pResult)
{
    status_t status = kStatus_FLEXCAN_UnHandled;
    uint32_t result = 0xFFU;

    /* For this implementation, we solve the Message with lowest MB index first. */
    for (result = 0U; result < (uint32_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base); result++)
    {
        /* Get the lowest unhandled Message Buffer */
        uint32_t u32flag = 1;
        if (0U != FLEXCAN_GetMbStatusFlags(base, u32flag << result))
        {
            if (FLEXCAN_IsMbIntEnabled(base, (uint8_t)result))
            {
                break;
            }
        }
    }

    /* find Message to deal with. */
    if (result < (uint32_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base))
    {
        /* Solve Legacy Rx FIFO interrupt. */
        if (((uint8_t)kFLEXCAN_StateIdle != handle->rxFifoState) && (result <= (uint32_t)CAN_IFLAG1_BUF7I_SHIFT) &&
            ((base->MCR & CAN_MCR_RFEN_MASK) != 0U))
        {
            uint32_t u32mask = 1;
            switch (u32mask << result)
            {
                case kFLEXCAN_RxFifoOverflowFlag:
                    status = kStatus_FLEXCAN_RxFifoOverflow;
                    break;

                case kFLEXCAN_RxFifoWarningFlag:
                    status = kStatus_FLEXCAN_RxFifoWarning;
                    break;

                case kFLEXCAN_RxFifoFrameAvlFlag:
                    status = FLEXCAN_ReadRxFifo(base, handle->rxFifoFrameBuf);
                    if (kStatus_Success == status)
                    {
                        /* Align the current (index 0) rxfifo timestamp to the timestamp array by handle. */
                        handle->timestamp[0] = handle->rxFifoFrameBuf->timestamp;
                        status               = kStatus_FLEXCAN_RxFifoIdle;
                    }
                    FLEXCAN_TransferAbortReceiveFifo(base, handle);
                    break;

                default:
                    status = kStatus_FLEXCAN_UnHandled;
                    break;
            }
        }
        else
        {
            /* Get current State of Message Buffer. */
            switch (handle->mbState[result])
            {
                /* Solve Rx Data Frame. */
                case (uint8_t)kFLEXCAN_StateRxData:
                    {
                        status = FLEXCAN_ReadRxMb(base, (uint8_t)result, handle->mbFrameBuf[result]);
                        if (kStatus_Success == status)
                        {
                            /* Align the current index of RX MB timestamp to the timestamp array by handle. */
                            handle->timestamp[result] = handle->mbFrameBuf[result]->timestamp;
                            status                    = kStatus_FLEXCAN_RxIdle;
                        }
                    }
                    {
                        FLEXCAN_TransferAbortReceive(base, handle, (uint8_t)result);
                    }
                    break;

                /* Sove Rx Remote Frame.  User need to Read the frame in Mail box in time by Read from MB API. */
                case (uint8_t)kFLEXCAN_StateRxRemote:
                    status = kStatus_FLEXCAN_RxRemote;
                    {
                        FLEXCAN_TransferAbortReceive(base, handle, (uint8_t)result);
                    }
                    break;

                /* Solve Tx Data Frame. */
                case (uint8_t)kFLEXCAN_StateTxData:
                    status = kStatus_FLEXCAN_TxIdle;
                    {
                        FLEXCAN_TransferAbortSend(base, handle, (uint8_t)result);
                    }
                    break;

                /* Solve Tx Remote Frame. */
                case (uint8_t)kFLEXCAN_StateTxRemote:
                    handle->mbState[result] = (uint8_t)kFLEXCAN_StateRxRemote;
                    status                  = kStatus_FLEXCAN_TxSwitchToRx;
                    break;

                default:
                    status = kStatus_FLEXCAN_UnHandled;
                    break;
            }
        }

        /* Clear resolved Message Buffer IRQ. */
        uint32_t u32flag = 1;
        FLEXCAN_ClearMbStatusFlags(base, u32flag << result);
    }

    *pResult = result;

    return status;
}

/*!
 * brief FlexCAN IRQ handle function.
 *
 * This function handles the FlexCAN Error, the Message Buffer, and the Rx FIFO IRQ request.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 */
void FLEXCAN_TransferHandleIRQ(CAN_Type *base, flexcan_handle_t *handle)
{
    /* Assertion. */
    assert(NULL != handle);

    status_t status;
    uint32_t mbNum = 0xFFU;
    uint32_t result = 0U;
    do
    {
        /* Get Current FlexCAN Module Error and Status. */
        result = FLEXCAN_GetStatusFlags(base);

        /* To handle FlexCAN Error and Status Interrupt first. */
        if (0U != (result & FLEXCAN_ERROR_AND_STATUS_INIT_FLAG))
        {
            status = kStatus_FLEXCAN_ErrorStatus;
            /* Clear FlexCAN Error and Status Interrupt. */
            FLEXCAN_ClearStatusFlags(base, FLEXCAN_ERROR_AND_STATUS_INIT_FLAG);
        }
        else if (0U != (result & FLEXCAN_WAKE_UP_FLAG))
        {
            status = kStatus_FLEXCAN_WakeUp;
            FLEXCAN_ClearStatusFlags(base, FLEXCAN_WAKE_UP_FLAG);
        }
        else
        {
            /* To handle Message Buffer or Legacy Rx FIFO transfer. */
            status = FLEXCAN_SubHandlerForDataTransfered(base, handle, &mbNum);
            result = mbNum;
        }

        /* Calling Callback Function if has one. */
        if (handle->callback != NULL)
        {
            handle->callback(base, handle, status, result, handle->userData);
        }
    } while (FLEXCAN_CheckUnhandleInterruptEvents(base));
}

#if defined(CAN0)
void CAN0_DriverIRQHandler(void);
void CAN0_DriverIRQHandler(void)
{
    assert(NULL != s_flexcanHandle[0]);

    s_flexcanIsr(CAN0, s_flexcanHandle[0]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(CAN1)
void CAN1_DriverIRQHandler(void);
void CAN1_DriverIRQHandler(void)
{
    assert(NULL != s_flexcanHandle[1]);

    s_flexcanIsr(CAN1, s_flexcanHandle[1]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCAN1)
void CAN_FD1_DriverIRQHandler(void)
{
    assert(NULL != s_flexcanHandle[1]);

    s_flexcanIsr(FLEXCAN1, s_flexcanHandle[1]);
    SDK_ISR_EXIT_BARRIER;
}
#endif
