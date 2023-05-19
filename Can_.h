/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _FSL_FLEXCAN_H_
#define _FSL_FLEXCAN_H_

#include "fsl_common.h"

/*!
 * @addtogroup flexcan_driver
 * @{
 */

/******************************************************************************
 * Definitions
 *****************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief FlexCAN driver version. */
#define FSL_FLEXCAN_DRIVER_VERSION (MAKE_VERSION(2, 8, 2))
/*@}*/

/*! @brief FlexCAN frame length helper macro. */

/*! @brief FlexCAN Frame ID helper macro. */
#define FLEXCAN_ID_STD(id) \
    (((uint32_t)(((uint32_t)(id)) << CAN_ID_STD_SHIFT)) & CAN_ID_STD_MASK) /*!< Standard Frame ID helper macro. */
#define FLEXCAN_ID_EXT(id)                                \
    (((uint32_t)(((uint32_t)(id)) << CAN_ID_EXT_SHIFT)) & \
     (CAN_ID_EXT_MASK | CAN_ID_STD_MASK)) /*!< Extend Frame ID helper macro. */

/*! @brief FlexCAN Rx Message Buffer Mask helper macro. */
#define FLEXCAN_RX_MB_STD_MASK(id, rtr, ide)                                   \
    (((uint32_t)((uint32_t)(rtr) << 31) | (uint32_t)((uint32_t)(ide) << 30)) | \
     FLEXCAN_ID_STD(id)) /*!< Standard Rx Message Buffer Mask helper macro. */
#define FLEXCAN_RX_MB_EXT_MASK(id, rtr, ide)                                   \
    (((uint32_t)((uint32_t)(rtr) << 31) | (uint32_t)((uint32_t)(ide) << 30)) | \
     FLEXCAN_ID_EXT(id)) /*!< Extend Rx Message Buffer Mask helper macro. */

/*! @brief FlexCAN Legacy Rx FIFO Mask helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_A(id, rtr, ide)                          \
    (((uint32_t)((uint32_t)(rtr) << 31) | (uint32_t)((uint32_t)(ide) << 30)) | \
     (FLEXCAN_ID_STD(id) << 1)) /*!< Standard Rx FIFO Mask helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_HIGH(id, rtr, ide)                     \
    (((uint32_t)((uint32_t)(rtr) << 31) | (uint32_t)((uint32_t)(ide) << 30)) | \
     (((uint32_t)(id)&0x7FF) << 19)) /*!< Standard Rx FIFO Mask helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_LOW(id, rtr, ide)                      \
    (((uint32_t)((uint32_t)(rtr) << 15) | (uint32_t)((uint32_t)(ide) << 14)) | \
     (((uint32_t)(id)&0x7FF) << 3)) /*!< Standard Rx FIFO Mask helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_HIGH(id) \
    (((uint32_t)(id)&0x7F8) << 21) /*!< Standard Rx FIFO Mask helper macro Type C upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_HIGH(id) \
    (((uint32_t)(id)&0x7F8) << 13) /*!< Standard Rx FIFO Mask helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_LOW(id) \
    (((uint32_t)(id)&0x7F8) << 5) /*!< Standard Rx FIFO Mask helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_LOW(id) \
    (((uint32_t)(id)&0x7F8) >> 3) /*!< Standard Rx FIFO Mask helper macro Type C lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_A(id, rtr, ide)                          \
    (((uint32_t)((uint32_t)(rtr) << 31) | (uint32_t)((uint32_t)(ide) << 30)) | \
     (FLEXCAN_ID_EXT(id) << 1)) /*!< Extend Rx FIFO Mask helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(id, rtr, ide)                        \
    (                                                                             \
        ((uint32_t)((uint32_t)(rtr) << 31) | (uint32_t)((uint32_t)(ide) << 30)) | \
        ((FLEXCAN_ID_EXT(id) & 0x1FFF8000)                                        \
         << 1)) /*!< Extend Rx FIFO Mask helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(id, rtr, ide)                      \
    (((uint32_t)((uint32_t)(rtr) << 15) | (uint32_t)((uint32_t)(ide) << 14)) | \
     ((FLEXCAN_ID_EXT(id) & 0x1FFF8000) >>                                     \
      15)) /*!< Extend Rx FIFO Mask helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_HIGH(id) \
    ((FLEXCAN_ID_EXT(id) & 0x1FE00000) << 3) /*!< Extend Rx FIFO Mask helper macro Type C upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_HIGH(id) \
    ((FLEXCAN_ID_EXT(id) & 0x1FE00000) >>            \
     5) /*!< Extend Rx FIFO Mask helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_LOW(id) \
    ((FLEXCAN_ID_EXT(id) & 0x1FE00000) >>           \
     13) /*!< Extend Rx FIFO Mask helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_LOW(id) \
    ((FLEXCAN_ID_EXT(id) & 0x1FE00000) >> 21) /*!< Extend Rx FIFO Mask helper macro Type C lower part helper macro. */

/*! @brief FlexCAN Rx FIFO Filter helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(id, rtr, ide) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_A(id, rtr, ide) /*!< Standard Rx FIFO Filter helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_B_HIGH(id, rtr, ide) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_HIGH(                    \
        id, rtr, ide) /*!< Standard Rx FIFO Filter helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_B_LOW(id, rtr, ide) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_LOW(                    \
        id, rtr, ide) /*!< Standard Rx FIFO Filter helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_HIGH(id) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_HIGH(          \
        id) /*!< Standard Rx FIFO Filter helper macro Type C upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_MID_HIGH(id) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_HIGH(          \
        id) /*!< Standard Rx FIFO Filter helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_MID_LOW(id) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_LOW(          \
        id) /*!< Standard Rx FIFO Filter helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_LOW(id) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_LOW(          \
        id) /*!< Standard Rx FIFO Filter helper macro Type C lower part helper macro.  */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_A(id, rtr, ide) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_A(id, rtr, ide) /*!< Extend Rx FIFO Filter helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_B_HIGH(id, rtr, ide) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(                    \
        id, rtr, ide) /*!< Extend Rx FIFO Filter helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_B_LOW(id, rtr, ide) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(                    \
        id, rtr, ide) /*!< Extend Rx FIFO Filter helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_HIGH(id) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_HIGH(          \
        id) /*!< Extend Rx FIFO Filter helper macro Type C upper part helper macro.           */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_MID_HIGH(id) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_HIGH(          \
        id) /*!< Extend Rx FIFO Filter helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_MID_LOW(id) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_LOW(          \
        id) /*!< Extend Rx FIFO Filter helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_LOW(id) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_LOW(id) /*!< Extend Rx FIFO Filter helper macro Type C lower part helper macro. */

/*! @brief FlexCAN interrupt/status flag helper macro. */

#define FLEXCAN_ERROR_AND_STATUS_INIT_FLAG                                                                          \
    ((uint32_t)kFLEXCAN_TxWarningIntFlag | (uint32_t)kFLEXCAN_RxWarningIntFlag | (uint32_t)kFLEXCAN_BusOffIntFlag | \
     (uint32_t)kFLEXCAN_ErrorIntFlag | FLEXCAN_MEMORY_ERROR_INIT_FLAG)

#define FLEXCAN_WAKE_UP_FLAG ((uint32_t)kFLEXCAN_WakeUpIntFlag)

#define FLEXCAN_MEMORY_ERROR_INIT_FLAG (0U)

    /*! @brief FlexCAN transfer status. */
    enum
    {
        kStatus_FLEXCAN_TxBusy       = MAKE_STATUS(kStatusGroup_FLEXCAN, 0), /*!< Tx Message Buffer is Busy. */
        kStatus_FLEXCAN_TxIdle       = MAKE_STATUS(kStatusGroup_FLEXCAN, 1), /*!< Tx Message Buffer is Idle. */
        kStatus_FLEXCAN_TxSwitchToRx = MAKE_STATUS(
            kStatusGroup_FLEXCAN, 2), /*!< Remote Message is send out and Message buffer changed to Receive one. */
        kStatus_FLEXCAN_RxBusy         = MAKE_STATUS(kStatusGroup_FLEXCAN, 3), /*!< Rx Message Buffer is Busy. */
        kStatus_FLEXCAN_RxIdle         = MAKE_STATUS(kStatusGroup_FLEXCAN, 4), /*!< Rx Message Buffer is Idle. */
        kStatus_FLEXCAN_RxOverflow     = MAKE_STATUS(kStatusGroup_FLEXCAN, 5), /*!< Rx Message Buffer is Overflowed. */
        kStatus_FLEXCAN_RxFifoBusy     = MAKE_STATUS(kStatusGroup_FLEXCAN, 6), /*!< Rx Message FIFO is Busy. */
        kStatus_FLEXCAN_RxFifoIdle     = MAKE_STATUS(kStatusGroup_FLEXCAN, 7), /*!< Rx Message FIFO is Idle. */
        kStatus_FLEXCAN_RxFifoOverflow = MAKE_STATUS(kStatusGroup_FLEXCAN, 8), /*!< Rx Message FIFO is overflowed. */
        kStatus_FLEXCAN_RxFifoWarning  = MAKE_STATUS(kStatusGroup_FLEXCAN, 9), /*!< Rx Message FIFO is almost overflowed. */
        kStatus_FLEXCAN_ErrorStatus    = MAKE_STATUS(kStatusGroup_FLEXCAN, 10), /*!< FlexCAN Module Error and Status. */
        kStatus_FLEXCAN_WakeUp         = MAKE_STATUS(kStatusGroup_FLEXCAN, 11), /*!< FlexCAN is waken up from STOP mode. */
        kStatus_FLEXCAN_UnHandled      = MAKE_STATUS(kStatusGroup_FLEXCAN, 12), /*!< UnHadled Interrupt asserted. */
        kStatus_FLEXCAN_RxRemote = MAKE_STATUS(kStatusGroup_FLEXCAN, 13), /*!< Rx Remote Message Received in Mail box. */
    };

/*! @brief FlexCAN frame format. */
typedef enum _flexcan_frame_format
{
    kFLEXCAN_FrameFormatStandard = 0x0U, /*!< Standard frame format attribute. */
    kFLEXCAN_FrameFormatExtend   = 0x1U, /*!< Extend frame format attribute. */
} flexcan_frame_format_t;

/*! @brief FlexCAN frame type. */
typedef enum _flexcan_frame_type
{
    kFLEXCAN_FrameTypeData   = 0x0U, /*!< Data frame type attribute. */
    kFLEXCAN_FrameTypeRemote = 0x1U, /*!< Remote frame type attribute. */
} flexcan_frame_type_t;

/*! @brief FlexCAN clock source.
 *  @deprecated Do not use the kFLEXCAN_ClkSrcOs.  It has been superceded kFLEXCAN_ClkSrc0
 *  @deprecated Do not use the kFLEXCAN_ClkSrcPeri.  It has been superceded kFLEXCAN_ClkSrc1
 */
typedef enum _flexcan_clock_source
{
    kFLEXCAN_ClkSrcOsc  = 0x0U, /*!< FlexCAN Protocol Engine clock from Oscillator. */
    kFLEXCAN_ClkSrcPeri = 0x1U, /*!< FlexCAN Protocol Engine clock from Peripheral Clock. */
    kFLEXCAN_ClkSrc0    = 0x0U, /*!< FlexCAN Protocol Engine clock selected by user as SRC == 0. */
    kFLEXCAN_ClkSrc1    = 0x1U, /*!< FlexCAN Protocol Engine clock selected by user as SRC == 1. */
} flexcan_clock_source_t;

/*! @brief FlexCAN wake up source. */
typedef enum _flexcan_wake_up_source
{
    kFLEXCAN_WakeupSrcUnfiltered = 0x0U, /*!< FlexCAN uses unfiltered Rx input to detect edge. */
    kFLEXCAN_WakeupSrcFiltered   = 0x1U, /*!< FlexCAN uses filtered Rx input to detect edge. */
} flexcan_wake_up_source_t;

/*! @brief FlexCAN Rx Fifo Filter type. */
typedef enum _flexcan_rx_fifo_filter_type
{
    kFLEXCAN_RxFifoFilterTypeA = 0x0U, /*!< One full ID (standard and extended) per ID Filter element. */
    kFLEXCAN_RxFifoFilterTypeB =
        0x1U, /*!< Two full standard IDs or two partial 14-bit ID slices per ID Filter Table element. */
    kFLEXCAN_RxFifoFilterTypeC =
        0x2U, /*!< Four partial 8-bit Standard or extended ID slices per ID Filter Table element. */
    kFLEXCAN_RxFifoFilterTypeD = 0x3U, /*!< All frames rejected. */
} flexcan_rx_fifo_filter_type_t;

/*!
 * @brief FlexCAN Enhanced/Legacy Rx FIFO priority.
 *
 * The matching process starts from the Rx MB(or Enhanced/Legacy Rx FIFO) with higher priority.
 * If no MB(or Enhanced/Legacy Rx FIFO filter) is satisfied, the matching process goes on with
 * the Enhanced/Legacy Rx FIFO(or Rx MB) with lower priority.
 */
typedef enum _flexcan_rx_fifo_priority
{
    kFLEXCAN_RxFifoPrioLow  = 0x0U, /*!< Matching process start from Rx Message Buffer first. */
    kFLEXCAN_RxFifoPrioHigh = 0x1U, /*!< Matching process start from Enhanced/Legacy Rx FIFO first. */
} flexcan_rx_fifo_priority_t;

/*!
 * @brief FlexCAN interrupt enable enumerations.
 *
 * This provides constants for the FlexCAN interrupt enable enumerations for use in the FlexCAN functions.
 * @note FlexCAN Message Buffers and Legacy Rx FIFO interrupts not included in.
 */
enum _flexcan_interrupt_enable
{
    kFLEXCAN_BusOffInterruptEnable    = CAN_CTRL1_BOFFMSK_MASK, /*!< Bus Off interrupt, use bit 15. */
    kFLEXCAN_ErrorInterruptEnable     = CAN_CTRL1_ERRMSK_MASK,  /*!< CAN Error interrupt, use bit 14. */
    kFLEXCAN_TxWarningInterruptEnable = CAN_CTRL1_TWRNMSK_MASK, /*!< Tx Warning interrupt, use bit 11. */
    kFLEXCAN_RxWarningInterruptEnable = CAN_CTRL1_RWRNMSK_MASK, /*!< Rx Warning interrupt, use bit 10. */
    kFLEXCAN_WakeUpInterruptEnable    = CAN_MCR_WAKMSK_MASK,    /*!< Self Wake Up interrupt, use bit 22. */

};

/*!
 * @brief FlexCAN status flags.
 *
 * This provides constants for the FlexCAN status flags for use in the FlexCAN functions.
 * @note The CPU read action clears the bits corresponding to the FlEXCAN_ErrorFlag macro, therefore user need to
 * read status flags and distinguish which error is occur using @ref _flexcan_error_flags enumerations.
 */
enum _flexcan_flags
{
    kFLEXCAN_SynchFlag            = CAN_ESR1_SYNCH_MASK,   /*!< CAN Synchronization Status. */
    kFLEXCAN_TxWarningIntFlag     = CAN_ESR1_TWRNINT_MASK, /*!< Tx Warning Interrupt Flag. */
    kFLEXCAN_RxWarningIntFlag     = CAN_ESR1_RWRNINT_MASK, /*!< Rx Warning Interrupt Flag. */
    kFLEXCAN_IdleFlag             = CAN_ESR1_IDLE_MASK,    /*!< FlexCAN In IDLE Status. */
    kFLEXCAN_FaultConfinementFlag = CAN_ESR1_FLTCONF_MASK, /*!< FlexCAN Fault Confinement State. */
    kFLEXCAN_TransmittingFlag     = CAN_ESR1_TX_MASK,      /*!< FlexCAN In Transmission Status. */
    kFLEXCAN_ReceivingFlag        = CAN_ESR1_RX_MASK,      /*!< FlexCAN In Reception Status. */
    kFLEXCAN_BusOffIntFlag        = CAN_ESR1_BOFFINT_MASK, /*!< Bus Off Interrupt Flag. */
    kFLEXCAN_ErrorIntFlag         = CAN_ESR1_ERRINT_MASK,  /*!< CAN Error Interrupt Flag. */
    kFLEXCAN_WakeUpIntFlag        = CAN_ESR1_WAKINT_MASK,  /*!< Self Wake-Up Interrupt Flag. */
    kFLEXCAN_ErrorFlag =
        (uint32_t)(/*!< All FlexCAN Read Clear Error Status. */
                   CAN_ESR1_TXWRN_MASK | CAN_ESR1_RXWRN_MASK | CAN_ESR1_BIT1ERR_MASK | CAN_ESR1_BIT0ERR_MASK |
                   CAN_ESR1_ACKERR_MASK | CAN_ESR1_CRCERR_MASK | CAN_ESR1_FRMERR_MASK | CAN_ESR1_STFERR_MASK),

};

/*!
 * @brief FlexCAN error status flags.
 *
 * The FlexCAN Error Status enumerations is used to report current error of the FlexCAN bus.
 * This enumerations should be used with KFLEXCAN_ErrorFlag in @ref _flexcan_flags enumerations
 * to ditermine which error is generated.
 */
enum _flexcan_error_flags
{
    kFLEXCAN_TxErrorWarningFlag = CAN_ESR1_TXWRN_MASK,   /*!< Tx Error Warning Status. */
    kFLEXCAN_RxErrorWarningFlag = CAN_ESR1_RXWRN_MASK,   /*!< Rx Error Warning Status. */
    kFLEXCAN_StuffingError      = CAN_ESR1_STFERR_MASK,  /*!< Stuffing Error. */
    kFLEXCAN_FormError          = CAN_ESR1_FRMERR_MASK,  /*!< Form Error. */
    kFLEXCAN_CrcError           = CAN_ESR1_CRCERR_MASK,  /*!< Cyclic Redundancy Check Error. */
    kFLEXCAN_AckError           = CAN_ESR1_ACKERR_MASK,  /*!< Received no ACK on transmission. */
    kFLEXCAN_Bit0Error          = CAN_ESR1_BIT0ERR_MASK, /*!< Unable to send dominant bit. */
    kFLEXCAN_Bit1Error          = CAN_ESR1_BIT1ERR_MASK, /*!< Unable to send recessive bit. */
};

/*!
 * @brief FlexCAN Legacy Rx FIFO status flags.
 *
 * The FlexCAN Legacy Rx FIFO Status enumerations are used to determine the status of the
 * Rx FIFO. Because Rx FIFO occupy the MB0 ~ MB7 (Rx Fifo filter also occupies
 * more Message Buffer space), Rx FIFO status flags are mapped to the corresponding
 * Message Buffer status flags.
 */
enum
{
    kFLEXCAN_RxFifoOverflowFlag = CAN_IFLAG1_BUF7I_MASK, /*!< Rx FIFO overflow flag. */
    kFLEXCAN_RxFifoWarningFlag  = CAN_IFLAG1_BUF6I_MASK, /*!< Rx FIFO almost full flag. */
    kFLEXCAN_RxFifoFrameAvlFlag = CAN_IFLAG1_BUF5I_MASK, /*!< Frames available in Rx FIFO flag. */
};


/*! @brief FlexCAN message frame structure. */
typedef struct _flexcan_frame
{
    struct
    {
        uint32_t timestamp : 16; /*!< FlexCAN internal Free-Running Counter Time Stamp. */
        uint32_t length : 4;     /*!< CAN frame data length in bytes (Range: 0~8). */
        uint32_t type : 1;       /*!< CAN Frame Type(DATA or REMOTE). */
        uint32_t format : 1;     /*!< CAN Frame Identifier(STD or EXT format). */
        uint32_t : 1;            /*!< Reserved. */
        uint32_t idhit : 9;      /*!< CAN Rx FIFO filter hit id(This value is only used in Rx FIFO receive mode). */
    };
    struct
    {
        uint32_t id : 29; /*!< CAN Frame Identifier, should be set using FLEXCAN_ID_EXT() or FLEXCAN_ID_STD() macro. */
        uint32_t : 3;     /*!< Reserved. */
    };
    union
    {
        struct
        {
            uint32_t dataWord0; /*!< CAN Frame payload word0. */
            uint32_t dataWord1; /*!< CAN Frame payload word1. */
        };
        struct
        {
            uint8_t dataByte3; /*!< CAN Frame payload byte3. */
            uint8_t dataByte2; /*!< CAN Frame payload byte2. */
            uint8_t dataByte1; /*!< CAN Frame payload byte1. */
            uint8_t dataByte0; /*!< CAN Frame payload byte0. */
            uint8_t dataByte7; /*!< CAN Frame payload byte7. */
            uint8_t dataByte6; /*!< CAN Frame payload byte6. */
            uint8_t dataByte5; /*!< CAN Frame payload byte5. */
            uint8_t dataByte4; /*!< CAN Frame payload byte4. */
        };
    };
} flexcan_frame_t;

/*! @brief FlexCAN protocol timing characteristic configuration structure. */
typedef struct _flexcan_timing_config
{
    uint16_t preDivider; /*!< Classic CAN or CAN FD nominal phase bit rate prescaler. */
    uint8_t rJumpwidth;  /*!< Classic CAN or CAN FD nominal phase Re-sync Jump Width. */
    uint8_t phaseSeg1;   /*!< Classic CAN or CAN FD nominal phase Segment 1. */
    uint8_t phaseSeg2;   /*!< Classic CAN or CAN FD nominal phase Segment 2. */
    uint8_t propSeg;     /*!< Classic CAN or CAN FD nominal phase Propagation Segment. */

} flexcan_timing_config_t;

/*! @brief FlexCAN module configuration structure.
 *  @deprecated Do not use the baudRate. It has been superceded bitRate
 *  @deprecated Do not use the baudRateFD. It has been superceded bitRateFD
 */
typedef struct _flexcan_config
{
    union
    {
        struct
        {
            uint32_t baudRate; /*!< FlexCAN bit rate in bps, for classical CAN or CANFD nominal phase. */
        };
        struct
        {
            uint32_t bitRate; /*!< FlexCAN bit rate in bps, for classical CAN or CANFD nominal phase. */
        };
    };
    flexcan_clock_source_t clkSrc;      /*!< Clock source for FlexCAN Protocol Engine. */
    flexcan_wake_up_source_t wakeupSrc; /*!< Wake up source selection. */
    uint8_t maxMbNum;                   /*!< The maximum number of Message Buffers used by user. */
    bool enableLoopBack;                /*!< Enable or Disable Loop Back Self Test Mode. */
    bool enableTimerSync;               /*!< Enable or Disable Timer Synchronization. */
    bool enableSelfWakeup;              /*!< Enable or Disable Self Wakeup Mode. */
    bool enableIndividMask;             /*!< Enable or Disable Rx Individual Mask and Queue feature. */
    bool disableSelfReception;          /*!< Enable or Disable Self Reflection. */
    bool enableListenOnlyMode;          /*!< Enable or Disable Listen Only Mode. */
    bool enableSupervisorMode; /*!< Enable or Disable Supervisor Mode, enable this mode will make registers allow only
                                  Supervisor access. */
    flexcan_timing_config_t timingConfig; /* Protocol timing . */
} Can_ConfigType;

/*!
 * @brief FlexCAN Receive Message Buffer configuration structure
 *
 * This structure is used as the parameter of FLEXCAN_SetRxMbConfig() function.
 * The FLEXCAN_SetRxMbConfig() function is used to configure FlexCAN Receive
 * Message Buffer. The function abort previous receiving process, clean the
 * Message Buffer and activate the Rx Message Buffer using given Message Buffer
 * setting.
 */
typedef struct _flexcan_rx_mb_config
{
    uint32_t id;                   /*!< CAN Message Buffer Frame Identifier, should be set using
                                        FLEXCAN_ID_EXT() or FLEXCAN_ID_STD() macro. */
    flexcan_frame_format_t format; /*!< CAN Frame Identifier format(Standard of Extend). */
    flexcan_frame_type_t type;     /*!< CAN Frame Type(Data or Remote). */
} flexcan_rx_mb_config_t;


/*! @brief FlexCAN Legacy Rx FIFO configuration structure. */
typedef struct _flexcan_rx_fifo_config
{
    uint32_t *idFilterTable;                    /*!< Pointer to the FlexCAN Legacy Rx FIFO identifier filter table. */
    uint8_t idFilterNum;                        /*!< The FlexCAN Legacy Rx FIFO Filter elements quantity. */
    flexcan_rx_fifo_filter_type_t idFilterType; /*!< The FlexCAN Legacy Rx FIFO Filter type. */
    flexcan_rx_fifo_priority_t priority;        /*!< The FlexCAN Legacy Rx FIFO receive priority. */
} flexcan_rx_fifo_config_t;

/*! @brief FlexCAN Message Buffer transfer. */
typedef struct _flexcan_mb_transfer
{
    flexcan_frame_t *frame; /*!< The buffer of CAN Message to be transfer. */
    uint8_t mbIdx;          /*!< The index of Message buffer used to transfer Message. */
} flexcan_mb_transfer_t;

/*! @brief FlexCAN Rx FIFO transfer. */
typedef struct _flexcan_fifo_transfer
{
    flexcan_frame_t *frame; /*!< The buffer of CAN Message to be received from Rx FIFO. */
} flexcan_fifo_transfer_t;

/*! @brief FlexCAN handle structure definition. */
typedef struct _flexcan_handle flexcan_handle_t;

/*! @brief FlexCAN transfer callback function.
 *
 *  The FlexCAN transfer callback returns a value from the underlying layer.
 *  If the status equals to kStatus_FLEXCAN_ErrorStatus, the result parameter is the Content of
 *  FlexCAN status register which can be used to get the working status(or error status) of FlexCAN module.
 *  If the status equals to other FlexCAN Message Buffer transfer status, the result is the index of
 *  Message Buffer that generate transfer event.
 *  If the status equals to other FlexCAN Message Buffer transfer status, the result is meaningless and should be
 *  Ignored.
 */
#define FLEXCAN_CALLBACK(x) \
    void(x)(CAN_Type * base, flexcan_handle_t * handle, status_t status, uint32_t result, void *userData)
typedef void (*flexcan_transfer_callback_t)(
    CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData);

/*! @brief FlexCAN handle structure. */
struct _flexcan_handle
{
    flexcan_transfer_callback_t callback; /*!< Callback function. */
    void *userData;                       /*!< FlexCAN callback function parameter.*/
    flexcan_frame_t
        *volatile mbFrameBuf[CAN_WORD1_COUNT]; /*!< The buffer for received CAN data from Message Buffers. */
    flexcan_frame_t *volatile rxFifoFrameBuf; /*!< The buffer for received CAN data from Legacy Rx FIFO. */
    volatile uint8_t mbState[CAN_WORD1_COUNT];    /*!< Message Buffer transfer state. */
    volatile uint8_t rxFifoState;                 /*!< Rx FIFO transfer state. */
    volatile uint32_t timestamp[CAN_WORD1_COUNT]; /*!< Mailbox transfer timestamp. */
};

/******************************************************************************
 * API
 *****************************************************************************/

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Enter FlexCAN Freeze Mode.
 *
 * This function makes the FlexCAN work under Freeze Mode.
 *
 * @param base FlexCAN peripheral base address.
 */
void FLEXCAN_EnterFreezeMode(CAN_Type *base);

/*!
 * @brief Exit FlexCAN Freeze Mode.
 *
 * This function makes the FlexCAN leave Freeze Mode.
 *
 * @param base FlexCAN peripheral base address.
 */
void FLEXCAN_ExitFreezeMode(CAN_Type *base);

/*!
 * @brief Get the FlexCAN instance from peripheral base address.
 *
 * @param base FlexCAN peripheral base address.
 * @return FlexCAN instance.
 */
uint32_t FLEXCAN_GetInstance(CAN_Type *base);

/*!
 * @brief Calculates the improved timing values by specific bit Rates for classical CAN.
 *
 * This function use to calculates the Classical CAN timing values according to the given bit rate. The Calculated
 * timing values will be set in CTRL1/CBT/ENCBT register. The calculation is based on the recommendation of the CiA 301
 * v4.2.0 and previous version document.
 *
 * @param base FlexCAN peripheral base address.
 * @param bitRate  The classical CAN speed in bps defined by user, should be less than or equal to 1Mbps.
 * @param sourceClock_Hz The Source clock frequency in Hz.
 * @param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 *
 * @return TRUE if timing configuration found, FALSE if failed to find configuration.
 */
bool FLEXCAN_CalculateImprovedTimingValues(CAN_Type *base,
                                           uint32_t bitRate,
                                           uint32_t sourceClock_Hz,
                                           flexcan_timing_config_t *pTimingConfig);

/*!
 * @brief Initializes a FlexCAN instance.
 *
 * This function initializes the FlexCAN module with user-defined settings.
 * This example shows how to set up the Can_ConfigType parameters and how
 * to call the FLEXCAN_Init function by passing in these parameters.
 *  @code
 *   Can_ConfigType flexcanConfig;
 *   flexcanConfig.clkSrc               = kFLEXCAN_ClkSrc0;
 *   flexcanConfig.bitRate              = 1000000U;
 *   flexcanConfig.maxMbNum             = 16;
 *   flexcanConfig.enableLoopBack       = false;
 *   flexcanConfig.enableSelfWakeup     = false;
 *   flexcanConfig.enableIndividMask    = false;
 *   flexcanConfig.enableDoze           = false;
 *   flexcanConfig.disableSelfReception = false;
 *   flexcanConfig.enableListenOnlyMode = false;
 *   flexcanConfig.timingConfig         = timingConfig;
 *   FLEXCAN_Init(CAN0, &flexcanConfig, 40000000UL);
 *   @endcode
 *
 * @param base FlexCAN peripheral base address.
 * @param pConfig Pointer to the user-defined configuration structure.
 * @param sourceClock_Hz FlexCAN Protocol Engine clock source frequency in Hz.
 */
void FLEXCAN_Init(CAN_Type *base, const Can_ConfigType *pConfig, uint32_t sourceClock_Hz);


/*!
 * @brief De-initializes a FlexCAN instance.
 *
 * This function disables the FlexCAN module clock and sets all register values
 * to the reset value.
 *
 * @param base FlexCAN peripheral base address.
 */
void FLEXCAN_Deinit(CAN_Type *base);

/*!
 * @brief Gets the default configuration structure.
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
 *   flexcanConfig->enableMemoryErrorControl             = true;
 *   flexcanConfig->enableNonCorrectableErrorEnterFreeze = true;
 *   flexcanConfig.timingConfig                          = timingConfig;
 *
 * @param pConfig Pointer to the FlexCAN configuration structure.
 */
void FLEXCAN_GetDefaultConfig(Can_ConfigType *pConfig);

/* @} */

/*!
 * @name Configuration.
 * @{
 */

/*!
 * @brief Sets the FlexCAN protocol timing characteristic.
 *
 * This function gives user settings to classical CAN or CANFD nominal phase timing characteristic.
 * The function is for an experienced user. For less experienced users, call the FLEXCAN_GetDefaultConfig()
 * and get the default timing characteristicsthe, then call FLEXCAN_Init() and fill the
 * bit rate field.
 *
 * @note Calling FLEXCAN_SetTimingConfig() overrides the bit rate set in FLEXCAN_Init().
 *
 * @param base FlexCAN peripheral base address.
 * @param pConfig Pointer to the timing configuration structure.
 */
void FLEXCAN_SetTimingConfig(CAN_Type *base, const flexcan_timing_config_t *pConfig);

/*!
 * @brief Sets the FlexCAN receive message buffer global mask.
 *
 * This function sets the global mask for the FlexCAN message buffer in a matching process.
 * The configuration is only effective when the Rx individual mask is disabled in the FLEXCAN_Init().
 *
 * @param base FlexCAN peripheral base address.
 * @param mask Rx Message Buffer Global Mask value.
 */
void FLEXCAN_SetRxMbGlobalMask(CAN_Type *base, uint32_t mask);

/*!
 * @brief Sets the FlexCAN receive FIFO global mask.
 *
 * This function sets the global mask for FlexCAN FIFO in a matching process.
 *
 * @param base FlexCAN peripheral base address.
 * @param mask Rx Fifo Global Mask value.
 */
void FLEXCAN_SetRxFifoGlobalMask(CAN_Type *base, uint32_t mask);

/*!
 * @brief Sets the FlexCAN receive individual mask.
 *
 * This function sets the individual mask for the FlexCAN matching process.
 * The configuration is only effective when the Rx individual mask is enabled in the FLEXCAN_Init().
 * If the Rx FIFO is disabled, the individual mask is applied to the corresponding Message Buffer.
 * If the Rx FIFO is enabled, the individual mask for Rx FIFO occupied Message Buffer is applied to
 * the Rx Filter with the same index. Note that only the first 32
 * individual masks can be used as the Rx FIFO filter mask.
 *
 * @param base FlexCAN peripheral base address.
 * @param maskIdx The Index of individual Mask.
 * @param mask Rx Individual Mask value.
 */
void FLEXCAN_SetRxIndividualMask(CAN_Type *base, uint8_t maskIdx, uint32_t mask);

/*!
 * @brief Configures a FlexCAN transmit message buffer.
 *
 * This function aborts the previous transmission, cleans the Message Buffer, and
 * configures it as a Transmit Message Buffer.
 *
 * @param base FlexCAN peripheral base address.
 * @param mbIdx The Message Buffer index.
 * @param enable Enable/disable Tx Message Buffer.
 *               - true: Enable Tx Message Buffer.
 *               - false: Disable Tx Message Buffer.
 */
void FLEXCAN_SetTxMbConfig(CAN_Type *base, uint8_t mbIdx, bool enable);

/*!
 * @brief Configures a FlexCAN Receive Message Buffer.
 *
 * This function cleans a FlexCAN build-in Message Buffer and configures it
 * as a Receive Message Buffer.
 *
 * @param base FlexCAN peripheral base address.
 * @param mbIdx The Message Buffer index.
 * @param pRxMbConfig Pointer to the FlexCAN Message Buffer configuration structure.
 * @param enable Enable/disable Rx Message Buffer.
 *               - true: Enable Rx Message Buffer.
 *               - false: Disable Rx Message Buffer.
 */
void FLEXCAN_SetRxMbConfig(CAN_Type *base, uint8_t mbIdx, const flexcan_rx_mb_config_t *pRxMbConfig, bool enable);

/*!
 * @brief Configures the FlexCAN Legacy Rx FIFO.
 *
 * This function configures the FlexCAN Rx FIFO with given configuration.
 * @note Legacy Rx FIFO only can receive classic CAN message.
 *
 * @param base FlexCAN peripheral base address.
 * @param pRxFifoConfig Pointer to the FlexCAN Legacy Rx FIFO configuration structure. Can be NULL when enable parameter
 *                      is false.
 * @param enable Enable/disable Legacy Rx FIFO.
 *               - true: Enable Legacy Rx FIFO.
 *               - false: Disable Legacy Rx FIFO.
 */
void FLEXCAN_SetRxFifoConfig(CAN_Type *base, const flexcan_rx_fifo_config_t *pRxFifoConfig, bool enable);

/* @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Gets the FlexCAN module interrupt flags.
 *
 * This function gets all FlexCAN status flags. The flags are returned as the logical
 * OR value of the enumerators @ref _flexcan_flags. To check the specific status,
 * compare the return value with enumerators in @ref _flexcan_flags.
 *
 * @param base FlexCAN peripheral base address.
 * @return FlexCAN status flags which are ORed by the enumerators in the _flexcan_flags.
 */
static inline uint32_t FLEXCAN_GetStatusFlags(CAN_Type *base)
{
    return base->ESR1;
}
/*!
 * @brief Clears status flags with the provided mask.
 *
 * This function clears the FlexCAN status flags with a provided mask. An automatically cleared flag
 * can't be cleared by this function.
 *
 * @param base FlexCAN peripheral base address.
 * @param mask The status flags to be cleared, it is logical OR value of @ref _flexcan_flags.
 */
static inline void FLEXCAN_ClearStatusFlags(CAN_Type *base, uint32_t mask)
{
    /* Write 1 to clear status flag. */
    base->ESR1 = mask;
}
/*!
 * @brief Gets the FlexCAN Bus Error Counter value.
 *
 * This function gets the FlexCAN Bus Error Counter value for both Tx and
 * Rx direction. These values may be needed in the upper layer error handling.
 *
 * @param base FlexCAN peripheral base address.
 * @param txErrBuf Buffer to store Tx Error Counter value.
 * @param rxErrBuf Buffer to store Rx Error Counter value.
 */
static inline void FLEXCAN_GetBusErrCount(CAN_Type *base, uint8_t *txErrBuf, uint8_t *rxErrBuf)
{
    if (NULL != txErrBuf)
    {
        *txErrBuf = (uint8_t)((base->ECR & CAN_ECR_TXERRCNT_MASK) >> CAN_ECR_TXERRCNT_SHIFT);
    }

    if (NULL != rxErrBuf)
    {
        *rxErrBuf = (uint8_t)((base->ECR & CAN_ECR_RXERRCNT_MASK) >> CAN_ECR_RXERRCNT_SHIFT);
    }
}

/*!
 * @brief Gets the FlexCAN Message Buffer interrupt flags.
 *
 * This function gets the interrupt flags of a given Message Buffers.
 *
 * @param base FlexCAN peripheral base address.
 * @param mask The ORed FlexCAN Message Buffer mask.
 * @return The status of given Message Buffers.
 */
static inline uint32_t FLEXCAN_GetMbStatusFlags(CAN_Type *base, uint32_t mask)
{
    return (base->IFLAG1 & mask);
}

/*!
 * @brief Clears the FlexCAN Message Buffer interrupt flags.
 *
 * This function clears the interrupt flags of a given Message Buffers.
 *
 * @param base FlexCAN peripheral base address.
 * @param mask The ORed FlexCAN Message Buffer mask.
 */
static inline void FLEXCAN_ClearMbStatusFlags(CAN_Type *base, uint32_t mask)
{
    base->IFLAG1 = mask;
}
/* @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enables FlexCAN interrupts according to the provided mask.
 *
 * This function enables the FlexCAN interrupts according to the provided mask. The mask
 * is a logical OR of enumeration members, see @ref _flexcan_interrupt_enable.
 *
 * @param base FlexCAN peripheral base address.
 * @param mask The interrupts to enable. Logical OR of @ref _flexcan_interrupt_enable.
 */

static inline void FLEXCAN_EnableInterrupts(CAN_Type *base, uint32_t mask)
{
    /* Solve Self Wake Up interrupt. */
    base->MCR |= (uint32_t)(mask & (uint32_t)kFLEXCAN_WakeUpInterruptEnable);

    /* Solve interrupt enable bits in CTRL1 register. */
    base->CTRL1 |=
        (uint32_t)(mask & ((uint32_t)kFLEXCAN_BusOffInterruptEnable | (uint32_t)kFLEXCAN_ErrorInterruptEnable |
                           (uint32_t)kFLEXCAN_RxWarningInterruptEnable | (uint32_t)kFLEXCAN_TxWarningInterruptEnable |
                           (uint32_t)kFLEXCAN_WakeUpInterruptEnable));
}

/*!
 * @brief Disables FlexCAN interrupts according to the provided mask.
 *
 * This function disables the FlexCAN interrupts according to the provided mask. The mask
 * is a logical OR of enumeration members, see @ref _flexcan_interrupt_enable.
 *
 * @param base FlexCAN peripheral base address.
 * @param mask The interrupts to disable. Logical OR of @ref _flexcan_interrupt_enable.
 */
static inline void FLEXCAN_DisableInterrupts(CAN_Type *base, uint32_t mask)
{
    /* Solve Wake Up Interrupt. */
    base->MCR &= ~(uint32_t)(mask & (uint32_t)kFLEXCAN_WakeUpInterruptEnable);

    /* Solve interrupt enable bits in CTRL1 register. */
    base->CTRL1 &=
        ~(uint32_t)(mask & ((uint32_t)kFLEXCAN_BusOffInterruptEnable | (uint32_t)kFLEXCAN_ErrorInterruptEnable |
                            (uint32_t)kFLEXCAN_RxWarningInterruptEnable | (uint32_t)kFLEXCAN_TxWarningInterruptEnable |
                            (uint32_t)kFLEXCAN_WakeUpInterruptEnable));
}

/*!
 * @brief Enables FlexCAN Message Buffer interrupts.
 *
 * This function enables the interrupts of given Message Buffers.
 *
 * @param base FlexCAN peripheral base address.
 * @param mask The ORed FlexCAN Message Buffer mask.
 */
static inline void FLEXCAN_EnableMbInterrupts(CAN_Type *base, uint32_t mask)
{
    base->IMASK1 |= mask;
}

/*!
 * @brief Disables FlexCAN Message Buffer interrupts.
 *
 * This function disables the interrupts of given Message Buffers.
 *
 * @param base FlexCAN peripheral base address.
 * @param mask The ORed FlexCAN Message Buffer mask.
 */

static inline void FLEXCAN_DisableMbInterrupts(CAN_Type *base, uint32_t mask)
{
    base->IMASK1 &= ~mask;
}

/* @} */

/*!
 * @name Bus Operations
 * @{
 */

/*!
 * @brief Enables or disables the FlexCAN module operation.
 *
 * This function enables or disables the FlexCAN module.
 *
 * @param base FlexCAN base pointer.
 * @param enable true to enable, false to disable.
 */
static inline void FLEXCAN_Enable(CAN_Type *base, bool enable)
{
    if (enable)
    {
        base->MCR &= ~CAN_MCR_MDIS_MASK;

        /* Wait FlexCAN exit from low-power mode. */
        while (0U != (base->MCR & CAN_MCR_LPMACK_MASK))
        {
        }
    }
    else
    {
        base->MCR |= CAN_MCR_MDIS_MASK;

        /* Wait FlexCAN enter low-power mode. */
        while (0U == (base->MCR & CAN_MCR_LPMACK_MASK))
        {
        }
    }
}

/*!
 * @brief Writes a FlexCAN Message to the Transmit Message Buffer.
 *
 * This function writes a CAN Message to the specified Transmit Message Buffer
 * and changes the Message Buffer state to start CAN Message transmit. After
 * that the function returns immediately.
 *
 * @param base FlexCAN peripheral base address.
 * @param mbIdx The FlexCAN Message Buffer index.
 * @param pTxFrame Pointer to CAN message frame to be sent.
 * @retval kStatus_Success - Write Tx Message Buffer Successfully.
 * @retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t FLEXCAN_WriteTxMb(CAN_Type *base, uint8_t mbIdx, const flexcan_frame_t *pTxFrame);

/*!
 * @brief Reads a FlexCAN Message from Receive Message Buffer.
 *
 * This function reads a CAN message from a specified Receive Message Buffer.
 * The function fills a receive CAN message frame structure with
 * just received data and activates the Message Buffer again.
 * The function returns immediately.
 *
 * @param base FlexCAN peripheral base address.
 * @param mbIdx The FlexCAN Message Buffer index.
 * @param pRxFrame Pointer to CAN message frame structure for reception.
 * @retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * @retval kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 * @retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t FLEXCAN_ReadRxMb(CAN_Type *base, uint8_t mbIdx, flexcan_frame_t *pRxFrame);

/*!
 * @brief Reads a FlexCAN Message from Legacy Rx FIFO.
 *
 * This function reads a CAN message from the FlexCAN Legacy Rx FIFO.
 *
 * @param base FlexCAN peripheral base address.
 * @param pRxFrame Pointer to CAN message frame structure for reception.
 * @retval kStatus_Success - Read Message from Rx FIFO successfully.
 * @retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t FLEXCAN_ReadRxFifo(CAN_Type *base, flexcan_frame_t *pRxFrame);
/* @} */

/*!
 * @name Transactional
 * @{
 */

/*!
 * @brief Performs a polling send transaction on the CAN bus.
 *
 * @note  A transfer handle does not need to be created  before calling this API.
 *
 * @param base FlexCAN peripheral base pointer.
 * @param mbIdx The FlexCAN Message Buffer index.
 * @param pTxFrame Pointer to CAN message frame to be sent.
 * @retval kStatus_Success - Write Tx Message Buffer Successfully.
 * @retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t FLEXCAN_TransferSendBlocking(CAN_Type *base, uint8_t mbIdx, flexcan_frame_t *pTxFrame);

/*!
 * @brief Performs a polling receive transaction on the CAN bus.
 *
 * @note  A transfer handle does not need to be created  before calling this API.
 *
 * @param base FlexCAN peripheral base pointer.
 * @param mbIdx The FlexCAN Message Buffer index.
 * @param pRxFrame Pointer to CAN message frame structure for reception.
 * @retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * @retval kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 * @retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t FLEXCAN_TransferReceiveBlocking(CAN_Type *base, uint8_t mbIdx, flexcan_frame_t *pRxFrame);

/*!
 * @brief Performs a polling receive transaction from Legacy Rx FIFO on the CAN bus.
 *
 * @note  A transfer handle does not need to be created before calling this API.
 *
 * @param base FlexCAN peripheral base pointer.
 * @param pRxFrame Pointer to CAN message frame structure for reception.
 * @retval kStatus_Success - Read Message from Rx FIFO successfully.
 * @retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t FLEXCAN_TransferReceiveFifoBlocking(CAN_Type *base, flexcan_frame_t *pRxFrame);

/*!
 * @brief Initializes the FlexCAN handle.
 *
 * This function initializes the FlexCAN handle, which can be used for other FlexCAN
 * transactional APIs. Usually, for a specified FlexCAN instance,
 * call this API once to get the initialized handle.
 *
 * @param base FlexCAN peripheral base address.
 * @param handle FlexCAN handle pointer.
 * @param callback The callback function.
 * @param userData The parameter of the callback function.
 */
void FLEXCAN_TransferCreateHandle(CAN_Type *base,
                                  flexcan_handle_t *handle,
                                  flexcan_transfer_callback_t callback,
                                  void *userData);

/*!
 * @brief Sends a message using IRQ.
 *
 * This function sends a message using IRQ. This is a non-blocking function, which returns
 * right away. When messages have been sent out, the send callback function is called.
 *
 * @param base FlexCAN peripheral base address.
 * @param handle FlexCAN handle pointer.
 * @param pMbXfer FlexCAN Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
 * @retval kStatus_Success        Start Tx Message Buffer sending process successfully.
 * @retval kStatus_Fail           Write Tx Message Buffer failed.
 * @retval kStatus_FLEXCAN_TxBusy Tx Message Buffer is in use.
 */
status_t FLEXCAN_TransferSendNonBlocking(CAN_Type *base, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer);

/*!
 * @brief Receives a message using IRQ.
 *
 * This function receives a message using IRQ. This is non-blocking function, which returns
 * right away. When the message has been received, the receive callback function is called.
 *
 * @param base FlexCAN peripheral base address.
 * @param handle FlexCAN handle pointer.
 * @param pMbXfer FlexCAN Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
 * @retval kStatus_Success        - Start Rx Message Buffer receiving process successfully.
 * @retval kStatus_FLEXCAN_RxBusy - Rx Message Buffer is in use.
 */
status_t FLEXCAN_TransferReceiveNonBlocking(CAN_Type *base, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer);

/*!
 * @brief Receives a message from Rx FIFO using IRQ.
 *
 * This function receives a message using IRQ. This is a non-blocking function, which returns
 * right away. When all messages have been received, the receive callback function is called.
 *
 * @param base FlexCAN peripheral base address.
 * @param handle FlexCAN handle pointer.
 * @param pFifoXfer FlexCAN Rx FIFO transfer structure. See the @ref flexcan_fifo_transfer_t.
 * @retval kStatus_Success            - Start Rx FIFO receiving process successfully.
 * @retval kStatus_FLEXCAN_RxFifoBusy - Rx FIFO is currently in use.
 */
status_t FLEXCAN_TransferReceiveFifoNonBlocking(CAN_Type *base,
                                                flexcan_handle_t *handle,
                                                flexcan_fifo_transfer_t *pFifoXfer);

/*!
 * @brief Gets the detail index of Mailbox's Timestamp by handle.
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
 * @param handle FlexCAN handle pointer.
 * @param mbIdx The FlexCAN Message Buffer index.
 * @retval the index of mailbox 's timestamp stored in the handle.
 *
 */
uint32_t FLEXCAN_GetTimeStamp(flexcan_handle_t *handle, uint8_t mbIdx);

/*!
 * @brief Aborts the interrupt driven message send process.
 *
 * This function aborts the interrupt driven message send process.
 *
 * @param base FlexCAN peripheral base address.
 * @param handle FlexCAN handle pointer.
 * @param mbIdx The FlexCAN Message Buffer index.
 */
void FLEXCAN_TransferAbortSend(CAN_Type *base, flexcan_handle_t *handle, uint8_t mbIdx);

/*!
 * @brief Aborts the interrupt driven message receive process.
 *
 * This function aborts the interrupt driven message receive process.
 *
 * @param base FlexCAN peripheral base address.
 * @param handle FlexCAN handle pointer.
 * @param mbIdx The FlexCAN Message Buffer index.
 */
void FLEXCAN_TransferAbortReceive(CAN_Type *base, flexcan_handle_t *handle, uint8_t mbIdx);

/*!
 * @brief Aborts the interrupt driven message receive from Rx FIFO process.
 *
 * This function aborts the interrupt driven message receive from Rx FIFO process.
 *
 * @param base FlexCAN peripheral base address.
 * @param handle FlexCAN handle pointer.
 */
void FLEXCAN_TransferAbortReceiveFifo(CAN_Type *base, flexcan_handle_t *handle);

/*!
 * @brief FlexCAN IRQ handle function.
 *
 * This function handles the FlexCAN Error, the Message Buffer, and the Rx FIFO IRQ request.
 *
 * @param base FlexCAN peripheral base address.
 * @param handle FlexCAN handle pointer.
 */
void FLEXCAN_TransferHandleIRQ(CAN_Type *base, flexcan_handle_t *handle);

/* @} */
/*! @}*/

#endif /* _FSL_FLEXCAN_H_ */
