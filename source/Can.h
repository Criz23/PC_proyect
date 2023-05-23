#include "fsl_common.h"

#define FLEXCAN_ID_STD(id) \
    (((uint32_t)(((uint32_t)(id)) << CAN_ID_STD_SHIFT)) & CAN_ID_STD_MASK) /*!< Standard Frame ID helper macro. */

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

enum _flexcan_mb_code_tx
{
    kFLEXCAN_TxMbInactive     = 0x8, /*!< MB is not active.*/
    kFLEXCAN_TxMbAbort        = 0x9, /*!< MB is aborted.*/
    kFLEXCAN_TxMbDataOrRemote = 0xC, /*!< MB is a TX Data Frame(when MB RTR = 0) or MB is a TX Remote Request
                                          Frame (when MB RTR = 1).*/
    kFLEXCAN_TxMbTanswer = 0xE,      /*!< MB is a TX Response Request Frame from an incoming Remote Request Frame.*/
    kFLEXCAN_TxMbNotUsed = 0xF,      /*!< Not used.*/
};

typedef enum _flexcan_frame_format
{
    kFLEXCAN_FrameFormatStandard = 0x0U, /*!< Standard frame format attribute. */
    kFLEXCAN_FrameFormatExtend   = 0x1U, /*!< Extend frame format attribute. */
} flexcan_frame_format_t;

typedef enum _flexcan_frame_type
{
    kFLEXCAN_FrameTypeData   = 0x0U, /*!< Data frame type attribute. */
    kFLEXCAN_FrameTypeRemote = 0x1U, /*!< Remote frame type attribute. */
} flexcan_frame_type_t;

typedef enum _flexcan_clock_source
{
    kFLEXCAN_ClkSrcOsc  = 0x0U, /*!< FlexCAN Protocol Engine clock from Oscillator. */
    kFLEXCAN_ClkSrcPeri = 0x1U, /*!< FlexCAN Protocol Engine clock from Peripheral Clock. */
    kFLEXCAN_ClkSrc0    = 0x0U, /*!< FlexCAN Protocol Engine clock selected by user as SRC == 0. */
    kFLEXCAN_ClkSrc1    = 0x1U, /*!< FlexCAN Protocol Engine clock selected by user as SRC == 1. */
} flexcan_clock_source_t;

typedef enum _flexcan_wake_up_source
{
    kFLEXCAN_WakeupSrcUnfiltered = 0x0U, /*!< FlexCAN uses unfiltered Rx input to detect edge. */
    kFLEXCAN_WakeupSrcFiltered   = 0x1U, /*!< FlexCAN uses filtered Rx input to detect edge. */
} flexcan_wake_up_source_t;

typedef struct _flexcan_timing_config
{
    uint16_t preDivider; /*!< Classic CAN or CAN FD nominal phase bit rate prescaler. */
    uint8_t rJumpwidth;  /*!< Classic CAN or CAN FD nominal phase Re-sync Jump Width. */
    uint8_t phaseSeg1;   /*!< Classic CAN or CAN FD nominal phase Segment 1. */
    uint8_t phaseSeg2;   /*!< Classic CAN or CAN FD nominal phase Segment 2. */
    uint8_t propSeg;     /*!< Classic CAN or CAN FD nominal phase Propagation Segment. */

} flexcan_timing_config_t;

typedef struct _flexcan_handle flexcan_handle_t;

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

typedef struct _can_config
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

typedef struct _flexcan_rx_mb_config
{
	flexcan_frame_type_t type;     /*!< CAN Frame Type(Data or Remote). */
    uint32_t id;                   /*!< CAN Message Buffer Frame Identifier, should be set using
                                        FLEXCAN_ID_EXT() or FLEXCAN_ID_STD() macro. */
    flexcan_frame_format_t format; /*!< CAN Frame Identifier format(Standard of Extend). */
    flexcan_frame_t kFrame;

} Can_PduType;

typedef void (*flexcan_transfer_callback_t)(
	CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData);

void Javier_SetTimingConfig(CAN_Type *base, const flexcan_timing_config_t *pConfig);

void Javier_SetRxMbConfig(CAN_Type *base, uint8_t mbIdx, const Can_PduType *pRxMbConfig, bool enable);

static inline void Javier_Enable(CAN_Type *base, bool enable);

static inline void Javier_ClearMbStatusFlags(CAN_Type *base, uint32_t mask);

static inline uint32_t Javier_GetMbStatusFlags(CAN_Type *base, uint32_t mask)
{
    return (base->IFLAG1 & mask);
}

void CAN_Init(const Can_ConfigType *pConfig);

extern Can_ConfigType flexcanConfig;

extern Can_ConfigType flexcanConfig;

extern flexcan_frame_t rxFrame, txFrame;

extern Can_PduType mbConfig;

extern Std_ReturnType Can_WriteTxMb(uint8_t mbIdx, const Can_PduType *pTxFrame);

extern void Can_MainFunction_Read();

