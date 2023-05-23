#include "Can.h"

#define DLC                        	(8)
#define RX_MESSAGE_BUFFER_NUM_      (9)
#define TX_MESSAGE_BUFFER_NUM_      (8)
#define EXAMPLE_CAN_            	CAN0
#define EXAMPLE_CAN_CLK_FREQ   		CLOCK_GetFreq(kCLOCK_BusClk)
#define MAX_PRESDIV           		(CAN_CTRL1_PRESDIV_MASK >> CAN_CTRL1_PRESDIV_SHIFT)
#define MIN_TIME_SEGMENT1 			(2U)
#define MIN_TIME_SEGMENT2 			(2U)
#define CAN_CLOCK_CHECK_NO_AFFECTS 	(true)


static CAN_Type *const s_flexcanBases[] = CAN_BASE_PTRS;

static const clock_ip_name_t s_flexcanClock[] = FLEXCAN_CLOCKS;

uint32_t Javier_GetInstance(CAN_Type *base)
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

    return instance;
}

void Javier_EnterFreezeMode(CAN_Type *base)
{
    /* Set Freeze, Halt bits. */
    base->MCR |= CAN_MCR_FRZ_MASK;
    base->MCR |= CAN_MCR_HALT_MASK;
    while (0U == (base->MCR & CAN_MCR_FRZACK_MASK))
    {
    }
}

void Javier_ExitFreezeMode(CAN_Type *base)
{
    /* Clear Freeze, Halt bits. */
    base->MCR &= ~CAN_MCR_HALT_MASK;
    base->MCR &= ~CAN_MCR_FRZ_MASK;

    /* Wait until the FlexCAN Module exit freeze mode. */
    while (0U != (base->MCR & CAN_MCR_FRZACK_MASK))
    {
    }
}

static bool Javier_IsMbOccupied(CAN_Type *base, uint8_t mbIdx)
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

static void Javier_Reset(CAN_Type *base)
{
    /* The module must should be first exit from low power
     * mode, and then soft reset can be applied.
     */

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
    base->MCR |= CAN_MCR_WRNEN_MASK | CAN_MCR_WAKSRC_MASK |
                 CAN_MCR_MAXMB((uint32_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base) - 1U);
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

static void Javier_SetBitRate(CAN_Type *base,
                               uint32_t sourceClock_Hz,
                               uint32_t bitRate_Bps,
                               flexcan_timing_config_t timingConfig)
{
    /* FlexCAN classical CAN frame or CAN FD frame nominal phase timing setting formula:
     * quantum = 1 + (phaseSeg1 + 1) + (phaseSeg2 + 1) + (propSeg + 1);
     */
    uint32_t quantum = (1U + ((uint32_t)timingConfig.phaseSeg1 + 1U) + ((uint32_t)timingConfig.phaseSeg2 + 1U) +
                        ((uint32_t)timingConfig.propSeg + 1U));

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
    Javier_SetTimingConfig(base, (const flexcan_timing_config_t *)(uint32_t)&timingConfig);
}

void CAN_Init(const Can_ConfigType *pConfig)
{
    uint32_t mcrTemp;
    uint32_t ctrl1Temp;
    uint32_t instance;
    instance = Javier_GetInstance(CAN0);
    /* Enable FlexCAN clock. */
    (void)CLOCK_EnableClock(s_flexcanClock[instance]);
    /*
     * Check the CAN clock in this device whether affected by Other clock gate
     * If it affected, we'd better to change other clock source,
     * If user insist on using that clock source, user need open these gate at same time,
     * In this scene, User need to care the power consumption.
     */
    assert(CAN_CLOCK_CHECK_NO_AFFECTS);
    {
        /* Disable FlexCAN Module. */
    	Javier_Enable(CAN0, false);

        /* Protocol-Engine clock source selection, This bit must be set
         * when FlexCAN Module in Disable Mode.
         */
        CAN0->CTRL1 = (kFLEXCAN_ClkSrc0 == pConfig->clkSrc) ? (CAN0->CTRL1 & ~CAN_CTRL1_CLKSRC_MASK) :
                                                              (CAN0->CTRL1 | CAN_CTRL1_CLKSRC_MASK);
    }

    /* Enable FlexCAN Module for configuration. */
    Javier_Enable(CAN0, true);

    /* Reset to known status. */
    Javier_Reset(CAN0);
    /* Save current CTRL1 value and enable to enter Freeze mode(enabled by default). */
    ctrl1Temp = CAN0->CTRL1;

    /* Save current MCR value and enable to enter Freeze mode(enabled by default). */
    mcrTemp = CAN0->MCR;

    /* Enable Loop Back Mode? */
    ctrl1Temp = (pConfig->enableLoopBack) ? (ctrl1Temp | CAN_CTRL1_LPB_MASK) : (ctrl1Temp & ~CAN_CTRL1_LPB_MASK);

    /* Enable Timer Sync? */
    ctrl1Temp = (pConfig->enableTimerSync) ? (ctrl1Temp | CAN_CTRL1_TSYN_MASK) : (ctrl1Temp & ~CAN_CTRL1_TSYN_MASK);

    /* Enable Listen Only Mode? */
    ctrl1Temp = (pConfig->enableListenOnlyMode) ? ctrl1Temp | CAN_CTRL1_LOM_MASK : ctrl1Temp & ~CAN_CTRL1_LOM_MASK;

    /* Enable Supervisor Mode? */
    mcrTemp = (pConfig->enableSupervisorMode) ? mcrTemp | CAN_MCR_SUPV_MASK : mcrTemp & ~CAN_MCR_SUPV_MASK;

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
    CAN0->CTRL1 = ctrl1Temp;

    /* Write back MCR Configuration to register. */
    CAN0->MCR = mcrTemp;

    /* Bit Rate Configuration.*/
    Javier_SetBitRate(CAN0, EXAMPLE_CAN_CLK_FREQ, pConfig->bitRate, pConfig->timingConfig);
}

void Javier_SetTimingConfig(CAN_Type *base, const flexcan_timing_config_t *pConfig)
{
    /* Enter Freeze Mode. */
    Javier_EnterFreezeMode(base);
    /* Cleaning previous Timing Setting. */
    base->CTRL1 &= ~(CAN_CTRL1_PRESDIV_MASK | CAN_CTRL1_RJW_MASK | CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PSEG2_MASK |
                     CAN_CTRL1_PROPSEG_MASK);

    /* Updating Timing Setting according to configuration structure. */
    base->CTRL1 |= (CAN_CTRL1_PRESDIV(pConfig->preDivider) | CAN_CTRL1_RJW(pConfig->rJumpwidth) |
                    CAN_CTRL1_PSEG1(pConfig->phaseSeg1) | CAN_CTRL1_PSEG2(pConfig->phaseSeg2) |
                    CAN_CTRL1_PROPSEG(pConfig->propSeg));

    /* Exit Freeze Mode. */
    Javier_ExitFreezeMode(base);
}

void Javier_SetTxMbConfig(CAN_Type *base, uint8_t mbIdx, bool enable)
{

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

void Javier_SetRxMbConfig(CAN_Type *base, uint8_t mbIdx, const Can_PduType *pRxMbConfig, bool enable)
{

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

Std_ReturnType Can_WriteTxMb(Can_HwHandleType Hth, const Can_PduType *PduInfo)
{
    uint32_t cs_temp = 0;
    Std_ReturnType status;

    Javier_SetRxMbConfig(EXAMPLE_CAN_, RX_MESSAGE_BUFFER_NUM_, &PduInfo, true);
    /* Setup Tx Message Buffer. */
    Javier_SetTxMbConfig(EXAMPLE_CAN_, TX_MESSAGE_BUFFER_NUM_, true);
    /* Check if Message Buffer is available. */
    if (CAN_CS_CODE(kFLEXCAN_TxMbDataOrRemote) != (EXAMPLE_CAN_->MB[Hth].CS & CAN_CS_CODE_MASK))
    {
        /* Inactive Tx Message Buffer. */
    	EXAMPLE_CAN_->MB[Hth].CS = (EXAMPLE_CAN_->MB[Hth].CS & ~CAN_CS_CODE_MASK) | CAN_CS_CODE(kFLEXCAN_TxMbInactive);

        /* Fill Message ID field. */
    	EXAMPLE_CAN_->MB[Hth].ID = PduInfo->kFrame.id;

        /* Fill Message Format field. */
        if ((uint32_t)kFLEXCAN_FrameFormatExtend == PduInfo->kFrame.format)
        {
            cs_temp |= CAN_CS_SRR_MASK | CAN_CS_IDE_MASK;
        }

        /* Fill Message Type field. */
        if ((uint32_t)kFLEXCAN_FrameTypeRemote == PduInfo->kFrame.type)
        {
            cs_temp |= CAN_CS_RTR_MASK;
        }

        cs_temp |= CAN_CS_CODE(kFLEXCAN_TxMbDataOrRemote) | CAN_CS_DLC(PduInfo->kFrame.length);

        /* Load Message Payload. */
        EXAMPLE_CAN_->MB[Hth].WORD0 = PduInfo->kFrame.dataWord0;
        EXAMPLE_CAN_->MB[Hth].WORD1 = PduInfo->kFrame.dataWord1;

        /* Activate Tx Message Buffer. */
        EXAMPLE_CAN_->MB[Hth].CS = cs_temp;
        status = kStatus_Success;
    }
    else
    {
        /* Tx Message Buffer is activated, return immediately. */
        status = kStatus_Fail;
    }

    return status;
}

void Can_MainFunction_Read()
{
/* Wait until CAN Message send out. */
        uint32_t u32flag = 1;
        while (0U == Javier_GetMbStatusFlags(EXAMPLE_CAN_, u32flag << TX_MESSAGE_BUFFER_NUM_))
        {
        }

        Javier_ClearMbStatusFlags(EXAMPLE_CAN_, u32flag << TX_MESSAGE_BUFFER_NUM_);
}

static inline void Javier_Enable(CAN_Type *base, bool enable)
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

static inline void Javier_ClearMbStatusFlags(CAN_Type *base, uint32_t mask)
{
    base->IFLAG1 = mask;
}

Can_ConfigType flexcanConfig = {
		.clkSrc = kFLEXCAN_ClkSrc1,
		.enableLoopBack = true
};

Can_PduType mbConfig = {
	/* Setup Rx Message Buffer. */
	.format = kFLEXCAN_FrameFormatStandard,
	.type   = kFLEXCAN_FrameTypeData,
	.id     = FLEXCAN_ID_STD(0x123),
	.kFrame = {
			.format = (uint8_t)kFLEXCAN_FrameFormatStandard,
			.type   = (uint8_t)kFLEXCAN_FrameTypeData,
			.id     = FLEXCAN_ID_STD(0x123),
			.length = (uint8_t)DLC,
			.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x11) | CAN_WORD0_DATA_BYTE_1(0x22) |
						 CAN_WORD0_DATA_BYTE_2(0x33) | CAN_WORD0_DATA_BYTE_3(0x44),
			.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55) | CAN_WORD1_DATA_BYTE_5(0x66) |
						 CAN_WORD1_DATA_BYTE_6(0x77) | CAN_WORD1_DATA_BYTE_7(0x88),
		}
};
