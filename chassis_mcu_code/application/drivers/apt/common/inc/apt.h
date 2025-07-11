/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file    apt.h
  * @author  MCU Driver Team
  * @brief   APT module driver.
  * @details This file provides functions declaration of the APT module.
  *           + APT handle structure definition.
  *           + Initialization and de-initialization functions.
  *           + APT Service Functions.
  */

#ifndef McuMagicTag_APT_H
#define McuMagicTag_APT_H

#include "apt_ip.h"

#define EM_OUT_EVT_FILTER_EN    0x0f
#define EM_CMB_MODE_OFFSET      16
#define EM_CMB_MODE_INTERVAL    4
#define EM_CMB_SRC_SEL_INTERVAL 4
#define EM_OR_INTERVAL          16
#define EM_CMB_EVT_NUM          4
#define EM_COMBINE_A1_SRC_ENABLE_ALL 0xF
/**
  * @defgroup APT APT
  * @brief APT module.
  * @{
  */


/**
  * @defgroup APT_Common APT Common
  * @brief APT common external module.
  * @{
  */

/**
  * @defgroup APT_Handle_Definition APT Handle Definition
  * @{
  */

/*
          Basic type     AHBL         ALBH         AHBH         ALBL
                         ___       __     __       ___       __     __
          ChannelA    __|   |__      |___|      __|   |__      |___|
                      __     __       ___          ___       __     __
          ChannelB      |___|      __|   |__    __|   |__      |___|
*/
/**
  * @brief Basic PWM waveform type.
  * @details waveform type:
  *             + APT_PWM_BASIC_A_HIGH_B_LOW -- Basic PWM waveform type 1.
  *             + APT_PWM_BASIC_A_LOW_B_HIGH -- Basic PWM waveform type 2.
  *             + APT_PWM_BASIC_A_HIGH_B_HIGH -- Basic PWM waveform type 3.
  *             + APT_PWM_BASIC_A_LOW_B_LOW -- Basic PWM waveform type 4.
  */
typedef enum {
    APT_PWM_BASIC_A_HIGH_B_LOW = 0x00000000U,
    APT_PWM_BASIC_A_LOW_B_HIGH = 0x00000001U,
    APT_PWM_BASIC_A_HIGH_B_HIGH = 0x00000002U,
    APT_PWM_BASIC_A_LOW_B_LOW = 0x00000003U,
} APT_PWMBasicType;

/**
  * @brief The actual outputs of PWM channelA and channelB.
  * @details Output:
  *             + APT_PWM_OUT_BASIC_TYPE = 0x00000000U -- PWM channel output the waveform according to basic PWM type.
  *             + APT_PWM_OUT_ALWAYS_LOW = 0x00000001U -- PWM channel output low level.
  *             + APT_PWM_OUT_ALWAYS_HIGH = 0x00000002U -- PWM channel output high level.
  */
typedef enum {
    APT_PWM_OUT_BASIC_TYPE = 0x00000000U,
    APT_PWM_OUT_ALWAYS_LOW = 0x00000001U,
    APT_PWM_OUT_ALWAYS_HIGH = 0x00000002U,
} APT_PWMChannelOutType;

/**
  * @brief PWM waveform configuration handle of APT module.
  */
typedef struct {
    APT_PWMBasicType        basicType;        /**< Basic PWM waveform type. */
    APT_PWMChannelOutType   chAOutType;       /**< Actual output of PWM channelA. */
    APT_PWMChannelOutType   chBOutType;       /**< Actual output of PWM channelB. */
    APT_CountMode           cntMode;          /**< Count mode of APT time-base counter. */
    unsigned short          dividerFactor;    /**< Divider factor. The range is 0~4095. */
    unsigned short          timerPeriod;      /**< Count period of APT time-base timer. */
    unsigned short          divInitVal;       /**< Initial value of divider. */
    unsigned short          cntInitVal;       /**< Initial value of time-base counter */
    unsigned short          cntCmpLeftEdge;   /**< Count compare point of the left edge of PWM waveform. */
    unsigned short          cntCmpRightEdge;  /**< Count compare point of the right edge of PWM waveform. */
    APT_BufferLoadMode      cntCmpLoadMode;   /**< Buffer load mode of PWM waveform count compare value. */
    unsigned int            cntCmpLoadEvt;    /**< Buffer load event of PWM waveform count compare value. */
    unsigned short          deadBandCnt;      /**< Count value of dead-band counter. In units of APT clock. */
} APT_PWMWaveForm;

/**
  * @brief ADC trigger configuration handle of APT module.
  */
typedef struct {
    bool                    trgEnSOCA;        /**< Enable of ADC trigger source SOCA. */
    APT_ADCTriggerSource    trgSrcSOCA;       /**< Source of ADC trigger source SOCA. */
    unsigned short          trgScaleSOCA;     /**< Scale of ADC trigger source SOCA. */
    unsigned short          cntCmpSOCA;       /**< Count compare point of ADC trigger source SOCA when using CMPA */
    bool                    trgEnSOCB;        /**< Enable of ADC trigger source SOCB. */
    APT_ADCTriggerSource    trgSrcSOCB;       /**< Source of ADC trigger source SOCB. */
    unsigned short          trgScaleSOCB;     /**< Scale of ADC trigger source SOCB. */
    unsigned short          cntCmpSOCB;       /**< Count compare point of ADC trigger source SOCB when using CMPB */
    APT_BufferLoadMode      cntCmpLoadMode;   /**< Buffer load mode of ADC trigger count compare value. */
    unsigned int          cntCmpLoadEvt;    /**< Buffer load event of ADC trigger count compare value. */
} APT_ADCTrigger;

/**
  * @brief Timer interrupt configuration handle of APT module.
  */
typedef struct {
    bool                    tmrInterruptEn;     /**< Enable of APT module timer interrupt. */
    APT_TimerInterruptSrc   tmrInterruptSrc;    /**< Source of APT module timer interrupt. */
    unsigned short          tmrInterruptScale;  /**< Scale of APT module timer interrupt. */
} APT_TimerInterrupt;

/**
  * @brief Output control protection configuration handle of APT module.
  */
typedef struct {
    bool                    ocEventEn;        /**< Enable of output control event. */
    APT_OutCtrlEvent        ocEvent;          /**< Output control event. Limited to IO events or system events. */
    APT_OutCtrlMode         ocEventMode;      /**< Output control protection mode. */
    APT_CBCClearMode        cbcClrMode;       /**< Event clear mode when using cycle-by-cycle mode. */
    APT_EMEventPolarity     evtPolarity;      /**< Event effective polarity. */
    APT_OutCtrlAction       ocAction;         /**< Output control protection action. */
    APT_EmulationMode       emMode;           /**< emulation mode */
    bool                    ocEvtInterruptEn; /**< Enable of output control event interrupt. */
} APT_OutCtrlProtect;

/**
  * @brief Source event of event magnagement.
  */
typedef enum {
    APT_EM_ORIGINAL_SRC_POE0             = 0x00000001U,
    APT_EM_ORIGINAL_SRC_POE1             = 0x00000002U,
    APT_EM_ORIGINAL_SRC_POE2             = 0x00000004U,
    APT_EM_ORIGINAL_SRC_ACMP0            = 0x00000008U,
    APT_EM_ORIGINAL_SRC_ACMP1            = 0x00000010U,
    APT_EM_ORIGINAL_SRC_ACMP2            = 0x00000020U,
    APT_EM_ORIGINAL_SRC_EVTMP4           = 0x00000040U,
    APT_EM_ORIGINAL_SRC_EVTMP5           = 0x00000080U,
    APT_EM_ORIGINAL_SRC_EVTMP6           = 0x00000100U,
} APT_EMOriginalEvtSrc;

/**
  * @brief Filter mask bit.
  */
typedef enum {
    APT_EM_POE0_INVERT_BIT             = 0x00000001U,
    APT_EM_POE1_INVERT_BIT             = 0x00000002U,
    APT_EM_POE2_INVERT_BIT             = 0x00000004U,
    APT_EM_ACMP0_INVERT_BIT            = 0x00000008U,
    APT_EM_ACMP1_INVERT_BIT            = 0x00000010U,
    APT_EM_ACMP2_INVERT_BIT            = 0x00000020U,
    APT_EM_EVTMP4_INVERT_BIT           = 0x00000040U,
    APT_EM_EVTMP5_INVERT_BIT           = 0x00000080U,
    APT_EM_EVTMP6_INVERT_BIT           = 0x00000100U,
} APT_EMPolarityMskBit;

/**
  * @brief System protect event;
  */
typedef enum {
    APT_SYS_EVT_DEBUG               = 0x00000010U,
    APT_SYS_EVT_CLK                 = 0x00000020U,
    APT_SYS_EVT_MEM                 = 0x00000040U,
} APT_SysOcEvent;

/**
  * @brief Output control protection configuration handle of APT module.
  */
typedef struct {
    bool                    ocEventEnEx;        /**< oc event enable */
    APT_OutCtrlMode         ocEventModeEx;      /**< Output control protection mode. */
    APT_CBCClearMode        cbcClrModeEx;       /**< Event clear mode when using cycle-by-cycle mode. */
    APT_OutCtrlAction       ocActionEx;         /**< Output control protection channel A action. */
    APT_OutCtrlAction       ocActionBEx;        /**< Output control protection channel B action. */
    bool                    ocEvtInterruptEnEx; /**< Enable of output control event interrupt. */
    APT_SysOcEvent          ocSysEvent;			/**< System protect event */
    APT_EMOriginalEvtSrc    originalEvtEx;      /**< Event management's event source */
    APT_EMPolarityMskBit    evtPolarityMaskEx;  /**< Event effective polarity. */
    unsigned char           filterCycleNumEx;   /**< input source event filter */
} APT_OutCtrlProtectEx;

/**
  * @brief struct of EM conbine event
  */
typedef struct {
    APT_EMCombineEvtSrc     emEvtSrc;         /**< combine event selection */
    APT_EMCombineEvtMode    emEvtCombineMode; /**< event combine mode */
    APT_EMEventPolarity     emEvtPolar;       /**< event source polarity */
    unsigned int            emEvtOrEnBits;    /**< event logic or enable bits */
} APT_CombineEvt;

/**
 * @brief Shield window and capture configurations
 */
typedef struct {
    bool                    wdEnable;         /**< Shield windows enable bit */
    bool                    emCapEnable;      /**< Enable EM captrue functions */
    APT_EMCombineEvent      eventSel;         /**< Window source event selection */
    APT_MaskWinResetMode    wdStartAndCapClr; /**< Window's offset start count and EM capture clear condition */
    unsigned short          wdOffset;         /**< Window's offset value */
    unsigned short          wdWidth;          /**< Window's width value */
    APT_MaskWinPolarity     wdPolar;          /**< Window's polarity */
} APT_WdAndCap;


/**
 * @brief Valley switch configurations
 */
typedef struct {
    bool                        vsEnable;       /**< Valley switch enable */
    APT_EMEdgeFilterMode        vsFilerEdgeSel; /**< Filter edge selection */
    unsigned char               vsFilterCnt;    /**< Filter edge count */
    APT_ValleyCapRstType        vsClrType;      /**< Clear type */
    APT_ValleyCountEdge         vsCapEdgeSel;   /**< Capture edge selection */
    unsigned char               vsCapStartEdge; /**< Capture start edge */
    unsigned char               vsCapEndEdge;   /**< Capture end edge */
    APT_ValleyDelayMode         vsCapDelayMode; /**< Capture delay mode */
    unsigned short              vsCapSoftDelay; /**< Capture software calibrate value */
} APT_ValleySw;

/**
  * @brief Event management handle of APT module
  */
typedef struct {
    bool                    emEnable;               /**< Enable bit of event management */
    APT_CombineEvt          emEvt[EM_CMB_EVT_NUM];  /**< Combine events configuration */
    APT_WdAndCap            emWdAndCap;             /**< Shield windows and capture configuration */
    APT_ValleySw            emValleySw;             /**< Valley switch configuration */
} APT_EventManage;

/**
  * @brief Synchronization handle of slave APT module.
  */
typedef struct {
    unsigned short          divPhase;         /**< Divider phase when receiving APT synchronization pulse. */
    unsigned short          cntPhase;         /**< Counter phase when receiving APT synchronization pulse. */
    APT_SyncCountMode       syncCntMode;      /**< Count mode when receiving APT synchronization pulse. */
    APT_SyncInSrc           syncInSrc;        /**< Sync-in source of APT module */
    unsigned short          cntrSyncSrc;
    /**< Sync-in source of time-base counter synchronization
         A logical OR of valid values can be passed as the cntrSyncSrc parameter.
         Valid values for cntrSyncSrc are:
              APT_CNTR_SYNC_SRC_COMBINE_EVENT_A1  - Enable combine event A1 as the counter synchronization source.
              APT_CNTR_SYNC_SRC_COMBINE_EVENT_B1  - Enable combine event B1 as the counter synchronization source.
              APT_CNTR_SYNC_SRC_SYNCIN            - Enable Sync-In source as the counter synchronization source. */
} APT_SlaveSyncIn;

/**
  * @brief Definition of callback function type.
  */
typedef void (* APT_CallbackType)(void *aptHandle);

/**
  * @brief Definition of callback function type.
  */
typedef struct {
    void (* EvtInterruptCallBack)(void *handle);
    void (* TmrInterruptCallBack)(void *handle);
} APT_UserCallBack;

/**
  * @brief The definition of the APT handle structure.
  */
typedef struct _APT_Handle {
    APT_RegStruct          *baseAddress;      /**< Register base address. */
    APT_PWMWaveForm         waveform;         /**< PWM waveform configuration handle. */
    APT_ADCTrigger          adcTrg;           /**< ADC trigger configuration handle. */
    APT_TimerInterrupt      tmrInterrupt;     /**< Timer interrupt configuration handle. */
    APT_UserCallBack        userCallBack;     /**< Interrupt callback function when APT event happens. */
    APT_ExtendHandle            handleEx;         /**< extra handle */
} APT_Handle;
/**
  * @}
  */

/**
  * @defgroup APT_API_Declaration APT HAL API
  * @{
  */

/**
  * @brief Definition of callback function ID.
  */
typedef enum {
    APT_TIMER_INTERRUPT = 0x00000000U,
    APT_EVENT_INTERRUPT = 0x00000001U,
} APT_InterruputType;

BASE_StatusType HAL_APT_PWMInit(APT_Handle *aptHandle);
BASE_StatusType HAL_APT_PWMDeInit(APT_Handle *aptHandle);
BASE_StatusType HAL_APT_ProtectInit(APT_Handle *aptHandle, APT_OutCtrlProtect *protect);
BASE_StatusType HAL_APT_ProtectDeInit(APT_Handle *aptHandle, APT_OutCtrlProtect *protect);
BASE_StatusType HAL_APT_ProtectInitEx(APT_Handle *aptHandle, APT_OutCtrlProtectEx *protect);
BASE_StatusType HAL_APT_ProtectDeInitEx(APT_Handle *aptHandle, APT_OutCtrlProtectEx *protect);
void HAL_APT_ForcePWMOutputLow(APT_Handle *aptHandle);
BASE_StatusType HAL_APT_MasterSyncInit(APT_Handle *aptHandle, unsigned short syncOutSrc);
BASE_StatusType HAL_APT_SlaveSyncInit(APT_Handle *aptHandle, APT_SlaveSyncIn *slaveSyncIn);
void HAL_APT_StartModule(unsigned int aptRunMask);
void HAL_APT_StopModule(unsigned int aptRunMask);
BASE_StatusType HAL_APT_SetPWMDuty(APT_Handle *aptHandle, unsigned short cntCmpLeftEdge, \
                                   unsigned short cntCmpRightEdge);
BASE_StatusType HAL_APT_SetPWMDutyByNumber(APT_Handle *aptHandle, unsigned int duty);
BASE_StatusType HAL_APT_SetADCTriggerTime(APT_Handle *aptHandle, unsigned short cntCmpSOCA, unsigned short cntCmpSOCB);
void HAL_APT_EventIrqHandler(void *handle);
void HAL_APT_TimerIrqHandler(void *handle);
void HAL_APT_RegisterCallBack(APT_Handle *aptHandle, APT_InterruputType typeID, APT_CallbackType pCallback);
BASE_StatusType HAL_APT_EMInit(APT_Handle *aptHandle, APT_EventManage *eventManage);
unsigned short HAL_APT_EMGetCapValue(APT_Handle *aptHandle);
void HAL_APT_EMSetWdOffsetAndWidth(APT_Handle *aptHandle, unsigned short offset, unsigned short width);
void HAL_APT_EMSetValleySwithSoftDelay(APT_Handle *aptHandle, unsigned short calibrate);
BASE_StatusType HAL_APT_ChangeOutputType(APT_Handle *aptHandle,
                                         APT_PWMChannel channel,
                                         APT_PWMChannelOutType aptAction);

/* Attribute configuration of each reference point. */
BASE_StatusType APT_ConfigRefA(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters);
BASE_StatusType APT_ConfigRefB(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters);
BASE_StatusType APT_ConfigRefC(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters);
BASE_StatusType APT_ConfigRefD(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters);
/* Combination configuration of reference point attributes. */
BASE_StatusType HAL_APT_ConfigRefDot(APT_Handle *aptHandle, APT_RefDotSelect refDotSelect,
                                     APT_RefDotParameters *refDotParameters);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_APT_H */