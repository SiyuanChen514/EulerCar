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
  * @file    apt_ip.h
  * @author  MCU Driver Team
  * @brief   Header file containing APT module DCL driver functions.
  *          This file provides functions to manage the following functionalities of APT module.
  *          + Definition of APT configuration parameters.
  *          + APT registers mapping structure.
  *          + Direct Configuration Layer driver functions.
  */

#ifndef McuMagicTag_APT_IP_H
#define McuMagicTag_APT_IP_H

#include "baseinc.h"

#ifdef APT_PARAM_CHECK
    #define APT_ASSERT_PARAM            BASE_FUNC_ASSERT_PARAM
    #define APT_PARAM_CHECK_NO_RET      BASE_FUNC_PARAMCHECK_NO_RET
    #define APT_PARAM_CHECK_WITH_RET    BASE_FUNC_PARAMCHECK_WITH_RET
#else
    #define APT_ASSERT_PARAM(para)               ((void)0U)
    #define APT_PARAM_CHECK_NO_RET(para)         ((void)0U)
    #define APT_PARAM_CHECK_WITH_RET(param, ret) ((void)0U)
#endif

/**
  * @addtogroup APT
  * @{
  */

/**
  * @defgroup APT_IP APT_IP
  * @brief APT_IP: apt_v1.
  * @{
  */

/**
 * @defgroup APT_Param_Def APT Parameters Definition
 * @brief Definition of APT configuration parameters
 * @{
 */

/* Bitmask of the aptx_run bits in SYSCTRL1 register. */
#define RUN_APT0                        0x00000001U
#define RUN_APT1                        0x00000002U
#define RUN_APT2                        0x00000004U
#define RUN_APT3                        0x00000008U

/* Limited values for some configuration items of APT module. */
#define DIVIDER_FACTOR_MAX              0x00000FFFU
#define TIMEBASE_COUNTER_MAX            0x0000FFFFU
#define TIMER_INTERRUPT_CNT_MAX         0x0000000FU
#define ADC_CONVERSION_START_CNT_MAX    0x0000000FU
#define VCAP_STARY_STOP_EDGE_CNT_MAX    0x0000000FU
#define EDGE_FILTER_EDGE_CNT_MAX        0x0000000FU
#define CNTR_SYNC_SOURCE_MAX            0x00000007U
#define SYNC_OUT_SOURCE_MAX             0x000000FFU
#define GLOBAL_LOAD_CNT_MAX             0x0000000FU

/* Values that can be passed to DCL_APT_SetPeriodLoadEvent() as the loadEvent parameter. */
#define APT_PERIOD_LOAD_EVENT_ZERO      0x00000001U
#define APT_PERIOD_LOAD_EVENT_A1        0x00000004U
#define APT_PERIOD_LOAD_EVENT_B1        0x00000008U
#define APT_PERIOD_LOAD_EVENT_SYNC      0x00000010U

/* Values that can be passed to DCL_APT_SetCompareLoadEvent() as the loadEvent parameter. */
#define APT_COMPARE_LOAD_EVENT_ZERO     0x00000001U
#define APT_COMPARE_LOAD_EVENT_PERIOD   0x00000002U
#define APT_COMPARE_LOAD_EVENT_A1       0x00000004U
#define APT_COMPARE_LOAD_EVENT_B1       0x00000008U
#define APT_COMPARE_LOAD_EVENT_SYNC     0x00000010U

/* Values that can be returned by DCL_APT_GetCounterDirection(). */
#define APT_COUNTER_STATUS_COUNT_DOWN   0x00000000U
#define APT_COUNTER_STATUS_COUNT_UP     0x00000001U

/* Values that can be passed to DCL_APT_SetPWMActionLoadEvent() and
 * DCL_APT_SetSwContActionLoadEvent() as the loadEvent parameter. */
#define APT_ACTION_LOAD_EVENT_ZERO      0x00000001U
#define APT_ACTION_LOAD_EVENT_PERIOD    0x00000002U
#define APT_ACTION_LOAD_EVENT_A1        0x00000004U
#define APT_ACTION_LOAD_EVENT_B1        0x00000008U
#define APT_ACTION_LOAD_EVENT_SYNC      0x00000010U


/* Values that can be passed to DCL_APT_SetDGConfigLoadEvent(), DCL_APT_SetREDCounterLoadEvent() and
 * DCL_APT_SetFEDCounterLoadEvent() as the loadEvent parameter. */

#define APT_DEAD_BAND_LOAD_EVENT_ZERO   0x00000001U
#define APT_DEAD_BAND_LOAD_EVENT_PERIOD 0x00000002U

/* Values that can be passed to DCL_APT_SetEMEventOR() as the event1OREn and event1OREn parameter. */
#define APT_EM_OR_EN_GPIO_EVENT_1       0x00000001U
#define APT_EM_OR_EN_GPIO_EVENT_2       0x00000002U
#define APT_EM_OR_EN_GPIO_EVENT_3       0x00000004U
#define APT_EM_OR_EN_MXU_EVENT_1        0x00000008U
#define APT_EM_OR_EN_MXU_EVENT_2        0x00000010U
#define APT_EM_OR_EN_MXU_EVENT_3        0x00000020U
#define APT_EM_OR_EN_MXU_EVENT_4        0x00000040U
#define APT_EM_OR_EN_MXU_EVENT_5        0x00000080U
#define APT_EM_OR_EN_MXU_EVENT_6        0x00000100U
#define APT_EM_OR_EN_MXU_EVENT_7        0x00000200U
#define APT_EM_OR_EN_MXU_EVENT_8        0x00000400U
#define APT_EM_OR_EN_MXU_EVENT_9        0x00000800U
#define APT_EM_OR_EN_MXU_EVENT_10       0x00001000U
#define APT_EM_OR_EN_MXU_EVENT_11       0x00002000U
#define APT_EM_OR_EN_MXU_EVENT_12       0x00004000U

/* Values that can be passed to DCL_APT_SetTimeBaseCounterSyncSrc() as the cntrSyncSrc parameter. */
#define APT_CNTR_SYNC_SRC_COMBINE_EVENT_A1  0x00000001U
#define APT_CNTR_SYNC_SRC_COMBINE_EVENT_B1  0x00000002U
#define APT_CNTR_SYNC_SRC_SYNCIN            0x00000004U

/* Values that can be passed to DCL_APT_SetSyncOutPulseSrc() as the syncOutSrc parameter. */
#define APT_SYNC_OUT_ON_CNTR_ZERO        0x00000001U
#define APT_SYNC_OUT_ON_CNTR_PERIOD      0x00000002U
#define APT_SYNC_OUT_ON_COMBINE_EVENT_A1 0x00000004U
#define APT_SYNC_OUT_ON_COMBINE_EVENT_B1 0x00000008U
#define APT_SYNC_OUT_ON_CNTR_CMPB        0x00000020U
#define APT_SYNC_OUT_ON_CNTR_CMPC        0x00000040U
#define APT_SYNC_OUT_ON_CNTR_CMPD        0x00000080U

/* Values that can be passed to DCL_APT_SetGlobalLoadPrescale() as the glbLoadEvt parameter. */
#define APT_GLB_LOAD_ON_CNTR_ZERO        0x00000001U
#define APT_GLB_LOAD_ON_CNTR_PERIOD      0x00000002U
#define APT_GLB_LOAD_ON_CNTR_SYNC        0x00000004U

/**
  * @brief APT Extra Handle.
  */
typedef struct {
    ;
} APT_ExtendHandle;

/**
  * @brief Emulation stop mode of APT module.
  */
typedef enum {
    APT_EMULATION_NO_STOP               = 0x00000001U,
    APT_EMULATION_STOP_COUNTER          = 0x00000002U,
    APT_EMULATION_STOP_APT              = 0x00000003U,
} APT_EmulationMode;

/**
  * @brief Count mode of time-base counter.
  */
typedef enum {
    APT_COUNT_MODE_UP                   = 0x00000000U,
    APT_COUNT_MODE_DOWN                 = 0x00000001U,
    APT_COUNT_MODE_UP_DOWN              = 0x00000002U,
    APT_COUNT_MODE_FREEZE               = 0x00000003U,
} APT_CountMode;

/**
  * @brief Count mode after synchronization for slave APT module.
  */
typedef enum {
    APT_COUNT_MODE_AFTER_SYNC_DOWN      = 0x00000000U,
    APT_COUNT_MODE_AFTER_SYNC_UP        = 0x00000001U,
} APT_SyncCountMode;

/**
  * @brief Count compare reference of time-base counter.
  */
typedef enum {
    APT_COMPARE_REFERENCE_A             = 0x00000000U,
    APT_COMPARE_REFERENCE_B             = 0x00000001U,
    APT_COMPARE_REFERENCE_C             = 0x00000002U,
    APT_COMPARE_REFERENCE_D             = 0x00000003U,
} APT_CompareRef;

/**
  * @brief Buffer load mode of the registers that support buffer register.
  * @details Load mode:
  *          + APT_BUFFER_DISABLE -- Disable register buffer
  *          + APT_BUFFER_INDEPENDENT_LOAD -- Enable register buffer and load independently
  *          + APT_BUFFER_GLOBAL_LOAD -- enable register buffer and load globally
  */
typedef enum {
    APT_BUFFER_DISABLE                  = 0x00000000U,
    APT_BUFFER_INDEPENDENT_LOAD         = 0x00000001U,
    APT_BUFFER_GLOBAL_LOAD              = 0x00000003U,
} APT_BufferLoadMode;

/**
  * @brief PWM waveform output channel.
  */
typedef enum {
    APT_PWM_CHANNEL_A                   = 0x00000000U,
    APT_PWM_CHANNEL_B                   = 0x00000001U,
} APT_PWMChannel;

/**
  * @brief PWM waveform action on PWM action events.
  */
typedef enum {
    APT_PWM_ACTION_HOLD                 = 0x00000000U,
    APT_PWM_ACTION_LOW                  = 0x00000001U,
    APT_PWM_ACTION_HIGH                 = 0x00000002U,
    APT_PWM_ACTION_TOGGLE               = 0x00000003U,
} APT_PWMAction;

/**
  * @brief Count compare event for generating PWM waveform actions.
  *        The enumeration values are the register bit field offset of the corresponding action events.
  */
typedef enum {
    APT_PWM_ACTION_ON_TIMEBASE_ZERO     = 0U,
    APT_PWM_ACTION_ON_TIMEBASE_PERIOD   = 2U,
    APT_PWM_ACTION_ON_CMPA_COUNT_UP     = 4U,
    APT_PWM_ACTION_ON_CMPA_COUNT_DOWN   = 6U,
    APT_PWM_ACTION_ON_CMPB_COUNT_UP     = 8U,
    APT_PWM_ACTION_ON_CMPB_COUNT_DOWN   = 10U,
    APT_PWM_ACTION_ON_CMPC_COUNT_UP     = 12U,
    APT_PWM_ACTION_ON_CMPC_COUNT_DOWN   = 14U,
    APT_PWM_ACTION_ON_CMPD_COUNT_UP     = 16U,
    APT_PWM_ACTION_ON_CMPD_COUNT_DOWN   = 18U,
    APT_PWM_ACTION_ON_C1_COUNT_UP       = 20U,
    APT_PWM_ACTION_ON_C1_COUNT_DOWN     = 22U,
    APT_PWM_ACTION_ON_C2_COUNT_UP       = 24U,
    APT_PWM_ACTION_ON_C2_COUNT_DOWN     = 26U,
} APT_PWMActionEvent;

/**
  * @brief PWM action when using software continuous action.
  */
typedef enum {
    APT_PWM_CONTINUOUS_ACTION_HOLD      = 0x00000000U,
    APT_PWM_CONTINUOUS_ACTION_LOW       = 0x00000001U,
    APT_PWM_CONTINUOUS_ACTION_HIGH      = 0x00000002U,
} APT_PWMContAction;

/**
  * @brief PWM Generation event C1 and C2.
  */
typedef enum {
    APT_PWM_GENERATION_EVENT_C1         = 0x00000000U,
    APT_PWM_GENERATION_EVENT_C2         = 0x00000001U,
} APT_PGEventCx;

/**
  * @brief Source of PWM Generation event C1 and C2.
  */
typedef enum {
    APT_PG_EVT_C_FORBIDDEN              = 0x00000000U,
    APT_PG_EVT_C_COMBINE_EVENT_A1       = 0x00000001U,
    APT_PG_EVT_C_COMBINE_EVENT_A2       = 0x00000002U,
    APT_PG_EVT_C_COMBINE_EVENT_B1       = 0x00000003U,
    APT_PG_EVT_C_COMBINE_EVENT_B2       = 0x00000004U,
    APT_PG_EVT_C_COMBINE_EVENT_FILT     = 0x00000005U,
    APT_PG_EVT_C_IO_EVENT1              = 0x00000006U,
    APT_PG_EVT_C_IO_EVENT2              = 0x00000007U,
    APT_PG_EVT_C_IO_EVENT3              = 0x00000008U,
    APT_PG_EVT_C_SYNC_IN                = 0x00000009U,
} APT_PGEventCxSrc;

/**
  * @brief Input source of Dead-Band rising edge delay counter.
  * @details Input source:
  *         + APT_DB_RED_INPUT_PWM_A -- Dead-Band rising edge delay input is PWM channel A
  *         + APT_DB_RED_INPUT_PWM_B -- Dead-Band rising edge delay input is PWM channel B
  */
typedef enum {
    APT_DB_RED_INPUT_PWM_A              = 0x00000000U,
    APT_DB_RED_INPUT_PWM_B              = 0x00000001U,
} APT_REDInput;

/**
  * @brief Output mode of Dead-Band rising edge delay counter.
  * @details Output mode:
  *         + APT_DB_RED_OUTPUT_NOT_INVERT -- Dead-Band rising edge delay output is not inverted
  *         + APT_DB_RED_OUTPUT_INVERT -- Dead-Band rising edge delay output is inverted
  *         + APT_DB_RED_OUTPUT_PWM_A -- Dead-Band rising edge delay is bypassed
  */
typedef enum {
    APT_DB_RED_OUTPUT_NOT_INVERT        = 0x00000000U,
    APT_DB_RED_OUTPUT_INVERT            = 0x00000002U,
    APT_DB_RED_OUTPUT_PWM_A             = 0x00000003U,
} APT_REDOutMode;

/**
  * @brief Input source of Dead-Band falling edge delay counter.
  * @details Input source:
  *         + APT_DB_FED_INPUT_PWM_B -- Dead-Band falling edge delay input is PWM channel B
  *         + APT_DB_FED_INPUT_PWM_A -- Dead-Band falling edge delay input is PWM channel A
  *         + APT_DB_FED_INPUT_RED_OUT -- Falling edge delay input is rising edge delay output
  *         + APT_DB_FED_INPUT_ZERO -- Dead-Band falling edge delay input is 0
  */
typedef enum {
    APT_DB_FED_INPUT_PWM_B              = 0x00000000U,
    APT_DB_FED_INPUT_PWM_A              = 0x00000001U,
    APT_DB_FED_INPUT_RED_OUT            = 0x00000002U,
    APT_DB_FED_INPUT_ZERO               = 0x00000003U,
} APT_FEDInput;

/**
  * @brief Output mode of Dead-Band falling edge delay counter.
  * @details Output mode:
  *         + APT_DB_FED_OUTPUT_NOT_INVERT -- Dead-Band falling edge delay output is not inverted
  *         + APT_DB_FED_OUTPUT_INVERT -- Dead-Band falling edge delay output is inverted
  *         + APT_DB_FED_OUTPUT_PWM_B -- Dead-Band falling edge delay is bypassed
  */
typedef enum {
    APT_DB_FED_OUTPUT_NOT_INVERT        = 0x00000000U, /**< Dead-Band falling edge delay output is not inverted */
    APT_DB_FED_OUTPUT_INVERT            = 0x00000002U, /**< Dead-Band falling edge delay output is inverted */
    APT_DB_FED_OUTPUT_PWM_B             = 0x00000003U, /**< Dead-Band falling edge delay is bypassed */
} APT_FEDOutMode;

/**
  * @brief Output control events.
  */
typedef enum {
    APT_OC_NO_EVENT                     = 0x00000000U,
    APT_OC_GPIO_EVENT_1                 = 0x00000001U,
    APT_OC_GPIO_EVENT_2                 = 0x00000002U,
    APT_OC_GPIO_EVENT_3                 = 0x00000004U,
    APT_OC_SYSTEM_EVENT_1               = 0x00000010U,
    APT_OC_SYSTEM_EVENT_2               = 0x00000020U,
    APT_OC_SYSTEM_EVENT_3               = 0x00000040U,
    APT_OC_COMBINE_EVENT_A1             = 0x00000100U,
    APT_OC_COMBINE_EVENT_A2             = 0x00000200U,
    APT_OC_COMBINE_EVENT_B1             = 0x00000400U,
    APT_OC_COMBINE_EVENT_B2             = 0x00000800U,
} APT_OutCtrlEvent;

/**
  * @brief Output control event mode.
  */
typedef enum {
    APT_OUT_CTRL_ONE_SHOT               = 0x00000000U,
    APT_OUT_CTRL_CYCLE_BY_CYBLE         = 0x00000001U,
} APT_OutCtrlMode;

/**
  * @brief Advanced output control events take into consideration of the direction of time-base counter.
  *        The enumeration values are the register bit field offset of the corresponding output control events.
  */
typedef enum {
    APT_OC_EVT_GPIO_OR_SYSTEM_UP        = 0U,
    APT_OC_EVT_COMBINE_EVENT_A1_UP      = 3U,
    APT_OC_EVT_COMBINE_EVENT_A2_UP      = 6U,
    APT_OC_EVT_COMBINE_EVENT_B1_UP      = 9U,
    APT_OC_EVT_COMBINE_EVENT_B2_UP      = 12U,
    APT_OC_EVT_GPIO_OR_SYSTEM_DOWN      = 16U,
    APT_OC_EVT_COMBINE_EVENT_A1_DOWN    = 19U,
    APT_OC_EVT_COMBINE_EVENT_A2_DOWN    = 22U,
    APT_OC_EVT_COMBINE_EVENT_B1_DOWN    = 25U,
    APT_OC_EVT_COMBINE_EVENT_B2_DOWN    = 28U,
} APT_OutCtrlEventDir;

/**
  * @brief Output control action.
  * @details Control action:
  *         + APT_OUT_CTRL_ACTION_DISABLE -- Disable output protect control. Output PWM directly
  *         + APT_OUT_CTRL_ACTION_LOW -- Output low level
  *         + APT_OUT_CTRL_ACTION_HIGH -- Output high level
  *         + APT_OUT_CTRL_ACTION_HOLD -- Hold the current output state
  *         + APT_OUT_CTRL_ACTION_TOGGLE -- Toggle the current output state
  *         + APT_OUT_CTRL_ACTION_HIGH_Z -- High-impedance output
  */
typedef enum {
    APT_OUT_CTRL_ACTION_DISABLE         = 0x00000000U,
    APT_OUT_CTRL_ACTION_LOW             = 0x00000001U,
    APT_OUT_CTRL_ACTION_HIGH            = 0x00000002U,
    APT_OUT_CTRL_ACTION_HOLD            = 0x00000003U,
    APT_OUT_CTRL_ACTION_TOGGLE          = 0x00000004U,
    APT_OUT_CTRL_ACTION_HIGH_Z          = 0x00000005U,
} APT_OutCtrlAction;

/**
  * @brief Event latch clear mode of cycle-by-cycle output control mode.
  */
typedef enum {
    APT_CLEAR_CBC_ON_CNTR_ZERO          = 0x00000001U,
    APT_CLEAR_CBC_ON_CNTR_PERIOD        = 0x00000002U,
    APT_CLEAR_CBC_ON_CNTR_ZERO_PERIOD   = 0x00000003U,
} APT_CBCClearMode;

/**
  * @brief Source of timer interrupt.
  */
typedef enum {
    APT_INT_SRC_CNTR_DISABLE            = 0x00000000U,
    APT_INT_SRC_CNTR_ZERO               = 0x00000001U,
    APT_INT_SRC_CNTR_PERIOD             = 0x00000002U,
    APT_INT_SRC_CNTR_ZERO_PERIOD        = 0x00000003U,
    APT_INT_SRC_CNTR_CMPA_UP            = 0x00000004U,
    APT_INT_SRC_CNTR_CMPA_DOWN          = 0x00000005U,
    APT_INT_SRC_CNTR_CMPB_UP            = 0x00000006U,
    APT_INT_SRC_CNTR_CMPB_DOWN          = 0x00000007U,
    APT_INT_SRC_CNTR_CMPC_UP            = 0x00000008U,
    APT_INT_SRC_CNTR_CMPC_DOWN          = 0x00000009U,
    APT_INT_SRC_CNTR_CMPD_UP            = 0x0000000AU,
    APT_INT_SRC_CNTR_CMPD_DOWN          = 0x0000000BU,
} APT_TimerInterruptSrc;

/**
  * @brief ADC trigger channels.
  */
typedef enum {
    APT_ADC_CONVERSION_START_A          = 0x00000001U,
    APT_ADC_CONVERSION_START_B          = 0x00000002U,
} APT_ADCTriggerChannel;

/**
  * @brief Source of ADC trigger channels.
  */
typedef enum {
    APT_CS_SRC_COMBINE_EVENT_A1         = 0x00000000U,
    APT_CS_SRC_CNTR_ZERO                = 0x00000001U,
    APT_CS_SRC_CNTR_PERIOD              = 0x00000002U,
    APT_CS_SRC_CNTR_ZERO_PERIOD         = 0x00000003U,
    APT_CS_SRC_CNTR_CMPA_UP             = 0x00000004U,
    APT_CS_SRC_CNTR_CMPA_DOWN           = 0x00000005U,
    APT_CS_SRC_CNTR_CMPB_UP             = 0x00000006U,
    APT_CS_SRC_CNTR_CMPB_DOWN           = 0x00000007U,
    APT_CS_SRC_CNTR_CMPC_UP             = 0x00000008U,
    APT_CS_SRC_CNTR_CMPC_DOWN           = 0x00000009U,
    APT_CS_SRC_CNTR_CMPD_UP             = 0x0000000AU,
    APT_CS_SRC_CNTR_CMPD_DOWN           = 0x0000000BU,
} APT_ADCTriggerSource;

/**
  * @brief DMA request source of ADC Converter Start submodule.
  */
typedef enum {
    APT_CS_DMA_REQ_SRC_DISABLE          = 0x00000000U,
    APT_CS_DMA_REQ_SRC_CHANNEL_A        = 0x00000001U,
    APT_CS_DMA_REQ_SRC_CHANNEL_B        = 0x00000002U,
} APT_ADCTrgDMAReqSrc;

/**
  * @brief DMA request type of ADC Converter Start submodule.
  */
typedef enum {
    APT_CS_DMA_SINGLE_REQUEST           = 0x00000000U,
    APT_CS_DMA_BURST_REQUEST            = 0x00000002U,
} APT_ADCTrgDMAReqType;

/**
  * @brief Polarity of the events of Event Management submodule.
  * @details Polarity:
  *         + APT_EM_EVENT_POLARITY_NOT_INVERT -- High active.
  *         + APT_EM_EVENT_POLARITY_INVERT -- Low active.
  *         + APT_EM_EVENT_POLARITY_FORCE_LOW -- Force event to low level.
  *         + APT_EM_EVENT_POLARITY_FORCE_HIGH -- Force event to high level.
  */
typedef enum {
    APT_EM_EVENT_POLARITY_NOT_INVERT    = 0x00000000U,
    APT_EM_EVENT_POLARITY_INVERT        = 0x00000001U,
    APT_EM_EVENT_POLARITY_FORCE_LOW     = 0x00000002U,
    APT_EM_EVENT_POLARITY_FORCE_HIGH    = 0x00000003U,
} APT_EMEventPolarity;

/**
  * @brief GPIO events and system events of Event Management submodule.
  *        The enumeration values are the register bit field offset of the corresponding GPIO/system events.
  */
typedef enum {
    APT_EM_GPIO_EVENT_1                 = 0U,
    APT_EM_GPIO_EVENT_2                 = 2U,
    APT_EM_GPIO_EVENT_3                 = 4U,
    APT_EM_GPIO_EVENT_4                 = 6U,
    APT_EM_GPIO_EVENT_5                 = 8U,
    APT_EM_SYSTEM_EVENT_1               = 16U,
    APT_EM_SYSTEM_EVENT_2               = 18U,
    APT_EM_SYSTEM_EVENT_3               = 20U,
} APT_EMIOSysEvent;

/**
  * @brief Multiplexing events of Event Management submodule.
  *        The enumeration values are the register bit field offset of the corresponding multiplexing events.
  */
typedef enum {
    APT_EM_MP_EVENT_1                  = 0U,
    APT_EM_MP_EVENT_2                  = 2U,
    APT_EM_MP_EVENT_3                  = 4U,
    APT_EM_MP_EVENT_4                  = 6U,
    APT_EM_MP_EVENT_5                  = 8U,
    APT_EM_MP_EVENT_6                  = 10U,
} APT_EMMuxEvent;

/**
  * @brief Event Module of Event Management submodule.
  */
typedef enum {
    APT_EM_MODULE_A                     = 0x00000000U,
    APT_EM_MODULE_B                     = 0x00000001U,
} APT_EMGroup;

/**
  * @brief Group of combine event source input.
  */
typedef enum {
    APT_EM_COMBINE_SRC_GRP_A1     = 0x00000000U,
    APT_EM_COMBINE_SRC_GRP_A2     = 0x00000001U,
    APT_EM_COMBINE_SRC_GRP_B1     = 0x00000002U,
    APT_EM_COMBINE_SRC_GRP_B2     = 0x00000003U,
} APT_EMCombineEvtSrcGrp;

/**
  * @brief Source of combine events A1, A2, B1, B2.
  */
typedef enum {
    APT_EM_COMBINE_SRC_EVT_1            = 0x00000000U,
    APT_EM_COMBINE_SRC_EVT_2            = 0x00000001U,
    APT_EM_COMBINE_SRC_EVT_3            = 0x00000002U,
    APT_EM_COMBINE_SRC_EVT_MP_1         = 0x00000003U,
    APT_EM_COMBINE_SRC_EVT_MP_2         = 0x00000004U,
    APT_EM_COMBINE_SRC_EVT_MP_3         = 0x00000005U,
    APT_EM_COMBINE_SRC_EVT_MP_4         = 0x00000006U,
    APT_EM_COMBINE_SRC_EVT_MP_5         = 0x00000007U,
    APT_EM_COMBINE_SRC_EVT_MP_6         = 0x00000008U,
    APT_EM_COMBINE_SRC_ALL_EVENT_OR     = 0x0000000FU, /* based on EM_AOR_EN/EM_BOR_EN */
} APT_EMCombineEvtSrc;

/**
  * @brief Combine events of Event Management submodule.
  */
typedef enum {
    APT_EM_COMBINE_EVENT_A1             = 0x00000000U,
    APT_EM_COMBINE_EVENT_A2             = 0x00000001U,
    APT_EM_COMBINE_EVENT_B1             = 0x00000002U,
    APT_EM_COMBINE_EVENT_B2             = 0x00000003U,
} APT_EMCombineEvent;

/**
  * @brief Combine Mode of combine events A1, A2, B1, B2.
  * @details combine mode:
  *         + The combine result is set output to low level
  *         + The combine result is qual to event 1
  *         + The combine result is the logical AND of group event 1 high level and group event 2 low level
  *         + The combine result is the logical AND of group event 1 high level and group event 2 low level
  *         + The combine result is the logical AND of group event 1 high level and group event 2 high level
  *         + The combine result is the logical AND of group event 1 low level and group event 2 low level
  */
typedef enum {
    APT_EM_COMBINE_LOW_LEVEL            = 0x00000000U,
    APT_EM_COMBINE_EVT1                 = 0x00000001U,
    APT_EM_COMBINE_EVT1_H_AND_EVT2_L    = 0x00000002U,
    APT_EM_COMBINE_EVT1_H_AND_EVT2_H    = 0x00000003U,
    APT_EM_COMBINE_EVT1_L_AND_EVT2_H    = 0x00000004U,
    APT_EM_COMBINE_EVT2                 = 0x00000005U,
} APT_EMCombineEvtMode;

/**
  * @brief Output type of combine events.
  * @details Output type:
  *                     +APT_EM_COMBINE_EVENT_OUT_ORIG_SIGNAL -- The source of combine event is unfiltered
  *                     +APT_EM_COMBINE_EVENT_OUT_FILT_SIGNAL -- The source of combine event is filtered
  */
typedef enum {
    APT_EM_COMBINE_EVENT_OUT_ORIG_SIGNAL = 0x00000000U,
    APT_EM_COMBINE_EVENT_OUT_FILT_SIGNAL = 0x00000001U,
} APT_EMCombineEventOut;

/**
  * @brief Polarity of mask window.
  */
typedef enum {
    APT_BLANK_EVENT_INSIDE_MASK_WIN     = 0x00000000U,
    APT_BLANK_EVENT_OUTSIDE_MASK_WIN    = 0x00000001U,
} APT_MaskWinPolarity;

/**
  * @brief Reset mode of mask window and count capture.
  */
typedef enum {
    APT_RESET_MASK_WIN_DISABLE          = 0x00000000U,
    APT_RESET_MASK_WIN_CNTR_ZERO        = 0x00000001U,
    APT_RESET_MASK_WIN_CNTR_PERIOD      = 0x00000002U,
    APT_RESET_MASK_WIN_CNTR_ZERO_PERIOD = 0x00000003U,
} APT_MaskWinResetMode;

/**
  * @brief Clock source of valley capture.
  */
typedef enum {
    APT_VALLY_CAP_USE_MAIN_CLOCK        = 0x00000000U,
    APT_VALLEY_CAP_USE_DIVIDER_CLOCK    = 0x00000001U,
} APT_ValleyCapClkMode;

/**
  * @brief Trigger source of valley capture.
  */
typedef enum {
    APT_VALLEY_CAP_SRC_DISABLE          = 0x00000000U,
    APT_VALLEY_CAP_SRC_CNTR_ZERO        = 0x00000001U,
    APT_VALLEY_CAP_SRC_CNTR_PERIOD      = 0x00000002U,
    APT_VALLEY_CAP_SRC_CNTR_ZERO_PERIOD = 0x00000003U,
    APT_VALLEY_CAP_SRC_COMBINE_EVENT_A1 = 0x00000004U,
    APT_VALLEY_CAP_SRC_COMBINE_EVENT_A2 = 0x00000005U,
    APT_VALLEY_CAP_SRC_COMBINE_EVENT_B1 = 0x00000006U,
    APT_VALLEY_CAP_SRC_COMBINE_EVENT_B2 = 0x00000007U,
} APT_ValleyCapRstType;

/**
  * @brief Edge type of valley capture.
  */
typedef enum {
    APT_VALLEY_CAP_RISING_EDGE          = 0x00000000U,
    APT_VALLEY_CAP_FALLING_EDGE         = 0x00000001U,
} APT_ValleyCapEdgeType;

/**
  * @brief Delay calibration of valley capture.
  * @details Delay calibration:
  *         + APT_VCAP_SW_DELAY -- Delay value = software delay value
  *         + APT_VCAP_VCNT_DELAY_DIVIDE_1_SW_DELAY -- Delay value = capture count value + software delay value
  *         + APT_VCAP_VCNT_DELAY_DIVIDE_2_SW_DELAY -- Delay value = capture count value / 2 + software delay value
  *         + APT_VCAP_VCNT_DELAY_DIVIDE_4_SW_DELAY -- Delay value = capture count value / 4 + software delay value
  *         + APT_VCAP_VCNT_DELAY_DIVIDE_8_SW_DELAY -- Delay value = capture count value / 8 + software delay value
  *         + APT_VCAP_VCNT_DELAY_DIVIDE_16_SW_DELAY -- Delay value = capture count value / 16 + software delay value
  *         + APT_VCAP_VCNT_DELAY_DIVIDE_32_SW_DELAY -- Delay value = capture count value / 32 + software delay value
  */
typedef enum {
    APT_VCAP_SW_DELAY                       = 0x00000000U,
    APT_VCAP_VCNT_DELAY_DIVIDE_1_SW_DELAY   = 0x00000001U,
    APT_VCAP_VCNT_DELAY_DIVIDE_2_SW_DELAY   = 0x00000002U,
    APT_VCAP_VCNT_DELAY_DIVIDE_4_SW_DELAY   = 0x00000003U,
    APT_VCAP_VCNT_DELAY_DIVIDE_8_SW_DELAY   = 0x00000004U,
    APT_VCAP_VCNT_DELAY_DIVIDE_16_SW_DELAY  = 0x00000005U,
    APT_VCAP_VCNT_DELAY_DIVIDE_32_SW_DELAY  = 0x00000006U,
} APT_ValleyDelayMode;

/**
  * @brief Start and stop edge of valley capture.
  */
typedef enum {
    APT_VALLEY_COUNT_START_EDGE         = 0x00000000U,
    APT_VALLEY_COUNT_STOP_EDGE          = 0x00000001U,
} APT_ValleyCountEdge;

/**
  * @brief Edge filter mode of Event Management submodule.
  */
typedef enum {
    APT_EM_EDGEFILTER_MODE_RISING       = 0x00000000U,
    APT_EM_EDGEFILTER_MODE_FALLING      = 0x00000002U,
    APT_EM_EDGEFILTER_MODE_BOTH         = 0x00000003U,
} APT_EMEdgeFilterMode;

/**
  * @brief Sync-in source of slave APT module.
  */
typedef enum {
    APT_SYNCIN_SRC_APT0_SYNCOUT         = 0x00000000U,
    APT_SYNCIN_SRC_APT1_SYNCOUT         = 0x00000001U,
    APT_SYNCIN_SRC_APT2_SYNCOUT         = 0x00000002U,
    APT_SYNCIN_SRC_APT3_SYNCOUT         = 0x00000003U,
    APT_SYNCIN_SRC_APT4_SYNCOUT         = 0x00000004U,
    APT_SYNCIN_SRC_APT5_SYNCOUT         = 0x00000005U,
    APT_SYNCIN_SRC_APT6_SYNCOUT         = 0x00000006U,
    APT_SYNCIN_SRC_APT7_SYNCOUT         = 0x00000007U,
    APT_SYNCIN_SRC_APT8_SYNCOUT         = 0x00000008U,
    APT_SYNCIN_SRC_CAPM0_SYNCOUT        = 0x00000009U,
    APT_SYNCIN_SRC_CAPM1_SYNCOUT        = 0x0000000AU,
    APT_SYNCIN_SRC_CAPM2_SYNCOUT        = 0x0000000BU,
    APT_SYNCIN_SRC_GPIO_EVENT_4         = 0x0000000CU,
    APT_SYNCIN_SRC_GPIO_EVENT_5         = 0x0000000DU,
    APT_SYNCIN_SRC_DISABLE              = 0x0000000EU,
} APT_SyncInSrc;

/**
  * @brief Sync-out mode of master APT module.
  * @details Sync-out mode:
  *         + APT_SYNCOUT_ONE_SHOT_MODE -- One-Shot synchronization mode
  *         + APT_SYNCOUT_MULTIPLE_MODE -- Multiple synchronization mode
  */
typedef enum {
    APT_SYNCOUT_ONE_SHOT_MODE           = 0x00000000U,
    APT_SYNCOUT_MULTIPLE_MODE           = 0x00000001U,
} APT_SyncOutMode;

/**
  * @brief  Selection of sync-out latch when using one-shot sync-out mode.
  * @details Sync-out latch:
  *         + APT_SYNCOUT_LATCH_SET_ON_SW_FORCE -- Select rg_latset_otsyn as the latch set condition
  *         + APT_SYNCOUT_LATCH_SET_ON_GLB_LOAD -- Select rg_latset_otgld as the latch set condition
  */
typedef enum {
    APT_SYNCOUT_LATCH_SET_ON_SW_FORCE   = 0x00000000U,
    APT_SYNCOUT_LATCH_SET_ON_GLB_LOAD   = 0x00000001U,
} APT_SyncOutLatSetSel;

/**
  * @brief Source of peripheral synchronization.
  */
typedef enum {
    APT_PER_SYNCOUT_SRC_DISABLE             = 0x00000000U,
    APT_PER_SYNCOUT_SRC_CNTR_ZERO           = 0x00000001U,
    APT_PER_SYNCOUT_SRC_CNTR_PERIOD         = 0x00000002U,
    APT_PER_SYNCOUT_SRC_CNTR_ZERO_PERIOD    = 0x00000003U,
    APT_PER_SYNCOUT_SRC_CNTR_CMPC_UP        = 0x00000004U,
    APT_PER_SYNCOUT_SRC_CNTR_CMPC_DOWN      = 0x00000005U,
    APT_PER_SYNCOUT_SRC_CNTR_CMPD_UP        = 0x00000006U,
    APT_PER_SYNCOUT_SRC_CNTR_CMPD_DOWN      = 0x00000007U,
} APT_PeriphSyncOutSrc;

/**
  * @brief Global buffer load mode.
  */
typedef enum {
    APT_GLB_LOAD_ONE_SHOT_MODE          = 0x00000000U,
    APT_GLB_LOAD_MULTIPLE_MODE          = 0x00000001U,
} APT_GlobalLoadMode;

/**
  * @brief The buffer of the registers that support buffer register.
  */
typedef enum {
    APT_REG_BUFFER_TC_PRD               = 0x00000001U,
    APT_REG_BUFFER_TC_REFA              = 0x00000002U,
    APT_REG_BUFFER_TC_REFB              = 0x00000004U,
    APT_REG_BUFFER_TC_REFC              = 0x00000008U,
    APT_REG_BUFFER_TC_REFD              = 0x00000010U,
    APT_REG_BUFFER_PG_ACT_A             = 0x00000040U,
    APT_REG_BUFFER_PG_ACT_B             = 0x00000080U,
    APT_REG_BUFFER_PG_OUT_FRC           = 0x00000100U,
    APT_REG_BUFFER_DG_RED               = 0x00000400U,
    APT_REG_BUFFER_DG_FED               = 0x00000800U,
    APT_REG_BUFFER_DG_CFG               = 0x00001000U,
} APT_RegBuffer;

/**
  * @brief Software force events.
  */
typedef enum {
    APT_FORCE_EVENT_COUNTER_SYNC        = 0x00000001U,
    APT_FORCE_EVENT_SYNCOUT             = 0x00000010U,
    APT_FORCE_EVENT_SYNC_PERIPH         = 0x00000100U,
    APT_FORCE_EVENT_GLOBAL_LOAD         = 0x00001000U,
    APT_FORCE_EVENT_VALLEY_CAP_RST      = 0x00010000U,
    APT_FORCE_EVENT_ADC_START_A         = 0x00100000U,
    APT_FORCE_EVENT_ADC_START_B         = 0x00200000U,
    APT_FORCE_EVENT_TIMER_INTERRUPT     = 0x01000000U,
    APT_FORCE_EVENT_PWM_ACTION_BUF_LOAD = 0x10000000U,
} APT_ForceEvtType;

/**
  * @brief Software force events.
  * @details Reference point selection.
  *          + APT_REFERENCE_DOTA -- Select referece dot A as action trigger point.
  *          + APT_REFERENCE_DOTB -- Select referece dot B as action trigger point.
  *          + APT_REFERENCE_DOTC -- Select referece dot C as action trigger point.
  *          + APT_REFERENCE_DOTD -- Select referece dot D as action trigger point.
  */
typedef enum {
    APT_REFERENCE_DOTA = 0x00000000U,
    APT_REFERENCE_DOTB = 0x00000001U,
    APT_REFERENCE_DOTC = 0x00000002U,
    APT_REFERENCE_DOTD = 0x00000003U,
} APT_RefDotSelect;

/**
  * @brief Configure action point parameters.
  * @details Property of the action point.
  *          + refDotValue -- the action point value.
  *          + refDotDivValue -- frequency division value of the action point.
  *          + pwmChannel -- number of channels for which the action point needs to be changed. @ref APT_PWMChannel
  *          + actionEvent -- action event configure of reference point. @ref APT_PWMActionEvent
  *          + action -- triggle action of reference point. @ref APT_PWMAction
  * @note: the value of Reference Point must be less than or equal to the value of period.
  */
typedef struct {
    unsigned int            refDotValue;
    APT_PWMChannel          pwmChannel;       /* PWM channel selection. */
    APT_PWMActionEvent      actionEvent;      /* Point triggle action event. */
    APT_PWMAction           action;           /* Point action. */
} APT_RefDotParameters;
/**
  * @}
  */

/**
  * @defgroup APT_REG_Definition APT Register Structure.
  * @brief APT Register Structure Definition.
  * @{
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int sub_version : 4; /**< ip subversion */
        unsigned int main_version : 4; /**< ip main verison */
        unsigned int reserved0 : 24;
    } BIT;
} volatile VER_INFO_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_cnt_mode : 2; /**< timer work mode */
        unsigned int reserved1 : 14;
        unsigned int rg_div_fac : 12; /**< divider factor */
        unsigned int rg_emu_stop : 2; /**< emulation stop mode */
        unsigned int reserved2 : 2;
    } BIT;
} volatile TC_MODE_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_cnt_phs : 16; /**< timer's phase */
        unsigned int reserved3 : 15;
        unsigned int rg_cnt_dir : 1; /**< timer count direction */
    } BIT;
} volatile TC_PHS_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_cnt_ovrid : 16; /**< timer count init value */
        unsigned int reserved4 : 15;
        unsigned int rg_cnt_ovrid_en : 1; /**< timer and divider init enable */
    } BIT;
} volatile TC_OVRID_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_cnt_prd : 16; /* count period */
        unsigned int reserved5 : 16;
    } BIT;
} volatile TC_PRD_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_cnt_refa : 16; /* reference A counter value */
        unsigned int reserved6 : 16;
    } BIT;
} volatile TC_REFA_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_cnt_refb : 16; /* reference B counter value */
        unsigned int reserved7 : 16;
    } BIT;
} volatile TC_REFB_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_cnt_refc : 16; /* reference C counter value */
        unsigned int reserved8 : 16;
    } BIT;
} volatile TC_REFC_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_cnt_refd : 16; /* reference D counter value */
        unsigned int reserved9 : 16;
    } BIT;
} volatile TC_REFD_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_prd_buf_en : 1; /**< period buffer enable */
        unsigned int rg_prd_gld_en : 1; /**< period global buffer enable */
        unsigned int reserved10 : 2;
        unsigned int rg_refa_buf_en : 1; /**< reference A buffer enable */
        unsigned int rg_refa_gld_en : 1; /**< reference A global buffer enable */
        unsigned int rg_refb_buf_en : 1; /**< reference B buffer enable */
        unsigned int rg_refb_gld_en : 1; /**< reference B global buffer enable */
        unsigned int rg_refc_buf_en : 1; /**< reference C buffer enable */
        unsigned int rg_refc_gld_en : 1; /**< reference C global buffer enable */
        unsigned int rg_refd_buf_en : 1; /**< reference D buffer enable */
        unsigned int rg_refd_gld_en : 1; /**< reference D global buffer enable */
        unsigned int reserved11 : 20;
    } BIT;
} volatile TC_BUF_EN_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_prd_ld_zroen : 1; /**< period value register load at zero */
        unsigned int reserved12 : 1;
        unsigned int rg_prd_ld_a1en : 1; /**< period value load at evt_a1 */
        unsigned int rg_prd_ld_b1en : 1; /**< period value load at evt_b1 */
        unsigned int rg_prd_ld_synen : 1; /**< period value load at sync signal input */
        unsigned int reserved13 : 27;
    } BIT;
} volatile TC_PRD_LOAD_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_refa_ld_zroen : 1; /**< reference A value load at zero */
        unsigned int rg_refa_ld_prden : 1; /**< reference A value load at period */
        unsigned int rg_refa_ld_a1en : 1;  /**< reference A value load at evt_a1 */
        unsigned int rg_refa_ld_b1en : 1;  /**< reference A value load at evt_b1 */
        unsigned int rg_refa_ld_synen : 1; /**< reference A value load at sync signal input */
        unsigned int reserved14 : 3;
        unsigned int rg_refb_ld_zroen : 1; /**< reference B value load at zero */
        unsigned int rg_refb_ld_prden : 1; /**< reference B value load at period */
        unsigned int rg_refb_ld_a1en : 1;  /**< reference B value load at evt_a1 */
        unsigned int rg_refb_ld_b1en : 1;  /**< reference B value load at evt_b1 */
        unsigned int rg_refb_ld_synen : 1; /**< reference B value load at sync signal input */
        unsigned int reserved15 : 3;
        unsigned int rg_refc_ld_zroen : 1; /**< reference C value load at zero */
        unsigned int rg_refc_ld_prden : 1; /**< reference C value load at period */
        unsigned int rg_refc_ld_a1en : 1;  /**< reference C value load at evt_a1 */
        unsigned int rg_refc_ld_b1en : 1;  /**< reference C value load at evt_b1 */
        unsigned int rg_refc_ld_synen : 1; /**< reference C value load at sync signal input */
        unsigned int reserved16 : 3;
        unsigned int rg_refd_ld_zroen : 1; /**< reference D value load at zero */
        unsigned int rg_refd_ld_prden : 1; /**< reference D value load at period */
        unsigned int rg_refd_ld_a1en : 1;  /**< reference D value load at evt_a1 */
        unsigned int rg_refd_ld_b1en : 1;  /**< reference D value load at evt_b1 */
        unsigned int rg_refd_ld_synen : 1; /**< reference D value load at sync signal input */
        unsigned int reserved17 : 3;
    } BIT;
} volatile TC_REF_LOAD_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_mskwd_psel : 1; /**< mask window polarity */
        unsigned int reserved18 : 30;
        unsigned int rg_mskwd_en : 1; /**< mask window enable */
    } BIT;
} volatile TC_MWD_EN_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_mwd_refa : 16; /**< mask window reference value A */
        unsigned int reserved19 : 16;
    } BIT;
} volatile TC_MWDREFA_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_mwd_refb : 16; /**< mask window reference value B */
        unsigned int reserved20 : 16;
    } BIT;
} volatile TC_MWDREFB_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_mwdrefa_act_inc : 2; /**< action at reference A increase */
        unsigned int rg_mwdrefa_act_dec : 2; /**< action at reference A decrease */
        unsigned int rg_mwdrefb_act_inc : 2; /**< action at reference B increase */
        unsigned int rg_mwdrefb_act_dec : 2; /**< action at reference B decrease */
        unsigned int reserved21 : 24;
    } BIT;
} volatile TC_MWD_ACT_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_mwdrefa_buf_en : 1; /**< mask window reference A buffer enable */
        unsigned int rg_mwdrefa_gld_en : 1; /**< mask window reference A global buffer enable */
        unsigned int rg_mwdrefb_buf_en : 1; /**< mask window reference B buffer enable */
        unsigned int rg_mwdrefb_gld_en : 1; /**< mask window reference B global buffer enable */
        unsigned int rg_mwd_act_buf_en : 1; /**< mask window action buffer enable */
        unsigned int rg_mwd_act_gld_en : 1; /**< mask window action global buffer enable */
        unsigned int reserved22 : 26;
    } BIT;
} volatile TC_MWD_BUF_EN_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_mwdrefa_ld_zroen : 1; /**< mask window refrence A load at zero enable */
        unsigned int rg_mwdrefa_ld_prden : 1; /**< mask window refrence A load at period enable */
        unsigned int reserved23 : 1;
        unsigned int rg_mwdrefb_ld_zroen : 1; /**< mask window refrence B load at zero enable */
        unsigned int rg_mwdrefb_ld_prden : 1; /**< mask window refrence B load at period enable */
        unsigned int reserved24 : 1;
        unsigned int rg_mwd_act_ld_zroen : 1; /**< mask window action register load at zero enable */
        unsigned int rg_mwd_act_ld_prden : 1; /**< mask window action register load at period enable */
        unsigned int reserved25 : 24;
    } BIT;
} volatile TC_MWD_LOAD_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int ro_cnt_val : 16; /**< counter value */
        unsigned int ro_div_cnt : 12; /**< divider value */
        unsigned int reserved26 : 3;
        unsigned int ro_cnt_dir : 1; /**< count direction */
    } BIT;
} volatile TC_STS_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_pga_act_zro : 2;       /**< PG channel A action at zero */
        unsigned int rg_pga_act_prd : 2;       /**< PG channel A action at period */
        unsigned int rg_pga_act_refa_inc : 2;  /**< PG channel A action at reference A increase */
        unsigned int rg_pga_act_refa_dec : 2;  /**< PG channel A action at reference A decrease */
        unsigned int rg_pga_act_refb_inc : 2;  /**< PG channel A action at reference B increase */
        unsigned int rg_pga_act_refb_dec : 2;  /**< PG channel A action at reference B decrease */
        unsigned int rg_pga_act_refc_inc : 2;  /**< PG channel A action at reference C increase */
        unsigned int rg_pga_act_refc_dec : 2;  /**< PG channel A action at reference C decrease */
        unsigned int rg_pga_act_refd_inc : 2;  /**< PG channel A action at reference D increase */
        unsigned int rg_pga_act_refd_dec : 2;  /**< PG channel A action at reference D decrease */
        unsigned int rg_pga_act_evtc1_inc : 2; /**< PG channel A action at evt_c1 increase */
        unsigned int rg_pga_act_evtc1_dec : 2; /**< PG channel A action at evt_c1 decrease */
        unsigned int rg_pga_act_evtc2_inc : 2; /**< PG channel A action at evt_c2 increase */
        unsigned int rg_pga_act_evtc2_dec : 2; /**< PG channel A action at evt_c2 decrease */
        unsigned int reserved27 : 4;
    } BIT;
} volatile PG_ACT_A_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_pgb_act_zro : 2;      /**< PG channel A action at zero */
        unsigned int rg_pgb_act_prd : 2;      /**< PG channel A action at period */
        unsigned int rg_pgb_act_refa_inc : 2; /**< PG channel A action at reference A increase */
        unsigned int rg_pgb_act_refa_dec : 2; /**< PG channel A action at reference A decrease */
        unsigned int rg_pgb_act_refb_inc : 2; /**< PG channel A action at reference B increase */
        unsigned int rg_pgb_act_refb_dec : 2; /**< PG channel A action at reference B decrease */
        unsigned int rg_pgb_act_refc_inc : 2; /**< PG channel A action at reference C increase */
        unsigned int rg_pgb_act_refc_dec : 2; /**< PG channel A action at reference C decrease */
        unsigned int rg_pgb_act_refd_inc : 2; /**< PG channel A action at reference D increase */
        unsigned int rg_pgb_act_refd_dec : 2; /**< PG channel A action at reference D decrease */
        unsigned int rg_pgb_act_evtc1_inc : 2; /**< PG channel A action at evt_c1 increase */
        unsigned int rg_pgb_act_evtc1_dec : 2; /**< PG channel A action at evt_c1 decrease */
        unsigned int rg_pgb_act_evtc2_inc : 2; /**< PG channel A action at evt_c2 increase */
        unsigned int rg_pgb_act_evtc2_dec : 2; /**< PG channel A action at evt_c2 decrease */
        unsigned int reserved28 : 4;
    } BIT;
} volatile PG_ACT_B_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_pga_act_evt_frc : 2; /**< channel A force action select */
        unsigned int rg_pga_evt_frc : 1;     /**< enable a force action at channel A */
        unsigned int reserved29 : 1;
        unsigned int rg_pgb_act_evt_frc : 2; /**< channel A force action select */
        unsigned int rg_pgb_evt_frc : 1;     /**< enable a force action at channel A */
        unsigned int reserved30 : 25;
    } BIT;
} volatile PG_ACT_FRC_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_pga_frc_act : 2; /**< channel A force output action select */
        unsigned int rg_pga_frc_en : 1;  /**< channel A force output action enable */
        unsigned int reserved31 : 1;
        unsigned int rg_pgb_frc_act : 2; /**< channel A force output action select */
        unsigned int rg_pgb_frc_en : 1;  /**< channel A force output action enable */
        unsigned int reserved32 : 25;
    } BIT;
} volatile PG_OUT_FRC_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_acta_buf_en : 1; /**< channel A action value buffer enable */
        unsigned int rg_acta_gld_en : 1; /**< channel A action value global buffer enable */
        unsigned int rg_actb_buf_en : 1; /**< channel B action value buffer enable */
        unsigned int rg_actb_gld_en : 1; /**< channel B action value global buffer enable */
        unsigned int rg_frc_buf_en : 1;  /**< force output config buffer enable */
        unsigned int rg_frc_gld_en : 1;  /**< force output config global buffer enable */
        unsigned int reserved33 : 26;
    } BIT;
} volatile PG_BUF_EN_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_pga_actld_zroen : 1; /**< enable PG channel A action value independent load at zero */
        unsigned int rg_pga_actld_prden : 1; /**< enable PG channel A action value independent load at period */
        unsigned int rg_pga_actld_a1en : 1;  /**< enable PG channel A action value independent load at evt_a1 */
        unsigned int rg_pga_actld_b1en : 1;  /**< enable PG channel A action value independent load at evt_b1 */
        unsigned int rg_pga_actld_synen : 1; /**< enable PG channel A action value independent load at sync signal */
        unsigned int reserved34 : 3;
        unsigned int rg_pgb_actld_zroen : 1; /**< enable PG channel B action value independent load at zero */
        unsigned int rg_pgb_actld_prden : 1; /**< enable PG channel B action value independent load at period */
        unsigned int rg_pgb_actld_a1en : 1;  /**< enable PG channel B action value independent load at evt_a1 */
        unsigned int rg_pgb_actld_b1en : 1;  /**< enable PG channel B action value independent load at evt_b1 */
        unsigned int rg_pgb_actld_synen : 1; /**< enable PG channel B action value independent load at sync signal */
        unsigned int reserved35 : 3;
        unsigned int rg_pg_frcld_zroen : 1; /**< enable force action config value independent load at zero */
        unsigned int rg_pg_frcld_prden : 1; /**< enable force action config value independent load at period */
        unsigned int reserved36 : 2;
        unsigned int rg_pg_frcld_synen : 1; /**< enable force action config value independent load at sync signal */
        unsigned int reserved37 : 3;
        unsigned int reserved38 : 8;
    } BIT;
} volatile PG_ACT_LD_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_pga_evtc1_sel : 4; /**< pga_evtc1 source select */
        unsigned int rg_pga_evtc2_sel : 4; /**< pga_evtc2 source select */
        unsigned int rg_pgb_evtc1_sel : 4; /**< pgb_evtc1 source select */
        unsigned int rg_pgb_evtc2_sel : 4; /**< pgb_evtc2 source select */
        unsigned int reserved39 : 16;
    } BIT;
} volatile PG_EVTC_SEL_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_dg_red : 16; /**< deadband time at rising edge */
        unsigned int reserved0 : 16;
    } BIT;
} volatile DG_RED_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_dg_fed : 16; /**< deadband timer at falling edge */
        unsigned int reserved0 : 16;
    } BIT;
} volatile DG_FED_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_dg_red_isel : 2; /**< rising edge delay source input select */
        unsigned int rg_dg_fed_isel : 2; /**< falling edge delay source input select */
        unsigned int rg_dg_red_osel : 2; /**< rising edge delay polarity select */
        unsigned int rg_dg_fed_osel : 2; /**< falling  edge delay polarity select */
        unsigned int rg_dga_osel : 1;    /**< dga output waveform swap select */
        unsigned int rg_dgb_osel : 1;    /**< dgb output waveform swap select */
        unsigned int reserved42 : 22;
    } BIT;
} volatile DG_CFG_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_red_buf_en : 1; /**< rising edge delay value buffer enable */
        unsigned int rg_red_gld_en : 1; /**< rising edge delay value global buffer enable */
        unsigned int rg_fed_buf_en : 1; /**< falling edge delay value buffer enable */
        unsigned int rg_fed_gld_en : 1; /**< falling edge delay value global buffer enable */
        unsigned int rg_cfg_buf_en : 1; /**< deadband config buffer enable */
        unsigned int rg_cfg_gld_en : 1; /**< deadband config global enable */
        unsigned int reserved43 : 26;
    } BIT;
} volatile DG_BUF_EN_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_red_ld_zroen : 1; /**< rising edge delay value load independent at zero */
        unsigned int rg_red_ld_prden : 1; /**< rising edge delay value load independent at period */
        unsigned int reserved44 : 6;
        unsigned int rg_fed_ld_zroen : 1; /**< falling edge delay value load independent at zero */
        unsigned int rg_fed_ld_prden : 1; /**< falling edge delay value load independent at period */
        unsigned int reserved45 : 6;
        unsigned int rg_cfg_ld_zroen : 1; /**< deadband config register value load independent at zero */
        unsigned int rg_cfg_ld_prden : 1; /**< deadband config register value load independent at period */
        unsigned int reserved46 : 14;
    } BIT;
} volatile DG_BUF_LOAD_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_oc_en_evt1 : 1; /**< evtio1 output control enable */
        unsigned int rg_oc_en_evt2 : 1; /**< evtio2 output control enable */
        unsigned int rg_oc_en_evt3 : 1; /**< evtio3 output control enable */
        unsigned int reserved47 : 1;
        unsigned int rg_oc_en_evts1 : 1; /**< evts1 output control enable */
        unsigned int rg_oc_en_evts2 : 1; /**< evts2 output control enable */
        unsigned int rg_oc_en_evts3 : 1; /**< evts3 output control enable */
        unsigned int reserved48 : 1;
        unsigned int rg_oc_en_evta1 : 1; /**< evta1 output control enable */
        unsigned int rg_oc_en_evta2 : 1; /**< evta2 output control enable */
        unsigned int rg_oc_en_evtb1 : 1; /**< evtb1 output control enable */
        unsigned int rg_oc_en_evtb2 : 1; /**< evtb2 output control enable */
        unsigned int reserved49 : 4;
        unsigned int rg_oc_mode_evt1 : 1; /**< evtio1 output mode select */
        unsigned int rg_oc_mode_evt2 : 1; /**< evtio2 output mode select */
        unsigned int rg_oc_mode_evt3 : 1; /**< evtio3 output mode select */
        unsigned int reserved50 : 1;
        unsigned int rg_oc_mode_evts1 : 1; /**< evts1 output mode select */
        unsigned int rg_oc_mode_evts2 : 1; /**< evts2 output mode select */
        unsigned int rg_oc_mode_evts3 : 1; /**< evts3 output mode select */
        unsigned int reserved51 : 1;
        unsigned int rg_oc_mode_evta1 : 1; /**< evta1 output mode select */
        unsigned int rg_oc_mode_evta2 : 1; /**< evta2 output mode select */
        unsigned int rg_oc_mode_evtb1 : 1; /**< evtb1 output mode select */
        unsigned int rg_oc_mode_evtb2 : 1; /**< evtb2 output mode select */
        unsigned int reserved52 : 4;
    } BIT;
} volatile OC_MODE_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_oc_laten_evt1 : 1; /**< output control evtio1 latch event enable */
        unsigned int rg_oc_laten_evt2 : 1; /**< output control evtio2 latch event enable */
        unsigned int rg_oc_laten_evt3 : 1; /**< output control evtio3 latch event enable */
        unsigned int reserved53 : 1;
        unsigned int rg_oc_laten_evts1 : 1; /**< output control evtis1 latch event enable */
        unsigned int rg_oc_laten_evts2 : 1; /**< output control evtis2 latch event enable */
        unsigned int rg_oc_laten_evts3 : 1; /**< output control evtis3 latch event enable */
        unsigned int reserved54 : 1;
        unsigned int rg_oc_laten_evta1 : 1; /**< output control evtia1 latch event enable */
        unsigned int rg_oc_laten_evta2 : 1; /**< output control evtia2 latch event enable */
        unsigned int rg_oc_laten_evtb1 : 1; /**< output control evtib1 latch event enable */
        unsigned int rg_oc_laten_evtb2 : 1; /**< output control evtib2 latch event enable */
        unsigned int reserved55 : 20;
    } BIT;
} volatile OC_LAT_EN_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_oca_evtio_inc : 3; /**< channel A output control action at evtio increase */
        unsigned int rg_oca_evta1_inc : 3; /**< channel A output control action at evta1 increase */
        unsigned int rg_oca_evta2_inc : 3; /**< channel A output control action at evta2 increase */
        unsigned int rg_oca_evtb1_inc : 3; /**< channel A output control action at evtb1 increase */
        unsigned int rg_oca_evtb2_inc : 3; /**< channel A output control action at evtb2 increase */
        unsigned int reserved56 : 1;
        unsigned int rg_oca_evtio_dec : 3; /**< channel A output control action at evtio decrease */
        unsigned int rg_oca_evta1_dec : 3; /**< channel A output control action at evta1 decrease */
        unsigned int rg_oca_evta2_dec : 3; /**< channel A output control action at evta2 decrease */
        unsigned int rg_oca_evtb1_dec : 3; /**< channel A output control action at evtb1 decrease */
        unsigned int rg_oca_evtb2_dec : 3; /**< channel A output control action at evtb2 decrease */
        unsigned int reserved57 : 1;
    } BIT;
} volatile OC_ACT_A_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_ocb_evtio_inc : 3; /**< channel B output control action at evtio increase */
        unsigned int rg_ocb_evta1_inc : 3; /**< channel B output control action at evta1 increase */
        unsigned int rg_ocb_evta2_inc : 3; /**< channel B output control action at evta2 increase */
        unsigned int rg_ocb_evtb1_inc : 3; /**< channel B output control action at evtb1 increase */
        unsigned int rg_ocb_evtb2_inc : 3; /**< channel B output control action at evtb2 increase */
        unsigned int reserved58 : 1;
        unsigned int rg_ocb_evtio_dec : 3; /**< channel B output control action at evtio decrease */
        unsigned int rg_ocb_evta1_dec : 3; /**< channel B output control action at evta1 decrease */
        unsigned int rg_ocb_evta2_dec : 3; /**< channel B output control action at evta2 decrease */
        unsigned int rg_ocb_evtb1_dec : 3; /**< channel B output control action at evtb1 decrease */
        unsigned int rg_ocb_evtb2_dec : 3; /**< channel B output control action at evtb2 decrease */
        unsigned int reserved59 : 1;
    } BIT;
} volatile OC_ACT_B_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int ro_oc_flag_evt1 : 1; /**< output control evtio1 flag */
        unsigned int ro_oc_flag_evt2 : 1; /**< output control evtio2 flag */
        unsigned int ro_oc_flag_evt3 : 1; /**< output control evtio3 flag */
        unsigned int reserved60 : 1;
        unsigned int ro_oc_flag_evts1 : 1; /**< output control evts1 flag */
        unsigned int ro_oc_flag_evts2 : 1; /**< output control evts2 flag */
        unsigned int ro_oc_flag_evts3 : 1; /**< output control evts3 flag */
        unsigned int reserved61 : 1;
        unsigned int ro_oc_flag_evta1 : 1; /**< output control evta1 flag */
        unsigned int ro_oc_flag_evta2 : 1; /**< output control evta2 flag */
        unsigned int ro_oc_flag_evtb1 : 1; /**< output control evtb1 flag */
        unsigned int ro_oc_flag_evtb2 : 1; /**< output control evtb2 flag */
        unsigned int reserved62 : 3;
        unsigned int ro_int_flag_evt : 1; /**< output control event interrupt flag */
        unsigned int rg_oc_clr_evt1 : 1;  /**< output control evtio1 clear bit */
        unsigned int rg_oc_clr_evt2 : 1;  /**< output control evtio2 clear bit */
        unsigned int rg_oc_clr_evt3 : 1;  /**< output control evtio3 clear bit */
        unsigned int reserved63 : 1;
        unsigned int rg_oc_clr_evts1 : 1; /**< output control evts1 clear bit */
        unsigned int rg_oc_clr_evts2 : 1; /**< output control evts2 clear bit */
        unsigned int rg_oc_clr_evts3 : 1; /**< output control evts3 clear bit */
        unsigned int reserved64 : 1;
        unsigned int rg_oc_clr_evta1 : 1; /**< output control evta1 clear bit */
        unsigned int rg_oc_clr_evta2 : 1; /**< output control evta2 clear bit */
        unsigned int rg_oc_clr_evtb1 : 1; /**< output control evtb1 clear bit */
        unsigned int rg_oc_clr_evtb2 : 1; /**< output control evtb2 clear bit */
        unsigned int reserved65 : 3;
        unsigned int rg_int_clr_evt : 1; /**< output control event interrupt clear bit */
    } BIT;
} volatile OC_EVT_FLAG_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_oc_clr_zroen_evt1 : 1; /**< enable clear evtio1 at zero */
        unsigned int rg_oc_clr_zroen_evt2 : 1; /**< enable clear evtio2 at zero */
        unsigned int rg_oc_clr_zroen_evt3 : 1; /**< enable clear evtio3 at zero */
        unsigned int reserved66 : 1;
        unsigned int rg_oc_clr_zroen_evts1 : 1; /**< enable clear evts1 at zero */
        unsigned int rg_oc_clr_zroen_evts2 : 1; /**< enable clear evts2 at zero */
        unsigned int rg_oc_clr_zroen_evts3 : 1; /**< enable clear evts3 at zero */
        unsigned int reserved67 : 1;
        unsigned int rg_oc_clr_zroen_evta1 : 1; /**< enable clear evta1 at zero */
        unsigned int rg_oc_clr_zroen_evta2 : 1; /**< enable clear evta2 at zero */
        unsigned int rg_oc_clr_zroen_evtb1 : 1; /**< enable clear evtb1 at zero */
        unsigned int rg_oc_clr_zroen_evtb2 : 1; /**< enable clear evtb2 at zero */
        unsigned int reserved68 : 4;
        unsigned int rg_oc_clr_prden_evt1 : 1; /**< enable clear evtio1 at period */
        unsigned int rg_oc_clr_prden_evt2 : 1; /**< enable clear evtio2 at period */
        unsigned int rg_oc_clr_prden_evt3 : 1; /**< enable clear evtio3 at period */
        unsigned int reserved69 : 1;
        unsigned int rg_oc_clr_prden_evts1 : 1; /**< enable clear evts1 at period */
        unsigned int rg_oc_clr_prden_evts2 : 1; /**< enable clear evts2 at period */
        unsigned int rg_oc_clr_prden_evts3 : 1; /**< enable clear evts3 at period */
        unsigned int reserved70 : 1;
        unsigned int rg_oc_clr_prden_evta1 : 1; /**< enable clear evta1 at period */
        unsigned int rg_oc_clr_prden_evta2 : 1; /**< enable clear evta2 at period */
        unsigned int rg_oc_clr_prden_evtb1 : 1; /**< enable clear evtb1 at period */
        unsigned int rg_oc_clr_prden_evtb2 : 1; /**< enable clear evtb2 at period */
        unsigned int reserved71 : 4;
    } BIT;
} volatile OC_PRD_CLR_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_oc_frc_evt1 : 1; /**< force enable evtio1 event */
        unsigned int rg_oc_frc_evt2 : 1; /**< force enable evtio2 event */
        unsigned int rg_oc_frc_evt3 : 1; /**< force enable evtio3 event */
        unsigned int reserved72 : 1;
        unsigned int rg_oc_frc_evts1 : 1; /**< force enable evts1 event */
        unsigned int rg_oc_frc_evts2 : 1; /**< force enable evts2 event */
        unsigned int rg_oc_frc_evts3 : 1; /**< force enable evts3 event */
        unsigned int reserved73 : 1;
        unsigned int rg_oc_frc_evta1 : 1; /**< force enable evta1 event */
        unsigned int rg_oc_frc_evta2 : 1; /**< force enable evta2 event */
        unsigned int rg_oc_frc_evtb1 : 1; /**< force enable evtb1 event */
        unsigned int rg_oc_frc_evtb2 : 1; /**< force enable evtb2 event */
        unsigned int reserved74 : 20;
    } BIT;
} volatile OC_FRC_EVT_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_int_en_evt1 : 1; /**< enable evtio1 intterrupt */
        unsigned int rg_int_en_evt2 : 1; /**< enable evtio2 intterrupt */
        unsigned int rg_int_en_evt3 : 1; /**< enable evtio3 intterrupt */
        unsigned int reserved75 : 1;
        unsigned int rg_int_en_evts1 : 1; /**< enable evts1 intterrupt */
        unsigned int rg_int_en_evts2 : 1; /**< enable evts2 intterrupt */
        unsigned int rg_int_en_evts3 : 1; /**< enable evts3 intterrupt */
        unsigned int reserved76 : 1;
        unsigned int rg_int_en_evta1 : 1; /**< enable evta1 intterrupt */
        unsigned int rg_int_en_evta2 : 1; /**< enable evta2 intterrupt */
        unsigned int rg_int_en_evtb1 : 1; /**< enable evtb1 intterrupt */
        unsigned int rg_int_en_evtb2 : 1; /**< enable evtb2 intterrupt */
        unsigned int reserved77 : 20;
    } BIT;
} volatile INT_EVT_EN_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_int_en_tmr : 1; /**< enable timer interrupt */
        unsigned int reserved0 : 31;
    } BIT;
} volatile INT_TMR_EN_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int ro_int_flag_tmr : 1; /**< timer interrupt clear bit */
        unsigned int reserved79 : 15;
        unsigned int rg_int_clr_tmr : 1; /**< timer interrupt flag */
        unsigned int reserved80 : 15;
    } BIT;
} volatile INT_TMR_FLAG_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_int_tmr_sel : 4; /**< timer interrupt source select */
        unsigned int reserved81 : 28;
    } BIT;
} volatile INT_TMR_SEL_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_int_prsc_prd : 4; /**< timer interrupt scale ratio */
        unsigned int reserved82 : 4;
        unsigned int ro_int_prsc_cnt : 4; /**< timer interrupt scale ratio value read register */
        unsigned int reserved83 : 4;
        unsigned int rg_int_prsc_phs : 4; /**< timer interrupt scale ratio phase value */
        unsigned int reserved84 : 4;
        unsigned int rg_int_prsc_synen : 1; /**< timer interrupt scale ratio phase value  */
        unsigned int rg_int_prsc_frc : 1;
        unsigned int reserved85 : 6;
    } BIT;
} volatile INT_PRSC_CFG_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_csa_tmr_sel : 4; /**< timer condition to trigger adc sample through SOCA */
        unsigned int reserved86 : 12;
        unsigned int rg_csa_en_cs : 1;   /**< timer trigger adc sample through SOCA enable */
        unsigned int reserved87 : 15;
    } BIT;
} volatile CS_TMR_SELA_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_csb_tmr_sel : 4; /**< timer condition to trigger adc sample  through SOCB */
        unsigned int reserved88 : 12;
        unsigned int rg_csb_en_cs : 1;   /**< timer trigger adc sample through SOCB enable */
        unsigned int reserved89 : 15;
    } BIT;
} volatile CS_TMR_SELB_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_csa_prsc_prd : 4;   /**< trigger adc scale ratio through SOCB */
        unsigned int reserved90 : 12;
        unsigned int rg_csa_prsc_phs : 4;   /**< trigger adc scale ratio phase value through SOCB */
        unsigned int reserved91 : 4;
        unsigned int rg_csa_prsc_synen : 1; /**< trigger adc scale ratio phase value sync enable through SOCB */
        unsigned int rg_csa_prsc_frc : 1;   /**< trigger adc scale ratio phase value force enable through SOCB */
        unsigned int reserved92 : 6;
    } BIT;
} volatile CS_PRSCA_CFG_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_csb_prsc_prd : 4;   /**< trigger adc scale ratio through SOCB */
        unsigned int reserved93 : 12;
        unsigned int rg_csb_prsc_phs : 4;   /**< trigger adc scale ratio phase value through SOCB */
        unsigned int reserved94 : 4;
        unsigned int rg_csb_prsc_synen : 1; /**< trigger adc scale ratio phase value sync enable through SOCB */
        unsigned int rg_csb_prsc_frc : 1;   /**< trigger adc scale ratio phase value force enable through SOCB */
        unsigned int reserved95 : 6;
    } BIT;
} volatile CS_PRSCB_CFG_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int ro_csa_flag : 1; /**< SOCA adc start sample flag */
        unsigned int ro_csb_flag : 1; /**< SOCB adc start sample flag */
        unsigned int reserved96 : 14;
        unsigned int rg_csa_clr_flag : 1; /**< SOCA adc start sample flag clear bit */
        unsigned int rg_csb_clr_flag : 1; /**< SOCB adc start sample flag clear bit */
        unsigned int reserved97 : 14;
    } BIT;
} volatile CS_FLAG_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_dma_breq_sel : 2; /**< DMA Burst request source select */
        unsigned int rg_dma_sreq_sel : 2; /**< DMA single request source select */
        unsigned int reserved98 : 28;
    } BIT;
} volatile CS_DMA_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_evtio1_psel : 2; /**< evtio1's polarity */
        unsigned int rg_evtio2_psel : 2; /**< evtio2's polarity */
        unsigned int rg_evtio3_psel : 2; /**< evtio3's polarity */
        unsigned int rg_evtio4_psel : 2; /**< evtio4's polarity */
        unsigned int rg_evtio5_psel : 2; /**< evtio5's polarity */
        unsigned int reserved99 : 6;
        unsigned int rg_evtsys1_psel : 2; /**< evts1's polarity */
        unsigned int rg_evtsys2_psel : 2; /**< evts2's polarity */
        unsigned int rg_evtsys3_psel : 2; /**< evts3's polarity */
        unsigned int reserved100 : 10;
    } BIT;
} volatile EM_EVTIO_PSEL_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_evtmp1_psel : 2; /**< evtmp1's polarity */
        unsigned int rg_evtmp2_psel : 2; /**< evtmp2's polarity */
        unsigned int rg_evtmp3_psel : 2; /**< evtmp3's polarity */
        unsigned int rg_evtmp4_psel : 2; /**< evtmp4's polarity */
        unsigned int rg_evtmp5_psel : 2; /**< evtmp5's polarity */
        unsigned int rg_evtmp6_psel : 2; /**< evtmp6's polarity */
        unsigned int reserved101 : 20;
    } BIT;
} volatile EM_EVTMP_PSEL_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_em_a1_oren : 9; /**< group A event 1 logic OR source enable */
        unsigned int reserved102 : 7;
        unsigned int rg_em_a2_oren : 9; /**< group A event 2 logic OR source enable */
        unsigned int reserved103 : 7;
    } BIT;
} volatile EM_AOR_EN_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_em_b1_oren : 9; /**< group B event 1 logic OR source enable */
        unsigned int reserved104 : 7;
        unsigned int rg_em_b2_oren : 9; /**< group B event 2 logic OR source enable */
        unsigned int reserved105 : 7;
    } BIT;
} volatile EM_BOR_EN_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_em_a1_sel : 4; /**< group A event 1 source select */
        unsigned int rg_em_a2_sel : 4; /**< group A event 2 source select */
        unsigned int rg_em_b1_sel : 4; /**< group B event 1 source select */
        unsigned int rg_em_b2_sel : 4; /**< group B event 2 source select */
        unsigned int rg_evta1t_sel : 3; /**< evta1t source select */
        unsigned int reserved106 : 1;
        unsigned int rg_evta2t_sel : 3; /**< evta2t source select */
        unsigned int reserved107 : 1;
        unsigned int rg_evtb1t_sel : 3; /**< evtb1t source select */
        unsigned int reserved108 : 1;
        unsigned int rg_evtb2t_sel : 3; /**< evtb2t source select */
        unsigned int reserved109 : 1;
    } BIT;
} volatile EM_MRG_SEL_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_evta1_sel : 1; /**< em_evta1 event source select */
        unsigned int rg_evta2_sel : 1; /**< em_evta2 event source select */
        unsigned int rg_evtb1_sel : 1; /**< em_evtb1 event source select */
        unsigned int rg_evtb2_sel : 1; /**< em_evtb2 event source select */
        unsigned int rg_evtfilt_sel : 2; /**< em_evfilt event source select */
        unsigned int reserved110 : 26;
    } BIT;
} volatile EM_OUT_SEL_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_syni_sel : 4; /**< em_evt_syni source select */
        unsigned int reserved111 : 12;
        unsigned int ro_syni_flag : 1; /**< em_evt_syni event active flag */
        unsigned int reserved112 : 3;
        unsigned int rg_syni_clr : 1; /**< em_evt_syni event active flag clear bit */
        unsigned int reserved113 : 11;
    } BIT;
} volatile SYNI_CFG_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_syncnt_a1en : 1; /**< TC value sync at em_evta1_pulse */
        unsigned int rg_syncnt_b1en : 1; /**< TC value sync at em_evtb1_pulse */
        unsigned int rg_syncnt_synien : 1; /**< TC value sync at em_synci_pulse */
        unsigned int reserved114 : 29;
    } BIT;
} volatile SYNCNT_CFG_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_syno_zroen : 1; /**< sync out at zero enable */
        unsigned int rg_syno_prden : 1; /**< sync out at period enable */
        unsigned int rg_syno_a1en : 1; /**< sync out at a1 enable */
        unsigned int rg_syno_b1en : 1; /**< sync out at b1 enable */
        unsigned int reserved115 : 1;
        unsigned int rg_syno_refben : 1; /**< sync out at reference B match enable */
        unsigned int rg_syno_refcen : 1; /**< sync out at reference C match enable */
        unsigned int rg_syno_refden : 1; /**< sync out at reference D match enable */
        unsigned int rg_mode_syno : 1; /**< sync out mode select */
        unsigned int rg_latset_sel : 1; /**< latch condition */
        unsigned int rg_latset_otsyn : 1; /**< control a sync out latch bit enable */
        unsigned int reserved116 : 21;
    } BIT;
} volatile SYNO_CFG_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_gld_zroen : 1; /**< enable global load when count zero */
        unsigned int rg_gld_prden : 1; /**< enable global load when count period */
        unsigned int rg_gld_cntsynen : 1; /**< enable global load when em_cnt_syn enable */
        unsigned int reserved117 : 5;
        unsigned int rg_gld_prsc_prd : 4; /**< global load scale ratio */
        unsigned int rg_mode_gld : 1; /**< buffer global load mode select */
        unsigned int reserved118 : 3;
        unsigned int rg_latset_otgld : 1; /**< control a global latch bit enable */
        unsigned int reserved119 : 15;
    } BIT;
} volatile GLB_LOAD_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int tc_prd_ld_sts : 1;  /**< count period buffer status */
        unsigned int tc_refa_ld_sts : 1; /**< reference A buffer status */
        unsigned int tc_refb_ld_sts : 1; /**< reference B buffer status */
        unsigned int tc_refc_ld_sts : 1; /**< reference C buffer status */
        unsigned int tc_refd_ld_sts : 1; /**< reference D buffer status */
        unsigned int reserved120 : 3;
        unsigned int pg_act_a_ld_sts : 1; /**< channel A action buffer status */
        unsigned int pg_act_b_ld_sts : 1; /**< channel B buffer status */
        unsigned int pg_out_frc_ld_sts : 1; /**< PG putput force buffer status */
        unsigned int reserved121 : 1;
        unsigned int dg_red_ld_sts : 1; /**< DG rising edge buffer status */
        unsigned int dg_fed_ld_sts : 1; /**< DG falling edge buffer status */
        unsigned int dg_cfg_ld_sts : 1; /**< DG config buffer status */
        unsigned int reserved122 : 1;
        unsigned int tc_mwdrefa_ld_sts : 1;
        unsigned int tc_mwdrefb_ld_sts : 1;
        unsigned int tc_mwd_act_ld_sts : 1;
        unsigned int reserved123 : 13;
    } BIT;
} volatile LOAD_STS_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int rg_syncnt_frc : 1; /**< force an em_cnt_syn event */
        unsigned int reserved124 : 3;
        unsigned int rg_syno_frc : 1; /**< force an apt_syno event */
        unsigned int reserved125 : 3;
        unsigned int reserved126 : 4;
        unsigned int rg_gld_frc : 1; /**< force an em_glb_ld event*/
        unsigned int reserved127 : 3;
        unsigned int reserved128 : 4;
        unsigned int rg_csa_syn_frc : 1; /**< force a SOCA trigger */
        unsigned int rg_csb_syn_frc : 1; /**< force a SOCB trigger */
        unsigned int reserved129 : 2;
        unsigned int rg_int_syn_frc : 1; /**< force timer interrupt scale load sync init value */
        unsigned int reserved130 : 3;
        unsigned int rg_synpg_frc : 1; /**< force create waveform buffer indepent load trigger event */
        unsigned int reserved131 : 3;
    } BIT;
} volatile SYN_FRC_REG;

/**
  * @brief APT registers definition structure.
  */
typedef struct {
    VER_INFO_REG        VER_INFO;       /**< VER_INFO_REG. Offset address 0x00000000U. */
    unsigned int        reserved0[3];
    TC_MODE_REG         TC_MODE;        /**< TC_MODE_REG. Offset address 0x00000010U. */
    TC_PHS_REG          TC_PHS;         /**< TC_PHS_REG. Offset address 0x00000014U. */
    TC_OVRID_REG        TC_OVRID;       /**< TC_OVRID_REG. Offset address 0x00000018U. */
    unsigned int        reserved1;
    TC_PRD_REG          TC_PRD;         /**< TC_PRD_REG. Offset address 0x00000020U. */
    unsigned int        reserved2[3];
    TC_REFA_REG         TC_REFA;        /**< TC_REFA_REG. Offset address 0x00000030U. */
    TC_REFB_REG         TC_REFB;        /**< TC_REFB_REG. Offset address 0x00000034U. */
    TC_REFC_REG         TC_REFC;        /**< TC_REFC_REG. Offset address 0x00000038U. */
    TC_REFD_REG         TC_REFD;        /**< TC_REFD_REG. Offset address 0x0000003CU. */
    unsigned int        reserved3[4];
    TC_BUF_EN_REG       TC_BUF_EN;      /**< TC_BUF_EN_REG. Offset address 0x00000040U. */
    TC_PRD_LOAD_REG     TC_PRD_LOAD;    /**< TC_PRD_LOAD_REG. Offset address 0x00000050U. */
    TC_REF_LOAD_REG     TC_REF_LOAD;    /**< TC_REF_LOAD_REG. Offset address 0x00000054U. */
    TC_MWD_EN_REG       TC_MWD_EN;      /**< TC_MWD_EN_REG. Offset address 0x0000005CU. */
    TC_MWDREFA_REG      TC_MWDREFA;     /**< TC_MWDREFA_REG. Offset address 0x00000060U. */
    TC_MWDREFB_REG      TC_MWDREFB;     /**< TC_MWDREFB_REG. Offset address 0x00000064U. */
    TC_MWD_ACT_REG      TC_MWD_ACT;     /**< TC_MWD_ACT_REG. Offset address 0x00000068U. */
    TC_MWD_BUF_EN_REG   TC_MWD_BUF_EN;  /**< TC_MWD_BUF_EN_REG. Offset address 0x0000006cU. */
    TC_MWD_LOAD_REG     TC_MWD_LOAD;    /**< TC_MWD_LOAD_REG. Offset address 0x00000070U. */
    TC_STS_REG          TC_STS;         /**< TC_STS_REG. Offset address 0x00000060U. */
    unsigned int        reserved4[34];
    PG_ACT_A_REG        PG_ACT_A;       /**< PG_ACT_A_REG. Offset address 0x00000100U. */
    PG_ACT_B_REG        PG_ACT_B;       /**< PG_ACT_B_REG. Offset address 0x00000104U. */
    unsigned int        reserved5[2];
    PG_ACT_FRC_REG      PG_ACT_FRC;      /**< PG_ACT_FRC_REG. Offset address 0x00000110U. */
    PG_OUT_FRC_REG      PG_OUT_FRC;     /**< PG_OUT_FRC_REG. Offset address 0x00000114U. */
    unsigned int        reserved6[2];
    PG_BUF_EN_REG       PG_BUF_EN;      /**< PG_BUF_EN_REG. Offset address 0x00000120U. */
    unsigned int        reserved7[3];
    PG_ACT_LD_REG       PG_ACT_LD;      /**< PG_ACT_LD_REG. Offset address 0x00000130U. */
    unsigned int        reserved8[3];
    PG_EVTC_SEL_REG     PG_EVTC_SEL;    /**< PG_EVTC_SEL_REG. Offset address 0x00000140U. */
    unsigned int        reserved9[47];
    DG_RED_REG          DG_RED;         /**< DG_RED_REG. Offset address 0x00000200U. */
    DG_FED_REG          DG_FED;         /**< DG_FED_REG. Offset address 0x00000204U. */
    DG_CFG_REG          DG_CFG;         /**< DG_CFG_REG. Offset address 0x00000208U. */
    unsigned int        reserved10;
    DG_BUF_EN_REG       DG_BUF_EN;      /**< DG_BUF_EN_REG. Offset address 0x00000210U. */
    DG_BUF_LOAD_REG     DG_BUF_LOAD;    /**< DG_BUF_LOAD_REG. Offset address 0x00000214U. */
    unsigned int        reserved11[58];
    OC_MODE_REG         OC_MODE;        /**< OC_MODE_REG. Offset address 0x00000300U. */
    OC_LAT_EN_REG       OC_LAT_EN;      /**< OC_LAT_EN_REG. Offset address 0x00000304U. */
    unsigned int        reserved12[2];
    OC_ACT_A_REG        OC_ACT_A;       /**< OC_ACT_A_REG. Offset address 0x00000310U. */
    OC_ACT_B_REG        OC_ACT_B;       /**< OC_ACT_B_REG. Offset address 0x00000314U. */
    unsigned int        reserved13[2];
    OC_EVT_FLAG_REG     OC_EVT_FLAG;    /**< OC_EVT_FLAG_REG. Offset address 0x00000320U. */
    OC_PRD_CLR_REG      OC_PRD_CLR;     /**< OC_PRD_CLR_REG. Offset address 0x00000324U. */
    unsigned int        reserved14[2];
    OC_FRC_EVT_REG      OC_FRC_EVT;     /**< OC_FRC_EVT_REG. Offset address 0x00000330U. */
    unsigned int        reserved15[55];
    INT_EVT_EN_REG      INT_EVT_EN;     /**< INT_EVT_EN_REG. Offset address 0x00000410U. */
    INT_TMR_EN_REG      INT_TMR_EN;     /**< INT_TMR_EN_REG. Offset address 0x00000414U. */
    unsigned int        reserved16[2];
    INT_TMR_FLAG_REG    INT_TMR_FLAG;   /**< INT_TMR_FLAG_REG. Offset address 0x00000420U. */
    INT_TMR_SEL_REG     INT_TMR_SEL;    /**< INT_TMR_SEL_REG. Offset address 0x00000424U. */
    INT_PRSC_CFG_REG    INT_PRSC_CFG;   /**< INT_PRSC_CFG_REG. Offset address 0x00000428U. */
    unsigned int        reserved17[53];
    CS_TMR_SELA_REG     CS_TMR_SELA;    /**< CS_TMR_SELA_REG. Offset address 0x00000500U. */
    CS_TMR_SELB_REG     CS_TMR_SELB;    /**< CS_TMR_SELB_REG. Offset address 0x00000504U. */
    CS_PRSCA_CFG_REG    CS_PRSCA_CFG;   /**< CS_PRSCA_CFG_REG. Offset address 0x00000508U. */
    CS_PRSCB_CFG_REG    CS_PRSCB_CFG;   /**< CS_PRSCB_CFG_REG. Offset address 0x0000050CU. */
    CS_FLAG_REG         CS_FLAG;        /**< CS_FLAG_REG. Offset address 0x00000510U. */
    unsigned int        reserved18[3];
    CS_DMA_REG          CS_DMA;         /**< CS_DMA_REG. Offset address 0x00000520U. */
    unsigned int        reserved19[55];
    EM_EVTIO_PSEL_REG   EM_EVTIO_PSEL;  /**< EM_EVTIO_PSEL_REG. Offset address 0x00000600U. */
    EM_EVTMP_PSEL_REG   EM_EVTMP_PSEL;  /**< EM_EVTMP_PSEL_REG. Offset address 0x00000604U. */
    EM_AOR_EN_REG       EM_AOR_EN;      /**< EM_AOR_EN_REG. Offset address 0x00000608U. */
    EM_BOR_EN_REG       EM_BOR_EN;      /**< EM_BOR_EN_REG. Offset address 0x0000060CU. */
    EM_MRG_SEL_REG      EM_MRG_SEL;     /**< EM_MRG_SEL_REG. Offset address 0x00000610U. */
    EM_OUT_SEL_REG      EM_OUT_SEL;     /**< EM_OUT_SEL_REG. Offset address 0x00000614U. */
    unsigned int        reserved20[58];
    SYNI_CFG_REG        SYNI_CFG;       /**< SYNI_CFG_REG. Offset address 0x00000700U. */
    SYNCNT_CFG_REG      SYNCNT_CFG;     /**< SYNCNT_CFG_REG. Offset address 0x00000704U. */
    SYNO_CFG_REG        SYNO_CFG;       /**< SYNO_CFG_REG. Offset address 0x00000708U. */
    unsigned int        reserved21;
    GLB_LOAD_REG        GLB_LOAD;       /**< GLB_LOAD_REG. Offset address 0x000000710U. */
    unsigned int        reserved22[3];
    LOAD_STS_REG        LOAD_STS;       /**< LOAD_STS_REG. Offset address 0x000000720U. */
    unsigned int        reserved23[3];
    SYN_FRC_REG         SYN_FRC;        /**< SYN_FRC_REG. Offset address 0x00000730U. */
} volatile APT_RegStruct;

/**
  * @brief Set the emulation stop mode of APT module.
  * @param aptx APT register base address.
  * @param emuMode Emulation stop mode.
  * @retval None.
  */
static inline void DCL_APT_SetEmulationMode(APT_RegStruct *aptx, APT_EmulationMode emuMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(emuMode <= APT_EMULATION_STOP_APT);
    aptx->TC_MODE.BIT.rg_emu_stop = emuMode;
}

/**
  * @brief Set the time-base divider factor.
  * @param aptx APT register base address.
  * @param divFactor Time-base divider factor.
  * @retval None.
  */
static inline void DCL_APT_SetDividerFactor(APT_RegStruct *aptx, unsigned short divFactor)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(divFactor <= DIVIDER_FACTOR_MAX);
    aptx->TC_MODE.BIT.rg_div_fac = divFactor;
}

/**
  * @brief Get the time-base divider factor.
  * @param aptx APT register base address.
  * @retval unsigned short: time-base divider factor.
  */
static inline unsigned short DCL_APT_GetDividerFactor(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    return (aptx->TC_MODE.BIT.rg_div_fac);
}

/**
  * @brief Set the count mode of time-base counter.
  * @param aptx APT register base address.
  * @param cntMode Count mode.
  * @retval None.
  */
static inline void DCL_APT_SetTimeBaseCountMode(APT_RegStruct *aptx, APT_CountMode cntMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(cntMode <= APT_COUNT_MODE_FREEZE);
    aptx->TC_MODE.BIT.rg_cnt_mode = cntMode;
}

/**
  * @brief Set the period of time-base counter.
  * @param aptx APT register base address.
  * @param periodCnt Time-base counter period.
  * @retval None.
  */
static inline void DCL_APT_SetTimeBasePeriod(APT_RegStruct *aptx, unsigned short periodCnt)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->TC_PRD.BIT.rg_cnt_prd = periodCnt;
}

/**
  * @brief Get the period of time-base counter.
  * @param aptx APT register base address.
  * @retval unsigned short: time-base counter period
  */
static inline unsigned short DCL_APT_GetTimeBasePeriod(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    return (aptx->TC_PRD.BIT.rg_cnt_prd);
}

/**
  * @brief Set the count mode of slave APT module after synchronization.
  * @param aptx APT register base address.
  * @param syncCntMode Count mode after synchronization.
  * @retval None.
  */
static inline void DCL_APT_SetCountModeAfterSync(APT_RegStruct *aptx, APT_SyncCountMode syncCntMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(syncCntMode >= APT_COUNT_MODE_AFTER_SYNC_DOWN);
    APT_PARAM_CHECK_NO_RET(syncCntMode <= APT_COUNT_MODE_AFTER_SYNC_UP);
    aptx->TC_PHS.BIT.rg_cnt_dir = syncCntMode;
}

/**
  * @brief Set the counter phase after synchronization.
  * @param aptx APT register base address.
  * @param cntPhase Counter phase after synchronization.
  * @retval None.
  */
static inline void DCL_APT_SetCounterPhase(APT_RegStruct *aptx, unsigned short cntPhase)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    TC_PHS_REG tmp = aptx->TC_PHS;
    tmp.BIT.rg_cnt_phs = cntPhase;
    aptx->TC_PHS = tmp;
}

/**
  * @brief Set the software override value of time-base counter.
  * @param aptx APT register base address.
  * @param cntOvrid Software override value of time-base counter.
  * @retval None.
  */
static inline void DCL_APT_SetCounterOverride(APT_RegStruct *aptx, unsigned short cntOvrid)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    TC_OVRID_REG tmp = aptx->TC_OVRID;
    tmp.BIT.rg_cnt_ovrid = cntOvrid;
    aptx->TC_OVRID = tmp;
}

/**
  * @brief Force software override on time-base divider and counter.
  * @param aptx APT register base address.
  * @retval None.
  */
static inline void DCL_APT_ForceOverride(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->TC_OVRID.BIT.rg_cnt_ovrid_en = BASE_CFG_SET;
}

/**
  * @brief Set the count compare reference value of time-base counter.
  * @param aptx APT register base address.
  * @param ref Count compare reference.
  * @param cntCmp Count compare reference value of counter.
  * @retval None.
  */
static inline void DCL_APT_SetCounterCompare(APT_RegStruct *aptx, APT_CompareRef ref, unsigned short cntCmp)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ref >= APT_COMPARE_REFERENCE_A);
    APT_PARAM_CHECK_NO_RET(ref <= APT_COMPARE_REFERENCE_D);
    TC_REFA_REG tmpA;
    TC_REFB_REG tmpB;
    TC_REFC_REG tmpC;
    TC_REFD_REG tmpD;
    switch (ref) {
        case APT_COMPARE_REFERENCE_A:
            tmpA = aptx->TC_REFA;
            tmpA.BIT.rg_cnt_refa = cntCmp;
            aptx->TC_REFA = tmpA;
            break;
        case APT_COMPARE_REFERENCE_B:
            tmpB = aptx->TC_REFB;
            tmpB.BIT.rg_cnt_refb = cntCmp;
            aptx->TC_REFB = tmpB;
            break;
        case APT_COMPARE_REFERENCE_C:
            tmpC = aptx->TC_REFC;
            tmpC.BIT.rg_cnt_refc = cntCmp;
            aptx->TC_REFC = tmpC;
            break;
        case APT_COMPARE_REFERENCE_D:
            tmpD = aptx->TC_REFD;
            tmpD.BIT.rg_cnt_refd = cntCmp;
            aptx->TC_REFD = tmpD;
            break;
        default:
            break;
    }
}

/**
  * @brief Get the count compare reference value of time-base counter.
  * @param aptx APT register base address.
  * @param ref Count compare reference.
  * @retval unsigned short: Count compare reference value of counter.
  */
static inline unsigned short DCL_APT_GetCounterCompare(APT_RegStruct *aptx, APT_CompareRef ref)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_WITH_RET(ref >= APT_COMPARE_REFERENCE_A, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(ref <= APT_COMPARE_REFERENCE_D, BASE_STATUS_ERROR);
    switch (ref) {
        case APT_COMPARE_REFERENCE_A:
            return (aptx->TC_REFA.BIT.rg_cnt_refa);
        case APT_COMPARE_REFERENCE_B:
            return (aptx->TC_REFB.BIT.rg_cnt_refb);
        case APT_COMPARE_REFERENCE_C:
            return (aptx->TC_REFC.BIT.rg_cnt_refc);
        case APT_COMPARE_REFERENCE_D:
            return (aptx->TC_REFD.BIT.rg_cnt_refd);
        default:
            return 0;
    }
}

/**
  * @brief Set the buffer load mode of time-base period register.
  * @param aptx APT register base address.
  * @param prdLoadMode Buffer load mode of time-base period register.
  * @retval None.
  */
static inline void DCL_APT_SetPeriodLoadMode(APT_RegStruct *aptx, APT_BufferLoadMode prdLoadMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(prdLoadMode >= APT_BUFFER_DISABLE);
    APT_PARAM_CHECK_NO_RET(prdLoadMode <= APT_BUFFER_GLOBAL_LOAD);
    aptx->TC_BUF_EN.reg &= (~0b11);      /* Clear rg_prd_buf_en and rg_prd_gld_en */
    aptx->TC_BUF_EN.reg |= prdLoadMode;  /* Write rg_prd_buf_en and rg_prd_gld_en */
}

/**
 * @brief Enable the buffer load events of TC_PRD register
 * @param aptx APT register base address.
 * @param loadEvent The buffer load events of TC_PRD register
 *        A logical OR of valid values that can be passed as the loadEvent parameter
 *        Valid values for loadEvent are:
 *            APT_PERIOD_LOAD_EVENT_ZERO      -   When counter value equal to zeor
 *            APT_PERIOD_LOAD_EVENT_A1        -   When combined event A1 is valid
 *            APT_PERIOD_LOAD_EVENT_B1        -   When combined event B1 is valid
 *            APT_PERIOD_LOAD_EVENT_SYNC      -   When synchronization event is valid
 * @retval None.
 */
static inline void DCL_APT_SetPeriodLoadEvent(APT_RegStruct *aptx, unsigned int prdLoadEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->TC_PRD_LOAD.reg = prdLoadEvent;
}

/**
  * @brief Set the buffer load mode of count compare reference register.
  * @param aptx APT register base address.
  * @param ref Count compare reference.
  * @param cmpLoadMode Buffer load mode of count compare reference register.
  * @retval None.
  */
static inline void DCL_APT_SetCompareLoadMode(APT_RegStruct *aptx,
                                              APT_CompareRef ref,
                                              APT_BufferLoadMode cmpLoadMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ref >= APT_COMPARE_REFERENCE_A);
    APT_PARAM_CHECK_NO_RET(ref <= APT_COMPARE_REFERENCE_D);
    APT_PARAM_CHECK_NO_RET(cmpLoadMode >= APT_BUFFER_DISABLE);
    APT_PARAM_CHECK_NO_RET(cmpLoadMode <= APT_BUFFER_GLOBAL_LOAD);
    unsigned int offsetA = 4;                       /* Buffer mode control bits offset of reference A */
    unsigned int tcBufField = 2;                    /* Field width of buffer load mode setting */
    unsigned int offset = offsetA + ref * tcBufField;
    aptx->TC_BUF_EN.reg &= (~(0b11 << offset));     /* Clear rg_refx_gld_en and rg_refx_buf_en */
    aptx->TC_BUF_EN.reg |= (cmpLoadMode << offset); /* Write rg_refx_gld_en and rg_refx_buf_en */
}

/**
 * @brief Enable the buffer load events of TC_REFA, TC_REFB, TC_REFC, TC_REFD register
 * @param aptx APT register base address.
 * @param ref Count compare reference
 * @param loadEvent The buffer load events of TC_REFA, TC_REFB, TC_REFC, TC_REFD register
 *        A logical OR of valid values can be passed as the loadEvent parameter
 *        Valid values for loadEvent are:
 *            APT_COMPARE_LOAD_EVENT_ZERO     -   When counter value equal to zero
 *            APT_COMPARE_LOAD_EVENT_PERIOD   -   When counter value equal to period
 *            APT_COMPARE_LOAD_EVENT_A1       -   When combined event A1 is valid
 *            APT_COMPARE_LOAD_EVENT_B1       -   When combined event B1 is valid
 *            APT_COMPARE_LOAD_EVENT_SYNC     -   When synchronization event is valid
 * @retval None.
 */
static inline void DCL_APT_SetCompareLoadEvent(APT_RegStruct *aptx, APT_CompareRef ref, unsigned int cmpLoadEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ref >= APT_COMPARE_REFERENCE_A);
    APT_PARAM_CHECK_NO_RET(ref <= APT_COMPARE_REFERENCE_D);
    unsigned int refBufField = 8; /* Field width of compare reference load event setting  */
    aptx->TC_REF_LOAD.reg &= (~(0x1F << (ref * refBufField))); /* Clear bit field for load event selection */
    aptx->TC_REF_LOAD.reg |= (cmpLoadEvent << (ref * refBufField));
}

/**
  * @brief Get the value of time-base divider.
  * @param aptx APT register base address.
  * @retval unsigned short: The value of time-base divider value.
  */
static inline unsigned short DCL_APT_GetDividerValue(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    return (aptx->TC_STS.BIT.ro_div_cnt);
}

/**
  * @brief Get the value of time-base counter.
  * @param aptx APT register base address.
  * @retval unsigned short: The value of time-base counter.
  */
static inline unsigned short DCL_APT_GetCounterValue(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    return (aptx->TC_STS.BIT.ro_cnt_val);
}

/**
 * @brief Return time base counter direction
 * @param aptx  APT register base address.
 * @retval unsigned short: The direction of time base counter
 *         Valid return values are:
 *             APT_COUNTER_STATUS_COUNT_DOWN   -   The counter is counting down
 *             APT_COUNTER_STATUS_COUNT_UP     -   The counter is counting up
 */
static inline unsigned short DCL_APT_GetCounterDirection(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    return (aptx->TC_STS.BIT.ro_cnt_dir);
}

/* --------------------------------------------------------------------------------------------- */
/* PWM Generation (PG) submodule Direct Configuration Layer functions -------------------------- */
/* --------------------------------------------------------------------------------------------- */
/**
  * @brief Set PWM waveform action on corresponding event.
  * @param aptx APT register base address.
  * @param channel PWM output channel.
  * @param actEvent PWM waveform action event.
  * @param action PWM waveform action.
  * @retval None.
  */
static inline void DCL_APT_SetPWMAction(APT_RegStruct *aptx,
                                        APT_PWMChannel channel,
                                        APT_PWMActionEvent actEvent,
                                        APT_PWMAction action)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(channel >= APT_PWM_CHANNEL_A);
    APT_PARAM_CHECK_NO_RET(channel <= APT_PWM_CHANNEL_B);
    APT_PARAM_CHECK_NO_RET(actEvent >= APT_PWM_ACTION_ON_TIMEBASE_ZERO);
    APT_PARAM_CHECK_NO_RET(actEvent <= APT_PWM_ACTION_ON_C2_COUNT_DOWN);
    APT_PARAM_CHECK_NO_RET(action <= APT_PWM_ACTION_TOGGLE);
    if (channel == APT_PWM_CHANNEL_A) {
        aptx->PG_ACT_A.reg &= (~(0b11 << actEvent));
        aptx->PG_ACT_A.reg |= (action << actEvent);
    } else if (channel == APT_PWM_CHANNEL_B) {
        aptx->PG_ACT_B.reg &= (~(0b11 << actEvent));
        aptx->PG_ACT_B.reg |= (action << actEvent);
    }
}

/**
 * @brief Select the event source of PWM Generation event C1 or C2.
 *        This function is only used when C1 or C2 event is selected as PWM action event.
 * @param aptx APT register base address.
 * @param channel PWM output channel.
 * @param eventCx The PWM Generation event, should be C1 or C2.
 * @param eventCxSrc The trigger source of PWM Generation event C1 or C2.
 * @retval None.
 */
static inline void DCL_APT_SelectCxEventSource(APT_RegStruct *aptx,
                                               APT_PWMChannel channel,
                                               APT_PGEventCx eventCx,
                                               APT_PGEventCxSrc eventCxSrc)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET((channel >= APT_PWM_CHANNEL_A) && (channel <= APT_PWM_CHANNEL_B));
    APT_PARAM_CHECK_NO_RET(eventCx >= APT_PWM_GENERATION_EVENT_C1);
    APT_PARAM_CHECK_NO_RET(eventCx <= APT_PWM_GENERATION_EVENT_C2);
    APT_PARAM_CHECK_NO_RET(eventCxSrc >= APT_PG_EVT_C_FORBIDDEN);
    APT_PARAM_CHECK_NO_RET(eventCxSrc <= APT_PG_EVT_C_SYNC_IN);
    unsigned int chOffset = 8; /* Bit field offset of PWM output channel */
    unsigned int cxOffset = 4; /* Bit field offset of event Cx */
    aptx->PG_EVTC_SEL.reg &= (~(0b1111 << (channel * chOffset + eventCx * cxOffset)));
    aptx->PG_EVTC_SEL.reg |= eventCxSrc << (channel * chOffset + eventCx * cxOffset);
}

/**
  * @brief Set the buffer load mode of PWM action register.
  * @param aptx APT register base address.
  * @param channel PWM output channel.
  * @param loadMode Buffer load mode.
  * @retval None.
  */
static inline void DCL_APT_SetPWMActionLoadMode(APT_RegStruct *aptx,
                                                APT_PWMChannel channel,
                                                APT_BufferLoadMode loadMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(channel >= APT_PWM_CHANNEL_A);
    APT_PARAM_CHECK_NO_RET(channel <= APT_PWM_CHANNEL_B);
    APT_PARAM_CHECK_NO_RET(loadMode >= APT_BUFFER_DISABLE);
    APT_PARAM_CHECK_NO_RET(loadMode <= APT_BUFFER_GLOBAL_LOAD);
    unsigned int bufFieldWidth = 2; /* Bit field width of buffer load mode setting */
    aptx->PG_BUF_EN.reg &= (~(0b11 << (channel * bufFieldWidth)));  /* Clear rg_actx_gld_en and rg_actx_buf_en */
    aptx->PG_BUF_EN.reg |= (loadMode << (channel * bufFieldWidth)); /* Write rg_actx_gld_en and rg_actx_buf_en */
}

/**
 * @brief Enable the buffer load events of PG_ACT_A or PG_ACT_B register
 * @param aptx APT register base address.
 * @param channel PWM output channel.
 * @param loadEvent The buffer load events of PG_ACT_A or PG_ACT_B register
 *        A logical OR of valid values can be passed as the loadEvent parameter
 *        Valid values for loadEvent are:
 *            APT_ACTION_LOAD_EVENT_ZERO      -   When counter value equal to zero
 *            APT_ACTION_LOAD_EVENT_PERIOD    -   When counter value equal to period
 *            APT_ACTION_LOAD_EVENT_A1        -   When combined event A1 is valid
 *            APT_ACTION_LOAD_EVENT_B1        -   When combined event B1 is valid
 *            APT_ACTION_LOAD_EVENT_SYNC      -   When synchronization event is valid
 * @retval None.
 */
static inline void DCL_APT_SetPWMActionLoadEvent(APT_RegStruct *aptx,
                                                 APT_PWMChannel channel,
                                                 unsigned int loadEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(channel >= APT_PWM_CHANNEL_A);
    APT_PARAM_CHECK_NO_RET(channel <= APT_PWM_CHANNEL_B);
    unsigned int actBufField = 8; /* Field width of PWM action load event setting  */
    aptx->PG_ACT_LD.reg &= (~(0x1F << (channel * actBufField)));
    aptx->PG_ACT_LD.reg |= (loadEvent << (channel * actBufField));
}

/**
  * @brief Set the PWM waveform action on one-shot action software event.
  * @param aptx APT register base address.
  * @param channel PWM output channel.
  * @param action PWM waveform action.
  * @retval None.
  */
static inline void DCL_APT_SetSwOneShotPWMAction(APT_RegStruct *aptx, APT_PWMChannel channel, APT_PWMAction action)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(channel >= APT_PWM_CHANNEL_A);
    APT_PARAM_CHECK_NO_RET(channel <= APT_PWM_CHANNEL_B);
    APT_PARAM_CHECK_NO_RET(action >= APT_PWM_ACTION_HOLD);
    APT_PARAM_CHECK_NO_RET(action <= APT_PWM_ACTION_TOGGLE);
    if (channel == APT_PWM_CHANNEL_A) {
        aptx->PG_ACT_FRC.BIT.rg_pga_act_evt_frc = action;
    } else if (channel == APT_PWM_CHANNEL_B) {
        aptx->PG_ACT_FRC.BIT.rg_pgb_act_evt_frc = action;
    }
}

/**
  * @brief Force one-shot software event.
  * @param aptx APT register base address.
  * @param channel PWM output channel.
  * @retval None.
  */
static inline void DCL_APT_ForceSwOneShotPWMAction(APT_RegStruct *aptx, APT_PWMChannel channel)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(channel >= APT_PWM_CHANNEL_A);
    APT_PARAM_CHECK_NO_RET(channel <= APT_PWM_CHANNEL_B);
    if (channel == APT_PWM_CHANNEL_A) {
        aptx->PG_ACT_FRC.BIT.rg_pga_evt_frc = BASE_CFG_SET;
    } else if (channel == APT_PWM_CHANNEL_B) {
        aptx->PG_ACT_FRC.BIT.rg_pgb_evt_frc = BASE_CFG_SET;
    }
}

/**
  * @brief Set the PWM waveform action on continuous action software event.
  * @param aptx APT register base address.
  * @param channel PWM output channel.
  * @param action PWM waveform action
  * @retval None.
  */
static inline void DCL_APT_SetSwContPWMAction(APT_RegStruct *aptx, APT_PWMChannel channel, APT_PWMContAction action)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(channel >= APT_PWM_CHANNEL_A);
    APT_PARAM_CHECK_NO_RET(channel <= APT_PWM_CHANNEL_B);
    APT_PARAM_CHECK_NO_RET(action >= APT_PWM_CONTINUOUS_ACTION_HOLD);
    APT_PARAM_CHECK_NO_RET(action <= APT_PWM_CONTINUOUS_ACTION_HIGH);
    if (channel == APT_PWM_CHANNEL_A) {
        aptx->PG_OUT_FRC.BIT.rg_pga_frc_act = action;
    } else if (channel == APT_PWM_CHANNEL_B) {
        aptx->PG_OUT_FRC.BIT.rg_pgb_frc_act = action;
    }
}

static void APT_ForcePWMAOutputLow(APT_RegStruct *aptx)
{
    unsigned int risingOutSelect = aptx->DG_CFG.BIT.rg_dg_red_osel;
    unsigned int fallingOutSelect = aptx->DG_CFG.BIT.rg_dg_fed_osel;
    unsigned int risingInSelect = aptx->DG_CFG.BIT.rg_dg_red_isel;
    unsigned int fallingInSelect = aptx->DG_CFG.BIT.rg_dg_fed_isel;
    /* Enable force output. */
    aptx->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_ENABLE;
    /* if PWMA invert */
    if (((risingOutSelect == APT_DB_RED_OUTPUT_INVERT) && (risingInSelect == APT_DB_RED_INPUT_PWM_A)) || \
         ((fallingOutSelect == APT_DB_FED_OUTPUT_INVERT) && (fallingInSelect == APT_DB_FED_INPUT_PWM_A))) {
        aptx->PG_OUT_FRC.BIT.rg_pga_frc_act = APT_PWM_CONTINUOUS_ACTION_HIGH; /* if invert, set high */
    } else { /* if PWMA not invert */
        aptx->PG_OUT_FRC.BIT.rg_pga_frc_act = APT_PWM_CONTINUOUS_ACTION_LOW; /* if not invert, set low */
    }
    return;
}

static void APT_ForcePWMBOutputLow(APT_RegStruct *aptx)
{
    unsigned int risingOutSelect = aptx->DG_CFG.BIT.rg_dg_red_osel;
    unsigned int fallingOutSelect = aptx->DG_CFG.BIT.rg_dg_fed_osel;
    unsigned int risingInSelect = aptx->DG_CFG.BIT.rg_dg_red_isel;
    unsigned int fallingInSelect = aptx->DG_CFG.BIT.rg_dg_fed_isel;
    /* Enable force output */
    aptx->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_ENABLE;
    /* if PWMB invert */
    if (((risingOutSelect == APT_DB_RED_OUTPUT_INVERT) && (risingInSelect == APT_DB_RED_INPUT_PWM_B)) || \
         ((fallingOutSelect == APT_DB_FED_OUTPUT_INVERT) && (fallingInSelect == APT_DB_FED_INPUT_PWM_B))) {
        aptx->PG_OUT_FRC.BIT.rg_pgb_frc_act = APT_PWM_CONTINUOUS_ACTION_HIGH; /* if invert, set high */
    } else { /* if PWMB not invert */
        aptx->PG_OUT_FRC.BIT.rg_pgb_frc_act = APT_PWM_CONTINUOUS_ACTION_LOW; /* if not invert, set low */
    }
    return;
}

/**
  * @brief Both PWMA and PWMB output low level.
  * @param aptx APT register base address.
  * @retval None.
  */
static inline void DCL_APT_ForcePWMOutputLow(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    
    APT_ForcePWMAOutputLow(aptx);
    APT_ForcePWMBOutputLow(aptx);

    return;
}

/**
  * @brief Set the buffer load mode of continuous aciton software event register.
  * @param aptx APT register base address.
  * @param loadMode Buffer load mode.
  * @retval None.
  */
static inline void DCL_APT_SetSwContActionLoadMode(APT_RegStruct *aptx, APT_BufferLoadMode loadMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(loadMode >= APT_BUFFER_DISABLE);
    APT_PARAM_CHECK_NO_RET(loadMode <= APT_BUFFER_GLOBAL_LOAD);
    unsigned int bufFieldWidth = 4; /* Bit field width of buffer load mode setting */
    aptx->PG_BUF_EN.reg &= (~(0b11 << bufFieldWidth));  /* Clear rg_frc_gld_en and rg_frc_buf_en */
    aptx->PG_BUF_EN.reg |= (loadMode << bufFieldWidth); /* Write rg_frc_gld_en and rg_frc_buf_en */
}

/**
 * @brief Enable the buffer load events of PG_OUT_FRC register
 * @param aptx APT register base address.
 * @param channel PWM output channel
 * @param loadEvent The buffer load events of PG_OUT_FRC register
 *        A logical OR of valid values can be passed as the loadEvent parameter
 *        Valid values for loadEvent are:
 *            APT_ACTION_LOAD_EVENT_ZERO      -   When counter value equal to zero
 *            APT_ACTION_LOAD_EVENT_PERIOD    -   When counter value equal to period
 *            APT_ACTION_LOAD_EVENT_SYNC      -   When synchronization event is valid
 * @retval None.
 */
static inline void DCL_APT_SetSwContActionLoadEvent(APT_RegStruct *aptx, unsigned int loadEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    unsigned int actBufField = 16; /* Field width of continuous PWM action load event setting  */
    aptx->PG_ACT_LD.reg &= (~(0x1F << actBufField));
    aptx->PG_ACT_LD.reg |= (loadEvent << actBufField);
}

/**
  * @brief Enable continuous action software event.
  * @param aptx APT register base address.
  * @param channel PWM output channel.
  * @retval None.
  */
static inline void DCL_APT_EnableSwContPWMAction(APT_RegStruct *aptx, APT_PWMChannel channel)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(channel >= APT_PWM_CHANNEL_A);
    APT_PARAM_CHECK_NO_RET(channel <= APT_PWM_CHANNEL_B);
    if (channel == APT_PWM_CHANNEL_A) {
        aptx->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_SET;
    } else if (channel == APT_PWM_CHANNEL_B) {
        aptx->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_SET;
    }
}

/**
  * @brief Disable continuous action software event.
  * @param aptx APT register base address.
  * @param channel PWM output channel.
  * @retval None.
  */
static inline void DCL_APT_DisableSwContPWMAction(APT_RegStruct *aptx, APT_PWMChannel channel)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(channel >= APT_PWM_CHANNEL_A);
    APT_PARAM_CHECK_NO_RET(channel <= APT_PWM_CHANNEL_B);
    if (channel == APT_PWM_CHANNEL_A) {
        aptx->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_UNSET;
    } else if (channel == APT_PWM_CHANNEL_B) {
        aptx->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_UNSET;
    }
}

/* --------------------------------------------------------------------------------------------- */
/* Dead-Band Generation (DG) submodule Direct Configuration Layer functions -------------------- */
/* --------------------------------------------------------------------------------------------- */
/**
 * @brief Configure the rising edge delay (RED) of Dead-Band Generation.
 * @param aptx APT register base address.
 * @param redInput The input source of RED counter.
 * @param redOutMode The output of RED counter.
 * @param dgaOutSwap The swap mode of Dead-Band Generation output signal A.
 *                       true    -   Select the output of FED counter.
 *                       false   -   Select the output of RED counter.
 * @param redCount The count value of RED counter, in units of APT clock.
 * @retval None.
 */
static inline void DCL_APT_SetDeadBandRisingEdge(APT_RegStruct *aptx,
                                                 APT_REDInput redInput,
                                                 APT_REDOutMode redOutMode,
                                                 bool dgaOutSwap,
                                                 unsigned short redCount)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(redInput >= APT_DB_RED_INPUT_PWM_A);
    APT_PARAM_CHECK_NO_RET(redInput <= APT_DB_RED_INPUT_PWM_B);
    APT_PARAM_CHECK_NO_RET(redOutMode >= APT_DB_RED_OUTPUT_NOT_INVERT);
    APT_PARAM_CHECK_NO_RET(redOutMode <= APT_DB_RED_OUTPUT_PWM_A);
    aptx->DG_CFG.BIT.rg_dg_red_isel = redInput;
    aptx->DG_CFG.BIT.rg_dg_red_osel = redOutMode;
    aptx->DG_CFG.BIT.rg_dga_osel = dgaOutSwap;
    aptx->DG_RED.BIT.rg_dg_red = redCount;
}

/**
 * @brief Configure the falling edge delay (FED) of Dead-Band Generation.
 * @param aptx APT register base address.
 * @param fedInput The input source of FED counter.
 * @param fedOutMode The output of FED counter.
 * @param dgbOutSwap The swap mode of Dead-Band Generation output signal B.
 *                       true    -   Select the output of RED counter.
 *                       false   -   Select the output of FED counter.
 * @param fedCount The count value of FED counter, in units of APT clock.
 * @retval None.
 */
static inline void DCL_APT_SetDeadBandFallingEdge(APT_RegStruct *aptx,
                                                  APT_FEDInput fedInput,
                                                  APT_FEDOutMode fedOutMode,
                                                  bool dgbOutSwap,
                                                  unsigned short fedCount)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(fedInput >= APT_DB_FED_INPUT_PWM_B);
    APT_PARAM_CHECK_NO_RET(fedInput <= APT_DB_FED_INPUT_ZERO);
    APT_PARAM_CHECK_NO_RET(fedOutMode >= APT_DB_FED_OUTPUT_NOT_INVERT);
    APT_PARAM_CHECK_NO_RET(fedOutMode <= APT_DB_FED_OUTPUT_PWM_B);
    aptx->DG_CFG.BIT.rg_dg_fed_isel = fedInput;
    aptx->DG_CFG.BIT.rg_dg_fed_osel = fedOutMode;
    aptx->DG_CFG.BIT.rg_dgb_osel = dgbOutSwap;
    aptx->DG_FED.BIT.rg_dg_fed = fedCount;
}

/**
  * @brief Set buffer load mode of Dead-Band configuration register.
  * @param aptx APT register base address.
  * @param dgCfgLoadMode Buffer load mode.
  * @retval None.
  */
static inline void DCL_APT_SetDGConfigLoadMode(APT_RegStruct *aptx, APT_BufferLoadMode dgCfgLoadMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(dgCfgLoadMode >= APT_BUFFER_DISABLE);
    APT_PARAM_CHECK_NO_RET(dgCfgLoadMode <= APT_BUFFER_GLOBAL_LOAD);
    unsigned int bufFieldWidth = 4; /* Bit field width of buffer load mode setting */
    aptx->DG_BUF_EN.reg &= (~(0b11 << bufFieldWidth)); /* Clear rg_cfg_gld_en and rg_cfg_buf_en */
    aptx->DG_BUF_EN.reg |= (dgCfgLoadMode << bufFieldWidth); /* Write rg_cfg_gld_en and rg_cfg_buf_en */
}

/**
 * @brief Enable the buffer load events of DG_CFG register.
 * @param aptx APT register base address.
 * @param loadEvent The buffer load events of DG_CFG register.
 *        A logical OR of valid values can be passed as the loadEvent parameter.
 *        Valid values for loadEvent are:
 *            APT_DEAD_BAND_LOAD_EVENT_ZERO   -   When time base counter value equal to zero.
 *            APT_DEAD_BAND_LOAD_EVENT_PERIOD -   When time base counter value equal to period.
 * @retval None.
 */
static inline void DCL_APT_SetDGConfigLoadEvent(APT_RegStruct *aptx, unsigned int dgCfgLoadEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    unsigned int dgBufField = 16; /* Field width of continuous PWM action load event setting  */
    aptx->DG_BUF_LOAD.reg &= (~(0b11 << dgBufField));
    aptx->DG_BUF_LOAD.reg |= (dgCfgLoadEvent << dgBufField);
}

/**
  * @brief Set buffer load mode of Dead-Band rising edge delay counter register.
  * @param aptx APT register base address.
  * @param redCntLoadMode Buffer load mode.
  * @retval None.
  */
static inline void DCL_APT_SetREDCounterLoadMode(APT_RegStruct *aptx, APT_BufferLoadMode redCntLoadMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(redCntLoadMode >= APT_BUFFER_DISABLE);
    APT_PARAM_CHECK_NO_RET(redCntLoadMode <= APT_BUFFER_GLOBAL_LOAD);
    aptx->DG_BUF_EN.reg &= (~(0b11 << 0)); /* Clear rg_red_gld_en and rg_red_buf_en */
    aptx->DG_BUF_EN.reg |= (redCntLoadMode << 0); /* Write rg_red_gld_en and rg_red_buf_en */
}

/**
 * @brief Enable the buffer load events of DG_RED register
 * @param aptx APT register base address.
 * @param loadEvent The buffer load events of DG_RED register.
 *        A logical OR of valid values can be passed as the loadEvent parameter.
 *        Valid values for loadEvent are:
 *            APT_DEAD_BAND_LOAD_EVENT_ZERO   -   When time base counter value equal to zero.
 *            APT_DEAD_BAND_LOAD_EVENT_PERIOD -   When time base counter value equal to period.
 * @retval None.
 */
static inline void DCL_APT_SetREDCounterLoadEvent(APT_RegStruct *aptx, unsigned int redCntLoadEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->DG_BUF_LOAD.reg &= (~(0b11 << 0));
    aptx->DG_BUF_LOAD.reg |= (redCntLoadEvent << 0);
}

/**
  * @brief Set buffer load mode of Dead-Band falling edge delay counter register.
  * @param aptx APT register base address.
  * @param fedCntLoadMode Buffer load mode.
  * @retval None.
  */
static inline void DCL_APT_SetFEDCounterLoadMode(APT_RegStruct *aptx, APT_BufferLoadMode fedCntLoadMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(fedCntLoadMode >= APT_BUFFER_DISABLE);
    APT_PARAM_CHECK_NO_RET(fedCntLoadMode <= APT_BUFFER_GLOBAL_LOAD);
    unsigned int bufFieldWidth = 2; /* Bit field width of buffer load mode setting */
    aptx->DG_BUF_EN.reg &= (~(0b11 << bufFieldWidth));  /* Clear rg_fed_gld_en and rg_fed_buf_en */
    aptx->DG_BUF_EN.reg |= (fedCntLoadMode << bufFieldWidth); /* Write rg_fed_gld_en and rg_fed_buf_en */
}

/**
 * @brief Enable the buffer load events of DG_FED register.
 * @param aptx APT register base address.
 * @param loadEvent The buffer load events of DG_FED register.
 *        A logical OR of valid values can be passed as the loadEvent parameter.
 *        Valid values for loadEvent are:
 *            APT_DEAD_BAND_LOAD_EVENT_ZERO   -   When time base counter value equal to zero.
 *            APT_DEAD_BAND_LOAD_EVENT_PERIOD -   When time base counter value equal to period.
 * @retval None.
 */
static inline void DCL_APT_SetFEDCounterLoadEvent(APT_RegStruct *aptx, unsigned int fedCntLoadEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    unsigned int dgBufField = 8; /* Field width of continuous PWM action load event setting */
    aptx->DG_BUF_LOAD.reg &= (~(0b11 << dgBufField));
    aptx->DG_BUF_LOAD.reg |= (fedCntLoadEvent << dgBufField);
}

/* --------------------------------------------------------------------------------------------- */
/* Output Control (OC) submodule Direct Configuration Layer functions -------------------------- */
/* --------------------------------------------------------------------------------------------- */
/**
  * @brief Enable an output control event.
  * @param aptx APT register base address.
  * @param ocEvent Output control event.
  * @retval None.
  */
static inline void DCL_APT_EnableOutCtrlEvent(APT_RegStruct *aptx, APT_OutCtrlEvent ocEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ocEvent >= APT_OC_NO_EVENT);
    APT_PARAM_CHECK_NO_RET(ocEvent <= APT_OC_COMBINE_EVENT_B2);
    aptx->OC_MODE.reg |= ocEvent;
}

/**
  * @brief Disable an output control event.
  * @param aptx APT register base address.
  * @param ocEvent Output control event.
  * @retval None.
  */
static inline void DCL_APT_DisableOutCtrlEvent(APT_RegStruct *aptx, APT_OutCtrlEvent ocEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ocEvent >= APT_OC_NO_EVENT);
    APT_PARAM_CHECK_NO_RET(ocEvent <= APT_OC_COMBINE_EVENT_B2);
    aptx->OC_MODE.reg &= ~ocEvent;
}

/**
  * @brief Clear OC_MODE register.
  * @param aptx APT register base address.
  * @retval None.
  */
static inline void DCL_APT_ClearOCEventReg(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->OC_MODE.reg = 0;
}

/**
  * @brief Set output control mode of an output control event.
  * @param aptx APT register base address.
  * @param ocEvent Output control event.
  * @param ocEventMode Output control mode.
  * @retval None.
  */
static inline void DCL_APT_SetOutCtrlEventMode(APT_RegStruct *aptx,
                                               APT_OutCtrlEvent ocEvent,
                                               APT_OutCtrlMode ocEventMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET((ocEvent >= APT_OC_NO_EVENT) && (ocEvent <= APT_OC_COMBINE_EVENT_B2));
    APT_PARAM_CHECK_NO_RET(ocEventMode >= APT_OUT_CTRL_ONE_SHOT);
    APT_PARAM_CHECK_NO_RET(ocEventMode <= APT_OUT_CTRL_CYCLE_BY_CYBLE);
    unsigned ocModeOffset = 16; /* Offset of output control mode setting */
    if (ocEventMode == APT_OUT_CTRL_ONE_SHOT) {
        aptx->OC_MODE.reg &= (~(ocEvent << ocModeOffset)); /* Set rg_oc_mode_evtx to 0 */
    } else if (ocEventMode == APT_OUT_CTRL_CYCLE_BY_CYBLE) {
        aptx->OC_MODE.reg |= (ocEvent << ocModeOffset);    /* Set rg_oc_mode_evtx to 1 */
    }
}

/**
  * @brief Set output control action of an output control event.
  * @param aptx APT register base address.
  * @param channel PWM output channel.
  * @param ocEvtDir Output control event that takes into consideration of counter direction.
  * @param ocAction Output control action.
  * @retval None.
  */
static inline void DCL_APT_SetOutCtrlAction(APT_RegStruct *aptx,
                                            APT_PWMChannel channel,
                                            APT_OutCtrlEventDir ocEvtDir,
                                            APT_OutCtrlAction ocAction)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(channel >= APT_PWM_CHANNEL_A);
    APT_PARAM_CHECK_NO_RET(channel <= APT_PWM_CHANNEL_B);
    APT_PARAM_CHECK_NO_RET(ocEvtDir >= APT_OC_EVT_GPIO_OR_SYSTEM_UP);
    APT_PARAM_CHECK_NO_RET(ocEvtDir <= APT_OC_EVT_COMBINE_EVENT_B2_DOWN);
    APT_PARAM_CHECK_NO_RET(ocAction >= APT_OUT_CTRL_ACTION_DISABLE);
    APT_PARAM_CHECK_NO_RET(ocAction <= APT_OUT_CTRL_ACTION_HIGH_Z);
    if (channel == APT_PWM_CHANNEL_A) {
        aptx->OC_ACT_A.reg &= (~(0b111 << ocEvtDir));
        aptx->OC_ACT_A.reg |= (ocAction << ocEvtDir);
    } else if (channel == APT_PWM_CHANNEL_B) {
        aptx->OC_ACT_B.reg &= (~(0b111 << ocEvtDir));
        aptx->OC_ACT_B.reg |= (ocAction << ocEvtDir);
    }
}

/**
  * @brief Get the flag of an output control event.
  * @param aptx APT register base address.
  * @param ocEvent Output control event.
  * @retval bool: true, false.
  */
static inline bool DCL_APT_GetOutCtrlEventFlag(APT_RegStruct *aptx, APT_OutCtrlEvent ocEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_WITH_RET(ocEvent >= APT_OC_NO_EVENT, false);
    APT_PARAM_CHECK_WITH_RET(ocEvent <= APT_OC_COMBINE_EVENT_B2, false);
    return ((aptx->OC_EVT_FLAG.reg & ocEvent) == ocEvent);
}

/**
  * @brief Clear the flag of an output control event.
  * @param aptx APT register base address.
  * @param ocEvent Output control event.
  * @retval None.
  */
static inline void DCL_APT_ClearOutCtrlEventFlag(APT_RegStruct *aptx, APT_OutCtrlEvent ocEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ocEvent >= APT_OC_NO_EVENT);
    APT_PARAM_CHECK_NO_RET(ocEvent <= APT_OC_COMBINE_EVENT_B2);
    unsigned int ocFlgOffset = 16; /* Offset of output control flag clear */
    aptx->OC_EVT_FLAG.reg |= (ocEvent << ocFlgOffset);
}

/**
  * @brief Enable the event latch of an output control event.
  * @param aptx APT register base address.
  * @param ocEvent Output control event.
  * @retval None.
  */
static inline void DCL_APT_EnableOutCtrlEventLatch(APT_RegStruct *aptx, APT_OutCtrlEvent ocEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ocEvent >= APT_OC_NO_EVENT);
    APT_PARAM_CHECK_NO_RET(ocEvent <= APT_OC_COMBINE_EVENT_B2);
    aptx->OC_LAT_EN.reg |= ocEvent;
}

/**
  * @brief Disable the event latch of an output control event.
  * @param aptx APT register base address.
  * @param ocEvent Output control event.
  * @retval None.
  */
static inline void DCL_APT_DisableOutCtrlEventLatch(APT_RegStruct *aptx, APT_OutCtrlEvent ocEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ocEvent >= APT_OC_NO_EVENT);
    APT_PARAM_CHECK_NO_RET(ocEvent <= APT_OC_COMBINE_EVENT_B2);
    aptx->OC_LAT_EN.reg &= ~ocEvent;
}

/**
  * @brief Set cycle-by-cycle event latch clear event.
  * @param aptx APT register base address.
  * @param ocEvent Output control event.
  * @param clrMode Latche clear event of cycle-by-cycle event.
  * @retval None.
  */
static inline void DCL_APT_SetCBCLatchClearEvent(APT_RegStruct *aptx,
                                                 APT_OutCtrlEvent ocEvent,
                                                 APT_CBCClearMode clrMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ocEvent >= APT_OC_NO_EVENT);
    APT_PARAM_CHECK_NO_RET(ocEvent <= APT_OC_COMBINE_EVENT_B2);
    APT_PARAM_CHECK_NO_RET(clrMode >= APT_CLEAR_CBC_ON_CNTR_ZERO);
    APT_PARAM_CHECK_NO_RET(clrMode <= APT_CLEAR_CBC_ON_CNTR_ZERO_PERIOD);
    unsigned int cbcClrOffsetZero = 0; /* Offset of CBC latch clear on counter equal to 0 */
    unsigned int cbcClrOffsetPrd = 16; /* Offset of CBC latch clear on counter euqal to period */
    unsigned int mask = (ocEvent << cbcClrOffsetPrd) | (ocEvent << cbcClrOffsetZero);
    mask &= clrMode;
    aptx->OC_PRD_CLR.reg |= mask;
}

/**
  * @brief Enable a software output control event.
  * @param aptx APT register base address.
  * @param ocEvent Output control event.
  * @retval None.
  */
static inline void DCL_APT_EnableSwOutCtrlEvent(APT_RegStruct *aptx, APT_OutCtrlEvent ocEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ocEvent >= APT_OC_NO_EVENT);
    APT_PARAM_CHECK_NO_RET(ocEvent <= APT_OC_COMBINE_EVENT_B2);
    aptx->OC_FRC_EVT.reg |= ocEvent;
}

/**
  * @brief Disable a software output control event.
  * @param aptx APT register base address.
  * @param ocEvent Output control event.
  * @retval None.
  */
static inline void DCL_APT_DisableSwOutCtrlEvent(APT_RegStruct *aptx, APT_OutCtrlEvent ocEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ocEvent >= APT_OC_NO_EVENT);
    APT_PARAM_CHECK_NO_RET(ocEvent <= APT_OC_COMBINE_EVENT_B2);
    aptx->OC_FRC_EVT.reg &= (~ocEvent);
}

/* --------------------------------------------------------------------------------------------- */
/* Interrupt Generation (IG) submodule Direct Configuration Layer functions -------------------- */
/* --------------------------------------------------------------------------------------------- */
/**
  * @brief Enable the output control event to generate an event interrupt.
  * @param aptx APT register base address.
  * @param ocEvent Output control event.
  * @retval None.
  */
static inline void DCL_APT_EnableEventInterrupt(APT_RegStruct *aptx, APT_OutCtrlEvent ocEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ocEvent >= APT_OC_NO_EVENT);
    APT_PARAM_CHECK_NO_RET(ocEvent <= APT_OC_COMBINE_EVENT_B2);
    aptx->INT_EVT_EN.reg |= ocEvent;
}

/**
  * @brief Disable the output control event to generate an event interrupt..
  * @param aptx APT register base address.
  * @param ocEvent Output control event.
  * @retval None.
  */
static inline void DCL_APT_DisableEventInterrupt(APT_RegStruct *aptx, APT_OutCtrlEvent ocEvent)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ocEvent >= APT_OC_NO_EVENT);
    APT_PARAM_CHECK_NO_RET(ocEvent <= APT_OC_COMBINE_EVENT_B2);
    aptx->INT_EVT_EN.reg &= (~ocEvent);
}

/**
  * @brief Enable timer interrupt of APT module.
  * @param aptx APT register base address.
  * @retval None.
  */
static inline void DCL_APT_EnableTimerInterrupt(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->INT_TMR_EN.BIT.rg_int_en_tmr = BASE_CFG_SET;
}

/**
  * @brief Disable timer interrupt of APT module.
  * @param aptx APT register base address.
  * @retval None.
  */
static inline void DCL_APT_DisableTimerInterrupt(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->INT_TMR_EN.BIT.rg_int_en_tmr = BASE_CFG_UNSET;
}

/**
  * @brief Get the event interrupt flag.
  * @param aptx APT register base address.
  * @retval bool: true, false.
  */
static inline bool DCL_APT_GetEventInterruptFlag(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    return (aptx->OC_EVT_FLAG.BIT.ro_int_flag_evt);
}

/**
  * @brief Clear the event interrupt flag.
  * @param aptx APT register base address.
  * @retval None.
  */
static inline void DCL_APT_ClearEventInterruptFlag(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->OC_EVT_FLAG.BIT.rg_int_clr_evt = BASE_CFG_SET;
}

/**
  * @brief Get the timer interrupt flag.
  * @param aptx APT register base address.
  * @retval bool: true, false.
  */
static inline bool DCL_APT_GetTimerInterruptFlag(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    return (aptx->INT_TMR_FLAG.BIT.ro_int_flag_tmr);
}

/**
  * @brief Clear the timer interrupt flag.
  * @param aptx APT register base address.
  * @retval None.
  */
static inline void DCL_APT_ClearTimerInterruptFlag(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->INT_TMR_FLAG.BIT.rg_int_clr_tmr = BASE_CFG_SET;
}

/**
  * @brief Select the source of timer interrupt.
  * @param aptx APT register base address.
  * @param tmrIntSrc Source of timer interrupt.
  * @retval None.
  */
static inline void DCL_APT_SetTimerInterruptSrc(APT_RegStruct *aptx, APT_TimerInterruptSrc tmrIntSrc)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(tmrIntSrc >= APT_INT_SRC_CNTR_DISABLE);
    APT_PARAM_CHECK_NO_RET(tmrIntSrc <= APT_INT_SRC_CNTR_CMPD_DOWN);
    aptx->INT_TMR_SEL.BIT.rg_int_tmr_sel = tmrIntSrc;
}

/**
  * @brief Enable the synchronization of timer interrupt scale initial count value.
  * @param aptx APT register base address.
  * @retval None.
  */
static inline void DCL_APT_EnableTimerInterruptCountSyncInit(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->INT_PRSC_CFG.BIT.rg_int_prsc_synen = BASE_CFG_SET;
}

/**
  * @brief Disable the synchronization of timer interrupt scale initial count value.
  * @param aptx APT register base address.
  * @retval None.
  */
static inline void DCL_APT_DisableTimerInterruptCountSyncInit(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->INT_PRSC_CFG.BIT.rg_int_prsc_synen = BASE_CFG_UNSET;
}

/**
  * @brief Set the initial count value of timer interrupt scale.
  * @param aptx APT register base address.
  * @param intCntInitVal Initial count value of timer interrupt scale.
  * @retval None.
  */
static inline void DCL_APT_SetTimerInterruptCountSyncInitVal(APT_RegStruct *aptx, unsigned short intCntInitVal)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(intCntInitVal <= TIMER_INTERRUPT_CNT_MAX);
    aptx->INT_PRSC_CFG.BIT.rg_int_prsc_phs = intCntInitVal;
}

/**
  * @brief Set the count period of timer interrupt scale.
  * @param aptx APT register base address.
  * @param intCntPeriod Count period of timer interrupt scale.
  * @retval None.
  */
static inline void DCL_APT_SetTimerInterruptCountPeriod(APT_RegStruct *aptx, unsigned short intCntPeriod)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(intCntPeriod <= TIMER_INTERRUPT_CNT_MAX);
    aptx->INT_PRSC_CFG.BIT.rg_int_prsc_prd = intCntPeriod;
}

/**
  * @brief Get the count value of timer interrupt scale.
  * @param aptx APT register base address.
  * @retval unsigned short: Count value of timer interrupt scale.
  */
static inline unsigned short DCL_APT_GetTimerInterruptCount(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    return (aptx->INT_PRSC_CFG.BIT.ro_int_prsc_cnt);
}

/**
  * @brief Force the count value of timer interrupt scale to increase.
  * @param aptx APT register base address.
  * @retval None.
  */
static inline void DCL_APT_ForceTimerInterruptCountIncr(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->INT_PRSC_CFG.BIT.rg_int_prsc_frc = BASE_CFG_SET;
}

/* --------------------------------------------------------------------------------------------- */
/* ADC Converter Start (CS) submodule Direct Configuration Layer functions --------------------- */
/* --------------------------------------------------------------------------------------------- */
/**
  * @brief Enable the ADC trigger channel.
  * @param aptx APT register base address.
  * @param csTrgCh ADC trigger channel.
  * @retval None.
  */
static inline void DCL_APT_EnableADCTrigger(APT_RegStruct *aptx, APT_ADCTriggerChannel csTrgCh)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(csTrgCh >= APT_ADC_CONVERSION_START_A);
    APT_PARAM_CHECK_NO_RET(csTrgCh <= APT_ADC_CONVERSION_START_B);
    if (csTrgCh == APT_ADC_CONVERSION_START_A) {
        aptx->CS_TMR_SELA.BIT.rg_csa_en_cs = BASE_CFG_SET;
    } else if (csTrgCh == APT_ADC_CONVERSION_START_B) {
        aptx->CS_TMR_SELB.BIT.rg_csb_en_cs = BASE_CFG_SET;
    }
}

/**
  * @brief Disable the ADC trigger channel.
  * @param aptx APT register base address.
  * @param csTrgCh ADC trigger channel.
  * @retval None.
  */
static inline void DCL_APT_DisableADCTrigger(APT_RegStruct *aptx, APT_ADCTriggerChannel csTrgCh)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(csTrgCh >= APT_ADC_CONVERSION_START_A);
    APT_PARAM_CHECK_NO_RET(csTrgCh <= APT_ADC_CONVERSION_START_B);
    if (csTrgCh == APT_ADC_CONVERSION_START_A) {
        aptx->CS_TMR_SELA.BIT.rg_csa_en_cs = BASE_CFG_UNSET;
    } else if (csTrgCh == APT_ADC_CONVERSION_START_B) {
        aptx->CS_TMR_SELB.BIT.rg_csb_en_cs = BASE_CFG_UNSET;
    }
}

/**
  * @brief Select the source of ADC trigger channel.
  * @param aptx APT register base address.
  * @param csTrgCh ADC trigger channel.
  * @param csTrgSrc Source of ADC trigger.
  * @retval None.
  */
static inline void DCL_APT_SetADCTriggerSrc(APT_RegStruct *aptx,
                                            APT_ADCTriggerChannel csTrgCh,
                                            APT_ADCTriggerSource csTrgSrc)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(csTrgCh >= APT_ADC_CONVERSION_START_A);
    APT_PARAM_CHECK_NO_RET(csTrgCh <= APT_ADC_CONVERSION_START_B);
    APT_PARAM_CHECK_NO_RET(csTrgSrc <= APT_CS_SRC_CNTR_CMPD_DOWN);
    if (csTrgCh == APT_ADC_CONVERSION_START_A) {
        aptx->CS_TMR_SELA.BIT.rg_csa_tmr_sel = csTrgSrc;
    } else if (csTrgCh == APT_ADC_CONVERSION_START_B) {
        aptx->CS_TMR_SELB.BIT.rg_csb_tmr_sel = csTrgSrc;
    }
}

/**
  * @brief Enable synchronization of ADC trigger scale initial count value.
  * @param aptx APT register base address.
  * @param csTrgCh ADC trigger channel.
  * @retval None.
  */
static inline void DCL_APT_EnableADCTriggerCountSyncInit(APT_RegStruct *aptx, APT_ADCTriggerChannel csTrgCh)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(csTrgCh >= APT_ADC_CONVERSION_START_A);
    APT_PARAM_CHECK_NO_RET(csTrgCh <= APT_ADC_CONVERSION_START_B);
    if (csTrgCh == APT_ADC_CONVERSION_START_A) {
        aptx->CS_PRSCA_CFG.BIT.rg_csa_prsc_synen = BASE_CFG_SET;
    } else if (csTrgCh == APT_ADC_CONVERSION_START_B) {
        aptx->CS_PRSCB_CFG.BIT.rg_csb_prsc_synen = BASE_CFG_SET;
    }
}

/**
  * @brief Disable synchronization of ADC trigger scale initial count value.
  * @param aptx APT register base address.
  * @param csTrgCh ADC trigger channel.
  * @retval None.
  */
static inline void DCL_APT_DisableADCTriggerCountSyncInit(APT_RegStruct *aptx, APT_ADCTriggerChannel csTrgCh)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(csTrgCh >= APT_ADC_CONVERSION_START_A);
    APT_PARAM_CHECK_NO_RET(csTrgCh <= APT_ADC_CONVERSION_START_B);
    if (csTrgCh == APT_ADC_CONVERSION_START_A) {
        aptx->CS_PRSCA_CFG.BIT.rg_csa_prsc_synen = BASE_CFG_UNSET;
    } else if (csTrgCh == APT_ADC_CONVERSION_START_B) {
        aptx->CS_PRSCB_CFG.BIT.rg_csb_prsc_synen = BASE_CFG_UNSET;
    }
}

/**
  * @brief Set the initial count value of ADC trigger scale.
  * @param aptx APT register base address.
  * @param csTrgCh ADC trigger channel.
  * @param csCntInitVal Initial count value of ADC trigger scale.
  * @retval None.
  */
static inline void DCL_APT_SetADCTriggerCountSyncInitVal(APT_RegStruct *aptx,
                                                         APT_ADCTriggerChannel csTrgCh,
                                                         unsigned short csCntInitVal)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(csTrgCh >= APT_ADC_CONVERSION_START_A);
    APT_PARAM_CHECK_NO_RET(csTrgCh <= APT_ADC_CONVERSION_START_B);
    APT_PARAM_CHECK_NO_RET(csCntInitVal <= ADC_CONVERSION_START_CNT_MAX);
    if (csTrgCh == APT_ADC_CONVERSION_START_A) {
        aptx->CS_PRSCA_CFG.BIT.rg_csa_prsc_phs = csCntInitVal;
    } else if (csTrgCh == APT_ADC_CONVERSION_START_B) {
        aptx->CS_PRSCB_CFG.BIT.rg_csb_prsc_phs = csCntInitVal;
    }
}

/**
  * @brief Set the count period of ADC trigger scale.
  * @param aptx APT register base address.
  * @param csTrgCh ADC trigger channel.
  * @param csCntPeriod Count period of ADC trigger scale.
  * @retval None.
  */
static inline void DCL_APT_SetADCTriggerCountPeriod(APT_RegStruct *aptx,
                                                    APT_ADCTriggerChannel csTrgCh,
                                                    unsigned short csCntPeriod)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(csTrgCh >= APT_ADC_CONVERSION_START_A);
    APT_PARAM_CHECK_NO_RET(csTrgCh <= APT_ADC_CONVERSION_START_B);
    APT_PARAM_CHECK_NO_RET(csCntPeriod <= ADC_CONVERSION_START_CNT_MAX);
    if (csTrgCh == APT_ADC_CONVERSION_START_A) {
        aptx->CS_PRSCA_CFG.BIT.rg_csa_prsc_prd = csCntPeriod;
    } else if (csTrgCh == APT_ADC_CONVERSION_START_B) {
        aptx->CS_PRSCB_CFG.BIT.rg_csb_prsc_prd = csCntPeriod;
    }
}

/**
  * @brief Force the count value of ADC trigger scale to increase.
  * @param aptx APT register base address.
  * @param csTrgCh ADC trigger channel.
  * @retval None.
  */
static inline void DCL_APT_ForceADCTriggerCountIncr(APT_RegStruct *aptx, APT_ADCTriggerChannel csTrgCh)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(csTrgCh >= APT_ADC_CONVERSION_START_A);
    APT_PARAM_CHECK_NO_RET(csTrgCh <= APT_ADC_CONVERSION_START_B);
    if (csTrgCh == APT_ADC_CONVERSION_START_A) {
        aptx->CS_PRSCA_CFG.BIT.rg_csa_prsc_frc = BASE_CFG_SET;
    } else if (csTrgCh == APT_ADC_CONVERSION_START_B) {
        aptx->CS_PRSCB_CFG.BIT.rg_csb_prsc_frc = BASE_CFG_SET;
    }
}

/**
  * @brief Get the flag of ADC trigger.
  * @param aptx APT register base address.
  * @param csTrgCh ADC trigger channel.
  * @retval bool: true, false.
  */
static inline bool DCL_APT_GetADCTriggerFlag(APT_RegStruct *aptx, APT_ADCTriggerChannel csTrgCh)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_WITH_RET(csTrgCh >= APT_ADC_CONVERSION_START_A, false);
    APT_PARAM_CHECK_WITH_RET(csTrgCh <= APT_ADC_CONVERSION_START_B, false);
    return ((aptx->CS_FLAG.reg & csTrgCh) == csTrgCh);
}

/**
  * @brief Clear the flag of ADC trigger.
  * @param aptx APT register base address.
  * @param csTrgCh ADC trigger channel.
  * @retval None.
  */
static inline void DCL_APT_ClearADCTriggerFlag(APT_RegStruct *aptx, APT_ADCTriggerChannel csTrgCh)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(csTrgCh >= APT_ADC_CONVERSION_START_A);
    APT_PARAM_CHECK_NO_RET(csTrgCh <= APT_ADC_CONVERSION_START_B);
    unsigned int trgFlgOffset = 16; /* Offset of ADC trigget flag clear */
    aptx->CS_FLAG.reg |= (csTrgCh << trgFlgOffset);
}

/**
  * @brief Configure the DMA request of ADC trigger.
  * @param aptx APT register base address.
  * @param csDMAReqSrc DMA request source of ADC Converter Start submodule.
  * @param csDMAType DMA request type of ADC Converter Start submodule.
  * @retval None.
  */
static inline void DCL_APT_SetADCTriggerDMAReq(APT_RegStruct *aptx,
                                               APT_ADCTrgDMAReqSrc csDMAReqSrc,
                                               APT_ADCTrgDMAReqType csDMAType)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(csDMAReqSrc >= APT_CS_DMA_REQ_SRC_DISABLE);
    APT_PARAM_CHECK_NO_RET(csDMAReqSrc <= APT_CS_DMA_REQ_SRC_CHANNEL_B);
    APT_PARAM_CHECK_NO_RET(csDMAType <= APT_CS_DMA_SINGLE_REQUEST);
    APT_PARAM_CHECK_NO_RET(csDMAType <= APT_CS_DMA_BURST_REQUEST);
    aptx->CS_DMA.reg &= (~(0b11 << csDMAType));
    aptx->CS_DMA.reg |= (csDMAReqSrc << csDMAType);
}

/* --------------------------------------------------------------------------------------------- */
/* Event Management (EM) submodule Direct Configuration Layer functions ------------------------ */
/* --------------------------------------------------------------------------------------------- */
/**
  * @brief Set the polarity of GPIO/system event.
  * @param aptx APT register base address.
  * @param ioSysEvt GPIO or system event.
  * @param ioSysEvtPolar Event polarity.
  * @retval None.
  */
static inline void DCL_APT_SetIOSysEventPolarity(APT_RegStruct *aptx,
                                                 APT_EMIOSysEvent ioSysEvt,
                                                 APT_EMEventPolarity ioSysEvtPolar)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(ioSysEvt >= APT_EM_GPIO_EVENT_1);
    APT_PARAM_CHECK_NO_RET(ioSysEvt <= APT_EM_SYSTEM_EVENT_3);
    APT_PARAM_CHECK_NO_RET(ioSysEvtPolar >= APT_EM_EVENT_POLARITY_NOT_INVERT);
    APT_PARAM_CHECK_NO_RET(ioSysEvtPolar <= APT_EM_EVENT_POLARITY_FORCE_HIGH);
    aptx->EM_EVTIO_PSEL.reg &= (~(0b11 << ioSysEvt));
    aptx->EM_EVTIO_PSEL.reg |= (ioSysEvtPolar << ioSysEvt);
}

/**
  * @brief Set the polarity of multiplexing event.
  * @param aptx APT register base address.
  * @param mpEvt Multiplexing event.
  * @param mpEvtPolar Event polarity.
  * @retval None.
  */
static inline void DCL_APT_SetMpEventPolarity(APT_RegStruct *aptx,
                                              APT_EMMuxEvent mpEvt,
                                              APT_EMEventPolarity mpEvtPolar)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(mpEvt >= APT_EM_MP_EVENT_1);
    APT_PARAM_CHECK_NO_RET(mpEvt <= APT_EM_MP_EVENT_6);
    APT_PARAM_CHECK_NO_RET(mpEvtPolar >= APT_EM_EVENT_POLARITY_NOT_INVERT);
    APT_PARAM_CHECK_NO_RET(mpEvtPolar <= APT_EM_EVENT_POLARITY_FORCE_HIGH);
    aptx->EM_EVTMP_PSEL.reg &= (~(0b11 << mpEvt));
    aptx->EM_EVTMP_PSEL.reg |= (mpEvtPolar << mpEvt);
}

/**
 * @brief When the logicial OR result of GPIO events and MUX events is selected as the source of EM group event,
 *        this function is called to enable which events can participate in the logical OR operation.
 * @param aptx APT register base address.
 * @param emGroup The group of Event Management, which can be APT_EM_MODULE_A or APT_EM_MODULE_B.
 *                Each EM group has 2 events. All the 4 group events are enumerated in APT_EMGroupEvent.
 * @param event1OREn The logical OR operation source of group event 1.
 * @param event2OREn The logical OR operation source of group event 2.
 *        event1OREn and event2ORE are the logical OR of some valid values.
 *        Each valid values indicates that the corresponding event is enabled to participate in the
 *        logical OR operation of EM group event source. These valid values are:
 *            APT_EM_OR_EN_GPIO_EVENT_1       -   GPIO event 1 is enabled
 *            APT_EM_OR_EN_GPIO_EVENT_2       -   GPIO event 2 is enabled
 *            APT_EM_OR_EN_GPIO_EVENT_3       -   GPIO event 3 is enabled
 *            APT_EM_OR_EN_MUX_EVENT_1        -   MUX event 1 is enabled
 *            APT_EM_OR_EN_MUX_EVENT_2        -   MUX event 2 is enabled
 *            APT_EM_OR_EN_MUX_EVENT_3        -   MUX event 3 is enabled
 *            APT_EM_OR_EN_MUX_EVENT_4        -   MUX event 4 is enabled
 *            APT_EM_OR_EN_MUX_EVENT_5        -   MUX event 5 is enabled
 *            APT_EM_OR_EN_MUX_EVENT_6        -   MUX event 6 is enabled
 * @retval None.
 */
static inline void DCL_APT_SetEMEventOR(APT_RegStruct *aptx,
                                        APT_EMGroup emGroup,
                                        unsigned short event1OREn,
                                        unsigned short event2OREn)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(emGroup >= APT_EM_MODULE_A);
    APT_PARAM_CHECK_NO_RET(emGroup <= APT_EM_MODULE_B);
    if (emGroup == APT_EM_MODULE_A) {
        aptx->EM_AOR_EN.BIT.rg_em_a1_oren = event1OREn;
        aptx->EM_AOR_EN.BIT.rg_em_a2_oren = event2OREn;
    } else if (emGroup == APT_EM_MODULE_B) {
        aptx->EM_BOR_EN.BIT.rg_em_b1_oren = event1OREn;
        aptx->EM_BOR_EN.BIT.rg_em_b2_oren = event2OREn;
    }
}

/**
  * @brief Select the combine event source of GRP_A1, GRP_A2, GRP_B1, GRP_B2.
  * @param aptx APT register base address.
  * @param evtGroup Combine event source group.
  * @param combineEvtSrc Combine event source.
  * @retval None.
  */
static inline void DCL_APT_SetCombineGroupSrc(APT_RegStruct *aptx,
                                              APT_EMCombineEvtSrcGrp evtGroup,
                                              APT_EMCombineEvtSrc combineEvtSrc)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(evtGroup >= APT_EM_COMBINE_SRC_GRP_A1);
    APT_PARAM_CHECK_NO_RET(evtGroup <= APT_EM_COMBINE_SRC_GRP_B2);
    APT_PARAM_CHECK_NO_RET(combineEvtSrc >= APT_EM_COMBINE_SRC_EVT_1);
    APT_PARAM_CHECK_NO_RET(combineEvtSrc <= APT_EM_COMBINE_SRC_ALL_EVENT_OR);
    unsigned int grpEvtFieldWidth = 4; /* Bit field width of combine event group input source setting */
    aptx->EM_MRG_SEL.reg &= (~(0b1111 << (evtGroup * grpEvtFieldWidth)));
    aptx->EM_MRG_SEL.reg |= (combineEvtSrc << (evtGroup * grpEvtFieldWidth));
}

/**
  * @brief Select Combine Mode
  * @param aptx APT register base address.
  * @param cmbEvt Combine event.
  * @param cmbMode Combine mode.
  * @retval None.
  */
static inline void DCL_APT_SetCombineEventSrc(APT_RegStruct *aptx,
                                              APT_EMCombineEvent cmbEvt,
                                              APT_EMCombineEvtMode cmbMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(cmbEvt >= APT_EM_COMBINE_EVENT_A1);
    APT_PARAM_CHECK_NO_RET(cmbEvt <= APT_EM_COMBINE_EVENT_B2);
    APT_PARAM_CHECK_NO_RET(cmbMode >= APT_EM_COMBINE_LOW_LEVEL);
    APT_PARAM_CHECK_NO_RET(cmbMode <= APT_EM_COMBINE_EVT2);
    unsigned int cmbModeOffset = 16; /* Offset of combine mode */
    unsigned int cmbModeFieldWidth = 4; /* Bit field width of combine mode */
    aptx->EM_MRG_SEL.reg &= (~(0b111 << (cmbModeOffset + cmbEvt * cmbModeFieldWidth)));
    aptx->EM_MRG_SEL.reg |= (cmbMode << (cmbModeOffset + cmbEvt * cmbModeFieldWidth));
}

/**
  * @brief Select the source of Event Management submodule filter event.
  * @param aptx APT register base address.
  * @param cmbEvt Combine event.
  * @retval None.
  */
static inline void DCL_APT_SelectFilterEventInput(APT_RegStruct *aptx, APT_EMCombineEvent cmbEvt)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(cmbEvt >= APT_EM_COMBINE_EVENT_A1);
    APT_PARAM_CHECK_NO_RET(cmbEvt <= APT_EM_COMBINE_EVENT_B2);
    aptx->EM_OUT_SEL.BIT.rg_evtfilt_sel = cmbEvt;
}

/**
  * @brief Set the output type of combine event.
  * @param aptx APT register base address.
  * @param cmbEvt Combine event.
  * @param filter Whether the output of combine event is filtered.
  * @retval None.
  */
static inline void DCL_APT_SetCombineEventOut(APT_RegStruct *aptx,
                                              APT_EMCombineEvent cmbEvt,
                                              APT_EMCombineEventOut filter)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(cmbEvt >= APT_EM_COMBINE_EVENT_A1);
    APT_PARAM_CHECK_NO_RET(cmbEvt <= APT_EM_COMBINE_EVENT_B2);
    APT_PARAM_CHECK_NO_RET(filter >= APT_EM_COMBINE_EVENT_OUT_ORIG_SIGNAL);
    APT_PARAM_CHECK_NO_RET(filter <= APT_EM_COMBINE_EVENT_OUT_FILT_SIGNAL);
    aptx->EM_OUT_SEL.reg &= (~(0b1 << cmbEvt));
    aptx->EM_OUT_SEL.reg |= (filter << cmbEvt);
}

/**
  * @brief Select the sync-in source of slave APT module.
  * @param aptx APT register base address.
  * @param syncInSrc Sync-in source of slave APT module.
  * @retval None.
  */
static inline void DCL_APT_SelectSyncInPulseSrc(APT_RegStruct *aptx, APT_SyncInSrc syncInSrc)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(syncInSrc >= APT_SYNCIN_SRC_APT0_SYNCOUT);
    APT_PARAM_CHECK_NO_RET(syncInSrc <= APT_SYNCIN_SRC_DISABLE);
    aptx->SYNI_CFG.BIT.rg_syni_sel = syncInSrc;
}

/**
  * @brief Get the flag of sync-in pulse.
  * @param aptx APT register base address.
  * @retval bool: true, false.
  */
static inline bool DCL_APT_GetSyncInPulseFlag(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    return (aptx->SYNI_CFG.BIT.ro_syni_flag);
}

/**
  * @brief Clear the flag of sync-in pulse.
  * @param aptx APT register base address.
  * @retval None.
  */
static inline void DCL_APT_ClearSyncInPulseFlag(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->SYNI_CFG.BIT.rg_syni_clr = BASE_CFG_SET;
}

/**
 * @brief Select the synchronization source of the time-base counter.
 * @param aptx APT register base address.
 * @param cntrSyncSrc The selection of synchronization source for the time-base counter.
 *        A logical OR of valid values can be passed as the cntrSyncSrc parameter.
 *        Valid values for cntrSyncSrc are:
 *            APT_CNTR_SYNC_SRC_COMBINE_EVENT_A1  - Enable combine event A1 as the counter synchronization source.
 *            APT_CNTR_SYNC_SRC_COMBINE_EVENT_B1  - Enable combine event B1 as the counter synchronization source.
 *            APT_CNTR_SYNC_SRC_SYNCIN            - Enable Sync-In source as the counter synchronization source.
 * @retval None.
 */
static inline void DCL_APT_SetTimeBaseCounterSyncSrc(APT_RegStruct *aptx, unsigned short cntrSyncSrc)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(cntrSyncSrc <= CNTR_SYNC_SOURCE_MAX);
    aptx->SYNCNT_CFG.reg = cntrSyncSrc;
}

/**
 * @brief Select the source of synchronization out pulse.
 * @param aptx APT register base address.
 * @param syncOutSrc The source of synchronization out pulse.
 *        A logical OR of valid values can be passed as the syncOutSrc parameter.
 *        Valid values for syncOutSrc are:
 *            APT_SYNC_OUT_ON_CNTR_ZERO           - Generate a sync out pulse when counter equals zero.
 *            APT_SYNC_OUT_ON_CNTR_PERIOD         - Generate a sync out pulse when counter equals period.
 *            APT_SYNC_OUT_ON_COMBINE_EVENT_A1    - Generate a sync out pulse when combine event A1 happens.
 *            APT_SYNC_OUT_ON_COMBINE_EVENT_B1    - Generate a sync out pulse when combine event B1 happens.
 *            APT_SYNC_OUT_ON_CNTR_CMPB           - Generate a sync out pulse when counter equals CMPB.
 *            APT_SYNC_OUT_ON_CNTR_CMPC           - Generate a sync out pulse when counter equals CMPC.
 *            APT_SYNC_OUT_ON_CNTR_CMPD           - Generate a sync out pulse when counter equals CMPD.
 * @retval None.
 */
static inline void DCL_APT_SetSyncOutPulseSrc(APT_RegStruct *aptx, unsigned short syncOutSrc)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(syncOutSrc <= SYNC_OUT_SOURCE_MAX);
    aptx->SYNO_CFG.reg &= (~(0xFF << 0));
    aptx->SYNO_CFG.reg |= (syncOutSrc << 0);
}

/**
  * @brief Set synchronization mode of master APT module.
  * @param aptx APT register base address.
  * @param syncOutMode Synchronization mode of master APT module.
  * @retval None.
  */
static inline void DCL_APT_SetSyncOutMode(APT_RegStruct *aptx, APT_SyncOutMode syncOutMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(syncOutMode >= APT_SYNCOUT_ONE_SHOT_MODE);
    APT_PARAM_CHECK_NO_RET(syncOutMode <= APT_SYNCOUT_MULTIPLE_MODE);
    aptx->SYNO_CFG.BIT.rg_mode_syno = syncOutMode;
}

/**
  * @brief Select the latch source of one-shot sync-out mode.
  * @param aptx APT register base address.
  * @param latSetSel Latch source of one-shot sync-out mode.
  * @retval None.
  */
static inline void DCL_APT_SelectSyncOutOneShotLatch(APT_RegStruct *aptx, APT_SyncOutLatSetSel latSetSel)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(latSetSel >= APT_SYNCOUT_LATCH_SET_ON_SW_FORCE);
    APT_PARAM_CHECK_NO_RET(latSetSel <= APT_SYNCOUT_LATCH_SET_ON_GLB_LOAD);
    aptx->SYNO_CFG.BIT.rg_latset_sel = latSetSel;
}

/**
 * @brief When in one-shot sync out mode and rg_latset_otsyn is selected as the latch set condition,
 *        this function is called to turn the one-shot latch condition ON.
 *        Upon occurrence of a chosen sync out source event, a sync out pulse is generated and the latch
 *        will be cleared. Hence writing 1 to rg_latset_otsyn will allow a sync out event to pass through
 *        and block other sync out source event.
 * @param aptx APT register base address.
 * @retval None.
 */
static inline void DCL_APT_SetSyncOutOneShotLatch(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->SYNO_CFG.BIT.rg_latset_otsyn = BASE_CFG_SET;
}

/**
 * @brief Select the pulse that causes global buffer load.
 * @param aptx APT register base address.
 * @param glbLoadEvt The pulse that causes global buffer load.
 *        A logical OR of valid values can be passed as the syncOutSrc parameter.
 *        Valid values for gloLoadTrg are:
 *            APT_GLB_LOAD_ON_CNTR_ZERO       -   Global buffer load when counter equals zero.
 *            APT_GLB_LOAD_ON_CNTR_PERIOD     -   Global buffer load when counter equals period.
 *            APT_GLB_LOAD_ON_CNTR_SYNC       -   Global buffer load when counter sync is effective.
 * @retval None.
 */
static inline void DCL_APT_SetGlobalLoadEvent(APT_RegStruct *aptx, unsigned short glbLoadEvt)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->GLB_LOAD.reg &= (~(0b111 << 0));
    aptx->GLB_LOAD.reg |= (glbLoadEvt << 0);
}

/**
  * @brief Set the prescale value of multiple global buffer load mode.
  * @param aptx APT register base address.
  * @param gldCntPeriod Prescale value of multiple global buffer load mode.
  * @retval None.
  */
static inline void DCL_APT_SetGlobalLoadPrescale(APT_RegStruct *aptx, unsigned short gldCntPeriod)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(gldCntPeriod <= GLOBAL_LOAD_CNT_MAX);
    aptx->GLB_LOAD.BIT.rg_gld_prsc_prd = gldCntPeriod;
}

/**
  * @brief Set the global buffer load mode.
  * @param aptx APT register base address.
  * @param glbLoadMode Global buffer load mode.
  * @retval None.
  */
static inline void DCL_APT_SetGlobalLoadMode(APT_RegStruct *aptx, APT_GlobalLoadMode glbLoadMode)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(glbLoadMode >= APT_GLB_LOAD_ONE_SHOT_MODE);
    APT_PARAM_CHECK_NO_RET(glbLoadMode <= APT_GLB_LOAD_MULTIPLE_MODE);
    aptx->GLB_LOAD.BIT.rg_mode_gld = glbLoadMode;
}

/**
 * @brief When in one-shot global buffer load mode, this function is called to turn the one-shot latch condition ON.
 *        Upon occurrence of a chosen global buffer load event, the registers that is set to global buffer load mode
 *        will load the buffer, and the one-shot latch will be cleared. Hence writing 1 to rg_latset_otgld will
 *        allow a global buffer load event to pass through and block other global buffer load event.
 * @param aptx APT register base address.
 * @retval None.
 */
static inline void DCL_APT_SetGlobalLoadOneShotLatch(APT_RegStruct *aptx)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    aptx->GLB_LOAD.BIT.rg_latset_otgld = BASE_CFG_SET;
}

/**
 * @brief Get buffer status of the registers that enable buffer load.
 * @param aptx The base address of APT module.
 * @param regBuf The buffer of the registers that enable buffer load.
 * @retval true: The register buffer is full.
 * @retval false: The register buffer is not full.
 */
static inline bool DCL_APT_GetRegBufferStatus(APT_RegStruct *aptx, APT_RegBuffer regBuf)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_WITH_RET(regBuf <= APT_REG_BUFFER_DG_CFG, false);
    return ((aptx->LOAD_STS.reg & regBuf) == regBuf);
}

/**
  * @brief Generate a synchronization force event.
  * @param aptx The base address of APT module.
  * @param frcEvt Synchronization force event.
  * @retval None.
  */
static inline void DCL_APT_ForceEvent(APT_RegStruct *aptx, APT_ForceEvtType frcEvt)
{
    APT_ASSERT_PARAM(IsAPTInstance(aptx));
    APT_PARAM_CHECK_NO_RET(frcEvt <= APT_FORCE_EVENT_PWM_ACTION_BUF_LOAD);
    aptx->SYN_FRC.reg |= frcEvt;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_APT_IP_H */
