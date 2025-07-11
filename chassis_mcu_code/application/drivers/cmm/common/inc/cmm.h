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
  * @file      cmm.h
  * @author    MCU Driver Team
  * @brief     CMM module driver.
  * @details   This file provides firmware CMM Handle structure and Functions
  *            prototypes to manage the following functionalities of the CMM module.
  *             + Initialization and de-initialization functions
  *             + config the register of CMM module
  */

#ifndef McuMagicTag_CMM_H
#define McuMagicTag_CMM_H

/* Includes ------------------------------------------------------------------ */
#include "cmm_ip.h"

/* Macro definitions ---------------------------------------------------------*/
/**
  * @defgroup CMM CMM
  * @brief CMM module.
  * @{
  */

/**
  * @defgroup CMM_Common CMM Common
  * @brief CMM common external module.
  * @{
  */

/**
  * @defgroup CMM_Handle_Definition CMM Handle Definition
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/

/**
  * @brief CMM module configurable item.
  */
typedef enum {
    CMM_CFG_TRIGGER_MODE = 0x00000001U,
    CMM_CFG_TARGET_SOURCE = 0x00000002U,
    CMM_CFG_TARGET_FREQ_DIV = 0x00000003U,
    CMM_CFG_REF_SOURCE = 0x00000004U,
    CMM_CFG_REF_FREQ_DIV = 0x00000005U,
    CMM_CFG_UPPER_BOUND = 0x00000006U,
    CMM_CFG_LOWER_BOUND = 0x00000007U,
    CMM_CFG_INT_TYPE = 0x00000008U,
    CMM_CFG_MAX
} CMM_CFG_TYPE;

/**
  * @brief CMM handle.
  */
typedef struct _CMM_Handle {
    CMM_RegStruct *baseAddress;                    /**< CMM registers base address. */
    CMM_Trigger_Mode mode;                         /**< Effective edge of the target clock. */
    CMM_Target_Freq_Div_Value targetFreqDivision;  /**< Frequency divider of the working target clock. */
    CMM_Ref_Freq_Div_Value refFreqDivision;        /**< Frequency divider of the working reference clock. */
    CMM_Target_Clock_Source targetClockSource;     /**< Working target clock source selection. */
    CMM_Ref_Clock_Source refClockSource;           /**< Working reference clock source selection. */
    unsigned short upperBound;                     /**< Upper bound of window. */
    unsigned short lowerBound;                     /**< Lower bound of window. */
    CMM_Interrupt_Type interruptType;              /**< Enabled interrupt type. */
    CMM_UserCallBack  userCallBack;                /**< CMM Interrupt callback functions.*/
    CMM_ExtendHandle  handleEx;                    /**< CMM extend parameter */
} CMM_Handle;

/**
  * @brief  Typedef callback function of CMM
  */
typedef void (*CMM_CallBackFuncType)(void *handle);

/**
  * @}
  */
 
/**
  * @defgroup CMM_API_Declaration CMM HAL API
  * @{
  */

/* Hardware abstraction layer functions -------------------------------------------------------- */
BASE_StatusType HAL_CMM_Init(CMM_Handle *handle);
BASE_StatusType HAL_CMM_DeInit(CMM_Handle *handle);
BASE_StatusType HAL_CMM_Config(CMM_Handle *handle, CMM_CFG_TYPE cfgType);
void HAL_CMM_GetConfig(CMM_Handle *handle);
void HAL_CMM_Start(CMM_Handle *handle);
void HAL_CMM_Stop(CMM_Handle *handle);
BASE_StatusType HAL_CMM_RegisterCallback(CMM_Handle *handle, CMM_Interrupt_Type type, CMM_CallBackFuncType callback);
void HAL_CMM_IrqHandler(void *handle);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_UART_H */