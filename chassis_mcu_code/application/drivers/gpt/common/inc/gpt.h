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
  * @file      gpt.c
  * @author    MCU Driver Team
  * @brief     GPT module driver.
  * @details   This file provides firmware GPT Handle Structure and functions
  *            prototypes  to manage the following functionalities of the GPT.
  *               + Initialization and de-initialization functions
  *               + config the register of GPT
  *               + interrupt register and register functions
  */

#ifndef McuMagicTag_GPT_H
#define McuMagicTag_GPT_H

/* Includes-------------------------------------------------------------------*/
#include "gpt_ip.h"

/**
 * @defgroup GPT GPT
 * @brief GPT module.
 * @{
 */

/**
 * @defgroup GPT_Common GPT Common
 * @brief GPT common external module.
 * @{
 */

/**
 * @defgroup GPT_Handle_Definition GPT Handle Definition
 * @{
 */
typedef struct {
    GPT_RegStruct                   *baseAddress;           /**< Base address of GPT register. */
    GPT_CountMode                    cntMode;               /**< GPT count mode. */
    unsigned int                     clockDiv;              /**< GPT clock div. */
    volatile GPT_RefValueAction      refA0;                 /**< GPT refA0 action setting. */
    volatile GPT_RefValueAction      refB0;                 /**< GPT refB0 action setting. */
    volatile unsigned int            period;                /**< PWM period. */
    volatile unsigned int            pwmNum;                /**< PWM number, only valid when pwmKeep is false. */
    bool                             pwmKeep;               /**< PWM output mode. */
    bool                             bufLoad;               /**< Indicates whether the cache is loaded immediately. */
    bool                             triggleAdcPeriod;      /**< triggle ADC when PWM counting period out finish. */
    bool                             triggleAdcOutFinish;   /**< triggle ADC when PWM out finish. */

    GPT_UserCallBack                 userCallBack;          /**< User callback function of GPT. */
    GPT_ExtendHandle                 handleEx;              /**< GPT extend handle. */
} GPT_Handle;

typedef void (* GPT_CallBackFunc)(void *handle);

/**
  * @}
  */

/**
  * @defgroup GPT_API_Declaration GPT HAL API
  * @{
  */
/**
 * GPT Control functions
 */
BASE_StatusType HAL_GPT_Init(GPT_Handle *handle);

void HAL_GPT_Start(GPT_Handle *handle);

void HAL_GPT_Stop(GPT_Handle *handle);

BASE_StatusType HAL_GPT_Config(GPT_Handle *handle);

BASE_StatusType HAL_GPT_GetConfig(GPT_Handle *handle);

/* Setting PWM reference points and corresponding actions */
BASE_StatusType HAL_GPT_SetReferCounterAndAction(GPT_Handle *handle, const GPT_ReferCfg *refer);

void HAL_GPT_GetReferCounterAndAction(GPT_Handle *handle, GPT_ReferCfg *refer);

/* GPT frequency divider and period. */
BASE_StatusType HAL_GPT_SetCountPeriod(GPT_Handle *handle, unsigned int period);

unsigned int HAL_GPT_GetCountPeriod(GPT_Handle *handle);

BASE_StatusType HAL_GPT_SetDivFactor(GPT_Handle *handle, unsigned int div);

unsigned int HAL_GPT_GetDivFactor(GPT_Handle *handle);

/* GPT cache loading settings and cache status. */
BASE_StatusType HAL_GPT_SetBufferLoad(GPT_Handle *handle, GPT_SetOption bufferLoad);

unsigned int HAL_GPT_GetBufferLoadStatus(GPT_Handle *handle);

/* Output completion interrupt configuration for the GPT channel. */
BASE_StatusType HAL_GPT_SetOutFinishInt(GPT_Handle *handle, GPT_SetOption outFinishInt);

/* GPT period interrupt configuration. */
BASE_StatusType HAL_GPT_SetPeriodInt(GPT_Handle *handle, GPT_SetOption periodInt);

/* GPT interrupt service and callback registration functions */
void HAL_GPT_IrqOutFinishHandler(void *handle);

void HAL_GPT_IrqPeriodHandler(void *handle);

BASE_StatusType HAL_GPT_RegisterCallBack(GPT_Handle *gptHandle, GPT_CallBackFunType typeID,
                                         GPT_CallBackFunc pCallback);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_GPT_H */