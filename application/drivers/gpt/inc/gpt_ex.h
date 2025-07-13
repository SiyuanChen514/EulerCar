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
  * @file      gpt_ex.h
  * @author    MCU Driver Team
  * @brief     GPT module driver.
  * @details   This file provides firmware functions to manage the extension
  *            functionalities of the GPT.
  */

#ifndef McuMagicTag_GPT_EX_H
#define McuMagicTag_GPT_EX_H

#include "gpt.h"

/**
  * @addtogroup GPT_IP
  * @{
  */

/**
  * @defgroup GPT_EX_API_Declaration GPT HAL API EX
  * @{
  */

/* The current count value of the counter. */
unsigned int HAL_GPT_GetCounterValueEx(GPT_Handle *handle);

/* Period trigger for DMA and ADC. */
BASE_StatusType HAL_GPT_TriggerDMAEnableEx(GPT_Handle *handle, GPT_TriggerDMAType triggerDMAType);

BASE_StatusType HAL_GPT_TriggerDMADisableEx(GPT_Handle *handle, GPT_TriggerDMAType triggerDMAType);

BASE_StatusType HAL_GPT_TriggerADCEnableEx(GPT_Handle *handle, GPT_TriggerADCType triggerADCType);

BASE_StatusType HAL_GPT_TriggerADCDisableEx(GPT_Handle *handle, GPT_TriggerADCType triggerADCType);

/* Current PWM Number, Only valid when PWM0_CFG.rg_pwm0_keep = 1 */
unsigned int HAL_GPT_GetCurrentPWM0NumberEx(GPT_Handle *handle);

/* Injected PWM output completion interrupt, which takes effect only when PWM waves are output. */
BASE_StatusType HAL_GPT_SoftInjOutFinIntEx(GPT_Handle *handle, GPT_SetOption softInjOutFin);

/* Injected PWM period finish interrupt, which takes effect only when PWM waves are output. */
BASE_StatusType HAL_GPT_SoftInjPeriodFinIntEx(GPT_Handle *handle, GPT_SetOption softInjPeriod);

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_GPT_EX_H */