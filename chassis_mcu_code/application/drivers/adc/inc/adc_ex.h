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
  * @file    adc_ex.h
  * @author  MCU Driver Team
  * @brief   ADC module driver
  * @details This file provides functions declaration of the ADC extend function.
  */

/* Macro definitions */
#ifndef McuMagicTag_ADC_EX_H
#define McuMagicTag_ADC_EX_H

#include "adc.h"
#define ADC_ANA_MUX ((ADC_ANA_MUX_APB_RegStruct *)0x18003000)

/**
 * @addtogroup ADC_IP
 * @{
 */

/**
 * @defgroup ADC_EX_API_Declaration ADC HAL API EX
 * @{
 */
BASE_StatusType HAL_ADC_EnableSocCotinueModeEx(ADC_Handle *adcHandle, ADC_SOCNumber soc);
BASE_StatusType HAL_ADC_DisableSocCotinueModeEx(ADC_Handle *adcHandle, ADC_SOCNumber soc);
BASE_StatusType HAL_ADC_GetControllerStatusEx(ADC_Handle *adcHandle);
BASE_StatusType HAL_ADC_CheckOversamplingFinishEx(ADC_Handle *adcHandle);
unsigned int HAL_ADC_GetOversamplingResultEx(ADC_Handle *adcHandle);
BASE_StatusType HAL_ADC_ConfigureOversamplingEx(ADC_Handle *adcHandle, ADC_SOCNumber soc, ADC_OversamplingParam *param);
BASE_StatusType HAL_ADC_ConfigureWorkModeEx(ADC_Handle *adcHandle, ADC_WorkMode mode);
BASE_StatusType HAL_ADC_EnablePPBxEventIntEx(ADC_Handle *adcHandle);
BASE_StatusType HAL_ADC_DisablePPBxEventIntEx(ADC_Handle *adcHandle);
BASE_StatusType HAL_ADC_ConfigurePPBxEx(ADC_Handle *adcHandle, ADC_SOCNumber soc, ADC_PPBNumber ppb,
                                        PPB_Function *fun);
BASE_StatusType HAL_ADC_SetPPBxOffsetEx(ADC_Handle *adcHandle, ADC_PPBNumber ppb, int offset);
BASE_StatusType HAL_ADC_SetPPBxThresholdEx(ADC_Handle *adcHandle, ADC_PPBNumber ppb, int up, int dn);
BASE_StatusType HAL_ADC_SetPPBxErrorRefEx(ADC_Handle *adcHandle, ADC_PPBNumber ppb, unsigned int ref);
int HAL_ADC_GetPPBxErrorResultEx(ADC_Handle *adcHandle, ADC_PPBNumber ppb);
unsigned int HAL_ADC_GetPPBxDelayCntEx(ADC_Handle *adcHandle, ADC_PPBNumber ppb);
/**
  * @}
  */

/**
  * @}
  */
#endif