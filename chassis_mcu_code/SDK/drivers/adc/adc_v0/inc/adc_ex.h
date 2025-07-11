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
  * @details This file provides DCL functions to manage ADC and Definition of
  *          specific parameters.
  *           + ADC Synchronous Sampling.
  *           + ADC Software Calibration.
  */
#ifndef McuMagicTag_ADC_EX_H
#define McuMagicTag_ADC_EX_H
#include "adc.h"
/**
 * @addtogroup ADC_IP
 * @{
 */

/**
 * @defgroup ADC_EX_API_Declaration ADC HAL API EX
 * @{
 */
 BASE_StatusType HAL_ADC_StartSyncSampleEx(ADC_Handle *adcHandle, SOC_SyncParam *syncParam);
 unsigned int HAL_ADC_ActiveCalibrateRetEx(ADC_RegStruct * const adcx, unsigned int soc, unsigned int originalRet);
 /**
  * @}
  */

/**
  * @}
  */
#endif