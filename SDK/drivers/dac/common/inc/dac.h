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
 * @file    dac.h
 * @author  MCU Driver Team.
 * @brief   DAC module driver.
 *          This file provides functions declaration of the Comparator.
 *           + DAC's Initialization and de-initialization functions
 *           + Set DAC value function
 */
#ifndef McuMagicTag_DAC_H
#define McuMagicTag_DAC_H

#include "dac_ip.h"

/**
  * @defgroup DAC DAC
  * @brief DAC module.
  * @{
  */

/**
  * @defgroup DAC_Common DAC Common
  * @brief DAC common external module.
  * @{
  */

/**
  * @defgroup DAC_Handle_Definition DAC Handle Definition
  * @{
  */

/**
  * @brief DAC Handle
  */
typedef struct _DAC_Handle {
    DAC_RegStruct            *baseAddress;       /**< DAC registers base address. */
    volatile unsigned int     dacValue;          /**< DAC configuration value. */

    DAC_ExtendHandle          handleEx;          /* DAC Handle Ex. */
} DAC_Handle;

/**
  * @}
  */

/**
  * @defgroup DAC_API_Declaration DAC HAL API
  * @{
  */
/* DAC APIs */
BASE_StatusType HAL_DAC_Init(DAC_Handle *dacHandle);
BASE_StatusType HAL_DAC_DeInit(DAC_Handle *dacHandle);
void HAL_DAC_SetValue(DAC_Handle *dacHandle, unsigned int value);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif