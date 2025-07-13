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
  * @file      wdg.h
  * @author    MCU Driver Team
  * @brief     WDG module driver
  * @details   The header file contains the following declaration:
  *             + WDG handle structure definition.
  *             + Initialization functions.
  *             + WDG Set And Get Functions.
  *             + Interrupt Handler Functions.
  */

#ifndef McuMagicTag_WDG_H
#define McuMagicTag_WDG_H

/* Includes ------------------------------------------------------------------*/
#include "wdg_ip.h"
/**
  * @defgroup WDG WDG
  * @brief WDG module.
  * @{
  */

/**
  * @defgroup WDG_Common WDG Common
  * @brief WDG common external module.
  * @{
  */

/**
  * @defgroup WDG_Handle_Definition WDG Handle Definition
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/
typedef void (* WDG_CallbackType)(void *handle);

/**
  * @brief WDG handle structure definition.
  */
typedef struct _WDG_Handle {
    WDG_RegStruct     *baseAddress;     /**< WDG Registers address. */
    unsigned int       timeValue;       /**< WDG time value. */
    WDG_TimeType       timeType;        /**< WDG time type. */
    bool               enableIT;        /**< true:enable false:disable interrupt. */
    WDG_UserCallBack   userCallBack;    /**< User callback */
    WDG_ExtendHandle   handleEx;        /**< WDG extend parameter */
} WDG_Handle;

/**
  * @}
  */
 
/**
  * @defgroup WDG_API_Declaration WDG HAL API
  * @{
  */

BASE_StatusType HAL_WDG_Init(WDG_Handle *handle);
void HAL_WDG_SetTimeValue(WDG_Handle *handle, unsigned int timeValue, WDG_TimeType timeType);
unsigned int HAL_WDG_GetLoadValue(WDG_Handle *handle);
unsigned int HAL_WDG_GetCounterValue(WDG_Handle *handle);
void HAL_WDG_Refresh(WDG_Handle *handle);
void HAL_WDG_Start(WDG_Handle *handle);
void HAL_WDG_Stop(WDG_Handle *handle);
void HAL_WDG_RegisterCallback(WDG_Handle *handle, WDG_CallbackType callBackFunc);
void HAL_WDG_IrqHandler(void *handle);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_WDG_H */