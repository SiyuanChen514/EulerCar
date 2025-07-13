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
  * @file      iwdg.h
  * @author    MCU Driver Team
  * @brief     IWDG module driver
  * @details   The header file contains the following declaration:
  *             + IWDG handle structure definition.
  *             + Initialization functions.
  *             + IWDG Set And Get Functions.
  *             + Interrupt Handler Functions.
  */

#ifndef McuMagicTag_IWDG_H
#define McuMagicTag_IWDG_H

/* Includes ------------------------------------------------------------------*/
#include "iwdg_ip.h"
/**
  * @defgroup IWDG IWDG
  * @brief IWDG module.
  * @{
  */

/**
  * @defgroup IWDG_Common IWDG Common
  * @brief IWDG common external module.
  * @{
  */

/**
  * @defgroup IWDG_Handle_Definition IWDG Handle Definition
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/
typedef void (* IWDG_CallbackType)(void *handle);

/**
  * @brief IWDG handle structure definition.
  */
typedef struct _IWDG_Handle {
    IWDG_RegStruct     *baseAddress;     /**< IWDG Registers address. */
    unsigned int        timeValue;       /**< IWDG time value. */
    unsigned int        freqDivValue;    /**< IWDG freq div value. */
    IWDG_TimeType       timeType;        /**< IWDG time type. */
    bool                enableIT;        /**< true:enable false:disable interrupt. */
    IWDG_UserCallBack   userCallBack;    /**< User callback */
    IWDG_ExtendHandle   handleEx;        /**< IWDG extend parameter */
} IWDG_Handle;

/**
  * @}
  */
 
/**
  * @defgroup IWDG_API_Declaration IWDG HAL API
  * @{
  */

BASE_StatusType HAL_IWDG_Init(IWDG_Handle *handle);
void HAL_IWDG_SetTimeValue(IWDG_Handle *handle, unsigned int timeValue, IWDG_TimeType timeType);
unsigned int HAL_IWDG_GetLoadValue(IWDG_Handle *handle);
unsigned int HAL_IWDG_GetCounterValue(IWDG_Handle *handle);
void HAL_IWDG_Refresh(IWDG_Handle *handle);
void HAL_IWDG_Start(IWDG_Handle *handle);
void HAL_IWDG_Stop(IWDG_Handle *handle);
void HAL_IWDG_RegisterCallback(IWDG_Handle *handle, IWDG_CallbackType callBackFunc);
void HAL_IWDG_IrqHandler(void *handle);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_IWDG_H */