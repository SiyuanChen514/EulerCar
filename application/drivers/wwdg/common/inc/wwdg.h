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
  * @file      wwdg.h
  * @author    MCU Driver Team
  * @brief     WWDG module driver
  * @details   The header file contains the following declaration:
  *             + WWDG handle structure definition.
  *             + Initialization functions.
  *             + WWDG Set And Get Functions.
  *             + Interrupt Handler Functions.
  */

#ifndef McuMagicTag_WWDG_H
#define McuMagicTag_WWDG_H

/* Includes ------------------------------------------------------------------*/
#include "wwdg_ip.h"
/**
  * @defgroup WWDG WWDG
  * @brief WWDG module.
  * @{
  */

/**
  * @defgroup WWDG_Common WWDG Common
  * @brief WWDG common external module.
  * @{
  */

/**
  * @defgroup WWDG_Handle_Definition WWDG Handle Definition
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/
typedef void (* WWDG_CallbackType)(void *handle);

/**
  * @brief WWDG handle structure definition.
  */
typedef struct _WWDG_Handle {
    WWDG_RegStruct     *baseAddress;      /**< WWDG Registers address. */
    unsigned int        timeValue;        /**< WWDG time value. */
    unsigned int        windowValue;      /**< WWDG window value. */
    unsigned int        freqDivValue;     /**< WWDG freq div value. */
    WWDG_TimeType       timeType;         /**< WWDG time type. */
    bool                enableIT;         /**< true:enable false:disable interrupt. */
    WWDG_UserCallBack   userCallBack;     /**< User callback */
    WWDG_ExtendHandle   handleEx;         /**< WWDG extend parameter */
} WWDG_Handle;

/**
  * @}
  */
 
/**
  * @defgroup WWDG_API_Declaration WWDG HAL API
  * @{
  */

BASE_StatusType HAL_WWDG_Init(WWDG_Handle *handle);
void HAL_WWDG_SetTimeValue(WWDG_Handle *handle, unsigned int timeValue, WWDG_TimeType timeType);
unsigned int HAL_WWDG_GetLoadValue(WWDG_Handle *handle);
unsigned int HAL_WWDG_GetWindowValue(WWDG_Handle *handle);
unsigned int HAL_WWDG_GetCounterValue(WWDG_Handle *handle);
void HAL_WWDG_Refresh(WWDG_Handle *handle);
void HAL_WWDG_Start(WWDG_Handle *handle);
void HAL_WWDG_Stop(WWDG_Handle *handle);
void HAL_WWDG_RegisterCallback(WWDG_Handle *handle, WWDG_CallbackType callBackFunc);
void HAL_WWDG_IrqHandler(void *handle);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_WWDG_H */