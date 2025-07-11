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
  * @file      iwdg_ex.h
  * @author    MCU Driver Team
  * @brief     IWDG module driver
  * @details   The header file contains the following declaration:
  *             + IWDG Set And Get Functions.
  */

#ifndef McuMagicTag_IWDG_EX_H
#define McuMagicTag_IWDG_EX_H

/* Includes ------------------------------------------------------------------*/
#include "iwdg.h"
/**
  * @addtogroup IWDG_IP
  * @{
  */

/**
  * @defgroup IWDG_API_EX_Declaration IWDG HAL API EX
  * @{
  */
void HAL_IWDG_SetWindowValueEx(IWDG_Handle *handle, unsigned int windowValue, IWDG_TimeType timeType);
unsigned int HAL_IWDG_GetWindowValueEx(IWDG_Handle *handle);
void HAL_IWDG_EnableWindowModeEx(IWDG_Handle *handle);
void HAL_IWDG_DisableWindowModeEx(IWDG_Handle *handle);
/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_IWDG_H */