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
  * @file      gpt.h
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
    GPT_RegStruct              *baseAddress;  /**< Base address of GPT register */
    volatile unsigned int      period;        /**< PWM period, unit ns */
    volatile unsigned int      duty;          /**< PWM duty, unit ns */
    volatile unsigned int      pwmNum;        /**< PWM number, only valid when pwmKeep is false */
    bool                       pwmKeep;       /**< PWM output mode */
    bool                       pwmPolarity;   /**< PWM output positive and negative control */
    bool                       pwmEnable;     /**< PWM Enable */
} GPT_Handle;

/**
  * @}
  */

/**
  * @defgroup GPT_API_Declaration GPT HAL API
  * @{
  */
/**
 * GPT Extended Control functions
 */
BASE_StatusType HAL_GPT_Init(GPT_Handle *handle);

void HAL_GPT_Start(GPT_Handle *handle);

void HAL_GPT_Stop(GPT_Handle *handle);

BASE_StatusType HAL_GPT_Config(GPT_Handle *handle);

BASE_StatusType HAL_GPT_GetConfig(GPT_Handle *handle);
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