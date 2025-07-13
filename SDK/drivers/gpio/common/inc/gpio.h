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
  * @file      gpio.h
  * @author    MCU Driver Team
  * @brief     GPIO module driver
  * @details   The header file contains the following declaration:
  *             + GPIO handle structure definition.
  *             + Initialization functions.
  *             + GPIO Set And Get Functions.
  *             + Interrupt Service Functions.
  */

#ifndef McuMagicTag_GPIO_H
#define McuMagicTag_GPIO_H

/* Includes ------------------------------------------------------------------*/
#include "gpio_ip.h"

/* Macro definition */
/**
  * @defgroup GPIO GPIO
  * @brief GPIO module.
  * @{
  */

/**
  * @defgroup GPIO_Common GPIO Common
  * @brief GPIO common external module.
  * @{
  */

/**
  * @defgroup GPIO_Handle_Definition GPIO Handle Definition
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/
typedef void (* GPIO_CallbackType)(void *param);

typedef struct _GPIO_Handle {
    GPIO_RegStruct     *baseAddress;      /**< GPIO Registers. */
    unsigned int        pins;             /**< Selected GPIO Pins. */
    GPIO_UserCallBcak   userCallBack;     /**< User-defined callback function. */
    GPIO_ExtendHandle   handleEx;         /**< Extend handle, configuring some special parameters. */
} GPIO_Handle;

/**
  * @}
  */
 
/**
  * @defgroup GPIO_API_Declaration GPIO HAL API
  * @{
  */
void HAL_GPIO_Init(GPIO_Handle *handle);
void HAL_GPIO_DeInit(GPIO_Handle *handle);
void HAL_GPIO_SetDirection(GPIO_Handle *handle, unsigned int pins, GPIO_Direction dir);
void HAL_GPIO_SetValue(GPIO_Handle *handle, unsigned int pins, GPIO_Value value);
GPIO_InterruptMode HAL_GPIO_GetPinIrqType(GPIO_Handle *handle, GPIO_PIN pin);
GPIO_Value HAL_GPIO_GetPinValue(GPIO_Handle *handle, GPIO_PIN pin);
unsigned int HAL_GPIO_GetAllValue(GPIO_Handle *handle);
GPIO_Direction HAL_GPIO_GetPinDirection(GPIO_Handle *handle, GPIO_PIN pin);
unsigned int HAL_GPIO_GetAllDirection(GPIO_Handle *handle);
void HAL_GPIO_TogglePin(GPIO_Handle *handle, unsigned int pins);
BASE_StatusType HAL_GPIO_SetIrqType(GPIO_Handle *handle, unsigned int pins, GPIO_InterruptMode mode);
void HAL_GPIO_RegisterCallBack(GPIO_Handle *handle, GPIO_PIN pin, GPIO_CallbackType pCallback);
void HAL_GPIO_IrqHandler(void *handle);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_GPIO_H */