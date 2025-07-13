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
  * @file      gpio.c
  * @author    MCU Driver Team
  * @brief     GPIO module driver
  * @details   This file provides firmware functions to manage the following functionalities of the GPIO.
  *             + GPIO configuration definetion.
  *             + Initialization functions.
  *             + GPIO Set And Get Functions.
  *             + Interrupt Service Functions.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "gpio.h"

static void GPIO_ExcuteCallBack(GPIO_Handle *handle, GPIO_PIN pin);
static void GPIO_SetLevelIrqType(GPIO_Handle *handle, unsigned int pins, GPIO_InterruptMode mode);
static void GPIO_SetEdgeIrqType(GPIO_Handle *handle, unsigned int pins, GPIO_InterruptMode mode);

/**
  * @brief Initializing GPIO register values.
  * @param handle Value of @ref GPIO_Handle.
  * @retval None.
  */
void HAL_GPIO_Init(GPIO_Handle *handle)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(handle->pins));

    /* Register GPIO callback ID */
    for (unsigned int i = 0; i < GPIO_PIN_NUM; i++) {
        handle->userCallBack.GPIO_CallbackFuncs[i].pin = (1 << i);
    }
}

/**
  * @brief DeInitializing GPIO register values.
  * @param handle Value of @ref GPIO_Handle.
  * @retval None.
  */
void HAL_GPIO_DeInit(GPIO_Handle *handle)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    /* Clean GPIO callback ID and interrupt callback functions. */
    for (unsigned int i = 0; i < GPIO_PIN_NUM; i++) {
        handle->userCallBack.GPIO_CallbackFuncs[i].pin = 0x00000000;
        handle->userCallBack.GPIO_CallbackFuncs[i].callbackFunc = NULL;
    }
    handle->pins = 0x00000000; /* Reset GPIO pins. */
}

/**
  * @brief Setting GPIO pins direction.
  * @param handle Value of @ref GPIO_Handle.
  * @param pins OR logical combination of pin.
  * @param dir GPIO pin direction.
  * @retval None.
  */
void HAL_GPIO_SetDirection(GPIO_Handle *handle, unsigned int pins, GPIO_Direction dir)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    GPIO_PARAM_CHECK_NO_RET(IsGpioDirection(dir));
    DCL_GPIO_SetDirection(handle->baseAddress, pins, dir);
}

/**
  * @brief Setting GPIO pins level
  * @param handle Value of @ref GPIO_Handle.
  * @param pins OR logical combination of pin.
  * @param value Value of @ref GPIO_Value.
  * @retval None.
  */
void HAL_GPIO_SetValue(GPIO_Handle *handle, unsigned int pins, GPIO_Value value)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    GPIO_PARAM_CHECK_NO_RET(IsGpioValue(value));
    DCL_GPIO_SetValue(handle->baseAddress, pins, value);
}

/**
  * @brief Getting GPIO pin level
  * @param handle Value of @ref GPIO_Handle.
  * @param pin Value of @ref GPIO_PIN.
  * @retval GPIO_Value Value of @ref GPIO_Value.
  */
GPIO_Value HAL_GPIO_GetPinValue(GPIO_Handle *handle, GPIO_PIN pin)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    GPIO_ASSERT_PARAM(IsGpioPin(pin));
    return (handle->baseAddress->GPIO_DATA[GPIO_PIN_MASK].reg & pin) == BASE_CFG_UNSET ? \
                                                                        GPIO_LOW_LEVEL : GPIO_HIGH_LEVEL;
}

/**
  * @brief Getting GPIO pins level
  * @param handle Value of @ref GPIO_Handle.
  * @retval unsigned int Value of all GPIO pin.
  */
unsigned int HAL_GPIO_GetAllValue(GPIO_Handle *handle)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    return handle->baseAddress->GPIO_DATA[GPIO_PIN_MASK].reg & GPIO_PIN_MASK;
}

/**
  * @brief Getting GPIO pin direction
  * @param handle Value of @ref GPIO_Handle.
  * @param pin GPIO pin.
  * @retval Value of @ref BASE_StatusType.
  */
GPIO_Direction HAL_GPIO_GetPinDirection(GPIO_Handle *handle, GPIO_PIN pin)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    GPIO_ASSERT_PARAM(IsGpioPin(pin));
    return (handle->baseAddress->GPIO_DIR.reg & pin) == BASE_CFG_UNSET ? GPIO_INPUT_MODE : GPIO_OUTPUT_MODE;
}

/**
  * @brief Getting GPIO pins direction
  * @param handle Value of @ref GPIO_Handle.
  * @retval Value of @ref BASE_StatusType.
  */
unsigned int HAL_GPIO_GetAllDirection(GPIO_Handle *handle)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    return handle->baseAddress->GPIO_DIR.reg & GPIO_PIN_MASK;
}

/**
  * @brief Toggle GPIO level
  * @param handle Value of @ref GPIO_Handle.
  * @param pins GPIO pins.
  * @retval None.
  */
void HAL_GPIO_TogglePin(GPIO_Handle *handle, unsigned int pins)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    handle->baseAddress->GPIO_DATA[pins].reg ^= pins;
}

/**
  * @brief Get GPIO pin interrupt types.
  * @param handle Value of @ref GPIO_Handle.
  * @param pin Value of @ref GPIO_PIN.
  * @retval GPIO_InterruptMode Value of @ref GPIO_InterruptMode.
  */
GPIO_InterruptMode HAL_GPIO_GetPinIrqType(GPIO_Handle *handle, GPIO_PIN pin)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    GPIO_ASSERT_PARAM(IsGpioPin(pin));
    /* If disable pin interrupt, return None mode  */
    if ((handle->baseAddress->GPIO_IE.reg & pin) == BASE_CFG_UNSET) {
        return GPIO_INT_TYPE_NONE;
    }
    unsigned int iev = ((handle->baseAddress->GPIO_IEV.reg & pin) != 0) ? 1 : 0; /* 1: iev effect. */
    unsigned int is = ((handle->baseAddress->GPIO_IS.reg & pin) != 0) ? 2 : 0; /* 2: is effect. */
    unsigned int ibe = ((handle->baseAddress->GPIO_IBE.reg & pin) != 0) ? 4 : 0; /* 4: ibe effect. */
    unsigned int value = (iev | is | ibe);
    if (value >= GPIO_INT_TYPE_NONE) {
        return GPIO_INT_TYPE_NONE;
    }
    return value;
}

/**
  * @brief Set GPIO level interrupt types.
  * @param handle Value of @ref GPIO_Handle.
  * @param pins OR logical combination of pin.
  * @param mode Value of @ref GPIO_InterruptMode.
  * @retval None.
  */
static void GPIO_SetLevelIrqType(GPIO_Handle *handle, unsigned int pins, GPIO_InterruptMode mode)
{
    handle->baseAddress->GPIO_IBE.reg &= ~pins; /* Disable edge detection */
    handle->baseAddress->GPIO_IS.reg |= pins; /* Enable level detection */
    if (mode == GPIO_INT_TYPE_HIGH_LEVEL) {
        handle->baseAddress->GPIO_IEV.reg |= pins;
    } else {
        handle->baseAddress->GPIO_IEV.reg &= ~pins;
    }
}

/**
  * @brief Set GPIO edge interrupt types.
  * @param handle Value of @ref GPIO_Handle.
  * @param pins OR logical combination of pin.
  * @param mode Value of @ref GPIO_InterruptMode.
  * @retval None.
  */
static void GPIO_SetEdgeIrqType(GPIO_Handle *handle, unsigned int pins, GPIO_InterruptMode mode)
{
    handle->baseAddress->GPIO_IS.reg &= ~pins; /* Disable level detection. */
    handle->baseAddress->GPIO_IBE.reg &= ~pins; /* Clear detection on both edges. */
    if (mode == GPIO_INT_TYPE_RISE_EDGE) {
        handle->baseAddress->GPIO_IEV.reg |= pins;
    } else {
        handle->baseAddress->GPIO_IEV.reg &= ~pins;
    }
}

/**
  * @brief Setting GPIO interrupt mode.
  * @param handle Value of @ref GPIO_Handle.
  * @param pins OR logical combination of pin.
  * @param mode Value of @ref GPIO_InterruptMode.
  * @retval Value of @ref BASE_StatusType.
  */
BASE_StatusType HAL_GPIO_SetIrqType(GPIO_Handle *handle, unsigned int pins, GPIO_InterruptMode mode)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    GPIO_PARAM_CHECK_WITH_RET(IsGpioPins(pins), BASE_STATUS_ERROR);
    GPIO_PARAM_CHECK_WITH_RET(IsGpioITMode(mode), BASE_STATUS_ERROR);

    /* It must be disabled to avoid triggering interrupts during configuration. */
    DCL_GPIO_DisableIrq(handle->baseAddress, pins);

    if ((mode == GPIO_INT_TYPE_HIGH_LEVEL) || (mode == GPIO_INT_TYPE_LOW_LEVEL)) {
        GPIO_SetLevelIrqType(handle, pins, mode);
    } else if (mode == GPIO_INT_TYPE_BOTH_EDGE) {
        handle->baseAddress->GPIO_IEV.reg &= ~pins;
        handle->baseAddress->GPIO_IS.reg &= ~pins;
        handle->baseAddress->GPIO_IBE.reg |= pins;
    } else if ((mode == GPIO_INT_TYPE_RISE_EDGE) || (mode == GPIO_INT_TYPE_FALL_EDGE)) {
        GPIO_SetEdgeIrqType(handle, pins, mode);
    } else if (mode == GPIO_INT_TYPE_NONE) {
        /* No interruptMode: disable everything. */
        handle->baseAddress->GPIO_IEV.reg &= ~pins;
        handle->baseAddress->GPIO_IS.reg &= ~pins;
        handle->baseAddress->GPIO_IBE.reg &= ~pins;
        return BASE_STATUS_ERROR;
    }

    DCL_GPIO_EnableIrq(handle->baseAddress, pins);
    return BASE_STATUS_OK;
}

/**
  * @brief Handle GPIO interrupt request.
  * @param handle Value of @ref GPIO_Handle.
  * @param pin Value of @ref GPIO_PIN.
  * @retval None.
  */
static void GPIO_ExcuteCallBack(GPIO_Handle *handle, GPIO_PIN pin)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pin));
    for (unsigned int i = 0; i < GPIO_PIN_NUM; i++) {
        if (handle->userCallBack.GPIO_CallbackFuncs[i].pin == pin) {
            if (handle->userCallBack.GPIO_CallbackFuncs[i].callbackFunc != NULL) {
                handle->userCallBack.GPIO_CallbackFuncs[i].callbackFunc(handle);
            }
        }
    }
}

/**
  * @brief Handle GPIO interrupt request.
  * @param handle Interrupt parameter.
  * @retval None.
  */
void HAL_GPIO_IrqHandler(void *handle)
{
    GPIO_Handle *gpioHandle = (GPIO_Handle *)handle;
    GPIO_ASSERT_PARAM(gpioHandle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpioHandle->baseAddress));
    unsigned int position = 0x00000000U;
    unsigned int pinCurrent = 0x00000000U;
    unsigned int mis = DCL_GPIO_GetMIS(gpioHandle->baseAddress); /* Queries the masked GPIO interrupt status. */

    /* Determine which pin sets the callback function. */
    while ((mis >> position) != BASE_CFG_UNSET) {
        pinCurrent = mis & (1 << position);
        if (pinCurrent) {
            gpioHandle->pins = pinCurrent;
            DCL_GPIO_ClearIrq(gpioHandle->baseAddress, pinCurrent);
            GPIO_ExcuteCallBack(gpioHandle, pinCurrent);
        }
        position++;
    }
}

/**
  * @brief Handle GPIO interrupt request.
  * @param handle Value of @ref GPIO_Handle.
  * @param pin Value of @ref GPIO_PIN.
  * @param pCallback Value of @ref GPIO_CallbackType.
  * @retval None.
  */
void HAL_GPIO_RegisterCallBack(GPIO_Handle *handle, GPIO_PIN pin, GPIO_CallbackType pCallback)
{
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_ASSERT_PARAM(IsGPIOInstance(handle->baseAddress));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPin(pin));
    for (unsigned int i = 0; i < GPIO_PIN_NUM; i++) {
        if (handle->userCallBack.GPIO_CallbackFuncs[i].pin == pin) {
            handle->userCallBack.GPIO_CallbackFuncs[i].callbackFunc = pCallback;
        }
    }
}