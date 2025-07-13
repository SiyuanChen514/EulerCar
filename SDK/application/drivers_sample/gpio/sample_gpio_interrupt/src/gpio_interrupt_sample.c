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
  * @file      gpio_interrupt_sample.c
  * @author    MCU Driver Team
  * @brief     GPIO module trigger each interrupt type sample
  * @details   Set two groups of GPIOs. One group is used to provide the trigger condition \
  *            for triggering the interrupt, and the other group is used to provide the interrupt response. The \
  *            pin numbers of the two groups correspond to each other to verify the five interrupt types. If the \
  *            hardware environment does not support this function, you need to set up an environment for verification.
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "gpio.h"
#include "main.h"
#include "gpio_interrupt_sample.h"

#define GPIO_LEVEL_SHIFT_SAFE_TIME    1
static bool g_intFlag = false;
/* prototype functions -------------------------------------------------------*/
void GPIO_CallBackFunc(void *param);
static void GPIO_InterruptTest(void);
/* Macro definitions ---------------------------------------------------------*/

/* ---------------------------------- Sample Parameters -------------------------------- */
/**
  * @brief GPIO Interrupt sample.
  * @param None
  * @retval Value of @ref BASE_StatusType.
  */
void GPIO_InterruptSample(void)
{
    SystemInit();
    DBG_PRINTF("GPIO Interrupt Sample \r\n");
    GPIO_InterruptTest();
}

/**
  * @brief GPIO Interrupt callback function.
  * @param param Value of @ref GPIO_Handle.
  * @retval None
  */
void GPIO_CallBackFunc(void *param)
{
    GPIO_Handle *handle = (GPIO_Handle *)param;
    GPIO_ASSERT_PARAM(handle != NULL);
    GPIO_InterruptMode mode = HAL_GPIO_GetPinIrqType(handle, handle->pins);
    DBG_PRINTF("In intMode[%d]'s callback -- ", mode);
    if (mode == GPIO_INT_TYPE_HIGH_LEVEL) {
        /* reverse trigHandle's value */
        HAL_GPIO_SetValue(&TRIG_HANDLE, TRIG_PIN, GPIO_LOW_LEVEL);
    } else if (mode == GPIO_INT_TYPE_LOW_LEVEL) {
        /* reverse trigHandle's value */
        HAL_GPIO_SetValue(&TRIG_HANDLE, TRIG_PIN, GPIO_HIGH_LEVEL);
    }
    g_intFlag = true;
}
/**
  * @brief Check GPIO Interrupt state.
  * @param None.
  * @retval None
  */
static void CheckGpioIntState(void)
{
    /* true means into interrupt callback, false is not */
    if (g_intFlag == true) {
        DBG_PRINTF("success ! \r\n");
    } else {
        DBG_PRINTF("fail ! \r\n");
    }
    g_intFlag = false; /* resumn status is false */
}
/**
  * @brief GPIO Interrupt test scene one: GPIO2 -> GPIO1.
  * @param trigHandle the handle provide trig source.
  * @param intHandle the handle generate interrupt.
  * @retval None
  */
static void GPIO_InterruptTest(void)
{
    /* Test high level interrupt type */
    DBG_PRINTF(" Test high level interrupt type : trig GPIO -> int GPIO high level-------------------- ");
    HAL_GPIO_SetValue(&TRIG_HANDLE, TRIG_PIN, GPIO_HIGH_LEVEL); /* Provide high level trig condition */
    BASE_FUNC_DELAY_MS(GPIO_LEVEL_SHIFT_SAFE_TIME);
    CheckGpioIntState();
    /* Test low level interrupt type */
    DBG_PRINTF(" Test low level interrupt type : trig GPIO -> int GPIO low level-------------------- ");
    HAL_GPIO_SetIrqType(&INT_HANDLE, INT_PIN, GPIO_INT_TYPE_LOW_LEVEL); /* change interrupt type to low level */
    BASE_FUNC_DELAY_MS(GPIO_LEVEL_SHIFT_SAFE_TIME);
    CheckGpioIntState();
    /* Test rise level interrupt type */
    DBG_PRINTF(" Test rise level interrupt type : trig GPIO -> int GPIO rising edge-------------------- ");
    HAL_GPIO_SetIrqType(&INT_HANDLE, INT_PIN, GPIO_INT_TYPE_RISE_EDGE); /* change interrupt type to rise type */
    HAL_GPIO_SetValue(&TRIG_HANDLE, TRIG_PIN, GPIO_LOW_LEVEL); /* Provide rise level trig condition */
    BASE_FUNC_DELAY_MS(GPIO_LEVEL_SHIFT_SAFE_TIME);
    HAL_GPIO_SetValue(&TRIG_HANDLE, TRIG_PIN, GPIO_HIGH_LEVEL);
    CheckGpioIntState();
    /* Test fall level interrupt type */
    DBG_PRINTF(" Test fall level interrupt type : trig GPIO -> int GPIO falling edge-------------------- ");
    HAL_GPIO_SetIrqType(&INT_HANDLE, INT_PIN, GPIO_INT_TYPE_FALL_EDGE); /* change interrupt type to fall type */
    HAL_GPIO_SetValue(&TRIG_HANDLE, TRIG_PIN, GPIO_LOW_LEVEL);
    BASE_FUNC_DELAY_MS(GPIO_LEVEL_SHIFT_SAFE_TIME);
    CheckGpioIntState();
    /* Test high/low both level interrupt type */
    DBG_PRINTF(" Test high/low both level interrupt type : trig GPIO -> int GPIO both edge-------------------- ");
    HAL_GPIO_SetIrqType(&INT_HANDLE, INT_PIN, GPIO_INT_TYPE_BOTH_EDGE); /* change interrupt type to both type */
    HAL_GPIO_SetValue(&TRIG_HANDLE, TRIG_PIN, GPIO_HIGH_LEVEL);
    BASE_FUNC_DELAY_MS(GPIO_LEVEL_SHIFT_SAFE_TIME);
    HAL_GPIO_SetValue(&TRIG_HANDLE, TRIG_PIN, GPIO_LOW_LEVEL);
    CheckGpioIntState();
}