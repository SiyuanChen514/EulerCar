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
  * @file      gpio_circle_sample.c
  * @author    MCU Driver Team
  * @brief     GPIO module circle sample
  * @details   Two groups of GPIOs are configured. One group is used to transmit signals and \
  *            the other group is used to receive signals. Then, a loopback is formed. The pin numbers of the \
  *            two groups correspond to the level and direction functions of the GPIOs. If the hardware environment \
  *            does not support this function, you need to set up an environment for verification.
  */

/* ---------------------------------- Includes -------------------------------- */
#include "debug.h"
#include "gpio.h"
#include "main.h"
#include "gpio_circle_sample.h"

/* ---------------------------------- Sample Parameters -------------------------------- */
static BASE_StatusType GpioLoopBackTest(void);
/**
  * @brief Handle GPIO circle loopBack sample.
  * @param None
  * @retval Value of @ref BASE_StatusType.
  */
BASE_StatusType GPIO_CircleSample(void)
{
    SystemInit();

    if (GpioLoopBackTest() == BASE_STATUS_ERROR) {
        return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}
/**
  * @brief Handle GPIO loopBack test.
  * @retval Value of @ref BASE_StatusType.
  */
static BASE_StatusType GpioLoopBackTest(void)
{
    GPIO_Value ret;
    /* Configure refHandle output and targetHandle input, Test Circle: refHandle -> targetHandle */
    HAL_GPIO_SetDirection(&TARGET_HANDLE, TARGET_PIN, GPIO_INPUT_MODE);
    HAL_GPIO_SetDirection(&REF_HANDLE, REF_PIN, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(&REF_HANDLE, REF_PIN, GPIO_HIGH_LEVEL); /* set refHandle high level */
    BASE_FUNC_DELAY_MS(1); /* 1: Wait 1ms for the level signal to stabilize. */
    ret = HAL_GPIO_GetPinValue(&TARGET_HANDLE, TARGET_PIN); /* get targetHandle's level */
    if (ret != GPIO_HIGH_LEVEL) {
        DBG_PRINTF("ref GPIO output to target GPIO high level error!\r\n");
        return BASE_STATUS_ERROR;
    } else {
        DBG_PRINTF("ref GPIO output to target GPIO high level success!\r\n");
    }
    HAL_GPIO_SetValue(&REF_HANDLE, REF_PIN, GPIO_LOW_LEVEL); /* set refHandle low level */
    BASE_FUNC_DELAY_MS(1); /* 1: Wait 1ms for the level signal to stabilize. */
    ret = HAL_GPIO_GetPinValue(&TARGET_HANDLE, TARGET_PIN); /* get targetHandle's level */
    if (ret != GPIO_LOW_LEVEL) {
        DBG_PRINTF("ref GPIO output to target GPIO low level error!\r\n");
        return BASE_STATUS_ERROR;
    } else {
        DBG_PRINTF("ref GPIO output to target GPIO low level success!\r\n");
    }
    /* Configure targetHandle output and refHandle input, Test Circle: targetHandle -> refHandle */
    HAL_GPIO_SetDirection(&TARGET_HANDLE, TARGET_PIN, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetDirection(&REF_HANDLE, REF_PIN, GPIO_INPUT_MODE);
    HAL_GPIO_SetValue(&TARGET_HANDLE, TARGET_PIN, GPIO_HIGH_LEVEL); /* set targetHandle high level */
    BASE_FUNC_DELAY_MS(1); /* 1: Wait 1ms for the level signal to stabilize. */
    ret = HAL_GPIO_GetPinValue(&REF_HANDLE, REF_PIN); /* get refHandle's level */
    if (ret != GPIO_HIGH_LEVEL) {
        DBG_PRINTF("target GPIO output to ref GPIO high level error! \r\n");
        return BASE_STATUS_ERROR;
    } else {
        DBG_PRINTF("target GPIO output to ref GPIO high level success!\r\n");
    }
    HAL_GPIO_SetValue(&TARGET_HANDLE, TARGET_PIN, GPIO_LOW_LEVEL); /* set targetHandle high level */
    BASE_FUNC_DELAY_MS(1); /* 1: Wait 1ms for the level signal to stabilize. */
    ret = HAL_GPIO_GetPinValue(&REF_HANDLE, REF_PIN); /* get refHandle's level */
    if (ret != GPIO_LOW_LEVEL) {
        DBG_PRINTF("target GPIO output to ref GPIO low level error! \r\n");
        return BASE_STATUS_ERROR;
    } else {
        DBG_PRINTF("target GPIO output to ref GPIO low level success!\r\n");
    }
    return BASE_STATUS_OK;
}