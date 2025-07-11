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
  * @file      gpio_key_sample.c
  * @author    MCU Driver Team
  * @brief     GPIO module realize a key function sample
  * @details   GPIO Capturing Key Triggered Interrupt Service Function. If the hardware environment does \
  *            not support this function, you need to set up an environment for verification.
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "gpio.h"
#include "main.h"
#include "gpio_key_sample.h"

#define PREVENT_SWIPE_SCREEN_TIME    50
#define CYCLE_INTERVAL_TIME   500

/* prototype functions -------------------------------------------------------*/
void GPIO_CallBackFunc(void *param);

/* ---------------------------------- Sample Parameters ---------------------- */
/**
  * @brief GPIO key test sample.
  * @param None
  * @retval Value of @ref BASE_StatusType.
  */
BASE_StatusType GPIO_KeySample(void)
{
    SystemInit();
    /* Waiting for the key to come. */
    while (1) {
        DBG_PRINTF("Wait key \r\n");
        BASE_FUNC_DELAY_MS(CYCLE_INTERVAL_TIME);
    }

    return BASE_STATUS_OK;
}

/**
  * @brief GPIO register interrupt callback function.
  * @param param Value of @ref GPIO_Handle.
  * @retval None
  */
void GPIO_CallBackFunc(void *param)
{
    BASE_FUNC_UNUSED(param);
    DBG_PRINTF("in GPIO Key Handler \r\n");
    BASE_FUNC_DELAY_MS(PREVENT_SWIPE_SCREEN_TIME);
}