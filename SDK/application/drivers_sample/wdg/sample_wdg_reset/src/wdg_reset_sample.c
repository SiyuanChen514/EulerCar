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
  * @file      wdg_reset_sample.c
  * @author    MCU Driver Team
  * @brief     WDG module realize reset function sample
  * @details   The watchdog timeout reset function is used to set the watchdog feeding time. When the timing \
  *            value decreases to 0, an interrupt is triggered. The watchdog is not fed and the count value is \
  *            overloaded. The reset operation is performed after the second count is cleared. If you do not \
  *            want to reset, you can feed the watchdog when the first count is cleared to trigger an interrupt. \
  *            The actual timeout reset time of the IP version is twice the count value. The reset is performed \
  *            only after the second count is cleared. clear interrupt signal will auto load value, reset function \
  *            invalid, not clear irq, second times will reset User can Add HAL_WDG_Refresh() API here, then, \
  *            wdg can use as a timer because clear with reload. so reset function invalid.
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "wdg.h"
#include "wdg_reset_sample.h"
#include "main.h"
#include "baseinc.h"

#define DOG_FEED_INTRVAL_TIME  2000
#define CYCLE_INTERVAL_TIME  1000

/* prototype functions -------------------------------------------------------*/
void WDG_CallbackFunc(void *param);

/**
  * @brief WDG reset sample function
  * @param None
  * @return BASE_StatusType
  */
BASE_StatusType WDG_ResetSample(void)
{
    SystemInit();
    DBG_UartPrintInit(115200); /* 115200 : baud rate */
    DBG_PRINTF("\r\n START : test wdg sample \r\n");
    while (1) {
        DBG_PRINTF("test wdg sample \r\n");
        BASE_FUNC_DELAY_MS(CYCLE_INTERVAL_TIME);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Handling WDG interrupt callback
  * @param handle    WDG handle for registers and initialnized values
  * @return None
  */
void WDG_CallbackFunc(void *param)
{
    WDG_Handle *handle = (WDG_Handle *)param;
    WDG_ASSERT_PARAM(handle != NULL);
    /* clear interrupt signal will auto load value, reset function invalid, \
    not clear irq, second times will reset */
    /* User can Add HAL_WDG_Refresh() API here, wdg not reset because refresh period, \
    if not refresh, next time reset */
    HAL_WDG_Refresh(handle);

    DBG_PRINTF("WDG Interrupt handle \r\n");
}