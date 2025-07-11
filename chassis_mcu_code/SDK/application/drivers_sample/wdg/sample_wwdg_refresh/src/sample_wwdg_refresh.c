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
  * @file      sample_wwdg_refresh.c
  * @author    MCU Driver Team
  * @brief     WWDG module realize refresh function sample
  * @details   The watchdog timeout reset function is used to set the time when the watchdog feeds the watchdog.
  *            When the window mode is enabled and the corresponding window value is set,
  *            an interrupt is generated when the count value is equal to the window value.
  *            When the watchdog is fed beyond the window value, a reset signal is generated.
  *            When the count value is 0 and the interrupt is not cleared, a reset signal is also generated.
  *            When the window mode is disabled, an interrupt is generated when the count value is reduced to half.
  *            The reset signal is generated only when the count value is 0 and the interrupt is not cleared.
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "wwdg.h"
#include "main.h"
#include "baseinc.h"
#include "sample_wwdg_refresh.h"

#define DOG_FEED_INTRVAL_TIME  2000
#define CYCLE_INTERVAL_TIME  1000

/* prototype functions -------------------------------------------------------*/
void WWDGCallbackFunction(void *param);

/**
  * @brief WWDG reset sample function
  * @param None
  * @return BASE_StatusType
  */
BASE_StatusType WWDG_RefreshSample(void)
{
    SystemInit();
    HAL_WWDG_Start(&g_wwdg); /* Turn on the watchdog timing */
    DBG_PRINTF("\r\n START : test wwdg sample \r\n");
    while (1) {
        DBG_PRINTF("test wwdg sample \r\n");
        BASE_FUNC_DELAY_MS(CYCLE_INTERVAL_TIME);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Handling WWDG interrupt callback
  * @param handle    WWDG handle for registers and initialnized values
  * @return None
  */
void WWDGCallbackFunction(void *param)
{
    WWDG_Handle *handle = (WWDG_Handle *)param;
    WWDG_ASSERT_PARAM(handle != NULL);
    /* clear interrupt signal will auto load value, reset function invalid, \
    not clear irq, second times will reset */
    /* User can Add HAL_WWDG_Refresh() API here, wwdg not reset because refresh period, \
    if not refresh, next time reset */
    HAL_WWDG_Refresh(handle);

    DBG_PRINTF("WWDG Interrupt handle \r\n");
}