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
  * @file      clock.c
  * @author    MCU Driver Team
  * @brief     Provides functions related to the dominant frequency operation and delay.
  */

/* Includes ------------------------------------------------------------------ */
#include "ip_crg_common.h"
#include "crg.h"
#include "clock.h"

/**
  * @brief Get the current CPU frequency.
  * @param None.
  * @retval System clock frequency in Hz.
  */
unsigned int BASE_FUNC_GetCpuFreqHz(void)
{
    return HAL_CRG_GetCoreClkFreq();
}

/**
  * @brief Delay number of us.
  * @param us The number of us to delay.
  * @retval None.
  */
void BASE_FUNC_DelayUs(unsigned int us)
{
    unsigned int preTick = DCL_SYSTICK_GetTick();
    unsigned int tickInUs = (SYSTICK_GetCRGHZ() / CRG_FREQ_1MHz) * us;
    unsigned int curTick;
    unsigned int delta;

	/* Wait until the delta is greater than tickInUs */
    do {
        curTick = DCL_SYSTICK_GetTick();
        delta = (curTick >= preTick) ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick + 1;
    } while (delta < tickInUs);
}

/**
  * @brief Delay number of ms.
  * @param ms The number of ms to delay.
  * @retval None.
  */
void BASE_FUNC_DelayMs(unsigned int ms)
{
    for (unsigned int i = 0; i < ms; ++i) {
        BASE_FUNC_DelayUs(BASE_DEFINE_DELAY_US_IN_MS);
    }
}

/**
  * @brief Delay number of seconds.
  * @param seconds The number of seconds to delay.
  * @retval None.
  */
void BASE_FUNC_DelaySeconds(unsigned int seconds)
{
    for (unsigned int i = 0; i < seconds; ++i) {
        BASE_FUNC_DelayMs(BASE_DEFINE_DELAY_MS_IN_SEC);
    }
}

/**
  * @brief Delay for a certain period of time based on parameters delay and units.
  * @param delay The number of 'units' to delay.
  * @param units Specifies the delay unit.
  * @retval None.
  */
void BASE_FUNC_Delay(unsigned int delay, BASE_DelayUnit units)
{
    switch (units) {
        case BASE_DEFINE_DELAY_SECS:
            BASE_FUNC_DelaySeconds(delay);
            break;
        case BASE_DEFINE_DELAY_MILLISECS:
            BASE_FUNC_DelayMs(delay);
            break;
        case BASE_DEFINE_DELAY_MICROSECS:
            BASE_FUNC_DelayUs(delay);
            break;
        default:
            break;
    }
    return;
}