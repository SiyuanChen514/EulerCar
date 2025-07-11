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
  * @file      sample_dimming.c
  * @author    MCU Application Team
  * @brief     Dimming sample code
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the common function sample.
  *             + Dimming sample functions
  */

/* Includes ------------------------------------------------------------------ */
#include "sample_dimming.h"

/**
  * @brief Using gpt dimming example.
  * @return int Execution result.
  */
int BOARD_DIMMING_Sample(void)
{
    SystemInit();
    BOARD_DIM_Init(&g_gptHandle, &g_timerHandle, 0, 3000); /* 3000ms. */
    HAL_GPT_Start(&g_gptHandle);
    while (1) {
        BOARD_DIM_SetDuty(0, 252); /* 252 : dimming level. */
        BASE_FUNC_DELAY_S(3); /* 3 : 3 second. */
        BOARD_DIM_SetDuty(0, 0);
        BASE_FUNC_DELAY_S(3); /* 3 : 3 second. */
    }

    return 0;
}