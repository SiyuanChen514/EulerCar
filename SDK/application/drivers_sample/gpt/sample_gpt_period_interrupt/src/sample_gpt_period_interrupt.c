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
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE..
  * @file    sample_gpt_period_interrupt.c
  * @author  MCU Driver Team
  * @brief   gpt sample module.
  * @details This file provides users with sample code to help use GPT function:
  *          1) Generate a continuous square wave with period and duty set in the function SystemInit.
  *          2) GPT period interrupt callback.
  */
#include "sample_gpt_period_interrupt.h"

/**
  * @brief GPT period interrupt callback
  * @param None.
  * @retval None.
  */
void GPT2PWMPeriodOutputFinishCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN IRQ_GPT2_PRD_INT */
    DBG_PRINTF("GPT period interrupt\n");
    /* USER CODE END IRQ_GPT2_PRD_INT */
}

/**
  * @brief GPT run and modify period and duty during running.
  * @param None.
  * @retval None.
  */
void GPT_PeriodInterrupt(void)
{
    SystemInit();
    DBG_PRINTF("GPT Continued Run begin\r\n");
    HAL_GPT_Start(&g_gptHandle);
    BASE_FUNC_DelaySeconds(10); /* Delay 10 seconds */
}
