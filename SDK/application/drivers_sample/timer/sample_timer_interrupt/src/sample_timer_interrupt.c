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
  * @file    sample_timer_interrupt.c
  * @author  MCU Driver Team
  * @brief   timer sample module.
  * @details This file provides users with sample code to help use TIMER function:
  *          1) TIMER runs period and triggers an interrupt every second.
  *          2) Change the period during TIMER running
  *          Phenomenon:
  *          The interrupt handler is executed every 1 second, after 10 second, the interrupt handler execute
  *          every 0.5 second and print "In interrupt" on the serial port.
  */
#include "sample_timer_interrupt.h"

void TIMER0_InterruptProcess(void *handle);

/**
  * @brief Timer run and triggers an interrupt.
  * @param None.
  * @retval None.
  */
void TIMER_SampleMain(void)
{
    TIMER_Handle timerHandle;

    SystemInit();
    DBG_PRINTF("TIMER_SampleMain begin\r\n");
    HAL_TIMER_Start(&g_timerHandle);

    BASE_FUNC_DelaySeconds(10); /* Delay 10 seconds */
    DBG_PRINTF("Change period of timer\r\n");
    timerHandle.baseAddress = g_timerHandle.baseAddress;
    HAL_TIMER_GetConfig(&timerHandle);
    timerHandle.bgLoad = HAL_CRG_GetIpFreq((void *)TIMER0) >> 1;
    HAL_TIMER_Config(&timerHandle, TIMER_CFG_BGLOAD);
}

/**
  * @brief Timer Interrupt callback function
  * @param handle Handle of Timer
  * @retval None.
  */
void TIMER0_InterruptProcess(void *handle)
{
    /* USER CODE BEGIN TIMER0_InterruptProcess */
    TIMER_Handle *timerHandle = (TIMER_Handle *)handle;
    BASE_FUNC_UNUSED(timerHandle);
    DBG_PRINTF("In interrupt\r\n");
    /* USER CODE END TIMER0_InterruptProcess */
}
