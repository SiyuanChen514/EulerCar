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
  * @file      systickinit.c
  * @author    MCU Driver Team
  * @brief     systick init module.
  * @details   systick initialization function during startup
  */
#include "baseaddr.h"
#include "timer.h"
#include "systick.h"
#include "systickinit.h"
#include "crg.h"

TIMER_Handle g_systickHandle;

#ifdef NOS_TASK_SUPPORT
#include "interrupt.h"
#include "systick.h"
#define NOS_TickPostDispatch OsHwiDispatchTick

void SYSTICK_Default_Callback(void)
{
    /* The default systick callback when using th nos task */
    HAL_TIMER_IrqClear(&g_systickHandle);
    NOS_TickPostDispatch();
}

void SYSTICK_IRQ_Enable(void)
{
    /* When Support NOS Task, Need to open the TickIRQ, us per tick will to update the load */
    g_systickHandle.irqNum = IRQ_TIMER3;
    g_systickHandle.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_RegisterCallback(&g_systickHandle, SYSTICK_Default_Callback);
    HAL_TIMER_Config(&g_systickHandle, TIMER_CFG_INTERRUPT);  // enable the systickIRQ
    IRQ_SetPriority(g_systickHandle.irqNum, 1);
    IRQ_EnableN(IRQ_TIMER3);
}

unsigned int SYSTICK_GetTickInterval(void)
{
    /* Get the tick interval(the number of usecond per tick) */
    return CFG_SYSTICK_TICKINTERVAL_US;
}

static inline unsigned int DCL_GetCpuCycle()
{
    /* Get the Cpu Cycle Register(CSR) */
    unsigned int cycle;
    asm volatile("csrr %0, cycle" : "=r"(cycle));

    return cycle;
}
#endif

unsigned int SYSTICK_GetCRGHZ(void)
{
    /* Get the Systick IP */
#ifdef NOS_TASK_SUPPORT
    return HAL_CRG_GetCoreClkFreq();
#else
    return HAL_CRG_GetIpFreq(SYSTICK_BASE);
#endif
}

unsigned int DCL_SYSTICK_GetTick(void)
{
#ifdef NOS_TASK_SUPPORT
    /* Return the load value(period) and the counter value, make the returned counter in count up mode */
    return DCL_GetCpuCycle();
#else
    /* Invert the counter value, make the returned counter in count up mode */
    return ~SYSTICK->timer_value;
#endif
}

unsigned int SYSTICK_GetTimeStampUs(void)
{
    /* Get the systick timestamp(convert from the systick value) */
    return DCL_SYSTICK_GetTick() / (SYSTICK_GetCRGHZ() / CRG_FREQ_1MHz);
}

/**
  * @brief   Init the systick
  * @param   None
  * @retval  None
  */
void SYSTICK_Init()
{
    /* Choose the config to support GetTick and Delay */
    g_systickHandle.baseAddress = SYSTICK;
#ifdef NOS_TASK_SUPPORT
    /* Change the period load to the user defined usecond */
    g_systickHandle.load        = (HAL_CRG_GetIpFreq(SYSTICK_BASE) / CRG_FREQ_1MHz) * CFG_SYSTICK_TICKINTERVAL_US;
    g_systickHandle.bgLoad      = (HAL_CRG_GetIpFreq(SYSTICK_BASE) / CRG_FREQ_1MHz) * CFG_SYSTICK_TICKINTERVAL_US;
#else
    g_systickHandle.load        = SYSTICK_MAX_VALUE;
    g_systickHandle.bgLoad      = SYSTICK_MAX_VALUE;
#endif
    g_systickHandle.mode        = TIMER_MODE_RUN_PERIODIC;
    g_systickHandle.prescaler   = TIMERPRESCALER_NO_DIV;
    g_systickHandle.size        = TIMER_SIZE_32BIT;
    /* Don't Support IRQ because only needs to read the value of systick */
    g_systickHandle.interruptEn = BASE_CFG_DISABLE;
    HAL_TIMER_Init(&g_systickHandle);
#ifdef NOS_TASK_SUPPORT
    /* Support IRQ to upload the totalCycle and detect the timeout lists */
    SYSTICK_IRQ_Enable();
#endif
    HAL_TIMER_Start(&g_systickHandle);
}