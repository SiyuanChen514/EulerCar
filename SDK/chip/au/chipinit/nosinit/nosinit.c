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
  * @file      nosinit.c
  * @author    MCU Driver Team
  * @brief     nos init module.
  * @details   nos initialization function during startup
  */

#ifdef NOS_TASK_SUPPORT
#include "nos_task.h"
#include "nosinit.h"
#include "systick.h"
#include "crg.h"

#ifndef CFG_NOS_MAINTASK_STACKSIZE
#define CFG_NOS_MAINTASK_STACKSIZE 0x500
#endif
#define CYCCLE_PERUS 200
// User define the stack size of main task by CFG_NOS_MAINTASK_STACKSIZE
unsigned char __attribute__((aligned(16))) g_taskMainStackSpace[CFG_NOS_MAINTASK_STACKSIZE];

static unsigned int g_nosSysFreq;
// Get tick in real time
static unsigned long long NOS_GetTick()
{
    unsigned int cycle, cycleh;
    asm volatile("csrr %0, cycle" : "=r"(cycle));
    asm volatile("csrr %0, cycleh" : "=r"(cycleh));
    unsigned long long longCycle = (unsigned long long)cycle + (unsigned long long)cycleh * 0xFFFFFFFF;
    // tick = cycle/freq
    return longCycle / g_nosSysFreq;
}

/**
  * @brief   Init the Nos Task Schedule
  * @param   None
  * @retval  None
  */
void NOS_Init(void)
{
    unsigned int taskId;
    NOS_SysConfig config = {};
    g_nosSysFreq = SYSTICK_GetCRGHZ() / CRG_FREQ_1MHz * CFG_SYSTICK_TICKINTERVAL_US;
    config.usecPerTick = SYSTICK_GetTickInterval(); /* Set the tick size of Nos task schedule */
    config.getTickFunc = NOS_GetTick;
    config.cyclePerUs = CYCCLE_PERUS;
    NOS_TaskInit(&config);
    NOS_TaskInitParam param = {};
    param.name = "mainTask";
    param.taskEntry = (NOS_TaskEntryFunc)main; /* Set the entry function by user define */
    param.param = 0;
    param.priority = NOS_TASK_PRIORITY_LOWEST;
    param.stackAddr = g_taskMainStackSpace;
    param.stackSize = sizeof(g_taskMainStackSpace);
    param.privateData = 0;
    (void)NOS_TaskCreate(&param, &taskId);
    (void)NOS_StartScheduler(); /* Nos task schedule start */
}
#endif