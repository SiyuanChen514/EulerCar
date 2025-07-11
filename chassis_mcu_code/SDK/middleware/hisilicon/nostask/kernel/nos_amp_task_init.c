/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      nos_amp_task_init.c
  */
#include "nos_task_internal.h"

/*
 * 描述: AMP Task init function.
 */
unsigned int OsTskAMPInit(void);
unsigned int OsTskAMPInit(void)
{
    unsigned int idx;

    // 1为Idle任务
    g_threadNum += (g_tskMaxNum + 1);

    if (g_tskCBArray == NULL) {
        return OS_ERRNO_TSK_NO_MEMORY;
    }

    OS_LIST_INIT(&g_tskCBFreeList);
    for (idx = 0; idx < OS_MAX_TCB_NUM - 1; idx++) {
        g_tskCBArray[idx].taskStatus = OS_TSK_UNUSED;
        g_tskCBArray[idx].taskPid = idx;
        ListTailAdd(&g_tskCBArray[idx].pendList, &g_tskCBFreeList);
    }

    /* 在初始化时给RUNNING_TASK的PID赋一个合法的无效值，放置在Trace使用时出现异常 */
    RUNNING_TASK = OS_PST_ZOMBIE_TASK;

    /* 在初始化时给RUNNING_TASK的PID赋一个合法的无效值，放置在Trace使用时出现异常 */
    RUNNING_TASK->taskPid = idx;

    /* Init empty ready list for each priority. */
    for (idx = 0; idx < OS_TSK_NUM_OF_PRIORITIES; idx++) {
        OS_LIST_INIT(&g_runQueue.readyList[idx]);
    }

    OS_LIST_INIT(&g_tskSortedDelay.tskList);
    OS_LIST_INIT(&g_tskRecyleList);

    /* 增加OS_TSK_INUSE状态，使得在Trace记录的第一条信息状态为OS_TSK_INUSE(创建状态) */
    RUNNING_TASK->taskStatus = (OS_TSK_INUSE | OS_TSK_RUNNING);
    RUNNING_TASK->priority = OS_TSK_PRIORITY_LOWEST + 1;

    return NOS_OK;
}

unsigned char __attribute__((aligned(16))) g_idleTaskStackSpace[OS_TSK_IDLE_STACK_SIZE];

/*
 * 描述: ilde Task create.
 */
unsigned int OsIdleTskAMPCreate(void)
{
    unsigned int ret;
    unsigned int taskHdl;
    struct TskInitParam taskInitParam;
    const char *tskName = "IdleCore000";
    /* Create background task. */
    taskInitParam.taskEntry = (TskEntryFunc)g_idleEntry;
    taskInitParam.stackSize = (unsigned int)sizeof(g_idleTaskStackSpace);
    taskInitParam.name = tskName;
    /* Lowest Priority */
    taskInitParam.taskPrio = OS_TSK_PRIORITY_LOWEST;
    taskInitParam.stackAddr = (uintptr_t)g_idleTaskStackSpace;
    taskInitParam.privateData = 0;

    /* 任务调度的必要条件就是有背景任务，此时背景任务还没有创建，因此不会发生任务切换 */
    ret = NOS_TaskCreateInner(&taskHdl, &taskInitParam);
    if (ret != NOS_OK) {
        return ret;
    }

    IDLE_TASK_ID = taskHdl;

    return ret;
}
