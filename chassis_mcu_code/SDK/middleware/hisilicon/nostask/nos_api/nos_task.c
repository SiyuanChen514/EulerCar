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
  * @file      nos_task.c
  */

#include "nos_base_task.h"
#include "nos_config_internal.h"
#include "nos_config.h"
#include "nos_task.h"

static NOS_SysConfig g_nosSysConfig = {.usecPerTick = 100, .cyclePerUs = 200};

/*
 * 描述: 任务初始化。
 */
int NOS_TaskInit(NOS_SysConfig *config)
{
    if (config == NULL || config->usecPerTick == 0 || config->getTickFunc == NULL ||
        config->cyclePerUs == 0) {
        return -1;
    }

    g_nosSysConfig.cyclePerUs = config->cyclePerUs;
    g_nosSysConfig.usecPerTick = config->usecPerTick;
    g_nosSysConfig.getTickFunc = config->getTickFunc;
    /* set getTickFunc */
    NOS_SetNosParam(config->getTickFunc);
    NOS_MoudleInit();
    return 0;
}

unsigned long long NOS_GetCycle(void)
{
    unsigned int cycle, cycleh;
    /* 时钟低32位寄存器 */
    asm volatile("csrr %0, cycle" : "=r"(cycle));
    /* 时钟高32位寄存器 */
    asm volatile("csrr %0, cycleh" : "=r"(cycleh));
    unsigned long long longCycle = (unsigned long long)cycle + (unsigned long long)cycleh * 0xFFFFFFFF;
    return longCycle;
}

int NOS_StartScheduler(void)
{
    /* OS start will switch to task_entry in system */
    return OsStart();
}

/*
 * 描述: 任务创建但不激活。
 */
int NOS_TaskCreateOnly(NOS_TaskInitParam *initParam, unsigned int *taskId)
{
    if (taskId == NULL) {
        return -1;
    }
    /* 参数校验 */
    if ((initParam == NULL) || (initParam->priority > NOS_TASK_PRIORITY_LOWEST) ||
       (initParam->stackAddr == 0) || (initParam->stackSize == 0)) {
        return -1;
    }
    *taskId = OS_TSK_MAX_SUPPORT_NUM + 1;
    struct TskInitParam param = {0};
    param.taskEntry = (TskEntryFunc)initParam->taskEntry;
    /* [0-NOS_TASK_PRIORITY_LOWEST] */
    param.taskPrio = initParam->priority;
    param.name = initParam->name;
    param.stackSize = initParam->stackSize;
    param.stackAddr = initParam->stackAddr;
    param.privateData = initParam->privateData;
    param.args[0] = (uintptr_t)initParam->param;
    /* 调用内部接口 */
    return (int)NOS_TaskCreateOnlyInner(taskId, &param);
}

/*
 * 描述: 任务创建。
 */
int NOS_TaskCreate(NOS_TaskInitParam *initParam, unsigned int *taskId)
{
    if ((taskId == NULL) || (initParam == NULL)) {
        return -1;
    }
    /* 参数校验 */
    if ((initParam->priority > NOS_TASK_PRIORITY_LOWEST) ||
       (initParam->stackAddr == 0) || (initParam->stackSize == 0)) {
        return -1;
    }
    *taskId = OS_TSK_MAX_SUPPORT_NUM + 1;
    struct TskInitParam param = {0};
    param.taskEntry = (TskEntryFunc)initParam->taskEntry;
    /* [0-NOS_TASK_PRIORITY_LOWEST] */
    param.taskPrio = initParam->priority;
    param.name = initParam->name;
    param.stackAddr = initParam->stackAddr;
    param.stackSize = initParam->stackSize;
    param.args[0] = (uintptr_t)initParam->param;
    param.privateData = initParam->privateData;
    /* 调用内部接口 */
    return (int)NOS_TaskCreateInner(taskId, &param);
}

/*
 * 描述: 任务删除。
 */
int NOS_TaskDelete(unsigned int taskId)
{
    return (int)NOS_TaskDeleteInner(taskId);
}

/*
 * 描述: 任务挂起。
 */
int NOS_TaskSuspend(unsigned int taskId)
{
    return (int)NOS_TaskSuspendInner(taskId);
}

/*
 * 描述: 任务恢复。
 */
int NOS_TaskResume(unsigned int taskId)
{
    return (int)NOS_TaskResumeInner(taskId);
}

/*
 * 描述: 任务延时。
 */
int NOS_TaskDelay(unsigned int timeout)
{
    if (g_nosSysConfig.cyclePerUs == 0) {
        return -1;
    }
    return (int)NOS_TaskDelayInner(timeout * g_nosSysConfig.cyclePerUs);
}

/*
 * 描述: 任务优先级设置。
 */
int NOS_TaskPrioritySet(unsigned int taskId, unsigned short priority)
{
    return (int)NOS_TaskPrioritySetInner(taskId, priority);
}

/*
 * 描述: 任务优先级获取。
 */
int NOS_TaskPriorityGet(unsigned int taskId, unsigned short *priority)
{
    return (int)NOS_TaskPriorityGetInner(taskId, priority);
}

/* **********************timer task********************* */

/*
 * 描述: timer task 回调函数。
 */
unsigned long NOS_TaskUpdateExpirCnt(unsigned int taskPid);
static void PeriodTaskEntry(int param0, int param1, int param2, int param3)
{
    (void)(param2);
    (void)(param3);
    void *timerParam = (void *)(uintptr_t)param1;
    unsigned int taskId;
    /* 获取当前任务ID */
    NOS_TaskSelf(&taskId);
    NOS_TimerCallBack callback = (NOS_TimerCallBack)(uintptr_t)param0;
    if (callback == NULL) {
        return;
    }
    while (1) {
        /* 执行回调函数 */
        callback(timerParam);
        if (NOS_TaskUpdateExpirCnt(taskId) == 0) {
            /* 任务挂起 */
            NOS_TaskSuspendInner(taskId);
        }
    }
}

/*
 * 描述: 创建定时任务。
 */
int NOS_CreateTimerTask(unsigned int *timerTaskId, NOS_TimerTaskInitParam *timerParam)
{
    /* 参数校验 */
    if (timerTaskId == NULL || timerParam == NULL || timerParam->callback == NULL ||
        g_nosSysConfig.usecPerTick == 0) {
        return -1;
    }
    if ((timerParam->timeout < g_nosSysConfig.usecPerTick) || (timerParam->stackAddr == 0) ||
        (timerParam->stackSize == 0) || (timerParam->priority > NOS_TASK_PRIORITY_LOWEST)) {
            return -1;
    }
    *timerTaskId = OS_TSK_MAX_SUPPORT_NUM + 1;
    struct TskInitParam param = {0};
    param.taskEntry = (TskEntryFunc)PeriodTaskEntry;
    /* [0-NOS_TASK_PRIORITY_LOWEST] */
    param.taskPrio = timerParam->priority;
    param.name = timerParam->name;
    param.stackSize = timerParam->stackSize;
    param.stackAddr = timerParam->stackAddr;
    param.privateData = (timerParam->timeout) * g_nosSysConfig.cyclePerUs;
    /* real callback */
    param.args[0] = (uintptr_t)timerParam->callback;
    param.args[1] = (uintptr_t)timerParam->callbackParam;
    return (int)NOS_TaskCreateOnlyInner(timerTaskId, &param);
}

/* cannot call this func before systick work on */
int NOS_StartTimerTask(unsigned int taskId)
{
    /* notice: taskId must created by NOS_CreateTimerTask. */
    unsigned int timeout = (unsigned int)NOS_TaskPrivateDataGetById(taskId);
    return (int)NOS_TaskStartPeriod(taskId, timeout);
}

/* cannot call this func before systick work on */
int NOS_StopTimerTask(unsigned int taskId)
{
    /* notice: taskId must created by NOS_CreateTimerTask.  */
    return (int)NOS_TaskStopPeriod(taskId);
}

