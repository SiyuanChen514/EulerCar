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
  * @file      nos_task.h
  */

#ifndef NOS_TASK_H
#define NOS_TASK_H

#define NOS_TASK_PRIORITY_LOWEST 4

typedef void (*NOS_TaskEntryFunc)(void* param);
typedef void (*NOS_TimerCallBack)(void* param);
typedef struct {
    const char *name;
    NOS_TaskEntryFunc taskEntry;
    void* param;
    unsigned int priority; /* scope:[0-NOS_TASK_PRIORITY_LOWEST] */
    unsigned int stackAddr; /* notice: addr must 16Bytes align && not zero */
    unsigned int stackSize;
    unsigned int privateData;
} NOS_TaskInitParam;

typedef struct {
    const char *name;
    unsigned int timeout; // us
    NOS_TimerCallBack callback;
    void *callbackParam;
    unsigned int priority; /* scope:[0-NOS_TASK_PRIORITY_LOWEST] */
    unsigned int stackSize;
    unsigned int stackAddr;
}NOS_TimerTaskInitParam;

typedef struct {
    unsigned int cyclePerUs;
    unsigned int usecPerTick;
    unsigned long long (*getTickFunc)(void);
}NOS_SysConfig;

int NOS_TaskInit(NOS_SysConfig *config);

int NOS_StartScheduler(void);

int NOS_TaskCreateOnly(NOS_TaskInitParam *initParam, unsigned int *taskId);

int NOS_TaskCreate(NOS_TaskInitParam *initParam, unsigned int *taskId);

int NOS_TaskDelete(unsigned int taskId);

int NOS_TaskSuspend(unsigned int taskId);

int NOS_TaskResume(unsigned int taskId);

int NOS_TaskDelay(unsigned int timeout);

int NOS_TaskPrioritySet(unsigned int taskId, unsigned short priority);

int NOS_TaskPriorityGet(unsigned int taskId, unsigned short *priority);

/* **********************timer task********************* */

int NOS_CreateTimerTask(unsigned int *timerTaskId, NOS_TimerTaskInitParam *timerParam);

/* 接口约束 必须systick启动后. taskId 必须是 NOS_CreateTimerTask 创建的 */
int NOS_StartTimerTask(unsigned int taskId);

/* 接口约束 必须systick启动后. taskId 必须是 NOS_CreateTimerTask 创建的 */
int NOS_StopTimerTask(unsigned int taskId);

#endif // NOS_TASK_H
