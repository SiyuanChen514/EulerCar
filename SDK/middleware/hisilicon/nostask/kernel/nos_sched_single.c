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
  * @file      nos_sched_single.c
  */
#include "nos_task_external.h"

/*
 * 描述: 调度的主入口
 * 备注: NA
 */
 void OsMainSchedule(void);
 void OsMainSchedule(void)
{
    if ((UNI_FLAG & OS_FLG_TSK_REQ) != 0) {
        /* 有任务切换钩子&最高优先级任务等待调度 */
        if (RUNNING_TASK != g_highestTask) {
            OsTskSwitchHookCaller(RUNNING_TASK->taskPid, g_highestTask->taskPid);
        }

        /* 清除OS_FLG_TSK_REQ标记位 */
        UNI_FLAG &= ~OS_FLG_TSK_REQ;

        RUNNING_TASK->taskStatus &= ~OS_TSK_RUNNING;
        g_highestTask->taskStatus |= OS_TSK_RUNNING;

        RUNNING_TASK = g_highestTask;
    }
    // 如果中断没有驱动一个任务ready，直接回到被打断的任务
    OsTskContextLoad((uintptr_t)RUNNING_TASK);
}

/*
 * 描述: 系统启动时的首次任务调度
 * 备注: NA
 */
void OsFirstTimeSwitch(void)
{
    OsTskHighestSet();
    RUNNING_TASK = g_highestTask;
    TSK_StatusSet(RUNNING_TASK, OS_TSK_RUNNING);
    OsTskContextLoad((uintptr_t)RUNNING_TASK);
    // never get here
    return;
}
