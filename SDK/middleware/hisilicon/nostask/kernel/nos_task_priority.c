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
  * @file      nos_task_priority.c
  */
#include "nos_task_internal.h"
#include "nos_task_external.h"

/*
 * 描述: 获取指定任务的优先级
 */
unsigned int NOS_TaskPriorityGetInner(unsigned int taskPid, unsigned short *taskPrio)
{
    uintptr_t intSave;
    struct TagTskCB *taskCB = NULL;

    if (CheckTaskPidOverflow(taskPid)) {
        return OS_ERRNO_TSK_ID_INVALID;
    }

    if (taskPrio == NULL) {
        return OS_ERRNO_TSK_PTR_NULL;
    }

    /* get task Tcb */
    taskCB = GetTcbHandle(taskPid);

    intSave = NOS_IntLock();

    /* 判断任务是否存在 */
    if (TSK_IsUnused(taskCB)) {
        NOS_IntRestore(intSave);

        return OS_ERRNO_TSK_NOT_CREATED;
    }

    *taskPrio = taskCB->priority;

    NOS_IntRestore(intSave);

    return NOS_OK;
}

/*
 * 描述: 设置优先级前做参数检查
 */
INLINE unsigned int OsTaskPrioritySetCheck(unsigned int taskPid, unsigned short taskPrio)
{
    /* 检查优先级是否合法 */
    if (taskPrio > OS_TSK_PRIORITY_LOWEST) {
        return OS_ERRNO_TSK_PRIOR_ERROR;
    }

    if (CheckTaskPidOverflow(taskPid)) {
        return OS_ERRNO_TSK_ID_INVALID;
    }

    if (taskPid == IDLE_TASK_ID) {
        return OS_ERRNO_TSK_OPERATE_IDLE;
    }
    return NOS_OK;
}

/*
 * 描述: 设置指定任务的优先级
 */
unsigned int NOS_TaskPrioritySetInner(unsigned int taskPid, unsigned short taskPrio)
{
    unsigned int ret;
    bool isReady = FALSE;
    uintptr_t intSave;
    struct TagTskCB *taskCB = NULL;

    ret = OsTaskPrioritySetCheck(taskPid, taskPrio);
    if (ret != NOS_OK) {
        return ret;
    }

    /* get task Tcb */
    taskCB = GetTcbHandle(taskPid);
    intSave = NOS_TaskIntLock();
    OsSpinLockTaskRq(taskCB);
    /* 判断任务是否存在 */
    if (TSK_IsUnused(taskCB)) {
        OsSpinUnlockTaskRq(taskCB);
        NOS_TaskIntRestore(intSave);
        return OS_ERRNO_TSK_NOT_CREATED;
    }

    /* 获取任务状态 */
    isReady = (OS_TSK_READY & taskCB->taskStatus);

    /* delete the task & insert with right priority into ready queue */
    if (isReady) {
        OsTskReadyDel(taskCB);
        taskCB->priority = taskPrio;
        OsTskReadyAdd(taskCB);
    } else {
        taskCB->priority = taskPrio;
    }

    taskCB->origPriority = taskPrio;
    OsSpinUnlockTaskRq(taskCB);

    /* reschedule if ready changed */
    if (isReady) {
        OsTskSchedule();
    }

    NOS_TaskIntRestore(intSave);
    return NOS_OK;
}
