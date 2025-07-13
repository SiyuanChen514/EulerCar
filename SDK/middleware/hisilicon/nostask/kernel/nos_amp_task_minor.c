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
  * @file      nos_amp_task_minor.c
  */
#include "nos_task_internal.h"

/*
 * 描述: Suspend task
 */
unsigned int NOS_TaskSuspendInner(unsigned int taskPid)
{
    uintptr_t intSave;
    struct TagTskCB *taskCB = NULL;

    /* check taskId */
    if (taskPid == IDLE_TASK_ID || CheckTaskPidOverflow(taskPid)) {
        return OS_ERRNO_TSK_ID_INVALID;
    }

    /* get taskTCB */
    taskCB = GetTcbHandle(taskPid);

    intSave = NOS_TaskIntLock();

    if (taskCB->taskStatus == OS_TSK_UNUSED) {
        NOS_TaskIntRestore(intSave);
        return OS_ERRNO_TSK_NOT_CREATED;
    }

    /* If task is suspended then return */
    if ((OS_TSK_SUSPEND & taskCB->taskStatus) != 0) {
        NOS_TaskIntRestore(intSave);
        return OS_ERRNO_TSK_ALREADY_SUSPENDED;
    }

    if (((OS_TSK_RUNNING & taskCB->taskStatus) != 0) && (g_uniTaskLock != 0)) {
        NOS_TaskIntRestore(intSave);
        return OS_ERRNO_TSK_SUSPEND_LOCKED;
    }

    /* If task is ready then remove from ready list */
    if ((OS_TSK_READY & taskCB->taskStatus) != 0) {
        OsTskReadyDel(taskCB);
    }

    taskCB->taskStatus |= OS_TSK_SUSPEND;

    if ((OS_TSK_RUNNING & taskCB->taskStatus) != 0) {
        OsTskScheduleFastPS(intSave);
    }

    NOS_TaskIntRestore(intSave);
    return NOS_OK;
}

/*
 * 描述: Resume suspend task
 */
unsigned int NOS_TaskResumeInner(unsigned int taskId)
{
    uintptr_t intSave;
    struct TagTskCB *taskCB = NULL;

    if (CheckTaskPidOverflow(taskId)) {
        return OS_ERRNO_TSK_ID_INVALID;
    }

    /* get taskTCB */
    taskCB = GetTcbHandle(taskId);

    intSave = NOS_TaskIntLock();

    if (TSK_IsUnused(taskCB)) {
        NOS_TaskIntRestore(intSave);
        return OS_ERRNO_TSK_NOT_CREATED;
    }

    /* If task is not suspended then return */
    if ((OS_TSK_SUSPEND & taskCB->taskStatus) == 0) {
        NOS_TaskIntRestore(intSave);
        return OS_ERRNO_TSK_NOT_SUSPENDED;
    }

    TSK_StatusClear(taskCB, OS_TSK_SUSPEND);

    /* If task is not blocked then move it to ready list */
    OsMoveTaskToReady(taskCB);
    NOS_TaskIntRestore(intSave);

    return NOS_OK;
}
