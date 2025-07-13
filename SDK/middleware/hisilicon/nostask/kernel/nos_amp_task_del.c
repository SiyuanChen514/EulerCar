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
  * @file      nos_amp_task_del.c
  */
#include "nos_task_internal.h"
#include "nos_buildef.h"

#if defined(OS_OPTION_TASK_DELETE)
/* 删除任务，重置Tcb块状态 */
static void OsTaskDeleteResInit(unsigned int taskPid, struct TagTskCB *taskCB)
{
    (void)taskPid;
    /* 判断任务状态 */
    if (((OS_TSK_PEND | OS_TSK_QUEUE_PEND) & taskCB->taskStatus) != 0) {
        ListDelete(&taskCB->pendList);
    }

    if (((OS_TSK_DELAY | OS_TSK_TIMEOUT | OS_TSK_PERIOD) & taskCB->taskStatus) != 0) {
        ListDelete(&taskCB->timerList);
    }

    if ((OS_TSK_READY & taskCB->taskStatus) != 0) {
        /* 从就绪队列中删除 */
        OsTskReadyDel(taskCB);
    }

    taskCB->taskStatus &= (~(OS_TSK_SUSPEND));
    taskCB->taskStatus |= OS_TSK_UNUSED;

    return;
}

/*
 * 描述: 删除一个任务线程
 */
unsigned int OsTaskDelete(unsigned int taskPid)
{
    unsigned int ret;
    uintptr_t intSave;
    struct TagTskCB *taskCB = NULL;

    if (taskPid == IDLE_TASK_ID) {
        return OS_ERRNO_TSK_OPERATE_IDLE;
    }

    if (CheckTaskPidOverflow(taskPid)) {
        return OS_ERRNO_TSK_ID_INVALID;
    }

    intSave = NOS_TaskIntLock();

    /* get task Tcb */
    taskCB = GetTcbHandle(taskPid);
    /* 任务是否创建 */
    if (TSK_IsUnused(taskCB)) {
        NOS_TaskIntRestore(intSave);
        return OS_ERRNO_TSK_NOT_CREATED;
    }

    /* check task status */
    ret = OsTaskDelStatusCheck(taskCB);
    if (ret != NOS_OK) {
        NOS_TaskIntRestore(intSave);
        return ret;
    }

    /* 删除任务 */
    OsTaskDeleteResInit(taskPid, taskCB);

    /* 寻找最高优先级任务 */
    OsTskHighestSet();

    if ((OS_TSK_RUNNING & taskCB->taskStatus) != 0) {
        UNI_FLAG |= OS_FLG_TSK_REQ;

        ListTailAdd(&taskCB->pendList, &g_tskRecyleList);

        RUNNING_TASK = OS_PST_ZOMBIE_TASK;
        RUNNING_TASK->taskPid = taskPid;
        RUNNING_TASK->taskStatus = taskCB->taskStatus;

        taskCB->taskStatus = OS_TSK_UNUSED;

        if (OS_INT_INACTIVE) {
            OsTaskTrap();
            NOS_TaskIntRestore(intSave);

            return NOS_OK;
        }
    } else {
        taskCB->taskStatus = OS_TSK_UNUSED;
        ListAdd(&taskCB->pendList, &g_tskCBFreeList);
    }

    NOS_TaskIntRestore(intSave);

    /* if deleteing current task this is unreachable. */
    return NOS_OK;
}

#endif