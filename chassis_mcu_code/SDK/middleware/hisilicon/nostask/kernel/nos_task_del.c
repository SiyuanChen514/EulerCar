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
  * @file      nos_task_del.c
  */
#include "nos_task_internal.h"
#include "nos_buildef.h"

#if defined(OS_OPTION_TASK_DELETE)
/*
 * 描述: 删除一个任务线程
 */
unsigned int OsTaskDelStatusCheck(struct TagTskCB *taskCB)
{
    /* If the task is running and scheduler is locked */
    /* then can not delete it */
    if ((taskCB == RUNNING_TASK) && (OS_TASK_LOCK_DATA != 0)) {
        return OS_ERRNO_TSK_DELETE_LOCKED;
    }

    if (TSK_StatusTst(taskCB, OS_TSK_QUEUE_BUSY)) {
        return OS_ERRNO_TSK_QUEUE_DOING;
    }

    return NOS_OK;
}
#endif

void OsTaskExit(struct TagTskCB *tsk)
{
#if defined(OS_OPTION_TASK_DELETE)
    /* delete task */
    (void)NOS_TaskDeleteInner(tsk->taskPid);
#else
    uintptr_t intSave = NOS_TaskIntLock();

    OsTskReadyDel(tsk);
    tsk->taskStatus = OS_TSK_UNUSED;

    /* 任务调度 */
    OsTskSchedule();

    NOS_TaskIntRestore(intSave);
#endif
}

/*
 * 描述: 删除一个任务线程
 */
unsigned int NOS_TaskDeleteInner(unsigned int taskPid)
{
    return OsTaskDelete(taskPid);
}