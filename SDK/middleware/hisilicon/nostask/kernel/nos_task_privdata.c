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
  * @file      nos_task_privdata.c
  */
#include "nos_task_external.h"

/*
 * 描述: 获取当前任务的私有数据。只由本任务操作，不存在竞争，无须关中断。
 */
uintptr_t NOS_TaskPrivateDataGet(void)
{
    struct TagTskCB *taskCB = RUNNING_TASK;

    return taskCB->privateData;
}

/*
 * 描述: 设置当前任务的私有数据。只由本任务操作，不存在竞争，无须关中断。
 */
void NOS_TaskPrivateDataSet(uintptr_t privateData)
{
    struct TagTskCB *taskCB = RUNNING_TASK;
    taskCB->privateData = privateData;
}

#if defined(OS_OPTION_306X)
/*
 * 描述: 获取指定任务的私有数据。
 */
unsigned int OsTaskPrivateDataGetById(unsigned int taskPid, uintptr_t *privateData)
{
    struct TagTskCB *taskCB;

    if (privateData == NULL) {
        return OS_ERRNO_TSK_PTR_NULL;
    }

    /* check taskId */
    if (CheckTaskPidOverflow(taskPid)) {
        return OS_ERRNO_TSK_ID_INVALID;
    }

    taskCB = GetTcbHandle(taskPid);
    // 排除大部分不法情况
    if (taskCB->taskStatus == OS_TSK_UNUSED) {
        return OS_ERRNO_TSK_NOT_CREATED;
    }

    *privateData = taskCB->privateData;

    return NOS_OK;
}

/*
 * 描述: 获取指定任务的私有数据。
 */
uintptr_t NOS_TaskPrivateDataGetById(unsigned int taskPid)
{
    uintptr_t privateData = 0;
    (void)OsTaskPrivateDataGetById(taskPid, &privateData);
    return privateData;
}


#endif
