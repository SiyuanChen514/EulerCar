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
  * @file      nos_task_minor.c
  */
#include "nos_task_internal.h"
#include "nos_buildef.h"

#if defined(OS_OPTION_TASK_YIELD)
INLINE unsigned int OsTaskYieldProc(unsigned int nextTaskId, unsigned int taskPrio, unsigned int *yieldTo,
    struct TagTskCB *currTask, struct TagListObject *tskPriorRdyList)
{
    struct TagTskCB *nextTask = NULL;
    /* there is no task that user particularly wishes */
    /* therefore second task will become ready */
    if (nextTaskId == OS_TSK_NULL_ID) {
        if (yieldTo != NULL) {
            *yieldTo = (GET_TCB_PEND(OS_LIST_FIRST(&currTask->pendList)))->taskPid;
        }
        ListDelete(&currTask->pendList);
        ListTailAdd(&currTask->pendList, tskPriorRdyList);
    } else {
        if (yieldTo != NULL) {
            *yieldTo = nextTaskId;
        }

        /* get task Tcb */
        nextTask = GetTcbHandle(nextTaskId);
        if ((nextTask->priority == taskPrio) && TSK_StatusTst(nextTask, OS_TSK_READY)) {
            ListDelete(&nextTask->pendList);
            ListAdd(&nextTask->pendList, tskPriorRdyList);
        } else { /* task is illegal */
            return OS_ERRNO_TSK_YIELD_INVALID_TASK;
        }
    }

    return NOS_OK;
}

/*
 * 描述: 调整指定优先级的任务调度顺序。调用者负责关中断
 * 备注: NA
 */
static unsigned int OsTaskYield(unsigned short taskPrio, unsigned int nextTaskId, unsigned int *yieldTo)
{
    unsigned int ret;
    unsigned int tskCount = 0;
    struct TagTskCB *currTask = NULL;
    struct TagTskCB *runTask = NULL;
    struct TagListObject *tskPriorRdyList = NULL;
    struct TagListObject *currNode = NULL;

    runTask = RUNNING_TASK;

    // 锁当前running任务的rq，yield只操作本核
    OsSpinLockTaskRq(runTask);
    tskPriorRdyList = &g_runQueue.readyList[taskPrio];
    /* In case there are more then one ready tasks at */
    /* this priority, remove first task and add it */
    /* to the end of the queue */
    currTask = GET_TCB_PEND(OS_LIST_FIRST(tskPriorRdyList));

    for (currNode = tskPriorRdyList->next; currNode != tskPriorRdyList; currNode = currNode->next) {
        tskCount++;
    }

    if (tskCount > 1) {
        ret = OsTaskYieldProc(nextTaskId, taskPrio, yieldTo, currTask, tskPriorRdyList);
        if (ret != NOS_OK) {
            OsSpinUnlockTaskRq(runTask);
            return ret;
        }

        // 如果是当前的running任务
        if (TSK_StatusTst(currTask, OS_TSK_RUNNING)) {
            OsSpinUnlockTaskRq(runTask);
            OsTskScheduleFast();

            return NOS_OK;
        }
    } else { /* There is only one task or none */
        OsSpinUnlockTaskRq(runTask);

        return OS_ERRNO_TSK_YIELD_NOT_ENOUGH_TASK;
    }

    OsSpinUnlockTaskRq(runTask);

    return NOS_OK;
}
#endif

/*
 * 描述: 延迟当前运行任务的执行
 */
unsigned int NOS_TaskDelayInner(unsigned int tick)
{
    unsigned int ret;
    uintptr_t intSave;
    struct TagTskCB *runTask = NULL;

    intSave = NOS_TaskIntLock();
    // 注意:宏里面包含了UNI_FLAG的隐晦percpu变量，需要关中断保护，否则任务迁移读取的变量为它核变量
    if (OS_INT_ACTIVE) {
        NOS_TaskIntRestore(intSave);
        return OS_ERRNO_TSK_DELAY_IN_INT;
    }

    /* 检查是否锁任务 */
    if (OS_TASK_LOCK_DATA != 0) {
        NOS_TaskIntRestore(intSave);
        return OS_ERRNO_TSK_DELAY_IN_LOCK;
    }
    /* 获取正在运行的任务 */
    runTask = RUNNING_TASK;

#if defined(OS_OPTION_TICK)
    if (tick > 0) {
        OsSpinLockTaskRq(runTask);
        OsTskReadyDel(runTask);
        TSK_StatusSet(runTask, OS_TSK_DELAY);
        OsTskTimerAdd(runTask, tick);
        OsSpinUnlockTaskRq(runTask);

        /* 发生调度 */
        OsTskScheduleFastPS(intSave);
        NOS_TaskIntRestore(intSave);

        return NOS_OK;
    }
#else
    (void)tick;
    (void)runTask;
#endif

#if defined(OS_OPTION_TASK_YIELD)
    // 内部调用使用内部接口，避免for告警
    ret = OsTaskYield(runTask->priority, OS_TSK_NULL_ID, NULL);
#else
    ret = NOS_OK;
#endif
    NOS_TaskIntRestore(intSave);

    return ret;
}

#if defined(OS_OPTION_TASK_YIELD)
/*
 * 描述: 调整指定优先级的任务调度顺序。
 */
unsigned int NOS_TaskYield(unsigned short taskPrio, unsigned int nextTask, unsigned int *yieldTo)
{
    uintptr_t intSave;
    unsigned int ret;

    if (taskPrio > OS_TSK_PRIORITY_LOWEST) {
        return OS_ERRNO_TSK_PRIOR_ERROR;
    }

    /* check taskId */
    if ((nextTask != OS_TSK_NULL_ID) && (CheckTaskPidOverflow(nextTask))) {
        return OS_ERRNO_TSK_ID_INVALID;
    }

    /* 关中断 */
    intSave = NOS_IntLock();
    ret = OsTaskYield(taskPrio, nextTask, yieldTo);
    NOS_IntRestore(intSave);

    return ret;
}
#endif

/*
 * 描述: 锁任务调度
 * 备注: NA
 */
void NOS_TaskLock(void)
{
    uintptr_t intSave;

    /* 关中断 */
    intSave = NOS_IntLock();

    if (g_uniTaskLock == 0) {
        UNI_FLAG &= (~OS_FLG_TSK_REQ);
    }

    OS_TASK_LOCK_DATA++;
    NOS_IntRestore(intSave);
}

/*
 * 描述: 锁任务调度
 * 备注: NA
 */
void NOS_TaskUnlock(void)
{
    /* 关中断 */
    uintptr_t intSave = NOS_IntLock();

    if (UNLIKELY(g_uniTaskLock == 0)) {
        NOS_IntRestore(intSave);
        return;
    }

    g_uniTaskLock--;
    if (g_uniTaskLock == 0) {
        if ((OS_FLG_BGD_ACTIVE & UNI_FLAG) != 0) {
            OsTskScheduleFastPS(intSave);
        }
    }

    NOS_IntRestore(intSave);
}

#if defined(OS_OPTION_306X)
unsigned long NOS_TaskUpdateExpirCnt(unsigned int taskPid)
{
    uintptr_t intSave;
    unsigned int cnt = 0;
    struct TagTskCB *taskCB = NULL;
    /* check taskId */
    if (CheckTaskPidOverflow(taskPid)) {
        return OS_ERRNO_TSK_ID_INVALID;
    }
    /* get task Tcb */
    taskCB = GetTcbHandle(taskPid);
    intSave = NOS_TaskIntLock();
    /* 判断任务是否存在 */
    if (TSK_IsUnused(taskCB)) {
        NOS_TaskIntRestore(intSave);
        return OS_ERRNO_TSK_NOT_CREATED;
    }
    /* 判断超时次数 */
    cnt = taskCB->expirationCnt;
    if (cnt > 0) {
        taskCB->expirationCnt--;
    }
    NOS_TaskIntRestore(intSave);
    return cnt;
}

unsigned int NOS_TaskStartPeriod(unsigned int taskPid, unsigned int timeout)
{
    uintptr_t intSave;
    struct TagTskCB *taskCB = NULL;
    /* check taskId */
    if (CheckTaskPidOverflow(taskPid)) {
        return OS_ERRNO_TSK_ID_INVALID;
    }
    /* get task Tcb */
    taskCB = GetTcbHandle(taskPid);
    intSave = NOS_TaskIntLock();
    /* 判断任务是否存在 */
    if (TSK_IsUnused(taskCB)) {
        NOS_TaskIntRestore(intSave);
        return OS_ERRNO_TSK_NOT_CREATED;
    }
    TSK_StatusSet(taskCB, OS_TSK_PERIOD);
    OsTskTimerAdd(taskCB, timeout);
    NOS_TaskIntRestore(intSave);
    return NOS_OK;
}

unsigned int NOS_TaskStopPeriod(unsigned int taskPid)
{
    uintptr_t intSave;
    struct TagTskCB *tcb = NULL;
    /* check taskPid */
    if (CheckTaskPidOverflow(taskPid)) {
        return OS_ERRNO_TSK_ID_INVALID;
    }
    /* get taskTcb */
    tcb = GetTcbHandle(taskPid);
    intSave = NOS_TaskIntLock();
    /* 判断任务是否存在 */
    if (TSK_IsUnused(tcb)) {
        NOS_TaskIntRestore(intSave);
        return OS_ERRNO_TSK_NOT_CREATED;
    }
    if (((tcb->taskStatus) & OS_TSK_PERIOD) != 0) {
        TSK_StatusClear(tcb, OS_TSK_PERIOD);
        ListDelete(&tcb->timerList);
    }
    NOS_TaskIntRestore(intSave);
    return NOS_OK;
}
#endif // OS_OPTION_306X
