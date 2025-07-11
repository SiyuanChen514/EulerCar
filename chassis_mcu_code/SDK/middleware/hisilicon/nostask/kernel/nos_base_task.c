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
  * @file      nos_base_task.c
  */
#include "nos_amp_task_internal.h"
#include "nos_base_task.h"

INLINE void OsTskReadyAddOnly(struct TagTskCB *task)
{
    struct TagOsRunQue *rq = &g_runQueue;

    // 本核还未启动时，允许加入本核运行队列
    TSK_StatusSet(task, OS_TSK_READY);

    /* 入队 */
    OsEnqueueTaskAmp(rq, task);

    OsIncNrRunning(rq);

    return;
}

/*
 * 描述: 将任务添加到就绪队列, 调用者确保不会换核，并锁上rq
 * 备注: SMP/AMP归一
 */
void OsTskReadyAdd(struct TagTskCB *task)
{
    /* 将任务添加到就绪队列 */
    OsTskReadyAddOnly(task);
}

/*
 * 描述: 将任务添加到就绪链表头，关中断外部保证
 * 备注: SMP/AMP归一
 */
void OsTskReadyDel(struct TagTskCB *taskCB)
{
    struct TagOsRunQue *runQue = NULL;
    TSK_StatusClear(taskCB, OS_TSK_READY);

    runQue = &g_runQueue;
    // 出队
    OsDequeueTaskAmp(runQue, taskCB);

    // 更新源运行队列的功耗值
    OsDecNrRunning(runQue);

    return;
}

/*
 * 描述: 添加任务到超时链表，TCB锁由外部调用者保证,TCB已在外部锁
 * 备注: SMP/AMP归一
 */
void OsTskTimerAdd(struct TagTskCB *taskCB, uintptr_t timeout)
{
    struct TagTskCB *tskDelay = NULL;
    struct TagOsTskSortedDelayList *tskDlyBase = NULL;
    struct TagListObject *taskList = NULL;

    tskDlyBase = &g_tskSortedDelay;

    /* get tick */
    unsigned long long curTick = NOS_GetCycle();
    if (curTick == 0) {
        return;
    }

    taskCB->expirationTick = curTick + timeout;
    taskList = &tskDlyBase->tskList;

    if (!ListEmpty(taskList)) {
         /* Put the task to right location */
        LIST_FOR_EACH(tskDelay, taskList, struct TagTskCB, timerList) {
            if (tskDelay->expirationTick > taskCB->expirationTick) {
                break;
            }
        }
        ListTailAdd(&taskCB->timerList, &tskDelay->timerList);
    } else {
        /* 加入超时链表尾部 */
        ListTailAdd(&taskCB->timerList, taskList);
    }

    return;
}
