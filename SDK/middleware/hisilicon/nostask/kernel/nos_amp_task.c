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
  * @file      nos_amp_task.c
  */
#include "nos_task_external.h"

struct TagOsTskSortedDelayList g_tskSortedDelay;
struct TagOsRunQue g_runQueue;  // 核的局部运行队列

/*
 * 描述: Task schedule, switch to the highest task.
 */
void OsTskSchedule(void)
{
    /* HIDSP平台为关中断下调度，外层已经关中断 */
    /* Find the highest task */
    OsTskHighestSet();

    /* In case that running is not highest then reschedule */
    if ((g_highestTask != RUNNING_TASK) && (g_uniTaskLock == 0)) {
        UNI_FLAG |= OS_FLG_TSK_REQ;

        /* only if there is not HWI or SWI or TICK the trap */
        if (OS_INT_INACTIVE) {
            OsTaskTrap();
            return;
        }
    }

    return;
}

static void OsReAddToTimerList(struct TagListObject *taskList, struct TagTskCB *taskCB)
{
    struct TagTskCB *tskDelay = NULL;
    if (ListEmpty(taskList)) {
        ListTailAdd(&taskCB->timerList, taskList);
    } else {
        /* Put the task to right location */
        LIST_FOR_EACH(tskDelay, taskList, struct TagTskCB, timerList) {
            if (tskDelay->expirationTick > taskCB->expirationTick) {
                break;
            }
        }
        /* Put the task to right location */
        ListTailAdd(&taskCB->timerList, &tskDelay->timerList);
    }
}

/* 任务扫描 */
unsigned long long NOS_GetCycle(void);
void OsTaskScan(void)
{
    struct TagTskCB *taskCB = NULL;
    bool needSchedule = FALSE;
    /* 获取延时链表 */
    struct TagListObject *tskSortedDelayList = &g_tskSortedDelay.tskList;
    unsigned long long curTick = (unsigned long long)(NOS_GetCycle());

    LIST_FOR_EACH(taskCB, tskSortedDelayList, struct TagTskCB, timerList) {
        if ((unsigned long long)taskCB->expirationTick > curTick) {
            break;
        }
        /* 从链表中删除 */
        ListDelete(&taskCB->timerList);
#if defined(OS_OPTION_306X)
        /* 任务是否被延时 */
        if ((OS_TSK_PERIOD & taskCB->taskStatus) != 0) {
            taskCB->expirationTick = (unsigned long long)taskCB->expirationTick +
             (unsigned long long)(taskCB->privateData);
            TSK_StatusClear(taskCB, OS_TSK_SUSPEND);
            OsReAddToTimerList(tskSortedDelayList, taskCB);
        }
#endif
        /* 任务是否被阻塞 */
        if ((OS_TSK_PEND & taskCB->taskStatus) != 0) {
            TSK_StatusClear(taskCB, OS_TSK_PEND);
            ListDelete(&taskCB->pendList);
        } else if (((OS_TSK_MSG_PEND | OS_TSK_VOS_PEND) & taskCB->taskStatus) != 0) {
            TSK_StatusClear(taskCB, (OS_TSK_MSG_PEND | OS_TSK_VOS_PEND));
        } else if (((OS_TSK_EVENT_PEND | OS_TSK_VOS_PEND) & taskCB->taskStatus) != 0) {
            TSK_StatusClear(taskCB, (OS_TSK_EVENT_PEND | OS_TSK_VOS_PEND));
        } else if ((OS_TSK_QUEUE_PEND & taskCB->taskStatus) != 0) {
            ListDelete(&taskCB->pendList);
            TSK_StatusClear(taskCB, OS_TSK_QUEUE_PEND);
        } else {
            /* 清除任务状态 */
            TSK_StatusClear(taskCB, OS_TSK_DELAY);
        }

        /* timer锁只要锁到链表删除位置，下面的ready添加为tcb与rq的操作，锁rq */
        if ((OS_TSK_SUSPEND_READY_BLOCK & taskCB->taskStatus) == 0) {
            OsTskReadyAddBGD(taskCB);
            needSchedule = TRUE;
        } else {
            taskCB->expirationCnt++;
        }

        taskCB = LIST_COMPONENT(tskSortedDelayList, struct TagTskCB, timerList);
    }

    if (needSchedule) {
        /* 发生调度 */
        OsTskScheduleFast();
    }
}
