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
  * @file      nos_amp_task_internal.h
  */
#ifndef NOS_AMP_TASK_INTERNAL_H
#define NOS_AMP_TASK_INTERNAL_H

#include "nos_task_external.h"

#define OS_TSK_PRIO_BIT_MAP_POW 5
#define OS_TSK_PRIO_RDY_BIT  0x80000000U

extern unsigned int OsTskAMPInit(void);

/* 加入队列尾部 */
INLINE void OsEnqueueTaskAmp(struct TagOsRunQue *runQue, struct TagTskCB *tsk)
{
    unsigned int priority = tsk->priority;

    ListTailAdd(&tsk->pendList, &runQue->readyList[priority]);
    runQue->taskReadyListBitMap |= (OS_TSK_PRIO_RDY_BIT >> priority);
}

/* 加入队列头部 */
INLINE void OsEnqueueTaskHeadAmp(struct TagOsRunQue *runQue, struct TagTskCB *tsk)
{
    unsigned int priority = tsk->priority;

    ListAdd(&tsk->pendList, &runQue->readyList[priority]);
    runQue->taskReadyListBitMap |= (OS_TSK_PRIO_RDY_BIT >> priority);
}

/* 出队 */
INLINE void OsDequeueTaskAmp(struct TagOsRunQue *runQue, struct TagTskCB *tsk)
{
    unsigned int priority = tsk->priority;
    struct TagListObject *readyList = &runQue->readyList[priority];

    /* 从链表中删除 */
    ListDelete(&tsk->pendList);
    if (ListEmpty(readyList)) {
        runQue->taskReadyListBitMap &= ~(OS_TSK_PRIO_RDY_BIT >> priority);
    }
}

/* 将运行队列的任务计数++ */
INLINE void OsIncNrRunning(struct TagOsRunQue *runQue)
{
    (void)runQue;
    return;
}

/* 将运行队列的任务计数-- */
INLINE void OsDecNrRunning(struct TagOsRunQue *runQue)
{
    (void)runQue;
    return;
}

#endif /* NOS_AMP_TASK_INTERNAL_H */
