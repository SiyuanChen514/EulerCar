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
  * @file      nos_task_internal.h
  */
#ifndef NOS_TASK_INTERNAL_H
#define NOS_TASK_INTERNAL_H

#include "nos_task_external.h"
#include "nos_task_sched_external.h"

/*
 * 模块内宏定义
 */
#define OS_TSK_STACK_MAGIC OS_TSK_STACK_MAGIC_WORD

/* 锁 */
INLINE unsigned int NOS_TaskIntLock(void)
{
#ifdef OS_OPTION_306X
    return NOS_SystickLock();
#else
    return (unsigned int)NOS_IntLock();
#endif
}

/* 锁恢复 */
INLINE void NOS_TaskIntRestore(unsigned int intSave)
{
#ifdef OS_OPTION_306X
    NOS_SystickRestore(intSave);
#else
    NOS_IntRestore(intSave);
#endif
}

/*
 * 模块内全局变量声明
 */
extern struct TagListObject g_tskCBFreeList;
extern struct TagListObject g_tskRecyleList;

/*
 * 模块内函数声明
 */
extern unsigned int OsTaskDelStatusCheck(struct TagTskCB *taskCB);
extern void OsTskRecycle(void);
extern unsigned long NOS_TaskUpdateExpirCnt(unsigned int taskPid);

INLINE void OsMoveTaskToReady(struct TagTskCB *taskCB)
{
    /* If task is not blocked then move it to ready list */
    if ((taskCB->taskStatus & OS_TSK_BLOCK) == 0) {
        OsTskReadyAdd(taskCB);

        if ((OS_FLG_BGD_ACTIVE & UNI_FLAG) != 0) {
            OsSpinUnlockTaskRq(taskCB);
            /* task Schedule */
            OsTskSchedule();
            return;
        }
    }

    OsSpinUnlockTaskRq(taskCB);
}

INLINE unsigned int OsTaskCreateUsrStkCfgInit(struct TskInitParam *initParam,
    unsigned int *curStackSize, struct TagTskCB *taskCB)
{
    (void)taskCB;
    /* Obtains the stack size. */
    *curStackSize = initParam->stackSize;
    return NOS_OK;
}

#endif /* NOS_TASK_INTERNAL_H */
