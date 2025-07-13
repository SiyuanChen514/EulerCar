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
  * @file      nos_task_external.h
  */
#ifndef NOS_TASK_EXTERNAL_H
#define NOS_TASK_EXTERNAL_H

#include "nos_list_external.h"
#include "nos_sys_external.h"
#include "nos_cpu_external.h"

#include "nos_config.h"

#define OS_MAX_U32 0xFFFFFFFFU

struct TagOsRunQue {
    unsigned int taskReadyListBitMap;
    struct TagListObject readyList[OS_TSK_NUM_OF_PRIORITIES];
};

struct TagOsTskSortedDelayList {
    /* 延时任务链表 */
    struct TagListObject tskList;
};

/*
 * 任务线程及进程控制块的结构体统一定义, 64字节对齐。
 *
 * stackPointer、taskStatus、priority、topOfStack、taskPID及
 * cpSaveAreaA、cpSaveAreaB、mmuBase、usp、cidr的位置不能变动，汇编中已写死。
 */
struct TagTskCB {
    /* 当前任务的SP */
    void *stackPointer;
    /* 任务状态,后续内部全改成U32 */
    unsigned int taskStatus;
    /* 任务的运行优先级 */
    unsigned short priority;
    /* 超时次数 */
    unsigned int expirationCnt;

    /* 任务栈大小 */
    unsigned int stackSize;
    /* 任务PID：SMP下为任务OS内部索引,AMP下为核号 | 任务索引 */
    unsigned int taskPid;

    /* 任务栈顶 */
    uintptr_t topOfStack;

    /* 任务入口函数 */
    TskEntryFunc taskEntry;
    /* 任务的参数 */
    uintptr_t args[4];
    /* 任务的原始优先级 */
    unsigned short origPriority;
    /* 任务栈配置标记 */
    unsigned short stackCfgFlg;
    /* 期望接收消息的QID */
    unsigned char reserved;
    /* 私有数据 */
    uintptr_t privateData;
    /* 信号量链表指针 */
    struct TagListObject pendList;
    /* 任务延时链表指针 */
    struct TagListObject timerList;

    /* 任务记录的最后一个错误码 */
    unsigned int lastErr;

    /* 任务恢复的时间点(单位Tick) */
    unsigned long long expirationTick;
};

extern unsigned short g_uniTaskLock;
extern unsigned int g_idleTaskID;
extern struct TagOsRunQue g_runQueue;
extern struct TagTskCB *g_runningTask;
extern struct TagTskCB *g_highestTask;
extern struct TagOsTskSortedDelayList g_tskSortedDelay;

extern unsigned int g_tskMaxNum;

#define OS_TSK_STACK_CFG_BY_USER 1
#define OS_TSK_STACK_CFG_BY_SYS  0

#define OS_TSK_PARA_0   0
#define OS_TSK_PARA_1   1
#define OS_TSK_PARA_2   2
#define OS_TSK_PARA_3   3

#define OS_TSK_STACK_TOP_MAGIC  0xAAAAAAAA

#define OS_TSK_OP_FREE       0
#define OS_TSK_OP_MIGRATING  (1UL << 0)   // 任务在迁移中，看到delete立即返回
#define OS_TSK_OP_SUSPENDING (1UL << 2)   // 任务在挂起中，看到delete立即返回
#define OS_TSK_OP_RESUMING   (1UL << 3)   // 任务在恢复中，看到delete立即返回
#define OS_TSK_OP_DELETING   (1UL << 9)   // 任务在删除中，必须等待上述迁移、挂起操作完
#define OS_TSK_OP_MOVING     (1UL << 10)  // 任务在迁移/回迁过程中置该标志，确保当前不会被他人操作该任务

#define OS_TSK_SUSPEND_READY_BLOCK (OS_TSK_SUSPEND | OS_TSK_READY)
#define OS_TASK_LOCK_DATA          g_uniTaskLock
#define IDLE_TASK_ID               g_idleTaskID
#define RUNNING_TASK               g_runningTask

/* 内核进程的进程及线程调度控制块使用同一类型 */
#define OS_MAX_TCB_NUM             (OS_TSK_MAX_SUPPORT_NUM + 1 + 1)  // (g_tskMaxNum + 1 + 1) 1个IDLE，1个无效任务

/* 定义任务的缺省任务栈大小 */
#define OS_PST_ZOMBIE_TASK             (&g_tskCBArray[OS_MAX_TCB_NUM - 1])
#define GET_TCB_PEND(ptr)              LIST_COMPONENT(ptr, struct TagTskCB, pendList)
#define OS_TSK_BLOCK                                                                                            \
        (OS_TSK_DELAY | OS_TSK_PEND | OS_TSK_SUSPEND | OS_TSK_FSEM_PEND | OS_TSK_MSG_PEND | OS_TSK_QUEUE_PEND | \
         OS_TSK_EVENT_PEND | OS_TSK_DELETING)

extern struct TagTskCB g_tskCBArray[OS_MAX_TCB_NUM];

extern void OsTaskScan(void);
extern void OsTskUnlock(void);
extern void OsTskSchedule(void);
extern void OsTskEntry(unsigned int taskId);
extern void OsTaskExit(struct TagTskCB *tsk);
extern void OsTskReadyAdd(struct TagTskCB *task);
extern void OsTskReadyDel(struct TagTskCB *taskCB);
extern void OsTskSwitchHookCaller(unsigned int prevPid, unsigned int nextPid);
extern void OsTskTimerAdd(struct TagTskCB *taskCB, uintptr_t timeout);

extern unsigned int OsIdleTskAMPCreate(void);
extern unsigned int OsTskMaxNumGet(void);
extern unsigned int OsTaskDelete(unsigned int taskPid);
extern unsigned int OsTaskPrivateDataGetById(unsigned int taskPid, uintptr_t *privateData);
extern unsigned int OsTaskCreateOnly(unsigned int *taskPid, struct TskInitParam *initParam, bool isSmpIdle);

extern void OsTaskFirstTimeSwitch(void);

/* get task tcb */
INLINE struct TagTskCB *GetTcbHandle(unsigned int taskPid)
{
    return (struct TagTskCB *)g_tskCBArray + taskPid;
}

/* 任务调度 */
INLINE void OsTskScheduleFast(void)
{
    OsTskSchedule();
}

/* 任务调度 */
INLINE void OsTskScheduleFastPS(uintptr_t intSave)
{
    (void)intSave;
    OsTskSchedule();
}

/* 任务是否创建 */
INLINE unsigned int TSK_IsUnused(struct TagTskCB *tsk)
{
    return tsk->taskStatus == OS_TSK_UNUSED;
}

/* check task status */
INLINE unsigned int TSK_StatusTst(struct TagTskCB *tsk, unsigned int statBit)
{
    return (tsk->taskStatus & statBit) != 0;
}

/* status clear */
INLINE unsigned int TSK_StatusClear(struct TagTskCB *tsk, unsigned int statBit)
{
    return tsk->taskStatus &= ~statBit;
}

/* status set */
INLINE unsigned int TSK_StatusSet(struct TagTskCB *tsk, unsigned int statBit)
{
    return tsk->taskStatus |= statBit;
}

/* check pid */
INLINE unsigned int CheckTaskPidOverflow(unsigned int taskId)
{
    return taskId >= OS_TSK_MAX_SUPPORT_NUM + 1;
}

/*
 * 模块内内联函数定义
 */
INLINE void OsTskHighestSet(void)
{
    unsigned int rdyListIdx = OsGetLMB1(g_runQueue.taskReadyListBitMap);
    struct TagListObject *readyList = &g_runQueue.readyList[rdyListIdx];

    g_highestTask = GET_TCB_PEND(OS_LIST_FIRST(readyList));
}

/*
 * 描述: 将任务添加到就绪队列。
 */
INLINE void OsTskReadyAddBGD(struct TagTskCB *task)
{
    OsTskReadyAdd(task);
}

#endif /* NOS_TASK_EXTERNAL_H */
