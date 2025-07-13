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
  * @file      nos_task_init.c
  */
#include "nos_task_internal.h"
#include "nos_amp_task_internal.h"
#include "nos_buildef.h"

/* Unused TCBs and ECBs that can be allocated. */
struct TagListObject g_tskCBFreeList = LIST_OBJECT_INIT(g_tskCBFreeList);
struct TagListObject g_tskRecyleList = LIST_OBJECT_INIT(g_tskRecyleList);

/*
 * 描述: Task init function.
 */
unsigned int OsTskInit(void);
unsigned int OsTskInit(void)
{
    unsigned int ret;

    /* AMP task init */
    ret = OsTskAMPInit();
    if (ret != NOS_OK) {
        return ret;
    }

#if defined(OS_OPTION_TICK)
    /* tick 扫描 */
    g_taskScanHook = OsTaskScan;
#endif

    return NOS_OK;
}

/*
 * 描述: Active task manage, leave main.
 */
unsigned int OsActivate(void);
unsigned int OsActivate(void)
{
    unsigned int ret;

    ret = OsIdleTskAMPCreate();
    if (ret != NOS_OK) {
        return ret;
    }

    /* Indicate that background task is running. */
    UNI_FLAG |= OS_FLG_BGD_ACTIVE | OS_FLG_TSK_REQ;

    /* Start Multitasking. */
    OsFirstTimeSwitch();

    return OS_ERRNO_TSK_ACTIVE_FAILED;
}

/*
 * 描述: All task entry
 */
void OsTskEntry(unsigned int taskId)
{
    struct TagTskCB *taskCB = NULL;
    uintptr_t intSave;

    (void)taskId;
    taskCB = RUNNING_TASK;

    taskCB->taskEntry(taskCB->args[OS_TSK_PARA_0], taskCB->args[OS_TSK_PARA_1], taskCB->args[OS_TSK_PARA_2],
                      taskCB->args[OS_TSK_PARA_3]);

    // 调度结束后会开中断，所以不需要自己添加开中断
    intSave = NOS_IntLock();

    OS_TASK_LOCK_DATA = 0;

    /* NOS_TaskDeleteInner不能关中断操作，否则可能会导致它核发SGI等待本核响应时死等 */
    NOS_IntRestore(intSave);

    OsTaskExit(taskCB);
}

/*
 * 描述: Create a task
 */
unsigned int NOS_TaskCreateInner(unsigned int *taskPid, struct TskInitParam *initParam)
{
    unsigned int ret;

    /* 任务创建并挂起 */
    ret = NOS_TaskCreateOnlyInner(taskPid, initParam);
    if (ret != NOS_OK) {
        return ret;
    }

    return NOS_TaskResumeInner(*taskPid);
}

/*
 * 描述: 创建任务参数检查
 */
static unsigned int OsTaskCreateParaCheck(const unsigned int *taskPid, struct TskInitParam *initParam)
{
    if ((taskPid == NULL) || (initParam == NULL)) {
        return OS_ERRNO_TSK_PTR_NULL;
    }

    /* check callback */
    if (initParam->taskEntry == NULL) {
        return OS_ERRNO_TSK_ENTRY_NULL;
    }

    /* check Prio */
    if (initParam->taskPrio > OS_TSK_PRIORITY_LOWEST) {
        return OS_ERRNO_TSK_PRIOR_ERROR;
    }

    /* 创建任务线程时做校验 */
    if (initParam->name == NULL || initParam->name[0] == '\0') {
        return OS_ERRNO_TSK_NAME_EMPTY;
    }

    return NOS_OK;
}

static unsigned int OsTaskCreateParaCheckStack(struct TskInitParam *initParam)
{
    unsigned long long stackAddrLen;

    /* 如未设置栈大小即赋予默认值 */
    if (initParam->stackSize == 0) {
        initParam->stackSize = OS_TSK_DEFAULT_STACK_SIZE;
    }

    if (((OS_TSK_STACK_SIZE_ALIGN - 1) & initParam->stackSize) != 0) {
        return OS_ERRNO_TSK_STKSZ_NOT_ALIGN;
    }

    /* 检查是否对齐 */
    if (((OS_TSK_STACK_SIZE_ALIGN - 1) & (uintptr_t)initParam->stackAddr) != 0) {
        return OS_ERRNO_TSK_STACKADDR_NOT_ALIGN;
    }

    /* 使用用户内存，则需要包含OS使用的资源，size最小值要包含OS的消耗 */
    if (initParam->stackAddr != 0) {
        if (initParam->stackSize < OS_TSK_MIN_STACK_SIZE) {
            return OS_ERRNO_TSK_USR_STKSZ_TOO_SMALL;
        }
        /* 保证栈空间在4G范围内不溢出 */
        stackAddrLen = (unsigned long long)(initParam->stackAddr) + (unsigned long long)(initParam->stackSize);
        if (stackAddrLen > OS_MAX_U32) {
            return OS_ERRNO_TSK_STACKADDR_TOO_BIG;
        }
    } else {
        return OS_ERRNO_TSK_STACKADDR_TOO_BIG;
    }

    return NOS_OK;
}

void OsTskRecycle(void)
{
    struct TagTskCB *taskCB = NULL;

    /* 释放掉之前自删除任务的资源,自删除时,由于任务还处于运行态,不会及时释放资源 */
    /* 调用处已加recycle list锁 */
    while (!ListEmpty(&g_tskRecyleList)) {
        taskCB = GET_TCB_PEND(OS_LIST_FIRST(&g_tskRecyleList));
        ListDelete(OS_LIST_FIRST(&g_tskRecyleList));
        ListTailAdd(&taskCB->pendList, &g_tskCBFreeList);
    }
}

/*
 * 描述: 初始化栈
 */
static void OsTskStackInit(unsigned int stackSize, uintptr_t topStack)
{
    unsigned int loop;
    unsigned int stackMagicWord = OS_TSK_STACK_MAGIC;

    /* 初始化任务栈，并写入栈魔术字 */
    for (loop = 1; loop < (stackSize / sizeof(unsigned int)); loop++) {
        *((unsigned int *)topStack + loop) = stackMagicWord;
    }
    *((unsigned int *)(topStack)) = OS_TSK_STACK_TOP_MAGIC;
}

INLINE unsigned int OsTaskCreateChkAndGetTcb(struct TagTskCB **taskCB, bool isSmpIdle)
{
    (void)isSmpIdle;

#if defined(OS_OPTION_TASK_DELETE)
    /* 回收资源 */
    OsTskRecycle();
#endif

    if (ListEmpty(&g_tskCBFreeList)) {
        return OS_ERRNO_TSK_TCB_UNAVAILABLE;
    }

    // 先获取到该控制块
    *taskCB = GET_TCB_PEND(OS_LIST_FIRST(&g_tskCBFreeList));
    // 成功，从空闲列表中移除
    ListDelete(OS_LIST_FIRST(&g_tskCBFreeList));

    return NOS_OK;
}

INLINE unsigned int OsTaskCreateRsrcInit(unsigned int taskId, struct TskInitParam *initParam, struct TagTskCB *taskCB,
    uintptr_t **topStackOut, unsigned int *curStackSize)
{
    uintptr_t *topStack = NULL;
    unsigned int ret = NOS_OK;

    (void)taskId;

    /* 查看用户是否配置了任务栈。 */
    if (initParam->stackAddr != 0) {
        topStack = (void *)(initParam->stackAddr);
        /* 如有，则标记为用户配置 */
        taskCB->stackCfgFlg = OS_TSK_STACK_CFG_BY_USER;
        ret = OsTaskCreateUsrStkCfgInit(initParam, curStackSize, taskCB);
    } else {
#ifdef OS_OPTION_306X
    /* 如没有，则进行内存申请，并标记为系统配置 */
    ret = OS_ERRNO_TSK_NO_MEMORY;
#endif
    }

    *topStackOut = topStack;
    return ret;
}

INLINE void OsTskCreateTcbInit(uintptr_t stackPtr, struct TskInitParam *initParam,
    uintptr_t topStackAddr, uintptr_t curStackSize, struct TagTskCB *taskCB)
{
    /* Initialize the task's stack */
    taskCB->stackPointer = (void *)stackPtr;
    taskCB->privateData = (uintptr_t)initParam->privateData;
    /* 传参 */
    taskCB->args[OS_TSK_PARA_0] = (uintptr_t)initParam->args[OS_TSK_PARA_0];
    taskCB->args[OS_TSK_PARA_1] = (uintptr_t)initParam->args[OS_TSK_PARA_1];
    taskCB->args[OS_TSK_PARA_2] = (uintptr_t)initParam->args[OS_TSK_PARA_2];
    taskCB->args[OS_TSK_PARA_3] = (uintptr_t)initParam->args[OS_TSK_PARA_3];
    taskCB->topOfStack = topStackAddr;
    /* 传参 */
    taskCB->stackSize = (unsigned int)curStackSize;
    taskCB->priority = initParam->taskPrio;
    taskCB->origPriority = initParam->taskPrio;
    taskCB->taskEntry = initParam->taskEntry;
    taskCB->lastErr = 0;
    // SMP下就绪链表也做初始化，调度器会做维测，不允许其为空
    OS_LIST_INIT(&taskCB->pendList);
    OS_LIST_INIT(&taskCB->timerList);

    return;
}

INLINE void OsTskCreateTcbStatusSet(struct TagTskCB *taskCB, struct TskInitParam *initParam)
{
    (void)initParam;
    /* 上锁之后置INUSE，保证外部判断可靠，且从这里开始任务控制块确实可用，任务创建成功 */
    OsSpinLockTaskRq(taskCB);
    /* 设置任务状态 */
    taskCB->taskStatus = OS_TSK_SUSPEND | OS_TSK_INUSE;
    OsSpinUnlockTaskRq(taskCB);
    return;
}

/*
 * 描述: 创建一个任务但不进行激活
 */
unsigned int OsTaskCreateOnly(unsigned int *taskPid, struct TskInitParam *initParam, bool isSmpIdle)
{
    unsigned int taskId;
    uintptr_t intSave;
    uintptr_t *topStack = NULL;
    void *stackPtr = NULL;
    struct TagTskCB *taskCB = NULL;
    unsigned int ret;
    unsigned int curStackSize = 0;

    /* check para */
    ret = OsTaskCreateParaCheck(taskPid, initParam);
    if (ret != NOS_OK) {
        return ret;
    }
    /* check task stack */
    ret = OsTaskCreateParaCheckStack(initParam);
    if (ret != NOS_OK) {
        return ret;
    }
    intSave = NOS_TaskIntLock();
    /* 获取Tcb资源 */
    ret = OsTaskCreateChkAndGetTcb(&taskCB, isSmpIdle);
    if (ret != NOS_OK || taskCB == NULL) {
        NOS_TaskIntRestore(intSave);
        return ret;
    }

    taskId = taskCB->taskPid;

    ret = OsTaskCreateRsrcInit(taskId, initParam, taskCB, &topStack, &curStackSize);
    if (ret != NOS_OK) {
        ListAdd(&taskCB->pendList, &g_tskCBFreeList);
        NOS_TaskIntRestore(intSave);
        return ret;
    }
    OsTskStackInit(curStackSize, (uintptr_t)topStack);

    stackPtr = OsTskContextInit(taskId, curStackSize, topStack, (uintptr_t)OsTskEntry);

    /* init Tcb */
    OsTskCreateTcbInit((uintptr_t)stackPtr, initParam, (uintptr_t)topStack, curStackSize, taskCB);

    /* set task status */
    OsTskCreateTcbStatusSet(taskCB, initParam);
    // 出参ID传出
    *taskPid = taskId;
    NOS_TaskIntRestore(intSave);

    return NOS_OK;
}

/*
 * 描述: 创建一个任务但不进行激活
 */
unsigned int NOS_TaskCreateOnlyInner(unsigned int *taskPid, struct TskInitParam *initParam)
{
    return OsTaskCreateOnly(taskPid, initParam, FALSE);
}
