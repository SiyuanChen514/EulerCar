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
  * @file      nos_base_task.h
  */

/*
 * @defgroup NOS_task 任务基本功能
 * @ingroup NOS_kernel
 */

#ifndef NOS_BASE_TASK_H
#define NOS_BASE_TASK_H

#include "nos_sys_external.h"
#include "nos_typedef.h"
#include "os_task.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/*
 * @ingroup OS_task
 * 空任务ID。
 *
 * 调用NOS_TaskYield时，使用OS_TSK_NULL_ID，由OS选择就绪队列中的第一个任务。
 */
#define OS_TSK_NULL_ID 0xFFFFFFFF

/*
 * @ingroup NOS_task
 * 任务创建参数的结构体定义。
 *
 * 传递任务创建时指定的参数信息。
 */
struct TskInitParam {
    /* 任务入口函数 */
    TskEntryFunc taskEntry;
    /* 任务优先级 */
    unsigned short taskPrio;
    /* 任务参数，最多4个 */
    uintptr_t args[4];
    /* 任务栈的大小 */
    unsigned int stackSize;
    /* 任务名 */
    const char *name;
    /*
     * 本任务的任务栈独立配置起始地址，用户必须对该成员进行初始化，
     * 若配置为0表示从系统内部空间分配，否则用户指定栈起始地址
     */
    uintptr_t stackAddr;
    /* 专属于本任务的私有数据 */
    uintptr_t privateData;
};

/*
 * @ingroup  NOS_task
 * Description: 创建任务。
 *
 * @par 描述
 * 创建一个任务。在系统OS初始化前创建的任务只是简单地加入就绪队列。
 * 系统初始化后创建的任务，如果优先级高于当前任务且未锁任务，则立即发生任务调度并被运行，否则加入就绪队列。
 *
 * @attention
 * <ul>
 * <li>若指定的任务栈独立配置起始地址不为0，则采用用户配置的独立任务栈进行栈空间分配，</li>
 * <li>并且系统会占用(消息队列个数乘以12字节)的空间用于消息队列头。</li>
 * <li>任务创建时，会对之前自删除任务的任务控制块和任务栈进行回收，用户独立配置的任务栈除外。</li>
 * <li>任务名的最大长度为16字节，含结束符。</li>
 * <li>创建任务时需要配置消息队列数。</li>
 * <li>同一核内任务名不允许重复，且不允许和软中断重名。</li>
 * <li>若指定的任务栈大小为0，则使用配置项#OS_TSK_DEFAULT_STACK_SIZE指定的默认的任务栈大小。</li>
 * <li>任务栈的大小必须按16字节大小对齐。确定任务栈大小的原则是，够用就行：多了浪费，少了任务栈溢出。</li>
 * <li>具体多少取决于需要消耗多少的栈内存，视情况而定：函数调用层次越深，栈内存开销越大，</li>
 * <li>局部数组越大，局部变量越多，栈内存开销也越大。</li>
 * <li>用户配置的任务栈首地址需16字节对齐，且配置的任务栈空间中，os会使用消息队列数乘以消息队列控制块的内存大小。</li>
 * <li>用户配置的任务栈空间需由用户保证其合法性，即对可cache空间的地址用户需要保证其任务栈首地址及栈大小cache</li>
 * <li>line对齐，系统不做对齐处理，并在使用后需由用户进行释放。</li>
 * <li>任务创建时，任务创建参数中的任务栈大小配置建议不要使用最小任务栈大小OS_TSK_MIN_STACK_SIZE，</li>
 * <li>该项只是包含任务上下文预留的栈空间，任务调度时额外的任务开销需要由用户自行保证足够的任务栈空间配置。</li>
 * </ul>
 *
 * @param taskPid   [OUT] 类型#unsigned int *，保存任务PID。
 * @param initParam [IN]  类型#struct TskInitParam *，任务创建参数，
 * 其结构体中的成员参数stackAddr传入时必须进行初始化，若不采用用户配置的独立任务栈进行栈空间分配，
 * 该成员必须初始化为0。
 *
 * @retval #OS_ERRNO_TSK_NO_MEMORY              0x02000800，申请内存失败。
 * @retval #OS_ERRNO_TSK_PTR_NULL               0x02000801，指针参数为空。
 * @retval #OS_ERRNO_TSK_STKSZ_NOT_ALIGN        0x02000802，任务栈大小未按16字节大小对齐，HIAVP未按64字节对齐。
 * @retval #OS_ERRNO_TSK_PRIOR_ERROR            0x02000803，任务优先级非法。
 * @retval #OS_ERRNO_TSK_ENTRY_NULL             0x02000804，任务入口函数为空。
 * @retval #OS_ERRNO_TSK_NAME_EMPTY             0x02000805，任务名的指针为空或任务名为空字符串。
 * @retval #OS_ERRNO_TSK_STKSZ_TOO_SMALL        0x02000806，指定的任务栈空间太小。
 * @retval #OS_ERRNO_TSK_ID_INVALID             0x02000807，任务PID非法。
 * @retval #OS_ERRNO_TSK_NOT_SUSPENDED          0x02000809，任务未被挂起。
 * @retval #OS_ERRNO_TSK_FREE_STACK_FAILED      0x02000817，释放任务栈失败。
 * @retval #OS_ERRNO_TSK_TCB_UNAVAILABLE        0x02000811，没有可用的任务控制块资源。
 * @retval #OS_ERRNO_TSK_MSG_Q_TOO_MANY         0x0200081d，任务的MSG队列个数超过15。
 * @retval #OS_ERRNO_HUNT_THREAD_NAME_REPEAT    0x02001f01，创建任务时，线程名重名。
 * @retval #OS_ERRNO_TSK_STACKADDR_NOT_ALIGN    0x02000822，创建任务时，用户配置任务栈地址未16字节对齐。
 * @retval #NOS_OK                              0x00000000，任务创建成功。
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskDeleteInner | NOS_TaskCreateInnerHookAdd | NOS_TaskCreateOnlyInner
 */
extern unsigned int NOS_TaskCreateInner(unsigned int *taskPid, struct TskInitParam *initParam);

/*
 * @ingroup  NOS_task
 * Description: 创建任务，但不激活任务。
 *
 * @par 描述
 * 创建一个任务。该任务不加入就绪队列，只处于挂起状态，用户需要激活该任务需要通过调用NOS_TaskResumeInner函数将其激活。
 *
 * @attention
 * <ul>
 * <li>若指定的任务栈独立配置起始地址不为0，则采用用户配置的独立任务栈进行栈空间分配，并且系统会占用(queNum</li>
 * <li>12字节)的空间用于消息队列头。</li>
 * <li>任务创建时，会对之前自删除任务的任务控制块和任务栈进行回收，用户独立配置的任务栈除外。</li>
 * <li>任务名的最大长度为16字节，含结束符。</li>
 * <li>创建任务时需要配置消息队列数。</li>
 * <li>同一核内任务名不允许重复，且不允许和软中断重名。</li>
 * <li>若指定的任务栈大小为0，则使用配置项#OS_TSK_DEFAULT_STACK_SIZE指定的默认的任务栈大小。</li>
 * <li>任务栈的大小必须按16字节大小对齐。确定任务栈大小的原则是，够用就行：多了浪费，少了任务栈溢出。</li>
 * <li>具体多少取决于需要消耗多少的栈内存，视情况而定：函数调用层次越深，栈内存开销越大，</li>
 * <li>局部数组越大，局部变量越多，栈内存开销也越大。</li>
 * <li>用户配置的任务栈首地址需16字节对齐，且配置的任务栈空间中，os会使用消息队列数*消息队列控制块的内存大小。</li>
 * <li>用户配置的任务栈空间需由用户保证其合法性，即对可cache空间的地址用户需要保证其任务栈首地址及栈大小cache</li>
 * <li>line对齐，系统不做对齐处理，并在使用后需由用户进行释放。</li>
 * <li>任务创建时，任务创建参数中的任务栈大小配置建议不要使用最小任务栈大小OS_TSK_MIN_STACK_SIZE，</li>
 * <li>该项只是包含任务上下文预留的栈空间，任务调度时额外的任务开销需要由用户自行保证足够的任务栈空间配置。</li>
 * </ul>
 *
 * @param taskPid   [OUT] 类型#unsigned int *，保存任务PID。
 * @param initParam [IN]  类型#struct TskInitParam *，任务创建参数，
 * 其结构体中的成员参数stackAddr传入时必须进行初始化，若不采用用户配置的独立任务栈进行栈空间分配，
 * 该成员必须初始化为0。
 *
 * @retval #OS_ERRNO_TSK_NO_MEMORY              0x02000800，申请内存失败。
 * @retval #OS_ERRNO_TSK_PTR_NULL               0x02000801，指针参数为空。
 * @retval #OS_ERRNO_TSK_STKSZ_NOT_ALIGN        0x02000802，任务栈大小未按16字节大小对齐。
 * @retval #OS_ERRNO_TSK_PRIOR_ERROR            0x02000803，任务优先级非法。
 * @retval #OS_ERRNO_TSK_ENTRY_NULL             0x02000804，任务入口函数为空。
 * @retval #OS_ERRNO_TSK_NAME_EMPTY             0x02000805，任务名的指针为空或任务名为空字符串。
 * @retval #OS_ERRNO_TSK_STKSZ_TOO_SMALL        0x02000806，指定的任务栈空间太小。
 * @retval #OS_ERRNO_TSK_ID_INVALID             0x02000807，任务PID非法。
 * @retval #OS_ERRNO_TSK_TCB_UNAVAILABLE        0x02000811，没有可用的任务控制块资源。
 * @retval #OS_ERRNO_TSK_MSG_Q_TOO_MANY         0x0200081d，任务的MSG队列个数超过15。
 * @retval #OS_ERRNO_HUNT_THREAD_NAME_REPEAT    0x02001f01，创建任务时，线程名重名.
 * @retval #OS_ERRNO_TSK_STACKADDR_NOT_ALIGN    0x02000822，创建任务时，用户配置任务栈地址未16字节对齐.
 * @retval #NOS_OK                              0x00000000，任务创建成功。
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskDeleteInner | NOS_TaskCreateInnerHookAdd | NOS_TaskCreateInner
 */
extern unsigned int NOS_TaskCreateOnlyInner(unsigned int *taskPid, struct TskInitParam *initParam);

/*
 * @ingroup  NOS_task
 * Description: 恢复任务。
 *
 * @par 描述
 * 恢复挂起的任务。
 *
 * @attention
 * <ul>
 * <li>在osStart之前不能调用该接口。</li>
 * <li>若任务仍处于延时、阻塞态，则只是取消挂起态，并不加入就绪队列。</li>
 * </ul>
 *
 * @param taskPid [IN]  类型#TskHandle，任务PID。
 *
 * @retval #OS_ERRNO_TSK_ID_INVALID             0x02000807，任务PID非法。
 * @retval #OS_ERRNO_TSK_NOT_CREATED            0x0200080a，任务未创建。
 * @retval #OS_ERRNO_TSK_OPERATE_IDLE           0x02000814，操作IDLE任务。
 * @retval #OS_ERRNO_TSK_NOT_SUSPENDED          0x02000809，任务未被挂起。
 * @retval #NOS_OK                              0x00000000，恢复任务成功。
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskSuspendInner
 */
extern unsigned int NOS_TaskResumeInner(unsigned int taskId);

/*
 * @ingroup  NOS_task
 * Description: 挂起任务。
 *
 * @par 描述
 * 挂起指定的任务，任务将从就绪队列中被删除。
 *
 * @attention
 * <ul>
 * <li>在osStart之前不能调用该接口。</li>
 * <li>若为当前任务且已锁任务，则不能被挂起。</li>
 * <li>IDLE任务不能被挂起。</li>
 * <li>SMP多核并发操作NOS_TaskCoreBind,NOS_TaskDeleteInner,NOS_TaskSuspendInner时避免关中断调用，</li>
 * <li>否则核间中断无法响应造成死循环。</li>
 * </ul>
 *
 * @param taskPid [IN]  类型#TskHandle，任务PID。
 *
 * @retval #OS_ERRNO_TSK_ID_INVALID             0x02000807，任务PID非法。
 * @retval #OS_ERRNO_TSK_ALREADY_SUSPENDED      0x02000808，任务已被挂起。
 * @retval #OS_ERRNO_TSK_NOT_CREATED            0x0200080a，任务未创建。
 * @retval #OS_ERRNO_TSK_OPERATE_IDLE           0x02000814，操作IDLE任务。
 * @retval #OS_ERRNO_TSK_SUSPEND_LOCKED         0x03000815，在锁任务的状态下挂起当前任务。
 * @retval #NOS_OK                              0x00000000，挂起任务成功。
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskResumeInner | NOS_TaskLock
 */
extern unsigned int NOS_TaskSuspendInner(unsigned int taskPid);

/*
 * @ingroup  NOS_task
 * Description: 删除任务。
 *
 * @par 描述
 * 删除指定的任务，释放任务栈和任务控制块资源。
 *
 * @attention
 * <ul>
 * <li>若为自删除，则任务控制块和任务栈在下一次创建任务时才回收。</li>
 * <li>SMP多核并发操作NOS_TaskCoreBind,NOS_TaskDeleteInner,NOS_TaskSuspendInner时避免关中断调用，</li>
 * <li>否则核间中断无法响应造成死循环。</li>
 * <li>对于任务自删除，处理该任务相关的信号量和接收到的消息会强制删除。</li>
 * <li>任务自删除时，删除钩子不允许进行pend信号量、挂起等操作。</li>
 * </ul>
 *
 * @param taskPid [IN]  类型#TskHandle，任务PID。
 *
 * @retval #OS_ERRNO_TSK_ID_INVALID             0x02000807，任务PID非法。
 * @retval #OS_ERRNO_TSK_NOT_CREATED            0x0200080a，任务未创建。
 * @retval #OS_ERRNO_TSK_DELETE_LOCKED          0x0300080b，在锁任务的状态下删除当前任务。
 * @retval #OS_ERRNO_TSK_MSG_NONZERO            0x0200080c，任务待处理的消息数非零。
 * @retval #OS_ERRNO_TSK_HAVE_MUTEX_SEM         0x02000826，任务持有互斥型信号量时删除该任务。
 * @retval #OS_ERRNO_TSK_OPERATE_IDLE           0x02000814，操作IDLE任务。
 * @retval #OS_ERRNO_TSK_QUEUE_DOING            0x02000823，任务正在操作队列。
 * @retval #NOS_OK                              0x00000000，删除任务成功。
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskCreateInner | NOS_TaskDeleteInnerHookAdd
 */
extern unsigned int NOS_TaskDeleteInner(unsigned int taskPid);

/*
 * @ingroup  NOS_task
 * Description: 延迟正在运行的任务。
 *
 * @par 描述
 * 延迟当前运行任务的执行。任务延时等待指定的Tick数后，重新参与调度。
 *
 * @attention
 * <ul>
 * <li>在osStart之前不能调用该接口。</li>
 * <li>硬中断或软中断处理中，或已锁任务，则延时操作失败。</li>
 * <li>若传入参数0，且未锁任务调度，则顺取同优先级队列中的下一个任务执行。如没有同级的就绪任务，</li>
 * <li>则不发生任务调度，继续执行原任务。</li>
 * </ul>
 *
 * @param tick [IN]  类型#U32，延迟的Tick数。
 *
 * @retval #OS_ERRNO_TSK_PRIOR_ERROR            0x02000803，任务优先级非法。
 * @retval #OS_ERRNO_TSK_ID_INVALID             0x02000807，任务PID非法。
 * @retval #OS_ERRNO_TSK_YIELD_INVALID_TASK     0x0200080f，任务ID不在Yield操作指定的优先级队列中。
 * @retval #OS_ERRNO_TSK_DELAY_IN_INT           0x0300080d，在硬中断或软中断的处理中进行延时操作。
 * @retval #OS_ERRNO_TSK_DELAY_IN_LOCK          0x0200080e，在锁任务的状态下进行延时操作。
 * @retval #OS_ERRNO_TSK_YIELD_NOT_ENOUGH_TASK  0x02000810，Yield操作指定的优先级队列中，就绪任务数小于2。
 * @retval #NOS_OK                              0x00000000，任务延时成功。
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskYield
 */
extern unsigned int NOS_TaskDelayInner(unsigned int tick);


extern unsigned int NOS_TaskStartPeriod(unsigned int taskPid, unsigned int timeout);
extern unsigned int NOS_TaskStopPeriod(unsigned int taskPid);
/*
 * @ingroup  NOS_task
 * Description: 锁任务调度。
 *
 * @par 描述
 * 锁任务调度。若任务调度被锁，则不发生任务切换。
 *
 * @attention
 * <ul>
 * <li>只是锁任务调度，并不关中断，因此任务仍可被中断打断。</li>
 * <li>执行此函数则锁计数值加1，解锁则锁计数值减1。因此，必须与#NOS_TaskUnlock配对使用。</li>
 * <li>Cortex-RX SMP下，只会锁当前核的任务调度，其他核的调度不受影响。</li>
 * <li>Cortex-RX SMP下，锁任务期间当前核不保证运行的是Top-N优先级任务，直至解锁后OS重新发起任务调度。</li>
 * </ul>
 *
 * @param 无。
 *
 * @retval 无
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskUnlock
 */
extern void NOS_TaskLock(void);

/*
 * @ingroup  NOS_task
 * Description: 解锁任务调度。
 *
 * @par 描述
 * 任务锁计数值减1。若嵌套加锁，则只有锁计数值减为0才真正的解锁了任务调度。
 *
 * @attention
 * <ul>
 * <li>在osStart之前不能调用该接口。</li>
 * <li>执行此函数则锁计数值加1，解锁则锁计数值减1。因此，必须与#NOS_TaskLock配对使用。</li>
 * </ul>
 *
 * @param 无。
 *
 * @retval 无
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskLock
 */
extern void NOS_TaskUnlock(void);

/*
 * @ingroup  NOS_task
 * Description: 获取当前任务PID。
 *
 * @par 描述
 * 获取处于运行态的任务PID。
 *
 * @attention
 * <ul>
 * <li>硬中断或软中断处理中，也可获取当前任务PID，即被中断打断的任务。</li>
 * <li>在任务切换钩子处理中调用时，获取的是切入任务的ID。</li>
 * </ul>
 *
 * @param taskPid [OUT] 类型#unsigned int *，保存任务PID。
 *
 * @retval #OS_ERRNO_TSK_PTR_NULL               0x02000801，指针参数为空。
 * @retval #OS_ERRNO_TSK_ID_INVALID             0x02000807，任务ID非法。
 * @retval #NOS_OK                              0x00000000，成功。
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskStatusGet | NOS_TaskInfoGet
 */
extern unsigned int NOS_TaskSelf(unsigned int *taskPid);

/*
 * @ingroup  NOS_task
 * Description: 获取私有数据。
 *
 * @par 描述
 * 获取当前任务的私有数据。
 *
 * @attention
 * <ul>
 * <li>当当前任务正在运行时，调用该接口能获取当前任务的私有数据。</li>
 * <li>当任务被中断打断时，中断里调用该接口能获取被中断打断的任务的私有数据。</li>
 * </ul>
 *
 * @param 无。
 *
 * @retval #当前任务的私有数据 任意值，如未设置，则返回的值不确定。
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskPrivateDataSet
 */
extern uintptr_t NOS_TaskPrivateDataGet(void);
extern uintptr_t NOS_TaskPrivateDataGetById(unsigned int taskPid);
/*
 * @ingroup  NOS_task
 * Description: 设置私有数据值。
 *
 * @par 描述
 * 设置当前任务的私有数据值。
 *
 * @attention
 * <ul>
 * <li>只能在任务处理中调用。若在中断中设置，则操作的是刚被打断的任务。</li>
 * </ul>
 *
 * @param privateData [IN]  类型#uintptr_t，数据值。
 *
 * @retval 无
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskPrivateDataGet
 */
extern void NOS_TaskPrivateDataSet(uintptr_t privateData);

/*
 * @ingroup  NOS_task
 * Description: 获取优先级。
 *
 * @par 描述
 * 获取指定任务的优先级。
 *
 * @attention 无
 *
 * @param taskPid  [IN]  类型#TskHandle，任务PID。
 * @param taskPrio [OUT] 类型#unsigned short *，保存任务优先级指针。
 *
 * @retval #OS_ERRNO_TSK_PTR_NULL               0x02000801，指针参数为空。
 * @retval #OS_ERRNO_TSK_ID_INVALID             0x02000807，任务PID非法。
 * @retval #OS_ERRNO_TSK_NOT_CREATED            0x0200080a，任务未创建。
 * @retval #NOS_OK                              0x00000000，成功。
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskPrioritySetInner
 */
extern unsigned int NOS_TaskPriorityGetInner(unsigned int taskPid, unsigned short *taskPrio);

/*
 * @ingroup  NOS_task
 * Description: 设置优先级。
 *
 * @par 描述
 * 设置指定任务的优先级。
 *
 * @attention
 * <ul>
 * <li>在osStart之前不能调用该接口。</li>
 * <li>若设置的优先级高于当前运行的任务，则可能引发任务调度。</li>
 * <li>若调整当前运行任务的优先级，同样可能引发任务调度。</li>
 * <li>若任务发生优先级继承，或者任务阻塞在互斥信号量或优先级唤醒模式的信号量上，不可以设置任务的优先级。</li>
 * </ul>
 *
 * @param taskPid  [IN]  类型#TskHandle，任务PID。
 * @param taskPrio [IN]  类型#TskPrior，任务优先级。
 *
 * @retval #OS_ERRNO_TSK_PRIOR_ERROR            0x02000803，任务优先级非法。
 * @retval #OS_ERRNO_TSK_ID_INVALID             0x02000807，任务PID非法。
 * @retval #OS_ERRNO_TSK_NOT_CREATED            0x0200080a，任务未创建。
 * @retval #OS_ERRNO_TSK_OPERATE_IDLE           0x02000814，操作IDLE任务。
 * @retval #OS_ERRNO_TSK_PRIORITY_INHERIT       0x02000824，任务发生优先级继承。
 * @retval #OS_ERRNO_TSK_PEND_ON_MUTEX          0x02000825，任务阻塞在互斥信号量上。
 * @retval #OS_ERRNO_TSK_PRIOR_LOW_THAN_PENDTSK 0x02000828，设置优先级低于阻塞于它持有的互斥信号量的
 * @retval                                                  最高优先级任务的优先级
 * @retval #NOS_OK                              0x00000000，成功。
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskPriorityGetInner
 */
extern unsigned int NOS_TaskPrioritySetInner(unsigned int taskPid, unsigned short taskPrio);

/*
 * @ingroup  NOS_task
 * Description: 调整指定优先级的任务调度顺序。
 *
 * @par 描述
 * 若nextTask为#OS_TSK_NULL_ID，则优先级队列中的第一个就绪任务调整至队尾,
 * 否则，将nextTask指定的任务调整至优先级队列的队首。
 *
 * @attention
 * <ul>
 * <li>在osStart之前不能调用该接口。</li>
 * <li>前提是指定优先级的就绪队列至少要有两个就绪任务，否则报错返回。</li>
 * <li>Cortex-RX SMP下，只会对本核(本Run Queue)指定优先级的就绪队列进行调整。</li>
 * </ul>
 *
 * @param taskPrio [IN]  类型#TskPrior，任务taskPrio，指定任务优先级队列。
 * @param nextTask [IN]  类型#TskHandle，任务ID或OS_TSK_NULL_ID。
 * @param yieldTo  [OUT] 类型#unsigned int *，保存被调整到队首的任务PID，可为NULL(不需要保存队首任务PID)。
 *
 * @retval #OS_ERRNO_TSK_PRIOR_ERROR            0x02000803，任务优先级非法。
 * @retval #OS_ERRNO_TSK_ID_INVALID             0x02000807，任务PID非法。
 * @retval #OS_ERRNO_TSK_YIELD_INVALID_TASK     0x0200080f，任务PID不在Yield操作指定的优先级队列中。
 * @retval #OS_ERRNO_TSK_YIELD_NOT_ENOUGH_TASK  0x02000810，Yield操作指定的优先级队列中，就绪任务数小于2。
 * @retval #NOS_OK                              0x00000000，成功。
 * @par 依赖
 * <ul><li>nos_task.h：该接口声明所在的头文件。</li></ul>
 * @see NOS_TaskSuspendInner
 */
extern unsigned int NOS_TaskYield(unsigned short taskPrio, unsigned int nextTask, unsigned int *yieldTo);

extern unsigned long long NOS_GetCycle(void);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* NOS_IN_TASK_H */
