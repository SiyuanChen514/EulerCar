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
  * @file      os_task.h
  */
#ifndef OS_TASK_H
#define OS_TASK_H

#include "nos_buildef.h"
#include "os_typedef.h"
#include "os_cpu_riscv.h"
#include "os_errno.h"
#include "os_module.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/*
 * @ingroup OS_task
 * 任务名的最大长度。
 *
 * 任务名的最大长度，包括结尾符'\0'。
 */
#define OS_TSK_NAME_LEN 16

/*
 * @ingroup OS_task
 * RX、HiTSP支持的优先级为(0~63)，其他平台芯片支持的优先级(0~31)，OS系统IDLE线程使用最低优先级(63或31)，用户不能使用。
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_00 0

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_01 1

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_02 2

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_03 3

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_04 4

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_05 5

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_06 6

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_07 7

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_08 8

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_09 9

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_10 10

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_11 11

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_12 12

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_13 13

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_14 14

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_15 15

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_16 16

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_17 17

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_18 18

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_19 19

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_20 20

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_21 21

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_22 22

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_23 23

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_24 24

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_25 25

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_26 26

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_27 27

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_28 28

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_29 29

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_30 30

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_31 31

/*
 * @ingroup OS_task
 * Cortex-AX支持的优先级为(0~63),下面优先级是32~63。
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_32 32

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_33 33

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_34 34

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_35 35

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_36 36

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_37 37

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_38 38

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_39 39

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_40 40

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_41 41

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_42 42

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_43 43

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_44 44

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_45 45

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_46 46

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_47 47

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_48 48

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_49 49

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_50 50

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_51 51

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_52 52

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_53 53

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_54 54

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_55 55

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_56 56

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_57 57

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_58 58

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_59 59

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_60 60

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_61 61

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_62 62

/*
 * @ingroup OS_task
 * 可用的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_63 63

/*
 * @ingroup OS_task
 * 默认的任务优先级宏定义。
 *
 */
#define OS_TSK_PRIORITY_DEF 32

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务控制块未被使用。
 */
#define OS_TSK_UNUSED 0x0000U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务控制块被使用,任务被创建。
 */
#define OS_TSK_INUSE 0x0001U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务被阻塞(等待VOS消息或事件)
 */
#define OS_TSK_VOS_PEND 0x0002U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务被挂起。
 */
#define OS_TSK_SUSPEND 0x0004U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务被阻塞（等待信号量）。
 */
#define OS_TSK_PEND 0x0008U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务在等待信号量或者事件的标志。
 */
#define OS_TSK_TIMEOUT 0x0010U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务被延时。
 */
#define OS_TSK_DELAY 0x0020U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务已就绪，已加入就绪队列。
 */
#define OS_TSK_READY 0x0040U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务正运行，仍在就绪队列。
 */
#define OS_TSK_RUNNING 0x0080

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务被阻塞（等待快速信号量）。
 */
#define OS_TSK_FSEM_PEND 0x0100U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务被阻塞（等待消息）。
 */
#define OS_TSK_MSG_PEND 0x0200U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务被阻塞（等待核间信号量）。
 */
#define OS_TSK_MCSEM_PEND 0x0400U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * OS_TSK_EVENT_PEND      --- 任务阻塞于等待读事件。
 */
#define OS_TSK_EVENT_PEND 0x0800U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * OS_TSK_EVENT_TYPE    --- 任务读事件类型，0:ANY; 1:ALL。
 */
#define OS_TSK_EVENT_TYPE 0x1000U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * OS_TSK_QUEUE_PEND      --- 任务阻塞与等待队列。
 */
#define OS_TSK_QUEUE_PEND 0x2000U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * OS_TSK_QUEUE_BUSY      --- 队列正在读写数据。
 */
#define OS_TSK_QUEUE_BUSY 0x4000U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * OS_TSK_DELETING       --- 任务正在被删除。
 */
#define OS_TSK_DELETING 0x8000U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务使用coprocessor的矢量寄存器现场已保存
 * 注意第16位已被用作82的矢量寄存器保存标志
 */
#define OS_CP_CONTEXT_SAVED 0x10000U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * OS_TSK_CRG_IDLE_SUSPEND       --- CRG执行队列空闲自动挂起。
 */
#define OS_TSK_CRG_IDLE_SUSPEND 0x20000U

/*
 * @ingroup OS_task
 * 任务或任务控制块状态标志。
 *
 * 任务被延时。
 */
#define OS_TSK_PERIOD 0x40000U

/*
 * 任务模块的错误码定义。
 */
/*
 * @ingroup OS_task
 * 任务错误码：申请内存失败。
 *
 * 值: 0x02000800
 *
 * 解决方案: 分配更大的私有FSC内存分区
 *
 */
#define OS_ERRNO_TSK_NO_MEMORY OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x00)

/*
 * @ingroup OS_task
 * 任务错误码：指针参数为空。
 *
 * 值: 0x02000801
 *
 * 解决方案: 检查参数指针是否为NUL。
 */
#define OS_ERRNO_TSK_PTR_NULL OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x01)

/*
 * @ingroup OS_task
 * 任务错误码：任务栈大小未按16字节大小对齐。
 *
 * 值: 0x02000802
 *
 * 解决方案: 检查入参任务栈大小是否按16字节对齐。
 */
#define OS_ERRNO_TSK_STKSZ_NOT_ALIGN OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x02)

/*
 * @ingroup OS_task
 * 任务错误码：任务优先级非法。
 *
 * 值: 0x02000803
 *
 * 解决方案: 检查入参任务优先级Balong平台不能大于63，其他平台不能大于31。
 */
#define OS_ERRNO_TSK_PRIOR_ERROR OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x03)

/*
 * @ingroup OS_task
 * 任务错误码：任务入口函数为空。
 *
 * 值: 0x02000804
 *
 * 解决方案: 检查入参任务入口函数是否为NULL。
 */
#define OS_ERRNO_TSK_ENTRY_NULL OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x04)

/*
 * @ingroup OS_task
 * 任务错误码：任务名的指针为空或任务名为空字符串。
 *
 * 值: 0x02000805
 *
 * 解决方案: 检查任务名指针和任务名。
 */
#define OS_ERRNO_TSK_NAME_EMPTY OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x05)

/*
 * @ingroup OS_task
 * 任务错误码：指定的任务栈空间太小。
 *
 * 值: 0x02000806
 *
 * 解决方案: 检查任务栈是否小于OS_TSK_MIN_STACK_SIZE。
 */
#define OS_ERRNO_TSK_STKSZ_TOO_SMALL OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x06)

/*
 * @ingroup OS_task
 * 任务错误码：任务ID非法。
 *
 * 值: 0x02000807
 *
 * 解决方案: 检查当前运行任务的PID是否超过任务最大数或检查用户入参任务PID是否合法。
 */
#define OS_ERRNO_TSK_ID_INVALID OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x07)

/*
 * @ingroup OS_task
 * 任务错误码：任务已被挂起。
 *
 * 值: 0x02000808
 *
 * 解决方案: 检查所挂起任务是否为已挂起任务。
 */
#define OS_ERRNO_TSK_ALREADY_SUSPENDED OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x08)

/*
 * @ingroup OS_task
 * 任务错误码：任务未被挂起。
 *
 * 值: 0x02000809
 *
 * 解决方案: 检查所恢复任务是否未挂起。
 */
#define OS_ERRNO_TSK_NOT_SUSPENDED OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x09)

/*
 * @ingroup OS_task
 * 任务错误码：任务未创建。
 *
 * 值: 0x0200080a
 *
 * 解决方案: 检查任务是否创建。
 */
#define OS_ERRNO_TSK_NOT_CREATED OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x0a)

/*
 * @ingroup OS_task
 * 任务错误码：在锁任务的状态下删除当前任务。
 *
 * 值: 0x0300080b
 *
 * 解决方案: 用户确保删除任务时，将任务解锁。
 *
 */
#define OS_ERRNO_TSK_DELETE_LOCKED OS_ERRNO_BUILD_FATAL(OS_MID_TSK, 0x0b)

/*
 * @ingroup OS_task
 * 任务错误码：任务待处理的消息数非零。
 *
 * 值: 0x0200080c
 *
 * 解决方案: 检查所删除任务是否尚有未处理的消息。
 */
#define OS_ERRNO_TSK_MSG_NONZERO OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x0c)

/*
 * @ingroup OS_task
 * 任务错误码：在硬中断或软中断的处理中进行延时操作。
 *
 * 值: 0x0300080d
 *
 * 解决方案: 此操作禁止在中断中进行调度。
 *
 */
#define OS_ERRNO_TSK_DELAY_IN_INT OS_ERRNO_BUILD_FATAL(OS_MID_TSK, 0x0d)

/*
 * @ingroup OS_task
 * 任务错误码：在锁任务的状态下进行延时操作。
 *
 * 值: 0x0200080e
 *
 * 解决方案: 检查是否锁任务。
 */
#define OS_ERRNO_TSK_DELAY_IN_LOCK OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x0e)

/*
 * @ingroup OS_task
 * 任务错误码：任务ID不在Yield操作指定的优先级队列中。
 *
 * 值: 0x0200080f
 *
 * 解决方案: 检查操作的任务的优先级。
 */
#define OS_ERRNO_TSK_YIELD_INVALID_TASK OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x0f)

/*
 * @ingroup OS_task
 * 任务错误码：Yield操作指定的优先级队列中，就绪任务数小于2。
 *
 * 值: 0x02000810
 *
 * 解决方案: 检查指定优先级就绪任务，确保就绪任务数大于1。
 */
#define OS_ERRNO_TSK_YIELD_NOT_ENOUGH_TASK OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x10)

/*
 * @ingroup OS_task
 * 任务错误码：没有可用的任务控制块资源或配置项中任务裁剪关闭。
 *
 * 值: 0x02000811
 *
 * 解决方案: 打开配置项中任务裁剪开关，并配置足够大的任务资源数。
 */
#define OS_ERRNO_TSK_TCB_UNAVAILABLE OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x11)

/*
 * @ingroup OS_task
 * 任务错误码：任务钩子不匹配，即要删除的钩子未注册或已取消。
 *
 * 值: 0x02000812
 *
 * 解决方案: 检查钩子是否已注册。
 */
#define OS_ERRNO_TSK_HOOK_NOT_MATCH OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x12)

/*
 * @ingroup OS_task
 * 任务错误码：操作IDLE任务。
 *
 * 值: 0x02000814
 *
 * 解决方案: 检查是否操作IDLE任务。
 */
#define OS_ERRNO_TSK_OPERATE_IDLE OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x14)

/*
 * @ingroup OS_task
 * 任务错误码：在锁任务的状态下挂起当前任务。
 *
 * 值: 0x03000815
 *
 * 解决方案: 确保任务挂起的时候，任务已经解锁。
 *
 */
#define OS_ERRNO_TSK_SUSPEND_LOCKED OS_ERRNO_BUILD_FATAL(OS_MID_TSK, 0x15)

/*
 * @ingroup OS_task
 * 任务错误码：释放任务栈失败。
 *
 * 值: 0x02000817
 *
 * 解决方案: 检查是否踩内存导致内存块不能释放。
 */
#define OS_ERRNO_TSK_FREE_STACK_FAILED OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x17)

/*
 * @ingroup OS_task
 * 任务错误码：任务栈区间配置太小。
 *
 * 值: 0x02000818
 *
 * 解决方案: 在nos_config.h中配置任务栈大小超过OS_TSK_MIN_STACK_SIZE 。
 */
#define OS_ERRNO_TSK_STKAREA_TOO_SMALL OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x18)

/*
 * @ingroup OS_task
 * 任务错误码：系统初始化任务激活失败。
 *
 * 值: 0x02000819
 *
 * 解决方案: 查看任务栈是否配置错误。
 *
 */
#define OS_ERRNO_TSK_ACTIVE_FAILED OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x19)

/*
 * @ingroup OS_task
 * 任务错误码：配置的任务数量太多，配置的最大任务个数不能大于254减去配置的软中断个数，
 * 总任务个数不包括Idle任务且不能为0。
 *
 * 值: 0x0200081a
 *
 * 解决方案: 在任务配置项中将最大任务数改为小于等于254减去配置的软中断个数且大于0。
 */
#define OS_ERRNO_TSK_MAX_NUM_NOT_SUITED OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x1a)

/*
 * @ingroup OS_task
 * 任务错误码：任务的coprocessor上下文保存区域未按16字节对齐。
 *
 * 值: 0x0200081b
 *
 * 解决方案: 检查保存区起始地址是否16字节对齐。
 */
#define OS_ERRNO_TSK_CP_SAVE_AREA_NOT_ALIGN OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x1b)

/*
 * @ingroup OS_task
 * 任务错误码：任务的MSG队列个数超过15。
 *
 * 值: 0x0200081d
 *
 * 解决方案: 确认任务创建的消息队列数不超过15。
 */
#define OS_ERRNO_TSK_MSG_Q_TOO_MANY OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x1d)

/*
 * @ingroup OS_task
 * 任务错误码：任务的coprocessor上下文保存区域的地址为空指针。
 *
 * 值: 0x0200081e
 *
 * 解决方案: 检查保存区起始地址是否为NULL。
 */
#define OS_ERRNO_TSK_CP_SAVE_AREA_NULL OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x1e)

/*
 * @ingroup OS_task
 * 任务错误码：任务自删除时释放未接收消息的内存失败。
 *
 * 值: 0x0200081f
 *
 * 解决方案: 无。
 */
#define OS_ERRNO_TSK_SELF_DELETE_ERR OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x1f)

/*
 * @ingroup OS_task
 * 任务错误码：获取任务信息时，用户实际欲获取任务数为0。
 *
 * 值: 0x02000821
 *
 * 解决方案: 获取任务信息时，用户实际输入的欲获取任务数不为0。
 */
#define OS_ERRNO_TSK_INPUT_NUM_ERROR OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x21)

/*
 * @ingroup OS_task
 * 任务错误码：用户配置的任务栈首地址未16字节对齐。
 *
 * 值: 0x02000822
 *
 * 解决方案: 配置进来任务栈首地址需16字节对齐。
 */
#define OS_ERRNO_TSK_STACKADDR_NOT_ALIGN OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x22)

/*
 * @ingroup OS_task
 * 任务错误码：任务正在操作队列。
 *
 * 值: 0x02000823
 *
 * 解决方案: 让被删除的任务得到调度读取完队列数据，即可删除。
 */
#define OS_ERRNO_TSK_QUEUE_DOING OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x23)

/*
 * @ingroup OS_task
 * 任务错误码：任务发生优先级继承。
 *
 * 值: 0x02000824
 *
 * 解决方案: 等待任务恢复优先级后再尝试设置任务的优先级。
 */
#define OS_ERRNO_TSK_PRIORITY_INHERIT OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x24)

/*
 * @ingroup OS_task
 * 任务错误码：任务阻塞在互斥信号量上。
 *
 * 值: 0x02000825
 *
 * 解决方案: 等待任务恢复调度后再尝试设置任务的优先级。
 */
#define OS_ERRNO_TSK_PEND_ON_MUTEX OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x25)

/*
 * @ingroup OS_task
 * 任务错误码：任务删除时持有互斥信号量。
 *
 * 值: 0x02000826
 *
 * 解决方案: 删除任务时必须将其持有的互斥信号量释放。
 */
#define OS_ERRNO_TSK_HAVE_MUTEX_SEM OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x26)

/*
 * @ingroup OS_task
 * 任务错误码：任务退出时没有完全释放资源。
 *
 * 值: 0x03000827
 *
 * 解决方案: 任务退出前确保完全释放其占有的资源(如消息，互斥信号量等)。
 *
 */
#define OS_ERRNO_TSK_EXIT_WITH_RESOURCE OS_ERRNO_BUILD_FATAL(OS_MID_TSK, 0x27)

/*
 * @ingroup OS_task
 * 任务错误码：任务设置优先级时低于阻塞在它持有的互斥信号量的最高优先级任务的优先级。
 *
 * 值: 0x02000828
 *
 * 解决方案: 重设优先级时不能低于阻塞在目标任务持有的互斥信号量的最高优先级任务的优先级。
 *
 */
#define OS_ERRNO_TSK_PRIOR_LOW_THAN_PENDTSK OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x28)

/*
 * @ingroup OS_task
 * 任务错误码：不能在当前正在运行的任务中获取当前任务的上下文信息。
 *
 * 值: 0x02000829
 *
 * 解决方案: 需要在中断中获取任务的上下文信息或者当前运行任务中获取其他任务的任务上下文信息。
 *
 */
#define OS_ERRNO_TSK_CONTEXT_NOT_GETED OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x29)

/*
 * @ingroup OS_task
 * 任务错误码：入参为NULL。
 *
 * 值: 0x0200082a
 *
 * 解决方案: 检查入参是否不为NULL。
 */
#define OS_ERRNO_TSK_PARA_NULL OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x2a)

/*
 * @ingroup OS_task
 * 任务错误码：SMP绑定核的核掩码非法。
 *
 * 值: 0x0200082b
 *
 * 解决方案: 检查有效的核掩码是否为0。
 */
#define OS_ERRNO_TSK_BIND_CORE_INVALID OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x2b)

/*
 * @ingroup OS_task
 * 任务错误码：任务阻塞在优先级唤醒方式的信号量上。
 *
 * 值: 0x0200082c
 *
 * 解决方案: 等待任务恢复调度后再尝试设置任务的优先级。
 */
#define OS_ERRNO_TSK_PEND_ON_PRIOR OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x2c)

/*
 * @ingroup OS_task
 * 任务错误码：查询目标核运行任务时指定的核号非法。
 *
 * 值: 0x0200082d
 *
 * 解决方案: 传入正确的核号再查询当前运行任务ID。
 */
#define OS_ERRNO_TSK_GET_CURRENT_COREID_INVALID OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x2d)

/*
 * @ingroup OS_task
 * 任务错误码：查询目标核还未开始运行任务。
 *
 * 值: 0x0200082e
 *
 * 解决方案: 目标核开始运行任务后即可获取当前运行任务。
 */
#define OS_ERRNO_TSK_DESTCORE_NOT_RUNNING OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x2e)

/*
 * @ingroup OS_task
 * 任务错误码：指定的任务栈空间太小。
 *
 * 值: 0x0200082f
 *
 * 解决方案: 检查任务栈是否小于OS_TSK_MIN_STACK_SIZE+多消息头。
 */
#define OS_ERRNO_TSK_USR_STKSZ_TOO_SMALL OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x2f)

/*
 * @ingroup OS_task
 * 任务错误码：调度器trace的buffer内存申请失败。
 *
 * 值: 0x02000840
 *
 * 解决方案: 放大0号分区的内存空间大小。
 */
#define OS_ERRNO_TSK_SCHED_TRACE_NO_MEMORY OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x40)

/*
 * @ingroup OS_task
 * 任务错误码：锁任务调度的情况下绑定自身至其他核。
 *
 * 值: 0x0200084c
 *
 * 解决方案: 锁任务调度的情况下不要将自身绑定至其他核。
 */
#define OS_ERRNO_TSK_BIND_SELF_WITH_TASKLOCK OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x4c)

/*
 * @ingroup OS_task
 * 任务错误码：任务在关键不可打断的操作中(如绑核，挂起，删任务，被删)，不可以再对任务做这些操作。
 *
 * 值: 0x0200084e
 *
 * 解决方案: 分析系统行为。
 */
#define OS_ERRNO_TSK_OPERATION_BUSY OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x4e)

/*
 * @ingroup OS_task
 * 任务错误码：内核启动过程中内核进程的所有线程(任务、软中断)数和用户进程数之和超过了255个
 *
 * 值: 0x0200084f
 *
 * 解决方案: 检查内核OS模块注册过程中所有线程(任务、软中断)数和用户进程数配置之和不超过255个。
 */
#define OS_ERRNO_TSK_LTID_OVERFLOW OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x4f)

/*
 * @ingroup OS_task
 * 任务错误码：创建线程时没有可使用的GTID资源分配。
 *
 * 值: 0x02000850
 *
 * 解决方案: 检查当前创建的线程过多，分配资源数不够。
 */
#define OS_ERRNO_TSK_NO_GTID_ALLOC_RESOURCE OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x50)

/*
 * @ingroup OS_task
 * 任务错误码：进程中删除线程时线程TID对应的GTID资源不存在。
 *
 * 值: 0x02000851
 *
 * 解决方案: 检查当前进程中的删除的线程TID是否存在。
 */
#define OS_ERRNO_TSK_NO_GTID_RESOURCE_DEL OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x51)

/*
 * @ingroup OS_task
 * 任务错误码：没有任务调度请求
 *
 * 值: 0x02000852
 *
 * 解决方案: 检查进程任务idle下是否存在其他用户进程切换。
 */
#define OS_ERRNO_TSK_NO_SCHEDULE_REQ OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x52)

/*
 * @ingroup OS_task
 * 任务错误码：在中断上下文绑定迁移任务。
 *
 * 值: 0x02000853
 *
 * 解决方案: 禁止在中断上下文绑定任务，中断互绑对方运行任务时可能会死循环。
 */
#define OS_ERRNO_TSK_BIND_IN_HWI OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x53)

/*
 * @ingroup OS_task
 * 任务错误码：解锁任务之前并未上锁。
 *
 * 值: 0x02000854
 *
 * 解决方案: 任务上锁解锁必须配对使用，不能不加锁，直接解锁，可能导致误解锁。
 */
#define OS_ERRNO_TSK_UNLOCK_NO_LOCK OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x54)

/*
 * @ingroup OS_task
 * 任务错误码：不支持任务删除。
 *
 * 值: 0x03000855
 *
 * 解决方案: 确保任务删除功能配置为YES。
 */
#define OS_ERRNO_TSK_DEL_NOT_SUPPORT OS_ERRNO_BUILD_FATAL(OS_MID_TSK, 0x55)

/*
 * @ingroup OS_task
 * 任务错误码：指定的任务栈size太大导致内存分配整形溢出。
 *
 * 值: 0x02000856
 *
 * 解决方案: 限制任务栈大小，确保内存大小不大于0xFFFFFFFF。
 */
#define OS_ERRNO_TSK_STKSZ_TOO_BIG OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x56)

/*
 * @ingroup OS_task
 * 任务错误码：指定的任务栈地址太大导致任务栈初始化的时候整形溢出。
 *
 * 值: 0x02000857
 *
 * 解决方案: 限制任务栈地址大小，确保任务栈初始化地址不大于0xFFFFFFFF。
 */
#define OS_ERRNO_TSK_STACKADDR_TOO_BIG OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x57)
/*
 * @ingroup OS_task
 * 任务错误码：配置的任务数量太多，最多127个。
 *
 * 值: 0x02000858
 *
 * 解决方案: 确认任务创建的消息队列数不超过15。
 */
#define OS_ERRNO_TSK_CONFIG_TOO_MANY OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x58)
/*
 * @ingroup OS_task
 * 任务错误码：任务钩子已达到最大允许的注册数。
 *
 * 值: 0x02000859
 *
 * 解决方案: 无
 */
#define OS_ERRNO_TSK_HOOK_IS_FULL OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x59)

/*
 * @ingroup OS_task
 * 任务错误码：任务动态私有数据参数指针为空。
 *
 * 值: 0x0200085a
 *
 * 解决方案: 排查接口参数是否合法。
 */
#define OS_ERRNO_TSK_SPEC_DATA_PARA_NULL              OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x5a)

/*
 * @ingroup OS_task
 * 任务错误码：任务动态私有数据创建满，当前最多支持16个。
 *
 * 值: 0x0200085b
 *
 * 解决方案: 排查是否有需要删除的动态私有数据并删除。
 */
#define OS_ERRNO_TSK_SPEC_DATA_CREATE_FULL            OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x5b)

/*
 * @ingroup OS_task
 * 任务错误码：任务动态私有数据参数句柄非法。
 *
 * 值: 0x0200085c
 *
 * 解决方案: 排查接口中传入的句柄参数是否合法。
 */
#define OS_ERRNO_TSK_SPEC_DATA_HANDLE_OVERFLOW        OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x5c)

/*
 * @ingroup OS_task
 * 任务错误码：任务动态私有数据参数句柄尚未创建。
 *
 * 值: 0x0200085d
 *
 * 解决方案: 排查接口中传入的句柄参数是否合法，必须先创建再使用。
 */
#define OS_ERRNO_TSK_SPEC_DATA_HANDLE_UNCREATE        OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x5d)

/*
 * @ingroup OS_task
 * 任务错误码：任务动态私有数据初始化时没有可用内存。
 *
 * 值: 0x0200085e
 *
 * 解决方案: 扩大默认内存分区大小。
 */
#define OS_ERRNO_TSK_SPEC_DATA_NO_MOMORY              OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x5e)

/*
 * @ingroup OS_task
 * 任务错误码：任务动态私有数据没有先设置再获取值。
 *
 * 值: 0x0200085f
 *
 * 解决方案: 必须先对任务动态私有数据进行设置后使用。
 */
#define OS_ERRNO_TSK_SPEC_DATA_HANDLE_UNSET           OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x5f)

/*
 * @ingroup OS_task
 * 任务错误码：任务动态私有数据在经过重新创建后没有先设置再获取值。
 *
 * 值: 0x02000860
 *
 * 解决方案: 必须先对任务动态私有数据进行设置后使用。
 */
#define OS_ERRNO_TSK_SPEC_DATA_HANDLE_RECREATE_UNSET  OS_ERRNO_BUILD_ERROR(OS_MID_TSK, 0x60)

/*
 * @ingroup OS_task
 * 任务切换类型。
 *
 * 任务快速切换。
 */
#define OS_TSK_FAST_TRAP 0x1

/*
 * @ingroup OS_task
 * 任务切换类型。
 *
 * 任务普通切换。
 */
#define OS_TSK_NORMAL_TRAP 0x0

/*
 * *@ingroup OS_task
 * 任务信息结构体
 */
struct TskInfo {
    /* 任务切换时的SP */
    uintptr_t sp;
    /* 任务切换时的PC */
    uintptr_t pc;
    /* 任务状态 */
    unsigned short taskStatus;
    /* 任务优先级 */
    unsigned short taskPrio;
    /* 任务栈的大小 */
    unsigned int stackSize;
    /* 任务栈的栈顶 */
    uintptr_t topOfStack;
    /* 任务接收到的消息数 */
    unsigned int numOfMsg;
    /* 任务名 */
    char name[OS_TSK_NAME_LEN];
    /* 任务当前核 */
    unsigned int core;
    /* 任务入口函数 */
    void *entry;
    /* 任务控制块地址 */
    void *tcbAddr;
    /* 栈底 */
    uintptr_t bottom;
    /* 栈当前使用的大小 */
    unsigned int currUsed;
    /* 栈使用的历史峰值 */
    unsigned int peakUsed;
    /* 栈是否溢出 */
    bool ovf;
    /* 任务上下文 */
    struct TskContext context;
};

/*
 * @ingroup  OS_task
 * Description: 任务入口函数类型定义。
 *
 * @par 描述
 * 用户通过任务入口函数类型定义任务入口函数，在任务创建触发之后调用该函数进行任务执行。
 * @attention 无。
 *
 * @param param1 [IN]  类型#uintptr_t，传递给任务处理函数的第一个参数。
 * @param param2 [IN]  类型#uintptr_t，传递给任务处理函数的第二个参数。
 * @param param3 [IN]  类型#uintptr_t，传递给任务处理函数的第三个参数。
 * @param param4 [IN]  类型#uintptr_t，传递给任务处理函数的第四个参数。
 *
 * @retval 无。
 * @par 依赖
 * <ul><li>os_task.h：该接口声明所在的头文件。</li></ul>
 * @see 无。
 */
typedef void (*TskEntryFunc)(uintptr_t param1, uintptr_t param2, uintptr_t param3, uintptr_t param4);

/*
 * @ingroup  os_task
 * Description: 任务创建钩子函数类型定义。
 *
 * @par 描述
 * 用户通过任务创建钩子函数类型定义任务创建钩子，在系统创建任务时，调用该钩子。
 * @attention 无。
 *
 * @param taskPid [IN]  类型#TskHandle，新创建任务的PID。
 *
 * @retval 无。
 * @par 依赖
 * <ul><li>os_task.h：该接口声明所在的头文件。</li></ul>
 * @see 无。
 */
typedef unsigned int(*TskCreateHook)(unsigned int taskPid);

/*
 * @ingroup  os_task
 * Description: 任务删除钩子函数类型定义。
 *
 * @par 描述
 * 用户通过任务删除钩子函数类型定义任务删除钩子，在系统对任务进行删除时，调用该钩子。
 * @attention 无。
 *
 * @param taskPid [IN]  类型#TskHandle，删除任务的PID。
 *
 * @retval 无。
 * @par 依赖
 * <ul><li>os_task.h：该接口声明所在的头文件。</li></ul>
 * @see 无。
 */
typedef unsigned int(*TskDeleteHook)(unsigned int taskPid);

/*
 * @ingroup  os_task
 * Description: 任务切换钩子函数类型定义。
 *
 * @par 描述
 * 用户通过任务切换钩子函数类型定义任务切换钩子，在系统对任务进行切换时，调用该钩子。
 * @attention 无。
 *
 * @param lastTaskPid [IN]  类型#TskHandle，上一个任务的PID。
 * @param nextTaskPid [IN]  类型#TskHandle，下一个任务的PID。
 *
 * @retval 无。
 * @par 依赖
 * <ul><li>os_task.h：该接口声明所在的头文件。</li></ul>
 * @see 无。
 */
typedef unsigned int(*TskSwitchHook)(unsigned int lastTaskPid, unsigned int nextTaskPid);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* OS_TASK_H */
