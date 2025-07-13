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
  * @file      os_cpu_riscv.h
  */

#ifndef OS_CPU_RISCV_H
#define OS_CPU_RISCV_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/*
 * @ingroup NOS_task
 * 任务上下文的结构体定义。
 */
struct TskContext {
    unsigned int fs0;
    unsigned int fs1;
    unsigned int fs2;
    unsigned int fs3;
    unsigned int fs4;
    unsigned int fs5;
    unsigned int fs6;
    unsigned int fs7;
    unsigned int fs8;
    unsigned int fs9;
    unsigned int fs10;
    unsigned int fs11;
    unsigned int s0;
    unsigned int s1;
    unsigned int s2;
    unsigned int s3;
    unsigned int s4;
    unsigned int s5;
    unsigned int s6;
    unsigned int s7;
    unsigned int s8;
    unsigned int s9;
    unsigned int s10;
    unsigned int s11;
    unsigned int t0;
    unsigned int t1;
    unsigned int t2;
    unsigned int a0;
    unsigned int a1;
    unsigned int a2;
    unsigned int prithd;
    unsigned int mstatus;
    unsigned int mepc;
    unsigned int ra;
    unsigned int a3;
    unsigned int a4;
    unsigned int a5;
    unsigned int a6;
    unsigned int a7;
    unsigned int t3;
    unsigned int t4;
    unsigned int t5;
    unsigned int t6;
    unsigned int tickIntNum;
    unsigned int rsvd[3];
    unsigned int f0;
    unsigned int f1;
    unsigned int f2;
    unsigned int f3;
    unsigned int f4;
    unsigned int f5;
    unsigned int f6;
    unsigned int f7;
    unsigned int f10;
    unsigned int f11;
    unsigned int f12;
    unsigned int f13;
    unsigned int f14;
    unsigned int f15;
    unsigned int f16;
    unsigned int f17;
    unsigned int f28;
    unsigned int f29;
    unsigned int f30;
    unsigned int f31;
    unsigned int fcsr;
};

/*
 * @ingroup  NOS_sys
 * Description: 获取当前核ID。
 *
 * @par 描述
 * 获取当前核ID。
 *
 * @attention
 * <ul>
 * <li>获取的核ID为硬件寄存器中的ID号。</li>
 * </ul>
 *
 * @param  无
 *
 * @retval 物理核ID。
 * @par 依赖
 * @see NOS_GetNumberOfCores | NOS_GetPrimaryCore
 */
INLINE unsigned int NOS_GetCoreID(void)
{
    return 0;
}

/*
 * @ingroup  NOS_sys
 * Description: 开中断。
 *
 * @par 描述
 * 开启全局可屏蔽中断。
 *
 * @attention
 * <ul><li>中断服务函数里慎用该接口，会引起中断优先级反转。</li></ul>
 *
 * @param 无
 *
 * @retval 开启全局中断前的中断状态值。
 * @par 依赖
 * @see NOS_IntLock | NOS_IntRestore
 */
extern uintptr_t NOS_IntUnLock(void);

/*
 * @ingroup  NOS_sys
 * Description: 关中断。
 *
 * @par 描述
 * 关闭全局可屏蔽中断。
 *
 * @attention
 * <ul><li>在关全局中断后，禁止调用引起内核调度的相关接口，如NOS_TaskDelayInner接口。</li></ul>
 *
 * @param 无
 *
 * @retval 关闭全局中断前的中断状态值。
 * @par 依赖
 * @see NOS_IntUnLock | NOS_IntRestore
 */
extern uintptr_t NOS_IntLock(void);

/*
 * @ingroup  NOS_sys
 * Description: 恢复中断状态接口。
 *
 * @par 描述
 * 恢复原中断状态寄存器。
 *
 * @attention
 * <ul>
 * <li>该接口必须和关闭全局中断或者是开启全局中断接口成对使用，以关全局中断或者开全局中断操作的返回值为入参</li>
 * <li>以保证中断可以恢复到关全局中断或者开全局中断操作前的状态</li>
 * </ul>
 * @param  intSave [IN]类型#UINTPTR，关全局中断NOS_IntLock和开全局中断NOS_IntUnLock的返回值。
 *
 * @retval 无
 * @par 依赖
 * @see NOS_IntUnLock | NOS_IntLock
 */
extern void NOS_IntRestore(uintptr_t intSave);

extern unsigned int NOS_SystickLock(void);
extern void NOS_SystickRestore(unsigned int intSave);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* OS_CPU_RISCV_H */
