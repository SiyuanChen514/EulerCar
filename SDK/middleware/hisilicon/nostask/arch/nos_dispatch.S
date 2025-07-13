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
  * @file      nos_dispatch.S
  */

#include "os_asm_cpu_riscv_external.h"
#include "nos_buildef.h"
#include "nos_306x_adapter.h"

#ifdef __riscv64
#define LREG ld
#define SREG sd
#define FLREG fld
#define FSREG fsd
#define REGBYTES 8
#else
#define LREG lw
#define SREG sw
#define FLREG flw
#define FSREG fsw
#define REGBYTES 4
#endif

#define MSTATUS_MIE  0x00000008
#define MSTATUS_MPIE 0x00000080
#define MSTATUS_MPP_M_MODE       (3U << 11)
#ifdef OS_OPTION_FPU
#define TOTAL_INT_SIZE_ON_STACK  (44 * REGBYTES)
#else
#define TOTAL_INT_SIZE_ON_STACK  (24 * REGBYTES)
#endif

    .global OsTskContextLoad
    .global OsTaskSwitch
    .global NOS_IntUnLock
    .global NOS_IntLock
    .global NOS_IntRestore
    .global OsTaskTrueSwitch
    .global OsHwiPostHandle
    .global NOS_SystickLock
    .global NOS_SystickRestore

    .type OsTskContextLoad, function
    .type OsTaskSwitch, function
    .type NOS_IntUnLock, function
    .type NOS_IntLock, function
    .type NOS_IntRestore, function
    .type OsTaskTrueSwitch, function
    .type OsHwiPostHandle,function
    .type NOS_SystickLock, function
    .type NOS_SystickRestore, function

    .align 2

OsHwiPostHandle:
    lw a0, g_uniFlag
    li a1, OS_FLG_TSK_REQ
    and a0, a0, a1
    beqz a0, OsTskNaroContextLoad

    addi sp, sp, -CALLEE_REG_SIZE
    /* Need reschedule, save callee-save registers */
    sw s0, REG_OFF_S0(sp)
    sw s1, REG_OFF_S1(sp)
    sw s2, REG_OFF_S2(sp)
    sw s3, REG_OFF_S3(sp)
    sw s4, REG_OFF_S4(sp)
    sw s5, REG_OFF_S5(sp)
    sw s6, REG_OFF_S6(sp)
    sw s7, REG_OFF_S7(sp)
    sw s8, REG_OFF_S8(sp)
    sw s9, REG_OFF_S9(sp)
    sw s10, REG_OFF_S10(sp)
    sw s11, REG_OFF_S11(sp)
#ifdef OS_OPTION_FPU
    addi sp, sp, -CALLEE_REG_SIZE
    FSREG fs0, REG_OFF_S0(sp)
    FSREG fs1, REG_OFF_S1(sp)
    FSREG fs2, REG_OFF_S2(sp)
    FSREG fs3, REG_OFF_S3(sp)
    FSREG fs4, REG_OFF_S4(sp)
    FSREG fs5, REG_OFF_S5(sp)
    FSREG fs6, REG_OFF_S6(sp)
    FSREG fs7, REG_OFF_S7(sp)
    FSREG fs8, REG_OFF_S8(sp)
    FSREG fs9, REG_OFF_S9(sp)
    FSREG fs10, REG_OFF_S10(sp)
    FSREG fs11, REG_OFF_S11(sp)
#endif
    lw a0, g_runningTask
    sw sp, 0(a0)

    tail OsMainSchedule


OsTaskTrueSwitch:
    addi  sp, sp, -(TOTAL_INT_SIZE_ON_STACK)
    SREG  a0, 3 * REGBYTES(sp)
    SREG  a1, 4 * REGBYTES(sp)
    csrr  a0, CIPRI
    csrr  a1, PRITHD
    csrw  PRITHD, a0                /* read prithd */
    SREG  a1, 6 * REGBYTES(sp)      /* save prithd */
    csrr  a1, mstatus               /* read mstatus */
    SREG  a1, 7 * REGBYTES(sp)      /* save mstatus */
    csrr  a1, mepc                  /* read mepc */
    addi  a1, a1, 4
    SREG  a1, 8 * REGBYTES(sp)      /* save mepc */
    li    a1, MSTATUS_MPP_M_MODE
    csrs  mstatus, a1

OsTaskTrueSwitchPhase2:
    SREG  t0, 0 * REGBYTES(sp)
    SREG  t1, 1 * REGBYTES(sp)
    SREG  t2, 2 * REGBYTES(sp)
    SREG  a2, 5 * REGBYTES(sp)
    SREG  ra, 9 * REGBYTES(sp)
    SREG  a3, 10 * REGBYTES(sp)
    SREG  a4, 11 * REGBYTES(sp)
    SREG  a5, 12 * REGBYTES(sp)
    SREG  a6, 13 * REGBYTES(sp)
    SREG  a7, 14 * REGBYTES(sp)
    SREG  t3, 15 * REGBYTES(sp)
    SREG  t4, 16 * REGBYTES(sp)
    SREG  t5, 17 * REGBYTES(sp)
    SREG  t6, 18 * REGBYTES(sp)
    csrr  a0, TICK_IRQ_EN_BASE
    andi  a0, a0, TICK_IRQ_EN_NUM
    SREG  a0, 19 * REGBYTES(sp)

#ifdef OS_OPTION_FPU
    FSREG  f0, 23 * REGBYTES(sp)
    FSREG  f1, 24 * REGBYTES(sp)
    FSREG  f2, 25 * REGBYTES(sp)
    FSREG  f3, 26 * REGBYTES(sp)
    FSREG  f4, 27 * REGBYTES(sp)
    FSREG  f5, 28 * REGBYTES(sp)
    FSREG  f6, 29 * REGBYTES(sp)
    FSREG  f7, 30 * REGBYTES(sp)
    FSREG  f10, 31 * REGBYTES(sp)
    FSREG  f11, 32 * REGBYTES(sp)
    FSREG  f12, 33 * REGBYTES(sp)
    FSREG  f13, 34 * REGBYTES(sp)
    FSREG  f14, 35 * REGBYTES(sp)
    FSREG  f15, 36 * REGBYTES(sp)
    FSREG  f16, 37 * REGBYTES(sp)
    FSREG  f17, 38 * REGBYTES(sp)
    FSREG  f28, 39 * REGBYTES(sp)
    FSREG  f29, 40 * REGBYTES(sp)
    FSREG  f30, 41 * REGBYTES(sp)
    FSREG  f31, 42 * REGBYTES(sp)
    frcsr  t0
    SREG   t0, 43 * REGBYTES(sp)
#endif

    addi sp, sp, -CALLEE_REG_SIZE
    sw s0, REG_OFF_S0(sp)
    sw s1, REG_OFF_S1(sp)
    sw s2, REG_OFF_S2(sp)
    sw s3, REG_OFF_S3(sp)
    sw s4, REG_OFF_S4(sp)
    sw s5, REG_OFF_S5(sp)
    sw s6, REG_OFF_S6(sp)
    sw s7, REG_OFF_S7(sp)
    sw s8, REG_OFF_S8(sp)
    sw s9, REG_OFF_S9(sp)
    sw s10, REG_OFF_S10(sp)
    sw s11, REG_OFF_S11(sp)
#ifdef OS_OPTION_FPU
    addi sp, sp, -CALLEE_REG_SIZE
    FSREG fs0, REG_OFF_S0(sp)
    FSREG fs1, REG_OFF_S1(sp)
    FSREG fs2, REG_OFF_S2(sp)
    FSREG fs3, REG_OFF_S3(sp)
    FSREG fs4, REG_OFF_S4(sp)
    FSREG fs5, REG_OFF_S5(sp)
    FSREG fs6, REG_OFF_S6(sp)
    FSREG fs7, REG_OFF_S7(sp)
    FSREG fs8, REG_OFF_S8(sp)
    FSREG fs9, REG_OFF_S9(sp)
    FSREG fs10, REG_OFF_S10(sp)
    FSREG fs11, REG_OFF_S11(sp)
#endif
    lw a0, g_runningTask
    sw sp, 0(a0)
    tail OsMainSchedule

OsTaskSwitch:
    li a0, NOS_TASK_SWITCH_MAGIC_NUM
    ecall
    ret

OsTskContextLoad:
    lw sp, 0(a0)
#ifdef OS_OPTION_FPU
    FLREG fs0, REG_OFF_S0(sp)
    FLREG fs1, REG_OFF_S1(sp)
    FLREG fs2, REG_OFF_S2(sp)
    FLREG fs3, REG_OFF_S3(sp)
    FLREG fs4, REG_OFF_S4(sp)
    FLREG fs5, REG_OFF_S5(sp)
    FLREG fs6, REG_OFF_S6(sp)
    FLREG fs7, REG_OFF_S7(sp)
    FLREG fs8, REG_OFF_S8(sp)
    FLREG fs9, REG_OFF_S9(sp)
    FLREG fs10, REG_OFF_S10(sp)
    FLREG fs11, REG_OFF_S11(sp)
    addi sp, sp, CALLEE_REG_SIZE
#endif

    lw s0, REG_OFF_S0(sp)
    lw s1, REG_OFF_S1(sp)
    lw s2, REG_OFF_S2(sp)
    lw s3, REG_OFF_S3(sp)
    lw s4, REG_OFF_S4(sp)
    lw s5, REG_OFF_S5(sp)
    lw s6, REG_OFF_S6(sp)
    lw s7, REG_OFF_S7(sp)
    lw s8, REG_OFF_S8(sp)
    lw s9, REG_OFF_S9(sp)
    lw s10, REG_OFF_S10(sp)
    lw s11, REG_OFF_S11(sp)
    addi sp, sp, CALLEE_REG_SIZE


OsTskNaroContextLoad:
    LREG  t1, 1 * REGBYTES(sp)
    LREG  t2, 2 * REGBYTES(sp)
    LREG  a2, 5 * REGBYTES(sp)
    LREG  ra, 9 * REGBYTES(sp)
    LREG  a3, 10 * REGBYTES(sp)
    LREG  a4, 11 * REGBYTES(sp)
    LREG  a5, 12 * REGBYTES(sp)
    LREG  a6, 13 * REGBYTES(sp)
    LREG  a7, 14 * REGBYTES(sp)
    LREG  t3, 15 * REGBYTES(sp)
    LREG  t4, 16 * REGBYTES(sp)
    LREG  t5, 17 * REGBYTES(sp)
    LREG  t6, 18 * REGBYTES(sp)

#ifdef OS_OPTION_FPU
    FLREG  f0, 23 * REGBYTES(sp)
    FLREG  f1, 24 * REGBYTES(sp)
    FLREG  f2, 25 * REGBYTES(sp)
    FLREG  f3, 26 * REGBYTES(sp)
    FLREG  f4, 27 * REGBYTES(sp)
    FLREG  f5, 28 * REGBYTES(sp)
    FLREG  f6, 29 * REGBYTES(sp)
    FLREG  f7, 30 * REGBYTES(sp)
    FLREG  f10, 31 * REGBYTES(sp)
    FLREG  f11, 32 * REGBYTES(sp)
    FLREG  f12, 33 * REGBYTES(sp)
    FLREG  f13, 34 * REGBYTES(sp)
    FLREG  f14, 35 * REGBYTES(sp)
    FLREG  f15, 36 * REGBYTES(sp)
    FLREG  f16, 37 * REGBYTES(sp)
    FLREG  f17, 38 * REGBYTES(sp)
    FLREG  f28, 39 * REGBYTES(sp)
    FLREG  f29, 40 * REGBYTES(sp)
    FLREG  f30, 41 * REGBYTES(sp)
    FLREG  f31, 42 * REGBYTES(sp)
    LREG   t0,  43 * REGBYTES(sp)
    fscsr  t0
#endif

    LREG  a0, 7 * REGBYTES(sp)  /* load mstatus */
    LREG  a1, 8 * REGBYTES(sp)  /* load mepc */
restore_mstatus:
    csrw  mstatus, a0 // close Global MIE
    LREG  t0, 19 * REGBYTES(sp)
    andi  t0, t0, TICK_IRQ_EN_NUM
    csrs TICK_IRQ_EN_BASE, t0

restore_mepc:
    LREG  t0, 0 * REGBYTES(sp)
    csrw  mepc, a1
    LREG  a0, 6 * REGBYTES(sp)  /* load prithd */
    csrw  PRITHD, a0
    LREG  a1, 4 * REGBYTES(sp)   /* 2 consecutive csrw instructions will have a bubble */
    LREG  a0, 3 * REGBYTES(sp)
    addi  sp, sp, TOTAL_INT_SIZE_ON_STACK
    mret

NOS_IntUnLock:
    csrrsi a0, mstatus, MSTATUS_MIE
    ret

NOS_IntLock:
    csrrci a0, mstatus, MSTATUS_MIE
    ret

NOS_IntRestore:
    andi a0, a0, MSTATUS_MIE
    csrs mstatus, a0
    ret

NOS_SystickLock:
    csrrci a0, TICK_IRQ_EN_BASE, TICK_IRQ_EN_NUM
    ret

NOS_SystickRestore:
    andi a0, a0, TICK_IRQ_EN_NUM
    csrs TICK_IRQ_EN_BASE, a0
    ret