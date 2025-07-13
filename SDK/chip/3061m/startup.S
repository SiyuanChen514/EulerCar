/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file    startup.S
  * @author  MCU Application Driver Team
  * @brief   RISC-V trap handling and startup code
  */

#ifndef ENTRY_S
#define ENTRY_S

#include "feature.h"

.extern __stack_top
.extern __irq_stack_top
.extern SysErrNmiEntry
.extern SysErrExcEntry
.extern trap_entry
#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
.extern g_RiscvPrivMode
#endif

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

#define NESTED_IRQ_SUPPORT
#define COMPILE_LDM         /**< Support stmia and ldmia instruction */

#ifdef FLOAT_SUPPORT
#define TOTAL_INT_SIZE_ON_STACK  (40 * REGBYTES)
#else
#define TOTAL_INT_SIZE_ON_STACK  (20 * REGBYTES)
#endif

#define SYSERR_INT_SIZE_ON_STACK  (28 * REGBYTES)

#define MSTATUS_MPP_MACHINE       0x00001800
#define MCAUSE_ECALL_FROM_MMODE  11
#define MCAUSE_ECALL_FROM_UMODE  8
#define EXC_SIZE_ON_STACK         (160)

#define MSTATUS_MIE  0x00000008
#define MSTATUS_MPIE 0x00000080
#define MCAUSE_MASK_INT_BIT       0x80000000
#define MCAUSE_MASK_INT_NUM       0x000000FF

#define locipri0                0xBC0
#define locipri1                0xBC1
#define locipri2                0xBC2
#define locipri3                0xBC3
#define locipri4                0xBC4
#define locipri5                0xBC5
#define locipri6                0xBC6
#define locipri7                0xBC7
#define locipri8                0xBC8
#define locipri9                0xBC9
#define locipri10               0xBCA
#define locipri11               0xBCB
#define locipri12               0xBCC
#define locipri13               0xBCD
#define locipri14               0xBCE
#define locipri15               0xBCF

#define EFC_BASE_ADDR          0x14710000   /* efc base address */
#define EFC_MAGIC_LOCK_RW      0x14710200   /* cmd operation magic word protection register */
#define EFC_MAGIC_NUMBER       0xFEDCBA98   /* magic number */
#define SYSRAM_ERROR           0x10108300
#define SC_SYS_STAT_ADDR       0x10100018  /**< System state register address */
#define TIMER0_CONTROL         0x14300008
#define TIMER0_INTENABLE       (1 << 5)
#define UART0_BASE_ADDR        0x14000000
#define IBRD_OFFSET            0x24
#define FBRD_OFFSET            0x28
#define LCR_H_OFFSET           0x2C
#define CR_OFFSET              0x30
#define DMACR_OFFSET           0x48

.equ cipri,  0x7ED
.equ prithd, 0xBFE

    .section      .data.magic
    .word         0xA37E95BD   /* eflash magic number, bootrom will check it */

    .section      .text.entry
    .global _start
    .option norvc
_start:
    j handle_reset

.macro push_reg
    addi  sp, sp, -(TOTAL_INT_SIZE_ON_STACK)
#ifdef  COMPILE_LDM
    stmia {ra, t0-t2, a0-a7, t3-t6}, (sp)
#else
    SREG  ra, 0 * REGBYTES(sp)
    SREG  t0, 1 * REGBYTES(sp)
    SREG  t1, 2 * REGBYTES(sp)
    SREG  t2, 3 * REGBYTES(sp)
    SREG  a0, 4 * REGBYTES(sp)
    SREG  a1, 5 * REGBYTES(sp)
    SREG  a2, 6 * REGBYTES(sp)
    SREG  a3, 7 * REGBYTES(sp)
    SREG  a4, 8 * REGBYTES(sp)
    SREG  a5, 9 * REGBYTES(sp)
    SREG  a6, 10 * REGBYTES(sp)
    SREG  a7, 11 * REGBYTES(sp)
    SREG  t3, 12 * REGBYTES(sp)
    SREG  t4, 13 * REGBYTES(sp)
    SREG  t5, 14 * REGBYTES(sp)
    SREG  t6, 15 * REGBYTES(sp)
#endif
    addi  sp, sp, -(TOTAL_INT_SIZE_ON_STACK)
.endm

.macro pop_reg
    addi  sp, sp, TOTAL_INT_SIZE_ON_STACK
#ifdef  COMPILE_LDM
    ldmia {ra, t0-t2, a0-a7, t3-t6},(sp)
#else
    LREG  ra, 0 * REGBYTES(sp)
    LREG  t0, 1 * REGBYTES(sp)
    LREG  t1, 2 * REGBYTES(sp)
    LREG  t2, 3 * REGBYTES(sp)
    LREG  a0, 4 * REGBYTES(sp)
    LREG  a1, 5 * REGBYTES(sp)
    LREG  a2, 6 * REGBYTES(sp)
    LREG  a3, 7 * REGBYTES(sp)
    LREG  a4, 8 * REGBYTES(sp)
    LREG  a5, 9 * REGBYTES(sp)
    LREG  a6, 10 * REGBYTES(sp)
    LREG  a7, 11 * REGBYTES(sp)
    LREG  t3, 12 * REGBYTES(sp)
    LREG  t4, 13 * REGBYTES(sp)
    LREG  t5, 14 * REGBYTES(sp)
    LREG  t6, 15 * REGBYTES(sp)
#endif
    addi  sp, sp, TOTAL_INT_SIZE_ON_STACK
.endm

.macro SAVE_SYSERR_REGS
    addi sp,sp,-(SYSERR_INT_SIZE_ON_STACK)
    SREG  s0, 16 * REGBYTES(sp)
    SREG  s1, 17 * REGBYTES(sp)
    SREG  s2, 18 * REGBYTES(sp)
    SREG  s3, 19 * REGBYTES(sp)
    SREG  s4, 20 * REGBYTES(sp)
    SREG  s5, 21 * REGBYTES(sp)
    SREG  s6, 22 * REGBYTES(sp)
    SREG  s7, 23 * REGBYTES(sp)
    SREG  s8, 24 * REGBYTES(sp)
    SREG  s9, 25 * REGBYTES(sp)
    SREG  s10, 26 * REGBYTES(sp)
    SREG  s11, 27 * REGBYTES(sp)

    addi  a1, sp, (TOTAL_INT_SIZE_ON_STACK + SYSERR_INT_SIZE_ON_STACK)
    SREG  a1, 28 * REGBYTES(sp)           /* save original sp */

    SREG  gp, 29 * REGBYTES(sp)
    SREG  tp, 30 * REGBYTES(sp)

    csrr a0, mepc
    csrr a1, mstatus
    csrr a2, mtval
    csrr a3, mcause
    # csrr a4, ccause

    SREG a0, 31 * REGBYTES(sp)   /* mepc */
    SREG a1, 32 * REGBYTES(sp)  /* mstatus */
    SREG a2, 33 * REGBYTES(sp)  /* mtval */
    SREG a3, 34 * REGBYTES(sp)  /* mcause */
    # SREG a4, 35 * REGBYTES(sp)  /* ccause */
    mv a0,sp
.endm

/* The interrupt vector table must be aligned with 4 bytes */
.align 2
TrapHandler:
    j     TrapVector        /* trap and INT 0 */
    j     TrapVector        /* INT 1 */
    j     TrapVector        /* INT 2 */
    j     TrapVector        /* INT 3 */
    j     TrapVector        /* INT 4 */
    j     TrapVector        /* INT 5 */
    j     TrapVector        /* INT 6 */
    j     TrapVector        /* INT 7 */
    j     TrapVector        /* INT 8 */
    j     TrapVector        /* INT 9 */
    j     TrapVector        /* INT 10 */
    j     TrapVector        /* INT 11 */
    j     TrapVector        /* INT 12 */
    j     TrapVector        /* INT 13 */
    j     TrapVector        /* INT 14 */
    j     TrapVector        /* INT 15 */
    j     TrapVector        /* INT 16 */
    j     TrapVector        /* INT 17 */
    j     TrapVector        /* INT 18 */
    j     TrapVector        /* INT 19 */
    j     TrapVector        /* INT 20 */
    j     TrapVector        /* INT 21 */
    j     TrapVector        /* INT 22 */
    j     TrapVector        /* INT 23 */
    j     TrapVector        /* INT 24 */
    j     TrapVector        /* INT 25 */

    j     IntHandler        /* INT 26 */
    j     IntHandler        /* INT 27 */
    j     IntHandler        /* INT 28 */
    j     IntHandler        /* INT 29 */
    j     IntHandler        /* INT 30 */
    j     IntHandler        /* INT 31 */
    j     IntHandler        /* INT 32 */
    j     IntHandler        /* INT 33 */
    j     IntHandler        /* INT 34 */
    j     IntHandler        /* INT 35 */
    j     IntHandler        /* INT 36 */
    j     IntHandler        /* INT 37 */
    j     IntHandler        /* INT 38 */
    j     IntHandler        /* INT 39 */
    j     IntHandler        /* INT 40 */
    j     IntHandler        /* INT 41 */
    j     IntHandler        /* INT 42 */
    j     IntHandler        /* INT 43 */
    j     IntHandler        /* INT 44 */
    j     IntHandler        /* INT 45 */
    j     IntHandler        /* INT 46 */
    j     IntHandler        /* INT 47 */
    j     IntHandler        /* INT 48 */
    j     IntHandler        /* INT 49 */
    j     IntHandler        /* INT 50 */
    j     IntHandler        /* INT 51 */
    j     IntHandler        /* INT 52 */
    j     IntHandler        /* INT 53 */
    j     IntHandler        /* INT 54 */
    j     IntHandler        /* INT 55 */
    j     IntHandler        /* INT 56 */
    j     IntHandler        /* INT 57 */
    j     IntHandler        /* INT 58 */
    j     IntHandler        /* INT 59 */
    j     IntHandler        /* INT 60 */
    j     IntHandler        /* INT 61 */
    j     IntHandler        /* INT 62 */
    j     IntHandler        /* INT 63 */
    j     IntHandler        /* INT 64 */
    j     IntHandler        /* INT 65 */
    j     IntHandler        /* INT 66 */
    j     IntHandler        /* INT 67 */
    j     IntHandler        /* INT 68 */
    j     IntHandler        /* INT 69 */
    j     IntHandler        /* INT 70 */
    j     IntHandler        /* INT 71 */
    j     IntHandler        /* INT 72 */
    j     IntHandler        /* INT 73 */
    j     IntHandler        /* INT 74 */
    j     IntHandler        /* INT 75 */
    j     IntHandler        /* INT 76 */
    j     IntHandler        /* INT 77 */
    j     IntHandler        /* INT 78 */
    j     IntHandler        /* INT 79 */
    j     IntHandler        /* INT 80 */
    j     IntHandler        /* INT 81 */
    j     IntHandler        /* INT 82 */
    j     IntHandler        /* INT 83 */
    j     IntHandler        /* INT 84 */
    j     IntHandler        /* INT 85 */
    j     IntHandler        /* INT 86 */
    j     IntHandler        /* INT 87 */
    j     IntHandler        /* INT 88 */
    j     IntHandler        /* INT 89 */
    j     IntHandler        /* INT 90 */
    j     IntHandler        /* INT 91 */
    j     IntHandler        /* INT 92 */
    j     IntHandler        /* INT 93 */
    j     IntHandler        /* INT 94 */
    j     IntHandler        /* INT 95 */
    j     IntHandler        /* INT 96 */
    j     IntHandler        /* INT 97 */
    j     IntHandler        /* INT 98 */
    j     IntHandler        /* INT 99 */
    j     IntHandler        /* INT 100 */
    j     IntHandler        /* INT 101 */
    j     IntHandler        /* INT 102 */
    j     IntHandler        /* INT 103 */
    j     IntHandler        /* INT 104 */
    j     IntHandler        /* INT 105 */
    j     IntHandler        /* INT 106 */
    j     IntHandler        /* INT 107 */
    j     IntHandler        /* INT 108 */
    j     IntHandler        /* INT 109 */
    j     IntHandler        /* INT 110 */
    j     IntHandler        /* INT 111 */
    j     IntHandler        /* INT 112 */
    j     IntHandler        /* INT 113 */
    j     IntHandler        /* INT 114 */
    j     IntHandler        /* INT 115 */
    j     IntHandler        /* INT 116 */
    j     IntHandler        /* INT 117 */
    j     IntHandler        /* INT 118 */
    j     IntHandler        /* INT 119 */
    j     IntHandler        /* INT 120 */
    j     IntHandler        /* INT 121 */

.align 2
NmiEntry:
    SAVE_SYSERR_REGS
    call SysErrNmiEntry
deadLoop1:
    tail deadLoop1
    nop

.align 2
TrapEntry:
    SAVE_SYSERR_REGS
    /* Exception run with interrupts masked */
    csrc mstatus, MSTATUS_MIE
    call SysErrExcEntry
deadLoop2:
    tail deadLoop2

.align 2
IntHandler:
    addi  sp, sp, -(TOTAL_INT_SIZE_ON_STACK)

    SREG  a0, 3 * REGBYTES(sp)
    SREG  a1, 4 * REGBYTES(sp)

#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
    la    a0, g_RiscvPrivMode
    lw    a1, (a0)
    addi  a1, a1, 1
    sw    a1, (a0)
#endif

#if defined(HARD_NESTED_IRQ_SUPPORT) && (HARD_NESTED_IRQ_SUPPORT == 1)
    csrr  a0, mcause
#else
    csrr  a0, cipri
    csrr  a1, prithd
    csrw  prithd, a0                /* read prithd */
    SREG  a1, 6 * REGBYTES(sp)      /* save prithd */
    csrr  a1, mstatus               /* read mstatus */
    SREG  a1, 7 * REGBYTES(sp)      /* save mstatus */
    csrr  a1, mepc                  /* read mepc */
    SREG  a1, 8 * REGBYTES(sp)      /* save mepc */

    csrr  a0, mcause

    li  a1, (3<<11)
    csrs  mstatus, a1
    la    a1, custom_nested_irq_main_handler_entry
    csrw  mepc, a1
    mret
#endif

.align 2
custom_nested_irq_main_handler_entry:
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

#ifdef FLOAT_SUPPORT
    FSREG  f0, 19 * REGBYTES(sp)
    FSREG  f1, 20 * REGBYTES(sp)
    FSREG  f2, 21 * REGBYTES(sp)
    FSREG  f3, 22 * REGBYTES(sp)
    FSREG  f4, 23 * REGBYTES(sp)
    FSREG  f5, 24 * REGBYTES(sp)
    FSREG  f6, 25 * REGBYTES(sp)
    FSREG  f7, 26 * REGBYTES(sp)
    FSREG  f10, 27 * REGBYTES(sp)
    FSREG  f11, 28 * REGBYTES(sp)
    FSREG  f12, 29 * REGBYTES(sp)
    FSREG  f13, 30 * REGBYTES(sp)
    FSREG  f14, 31 * REGBYTES(sp)
    FSREG  f15, 32 * REGBYTES(sp)
    FSREG  f16, 33 * REGBYTES(sp)
    FSREG  f17, 34 * REGBYTES(sp)
    FSREG  f28, 35 * REGBYTES(sp)
    FSREG  f29, 36 * REGBYTES(sp)
    FSREG  f30, 37 * REGBYTES(sp)
    FSREG  f31, 38 * REGBYTES(sp)
#endif

    andi  a0, a0, MCAUSE_MASK_INT_NUM
    call  InterruptEntry

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

#ifdef FLOAT_SUPPORT
    FLREG  f0, 19 * REGBYTES(sp)
    FLREG  f1, 20 * REGBYTES(sp)
    FLREG  f2, 21 * REGBYTES(sp)
    FLREG  f3, 22 * REGBYTES(sp)
    FLREG  f4, 23 * REGBYTES(sp)
    FLREG  f5, 24 * REGBYTES(sp)
    FLREG  f6, 25 * REGBYTES(sp)
    FLREG  f7, 26 * REGBYTES(sp)
    FLREG  f10, 27 * REGBYTES(sp)
    FLREG  f11, 28 * REGBYTES(sp)
    FLREG  f12, 29 * REGBYTES(sp)
    FLREG  f13, 30 * REGBYTES(sp)
    FLREG  f14, 31 * REGBYTES(sp)
    FLREG  f15, 32 * REGBYTES(sp)
    FLREG  f16, 33 * REGBYTES(sp)
    FLREG  f17, 34 * REGBYTES(sp)
    FLREG  f28, 35 * REGBYTES(sp)
    FLREG  f29, 36 * REGBYTES(sp)
    FLREG  f30, 37 * REGBYTES(sp)
    FLREG  f31, 38 * REGBYTES(sp)
#endif

quit_int:
    /*
     *  Since the interrupt is already turned off when loading mstatus (after entering the interrupt,
     *  the hardware will turn off the interrupt, so when saving mstatus, the interrupt is already turned off),
     *  so there is no need to turn off the interrupt separately.
     */

#if defined(HARD_NESTED_IRQ_SUPPORT) && (HARD_NESTED_IRQ_SUPPORT == 1)
    LREG  t0, 0 * REGBYTES(sp)
#else
    LREG  a0, 7 * REGBYTES(sp)  /* load mstatus */
    csrr  t0, mstatus
    LREG  a1, 8 * REGBYTES(sp)  /* load mepc */
    andi  t0, t0, MSTATUS_MIE
    bnei  t0, 0, restore_mstatus
    andi  a0, a0, ~(MSTATUS_MIE | MSTATUS_MPIE)
restore_mstatus:
    csrw  mstatus, a0

    LREG  t0, 0 * REGBYTES(sp)
    csrw  mepc, a1
    LREG  a0, 6 * REGBYTES(sp)  /* load prithd */
    csrw  prithd, a0
#endif

#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
    la    a0, g_RiscvPrivMode
    lw    a1, (a0)
    addi  a1, a1, -1
    sw    a1, (a0)
#endif

    LREG  a1, 4 * REGBYTES(sp)   /* 2 consecutive csrw instructions will have a bubble */

    LREG  a0, 3 * REGBYTES(sp)

    addi  sp, sp, TOTAL_INT_SIZE_ON_STACK

    mret

.align 2
TrapVector:
    push_reg
    csrr  a0, mcause
    li    t1, MCAUSE_ECALL_FROM_MMODE
#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
    beq   a0, t1, switch_to_umode
#else
    beq   a0, t1, switch_to_mmode
#endif
    li    t1, MCAUSE_ECALL_FROM_UMODE
    beq   a0, t1, switch_to_mmode

    li    a1, MCAUSE_MASK_INT_BIT
    li    a2, MCAUSE_MASK_INT_NUM
    and   a1, a0, a1
    and   a0, a0, a2

    li    a2, 0xc
    beq   a0, a2, NmiEntry
    beqz  a1, TrapEntry
    pop_reg
    mret

#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
.align 2
switch_to_umode:
    li    t2,  MSTATUS_MPP_MACHINE
    csrc  mstatus, t2
    csrr  t0,  mepc
    addi  t0,  t0,  4
    csrw  mepc, t0
    pop_reg
    mret
#endif

.align 2
switch_to_mmode:
    li    t2,  MSTATUS_MPP_MACHINE
    csrs  mstatus, t2
    csrr  t0,  mepc
    addi  t0,  t0,  4
    csrw  mepc, t0
    pop_reg
    mret

.align 2
mem_cpy:
    bge t0, t2, cpy_done
    lw  t3, (t1)
    sw  t3, (t0)
    addi t0, t0, 4
    addi t1, t1, 4
    j mem_cpy
cpy_done:
    ret

.align 2
handle_reset:
    csrwi mstatus, 0
    csrwi mie,  0
    csrci mstatus, 0x08
    la    t0,  TrapHandler
    addi  t0,  t0, 1
    csrw  mtvec, t0
    csrwi 0x7EF, 0x1               /* lock mtvec */

#if defined(HARD_NESTED_IRQ_SUPPORT) && (HARD_NESTED_IRQ_SUPPORT == 1)
    csrwi 0x7C8, 0x1              /* enable hardware nest interrupt support */
#endif

flash_init:
/* eflash prefetch enable */
    li  t0, EFC_BASE_ADDR
    lw  t1, 0x120(t0)
    ori t1, t1, 1
    sw  t1, 0x120(t0)

/* eflash cache enable */
    lw  t1, 0x124(t0)
    ori t1, t1, 1
    sw  t1, 0x124(t0)

/* enable flash cmd */
    li t0, EFC_MAGIC_NUMBER
    li t1, EFC_MAGIC_LOCK_RW
    sw t0, (t1)

/* initialize global pointer */
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop

/* initialize stack pointer */
    la sp, __stack_top

/* timer0 interrupt enable */
    li t0, TIMER0_CONTROL
    lw t1, (t0)
    andi t1, t1, TIMER0_INTENABLE
    sw t1, (t0)

/* uart0 deinit */
    li t0, 0x14000000
    li t1, 0
    sw t1, IBRD_OFFSET(t0)
    sw t1, FBRD_OFFSET(t0)
    sw t1, LCR_H_OFFSET(t0)
    sw t1, CR_OFFSET(t0)
    sw t1, DMACR_OFFSET(t0)

/* perform the rest of initialization in C */
clear_sram:
    /* clear sysram parity error */
    li  t0, SYSRAM_ERROR
    lw  t1, (t0)
    ori t1, t1, 1
    sw  t1, (t0)

    la t0, SRAM_START
    la t1, SRAM_END
    li t2, 0

clear_sram_loop:
    sw      t2, (t0)            /* clear all sram */
    addi    t0, t0, 4           /* increment clear index pointer */
    blt     t0, t1, clear_sram_loop /* are we at the end yet, if not , contiue till the end */

start_coderom_code_copy:
    la t0, __sram_code_start_addr    /* SRAM addr */
    la t1, __sram_code_load_addr     /* ROM addr  */
    la t2, __sram_code_end_addr
    jal mem_cpy

start_reserved_data_copy:
    la t0, __reserved_code_start_addr  /* SRAM addr */
    la t1, __reserved_code_load_addr   /* ROM addr  */
    la t2, __reserved_code_end_addr
    jal mem_cpy

start_coderom_data_copy:
    la t0, __data_start              /* SRAM addr */
    la t1, __data_load               /* ROM addr  */
    la t2, __data_end
    jal mem_cpy

pmp_init:
    li t0, 0xB00
    csrw pmpaddr0, t0
    li t0, 0x400400   /* 0x2C00~0x1000FFF, BOOTROM, enable R+X */
    csrw pmpaddr1, t0
    li t0, 0x800000   /* 0x1001000~0x1FFFFFF, Reserved: diable R+X+W */
    csrw pmpaddr2, t0
    li t0, 0x802000   /* 0x2000000~0x2007FFF, SYSRAM_ITCM */
    csrw pmpaddr3, t0
    li t0, 0xC00000   /* 0x2008000 ~ 0x2FFFFFF, Reserved: disable R+X+W */
    csrw pmpaddr4, t0
    li t0, 0x1000000  /* 0x3000000 ~ 0x03FFFFFF: EFLASH: enable R+X */
    csrw pmpaddr5, t0
    li t0, 0x1002000  /* 0x4000000 ~ 0x04007FFF: SYSTEM_DTCM enable R+W */
    csrw pmpaddr6, t0
    li t0,0x7000400   /* 0x4008000 ~ 0x01C000FFF: REGISTER R+W  */
    csrw pmpaddr7, t0

    li t0,0xf3333333  /* register TOR-R-W */
    csrw 0x7d8,t0
#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
    li t0,0x0d080d8b  /* 0x0d:TOR-R-X; 0x0b:TOR-R-W; 0x08:TOR; 0x0c:TOR-x; 0x09:TOR-R */
    csrw pmpcfg0,t0
    li t0,0x0b0b0d08
    csrw pmpcfg1,t0
#else
    li t0,0x8d888d8b  /* 0x0d:TOR-R-X; 0x0b:TOR-R-W; 0x08:TOR; 0x0c:TOR-x; 0x09:TOR-R */
    csrw pmpcfg0,t0
    li t0,0x8b8b8d88
    csrw pmpcfg1,t0
#endif

/* disable Icache */
    csrwi  0x7C0, 0x0 /* disable ICACHE */
    fence

/* disable Dcache */
    csrwi  0x7C1, 0x0 /* disable DCACHE */
    fence

/* support float and mie */
    li t0,0x2008
    csrs mstatus,t0
    li t0,0x20
    csrs misa,t0

/* Interrupt set default priority = 1*/
    li t0, 0x11111111
    csrw locipri0, t0
    csrw locipri1, t0
    csrw locipri2, t0
    csrw locipri3, t0
    csrw locipri4, t0
    csrw locipri5, t0
    csrw locipri6, t0
    csrw locipri7, t0
    csrw locipri8, t0
    csrw locipri9, t0
    csrw locipri10, t0
    csrw locipri11, t0
    csrw locipri12, t0
    csrw locipri13, t0
    csrw locipri14, t0
    csrw locipri15, t0

    ecall
    jal Chip_Init

/* jump to C func. */
    jal main

dead_loop:
    j dead_loop

#endif
