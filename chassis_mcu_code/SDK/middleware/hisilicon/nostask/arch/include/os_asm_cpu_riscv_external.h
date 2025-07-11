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
  * @file      os_asm_cpu_riscv_external.h
  */
#ifndef OS_ASM_CPU_RISCV_EXTERNAL_H
#define OS_ASM_CPU_RISCV_EXTERNAL_H

#define OS_FLG_TSK_REQ 0x1000U
#define OS_FLG_BGD_ACTIVE 0x0002U
#define OS_FLG_HWI_ACTIVE 0x0001U
#define OS_FLG_SWI_ACTIVE 0x0004U
#define OS_FLG_TICK_ACTIVE 0x0008U
#define OS_FLG_SYS_ACTIVE 0x0010U
#define OS_FLG_EXC_ACTIVE 0x0020U
#define OS_FLG_FIBER_ACTIVE 0x40000U   /* 纤程激活 */

#define REG_OFF_RA      0
#define REG_OFF_GP      (REG_OFF_RA + 4)      // 4
#define REG_OFF_TP      (REG_OFF_GP + 4)      // 8
#define REG_OFF_T0      (REG_OFF_TP + 4)      // 12
#define REG_OFF_T1      (REG_OFF_T0 + 4)      // 16
#define REG_OFF_T2      (REG_OFF_T1 + 4)      // 20
#define REG_OFF_T3      (REG_OFF_T2 + 4)      // 24
#define REG_OFF_T4      (REG_OFF_T3 + 4)      // 28
#define REG_OFF_T5      (REG_OFF_T4 + 4)      // 32
#define REG_OFF_T6      (REG_OFF_T5 + 4)      // 36
#define REG_OFF_A0      (REG_OFF_T6 + 4)      // 40
#define REG_OFF_A1      (REG_OFF_A0 + 4)      // 44
#define REG_OFF_A2      (REG_OFF_A1 + 4)      // 48
#define REG_OFF_A3      (REG_OFF_A2 + 4)      // 52
#define REG_OFF_A4      (REG_OFF_A3 + 4)      // 56
#define REG_OFF_A5      (REG_OFF_A4 + 4)      // 60
#define REG_OFF_A6      (REG_OFF_A5 + 4)      // 64
#define REG_OFF_A7      (REG_OFF_A6 + 4)      // 68
#define REG_OFF_MEPC    (REG_OFF_A7 + 4)      // 72
#define REG_OFF_MSTATUS (REG_OFF_MEPC + 4)    // 76

#define CALLER_REG_SIZE (REG_OFF_MSTATUS + 4) // 80

#define REG_OFF_S0      0
#define REG_OFF_S1      (REG_OFF_S0 + 4)      // 4
#define REG_OFF_S2      (REG_OFF_S1 + 4)      // 8
#define REG_OFF_S3      (REG_OFF_S2 + 4)      // 12
#define REG_OFF_S4      (REG_OFF_S3 + 4)      // 16
#define REG_OFF_S5      (REG_OFF_S4 + 4)      // 20
#define REG_OFF_S6      (REG_OFF_S5 + 4)      // 24
#define REG_OFF_S7      (REG_OFF_S6 + 4)      // 28
#define REG_OFF_S8      (REG_OFF_S7 + 4)      // 32
#define REG_OFF_S9      (REG_OFF_S8 + 4)      // 36
#define REG_OFF_S10     (REG_OFF_S9 + 4)      // 40
#define REG_OFF_S11     (REG_OFF_S10 + 4)     // 44

#define CALLEE_REG_SIZE (REG_OFF_S11 + 4)     // 48

#define MCAUSE_ECALL    11
#define MCAUSE_EXC_MASK 0x7fffffffU

#endif /* OS_ASM_CPU_RISCV_EXTERNAL_H */
