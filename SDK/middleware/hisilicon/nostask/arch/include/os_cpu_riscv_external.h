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
  * @file      os_cpu_riscv_external.h
  */
#ifndef OS_CPU_RISCV_EXTERNAL_H
#define OS_CPU_RISCV_EXTERNAL_H

#include "nos_buildef.h"

#define OS_TSK_STACK_SIZE_ALIGN 16
#define OS_TSK_MIN_STACK_SIZE 0x100

/* Previous Privilege Mode - Machine Mode */
#define MSTATUS_MPP_M_MODE       (3U << 11)
/* Interrupt Enable Bit in Previous Privilege Mode */
#define MSTATUS_MPIE             (1U << 7)

#define MSTATUS_FS               (1U << 13)

/*
 * Default MSTATUS register value to restore from stack
 * upon scheduling a thread for the first time
 */
#define MSTATUS_DEF_RESTORE      (MSTATUS_MPP_M_MODE | MSTATUS_MPIE | MSTATUS_FS)

extern void OsTskContextLoad(uintptr_t tcbAddr);
extern void OsTaskSwitch(void);
extern unsigned int OsGetLMB1(unsigned int value);

/* task switch */
INLINE void OsTaskTrap(void)
{
    OsTaskSwitch();
}

#endif /* OS_CPU_RISCV_EXTERNAL_H */
