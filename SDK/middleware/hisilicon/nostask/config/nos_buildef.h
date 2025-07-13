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
  * @file      nos_buildef.h
  */
#ifndef NOS_BUILDEF_H
#define NOS_BUILDEF_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#define OS_ARCH_RISCV                       YES

#ifndef OS_HARDWARE_PLATFORM
#define OS_HARDWARE_PLATFORM                OS_RISCV
#endif

#ifndef OS_CPU_TYPE
#define OS_CPU_TYPE                         OS_HIMIDEER
#endif

#define OS_MAX_CORE_NUM                     1

#define OS_OPTION_FPU                       YES

#define OS_OPTION_HWI_PRIORITY              YES

#define OS_OPTION_TASK                      YES

#define OS_OPTION_TASK_DELETE               YES

#define OS_OPTION_TASK_YIELD                YES

#define OS_TSK_PRIORITY_HIGHEST             0

#define OS_TSK_PRIORITY_LOWEST              5

#define OS_TSK_NUM_OF_PRIORITIES            6

#define OS_TSK_CORE_BYTES_IN_PID            2

#define OS_OPTION_TICK                      YES

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

/* common macros's definitions */
#include "os_buildef_common.h"

#endif
