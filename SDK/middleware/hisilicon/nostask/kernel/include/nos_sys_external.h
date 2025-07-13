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
  * @file      nos_sys_external.h
  */
#ifndef NOS_SYS_EXTERNAL_H
#define NOS_SYS_EXTERNAL_H

#include "nos_base_task.h"
#include "nos_list_external.h"
#include "nos_cpu_external.h"

#define OS_INT_ACTIVE_MASK                                        \
    (OS_FLG_HWI_ACTIVE | OS_FLG_SWI_ACTIVE | OS_FLG_TICK_ACTIVE | \
     OS_FLG_SYS_ACTIVE | OS_FLG_EXC_ACTIVE | OS_FLG_FIBER_ACTIVE)

#define OS_INT_ACTIVE ((UNI_FLAG & OS_INT_ACTIVE_MASK) != 0)
#define OS_INT_INACTIVE (!(OS_INT_ACTIVE))
#define OS_THREAD_FLAG_MASK  (OS_FLG_BGD_ACTIVE | OS_INT_ACTIVE_MASK)

/* 有TICK情况下CPU占用率触发函数类型定义。 */
typedef void (*TickEntryFunc)(void);
typedef void (*TaskScanFunc)(void);

/*
 * 模块间全局变量声明
 */
extern unsigned int g_threadNum;

extern unsigned int g_tickNoRespondCnt;
#define TICK_NO_RESPOND_CNT g_tickNoRespondCnt

extern unsigned long long g_uniTicks;
/* Tick计数补偿值 */
extern TaskScanFunc g_taskScanHook;
extern OsVoidFunc g_idleEntry;
extern unsigned int g_uniFlag;
#define UNI_FLAG g_uniFlag

typedef unsigned long long (*GetTickFunc)(void);

void NOS_SetNosParam(GetTickFunc func);
unsigned long long NOS_GetCurTick(void);

#endif /* NOS_SYS_EXTERNAL_H */
