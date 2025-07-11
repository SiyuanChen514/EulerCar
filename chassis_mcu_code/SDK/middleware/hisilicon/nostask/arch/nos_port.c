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
  * @file      nos_port.c
  */
#include "nos_sys_external.h"

#define INDEX_STEP                  8
#define HIGH_8BIT_U32_OFFSET        24
#define HIGH_4BIT_U32_OFFSET        28
#define LEADING_ZERO_NUM_OF_ZERO    32
#define OS_MAX_U8 0xFFU
#define OS_GET_8BIT_HIGH_4BIT(num)      ((num) >> 4)
#define OS_GET_8BIT_LOW_4BIT(num)       ((num) & 0xFU)

void OsTskContextGet(uintptr_t saveAddr, struct TskContext *context)
{
    if (OS_INT_ACTIVE) {
        /* 中断上下文中，callee-save寄存器是未压栈且不准确的 */
        *context = *(struct TskContext *)(saveAddr - CALLER_REG_SIZE);
    } else {
        *context = *(struct TskContext *)saveAddr;
    }
}
#ifdef OS_OPTION_306X
#include "nos_306x_adapter.h"
#endif
/* 任务上下文初始化 */
void *OsTskContextInit(unsigned int taskId, unsigned int stackSize, uintptr_t *topStack, uintptr_t funcTskEntry)
{
    struct TskContext *context = (struct TskContext *)((uintptr_t)topStack + stackSize - sizeof(struct TskContext));

    /* 传参 */
    context->ra = 0;
    context->a0 = taskId;
    context->mepc = funcTskEntry;
    context->mstatus = MSTATUS_DEF_RESTORE;
    context->prithd = 0;
    context->fcsr = 0;
#ifdef OS_OPTION_306X
    context->tickIntNum = TICK_IRQ_EN_NUM;
#endif
    return (void *)context;
}

/* 32表示无效，3、2、1、0表示在4bit位中前导零的个数 */
unsigned short g_lmb1Idx[16] = { 32, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

/* getLMB1 */
unsigned int OsGetLMB1(unsigned int value)
{
    signed int idx;
    unsigned int check1;
    unsigned int check2;

    for (idx = HIGH_8BIT_U32_OFFSET; idx >= 0; idx -= INDEX_STEP) {
        check1 = (value >> (unsigned int)idx) & OS_MAX_U8;

        if (check1 != 0) {
            /* get high bit */
            check2 = OS_GET_8BIT_HIGH_4BIT(check1);
            if (check2 != 0) {
                return ((unsigned int)g_lmb1Idx[check2] + (unsigned int)(HIGH_8BIT_U32_OFFSET - idx));
            }

            /* get low bit */
            return ((unsigned int)g_lmb1Idx[OS_GET_8BIT_LOW_4BIT(check1)] + (unsigned int)(HIGH_4BIT_U32_OFFSET - idx));
        }
    }

    return LEADING_ZERO_NUM_OF_ZERO;
}
