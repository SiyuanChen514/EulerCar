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
  * @file      nos_tick.c
  */
#include "nos_task_external.h"
/*
 * 描述: Tick中断的处理函数。扫描任务超时链表、扫描超时软件定时器、扫描TSKMON等。
 */
static void OsTickDispatcher(void)
{
    uintptr_t intSave;

    intSave = NOS_IntLock();

    g_uniTicks++;

    // 任务超时扫描
    if (g_taskScanHook != NULL) {
        g_taskScanHook();
    }

    NOS_IntRestore(intSave);
}

/*
 * 描述: tick尾部处理
 */
void OsHwiDispatchTick(void);
void OsHwiDispatchTick(void)
{
    TICK_NO_RESPOND_CNT++;

    if ((UNI_FLAG & OS_FLG_TICK_ACTIVE) != 0) {
        // OsTskContextLoad， 回到被打断的tick处理现场
        return;
    }

    UNI_FLAG |= OS_FLG_TICK_ACTIVE;

    do {
        /* 暂不开中断（否则中断处理流程切栈代码需要修改） */
        OsTickDispatcher();
        TICK_NO_RESPOND_CNT--;
    } while (TICK_NO_RESPOND_CNT > 0);

    UNI_FLAG &= ~OS_FLG_TICK_ACTIVE;
}
