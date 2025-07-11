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
  * @file      nos_config.c
  */
#include "nos_config_internal.h"
#include "nos_sys_external.h"
#include "os_sys.h"

/* 任务初始化 */
static unsigned int OsTskConfigInit(void)
{
    unsigned int ret;
    ret = OsTskInit();
    return ret;
}

static unsigned int OsSysConfigReg(void)
{
    /* idle注册 */
    OsIdleReg();
    UNI_FLAG = 0;

    return 0;
}

/* OsStart阶段对相应的运行模块不进行OS_INIT_TRACE_END TRACE记录 */
unsigned int OsStart(void)
{
    unsigned int ret;

    /* 表示系统在进行启动阶段，匹配MOUDLE_ID之后，标记进入任务模块的启动 */
    ret = OsActivate();

    return ret;
}

int NOS_MoudleInit(void)
{
    OsSysConfigReg();
    OsTskConfigInit();

    /* Execution should not reach this point */
    return (signed int)OS_OK;
}
