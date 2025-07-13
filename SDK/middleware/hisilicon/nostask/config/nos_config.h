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
  * @file      nos_config.h
  */
#ifndef NOS_CONFIG_H
#define NOS_CONFIG_H

#include "nos_buildef.h"
#include "nos_typedef.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/* ***************************** 配置任务模块 ******************************* */
/* 任务模块裁剪开关 */
#define OS_INCLUDE_TASK                                 YES
/* 最大支持的任务数,软中断和任务最大共支持254个 */
#define OS_TSK_MAX_SUPPORT_NUM                          5
/* 缺省的任务栈大小 */
#define OS_TSK_DEFAULT_STACK_SIZE                       0x200
/* IDLE任务栈的大小 */
#define OS_TSK_IDLE_STACK_SIZE                          0x150
/* 任务栈初始化魔术字，默认是0xCA，只支持配置一个字节 */
#define OS_TSK_STACK_MAGIC_WORD                         0xCACACACA

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#endif /* NOS_CONFIG_H */
