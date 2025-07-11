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
  * @file      dfx_log.h
  * @author    MCU Driver Team
  * @brief     dfx_log module driver
  * @details   The header file contains the following declaration:
  *             + Perhaps and print the log content.
  */
#ifndef DFX_LOG_H
#define DFX_LOG_H

#include "ext_log.h"

#ifdef __cplusplus__
#if __cplusplus__
extern "C" {
#endif
#endif

#define LOG_UINT_MAX_LEN 512
#define LOG_MEM_POOL_MAX_LEN 1024
#define LOG_LAST_WORD_MAX_LEN 1024

#define LOG_STATEMENT_MAX_LEN 20

/**
  * @addtogroup DEBUG_Log
  * @brief DEBUG external module.
  * @{
  */

 /**
  * @defgroup DFX_LOG_Def DFX_LOG_Def
  * @brief Initialization of Miniaturized Logs.
  * @{
  */

struct MemoryLog {
    unsigned char enable;
    unsigned char mmzBuf[LOG_MEM_POOL_MAX_LEN];
    unsigned int writePos;
    unsigned int logLen;
};
struct SysLogCtx {
    unsigned char init;
    char **modStr;
    enum ExtLogLevel logLevel[EXT_MODULE_BUTT];
    struct MemoryLog memLog;
};
struct SysDebugSwitch {
    unsigned char enable;
    struct SysLogCtx logCtx;
};

/**
 * @brief get log context.
 * @attention None
 *
 * @retval struct SysLogCtx *.
 */
struct SysLogCtx *GetLogCtx(void);

/**
 * @brief init log context.
 * @attention None
 *
 * @param ctx: Pointer to the SysLogCtx structure to be initialized.
 * @retval None
 */
void LogCtxInit(struct SysLogCtx *ctx);

/**
 * @brief init struct MemoryLog.
 * @attention None
 *
 * @param memData: Pointer to the MemoryLog structure to be initialized
 * @retval None
 */
void InitMemoryData(struct MemoryLog *memData);

/**
 * @brief get debug switch.
 * @attention None
 *
 * @retval struct SysDebugSwitch *.
 */
struct SysDebugSwitch *GetDebugSwitch(void);

/**
 * @brief get memory data.
 * @attention None
 *
 * @retval struct MemoryLog *.
 */
struct MemoryLog *GetMemoryData(void);

/**
 * @brief Register the dfx cmd
 * @attention None
 *
 * @retval None
 */
void DfxCmdRegister(void);

/**
 * @brief get version info cmd
 * @attention None
 *
 * @param argc: Number of input parameters.
 * @param argv: Array of pointers
 * @retval Return the setting result, success or failure.
 */
int CmdGetVersionInfo(void);

#ifdef __cplusplus__
#if __cplusplus__
}
#endif
#endif /* end of __cplusplus */
/**
  * @}
  */

/**
  * @}
  */
#endif
