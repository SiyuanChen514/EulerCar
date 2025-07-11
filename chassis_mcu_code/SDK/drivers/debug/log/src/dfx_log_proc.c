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
  * @file      gpio_ip.h
  * @author    MCU Driver Team
  * @brief     GPIO module driver
  * @details   The header file contains the following declaration:
  *             + GPIO configuration enums.
  *             + GPIO register structures.
  *             + GPIO DCL Functions.
  *             + Parameters check functions.
  */
#include <stdlib.h>
#include <string.h>
#include "command.h"
#include "dfx_log.h"
#include "log.h"
#include "console.h"
#include "type.h"

/**
 * @brief show the log information.
 * @param None
 * @retval return whether the display is successful
 */
static int DrvLogShowLogLevel(void)
{
    struct SysLogCtx *ctx = GetLogCtx();

    if (!ctx->init) {
        LogCtxInit(ctx); /* Initialize the structure */
    }

    EXT_PRINT("\n");
    EXT_PRINT("\t ------- module log level -------\n"); /* Delimiter Display Title */
    struct SysDebugSwitch *debugSwitch = GetDebugSwitch();
    if (!debugSwitch->enable) {
        EXT_PRINT("The debug mode is disabled, and only err-level information is output\n");
    }
    EXT_PRINT("\n");
    EXT_PRINT("ModuleName ModuleId LogLevel\n");
    /* Displays log information line by line in sequence */
    for (unsigned char i = 0; i < EXT_MODULE_BUTT; i++) {
        EXT_PRINT("%s\t", ctx->modStr[i]);
        EXT_PRINT("%d\t", i);
        EXT_PRINT("%d", ctx->logLevel[i]);
        EXT_PRINT("\n");
    }
    return EXT_SUCCESS;
}

/**
 * @brief write log to memory
 * @param enable: Enables log writing to the memory
 * @retval return whether the display is successful
 */
static int DrvLogPutLogToMem(unsigned char enable)
{
    if (enable != EXT_TRUE && enable != EXT_FALSE) {
        EXT_PRINT("param err\n");
        return EXT_FAILURE;
    }

    struct SysLogCtx *ctx = GetLogCtx();
    if (!ctx->init) {
        LogCtxInit(ctx); /* Initialize the structure */
    }
    /* Flag bit 1 to start writing */
    if (enable) {
        ctx->memLog.enable = EXT_TRUE;
        EXT_PRINT("log put memory:0x%x enable\n", ctx->memLog.mmzBuf);
    } else {
        ctx->memLog.enable = EXT_FALSE;
        EXT_PRINT("log put memory disable\n");
    }

    /* Initialize Pointer */
    ctx->memLog.writePos = 0;
    ctx->memLog.logLen = 0;
    return EXT_SUCCESS;
}

/**
 * @brief print the logs stored in the memory
 * @param None
 * @retval None
 */
static void DrvLogPrintMemLog(void)
{
    struct SysLogCtx *ctx = GetLogCtx();
    if (!ctx->init) {
        LogCtxInit(ctx); /* Initialize the structure */
    }

    if (!ctx->memLog.enable) {
        EXT_PRINT("mem record log not enable\n");
        return;
    }

    unsigned short i;
    if (ctx->memLog.logLen == LOG_MEM_POOL_MAX_LEN) {
    /* Logs are printed one by one */
        for (i = ctx->memLog.writePos; i < LOG_MEM_POOL_MAX_LEN; ++i) {
            EXT_PRINT("%c", ctx->memLog.mmzBuf[i]);
        }
    }

    /* Cyclic Print Characters */
    for (i = 0; i < ctx->memLog.writePos; ++i) {
        EXT_PRINT("%c", ctx->memLog.mmzBuf[i]);
    }
}

/**
 * @brief Prints the help information about the log command
 * @param None
 * @retval None
 */
static void DrvLogCmdHelp(void)
{
    /* Print Command Prompt */
    EXT_PRINT("Usage:\n");
    EXT_PRINT("logcmd show  show log info\n");
    EXT_PRINT("logcmd  setlevel [moduleId][level] set log level(0:F,1:E,2:W,3:I,4:D)\n");
    EXT_PRINT("logcmd  setmem [0/1] enable mem log(1: print to memory, 0: print to console)\n");
    EXT_PRINT("logcmd print print log from memory\n");
}

/**
 * @brief Command Parsing of Driver Miniaturization Logs
 * @param argc: Total number of input strings
 * @param argv[]: Entered character string information.
 * @retval return whether the display is successful
 */
static int DrvLogCmd(unsigned int argc, const char *argv[])
{
    char *endp = NULL;
    if (argc < 2) { /* 2 is agrc */
        DrvLogCmdHelp();
        return EXT_FAILURE;
    } else if (strcmp(argv[1], "show") == 0) {
        DrvLogShowLogLevel();
    } else if (strcmp(argv[1], "setlevel") == 0) {
        if (argc < 4) { /* 4 is argc */
            DrvLogCmdHelp();
            return EXT_FAILURE;
        }
        unsigned int modId = strtoul(argv[2], &endp, 0); /* 2 is argv */
        unsigned int level = strtoul(argv[3], &endp, 0); /* 3 is argv */
        if (ExtDrvLogSetLogLevel(modId, level) != EXT_SUCCESS) {
            EXT_PRINT("set log level err\n");
            return EXT_FAILURE;
        }
        EXT_PRINT("setlevel succsee!\r\n");
    } else if (strcmp(argv[1], "setmem") == 0) {
        if (argc < 3) { /* 3 is argc */
            DrvLogCmdHelp();
            return EXT_FAILURE;
        }

        unsigned char enable = (unsigned char)strtoul(argv[2], &endp, 0); /* 2 is argv */
        if (DrvLogPutLogToMem(enable) != EXT_SUCCESS) {
            EXT_PRINT("set put mem err\n");
            return EXT_FAILURE;
        }
    } else if (strcmp(argv[1], "print") == 0) {
        DrvLogPrintMemLog();
    }

    return EXT_SUCCESS;
}

/**
 * @brief init dfx
 * @param None
 * @retval None
 */
void DfxCmdRegister(void)
{
    ExtCmdRegister("logcmd", &DrvLogCmd);
}