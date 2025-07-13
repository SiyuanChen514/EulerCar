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
  * @file      dfx_log.c
  * @author    MCU Driver Team
  * @brief     dfx_log module driver
  * @details   The header file contains the following declaration:
  *             + Small-scale log output
  *             + Miniaturized log output with different numbers of int types
  */
#include <stdio.h>
#include "string.h"
#include "stdarg.h"
#include "type.h"
#include "dfx_log.h"
#include "common.h"
#include "log.h"
#include "ext_log.h"
#include "securec.h"

#define EXT_DEFAULT_LOG_LEVEL  EXT_LOG_LEVEL_ERROR
#define THIS_FILE_ID    FILE_ID_LOG_C
static struct MemoryLog g_memoryLog = {0};
#define DIVISOR 10
#define EXT_MODULE_DFX 12 /* Test Version Information Cases */
/* Address of the test case for obtaining version information */
#define VERSION_INFO_ADDR 0x4000000
/* Device Name */
char *moduleStr[EXT_MODULE_BUTT] = {
    "app_main",
    "app_console",
    "app_chip",
    "drv_base",
    "drv_chips",
    "drv_crg",
    "drv_gpio",
    "drv_i2c",
    "drv_irq",
    "drv_pinctrl",
    "drv_timer",
    "drv_uart",
    "dfx",
};
/* Levels that can be set */
char *ExtLogLevel1[6] = {
    "EXT_LOG_LEVEL_FATAL",
    "EXT_LOG_LEVEL_ERROR",
    "EXT_LOG_LEVEL_WARNING",
    "EXT_LOG_LEVEL_INFO",
    "EXT_LOG_LEVEL_DBG",
    "EXT_LOG_LEVEL_BUTT",
};
struct SysLogCtx g_logCtx = { 0 };
/**
 * @defgroup log Common
 * @brief Initialize miniaturization log information.
 * @{
 */
struct SysLogCtx *GetLogCtx(void)
{
    return &g_logCtx;
}
static struct SysDebugSwitch g_debugSwitch = {.enable = 1};
struct SysDebugSwitch *GetDebugSwitch(void)
{
    /* Return Enable Initialization */
    return &g_debugSwitch;
}
/**
 * @brief Initialize register information.
 * @param memData: Register structure variable
 * @retval None.
 */
void InitMemoryData(struct MemoryLog *memData)
{
    memData->enable = EXT_TRUE;
    memData->logLen = 0;
    memData->writePos = 0;
}

/**
 * @brief Obtains the value of register information.
 * @param None.
 * @retval memory address
 */
struct MemoryLog *GetMemoryData(void)
{
    return &g_memoryLog;
}

/**
 * @brief Initialize the environment information for miniaturization logs.
 * @param ctx: Environment information of miniaturized logs
 * @retval None.
 */
void LogCtxInit(struct SysLogCtx *ctx)
{
    ctx->modStr = moduleStr;
    for (unsigned char i = 0; i < EXT_MODULE_BUTT; i++) {
        ctx->logLevel[i] = EXT_DEFAULT_LOG_LEVEL;
    }
    ctx->init = EXT_TRUE;
}

/**
 * @brief Write the log to the memory.
 * @param *memlog: memory address
 * @param src: Stored Information
 * @param cnt: Length of the stored information
 * @retval None.
 */
static void PutLogToMem(struct MemoryLog *memLog, const char *src, unsigned char cnt)
{
    unsigned char len = cnt; /* default mem write pos < LOG_MEM_POOL_MAX_LEN - cnt */

    if (cnt > LOG_MEM_POOL_MAX_LEN - memLog->writePos) {
        len = LOG_MEM_POOL_MAX_LEN - memLog->writePos;
        /* put log data to buf */
        if (memcpy_s(memLog->mmzBuf + memLog->writePos, LOG_MEM_POOL_MAX_LEN - memLog->writePos, src, len) !=
            EXT_SUCCESS) {
            EXT_PRINT("put log to memory memcpy err\n");
            return;
        }
        /* if the data is full, the position pointer returns to the origin.  */
        memLog->writePos = 0;
        src += len;
        len = cnt - len;
    }
    /* if the data is full, cyclic write log data */
    if (memcpy_s(memLog->mmzBuf + memLog->writePos, LOG_MEM_POOL_MAX_LEN - memLog->writePos, src, len) != EXT_SUCCESS) {
        EXT_PRINT("put log to memory memcpy err\n");
        return;
    }

    /* The pointer position is increased by the write length */
    memLog->writePos += len;
    memLog->logLen += cnt;
    if (memLog->logLen > LOG_MEM_POOL_MAX_LEN) {
        memLog->logLen = LOG_MEM_POOL_MAX_LEN;
    }
}

/**
 * @brief Calculates the length of an int number converted to a character string.
 * @param num: number to calculate.
 * @retval Length after being converted to a character string.
 */
static int CountNumberLen(unsigned int num)
{
    int count = 0;
    do {
        count += 1;
        num = num/DIVISOR;
    } while (num != 0); /* divided by 10 to round */
    return count;
}

/**
 * @brief Check whether the log output is proper.
 * @param level: Specifies the log level.
 * @param debugSwitch: Pointer to the debug mode
 * @param modId: Device ID
 * @param ctx Pointer to storing log information
 * @retval Indicates whether the printing is successful.
 */
static unsigned int IsLogOutBufLegal(enum ExtLogLevel level, struct SysDebugSwitch *debugSwitch,
    enum ExtModule modId, struct SysLogCtx *ctx)
{
    /* Check whether the value is out of range */
    if (level > EXT_LOG_LEVEL_BUTT || modId > EXT_MODULE_BUTT)
        return EXT_FAILURE;
    /* Checking the Status of debug */
    if (((!debugSwitch->enable) && (level != EXT_LOG_LEVEL_ERROR)) || (level > ctx->logLevel[modId])) {
        return EXT_SUCCESS;
    }
    return EXT_FAILURE;
}

/**
 * @brief Log output and printing
 * @param level: Specifies the log level.
 * @param modId: Device ID
 * @param id: device name
 * @param logBuf: Character string information to be printed
 * @param logBuflen: Indicates the length of the printed information.
 * @retval Indicates whether the printing is successful.
 */
int ExtDrvLogOutBuf(enum ExtLogLevel level, enum ExtModule modId, unsigned int id, const unsigned int* logBuf,
    unsigned short logBufLen)
{
    /* Check whether the array is empty */
    if (logBuf == NULL)
        return EXT_FAILURE;
    /* Value Definition Initialization */
    char buf[LOG_UINT_MAX_LEN] = { 0 };
    int cnt = 0;
    int len = 0;
    int count = 0;

    struct SysLogCtx *ctx = GetLogCtx();

    struct SysDebugSwitch *debugSwitch = GetDebugSwitch();
    if (!ctx->init) { LogCtxInit(ctx); } /* Initialize the structure */
    if (!(IsLogOutBufLegal(level, debugSwitch, modId, ctx))) { return EXT_SUCCESS; }
    cnt = sprintf_s(buf, LOG_UINT_MAX_LEN, "%u", id);
    /* an error message is displayed when the return value is a negative value */
    if (cnt < 0) {
        EXT_PRINT("sprintf err\n");
        return EXT_FAILURE;
    }
    len += cnt;

    unsigned short i = 0;
    /* Write characters cyclically */
    for (; i < logBufLen; ++i) {
        count = CountNumberLen(logBuf[i]);
        if ((count + len + 1) >= LOG_UINT_MAX_LEN) { return EXT_FAILURE; }
        cnt = sprintf_s(buf + len, LOG_UINT_MAX_LEN - len, " %u", logBuf[i]);
        /* an error message is displayed when the return value is a negative value */
        if (cnt < 0) {
            EXT_PRINT("sprintf err\n");
            return EXT_FAILURE;
        }
        len += cnt;
    }
    cnt = sprintf_s(buf + len, LOG_UINT_MAX_LEN - len, "\n");
    len += cnt;
    /* an error message is displayed when the return value is a negative value */
    if (cnt < 0) {
        EXT_PRINT("sprintf err\n");
        return EXT_FAILURE;
    } else if (!ctx->memLog.enable) {
        EXT_PRINT("%s", buf);
        return EXT_SUCCESS;
    }

    PutLogToMem(&ctx->memLog, buf, len); /* Storing the log information into the memory */
    return EXT_SUCCESS;
}


/**
 * @brief get version info cmd
 * @param None
 * @retval Return the setting result, success or failure.
 */
int CmdGetVersionInfo(void)
{
    int versionInfo;
    versionInfo = EXT_REG_READ32(VERSION_INFO_ADDR);
    /* Print version information */
    ExtLog1(ERR, EXT_MODULE_DFX, "version info is : %x\n", versionInfo);
    return EXT_SUCCESS;
}
/**
 * @brief Processing log buffer
 * @param len: Length of the processed data.
 * @param level: Specifies the log level.
 * @param modId: Device ID
 * @param buf: Log information to be processed.
 * @retval Indicates whether the printing is successful.
 */
static int DealLogBuf(int len, enum ExtLogLevel level, enum ExtModule modId, const char buf[])
{
    struct SysLogCtx *ctx = GetLogCtx();
    struct SysDebugSwitch *debugSwitch = GetDebugSwitch();
    /* Checking the Status of debug */
    if (!debugSwitch->enable) {
        if (level != EXT_LOG_LEVEL_ERROR) {
            return EXT_SUCCESS;
        }
    }

    if (!ctx->init) { LogCtxInit(ctx); } /* Initialize the structure */

    if (level > ctx->logLevel[modId]) { return EXT_SUCCESS; }
    /* If the length is negative, an error value is returned */
    if (len < 0) {
        EXT_PRINT("sprintf err\n");
        return EXT_FAILURE;
    }

    if (!ctx->memLog.enable) {
        EXT_PRINT("%s", buf);
        return EXT_SUCCESS;
    }

    PutLogToMem(&ctx->memLog, buf, len); /* Storing the log information into the memory */
    return EXT_SUCCESS;
}

/**
 * @brief Print with no int number
 * @param level: Specifies the log level.
 * @param modId: Device ID
 * @param id: custom string variable
 * @retval Indicates whether the printing is successful.
 */
int ExtDrvLogOut0(enum ExtLogLevel level, enum ExtModule modId, unsigned int id)
{
    /* Check whether the value is out of range */
    if (level > EXT_LOG_LEVEL_BUTT || modId > EXT_MODULE_BUTT)
        return EXT_FAILURE;
    char buf[LOG_UINT_MAX_LEN] = { 0 };
    int len = 0;
    len = sprintf_s(buf, LOG_UINT_MAX_LEN, "%u\n", id);
    if (len < 0) {
        EXT_PRINT("sprintf err\n");
        return EXT_FAILURE;
    }
    /* Process logs and determine whether to write to memory */
    return (DealLogBuf(len, level, modId, buf));
}

/**
 * @brief Print with an int number
 * @param level: Specifies the log level.
 * @param modId: Device ID
 * @param id: custom string variable
 * @param d0: User-defined first variable of the int type
 * @retval Indicates whether the printing is successful.
 */
int ExtDrvLogOut1(enum ExtLogLevel level, enum ExtModule modId, unsigned int id, unsigned int d0)
{
    /* Check whether the value is out of range */
    if (level > EXT_LOG_LEVEL_BUTT || modId > EXT_MODULE_BUTT)
        return EXT_FAILURE;
    char buf[LOG_UINT_MAX_LEN] = { 0 };
    int len = 0;
    len = sprintf_s(buf, LOG_UINT_MAX_LEN, "%u %u\n", id, d0);
    if (len < 0) {
        EXT_PRINT("sprintf err\n");
        return EXT_FAILURE;
    }
    /* Process logs and determine whether to write to memory */
    return (DealLogBuf(len, level, modId, buf));
}

/**
 * @brief Print with two int numbers
 * @param level: Specifies the log level.
 * @param modId: Device ID
 * @param id: custom string variable
 * @param d0: User-defined first variable of the int type
 * @param d1: User-defined second variable of the int type
 * @retval Indicates whether the printing is successful.
 */
int ExtDrvLogOut2(enum ExtLogLevel level, enum ExtModule modId, unsigned int id, unsigned int d0, unsigned int d1)
{
    /* Check whether the value is out of range */
    if (level > EXT_LOG_LEVEL_BUTT || modId > EXT_MODULE_BUTT)
        return EXT_FAILURE;
    char buf[LOG_UINT_MAX_LEN] = { 0 };
    int len = 0;
    len = sprintf_s(buf, LOG_UINT_MAX_LEN, "%u %u %u\n", id, d0, d1);
    if (len < 0) {
        EXT_PRINT("sprintf err\n");
        return EXT_FAILURE;
    }
    /* Process logs and determine whether to write to memory */
    return (DealLogBuf(len, level, modId, buf));
}

/**
 * @brief Print with three int numbers
 * @param level: Specifies the log level.
 * @param modId: Device ID
 * @param id: custom string variable
 * @param d0: User-defined first variable of the int type
 * @param d1: User-defined second variable of the int type
 * @param d2: User-defined third variable of the int type
 * @retval Indicates whether the printing is successful.
 */
int ExtDrvLogOut3(enum ExtLogLevel level, enum ExtModule modId, unsigned int id, unsigned int d0, unsigned int d1,
                  unsigned int d2)
{
    /* Check whether the value is out of range */
    if (level > EXT_LOG_LEVEL_BUTT || modId > EXT_MODULE_BUTT)
        return EXT_FAILURE;
    char buf[LOG_UINT_MAX_LEN] = { 0 };
    int len = 0;
    len = sprintf_s(buf, LOG_UINT_MAX_LEN, "%u %u %u %u\n", id, d0, d1, d2);
    if (len < 0) {
        EXT_PRINT("sprintf err\n");
        return EXT_FAILURE;
    }
    /* Process logs and determine whether to write to memory */
    return (DealLogBuf(len, level, modId, buf));
}

/**
 * @brief Setting the log level
 * @param id: Indicates the device ID of the specified level
 * @param level: Level set for the device
 * @retval Indicates whether the printing is successful
 */
int ExtDrvLogSetLogLevel(enum ExtModule id, enum ExtLogLevel level)
{
    /* Exceeded the maximum value of the storage array */
    if (level >= EXT_LOG_LEVEL_BUTT || id >= EXT_MODULE_BUTT) {
        EXT_PRINT("module or level unsupport\n");
        return EXT_FAILURE;
    }

    struct SysLogCtx *ctx = GetLogCtx();

    if (!ctx->init) {
        LogCtxInit(ctx); /* Initialize the structure */
    }
    ctx->logLevel[id] = level;
    return EXT_SUCCESS;
}

/**
 * @brief Logs are output based on different levels
 * @param level: Pre-set level
 * @param id: Indicates the device ID of the output log
 * @param fmt: character string to be output
 * @retval Indicates whether the printing is successful
 */
int ExtDrvLogOutFmt(enum ExtLogLevel level, enum ExtModule id, const char *fmt, ...)
{
    /* define value initialization */
    va_list args;

    struct SysDebugSwitch *debugSwitch = GetDebugSwitch();
    if ((!debugSwitch->enable) && (level != EXT_LOG_LEVEL_ERROR)) {
        return EXT_SUCCESS;
    }

    /* Outputs character strings by level and ID */
    if (level >= EXT_LOG_LEVEL_BUTT || id >= EXT_MODULE_BUTT) {
        EXT_PRINT("level %d or module %d err\n", level, id);
        return EXT_FAILURE;
    }

    char *tag = "FEWIDB";
    struct SysLogCtx *ctx = GetLogCtx();

    if (!ctx->init) {
        LogCtxInit(ctx); /* Initialize the structure */
    }

    if (level > ctx->logLevel[id]) {
        return EXT_SUCCESS;
    }
    EXT_PRINT("%c-%s:", *(tag + level), ctx->modStr[id]); /* Calculate the print length */

    va_start(args, fmt);
    EXT_PRINT(fmt, args);
    va_end(args);
    EXT_PRINT("\r\n");
    return EXT_SUCCESS;
}
