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
  * @file      log.h
  * @author    MCU Driver Team
  * @brief     log module driver
  * @details   The header file contains the following declaration:
  *             + Definition of log level settings for miniaturization.
  *             + Output of miniaturized logs based on different conditions.
  */
#ifndef LOG_H
#define LOG_H

#include "ext_log.h"
#include "module.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

/**
  * @addtogroup DEBUG_Log
  * @brief DEBUG external module.
  * @{
  */

 /**
  * @defgroup LOG_Def LOG_Def
  * @brief Printing miniaturized logs.
  * @{
  */

/**
 * @brief Set Log Level.
 * @attention None
 *
 * @param id [IN] ID of the module whose log level is to be set, which is defined by ExtModule.
 * @param logLevel [IN] Log level, which is defined by ExtLogLevel.
 * @retval int Whether the setting is successful
 */
int ExtSetLogLevel(enum ExtModule id, enum ExtLogLevel logLevel);

/**
 * @brief Used to report the content of a specified buffer to the message interface of the PC tool.
 * @attention None
 *
 * @param logLevel [IN] Log level, which is defined by ExtLogLevel.
 * @param modId [IN] Module ID, which is planned in advance based on the service scenario.
 * @param msg [IN] The value is a character string constant and does not support carriage returns.
 * The current version does not support parameter formatting. Only the character string is displayed.
 * @param log_buf[IN] Log buffer
 * @param log_buf_len[IN] Length of the log buffer, in bytes
 *
 * @retval None
 */
#define ExtLogBuf(logLevel, modId, msg, logBuf, logBufLen) EXT_LOG_BUF(logLevel, modId, msg, logBuf, logBufLen)

/**
 * @brief extLog0,Output logs without variables
 * @attention None
 *
 * @param logLevel [IN] Log level, which is defined by ExtLogLevel
 * @param modId [IN] Module ID, which is planned in advance based on the service scenario.
 * @param msg [IN] The value is a character string constant and does not support carriage returns.
 * The current version does not support parameter formatting. Only the character string is displayed.
 *
 * @retval None
 */
#define ExtLog0(logLevel, modId, msg) EXT_LOG_0(logLevel, modId, msg)

/**
 * @brief Logs with one int value are output
 * @attention None
 *
 * @param logLevel [IN] Log level, which is defined by ExtLogLevel
 * @param modId [IN] Module ID, which is planned in advance based on the service scenario.
 * @param msg [IN] The value is a character string constant and does not support carriage returns.
 * The current version does not support parameter formatting. Only the character string is displayed.
 * @param d0 [IN] Variable of the unsigned int type
 *
 * @retval None
 */
#define ExtLog1(logLevel, modId, msg, d0) EXT_LOG_1(logLevel, modId, msg, d0)

/**
 * @brief Logs with two int values are output.
 * @attention None
 *
 * @param logLevel [IN] Log level, which is defined by ExtLogLevel
 * @param modId [IN] Module ID, which is planned in advance based on the service scenario.
 * @param msg [IN] The value is a character string constant and does not support carriage returns.
 * The current version does not support parameter formatting. Only the character string is displayed.
 * @param d0 [IN] Variable of the unsigned int type
 * @param d1 [IN] Variable of the unsigned int type
 *
 * @retval None
 */
#define ExtLog2(logLevel, modId, msg, d0, d1) EXT_LOG_2(logLevel, modId, msg, d0, d1)

/**
 * @brief Logs with three int values are output
 * @attention None
 *
 * @param logLevel [IN] Log level, which is defined by ExtLogLevel
 * @param modId [IN] Module ID, which is planned in advance based on the service scenario.
 * @param msg [IN] The value is a character string constant and does not support carriage returns.
 * The current version does not support parameter formatting. Only the character string is displayed.
 * @param d0 [IN] Variable of the unsigned int type
 * @param d1 [IN] Variable of the unsigned int type
 * @param d2 [IN] Variable of the unsigned int type
 *
 * @retval None
 */
#define ExtLog3(logLevel, modId, msg, d0, d1, d2)   EXT_LOG_3(logLevel, modId, msg, d0, d1, d2)

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __EXT_DEBUG_H__ */
