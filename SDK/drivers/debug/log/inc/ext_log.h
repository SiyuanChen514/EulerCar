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
  * @file      ext_loh.h
  * @author    MCU Driver Team
  * @brief     log module driver
  * @details   The header file contains the following declaration:
  *             + Definition of the Miniaturized Log Structure
  *             + Definition of Miniaturized Log Output Functions
  */
#ifndef EXT_LOG_H
#define EXT_LOG_H

#include "module.h"
#include "console.h"
#include "file_id_defs.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

/* Serial port print definition */
#define EXT_PRINT      UartPrintf
/**
 * @brief Initializing the Log Output Level
 */
enum ExtLogLevel {
    EXT_LOG_LEVEL_FATAL,
    EXT_LOG_LEVEL_ERROR,
    EXT_LOG_LEVEL_WARNING,
    EXT_LOG_LEVEL_INFO,
    EXT_LOG_LEVEL_DBG,
    EXT_LOG_LEVEL_BUTT,
};

enum ExtLogLevelToken {
    FATAL,
    ERR,
    WARN,
    INFO,
    DBG,
};
#define MAKE_XML_ID_UINT32(a, b) ((unsigned int)(((unsigned short)(a)) | ((unsigned int)((unsigned short)(b))) << 16))

#define EXT_FATAL_PRINT(modId, fmt...)
#define EXT_ERR_PRINT(modId, fmt...) UartPrintf(fmt)
#define EXT_WARN_PRINT(modId, fmt...)
#define EXT_INFO_PRINT(modId, fmt...) UartPrintf(fmt)
#define EXT_DBG_PRINT(modId, fmt...)

#ifndef CFG_DFX_MINILOG_SUPPORT
#define CFG_DFX_MINILOG_SUPPORT 1
#endif

/**
  * @addtogroup DEBUG_Log
  * @brief DEBUG external module.
  * @{
  */

 /**
  * @defgroup EXT_LOG_Def EXT_LOG_Def
  * @brief Interface for Printing Miniaturized Logs.
  * @{
  */

/**
 * @defgroup Various types of miniaturized log output
 * @brief log output external module.
 * @{
 */
int ExtDrvLogOutBuf(enum ExtLogLevel level, enum ExtModule modId, unsigned int xmlId,
                    const unsigned int* logBuf, unsigned short logBufLen);
int ExtDrvLogOut0(enum ExtLogLevel level, enum ExtModule modId, unsigned int xmlId);
int ExtDrvLogOut1(enum ExtLogLevel level, enum ExtModule modId, unsigned int xmlId, unsigned int d0);
int ExtDrvLogOut2(enum ExtLogLevel level, enum ExtModule modId, unsigned int xmlId, unsigned int d0, unsigned int d1);
int ExtDrvLogOut3(enum ExtLogLevel level, enum ExtModule modId, unsigned int xmlId, unsigned int d0, unsigned int d1,
                  unsigned int d2);

/**
 * @defgroup Printing and outputting miniaturized logs
 * @brief log output external module.
 * @{
 */
int ExtDrvLogSetLogLevel(enum ExtModule modId, enum ExtLogLevel level);
int ExtDrvLogOutFmt(enum ExtLogLevel level, enum ExtModule id, const char *fmt, ...);

#ifndef EXT_LOG_LEVEL
#define EXT_LOG_LEVEL   EXT_LOG_LEVEL_DBG
#endif
#define EXT_LOG_0(level, modId, msg)                        LOG_##level##_0(modId, msg)
#define EXT_LOG_1(level, modId, msg, d0)                    LOG_##level##_1(modId, msg, d0)
#define EXT_LOG_2(level, modId, msg, d0, d1)                LOG_##level##_2(modId, msg, d0, d1)
#define EXT_LOG_3(level, modId, msg, d0, d1, d2)            LOG_##level##_3(modId, msg, d0, d1, d2)
#define EXT_LOG_BUF(level, modId, msg, logBuf, logBufLen)   LOG_##level##_BUF(modId, msg, logBuf, logBufLen)

#ifdef MAKE_PRIM_XML_PROCESS_IN

#define LOG_FATAL_0(modId, msg)                                                                                  \
    {                                                                                                            \
        _PRIM_ST_, _PRIM_PRI_ = 0, _PRIM_MSG_ = msg, _PRIM_LINE_ = __LINE__,                                     \
        _PRIM_FILE_ID_ = __FILE_IDX__, _PRIM_MOD_ID_ = modId, _PRIM_END_                                         \
    }
#define LOG_FATAL_1(modId, msg, d0)                         LOG_FATAL_0(modId, msg)
#define LOG_FATAL_2(modId, msg, d0, d1)                     LOG_FATAL_0(modId, msg)
#define LOG_FATAL_3(modId, msg, d0, d1, d2)                 LOG_FATAL_0(modId, msg)
#define LOG_FATAL_BUF(modId, msg, logBuf, logBufLen)        LOG_FATAL_0(modId, msg)
#define LOG_LAST_WORD_BUF(modId, msg, logBuf, logBufLen)    LOG_FATAL_0(modId, msg)


#define LOG_ERR_0(modId, msg)                                                                                    \
    {                                                                                                            \
        _PRIM_ST_, _PRIM_PRI_ = 1, _PRIM_MSG_ = msg, _PRIM_LINE_ = __LINE__,                                     \
        _PRIM_FILE_ID_ = __FILE_IDX__, _PRIM_MOD_ID_ = modId, _PRIM_END_                                         \
    }
#define LOG_ERR_1(modId, msg, d0)                           LOG_ERR_0(modId, msg)
#define LOG_ERR_2(modId, msg, d0, d1)                       LOG_ERR_0(modId, msg)
#define LOG_ERR_3(modId, msg, d0, d1, d2)                   LOG_ERR_0(modId, msg)
#define LOG_ERR_BUF(modId, msg, logBuf, logBufLen)          LOG_ERR_0(modId, msg)

#define LOG_WARN_0(modId, msg)                                                                                   \
    {                                                                                                            \
        _PRIM_ST_, _PRIM_PRI_ = 2, _PRIM_MSG_ = msg, _PRIM_LINE_ = __LINE__,                                     \
        _PRIM_FILE_ID_ = __FILE_IDX__, _PRIM_MOD_ID_ = modId, _PRIM_END_                                         \
    }
#define LOG_WARN_1(modId, msg, d0)                          LOG_WARN_0(modId, msg)
#define LOG_WARN_2(modId, msg, d0, d1)                      LOG_WARN_0(modId, msg)
#define LOG_WARN_3(modId, msg, d0, d1, d2)                  LOG_WARN_0(modId, msg)
#define LOG_WARN_BUF(modId, msg, logBuf, logBufLen)         LOG_WARN_0(modId, msg)

#define LOG_INFO_0(modId, msg)                                                                                   \
    {                                                                                                            \
        _PRIM_ST_, _PRIM_PRI_ = 3, _PRIM_MSG_ = msg, _PRIM_LINE_ = __LINE__,                                     \
        _PRIM_FILE_ID_ = __FILE_IDX__, _PRIM_MOD_ID_ = modId, _PRIM_END_                                         \
    }
#define LOG_INFO_1(modId, msg, d0)                          LOG_INFO_0(modId, msg)
#define LOG_INFO_2(modId, msg, d0, d1)                      LOG_INFO_0(modId, msg)
#define LOG_INFO_3(modId, msg, d0, d1, d2)                  LOG_INFO_0(modId, msg)
#define LOG_INFO_BUF(modId, msg, logBuf, logBufLen)         LOG_INFO_0(modId, msg)

#define LOG_DBG_0(modId, msg)                                                                                    \
    {                                                                                                            \
        _PRIM_ST_, _PRIM_PRI_ = 4, _PRIM_MSG_ = msg, _PRIM_LINE_ = __LINE__,                                     \
        _PRIM_FILE_ID_ = __FILE_IDX__, _PRIM_MOD_ID_ = modId, _PRIM_END_                                         \
    }
#define LOG_DBG_1(modId, msg, d0)                           LOG_DBG_0(modId, msg)
#define LOG_DBG_2(modId, msg, d0, d1)                       LOG_DBG_0(modId, msg)
#define LOG_DBG_3(modId, msg, d0, d1, d2)                   LOG_DBG_0(modId, msg)
#define LOG_DBG_BUF(modId, msg, logBuf, logBufLen)          LOG_DBG_0(modId, msg)

#else

#define MAKE_XML_ID_UINT32(a, b) ((unsigned int)(((unsigned short)(a)) | ((unsigned int)((unsigned short)(b))) << 16))

#define LOG_0(level, modId, msg)                                                                                 \
    ExtDrvLogOut0(level, modId, MAKE_XML_ID_UINT32(__LINE__, THIS_FILE_ID))
#define LOG_1(level, modId, msg, d0)                                                                             \
    ExtDrvLogOut1(level, modId, MAKE_XML_ID_UINT32(__LINE__, THIS_FILE_ID), d0)
#define LOG_2(level, modId, msg, d0, d1)                                                                         \
    ExtDrvLogOut2(level, modId, MAKE_XML_ID_UINT32(__LINE__, THIS_FILE_ID), d0, d1)
#define LOG_3(level, modId, msg, d0, d1, d2)                                                                     \
    ExtDrvLogOut3(level, modId, MAKE_XML_ID_UINT32(__LINE__, THIS_FILE_ID), d0, d1, d2)

#if CFG_DFX_MINILOG_SUPPORT
#define LOG_FATAL_0(modId, msg)                                                                                  \
    LOG_0(EXT_LOG_LEVEL_FATAL, modId, msg)
#define LOG_FATAL_1(modId, msg, d0)                                                                              \
    LOG_1(EXT_LOG_LEVEL_FATAL, modId, msg, d0)
#define LOG_FATAL_2(modId, msg, d0, d1)                                                                          \
    LOG_2(EXT_LOG_LEVEL_FATAL, modId, msg, d0, d1)
#define LOG_FATAL_3(modId, msg, d0, d1, d2)                                                                      \
    LOG_3(EXT_LOG_LEVEL_FATAL, modId, msg, d0, d1, d2)
#define LOG_ERR_0(modId, msg)                                                                                    \
    LOG_0(EXT_LOG_LEVEL_ERROR, modId, msg)
#define LOG_ERR_1(modId, msg, d0)                                                                                \
    LOG_1(EXT_LOG_LEVEL_ERROR, modId, msg, d0)
#define LOG_ERR_2(modId, msg, d0, d1)                                                                            \
    LOG_2(EXT_LOG_LEVEL_ERROR, modId, msg, d0, d1)
#define LOG_ERR_3(modId, msg, d0, d1, d2)                                                                        \
    LOG_3(EXT_LOG_LEVEL_ERROR, modId, msg, d0, d1, d2)
#define LOG_WARN_0(modId, msg)                                                                                   \
    LOG_0(EXT_LOG_LEVEL_WARNING, modId, msg)
#define LOG_WARN_1(modId, msg, d0)                                                                               \
    LOG_1(EXT_LOG_LEVEL_WARNING, modId, msg, d0)
#define LOG_WARN_2(modId, msg, d0, d1)                                                                           \
    LOG_2(EXT_LOG_LEVEL_WARNING, modId, msg, d0, d1)
#define LOG_WARN_3(modId, msg, d0, d1, d2)                                                                       \
    LOG_3(EXT_LOG_LEVEL_WARNING, modId, msg, d0, d1, d2)
#define LOG_INFO_0(modId, msg)                                                                                   \
    LOG_0(EXT_LOG_LEVEL_INFO, modId, msg)
#define LOG_INFO_1(modId, msg, d0)                                                                               \
    LOG_1(EXT_LOG_LEVEL_INFO, modId, msg, d0)
#define LOG_INFO_2(modId, msg, d0, d1)                                                                           \
    LOG_2(EXT_LOG_LEVEL_INFO, modId, msg, d0, d1)
#define LOG_INFO_3(modId, msg, d0, d1, d2)                                                                       \
    LOG_3(EXT_LOG_LEVEL_INFO, modId, msg, d0, d1, d2)
#define LOG_DBG_0(modId, msg)                                                                                    \
    LOG_0(EXT_LOG_LEVEL_DBG, modId, msg)
#define LOG_DBG_1(modId, msg, d0)                                                                                \
    LOG_1(EXT_LOG_LEVEL_DBG, modId, msg, d0)
#define LOG_DBG_2(modId, msg, d0, d1)                                                                            \
    LOG_2(EXT_LOG_LEVEL_DBG, modId, msg, d0, d1)
#define LOG_DBG_3(modId, msg, d0, d1, d2)                                                                        \
    LOG_3(EXT_LOG_LEVEL_DBG, modId, msg, d0, d1, d2)
#else
#define LOG_FATAL_0(modId, fmt...)  ExtDrvLogOutFmt(EXT_LOG_LEVEL_FATAL, modId, fmt)
#define LOG_FATAL_1(modId, fmt...)  LOG_FATAL_0(modId, fmt)
#define LOG_FATAL_2(modId, fmt...)  LOG_FATAL_0(modId, fmt)
#define LOG_FATAL_3(modId, fmt...)  LOG_FATAL_0(modId, fmt)
#define LOG_ERR_0(modId, fmt...)    ExtDrvLogOutFmt(EXT_LOG_LEVEL_ERROR, modId, fmt)
#define LOG_ERR_1(modId, fmt...)    LOG_ERR_0(modId, fmt)
#define LOG_ERR_2(modId, fmt...)    LOG_ERR_0(modId, fmt)
#define LOG_ERR_3(modId, fmt...)    LOG_ERR_0(modId, fmt)
#define LOG_WARN_0(modId, fmt...)   ExtDrvLogOutFmt(EXT_LOG_LEVEL_WARNING, modId, fmt)
#define LOG_WARN_1(modId, fmt...)   LOG_WARN_0(modId, fmt)
#define LOG_WARN_2(modId, fmt...)   LOG_WARN_0(modId, fmt)
#define LOG_WARN_3(modId, fmt...)   LOG_WARN_0(modId, fmt)
#define LOG_INFO_0(modId, fmt...)   ExtDrvLogOutFmt(EXT_LOG_LEVEL_INFO, modId, fmt)
#define LOG_INFO_1(modId, fmt...)   LOG_INFO_0(modId, fmt)
#define LOG_INFO_2(modId, fmt...)   LOG_INFO_0(modId, fmt)
#define LOG_INFO_3(modId, fmt...)   LOG_INFO_0(modId, fmt)
#define LOG_DBG_0(modId, fmt...)    ExtDrvLogOutFmt(EXT_LOG_LEVEL_DBG, modId, fmt)
#define LOG_DBG_1(modId, fmt...)    LOG_DBG_0(modId, fmt)
#define LOG_DBG_2(modId, fmt...)    LOG_DBG_0(modId, fmt)
#define LOG_DBG_3(modId, fmt...)    LOG_DBG_0(modId, fmt)
#endif

#define LOG_BUF(level, modId, msg, logBuf, logBufLen)                                                            \
    ExtDrvLogOutBuf(level, modId, MAKE_XML_ID_UINT32(__LINE__, THIS_FILE_ID), logBuf, logBufLen)


#define LOG_FATAL_BUF(modId, msg, logBuf, logBufLen)                                                             \
    LOG_BUF(EXT_LOG_LEVEL_FATAL, modId, msg, logBuf, logBufLen)
#define LOG_ERR_BUF(modId, msg, logBuf, logBufLen)                                                               \
    LOG_BUF(EXT_LOG_LEVEL_ERROR, modId, msg, logBuf, logBufLen)
#define LOG_WARN_BUF(modId, msg, logBuf, logBufLen)                                                              \
    LOG_BUF(EXT_LOG_LEVEL_WARNING, modId, msg, logBuf, logBufLen)
#define LOG_INFO_BUF(modId, msg, logBuf, logBufLen)                                                              \
    LOG_BUF(EXT_LOG_LEVEL_INFO, modId, msg, logBuf, logBufLen)
#define LOG_DBG_BUF(modId, msg, logBuf, logBufLen)                                                               \
    LOG_BUF(EXT_LOG_LEVEL_DBG, modId, msg, logBuf, logBufLen)

#endif

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
