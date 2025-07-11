/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      cmd_common.h
  * @author    MCU Driver Team
  * @brief     cmd module driver
  * @details   The header file contains the following declaration:
  *             + cmd configuration enums.
  *             + cmd register structures.
  *             + cmd DCL Functions.
  *             + Parameters check functions.
  */
#ifndef CMD_COMMON_H
#define CMD_COMMON_H

/* Include Header Files */
#include "type.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/* Macro Definition */
#define MATCH_CMD_BUF_CNT 8

#define ARGS_NUM_MAX 16
#define CMD_BUF_MAX 128

#define CMD_NUM_MAX 5

#define DIR_KEY_HEAD (0x1b)
#define DEL ((char)255)
#define DEL7 ((char)127)

#define CTL_CH(c) ((c) - 'a' + 1)
#define CTL_CH_C 3 /* define the ctl c key */
#define CTL_CH_P 16 /* define the ctl p key */
#define CTL_CH_N 14 /* define the ctl n key */
#define CTL_BACKSPACE ('\b')
#define SPACE_KEY (' ')
#define TAB_KEY 9 /* define the tab key */
#define ENTER_KEY1 13 /* define the '\r' */
#define ENTER_KEY2 10 /* define the '\n' */

#define APP_CMD_ERR_PRINT(fmt...) EXT_ERR_PRINT(EXT_MODULE_APP_CMD, fmt)

/**
  * @addtogroup DEBUG
  * @brief DEBUG module.
  * @{
  */

/**
  * @defgroup DEBUG_Log DEBUG_Log
  * @brief DEBUG external module.
  * @{
  */

 /**
  * @defgroup CMD_COMMON_Def CMD_COMMON_Def
  * @brief Common Command Line Interface.
  * @{
  */
/**
  * @}
  */

/**
  * @brief Interprets the string of characters.
  * @param cmdStr command string
  * @argv At the command line, type a string of cosmonies
  * @retval the following is the standard
  */
unsigned int CmdParserParam(char *cmdStr, const char *argv[]);
/**
  * @brief Interprets the string of characters.
  * @param None
  * @retval None
  */
void ExtAppCmdProcess(void);
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
#endif /* __APP_COMMAND_H__ */
