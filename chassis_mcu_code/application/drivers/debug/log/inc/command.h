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
  * @file    command.h
  * @author  MCU Driver Team
  * @brief   Header file containing functions prototypes of DEBUG module.
  *          + Initialization and de-initialization functions
  *          + Format command function
  */
#ifndef COMMAND_H
#define COMMAND_H

#include "cmd.h"

#define UART_SWITCH_CMD "soct_pq_tool"

#define CMD_SECTION __attribute__((unused, section(".command")))

struct CmdTable {
    const char  *name; /* Command Name */
    int (*pfncmd)(unsigned int argc, const char *argv[]);
};

#define CMD_REGESTER(name, cmd) \
    struct CmdTable __cmd_##name CMD_SECTION = { #name, cmd }

/**
  * @addtogroup DEBUG_Log
  * @brief DEBUG external module.
  * @{
  */

 /**
  * @defgroup COMMAND_Def COMMAND_Def
  * @brief Command processing.
  * @{
  */

/**
 * @brief use cmd line to find cmd
 * @attention None
 *
 * @param str [IN] command character string carried in the command line
 * @retval struct cmdRegisterTable *
 */
struct cmdRegisterTable *ExtCmdFindCmd(const char *str);

/**
 * @brief use cmd line to match cmd read
 * @attention None
 * @param head [IN] enter a portion of the complete command you want at the command line
 * @param res [out] the string array is used to store all matching strings
 * @param len [IN] length of the string array
 * @param findCnt [out] Number of matched strings
 * @param tailId [out] Record the location of the last search
 * @retval unsigned char Whether the matching is complete.
 * If the matching is successful, true is returned. If the matching fails, false is returned
 */
unsigned char ExtCmdFindMatchCmd(const char *head, const char *res[], unsigned char resLen, unsigned char *findCnt,
                                 unsigned int *tailId);
#define UESR_CMD_SECTION __attribute__((unused, section(".user_command")))

struct UserCmdTable {
    unsigned short cmd; /* Command ID */
    int (*pfnUserMCUCmd)(unsigned char len, unsigned char* param);
};

#define USRER_CMD_REGESTER(pfn, cmd) \
    struct UserCmdTable __user_cmd_##pfn USER_CMD_SECTION = { cmd, pfn }
/**
  * @}
  */

/**
  * @}
  */
#endif
