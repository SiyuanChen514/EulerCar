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
  * @file      ext_command.c
  * @author    MCU Driver Team
  * @brief     command module driver
  * @details   The header file contains the following declaration:
  *             + Mainly including query commands.
  *             + Mainly including matching command.
  */
#include <stdio.h>
#include <string.h>
#include "command.h"
#include "cmd.h"
#include "common.h"
#define USER_COMMAND_START  0x50000
#define USER_COMMAND_END    0x60000

/**
 * @brief Commands contained in the query string.
 * @param cmd: Contains command information.
 * @retval Indicates whether the query is successful.
 */
struct cmdRegisterTable *ExtCmdFindCmd(const char *cmd)
{
    struct cmdRegisterTable *cmdtp = GetRegisterAddr();
    const char *p = NULL;

    unsigned int tblLen = CMD_REGESTER_MAX_NUM;
    unsigned int cmdLen;

    if (tblLen == 0 || cmd == NULL) {
        return NULL;
    }

    /* compare command name only until first dot */
    p = strchr(cmd, '.');
    cmdLen = (p == NULL) ? (unsigned char)strlen(cmd) : (unsigned char)(p - cmd);

    for (int i = 0; i < CMD_REGESTER_MAX_NUM; i++, cmdtp++) {
        if (cmdtp->name == NULL) {
            return NULL;
        }

        if ((p != NULL) && (cmdLen != 0)) {
            if (strncmp(cmd, cmdtp->name, cmdLen) == 0) {
                return cmdtp; /* only match part before dot */
            }
        }

        if (strcmp(cmd, cmdtp->name) == 0) {
            return cmdtp; /* full match */
        }
    }
    return NULL; /* not found */
}

/**
 * @brief Matches valid commands based on included commands.
 * @param head: Command to be queried.
 * @param resLen: Length of the string to be found.
 * @param finCnt: Set the number of times to be searched.
 * @param tailId: End Flag Character
 * @retval Indicates whether the query is successful.
 */
static unsigned char IsFindMatchCmdParamLegal(const char *head, unsigned char resLen, unsigned char *findCnt,
                                              const char *res[])
{
    if (head == NULL || resLen == 0 || findCnt == NULL || res == NULL) { return EXT_FALSE; }
    return EXT_TRUE;
}

/**
 * @brief Matches valid commands based on included commands.
 * @param head: Command to be queried.
 * @param *res[]: An array that temporarily stores strings.
 * @param resLen: Length of the string to be found.
 * @param finCnt: Set the number of times to be searched.
 * @param tailId: End Flag Character
 * @retval Indicates whether the query is successful.
 */
unsigned char ExtCmdFindMatchCmd(const char *head, const char *res[], unsigned char resLen, unsigned char *findCnt,
                                 unsigned int *tailId)
{
    /* Define Value Initialization */
    unsigned char ret = EXT_TRUE;
    unsigned int cmdId = 0;
    size_t headLen = 0;
    /* initialization structure */
    struct cmdRegisterTable *cmdtp = GetRegisterAddr();

    if (!IsFindMatchCmdParamLegal(head, resLen, findCnt, res)) { return EXT_FALSE; }
    headLen = strlen(head);
    for (int i = 0; i < CMD_REGESTER_MAX_NUM; cmdtp++, cmdId++, i++) {
        if (cmdtp->name == NULL) { break; } /* search finish */
        if (*findCnt >= resLen) { /* search not finish */
            ret = EXT_FALSE;
            break;
        }

        /* detect registered name */
        if ((*tailId > 0 && (unsigned int)cmdId < *tailId) || strlen(cmdtp->name) < (unsigned int)headLen
            || strcmp(cmdtp->name, UART_SWITCH_CMD) == 0) { continue; }

        if (strncmp(head, cmdtp->name, headLen) == 0) { res[(*findCnt)++] = cmdtp->name; }
    }
    *tailId = cmdId;
    return ret;
}