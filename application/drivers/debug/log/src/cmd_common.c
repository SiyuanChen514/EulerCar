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
  * @file      cmd_common.c
  * @author    MCU Driver Team
  * @brief     cmd module driver
  * @details   The header file contains the following declaration:
  *             + total invoking of entry parameter parsing
  */
#include "cmd_common.h"

/**
 * @brief invoking of entry parameter parsing
 * @param cdmStr : single character from user
 * @param argv[] : Character string directly entered through the serial port
 * @retval pointer address of the string
 */
unsigned int CmdParserParam(char *cmdStr, const char *argv[])
{
    unsigned int nargs = 0;

    while (nargs < ARGS_NUM_MAX) {
        /* skip any white space */
        while ((*cmdStr == ' ') || (*cmdStr == '\t')) {
            ++cmdStr;
        }

        /* end of line, no more args  */
        if (*cmdStr == '\0') {
            argv[nargs] = NULL;
            return (nargs);
        }

        /* begin of argument string */
        argv[nargs++] = cmdStr;

        /* find end of string */
        while ((*cmdStr != '\0') && (*cmdStr != ' ') && (*cmdStr != '\t')) {
            ++cmdStr;
        }

        /* end of line, no more args */
        if (*cmdStr == '\0') {
            argv[nargs] = NULL;
            return (nargs);
        }

        /* terminate current arg     */
        *cmdStr++ = '\0';
    }
    return nargs;
}

