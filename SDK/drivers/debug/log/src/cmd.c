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
  * @file      cmd.c
  * @author    MCU Driver Team
  * @brief     cmd module driver
  * @details   The header file contains the following declaration:
  *             + basic register information assignment
  */
#include "cmd.h"
#include "console.h"
#include "ext_log.h"
struct cmdRegisterTable g_cmdRegister[CMD_REGESTER_MAX_NUM] = {0};

int g_cmdIndex = 0;

/**
 * @brief assign a value to the information in the RX register.
 * @param None
 * @retval the information in the RX register.
 */
struct cmdRegisterTable *GetRegisterAddr(void)
{
    return g_cmdRegister;
}

/**
 * @brief Registering a User-Defined Function
 * @param cmdName : customize a name for the implemented function.
 * @param func : pointer to the customized function.
 * @retval None
 */
void ExtCmdRegister(char *cmdName, pfncmd func)
{
    if (g_cmdIndex >= CMD_REGESTER_MAX_NUM || g_cmdIndex < 0) {
        EXT_PRINT("the number of registration commmamds has reached the maximum\n");
        return;
    }
    g_cmdRegister[g_cmdIndex].name = cmdName; /* enter the user-defined name */
    g_cmdRegister[g_cmdIndex].func = func; /* pointing a function pointer to a user-defined function */
    g_cmdIndex++;
}
