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
  * @file    cmd.h
  * @author  MCU Driver Team
  * @brief   Header file containing functions prototypes of DEBUG module.
  *          + Initialization and de-initialization functions
  *          + Format cmd function
  */
#ifndef CMD_H
#define CMD_H

#include "module.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

#ifndef CMD_REGESTER_MAX_NUM
#define CMD_REGESTER_MAX_NUM        128 // The maximum length of the command
#endif

/**
  * @addtogroup DEBUG_Log
  * @brief DEBUG external module.
  * @{
  */

 /**
  * @defgroup CMD_Def CMD_Def
  * @brief Command line registration initialization.
  * @{
  */

/* Defines a function pointer to command registered function */
typedef int (*pfncmd)(unsigned int argc, const char *argv[]);
/* defines the structure required for registering a function */
struct cmdRegisterTable {
    char *name;
    pfncmd func;
};

/**
 * @brief cmd_regester
 * @attention None
 *
 * @param cmdName [IN] registration name, which is a character string
 * @param func [IN] register the function corresponding to the name
 * @retval void None
 */
void ExtCmdRegister(char *cmdName, pfncmd func);

/**
 * @brief get regester address
 * @attention None
 *
 * @retval struct cmdRegisterTable *
 */
struct cmdRegisterTable *GetRegisterAddr(void);

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
