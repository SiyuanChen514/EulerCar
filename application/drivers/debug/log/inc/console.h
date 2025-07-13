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
  * @file    console.h
  * @author  MCU Driver Team
  * @brief   Header file containing functions prototypes of console module.
  *          + Initialization and de-initialization functions
  *          + Format console function
  */
#ifndef CONSOLE_H
#define CONSOLE_H

#include "uart.h"

/**
  * @addtogroup DEBUG_Log
  * @brief DEBUG external module.
  * @{
  */

 /**
  * @defgroup CONSOLE_Def CONSOLE_Def
  * @brief Serial port printing initialization.
  * @{
  */

  /**
  * @brief Read Status Query
  * @{
  */
int ConsoleGetQuery(void);
  /**
  * @brief Read a single character
  * @{
  */
int ConsoleGetc(void);
  /**
  * @brief Output String
  * @{
  */
int ConsolePuts(const char *str);
  /**
  * @brief Output Characters
  * @{
  */
void ConsolePutc(const char c);
/* Format print function */
int UartPrintf(const char *format, ...);

/* init console uart */
void ConsoleInit(UART_Handle uart);
/**
  * @}
  */

/**
  * @}
  */
#endif
