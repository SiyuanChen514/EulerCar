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
  * @file    sample_uart_character_match.c
  * @author  MCU Driver Team
  * @brief   uart sample module.
  * @details This file provides sample code for users to help use
  *          the character match of uart.
  */

#include "sample_uart_character_match.h"

#define MATCH_CHARACTER_LENGTH 10
#define WAIT_TIMER 10

unsigned char save_data[MATCH_CHARACTER_LENGTH];

/**
  * @brief UART read interrupt finish.
  * @param None.
  * @retval None.
  */
void ReadFinish(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("Receive Characters: %s", save_data);
}

/**
  * @brief UART charter match
  * @param None.
  * @retval None.
  */
void UART_CharacterMatchCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("Match character success\r\n");
}

/**
  * @brief UART charter match.
  * @param None.
  * @retval None.
  */
void UART_CharacterMatch(void)
{
    SystemInit();
    /* Message */
    DBG_PRINTF("Match start! Send 10 characters containing the uppercase character A.\r\n");
    HAL_UART_ReadIT(&g_uart, save_data, MATCH_CHARACTER_LENGTH);  /* Read interrupt callback. */
    BASE_FUNC_DELAY_MS(WAIT_TIMER);
    return;
}