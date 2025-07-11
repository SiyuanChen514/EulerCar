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
  * @file      sample_key.c
  * @author    MCU Application Team
  * @brief     Key sample code
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the common function sample.
  *             + Key sample functions
  */

/* Includes ------------------------------------------------------------------ */
#include "sample_key.h"
#include "main.h"

/* Private macro ------------------------------------------------------------- */
#define BOARD_KEY_SAMPLE_BAUDRATE 115200

/* Private variables --------------------------------------------------------- */
void Key0LongPress(void);
void Key0ShortPress(void);
void Key1LongPress(void);
void Key1ShortPress(void);

/**
  * @brief Pin 0 long press binding functions.
  */
void Key0LongPress()
{
    DBG_PRINTF("pin0 long\r\n");
}

/**
  * @brief Pin 0 short press binding functions.
  */
void Key0ShortPress()
{
    DBG_PRINTF("pin0 short\r\n");
}

/**
  * @brief Pin 1 long press binding functions.
  */
void Key1LongPress()
{
    DBG_PRINTF("pin1 long\r\n");
}

/**
  * @brief Pin 1 short press binding functions.
  */
void Key1ShortPress()
{
    DBG_PRINTF("pin1 short\r\n");
}

/**
  * @brief Key press detection example.
  * @return int Execution result.
  */
int BOARD_KEY_Sample(void)
{
    SystemInit();
    DBG_PRINTF("BOARD KEY Sample\r\n");
    BOARD_KEY_Init(&g_timer1, 100, 1000); /* 100 : 100ms, 1000 : 1000ms */

    BOARD_KEY_Register((KEY1_HANDLE.baseAddress), KEY1_PIN, Key0LongPress, Key0ShortPress);
    BOARD_KEY_Register((KEY2_HANDLE.baseAddress), KEY2_PIN, Key1LongPress, Key1ShortPress);

    HAL_TIMER_Start(&g_timer1);
    return 0;
}