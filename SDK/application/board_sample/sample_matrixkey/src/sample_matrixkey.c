/**
  * @ Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2023. All rights reserved.
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
  * @file      sample_matrixkey.c
  * @author    MCU Application Team
  * @brief     Matrix key sample code
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the common function sample.
  *            + Matrix key sample functions.
  */

/* Includes ------------------------------------------------------------------ */
#include "sample_matrixkey.h"

/* Private macro ------------------------------------------------------------- */
#define BOARD_MATRIXKEY_SAMPLE_BAUDRATE 115200

/* Private variables --------------------------------------------------------- */
void MKey1Func(void);
void MKey2Func(void);
void MKey3Func(void);
void MKey4Func(void);
void MKey5Func(void);
void MKey6Func(void);
void MKey7Func(void);
void MKey8Func(void);
void MKey9Func(void);
void MKey10Func(void);
void MKey11Func(void);
void MKey12Func(void);
void MKey13Func(void);
void MKey14Func(void);
void MKey15Func(void);
void MKey16Func(void);

/**
  * @brief Key 1 binding function.
  */
void MKey1Func()
{
    DBG_PRINTF("key1 press\r\n");
}

/**
  * @brief Key 2 binding function.
  */
void MKey2Func()
{
    DBG_PRINTF("key2 press\r\n");
}
/**
  * @brief Key 3 binding function.
  */
void MKey3Func()
{
    DBG_PRINTF("key3 press\r\n");
}

/**
  * @brief Key 4 binding function.
  */
void MKey4Func()
{
    DBG_PRINTF("key4 press\r\n");
}

/**
  * @brief Key 5 binding function.
  */
void MKey5Func()
{
    DBG_PRINTF("key5 press\r\n");
}

/**
  * @brief Key 6 binding function.
  */
void MKey6Func()
{
    DBG_PRINTF("key6 press\r\n");
}

/**
  * @brief Key 7 binding function.
  */
void MKey7Func()
{
    DBG_PRINTF("key7 press\r\n");
}

/**
  * @brief Key 8 binding function.
  */
void MKey8Func()
{
    DBG_PRINTF("key8 press\r\n");
}

/**
  * @brief Key 9 binding function.
  */
void MKey9Func()
{
    DBG_PRINTF("key9 press\r\n");
}

/**
  * @brief Key 10 binding function.
  */
void MKey10Func()
{
    DBG_PRINTF("key10 press\r\n");
}

/**
  * @brief Key 11 binding function.
  */
void MKey11Func()
{
    DBG_PRINTF("key11 press\r\n");
}

/**
  * @brief Key 12 binding function.
  */
void MKey12Func()
{
    DBG_PRINTF("key12 press\r\n");
}

/**
  * @brief Key 13 binding function.
  */
void MKey13Func()
{
    DBG_PRINTF("key13 press\r\n");
}

/**
  * @brief Key 14 binding function.
  */
void MKey14Func()
{
    DBG_PRINTF("key14 press\r\n");
}

/**
  * @brief Key 15 binding function.
  */
void MKey15Func()
{
    DBG_PRINTF("key15 press\r\n");
}

/**
  * @brief Key 16 binding function.
  */
void MKey16Func()
{
    DBG_PRINTF("key16 press\r\n");
}

/**
  * @brief Matrix keys sample.
  * @return int Execution result.
  */
int BOARD_MATRIXKEY_Sample(void)
{
    SystemInit();
    DBG_PRINTF("Board matrixkey sample! \r\n");

    BOARD_MKEY_Init(&g_timer1, 100); /* 100 : 100ms. */

    BOARD_MKEY_ConfigOutputPin((ROW0_HANDLE.baseAddress), ROW0_PIN, 0); /* 0 : index value. */
    BOARD_MKEY_ConfigOutputPin((ROW1_HANDLE.baseAddress), ROW1_PIN, 1); /* 1 : index value. */
    BOARD_MKEY_ConfigOutputPin((ROW2_HANDLE.baseAddress), ROW2_PIN, 2); /* 2 : index value. */
    BOARD_MKEY_ConfigOutputPin((ROW3_HANDLE.baseAddress), ROW3_PIN, 3); /* 3 : index value. */

    BOARD_MKEY_ConfigInputPin((COLUMN0_HANDLE.baseAddress), COLUMN0_PIN, 0); /* 0 : index value. */
    BOARD_MKEY_ConfigInputPin((COLUMN1_HANDLE.baseAddress), COLUMN1_PIN, 1); /* 1 : index value. */
    BOARD_MKEY_ConfigInputPin((COLUMN2_HANDLE.baseAddress), COLUMN2_PIN, 2); /* 2 : index value. */
    BOARD_MKEY_ConfigInputPin((COLUMN3_HANDLE.baseAddress), COLUMN3_PIN, 3); /* 3 : index value. */

    BOARD_MKEY_RegisterKeyFun(0, MKey1Func); /* Index 0 : Key1 callback function. */
    BOARD_MKEY_RegisterKeyFun(1, MKey2Func); /* Index 1 : Key2 callback function. */
    BOARD_MKEY_RegisterKeyFun(2, MKey3Func); /* Index 2 : Key3 callback function. */
    BOARD_MKEY_RegisterKeyFun(3, MKey4Func); /* Index 3 : Key4 callback function. */
    BOARD_MKEY_RegisterKeyFun(4, MKey5Func); /* Index 4 : Key5 callback function. */
    BOARD_MKEY_RegisterKeyFun(5, MKey6Func); /* Index 5 : Key6 callback function. */
    BOARD_MKEY_RegisterKeyFun(6, MKey7Func); /* Index 6 : Key7 callback function. */
    BOARD_MKEY_RegisterKeyFun(7, MKey8Func); /* Index 7 : Key8 callback function. */
    BOARD_MKEY_RegisterKeyFun(8, MKey9Func); /* Index 8 : Key9 callback function. */
    BOARD_MKEY_RegisterKeyFun(9, MKey10Func); /* Index 9 : Key10 callback function. */
    BOARD_MKEY_RegisterKeyFun(10, MKey11Func); /* Index 10 : Key11 callback function. */
    BOARD_MKEY_RegisterKeyFun(11, MKey12Func); /* Index 11 : Key12 callback function. */
    BOARD_MKEY_RegisterKeyFun(12, MKey13Func); /* Index 12 : Key13 callback function. */
    BOARD_MKEY_RegisterKeyFun(13, MKey14Func); /* Index 13 : Key14 callback function. */
    BOARD_MKEY_RegisterKeyFun(14, MKey15Func); /* Index 14 : Key15 callback function. */
    BOARD_MKEY_RegisterKeyFun(15, MKey16Func); /* Index 15 : Key16 callback function. */

    HAL_TIMER_Start(&g_timer1);
    while (1) {
#if (BOARD_MKEY_SCHEME_NUMBER == BOARD_MKEY_SCHEME_NUMBER_TWO)
        BOARD_MKEY_SCAN_KEY();
#endif
    }

    return 0;
}