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
  * @file      key.c
  * @author    MCU Application Team
  * @brief     key module application.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the dimming.
  *             + Initialization and de-initialization functions
  *             + Key event registration function
  * @verbatim
  * usage:
  * 1) Use the chip config tool to generate the timer initialization code.
  * 2) Call the BOARD_KEY_Init() function to select the specified TIMER and setting the effective press time.
  * 3) Call the BOARD_KEY_Register() function to set GPIO information about keys and associated key events.
  * 4) Start the corresponding TIMER to monitor the key press status.
  * @endverbatim
  */

/* Includes ------------------------------------------------------------------ */
#include "key.h"

/* Private variables --------------------------------------------------------- */
BOARD_KEY_Handle g_keyInfo[BOARD_KEY_NUM]; /**< Key handles array. */
static unsigned int g_keyLongCnt; /**< Valid cycle value of the long press. */
static unsigned int g_keyShortCnt; /**< Valid cycle value of the short press. */
/* Function declare  --------------------------------------------------------- */
static void KEY_Process(unsigned int index);
/**
  * @brief Initial configuration, configure the timer, validity duration of a long press and a short press.
  * @param timerHandle Timer Handle.
  * @param shortPressMs Short press validity period, in milliseconds.
  * @param longPressMs Press and hold the validity period, in milliseconds.
  */
void BOARD_KEY_Init(TIMER_Handle *timerHandle, unsigned int shortPressMs, unsigned int longPressMs)
{
    KEY_ASSERT_PARAM(timerHandle != NULL);

    g_keyLongCnt = longPressMs / 10;   /* 10: 10ms interrupt. */
    g_keyShortCnt = shortPressMs / 10; /* 10: 10ms interrupt. */

    unsigned int loadVal = HAL_CRG_GetIpFreq((void *)timerHandle->baseAddress) / 100; /* 100 : get cycles in 10 ms. */
    timerHandle->load = loadVal;
    timerHandle->bgLoad = loadVal;
    HAL_TIMER_Config(timerHandle, TIMER_CFG_LOAD);
    HAL_TIMER_Config(timerHandle, TIMER_CFG_BGLOAD);
    HAL_TIMER_RegisterCallback(timerHandle, TIMER_PERIOD_FIN, BOARD_KEY_TimerCallBack);
}

/**
  * @brief Key Process function, get key's status and execute func.
  * @param index key's index.
  * @return None.
  */
static void KEY_Process(unsigned int index)
{
    if (g_keyInfo[index].status == BOARD_KEY_NO_REGIS) {
        return;
    }
    /* Get the key status and determine whether the key is pressed. */
    if (DCL_GPIO_GetPinValue(g_keyInfo[index].gpiox, g_keyInfo[index].pins) != BOARD_KEY_PRESS_OFF) {
        g_keyInfo[index].cnt++;
        if (g_keyInfo[index].cnt >= g_keyLongCnt) {
            g_keyInfo[index].status = BOARD_KEY_LONG_PRESS;
        } else if (g_keyInfo[index].cnt >= g_keyShortCnt) { /* This is the short press state. */
            g_keyInfo[index].status = BOARD_KEY_SHORT_PRESS;
        }
        return;
    }
    g_keyInfo[index].cnt = 0; /* Clear the count and call the callback function. */
    if (g_keyInfo[index].status == BOARD_KEY_LONG_PRESS && g_keyInfo[index].longFun) {
        g_keyInfo[index].longFun();
    } else if (g_keyInfo[index].status == BOARD_KEY_SHORT_PRESS && g_keyInfo[index].shortFun) {
        g_keyInfo[index].shortFun(); /* Invoke the short press callback function. */
    }
    g_keyInfo[index].status = BOARD_KEY_NO_PRESS;
}

/**
  * @brief Timer callback function, Determines the status of each key and triggers the corresponding event.
  * @param irqNum Interrupt number.
  * @param param Pointer to the timer handle.
  */
void BOARD_KEY_TimerCallBack(void *param)
{
    BASE_FUNC_UNUSED(param);

    for (unsigned int index = 0; index < BOARD_KEY_NUM; index++) {
        KEY_Process(index);
    }
}

/**
  * @brief Key registration function, set GPIO, and bind the long-press and short-press functions.
  * @param gpiox GPIO group: GPIO0, GPIO1, GPIO2...
  * @param pins Specific pin value.
  * @param longFun Pointer to the touch and hold event function.
  * @param shortFun Pointer to the short press event function.
  * @return BOARD_KEY_Ret @ref BOARD_KEY_Ret.
  */
BOARD_KEY_Ret BOARD_KEY_Register(GPIO_RegStruct *gpiox, unsigned int pins, KeyHandleFun longFun, KeyHandleFun shortFun)
{
    KEY_ASSERT_PARAM(gpiox != NULL);

    int index;
    for (index = 0; index < BOARD_KEY_NUM; index++) { /* Search for keys that have not been registered. */
        if (g_keyInfo[index].status == BOARD_KEY_NO_REGIS) {
            break;
        }
    }

    if (index == BOARD_KEY_NUM) {
        return BOARD_KEY_RET_NO_RESOURCE;
    }
     /* Setting parameters related to keys. */
    g_keyInfo[index].gpiox = gpiox;
    g_keyInfo[index].pins = pins;
    g_keyInfo[index].status = BOARD_KEY_NO_PRESS;
    g_keyInfo[index].cnt = 0;
    g_keyInfo[index].longFun = longFun;
    g_keyInfo[index].shortFun = shortFun;

    DCL_GPIO_SetDirection(gpiox, pins, GPIO_INPUT_MODE); /* Set input mode. */

    return BOARD_KEY_RET_OK;
}