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
  * @file      matrixkey.h
  * @author    MCU Application Team
  * @brief     Include the header file of the matrixkey.c file.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_MATRIXKEY_H
#define McuMagicTag_MATRIXKEY_H

/* Includes ------------------------------------------------------------------ */
#include "baseinc.h"
#include "timer.h"
#include "gpio.h"

#ifdef MKEY_PARAM_CHECK
#define MKEY_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define MKEY_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define MKEY_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define MKEY_ASSERT_PARAM(para) ((void)0U)
#define MKEY_PARAM_CHECK_NO_RET(para) ((void)0U)
#define MKEY_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

/* Macro definitions --------------------------------------------------------- */
/** Solution 1 : Proactively process keystroke events. The interrupt processing \
    function scans the I/O port. When the key press tick reaches the press \
    threshold, the key registration function is invoked in the interrupt. This \
    function applies to scenarios where the key event execution time is short. \
    Advantage: It is easy to invoke. When the while loop in main is long, this \
    method responds to keys in real time. */
#define BOARD_MKEY_SCHEME_NUMBER_ONE 0
/** Solution 2: Passively process keystroke events. The interrupt processing \
    function scans I/O ports and counts ticks. The main function polls tick \
    values in the while loop. When the threshold is reached, the registration \
    function is called. Advantage: The interrupt function processing period is \
    reduced. */
#define BOARD_MKEY_SCHEME_NUMBER_TWO 1

/* Typedef definitions ------------------------------------------------------- */
/**
  * @brief Function return type definition.
  */
typedef enum {
    BOARD_MKEY_RET_OK = 0x00000000U,
    BOARD_MKEY_RET_INVALID_PARAM = 0x00000001U
} BOARD_MKEY_RET;

/**
  * @brief GPIO group and Pin definition.
  */
typedef struct {
    GPIO_RegStruct *gpiox; /**< GPIO register. */
    unsigned int pins; /**< Pin value. */
} GPIO_MKEY_PinInfo;

typedef void (*MKeyHandleFun)(void);

/**
  * @brief Key attribute definition.
  */
typedef struct {
    unsigned int tick; /**< Current key press count. */
    MKeyHandleFun fun; /**< Function pointer bound when a key event is triggered. */
} GPIO_MKEY_KeyInfo;

/**
  * @brief Matrix key handle.
  */
typedef struct {
    GPIO_MKEY_PinInfo outPins[BOARD_MKEY_OUT_NUM]; /**< GPIO pin used as output during scanning. */
    GPIO_MKEY_PinInfo inPins[BOARD_MKEY_IN_NUM]; /**< GPIO pin used as input during scanning. */
    GPIO_MKEY_KeyInfo keyInfo[BOARD_MKEY_OUT_NUM * BOARD_MKEY_IN_NUM]; /**< Stores information about each key. */
} BOARD_MKEY_Handle;

/* Exported global functions ------------------------------------------------- */
void BOARD_MKEY_Init(TIMER_Handle *timerHandle, unsigned int validPressTimeMs);
BOARD_MKEY_RET BOARD_MKEY_ConfigOutputPin(GPIO_RegStruct *gpiox, unsigned int pins, unsigned int index);
BOARD_MKEY_RET BOARD_MKEY_ConfigInputPin(GPIO_RegStruct *gpiox, unsigned int pins, unsigned int index);
BOARD_MKEY_RET BOARD_MKEY_RegisterKeyFun(unsigned int keyNum, MKeyHandleFun fun);
void BOARD_MKEY_TimerCallBack(void *param);
#if (BOARD_MKEY_SCHEME_NUMBER == BOARD_MKEY_SCHEME_NUMBER_TWO)
void BOARD_MKEY_SCAN_KEY(void);
#endif

#endif /* McuMagicTag_MATRIXKEY_H */