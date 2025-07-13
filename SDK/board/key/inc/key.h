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
  * @file      key.h
  * @author    MCU Application Team
  * @brief     Include the header file of the key.c file.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_KEY_H
#define McuMagicTag_KEY_H

/* Includes ------------------------------------------------------------------ */
#include "baseinc.h"
#include "timer.h"
#include "gpio.h"

#ifdef KEY_PARAM_CHECK
#define KEY_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define KEY_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define KEY_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define KEY_ASSERT_PARAM(para) ((void)0U)
#define KEY_PARAM_CHECK_NO_RET(para) ((void)0U)
#define KEY_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif
/* Typedef definitions ------------------------------------------------------- */
/**
  * @brief Key Status Definition.
  */
typedef enum {
    BOARD_KEY_NO_REGIS = 0x00000000U,
    BOARD_KEY_NO_PRESS = 0x00000001U,
    BOARD_KEY_SHORT_PRESS = 0x00000002U,
    BOARD_KEY_LONG_PRESS = 0x00000003U,
} BOARD_KEY_Status;

/**
  * @brief Function return type definition.
  */
typedef enum {
    BOARD_KEY_RET_OK = 0x00000000U,
    BOARD_KEY_RET_NO_RESOURCE = 0x00000001U,
} BOARD_KEY_Ret;

typedef void (*KeyHandleFun)(void);

/**
  * @brief Key handle.
  */
typedef struct {
    GPIO_RegStruct *gpiox; /**< GPIO group. */
    unsigned int pins; /**< GPIO pin value. */
    unsigned int cnt; /**< Key press count. */
    BOARD_KEY_Status status; /**< Current status of the key. */
    KeyHandleFun longFun; /**< Pointer to the long press binding function. */
    KeyHandleFun shortFun; /** Pointer to the short press binding function. */
} BOARD_KEY_Handle;

/* Exported global functions ------------------------------------------------- */
void BOARD_KEY_Init(TIMER_Handle *timerHandle, unsigned int longPressMs, unsigned int shortPressMs);
void BOARD_KEY_TimerCallBack(void *param);
BOARD_KEY_Ret BOARD_KEY_Register(GPIO_RegStruct *gpiox, unsigned int pins, KeyHandleFun longFun, KeyHandleFun shortFun);

#endif /* McuMagicTag_KEY_H */