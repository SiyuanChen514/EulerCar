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
  * @file      pulses.h
  * @author    MCU Application Team
  * @brief     Include the header file of the pulses.c file.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_PULSES_H
#define McuMagicTag_PULSES_H

/* Includes ------------------------------------------------------------------ */
#include "baseinc.h"
#include "gpio.h"
#include "timer.h"

#ifdef PULSES_PARAM_CHECK
#define PULSES_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define PULSES_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define PULSES_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define PULSES_ASSERT_PARAM(para) ((void)0U)
#define PULSES_PARAM_CHECK_NO_RET(para) ((void)0U)
#define PULSES_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

/* Typedef definitions ------------------------------------------------------- */
/**
  * @brief Pulse Handle.
  */
typedef struct {
    unsigned int status; /**< Status of the current handle pulse. */
    unsigned int highCnt; /**< Cycle value of continuous high-level output. */
    unsigned int lowCnt; /**< Cycle value of continuous low-level output. */
    unsigned int pins; /**< Pin value for outputting pulses. */
    unsigned int value; /**< Level status of the current handle pulse output. */
    GPIO_RegStruct *gpiox; /**< GPIO group for pulse generation. */
    TIMER_Handle *timerHandle; /**< Timer handle for generating pulses. */
} BOARD_PULSES_Handle;

/**
  * @brief Function return type definition.
  */
typedef enum {
    BOARD_PULSES_OK = 0x00000000U,
    BOARD_PULSES_ERR_PARAM_INVALID = 0x00000001U,
    BOARD_PULSES_ERR_NO_RESOURCE = 0x00000002U,
    BOARD_PULSES_ERR_REPEATED_STARTUP = 0x00000003U,
    BOARD_PULSES_ERR_NOT_RUN = 0x00000004U
} BOARD_PULSES_Ret;

/**
  * @brief Pulse output status definition.
  */
typedef enum {
    BOARD_PULSES_STATUS_STOP = 0x00000000U,
    BOARD_PULSES_STATUS_START = 0x00000001U,
} BOARD_PULSES_Status;

/* Exported global functions ------------------------------------------------- */
void BOARD_PULSES_Init(GPIO_Handle *gpioHandle, TIMER_Handle *timerHandle, float period,
                       float dutyCycle, int index);
BOARD_PULSES_Ret BOARD_PULSES_Deinit(int index);
BOARD_PULSES_Ret BOARD_PULSES_Start(int index);
BOARD_PULSES_Ret BOARD_PULSES_Stop(int index);
void BOARD_PULSES_TimerCallBack(void *param);

#endif /* McuMagicTag_PULSES_H */