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
  * @file      dimming.h
  * @author    MCU Application Team
  * @brief     Include the header file of the dimming.c file.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_DIMMING_H
#define McuMagicTag_DIMMING_H

/* Includes ------------------------------------------------------------------ */
#include "baseinc.h"
#include "gpt.h"
#include "timer.h"
#include "crg.h"
#include "debug.h"

#ifdef DIM_PARAM_CHECK
#define DIM_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define DIM_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define DIM_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define DIM_ASSERT_PARAM(para) ((void)0U)
#define DIM_PARAM_CHECK_NO_RET(para) ((void)0U)
#define DIM_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif
/* Macro definitions --------------------------------------------------------- */
#define BOARD_DIM_TABLE_MAX_VALUE 65535  /**< Dimming curve max value. */
#define BOARD_DIM_TABLE_MAX_INDEX 252 /**< Maximum index value of the dimming curve. */

/* Typedef definitions ------------------------------------------------------- */
/**
  * @brief Channel Status Definition.
  */
typedef enum {
    BOARD_DIM_NOT_REGIS = 0x00000000U,
    BOARD_DIM_NOT_RUNNING = 0x00000001U,
    BOARD_DIM_IN_RUNNING = 0x00000002U,
} BOARD_DIM_Status;

/**
  * @brief Function return type definition.
  */
typedef enum {
    BOARD_DIM_RET_OK = 0x00000000U,
    BOARD_DIM_RET_INVALID_PARAM = 0x00000001U,
    BOARD_DIM_RET_ERR_NOT_REGIS = 0x00000002U,
} BOARD_DIM_Ret;

/**
  * @brief Dimming Handle.
  */
typedef struct {
    BOARD_DIM_Status status; /**< Status of the current handle. */
    GPT_Handle *gptHandle; /**< GPT Handle for dimming. */
    unsigned int nowIndex; /**< Current index value of the dimming curve. */
    unsigned int targetIndex; /**< Dimming curve target index value. */
} BOARD_DIM_Handle;

/* Exported global functions ------------------------------------------------- */
void BOARD_DIM_Init(GPT_Handle *gptHandle, TIMER_Handle *timerHandle, int index, int targetTimeMs);
void BOARD_DIM_DeInit(int index);
BOARD_DIM_Ret BOARD_DIM_SetDuty(int index, int targetLevel);
void BOARD_DIM_TimerCallBack(void *param);

#endif /* McuMagicTag_DIMMING_H */