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
  * @file      led.h
  * @author    MCU Application Team
  * @brief     Include the header file of the led.c file.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_LED_H
#define McuMagicTag_LED_H

/* Includes ------------------------------------------------------------------ */
#include "baseinc.h"
#include "gpio.h"
#include "timer.h"

#ifdef LED_PARAM_CHECK
#define LED_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define LED_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define LED_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define LED_ASSERT_PARAM(para) ((void)0U)
#define LED_PARAM_CHECK_NO_RET(para) ((void)0U)
#define LED_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

/* Macro definitions --------------------------------------------------------- */
#define BOARD_LED_CHAR_NUMBER 17 /**< Number of character mapping tables. */
#define BOARD_LED_PIN_NUMBER 8 /**< Number of pins. */

/* Typedef definitions ------------------------------------------------------- */
/**
  * @brief Function return type definition.
  */
typedef enum {
    BOARD_LED_OK = 0x00000000U,
    BOARD_LED_ERR_INVALID_CHAR = 0x00000001U,
    BOARD_LED_ERR_NO_CHAR = 0x00000002U,
    BOARD_LED_ERR_STRING_TOO_LONG = 0x00000003U
} BOARD_LED_Ret;

/**
  * @brief Used to identify the use of co-positive or co-negative control leds.
  */
typedef enum {
    BOARD_LED_ANODE = 0x00000000U, /**< Using common anode control. */
    BOARD_LED_CATHODE = 0x00000001U /**< Using Common Cathode Control. */
} BOARD_LED_Polarity;

/**
  * @brief GPIO group and Pin definition.
  */
typedef struct {
    GPIO_RegStruct *reg; /**< GPIO register. */
    unsigned int pin; /**< Pin value. */
} BOARD_LED_GpioInfo;

/**
  * @brief Led handle
  */
typedef struct {
    char disChar[BOARD_LED_SEG_NUM]; /**< String to be displayed. */
    BOARD_LED_Polarity polarity; /**< LED polarity. */
    BOARD_LED_GpioInfo gpioInfo[BOARD_LED_PIN_NUMBER]; /**< GPIO information. */
    BOARD_LED_GpioInfo segmentInfo[BOARD_LED_SEG_NUM]; /**< GPIO information of a segment. */
    int disIndex; /**< Index value of the currently displayed segment. */
    unsigned int disLength; /**< Display string length. */
    unsigned int atTimeStep; /**< Number of times the current time is displayed. */
    unsigned int totalTimeStep; /**< Total number of times that the time is displayed. */
} BOARD_LED_Handle;

/* Exported global functions ------------------------------------------------- */
void BOARD_LED_PinConfig(GPIO_RegStruct *gpiox, unsigned int pin, int index);
void BOARD_LED_SegmentConfig(GPIO_RegStruct *gpiox, unsigned int pin, int index);
void BOARD_LED_PolarityConfig(BOARD_LED_Polarity polarity);
void BOARD_LED_TimerCallBack(void *param);
BOARD_LED_Ret BOARD_LED_ShowString(char *str, unsigned int displayMs, TIMER_Handle *timerHandle);

#endif /* McuMagicTag_LED_H */