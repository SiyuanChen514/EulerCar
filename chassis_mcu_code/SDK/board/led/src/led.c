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
  * @file      led.c
  * @author    MCU Application Team
  * @brief     LED digital tube module application.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the digital tube.
  *             + Digital tube configuration function
  *             + Digital tube display string function
  * @verbatim
  * usage:
  * 1) Use the chip config tool to generate the timer initialization code.
  * 2) Call the BOARD_LED_PinConfig() function and BOARD_LED_SegmentConfig() function to configure Pin and Segment.
  * 3) Call the BOARD_LED_ShowString() function to display string.
  * The pin numbers of the digital tube are as follows:
  *     a
  *  -------
  * f|     |b
  *  |  g  |
  *  -------
  *  |     |
  * e|     |c
  *  ------- .h
  *     d
  * Corresponding relationship is as follows:
  * gpioInfo[0] = a, gpioInfo[1] = b ...
  * @endverbatim
  */

/* Includes ------------------------------------------------------------------ */
#include "led.h"
/* Private variables --------------------------------------------------------- */
BOARD_LED_Handle g_ledHandle; /**< Led handle array. */
const unsigned char g_ledTable[BOARD_LED_CHAR_NUMBER] = {0x03, 0x9F, 0x25, 0x0D, 0x99, 0x49, 0x41, 0x1F, 0x01, 0x09,
                                                         0x11, 0xC1, 0x63, 0x85, 0x61, 0x71, 0xFD};
/**< character mapping table */
/* Function declare --------------------------------------------------------- */
static BOARD_LED_Ret DisplayCharIndexProcess(char *str);

/**
  * @brief GPIO configuration function of the digital tube.
  * @param gpiox GPIO group: GPIO0, GPIO1, GPIO2...
  * @param pin Specific pin value.
  * @param index Specifies the index value, in [0, BOARD_LED_PIN_NUMBER).
  */
void BOARD_LED_PinConfig(GPIO_RegStruct *gpiox, unsigned int pin, int index)
{
    g_ledHandle.gpioInfo[index].reg = gpiox;
    g_ledHandle.gpioInfo[index].pin = pin;
    DCL_GPIO_SetDirection(gpiox, pin, GPIO_OUTPUT_MODE);
    DCL_GPIO_SetValue(gpiox, pin, GPIO_LOW_LEVEL);
}

/**
  * @brief Segment configuration function of the digital tube.
  * @param gpiox GPIO group: GPIO0, GPIO1, GPIO2...
  * @param pin Specific pin value.
  * @param index Specifies the index value, in [0, BOARD_LED_SEG_NUM).
  */
void BOARD_LED_SegmentConfig(GPIO_RegStruct *gpiox, unsigned int pin, int index)
{
    g_ledHandle.segmentInfo[index].reg = gpiox;
    g_ledHandle.segmentInfo[index].pin = pin;
    DCL_GPIO_SetDirection(gpiox, pin, GPIO_OUTPUT_MODE);
    DCL_GPIO_SetValue(gpiox, pin, BOARD_LED_SEGMENT_OFF);
}

/**
  * @brief Configuring the polarity of the digital tube.
  * @param polarity @ref BOARD_LED_Polarity, Anodic or Cathode.
  */
void BOARD_LED_PolarityConfig(BOARD_LED_Polarity polarity)
{
    g_ledHandle.polarity = polarity;
}

/**
  * @brief Clear the output of the digital tube and segment.
  */
static void BOARD_LED_ClearPinAndSeg(void)
{
    unsigned int disIndex = g_ledHandle.disIndex;
    GPIO_RegStruct *gpiox = g_ledHandle.segmentInfo[disIndex].reg;
    GPIO_PIN pin = g_ledHandle.segmentInfo[disIndex].pin;

    for (int i = 0; i < BOARD_LED_SEG_NUM; i++) { /* Turn off the nixie tube according to the nixie tube polarity. */
        if (g_ledHandle.polarity == BOARD_LED_CATHODE) {
            DCL_GPIO_SetValue(gpiox, pin, GPIO_HIGH_LEVEL);
        } else {
            DCL_GPIO_SetValue(gpiox, pin, GPIO_LOW_LEVEL);
        }
    }
    /* Set the value according to the polarity of the nixie tube. */
    for (int i = 0; i < BOARD_LED_PIN_NUMBER; i++) {
        if (g_ledHandle.polarity == BOARD_LED_CATHODE) {
            DCL_GPIO_SetValue(g_ledHandle.gpioInfo[i].reg, g_ledHandle.gpioInfo[i].pin, GPIO_LOW_LEVEL);
        } else {
            DCL_GPIO_SetValue(g_ledHandle.gpioInfo[i].reg, g_ledHandle.gpioInfo[i].pin, GPIO_HIGH_LEVEL);
        }
    }
}

/**
  * @brief Output characters on specified segments.
  * @param ch Character Value.
  * @return BOARD_LED_Ret @ref BOARD_LED_Ret.
  */
static BOARD_LED_Ret BOARD_LED_ShowSingle(unsigned char ch)
{
    LED_PARAM_CHECK_WITH_RET((ch <= BOARD_LED_CHAR_NUMBER), BOARD_LED_ERR_INVALID_CHAR);
    unsigned char tmp;
    if (g_ledHandle.polarity == BOARD_LED_CATHODE) { /* Set the value according to the polarity of the nixie tube. */
        tmp = ~g_ledTable[ch];
    } else {
        tmp = g_ledTable[ch];
    }

    for (int i = 0; i < BOARD_LED_PIN_NUMBER; i++) {
        if (tmp & 0x80) { /* 0x80 : Obtains the value of the eighth digit. */
            DCL_GPIO_SetValue(g_ledHandle.gpioInfo[i].reg, g_ledHandle.gpioInfo[i].pin, GPIO_HIGH_LEVEL);
        } else {
            DCL_GPIO_SetValue(g_ledHandle.gpioInfo[i].reg, g_ledHandle.gpioInfo[i].pin, GPIO_LOW_LEVEL);
        }
        tmp <<= 1;
    }

    return BOARD_LED_OK;
}

/**
  * @brief Timer callback function. Display the character string according to the settings.
  * @param irqNum Interrupt number.
  * @param param Pointer to the timer handle.
  */
void BOARD_LED_TimerCallBack(void *param)
{
    TIMER_Handle *handle = (TIMER_Handle *)param;

    g_ledHandle.atTimeStep++;
    if (g_ledHandle.atTimeStep > g_ledHandle.totalTimeStep) {
        BOARD_LED_ClearPinAndSeg();
        HAL_TIMER_Stop(handle);
        return;
    }

    /* Invalidate the current segment (pointed by disIndex). */
    if (g_ledHandle.polarity == BOARD_LED_CATHODE) {
        DCL_GPIO_SetValue(g_ledHandle.segmentInfo[g_ledHandle.disIndex].reg,
                          g_ledHandle.segmentInfo[g_ledHandle.disIndex].pin, GPIO_HIGH_LEVEL);
    } else {
        DCL_GPIO_SetValue(g_ledHandle.segmentInfo[g_ledHandle.disIndex].reg,
                          g_ledHandle.segmentInfo[g_ledHandle.disIndex].pin, GPIO_LOW_LEVEL);
    }
    /* Point to the next seg. Point to the far left when reach the far right. */
    if ((++(g_ledHandle.disIndex)) == BOARD_LED_SEG_NUM) {
        g_ledHandle.disIndex = BOARD_LED_SEG_NUM - g_ledHandle.disLength;
    }
    /* Validate the current segment. */
    if (g_ledHandle.polarity == BOARD_LED_CATHODE) {
        DCL_GPIO_SetValue(g_ledHandle.segmentInfo[g_ledHandle.disIndex].reg,
            g_ledHandle.segmentInfo[g_ledHandle.disIndex].pin, GPIO_LOW_LEVEL);
    } else {
        DCL_GPIO_SetValue(g_ledHandle.segmentInfo[g_ledHandle.disIndex].reg,
            g_ledHandle.segmentInfo[g_ledHandle.disIndex].pin, GPIO_HIGH_LEVEL);
    }

    /* Output current character. */
    BOARD_LED_ShowSingle(g_ledHandle.disChar[g_ledHandle.disIndex - (BOARD_LED_SEG_NUM - g_ledHandle.disLength)]);
}

/**
  * @brief Get char's index in led display table.
  * @param str point of str.
  * @return BOARD_LED_Ret @ref BOARD_LED_Ret.
  */
static BOARD_LED_Ret DisplayCharIndexProcess(char *str)
{
    if (str[0] >= '0' && str[0] <= '9') {
        g_ledHandle.disChar[(g_ledHandle.disLength)++] = str[0] - '0';
    } else if (str[0] >= 'a' && str[0] <= 'f') {
        g_ledHandle.disChar[(g_ledHandle.disLength)++] = str[0] - 'a' + 10; /* 10: Additional 10 */
    } else if (str[0] >= 'A' && str[0] <= 'F') {
        g_ledHandle.disChar[(g_ledHandle.disLength)++] = str[0] - 'A' + 10; /* 10: Additional 10 */
    } else if (str[0] == '-') {
        /* '-' is in the last digit of the mapping table */
        g_ledHandle.disChar[(g_ledHandle.disLength)++] = BOARD_LED_CHAR_NUMBER - 1;
    } else {
        return BOARD_LED_ERR_INVALID_CHAR;
    }
    return BOARD_LED_OK;
}

/**
  * @brief Output string on digital tube.
  * @param str Pointer to string.
  * @param displayMs Display time, in milliseconds.
  * @param timerHandle Timer Handle.
  * @return BOARD_LED_Ret @ref BOARD_LED_Ret.
  */
BOARD_LED_Ret BOARD_LED_ShowString(char *str, unsigned int displayMs, TIMER_Handle *timerHandle)
{
    LED_ASSERT_PARAM(str != NULL);

    g_ledHandle.disLength = 0;
    while (str[0] != '\0') {
        if (g_ledHandle.disLength >= BOARD_LED_SEG_NUM) {
            return BOARD_LED_ERR_STRING_TOO_LONG;
        }

        if (DisplayCharIndexProcess(str) != BOARD_LED_OK) {
            return BOARD_LED_ERR_INVALID_CHAR;
        }

        str++;
    }
    /* Point index to the rightmost seg of the digital tube. */
    g_ledHandle.disIndex = g_ledHandle.disLength - 1;

    if (g_ledHandle.disIndex < 0) {
        return BOARD_LED_ERR_NO_CHAR;
    }

    g_ledHandle.atTimeStep = 0;
    g_ledHandle.totalTimeStep = displayMs / 5; /* 5 : 5ms */

    /* Set the GPIO output of all digital tubes and segment control to low level. */
    BOARD_LED_ClearPinAndSeg();

    timerHandle->bgLoad = (unsigned int)HAL_CRG_GetIpFreq((void *)timerHandle->baseAddress) / 200; /* 200: 5ms */
    HAL_TIMER_Config(timerHandle, TIMER_CFG_BGLOAD);
    HAL_TIMER_RegisterCallback(timerHandle, TIMER_PERIOD_FIN, BOARD_LED_TimerCallBack);
    HAL_TIMER_Start(timerHandle);
    return BASE_STATUS_OK;
}