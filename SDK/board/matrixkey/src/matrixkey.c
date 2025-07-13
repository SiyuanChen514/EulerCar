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
  * @file      matrixkey.c
  * @author    MCU Application Team
  * @brief     Dynamic key scanning module application.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the Dynamic key scanning.
  *             + Initialization functions.
  *             + Pin configuration and key event registration function.
  * @verbatim
  * usage:
  * 1) Use the chip config tool to generate the timer/gpt initialization code.
  * 2) Call the BOARD_MKEY_Init() function to select the specified TIMER and set Key valid values.
  * 3) Call the BOARD_MKEY_ConfigOutputPin() and BOARD_MKEY_ConfigInputPin() function to set pins.
  * 4) Call the BOARD_MKEY_RegisterKeyFun() to register the key triggering event.
  * 5) Start the corresponding TIMER to monitor the key press status.
  * 6) Note: If BOARD_MKEY_SCHEME_NUMBER_TWO is used, the BOARD_MKEY_SCAN_KEY() function should be called in the main
  *          function while loop to complete key scanning.
  * The pin numbers of the Key matrix are as follows:
  * out1  |  key1    key2    key3    key4
  *       |
  * out2  |  key5    key6    key7    key8
  *       |
  * out3  |  key9    key10   key11   key12
  *       |
  * out4  |  key13   key14   key15   key16
  *       |--------------------------------
  *          in1     in2     in3     in4
  * @endverbatim
  */

/* Includes ------------------------------------------------------------------ */
#include "matrixkey.h"

/* Private variables --------------------------------------------------------- */
BOARD_MKEY_Handle g_mKeyInfo; /**< Matrix key array. */
static unsigned int g_mKeyValidTick; /**< Number of ticks for which keys take effect. */
static unsigned int g_mKeyTotalKeyNumber = BOARD_MKEY_IN_NUM * BOARD_MKEY_OUT_NUM; /**< Total number of keys. */

/**
  * @brief Initialization function, setting the timer and key validity period (ms).
  * @param timerHandle Timer handle.
  * @param validPressTimeMs Key validity period, in milliseconds.
  */
void BOARD_MKEY_Init(TIMER_Handle *timerHandle, unsigned int validPressTimeMs)
{
    MKEY_ASSERT_PARAM(timerHandle != NULL);

    g_mKeyValidTick = validPressTimeMs;

    unsigned int loadVal = HAL_CRG_GetIpFreq((void *)timerHandle->baseAddress) / 1000; /* 1000: get cycles in 1 ms. */
    timerHandle->load = loadVal;
    timerHandle->bgLoad = loadVal;
    HAL_TIMER_Config(timerHandle, TIMER_CFG_LOAD); /* Configuring the Counting Period of a Timer. */
    HAL_TIMER_Config(timerHandle, TIMER_CFG_BGLOAD);
    HAL_TIMER_RegisterCallback(timerHandle, TIMER_PERIOD_FIN, BOARD_MKEY_TimerCallBack);

    for (unsigned int i = 0; i < g_mKeyTotalKeyNumber; i++) { /* Initialize the callback function array. */
        g_mKeyInfo.keyInfo[i].fun = NULL;
        g_mKeyInfo.keyInfo[i].tick = 0;
    }
}

/**
  * @brief Configuring GPIO for Output.
  * @param gpiox GPIO group: GPIO0, GPIO1, GPIO2...
  * @param pins Specific pin value.
  * @param index Specifies the index value, in [0, BOARD_MKEY_OUT_NUM).
  * @return BOARD_MKEY_RET @ref BOARD_MKEY_RET.
  */
BOARD_MKEY_RET BOARD_MKEY_ConfigOutputPin(GPIO_RegStruct *gpiox, unsigned int pins, unsigned int index)
{
    MKEY_ASSERT_PARAM(gpiox != NULL);
    MKEY_PARAM_CHECK_WITH_RET((index < BOARD_MKEY_OUT_NUM), BOARD_MKEY_RET_INVALID_PARAM);

    g_mKeyInfo.outPins[index].gpiox = gpiox;
    g_mKeyInfo.outPins[index].pins = pins;
    DCL_GPIO_SetDirection(gpiox, pins, GPIO_OUTPUT_MODE); /* Set to Output Mode. */
    DCL_GPIO_SetValue(gpiox, pins, BOARD_MKEY_OUT_PIN_INVALID);

    return BOARD_MKEY_RET_OK;
}

/**
  * @brief Configuring GPIO for Input.
  * @param gpiox GPIO group: GPIO0, GPIO1, GPIO2...
  * @param pins Specific pin value.
  * @param index Specifies the index value, in [0, BOARD_MKEY_In_NUM).
  * @return BOARD_MKEY_RET @ref BOARD_MKEY_RET.
  */
BOARD_MKEY_RET BOARD_MKEY_ConfigInputPin(GPIO_RegStruct *gpiox, unsigned int pins, unsigned int index)
{
    MKEY_ASSERT_PARAM(gpiox != NULL);
    MKEY_PARAM_CHECK_WITH_RET((index < BOARD_MKEY_IN_NUM), BOARD_MKEY_RET_INVALID_PARAM);

    g_mKeyInfo.inPins[index].gpiox = gpiox;
    g_mKeyInfo.inPins[index].pins = pins;
    DCL_GPIO_SetDirection(gpiox, pins, GPIO_INPUT_MODE); /* Set to input mode. */
    DCL_GPIO_SetValue(gpiox, pins, BOARD_MKEY_IN_PIN_INVALID);

    return BOARD_MKEY_RET_OK;
}

/**
  * @brief Registering the Key Binding Function.
  * @param keyNum Key Number.Calculated based on the input and output index values.
  * @param fun Key function pointer.
  * @return BOARD_MKEY_RET @ref BOARD_MKEY_RET.
  */
BOARD_MKEY_RET BOARD_MKEY_RegisterKeyFun(unsigned int keyNum, MKeyHandleFun fun)
{
    MKEY_ASSERT_PARAM(fun != NULL);
    MKEY_PARAM_CHECK_WITH_RET((keyNum < g_mKeyTotalKeyNumber), BOARD_MKEY_RET_INVALID_PARAM);

    g_mKeyInfo.keyInfo[keyNum].fun = fun;
    return BOARD_MKEY_RET_OK;
}

/**
  * @brief Disable all output GPIOs.
  */
static void BOARD_MKEY_TurnOffOutPin(void)
{
    for (int i = 0; i < BOARD_MKEY_OUT_NUM; i++) {
        MKEY_PARAM_CHECK_NO_RET(g_mKeyInfo.outPins[i].gpiox);
        DCL_GPIO_SetValue(g_mKeyInfo.outPins[i].gpiox, g_mKeyInfo.outPins[i].pins, BOARD_MKEY_OUT_PIN_INVALID);
    }
}

#if (BOARD_MKEY_SCHEME_NUMBER == BOARD_MKEY_SCHEME_NUMBER_ONE)
/**
  * @brief Timer callback function. Determines the status of each key and executes the registration event according to
  *        the status.
  * @param irqNum Interrupt number.
  * @param param Pointer to the timer handle.
  */
void BOARD_MKEY_TimerCallBack(void *param)
{
    BASE_FUNC_UNUSED(param);

    for (unsigned int i = 0; i < BOARD_MKEY_OUT_NUM; i++) {
        BOARD_MKEY_TurnOffOutPin(); /* Turn off all output pins, i.e. set to high level. */
        DCL_GPIO_SetValue(g_mKeyInfo.outPins[i].gpiox, g_mKeyInfo.outPins[i].pins, BOARD_MKEY_OUT_PIN_VALID);
        for (unsigned int j = 0; j < BOARD_MKEY_IN_NUM; j++) {
            unsigned int tmp = i * BOARD_MKEY_IN_NUM + j;
            GPIO_Value value = DCL_GPIO_GetPinValue(g_mKeyInfo.inPins[j].gpiox, g_mKeyInfo.inPins[j].pins);
            if (value == BOARD_MKEY_IN_PIN_VALID) {
                g_mKeyInfo.keyInfo[tmp].tick++; /* If the key is pressed, the count increases. */
                continue;
            }
            /* If the count is greater than the threshold, the callback function is called. */
            if (g_mKeyInfo.keyInfo[tmp].tick >= g_mKeyValidTick && g_mKeyInfo.keyInfo[tmp].fun) {
                g_mKeyInfo.keyInfo[tmp].fun();
            }
            g_mKeyInfo.keyInfo[tmp].tick = 0;
        }
    }
}
#endif

#if (BOARD_MKEY_SCHEME_NUMBER == BOARD_MKEY_SCHEME_NUMBER_TWO)
/**
  * @brief Timer callback function. Check the status of each key and update the tick value.
  * @param irqNum Interrupt number.
  * @param param Pointer to the timer handle.
  */
void BOARD_MKEY_TimerCallBack(void *param)
{
    BASE_FUNC_UNUSED(param);

    for (int i = 0; i < BOARD_MKEY_OUT_NUM; i++) {
        BOARD_MKEY_TurnOffOutPin(); /* Turn off all output pins, i.e. set to high level. */
        DCL_GPIO_SetValue(g_mKeyInfo.outPins[i].gpiox, g_mKeyInfo.outPins[i].pins, BOARD_MKEY_OUT_PIN_VALID);
        for (int j = 0; j < BOARD_MKEY_IN_NUM; j++) {
            int tmp = i * BOARD_MKEY_IN_NUM + j;
            int gpioVal = DCL_GPIO_GetPinValue(g_mKeyInfo.inPins[j].gpiox, g_mKeyInfo.inPins[j].pins);
            if (gpioVal == BOARD_MKEY_IN_PIN_VALID && g_mKeyInfo.keyInfo[tmp].tick < g_mKeyValidTick) {
                g_mKeyInfo.keyInfo[tmp].tick++;  /* The count increases when the level status is invalid. */
                continue;
            }
            if (gpioVal == BOARD_MKEY_IN_PIN_INVALID && g_mKeyInfo.keyInfo[tmp].tick < g_mKeyValidTick) {
                g_mKeyInfo.keyInfo[tmp].tick = 0; /* The count is cleared when the level status is invalid. */
                continue;
            }
            if (gpioVal == BOARD_MKEY_IN_PIN_INVALID && g_mKeyInfo.keyInfo[tmp].tick == g_mKeyValidTick) {
                g_mKeyInfo.keyInfo[tmp].tick++;
            }
        }
    }
}

/**
  * @brief Scans the key tick value and executes the binding function when the value reaches the threshold.
  *        This function is called in the main loop.
  */
void BOARD_MKEY_SCAN_KEY(void)
{
    for (unsigned int i = 0; i < g_mKeyTotalKeyNumber; i++) {
        if (g_mKeyInfo.keyInfo[i].tick > g_mKeyValidTick) {
            g_mKeyInfo.keyInfo[i].tick = 0;
            if (g_mKeyInfo.keyInfo[i].fun) { /* Invoke the callback function. */
                g_mKeyInfo.keyInfo[i].fun();
            }
        }
    }
}
#endif