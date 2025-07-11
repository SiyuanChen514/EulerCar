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
  * @file      pulses.c
  * @author    MCU Application Team
  * @brief     Provides an interface that uses a timer to generate pulses with configurable cycles and duty cycles.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the pulses.
  *             + Initialization and de-initialization functions.
  *             + Pulse generation and stop function.
  * @verbatim
  * usage:
  * 1) Use the chip config tool to generate the timer initialization code.
  * 2) Call the BOARD_PULSES_Init() function to configure the GPIO, pulse period, and duty cycle.
  * 3) Call the BOARD_PULSES_Start() function to generate pulses.
  * 4) Call the BOARD_PULSES_Stop() function to stop generating pulses.
  * @endverbatim
  */

/* Includes ------------------------------------------------------------------ */
#include "pulses.h"
#include "debug.h"
/* Private variables --------------------------------------------------------- */
BOARD_PULSES_Handle g_pulsesInfo[BOARD_PULSES_NUM]; /**< Pulese handles array. */

/**
  * @brief Initial configuration, including GPIO, TIMER, pulse period, and duty cycle.
  * @param gpioHandle GPIO handle.
  * @param timerHandle TIMER handle.
  * @param period Pulse period, in seconds.
  * @param dutyCycle Pulse duty cycle. The value is greater than or equal to 0 and less than or equal to 1.
  * @param index This parameter specifies the index value of the pulse information. The value range is as follows:
  *              [0, BOARD_PULSES_NUM].
  * @return None.
  */
void BOARD_PULSES_Init(GPIO_Handle *gpioHandle, TIMER_Handle *timerHandle, float period,
                       float dutyCycle, int index)
{
    PULSES_ASSERT_PARAM(gpioHandle != NULL);
    PULSES_ASSERT_PARAM(timerHandle != NULL);
    /* Set to output mode. The default level is low. */
    DCL_GPIO_SetDirection(gpioHandle->baseAddress, gpioHandle->pins, GPIO_OUTPUT_MODE);
    DCL_GPIO_SetValue(gpioHandle->baseAddress, gpioHandle->pins, GPIO_LOW_LEVEL);

    unsigned int totalTimCnt = period * HAL_CRG_GetIpFreq((void *)timerHandle->baseAddress);
    /* Configuring Analog PWM Wave Output Parameters. */
    g_pulsesInfo[index].highCnt = totalTimCnt * dutyCycle;
    g_pulsesInfo[index].lowCnt = totalTimCnt * (1 - dutyCycle);
    g_pulsesInfo[index].status = BOARD_PULSES_STATUS_STOP;
    g_pulsesInfo[index].gpiox = gpioHandle->baseAddress;
    g_pulsesInfo[index].pins = gpioHandle->pins;
    g_pulsesInfo[index].timerHandle = timerHandle;
    g_pulsesInfo[index].value = GPIO_LOW_LEVEL;
    HAL_TIMER_RegisterCallback(timerHandle, TIMER_PERIOD_FIN, BOARD_PULSES_TimerCallBack);
}

/**
  * @brief Pulse deinitialization.
  * @param index Pulse index value.
  * @return BOARD_PULSES_Ret Function Return Status.
  */
BOARD_PULSES_Ret BOARD_PULSES_Deinit(int index)
{
    PULSES_PARAM_CHECK_WITH_RET((index >= 0 && index <= BOARD_PULSES_NUM), BOARD_PULSES_ERR_PARAM_INVALID);
    g_pulsesInfo[index].gpiox = NULL;
    return BOARD_PULSES_OK;
}

/**
  * @brief Start output pulse.
  * @param index Pulse index value.
  * @return BOARD_PULSES_Ret Function Return Status.
  */
BOARD_PULSES_Ret BOARD_PULSES_Start(int index)
{
    PULSES_PARAM_CHECK_WITH_RET((index >= 0 && index <= BOARD_PULSES_NUM), BOARD_PULSES_ERR_PARAM_INVALID);
    PULSES_PARAM_CHECK_WITH_RET((g_pulsesInfo[index].gpiox != NULL), BOARD_PULSES_ERR_PARAM_INVALID);

    g_pulsesInfo[index].value = GPIO_LOW_LEVEL;
    g_pulsesInfo[index].status = BOARD_PULSES_STATUS_START;
    DCL_TIMER_SetLoad(g_pulsesInfo[index].timerHandle->baseAddress, 1);
    DCL_TIMER_Enable(g_pulsesInfo[index].timerHandle->baseAddress); /* Start timer timing. */
    return BOARD_PULSES_OK;
}

/**
  * @brief Stop output pulse.
  * @param index Pulse index value.
  * @return BOARD_PULSES_Ret Function Return Status.
  */
BOARD_PULSES_Ret BOARD_PULSES_Stop(int index)
{
    PULSES_PARAM_CHECK_WITH_RET((index >= 0 && index <= BOARD_PULSES_NUM), BOARD_PULSES_ERR_PARAM_INVALID);
    PULSES_PARAM_CHECK_WITH_RET((g_pulsesInfo[index].gpiox != NULL), BOARD_PULSES_ERR_PARAM_INVALID);

    g_pulsesInfo[index].status = BOARD_PULSES_STATUS_STOP;
    return BOARD_PULSES_OK;
}

/**
  * @brief Timer callback function. Used to determine and change the GPIO state to generate pulses.
  * @param irqNum Interrupt number.
  * @param param Pointer to the timer handle.
  */
void BOARD_PULSES_TimerCallBack(void *param)
{
    TIMER_Handle *handle = (TIMER_Handle *)param;
    int index;

    for (index = 0; index < BOARD_PULSES_NUM; index++) {
        if (g_pulsesInfo[index].timerHandle && g_pulsesInfo[index].timerHandle->baseAddress == handle->baseAddress) {
            break; /* Select timer for PWM wave output. */
        }
    }

    if (index == BOARD_PULSES_NUM) {
        return;
    }
    /* If not in the start state, set to default low and exit. */
    if (g_pulsesInfo[index].status != BOARD_PULSES_STATUS_START) {
        DCL_GPIO_SetValue(g_pulsesInfo[index].gpiox, g_pulsesInfo[index].pins, GPIO_LOW_LEVEL);
        return;
    }

    if (g_pulsesInfo[index].value == GPIO_HIGH_LEVEL) {
        DCL_GPIO_SetValue(g_pulsesInfo[index].gpiox, g_pulsesInfo[index].pins, GPIO_HIGH_LEVEL);
        DCL_TIMER_SetLoad(g_pulsesInfo[index].timerHandle->baseAddress, g_pulsesInfo[index].highCnt);
        g_pulsesInfo[index].value = GPIO_LOW_LEVEL;
    } else { /* Set Low Level and Duration. */
        DCL_GPIO_SetValue(g_pulsesInfo[index].gpiox, g_pulsesInfo[index].pins, GPIO_LOW_LEVEL);
        DCL_TIMER_SetLoad(g_pulsesInfo[index].timerHandle->baseAddress, g_pulsesInfo[index].lowCnt);
        g_pulsesInfo[index].value = GPIO_HIGH_LEVEL;
    }

    HAL_TIMER_Start(handle);
}