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
  * @file      sample_led.c
  * @author    MCU Application Team
  * @brief     Led sample code
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the common function sample.
  *             + Led sample functions
  */

/* Includes ------------------------------------------------------------------ */
#include "main.h"
#include "sample_led.h"

/* Private macro ------------------------------------------------------------- */
#define BOARD_LED_SAMPLE_BAUDRATE 115200

/* Private variables --------------------------------------------------------- */

/**
  * @brief LED digit tube display example.
  * @return int Execution result.
  */
int BOARD_LED_Sample(void)
{
    SystemInit();

    BOARD_LED_PinConfig((LED_A_HANDLE.baseAddress), LED_A_PIN, 0); /* 0 : index value. */
    BOARD_LED_PinConfig((LED_B_HANDLE.baseAddress), LED_B_PIN, 1); /* 1 : index value. */
    BOARD_LED_PinConfig((LED_C_HANDLE.baseAddress), LED_C_PIN, 2); /* 2 : index value. */
    BOARD_LED_PinConfig((LED_D_HANDLE.baseAddress), LED_D_PIN, 3); /* 3 : index value. */
    BOARD_LED_PinConfig((LED_E_HANDLE.baseAddress), LED_E_PIN, 4); /* 4 : index value. */
    BOARD_LED_PinConfig((LED_F_HANDLE.baseAddress), LED_F_PIN, 5); /* 5 : index value. */
    BOARD_LED_PinConfig((LED_G_HANDLE.baseAddress), LED_G_PIN, 6); /* 6 : index value. */
    BOARD_LED_PinConfig((LED_H_HANDLE.baseAddress), LED_H_PIN, 7); /* 7 : index value. */

    BOARD_LED_SegmentConfig((LED1_SELECT_HANDLE.baseAddress), LED1_SELECT_PIN, 0); /* 0 : index value. */
    BOARD_LED_SegmentConfig((LED2_SELECT_HANDLE.baseAddress), LED2_SELECT_PIN, 1); /* 1 : index value. */
    BOARD_LED_SegmentConfig((LED3_SELECT_HANDLE.baseAddress), LED3_SELECT_PIN, 2); /* 2 : index value. */
    BOARD_LED_SegmentConfig((LED4_SELECT_HANDLE.baseAddress), LED4_SELECT_PIN, 3); /* 3 : index value. */
    BOARD_LED_PolarityConfig(BOARD_LED_CATHODE);

    BOARD_LED_ShowString("1359", 3000, &g_timer1); /* 3000 : 3000ms. */
    return 0;
}