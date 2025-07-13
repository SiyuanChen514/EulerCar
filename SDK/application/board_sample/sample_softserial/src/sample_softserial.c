/**
  * @ Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2023. All rights reserved.
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
  * @file      sample_softserial.c
  * @author    MCU Application Team
  * @brief     Softserial sample code
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the common function sample.
  *            + Softserial sample functions.
  */

/* Includes ------------------------------------------------------------------ */
#include "main.h"
#include "sample_softserial.h"

/* Private macro ------------------------------------------------------------- */
#define BOARD_SOFTSERIAL_SAMPLE_BAUDRATE 1200
#define BOARD_SOFTSERIAL_SAMPLE_DATALENGTH 8

/* Private variables --------------------------------------------------------- */

/**
  * @brief Soft serial port loopback example.
  * @return int Execution result.
  */
int BOARD_SOFTSERIAL_Sample(void)
{
    SystemInit();

    BOARD_SOFTSERIAL_Config txConfig;
    BOARD_SOFTSERIAL_Config rxConfig;
    /* Configuring UART TX Parameters. */
    txConfig.baudRate = BOARD_SOFTSERIAL_SAMPLE_BAUDRATE;
    txConfig.dataLength = BOARD_SOFTSERIAL_SAMPLE_DATALENGTH;
    txConfig.parityMode = BOARD_SOFTSERIAL_PARITY_NONE;
    txConfig.stopBit = BOARD_SOFTSERIAL_STOPBITS_ONE;
    BOARD_SOFTSERIAL_TxInit(&TX_HANDLE, &g_timer1, txConfig);
    /* Configuring UART RX Parameters. */
    rxConfig.baudRate = BOARD_SOFTSERIAL_SAMPLE_BAUDRATE;
    rxConfig.dataLength = BOARD_SOFTSERIAL_SAMPLE_DATALENGTH;
    rxConfig.parityMode = BOARD_SOFTSERIAL_PARITY_NONE;
    rxConfig.stopBit = BOARD_SOFTSERIAL_STOPBITS_ONE;
    BOARD_SOFTSERIAL_RxInit(&RX_HANDLE, &g_timer2, rxConfig);

    unsigned char tmp;
    while (1) { /* Transmit and receive data cyclically. */
        if (BOARD_SOFTSERIAL_GetChar(&tmp) == BOARD_SOFTSERIAL_OK) {
            BOARD_SOFTSERIAL_PrintCh(tmp);
        }
    }
    return BASE_STATUS_OK;
}