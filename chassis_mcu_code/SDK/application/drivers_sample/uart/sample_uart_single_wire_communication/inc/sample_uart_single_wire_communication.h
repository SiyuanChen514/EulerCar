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
  * @file    sample_uart_single_wire_communication.h
  * @author  MCU Driver Team
  * @brief   uart sample module.
  * @details This file provides sample code for users to help use
  *          the single wire transmission of the UART in interrupt mode.
  */

#ifndef SAMPLE_UART_SINGLE_WIRE_COMMUNICATION
#define SAMPLE_UART_SINGLE_WIRE_COMMUNICATION

#include "debug.h"
#include "uart.h"
#include "interrupt.h"
#include "main.h"
#include "ioconfig.h"
/* Indicates the bus status. */
typedef enum {
    UART_SINGLE_WIRE_READY       = 0x00000000U,
    UART_SINGLE_WIRE_TX_FIN      = 0x00000001U,
    UART_SINGLE_WIRE_RX_FIN      = 0x00000002U,
    UART_SINGLE_WIRE_BUSY        = 0x00000004U,
    UART_SINGLE_WIRE_SEND_SUCCESS = 0x00000008U
} UART_SINGLEWIRESTATUS;

extern volatile unsigned int g_uartSingleWireStatus;  /* A global variable that indicates the bus status. */
extern volatile bool g_uartNotBusy;                   /* Global variable indicating whether the bus is busy. */

/* Function declaration */
void UART_SingleWireCommunication(void);
void UART_EnableTx(UART_Handle *handle);
void UART_DisableTx(UART_Handle *handle);
void UART_EnableRx(UART_Handle *handle);
void UART_DisableRx(UART_Handle *handle);
void UART2TxIOEnable(void);   /* IO config setting. */
void UART1TxIOEnable(void);
void UART1TxToUART2Rx(UART_Handle *uart1Handle, UART_Handle *uart2Handle);  /* Pin configure. */
void UART2TxToUART1Rx(UART_Handle *uart1Handle, UART_Handle *uart2Handle);
#endif