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
  * @file    sample_uart_single_wire_communication.c
  * @author  MCU Driver Team
  * @brief   uart sample module.
  * @details In single-line communication, one line is used to connect the TX and RX of the UART.
  *          The UART continuously switches the RX and RX functions to receive and transmit data on one data line.
  *          (1) This sample is applicable to the single-wire UART communication function. In this case, the UART works
  *              in half-duplex mode.
  *          (2) Serial port 0 (UART0) is used to print debugging information, aUART1 and UART2 are used for
  *              single-wire communication. The connection modes of UART1 and UART2 are as follows:
  *                UART1_Tx---|_____|---UART2_Tx
  *                UART1_Rx---|     |---UART2_Rx
  *          (3) The TIMER is used to query the UART single-wire status in interrupt mode (or block mode).
  */
#include "sample_uart_single_wire_communication.h"


#define UART_SINGLE_WIRE_TX_COUNT 5
#define UART_SINGLE_WIRE_RX_COUNT 5
#define UART_SINGLE_DELAY_TIMER 1000
#define UART2_ENABLE_TX 0x3
#define UART1_ENABLE_TX 0x4
#define UART_RESET_STATUS 0x1
 
static unsigned char g_singleWireTx[UART_SINGLE_WIRE_TX_COUNT] = "12345"; /* Single wire Tx data length is 5 */
static unsigned char g_singleWireRx[UART_SINGLE_WIRE_RX_COUNT] = {0};     /* Single wire Rx data length is 5 */
 
 
volatile bool g_uart2NotBusy = true;                                    /* Flag of uart not busy. */
volatile bool g_uart1NotBusy = true;
volatile unsigned int g_uartSingleWireStatus = UART_SINGLE_WIRE_READY; /* By default, the UART is ready. */
 
/**
 * @brief Enables the UART Tx interrupt and transmit functions.
 * @param handle UART handle.
 * @retval None.
 */
void UART_EnableTx(UART_Handle *handle)
{
    UART_ASSERT_PARAM(handle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    /* Enable the TX and TX completion interrupts. */
    handle->baseAddress->UART_CR.BIT.uarten = BASE_CFG_DISABLE;
    handle->baseAddress->UART_CR.BIT.txe = BASE_CFG_ENABLE;     /* Enables UART transmit. */
    /* Clears the TX and RX FIFOs. */
    handle->baseAddress->UART_LCR_H.BIT.fen = BASE_CFG_DISABLE;
    __asm__ volatile ("nop");
    handle->baseAddress->UART_LCR_H.BIT.fen = handle->fifoMode;
    handle->baseAddress->UART_IMSC.BIT.txim = BASE_CFG_ENABLE;  /* TX interrupt enable. */
    /* Make all configurations take effect. */
    handle->baseAddress->UART_CR.BIT.uarten = BASE_CFG_ENABLE;
}
 
/**
 * @brief Disable the UART Tx interrupt and transmit functions.
 * @param handle UART handle.
 * @retval None.
 */
void UART_DisableTx(UART_Handle *handle)
{
    UART_ASSERT_PARAM(handle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    /* Disable the TX and TX completion interrupts. */
    handle->baseAddress->UART_CR.BIT.uarten = BASE_CFG_DISABLE;
    handle->baseAddress->UART_CR.BIT.txe = BASE_CFG_DISABLE;       /* Disable UART transmit. */
    handle->baseAddress->UART_LCR_H.BIT.fen = BASE_CFG_DISABLE;    /* Clears the FIFOs. */
    __asm__ volatile ("nop");
    handle->baseAddress->UART_LCR_H.BIT.fen = handle->fifoMode;   /* Flesh the FIFOs */
    handle->baseAddress->UART_IMSC.BIT.txim = BASE_CFG_DISABLE;   /* Mask TX interrupt enable. */
    handle->baseAddress->UART_CR.BIT.uarten = BASE_CFG_ENABLE;    /* Enable UART. */
}
 
/**
 * @brief Enable the UART Rx interrupt and transmit functions.
 * @param handle UART handle.
 * @retval None.
 */
void UART_EnableRx(UART_Handle *handle)
{
    UART_ASSERT_PARAM(handle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    /* Enable the RX and RX completion interrupts. */
    handle->baseAddress->UART_CR.BIT.uarten = BASE_CFG_DISABLE;
    handle->baseAddress->UART_CR.BIT.rxe = BASE_CFG_ENABLE;        /* Enables UART reception. */
 
    handle->baseAddress->UART_LCR_H.BIT.fen = BASE_CFG_DISABLE;    /* Flesh the FIFOs */
    __asm__ volatile ("nop");
    handle->baseAddress->UART_LCR_H.BIT.fen = handle->fifoMode;
    handle->baseAddress->UART_IMSC.BIT.rxim = BASE_CFG_ENABLE;    /* Mask RX interrupt enable. */
 
    handle->baseAddress->UART_CR.BIT.uarten = BASE_CFG_ENABLE;    /* The configuration takes effect. */
}
 
/**
 * @brief Disable the UART Rx interrupt and transmit functions.
 * @param handle UART handle.
 * @retval None.
 */
void UART_DisableRx(UART_Handle *handle)
{
    UART_ASSERT_PARAM(handle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    /* disable the RX and RX completion interrupts. */
    handle->baseAddress->UART_CR.BIT.uarten = BASE_CFG_DISABLE;
    /* Disable UART reception. */
    handle->baseAddress->UART_CR.BIT.rxe = BASE_CFG_DISABLE;
    /* Flesh the FIFOs */
    handle->baseAddress->UART_LCR_H.BIT.fen = BASE_CFG_DISABLE;
    __asm__ volatile ("nop");
    handle->baseAddress->UART_LCR_H.BIT.fen = handle->fifoMode;
    handle->baseAddress->UART_IMSC.BIT.rxim = BASE_CFG_DISABLE;  /* Mask RX interrupt disable. */
    /* Enable UART */
    handle->baseAddress->UART_CR.BIT.uarten = BASE_CFG_ENABLE;
}

/**
 * @brief Enable the transmit pin of the UART2 and disable the transmit pin of the UART1.
 * @param None
 * @retval None.
 */
void UART2TxIOEnable(void)
{
    IOConfig_RegStruct *iconfig = IOCONFIG;
    iconfig->iocmg_15.BIT.func =  UART_RESET_STATUS;     /* Disable the Tx of UART1. */
    iconfig->iocmg_51.BIT.func =  UART2_ENABLE_TX;    /* Enable the Tx of UART2. */
}

/**
 * @brief Enable the transmit pin of the UART1 and disable the transmit pin of the UART2.
 * @param None
 * @retval None.
 */
void UART1TxIOEnable(void)
{
    IOConfig_RegStruct *iconfig = IOCONFIG;
    iconfig->iocmg_15.BIT.func = UART1_ENABLE_TX;     /* Enable the Tx of UART1. */
    iconfig->iocmg_51.BIT.func = UART_RESET_STATUS;    /* Disable the Tx of UART2. */
}

/**
 * @brief Clear string.
 * @param str, String to be cleared.
 * @retval None.
 */
static void ClearString(unsigned char *str)
{
    for (unsigned int i = 0; i < UART_SINGLE_WIRE_RX_COUNT; ++i) {
        str[i] = 0;
    }
}
 
/**
 * @brief Periodic callback function of TIMER2.
 * @param handle TIMER handle
 * @retval None.
 */
void TIMER2CallbackFunction(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* Queries the bus status of UART2. */
    if (g_uart2.baseAddress->UART_FR.BIT.busy == BASE_CFG_DISABLE) {
        g_uart2NotBusy = true;
    } else {
        g_uart2NotBusy = false;    /* UART2 busy */
    }
    /* Queries the bus status of UART1. */
    if (g_uart1.baseAddress->UART_FR.BIT.busy == BASE_CFG_DISABLE) {
        g_uart1NotBusy = true;
    } else {
        g_uart1NotBusy = false;    /* UART1 busy */
    }
}
 
/**
 * @brief User-defined read completion interrupt callback function of UART2.
 * @param handle UART handle.
 * @retval None.
 */
void UART2ReadInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("\r\nUART 2 Read Finish: %s\r\n", g_singleWireRx);
    g_uartSingleWireStatus = UART_SINGLE_WIRE_RX_FIN;
    return;
}
 
/**
 * @brief User-defined write completion interrupt callback function of UART2.
 * @param handle UART handle.
 * @retval None.
 */
void UART2WriteInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("\r\nUART 2 Write Finish\r\n");
    g_uartSingleWireStatus = UART_SINGLE_WIRE_TX_FIN;
    return;
}

/**
 * @brief User-defined write completion interrupt callback function of UART 1.
 * @param handle UART handle.
 * @retval None.
 */
void UART1WriteInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("\r\nUART 1 Write Finish\r\n");
    /* USER CODE BEGIN UART1_WRITE_IT_FINISH */
    /* USER CODE END UART1_WRITE_IT_FINISH */
}

/**
 * @brief User-defined read completion interrupt callback function of UART1.
 * @param handle UART handle.
 * @retval None.
 */
void UART1ReadInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("\r\nUART 1 Read Finish: %s\r\n", g_singleWireRx);
    /* USER CODE BEGIN UART1_READ_IT_FINISH */
    /* USER CODE END UART1_READ_IT_FINISH */
}

/**
 * @brief Prompt message
 * @param None.
 * @retval None.
 */
static void SingleWirePromptMessage(void)
{
    /* Message Begin */
    DBG_PRINTF("UART Init finish, UART single wire commiuncation:\r\n");
    DBG_PRINTF("The sent data is 12345 and the received data length is 5. \r\n");
    DBG_PRINTF("By default, the message is sent before the message is received. \r\n");
    DBG_PRINTF("--- UART0 is used for printing and debugging. \r\n");
    DBG_PRINTF("UART1 and UART2 are used for single-wire communication.");
    DBG_PRINTF("The connection modes of UART1 and UART2 are as follows:\r\n");
    DBG_PRINTF("\r\n  UART1_Tx---|_____|---UART2_Tx   \r\n");
    DBG_PRINTF("\r\n  UART1_Rx---|     |---UART2_Rx   \r\n");
    DBG_PRINTF("You can view prompt information through serial port 0 and check whether single-wire communication");
    DBG_PRINTF("is normal. \r\n");
    /* Message End */
}

/**
 * @brief UART1 TX, UART2 RX, pin configuration.
 * @param uart1Handle UART Handle.
 * @param uart2Handle UART Handle
 * @retval None.
 */
void UART1TxToUART2Rx(UART_Handle *uart1Handle, UART_Handle *uart2Handle)
{
    UART_ASSERT_PARAM((uart1Handle != NULL) && (uart2Handle != NULL));
    UART_ASSERT_PARAM(IsUARTInstance(uart1Handle->baseAddress) && IsUARTInstance(uart2Handle->baseAddress));
    DBG_PRINTF("UART single wire communications:  UART1 -> UART2 \r\n");
    UART1TxIOEnable();            /* Pin and Tx enable configuration. */
    UART_DisableRx(uart1Handle);  /*  UART1 Tx enable and UART2 Rx enable. */
    UART_EnableTx(uart1Handle);
    UART_DisableTx(uart2Handle);
    UART_EnableRx(uart2Handle);
}

/**
 * @brief UART2 TX, UART1 RX, pin configuration.
 * @param uart1Handle UART Handle.
 * @param uart2Handle UART Handle
 * @retval None.
 */
void UART2TxToUART1Rx(UART_Handle *uart1Handle, UART_Handle *uart2Handle)
{
    UART_ASSERT_PARAM((uart1Handle != NULL) && (uart2Handle != NULL));
    UART_ASSERT_PARAM(IsUARTInstance(uart1Handle->baseAddress) && IsUARTInstance(uart2Handle->baseAddress));
    DBG_PRINTF("UART single wire communications: UART2 -> UART1 \r\n");
    UART2TxIOEnable();            /* Pin and Tx enable configuration. */
    UART_DisableRx(uart2Handle);  /*  UART2 Tx enable and UART1 Rx enable. */
    UART_EnableTx(uart2Handle);
    UART_DisableTx(uart1Handle);
    UART_EnableRx(uart1Handle);
}

/**
 * @brief UART Interrupt Tx and Rx simultaneously.
 * @param None.
 * @retval None.
 */
void UART_SingleWireCommunication(void)
{
    SystemInit();
    /* Prompt message. */
    SingleWirePromptMessage();
    while (1) {
        /* By default, data is sent first and then received. */
        switch (g_uartSingleWireStatus) {
            case UART_SINGLE_WIRE_READY:
                /* Initially, it is sent by default. You can set whether to receive or send first. */
                UART2TxToUART1Rx(&g_uart1, &g_uart2);
                ClearString(g_singleWireRx);
                /* UART2 transmits data to UART1. */
                HAL_UART_WriteIT(&g_uart2, g_singleWireTx, UART_SINGLE_WIRE_TX_COUNT);
                HAL_UART_ReadIT(&g_uart1, g_singleWireRx, UART_SINGLE_WIRE_RX_COUNT);
                g_uartSingleWireStatus = UART_SINGLE_WIRE_BUSY;
                break;
            case UART_SINGLE_WIRE_TX_FIN:
                 /* UART1 transmits data to UART2. */
                 /* You need to query the bus status before configuring the TX. */
                if (g_uart2NotBusy == BASE_CFG_ENABLE) {   /* Check whether the UART2 bus is busy. */
                    UART1TxToUART2Rx(&g_uart1, &g_uart2);  /* Pin and Tx enable configuration. UART1 -> UART2 */
                    HAL_UART_WriteIT(&g_uart1, g_singleWireTx, UART_SINGLE_WIRE_TX_COUNT);
                }
                /* UART2 RX data. */
                ClearString(g_singleWireRx);
                HAL_UART_ReadIT(&g_uart2, g_singleWireRx, UART_SINGLE_WIRE_RX_COUNT);
                g_uartSingleWireStatus = UART_SINGLE_WIRE_BUSY;
                break;
            case UART_SINGLE_WIRE_RX_FIN:
                /* UART2 transmits data to UART1.  */
                /* You need to query the bus status before configuring the TX. */
                if (g_uart1NotBusy == BASE_CFG_ENABLE) {   /* Check whether the UART1 bus is busy. */
                    UART2TxToUART1Rx(&g_uart1, &g_uart2);  /* Pin and Tx enable configuration. UART2 -> UART1 */
                    HAL_UART_WriteIT(&g_uart2, g_singleWireTx, UART_SINGLE_WIRE_TX_COUNT);
                }
                /* UART2 RX data. */
                ClearString(g_singleWireRx);
                HAL_UART_ReadIT(&g_uart1, g_singleWireRx, UART_SINGLE_WIRE_RX_COUNT);
                g_uartSingleWireStatus = UART_SINGLE_WIRE_BUSY;
                break;
            case UART_SINGLE_WIRE_BUSY:                    /* Bus busy, pause for a while. */
                __asm__ volatile ("nop");
                break;
            default:
                break;
        }
        BASE_FUNC_DELAY_MS(UART_SINGLE_DELAY_TIMER);  /* Communication delay time (not mandatory). */
    }
    return;
}