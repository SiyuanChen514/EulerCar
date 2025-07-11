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
  * @file    sample_uart_int_tx_dma_rx_simultaneously.c
  * @author  MCU Driver Team
  * @brief   uart sample module.
  * @details Transmits and receive data simultaneously by interrupt tx and dma rx. During interrupt writing,
  *          UART_DMA can be read.
  *          (1) Interrupt sending data: The start address and character length to be sent are used as input parameters
  *          and transferred to HAL_UART_WriteIT().
  *          (2) UART_DMA receive data: The start address and receive length are used as input parameters to
  *          HAL_UART_ReadDMA().
  *          (3) The sample program continuously transmits interrupts and receives UART_DMA. During interrupt writing,
  *          UART_DMA can be read.
  */

#include "sample_uart_int_tx_dma_rx_simultaneously.h"

#define RX_IT_DMA_DATA_LENGTH 10
#define TX_IT_DMA_DATA_LENGTH 15
#define IT_DMA_REQUIRE_TIME 30

static unsigned char g_txITDMAStr[TX_IT_DMA_DATA_LENGTH] = "123456789012345"; /* The transmit data length is 15 */
static unsigned char g_rxITDMAStr[RX_IT_DMA_DATA_LENGTH] = {0};               /* The receive data length is 10 */
volatile bool txITDMAFlag = false;                                /* This parameter is frequently changed in
                                                                  the main function and callback function,
                                                                  indicating whether the transmission is complete */

volatile bool rxITDMAFlag = false; /* This parameter is frequently changed in
                                   the main function and callback function,
                                   indicating whether the reception is complete */

/**
 * @brief Clear string.
 * @param str, String to be cleared.
 * @retval None.
 */
static void ClearString(unsigned char *str)
{
    for (unsigned int i = 0; i < RX_IT_DMA_DATA_LENGTH; ++i) {
        str[i] = 0;
    }
}

/**
 * @brief User-defined write completion interrupt callback function.
 * @param UART_Handle UART handle.
 * @retval None.
 */
void WriteCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("\r\nWrite Finish\r\n");
    txITDMAFlag = true;
    return;
}

/**
 * @brief User-defined read completion DMA callback function.
 * @param handle UART handle.
 * @retval None.
 */
void DMA_Channel2CallBack(void *handle)
{
    UART_Handle *uartHandle = (UART_Handle *)handle;
    DBG_PRINTF("read_finish: %s\r\n", g_rxITDMAStr);
    BASE_FUNC_UNUSED(uartHandle);  /* Use uart handle. */
    rxITDMAFlag = true;
    return;
}

/**
 * @brief UART Interrupt Tx and DMA Rx simultaneously.
 * @param None.
 * @retval None.
 */
void UART_INTTxAndDMARxSimultaneously(void)
{
    SystemInit();
    DBG_PRINTF("UART Init finish, Interrupt Tx and DMA Rx simultaneously mode:\r\n");
    DBG_PRINTF("Tx transmits data 123456789012345, and Rx receives data with the length of 10 \r\n");

    txITDMAFlag = true; /* Enable IT transmission */
    rxITDMAFlag = true; /* Enable DMA reception */
    while (1) {
        if (txITDMAFlag) {
            txITDMAFlag = false;
            /* transmit data using UART_IT */
            HAL_UART_WriteIT(&g_uart, g_txITDMAStr, TX_IT_DMA_DATA_LENGTH);
        }

        if (rxITDMAFlag) {
            rxITDMAFlag = false;
            ClearString(g_rxITDMAStr);  /* Clear the array of g_rxITDMAStr */
            /* DMA read: Length of the received data must be equal to the RX_IT_DMA_DATA_LENGTH */
            HAL_UART_ReadDMA(&g_uart, g_rxITDMAStr, RX_IT_DMA_DATA_LENGTH);
        }
        BASE_FUNC_DELAY_MS(IT_DMA_REQUIRE_TIME); /* Add a deletion delay as required */
    }
    return;
}