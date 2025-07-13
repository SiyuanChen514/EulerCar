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
  * @file    sample_uart_dma_tx_dma_rx_simultaneously.c
  * @author  MCU Driver Team
  * @brief   uart sample module, DMA transfer simultaneously.
  * @details Transmits and receives UART_DMA data simultaneously. During UART_DMA write, UART_DMA can be read.
  *          (1) UART_DMA transmit data: The to-be-sent start address and character length are used as input
  *          parameters and transferred to HAL_UART_WriteDMA().
  *          (2) UART_DMA receive data: The start address and receive length are used as input parameters to
  *          HAL_UART_ReadDMA().
  *          (3) The sample program continuously transmits and receives UART_DMA. During UART_DMA write,
  *          UART_DMA can be read.
  */

#include "sample_uart_dma_tx_dma_rx_simultaneously.h"

#define DMA_RX_DATA_LENGTH 10
#define DMA_TX_DATA_LENGTH 15
#define NEED_REQUIRE_TIME 30

static unsigned char g_txDMAStr[DMA_TX_DATA_LENGTH] = "123456789012345"; /* The transmit data length is 15 */
static unsigned char g_rxDMAStr[DMA_RX_DATA_LENGTH] = {0};               /* The receive data length is 10 */
volatile bool txDMAFlag = false;                                     /* This parameter is frequently changed in
                                                                     the main function and callback function,
                                                                     indicating whether the transmission is complete */

volatile bool rxDMAFlag = false;                                     /* This parameter is frequently changed in
                                                                     the main function and callback function,
                                                                     indicating whether the reception is complete */

/**
 * @brief Clear string.
 * @param str, String to be cleared.
 * @retval None.
 */
static void ClearString(unsigned char *str)
{
    for (unsigned int i = 0; i < DMA_RX_DATA_LENGTH; ++i) {
        str[i] = 0;
    }
}

/**
 * @brief User-defined write completion DMA write finish callback function.
 * @param handle UART handle.
 * @retval None.
 */
void DMA_Channel3CallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("\r\nwrite_finish\r\n");
    txDMAFlag = true;
    return;
}

/**
 * @brief User-defined write completion DMA read finish callback function.
 * @param handle UART handle.
 * @retval None.
 */
void DMA_Channel2CallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("\r\nread_finish: %s\r\n", g_rxDMAStr);
    rxDMAFlag = true;
    return;
}

/**
 * @brief UART send and receive sample code in DMA mode.
 * @param None.
 * @retval None.
 */
void UART_DMATxAndRxSimultaneously(void)
{
    SystemInit();
    DBG_PRINTF("UART and DMA Init finish, UART DMA Tx DMA Rx simultaneously mode:\r\n");
    DBG_PRINTF("Tx transmits data 123456789012345, and Rx receives data with the length of 10 \r\n");

    txDMAFlag = true; /* Enable DMA transmission */
    rxDMAFlag = true; /* Enable DMA reception */
    while (1) {
        if (txDMAFlag) {
            txDMAFlag = false;
            HAL_UART_WriteDMA(&g_uart, g_txDMAStr, DMA_TX_DATA_LENGTH); /* DMA transmit data */
        }

        if (rxDMAFlag) {
            rxDMAFlag = false;
            ClearString(g_rxDMAStr);                           /* Clear the received data */
            /* DMA read: Length of the received data must be equal to the DMA_RX_DATA_LENGTH */
            HAL_UART_ReadDMA(&g_uart, g_rxDMAStr, DMA_RX_DATA_LENGTH);
        }
        /* Add a deletion delay as required */
        BASE_FUNC_DELAY_MS(NEED_REQUIRE_TIME);
    }
    return;
}