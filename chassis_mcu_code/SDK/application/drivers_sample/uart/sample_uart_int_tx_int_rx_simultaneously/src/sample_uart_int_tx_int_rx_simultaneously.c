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
  * @file    sample_uart_int_tx_int_rx_simultaneously.c
  * @author  MCU Driver Team
  * @brief   uart sample module.
  * @details Transmits and receives data simultaneously by interrupt tx and rx. During interrupt writing, interrupts
  *          can be read.
  *          (1) Interrupt sending data: The start address and character length to be sent are used as input
  *          parameters and transferred to HAL_UART_WriteIT().
  *          (2) Interrupt receive data: The start address and receive length are used as input parameters to
  *          HAL_UART_ReadIT().
  *          (3) The example program continuously transmits and receives interrupts. During interrupt writing,
  *          interrupt reading can be performed.
  */
#include "sample_uart_int_tx_int_rx_simultaneously.h"

#define RX_IT_DATA_LENGTH 10
#define TX_IT_DATA_LENGTH 15
#define REQUIRE_TIME_IT 30

static unsigned char g_txITStr[TX_IT_DATA_LENGTH] = "123456789012345"; /* The transmit data length is 15 */
static unsigned char g_rxITStr[RX_IT_DATA_LENGTH] = {0};               /* The receive data length is 10 */
volatile bool txITFlag = false;                                     /* This parameter is frequently changed in
                                                                     the main function and callback function,
                                                                     indicating whether transmission is complete */

volatile bool rxITFlag = false;                                     /* This parameter is frequently changed in
                                                                     the main function and callback function,
                                                                     indicating whether reception is complete */

/**
 * @brief Clear string.
 * @param str, String to be cleared.
 * @retval None.
 */
static void ClearString(unsigned char *str)
{
    for (unsigned int i = 0; i < RX_IT_DATA_LENGTH; ++i) {
        str[i] = 0;
    }
}

/**
 * @brief User-defined read completion interrupt callback function.
 * @param UART_Handle UART handle.
 * @retval None.
 */
void ReadCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("Read Finish: %s\r\n", g_rxITStr);
    rxITFlag = true;
    return;
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
    txITFlag = true;
    return;
}

/**
 * @brief UART Interrupt Tx and Rx simultaneously.
 * @param None.
 * @retval None.
 */
void UART_INTTxAndINTRxSimultaneously(void)
{
    SystemInit();
    DBG_PRINTF("UART Init finish, UART intrrupt tx and interrupt rx simultaneously mode:\r\n");
    DBG_PRINTF("Tx transmits data 123456789012345, and Rx receives data with the length of 10 \r\n");

    txITFlag = true; /* Enable interrupt transmission */
    rxITFlag = true; /* Enable interrupt reception */
    while (1) {
        if (txITFlag) {
            txITFlag = false;
            /* UART interrupt send data */
            HAL_UART_WriteIT(&g_uart, g_txITStr, TX_IT_DATA_LENGTH);
        }

        if (rxITFlag) {
            rxITFlag = false;
            /* Clear received data */
            ClearString(g_rxITStr);
            /* Length of the received data must be equal to the RX_IT_DATA_LENGTH */
            HAL_UART_ReadIT(&g_uart, g_rxITStr, RX_IT_DATA_LENGTH);
        }
        BASE_FUNC_DELAY_MS(REQUIRE_TIME_IT); /* Add a deletion delay as required */
    }
    return;
}