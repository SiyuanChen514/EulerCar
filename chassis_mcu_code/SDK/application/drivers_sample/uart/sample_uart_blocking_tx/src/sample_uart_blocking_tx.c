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
  * @file    sample_uart_blocking_tx.c
  * @author  MCU Driver Team
  * @brief   uart sample module.
  * @details In block mode, the UART is blocked for a period of time and waits for the UART to transmit data.
  *          (1) Data to be sent: The start address and character length to be sent are used as input parameters
  *          and transferred to HAL_UART_WriteBlocking().
  *          (2)Blocking time: It is as HAL_UART_WriteBlocking() parameter and the unit is ms.
  *          (3)Result judgment: BASE_STATUS_OK, indicating that send data successfully. The return value is
  *          BASE_STATUS_TIMEOUT, indicating that send data fails within the specified time.
  *          Other return values, indicating an error.
  */
#include "sample_uart_blocking_tx.h"

#define BAND_RATE 115200
#define BLOCKING_TIME 2000

/**
  * @brief UART blocking send sample code.
  * @param None.
  * @retval None.
  */
void UART_BlcokingTX(void)
{
    SystemInit();
    DBG_PRINTF("TX: UART Init finish\r\n");
    unsigned char txStr[] = "UARTWrite";
    unsigned int ret;
    ret = HAL_UART_WriteBlocking(&g_uart, txStr, 9, BLOCKING_TIME);  /* 9 is transmission length */
    if (ret == BASE_STATUS_OK) {
        DBG_PRINTF("Send success!\r\n");        /* Sending succeeded */
    } else if (ret == BASE_STATUS_TIMEOUT) {
        DBG_PRINTF("Send time out!\r\n");       /* Sending failed */
    } else {
        DBG_PRINTF("Send verification error!\r\n");
    }
}