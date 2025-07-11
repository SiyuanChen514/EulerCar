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
  * @file    sample_uart_blocking_rx.c
  * @author  MCU Driver Team
  * @brief   uart sample module.
  * @details In blocking mode, the ADC blocks for a period of time and waits for the UART to receive data.
  *          The received data is read from the preset buffer.
  *          (1)Preset buffer: The user needs to create the memory space.
  *          (2)Blocking time: It is as HAL_UART_ReadBlocking() parameter and the unit is ms.
  *          (3)Result judgment: BASE_STATUS_OK, indicating that data receive successfully. The return value is
  *          BASE_STATUS_TIMEOUT, indicating that data reception fails within the specified time.
  *          Other return values, indicating an error.
  */
#include "sample_uart_blocking_rx.h"
#define BLOCKING_TIME 2000

/**
  * @brief UART blocking receive sample code.
  * @param None.
  * @retval None.
  */
void UART_BlcokingRX(void)
{
    SystemInit();
    unsigned char rxStr[20] = {0};  /* rxStr[20], Receive memory address */
    DBG_PRINTF("RX: UART Init finish\r\n");
    unsigned int ret;
    while (1) {
        ret = HAL_UART_ReadBlocking(&g_uart, rxStr, 5, BLOCKING_TIME);  /* 5 is data length, 2000 is timeout limit */
        if (ret == BASE_STATUS_OK) {
            DBG_PRINTF("Receive success: %s \r\n", rxStr);
        } else if (ret == BASE_STATUS_TIMEOUT) {
            DBG_PRINTF("Receive time out!\r\n");
        } else {
            DBG_PRINTF("Receive verification error!\r\n");
        }
    }
}