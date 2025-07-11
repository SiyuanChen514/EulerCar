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
  * @file    sample_dma_per_to_per.c
  * @author  MCU Driver Team
  * @brief   dma sample module, peripheral-to-peripheral transfer.
  * @details This file provides sample code for users to help use
  *          the data transfer function of the dma.
  */
#include "sample_dma_per_to_per.h"

static UART_Handle g_uart1;
static UART_Handle g_uart0;
static DMA_Handle g_dmac;
static DMA_ChannelParam g_param;

/**
  * @brief DMA controller initialization.
  * @param None.
  * @retval None.
  */
static void DMA_ControllerInit(void)
{
    g_dmac.baseAddress = DMA;
    g_dmac.srcByteOrder = DMA_BYTEORDER_SMALLENDIAN;
    g_dmac.destByteOrder = DMA_BYTEORDER_SMALLENDIAN;
    g_dmac.irqNumTc = IRQ_DMA_TC;
    g_dmac.irqNumError = IRQ_DMA_ERR;
    HAL_DMA_Init(&g_dmac);
}

/**
  * @brief DMA interrupt initialization.
  * @param handle DMA handle.
  * @retval None.
  */
static void DMA_InterruptInit(DMA_Handle *handle)
{
    IRQ_Enable();
    handle->irqNumTc = IRQ_DMA_TC;
    handle->irqNumError = IRQ_DMA_ERR;
    HAL_DMA_IRQService(handle);
    IRQ_EnableN(IRQ_DMA_TC);
    IRQ_EnableN(IRQ_DMA_ERR);
}

/**
  * @brief User-defined callback function for completing the transfer of peripheral to the peripheral.
  * @param handle callback handle.
  * @retval None.
  */
static void DMA_PeriToPeriFinish(void *handle)
{
    DBG_PRINTF("Interrupt Finish!\r\n");
    BASE_FUNC_UNUSED(handle);
}

/**
  * @brief DMA sample code for the transfer of peripheral to the peripheral.
  * @param None.
  * @retval None.
  */
int DMA_PeriphToPeriphIT(void)
{
    g_uart0.baseAddress = UART0;
    g_uart0.baudRate = 115200;  /* baud rate is 115200 */
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.fifoMode = true;
    g_uart0.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.hwFlowCtr = UART_HW_FLOWCTR_DISABLE;
    HAL_UART_Init(&g_uart0);

    g_uart1.baseAddress = UART1;
    g_uart1.baudRate = 115200;  /* baud rate is 115200 */
    g_uart1.dataLength = UART_DATALENGTH_8BIT;
    g_uart1.stopBits = UART_STOPBITS_ONE;
    g_uart1.parity = UART_PARITY_NONE;
    g_uart1.fifoMode = true;
    g_uart1.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart1.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart1.hwFlowCtr = UART_HW_FLOWCTR_DISABLE;
    HAL_UART_Init(&g_uart1);
    
    DBG_PRINTF("PeriphToPeriph Begin: \r\n");
    unsigned int channel = 0;  /* select transfer channel 0 */
    DMA_ControllerInit();
    DMA_InterruptInit(&g_dmac);
    HAL_DMA_RegisterCallback(&g_dmac, DMA_CHANNEL_FINISH, channel, DMA_PeriToPeriFinish);
    g_param.direction = DMA_PERIPH_TO_PERIPH_BY_DMAC;
    g_param.srcAddrInc = DMA_ADDR_UNALTERED;
    g_param.destAddrInc = DMA_ADDR_UNALTERED;
    g_param.destPeriph = DMA_REQUEST_UART0_TX;
    g_param.srcPeriph = DMA_REQUEST_UART1_RX;
    g_param.srcWidth = DMA_TRANSWIDTH_BYTE;
    g_param.destWidth = DMA_TRANSWIDTH_BYTE;
    g_param.srcBurst = DMA_BURST_LENGTH_1;
    g_param.destBurst = DMA_BURST_LENGTH_1;
    g_param.pHandle = &g_dmac;
    HAL_DMA_InitChannel(&g_dmac, &g_param, channel);
    unsigned int ret;
    ret = HAL_DMA_StartIT(&g_dmac, (uintptr_t)(void *)&(g_uart1.baseAddress->UART_DR),
                          (uintptr_t)(void *)&(g_uart0.baseAddress->UART_DR), 8, channel);  /* Tx length is 8 */
    if (ret == BASE_STATUS_ERROR) {
        DBG_PRINTF("HAL_DMA_StartIT: BASE_STATUS_ERROR\r\n");
    } else {
        g_uart1.baseAddress->UART_DMACR.BIT.rxdmae = BASE_CFG_ENABLE;
        g_uart0.baseAddress->UART_DMACR.BIT.txdmae = BASE_CFG_ENABLE;
    }
    return 0;
}