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
  * @file    uart.h
  * @author  MCU Driver Team
  * @brief   UART module driver.
  * @details This file provides functions declaration of the UART,
  *           + Initialization and de-initialization functions
  *           + Peripheral querying the state functions.
  *           + Peripheral transmit and abort functions.
  *           + Peripheral interrupt service and callback registration functions.
  *          This file also provides the definition of the UART handle structure.
  */

/* Includes ------------------------------------------------------------------*/
#ifndef McuMagicTag_UART_H
#define McuMagicTag_UART_H

#include "uart_ip.h"
#include "dma.h"

/**
  * @defgroup UART UART
  * @brief UART module.
  * @{
  */

/**
  * @defgroup UART_Common UART Common
  * @brief UART common external module.
  * @{
  */

/**
  * @defgroup UART_Handle_Definition UART Handle Definition
  * @{
  */

/**
  * @brief The definition of the UART handle structure.
  */
typedef struct _UART_Handle {
    UART_RegStruct                  *baseAddress;        /**< UART registers base address */
    unsigned int                     baudRate;           /**< UART communication baud rate */
    UART_DataLength                  dataLength;         /**< The length of UART frame */
    UART_StopBits                    stopBits;           /**< The stop bit of UART frame */
    UART_Parity_Mode                 parity;             /**< The parity bit of UART frame */
    UART_Transmit_Mode               txMode;             /**< Tx transmit mode setting */
    UART_Transmit_Mode               rxMode;             /**< tx transmit mode setting */
    volatile unsigned char          *txbuff;             /**< Start address of tx */
    volatile unsigned char          *rxbuff;             /**< Start address of rx */
    volatile unsigned int            txBuffSize;         /**< The length of tx buff */
    volatile unsigned int            rxBuffSize;         /**< The length of rx buff */
    bool                             fifoMode;           /**< The FIFO mode */
    UART_FIFO_Threshold              fifoTxThr;          /**< Interrupt threshold of tx FIFO */
    UART_FIFO_Threshold              fifoRxThr;          /**< Interrupt threshold of rx FIFO */
    UART_HW_FlowCtr                  hwFlowCtr;          /**< UART hardware flow control */
    DMA_Handle                      *dmaHandle;          /**< UART_DMA control */
    unsigned int                     uartDmaTxChn;       /**< UART_DMA tx channel */
    unsigned int                     uartDmaRxChn;       /**< UART_DMA rx channel */
    volatile UART_State_Type         txState;            /**< The tx status of UART */
    volatile UART_State_Type         rxState;            /**< The rx status of UART */
    UART_Error_Type                  errorType;          /**< The error of UART */

    UART_UserCallBack                userCallBack;       /**< User callback function of UART. */
    UART_ExtendHandle                handleEx;           /**< UART extend handle. */
} UART_Handle;

typedef void (* UART_CallbackType)(void *handle);
/**
  * @}
  */

/**
  * @defgroup UART_API_Declaration UART HAL API
  * @{
  */
/*  Peripheral initialization and deinitialize functions */
BASE_StatusType HAL_UART_Init(UART_Handle *uartHandle);
BASE_StatusType HAL_UART_DeInit(UART_Handle *uartHandle);

/* Peripheral querying the state functions */
UART_State_Type HAL_UART_GetState(UART_Handle *uartHandle);

/* Peripheral transmit and abort functions */
BASE_StatusType HAL_UART_WriteBlocking(UART_Handle *uartHandle, unsigned char *srcData,
                                       unsigned int dataLength, unsigned int blockingTime);
BASE_StatusType HAL_UART_WriteIT(UART_Handle *uartHandle, unsigned char *srcData, unsigned int dataLength);
BASE_StatusType HAL_UART_WriteDMA(UART_Handle *uartHandle, unsigned char *srcData,
                                  unsigned int dataLength);
BASE_StatusType HAL_UART_ReadBlocking(UART_Handle *uartHandle, unsigned char *saveData,
                                      unsigned int dataLength, unsigned int blockingTime);
BASE_StatusType HAL_UART_ReadIT(UART_Handle *uartHandle, unsigned char *saveData, unsigned int dataLength);
BASE_StatusType HAL_UART_ReadDMA(UART_Handle *uartHandle, unsigned char *saveData,
                                 unsigned int dataLength);
BASE_StatusType HAL_UART_StopRead(UART_Handle *uartHandle);
BASE_StatusType HAL_UART_StopWrite(UART_Handle *uartHandle);

/* brief Peripheral interrupt service and callback registration functions */
void HAL_UART_IrqHandler(void *handle);
BASE_StatusType HAL_UART_RegisterCallBack(UART_Handle *uartHandle, UART_CallbackFun_Type typeID,
                                          UART_CallbackType pCallback);

/* UART read using DMA cyclically stored function */
BASE_StatusType HAL_UART_ReadDMAAndCyclicallyStored(UART_Handle *uartHandle, unsigned char *saveData,
                                                    DMA_LinkList *tempNode, unsigned int dataLength);
unsigned int HAL_UART_ReadDMAGetPos(UART_Handle *uartHandle);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_UART_H */