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
  * @file    uart.c
  * @author  MCU Driver Team
  * @brief   UART module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the UART.
  *           + Initialization and de-initialization functions.
  *           + Peripheral send and receive functions in blocking mode.
  *           + Peripheral send and receive functions in interrupt mode.
  *           + Peripheral send and receive functions in DMA mode.
  *           + Peripheral stop sending and receiving functions in interrupt/DMA mode.
  *           + Interrupt callback function and user registration function.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "systick.h"
#include "uart.h"
/* Macro definitions ---------------------------------------------------------*/

#define OVERSAMPLING_PARAM 16
#define SYSTICK_MS_DIV 1000

static unsigned int GreaterMaxraund(unsigned int clk)
{
    return clk * 8;  /* 8 is greaterMaxraund param */
}
static unsigned int SmallerMaxraund(unsigned int clk)
{
    return clk * 4;  /* 4 is smallerMaxraund param */
}

static unsigned int DivClosest(unsigned int x, unsigned int divisor)
{
    unsigned int ret;
    if (divisor == 0) {
        return 0;
    }
    ret = (((x) + ((divisor) / 2)) / (divisor));  /* Round up the result, add 1/2 */
    return ret;
}

static void WriteDMAFinishFun(void *handle);
static void ReadDMAFinishFun(void *handle);
static void TransmitDMAErrorFun(void *handle);

static void ReadITCallBack(UART_Handle *uartHandle);
static void WriteITCallBack(UART_Handle *uartHandle);
static void ErrorServiceCallback(UART_Handle *uartHandle);

/**
  * @brief Initialize the UART hardware configuration and configure parameters based on the specified handle.
  * @param uartHandle UART handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_UART_Init(UART_Handle *uartHandle)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_PARAM_CHECK_WITH_RET(uartHandle->txState == UART_STATE_NONE_INIT, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(uartHandle->rxState == UART_STATE_NONE_INIT, BASE_STATUS_ERROR);
    unsigned int uartClock, quot;
    UART_PARAM_CHECK_WITH_RET(IsUartDatalength(uartHandle->dataLength), BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(IsUartStopbits(uartHandle->stopBits), BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(IsUartParitymode(uartHandle->parity), BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(IsUartTransmode(uartHandle->txMode), BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(IsUartTransmode(uartHandle->rxMode), BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(IsUartFIFOThreshold(uartHandle->fifoTxThr), BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(IsUartFIFOThreshold(uartHandle->fifoRxThr), BASE_STATUS_ERROR);

    while (uartHandle->baseAddress->UART_FR.BIT.busy == 0x01) {
        ;
    }
    uartHandle->baseAddress->UART_CR.BIT.uarten = BASE_CFG_DISABLE;
    uartClock = HAL_CRG_GetIpFreq((void *)uartHandle->baseAddress);
    /* The baud rate divider(BRD) based on the baud rate and clock frequency, calculation formula */
    if (uartHandle->baudRate > (uartClock / OVERSAMPLING_PARAM)) {
        quot = DivClosest(GreaterMaxraund(uartClock), uartHandle->baudRate);
    } else {
        quot = DivClosest(SmallerMaxraund(uartClock), uartHandle->baudRate);
    }
    /* Clear the baud rate divider register */
    uartHandle->baseAddress->UART_FBRD.reg = 0;
    uartHandle->baseAddress->UART_IBRD.reg = 0;
    /* The fractional baud rate divider value is stored to the lower 6 bits of the FBRD */
    uartHandle->baseAddress->UART_FBRD.reg = (quot & 0x3F);
    /* Right shift 6 bits is the integer baud rate divider value, is stored to IBRD */
    uartHandle->baseAddress->UART_IBRD.reg = (quot >> 6);

    uartHandle->baseAddress->UART_LCR_H.BIT.wlen = uartHandle->dataLength;      /* Frame length setting */
    uartHandle->baseAddress->UART_LCR_H.BIT.stp2 = uartHandle->stopBits;        /* Stop bit setting */
    if (uartHandle->parity == UART_PARITY_NONE) {                               /* Parity setting */
        uartHandle->baseAddress->UART_LCR_H.BIT.pen = BASE_CFG_DISABLE;
    } else {
        unsigned int val = 0x0002;                          /* UART_LCR_H [1]bit enable */
        val |= (uartHandle->parity) << 2;                   /* Set uartHandle->parity into UART_LCR_H [2]bit */
        uartHandle->baseAddress->UART_LCR_H.reg |= val;
    }
    if (uartHandle->fifoMode == true) {                     /* FIFO threshold setting */
        uartHandle->baseAddress->UART_LCR_H.BIT.fen = BASE_CFG_ENABLE;
        uartHandle->baseAddress->UART_IFLS.BIT.rxiflsel = uartHandle->fifoRxThr;
        uartHandle->baseAddress->UART_IFLS.BIT.txiflsel = uartHandle->fifoTxThr;
    }
    if (uartHandle->hwFlowCtr == UART_HW_FLOWCTR_ENABLE) {  /* Hardwarer flow control setting */
        uartHandle->baseAddress->UART_CR.reg |= 0xC000;
    }
    uartHandle->baseAddress->UART_CR.reg |= 0x301;          /* Enable bit use 0x301 is to set txe/rxe/uarten */
    uartHandle->txState = UART_STATE_READY;
    uartHandle->rxState = UART_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief DeInitialize the UART and restoring default parameters based on the specified handle.
  * @param uartHandle UART handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_UART_DeInit(UART_Handle *uartHandle)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    uartHandle->baseAddress->UART_CR.reg = BASE_CFG_DISABLE;
    uartHandle->baseAddress->UART_ICR.reg |= 0xFFFF;                        /* Clear all interruptions */
    uartHandle->baseAddress->UART_IMSC.reg = BASE_CFG_DISABLE;
    uartHandle->baseAddress->UART_DMACR.reg = BASE_CFG_DISABLE;
    uartHandle->baseAddress->UART_LCR_H.BIT.brk = BASE_CFG_DISABLE;
    uartHandle->baseAddress->UART_LCR_H.BIT.fen = BASE_CFG_DISABLE;
    uartHandle->userCallBack.WriteItFinishCallBack = NULL;          /* Clear all user call back function. */
    uartHandle->userCallBack.ReadItFinishCallBack = NULL;
    uartHandle->userCallBack.TransmitItErrorCallBack = NULL;
    uartHandle->userCallBack.WriteDmaFinishCallBack = NULL;        /* Clear user call back function of DMA. */
    uartHandle->userCallBack.ReadDmaFinishCallBack = NULL;
    uartHandle->userCallBack.TransmitDmaErrorCallBack = NULL;
    uartHandle->txState = UART_STATE_NONE_INIT;         /* Clear all state */
    uartHandle->rxState = UART_STATE_NONE_INIT;
    return BASE_STATUS_OK;
}

/**
  * @brief Return the specified UART state.
  * @param uartHandle UART handle.
  * @retval UART state: UART_STATE_NONE_INIT(can not use), UART_STATE_READY, UART_STATE_BUSY
  * @retval UART_STATE_BUSY_TX, UART_STATE_BUSY_RX.
  */
UART_State_Type HAL_UART_GetState(UART_Handle *uartHandle)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    if (uartHandle->txState == UART_STATE_NONE_INIT) {
        return UART_STATE_NONE_INIT;            /* Uart Tx and Rx are not initialized */
    }
    if (uartHandle->txState == UART_STATE_READY && uartHandle->rxState == UART_STATE_READY) {
        return UART_STATE_READY;               /* Uart Tx and Rx are ready */
    }
    if (uartHandle->txState == UART_STATE_READY) {
        return UART_STATE_BUSY_RX;            /* Uart Rx is busy */
    }
    if (uartHandle->rxState == UART_STATE_READY) {
        return UART_STATE_BUSY_TX;           /* Uart Tx is busy */
    }
    return UART_STATE_BUSY;                 /* Uart Tx and Rx are busy */
}

/**
  * @brief Send data in blocking mode.
  * @param uartHandle UART handle.
  * @param srcData Address of the data buff to be sent.
  * @param dataLength number of the data to be sent.
  * @param blockingTime Blocking time, unit: milliseconds.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_UART_WriteBlocking(UART_Handle *uartHandle, unsigned char *srcData,
                                       unsigned int dataLength, unsigned int blockingTime)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_ASSERT_PARAM(srcData != NULL);
    UART_PARAM_CHECK_WITH_RET(uartHandle->txMode == UART_MODE_BLOCKING, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(dataLength > 0, BASE_STATUS_ERROR);
    unsigned long long setTick = SYSTICK_GetCRGHZ() / SYSTICK_MS_DIV * blockingTime;
    UART_PARAM_CHECK_WITH_RET(setTick < SYSTICK_MAX_VALUE, BASE_STATUS_ERROR);
    if (uartHandle->txState == UART_STATE_READY) {
        uartHandle->txState = UART_STATE_BUSY_TX;
        unsigned int txCount = dataLength;
        unsigned char *src = srcData;
        uartHandle->baseAddress->UART_IMSC.BIT.txim = BASE_CFG_DISABLE;  /* Disable TX interrupt bit */
        uartHandle->baseAddress->UART_CR.BIT.txe = BASE_CFG_ENABLE;
        unsigned long long deltaTick;
        unsigned int preTick = DCL_SYSTICK_GetTick();
        unsigned int curTick = preTick;
        while (txCount > 0x00) {
            curTick = DCL_SYSTICK_GetTick();
            deltaTick = (curTick > preTick) ? (curTick - preTick) : (SYSTICK_MAX_VALUE - preTick + curTick);
            if (deltaTick >= setTick) {
                uartHandle->txState = UART_STATE_READY;
                return BASE_STATUS_TIMEOUT;
            }
            if (uartHandle->baseAddress->UART_FR.BIT.txff == 0x01) {    /* True when the TX FIFO is full */
                continue;
            }
            /* Blocking write to DR when register is empty */
            uartHandle->baseAddress->UART_DR.BIT.data = *(src);
            src++;
            txCount--;
        }
    } else {
        return BASE_STATUS_BUSY;
    }
    uartHandle->txState = UART_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief Send data in interrupt mode.
  * @param uartHandle UART handle.
  * @param srcData Address of the data buff to be sent.
  * @param dataLength Number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_UART_WriteIT(UART_Handle *uartHandle, unsigned char *srcData, unsigned int dataLength)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_PARAM_CHECK_WITH_RET(uartHandle->txMode == UART_MODE_INTERRUPT, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(srcData != NULL, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(dataLength > 0, BASE_STATUS_ERROR);

    if (uartHandle->txState == UART_STATE_READY) {
        uartHandle->txState = UART_STATE_BUSY_TX;
        uartHandle->txbuff = srcData;
        uartHandle->txBuffSize = dataLength;
        uartHandle->baseAddress->UART_ICR.BIT.txic = BASE_CFG_ENABLE;
        uartHandle->baseAddress->UART_IMSC.BIT.txim = BASE_CFG_ENABLE;
        WriteITCallBack(uartHandle);
    } else {
        return BASE_STATUS_BUSY;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Interrupt Clearing and Interrupt Callback.
  * @param uartHandle UART handle.
  * @retval None.
  */
static void WriteItCheck(UART_Handle *uartHandle)
{
    if (uartHandle->txBuffSize == 0) {
        uartHandle->baseAddress->UART_IMSC.BIT.txim = BASE_CFG_DISABLE;
        uartHandle->baseAddress->UART_ICR.BIT.txic = BASE_CFG_ENABLE;
        uartHandle->txState = UART_STATE_READY;
        if (uartHandle->userCallBack.WriteItFinishCallBack != NULL) {
            uartHandle->userCallBack.WriteItFinishCallBack(uartHandle);  /* user function callback */
        }
    } else {
        uartHandle->baseAddress->UART_IMSC.BIT.txim = BASE_CFG_ENABLE;
    }
}


/**
  * @brief Interrupt sending callback function.
  *        The hanler function is called when Tx interruption occurs.
  * @param uartHandle UART handle.
  * @retval None.
  */
static void WriteITCallBack(UART_Handle *uartHandle)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_ASSERT_PARAM(uartHandle->txbuff != NULL);
    uartHandle->baseAddress->UART_IMSC.BIT.txim = BASE_CFG_DISABLE;
    if (uartHandle->txState == UART_STATE_BUSY_TX) {
        while (uartHandle->txBuffSize > 0) {
            if (uartHandle->baseAddress->UART_FR.BIT.txff == 1) {  /* True when the TX FIFO is full */
                break;
            }
            uartHandle->txBuffSize -= 1;
            uartHandle->baseAddress->UART_DR.BIT.data = *(uartHandle->txbuff);
            (uartHandle->txbuff)++;
        }
        WriteItCheck(uartHandle);
    }
    return;
}

/**
  * @brief Send data in DMA mode.
  * @param uartHandle UART handle.
  * @param srcData Address of the data buff to be sent.
  * @param dataLength Number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_UART_WriteDMA(UART_Handle *uartHandle, unsigned char *srcData,
                                  unsigned int dataLength)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_PARAM_CHECK_WITH_RET(uartHandle->txMode == UART_MODE_DMA, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(srcData != NULL, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(dataLength > 0, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(IsDmaChannelNum(uartHandle->uartDmaTxChn) == true, BASE_STATUS_ERROR);
    UART_ASSERT_PARAM(uartHandle->dmaHandle != NULL);
    unsigned int channel = uartHandle->uartDmaTxChn;
    if (uartHandle->txState == UART_STATE_READY) {
        uartHandle->txState = UART_STATE_BUSY_TX;
        uartHandle->baseAddress->UART_IMSC.BIT.txim = BASE_CFG_DISABLE;          /* Disable TX interrupt bit */
        uartHandle->dmaHandle->userCallBack.DMA_CallbackFuns[channel].ChannelFinishCallBack = WriteDMAFinishFun;
        uartHandle->dmaHandle->userCallBack.DMA_CallbackFuns[channel].ChannelErrorCallBack = TransmitDMAErrorFun;
        uartHandle->txbuff = srcData;
        uartHandle->txBuffSize = dataLength;
        if (HAL_DMA_StartIT(uartHandle->dmaHandle, (uintptr_t)(void *)uartHandle->txbuff,
                            (uintptr_t)(void *)&(uartHandle->baseAddress->UART_DR), \
                            dataLength, channel) != BASE_STATUS_OK) {
            uartHandle->txState = UART_STATE_READY;
            return BASE_STATUS_ERROR;
        }
        uartHandle->baseAddress->UART_DMACR.BIT.txdmae = BASE_CFG_ENABLE;        /* Enable TX DMA bit */
    } else {
        return BASE_STATUS_BUSY;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Receive data in blocking mode.
  * @param uartHandle UART handle.
  * @param saveData Address of the data buff to be saved.
  * @param dataLength Length of the data int the storage buffer.
  * @param blockingTime Blocking time, unit: milliseconds.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_UART_ReadBlocking(UART_Handle *uartHandle, unsigned char *saveData,
                                      unsigned int dataLength, unsigned int blockingTime)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(saveData != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_PARAM_CHECK_WITH_RET(uartHandle->rxMode == UART_MODE_BLOCKING, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(dataLength > 0, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(blockingTime > 0, BASE_STATUS_ERROR);
    unsigned long long setTick = SYSTICK_GetCRGHZ() / SYSTICK_MS_DIV * blockingTime;
    UART_PARAM_CHECK_WITH_RET(setTick < SYSTICK_MAX_VALUE, BASE_STATUS_ERROR);
    if (uartHandle->rxState == UART_STATE_READY) {
        uartHandle->rxState = UART_STATE_BUSY_RX;
        unsigned int rxCount = dataLength;
        unsigned char *save = saveData;
        uartHandle->baseAddress->UART_IMSC.BIT.rxim = BASE_CFG_DISABLE;      /* Disable RX interrupt bit */
        uartHandle->baseAddress->UART_ICR.reg = 0XFF;                   /* Clear interrupt flag */
        unsigned int tmp;
        unsigned long long deltaTick;
        unsigned int preTick = DCL_SYSTICK_GetTick();
        unsigned int curTick = preTick;
        while (rxCount > 0) {
            curTick = DCL_SYSTICK_GetTick();
            deltaTick = (curTick > preTick) ? (curTick - preTick) : (SYSTICK_MAX_VALUE - preTick + curTick);
            if (deltaTick >= setTick) {
                uartHandle->rxState = UART_STATE_READY;
                return BASE_STATUS_TIMEOUT;
            }
            if (uartHandle->baseAddress->UART_FR.BIT.rxfe == 0x01) {
                continue;
            }
            tmp = uartHandle->baseAddress->UART_DR.reg;
            if (tmp & 0xF00) {                                      /* True when receiving generated error */
                uartHandle->rxState = UART_STATE_READY;
                return BASE_STATUS_ERROR;
            }
            *(save) = (tmp & 0xFF); /* The lower eight bits are the register data bits */
            save++;
            rxCount--;
        }
    } else {
        return BASE_STATUS_BUSY;
    }
    uartHandle->rxState = UART_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief Receive data in interrupt mode.
  * @param uartHandle UART handle.
  * @param saveData Address of the data buff to be saved.
  * @param dataLength length of the data int the storage buffer.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_UART_ReadIT(UART_Handle *uartHandle, unsigned char *saveData, unsigned int dataLength)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_ASSERT_PARAM(saveData != NULL);
    UART_PARAM_CHECK_WITH_RET(uartHandle->rxMode == UART_MODE_INTERRUPT, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(dataLength > 0, BASE_STATUS_ERROR);

    if (uartHandle->rxState == UART_STATE_READY) {
        uartHandle->rxState = UART_STATE_BUSY_RX;
        uartHandle->rxbuff = saveData;
        uartHandle->rxBuffSize = dataLength;
        uartHandle->baseAddress->UART_IMSC.reg |= 0x7D0;    /* Enable rx interrupt and rx timeout interrupt */
    } else {
        return BASE_STATUS_BUSY;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Interrupt receiving callback function.
  *        The hanler function is called when Rx interruption occurs.
  * @param uartHandle UART handle.
  * @retval None.
  */
static void ReadITCallBack(UART_Handle *uartHandle)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(uartHandle->rxbuff != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    if (uartHandle->rxState == UART_STATE_BUSY_RX) {
        unsigned int tmp;
        while (uartHandle->rxBuffSize > 0) {
            if (uartHandle->baseAddress->UART_FR.BIT.rxfe == 0x01) {    /* True when the RX FIFO is empty */
                break;
            }
            uartHandle->rxBuffSize -= 1;
            tmp = uartHandle->baseAddress->UART_DR.reg;
            *(uartHandle->rxbuff) = (tmp & 0xFF);     /* Read from DR when holding register/FIFO is not empty */
            uartHandle->rxbuff++;
        }
        if (uartHandle->rxBuffSize == 0) {
            uartHandle->baseAddress->UART_IMSC.reg &= 0xFFAF;   /* Disable rxim and rtim */
            uartHandle->rxState = UART_STATE_READY;
        }
        uartHandle->baseAddress->UART_ICR.reg |= 0x50;      /* Clear rxic and rtic */
        if (uartHandle->userCallBack.ReadItFinishCallBack != NULL && uartHandle->rxBuffSize == 0) {
            uartHandle->userCallBack.ReadItFinishCallBack(uartHandle);
        }
    }
    return;
}

/**
  * @brief Callback function of finishing receiving in DMA mode.
  *        The hanler function is called when Rx DMA Finish interruption occurs.
  * @param handle DMA handle.
  * @retval None.
  */
static void ReadDMAFinishFun(void *handle)
{
    UART_ASSERT_PARAM(handle != NULL);
    UART_Handle *uartHandle = (UART_Handle *)(handle);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    uartHandle->rxState = UART_STATE_READY;
    uartHandle->baseAddress->UART_DMACR.BIT.rxdmae = BASE_CFG_DISABLE;
    uartHandle->rxBuffSize = 0;
    if (uartHandle->userCallBack.ReadDmaFinishCallBack != NULL) {
        uartHandle->userCallBack.ReadDmaFinishCallBack(uartHandle);   /* User callback function */
    }
    return;
}

/**
  * @brief Callback function of finishing sending in DMA mode.
  *        The hanler function is called when Tx DMA Finish interruption occurs.
  * @param handle DMA handle.
  * @retval None.
  */
static void WriteDMAFinishFun(void *handle)
{
    UART_ASSERT_PARAM(handle != NULL);
    UART_Handle *uartHandle = (UART_Handle *)(handle);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    uartHandle->txState = UART_STATE_READY;
    uartHandle->baseAddress->UART_DMACR.BIT.txdmae = BASE_CFG_DISABLE;
    uartHandle->txBuffSize = 0;
    if (uartHandle->userCallBack.WriteDmaFinishCallBack != NULL) {
        uartHandle->userCallBack.WriteDmaFinishCallBack(uartHandle);          /* User callback function */
    }
    return;
}

/**
  * @brief Callback function of Tx/Rx error interrupt in DMA mode.
  *        The hanler function is called when Tx/Rx transmission error interruption occurs.
  * @param handle DMA handle.
  * @retval None.
  */
static void TransmitDMAErrorFun(void *handle)
{
    UART_ASSERT_PARAM(handle != NULL);
    UART_Handle *uartHandle = (UART_Handle *)(handle);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    if (uartHandle->rxState == UART_STATE_BUSY_RX) {
        uartHandle->baseAddress->UART_DMACR.BIT.rxdmae = BASE_CFG_DISABLE;
    }
    if (uartHandle->txState == UART_STATE_BUSY_TX) {
        uartHandle->baseAddress->UART_DMACR.BIT.txdmae = BASE_CFG_DISABLE;
    }
    if (uartHandle->userCallBack.TransmitDmaErrorCallBack != NULL) {
        uartHandle->userCallBack.TransmitDmaErrorCallBack(uartHandle);
    }
    uartHandle->txState = UART_STATE_READY;
    uartHandle->rxState = UART_STATE_READY;
    return;
}

/**
  * @brief Receive data in DMA mode.
  * @param uartHandle UART handle.
  * @param saveData Address of the data buff to be sent.
  * @param dataLength number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_UART_ReadDMA(UART_Handle *uartHandle, unsigned char *saveData,
                                 unsigned int dataLength)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_ASSERT_PARAM(saveData != NULL);
    UART_PARAM_CHECK_WITH_RET(uartHandle->rxMode == UART_MODE_DMA, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(dataLength > 0, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(IsDmaChannelNum(uartHandle->uartDmaRxChn) == true, BASE_STATUS_ERROR);
    UART_ASSERT_PARAM(uartHandle->dmaHandle != NULL);
    unsigned int channel = uartHandle->uartDmaRxChn;
    if (uartHandle->rxState == UART_STATE_READY) {
        uartHandle->rxState = UART_STATE_BUSY_RX;
        uartHandle->baseAddress->UART_IMSC.BIT.rxim = BASE_CFG_DISABLE;  /* Disable RX interrupt bit */
        uartHandle->dmaHandle->userCallBack.DMA_CallbackFuns[channel].ChannelFinishCallBack = ReadDMAFinishFun;
        uartHandle->dmaHandle->userCallBack.DMA_CallbackFuns[channel].ChannelErrorCallBack = TransmitDMAErrorFun;
        uartHandle->rxbuff = saveData;
        uartHandle->rxBuffSize = dataLength;
        /* Can not masking overflow error, break error, check error, frame error interrupt */
        if (HAL_DMA_StartIT(uartHandle->dmaHandle, (uintptr_t)(void *)&(uartHandle->baseAddress->UART_DR),
                            (uintptr_t)(void *)uartHandle->rxbuff, dataLength, channel) != BASE_STATUS_OK) {
            uartHandle->rxState = UART_STATE_READY;
            return BASE_STATUS_ERROR;
        }
        uartHandle->baseAddress->UART_DMACR.BIT.rxdmae = BASE_CFG_ENABLE;        /* Enable RX_DMA bit */
    } else {
        return BASE_STATUS_BUSY;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Stop the process of sending data in interrupt or DMA mode.
  * @param uartHandle UART handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_UART_StopWrite(UART_Handle *uartHandle)  /* Only support UART_MODE_INTERRUPT and UART_MODE_DMA */
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_ASSERT_PARAM(uartHandle->dmaHandle != NULL);
    UART_PARAM_CHECK_WITH_RET(IsDmaChannelNum(uartHandle->uartDmaRxChn) == true, BASE_STATUS_ERROR);
    /* Blocking send interrupt. */
    uartHandle->baseAddress->UART_IMSC.BIT.txim = BASE_CFG_DISABLE;
    if (uartHandle->txMode == UART_MODE_DMA) {
        uartHandle->baseAddress->UART_DMACR.BIT.txdmae = BASE_CFG_DISABLE; /* Disable TX FIFO of DMA. */
        if (HAL_DMA_StopChannel(uartHandle->dmaHandle, uartHandle->uartDmaTxChn) != BASE_STATUS_OK) {
            return BASE_STATUS_ERROR;
        }
    }
    uartHandle->txState = UART_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief Stop the process of receiving data in interrupt or DMA mode.
  * @param uartHandle UART handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_UART_StopRead(UART_Handle *uartHandle)  /* Only support UART_MODE_INTERRUPT and UART_MODE_DMA */
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_ASSERT_PARAM(uartHandle->dmaHandle != NULL);
    UART_PARAM_CHECK_WITH_RET(IsDmaChannelNum(uartHandle->uartDmaRxChn) == true, BASE_STATUS_ERROR);
    unsigned int val = uartHandle->baseAddress->UART_IMSC.reg;
    val &= 0xFFFFF82F;               /* Disable bits: rxim, rtim, feim, peim, beim, oeim */
    uartHandle->baseAddress->UART_IMSC.reg = val;
    if (uartHandle->rxMode == UART_MODE_DMA) {
        uartHandle->baseAddress->UART_DMACR.BIT.rxdmae = BASE_CFG_DISABLE;
        if (HAL_DMA_StopChannel(uartHandle->dmaHandle, uartHandle->uartDmaRxChn) != BASE_STATUS_OK) {
            return BASE_STATUS_ERROR;
        }
    }
    uartHandle->rxState = UART_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief Error handler function of receiving.
  * @param uartHandle UART handle.
  * @retval None.
  */
static void ErrorServiceCallback(UART_Handle *uartHandle)
{
    unsigned int error = 0x00;
    if (uartHandle->baseAddress->UART_MIS.BIT.oemis == BASE_CFG_ENABLE) {            /* Overflow error interrupt */
        error |= uartHandle->baseAddress->UART_MIS.BIT.oemis;
        uartHandle->baseAddress->UART_ICR.BIT.oeic = BASE_CFG_ENABLE;
    } else if (uartHandle->baseAddress->UART_MIS.BIT.bemis == BASE_CFG_ENABLE) {     /* Break error interrupt */
        error |= uartHandle->baseAddress->UART_MIS.BIT.bemis;
        uartHandle->baseAddress->UART_ICR.BIT.beic = BASE_CFG_ENABLE;
    } else if (uartHandle->baseAddress->UART_MIS.BIT.pemis == BASE_CFG_ENABLE) {     /* Check error interrupt */
        error |= uartHandle->baseAddress->UART_MIS.BIT.pemis;
        uartHandle->baseAddress->UART_ICR.BIT.peic = BASE_CFG_ENABLE;
    } else if (uartHandle->baseAddress->UART_MIS.BIT.femis == BASE_CFG_ENABLE) {     /* Frame error interrupt */
        error |= uartHandle->baseAddress->UART_MIS.BIT.femis;
        uartHandle->baseAddress->UART_ICR.BIT.feic = BASE_CFG_ENABLE;
    }
    if (error != 0x00) {
        uartHandle->errorType = error;
        if (uartHandle->rxMode == UART_MODE_INTERRUPT && uartHandle->userCallBack.TransmitItErrorCallBack != NULL) {
            uartHandle->userCallBack.TransmitItErrorCallBack(uartHandle);
        }
    }
    return;
}

/**
  * @brief UART Interrupt service processing function.
  * @param handle UART handle.
  * @retval None.
  */
void HAL_UART_IrqHandler(void *handle)
{
    UART_ASSERT_PARAM(handle != NULL);
    UART_Handle *uartHandle = (UART_Handle *)handle;
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    /* when tx interrupt is generated and register/FIFO is empty */
    if ((uartHandle->baseAddress->UART_MIS.BIT.txmis == 0x01) &&
        (uartHandle->baseAddress->UART_FR.BIT.txfe == 0x01)) {
        WriteITCallBack(uartHandle);
    }
    /* when rx interrupt is generated and register/FIFO is not empty */
    if ((uartHandle->baseAddress->UART_MIS.BIT.rxmis == 0x01 || uartHandle->baseAddress->UART_MIS.BIT.rtmis == 0x01) &&
        (uartHandle->baseAddress->UART_FR.BIT.rxfe != 0x1)) {
        ReadITCallBack(uartHandle);
    }
    if ((uartHandle->baseAddress->UART_MIS.reg & 0x780)) {
        ErrorServiceCallback(uartHandle);
    }
    return;
}

/**
  * @brief User callback function registration interface.
  * @param uartHandle UART handle.
  * @param typeID Id of callback function type, @ref UART_CallbackFun_Type
  * @param pCallback pointer of the specified callbcak function, @ref UART_CallbackType
  * @retval BASE_StatusType: OK, ERROR.
  */
BASE_StatusType HAL_UART_RegisterCallBack(UART_Handle *uartHandle, UART_CallbackFun_Type typeID,
                                          UART_CallbackType pCallback)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    switch (typeID) {
        case UART_WRITE_IT_FINISH:   /* uart write finish call back. */
            uartHandle->userCallBack.WriteItFinishCallBack = pCallback;
            break;
        case UART_READ_IT_FINISH:  /* uart read interrupt call back. */
            uartHandle->userCallBack.ReadItFinishCallBack = pCallback;
            break;
        case UART_WRITE_DMA_FINISH: /* uart write dma finish call back. */
            uartHandle->userCallBack.WriteDmaFinishCallBack = pCallback;
            break;
        case UART_READ_DMA_FINISH:  /* uart read dma finish call back. */
            uartHandle->userCallBack.ReadDmaFinishCallBack = pCallback;
            break;
        case UART_TRNS_IT_ERROR:  /* uart transmit error call back. */
            uartHandle->userCallBack.TransmitItErrorCallBack = pCallback;
            break;
        case UART_TRNS_DMA_ERROR:  /* uart transmit DMA error call back. */
            uartHandle->userCallBack.TransmitDmaErrorCallBack = pCallback;
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief UART DAM(rx to memory), cyclically stores data to specified memory(saveData).
  * @param uartHandle UART handle.
  * @param saveData Address of the data buff to be sent.
  * @param tempNode DMA Link List, @ref DMA_LinkList
  * @param dataLength number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_UART_ReadDMAAndCyclicallyStored(UART_Handle *uartHandle, unsigned char *saveData,
                                                    DMA_LinkList *tempNode, unsigned int dataLength)
{
    /* Param check */
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(tempNode != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_ASSERT_PARAM(saveData != NULL);
    UART_PARAM_CHECK_WITH_RET(dataLength > 0, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(uartHandle->rxMode == UART_MODE_DMA, BASE_STATUS_ERROR);
    UART_PARAM_CHECK_WITH_RET(IsDmaChannelNum(uartHandle->uartDmaRxChn) == true, BASE_STATUS_ERROR);
    UART_ASSERT_PARAM(uartHandle->dmaHandle != NULL);

    unsigned int channel = uartHandle->uartDmaRxChn;
    if (uartHandle->rxState == UART_STATE_READY) {
        uartHandle->rxState = UART_STATE_BUSY_RX;
        uartHandle->baseAddress->UART_IMSC.BIT.rxim = BASE_CFG_DISABLE;  /* Disable RX interrupt bit */
        uartHandle->rxbuff = saveData;
        uartHandle->rxBuffSize = dataLength;

        /* Init DAM Channel Params */
        DMA_ChannelParam dmaParams;
        dmaParams.direction   =  uartHandle->dmaHandle->DMA_Channels[channel].direction;
        dmaParams.srcAddrInc  =  uartHandle->dmaHandle->DMA_Channels[channel].srcAddrInc;
        dmaParams.destAddrInc =  uartHandle->dmaHandle->DMA_Channels[channel].destAddrInc;
        dmaParams.srcPeriph   =  uartHandle->dmaHandle->DMA_Channels[channel].srcPeriph;
        dmaParams.destPeriph  =  uartHandle->dmaHandle->DMA_Channels[channel].destPeriph;
        dmaParams.srcWidth    =  uartHandle->dmaHandle->DMA_Channels[channel].srcWidth;
        dmaParams.destWidth   =  uartHandle->dmaHandle->DMA_Channels[channel].destWidth;
        dmaParams.srcBurst    =  uartHandle->dmaHandle->DMA_Channels[channel].srcBurst;
        dmaParams.destBurst   =  uartHandle->dmaHandle->DMA_Channels[channel].destBurst;

        /* Initialize List Node */
        if (HAL_DMA_InitNewNode(tempNode, &dmaParams, (uintptr_t)(void *)&(uartHandle->baseAddress->UART_DR), \
            (uintptr_t)(void *)uartHandle->rxbuff, dataLength) != BASE_STATUS_OK) {
            uartHandle->rxState = UART_STATE_READY;
            return BASE_STATUS_ERROR;
        }
        if (HAL_DMA_ListAddNode(tempNode, tempNode) != BASE_STATUS_OK) {
            uartHandle->rxState = UART_STATE_READY;
            return BASE_STATUS_ERROR;
        }

        /* Can not masking overflow error, break error, check error, frame error interrupt */
        if (HAL_DMA_StartListTransfer(uartHandle->dmaHandle, tempNode, channel) != BASE_STATUS_OK) {
            uartHandle->rxState = UART_STATE_READY;
            return BASE_STATUS_ERROR;
        }
        uartHandle->baseAddress->UART_DMACR.BIT.rxdmae = BASE_CFG_ENABLE;        /* Enable RX_DMA bit */
    } else {
        /* Rx not ready */
        return BASE_STATUS_BUSY;
    }
    /* All done */
    return BASE_STATUS_OK;
}

/**
  * @brief Obtains offset address of DMA transfer address relative to specified memory (rxbuff).
  * @param uartHandle UART handle.
  * @retval offset address of DMA transfer address relative to specified memory (rxbuff).
  */
unsigned int HAL_UART_ReadDMAGetPos(UART_Handle *uartHandle)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(uartHandle->dmaHandle != NULL);
    UART_ASSERT_PARAM(uartHandle->rxbuff != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_PARAM_CHECK_WITH_RET(IsDmaChannelNum(uartHandle->uartDmaRxChn) == true, BASE_STATUS_ERROR);
    UART_ASSERT_PARAM(uartHandle->dmaHandle->DMA_Channels[uartHandle->uartDmaRxChn].channelAddr != NULL);
    unsigned int writePos = 0;
    /* Obtain the read destination address */
    unsigned int readAddress = uartHandle->dmaHandle->\
                              DMA_Channels[uartHandle->uartDmaRxChn].channelAddr->DMAC_Cn_DEST_ADDR.reg;
    if (readAddress > (uintptr_t)uartHandle->rxbuff) {
        writePos = readAddress - (uintptr_t)uartHandle->rxbuff; /* Number of characters currently transferred */
    } else {
        writePos = 0;
    }
    return writePos;
}