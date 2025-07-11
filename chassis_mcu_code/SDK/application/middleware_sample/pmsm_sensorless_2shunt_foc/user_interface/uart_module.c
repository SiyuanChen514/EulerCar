/**
    * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
    * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
    * following conditions are met:
    * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
    * following disclaimer.
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
    * @file      uart_module.c
    * @author    MCU Algorithm Team
    * @brief     This file provides functions declaration of Serial port communication.
    */
#include "uart_module.h"
#include "debug.h"
#include "main.h"
#include "baseinc.h"

/* Buffer size */
#define UI_TX_BUF_LEN    (96)
#define UI_RX_BUF_LEN    (96)

/* Receiving Timeout Interval */
#define UART_TIME_OUT_MS (100)

/* Start sending data to host delay after uart connect success */
#define UART_UPDATA_DELAY_TIME_MS (50)

/* Uart baudrate */
#define UART0BAUDRATE (1843200)

/* Data buffer */
unsigned char g_uartRxBuf[UI_RX_BUF_LEN] = {0};
unsigned char g_uartTxBuf[UI_TX_BUF_LEN] = {0};
static unsigned int getdeltaSystickCnt = 0;
static FRAME_Handle g_uartFrame;
/**
    * @brief Receive Data Clear.
    * @param uartFrame  Receice Data.
    */
static void FrameRecvClear(FRAME_Handle *uartFrame)
{
    /* Clear buffer lenth. */
    uartFrame->buffLen = 0;
    uartFrame->timeOutCnt = 0;
    uartFrame->frameFlag = 0;
    /* Clear received data lenth. */
    uartFrame->rxLen = 0;
    /* Clear received flag. */
    uartFrame->rxFlag = 0;
    uartFrame->upDataCnt = 0;
}

/**
  * @brief Set Dma status.
  * @param mtrCtrl The motor control handle.
  */
static void SetUartDmaStatus(MTRCTRL_Handle *mtrCtrl)
{
    /* Delay 50ms start uart Tx DMA . */
    if (mtrCtrl->uartConnectFlag == CONNECTING && g_uartFrame.upDataDelayCnt++ > UART_UPDATA_DELAY_TIME_MS) {
        g_uartFrame.txFlag = 1;     /* Start send data flag. */
        mtrCtrl->uartConnectFlag = CONNECTED;
        g_uartFrame.upDataDelayCnt = 0;
    }
    if (mtrCtrl->uartConnectFlag == DISCONNECT) {
        g_uartFrame.txFlag = 0;     /* Stop send data flag. */
        mtrCtrl->uartTimeStamp = 0;
    }
}

/**
    * @brief Set uart baudRate.
    * @param baudrate  Uart baudRate.
    */
static void SetUartBaudRate(unsigned int baudrate)
{
    /* Re_Write uart0 baudrate. */
    g_uart0.baudRate = baudrate;
    HAL_UART_Init(&g_uart0);
}

/**
    * @brief Uart Dma interupt callback func.
    * @param null.
    */
void UART0_TXDMACallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    unsigned int getCurSystickCnt = 0;
    static unsigned int getlastSystickCnt = 0;
    /* USER CODE BEGIN UART0_WRITE_DMA_FINISH */
    g_uartFrame.txFlag = 1;
    getCurSystickCnt =  DCL_SYSTICK_GetTick();
    if (getlastSystickCnt != 0) {
        /* Calculate unit frame data send time */
        getdeltaSystickCnt = getCurSystickCnt - getlastSystickCnt;
    }
    getlastSystickCnt = getCurSystickCnt;
    /* USER CODE END UART0_WRITE_DMA_FINISH */
}

/**
  * @brief Uart0  interruput Write CallBack Function.
  * @param handle  Uart handle.
  */
void UART0WriteInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0_WRITE_IT_FINISH */
    g_uartFrame.uartItTxFlag = 1;
    g_uartFrame.txFlag = 1;
    /* USER CODE END UART0_WRITE_IT_FINISH */
}

/**
  * @brief Uart0 Interruput Read CallBack Function.
  * @param handle  Uart handle.
  */
void UART0ReadInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0_READ_IT_FINISH */
    if (g_uartFrame.rxLen >= UI_RX_BUF_LEN - 1) {
        g_uartFrame.rxLen = 0;
    }
    HAL_UART_ReadIT(handle, &g_uartFrame.rxData, 1);
    g_uartRxBuf[g_uartFrame.rxLen] = g_uartFrame.rxData;
    g_uartFrame.rxLen++;
    g_uartFrame.rxFlag = 1;
    g_uartFrame.uartItTxFlag = 0;
    return;
    /* USER CODE END UART0_READ_IT_FINISH */
}

/**
  * @brief Uart Read Data Init Function.
  * @param void.
  */
void UartRecvInit(void)
{
    /* Uart reception initialization */
    FrameRecvClear(&g_uartFrame);
    SetUartBaudRate(UART0BAUDRATE);
    HAL_UART_ReadIT(&g_uart0, &g_uartFrame.rxData, 1);
}

/**
  * @brief Uart Read Data Process Function.
  * @param mtrCtrl The motor control handle.
  */
void UartModuleProcess_Rx(MTRCTRL_Handle *mtrCtrl)
{
    /* Verify Parameters */
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    SetUartDmaStatus(mtrCtrl);
    if (g_uartFrame.rxFlag == 1) {  /* Receive data flag. */
        if (g_uartFrame.timeOutCnt++ > UART_TIME_OUT_MS) {
            /* Received data from the host. */
            g_uartFrame.frameFlag = 1;
            g_uartFrame.rxFlag = 0;
            g_uartFrame.timeOutCnt = 0;
            /* Execute data process. */
            CUST_DataReceProcss(mtrCtrl, g_uartRxBuf);
            g_uartFrame.rxLen = 0;
        }
    }
    g_uartFrame.frameFlag = 0;
}

/**
    * @brief Uart Write Data Process Function.
    * @param mtrCtrl The motor control handle.
    */
void UartModuleProcess_Tx(MTRCTRL_Handle *mtrCtrl)
{
    /* Verify Parameters */
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    if (g_uartFrame.txFlag == 1) {   /* Send data flag. */
            mtrCtrl->uartTimeStamp = (float)getdeltaSystickCnt;  /* Unit data time stamp */
            g_uartFrame.upDataCnt = 0;
            g_uartFrame.txFlag = 0;
            /* Send data to host. */
            unsigned int txLen = CUST_TransmitData(mtrCtrl, g_uartTxBuf);
            /* If txIT mode send data finish, convert to DMA mode */
            if (g_uartFrame.uartItTxFlag == 1) {
                    HAL_UART_WriteDMA(&g_uart0, g_uartTxBuf, txLen);
            }
    }
}
