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
  * @file      protocol.c
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of port communication.
  */

#include "protocol.h"
#include "apt.h"
#include "typedefs.h"
#include "main.h"
#include "mcs_assert.h"
#include "cust_process.h"

/**
  * @brief Callback function for receiving data analysis and processing.
  * @param rxBuf Receive buffer.
  */
__weak void CUST_UartDataProcess(MTRCTRL_Handle *mtrCtrl, unsigned char *rxBuf)
{
    BASE_FUNC_UNUSED(mtrCtrl);
    BASE_FUNC_UNUSED(rxBuf);
}
/**
  * @brief User-defined protocol message sending function (weak function).
  * @param rxData Sending Messages..
  */
__weak void CUST_SetTxMsg(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *txData)
{
    BASE_FUNC_UNUSED(mtrCtrl);
    BASE_FUNC_UNUSED(*txData);
}

static void (*g_ptrDataProcess)(MTRCTRL_Handle *mtrCtrl, unsigned char *rxBuf) = CUST_UartDataProcess;

/**
  * @brief Frame checksum.
  * @param ptr Pointer to the data to be checked
  * @param num Number of bytes
  * @retval unsigned char Checksum
  */
static unsigned char CheckSum(unsigned char *ptr, unsigned char num)
{
    unsigned char sum = 0;
    unsigned char *p = ptr;
    /* Calculate the sum of received data. */
    for (int i = 0; i < num; i++) {
        sum += *p;
        p++;
    }
    return sum;
}

/**
  * @brief Transmitting Data Frames.
  * @param mtrCtrl The motor control handle.
  * @param txBuf Sending Messages.
  */
unsigned int CUST_TransmitData(MTRCTRL_Handle *mtrCtrl, unsigned char *txBuf)
{
    /* Verify Parameters */
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(txBuf != NULL);
    unsigned int dataLen = FRAME_ONE_DATA_LENTH * SEND_FRAME_DATA_NUM;
    unsigned int i = 0;
    CUSTDATATYPE_DEF txData = {0};
    CUST_SetTxMsg(mtrCtrl, &txData);
    txBuf[i++] = FRAME_START;
    /* Message function code */
    txBuf[i++] = FRAME_SENT;
    /* Message data */
    for (unsigned char  x = 0; x < SEND_FRAME_DATA_NUM; x++) {
        int floatIndex = 0;
        int byteOffset = i;
        txBuf[x * FRAME_ONE_DATA_LENTH + byteOffset++] = txData.data[x].typeCh[floatIndex++];
        txBuf[x * FRAME_ONE_DATA_LENTH + byteOffset++] = txData.data[x].typeCh[floatIndex++];
        txBuf[x * FRAME_ONE_DATA_LENTH + byteOffset++] = txData.data[x].typeCh[floatIndex++];
        txBuf[x * FRAME_ONE_DATA_LENTH + byteOffset++] = txData.data[x].typeCh[floatIndex++];
    }
    /* Message verification domain */
    txBuf[dataLen + i++] = CheckSum((unsigned char*)&txBuf[FRAME_CHECK_BEGIN], dataLen + 1);
    /* End of Message */
    txBuf[dataLen + i++] = FRAME_END;
    return dataLen + i;
}

/**
  * @brief Transmitting Data Frames.
  * @param txBuf Sending Cust Ack Code.
  * @param ackCode Ack Code.
  * @param varParams Host set parameter.
  */
void CUST_AckCode(unsigned char *txBuf, unsigned char ackCode, float varParams)
{
    /* Verify Parameters */
    MCS_ASSERT_PARAM(txBuf != NULL);
    CUSTDATATYPE_DEF txData = {0};
    int dataIndex = 0;
    unsigned int i = 0;
    unsigned int txLen = 0;
    unsigned int dataLen = FRAME_ONE_CHAR_LENTH + FRAME_ONE_DATA_LENTH;

    txData.data[0].typeF = varParams;
    txBuf[i++] = FRAME_START;
    /* Message function code */
    txBuf[i++] = FRAME_CUSTACK;
    /* Message ack code */
    txBuf[i++] = ackCode;
    /* Message data */
    txBuf[i++] = txData.data[0].typeCh[dataIndex++];
    txBuf[i++] = txData.data[0].typeCh[dataIndex++];
    txBuf[i++] = txData.data[0].typeCh[dataIndex++];
    txBuf[i++] = txData.data[0].typeCh[dataIndex++];

    /* Message verification domain */
    txBuf[FRAME_ONE_CHAR_LENTH + i++] = CheckSum((unsigned char*)&txBuf[FRAME_CHECK_BEGIN], dataLen + 1);
    /* End of Message */
    txBuf[FRAME_ONE_CHAR_LENTH + i++] = FRAME_END;
    txLen = FRAME_ONE_CHAR_LENTH + i++;
    HAL_UART_WriteIT(&g_uart0, txBuf, txLen);
}

/**
  * @brief Cust receive data process.
  * @param mtrCtrl The motor control handle.
  * @param rxBuf Receive buffer
  */
void CUST_DataReceProcss(MTRCTRL_Handle *mtrCtrl, unsigned char *rxBuf)
{
    /* Verify Parameters */
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(rxBuf != NULL);
    unsigned char g_uartTxBuf[10]  = {0};
    unsigned char ackCode = 0;
    /* Frame header check */
    if (rxBuf[0] != FRAME_START) {
        ackCode = 0X78;
        CUST_AckCode(g_uartTxBuf, ackCode, 0);
        return;
    }
    /* Frame trailer check */
    if (rxBuf[FRAME_LENTH - 1] != FRAME_END) {
        ackCode = 0X79;
        CUST_AckCode(g_uartTxBuf, ackCode, 0);
        return;
    }
    /* Checksum */
    if (CheckSum((unsigned char*)&rxBuf[FRAME_CHECK_BEGIN], FRAME_CHECK_NUM) != rxBuf[FRAME_CHECKSUM]) {
        ackCode = 0X7A;
        CUST_AckCode(g_uartTxBuf, ackCode, 0);
        return;
    } else {
        if (g_ptrDataProcess == NULL) {
            return;
        } else {
            g_ptrDataProcess(mtrCtrl, rxBuf);
        }
    }
}
