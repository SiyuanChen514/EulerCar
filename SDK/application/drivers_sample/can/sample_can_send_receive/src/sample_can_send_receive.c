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
  * @file    sample_can_send_receive.c
  * @author  MCU Driver Team
  * @brief   can sample module.
  * @details This file provides sample code for users to help use
  *          the filtering function of the CAN.
  */
#include "sample_can_send_receive.h"
#include "main.h"
#include "debug.h"

/**
  * @brief User-defined read completion callback function.
  * @param UART_Handle UART handle.
  * @retval None.
  */
void CAN_ReadFinish(void *handle)
{
    CAN_Handle *canHandle = (CAN_Handle *)handle;
    DBG_PRINTF("CAN Read Finish\r\n");
    DBG_PRINTF("data[0]: %d \r\n", canHandle->rxFrame->frame[0]);  /* frame[0] data */
    DBG_PRINTF("data[1]: %d \r\n", canHandle->rxFrame->frame[1]);  /* frame[1] data */
    DBG_PRINTF("data[2]: %d \r\n", canHandle->rxFrame->frame[2]);  /* frame[2] data */
    DBG_PRINTF("data[3]: %d \r\n", canHandle->rxFrame->frame[3]);  /* frame[3] data */
    DBG_PRINTF("data[4]: %d \r\n", canHandle->rxFrame->frame[4]);  /* frame[4] data */
    DBG_PRINTF("data[5]: %d \r\n", canHandle->rxFrame->frame[5]);  /* frame[5] data */
    DBG_PRINTF("data[6]: %d \r\n", canHandle->rxFrame->frame[6]);  /* frame[6] data */
    DBG_PRINTF("data[7]: %d \r\n", canHandle->rxFrame->frame[7]);  /* frame[7] data */
    DBG_PRINTF("ID: 0x%x \r\n", canHandle->rxFrame->CANId);
    DBG_PRINTF("len: %d \r\n", canHandle->rxFrame->dataLength);
    DBG_PRINTF("type: %d \r\n",  canHandle->rxFrame->type);
    BASE_FUNC_UNUSED(canHandle);
    return;
}

/**
  * @brief User-defined write completion callback function.
  * @param CAN_Handle CAN handle.
  * @retval None.
  */
void CAN_WriteFinish(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("CAN Write Finish\r\n");
    return;
}
/**
  * Sample Note:
  * baund rate : 100k Bit/s
  * To-be-received data frame : Extended Data Frame, ID = 0x1314
  *
  * There are two filtering methods:
  * 1. Receive only the specified ID:
  *         rxFilter.receiveType = CAN_FilterFrame_EXT_DATA;
  *         rxFilter.filterID = 0x1314;
  *         rxFilter.filterMask = 0xFFFFFFFF;
  *
  * 2. Use mask to receive:
  *         rxFilter.receiveType = CAN_FilterFrame_EXT_DATA;
  *         rxFilter.filterID = 0x131_;
  *         rxFilter.filterMask = 0xFFFFFFF0;
  *
  *         rxFilter.receiveType = CAN_FilterFrame_EXT_DATA;
  *         rxFilter.filterID = 0x13_4;
  *         rxFilter.filterMask = 0xFFFFFF0F;
  *
  *         rxFilter.receiveType = CAN_FilterFrame_EXT_DATA;
  *         rxFilter.filterID = 0x1_14;
  *         rxFilter.filterMask = 0xFFFFF0FF;
  *
  *         rxFilter.receiveType = CAN_FilterFrame_EXT_DATA;
  *         rxFilter.filterID = 0x_314;
  *         rxFilter.filterMask = 0xFFFF0FFF;
  *
  * The value of the ID mask bit, which is not used for filtering.
  *
  * Do not filter by ID:
  *         rxFilter.filterMask = 0x00000000;
  *
  * Both standard frames and extended frames can be received:
  *         rxFilter.receiveType = CAN_FilterFrame_STD_EXT_DATA;
  */
/**
  * @brief CAN sample code for configuring the filtering function and can send and receive packets.
  * @param None.
  * @retval None.
  */
CANFrame g_sendFrame;
CANFrame g_receiveFrame;
int CAN_ReceiveFilter(void)
{
    SystemInit();
    DBG_PRINTF("CAN Init \r\n");
    CAN_FilterConfigure rxFilter;
    g_can.rxFrame = &g_receiveFrame;   /* Address for storing received frame data */
    DBG_PRINTF("CAN interrupt register \r\n");
    g_sendFrame.type = CAN_TYPEFRAME_EXT_DATA; /* Transmit extended data frame */
    g_sendFrame.CANId = 0x1314;       /* 0x1314 is ID of transmitted data frames */
    g_sendFrame.dataLength = 1;       /* 1 is length of the sent frame */
    g_sendFrame.frame[0] = '0';
    HAL_CAN_Write(&g_can, &g_sendFrame);
    rxFilter.receiveType = CAN_FILTERFRAME_EXT_DATA;
    rxFilter.filterID = 0x1014;        /* 0x1014 and 0xFFFFF0FF constitute filtering rules */
    rxFilter.filterMask = 0xFFFFF0FF;  /* 0xFFFFF0FF is filter ID mask */
    HAL_CAN_ReadIT(&g_can, &g_receiveFrame, &rxFilter);
    return 0;
}