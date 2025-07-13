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
  * @file      uart_module.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of Serial port communication.
  */
#ifndef McsMagicTag_UART_MODULE_H
#define McsMagicTag_UART_MODULE_H

#include "protocol.h"
#include "mcs_ctlmode_config.h"

typedef struct {
    unsigned int buffLen;
    unsigned int timeOutCnt;
    unsigned char frameFlag;
    unsigned int rxLen;
    unsigned char rxFlag;
    unsigned char txFlag;
    unsigned char rxData;
    unsigned int upDataCnt;
    unsigned int upDataDelayCnt;
    unsigned char uartItTxFlag;
} FRAME_Handle;


void UartRecvInit(void);
void UartModuleProcess_Rx(MTRCTRL_Handle *mtrCtrl);
void UartModuleProcess_Tx(MTRCTRL_Handle *mtrCtrl);

#endif