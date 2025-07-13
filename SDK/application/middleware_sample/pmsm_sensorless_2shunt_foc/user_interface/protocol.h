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
  * @file      protocol.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of port communication.
  */
#ifndef McsMagicTag_PORTOCOL_H
#define McsMagicTag_PORTOCOL_H

#include "uart.h"
#include "mcs_carrier.h"

#define RS485_SEND_ENABLE   GPIO6->GPIO_DATA[GPIO_PIN_7].reg = GPIO_PIN_7
#define RS485_SEND_DISABLE   GPIO6->GPIO_DATA[GPIO_PIN_7].reg = BASE_CFG_UNSET

#define RX_BUF_LEN                          (16)
#define SEND_FRAME_DATA_NUM                 (CUSTDATA_MAX)
/* Service Uart0  Communication Deal */
#define  FRAME_ONE_DATA_LENTH               4
#define  FRAME_ONE_CHAR_LENTH               1
#define  FRAME_RECV_DATA_LENTH              4
#define  FRAME_LENTH                        20     /* Data length */
#define  FRAME_SENT                         0X8F
#define  FRAME_CUSTACK                      0X8A
#define  FRAME_START                        0x0F   /* Start frame */
#define  FRAME_END                          '/'    /* StOP frame */
#define  FRAME_CHECK_BEGIN                  1     /* Check frame */
#define  FRAME_CHECKSUM                     18     /* Check sum   */
#define  FRAME_CHECK_NUM                    17
#define  CMDCODE_IDLE_FRAME                 0x55  /* Fill frame   */
#define  CMDCODE_GET_MOTOR_PARAMS           0x01
#define  CMDCODE_SEND_MOTOR_PARAMS          0x02
#define  CMDCODE_SET_MOTOR_CTLMODE          0x03
#define  CMDCODE_SET_OBSERVER_TYPE          0x04
#define  CMDCODE_SET_STARTUP_MODE           0x05
#define  CMDCODE_SET_PID_PARAMS             0x06
#define  CMDCODE_SET_STARTUP_PARAMS         0x07
#define  CMDCODE_SET_OBSERVER_PARAMS        0x08
#define  CMDCODE_SET_MOTOR_TARGETSPD        0x09
#define  CMDCODE_SET_MOTOR_PARAMS           0x0A
#define  CMDCODE_MOTOR_START                0x0B
#define  CMDCODE_MOTOR_STOP                 0x0C
#define  CMDCODE_MOTORSTATE_RESET           0x0D
#define  CMDCODE_SEND_FIRMVERSION           0x0E
#define  CMDCODE_SET_ADJUSTSPD_MODE         0x11
#define  CMDCODE_UART_HANDSHAKE             0x12
#define  CMDCODE_UART_HEARTDETECT           0x13

typedef union {
    char typeCh[4];
    float typeF;
    int typeI;
} UNIONDATATYPE_DEF;

typedef enum {
    OFFLINE_RES = 0,
    OFFLINE_LD,
    OFFLINE_LQ,
    OFFLINE_PSIF,
    OFFLINE_JS,
    OFFLINE_NP,
    OFFLINE_B,
    OFFLINE_KPD,
    OFFLINE_KID,
    OFFLINE_KPQ,
    OFFLINE_KIQ,
    OFFLINE_KPS,
    OFFLINE_KIS,
    OFFLINE_SPEED,
    OFLINE_MAX
} OFFLINE_IDEN_TYPE;

typedef enum {
    CURRDQ_Q = 0,
    CURRDQ_D,
    CURRREFDQ_Q,
    CURRREFDQ_D,
    CURRSPD,
    SPDCMDHZ,
    UDC,
    POWERBOARDTEMP,
    CUST_ERR_CODE,
    CURRUVW_U,
    CURRUVW_V,
    CURRUVW_W,
    PWMDUTYUVW_U,
    PWMDUTYUVW_V,
    PWMDUTYUVW_W,
    AXISANGLE,
    VDQ_Q,
    VDQ_D,
    SPDREFHZ,
    SENDTIMESTAMP,
    CUSTDATA_MAX
} SENDTOHOSTPARAMS;

typedef struct {
    volatile unsigned char code;
    volatile UNIONDATATYPE_DEF data[SEND_FRAME_DATA_NUM];
} CUSTDATATYPE_DEF;


void CUST_DataReceProcss(MTRCTRL_Handle *mtrCtrl, unsigned char *rxBuf);
unsigned int CUST_TransmitData(MTRCTRL_Handle *mtrCtrl, unsigned char *txBuf);
void CUST_AckCode(unsigned char *txBuf, unsigned char ackCode, float varParams);
#endif /* McsMagicTag_PORTOCOL_H */
