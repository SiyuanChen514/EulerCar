/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      mcs_dc_volt_prot.h
  * @author    MCU Algorithm Team
  * @brief     This file contains dc-link voltage protection data struct and api declaration.
  */
 
/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_DC_VOLT_PROT_H
#define McuMagicTag_MCS_DC_VOLT_PROT_H

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_prot_cmm.h"
#include "apt_ip.h"

typedef struct {
    unsigned int protCnt;
    unsigned int recyCnt;
    unsigned int protCntLimit;
    unsigned int recyCntLimit;
    float protValThr[PROT_VAL_THRESHOLD_NUMS];  /* from low voltage level to high. */
    float protLimitTime[PROT_LIMIT_TIME_NUMS]; /* from level 3 to level 1. */
    float recyDelta;
    float timer;
    float ts;
    PROT_Level protLevel;
} OVP_Handle;

typedef struct {
    unsigned int protCnt;
    unsigned int recyCnt;
    unsigned int protCntLimit;
    unsigned int recyCntLimit;
    float protValThr[PROT_VAL_THRESHOLD_NUMS];  /* from high voltage level to low. */
    float protLimitTime[PROT_LIMIT_TIME_NUMS]; /* from level 3 to level 1. */
    float recyDelta;
    float timer;
    float ts;
    PROT_Level protLevel;
} LVP_Handle;

void OVP_Init(OVP_Handle *ovp, float ts);
void OVP_Det(OVP_Handle *ovp, MotorErrStatusReg *motorErrStatus, float udc);
void OVP_Exec(OVP_Handle *ovp, float *duty, APT_RegStruct **aptAddr);
void OVP_Recy(OVP_Handle *ovp, MotorErrStatusReg *motorErrStatus, float udc);
void OVP_Clear(OVP_Handle *ovp);

void LVP_Init(LVP_Handle *lvp, float ts);
void LVP_Det(LVP_Handle *lvp, MotorErrStatusReg *motorErrStatus, float udc);
void LVP_Exec(LVP_Handle *lvp, float *spdRef, APT_RegStruct **aptAddr);
void LVP_Recy(LVP_Handle *lvp, MotorErrStatusReg *motorErrStatus, float udc);
void LVP_Clear(LVP_Handle *lvp);

#endif
