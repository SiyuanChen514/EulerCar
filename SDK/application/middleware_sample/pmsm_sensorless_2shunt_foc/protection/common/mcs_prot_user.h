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
  * @file      mcs_prot_user.h
  * @author    MCU Algorithm Team
  * @brief     This file contains user protection data struct, inquiry api and initialization api declaration.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_PROT_USER_H
#define McuMagicTag_MCS_PROT_USER_H

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_prot_cmm.h"
#include "mcs_curr_prot.h"
#include "mcs_dc_volt_prot.h"
#include "mcs_temp_prot.h"
#include "mcs_motor_stalling.h"
#include "typedefs.h"

typedef struct {
    MotorErrStatusReg   motorErrStatus;   /**< Motor error status. */
    OCP_Handle    ocp;                    /**< Over current protection. */
    OVP_Handle    ovp;                    /**< Over dc-link voltage protection. */
    LVP_Handle    lvp;                    /**< Lower dc-link voltage protection. */
    OTP_Handle    otp;                    /**< Over IPM temperature protection. */
    STP_Handle    stall;                  /**< Motor stalling protection. */
} MotorProtStatus_Handle;

void MotorProt_Init(MotorProtStatus_Handle *motorProt);

/**< Inquiry motor error status */
bool IsMotorOverCurrErr(MotorErrStatusReg motorErrStatus);
bool IsDcLinkOverVoltErr(MotorErrStatusReg motorErrStatus);
bool IsDcLinkLowerVoltErr(MotorErrStatusReg motorErrStatus);
bool IsIpmOverTempErr(MotorErrStatusReg motorErrStatus);
bool IsMotorOverTempErr(MotorErrStatusReg motorErrStatus);
bool IsMotorStallingErr(MotorErrStatusReg motorErrStatus);
void ClearMotorErrStatus(MotorProtStatus_Handle *motorProt);
#endif