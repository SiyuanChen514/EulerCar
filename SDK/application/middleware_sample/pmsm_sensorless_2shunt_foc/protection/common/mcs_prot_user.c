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
  * @file      mcs_prot_user.c
  * @author    MCU Algorithm Team
  * @brief     This file contains user protection data struct, inquiry api and initialization api definition.
  */

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_prot_user.h"
#include "mcs_prot_user_config.h"
#include "mcs_assert.h"

/**
  * @brief Get motor over current error status.
  * @param motorErrStatus Motor error status.
  * @retval Error status.
  */
bool IsMotorOverCurrErr(MotorErrStatusReg motorErrStatus)
{
    if (motorErrStatus.Bit.overCurrErr) {
        return true;
    } else {
        return false;
    }
}

/**
  * @brief Get motor over dc-link voltage error status.
  * @param motorErrStatus Motor error status.
  * @retval Error status.
  */
bool IsDcLinkOverVoltErr(MotorErrStatusReg motorErrStatus)
{
    if (motorErrStatus.Bit.overVoltErr) {
        return true;
    } else {
        return false;
    }
}

/**
  * @brief Get motor lower dc-link voltage error status.
  * @param motorErrStatus Motor error status.
  * @retval Error status.
  */
bool IsDcLinkLowerVoltErr(MotorErrStatusReg motorErrStatus)
{
    if (motorErrStatus.Bit.lowerVoltErr) {
        return true;
    } else {
        return false;
    }
}

/**
  * @brief Get motor over Ipm temperature error status.
  * @param motorErrStatus Motor error status.
  * @retval Error status.
  */
bool IsIpmOverTempErr(MotorErrStatusReg motorErrStatus)
{
    if (motorErrStatus.Bit.overIpmTempErr) {
        return true;
    } else {
        return false;
    }
}

/**
  * @brief Get motor over Motor temperature error status.
  * @param motorErrStatus Motor error status.
  * @retval Error status.
  */
bool IsMotorOverTempErr(MotorErrStatusReg motorErrStatus)
{
    if (motorErrStatus.Bit.overMotorTempErr) {
        return true;
    } else {
        return false;
    }
}

/**
  * @brief Get motor stalling error status.
  * @param motorErrStatus Motor error status.
  * @retval Error status.
  */
bool IsMotorStallingErr(MotorErrStatusReg motorErrStatus)
{
    if (motorErrStatus.Bit.motorStalling) {
        return true;
    } else {
        return false;
    }
}

/**
  * @brief Clear the motor error status.
  * @param motorProt Motor protection handle.
  * @retval Error status.
  */
void ClearMotorErrStatus(MotorProtStatus_Handle *motorProt)
{
    MCS_ASSERT_PARAM(motorProt != NULL);
    /* Clear the motor error status. */
    motorProt->motorErrStatus.all = 0x00;
    OCP_Clear(&motorProt->ocp);
    OVP_Clear(&motorProt->ovp);
    LVP_Clear(&motorProt->lvp);
    OTP_Clear(&motorProt->otp);
}

/**
  * @brief Motor protection function initialization.
  * @param motorProt Motor protection handle.
  * @retval Error status.
  */
void MotorProt_Init(MotorProtStatus_Handle *motorProt)
{
    MCS_ASSERT_PARAM(motorProt != NULL);
    motorProt->motorErrStatus.all = 0x00;
}