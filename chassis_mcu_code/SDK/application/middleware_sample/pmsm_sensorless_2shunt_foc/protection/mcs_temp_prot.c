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
  * @file      mcs_temp_prot.c
  * @author    MCU Algorithm Team
  * @brief     This file contains over temperature protection api definition.
  */

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_temp_prot.h"
#include "mcs_prot_user_config.h"
#include "mcs_assert.h"

/**
  * @brief Initilization over temperation protection function.
  * @param otp Over temperature protection handle.
  * @param ts Ctrl period.
  * @retval None.
  */
void OTP_Init(OTP_Handle *otp, float ts)
{
    MCS_ASSERT_PARAM(otp != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    otp->ts = ts;
    otp->protCntLimit = PROT_CNT_LIMIT;
    otp->recyCntLimit = RECY_CNT_LIMIT;
    /* Configuring the temperature protection threshold. */
    otp->protValThr[PROT_VAL_THRESHOLD_0] = PROT_OVER_IPM_TEMP_POW_DN1;
    otp->protValThr[PROT_VAL_THRESHOLD_1] = PROT_OVER_IPM_TEMP_POW_DN2;
    otp->protValThr[PROT_VAL_THRESHOLD_2] = PROT_OVER_IPM_TEMP_POW_DN3;
    otp->protValThr[PROT_VAL_THRESHOLD_3] = PROT_OVER_IPM_TEMP_POW_OFF;
    /* Configuring the protection limiting time. */
    otp->protLimitTime[PROT_LIMIT_TIME_0] = PROT_OVER_TEMP_LIMIT1_TIME_SEC;
    otp->protLimitTime[PROT_LIMIT_TIME_1] = PROT_OVER_TEMP_LIMIT2_TIME_SEC;
    otp->protLimitTime[PROT_LIMIT_TIME_2] = PROT_OVER_TEMP_LIMIT3_TIME_SEC;
    otp->recyDelta = PROT_OVER_IPM_TEMP_RECY_DELTA;
    OTP_Clear(otp);
}

/**
  * @brief Over temperatre protection detection.
  * @param otp Over temperature protection handle.
  * @param tempErrBit Temperature error status bit.
  * @param idq DQ-axis feedback currents.
  * @retval None.
  */
void OTP_Det(OTP_Handle *otp, MotorErrStatusReg *motorErrStatus, PROT_ErrBit protBit, float temp)
{
    MCS_ASSERT_PARAM(otp != NULL);
    MCS_ASSERT_PARAM(motorErrStatus != NULL);
    MCS_ASSERT_PARAM(temp > 0.0f);
    /* Check if value goes over threshold for continuous cycles. */
    if (temp < otp->protValThr[PROT_VAL_THRESHOLD_0]) {
        otp->protCnt = 0;
        ClearBit(&motorErrStatus->all, protBit);
        return;
    }

    if (otp->protCnt < otp->protCntLimit) {
        otp->protCnt++;
        return;
    }

    /* Check which protection level should be activated. */
    /* Once enter error status, error status can only be changed by recover api. */
    /* protect level: level 4. */
    if (temp >= otp->protValThr[PROT_VAL_THRESHOLD_3] && otp->protLevel < PROT_LEVEL_4) {
        otp->protLevel = PROT_LEVEL_4;
        SetBit(&motorErrStatus->all, protBit);
        otp->protCnt = 0;
        return;
    }

    /* Protect level: level 3. */
    if (temp >= otp->protValThr[PROT_VAL_THRESHOLD_2] && otp->protLevel < PROT_LEVEL_3) {
        otp->protLevel = PROT_LEVEL_3;
        SetBit(&motorErrStatus->all, protBit);
        otp->protCnt = 0;
        otp->timer = 0.0f;
        return;
    }

    /* Protect level: level 2. */
    if (temp >= otp->protValThr[PROT_VAL_THRESHOLD_1] && otp->protLevel < PROT_LEVEL_2) {
        otp->protLevel = PROT_LEVEL_2;
        SetBit(&motorErrStatus->all, protBit);
        otp->protCnt = 0;
        otp->timer = 0.0f;
        return;
    }

    /* Protect level: level 1. */
    if (temp >= otp->protValThr[PROT_VAL_THRESHOLD_0] && otp->protLevel < PROT_LEVEL_1) {
        otp->protLevel = PROT_LEVEL_1;
        SetBit(&motorErrStatus->all, protBit);
        otp->protCnt = 0;
        otp->timer = 0.0f;
        return;
    }
}

/**
  * @brief Over temperature protection execution.
  * @param otp Over temperature protection handle.
  * @param spdRef Speed reference (Hz).
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
void OTP_Exec(OTP_Handle *otp, float *spdRef, APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(otp != NULL);
    MCS_ASSERT_PARAM(spdRef != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);
    /* According to protect level, take corresponding action. */
    switch (otp->protLevel) {
      /* level 4: disable all PWM output. */
        case PROT_LEVEL_4:
            /* Disable three-phase pwm output. */
            ProtSpo_Exec(aptAddr);
            *spdRef *= 0.0f;
            break;

        /* level 3: derate speed reference. */
        case PROT_LEVEL_3:
            *spdRef *= PROT_POW_DN2_PCT;
            otp->timer += otp->ts;
            if (otp->timer > otp->protLimitTime[PROT_LIMIT_TIME_2]) {
                *spdRef *= PROT_POW_DN3_PCT;
            }
            break;

        /* level 2: derate speed reference. */
        case PROT_LEVEL_2:
            /* Reducte motor speed to level 2 */
            *spdRef *= PROT_POW_DN1_PCT;
            otp->timer += otp->ts;
            if (otp->timer > otp->protLimitTime[PROT_LIMIT_TIME_1]) {
                *spdRef *= PROT_POW_DN2_PCT;
            }
            break;

        /* level 0: take no protection action. */
        case PROT_LEVEL_0:
            break;

        case PROT_LEVEL_1:
        /* Reducte motor speed to level 0 */
            otp->timer += otp->ts;
            if (otp->timer > otp->protLimitTime[PROT_LIMIT_TIME_0]) {
                /* level 1: derate speed reference. */
                *spdRef *= PROT_POW_DN1_PCT;
            }
            break;

        default:
            break;
    }
    return;
}

/**
  * @brief Over temperature protection recovery.
  * @param otp Over temperature protection handle.
  * @param tempErrBit Temperature error status bit.
  * @param temp Temperature (celsius).
  * @retval None.
  */
void OTP_Recy(OTP_Handle *otp,  MotorErrStatusReg *motorErrStatus, PROT_ErrBit protBit, float temp)
{
    MCS_ASSERT_PARAM(otp != NULL);
    MCS_ASSERT_PARAM(motorErrStatus != NULL);
    MCS_ASSERT_PARAM(temp > 0.0f);
    /* If not under error state, just return without any operation. */
    if (otp->protLevel == PROT_LEVEL_0) {
        motorErrStatus->all &=  (~(1 >> protBit));
        return;
    }

    /* According to protection level, take corresponding recovery action. */
    switch (otp->protLevel) {
        /* level 4 */
        case PROT_LEVEL_4:
            /* If the temperature is less than threshold 3, level-3 protection is restored. */
            if (temp < otp->protValThr[PROT_VAL_THRESHOLD_3] - otp->recyDelta) {
                otp->recyCnt++;
                if (otp->recyCnt > otp->recyCntLimit) {
                    otp->protLevel = PROT_LEVEL_3;
                    otp->recyCnt = 0;
                }
            }
            break;
  
        /* level 3 */
        case PROT_LEVEL_3:
            /* If the temperature is less than threshold 2, level-2 protection is restored. */
            if (temp < otp->protValThr[PROT_VAL_THRESHOLD_2] - otp->recyDelta) {
                otp->recyCnt++;
                if (otp->recyCnt > otp->recyCntLimit) {
                    otp->protLevel = PROT_LEVEL_2;
                    otp->recyCnt = 0;
                }
            }
            break;
    
          /* level 2 */
        case PROT_LEVEL_2:
            /* If the temperature is less than threshold 1, level-1 protection is restored. */
            if (temp < otp->protValThr[PROT_VAL_THRESHOLD_1] - otp->recyDelta) {
                otp->recyCnt++;
                if (otp->recyCnt > otp->recyCntLimit) {
                    otp->protLevel = PROT_LEVEL_1;
                    otp->recyCnt = 0;
                }
            }
            break;
    
          /* level 1 */
        case PROT_LEVEL_1:
            /* If the temperature is less than threshold 0, level-0 protection is restored. */
            if (temp < otp->protValThr[PROT_VAL_THRESHOLD_0] - otp->recyDelta) {
                otp->recyCnt++;
                if (otp->recyCnt > otp->recyCntLimit) {
                    otp->protLevel = PROT_LEVEL_0;
                    otp->recyCnt = 0;
                }
            }
            break;

        default:
            break;
    }
        return;
}

/**
  * @brief Over temperature protection error status clear.
  * @param otp Over temperature protection handle.
  * @retval None.
  */
void OTP_Clear(OTP_Handle *otp)
{
    MCS_ASSERT_PARAM(otp != NULL);
    /* Clear the history value. */
    otp->protCnt = 0;
    otp->protLevel = PROT_LEVEL_0;
    otp->recyCnt = 0;
    otp->timer = 0.0f;
}