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
  * @file      mcs_curr_prot.c
  * @author    MCU Algorithm Team
  * @brief     This file contains current protecion api definition.
  */
 
/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_curr_prot.h"
#include "mcs_math.h"
#include "mcs_prot_user_config.h"
#include "mcs_assert.h"

/**
  * @brief Initilization over current protection function.
  * @param ocp Over current protection handle.
  * @param ts Ctrl period (s).
  * @retval None.
  */
void OCP_Init(OCP_Handle *ocp, float ts)
{
    MCS_ASSERT_PARAM(ocp != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    ocp->ts = ts;
    OCP_Clear(ocp);

    ocp->protCntLimit = PROT_CNT_LIMIT;
    ocp->recyCntLimit = RECY_CNT_LIMIT;
    /* Configuring four levels of current protection thresholds. */
    ocp->protValThr[PROT_VAL_THRESHOLD_0] = PROT_OVER_CURR_POW_DN1;
    ocp->protValThr[PROT_VAL_THRESHOLD_1] = PROT_OVER_CURR_POW_DN2;
    ocp->protValThr[PROT_VAL_THRESHOLD_2] = PROT_OVER_CURR_POW_DN3;
    ocp->protValThr[PROT_VAL_THRESHOLD_3] = PROT_OVER_CURR_POW_OFF;
    /* Configure the protection limit time. */
    ocp->protLimitTime[PROT_LIMIT_TIME_0] = PROT_OVER_CURR_LIMIT1_TIME_SEC;
    ocp->protLimitTime[PROT_LIMIT_TIME_1] = PROT_OVER_CURR_LIMIT2_TIME_SEC;
    ocp->protLimitTime[PROT_LIMIT_TIME_2] = PROT_OVER_CURR_LIMIT3_TIME_SEC;
    ocp->recyDelta = PROT_OVER_CURR_RECY_DELTA;
}

/**
  * @brief Over current protection detection.
  * @param ocp Over current protection handle.
  * @param motorErrStatus Motor error status.
  * @param idq DQ-axis feedback currents.
  * @retval None.
  */
void OCP_Det(OCP_Handle *ocp, MotorErrStatusReg *motorErrStatus, DqAxis idq)
{
    MCS_ASSERT_PARAM(ocp != NULL);
    MCS_ASSERT_PARAM(motorErrStatus != NULL);
    /* Calculate current amplitude. */
    ocp->currAmp = Sqrt(idq.d * idq.d + idq.q * idq.q);
  
    /* Check if value goes over threshold for continuous cycles. */
    if (ocp->currAmp < ocp->protValThr[PROT_VAL_THRESHOLD_0]) {
        ocp->protCnt = 0;
        return;
    }

    if (ocp->protCnt < ocp->protCntLimit) {
        ocp->protCnt++;
        return;
    }

    /* Check which protection level should be activated. */
    /* Once enter error status, error status can only be changed by recover api. */
    /* protect level: level 4. */
    if (ocp->currAmp >= ocp->protValThr[PROT_VAL_THRESHOLD_3] && ocp->protLevel < PROT_LEVEL_4) {
        ocp->protLevel = PROT_LEVEL_4;
        motorErrStatus->Bit.overCurrErr = 1;
        ocp->protCnt = 0;
        return;
    }

    /* Protect level: level 3. */
    if (ocp->currAmp >= ocp->protValThr[PROT_VAL_THRESHOLD_2] && ocp->protLevel < PROT_LEVEL_3) {
        ocp->protLevel = PROT_LEVEL_3;
        motorErrStatus->Bit.overCurrErr = 1;
        ocp->protCnt = 0;
        ocp->timer = 0.0f;
        return;
    }

    /* Protect level: level 2. */
    if (ocp->currAmp >= ocp->protValThr[PROT_VAL_THRESHOLD_1] && ocp->protLevel < PROT_LEVEL_2) {
        ocp->protLevel = PROT_LEVEL_2;
        motorErrStatus->Bit.overCurrErr = 1;
        ocp->protCnt = 0;
        ocp->timer = 0.0f;
        return;
    }

    /* Protect level: level 1. */
    if (ocp->currAmp >= ocp->protValThr[PROT_VAL_THRESHOLD_0] && ocp->protLevel < PROT_LEVEL_1) {
        ocp->protLevel = PROT_LEVEL_1;
        motorErrStatus->Bit.overCurrErr = 1;
        ocp->protCnt = 0;
        ocp->timer = 0.0f;
        return;
    }
}

/**
  * @brief Over current protection execution.
  * @param ocp Over current protection handle.
  * @param idqRef DQ-axis current references.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
void OCP_Exec(OCP_Handle *ocp, DqAxis *idqRef, APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(ocp != NULL);
    MCS_ASSERT_PARAM(idqRef != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);

    float id = idqRef->d;
    float iq = idqRef->q;
    float idqAmp = ocp->currAmp;
    /* According to protect level, take corresponding action. */
    switch (ocp->protLevel) {
      /* level 4: disable all PWM output. */
        case PROT_LEVEL_4:
            /* Disable three-phase pwm output. */
            ProtSpo_Exec(aptAddr);
            idqRef->d = 0.0f;
            idqRef->q = 0.0f;
            break;

      /* level 3: derate speed reference. */
        case PROT_LEVEL_3:
            ocp->timer += ocp->ts;
            if (ocp->timer > ocp->protLimitTime[PROT_LIMIT_TIME_2]) {
                idqRef->d = id / idqAmp * PROT_MOTOR_RATED_CURR;
                idqRef->q = iq / idqAmp * PROT_MOTOR_RATED_CURR;
            }
            break;

      /* level 2: derate speed reference. */
        case PROT_LEVEL_2:
            ocp->timer += ocp->ts;
            if (ocp->timer > ocp->protLimitTime[PROT_LIMIT_TIME_1]) {
                idqRef->d = id / idqAmp * PROT_MOTOR_RATED_CURR;
                idqRef->q = iq / idqAmp * PROT_MOTOR_RATED_CURR;
            }
            break;

      /* level 1: derate speed reference. */
        case PROT_LEVEL_1:
            ocp->timer += ocp->ts;
            if (ocp->timer > ocp->protLimitTime[PROT_LIMIT_TIME_0]) {
                idqRef->d = id / idqAmp * PROT_MOTOR_RATED_CURR;
                idqRef->q = iq / idqAmp * PROT_MOTOR_RATED_CURR;
            }
            break;

      /* level 0: take no protection action. */
        case PROT_LEVEL_0:
            break;

        default:
            break;
    }
}

/**
  * @brief Over current protection recovery.
  * @param ocp Over current protection handle.
  * @param motorErrStatus Motor error status.
  * @retval None.
  */
void OCP_Recy(OCP_Handle *ocp, MotorErrStatusReg *motorErrStatus)
{
    MCS_ASSERT_PARAM(ocp != NULL);
    /* If not under error state, just return without any operation. */
    if (!motorErrStatus->Bit.overCurrErr) {
        return;
    }

    /* Calculate current amplitude. */
    float currAmp = ocp->currAmp;

    /* According to protection level, take corresponding recovery action. */
    switch (ocp->protLevel) {
        /* level 4 */
        case PROT_LEVEL_4:
            if (currAmp < ocp->protValThr[PROT_VAL_THRESHOLD_3] - ocp->recyDelta) {
                ocp->recyCnt++;
                if (ocp->recyCnt > ocp->recyCntLimit) {
                    ocp->protLevel = PROT_LEVEL_3;
                    ocp->recyCnt = 0;
                }
            }
            break;

        /* level 3 */
        case PROT_LEVEL_3:
            if (currAmp < ocp->protValThr[PROT_VAL_THRESHOLD_2] - ocp->recyDelta) {
                ocp->recyCnt++;
                if (ocp->recyCnt > ocp->recyCntLimit) {
                    ocp->protLevel = PROT_LEVEL_2;
                    ocp->recyCnt = 0;
                }
            }
            break;

        /* level 2 */
        case PROT_LEVEL_2:
            if (currAmp < ocp->protValThr[PROT_VAL_THRESHOLD_1] - ocp->recyDelta) {
                ocp->recyCnt++;
                if (ocp->recyCnt > ocp->recyCntLimit) {
                    ocp->protLevel = PROT_LEVEL_1;
                    ocp->recyCnt = 0;
                }
            }
            break;
    
        /* level 1 */
        case PROT_LEVEL_1:
            if (currAmp < ocp->protValThr[PROT_VAL_THRESHOLD_0] - ocp->recyDelta) {
                ocp->recyCnt++;
                if (ocp->recyCnt > ocp->recyCntLimit) {
                    ocp->protLevel = PROT_LEVEL_0;
                    ocp->recyCnt = 0;
                }
            }
            break;

        /* level 0 */
        case PROT_LEVEL_0:
            motorErrStatus->Bit.overCurrErr = 0;
            break;
    
        default:
            break;
    }
}

/**
  * @brief Over current protection error status clear.
  * @param ocp Over current protection handle.
  * @retval None.
  */
void OCP_Clear(OCP_Handle *ocp)
{
    MCS_ASSERT_PARAM(ocp != NULL);
    /* Clear the history value. */
    ocp->protCnt = 0;
    ocp->protLevel = PROT_LEVEL_0;
    ocp->recyCnt = 0;
    ocp->timer = 0.0f;
}