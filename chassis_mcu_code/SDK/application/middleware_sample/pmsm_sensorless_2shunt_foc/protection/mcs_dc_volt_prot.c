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
  * @file      mcs_dc_volt_prot.c
  * @author    MCU Algorithm Team
  * @brief     This file contains dc-link voltage protection api declaration.
  */

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_dc_volt_prot.h"
#include "mcs_prot_user_config.h"
#include "mcs_assert.h"

/**
  * @brief Initilization over dc-link voltage protection function.
  * @param ovp Over dc-link voltage protection handle.
  * @param ts Ctrl period (s).
  * @retval None.
  */
void OVP_Init(OVP_Handle *ovp, float ts)
{
    MCS_ASSERT_PARAM(ovp != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    ovp->ts = ts;
    OVP_Clear(ovp);
    ovp->protCntLimit = PROT_CNT_LIMIT;
    ovp->recyCntLimit = OVER_VOLT_RECY_CNT_LIMIT;
    /* Configuring four levels of overvoltage protection thresholds. */
    ovp->protValThr[PROT_VAL_THRESHOLD_0] = PROT_OVER_VOLT_BRK_ON1;
    ovp->protValThr[PROT_VAL_THRESHOLD_1] = PROT_OVER_VOLT_BRK_ON2;
    ovp->protValThr[PROT_VAL_THRESHOLD_2] = PROT_OVER_VOLT_BRK_ON3;
    ovp->protValThr[PROT_VAL_THRESHOLD_3] = PROT_OVER_VOLT_BRK_ALL;
    /* Configure the protection limit time. */
    ovp->protLimitTime[PROT_LIMIT_TIME_0] = PROT_OVER_VOLT_LIMIT1_TIME_SEC;
    ovp->protLimitTime[PROT_LIMIT_TIME_1] = PROT_OVER_VOLT_LIMIT2_TIME_SEC;
    ovp->protLimitTime[PROT_LIMIT_TIME_2] = PROT_OVER_VOLT_LIMIT3_TIME_SEC;
    ovp->recyDelta = PROT_OVER_VOLT_RECY_DELTA;
}

/**
  * @brief Initilization lower dc-link voltage protection function.
  * @param lvp Lower dc-link voltage protection handle.
  * @param ts Ctrl period (s).
  * @retval None.
  */
void LVP_Init(LVP_Handle *lvp, float ts)
{
    MCS_ASSERT_PARAM(lvp != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);

    lvp->ts = ts;
    LVP_Clear(lvp);

    lvp->protCntLimit = PROT_CNT_LIMIT;
    lvp->recyCntLimit = LOWER_VOLT_RECY_CNT_LIMIT;
    /* Configuring four levels of lower voltage protection thresholds. */
    lvp->protValThr[PROT_VAL_THRESHOLD_0] = PROT_LOWER_VOLT_POW_DN1;
    lvp->protValThr[PROT_VAL_THRESHOLD_1] = PROT_LOWER_VOLT_POW_DN2;
    lvp->protValThr[PROT_VAL_THRESHOLD_2] = PROT_LOWER_VOLT_POW_DN3;
    lvp->protValThr[PROT_VAL_THRESHOLD_3] = PROT_LOWER_VOLT_POW_OFF;
    /* Configure the protection limit time. */
    lvp->protLimitTime[PROT_LIMIT_TIME_0] = PROT_LOWER_VOLT_LIMIT1_TIME_SEC;
    lvp->protLimitTime[PROT_LIMIT_TIME_1] = PROT_LOWER_VOLT_LIMIT2_TIME_SEC;
    lvp->protLimitTime[PROT_LIMIT_TIME_2] = PROT_LOWER_VOLT_LIMIT3_TIME_SEC;
    lvp->recyDelta = PROT_OVER_VOLT_RECY_DELTA;
}

/**
  * @brief Over dc-link voltage protection detection.
  * @param ovp Over dc-link voltage protection handle.
  * @param motorErrStatus Motor error status.
  * @param udc DC-link voltage feedback (V).
  * @retval None.
  */
void OVP_Det(OVP_Handle *ovp, MotorErrStatusReg *motorErrStatus, float udc)
{
    MCS_ASSERT_PARAM(ovp != NULL);
    MCS_ASSERT_PARAM(motorErrStatus != NULL);
    MCS_ASSERT_PARAM(udc > 0.0f);
    /* Check if value goes over threshold for continuous cycles. */
    if (udc < ovp->protValThr[PROT_VAL_THRESHOLD_0]) {
        ovp->protCnt = 0;
        return;
    }

    if (ovp->protCnt < ovp->protCntLimit) {
        ovp->protCnt++;
        return;
    }

    /* Check which protection level should be activated. */
    /* Once enter error status, error status can only be changed by recover api. */
    /* protect level: level 4. */
    if (udc >= ovp->protValThr[PROT_VAL_THRESHOLD_3] && ovp->protLevel < PROT_LEVEL_4) {
        ovp->protLevel = PROT_LEVEL_4;
        motorErrStatus->Bit.overVoltErr = 1;
        ovp->protCnt = 0;
        return;
    }

    /* Protect level: level 3. */
    if (udc >= ovp->protValThr[PROT_VAL_THRESHOLD_2] && ovp->protLevel < PROT_LEVEL_3) {
        ovp->protLevel = PROT_LEVEL_3;
        motorErrStatus->Bit.overVoltErr = 1;
        ovp->protCnt = 0;
        ovp->timer = 0.0f;
        return;
    }

    /* Protect level: level 2. */
    if (udc >= ovp->protValThr[PROT_VAL_THRESHOLD_1] && ovp->protLevel < PROT_LEVEL_2) {
        ovp->protLevel = PROT_LEVEL_2;
        motorErrStatus->Bit.overVoltErr = 1;
        ovp->protCnt = 0;
        ovp->timer = 0.0f;
        return;
    }

    /* Protect level: level 1. */
    if (udc >= ovp->protValThr[PROT_VAL_THRESHOLD_0] && ovp->protLevel < PROT_LEVEL_1) {
        ovp->protLevel = PROT_LEVEL_1;
        motorErrStatus->Bit.overVoltErr = 1;
        ovp->protCnt = 0;
        ovp->timer = 0.0f;
        return;
    }
}

/**
  * @brief Lower dc-link voltage protection detection.
  * @param lvp Lower dc-link voltage protection handle.
  * @param motorErrStatus Motor error status.
  * @param udc DC-link voltage feedback (V).
  * @retval None.
  */
void LVP_Det(LVP_Handle *lvp, MotorErrStatusReg *motorErrStatus, float udc)
{
    MCS_ASSERT_PARAM(lvp != NULL);
    MCS_ASSERT_PARAM(motorErrStatus != NULL);
    MCS_ASSERT_PARAM(udc > 0.0f);
    /* Check if value goes over threshold for continuous cycles. */
    if (udc > lvp->protValThr[PROT_VAL_THRESHOLD_0]) {
        lvp->protCnt = 0;
        return;
    }

    if (lvp->protCnt < lvp->protCntLimit) {
        lvp->protCnt++;
        return;
    }

    /* Check which protection level should be activated. */
    /* Once enter error status, error status can only be changed by recover api. */
    /* protect level: level 4. */
    if (udc <= lvp->protValThr[PROT_VAL_THRESHOLD_3] && lvp->protLevel < PROT_LEVEL_4) {
        lvp->protLevel = PROT_LEVEL_4;
        motorErrStatus->Bit.lowerVoltErr = 1;
        lvp->protCnt = 0;
        return;
    }

    /* Protect level: level 3. */
    if (udc <= lvp->protValThr[PROT_VAL_THRESHOLD_2] && lvp->protLevel < PROT_LEVEL_3) {
        lvp->protLevel = PROT_LEVEL_3;
        motorErrStatus->Bit.lowerVoltErr = 1;
        lvp->protCnt = 0;
        lvp->timer = 0.0f;
        return;
    }

    /* Protect level: level 2. */
    if (udc <= lvp->protValThr[PROT_VAL_THRESHOLD_1] && lvp->protLevel < PROT_LEVEL_2) {
        lvp->protLevel = PROT_LEVEL_2;
        motorErrStatus->Bit.lowerVoltErr = 1;
        lvp->protCnt = 0;
        lvp->timer = 0.0f;
        return;
    }

    /* Protect level: level 1. */
    if (udc <= lvp->protValThr[PROT_VAL_THRESHOLD_0] && lvp->protLevel < PROT_LEVEL_1) {
        lvp->protLevel = PROT_LEVEL_1;
        motorErrStatus->Bit.lowerVoltErr = 1;
        lvp->protCnt = 0;
        lvp->timer = 0.0f;
        return;
    }
}

/**
  * @brief Over dc-link voltage protection execution.
  * @param ovp Over dc-link voltage protection handle.
  * @param duty Brake loop output duty (0-1).
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
void OVP_Exec(OVP_Handle *ovp, float *duty, APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(ovp != NULL);
    MCS_ASSERT_PARAM(duty != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);
    /* According to protect level, take corresponding action. */
    switch (ovp->protLevel) {
        /* level 4: brake loop duty maximum. */
        case PROT_LEVEL_4:
            *duty = PROT_OVER_VOLT_BRK_DUTY4;
            /* Disable three-phase pwm output. */
            ProtSpo_Exec(aptAddr);
            break;
  
        /* level 3: brake loop duty level 3. */
        case PROT_LEVEL_3:
            *duty = PROT_OVER_VOLT_BRK_DUTY2;
            ovp->timer += ovp->ts;
            if (ovp->timer > ovp->protLimitTime[PROT_LIMIT_TIME_2]) {
                *duty = PROT_OVER_VOLT_BRK_DUTY3;
            }
            break;
  
        /* level 2: brake loop duty level 2. */
        case PROT_LEVEL_2:
            *duty = PROT_OVER_VOLT_BRK_DUTY1;
            ovp->timer += ovp->ts;
            if (ovp->timer > ovp->protLimitTime[PROT_LIMIT_TIME_1]) {
                *duty = PROT_OVER_VOLT_BRK_DUTY2;
            }
            break;
  
        /* level 1: brake loop duty level 1. */
        case PROT_LEVEL_1:
            ovp->timer += ovp->ts;
            if (ovp->timer > ovp->protLimitTime[PROT_LIMIT_TIME_0]) {
                *duty = PROT_OVER_VOLT_BRK_DUTY1;
            }
            break;
  
        /* level 0: take no protection action. */
        case PROT_LEVEL_0:
            break;
  
        default:
            break;
    }
    return;
}

/**
  * @brief Lower dc-link voltage protection execution.
  * @param lvp Lower dc-link voltage protection handle.
  * @param spdRef Speed Reference (Hz).
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
void LVP_Exec(LVP_Handle *lvp, float *spdRef, APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(lvp != NULL);
    MCS_ASSERT_PARAM(spdRef != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);
    /* According to protect level, take corresponding action. */
    switch (lvp->protLevel) {
        /* level 4: disable all PWM output. */
        case PROT_LEVEL_4:
            /* Disable three-phase pwm output. */
            ProtSpo_Exec(aptAddr);
            *spdRef *= 0.0f;
            break;
  
        /* level 3: derate speed reference. */
        case PROT_LEVEL_3:
            *spdRef *= PROT_POW_DN2_PCT;
            lvp->timer += lvp->ts;
            if (lvp->timer > lvp->protLimitTime[PROT_LIMIT_TIME_2]) {
                *spdRef *= PROT_POW_DN3_PCT;
            }
            break;
  
        /* level 2: derate speed reference. */
        case PROT_LEVEL_2:
            *spdRef *= PROT_POW_DN1_PCT;
            lvp->timer += lvp->ts;
            if (lvp->timer > lvp->protLimitTime[PROT_LIMIT_TIME_1]) {
                *spdRef *= PROT_POW_DN2_PCT;
            }
            break;
  
        /* level 1: derate speed reference. */
        case PROT_LEVEL_1:
            lvp->timer += lvp->ts;
            if (lvp->timer > lvp->protLimitTime[PROT_LIMIT_TIME_0]) {
                *spdRef *= PROT_POW_DN1_PCT;
            }
            break;
  
        /* level 0: take no protection action. */
        case PROT_LEVEL_0:
            break;
  
        default:
            break;
    }
    return;
}

/**
  * @brief Over dc-link voltage protection recovery.
  * @param ovp Over dc-link voltage protection handle.
  * @param motorErrStatus  Motor error status.
  * @param udc DC-link voltage (V).
  * @retval None.
  */
void OVP_Recy(OVP_Handle *ovp, MotorErrStatusReg *motorErrStatus, float udc)
{
    MCS_ASSERT_PARAM(ovp != NULL);
    /* If not under error state, just return without any operation. */
    if (!motorErrStatus->Bit.overVoltErr) {
        return;
    }

    /* According to protection level, take corresponding recovery action. */
    switch (ovp->protLevel) {
        /* level 4 */
        case PROT_LEVEL_4:
            if (udc < ovp->protValThr[PROT_VAL_THRESHOLD_3] - ovp->recyDelta) {
                ovp->recyCnt++;
                if (ovp->recyCnt > ovp->recyCntLimit) {
                    ovp->protLevel = PROT_LEVEL_3;
                    ovp->recyCnt = 0;
                }
            }
            break;
  
        /* level 3 */
        case PROT_LEVEL_3:
            /* If the dc-link voltage is less than threshold 2, level-2 protection is restored. */
            if (udc < ovp->protValThr[PROT_VAL_THRESHOLD_2] - ovp->recyDelta) {
                ovp->recyCnt++;
                if (ovp->recyCnt > ovp->recyCntLimit) {
                    ovp->protLevel = PROT_LEVEL_2;
                    ovp->recyCnt = 0;
                }
            }
            break;
  
        /* level 2 */
        case PROT_LEVEL_2:
            /* If the dc-link voltage is less than threshold 1, level-1 protection is restored. */
            if (udc < ovp->protValThr[PROT_VAL_THRESHOLD_1] - ovp->recyDelta) {
                ovp->recyCnt++;
                if (ovp->recyCnt > ovp->recyCntLimit) {
                    ovp->protLevel = PROT_LEVEL_1;
                    ovp->recyCnt = 0;
                }
            }
            break;
  
        /* level 1 */
        case PROT_LEVEL_1:
            /* If the dc-link voltage is less than threshold 0, level-0 protection is restored. */
            if (udc < ovp->protValThr[PROT_VAL_THRESHOLD_0] - ovp->recyDelta) {
                ovp->recyCnt++;
                if (ovp->recyCnt > ovp->recyCntLimit) {
                    ovp->protLevel = PROT_LEVEL_0;
                    ovp->recyCnt = 0;
                }
            }
            break;
        /* level 0 Fault-free state. */
        case PROT_LEVEL_0:
            motorErrStatus->Bit.overVoltErr = 0;
            break;

        default:
            break;
    }
    return;
}

/**
  * @brief Lower dc-link voltage protection recovery.
  * @param lvp Lower dc-link voltage protection handle.
  * @param motorErrStatus  Motor error status.
  * @param udc DC-link voltage (V).
  * @retval None.
  */
void LVP_Recy(LVP_Handle *lvp, MotorErrStatusReg *motorErrStatus, float udc)
{
    MCS_ASSERT_PARAM(lvp != NULL);
    /* If not under error state, just return without any operation. */
    if (!motorErrStatus->Bit.lowerVoltErr) {
        return;
    }

    /* According to protection level, take corresponding recovery action. */
    switch (lvp->protLevel) {
        /* level 4 */
        case PROT_LEVEL_4:
            /* If the dc-link voltage is greater than threshold 3, level-3 protection is restored. */
            if (udc > lvp->protValThr[PROT_VAL_THRESHOLD_3] + lvp->recyDelta) {
                lvp->recyCnt++;
                if (lvp->recyCnt > lvp->recyCntLimit) {
                    lvp->protLevel = PROT_LEVEL_3;
                    lvp->recyCnt = 0;
                }
            }
            break;
  
        /* level 3 */
        case PROT_LEVEL_3:
            /* If the dc-link voltage is greater than threshold 2, level-2 protection is restored. */
            if (udc > lvp->protValThr[PROT_VAL_THRESHOLD_2] + lvp->recyDelta) {
                lvp->recyCnt++;
                if (lvp->recyCnt > lvp->recyCntLimit) {
                    lvp->protLevel = PROT_LEVEL_2;
                    lvp->recyCnt = 0;
                }
            }
            break;
  
        /* level 2 */
        case PROT_LEVEL_2:
            /* If the dc-link voltage is greater than threshold 1, level-1 protection is restored. */
            if (udc > lvp->protValThr[PROT_VAL_THRESHOLD_1] + lvp->recyDelta) {
                lvp->recyCnt++;
                if (lvp->recyCnt > lvp->recyCntLimit) {
                    lvp->protLevel = PROT_LEVEL_1;
                    lvp->recyCnt = 0;
                }
            }
            break;
  
        /* level 1 */
        case PROT_LEVEL_1:
            /* If the dc-link voltage is greater than threshold 0, level-0 protection is restored. */
            if (udc > lvp->protValThr[PROT_VAL_THRESHOLD_0] + lvp->recyDelta) {
                lvp->recyCnt++;
                if (lvp->recyCnt > lvp->recyCntLimit) {
                    lvp->protLevel = PROT_LEVEL_0;
                    lvp->recyCnt = 0;
                }
            }
            break;
        /* level 0 Fault-free state. */
        case PROT_LEVEL_0:
            motorErrStatus->Bit.lowerVoltErr = 0;
            break;
    
        default:
            break;
    }
    return;
}

/**
  * @brief Over dc-link voltage protection error status clear.
  * @param ovp Over voltage protection handle.
  * @retval None.
  */
void OVP_Clear(OVP_Handle *ovp)
{
    MCS_ASSERT_PARAM(ovp != NULL);
    /* Clear the history value. */
    ovp->protCnt = 0;
    ovp->protLevel = PROT_LEVEL_0;
    ovp->recyCnt = 0;
    ovp->timer = 0.0f;
}

/**
  * @brief Lower dc-link voltage protection error status clear.
  * @param lvp Lower voltage protection handle.
  * @retval None.
  */
void LVP_Clear(LVP_Handle *lvp)
{
    MCS_ASSERT_PARAM(lvp != NULL);
    /* Clear the history value. */
    lvp->protCnt = 0;
    lvp->protLevel = PROT_LEVEL_0;
    lvp->recyCnt = 0;
    lvp->timer = 0.0f;
}