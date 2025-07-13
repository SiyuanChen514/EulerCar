/**
  * @ Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2023. All rights reserved.
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
  * @file      mcs_unbalance_det.c
  * @author    MCU Algorithm Team
  * @brief     This file provides motor application for Three-phase imbalance detection.
  */


#include "mcs_unbalance_det.h"
#include "mcs_math.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"


#define START_UNBAL_DET_TIME_S    5

/**
  * @brief Initilization three-phase unbalance protection function.
  * @param unbal Three-phase unbalance detect handle.
  * @param currDelta Threshold for determining the zero-crossing point of the phase current.
  * @param timeThr Time thredhold of duration , unit: s.
  * @param unbalDegreeLim Threshold of the imbalance degree.
  * @param ts Ctrl period (s).
  * @retval None.
  */
void UNBAL_Init(UNBAL_Handle *unbal, float currDelta, float timeThr, float unbalDegreeLim, float ts)
{
    MCS_ASSERT_PARAM(unbal != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    unbal->detCntLimit = (unsigned int)(timeThr / ts);
    unbal->unbalDegreeLimit = unbalDegreeLim;
    /* Configuring Limit Times */
    unbal->delta = currDelta;
    /* Restore the initial state. */
    unbal->startFlagLast = false;
    unbal->startFlag = false;
    unbal->ts = ts;
    unbal->startupCnt = (unsigned int)(START_UNBAL_DET_TIME_S / ts);
    unbal->detCnt = 0;
    UNBAL_Clear(unbal);
}


/**
  * @brief  Get three-phase current rms.
  * @param unbal Three-phase unbalance detect handle.
  * @param iUvw Three-phase current (A).
  * @retval None.
  */
static void UNBAL_RmsCurrCalc(UNBAL_Handle *unbal, UvwAxis iUvw)
{
    MCS_ASSERT_PARAM(unbal != NULL);
    if (unbal->calFlag) {
        /* rms integral */
        unbal->ia += (iUvw.u * iUvw.u * unbal->ts);
        unbal->ib += (iUvw.v * iUvw.v * unbal->ts);
        unbal->ic += (iUvw.w * iUvw.w * unbal->ts);
        unbal->ia = Clamp(unbal->ia, LARGE_FLOAT, -LARGE_FLOAT);
        unbal->ib = Clamp(unbal->ib, LARGE_FLOAT, -LARGE_FLOAT);
        unbal->ic = Clamp(unbal->ic, LARGE_FLOAT, -LARGE_FLOAT);

        unbal->timeCnt++;
        /* Filter out the incomplete period data before the calculation starts. */
        if (unbal->timeCnt < unbal->startupCnt) {
            unbal->unbalDegree = 0.0f;
        } else if (unbal->timeCnt > unbal->startupCnt + unbal->startupCnt) {
            /* Current accumulation is abnormal. */
            unbal->calFlag = false;
            unbal->unbalDegree = 0.0f;
            unbal->timeCnt = 0;
        } else {
            if (unbal->startFlagLast != unbal->startFlag) {
                unbal->timeCnt = unbal->startupCnt;
            }
        }
    }
}


/**
  * @brief  Get three-phase current rms.
  * @param unbal Three-phase unbalance detect handle.
  * @param iuvwFbk Three-phase current (A).
  * @retval None.
  */
static void UNBAL_RmsCurrGet(UNBAL_Handle *unbal, UvwAxis *iuvwFbk)
{
    UvwAxis iUvw;
    float delta = unbal->delta;

    iUvw.u = iuvwFbk->u;
    iUvw.v = iuvwFbk->v;
    iUvw.w = iuvwFbk->w;
    /* Current zero-crossing detection */
    if (iUvw.u < -delta && unbal->startFlag == false) {
        unbal->zeroFlag = true;
    }
    /* Current cycle start judgment */
    if (iUvw.u > delta && unbal->startFlag == false && unbal->zeroFlag) {
        unbal->startFlag = true;
        unbal->zeroFlag = false;
    }

    if (unbal->startFlag) {
       /* Accumulated number of integral */
        unbal->integralCnt++;
        /* Periodic zero crossing detection */
        if (iUvw.u < -delta) {
            unbal->zeroFlag = true;
        }
        if (iUvw.u > delta && unbal->zeroFlag) {
            unbal->zeroFlag = false;
            unbal->startFlag = false;
        }
    }
    UNBAL_RmsCurrCalc(unbal, iUvw);
}


/**
  * @brief Three-phase unbalance calculation.
  * @param unbal Three-phase unbalance detect handle.
  * @param iuvwFbk Three-phase current (A).
  * @param unbalFltCoeff Average filter coefficient for calculating current unbalance degree.
  * @retval None.
  */
static void UNBAL_Calc(UNBAL_Handle *unbal, UvwAxis *iuvwFbk, float unbalFltCoeff)
{
    /* Get rms current */
    UNBAL_RmsCurrGet(unbal, iuvwFbk);
    /* Current cycle sampling completed */
    if (unbal->startFlagLast != unbal->startFlag) {
        unbal->calFlag = true;
        if (Abs(unbal->ia) <= 1e-6) { /* Whether there is current */
            unbal->unbalDegree = 0.0f;
            return;
        }
        /* Calculate the three-phase current rms value. */
        float time = (float)unbal->integralCnt * unbal->ts;
        float ia = Sqrt(unbal->ia / time);
        float ib = Sqrt(unbal->ib / time);
        float ic = Sqrt(unbal->ic / time);
        unbal->integralCnt = 0;

        /* Based on the symmetrical component method,
           three groups of symmetrical components and three-phase currents are
           decomposed under the condition of three-phase phase symmetry.
           The relationship between amplitudes is as follows: */
        float ia1 = ONE_DIV_THREE * (ia + ib + ic); /* Ia1 = 1/3 * (ia + ib + ic) */
        float tmp = (ia - 0.5f * ib - 0.5f * ic) * (ia - 0.5f * ib - 0.5f * ic);
        float tmp2 = 0.75f * (ib - ic) * (ib - ic); /* Ia2 = 1/3 * sqrt((ia - 0.5 * ib)^2 + 3/4 * (ib -ic)^2) */
        float tmp3 = 0.75f * (ic - ib) * (ic - ib); /* Ia0 = 1/3 * sqrt((ia - 0.5 * ib)^2 + 3/4 * (ic -ib)^2) */
        float ia2 = ONE_DIV_THREE * Sqrt(tmp + tmp2);
        float ia0 = ONE_DIV_THREE * Sqrt(tmp + tmp3);
        float ig = Sqrt(ia0 * ia0 + ia2 * ia2); /* Total unbalanced current */
        float igPer = ig / ia1;                     /* Current unbalance factor */
        unbal->unbalDegree = unbal->unbalDegree * (1.0f - unbalFltCoeff) + igPer * unbalFltCoeff;
        /* Clear current history value */
        unbal->ia = 0.0f;
        unbal->ib = 0.0f;
        unbal->ic = 0.0f;
    }
    unbal->startFlagLast = unbal->startFlag;
}


/**
  * @brief Three-phase unbalance protection detection.
  * @param unbal Three-phase unbalance detect handle.
  * @param iuvwFbk Three-phase current.
  * @retval None.
  */
bool UNBAL_Det(UNBAL_Handle *unbal, UvwAxis *iuvwFbk, float unbalFltCoeff)
{
    MCS_ASSERT_PARAM(unbal != NULL);
    MCS_ASSERT_PARAM(iuvwFbk != NULL);

    UNBAL_Calc(unbal, iuvwFbk, unbalFltCoeff);
    /* The three-phase imbalance exceeds the limit value. */
    if (unbal->unbalDegree > unbal->unbalDegreeLimit) {
        unbal->detCnt++;
        /* Current out of balance fault is detected, */
        /* when the protection hysteresis count is greater than the threshold. */
        if (unbal->detCnt > unbal->detCntLimit) {
            unbal->detCnt = 0;
            return false;
        }
    } else {
        unbal->detCnt = 0;
    }
    return true;
}

/**
  * @brief Clear historical status of three-phase unbalance detection.
  * @param unbal Three-phase unbalance detect handle.
  * @retval None.
  */
void UNBAL_Clear(UNBAL_Handle *unbal)
{
    MCS_ASSERT_PARAM(unbal != NULL);
    /* Clear historical status */
    unbal->ia = 0.0f;
    unbal->ib = 0.0f;
    unbal->ic = 0.0f;
    unbal->unbalDegree = 0.0f;
    unbal->calFlag = false;
    /* Detection time count. */
    unbal->timeCnt = 0;
}