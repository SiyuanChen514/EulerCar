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
  * @file      mcs_openphs_det.c
  * @author    MCU Algorithm Team
  * @brief     This file contains self-check open phase fault detection function data struct and api declaration.
  */


#include "mcs_openphs_det.h"
#include "mcs_assert.h"


/**
  * @brief Open phase detection initialization.
  * @param opp Open phase detection handle.
  * @param minOpenPhsCurr Minimum current for open-phase detection (A).
  * @retval None.
  */
void OPD_Init(OPD_Handle *opd, float minOpenPhsCurr)
{
    MCS_ASSERT_PARAM(opd != NULL);
    MCS_ASSERT_PARAM(minOpenPhsCurr > 0.0f);
    /* Minimum current for open-phase detection (A). */
    opd->minOpenPhsCurr = minOpenPhsCurr;
    /* No phase open. */
    opd->isOpenPhsU = 0;
    opd->isOpenPhsV = 0;
    opd->isOpenPhsW = 0;
}


/**
  * @brief Open phase detection execution.
  * @param opd Open phase detection handle.
  * @param iuvw Phase current feedback values (A).
  * @retval Whether the motor is open phase, ture: open phase, 0: no open phase.
  */
bool OPD_Exec(OPD_Handle *opd, const float *iuvw)
{
    MCS_ASSERT_PARAM(opd != NULL);
    MCS_ASSERT_PARAM(iuvw != NULL);
    float minCurr = opd->minOpenPhsCurr;
    /* Open phase detection for phase U */
    if (iuvw[OPD_V_U] <= minCurr && iuvw[OPD_W_U] <= minCurr) { /* 4th step curr */
        opd->isOpenPhsU = true;
    }
    /* Open phase detection for phase V */
    if (iuvw[OPD_U_V] <= minCurr && iuvw[OPD_W_V] <= minCurr) { /* 2th step curr */
        opd->isOpenPhsV = true;
    }

    /* Open phase detection for phase W */
    if (iuvw[OPD_V_W] <= minCurr && iuvw[OPD_U_W] <= minCurr) { /* 2th ,4th step curr */
        opd->isOpenPhsW = true;
    }

    return (opd->isOpenPhsU || opd->isOpenPhsV || opd->isOpenPhsW);
}


/**
  * @brief Clear Open phase history value.
  * @param opd Open phase detection handle.
  * @retval None.
  */
void OPD_Clear(OPD_Handle *opd)
{
    MCS_ASSERT_PARAM(opd != NULL);
    opd->isOpenPhsU = 0;
    opd->isOpenPhsV = 0;
    opd->isOpenPhsW = 0;
}