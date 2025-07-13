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
  * @file    tsensor_ex.c
  * @author  MCU Driver Team
  * @brief   tsensor module driver
  * @details This file provides functions to manage Extended tsensor functions.
  */

/* Includes ------------------------------------------------------------------*/

#include "adc_tsensor.h"
#include "tsensor_ex.h"

static int g_lastCode;
static float g_lastTemp;
static bool g_firstFlag = false;

/**
  * @brief The formula calculates the compensation value.
  * @param temp, Temperature collected by the tsensor.
  * @retval int, HOSC compensation code.
  */
static int HOSC_CalculateValue(float temp)
{
    float hosc = (-0.00002f * temp * temp) - (0.0013f * temp) + 25.2652f;  /* Formula : -0.00002, 0.0013, 25.2652 */
    float tCode = (25.217f - hosc) / 0.057f;  /* Formula Parameter: 25.217, 0.057 */
    float ret;
    if (tCode >= 0) {
        ret = tCode + 0.5f; /* 0.5 is for rounding */
        return (int)ret;
    } else {
        ret = 0.5f - tCode; /* 0.5 is for rounding */
        return (0 - (int)ret);
    }
}

/**
  * @brief Set the compensation value.
  * @param temp, Temperature collected by the tsensor.
  * @param hoscCode, HOSC compensation code.
  * @retval bool.
  */
static void HOSC_SetValue(float temp, int hoscCode)
{
    int value = (int)g_hosc_ctrim - hoscCode + 1;  /* Calculate the value of the configured register */
    unsigned int *hoscReg;
    hoscReg = (void *)0x10000100;
    *hoscReg = (unsigned int)value;
    g_lastCode =  hoscCode;
    g_lastTemp = temp;
}

/**
  * @brief Temperature compensation threshold check.
  * @param temp, Temperature collected by the tsensor.
  * @param hoscCode, HOSC compensation code
  * @retval bool.
  */
static bool HOSC_ThresholdCheck(float temp, int hoscCode)
{
    if (hoscCode == g_lastCode) {
        return false;
    }
    float threshold = ((temp > g_lastTemp) ? (temp - g_lastTemp) : (g_lastTemp - temp));
    if (threshold <= 5.0f) { /* 5.0f is the temperature threshold. */
        return false;
    }
    return true;
}

/**
  * @brief Get the result from the tsensor.
  * @param None.
  * @retval BASE_StatusType, OK, ERROR.
  */
BASE_StatusType HAL_TSENSOR_CalibrateHosc(void)
{
    if (g_hosc_ctrim == 0x1FF) {
        return BASE_STATUS_ERROR; /* g_hosc_ctrim is invalid */
    }
    if (g_firstFlag == false) {
        HAL_TSENSOR_Init();
    }
    float temp = HAL_TSENSOR_GetTemperature();
    if (temp < -55.0f || temp > 130.0f) { /* Collection Scope is: -55.0 ~ 130.0 */
        return BASE_STATUS_ERROR;
    }
    int hoscCode = HOSC_CalculateValue(temp);
    if (g_firstFlag == false) {
        HOSC_SetValue(temp, hoscCode);
        g_firstFlag = true;
    } else if (HOSC_ThresholdCheck(temp, hoscCode) == true) { /* The threshold condition is met */
        HOSC_SetValue(temp, hoscCode);
    }
    return BASE_STATUS_OK;
}