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
  * @file     mcs_adcCalibr.h
  * @author   MCU Algorithm Team
  * @brief    This file provides functions declaration for adc bias calibration function.
  */
#ifndef McuMagicTag_MCS_ADCCALIB_H
#define McuMagicTag_MCS_ADCCALIB_H

#include "adc.h"
#include "typedefs.h"

/* Macro definitions --------------------------------------------------------------------------- */
#define ADC_CNT_POINTS 50 /* the number of continuous adc results for calibration */

/**
  * @brief  ADC temperature calibration state.
  */
typedef enum {
    ADC_CALIBR_NOT_FINISH = 0,
    ADC_CALIBR_FINISH
} ADC_CALIBR_State;

/**
  * @brief Adc temperature shift calibration structure.
  */
typedef struct {
    unsigned int adcShiftAccu;
    unsigned int cnt;
    ADC_CALIBR_State state;
} ADC_CALIBR_Handle;


void ADCCALIBR_Init(ADC_CALIBR_Handle *adcCalibr);

unsigned int ADCCALIBR_Exec(ADC_CALIBR_Handle *adcCalibr, ADC_Handle *adcHandle, unsigned int soc);

bool ADCCALIBR_IsFinish(ADC_CALIBR_Handle *adcCalibr);

#endif