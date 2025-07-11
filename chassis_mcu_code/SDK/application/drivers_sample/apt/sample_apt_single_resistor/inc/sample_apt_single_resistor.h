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
  * @file    apt_hal_sample.h
  * @author  MCU Driver Team
  * @brief   Header file containing functions prototypes of APT module HAL sample.
  */

#ifndef McuMagicTag_APT_HAL_SAMPLE_H
#define McuMagicTag_APT_HAL_SAMPLE_H

#include "apt_ip.h"
#include "interrupt.h"

/**
  * @brief ADC current sample mode.
  */
typedef enum {
    ADC_SINGLE_RESISTOR = 0x00000000U,
    ADC_THREE_RESISTORS = 0x00000001U,
} ADC_SampleMode;

void APT_PWMInitHALSample(void);
void APT_SetPWMDutyU(unsigned int duty);
void APT_SetPWMDutyV(unsigned int duty);
void APT_SetPWMDutyW(unsigned int duty);
void APT_SetADCTrgTime(unsigned short cntCmpSOCA, unsigned short cntCmpSOCB, ADC_SampleMode mode);
void APT_PhaseOut(bool enable);
void APT_RunAllPwm(void);
void APT_StopAllPwm(void);
#endif /* McuMagicTag_APT_HAL_SAMPLE_H */