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
  * @file      mcs_chip_config.h
  * @author    MCU Algorithm Team
  * @brief     This file provides config macros for ECMCU105H/ECBMCU201MPC app.
  */

#ifndef McuMagicTag_MCS_CHIPCONFIG_H
#define McuMagicTag_MCS_CHIPCONFIG_H

#include "feature.h"

#ifdef CHIP_ZDU3061MNPICA

    #define ADCPTT_HANDLE       g_adc0
    #define ADCRESIS_HANDLE     g_adc0
    #define ADCUDC_HANDLE       g_adc0
    #define ADCU_HANDLE         g_adc0
    #define ADCW_HANDLE         g_adc0
    #define LED1_HANDLE         g_gpio2
    #define LED2_HANDLE         g_gpio1
    #define LED2_PIN            GPIO_PIN_0
    #define LED1_PIN            GPIO_PIN_3
    #define ADCPTTSOCNUM        ADC_SOC_NUM0
    #define ADCUSOCNUM          ADC_SOC_NUM1
    #define ADCRESISSOCNUM      ADC_SOC_NUM3
    #define ADCUDCSOCNUM        ADC_SOC_NUM4
    #define ADCWSOCNUM          ADC_SOC_NUM6

    #define ADC0COMPENSATE      2037.0f
    #define ADC1COMPENSATE      2027.0f

#endif

#ifdef CHIP_3065HRPIRZ

    #define ADCU_HANDLE         g_adc0
    #define ADCW_HANDLE         g_adc1
    #define ADCRESIS_HANDLE     g_adc2
    #define ADCUDC_HANDLE       g_adc2
    #define ADCPTT_HANDLE       g_adc2
    #define LED1_HANDLE         g_gpio0
    #define LED2_HANDLE         g_gpio0
    #define LED2_PIN            GPIO_PIN_6
    #define LED1_PIN            GPIO_PIN_7
    #define ADCPTTSOCNUM        ADC_SOC_NUM0
    #define ADCRESISSOCNUM      ADC_SOC_NUM1
    #define ADCUSOCNUM          ADC_SOC_NUM8
    #define ADCWSOCNUM          ADC_SOC_NUM8
    #define ADCUDCSOCNUM        ADC_SOC_NUM14

    #define ADC0COMPENSATE      2033.0f
    #define ADC1COMPENSATE      2070.0f

#endif


#endif