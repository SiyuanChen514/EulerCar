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
  * @file    sample_tsensor.c
  * @author  MCU Driver Team
  * @brief   tsensor sample module.
  * @details This file provides sample to get temperature of mcu by using tsensor.
  */
#include "sample_tsensor.h"

#define SAMPLE_COUNT 16

/**
  * @brief Tsensor samples 16 times, takes the average value, and converts the average value to degrees Celsius.
  * @param None.
  * @retval None.
  */
void TSENSOR_GetAveTemp(void)
{
    DBG_UartPrintInit(BAUDRATE);
    float total = 0;
    float ret;
    float aveTemp;
    HAL_TSENSOR_Init();
    /* Tsensor cyclic sampling */
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        ret = HAL_TSENSOR_GetTemperature();
        total += ret;
    }
    /* Converting the sampled average value of the test sensor to temperature */
    aveTemp = total / SAMPLE_COUNT;
    DBG_PRINTF("Tsensor Average temperature: %f\r\n", aveTemp);
    return;
}