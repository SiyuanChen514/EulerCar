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
  * @file    sample_adc_single_with_ppb.c
  * @author  MCU Driver Team
  * @brief   adc sample module.
  * @details In the post-processing example of the ADC, there is the normal trigger sampling function and four
  *          post-processing functions: error calculation, upper threshold detection, lower threshold detection,
  *          and zero-crossing detection.
  */
#include "sample_adc_single_with_ppb.h"

void ADC_PPB1Error(void *handle);
void ADC_PPB1Up(void *handle);
void ADC_PPB1Down(void *handle);
void ADC_PPB1Zero(void *handle);
/**
  * @brief User callback function of ADC interrupt one.
  * @param handle ADC handle.
  * @retval None.
  */
void ADC_Int1Finish(ADC_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("ADC_Int1Finish\r\n");
    unsigned int ret = HAL_ADC_GetConvResult(&g_adc, ADC_SOC_NUM1);
    DBG_PRINTF("result: %d\r\n", ret);
    float voltage = (float)ret / (float)4096 * 3.3;  /* 4096 and 3.3 are for Sample Value Conversion */
    DBG_PRINTF("voltage: %f\r\n", voltage);
}

/**
  * @brief User callback function of ADC interrupt one.
  * @param handle ADC handle.
  * @retval None.
  */
void ADC_PPB1Error(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("ADC_PPB1Error: Calculation error completed, errorRef - result\r\n");
    int ret = HAL_ADC_GetPPBxErrorResultEx(&g_adc, ADC_PPB_NUM1);  /* Obtain the error value */
    DBG_PRINTF("ErrorResultEx: %d\r\n", ret);
    unsigned int delay = HAL_ADC_GetPPBxDelayCntEx(&g_adc, ADC_PPB_NUM1);
    DBG_PRINTF("delay: %d\r\n", delay);
}

/**
  * @brief User callback function of ADC interrupt one.
  * @param handle ADC handle.
  * @retval None.
  */
void ADC_PPB1Up(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("ADC_PPB1Up: The PPB upper threshold overflows\r\n");
}

/**
  * @brief User callback function of ADC interrupt one.
  * @param handle ADC handle.
  * @retval None.
  */
void ADC_PPB1Down(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("ADC_PPB1Down: The PPB down threshold overflows\r\n");
}

/**
  * @brief User callback function of ADC interrupt one.
  * @param handle ADC handle.
  * @retval None.
  */
void ADC_PPB1Zero(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("ADC_PPB1Zero\r\n");
}

/**
  * @brief ADC single channel sample with interrupt. Each channel independently generate an interrupt.
  * @param None.
  * @retval None.
  */
void ADC_SingleTriggerItWithPPB(void)
{
    SystemInit();
    DBG_PRINTF("ADC_SingleTriggerItWithPPB begin\r\n");
    HAL_ADC_RegisterCallBack(&g_adc, ADC_CALLBACK_EVENT_PPB1_ERROR, ADC_PPB1Error);
    HAL_ADC_RegisterCallBack(&g_adc, ADC_CALLBACK_EVENT_PPB1_UP, ADC_PPB1Up);
    HAL_ADC_RegisterCallBack(&g_adc, ADC_CALLBACK_EVENT_PPB1_DOWN, ADC_PPB1Down);
    HAL_ADC_RegisterCallBack(&g_adc, ADC_CALLBACK_EVENT_PPB1_ZERO, ADC_PPB1Zero);
    HAL_ADC_StartIt(&g_adc);
    HAL_ADC_SoftTrigSample(&g_adc, ADC_SOC_NUM1);  /* Software trigger ADC sampling */

    BASE_FUNC_DELAY_S(1);   /* After the delay is 1s, the PPB is enabled for re-sampling */
    DBG_PRINTF("\r\nPPB1 Setting offset: -2000; errorRef: 100; threshold: < -10 or > 10\r\n");
    PPB_Function ppbfun = {0};
    ppbfun.offset = BASE_CFG_ENABLE;   /* Using the Offset Calculation Function */
    ppbfun.detect = BASE_CFG_ENABLE;   /* Use the threshold detection function for offset calculation results */
    ppbfun.delay = BASE_CFG_ENABLE;    /* Enable the sampling delay recording function */
    HAL_ADC_ConfigurePPBxEx(&g_adc, ADC_SOC_NUM1, ADC_PPB_NUM1, &ppbfun);

    /* Sets -2000 as the offset value. The original result and offset result are summed up */
    HAL_ADC_SetPPBxOffsetEx(&g_adc, ADC_PPB_NUM1, -2000);
    /* Set 100 as the error reference value. Error reference value minus offset calculation data */
    HAL_ADC_SetPPBxErrorRefEx(&g_adc, ADC_PPB_NUM1, 100);
    /* Setting Comparison Thresholds 10 and -10. Check data that error reference value minus offset calculation data */
    HAL_ADC_SetPPBxThresholdEx(&g_adc, ADC_PPB_NUM1, 10, -10);
    HAL_ADC_EnablePPBxEventIntEx(&g_adc);
    HAL_ADC_SoftTrigSample(&g_adc, ADC_SOC_NUM1);  /* Software trigger ADC sampling */

    BASE_FUNC_DELAY_S(1);   /* After the delay is 1s, the PPB is enabled for re-sampling */
    DBG_PRINTF("\r\nPPB1 Setting offset: -2000; errorRef: 40; threshold: < -10 or > 10\r\n");
    HAL_ADC_SetPPBxErrorRefEx(&g_adc, ADC_PPB_NUM1, 40); /* Set 40 as the error reference value */
    HAL_ADC_SoftTrigSample(&g_adc, ADC_SOC_NUM1);  /* Software trigger ADC sampling */
}