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
  * @file    adc.h
  * @author  MCU Driver Team
  * @brief   ADC module driver
  * @details This file provides functions declaration of the ADC,
  *           + ADC initialization function.
  *           + Start ADC sample and conversion.
  *           + Start ADC sample and conversion with interrupt.
  *           + Start ADC sample and conversion with DMA.
  *           + Start ADC sample and conversion synchronously.
  *           + Query the ADC conversion result.
  *           + Single channel and multichannel software trigger functions.
  *           + Interrupt callback function and user registration function.
  *          This file also provides the definition of the ADC handle structure.
  */

/* Macro definitions */
#ifndef McuMagicTag_ADC_H
#define McuMagicTag_ADC_H

#include "adc_ip.h"
#include "dma.h"
#include "dac.h"
#include "interrupt.h"

/**
  * @defgroup ADC ADC
  * @brief ADC module.
  * @{
  */

/**
  * @defgroup ADC_Common ADC Common
  * @brief ADC common external module.
  * @{
  */

/**
  * @defgroup ADC_Handle_Definition ADC Handle Definition
  * @{
  */

/**
  * @brief The definition of the ADC handle structure.
  */
typedef struct _ADC_Handle {
    ADC_RegStruct           *baseAddress;       /**< ADC registers base address */
    ADC_PriorityMode        socPriority;        /**< ADC clock divider */
    DMA_Handle              *dmaHandle;         /**< ADC_DMA control */
    unsigned int            adcDmaChn;          /**< ADC_DMA channel */
    ADC_OverState           overState;          /**< ADC overflow state */
    struct {
        unsigned short      finishMode;         /**< sample finish mode, defined in ADC_SOCFinishMode */
    } ADC_SOCxParam[SOC_MAX_NUM];
    struct {
        unsigned short      socxFinish;         /**< After each SOC is completed, the corresponding bit is set as 1 */
    } ADC_IntxParam[INT_MAX_NUM];
    ADC_UserCallBack        userCallBack;       /**< ADC User Callback Function */
    ADC_ExtendHandle        handleEx;           /**< ADC extend handle */
} ADC_Handle;

/**
  * @brief The definition of the ADC callback function.
  */
typedef void (* ADC_CallbackType)(void *handle);

/**
  * @}
  */

/**
  * @defgroup ADC_API_Declaration ADC HAL API
  * @{
  */
BASE_StatusType HAL_ADC_Init(ADC_Handle *adcHandle);
BASE_StatusType HAL_ADC_Deinit(ADC_Handle *adcHandle);
BASE_StatusType HAL_ADC_ConfigureSoc(ADC_Handle *adcHandle, ADC_SOCNumber soc, SOC_Param *socParam);
BASE_StatusType HAL_ADC_StartDma(ADC_Handle *adcHandle, unsigned int startSoc,
                                 unsigned int endSoc, unsigned int *saveData);
BASE_StatusType HAL_ADC_StartIt(ADC_Handle *adcHandle);
BASE_StatusType HAL_ADC_SoftTrigMultiSample(ADC_Handle *adcHandle, ADC_SoftMultiTrig syncTrig);
BASE_StatusType HAL_ADC_SoftTrigSample(ADC_Handle *adcHandle, unsigned int soc);
unsigned int HAL_ADC_GetConvResult(ADC_Handle *adcHandle, unsigned int soc);
BASE_StatusType HAL_ADC_CheckSocFinish(ADC_Handle *adcHandle, unsigned int soc);
void HAL_ADC_RegisterCallBack(ADC_Handle *adcHandle, ADC_CallbackFunType typeID, ADC_CallbackType pCallback);
void HAL_ADC_IrqHandlerInt0(void *handle);
void HAL_ADC_IrqHandlerInt1(void *handle);
void HAL_ADC_IrqHandlerInt2(void *handle);
void HAL_ADC_IrqHandlerInt3(void *handle);
void HAL_ADC_IrqHandlerOver(void *handle);
void HAL_ADC_IrqHandlerAllEvent(void *handle);
#if defined (CHIP_3065HRPIRZ) || defined (CHIP_3065HRPICZ) || defined (CHIP_3061HRPIKZ) || \
    defined (AU302PDF51) || defined (AU302NDF51) || defined (AU301LDF51)
BASE_StatusType HAL_ADC_InitForVdda(ADC_RegStruct *adcx, ADC_SOCNumber soc, DAC_RegStruct *dacx, bool useDac);
float HAL_ADC_GetVddaByDac(ADC_RegStruct *adcx, ADC_SOCNumber soc, DAC_RegStruct *dacx, bool useDac);
unsigned int HAL_ADC_GetTransResultByVdda(ADC_RegStruct *adcx, ADC_SOCNumber soc, float vdda);
void HAL_ADC_IrqHandlerInt1(void *handle);
void HAL_ADC_IrqHandlerInt2(void *handle);
void HAL_ADC_IrqHandlerInt3(void *handle);
void HAL_ADC_IrqHandlerInt4(void *handle);
#endif
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif