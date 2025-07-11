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
  * @file      gpt.c
  * @author    MCU Driver Team
  * @brief     GPT module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the GPT.
  *                + Initialization function of GPT
  *                + Clock Configuration of GPT
  *                + Get GPT State and Apply GPT
  */

#include "gpt.h"

static unsigned int GPT_GetKeepState(GPT_Handle *handle);

/**
 * @brief   Get Keep state
 * @param   handle   GPT Handle
 * @retval  keep   0: Outputs a fixed number of square waves
 *                 1: Output continuous square wave
 */
static unsigned int GPT_GetKeepState(GPT_Handle *handle)
{
    GPT_PWM0_CFG_REG pwm0Cfg;
    pwm0Cfg.reg = handle->baseAddress->GPT_PWM0_CFG.reg;
    return pwm0Cfg.BIT.rg_pwm0_keep;
}

/**
 * @brief   Init the GPT.
 * @param   handle   GPT Handle.
 * @retval  BASE_STATUS_OK   Success
 * @retval  BASE_STATUS_ERROR Paramter check fail
 */
BASE_StatusType HAL_GPT_Init(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));

    HAL_GPT_Stop(handle);
    if (HAL_GPT_Config(handle) == BASE_STATUS_ERROR) {
        return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
 * @brief   Start GPT
 * @param   handle   GPT Handle.
 * @retval  None
 */
void HAL_GPT_Start(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    /* Enables the GPT to output PWM waves according to the configuration. */
    GPT_EN_REG gptEn;
    gptEn.BIT.rg_gpt_en = BASE_CFG_SET;
    handle->baseAddress->GPT_EN.reg = gptEn.reg;
}

/**
 * @brief   Stop GPT
 * @param   handle   GPT Handle.
 * @retval  None
 */
void HAL_GPT_Stop(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
     /* Disable the GPT to output PWM waves. */
    GPT_EN_REG gptEn;
    gptEn.BIT.rg_gpt_en = BASE_CFG_UNSET;
    handle->baseAddress->GPT_EN.reg = gptEn.reg;
}

/**
 * @brief   GPT Configuration
 * @param   handle   GPT Handle.
 * @retval  BASE_STATUS_OK
 * @retval  BASE_STATUS_ERROR
 */
BASE_StatusType HAL_GPT_Config(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    GPT_PARAM_CHECK_WITH_RET(IsGptPeriod(handle->period), BASE_STATUS_ERROR);
    GPT_PARAM_CHECK_WITH_RET(IsGptDiv(handle->clockDiv), BASE_STATUS_ERROR);
    GPT_PARAM_CHECK_WITH_RET(IsGptRefDot(handle->refA0.refdot), BASE_STATUS_ERROR);
    GPT_PARAM_CHECK_WITH_RET(IsGptRefDot(handle->refB0.refdot), BASE_STATUS_ERROR);
    GPT_PARAM_CHECK_WITH_RET(handle->refA0.refdot <= handle->refB0.refdot, BASE_STATUS_ERROR);
    GPT_PARAM_CHECK_WITH_RET(handle->refB0.refdot <= handle->period, BASE_STATUS_ERROR);
    GPT_PARAM_CHECK_WITH_RET(IsGptAction(handle->refA0.refAction), BASE_STATUS_ERROR);
    GPT_PARAM_CHECK_WITH_RET(IsGptAction(handle->refB0.refAction), BASE_STATUS_ERROR);
    GPT_PARAM_CHECK_WITH_RET(IsGptPwmNum(handle->pwmNum), BASE_STATUS_ERROR);

    GPT_RegStruct *gptReg;
    gptReg = handle->baseAddress;
    /* Configure whether to enable cache loading. */
    gptReg->GPT_BUF_LOAD_EN.BIT.rg_buf_load_en = handle->bufLoad;
    
    /* Configuring the Cycle and Frequency Divider */
    gptReg->GPT_TC_DIV.reg = handle->clockDiv;
    gptReg->GPT_TC_PRD.reg = handle->period;
    /* Set the count reference point and the corresponding reference action. */
    gptReg->GPT_TC_REFA0.reg = handle->refA0.refdot;
    gptReg->GPT_PG_ACT0.BIT.rg_pg_act0_refa0 = handle->refA0.refAction;
    gptReg->GPT_TC_REFB0.reg = handle->refB0.refdot;
    gptReg->GPT_PG_ACT0.BIT.rg_pg_act0_refb0 = handle->refB0.refAction;
    
    /* Sets the PWM output mode: outputs infinite PWM waves and outputs fixed number PWM. */
    gptReg->GPT_PWM0_CFG.BIT.rg_pwm0_keep = handle->pwmKeep;
    /* Sets the number of output PWM wavelengths. This parameter is valid only when outputs fixed number PWM. */
    gptReg->GPT_PWM0_CFG.BIT.rg_pwm0_num = handle->pwmNum;
    
    /* Sets the GPT output completion interrupt and periodic interrupt. */
    gptReg->GPT_INT_EN.BIT.rg_prd_int_en = handle->handleEx.periodIntEnable;
    gptReg->GPT_INT_EN.BIT.rg_pwm0_int_en = handle->handleEx.outputFinIntEnable;

    /* ADC Trigger Sampling Configuration */
    gptReg->GPT_SOCDR_EN.BIT.rg_soc_pwm0_en = handle->triggleAdcOutFinish;
    gptReg->GPT_SOCDR_EN.BIT.rg_soc_prd_en = handle->triggleAdcPeriod;
    return BASE_STATUS_OK;
}

/**
 * @brief   Obtains GPT configuration parameters.
 * @param   handle   GPT Handle.
 * @retval  BASE_STATUS_OK    Success
 * @retval  BASE_STATUS_ERROR Fail
 */
BASE_StatusType HAL_GPT_GetConfig(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));

    GPT_RegStruct *gptReg = handle->baseAddress;
    /* Obtains the configuration parameters of the PWM wavelength. */
    handle->clockDiv        =   gptReg->GPT_TC_DIV.reg;
    handle->period          =   gptReg->GPT_TC_PRD.reg;
    handle->refA0.refdot    =   gptReg->GPT_TC_REFA0.reg;
    handle->refB0.refdot    =   gptReg->GPT_TC_REFB0.reg;
    handle->refA0.refAction =   gptReg->GPT_PG_ACT0.BIT.rg_pg_act0_refa0;
    handle->refB0.refAction =   gptReg->GPT_PG_ACT0.BIT.rg_pg_act0_refb0;
    /* Obtains the cache loading status. */
    handle->bufLoad         =   gptReg->GPT_BUF_LOAD_EN.BIT.rg_buf_load_en;

    /* Obtaining the Interrupt Status */
    handle->handleEx.periodIntEnable    =  gptReg->GPT_INT_EN.BIT.rg_prd_int_en;
    handle->handleEx.outputFinIntEnable =  gptReg->GPT_INT_EN.BIT.rg_pwm0_int_en;

    /* Obtains ADC configuration parameters. */
    handle->triggleAdcOutFinish         =  gptReg->GPT_SOCDR_EN.BIT.rg_soc_pwm0_en;
    handle->triggleAdcPeriod            =  gptReg->GPT_SOCDR_EN.BIT.rg_soc_prd_en;
    
    /* Obtains the PWM output mode. */
    GPT_PWM0_CFG_REG pwm0Cfg;
    pwm0Cfg.reg              =   gptReg->GPT_PWM0_CFG.reg;
    handle->pwmKeep          =   GPT_GetKeepState(handle);
    handle->pwmNum           =   pwm0Cfg.BIT.rg_pwm0_num;
    return BASE_STATUS_OK;
}

/**
 * @brief   Set GPT count reference value and action configuration.
 * @param   handle   GPT Handle.
 * @param   refer    Input Pointer to the reference, @ref GPT_ReferCfg
 * @retval  BASE_STATUS_OK    Success
 * @retval  BASE_STATUS_ERROR Parameter check fail
 */
BASE_StatusType HAL_GPT_SetReferCounterAndAction(GPT_Handle *handle, const GPT_ReferCfg *refer)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(refer != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));

    unsigned int period = handle->baseAddress->GPT_TC_PRD.reg;
    /* Verifying ref value parameters */
    if ((refer->refA0.refdot > period) || (refer->refB0.refdot > period)) {
        return BASE_STATUS_ERROR;
    }
    GPT_RegStruct *gptReg = handle->baseAddress;
    /* Set reference value parameters. */
    gptReg->GPT_TC_REFA0.reg = refer->refA0.refdot;
    gptReg->GPT_TC_REFB0.reg = refer->refB0.refdot;
    /* Set reference dot action */
    gptReg->GPT_PG_ACT0.BIT.rg_pg_act0_refa0 = refer->refA0.refAction;
    gptReg->GPT_PG_ACT0.BIT.rg_pg_act0_refb0 = refer->refB0.refAction;
    return BASE_STATUS_OK;
}

/**
 * @brief   Get GPT count reference value and action configuration
 * @param   handle   GPT Handle.
 * @param   refer    Pointer to the reference, @ref GPT_ReferCfg
 * @retval  None
 */
void HAL_GPT_GetReferCounterAndAction(GPT_Handle *handle, GPT_ReferCfg *refer)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(refer != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    GPT_RegStruct *gptReg = handle->baseAddress;
    /* Obtain the reference value of PWM. */
    refer->refA0.refdot = gptReg->GPT_TC_REFA0.reg;
    refer->refB0.refdot = gptReg->GPT_TC_REFB0.reg;
    /* The action of obtaining a reference value. */
    refer->refA0.refAction = gptReg->GPT_PG_ACT0.BIT.rg_pg_act0_refa0;
    refer->refB0.refAction = gptReg->GPT_PG_ACT0.BIT.rg_pg_act0_refb0;
}

/**
 * @brief   Set GPT counting period
 * @param   handle   GPT Handle.
 * @param   period   Counting period.
 * @retval  BASE_STATUS_OK   Success
 * @retval  BASE_STATUS_ERROR Parameter check fail
 */
BASE_StatusType HAL_GPT_SetCountPeriod(GPT_Handle *handle, unsigned int period)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    GPT_PARAM_CHECK_WITH_RET(IsGptPeriod(period), BASE_STATUS_ERROR);
    /* Sets the GPT counting period. The larger the value, the longer the period time. */
    GPT_TC_PRD_REG periodReg;
    periodReg.reg = handle->baseAddress->GPT_TC_PRD.reg;
    periodReg.BIT.rg_cnt_prd = period;
    handle->baseAddress->GPT_TC_PRD.reg = periodReg.reg;
    return BASE_STATUS_OK;
}

/**
 * @brief   Get GPT Period
 * @param   handle   GPT Handle.
 * @retval  unsigned int GPT Counting Period.
 */
unsigned int HAL_GPT_GetCountPeriod(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    GPT_TC_PRD_REG periodReg;
    /* return period value index */
    periodReg.reg = handle->baseAddress->GPT_TC_PRD.reg;
    return periodReg.BIT.rg_cnt_prd;
}

/**
 * @brief   Set GPT divider factor
 * @param   handle   GPT Handle.
 * @param   div      Input divider factor, Frequency division multiple equal configured
 *                   frequency division factor + 1
 * @retval  BASE_STATUS_OK   Success
 * @retval  BASE_STATUS_ERROR Parameter check fail
 */
BASE_StatusType HAL_GPT_SetDivFactor(GPT_Handle *handle, unsigned int div)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    GPT_PARAM_CHECK_WITH_RET((div <= GPT_DIV_FACTOR_MAX_VALUE), BASE_STATUS_ERROR);
    GPT_TC_DIV_REG divReg;

    /* Frequency division multiple = configured frequency division factor + 1 */
    divReg.reg = handle->baseAddress->GPT_TC_DIV.reg;
    divReg.BIT.rg_div_fac = div;
    handle->baseAddress->GPT_TC_DIV.reg = divReg.reg;
    return BASE_STATUS_OK;
}

/**
 * @brief   Get GPT Divison Factor
 * @param   handle   GPT Handle
 * @retval  divCnt   The current count value of the divider
 */
unsigned int HAL_GPT_GetDivFactor(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));

    GPT_TC_DIV_REG tcValue;
    /* Obtains the frequency division value of the counter. */
    tcValue.reg = handle->baseAddress->GPT_TC_DIV.reg;
    return tcValue.BIT.rg_div_fac;
}

/**
 * @brief   Set GPT Cache Load Enable/Disable for Cache-enabled Registers
 * @param   handle   GPT Handle
 * @param   bufferLoad   Cache load enable/disable, @ref GPT_SetOption
 * @retval  BASE_STATUS_OK    Success
 * @retval  BASE_STATUS_ERROR Fail
 */
BASE_StatusType HAL_GPT_SetBufferLoad(GPT_Handle *handle, GPT_SetOption bufferLoad)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    GPT_PARAM_CHECK_WITH_RET(IsGptSetOption(bufferLoad), BASE_STATUS_ERROR);

    GPT_BUF_LOAD_EN_REG bufLoadEn;
    /* Set buffer load of GPT */
    bufLoadEn.reg = handle->baseAddress->GPT_BUF_LOAD_EN.reg;
    bufLoadEn.BIT.rg_buf_load_en = bufferLoad;
    handle->baseAddress->GPT_BUF_LOAD_EN.reg = bufLoadEn.reg;
    return BASE_STATUS_OK;
}

/**
 * @brief   Get Buffer status
 * @param   handle   GPT Handle
 * @retval  loadStatus @ref GPT_LoadStatus
 */
unsigned int HAL_GPT_GetBufferLoadStatus(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));

    return handle->baseAddress->GPT_LOAD_STS.reg;
}


/**
 * @brief   Set GPT PWM output finish interrupt
 * @param   handle   GPT Handle
 * @param   outFinishInt  Out finish interrupt enable/disable @ref GPT_SetOption
 * @retval  BASE_STATUS_OK    Success
 * @retval  BASE_STATUS_ERROR Fail
 */
BASE_StatusType HAL_GPT_SetOutFinishInt(GPT_Handle *handle, GPT_SetOption outFinishInt)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    GPT_PARAM_CHECK_WITH_RET(IsGptSetOption(outFinishInt), BASE_STATUS_ERROR);
    /* Set output finish interrupt, 0x1: enabel, 0x0: disable. */
    GPT_INT_EN_REG intEn;
    intEn.reg = handle->baseAddress->GPT_INT_EN.reg;
    intEn.BIT.rg_pwm0_int_en = outFinishInt;
    handle->baseAddress->GPT_INT_EN.reg = intEn.reg;
    return BASE_STATUS_OK;
}

/**
 * @brief   Set GPT period interrupt enable
 * @param   handle   GPT Handle
 * @param   periodInt  Period interrupt enable/disable @ref GPT_SetOption
 * @retval  BASE_STATUS_OK    Success
 * @retval  BASE_STATUS_ERROR Fail
 */
BASE_StatusType HAL_GPT_SetPeriodInt(GPT_Handle *handle, GPT_SetOption periodInt)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    GPT_PARAM_CHECK_WITH_RET(IsGptSetOption(periodInt), BASE_STATUS_ERROR);
    /* Set period output finish interrupt, 0x1: enable, 0x0:disable. */
    GPT_INT_EN_REG intEn;
    intEn.reg = handle->baseAddress->GPT_INT_EN.reg;
    intEn.BIT.rg_prd_int_en = periodInt;
    handle->baseAddress->GPT_INT_EN.reg = intEn.reg;
    return BASE_STATUS_OK;
}

/**
 * @brief   Gpt pwm output finish interrupt service processing function.
 * @param   handle   GPT Handle
 * @retval  None
 */
void HAL_GPT_IrqOutFinishHandler(void *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_Handle *gptHandle = (GPT_Handle *)handle;
    GPT_ASSERT_PARAM(IsGPTInstance(gptHandle->baseAddress));
    /* Check interrupt whether the injection interrupt */
    /* period and finish interrupt */
    if (gptHandle->baseAddress->GPT_INT_FLAG.BIT.ro_pwm0_int_flag == BASE_CFG_ENABLE) {
        /* channel out put finish interrupt */
        gptHandle->baseAddress->GPT_INT_FLAG.BIT.rg_pwm0_int_clr = BASE_CFG_ENABLE;
        if (gptHandle->userCallBack.PWMOutPutFin != NULL) {
            gptHandle->userCallBack.PWMOutPutFin(gptHandle);
        }
    }
    return;
}

/**
 * @brief   Gpt period interrupt service processing function.
 * @param   handle   GPT Handle
 * @retval  None
 */
void HAL_GPT_IrqPeriodHandler(void *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_Handle *gptHandle = (GPT_Handle *)handle;
    GPT_ASSERT_PARAM(IsGPTInstance(gptHandle->baseAddress));

    if (gptHandle->baseAddress->GPT_INT_FLAG.BIT.ro_prd_int_flag == BASE_CFG_ENABLE) {
        /* period interrupt */
        gptHandle->baseAddress->GPT_INT_FLAG.BIT.rg_prd_int_clr = BASE_CFG_ENABLE;
        if (gptHandle->userCallBack.PWMPeriod != NULL) {
            gptHandle->userCallBack.PWMPeriod(gptHandle);
        }
    }
    return;
}

/**
  * @brief User callback function registration interface.
  * @param gptHandle GPT handle.
  * @param typeID Id of callback function type. @ref GPT_CallBackFunType
  * @param pCallback pointer of the specified callbcak function. @ref GPT_CallBackFunc
  * @retval  BASE_STATUS_OK    Success
  * @retval  BASE_STATUS_ERROR fail
  */
BASE_StatusType HAL_GPT_RegisterCallBack(GPT_Handle *gptHandle, GPT_CallBackFunType typeID,
                                         GPT_CallBackFunc pCallback)
{
    GPT_ASSERT_PARAM(gptHandle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(gptHandle->baseAddress));
    /* Registering interrupt callback function according to different types */
    switch (typeID) {
        case GPT_INT_PERIOD:
            /* Registers function for handling period output finish interrupt. */
            gptHandle->userCallBack.PWMPeriod = pCallback;
            break;
        case GPT_INT_PWM_OUTPUT_FIN:
            /* Registers function for handling output finish interrupt. */
            gptHandle->userCallBack.PWMOutPutFin = pCallback;
            break;
        default:
            return BASE_STATUS_ERROR;  /* Failed to register the callback function. */
    }
    return BASE_STATUS_OK;
}