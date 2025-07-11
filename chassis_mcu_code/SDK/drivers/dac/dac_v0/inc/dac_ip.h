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
  * @file    dac_ip.h
  * @author  MCU Driver Team
  * @brief   DAC module driver.
  *          This file provides DCL functions to manage DAC and Definitions of specific parameters.
  *           + Definition of DAC configuration parameters.
  *           + DAC register mapping structure.
  *           + Parameters check functions.
  *           + Direct configuration layer interface.
  */
#ifndef McuMagicTag_DAC_IP_H
#define McuMagicTag_DAC_IP_H

#include "baseinc.h"

#ifdef DAC_PARAM_CHECK
#define DAC_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define DAC_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define DAC_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define DAC_ASSERT_PARAM(para) ((void)0U)
#define DAC_PARAM_CHECK_NO_RET(para) ((void)0U)
#define DAC_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

#define DAC_MAX_OUT_VALUE 0xFF
#define DAC_MAX_SINE_INTERVAL_VALUE 0xFF
/**
  * @addtogroup DAC
  * @{
  */

/**
  * @defgroup DAC_IP DAC_IP
  * @brief DAC_IP: dac_v0.
  * @{
  */

/**
  * @defgroup DAC_REG_Definition DAC Register Structure.
  * @brief DAC Register Structure Definition.
  * @{
  */
typedef struct {
    bool             sineMode;      /* DAC sine mode enable. */
    unsigned short   sineCountValue;       /* DAC output number in sine mode. */
} DAC_ExtendHandle;

/**
  * @brief DAC control.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dac_en        : 1;   /**< DAC Enable. */
        unsigned int reserved_0    : 1;
        unsigned int dac_test_en   : 1;   /**< DAC sine wave mode enable. */
        unsigned int reserved_1    : 5;
        unsigned int dac_test_num  : 8;   /**< DAC sine wave interval count value. */
        unsigned int reserved_2    : 16;
    } BIT;
} volatile DAC_CTRL_REG;

/**
  * @brief DAC configuration value.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dac_value : 8;    /**< DAC digital signal, corresponding to the voltage */
        unsigned int reserved_0 : 24;
    } BIT;
} volatile DAC_VALUE_REG;

/**
  * @brief DAC registers definition structure.
  */
typedef struct _DAC_RegStruct {
    DAC_CTRL_REG        DAC_CTRL;      /**< DAC control register. Offset address: 0x00000000U */
    DAC_VALUE_REG       DAC_VALUE;     /**< DAC configuration value register. Offset address: 0x00000004U */
} volatile DAC_RegStruct;

/* Parameter Check -----------------------------------------------------------*/

/**
  * @brief Verify count value of the DAC sine wave interval.
  * @param intervalNum interval value.
  * @retval true
  * @retval false
  */
static inline bool IsDacSineIntervalNum(unsigned short intervalNum)
{
    return (intervalNum <= DAC_MAX_SINE_INTERVAL_VALUE);
}

/**
  * @brief Set DAC test value
  * @param dacx: DAC register base address.
  * @param value: DAC test value.
  * @retval None.
  */
static inline void DCL_DAC_SetTstValue(DAC_RegStruct *dacx, unsigned short value)
{
    DAC_ASSERT_PARAM(IsDACInstance(dacx));
    DAC_PARAM_CHECK_NO_RET(IsDacSineIntervalNum(value));
    DAC_CTRL_REG dacCtrlReg;      /* Set sine interval value. */
    dacCtrlReg.reg = dacx->DAC_CTRL.reg;
    dacCtrlReg.BIT.dac_test_num = value;
    dacx->DAC_CTRL.reg = dacCtrlReg.reg;
}

/**
  * @brief Enable DAC dynamic test
  * @param dacx: DAC register base address.
  * @retval None.
  */
static inline void DCL_DAC_EnableDynamicTst(DAC_RegStruct *dacx)
{
    DAC_ASSERT_PARAM(IsDACInstance(dacx));
    dacx->DAC_CTRL.BIT.dac_test_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable DAC dynamic test
  * @param dacx: DAC register base address.
  * @retval None.
  */
static inline void DCL_DAC_DisableDynamicTst(DAC_RegStruct *dacx)
{
    DAC_ASSERT_PARAM(IsDACInstance(dacx));
    dacx->DAC_CTRL.BIT.dac_test_en = BASE_CFG_DISABLE;
}

/**
  * @brief Enable DAC
  * @param dacx: DAC register base address.
  * @retval None.
  */
static inline void DCL_DAC_Enable(DAC_RegStruct *dacx)
{
    DAC_ASSERT_PARAM(IsDACInstance(dacx));
    dacx->DAC_CTRL.BIT.dac_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable DAC
  * @param dacx: DAC register base address.
  * @retval None.
  */
static inline void DCL_DAC_Disable(DAC_RegStruct *dacx)
{
    DAC_ASSERT_PARAM(IsDACInstance(dacx));
    dacx->DAC_CTRL.BIT.dac_en = BASE_CFG_DISABLE;
}

/**
  * @brief Set DAC value
  * @param dacx: DAC register base address.
  * @param value: DAC value.
  */
static inline void DCL_DAC_SetValue(DAC_RegStruct *dacx, unsigned int value)
{
    DAC_ASSERT_PARAM(IsDACInstance(dacx));
    DAC_PARAM_CHECK_NO_RET(value <= DAC_MAX_OUT_VALUE);
    dacx->DAC_VALUE.BIT.dac_value = value;
}

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
