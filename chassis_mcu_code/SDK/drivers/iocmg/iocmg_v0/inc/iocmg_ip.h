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
 * @file    iocmg_ip.h
 * @author  MCU Driver Team
 * @brief   IOCMG module driver
 * @details This file provides IOConfig register mapping structure.
 */

/* Macro definitions */
#ifndef McuMagicTag_IOCMG_IP_H
#define McuMagicTag_IOCMG_IP_H

/* Includes ------------------------------------------------------------------ */
#include "baseinc.h"
#include "ioconfig.h"
#include "iomap.h"
/* Macro definitions ---------------------------------------------------------*/
#ifdef IOCMG_PARAM_CHECK
    #define IOCMG_ASSERT_PARAM         BASE_FUNC_ASSERT_PARAM
    #define IOCMG_PARAM_CHECK_NO_RET   BASE_FUNC_PARAMCHECK_NO_RET
    #define IOCMG_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
    #define IOCMG_ASSERT_PARAM(para)                ((void)0U)
    #define IOCMG_PARAM_CHECK_NO_RET(para)          ((void)0U)
    #define IOCMG_PARAM_CHECK_WITH_RET(param, ret)  ((void)0U)
#endif
/**
  * @addtogroup IOCMG
  * @{
  */

/**
  * @defgroup IOCMG_IP
  * @{
  */
#define IOCMG_BASE_ADDR_MASK            0xFFFF0000
#define IOCMG_FUNC_NUM_MASK             0x0000000F
#define IOCMG_REG_VALUE_MASK            0x0000FFFF
/**
  * @defgroup IOCMG_Param_Def IOCMG Parameters Definition
  * @brief Description of IOCMG configuration parameters.
  * @{
  */
typedef enum {
    FUNC_MODE_0 = 0u,
    FUNC_MODE_1,
    FUNC_MODE_2,
    FUNC_MODE_3,
    FUNC_MODE_4,
    FUNC_MODE_5,
    FUNC_MODE_6,
    FUNC_MODE_7,
    FUNC_MODE_8,
    FUNC_MODE_9,
    FUNC_MODE_10,
    FUNC_MODE_11,
    FUNC_MODE_12,
    FUNC_MODE_13,
    FUNC_MODE_14,
    FUNC_MODE_15,
    FUNC_MODE_MAX
} IOCMG_FuncMode;

typedef enum {
    SCHMIDT_DISABLE = 0u,
    SCHMIDT_ENABLE
} IOCMG_SchmidtMode;

typedef enum {
    PULL_NONE = 0u,
    PULL_DOWN,
    PULL_UP,
    PULL_BOTH,
    PULL_MODE_MAX
} IOCMG_PullMode;

typedef enum {
    LEVEL_SHIFT_RATE_FAST = 0u,
    LEVEL_SHIFT_RATE_SLOW,
    LEVEL_SHIFT_RATE_MAX
} IOCMG_LevelShiftRate;

typedef enum {
    DRIVER_RATE_4 = 0u,
    DRIVER_RATE_3,
    DRIVER_RATE_2,
    DRIVER_RATE_1,
    DRIVER_RATE_MAX
} IOCMG_DriveRate;

typedef enum {
    OSC_CLK_DRIVER_RATE_1 = 0u,
    OSC_CLK_DRIVER_RATE_2,
    OSC_CLK_DRIVER_RATE_3,
    OSC_CLK_DRIVER_RATE_4,
    OSC_CLK_DRIVER_RATE_MAX
} IOCMG_OscClkDriveRate;

typedef enum {
    IOCMG_STATUS_OK,
    IOCMG_BASE_ADDR_ERROR,
    IOCMG_REG_ADDR_ERROR,
    IOCMG_PIN_FUNC_ERROR,
    IOCMG_PARAM_ERROR
} IOCMG_Status;
/**
  * @}
  */

/**
  * @brief Set iocmg reg value.
  * @param iocmgRegx Value of @ref IOCMG_REG.
  * @param regValue value of @ref IOCMG_REG.
  * @retval None.
  */
static inline void DCL_IOCMG_SetRegValue(IOCMG_REG *iocmgRegx, unsigned int regValue)
{
    IOCMG_ASSERT_PARAM(IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegx & IOCMG_BASE_ADDR_MASK)));
    iocmgRegx->reg = regValue;
}

/**
  * @brief Get iocmg reg value.
  * @param iocmgRegx Value of @ref IOCMG_REG.
  * @retval iocmgRegx register value.
  */
static inline unsigned int DCL_IOCMG_GetRegValue(IOCMG_REG *iocmgRegx)
{
    IOCMG_ASSERT_PARAM(IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegx & IOCMG_BASE_ADDR_MASK)));
    return iocmgRegx->reg;
}

/**
  * @brief Set iocmg function number mode.
  * @param iocmgRegx Value of @ref IOCMG_REG.
  * @param funcnum value of @ref IOCMG_FuncMode.
  * @retval None.
  */
static inline void DCL_IOCMG_SetFuncNum(IOCMG_REG *iocmgRegx, IOCMG_FuncMode funcnum)
{
    IOCMG_ASSERT_PARAM(IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegx & IOCMG_BASE_ADDR_MASK)));
    IOCMG_PARAM_CHECK_NO_RET(funcnum < FUNC_MODE_MAX && funcnum >= FUNC_MODE_0);
    iocmgRegx->BIT.func = funcnum;
}

/**
  * @brief Get iocmg function number mode.
  * @param iocmgRegx Value of @ref IOCMG_REG.
  * @retval Value of @ref IOCMG_FuncMode.
  */
static inline IOCMG_FuncMode DCL_IOCMG_GetFuncMode(IOCMG_REG *iocmgRegx)
{
    IOCMG_ASSERT_PARAM(IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegx & IOCMG_BASE_ADDR_MASK)));
    return iocmgRegx->BIT.func;
}

/**
  * @brief Set iocmg drive rate mode.
  * @param iocmgRegx Value of @ref IOCMG_REG.
  * @param driveRate value of @ref IOCMG_DriveRate.
  * @retval None.
  */
static inline void DCL_IOCMG_SetDriveRate(IOCMG_REG *iocmgRegx, IOCMG_DriveRate driveRate)
{
    IOCMG_ASSERT_PARAM(IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegx & IOCMG_BASE_ADDR_MASK)));
    IOCMG_PARAM_CHECK_NO_RET(driveRate < DRIVER_RATE_MAX && driveRate >= DRIVER_RATE_4);
    iocmgRegx->BIT.ds = driveRate;
}

/**
  * @brief Get iocmg drive rate mode.
  * @param iocmgRegx Value of @ref IOCMG_REG.
  * @retval Value of @ref IOCMG_DriveRate.
  */
static inline IOCMG_DriveRate DCL_IOCMG_GetDriveRate(IOCMG_REG *iocmgRegx)
{
    IOCMG_ASSERT_PARAM(IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegx & IOCMG_BASE_ADDR_MASK)));
    return iocmgRegx->BIT.ds;
}

/**
  * @brief Set iocmg pull up or down mode.
  * @param iocmgRegx Value of @ref IOCMG_REG.
  * @param pullMode value of @ref IOCMG_PullMode.
  * @retval None.
  */
static inline void DCL_IOCMG_SetPullMode(IOCMG_REG *iocmgRegx, IOCMG_PullMode pullMode)
{
    IOCMG_ASSERT_PARAM(IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegx & IOCMG_BASE_ADDR_MASK)));
    IOCMG_PARAM_CHECK_NO_RET(pullMode < PULL_MODE_MAX && pullMode >= PULL_NONE);
    iocmgRegx->BIT.pu = (pullMode & 0x02) >> 1; /* 10b: pull up mode */
    iocmgRegx->BIT.pd = pullMode & 0x01; /* 01b: pull down mode */
}

/**
  * @brief Get iocmg pull up or down mode.
  * @param iocmgRegx Value of @ref IOCMG_REG.
  * @retval pullMode value of @ref IOCMG_PullMode.
  */
static inline IOCMG_PullMode DCL_IOCMG_GetPullMode(IOCMG_REG *iocmgRegx)
{
    IOCMG_ASSERT_PARAM(IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegx & IOCMG_BASE_ADDR_MASK)));
    unsigned int pullUpMode = iocmgRegx->BIT.pu;
    unsigned int pullDownMode = iocmgRegx->BIT.pd;
    return (pullUpMode << 1) | pullDownMode; /* 1: shift for up mode bit */
}

/**
  * @brief Set iocmg level shift rate mode.
  * @param iocmgRegx Value of @ref IOCMG_REG.
  * @param levelShiftRate value of @ref IOCMG_LevelShiftRate.
  * @retval None.
  */
static inline void DCL_IOCMG_SetLevelShiftRate(IOCMG_REG *iocmgRegx, IOCMG_LevelShiftRate levelShiftRate)
{
    IOCMG_ASSERT_PARAM(IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegx & IOCMG_BASE_ADDR_MASK)));
    IOCMG_PARAM_CHECK_NO_RET(levelShiftRate < LEVEL_SHIFT_RATE_MAX && levelShiftRate >= LEVEL_SHIFT_RATE_FAST);
    iocmgRegx->BIT.sr = levelShiftRate;
}

/**
  * @brief Get iocmg level shift rate mode.
  * @param iocmgRegx Value of @ref IOCMG_REG.
  * @retval levelShiftRate value of @ref IOCMG_LevelShiftRate.
  */
static inline IOCMG_LevelShiftRate DCL_IOCMG_GetLevelShiftRate(IOCMG_REG *iocmgRegx)
{
    IOCMG_ASSERT_PARAM(IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegx & IOCMG_BASE_ADDR_MASK)));
    return iocmgRegx->BIT.sr;
}

/**
  * @brief Set iocmg schmidt enable mode.
  * @param iocmgRegx Value of @ref IOCMG_REG.
  * @param schmidtMode value of @ref IOCMG_SchmidtMode.
  * @retval None.
  */
static inline void DCL_IOCMG_SetSchmidtMode(IOCMG_REG *iocmgRegx, IOCMG_SchmidtMode schmidtMode)
{
    IOCMG_ASSERT_PARAM(IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegx & IOCMG_BASE_ADDR_MASK)));
    IOCMG_PARAM_CHECK_NO_RET(schmidtMode <= SCHMIDT_ENABLE && schmidtMode >= SCHMIDT_DISABLE);
    iocmgRegx->BIT.se = schmidtMode;
}

/**
  * @brief Get iocmg schmidt enable mode.
  * @param iocmgRegx Value of @ref IOCMG_REG.
  * @retval schmidtMode value of @ref IOCMG_SchmidtMode.
  */
static inline IOCMG_SchmidtMode DCL_IOCMG_GetSchmidtMode(IOCMG_REG *iocmgRegx)
{
    IOCMG_ASSERT_PARAM(IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegx & IOCMG_BASE_ADDR_MASK)));
    return iocmgRegx->BIT.se;
}

/**
  * @brief set iocmg OSC clock output mode.
  * @param mode function enable or not.
  * @retval None.
  */
static inline void DCL_IOCMG_SetOscClkOutputMode(bool mode)
{
    IOCMG_ASSERT_PARAM(mode == BASE_CFG_ENABLE || mode == BASE_CFG_DISABLE);
    IOCMG->iocmg_6.BIT.osc_e = mode;
}

/**
  * @brief Get iocmg OSC clock output mode.
  * @param None
  * @retval None.
  */
static inline bool DCL_IOCMG_GetOscClkOutputMode(void)
{
    return IOCMG->iocmg_6.BIT.osc_e;
}

/**
  * @brief set iocmg OSC clock output mode.
  * @param mode function enable or not.
  * @retval None.
  */
static inline void DCL_IOCMG_SetOscClkFuncMode(bool mode)
{
    IOCMG_ASSERT_PARAM(mode == BASE_CFG_ENABLE || mode == BASE_CFG_DISABLE);
    IOCMG->iocmg_6.BIT.osc_ie = mode;
}

/**
  * @brief Get iocmg OSC clock output enable mode.
  * @param None.
  * @retval None.
  */
static inline bool DCL_IOCMG_GetOscClkFuncMode(void)
{
    return IOCMG->iocmg_6.BIT.osc_ie;
}

/**
  * @brief Set iocmg OSC drive rate mode.
  * @param oscClkDriveRate value of @ref IOCMG_DriveRate.
  * @retval None.
  */
static inline void DCL_IOCMG_SetOscClkDriveRate(IOCMG_OscClkDriveRate oscClkDriveRate)
{
    IOCMG_PARAM_CHECK_NO_RET(oscClkDriveRate < OSC_CLK_DRIVER_RATE_MAX && oscClkDriveRate >= OSC_CLK_DRIVER_RATE_1);
    IOCMG->iocmg_6.BIT.osc_ds = oscClkDriveRate;
}

/**
  * @brief Get iocmg OSC drive rate mode.
  * @param None.
  * @retval oscClkDriveRate value of @ref IOCMG_DriveRate.
  */
static inline IOCMG_DriveRate DCL_IOCMG_GetOscClkDriveRate(void)
{
    return IOCMG->iocmg_6.BIT.osc_ds;
}
/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_IOCMG_IP_H */