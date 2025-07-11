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
  * @file      iocmg.c
  * @author    MCU Driver Team
  * @brief     Provides functions about iocmg reg init and config.
  */

/* Includes ---------------------------------------------------------------------- */
#include "iocmg.h"
/* param definition -------------------------------------------------------------- */
/* Function declaration----------------------------------------------------------- */
static IOCMG_REG* IOCMG_GetRegAddr(unsigned int pinTypedef);

IOCMG_Status HAL_IOCMG_Init(IOCMG_Handle* handle);
IOCMG_Status HAL_IOCMG_SetPinAltFuncMode(unsigned int pinTypedef);
IOCMG_Status HAL_IOCMG_SetPinPullMode(unsigned int pinTypedef, IOCMG_PullMode pullMode);
IOCMG_Status HAL_IOCMG_SetPinSchmidtMode(unsigned int pinTypedef, IOCMG_SchmidtMode schmidtMode);
IOCMG_Status HAL_IOCMG_SetPinLevelShiftRate(unsigned int pinTypedef, IOCMG_LevelShiftRate levelShiftRate);
IOCMG_Status HAL_IOCMG_SetPinDriveRate(unsigned int pinTypedef, IOCMG_DriveRate driveRate);
IOCMG_Status HAL_IOCMG_SetOscClkOutputMode(bool mode);
IOCMG_Status HAL_IOCMG_SetOscClkFuncMode(bool mode);
IOCMG_Status HAL_IOCMG_SetOscClkDriveRate(IOCMG_OscClkDriveRate oscClkDriveRate);

IOCMG_FuncMode HAL_IOCMG_GetPinAltFuncMode(unsigned int pinTypedef);
IOCMG_PullMode HAL_IOCMG_GetPinPullMode(unsigned int pinTypedef);
IOCMG_SchmidtMode HAL_IOCMG_GetPinSchmidtMode(unsigned int pinTypedef);
IOCMG_LevelShiftRate HAL_IOCMG_GetPinLevelShiftRate(unsigned int pinTypedef);
IOCMG_DriveRate HAL_IOCMG_GetPinDriveRate(unsigned int pinTypedef);
bool HAL_IOCMG_GetOscClkOutputMode(void);
bool HAL_IOCMG_GetOscClkFuncMode(void);
IOCMG_OscClkDriveRate HAL_IOCMG_GetOscClkDriveRate(void);
/* Function definiton----------------------------------------------------------- */
/**
  * @brief Get pins iocmg reg address
  * @param pinTypedef the pin type defined in iomap.h
  * @retval IOCMG_REG iocmg reg address.
  */
static IOCMG_REG* IOCMG_GetRegAddr(unsigned int pinTypedef)
{
    /* decode pin's iocmg reg offset address in base address, and conver value to point address */
    unsigned int iocmgBaseAddrValue = (uintptr_t)IOCMG_BASE;
    unsigned int iocmgRegOffsetAddrValue = (pinTypedef >> 16) & 0x00000FFF; /* 16 : shift 16 bit */
    IOCMG_REG* iocmgRegxAddr = (IOCMG_REG*)(void*)(iocmgBaseAddrValue + iocmgRegOffsetAddrValue);
    if (!IsIOCMGInstance((void *)((uintptr_t)(void *)iocmgRegxAddr & IOCMG_BASE_ADDR_MASK))) {
        return NULL;
    }
    return iocmgRegxAddr;
}

/**
  * @brief Initial IOCMG reg by pin number and function mode.
  * @param handle IOCMG_Handle.
  * @retval status @ref IOCMG_Status.
  */
IOCMG_Status HAL_IOCMG_Init(IOCMG_Handle* handle)
{
    IOCMG_ASSERT_PARAM(handle != NULL);
    IOCMG_REG* iocmgRegx = IOCMG_GetRegAddr(handle->pinTypedef);
    IOCMG_REG regValue = {0};
    regValue.BIT.func = (handle->pinTypedef & IOCMG_FUNC_NUM_MASK);
    regValue.BIT.ds = handle->driveRate;
    regValue.BIT.pd = handle->pullMode & 0x01; /* bit0 : pd  */
    regValue.BIT.pu = handle->pullMode >> 1; /* bit1 : pu  */
    regValue.BIT.se = handle->schmidtMode;
    regValue.BIT.sr = handle->levelShiftRate;
    DCL_IOCMG_SetRegValue(iocmgRegx, regValue.reg);
    return IOCMG_STATUS_OK;
}

/**
  * @brief Set pins as function mode
  * @param pinTypedef the pin type defined in iomap.h
  * @retval IOCMG_Status @ref IOCMG_Status.
  */
IOCMG_Status HAL_IOCMG_SetPinAltFuncMode(unsigned int pinTypedef)
{
    /* get iocmg reg address */
    IOCMG_REG* iocmgRegx = IOCMG_GetRegAddr(pinTypedef);
    /* get iocmg reg default value */
    unsigned int regValue = pinTypedef & IOCMG_REG_VALUE_MASK;
    DCL_IOCMG_SetRegValue(iocmgRegx, regValue);
    return IOCMG_STATUS_OK;
}

/**
  * @brief Get pins func number
  * @param pinTypedef the pin type defined in iomap.h
  * @retval pin func number @ref IOCMG_FuncMode.
  */
IOCMG_FuncMode HAL_IOCMG_GetPinAltFuncMode(unsigned int pinTypedef)
{
    /* get iocmg reg address */
    IOCMG_REG* iocmgRegx = IOCMG_GetRegAddr(pinTypedef);
    return DCL_IOCMG_GetFuncMode(iocmgRegx);
}

/**
  * @brief Set pins pull mode
  * @param pinTypedef the pin type defined in iomap.h
  * @param pullMode function define as @ref IOCMG_PullMode
  * @retval IOCMG_Status @ref IOCMG_Status.
  */
IOCMG_Status HAL_IOCMG_SetPinPullMode(unsigned int pinTypedef, IOCMG_PullMode pullMode)
{
    IOCMG_PARAM_CHECK_WITH_RET(pullMode < PULL_MODE_MAX && pullMode >= PULL_NONE, IOCMG_PARAM_ERROR);
    /* get iocmg reg address */
    IOCMG_REG* iocmgRegx = IOCMG_GetRegAddr(pinTypedef);
    DCL_IOCMG_SetPullMode(iocmgRegx, pullMode);
    return IOCMG_STATUS_OK;
}

/**
  * @brief Get pins pull mode
  * @param pinTypedef the pin type defined in iomap.h
  * @retval IOCMG_Status @ref IOCMG_Status.
  */
IOCMG_PullMode HAL_IOCMG_GetPinPullMode(unsigned int pinTypedef)
{
    /* get iocmg reg address */
    IOCMG_REG* iocmgRegx = IOCMG_GetRegAddr(pinTypedef);
    return DCL_IOCMG_GetPullMode(iocmgRegx);
}

/**
  * @brief Set Pin Schmidt Mode
  * @param pinTypedef the pin type defined in iomap.h
  * @param schmidtMode function define as @ref IOCMG_SchmidtMode
  * @retval IOCMG_Status @ref IOCMG_Status.
  */
IOCMG_Status HAL_IOCMG_SetPinSchmidtMode(unsigned int pinTypedef, IOCMG_SchmidtMode schmidtMode)
{
    IOCMG_PARAM_CHECK_WITH_RET(schmidtMode <= SCHMIDT_ENABLE && schmidtMode >= SCHMIDT_DISABLE, IOCMG_PARAM_ERROR);
    /* get iocmg reg address */
    IOCMG_REG* iocmgRegx = IOCMG_GetRegAddr(pinTypedef);
    DCL_IOCMG_SetSchmidtMode(iocmgRegx, schmidtMode);
    return IOCMG_STATUS_OK;
}

/**
  * @brief Get Pin Schmidt Mode
  * @param pinTypedef the pin type defined in iomap.h
  * @retval IOCMG_Status @ref IOCMG_Status.
  */
IOCMG_SchmidtMode HAL_IOCMG_GetPinSchmidtMode(unsigned int pinTypedef)
{
    /* get iocmg reg address */
    IOCMG_REG* iocmgRegx = IOCMG_GetRegAddr(pinTypedef);
    return DCL_IOCMG_GetSchmidtMode(iocmgRegx);
}

/**
  * @brief Set Pin level Shift Rate Mode
  * @param pinTypedef the pin type defined in iomap.h
  * @param schmidtMode function define as @ref IOCMG_SchmidtMode
  * @retval IOCMG_Status @ref IOCMG_Status.
  */
IOCMG_Status HAL_IOCMG_SetPinLevelShiftRate(unsigned int pinTypedef, IOCMG_LevelShiftRate levelShiftRate)
{
    IOCMG_PARAM_CHECK_WITH_RET(levelShiftRate < LEVEL_SHIFT_RATE_MAX, IOCMG_PARAM_ERROR);
    IOCMG_PARAM_CHECK_WITH_RET(levelShiftRate >= LEVEL_SHIFT_RATE_FAST, IOCMG_PARAM_ERROR);
    /* get iocmg reg address */
    IOCMG_REG* iocmgRegx = IOCMG_GetRegAddr(pinTypedef);
    DCL_IOCMG_SetLevelShiftRate(iocmgRegx, levelShiftRate);
    return IOCMG_STATUS_OK;
}

/**
  * @brief Get Pin Schmidt Mode
  * @param pinTypedef the pin type defined in iomap.h
  * @retval IOCMG_Status @ref IOCMG_Status.
  */
IOCMG_LevelShiftRate HAL_IOCMG_GetPinLevelShiftRate(unsigned int pinTypedef)
{
    /* get iocmg reg address */
    IOCMG_REG* iocmgRegx = IOCMG_GetRegAddr(pinTypedef);
    return DCL_IOCMG_GetLevelShiftRate(iocmgRegx);
}

/**
  * @brief Set Pin drive Rate Mode
  * @param pinTypedef the pin type defined in iomap.h
  * @param driveRate function define as @ref IOCMG_DriveRate
  * @retval IOCMG_Status @ref IOCMG_Status.
  */
IOCMG_Status HAL_IOCMG_SetPinDriveRate(unsigned int pinTypedef, IOCMG_DriveRate driveRate)
{
    /* get iocmg reg address */
    IOCMG_PARAM_CHECK_WITH_RET(driveRate < DRIVER_RATE_MAX && driveRate >= DRIVER_RATE_4, IOCMG_PARAM_ERROR);
    IOCMG_REG* iocmgRegx = IOCMG_GetRegAddr(pinTypedef);
    DCL_IOCMG_SetDriveRate(iocmgRegx, driveRate);
    return IOCMG_STATUS_OK;
}

/**
  * @brief Get Pin drive Rate Mode
  * @param pinTypedef the pin type defined in iomap.h
  * @retval Value of @ref IOCMG_DriveRate.
  */
IOCMG_DriveRate HAL_IOCMG_GetPinDriveRate(unsigned int pinTypedef)
{
    /* get iocmg reg address */
    IOCMG_REG* iocmgRegx = IOCMG_GetRegAddr(pinTypedef);
    return DCL_IOCMG_GetDriveRate(iocmgRegx);
}

/**
  * @brief Set OSC Pin clock output enable mode
  * @param mode function enable or not
  * @retval IOCMG_Status @ref IOCMG_Status.
  */
IOCMG_Status HAL_IOCMG_SetOscClkOutputMode(bool mode)
{
    IOCMG_ASSERT_PARAM(mode == BASE_CFG_ENABLE || mode == BASE_CFG_DISABLE);
    DCL_IOCMG_SetOscClkOutputMode(mode);
    return IOCMG_STATUS_OK;
}

/**
  * @brief Get OSC Pin clock output enable mode
  * @retval bool enable or not
  */
bool HAL_IOCMG_GetOscClkOutputMode(void)
{
    return DCL_IOCMG_GetOscClkOutputMode();
}

/**
  * @brief Set OSC Pin function enable mode
  * @param mode function enable or not
  * @retval IOCMG_Status @ref IOCMG_Status.
  */
IOCMG_Status HAL_IOCMG_SetOscClkFuncMode(bool mode)
{
    IOCMG_ASSERT_PARAM(mode == BASE_CFG_ENABLE || mode == BASE_CFG_DISABLE);
    DCL_IOCMG_SetOscClkFuncMode(mode);
    return IOCMG_STATUS_OK;
}

/**
  * @brief Get OSC Pin Pin function enable mode
  * @retval bool enable or not
  */
bool HAL_IOCMG_GetOscClkFuncMode(void)
{
    return DCL_IOCMG_GetOscClkFuncMode();
}

/**
  * @brief Set OSC Pin drive rate mode
  * @param driveRate osc drive rate
  * @retval IOCMG_Status @ref IOCMG_Status.
  */
IOCMG_Status HAL_IOCMG_SetOscClkDriveRate(IOCMG_OscClkDriveRate oscClkDriveRate)
{
    IOCMG_PARAM_CHECK_WITH_RET(oscClkDriveRate < OSC_CLK_DRIVER_RATE_MAX && \
                               oscClkDriveRate >= OSC_CLK_DRIVER_RATE_1, IOCMG_PARAM_ERROR);
    DCL_IOCMG_SetOscClkDriveRate(oscClkDriveRate);
    return IOCMG_STATUS_OK;
}

/**
  * @brief Get OSC Pin drive rate mode
  * @retval IOCMG_Status @ref IOCMG_Status.
  */
IOCMG_OscClkDriveRate HAL_IOCMG_GetOscClkDriveRate(void)
{
    return DCL_IOCMG_GetOscClkDriveRate();
}