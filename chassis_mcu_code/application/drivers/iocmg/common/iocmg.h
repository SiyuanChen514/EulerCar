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
  * @file      iocmg.h
  * @author    MCU Driver Team
  * @brief     IOCMG module driver
  * @details   This file provides functions declaration of iocmg
  */
/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_IOCMG_H
#define McuMagicTag_IOCMG_H

/* Includes ------------------------------------------------------------------ */
#include "iocmg_ip.h"
/**
  * @defgroup IOCMG ICOMG
  * @brief IOCMG module.
  * @{
  */

/**
  * @defgroup IOCMG_Common IOMG Common
  * @brief IOCMG common external module.
  * @{
  */

/**
  * @defgroup IOCMG_Handle_Definition IOCMG Handle Definition
  * @{
  */
typedef struct {
    unsigned int pinTypedef;
    IOCMG_PullMode pullMode;
    IOCMG_SchmidtMode schmidtMode;
    IOCMG_LevelShiftRate levelShiftRate;
    IOCMG_DriveRate driveRate;
} IOCMG_Handle;
/**
  * @}
  */
 
/**
  * @defgroup IOCMG_API_Declaration IOCMG HAL API
  * @{
  */
/* Exported global functions ------------------------------------------------- */
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
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_IOCMG_H */