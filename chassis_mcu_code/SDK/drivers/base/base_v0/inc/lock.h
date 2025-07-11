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
  * @file      lock.h
  * @author    MCU Driver Team
  * @brief     BASE module driver
  * @details   This file provides functions declaration of lock
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_LOCK_H
#define McuMagicTag_LOCK_H

/* Includes ------------------------------------------------------------------ */
#include "chipinc.h"
#include "typedefs.h"
#include "assert.h"

/**
  * @defgroup BASE BASE
  * @brief BASE module.
  * @{
  */

/**
  * @defgroup LOCK Lock Definition
  * @brief Definition of LOCK Definition.
  * @{
  */

/**
  * @defgroup LOCK_ENUM_DEFINITION BASE_LockStatus Definition
  * @brief Definition of LOCK Definition.
  * @{
  */
/* Typedef definitions ------------------------------------------------------- */
/**
  * @brief Lock status definition
  */
typedef enum {
    BASE_STATUS_UNLOCKED = 0,
    BASE_STATUS_LOCKED = 1
} BASE_LockStatus;
/**
  * @}
  */

/**
  * @defgroup LOCK_API_DEFINITION Lock API
  * @brief Definition of lock API Definition.
  * @{
  */
/* Exported global functions ------------------------------------------------- */
bool BASE_FUNC_SoftwareLock(unsigned int * const addr);
void BASE_FUNC_SoftwareUnLock(unsigned int * const addr);
bool BASE_FUNC_HardwareLock(CHIP_LockType const hwIndex);
void BASE_FUNC_HardwareUnLock(CHIP_LockType const hwIndex);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_LOCK_H */