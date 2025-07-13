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
  * @file      lock.c
  * @author    MCU Driver Team
  * @brief     Provides functions about locks.
  */

/* Includes ------------------------------------------------------------------ */
#include "lock.h"

/* Global Variables----------------------------------------------------------- */
unsigned int g_baseLock[CHIP_LOCK_TOTAL]; /**< Used to store the hardware lock status */

/**
  * @brief Attempt to acquire a lock for the specified address.
  * @param addr Point to the address where the lock is obtained.
  * @retval true, Succeeded in obtaining the lock.
  * @retval false, Failed to obtain the lock. The resource has been locked.
  */
bool BASE_FUNC_SoftwareLock(unsigned int * const addr)
{
    BASE_FUNC_PARAMCHECK_WITH_RET(addr, false);

    unsigned int tmpLocked = *addr;
    *addr = BASE_STATUS_LOCKED;
    /* Atomic exchange instructions are not supported. Lock determination and locking may be interrupted by */
    /* interrupts. To ensure atomicity, disable the corresponding interrupts. */
    if (tmpLocked == BASE_STATUS_UNLOCKED) {
        return true;
    }
    return false;
}

/**
  * @brief Releases the lock of the specified address.
  * @param addr Point to the address that releases the lock.
  * @retval None.
  */
void BASE_FUNC_SoftwareUnLock(unsigned int * const addr)
{
    BASE_FUNC_PARAMCHECK_NO_RET(addr);

    *addr = BASE_STATUS_UNLOCKED;
}

/**
  * @brief Attempt to acquire a lock on the specified hardware resource by hwIndex.
  * @param hwIndex Hardware Resource ID.
  * @retval true, Succeeded in obtaining the Hardware Resource lock.
  * @retval false, Failed to obtain the Hardware Resource lock. The resource has been locked.
  */
bool BASE_FUNC_HardwareLock(CHIP_LockType const hwIndex)
{
    BASE_FUNC_PARAMCHECK_WITH_RET((hwIndex >= 0 && hwIndex < CHIP_LOCK_TOTAL), false);
    return BASE_FUNC_SoftwareLock(&g_baseLock[hwIndex]);
}

/**
  * @brief Releases the lock of a specified hardware resource.
  * @param hwIndex Hardware Resource ID.
  * @retval None.
  */
void BASE_FUNC_HardwareUnLock(CHIP_LockType const hwIndex)
{
    BASE_FUNC_PARAMCHECK_NO_RET(hwIndex >= 0 && hwIndex < CHIP_LOCK_TOTAL);
    BASE_FUNC_SoftwareUnLock(&g_baseLock[hwIndex]);
}
