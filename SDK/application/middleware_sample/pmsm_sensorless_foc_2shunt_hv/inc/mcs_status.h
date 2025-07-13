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
  * @file      mcs_status.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of system status.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_STATUS_H
#define McuMagicTag_MCS_STATUS_H

/* Includes ------------------------------------------------------------------------------------ */
#include "typedefs.h"
#include "mcs_assert.h"

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief System status define
  */
typedef union {
    unsigned short all;
    struct {
        unsigned short cmdStart   : 1; /**< Indicates that a start system command has been received. */
        unsigned short cmdStop    : 1; /**< Indicates that a stop system command has been received. */
        unsigned short isRunning  : 1; /**< Indicates that the system is running (enable signal) */
        unsigned short sysError   : 1; /**< Indicates that the system reports an error. */
        unsigned short poweron    : 1; /**< Indicates that the power-on initialization phase is complete. */
        unsigned short capcharge  : 1; /**< Indicates that the bootstrap capacitor charging phase is complete. */
        unsigned short adczero    : 1; /**< The current sampling point is reset to zero after power-on. */
    } Bit;
} SysStatusReg;

/**
  * @brief Get status of Bit cmdStart.
  * @param sysStatus System status register handle.
  * @retval Status of Bit cmdStart.
  */
static inline bool SysGetCmdStart(SysStatusReg *sysStatus)
{
    MCS_ASSERT_PARAM(sysStatus != NULL);
    bool ret;
    ret = (sysStatus->Bit.cmdStart == 1) ? true : false;
    return ret;
}

/**
  * @brief Set Bit cmdStart.
  * @param sysStatus System status register handle.
  * @retval None.
  */
static inline void SysCmdStartSet(SysStatusReg *sysStatus)
{
    MCS_ASSERT_PARAM(sysStatus != NULL);
    sysStatus->Bit.cmdStart = 1;
}

/**
  * @brief Clear Bit cmdStart.
  * @param handle System status register handle.
  * @retval None.
  */
static inline void SysCmdStartClr(SysStatusReg *sysStatus)
{
    MCS_ASSERT_PARAM(sysStatus != NULL);
    sysStatus->Bit.cmdStart = 0;
}

/**
  * @brief Get status of Bit cmdStop.
  * @param sysStatus System status register handle.
  * @retval Status of Bit cmdStart.
  */
static inline bool SysGetCmdStop(SysStatusReg *sysStatus)
{
    MCS_ASSERT_PARAM(sysStatus != NULL);
    bool ret;
    ret = (sysStatus->Bit.cmdStop == 1) ?  true : false;
    return ret;
}

/**
  * @brief Set Bit cmdStop.
  * @param sysStatus System status register handle.
  * @retval None.
  */
static inline void SysCmdStopSet(SysStatusReg *sysStatus)
{
    MCS_ASSERT_PARAM(sysStatus != NULL);
    sysStatus->Bit.cmdStop = 1;
}

/**
  * @brief Clear Bit cmdStop.
  * @param sysStatus System status register handle.
  * @retval None.
  */
static inline void SysCmdStopClr(SysStatusReg *sysStatus)
{
    MCS_ASSERT_PARAM(sysStatus != NULL);
    sysStatus->Bit.cmdStop = 0;
}

/**
  * @brief Get status of Bit isRunning.
  * @param sysStatus System status register handle.
  * @retval Status of Bit isRunning.
  */
static inline bool SysIsRunning(SysStatusReg *sysStatus)
{
    MCS_ASSERT_PARAM(sysStatus != NULL);
    bool ret;
    ret = (sysStatus->Bit.isRunning == 1) ? true : false;
    return ret;
}

/**
  * @brief Set Bit isRuning.
  * @param sysStatus System status register handle.
  * @retval None.
  */
static inline void SysRunningSet(SysStatusReg *sysStatus)
{
    MCS_ASSERT_PARAM(sysStatus != NULL);
    sysStatus->Bit.isRunning = 1;
}

/**
  * @brief Clear Bit isRuning.
  * @param sysStatus System status register handle.
  * @retval None.
  */
static inline void SysRunningClr(SysStatusReg *sysStatus)
{
    MCS_ASSERT_PARAM(sysStatus != NULL);
    sysStatus->Bit.isRunning = 0;
}

/**
  * @brief Get status of Bit sysError.
  * @param sysStatus System status register handle.
  * @retval Status of Bit sysError.
  */
static inline bool SysIsError(SysStatusReg *sysStatus)
{
    MCS_ASSERT_PARAM(sysStatus != NULL);
    bool ret;
    ret = (sysStatus->Bit.sysError == 1) ? true : false;
    return ret;
}

/**
  * @brief Set Bit sysError.
  * @param sysStatus System status register handle.
  * @retval None.
  */
static inline void SysErrorSet(SysStatusReg *sysStatus)
{
    MCS_ASSERT_PARAM(sysStatus != NULL);
    sysStatus->Bit.sysError = 1;
}

/**
  * @brief Clear Bit sysError.
  * @param sysStatus System status register handle.
  * @retval None.
  */
static inline void SysErrorClr(SysStatusReg *sysStatus)
{
    MCS_ASSERT_PARAM(sysStatus != NULL);
    sysStatus->Bit.sysError = 0;
}

#endif
