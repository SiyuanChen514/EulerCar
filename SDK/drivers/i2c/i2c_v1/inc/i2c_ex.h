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
  * @file      i2c_ex.h
  * @author    MCU Driver Team
  * @brief     I2C module driver
  * @details   The header file contains the following declaration:
  *             + Setting the Special Function Configuration.
  */

#ifndef McuMagicTag_I2C_EX_H
#define McuMagicTag_I2C_EX_H

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
/* Macro definitions ---------------------------------------------------------*/
/**
  * @addtogroup I2C_IP
  * @{
  */

/**
  * @defgroup I2C_EX_API_Declaration I2C HAL API EX
  * @{
  */
BASE_StatusType HAL_I2C_SetDataTransferSequenceEx(I2C_Handle *handle, I2C_DataTransferSequenceType sequence);
BASE_StatusType HAL_I2C_SetSclStretchModeEx(I2C_Handle *handle, I2C_ClockStretchType clkStretch);
BASE_StatusType HAL_I2C_SetSclLowTimeoutEx(I2C_Handle *handle, unsigned int sclLowTimeout);
BASE_StatusType HAL_I2C_SetBusFreeTimeEx(I2C_Handle *handle, unsigned int busFreeTime);
BASE_StatusType HAL_I2C_Set10BitSlaveEnableEx(I2C_Handle *handle);
BASE_StatusType HAL_I2C_SetDeviceIdAddressEnableEx(I2C_Handle *handle);
BASE_StatusType HAL_I2C_SetStartByteEnableEx(I2C_Handle *handle);
BASE_StatusType HAL_I2C_SetOwnAddressMaskEx(I2C_Handle *handle, unsigned int addrMask);
BASE_StatusType HAL_I2C_SetOwnXmbAddressMaskEx(I2C_Handle *handle, unsigned int addrMask);
/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_I2C_H */