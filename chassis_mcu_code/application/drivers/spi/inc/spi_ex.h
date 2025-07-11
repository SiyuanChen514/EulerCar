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
  * @file      spi_ex.h
  * @author    MCU Driver Team
  * @brief     SPI module driver.
  * @details   This file provides firmware functions to manage the following.
  *            functionalities of the SPI.
  *            + SPI Set Functions.
  */
#ifndef McuMagicTag_SPI_EX_H
#define McuMagicTag_SPI_EX_H
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* Macro definitions ---------------------------------------------------------*/
/**
  * @addtogroup SPI_IP
  * @{
  */

/**
  * @defgroup SPI_EX_API_Declaration SPI HAL API EX
  * @{
  */
BASE_StatusType HAL_SPI_SetChipConfigSelectEx(SPI_Handle *handle, HAL_SPI_CHIP_CONFIG mode);
HAL_SPI_CHIP_CONFIG HAL_SPI_GetChipConfigSelectEx(SPI_Handle *handle);
/**
  * @}
  */

/**
  * @}
  */
#endif /* #ifndef McuMagicTag_SPI_EX_H */