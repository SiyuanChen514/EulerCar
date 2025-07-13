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
  * @file    spi_ex.c
  * @author  MCU Driver Team
  * @brief   SPI module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the SPI.
  *          + Peripheral Control functions
  */

/* Includes ------------------------------------------------------------------*/
#include "spi_ex.h"

/**
  * @brief SPI SET CHIP CONGFIG SELECT.
  * @param handle SPI_handle.
  * @param mode SPI CS mode.For details, see the enumeration definition of HAL_SPI_CHIP_CONFIG
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_SPI_SetChipConfigSelectEx(SPI_Handle *handle, HAL_SPI_CHIP_CONFIG mode)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    SPI_PARAM_CHECK_WITH_RET(IsSpiChipConfigMode(mode), BASE_STATUS_ERROR);
    handle->baseAddress->SPICSNCR.BIT.spi_csn_mode = mode; /* set chip mode */
    return BASE_STATUS_OK;
}

/**
  * @brief SPI GET CHIP CONGFIG SELECT.
  * @param handle SPI_handle.
  * @retval HAL_SPI_CHIP_CONFIG.
  */
HAL_SPI_CHIP_CONFIG HAL_SPI_GetChipConfigSelectEx(SPI_Handle *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    return handle->baseAddress->SPICSNCR.BIT.spi_csn_mode;
}