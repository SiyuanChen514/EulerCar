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
  * @file    acmp_ex.c
  * @author  MCU Driver Team
  * @brief   ACMP module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the acmp.
  *           + Set ACMP trim value functions.
  */

/* Includes ------------------------------------------------------------------*/
#include "acmp_ex.h"

/* Macro definitions ---------------------------------------------------------*/
#define TRIM_MAX_VALUE 255

/**
  * @brief Trim value setting
  * @param acmpHandle acmp handle.
  * @param trimValue trim value.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ACMP_SetTrimValueEx(ACMP_Handle *acmpHandle, unsigned char trimValue)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    ACMP_PARAM_CHECK_WITH_RET((trimValue < TRIM_MAX_VALUE), BASE_STATUS_ERROR);
    acmpHandle->baseAddress->ACMP_TRIM.BIT.da_acmp_trim = trimValue;  /* Trim value setting. */
    return BASE_STATUS_OK;
}