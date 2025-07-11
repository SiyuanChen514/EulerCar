/**
  * @ Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2023. All rights reserved.
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
  * @file      mcs_assert.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of the assert.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_MCS_ASSERT_H
#define McuMagicTag_MCS_ASSERT_H

/* Includes ------------------------------------------------------------------ */
#include "baseinc.h"

/**
  * @defgroup MCS_ASSERT MCS_ASSERT
  * @brief MCS ASSERT module.
  * @{
  */

/**
  * @defgroup ASSERT_Macro ASSERT Macro Function Definition
  * @{
  */
#ifdef MCS_PARAM_CHECK
#define MCS_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define MCS_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define MCS_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define MCS_ASSERT_PARAM(para) ((void)0U)
#define MCS_PARAM_CHECK_NO_RET(para) ((void)0U)
#define MCS_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

/**
  * @}
  */

/**
  * @}
  */
#endif