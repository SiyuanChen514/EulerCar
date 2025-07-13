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
  * @file      assert.h
  * @author    MCU Driver Team
  * @brief     BASE module driver
  * @details   This file provides functions declaration of the assert,
  *           + BASE_FUNC_PARAMCHECK_NO_RET macro function definition.
  *           + BASE_FUNC_PARAMCHECK_WITH_RET macro function definition.
  *           + BASE_FUNC_ASSERT_PARAM macro function definition.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_ASSERT_H
#define McuMagicTag_ASSERT_H

/* Includes ------------------------------------------------------------------ */
#include "chipinc.h"
#include "typedefs.h"
/**
  * @defgroup BASE BASE
  * @brief BASE module.
  * @{
  */

void AssertErrorLog(char *file, unsigned int line);

/**
  * @defgroup ASSERT Assert Definition
  * @brief Definition of different assert.
  * @{
  */

/**
  * @defgroup ASSERT_Macro ASSERT Macro Function Definition
  * @{
  */
/* Macro definitions --------------------------------------------------------- */
#if (BASE_DEFINE_USE_ASSERT == BASE_CFG_ENABLE)
#define BASE_FUNC_PARAMCHECK_NO_RET(param) \
    do { \
        if (!(param)) { \
            AssertErrorLog(__FILE__, __LINE__); \
            return; \
        } \
    } while (0)

#define BASE_FUNC_PARAMCHECK_WITH_RET(param, ret) \
    do { \
        if (!(param)) { \
            AssertErrorLog(__FILE__, __LINE__); \
            return ret; \
        } \
    } while (0)

#define BASE_FUNC_ASSERT_PARAM(param) \
    do { \
        if (!(param)) { \
            AssertErrorLog(__FILE__, __LINE__); \
            while (1) { \
            }; \
        } \
    } while (0)

#else
#define BASE_FUNC_ASSERT_PARAM(param)               ((void)0U)
#define BASE_FUNC_PARAMCHECK_NO_RET(param)          ((void)0U)
#define BASE_FUNC_PARAMCHECK_WITH_RET(param, ret)   ((void)0U)

#endif /* BASE_DEFINE_USE_ASSERT */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_ASSERT_H */