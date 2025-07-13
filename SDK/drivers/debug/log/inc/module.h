/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      mpdule.h
  * @author    MCU Driver Team
  * @brief     Definition of the Miniaturized Log Module
  * @details   The header file contains the following declaration:
  *             + Definition of the ID of the miniaturized log module
  */
#ifndef MODULE_H
#define MODULE_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

/**
  * @addtogroup DEBUG_Log
  * @brief DEBUG external module.
  * @{
  */

 /**
  * @defgroup MODULE_Def MODULE_Def
  * @brief define the device model.
  * @{
  */

/* * Module ID flags */
enum ExtModule {
    EXT_MODULE_APP_MAIN,
    EXT_MODULE_APP_CONSOLE,
    EXT_MODULE_APP_CHIP,
    EXT_MODULE_DRV_BASE,
    EXT_MODULE_DRV_CHIPS,
    EXT_MODULE_DRV_CRG,
    EXT_MODULE_DRV_GPIO,
    EXT_MODULE_DRV_I2C,
    EXT_MODULE_DRV_IRQ,
    EXT_MODULE_DRV_PINCTRL,
    EXT_MODULE_DRV_TIMER,
    EXT_MODULE_DRV_UART,
    EXT_MODULE_DFX,
    EXT_MODULE_BUTT
};

#define MODULE_ID_MASK      0xFF000000
#define FEATURE_ID_MASK     0x00FF0000
#define PARAM_USE_ID_MASK   0x0000FFFF

#define MODULE_ID_OFFSET    0x18
#define FEATURE_ID_OFFSET   0x10
#define PARAM_USE_ID_OFFSET 0x8

#define SCENE_UINT_MAX_ID   0xFF

#define SCENE_END_LABEL  0xABCDABCD

#define STATE_CODE_MODULE_MASK    0x10
#define STATE_CODE_MASK    0xFFFF0000

#define STATE_CODE(moduleId, stateCode)    (moduleId << STATE_CODE_MODULE_MASK | (stateCode & STATE_CODE_MASK))
#define STATE_CODE_ERR_CHECK(ret)    ((ret & STATE_CODE_MASK) > EXT_RIGHT_UNKNOWN)
#define STATE_CODE_RIGHT_CHECK(ret)    ((ret & STATE_CODE_MASK) <= EXT_RIGHT_UNKNOWN)

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

/**
  * @}
  */

/**
  * @}
  */
 
#endif /* __EXT__MODULE__ */
