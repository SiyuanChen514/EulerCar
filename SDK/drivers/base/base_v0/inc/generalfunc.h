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
  * @file      generalfunc.h
  * @author    MCU Driver Team
  * @brief     BASE module driver
  * @details   This file provides functions declaration of the basic function
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_GENERAL_FUNC_H
#define McuMagicTag_GENERAL_FUNC_H

/* Includes ------------------------------------------------------------------ */
#include "chipinc.h"
#include "typedefs.h"
#include "assert.h"
#include "clock.h"

/**
  * @defgroup BASE BASE
  * @brief BASE module.
  * @{
  */

/**
  * @defgroup GeneralFunc GeneralFunc Definition
  * @brief Definition of GeneralFunc function.
  * @{
  */

/* Typedef definitions ------------------------------------------------------- */
/**
  * @defgroup BASIC_Structure_Definition BASE_AverageHandle Definition
  * @{
  */

/**
  * @brief Structure for configuring and controlling averaging
  */
typedef struct {
    unsigned int cnt;     /**< Used to record the divisor of the average */
    float *buf;  /**< Buffer pointer */
    unsigned int size;    /**< Buffer size */
    unsigned int at;      /**< Index value of the currently inserted value */
    unsigned int calNum;  /**< Total number to be averaged */
    float total; /**< Current Cumulative Sum */
} BASE_AverageHandle;
/**
  * @}
  */

/**
  * @defgroup BASIC_Structure_Definition BASE_FSM_Handle Definition
  * @{
  */
typedef BASE_FSM_Status (*FunType)(void);
/**
  * @brief General state machine handle
  */
typedef struct {
    FunType funList[BASE_DEFINE_FSM_END + 1];    /**< function list */
    BASE_FSM_Status nextFun;              /**< next function status */
} BASE_FSM_Handle;
/**
  * @}
  */

/**
  * @defgroup GENERAL_API_Definition GENERAL_API
  * @{
  */
/* Exported global functions ------------------------------------------------- */
unsigned int BASE_FUNC_GetTick(void);
unsigned int BASE_FUNC_FindArrayValue(const unsigned short *nums, unsigned int leng, unsigned int value);
unsigned char BASE_FUNC_CalcSumByte(const unsigned char *pt, unsigned int len);
unsigned short BASE_FUNC_CalcSumShort(unsigned char const * pt, unsigned int len);
BASE_StatusType BASE_FUNC_AverageInit(unsigned int index, float *buf, unsigned int size, unsigned int calNum);
float BASE_FUNC_GetSlipAverageVal(unsigned int index, float val);
void BASE_FUNC_AverageDeInit(unsigned int index);
void BASE_FSM_FunRegister(BASE_FSM_Status index, FunType funAddress);
void BASE_FSM_Run(unsigned int delayTime, BASE_DelayUnit delayUnit);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_GENERAL_FUNC_H */