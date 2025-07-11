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
 * @file    pga.c
 * @author  MCU Driver Team.
 * @brief   Programmable Gain Amplifier HAL level module driver.
 *          This file provides firmware functions to manage the following
 *          functionalities of the amplifier
 *           + Programmable Gain Amplifier's Initialization and de-initialization functions
 *           + Set amplifier's gain value
 */
#include "pga.h"
#include "assert.h"

/**
  * @brief PGA HAL Init
  * @param pgaHandle: PGA handle.
  * @retval BASE_StatusType. BASE_STATUS_OK: success, BASE_STATUS_ERROR: fail.
  */
BASE_StatusType HAL_PGA_Init(PGA_Handle *pgaHandle)
{
    PGA_ASSERT_PARAM(pgaHandle != NULL);
    PGA_ASSERT_PARAM(IsPGAInstance(pgaHandle->baseAddress));
    PGA_PARAM_CHECK_WITH_RET(IsPgaGain(pgaHandle->gain), BASE_STATUS_ERROR);
    PGA_PARAM_CHECK_WITH_RET(IsPgaExtCapCompensation(pgaHandle->handleEx.extCapCompensation), BASE_STATUS_ERROR);
    /* Initial configuration of the PGA. */
    PGA_CTRL1_REG pgaControl1;
    pgaControl1.reg = pgaHandle->baseAddress->PGA_CTRL1.reg;
    pgaControl1.BIT.da_pga_mode_ctrl = pgaHandle->externalResistorMode;  /* PGA mode configuration. */
    pgaControl1.BIT.da_pga_gain_ctrl = pgaHandle->gain;                  /* PGA gain setting. */
    pgaControl1.BIT.da_pga_cf_ctrl = pgaHandle->handleEx.extCapCompensation;
    pgaHandle->baseAddress->PGA_CTRL1.reg = pgaControl1.reg;
    /* Enable PGA */
    pgaHandle->baseAddress->PGA_CTRL0.BIT.da_pga_enh = BASE_CFG_ENABLE;
    return BASE_STATUS_OK;
}

/**
  * @brief PGA HAL DeInit
  * @param pgaHandle: PGA handle.
  * @retval BASE_StatusType.
  */
BASE_StatusType HAL_PGA_DeInit(PGA_Handle *pgaHandle)
{
    PGA_ASSERT_PARAM(pgaHandle != NULL);
    PGA_ASSERT_PARAM(IsPGAInstance(pgaHandle->baseAddress));
    pgaHandle->baseAddress->PGA_CTRL0.reg = BASE_CFG_DISABLE;  /* Disable PGA. */
    pgaHandle->baseAddress->PGA_CTRL1.reg = BASE_CFG_DISABLE;  /* Gain and mode deinitialization. */
    return BASE_STATUS_OK;
}

/**
  * @brief Set Gain value
  * @param pgaHandle: PGA handle.
  * @param gain: gain value. @ref PGA_GainValue
  * @retval None.
  */
void HAL_PGA_SetGain(PGA_Handle *pgaHandle, PGA_GainValue gain)
{
    PGA_ASSERT_PARAM(pgaHandle != NULL);
    PGA_ASSERT_PARAM(IsPGAInstance(pgaHandle->baseAddress));
    pgaHandle->baseAddress->PGA_CTRL1.BIT.da_pga_gain_ctrl = gain;  /* Gain value setting. */
}

/**
  * @brief Start PGA
  * @param pgaHandle: PGA handle.
  * @retval None
  */
void HAL_PGA_Start(PGA_Handle *pgaHandle)
{
    PGA_ASSERT_PARAM(pgaHandle != NULL);
    PGA_ASSERT_PARAM(IsPGAInstance(pgaHandle->baseAddress));
    pgaHandle->baseAddress->PGA_CTRL0.BIT.da_pga_enh = BASE_CFG_ENABLE; /* Enable PGA. */
}

/**
  * @brief Stop PGA
  * @param pgaHandle: PGA handle.
  * @retval None
  */
void HAL_PGA_Stop(PGA_Handle *pgaHandle)
{
    PGA_ASSERT_PARAM(pgaHandle != NULL);
    PGA_ASSERT_PARAM(IsPGAInstance(pgaHandle->baseAddress));
    pgaHandle->baseAddress->PGA_CTRL0.BIT.da_pga_enh = BASE_CFG_DISABLE;    /* Disable PGA. */
}