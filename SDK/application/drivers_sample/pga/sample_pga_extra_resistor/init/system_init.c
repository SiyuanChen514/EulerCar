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
  * @file      system_init.c
  * @author    MCU Driver Team
  * @brief     This file contains driver init functions.
  */

#include "main.h"
#include "ioconfig.h"

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 32; /* PLL Multiplier 32 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_1;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;
    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static void PGA1_Init(void)
{
    HAL_CRG_IpEnableSet(PGA1_BASE, IP_CLK_ENABLE);

    g_pgaExt.baseAddress = PGA1_BASE; /* pga1 */
    g_pgaExt.enable = BASE_CFG_ENABLE;
    g_pgaExt.extLoopbackEn = BASE_CFG_DISABLE;
    g_pgaExt.pgaMux = PGA_EXT_RES_VI0; /* use external resistor */
    g_pgaExt.gain = PGA_GAIN_1X;

    HAL_PGA_Init(&g_pgaExt);
}

static void IOConfig(void)
{
    IOConfig_RegStruct *iconfig = IOCONFIG;

    iconfig->iocmg_22.BIT.func = 0x9; /* 0x9 is PGA1_ANA_P */
    iconfig->iocmg_22.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_22.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_22.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_22.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_22.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_23.BIT.func = 0x9; /* 0x9 is PGA1_ANA_N */
    iconfig->iocmg_23.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_23.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_23.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_23.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_23.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_24.BIT.func = 0x8; /* 0x8 is PGA1_ANA_EXT */
    iconfig->iocmg_24.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_24.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_24.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_24.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_24.BIT.se = BASE_CFG_DISABLE;
}

void SystemInit(void)
{
    IOConfig();
    PGA1_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}