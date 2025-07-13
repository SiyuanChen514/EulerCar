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

static void ACMP1_Init(void)
{
    g_acmp1.baseAddress =  ACMP1_BASE;

    g_acmp1.enable = true;
    g_acmp1.syncEn = false;
    g_acmp1.inOutConfig.vinNNum = ACMP_VIN_MUX3; /* input select */
    g_acmp1.inOutConfig.vinPNum = ACMP_VIN_MUX3; /* input select */
    g_acmp1.inOutConfig.swVinPNum = ACMP_SW_VIN3;
    g_acmp1.inOutConfig.swVinNNum = ACMP_SW_VIN3;
    g_acmp1.inOutConfig.polarity = ACMP_OUT_NOT_INVERT;
    g_acmp1.filterCtrl.filterMode = ACMP_FILTER_NONE; /* filter level */
    g_acmp1.hysteresisVol = 0; /* 0: without hysteresis */
    HAL_ACMP_Init(&g_acmp1);
}

static void IOConfig(void)
{
    IOConfig_RegStruct *iconfig = IOCONFIG;

    iconfig->iocmg_18.BIT.func = 0x9; /* 0x9 is ACMP1_ANA_N */
    iconfig->iocmg_18.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_18.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_18.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_18.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_18.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_21.BIT.func = 0x9; /* 0x9 is ACMP1_ANA_P */
    iconfig->iocmg_21.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_21.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_21.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_21.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_21.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_26.BIT.func = 0x2; /* 0x2 is ACMP1_OUT */
    iconfig->iocmg_26.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_26.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_26.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_26.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_26.BIT.se = BASE_CFG_DISABLE;
}

void SystemInit(void)
{
    IOConfig();
    ACMP1_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}