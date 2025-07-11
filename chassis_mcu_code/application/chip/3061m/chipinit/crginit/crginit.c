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
  * @file      crginit.c
  * @author    MCU Driver Team
  * @brief     crg init modlue.
  * @details   crg initialization function during startup
  */
#include "crginit.h"

/**
 * @brief CRG Config
 * @param coreClkSelect OutPut core clock select value
 * @retval None
 */
__weak BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    BASE_FUNC_ASSERT_PARAM(coreClkSelect != NULL);

    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllFbDiv        = 0x30;  /* PLL loop divider ratio = 0x30 */
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllPostDiv      = CRG_PLL_POSTDIV_2;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;
    crg.handleEx.pllPostDiv2 = CRG_PLL_POSTDIV2_3;
    crg.handleEx.clk1MSelect = CRG_1M_CLK_SELECT_HOSC; /* Set the 1MHz clock select. */
    crg.handleEx.clk1MDiv = 0x18;   /* 0x18 : ensure that the 1MHz clock frequency is 1 MHz. */
    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

/**
 * @brief Set Crg Core clock select
 * @param coreClkSelect Input core clock select value
 * @retval None
 */
void CRG_SetCoreClockSelect(CRG_CoreClkSelect coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress = CRG;
    crg.coreClkSelect = coreClkSelect;
    if (crg.coreClkSelect == CRG_CORE_CLK_SELECT_TCXO) { /* If an external crystal oscillator is selected. */
        BASE_FUNC_DELAY_MS(10);  /* 10: delay 10ms, wait clokc stable. */
    }
    HAL_CRG_SetCoreClockSelect(&crg);
}