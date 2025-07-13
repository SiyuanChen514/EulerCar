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
  * @file      flashinit.c
  * @author    MCU Driver Team
  * @brief     flash init modlue.
  * @details   flash initialization function during startup
  */
#include "chipinit.h"
#include "crg.h"
#include "flash_ip.h"
#include "flashinit.h"

#define FLASH_BASE_FREQ          (375 * 100 * 1000)   /* 37.5MHz. */
#define FLASH_MAX_DIV            4

/**
 * @brief Get the Rounding up value
 * @param frequence frequnce
 * @param div Output Divison
 * @retval None
 */
static void SetFlashDiv(unsigned int frequency, unsigned int *nreadDiv)
{
    unsigned int div;
    unsigned int freq = frequency;
    /* Get frequency divider of flash. */
    if (freq < FLASH_BASE_FREQ) {
        freq = FLASH_BASE_FREQ;
    }
    /* Get the flash frequency division based on the frequency. */
    if ((freq % FLASH_BASE_FREQ) == 0) {
        div = freq / FLASH_BASE_FREQ;
    } else {
        div = (freq / FLASH_BASE_FREQ) + 1;
    }
    /* Ensure the flash frequency division is valid. */
    if (div > FLASH_MAX_DIV) {
        div = FLASH_MAX_DIV;
    }
    *nreadDiv = div;
}

/**
 * @brief Get the Rounding up value
 * @param coreClkSelect Core Clock select
 * @retval Frequency of Flash
 */
static unsigned int GetFlashFreq(CRG_CoreClkSelect coreClkSelect)
{
    unsigned int hclk;
    /* Get frequency of flash. */
    switch (coreClkSelect) {
        case CRG_CORE_CLK_SELECT_HOSC:
            hclk = HOSC_FREQ;
            break;
        case CRG_CORE_CLK_SELECT_TCXO:
            hclk = XTRAIL_FREQ;
            break;
        case CRG_CORE_CLK_SELECT_PLL:
            hclk = HAL_CRG_GetPllFreq();
            break;
        default:
            hclk = LOSC_FREQ;
            break;
    }
    return hclk;
}

/**
 * @brief Set flash clock frequence base on hclk
 * @param coreClkSelect core clock select
 * @retval None
 */
void FLASH_ClockConfig(CRG_CoreClkSelect coreClkSelect)
{
    EFC_RegStruct *efc = EFC;
    EFLASH_CLK_CFG_REG cfg;
    unsigned int hclk;
    unsigned int nreadDiv;

    /* Step 1: Set nread_div */
    hclk = GetFlashFreq(coreClkSelect);
    cfg.reg = efc->EFLASH_CLK_CFG.reg;
    SetFlashDiv(hclk, &nreadDiv);
    cfg.BIT.nread_div = nreadDiv;
    cfg.BIT.busclk_sw_req = BASE_CFG_SET;
    cfg.BIT.cur_read_vref_cal = BASE_CFG_SET;

    /* Step 2: Wait Busclk_sw_req */
    cfg.BIT.cur_read_vref_cal = BASE_CFG_SET;
    cfg.BIT.busclk_sw_req = BASE_CFG_SET;
    efc->EFLASH_CLK_CFG.reg = cfg.reg;
    while (efc->EFLASH_CLK_CFG.BIT.busclk_sw_req == BASE_CFG_SET) {
        ;
    }
}