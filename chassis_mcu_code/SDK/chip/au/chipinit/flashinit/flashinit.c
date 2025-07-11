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
 * @brief     flash init module.
 * @details   flash initialization function during startup
 */
#include "chipinit.h"
#include "crg.h"
#include "flash_ip.h"
#include "flashinit.h"

#define FREQ_1MHz              (1000 * 1000)
#define SREAD_DIV_STEP         (5 * FREQ_1MHz)
#define NREAD_DIV_STEP         (50 * FREQ_1MHz)
#define SWMTIMER_OPTVAL_STEP   (10 * FREQ_1MHz)
#define SMWTIMER_OPTVAL_MIN_VAL 2

/**
 * @brief Get the Rounding up value
 * @param val input value
 * @param modulo modeulo value
 * @retval value after rounding up process
 */
static inline unsigned int RoundingUp(unsigned int val, unsigned int modulo)
{
    return (val + modulo - 1) / modulo;
}

/**
 * @brief Get the Rounding up value
 * @param coreClkSelect Core Clock select
 * @retval Frequency of Flash
 */
static unsigned int GetFlashFreq(CRG_CoreClkSelect coreClkSelect)
{
    unsigned int hclk;

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
    unsigned int pclk;
    unsigned int freq;
    unsigned int sreadDiv;
    unsigned int smwTimerOptVal;

    hclk = GetFlashFreq(coreClkSelect);
    pclk = hclk >> 1;

    cfg.reg = efc->EFLASH_CLK_CFG.reg;
    cfg.BIT.busclk_switch_protect_enable = BASE_CFG_SET;
    cfg.BIT.busclk_sw_req = BASE_CFG_SET;

    cfg.BIT.ef_timer_option_unit = RoundingUp(pclk, FREQ_1MHz);
    freq = hclk;
    sreadDiv = 0;
    while (freq > SREAD_DIV_STEP) {
        sreadDiv++;
        freq >>= 1;
    }
    cfg.BIT.sread_div = sreadDiv;
    cfg.BIT.nread_div = RoundingUp(hclk, NREAD_DIV_STEP) - 1;
    cfg.BIT.m20ns_div = cfg.BIT.nread_div;
    efc->EFLASH_CLK_CFG.reg = cfg.reg;
    smwTimerOptVal = RoundingUp(pclk, SWMTIMER_OPTVAL_STEP);
    if (smwTimerOptVal < SMWTIMER_OPTVAL_MIN_VAL) {
        smwTimerOptVal = SMWTIMER_OPTVAL_MIN_VAL;
    }
    efc->SMW_TIMER_OPTION.BIT.smw_timer_option_value = smwTimerOptVal;
}

/**
 * @brief Wait flash clock config done
 * @param None
 * @retval None
 */
void FLASH_WaitClockConfigDone(void)
{
    EFC_RegStruct *efc = EFC;
    while (efc->EFLASH_CLK_CFG.BIT.busclk_sw_req == BASE_CFG_SET) {
        ; /* Wait Eflash frequency switching completes configuration query */
    }
}