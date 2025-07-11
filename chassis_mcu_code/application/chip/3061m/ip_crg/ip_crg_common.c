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
  * @file      ip_crg_common.c
  * @author    MCU Driver Team
  * @brief     Contains ip crg common header files.
  */

/* Includes ----------------------------------------------------------------- */
#include "baseaddr.h"
#include "ip_crg_common.h"

/**
 * @brief Get IP frequency by ip register base address
 * @param ipBaseAddr The ip base address
 * @retval The bus frequency where the IP is located
 */
#ifdef FPGA
unsigned int CHIP_GetIpFreqHz(const void *ipBaseAddr)
{
    void *highRateIp[] = { /* Defines the module base address for FPGA development. */
        SYSCTRL1_BASE,
        CRC_BASE,
        APT0_BASE, APT1_BASE, APT2_BASE, APT3_BASE,
        CAPM0_BASE, CAPM1_BASE, CAPM2_BASE, CAPM_COMM_BASE,
        QDM0_BASE,
        ADC0_BASE,
        PGA0_BASE, PGA1_BASE,
        ACMP0_BASE,
    };

    if (ipBaseAddr == IWDG_BASE) { /* The IWDG working clock is LOSC clock. */
        return CHIP_IP_CLK_LOSC;
    } else if (ipBaseAddr == CAN_BASE) {  /* The CAN working clock is HOSC. */
        return CHIP_IP_CLK_CAN;
    } else {
        for (unsigned int i = 0; i < sizeof(highRateIp) / sizeof(highRateIp[0]); ++i) {
            if (ipBaseAddr == highRateIp[i]) {
                return CHIP_IP_CLK_HS;
            }
        }
        return CHIP_IP_CLK_LS; /* The base address does not match, return LOSC freq. */
    }
}
#endif

static const CHIP_CrgIpMatchInfo g_crgIpMatch[] = {
    {UART0_BASE, CRG_IP_WITH_CLK_SEL,  0x140, 0},
    {UART1_BASE, CRG_IP_WITH_CLK_SEL,  0x144, 0},
    {UART2_BASE, CRG_IP_WITH_CLK_SEL,  0x148, 0},
    {UART3_BASE, CRG_IP_WITH_CLK_SEL,  0x14C, 0},
    {TIMER0_BASE, CRG_IP_WITH_CLK_SEL, 0x240, 0},
    {TIMER1_BASE, CRG_IP_WITH_CLK_SEL, 0x244, 0},
    {TIMER2_BASE, CRG_IP_WITH_CLK_SEL, 0x248, 0},
    {TIMER3_BASE, CRG_IP_WITH_CLK_SEL, 0x24C, 0},
    {SYSTICK_BASE, CRG_IP_WITH_CLK_SEL, 0x40, 0},
    {SPI0_BASE, CRG_IP_WITH_CLK_SEL, 0x180, 0},
    {SPI1_BASE, CRG_IP_WITH_CLK_SEL, 0x184, 0},
    {I2C0_BASE, CRG_IP_WITH_CLK_SEL, 0x1C0, 0},
    {I2C1_BASE, CRG_IP_WITH_CLK_SEL, 0x1C4, 0},
    {CAN_BASE, CRG_IP_CAN, 0x2C0, 0},
    {GPT0_BASE, CRG_IP_NONE_CLK_SEL, 0x440, 0},
    {GPT1_BASE, CRG_IP_NONE_CLK_SEL, 0x444, 0},
    {GPT2_BASE, CRG_IP_NONE_CLK_SEL, 0x448, 0},
    {GPT3_BASE, CRG_IP_NONE_CLK_SEL, 0x44C, 0},
    {WWDG_BASE, CRG_IP_NONE_CLK_SEL, 0x200, 0},
    {CAPM0_BASE, CRG_IP_NONE_CLK_SEL, 0x280, 0},
    {CAPM1_BASE, CRG_IP_NONE_CLK_SEL, 0x284, 0},
    {CAPM2_BASE, CRG_IP_NONE_CLK_SEL, 0x288, 0},
    {DMA_BASE, CRG_IP_NONE_CLK_SEL, 0x300, 0},
    {GPIO0_BASE, CRG_IP_NONE_CLK_SEL, 0x480, 0},
    {GPIO1_BASE, CRG_IP_NONE_CLK_SEL, 0x484, 0},
    {GPIO2_BASE, CRG_IP_NONE_CLK_SEL, 0x488, 0},
    {GPIO3_BASE, CRG_IP_NONE_CLK_SEL, 0x48C, 0},
    {GPIO4_BASE, CRG_IP_NONE_CLK_SEL, 0x490, 0},
    {GPIO5_BASE, CRG_IP_NONE_CLK_SEL, 0x494, 0},
    {IWDG_BASE, CRG_IP_IWDG, 0x3C0, 0},
    {QDM0_BASE, CRG_IP_NONE_CLK_SEL, 0x4C0, 0},
    {QDM1_BASE, CRG_IP_NONE_CLK_SEL, 0x4C4, 0},
    {HPM_BASE, CRG_IP_NONE_CLK_SEL, 0xB00, 0},
    {CRC_BASE, CRG_IP_NONE_CLK_SEL, 0x380, 0},
    {APT0_BASE, CRG_IP_NONE_CLK_SEL, 0x400, 0},
    {APT1_BASE, CRG_IP_NONE_CLK_SEL, 0x404, 0},
    {APT2_BASE, CRG_IP_NONE_CLK_SEL, 0x408, 0},
    {APT3_BASE, CRG_IP_NONE_CLK_SEL, 0x40C, 0},
    {CMM_BASE, CRG_IP_NONE_CLK_SEL, 0x0340, 0},
    {VREF_BASE, CRG_IP_ANA, 0xA60, 0},
    {ACMP0_BASE, CRG_IP_ANA, 0xA70, 0},
    {DAC0_BASE, CRG_IP_ANA, 0xA80, 0},
    {PGA0_BASE, CRG_IP_ANA, 0xA90, 0},
    {PGA1_BASE, CRG_IP_ANA, 0xA90, 4},

    {ADC0_BASE, CRG_IP_ADC, 0xA00, 0},

    {EFC_BASE,  CRG_IP_EFC, 0x500, 0},
};

/**
  * @brief Get IP Match Info, @see g_crgIpMatch
  * @param baseAddr The ip base address
  * @retval The Address(offset) in g_crgIpMatch if match success
  * @retval 0 if match fail
  */
CHIP_CrgIpMatchInfo *GetCrgIpMatchInfo(const void *baseAddr)
{
    unsigned int i;
    for (i = 0; i < sizeof(g_crgIpMatch) / sizeof(g_crgIpMatch[0]); ++i) {
        if (baseAddr == g_crgIpMatch[i].ipBaseAddr) {
            return (CHIP_CrgIpMatchInfo *)&g_crgIpMatch[i];
        }
    }
    return (CHIP_CrgIpMatchInfo *)0; /* The base address does not match, return 0. */
}
