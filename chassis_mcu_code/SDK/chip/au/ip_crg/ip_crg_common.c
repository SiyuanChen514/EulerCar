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
    void *highRateIp[] = {
        SYSCTRL1_BASE,
        CRC_BASE,
        APT0_BASE, APT1_BASE, APT2_BASE, APT3_BASE, APT4_BASE, APT5_BASE, APT6_BASE, APT7_BASE, APT8_BASE,
        CAPM0_BASE, CAPM1_BASE, CAPM2_BASE, CAPM_COMM_BASE,
        QDM0_BASE,
        ADC0_BASE, ADC1_BASE, ADC2_BASE,
        PGA0_BASE, PGA1_BASE, PGA2_BASE,
        ACMP0_BASE, ACMP1_BASE, ACMP2_BASE,
    };

    if (ipBaseAddr == IWDG_BASE) {
        return CHIP_IP_CLK_LOSC;
    } else if (ipBaseAddr == CAN_BASE) {
        return CHIP_IP_CLK_CAN;
    } else {
        for (unsigned int i = 0; i < sizeof(highRateIp) / sizeof(highRateIp[0]); ++i) {
            if (ipBaseAddr == highRateIp[i]) {
                return CHIP_IP_CLK_HS;
            }
        }
        return CHIP_IP_CLK_LS;
    }
}
#endif

static const CHIP_CrgIpMatchInfo g_crgIpMatch[] = {
    {UART0_BASE, CRG_IP_WITH_LS, 0x30, 0},
    {UART1_BASE, CRG_IP_WITH_LS, 0x34, 0},
    {UART2_BASE, CRG_IP_WITH_LS, 0x38, 0},
    {TIMER0_BASE, CRG_IP_WITH_LS, 0x3C, 0},
    {TIMER1_BASE, CRG_IP_WITH_LS, 0x3C, 0},
    {TIMER2_BASE, CRG_IP_WITH_LS, 0x40, 0},
    {SYSTICK_BASE, CRG_IP_WITH_LS, 0x40, 0},
    {GPT0_BASE, CRG_IP_WITH_LS, 0x44, 0},
    {GPT1_BASE, CRG_IP_WITH_LS, 0x48, 0},
    {WDG_BASE, CRG_IP_WITH_LS, 0x4C, 0},
    {SPI_BASE, CRG_IP_WITH_LS, 0x50, 0},
    {CAN_BASE, CRG_IP_CAN, 0x54, 0},
    {CAPM0_BASE, CRG_IP_WITH_HS, 0x58, 0},
    {CAPM1_BASE, CRG_IP_WITH_HS, 0x58, 1},
    {CAPM2_BASE, CRG_IP_WITH_HS, 0x58, 2},
    {DMA_BASE, CRG_IP_WITH_HS, 0x5C, 0},
    {GPIO0_BASE, CRG_IP_WITH_HS, 0x64, 0},
    {GPIO1_BASE, CRG_IP_WITH_HS, 0x64, 1},
    {GPIO2_BASE, CRG_IP_WITH_HS, 0x64, 2},
    {GPIO3_BASE, CRG_IP_WITH_HS, 0x64, 3},
    {GPIO4_BASE, CRG_IP_WITH_HS, 0x64, 4},
    {GPIO5_BASE, CRG_IP_WITH_HS, 0x64, 5},
    {GPIO6_BASE, CRG_IP_WITH_HS, 0x64, 6},
    {GPIO7_BASE, CRG_IP_WITH_HS, 0x64, 7},
    {I2C_BASE, CRG_IP_WITH_HS, 0x68, 0},
    {IWDG_BASE, CRG_IP_IWDG, 0x6C, 0},
    {QDM0_BASE, CRG_IP_WITH_HS, 0x70, 0},
    {HPM_BASE, CRG_IP_WITH_HS, 0x74, 0},
    {CRC_BASE, CRG_IP_WITH_HS, 0x7C, 0},
    {APT0_BASE, CRG_IP_WITH_HS, 0x80, 0},
    {APT1_BASE, CRG_IP_WITH_HS, 0x80, 1},
    {APT2_BASE, CRG_IP_WITH_HS, 0x80, 2},
    {APT3_BASE, CRG_IP_WITH_HS, 0x80, 3},
    {APT4_BASE, CRG_IP_WITH_HS, 0x80, 4},
    {APT5_BASE, CRG_IP_WITH_HS, 0x80, 5},
    {APT6_BASE, CRG_IP_WITH_HS, 0x80, 6},
    {APT7_BASE, CRG_IP_WITH_HS, 0x80, 7},
    {APT8_BASE, CRG_IP_WITH_HS, 0x80, 8},
    {ACMP0_BASE, CRG_IP_WITH_HS, 0x90, 0},
    {ACMP1_BASE, CRG_IP_WITH_HS, 0x90, 1},
    {ACMP2_BASE, CRG_IP_WITH_HS, 0x90, 2},
    {PGA0_BASE, CRG_IP_WITH_HS, 0x98, 0},
    {PGA1_BASE, CRG_IP_WITH_HS, 0x98, 1},
    {PGA2_BASE, CRG_IP_WITH_HS, 0x98, 2},

    {ADC0_BASE, CRG_IP_ADC, 0x84, 0},
    {ADC1_BASE, CRG_IP_ADC, 0x88, 0},
    {ADC2_BASE, CRG_IP_ADC, 0x8C, 0},

    {DAC0_BASE, CRG_IP_DAC, 0x94, 0},
    {DAC1_BASE, CRG_IP_DAC, 0x94, 1},
    {DAC2_BASE, CRG_IP_DAC, 0x94, 2},

    {EFC_BASE,  CRG_IP_EFC, 0x60, 0},
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
    return (CHIP_CrgIpMatchInfo *)0;
}
