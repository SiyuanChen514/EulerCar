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
  * @file      ip_crg_common.h
  * @author    MCU Driver Team
  * @brief     Contains crg ip common header files.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_IP_CRG_COMMON_H
#define McuMagicTag_IP_CRG_COMMON_H

/**
 * @brief define the frequence of hosc, losc and xtrail
 */
#define HOSC_FREQ    25000000U
#define LOSC_FREQ       32000U

#ifdef FPGA
#define FLASH_SUPPORT
typedef enum {
    CHIP_IP_CLK_LOSC = 32000U,
    CHIP_IP_CLK_CAN  = 25000000U,
#ifdef FLASH_SUPPORT
    CHIP_IP_CLK_LS   = 20000000U,
    CHIP_IP_CLK_HS   = 40000000U,
#else
    CHIP_IP_CLK_LS   = 30000000U,
    CHIP_IP_CLK_HS   = 60000000U,
#endif
} CHIP_IpRate;
#else
typedef enum {
    CHIP_IP_CLK_LOSC = 32000U,
    CHIP_IP_CLK_CAN  = 25000000U,
    CHIP_IP_CLK_LS   = 12500000U,
    CHIP_IP_CLK_HS   = 25000000U,
} CHIP_IpRate;
#endif

/**
 * @brief CRG Ip Type, Sorting based on operable registers
 */
typedef enum {
    CRG_IP_WITH_LS = 0x00,
    CRG_IP_WITH_HS = 0x01,
    CRG_IP_CAN = 0x02,
    CRG_IP_ADC = 0x03,
    CRG_IP_DAC = 0x04,
    CRG_IP_EFC = 0x05,
    CRG_IP_IWDG = 0x06,
    CRG_IP_MAX_TYPE = 0x07,
} CHIP_CrgIpType;

/**
 * @brief  CRG register and IP address matching relationship table
 */
typedef struct {
    void            *ipBaseAddr;   /**< Ip base address */
    CHIP_CrgIpType  type;          /**< Ip type, @see CHIP_CrgIpType */
    unsigned short  regOffset;     /**< Offset in CRG registers */
    unsigned char   bitOffset;     /**< Bit Offset in CRG register */
} CHIP_CrgIpMatchInfo;

unsigned int CHIP_GetIpFreqHz(const void *ipBaseAddr);
CHIP_CrgIpMatchInfo *GetCrgIpMatchInfo(const void *baseAddr);
extern unsigned int HAL_CRG_GetIpFreq(const void *baseAddress);

#endif /* McuMagicTag_IP_CRG_COMMON_H */
