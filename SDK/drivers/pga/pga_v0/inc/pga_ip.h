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
  * @file    pga_ip.h
  * @author  MCU Driver Team
  * @brief   Programmable Gain Amplifier module driver.
  *          This file provides DCL functions to manage amplifier.
  *          + Programmable Gain Amplifier register mapping strtucture.
  *          + Direct configuration layer interface.
  */

#ifndef McuMagicTag_PGA_IP_H
#define McuMagicTag_PGA_IP_H

#include "baseinc.h"

#ifdef PGA_PARAM_CHECK
#define PGA_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define PGA_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define PGA_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define PGA_ASSERT_PARAM(para) ((void)0U)
#define PGA_PARAM_CHECK_NO_RET(para) ((void)0U)
#define PGA_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

#define PGA_PGA_MAX_GAIN 7
#define PGA_PAG_MAX_SMUX 7
#define PGA_MAX_INPUT    8
#define PGA_MAX_SWITCH   3
/**
  * @addtogroup PGA
  * @{
  */

/**
  * @defgroup PGA_IP PGA_IP
  * @brief PGA_IP: pga_v0.
  * @{
  */

/**
  * @defgroup PGA_REG_Definition PGA Register Structure.
  * @brief PGA Register Structure Definition.
  * @{
  */
/**
  * @brief vin select
  */
typedef enum {
    PGA_INTER_RES_VI0 = 0x00000000U,
    PGA_INTER_RES_VI1 = 0x00000001U,
    PGA_INTER_RES_VI2 = 0x00000002U,
    PGA_INTER_RES_VI3 = 0x00000003U,
    PGA_EXT_RES_VI0   = 0x00000004U,
    PGA_EXT_RES_VI1   = 0x00000005U,
    PGA_EXT_RES_VI2   = 0x00000006U,
    PGA_EXT_RES_VI3   = 0x00000007U,
} PGA_VinMux;

/**
  * @brief PGA vin switch selection
  */
typedef enum {
    PGA_SW_VIN0 = 0x00000001U,
    PGA_SW_VIN1 = 0x00000002U,
    PGA_SW_VIN2 = 0x00000004U,
    PGA_SW_VIN3 = 0x00000008U,
} PGA_SW;

/**
  * @brief PGA gain value selection
  */
typedef enum {
    PGA_GAIN_1X =  0x00000000U,
    PGA_GAIN_2X =  0x00000001U,
    PGA_GAIN_4X =  0x00000002U,
    PGA_GAIN_8X =  0x00000003U,
    PGA_GAIN_16X = 0x00000004U,
} PGA_GainValue;

/**
 * @brief  Extent handle definition of PGA.
 */
typedef struct {
    PGA_VinMux      pgaMux;        /**< PGA Input channel. */
} PGA_ExtendHandle;

/**
  * @brief PGA control 0.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pga_ana_en   : 1;  /**< Overall enable of the PGA. */
        unsigned int pga_en_ext0  : 1;  /**< PGA external output enable. */
        unsigned int pga_en_out   : 1;  /**< PGA output enable. */
        unsigned int reserved_0   : 29;
    } BIT;
} volatile PGA_CTRL0_REG;

/**
  * @brief PGA control 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pga_trim_ofstp  : 5;   /**< PGA trim PMOS offset. */
        unsigned int pga_trim_ofstn  : 5;   /**< PGA trim NMOS offset. */
        unsigned int reserved_0      : 22;
    } BIT;
} volatile PGA_CTRL1_REG;

/**
  * @brief PGA control 2.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pga_smux   : 3;  /**< PGA input channel select. */
        unsigned int pga_gain   : 3;  /**< PGA gain. */
        unsigned int reserved_0 : 26;
    } BIT;
} volatile PGA_CTRL2_REG;

/**
  * @brief Controls PGA SW.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pga_sw_enlv_n    : 4;  /**< PGA N input SW enable, configured together with channel select. */
        unsigned int pga_sw_enlv_p    : 4;  /**< PGA P input SW enable, configured together with channel select. */
        unsigned int pga_ext_loopback : 2;  /**< PGA loopback switch. */
        unsigned int reserved_0       : 22;
    } BIT;
} volatile PGA_CTRL3_REG;

/**
  * @brief Register mapping structure.
  */
typedef struct _PGA_RegStruct {
    PGA_CTRL0_REG       PGA_CTRL0;  /**< PGA control 0 register. Offset address: 0x00000000U. */
    PGA_CTRL1_REG       PGA_CTRL1;  /**< PGA control 1 register. Offset address: 0x00000004U. */
    PGA_CTRL2_REG       PGA_CTRL2;  /**< PGA control 2 register. Offset address: 0x00000008U. */
    PGA_CTRL3_REG       PGA_CTRL3;  /**< PGA SW control register. Offset address: 0x0000000CU. */
} volatile PGA_RegStruct;

/**
  * @brief Enable amplifier's output
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_EnableOut(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    pgax->PGA_CTRL0.BIT.pga_en_out = BASE_CFG_ENABLE;
}

/**
  * @brief Disable amplifier's output
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_DisableOut(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    pgax->PGA_CTRL0.BIT.pga_en_out = BASE_CFG_DISABLE;
}

/**
  * @brief Enable amplifier's extra output
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_EnableExtOut(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    pgax->PGA_CTRL0.BIT.pga_en_ext0 = BASE_CFG_ENABLE;
}

/**
  * @brief Disable amplifier's extra output
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_DisableExtOut(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    pgax->PGA_CTRL0.BIT.pga_en_ext0 = BASE_CFG_DISABLE;
}

/**
  * @brief Enable amplifier module
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_EnableAnaOut(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    pgax->PGA_CTRL0.BIT.pga_ana_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable amplifier module
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_DisableAnaOut(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    pgax->PGA_CTRL0.BIT.pga_ana_en = BASE_CFG_DISABLE;
}

/**
  * @brief Set amplifier's gain
  * @param pgax: amplifier register base address.
  * @param value: gain value.
  * @retval None.
  */
static inline void  DCL_PGA_SetGain(PGA_RegStruct *pgax, unsigned int value)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    PGA_PARAM_CHECK_NO_RET(value <= PGA_PGA_MAX_GAIN);
    pgax->PGA_CTRL2.BIT.pga_gain = value;
}

/**
  * @brief Get amplifier's gain
  * @param pgax: amplifier register base address.
  * @retval gain value.
  */
static inline unsigned int DCL_PGA_GetGain(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    return pgax->PGA_CTRL2.BIT.pga_gain;
}

/**
  * @brief Set amplifier mux
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_SetMux(PGA_RegStruct *pgax, unsigned int value)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    PGA_PARAM_CHECK_NO_RET(value <= PGA_PAG_MAX_SMUX);
    pgax->PGA_CTRL2.BIT.pga_smux = value;
}

/**
  * @brief Get amplifier's Mux
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline unsigned int DCL_PGA_GetMux(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    return pgax->PGA_CTRL2.BIT.pga_smux;
}

/**
  * @brief Set loopback switch P
  * @param pgax: amplifier register base address.
  * @param pgaNum: number of amplifier
  * @retval None.
  */
static inline void DCL_PGA_EnablePInputByNum(PGA_RegStruct *pgax, unsigned int pgaNum)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    PGA_PARAM_CHECK_NO_RET(pgaNum < PGA_MAX_INPUT);
    pgax->PGA_CTRL3.BIT.pga_sw_enlv_p |= pgaNum;
}

/**
  * @brief Unset loopback switch
  * @param pgax: amplifier register base address.
  * @param pgaNum: number of amplifier
  * @retval None.
  */
static inline void DCL_PGA_DisablePInputByNum(PGA_RegStruct *pgax, unsigned int pgaNum)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    PGA_PARAM_CHECK_NO_RET(pgaNum < PGA_MAX_INPUT);
    pgax->PGA_CTRL3.BIT.pga_sw_enlv_p &= ~pgaNum;
}

/**
  * @brief Set loopback switch N
  * @param pgax: amplifier register base address.
  * @param pgaNum: number of amplifier
  * @retval None.
  */
static inline void DCL_PGA_EnableNInputByNum(PGA_RegStruct *pgax, unsigned int pgaNum)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    PGA_PARAM_CHECK_NO_RET(pgaNum < PGA_MAX_INPUT);
    pgax->PGA_CTRL3.BIT.pga_sw_enlv_n |= pgaNum;
}

/**
  * @brief Unset loopback switch N
  * @param pgax: amplifier register base address.
  * @param pgaNum: number of amplifier
  * @retval None.
  */
static inline void DCL_PGA_DisableNInputByNum(PGA_RegStruct *pgax, unsigned int pgaNum)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    PGA_PARAM_CHECK_NO_RET(pgaNum < PGA_MAX_INPUT);
    pgax->PGA_CTRL3.BIT.pga_sw_enlv_n &= ~pgaNum;
}

/**
  * @brief Sets the PGA loopback switch.
           Bit[8] controls channel 0 and bit[9] controls channel 3.
  * @param pgax: amplifier register base address.
  * @param switchNum: Loopback switch.
  * @retval None.
  */
static inline void DCL_PGA_SetLoopBackSwitch(PGA_RegStruct *pgax, unsigned int switchNum)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    PGA_PARAM_CHECK_NO_RET(switchNum < PGA_MAX_SWITCH);
    pgax->PGA_CTRL3.BIT.pga_ext_loopback = switchNum;
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif