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
#define PGA_MAX_GAIN_VALUE 3
#define PGA_MAX_EXT_CAP_COMP 7
#define PGA_EXT_VLAUE 3
/**
  * @addtogroup PGA
  * @{
  */

/**
  * @defgroup PGA_IP PGA_IP
  * @brief PGA_IP: pga_v1.
  * @{
  */

/**
  * @defgroup PGA_REG_Definition PGA Register Structure.
  * @brief PGA Register Structure Definition.
  * @{
  */

/**
  * @brief PGA gain value selection
  */
typedef enum {
    PGA_GAIN_2X =  0x00000000U,
    PGA_GAIN_4X =  0x00000001U,
    PGA_GAIN_8X =  0x00000002U,
    PGA_GAIN_16X = 0x00000003U,
} PGA_GainValue;

/**
  * @brief PGA gain value selection
  */
typedef enum {
    PGA_EXT_COMPENSATION_2X =  0x00000000U,
    PGA_EXT_COMPENSATION_3X =  0x00000001U,
    PGA_EXT_COMPENSATION_4X =  0x00000002U,
    PGA_EXT_COMPENSATION_5X =  0x00000003U,
    PGA_EXT_COMPENSATION_6X =  0x00000004U,
    PGA_EXT_COMPENSATION_7X =  0x00000005U,
    PGA_EXT_COMPENSATION_8X =  0x00000006U,
    PGA_EXT_COMPENSATION_9X =  0x00000007U,
} PGA_ExtCapCompValue;

/**
 * @brief  Extent handle definition of PGA.
 */
typedef struct {
    PGA_ExtCapCompValue extCapCompensation;  /**< Feedforward Capacitance Compensation in PGA External Gain Mode. */
} PGA_ExtendHandle;

/**
  * @brief PGA control 0.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int da_pga_enh   : 1;  /**< Overall enable of the PGA. */
        unsigned int reserved_0   : 31;
    } BIT;
} volatile PGA_CTRL0_REG;

/**
  * @brief PGA control 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int da_pga_cf_ctrl    : 3; /**< Feedforward capacitor compensation config in PGA external gain mode. */
        unsigned int reserved_0        : 5;
        unsigned int da_pga_gain_ctrl  : 3; /**< Gain configuration of the internal resistor of the PGA. */
        unsigned int reserved_1        : 5;
        unsigned int da_pga_mode_ctrl  : 2; /**< PGA mode configuration. 0: internal resistor gain mode;
                                                                         1: external resistor gain mode; */
        unsigned int reserved_2        : 14;
    } BIT;
} volatile PGA_CTRL1_REG;

/**
  * @brief PGA control register 2.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int da_pga_ibias_sel  : 4;  /**< PGA bias current configuration level select. */
        unsigned int reserved_0        : 28;
    } BIT;
} volatile PGA_CTRL2_REG;

/**
  * @brief PGA TRIM register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int da_pga_vos_trim  : 9;  /**< Offset trim information of the PGA. */
        unsigned int reserved_0       : 23;
    } BIT;
} volatile PGA_TRIM_REG;

/**
  * @brief PGA test register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int da_pga_test_enh    : 1;  /**< PGA test enable.  */
        unsigned int da_pga_test_sel    : 2;  /**< PGA test select. */
        unsigned int reserved_0         : 29;
    } BIT;
} volatile PGA_TEST_REG;

/**
  * @brief PGA reserved register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int da_pga_cf_ctrl1    : 2; /**< Feedforward capacitor compensation config in external gain mode. */
        unsigned int reserved_0         : 30;
    } BIT;
} volatile PGA_RSV_REG;

/**
  * @brief Register mapping structure.
  */
typedef struct _PGA_RegStruct {
    PGA_CTRL0_REG       PGA_CTRL0;    /**< PGA control 0 register. Offset address: 0x00000000U. */
    PGA_CTRL1_REG       PGA_CTRL1;    /**< PGA control 1 register. Offset address: 0x00000004U. */
    PGA_CTRL2_REG       PGA_CTRL2;    /**< PGA control 2 register. Offset address: 0x00000008U. */
    unsigned char       space0[20];
    PGA_TRIM_REG        PGA_TRIM;     /**< PGA TRIM register. Offset address: 0x00000020U. */
    unsigned char       space1[28];
    PGA_TEST_REG        PGA_TEST;     /**< PGA test control register. Offset address: 0x00000040U. */
    unsigned char       space2[28];
    PGA_RSV_REG         PGA_RSV;      /**< PGA reserved register. Offset address: 0x00000060U. */
} volatile PGA_RegStruct;

/* Parameter Check -----------------------------------------------------------*/

/**
  * @brief Verify gain value of PGA.
  * @param pgaGainValue pga gain value @ref PGA_GainValue
  * @retval true
  * @retval false
  */
static inline bool IsPgaGain(PGA_GainValue pgaGainValue)
{
    return (pgaGainValue <= PGA_MAX_GAIN_VALUE);
}

/**
  * @brief Verify feedforward capacitance compensation value.
  * @param pgaExtCapCompValue feedforward capacitance compensation value @ref PGA_ExtCapCompValue
  * @retval true
  * @retval false
  */
static inline bool IsPgaExtCapCompensation(PGA_ExtCapCompValue pgaExtCapCompValue)
{
    return (pgaExtCapCompValue <= PGA_MAX_EXT_CAP_COMP);
}

/* DCL layer -----------------------------------------------------------*/
/**
  * @brief Enable amplifier's output
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_EnableOut(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    pgax->PGA_CTRL0.BIT.da_pga_enh = BASE_CFG_ENABLE;
}

/**
  * @brief Disable amplifier's output
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_DisableOut(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    pgax->PGA_CTRL0.BIT.da_pga_enh = BASE_CFG_DISABLE;
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
    pgax->PGA_CTRL1.BIT.da_pga_gain_ctrl = value;
}

/**
  * @brief Get amplifier's gain
  * @param pgax: amplifier register base address.
  * @retval gain value.
  */
static inline unsigned int DCL_PGA_GetGain(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    return pgax->PGA_CTRL1.BIT.da_pga_gain_ctrl;
}

/**
  * @brief PGA mode configuration, enable external resistor gain mode.
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_EnableExtGainMode(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    pgax->PGA_CTRL1.BIT.da_pga_mode_ctrl = BASE_CFG_ENABLE;
}

/**
  * @brief PGA mode configuration, disable external resistor gain mode.
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_DisableExtGainMode(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    pgax->PGA_CTRL1.BIT.da_pga_mode_ctrl = BASE_CFG_DISABLE;
}

/**
  * @brief Fedforward capacitor compensation configuration in PGA external gain mode
  * @param pgax: amplifier register base address.
  * @param extValue: Configured value of the capacitor compensation.
  * @retval None.
  */
static inline void DCL_PGA_SetExtCompensation(PGA_RegStruct *pgax, unsigned short extValue)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    PGA_PARAM_CHECK_NO_RET(extValue <= PGA_PGA_MAX_GAIN);
    pgax->PGA_CTRL1.BIT.da_pga_cf_ctrl = extValue;
}

/**
  * @brief PGA enable Test mode.
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_EnableTestMode(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    pgax->PGA_TEST.BIT.da_pga_test_enh = BASE_CFG_ENABLE;
}

/**
  * @brief PGA disable Test mode.
  * @param pgax: amplifier register base address.
  * @retval None.
  */
static inline void DCL_PGA_DisableTestMode(PGA_RegStruct *pgax)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    pgax->PGA_TEST.BIT.da_pga_test_enh = BASE_CFG_DISABLE;
}

/**
  * @brief Set feedforward capacitance compensation in external gain mode.
  * @param pgax: amplifier register base address.
  * @param extBigvalue  feedforward capacitance compensation.
  * @retval None.
  * @note   To configure this register, must set da_pga_cf_ctrl to 111.
  */
static inline void DCL_PGA_SetExtCapCompValue(PGA_RegStruct *pgax, unsigned short extBigvalue)
{
    PGA_ASSERT_PARAM(IsPGAInstance(pgax));
    PGA_PARAM_CHECK_NO_RET(extBigvalue <= PGA_EXT_VLAUE);
    pgax->PGA_RSV.BIT.da_pga_cf_ctrl1 = extBigvalue;
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
