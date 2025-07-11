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
 * @file      crg_ip.h
 * @author    MCU Driver Team
 * @brief     TIMER module driver.
 * @details   This file provides firmware functions to manage the following
 *            functionalities of the TIMER.
 *                + CRG register mapping structure
 *                + Direct Configuration Layer functions of CRG
 */
#ifndef McuMagicTag_CRG_IP_H
#define McuMagicTag_CRG_IP_H

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"

/**
  * @addtogroup CRG
  * @{
  */

/**
  * @defgroup CRG_IP CRG_IP
  * @brief CRG_IP: crg_v1
  * @{
  */

/**
 * @defgroup CRG_Param_Def CRG Parameters Definition
 * @brief Definition of CRG configuration parameters.
 * @{
 */
#ifdef  CRG_PARAM_CHECK
#define CRG_ASSERT_PARAM          BASE_FUNC_ASSERT_PARAM
#define CRG_PARAM_CHECK_NO_RET    BASE_FUNC_PARAMCHECK_NO_RET
#define CRG_PARAM_CHECK_WITH_RET  BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define CRG_ASSERT_PARAM(para)               ((void)0U)
#define CRG_PARAM_CHECK_NO_RET(para)         ((void)0U)
#define CRG_PARAM_CHECK_WITH_RET(para, ret)  ((void)0U)
#endif

#define IP_CLK_DISABLE               0x00000000U   /**< IP Clock disable bitmask */
#define IP_CLK_ENABLE                0x00000001U   /**< IP Clock disable bitmask */
#define HPM_CLK_ENABLE               0x00000001U   /**< HPM Clock enable bitmask */
#define HPM_1M_CLK_ENABLE            0x00000002U   /**< HPM 1M Clock enable bitmask */
#define IP_SYSCLK_ENABLE             0x00000002U   /**< IP SysClock disable bitmask, Only valid for ADC */

#define DAC_DIV_BITLEN               4U            /**< DIV bit length */
#define DAC_DIV_MASK ((1 << DAC_DIV_BITLEN) - 1)   /**< DAC div mask, base on the bit length */

#define ADC_DIV_FACTOR          (1 << 1)           /**< ADC div min factor */
#define CRG_1MHZ_CLK_MAX_DIV    63
#define CRG_FREQ_1MHz           (1000 * 1000)
#define CRG_CLK_PFD_MIN_FREQ    (4 * CRG_FREQ_1MHz)
#define CRG_CLK_PFD_MAX_FREQ    (75 * CRG_FREQ_1MHz / 10)
#define CRG_CLK_VCO_MIN_FREQ    (100 * CRG_FREQ_1MHz)
#define CRG_CLK_VCO_MAX_FREQ    (300 * CRG_FREQ_1MHz)
#define CRG_CLK_TARGET_MAX_FREQ (150 * CRG_FREQ_1MHz)
#define CRG_CLK_PST2_MAX_FREQ   (100 * CRG_FREQ_1MHz)
/**
 * @brief PLL refer clock Select
 */
typedef enum {
    CRG_PLL_REF_CLK_SELECT_HOSC = 0,
    CRG_PLL_REF_CLK_SELECT_XTAL = 1,
} CRG_PllRefClkSelect;

/**
 * @brief PLL previous divsion value in register
 */
typedef enum {
    CRG_PLL_PREDIV_1 = 0,
    CRG_PLL_PREDIV_2 = 1,
    CRG_PLL_PREDIV_3 = 2,
    CRG_PLL_PREDIV_4 = 3,
    CRG_PLL_PREDIV_5 = 4,
    CRG_PLL_PREDIV_6 = 5,
    CRG_PLL_PREDIV_7 = 6,
    CRG_PLL_PREDIV_8 = 7,
} CRG_PllPreDiv;

/**
 * @brief PLL previous divison value in Calc frequency
 */
typedef enum {
    PLL_PREDIV_OUT_1 = 1,
    PLL_PREDIV_OUT_2 = 2,
    PLL_PREDIV_OUT_3 = 3,
    PLL_PREDIV_OUT_4 = 4,
    PLL_PREDIV_OUT_5 = 5,
    PLL_PREDIV_OUT_6 = 6,
    PLL_PREDIV_OUT_7 = 7,
    PLL_PREDIV_OUT_8 = 8,
} PLL_PreDivOut;

/**
 * @brief PLL post division 1 value in register
 */
typedef enum {
    CRG_PLL_POSTDIV_1 = 0,
    CRG_PLL_POSTDIV_2 = 1,
    CRG_PLL_POSTDIV_3 = 2,
    CRG_PLL_POSTDIV_4 = 3,
    CRG_PLL_POSTDIV_5 = 4,
    CRG_PLL_POSTDIV_6 = 5,
    CRG_PLL_POSTDIV_7 = 6,
    CRG_PLL_POSTDIV_8 = 7,
} CRG_PllPostDiv;

/**
 * @brief PLL post division 2 value in register
 */
typedef enum {
    CRG_PLL_POSTDIV2_1 = 0,
    CRG_PLL_POSTDIV2_2 = 1,
    CRG_PLL_POSTDIV2_3 = 2,
    CRG_PLL_POSTDIV2_4 = 3,
    CRG_PLL_POSTDIV2_5 = 4,
    CRG_PLL_POSTDIV2_6 = 5,
    CRG_PLL_POSTDIV2_7 = 6,
    CRG_PLL_POSTDIV2_8_MAX = 7,
} CRG_PllPostDiv2;


/**
 * @brief Core clock selection
 * @note  default select HOSC
 */
typedef enum {
    CRG_CORE_CLK_SELECT_HOSC = 0,
    CRG_CORE_CLK_SELECT_TCXO = 1,
    CRG_CORE_CLK_SELECT_PLL  = 2,
} CRG_CoreClkSelect;

/**
 * @brief Core clock selection 2
 * @note  default select HOSC
 */
typedef enum {
    CRG_CORE_CLK2_SELECT_HOSC = 0,
    CRG_CORE_CLK2_SELECT_TCXO = 1,
    CRG_CORE_CLK2_SELECT_PLL  = 2,
} CRG_CoreClkSelect2;

/**
 * @brief 1M clock selection
 * @note  default select HOSC
 */
typedef enum {
    CRG_1M_CLK_SELECT_HOSC = 0,
    CRG_1M_CLK_SELECT_TCXO = 1,
} CRG_1MClkSelect;

/**
 * @brief PLL frequency multiplication range
 */
typedef enum {
    CRG_PLL_FBDIV_MIN  = 6,
    CRG_PLL_FBDIV_MAX  = 127,
} CRG_PllFbDivRange;

/**
 * @brief PLL diagnose post div selection
 */
typedef enum {
    CRG_PLL_DIG_POST_DIV_SELECT_FREF = 0,
    CRG_PLL_DIG_POST_DIV_SELECT_PLL  = 1,
} CRG_PllDigPostDivInSelect;

/**
 * @brief PLL diagnose loct detect lpsel
 */
typedef enum {
    CRG_PLL_DIG_LOCKDET_LP_SELECT_2048 = 0,
    CRG_PLL_DIG_LOCKDET_LP_SELECT_1024 = 1,
    CRG_PLL_DIG_LOCKDET_LP_SELECT_512  = 2,
    CRG_PLL_DIG_LOCKDET_LP_SELECT_256  = 3,
} CRG_PllDigLockDetLpSelect;

/**
 * @brief PLL Test selection
 */
typedef enum {
    CRG_PLL_TEST_SELECT_FPFD   = 0,
    CRG_PLL_TEST_SELECT_CKFB   = 1,
    CRG_PLL_TEST_SELECT_LOCKDET_OUTPUT   = 2,
    CRG_PLL_TEST_SELECT_FOUTPOSTDIV_128  = 3,
    CRG_PLL_TEST_SELECT_OUTPUT_0 = 4,
} CRG_PllDigTestSelect;

/**
 * @brief CRG Test Clock Select
 */
typedef enum {
    CRG_TEST_CLK_PLL_PFD         = 0x00000000U,
    CRG_TEST_CLK_HOSC            = 0x00000001U,
    CRG_TEST_CLK_LOSC            = 0x00000002U,
    CRG_TEST_CLK_TCXO            = 0x00000003U,
    CRG_TEST_CLK_BG_CHOPPER      = 0x00000004U,
    CRG_TEST_CLK_ADC_DIV4        = 0x00000005U,
    CRG_TEST_CLK_HCLK_DIV6       = 0x00000006U,
    CRG_TEST_CLK_HOSC_DIV        = 0x00000007U,
} CRG_TestClkSel;

/**
  * @brief Source clock selection of ip in AHB
  */
typedef enum {
    CRG_AHB_CLK_NO_PREDV   = 0,
    CRG_AHB_CLK_PREDIV_2   = 1,
    CRG_AHB_CLK_PREDIV_4   = 2,
    CRG_AHB_CLK_PREDIV_8   = 3,
} CRG_AHBClkSelect;

/**
 * @brief ADC source clock select
 */
typedef enum {
    CRG_ADC_CLK_ASYN_HOSC = 0,
    CRG_ADC_CLK_ASYN_TCXO = 1,
    CRG_ADC_CLK_ASYN_PLL_DIV = 2,
    CRG_ADC_CLK_SYN_CORE = 3,
} CRG_AdcClkSelect;

/**
 * @brief ADC synchronous and asynchronous clock source selection
 */
typedef enum {
    CRG_ADC_CLK_ASYNCHRONOUS = 0,
    CRG_ADC_CLK_SYNCHRONOUS = 1,
} CRG_AdcClkModeSelect;

/**
 * @brief ADC Div set Value
 */
typedef enum {
    CRG_ADC_DIV_1 = 0,
    CRG_ADC_DIV_2 = 1,
    CRG_ADC_DIV_3 = 2,
    CRG_ADC_DIV_4 = 3,
} CRG_AdcDiv;

/**
  * @brief CRG Extra Handle, include CRG's other config
  */
typedef struct {
    CRG_PllPostDiv2     pllPostDiv2;      /**< PLL post 2  ratio */
    CRG_1MClkSelect     clk1MSelect;      /**< 1M clock selection */
    unsigned int        clk1MDiv;         /**< 1M clock ratio */
} CRG_ExtendHandle;

/**
 * @brief PLL Divison Config
 */
typedef struct {
    unsigned int PreDiv;
    unsigned int fbDiv;
    unsigned int postDiv;
} CRG_PllDivCfg;

/**
 * @brief APB_LS_SUBSYS IP config
 */
typedef union {
    unsigned int  value;
    struct {
        unsigned int    cken           : 1;
        unsigned int    reserved       : 15;
        unsigned int    srst_req       : 1;
        unsigned int    reserved1      : 7;
        unsigned int    cksel          : 2;
        unsigned int    reserved2      : 6;
    } BIT;
} volatile CRG_IpWithClkSelectCfg;

/**
 * @brief APB_HS_SUBSYS IP config
 */
typedef union {
    unsigned int  value;
    struct {
        unsigned int  clkEnMask    : 16;
        unsigned int  softResetReq : 16;
    } BIT;
} volatile CRG_IpWoClkSelectCfg;

/**
 * @brief ADC config
 * @see   PERI_CRG41_Reg and PERI_CRG42_Reg and PERI_CRG43_Reg
 */
typedef union {
    unsigned int value[2];
    struct {
        unsigned int    eflash_cken           : 1;
        unsigned int    reserved              : 31;
        unsigned int    eflash_clk_tick_cksel : 1;
        unsigned int    reserved1             : 31;
    } BIT;
} volatile CRG_EfcIpCfg;

typedef union {
    unsigned int  value[2];
    struct {
        unsigned int    clk_adc_div0       : 2;
        unsigned int    reserved           : 6;
        unsigned int    clk_adc_div1       : 2;
        unsigned int    reserved1          : 22;
        unsigned int    clk_adc_cken       : 1;
        unsigned int    reserved2          : 15;
        unsigned int    adc_srst_req       : 1;
        unsigned int    reserved3          : 7;
        unsigned int    cfg_adc_ckmode_sel : 1;
        unsigned int    reserved4          : 7;
    } BIT;
} volatile CRG_AdcIpCfg;

/**
 * @brief DAC config
 * @see   PERI_CRG45_Reg
 */
typedef union {
    unsigned int  value;
    struct {
        unsigned int    clkEnMask    : 3;
        unsigned int    reserved_0   : 1;
        unsigned int    div          : 12;
        unsigned int    softResetReq : 3;
        unsigned int    reserved_1   : 13;
    } BIT;
} volatile CRG_DacIpCfg;

/**
 * @brief ANA config
 * @see   PERI_CRG664_Reg - PERI_CRG677_Reg
 */
typedef union {
    unsigned int  value;
    struct {
        unsigned int    reserved     : 16;
        unsigned int    ip_srst_req  : 1;
        unsigned int    reserved1    : 15;
    } BIT;
} volatile CRG_AnaIpCfg;

/**
 * @brief IP match info for ip process
 */
typedef struct {
    void         *baseAddr;    /**< Base address of ip */
    unsigned int  offset;      /**< The offset in CRG_RegStruct */
    unsigned int  idx;         /**< index in Reg, for example: 0 -capm0_cken 1 - capm1_cken in PERI_CRG30_Reg */
} CRG_IpMatchInfo;

/**
  * @}
  */

/**
  * @brief CRG REG0 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_ref_cksel : 1;  /**< pll reference select */
        unsigned int    reserved      : 31;
    } BIT;
} volatile PERI_CRG0_REG;

/**
  * @brief CRG REG1 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_prediv : 4; /**< predivider value */
        unsigned int    reserved   : 28;
    } BIT;
} volatile PERI_CRG1_REG;

/**
  * @brief CRG REG2 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_fbdiv : 8; /**< feedback divider value */
        unsigned int    reserved  : 24;
    } BIT;
} volatile PERI_CRG2_REG;

/**
  * @brief CRG REG3 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_postdiv1 : 4; /**< post divider value */
        unsigned int    pll_postdiv2 : 4;
        unsigned int    reserved     : 24;
    } BIT;
} volatile PERI_CRG3_REG;

/**
  * @brief CRG REG4 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_pd   : 1; /**< pll power down */
        unsigned int    reserved : 31;
    } BIT;
} volatile PERI_CRG4_REG;

/**
  * @brief CRG REG 7 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_lock : 1; /**< pll lock flag */
        unsigned int    reserved : 31;
    } BIT;
} volatile PERI_CRG7_REG;

/**
  * @brief CRG REG 8 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_lock_deglitch : 1; /**< pll out without deburring */
        unsigned int    reserved          : 31;
    } BIT;
} volatile PERI_CRG8_REG;

/**
  * @brief CRG REG64 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_pst1_sw_sel : 2; /**< hclk_sw_sel select */
        unsigned int    reserved        : 2;
        unsigned int    clk_pst2_sw_sel : 2; /**< clk_pst2_sw_sel select */
        unsigned int    reserved1       : 2;
        unsigned int    reserved2       : 24;
    } BIT;
} volatile PERI_CRG64_REG;

/**
  * @brief CRG REG65 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pvd_rst_enable : 1; /**< pvd reset enable */
        unsigned int    reserved       : 3;
        unsigned int    reserved1      : 28;
    } BIT;
} volatile PERI_CRG65_REG;

/**
  * @brief CRG REG66 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_1m_ini_cksel : 1; /**< clock 1M selection */
        unsigned int    reserved         : 3;
        unsigned int    reserved1        : 28;
    } BIT;
} volatile PERI_CRG66_REG;

/**
  * @brief CRG REG registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_1m_div : 6; /**< clock 1M divide */
        unsigned int    reserved   : 2;
        unsigned int    reserved1  : 24;
    } BIT;
} volatile PERI_CRG67_REG;

/**
  * @brief CRG REG80 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    uart0_cken     : 1; /**< uart0 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    uart0_srst_req : 1; /**< uart0 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    uart0_cksel    : 2; /**< uart0 clock select */
        unsigned int    reserved2      : 2;
        unsigned int    reserved3      : 4;
    } BIT;
} volatile PERI_CRG80_REG;

/**
  * @brief CRG REG81 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    uart1_cken     : 1; /**< uart1 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    uart1_srst_req : 1; /**< uart1 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    uart1_cksel    : 2; /**< uart1 clock select */
        unsigned int    reserved2      : 2;
        unsigned int    reserved3      : 4;
    } BIT;
} volatile PERI_CRG81_REG;

/**
  * @brief CRG REG82 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    uart2_cken     : 1; /**< uart2 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    uart2_srst_req : 1; /**< uart2 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    uart2_cksel    : 2; /**< uart2 clock select */
        unsigned int    reserved2      : 2;
        unsigned int    reserved3      : 4;
    } BIT;
} volatile PERI_CRG82_REG;

/**
  * @brief CRG REG83 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    uart3_cken     : 1; /**< uart3 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    uart3_srst_req : 1; /**< uart3 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    uart3_cksel    : 2; /**< uart3 clock select */
        unsigned int    reserved2      : 2;
        unsigned int    reserved3      : 4;
    } BIT;
} volatile PERI_CRG83_REG;

/**
  * @brief CRG REG96 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    spi0_cken     : 1;  /**< spi0 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    spi0_srst_req : 1;  /**< spi0 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    spi0_cksel    : 2;  /**< spi0 clock select */
        unsigned int    reserved2     : 2;
        unsigned int    reserved3     : 4;
    } BIT;
} volatile PERI_CRG96_REG;

/**
  * @brief CRG REG97 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    spi1_cken     : 1; /**< spi1 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    spi1_srst_req : 1; /**< spi1 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    spi1_cksel    : 2; /**< spi1 clock select */
        unsigned int    reserved2     : 2;
        unsigned int    reserved3     : 4;
    } BIT;
} volatile PERI_CRG97_REG;

/**
  * @brief CRG REG112 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    i2c0_cken     : 1; /**< i2c0 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    i2c0_srst_req : 1; /**< i2c0 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    i2c0_cksel    : 2; /**< i2c0 clock select */
        unsigned int    reserved2     : 2;
        unsigned int    reserved3     : 4;
    } BIT;
} volatile PERI_CRG112_REG;

/**
  * @brief CRG REG113 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    i2c1_cken     : 1;  /**< i2c1 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    i2c1_srst_req : 1;  /**< i2c1 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    i2c1_cksel    : 2;  /**< i2c1 clock select */
        unsigned int    reserved2     : 2;
        unsigned int    reserved3     : 4;
    } BIT;
} volatile PERI_CRG113_REG;

/**
  * @brief CRG REG128 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    wwdg_cken     : 1;  /**< wwdg clock enable */
        unsigned int    reserved      : 15;
        unsigned int    wwdg_srst_req : 1;  /**< wwdg reset request */
        unsigned int    reserved1     : 7;
        unsigned int    wwdg_cksel    : 2;  /**< wwdg clock select */
        unsigned int    reserved2     : 2;
        unsigned int    reserved3     : 4;
    } BIT;
} volatile PERI_CRG128_REG;

/**
  * @brief CRG REG144 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    timer0_cken     : 1; /**< timer0 clock enable */
        unsigned int    reserved        : 15;
        unsigned int    timer0_srst_req : 1; /**< timer0 reset request */
        unsigned int    reserved1       : 7;
        unsigned int    timer0_cksel    : 2; /**< timer0 clock select */
        unsigned int    reserved2       : 2;
        unsigned int    reserved3       : 4;
    } BIT;
} volatile PERI_CRG144_REG;

/**
  * @brief CRG REG145 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    timer1_cken     : 1; /**< timer1 clock enable */
        unsigned int    reserved        : 15;
        unsigned int    timer1_srst_req : 1; /**< timer1 reset request */
        unsigned int    reserved1       : 7;
        unsigned int    timer1_cksel    : 2; /**< timer1 clock select */
        unsigned int    reserved2       : 2;
        unsigned int    reserved3       : 4;
    } BIT;
} volatile PERI_CRG145_REG;

/**
  * @brief CRG REG146 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    timer2_cken     : 1; /**< timer2 clock enable */
        unsigned int    reserved        : 15;
        unsigned int    timer2_srst_req : 1; /**< timer2 reset request */
        unsigned int    reserved1       : 7;
        unsigned int    timer2_cksel    : 2; /**< timer2 clock select */
        unsigned int    reserved2       : 2;
        unsigned int    reserved3       : 4;
    } BIT;
} volatile PERI_CRG146_REG;

/**
  * @brief CRG REG147 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    timer3_cken     : 1; /**< timer3 clock enable */
        unsigned int    reserved        : 15;
        unsigned int    timer3_srst_req : 1; /**< timer3 reset request */
        unsigned int    reserved1       : 7;
        unsigned int    timer3_cksel    : 2; /**< timer3 clock select */
        unsigned int    reserved2       : 2;
        unsigned int    reserved3       : 4;
    } BIT;
} volatile PERI_CRG147_REG;

/**
  * @brief CRG REG160 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    capm0_cken     : 1; /**< capm0 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    capm0_srst_req : 1; /**< capm1 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    reserved2      : 8;
    } BIT;
} volatile PERI_CRG160_REG;

/**
  * @brief CRG REG161 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    capm1_cken     : 1; /**< capm1 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    capm1_srst_req : 1; /**< capm1 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    reserved2      : 8;
    } BIT;
} volatile PERI_CRG161_REG;

/**
  * @brief CRG REG162 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    capm2_cken     : 1; /**< capm2 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    capm2_srst_req : 1; /**< capm2 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    reserved2      : 8;
    } BIT;
} volatile PERI_CRG162_REG;

/**
  * @brief CRG REG176 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    can_cken     : 1; /**< can clock enable */
        unsigned int    reserved     : 15;
        unsigned int    can_srst_req : 1; /**< can reset request */
        unsigned int    reserved1    : 7;
        unsigned int    reserved2    : 8;
    } BIT;
} volatile PERI_CRG176_REG;

/**
  * @brief CRG REG192 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    dma_cken     : 1; /**< dma clock enable */
        unsigned int    reserved     : 15;
        unsigned int    dma_srst_req : 1; /**< dma reset request */
        unsigned int    reserved1    : 7;
        unsigned int    reserved2    : 8;
    } BIT;
} volatile PERI_CRG192_REG;

/**
  * @brief CRG REG208 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    cmm_cken     : 1; /**< cmm clock enable */
        unsigned int    reserved     : 15;
        unsigned int    cmm_srst_req : 1; /**< cmm reset request */
        unsigned int    reserved1    : 7;
        unsigned int    reserved2    : 8;
    } BIT;
} volatile PERI_CRG208_REG;

/**
  * @brief CRG REG224 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    crc_cken     : 1; /**< crc clock enable */
        unsigned int    reserved     : 15;
        unsigned int    crc_srst_req : 1; /**< crc reset request */
        unsigned int    reserved1    : 7;
        unsigned int    reserved2    : 8;
    } BIT;
} volatile PERI_CRG224_REG;

/**
  * @brief CRG REG240 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    iwdg_cken     : 1; /**< iwdg clock enable */
        unsigned int    reserved      : 15;
        unsigned int    iwdg_srst_req : 1; /**< iwdg reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG240_REG;

/**
  * @brief CRG REG256 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt0_cken     : 1; /**< apt0 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    apt0_srst_req : 1; /**< apt0 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG256_REG;

/**
  * @brief CRG REG257 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt1_cken     : 1; /**< apt1 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    apt1_srst_req : 1; /**< apt1 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG257_REG;

/**
  * @brief CRG REG258 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt2_cken     : 1; /**< apt2 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    apt2_srst_req : 1; /**< apt2 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG258_REG;

/**
  * @brief CRG REG259 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt3_cken     : 1; /**< apt3 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    apt3_srst_req : 1; /**< apt3 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG259_REG;

/**
  * @brief CRG REG272 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpt0_cken     : 1; /**< gpt0 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    gpt0_srst_req : 1; /**< gpt0 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG272_REG;

/**
  * @brief CRG REG273 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpt1_cken     : 1; /**< gpt1 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    gpt1_srst_req : 1; /**< gpt1 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG273_REG;

/**
  * @brief CRG REG274 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpt2_cken     : 1; /**< gpt2 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    gpt2_srst_req : 1; /**< gpt2 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG274_REG;

/**
  * @brief CRG REG275 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpt3_cken     : 1;  /**< gpt3 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    gpt3_srst_req : 1;  /**< gpt3 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG275_REG;

/**
  * @brief CRG REG288 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio0_cken     : 1; /**< gpio0 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    gpio0_srst_req : 1; /**< gpio0 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    reserved2      : 8;
    } BIT;
} volatile PERI_CRG288_REG;

/**
  * @brief CRG REG289 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio1_cken     : 1; /**< gpio1 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    gpio1_srst_req : 1; /**< gpio1 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    reserved2      : 8;
    } BIT;
} volatile PERI_CRG289_REG;

/**
  * @brief CRG REG290 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio2_cken     : 1; /**< gpio2 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    gpio2_srst_req : 1; /**< gpio2 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    reserved2      : 8;
    } BIT;
} volatile PERI_CRG290_REG;

/**
  * @brief CRG REG291 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio3_cken     : 1; /**< gpio3 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    gpio3_srst_req : 1; /**< gpio3 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    reserved2      : 8;
    } BIT;
} volatile PERI_CRG291_REG;

/**
  * @brief CRG REG292 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio4_cken     : 1; /**< gpio4 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    gpio4_srst_req : 1; /**< gpio4 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    reserved2      : 8;
    } BIT;
} volatile PERI_CRG292_REG;

/**
  * @brief CRG REG293 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio5_cken     : 1; /**< gpio5 clock enable */
        unsigned int    reserved       : 15;
        unsigned int    gpio5_srst_req : 1; /**< gpio5 reset request */
        unsigned int    reserved1      : 7;
        unsigned int    reserved2      : 8;
    } BIT;
} volatile PERI_CRG293_REG;

/**
  * @brief CRG REG304 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    qdm0_cken     : 1; /**< qdm0 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    qdm0_srst_req : 1; /**< qdm0 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG304_REG;

/**
  * @brief CRG REG305 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    qdm1_cken     : 1; /**< qdm1 clock enable */
        unsigned int    reserved      : 15;
        unsigned int    qdm1_srst_req : 1; /**< qdm1 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG305_REG;

/**
  * @brief CRG REG320 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    eflash_cken : 1; /**< eflash clock enable */
        unsigned int    reserved    : 15;
        unsigned int    reserved1   : 16;
    } BIT;
} volatile PERI_CRG320_REG;

/**
  * @brief CRG REG639 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    test_clk_en  : 1; /**< test clock enable */
        unsigned int    reserved     : 15;
        unsigned int    test_clk_sel : 3; /**< test clock select */
        unsigned int    reserved1    : 13;
    } BIT;
} volatile PERI_CRG639_REG;

/**
  * @brief CRG REG640 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_adc_div0 : 2; /**< adc divide register 0 */
        unsigned int    reserved     : 6;
        unsigned int    clk_adc_div1 : 2; /**< adc divide register 1 */
        unsigned int    reserved1   : 22;
    } BIT;
} volatile PERI_CRG640_REG;

/**
  * @brief CRG REG641 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    adc_cken     : 1; /**< adc clock enable */
        unsigned int    reserved     : 15;
        unsigned int    adc_srst_req : 1; /**< adc reset request */
        unsigned int    reserved1    : 7;
        unsigned int    adc_clk_mode : 1; /**< adc clock selection */
        unsigned int    reserved2    : 7;
    } BIT;
} volatile PERI_CRG641_REG;

/**
  * @brief CRG REG656 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_bg_chopper_div        : 6;  /**< clk_bg_chopper divide factor */
        unsigned int    reserved                  : 2;
        unsigned int    clk_bg_chopper_div_bypass : 1;  /**< clk_bg_chopper divide bypass signal */
        unsigned int    reserved1                 : 23;
    } BIT;
} volatile PERI_CRG656_REG;

/**
  * @brief CRG REG660 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_ana_cken : 1; /**< anolog clock enable */
        unsigned int    reserved     : 15;
        unsigned int    ana_srst_req : 1; /**< anolog reset request */
        unsigned int    reserved1    : 7;
        unsigned int    reserved2    : 8;
    } BIT;
} volatile PERI_CRG660_REG;

/**
  * @brief CRG REG664 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved      : 16;
        unsigned int    vref_srst_req : 1; /**< VREF reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG664_REG;

/**
  * @brief CRG REG668 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved      : 16;
        unsigned int    acmp_srst_req : 1; /**< acmp reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG668_REG;

/**
  * @brief CRG REG672 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved     : 16;
        unsigned int    dac_srst_req : 1; /**< dac reset request */
        unsigned int    reserved1    : 7;
        unsigned int    reserved2    : 8;
    } BIT;
} volatile PERI_CRG672_REG;

/**
  * @brief CRG REG676 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved      : 16;
        unsigned int    pga0_srst_req : 1; /**< pga0 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG676_REG;

/**
  * @brief CRG REG677 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved      : 16;
        unsigned int    pga1_srst_req : 1; /**< pga1 reset request */
        unsigned int    reserved1     : 7;
        unsigned int    reserved2     : 8;
    } BIT;
} volatile PERI_CRG677_REG;


/**
  * @brief HOSC CTRL4 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    hosc_pd  : 1; /**< HOSC power down enable bit */
        unsigned int    reserved : 31;
    } BIT;
} volatile HOSC_PD_REG;

/**
  * @brief HOSC CTRL5 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    hosc_lock : 1; /**< HOSC lock signal */
        unsigned int    reserved  : 31;
    } BIT;
} volatile HOSC_LOCK_REG;

typedef struct {
    PERI_CRG0_REG      PERI_CRG0; /**< CRG0 register. Offset address 0x00000000U. */
    PERI_CRG1_REG      PERI_CRG1; /**< CRG1 register. Offset address 0x00000004U. */
    PERI_CRG2_REG      PERI_CRG2; /**< CRG2 register. Offset address 0x00000008U. */
    PERI_CRG3_REG      PERI_CRG3; /**< CRG3 register. Offset address 0x0000000CU. */
    PERI_CRG4_REG      PERI_CRG4; /**< CRG4 register. Offset address 0x00000010U. */
    unsigned char      reserved0[8];
    PERI_CRG7_REG      PERI_CRG7; /**< CRG7 register. Offset address 0x0000001CU. */
    PERI_CRG8_REG      PERI_CRG8; /**< CRG8 register. Offset address 0x00000020U. */
    unsigned char      reserved1[0xdc];
    PERI_CRG64_REG     PERI_CRG64; /**< CRG64 register. Offset address 0x00000100U. */
    PERI_CRG65_REG     PERI_CRG65; /**< CRG65 register. Offset address 0x00000104U. */
    PERI_CRG66_REG     PERI_CRG66; /**< CRG66 register. Offset address 0x00000108U. */
    PERI_CRG67_REG     PERI_CRG67; /**< CRG67 register. Offset address 0x0000010CU. */
    unsigned char      reserved2[0x30];
    PERI_CRG80_REG     PERI_CRG80; /**< CRG80 register. Offset address 0x00000140U. */
    PERI_CRG81_REG     PERI_CRG81; /**< CRG81 register. Offset address 0x00000144U. */
    PERI_CRG82_REG     PERI_CRG82; /**< CRG82 register. Offset address 0x00000148U. */
    PERI_CRG83_REG     PERI_CRG83; /**< CRG83 register. Offset address 0x0000014CU. */
    unsigned char      reserved3[0x30];
    PERI_CRG96_REG     PERI_CRG96; /**< CRG96 register. Offset address 0x00000180U. */
    PERI_CRG97_REG     PERI_CRG97; /**< CRG97 register. Offset address 0x00000184U. */
    unsigned char      reserved4[0x38];
    PERI_CRG112_REG    PERI_CRG112; /**< CRG112 register. Offset address 0x000001C0U. */
    PERI_CRG113_REG    PERI_CRG113; /**< CRG113 register. Offset address 0x000001C4U. */
    unsigned char      reserved5[0x38];
    PERI_CRG128_REG    PERI_CRG128; /**< CRG128 register. Offset address 0x00000200U. */
    unsigned char      reserved6[0x3c];
    PERI_CRG144_REG    PERI_CRG144; /**< CRG144 register. Offset address 0x00000240U. */
    PERI_CRG145_REG    PERI_CRG145; /**< CRG145 register. Offset address 0x00000244U. */
    PERI_CRG146_REG    PERI_CRG146; /**< CRG146 register. Offset address 0x00000248U. */
    PERI_CRG147_REG    PERI_CRG147; /**< CRG147 register. Offset address 0x0000024CU. */
    unsigned char      reserved7[0x30];
    PERI_CRG160_REG    PERI_CRG160; /**< CRG160 register. Offset address 0x00000280U. */
    PERI_CRG161_REG    PERI_CRG161; /**< CRG161 register. Offset address 0x00000284U. */
    PERI_CRG162_REG    PERI_CRG162; /**< CRG162 register. Offset address 0x00000288U. */
    unsigned char      reserved8[0x34];
    PERI_CRG176_REG    PERI_CRG176; /**< CRG176 register. Offset address 0x000002C0U. */
    unsigned char      reserved9[0x3c];
    PERI_CRG192_REG    PERI_CRG192; /**< CRG192 register. Offset address 0x00000300U. */
    unsigned char      reserved10[0x3c];
    PERI_CRG208_REG    PERI_CRG208; /**< CRG208 register. Offset address 0x00000340U. */
    unsigned char      reserved11[0x3c];
    PERI_CRG224_REG    PERI_CRG224; /**< CRG224 register. Offset address 0x00000380U. */
    unsigned char      reserved12[0x3c];
    PERI_CRG240_REG    PERI_CRG240; /**< CRG240 register. Offset address 0x000003C0U. */
    unsigned char      reserved13[0x3c];
    PERI_CRG256_REG    PERI_CRG256; /**< CRG256 register. Offset address 0x00000400U. */
    PERI_CRG257_REG    PERI_CRG257; /**< CRG257 register. Offset address 0x00000404U. */
    PERI_CRG258_REG    PERI_CRG258; /**< CRG258 register. Offset address 0x00000408U. */
    PERI_CRG259_REG    PERI_CRG259; /**< CRG259 register. Offset address 0x0000040CU. */
    unsigned char      reserved14[0x30];
    PERI_CRG272_REG    PERI_CRG272; /**< CRG272 register. Offset address 0x00000440U. */
    PERI_CRG273_REG    PERI_CRG273; /**< CRG273 register. Offset address 0x00000444U. */
    PERI_CRG274_REG    PERI_CRG274; /**< CRG274 register. Offset address 0x00000448U. */
    PERI_CRG275_REG    PERI_CRG275; /**< CRG275 register. Offset address 0x0000044CU. */
    unsigned char      reserved15[0x30];
    PERI_CRG288_REG    PERI_CRG288; /**< CRG288 register. Offset address 0x00000480U. */
    PERI_CRG289_REG    PERI_CRG289; /**< CRG289 register. Offset address 0x00000484U. */
    PERI_CRG290_REG    PERI_CRG290; /**< CRG290 register. Offset address 0x00000488U. */
    PERI_CRG291_REG    PERI_CRG291; /**< CRG291 register. Offset address 0x0000048cU. */
    PERI_CRG292_REG    PERI_CRG292; /**< CRG292 register. Offset address 0x00000490U. */
    PERI_CRG293_REG    PERI_CRG293; /**< CRG293 register. Offset address 0x00000494U. */
    unsigned char      reserved16[0x28];
    PERI_CRG304_REG    PERI_CRG304; /**< CRG304 register. Offset address 0x000004C0U. */
    PERI_CRG305_REG    PERI_CRG305; /**< CRG305 register. Offset address 0x000004C4U. */
    unsigned char      reserved17[0x38];
    PERI_CRG320_REG    PERI_CRG320; /**< CRG320 register. Offset address 0x00000500U. */
    unsigned char      reserved18[0x4f8];
    PERI_CRG639_REG    PERI_CRG639; /**< CRG639 register. Offset address 0x000009FCU. */
    PERI_CRG640_REG    PERI_CRG640; /**< CRG640 register. Offset address 0x00000A00U. */
    PERI_CRG641_REG    PERI_CRG641; /**< CRG640 register. Offset address 0x00000A04U. */
    unsigned char      reserved19[0x38];
    PERI_CRG656_REG    PERI_CRG656; /**< CRG656 register. Offset address 0x00000A40U. */
    unsigned char      reserved20[0xc];
    PERI_CRG660_REG    PERI_CRG660; /**< CRG660 register. Offset address 0x00000A50U. */
    unsigned char      reserved21[0xc];
    PERI_CRG664_REG    PERI_CRG664; /**< CRG664 register. Offset address 0x00000A60U. */
    unsigned char      reserved22[0xc];
    PERI_CRG668_REG    PERI_CRG668; /**< CRG668 register. Offset address 0x00000A70U. */
    unsigned char      reserved23[0xc];
    PERI_CRG672_REG    PERI_CRG672; /**< CRG672 register. Offset address 0x00000A80U. */
    unsigned char      reserved24[0xc];
    PERI_CRG676_REG    PERI_CRG676; /**< CRG676 register. Offset address 0x00000A90U. */
    PERI_CRG677_REG    PERI_CRG677; /**< CRG677 register. Offset address 0x00000A94U. */
    unsigned char      reserved25[1140];
    HOSC_PD_REG        HOSC_PD; /**< HOSC_PD register. Offset address 0x000000F0CU. */
    HOSC_LOCK_REG      HOSC_LOCK; /**< HOSC_LOCK register. Offset address 0x000000F10U. */
} volatile CRG_RegStruct;

/**
  * @}
  */

/* Parameter Check -----------------------------------------------------------*/
/**
  * @brief Verify pll_ref_cksel configuration
  * @param clkSelect pll_ref_cksel
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllRefClkSelect(CRG_PllRefClkSelect clkSelect)
{
    return ((clkSelect == CRG_PLL_REF_CLK_SELECT_HOSC) ||
            (clkSelect == CRG_PLL_REF_CLK_SELECT_XTAL));
}

/**
  * @brief Verify Crg pll_prediv configuration
  * @param preDiv pll prediv value
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllPreDiv(CRG_PllPreDiv preDiv)
{
    return ((preDiv >= CRG_PLL_PREDIV_1) &&
            (preDiv <= CRG_PLL_PREDIV_8));
}

/**
  * @brief Verify Crg pll_postdiv configuration
  * @param postDiv  pll_postdiv value
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllPostDiv(CRG_PllPostDiv postDiv)
{
    return ((postDiv >= CRG_PLL_POSTDIV_1) &&
            (postDiv <= CRG_PLL_POSTDIV_8));
}

/**
  * @brief Verify Crg pll_postdiv2 configuration
  * @param postDiv  pll_postdiv2 value
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllPostDiv2(CRG_PllPostDiv2 postDiv)
{
    return ((postDiv >= CRG_PLL_POSTDIV2_1) &&
            (postDiv <= CRG_PLL_POSTDIV2_8_MAX));
}

/**
  * @brief Verify Crg pll_fbdiv configuration
  * @param fbDiv  pll fbdiv value
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllFbDiv(unsigned int fbDiv)
{
    return (fbDiv <= CRG_PLL_FBDIV_MAX);
}

/**
  * @brief Verify Crg pll_digpostdiv_in_sel configuration
  * @param select  pll_digpostdiv_in_sel value
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllDigPostDivInSel(CRG_PllDigPostDivInSelect select)
{
    return ((select == CRG_PLL_DIG_POST_DIV_SELECT_FREF) ||
            (select == CRG_PLL_DIG_POST_DIV_SELECT_PLL));
}

/**
  * @brief Verify Crg core_cksel configuration
  * @param select  core_cksel value
  * @retval true
  * @retval false
  */
static inline bool IsCrgCoreCkSel(CRG_CoreClkSelect select)
{
    return ((select == CRG_CORE_CLK_SELECT_HOSC) ||
            (select == CRG_CORE_CLK_SELECT_TCXO) ||
            (select == CRG_CORE_CLK_SELECT_PLL));
}

/**
  * @brief Verify Crg configuration
  * @param select  1M clock selection
  * @retval true
  * @retval false
  */
static inline bool IsCrg1MCkSel(CRG_1MClkSelect select)
{
    return ((select == CRG_1M_CLK_SELECT_HOSC) ||
            (select == CRG_1M_CLK_SELECT_TCXO));
}

/**
  * @brief Verify Crg configuration
  * @param div  1M clock ratio
  * @retval true
  * @retval false
  */
static inline bool IsCrg1MCkDiv(unsigned int div)
{
    return (div <= CRG_1MHZ_CLK_MAX_DIV);
}

/**
  * @brief Verify AHB Crg Ip clock select configuration
  * @param select  AHB ip clock select value
  * @retval true
  * @retval false
  */
static inline bool IsCrgAHBCkSel(CRG_AHBClkSelect select)
{
    return ((select == CRG_AHB_CLK_NO_PREDV) ||
            (select == CRG_AHB_CLK_PREDIV_2) ||
            (select == CRG_AHB_CLK_PREDIV_4) ||
            (select == CRG_AHB_CLK_PREDIV_8));
}

/**
  * @brief Verify Crg Ip (exclude adc) clock enable configuration
  * @param enable  ip clock enable value
  * @retval true
  * @retval false
  */
static inline bool IsCrgIpClkEnable(unsigned int enable)
{
    return ((enable == IP_CLK_DISABLE) ||
            (enable == IP_CLK_ENABLE));
}

/**
  * @brief Check the PLL PreDiv is valid or not
  * @param clkPllRef PLL Refer clock
  * @param preDiv PLL Previous Divsion
  * @retval true
  * @retval false
  */
static inline bool IsCrgValidPreDiv(unsigned int pllRefFreq, unsigned int preDiv)
{
    unsigned int freq = pllRefFreq;
    if (preDiv != 0) {
        freq /= preDiv;
    }
    return (freq >= CRG_CLK_PFD_MIN_FREQ) && (freq <= CRG_CLK_PFD_MAX_FREQ);
}

/**
  * @brief Check the PLL FbDiv is valid or not
  * @param clkPfdFreq PLL PFD clock
  * @param fdDiv PLL FD Divsion
  * @retval true
  * @retval false
  */
static inline bool IsCrgValidFdDiv(unsigned int clkPfdFreq, unsigned int fdDiv)
{
    if (clkPfdFreq > 30000000U) {  /* The maximum speed of the external clock source is 30000000U. */
        return false;
    } else if (fdDiv > CRG_PLL_FBDIV_MAX) {
        return false;
    }

    unsigned int freq = (fdDiv > 0x6) ? (clkPfdFreq * fdDiv) : (clkPfdFreq * 0x6); /* 0x0-0x6: divided by 0x6 */
    return (freq >= CRG_CLK_VCO_MIN_FREQ) && (freq <= CRG_CLK_VCO_MAX_FREQ);
}

/**
  * @brief Check the PLL PostDiv is valid or not
  * @param clkPllRef PLL Vco clock
  * @param postDiv PLL Post Divsion
  * @retval true
  * @retval false
  */
static inline bool IsCrgValidPostDiv(unsigned int clkVcoFreq, unsigned int postDiv)
{
    unsigned int freq = clkVcoFreq;
    if (postDiv != 0) {
        freq /= (postDiv + 1);
    }
    return (freq <= CRG_CLK_TARGET_MAX_FREQ);
}

/**
  * @brief Check the PLL PostDiv is valid or not
  * @param clkPllRef PLL Vco clock
  * @param postDiv2 PLL Post Divsion2
  * @retval true
  * @retval false
  */
static inline bool IsCrgValidPostDiv2(unsigned int clkVcoFreq, unsigned int postDiv2)
{
    unsigned int freq = clkVcoFreq;
    if (postDiv2 != 0) {
        freq /= (postDiv2 + 1);
    }
    return (freq <= CRG_CLK_PST2_MAX_FREQ);
}

/**
  * @brief Check the adc clock select value
  * @param adcClkSelect the value of adc clock select
  * @retval true
  * @retval false
  */
static inline bool IsCrgAdcClkModeSelect(CRG_AdcClkSelect adcClkSelect)
{
    return (adcClkSelect == CRG_ADC_CLK_ASYN_HOSC || \
            adcClkSelect == CRG_ADC_CLK_ASYN_TCXO || \
            adcClkSelect == CRG_ADC_CLK_ASYN_PLL_DIV || \
            adcClkSelect == CRG_ADC_CLK_SYN_CORE);
}

/**
  * @brief Check the adc clock div value
  * @param div the value of adc clock div
  * @retval true
  * @retval false
  */
static inline bool IsCrgAdcClkDiv(CRG_AdcDiv div)
{
    return (div == CRG_ADC_DIV_1 || \
            div == CRG_ADC_DIV_2 || \
            div == CRG_ADC_DIV_3 || \
            div == CRG_ADC_DIV_4);
}

/**
  * @brief Set Pll Ref clock select
  * @param clk     Clock register base address
  * @param clkSel  clock source select
  * @retval None
  */
static inline void DCL_CRG_SetPllRefClkSel(CRG_RegStruct *clk, CRG_PllRefClkSelect clkSel)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllRefClkSelect(clkSel));
    clk->PERI_CRG0.BIT.pll_ref_cksel = (unsigned int)clkSel;
}

/**
  * @brief Get Pll Ref clock selection
  * @param clk             Clock register base address
  * @retval pll_ref_cksel  Ref clock selection
  */
static inline CRG_PllRefClkSelect DCL_CRG_GetPllRefClkSel(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return (CRG_PllRefClkSelect)clk->PERI_CRG0.BIT.pll_ref_cksel;
}

/**
  * @brief Set prevous division ratio
  * @param clk     Clock register base address
  * @param preDiv  prevous division ratio
  * @retval None
  */
static inline void DCL_CRG_SetPllPreDiv(CRG_RegStruct *clk, CRG_PllPreDiv preDiv)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllPreDiv(preDiv));
    clk->PERI_CRG1.BIT.pll_prediv = (unsigned int)preDiv;
}

/**
  * @brief Get prevous division ratio
  * @param clk      Clock register base address
  * @retval prediv  prevous division ratio
  */
static inline CRG_PllPreDiv DCL_CRG_GetPllPreDiv(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return (CRG_PllPreDiv)clk->PERI_CRG1.BIT.pll_prediv;
}

/**
  * @brief Set PLL frequency multiplication factor
  * @param clk    Clock register base address
  * @param fbDiv  Multiplication factor
  * @retval None
  */
static inline void DCL_CRG_SetPllFbDiv(CRG_RegStruct *clk, unsigned int fbDiv)
{
    unsigned int div = fbDiv;
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllFbDiv(fbDiv));
    clk->PERI_CRG2.BIT.pll_fbdiv = div;
}

/**
  * @brief Get PLL frequency multiplication factor
  * @param clk         Clock register base address
  * @retval pll_fbdiv  Multiplication factor
  */
static inline unsigned int DCL_CRG_GetPllFbDiv(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->PERI_CRG2.BIT.pll_fbdiv;
}

/**
  * @brief Set PLL post division ratio
  * @param clk     Clock register base address
  * @param postDiv Post division ratio
  * @retval None
  */
static inline void DCL_CRG_SetPllPostDiv1(CRG_RegStruct *clk, CRG_PllPostDiv postDiv)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllPostDiv(postDiv));
    clk->PERI_CRG3.BIT.pll_postdiv1 = (unsigned int)postDiv;
}

/**
  * @brief Get PLL post division ratio
  * @param clk           Clock register base address
  * @retval pll_postdiv  Post division ratio
  */
static inline CRG_PllPostDiv DCL_CRG_GetPllPostDiv1(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return (CRG_PllPostDiv)clk->PERI_CRG3.BIT.pll_postdiv1;
}

/**
  * @brief Set PLL post division ratio
  * @param clk     Clock register base address
  * @param postDiv Post division ratio
  * @retval None
  */
static inline void DCL_CRG_SetPllPostDiv2(CRG_RegStruct *clk, CRG_PllPostDiv postDiv)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllPostDiv(postDiv));
    clk->PERI_CRG3.BIT.pll_postdiv2 = (unsigned int)postDiv;
}

/**
  * @brief Get PLL post division ratio
  * @param clk           Clock register base address
  * @retval pll_postdiv  Post division ratio
  */
static inline CRG_PllPostDiv DCL_CRG_GetPllPostDiv2(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return (CRG_PllPostDiv)clk->PERI_CRG3.BIT.pll_postdiv2;
}

/**
  * @brief Set PLL Power
  * @param clk  Clock register base address
  * @param pd   pll power down or not
  * @retval None
  */
static inline void DCL_CRG_SetPllPd(CRG_RegStruct *clk, bool pd)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    clk->PERI_CRG4.BIT.pll_pd = (unsigned int)pd;
}

/**
  * @brief Get PLL power status
  * @param clk  Clock register base address
  * @retval 0: power up, 1: power down
  */
static inline bool DCL_CRG_GetPllPd(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->PERI_CRG4.BIT.pll_pd;
}

/**
  * @brief Set core clock selection
  * @param clk  Clock register base address
  * @param select  Core clock selection
  * @retval None
  */
static inline void DCL_CRG_SetCoreClkSel(CRG_RegStruct *clk, CRG_CoreClkSelect select)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgCoreCkSel(select));
    clk->PERI_CRG64.BIT.clk_pst1_sw_sel = select;
}

/**
  * @brief Get core clock selection
  * @param clk  Clock register base address
  * @retval Core clock selection
  */
static inline unsigned int DCL_CRG_GetCoreClkSel(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->PERI_CRG64.BIT.clk_pst1_sw_sel;
}

/**
  * @brief Set core clock selection
  * @param clk  Clock register base address
  * @param select  Core clock selection
  * @retval None
  */
static inline void DCL_CRG_SetAdcAsynClkSel(CRG_RegStruct *clk, CRG_CoreClkSelect select)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgCoreCkSel(select));
    clk->PERI_CRG64.BIT.clk_pst2_sw_sel = select;
}

/**
  * @brief Get adc core clock selection
  * @param clk  Clock register base address
  * @retval Core clock selection
  */
static inline unsigned int DCL_CRG_GetAdcAsynClkSel(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->PERI_CRG64.BIT.clk_pst2_sw_sel;
}

/**
  * @brief Set 1M clock selection
  * @param clk  Clock register base address
  * @param select  Core clock selection
  * @retval None
  */
static inline void DCL_CRG_Set1MClkSel(CRG_RegStruct *clk, CRG_1MClkSelect select)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrg1MCkSel(select));
    clk->PERI_CRG66.BIT.clk_1m_ini_cksel = select;
}

/**
  * @brief Get 1M clock selection
  * @param clk  Clock register base address
  * @retval Core clock selection
  */
static inline unsigned int DCL_CRG_Get1MClkSel(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->PERI_CRG66.BIT.clk_1m_ini_cksel;
}

/**
  * @brief Set 1M clock division ratio
  * @param clk  Clock register base address
  * @param div  Division ratio
  * @retval None
  */
static inline void DCL_CRG_Set1MClkDiv(CRG_RegStruct *clk, unsigned int div)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(div <= CRG_1MHZ_CLK_MAX_DIV);
    clk->PERI_CRG67.BIT.clk_1m_div = div;
}

/**
  * @brief  Enable test clock function
  * @param clk  Clock register base address
  * @retval None
  */
static inline void DCL_CRG_TestClkEnable(CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    clk->PERI_CRG639.BIT.test_clk_en = BASE_CFG_ENABLE;   /* Enable the test clock. */
}

/**
  * @brief  Disable test clock function
  * @param clk  Clock register base address
  * @retval None
  */
static inline void DCL_CRG_TestClkDisable(CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    clk->PERI_CRG639.BIT.test_clk_en = BASE_CFG_DISABLE;  /* Disable the test clock. */
}

/**
  * @brief CRG test clock select.
  * @param clk  Clock register base address
  * @param clkSel  Clock select.
  * @retval None
  */
static inline void DCL_CRG_TestClkSel(CRG_RegStruct *clk, CRG_TestClkSel clkSel)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(clkSel >= CRG_TEST_CLK_PLL_PFD);
    CRG_PARAM_CHECK_NO_RET(clkSel <= CRG_TEST_CLK_HOSC_DIV);
    clk->PERI_CRG639.BIT.test_clk_sel = clkSel; /* Set the test clock select. */
}
/**
  * @}
  */

/**
 * @}
 */
#endif /* McuMagicTag_CRG_IP_H */
