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
  * @brief CRG_IP: crg_v0
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
#define IP_SYSCLK_ENABLE             0x00000002U   /**< IP SysClock disable bitmask, Only valid for ADC */

#define DAC_DIV_BITLEN               4U            /**< DIV bit length */
#define DAC_DIV_MASK ((1 << DAC_DIV_BITLEN) - 1)   /**< DAC div mask, base on the bit length */

#define ADC_DIV_FACTOR          (1 << 1)           /**< ADC div min factor */

#define CRG_FREQ_1MHz           (1000 * 1000)
#define CRG_CLK_PFD_MIN_FREQ    (4 * CRG_FREQ_1MHz)
#define CRG_CLK_PFD_MAX_FREQ    (75 * CRG_FREQ_1MHz / 10)
#define CRG_CLK_VCO_MIN_FREQ    (100 * CRG_FREQ_1MHz)
#define CRG_CLK_VCO_MAX_FREQ    (200 * CRG_FREQ_1MHz)
#define CRG_CLK_TARGET_MIN_FREQ (3125 * 1000)
#define CRG_CLK_TARGET_MAX_FREQ (200 * CRG_FREQ_1MHz)
#define CRG_CLK_XTAL_MIN_FREQ   (2 * CRG_FREQ_1MHz)
#define CRG_CLK_XTAL_MAX_FREQ   (30 * CRG_FREQ_1MHz)
/**
 * @brief PLL refer clock Select
 */
typedef enum {
    CRG_PLL_REF_CLK_SELECT_HOSC = 0,
    CRG_PLL_REF_CLK_SELECT_XTAL = 1,
} CRG_PllRefClkSelect;

/**
 * @brief PLL previous division value in register
 */
typedef enum {
    CRG_PLL_NO_PREDV = 0,
    CRG_PLL_PREDIV_1 = 1,
    CRG_PLL_PREDIV_2 = 2,
    CRG_PLL_PREDIV_4 = 3,
} CRG_PllPreDiv;

/**
 * @brief PLL previous division value in Calc frequency
 */
typedef enum {
    PLL_PREDIV_OUT_1 = 1,
    PLL_PREDIV_OUT_2 = 2,
    PLL_PREDIV_OUT_4 = 4,
} PLL_PreDivOut;

/**
 * @brief PLL post division value in register
 */
typedef enum {
    CRG_PLL_POSTDIV_1 = 0,
    CRG_PLL_POSTDIV_2 = 1,
    CRG_PLL_POSTDIV_4 = 2,
    CRG_PLL_POSTDIV_8 = 3,
    CRG_PLL_POSTDIV_16 = 4,
    CRG_PLL_POSTDIV_32 = 5,
    CRG_PLL_POSTDIV_32_MAX = 7,
} CRG_PllPostDiv;

/**
 * @brief Core clock selection
 * @note  CRG_CORE_CLK_SELECT_LOSC will be selected under other invalid conditions
 */
typedef enum {
    CRG_CORE_CLK_SELECT_HOSC = 0,
    CRG_CORE_CLK_SELECT_TCXO = 1,
    CRG_CORE_CLK_SELECT_PLL  = 2,
} CRG_CoreClkSelect;

/**
 * @brief PLL frequency multiplication range
 */
typedef enum {
    CRG_PLL_FBDIV_MIN  = 6,
    CRG_PLL_FBDIV_MAX  = 63,
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
    CRG_PLL_TEST_SELECT_PFD   = 0,
    CRG_PLL_TEST_SELECT_PLL_DIV_128  = 3,
} CRG_PllTestClkSelect;

/**
 * @brief Source clock selection of ip in APB_LS_SUBSYS
 */
typedef enum {
    CRG_CLK_LS       = 0,
    CRG_CLK_LS_DIV_2 = 1,
    CRG_CLK_LS_DIV_4 = 2,
    CRG_CLK_LS_DIV_8 = 3,
} CRG_APBLsClkSelect;

/**
 * @brief ADC source clock select
 */
typedef enum {
    CRG_ADC_CLK_SELECT_HOSC = 0,
    CRG_ADC_CLK_SELECT_TCXO = 1,
    CRG_ADC_CLK_SELECT_PLL_DIV = 2,
} CRG_AdcClkSelect;

/**
 * @brief ADC Div set Value
 */
typedef enum {
    CRG_ADC_DIV_1 = 0,
    CRG_ADC_DIV_1_P_5 = 1,
    CRG_ADC_DIV_2 = 2,
    CRG_ADC_DIV_2_P_5 = 3,
    CRG_ADC_DIV_3 = 4,
    CRG_ADC_DIV_3_P_5 = 5,
    CRG_ADC_DIV_4 = 6,
    CRG_ADC_DIV_4_P_5 = 7,
    CRG_ADC_DIV_5 = 8,
    CRG_ADC_DIV_5_P_5 = 9,
    CRG_ADC_DIV_6 = 10,
    CRG_ADC_DIV_6_P_5 = 11,
    CRG_ADC_DIV_7 = 12,
    CRG_ADC_DIV_7_P_5 = 13,
    CRG_ADC_DIV_8 = 14,
    CRG_ADC_DIV_8_P_5 = 15,
    CRG_ADC_DIV_9 = 16,
    CRG_ADC_DIV_9_P_5 = 17,
    CRG_ADC_DIV_10 = 18,
    CRG_ADC_DIV_10_P_5 = 19,
    CRG_ADC_DIV_11 = 20,
    CRG_ADC_DIV_11_P_5 = 21,
    CRG_ADC_DIV_12 = 22,
    CRG_ADC_DIV_12_P_5 = 23,
    CRG_ADC_DIV_13 = 24,
    CRG_ADC_DIV_13_P_5 = 25,
    CRG_ADC_DIV_14 = 26,
    CRG_ADC_DIV_14_P_5 = 27,
    CRG_ADC_DIV_15 = 28,
    CRG_ADC_DIV_15_P_5 = 29,
    CRG_ADC_DIV_16 = 30,
    CRG_ADC_DIV_16_P_5 = 31,
} CRG_AdcDiv;

/**
 * @brief DAC Div set Value
 */
typedef enum {
    CRG_DAC_DIV_1 = 0,
    CRG_DAC_DIV_2 = 1,
    CRG_DAC_DIV_3 = 2,
    CRG_DAC_DIV_4 = 3,
    CRG_DAC_DIV_5 = 4,
    CRG_DAC_DIV_6 = 5,
    CRG_DAC_DIV_7 = 6,
    CRG_DAC_DIV_8 = 7,
    CRG_DAC_DIV_9 = 8,
    CRG_DAC_DIV_10 = 9,
    CRG_DAC_DIV_11 = 10,
    CRG_DAC_DIV_12 = 11,
    CRG_DAC_DIV_13 = 12,
    CRG_DAC_DIV_14 = 13,
    CRG_DAC_DIV_15 = 14,
    CRG_DAC_DIV_16 = 15,
} CRG_DacDiv;

/**
 * @brief CRG Test Clock Select
 */
typedef enum {
    CRG_TEST_CLK_PLL  = 0x00000000U,
    CRG_TEST_CLK_HOSC = 0x00000001U,
    CRG_TEST_CLK_LOSC = 0x00000002U,
    CRG_TEST_CLK_XTAL = 0x00000003U,
    CRG_TEST_CLK_DAC0 = 0x00000004U,
    CRG_TEST_CLK_DAC1 = 0x00000005U,
    CRG_TEST_CLK_DAC2 = 0x00000006U,
    CRG_TEST_CLK_ADC0 = 0x00000007U,
    CRG_TEST_CLK_ADC1 = 0x00000008U,
    CRG_TEST_CLK_ADC2 = 0x00000009U,
} CRG_TestClkSel;

/**
  * @brief CRG Extra Handle, include CRG's other config
  */
typedef struct {
    bool                ckSwitchEn;       /**< Clock switch interactive function enable */
} CRG_ExtendHandle;

/**
 * @brief PLL Division Config
 */
typedef struct {
    unsigned int PreDiv;  /**< prescale division */
    unsigned int fbDiv;   /**< feedback division */
    unsigned int postDiv; /**< post division */
} CRG_PllDivCfg;

/**
 * @brief APB_LS_SUBSYS IP config
 */
typedef union {
    unsigned int  value;
    struct {
        unsigned int    cken       : 1;
        unsigned int    reserved_0 : 7;
        unsigned int    cksel      : 2;
        unsigned int    reserved_1 : 6;
        unsigned int    srst_req   : 1;
        unsigned int    reserved_2 : 15;
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
    unsigned int  value;
    struct {
        unsigned int    cken         : 1;
        unsigned int    sys_cken     : 1;
        unsigned int    reserved_0   : 2;
        unsigned int    div          : 5;
        unsigned int    reserved_1   : 3;
        unsigned int    cksel        : 2;
        unsigned int    reserved_2   : 2;
        unsigned int    srst_req     : 1;
        unsigned int    sys_srst_req : 1;
        unsigned int    ana_srst_req : 1;
        unsigned int    reserved_3   : 13;
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
 * @defgroup CRG_Reg_Def CRG Register Definition
 * @brief register mapping structure
 * @{
 */
typedef struct {
    unsigned int    pll_ref_cksel : 1;  /**< pll reference select */
    unsigned int    reserved_0    : 31;
} volatile PERI_CRG0_Reg;

typedef struct {
    unsigned int    pll_prediv : 4; /**< predivider value */
    unsigned int    reserved_0 : 28;
} volatile PERI_CRG1_Reg;

typedef struct {
    unsigned int    pll_fbdiv  : 8; /**< feedback divider value */
    unsigned int    reserved_0 : 24;
} volatile PERI_CRG2_Reg;

typedef struct {
    unsigned int    pll_postdiv : 4; /**< post divider value */
    unsigned int    reserved_0  : 28;
} volatile PERI_CRG3_Reg;

typedef struct {
    unsigned int    pll_pd     : 1; /**< pll power down */
    unsigned int    reserved_0 : 31;
} volatile PERI_CRG4_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    reserved0              : 13;
        unsigned int    pll_dig_eb_lockdet     : 1; /**< lock detector window size */
        unsigned int    reserved_0             : 18;
    } BIT;
} volatile PERI_CRG5_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    reserved0              : 19;
        unsigned int    pll_test_clk           : 3; /**< pll test clock */
        unsigned int    reserved1              : 10;
    } BIT;
} volatile PERI_CRG6_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    core_cksel  : 2; /**< core clock select */
        unsigned int    reserved0   : 1;
        unsigned int    ck_switchen : 1; /**< clock switch enable */
        unsigned int    reserved    : 28;
    } BIT;
} volatile PERI_CRG7_Reg;

typedef struct {
    unsigned int    pll_lock : 1; /**< pll clock */
    unsigned int    reserved : 31;
} volatile PERI_CRG8_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    uart0_cken     : 1; /**< uart0 clock enable */
        unsigned int    reserved_0     : 7;
        unsigned int    uart0_cksel    : 2; /**< uart0 clock select */
        unsigned int    reserved_1     : 6;
        unsigned int    uart0_srst_req : 1; /**< uart0 reset request */
        unsigned int    reserved_2     : 15;
    } BIT;
} volatile PERI_CRG20_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    uart1_cken     : 1; /**< uart1 clock enable */
        unsigned int    reserved_0     : 7;
        unsigned int    uart1_cksel    : 2; /**< uart1 clock select */
        unsigned int    reserved_1     : 6;
        unsigned int    uart1_srst_req : 1; /**< uart1 reset request */
        unsigned int    reserved_2     : 15;
    } BIT;
} volatile PERI_CRG21_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    uart2_cken     : 1; /**< uart2 clock enable */
        unsigned int    reserved_0     : 7;
        unsigned int    uart2_cksel    : 2; /**< uart2 clock select */
        unsigned int    reserved_1     : 6;
        unsigned int    uart2_srst_req : 1; /**< uart2 reset request */
        unsigned int    reserved_2     : 15;
    } BIT;
} volatile PERI_CRG22_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    timer01_cken     : 1; /**< timer0/1 clock enable */
        unsigned int    reserved_2       : 7;
        unsigned int    timer01_cksel    : 2; /**< timer0/1 clock select */
        unsigned int    reserved_0       : 6;
        unsigned int    timer01_srst_req : 1; /**< timer0/1 reset request */
        unsigned int    reserved_1       : 15;
    } BIT;
} volatile PERI_CRG23_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    timer23_cken     : 1; /**< timer2/3 clock enable */
        unsigned int    reserved_0       : 7;
        unsigned int    timer23_cksel    : 2; /**< timer2/3 clock select */
        unsigned int    reserved_1       : 6;
        unsigned int    timer23_srst_req : 1; /**< timer2/3 reset request */
        unsigned int    reserved_2       : 15;
    } BIT;
} volatile PERI_CRG24_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    pwm0_cken     : 1; /**< gpt0 clock enable */
        unsigned int    reserved_0    : 7;
        unsigned int    pwm0_cksel    : 2; /**< gpt0 clock select */
        unsigned int    reserved_1    : 6;
        unsigned int    pwm0_srst_req : 1; /**< gpt0 reset request */
        unsigned int    reserved_2    : 15;
    } BIT;
} volatile PERI_CRG25_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    pwm1_cken     : 1; /**< gpt1 clock enable */
        unsigned int    reserved_0    : 7;
        unsigned int    pwm1_cksel    : 2; /**< gpt1 clock select */
        unsigned int    reserved_1    : 6;
        unsigned int    pwm1_srst_req : 1; /**< gpt1 reset request */
        unsigned int    reserved_2    : 15;
    } BIT;
} volatile PERI_CRG26_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    wdog_cken     : 1; /**< watchdog clock enable */
        unsigned int    reserved_0    : 7;
        unsigned int    wdog_cksel    : 2; /**< watchdog clock select */
        unsigned int    reserved_1    : 6;
        unsigned int    wdog_srst_req : 1; /**< watchdog reset request */
        unsigned int    reserved_2    : 15;
    } BIT;
} volatile PERI_CRG27_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    ssp_cken     : 1; /**< spi clock enable */
        unsigned int    reserved_0   : 7;
        unsigned int    ssp_cksel    : 2; /**< spi clock select */
        unsigned int    reserved_1   : 6;
        unsigned int    ssp_srst_req : 1; /**< spi reset request */
        unsigned int    reserved_2   : 15;
    } BIT;
} volatile PERI_CRG28_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    can_cken     : 1; /**< can clock enable */
        unsigned int    reserved_1   : 15;
        unsigned int    can_srst_req : 1; /**< can reset request */
        unsigned int    reserved_0   : 15;
    } BIT;
} volatile PERI_CRG29_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    capm0_cken     : 1; /**< capm0 clock enable */
        unsigned int    capm1_cken     : 1; /**< capm1 clock enable */
        unsigned int    capm2_cken     : 1; /**< capm0 clock enable */
        unsigned int    reserved_1     : 13;
        unsigned int    capm0_srst_req : 1; /**< capm0 reset request */
        unsigned int    capm1_srst_req : 1; /**< capm1 reset request */
        unsigned int    capm2_srst_req : 1; /**< capm2 reset request */
        unsigned int    reserved_0     : 13;
    } BIT;
} volatile PERI_CRG30_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    dma_cken     : 1; /**< dma clock enable */
        unsigned int    reserved_0   : 15;
        unsigned int    dma_srst_req : 1; /**< dma reset request */
        unsigned int    reserved_1   : 15;
    } BIT;
} volatile PERI_CRG31_Reg;

typedef struct {
    unsigned int    eflash_cken : 1; /**< flash clock enable */
    unsigned int    reserved    : 31;
} volatile PERI_CRG32_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    gpio0_cken     : 1; /**< gpio0 clock enable */
        unsigned int    gpio1_cken     : 1; /**< gpio1 clock enable */
        unsigned int    gpio2_cken     : 1; /**< gpio2 clock enable */
        unsigned int    gpio3_cken     : 1; /**< gpio3 clock enable */
        unsigned int    gpio4_cken     : 1; /**< gpio4 clock enable */
        unsigned int    gpio5_cken     : 1; /**< gpio5 clock enable */
        unsigned int    gpio6_cken     : 1; /**< gpio6 clock enable */
        unsigned int    gpio7_cken     : 1; /**< gpio7 clock enable */
        unsigned int    reserved_0     : 8;
        unsigned int    gpio0_srst_req : 1; /**< gpio0 reset request */
        unsigned int    gpio1_srst_req : 1; /**< gpio1 reset request */
        unsigned int    gpio2_srst_req : 1; /**< gpio2 reset request */
        unsigned int    gpio3_srst_req : 1; /**< gpio3 reset request */
        unsigned int    gpio4_srst_req : 1; /**< gpio4 reset request */
        unsigned int    gpio5_srst_req : 1; /**< gpio5 reset request */
        unsigned int    gpio6_srst_req : 1; /**< gpio6 reset request */
        unsigned int    gpio7_srst_req : 1; /**< gpio7 reset request */
        unsigned int    reserved_1     : 8; /**< gpio8 reset request */
    } BIT;
} volatile PERI_CRG33_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    i2c_cken     : 1; /**< i2c clock enable */
        unsigned int    reserved_0   : 15;
        unsigned int    i2c_srst_req : 1; /**< i2c reset request */
        unsigned int    reserved_1   : 15;
    } BIT;
} volatile PERI_CRG34_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    iwdog_cken     : 1; /**< iwdog clock enable */
        unsigned int    reserved_1     : 15;
        unsigned int    iwdog_srst_req : 1; /**< iwdog reset request */
        unsigned int    reserved_0     : 15;
    } BIT;
} volatile PERI_CRG35_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    qdm0_cken     : 1; /**< clock enable */
        unsigned int    reserved_0    : 15;
        unsigned int    qdm0_srst_req : 1; /**< reset request */
        unsigned int    reserved_1    : 14;
    } BIT;
} volatile PERI_CRG36_Reg;

typedef union {
    unsigned int value;
    struct {
        unsigned int    cs_cken    : 1; /**< clock enable */
        unsigned int    reserved_0 : 31;
    } BIT;
} volatile PERI_CRG38_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    crc_cken     : 1; /**< clock enable */
        unsigned int    reserved_0   : 15;
        unsigned int    crc_srst_req : 1; /**< reset request */
        unsigned int    reserved_1   : 15;
    } BIT;
} volatile PERI_CRG39_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    apt0_cken     : 1; /**< apt0 clock enable */
        unsigned int    apt1_cken     : 1; /**< apt1 clock enable */
        unsigned int    apt2_cken     : 1; /**< apt2 clock enable */
        unsigned int    apt3_cken     : 1; /**< apt3 clock enable */
        unsigned int    apt4_cken     : 1; /**< apt4 clock enable */
        unsigned int    apt5_cken     : 1; /**< apt5 clock enable */
        unsigned int    apt6_cken     : 1; /**< apt6 clock enable */
        unsigned int    apt7_cken     : 1; /**< apt7 clock enable */
        unsigned int    apt8_cken     : 1; /**< apt8 clock enable */
        unsigned int    reserved_0    : 7;
        unsigned int    apt0_srst_req : 1; /**< apt0 reset request */
        unsigned int    apt1_srst_req : 1; /**< apt1 reset request */
        unsigned int    apt2_srst_req : 1; /**< apt2 reset request */
        unsigned int    apt3_srst_req : 1; /**< apt3 reset request */
        unsigned int    apt4_srst_req : 1; /**< apt4 reset request */
        unsigned int    apt5_srst_req : 1; /**< apt5 reset request */
        unsigned int    apt6_srst_req : 1; /**< apt6 reset request */
        unsigned int    apt7_srst_req : 1; /**< apt7 reset request */
        unsigned int    apt8_srst_req : 1; /**< apt8 reset request */
        unsigned int    reserved_1    : 7; /**< apt9 reset request */
    } BIT;
} volatile PERI_CRG40_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    adc0_cken         : 1; /**< adc0 controller sampling clock enable */
        unsigned int    adc0sys_cken      : 1; /**< adc0 controller config clock enable */
        unsigned int    reserved_0        : 2;
        unsigned int    adc0_div          : 5; /**< adc0 divider value */
        unsigned int    reserved_1        : 3;
        unsigned int    adc0_cksel        : 2; /**< adc0 clock select */
        unsigned int    reserved_2        : 2;
        unsigned int    adc0_srst_req     : 1; /**< adc0 controller sampling clock reset request */
        unsigned int    adc0sys_srst_req  : 1; /**< adc0 controller config clock reset request */
        unsigned int    adc0_ana_srst_req : 1; /**< adc0 controller analog clock reset request */
        unsigned int    reserved_3        : 13;
    } BIT;
} volatile PERI_CRG41_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    adc1_cken           : 1; /**< adc1 controller sampling clock enable */
        unsigned int    adc1sys_cken        : 1; /**< adc1 controller config clock enable */
        unsigned int    reserved_0          : 2;
        unsigned int    adc1_div            : 5; /**< adc1 divider value */
        unsigned int    reserved_1          : 3;
        unsigned int    adc1_cksel          : 2; /**< adc1 clock select */
        unsigned int    reserved_2          : 2;
        unsigned int    adc1_srst_req       : 1; /**< adc1 controller sampling clock reset request */
        unsigned int    adc1sys_srst_req    : 1; /**< adc1 controller config clock reset request */
        unsigned int    adc1_ana_srst_req   : 1; /**< adc1 controller analog clock reset request */
        unsigned int    reserved_3          : 13;
    } BIT;
} volatile PERI_CRG42_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    adc2_cken           : 1; /**< adc1 controller sampling clock enable */
        unsigned int    adc2sys_cken        : 1; /**< adc1 controller config clock enable */
        unsigned int    reserved_0          : 2;
        unsigned int    adc2_div            : 5; /**< adc1 divider value */
        unsigned int    reserved_1          : 3;
        unsigned int    adc2_cksel          : 2; /**< adc1 clock select */
        unsigned int    reserved_2          : 2;
        unsigned int    adc2_srst_req       : 1; /**< adc1 controller sampling clock reset request */
        unsigned int    adc2sys_srst_req    : 1; /**< adc1 controller config clock reset request */
        unsigned int    adc2_ana_srst_req   : 1; /**< adc1 controller analog clock reset request */
        unsigned int    reserved_3          : 13;
    } BIT;
} volatile PERI_CRG43_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    acmp0_cken     : 1; /**< acmp0 clock enable */
        unsigned int    acmp1_cken     : 1; /**< acmp1 clock enable */
        unsigned int    acmp2_cken     : 1; /**< acmp2 clock enable */
        unsigned int    reserved_0     : 13;
        unsigned int    acmp0_srst_req : 1; /**< acmp0 reset request */
        unsigned int    acmp1_srst_req : 1; /**< acmp1 reset request */
        unsigned int    acmp2_srst_req : 1; /**< acmp2 reset request */
        unsigned int    reserved_1     : 13;
    } BIT;
} volatile PERI_CRG44_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    dac0_cken     : 1; /**< dac0 clock enable */
        unsigned int    dac1_cken     : 1; /**< dac1 clock enable */
        unsigned int    dac2_cken     : 1; /**< dac2 clock enable */
        unsigned int    reserved_0    : 1;
        unsigned int    dac0_div      : 4; /**< dac0 divider */
        unsigned int    dac1_div      : 4; /**< dac1 divider */
        unsigned int    dac2_div      : 4; /**< dac2 divider */
        unsigned int    dac0_srst_req : 1; /**< dac0 reset request */
        unsigned int    dac1_srst_req : 1; /**< dac1 reset request */
        unsigned int    dac2_srst_req : 1; /**< dac2 reset request */
        unsigned int    reserved_1    : 13;
    } BIT;
} volatile PERI_CRG45_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    pga0_cken     : 1; /**< pga0 clock enable */
        unsigned int    pga1_cken     : 1; /**< pga1 clock enable */
        unsigned int    pga2_cken     : 1; /**< pga2 clock enable */
        unsigned int    reserved_1    : 13;
        unsigned int    pga0_srst_req : 1; /**< pga0 reset request */
        unsigned int    pga1_srst_req : 1; /**< pga1 reset request */
        unsigned int    pga2_srst_req : 1; /**< pga2 reset request */
        unsigned int    reserved_0    : 13;
    } BIT;
} volatile PERI_CRG46_Reg;

typedef union {
    unsigned int  value;
    struct {
        unsigned int    test_clk_sel : 4; /**< test clock select */
        unsigned int    test_clk_en  : 1; /**< test clock enable */
        unsigned int    reserved     : 27;
    } BIT;
} volatile PERI_CRG47_Reg;

typedef union {
    unsigned int value;
    struct {
        unsigned int    pvd_rst_enable : 1; /**< pvd reset enable */
        unsigned int    reserved_0     : 31 ;
    } BIT;
} volatile PERI_CRG48_Reg;

/**
 * @brief CRG Register
 */
typedef struct {
    PERI_CRG0_Reg    PERI_CRG0; /**< CRG0 register. Offset address 0x00000000U. */
    PERI_CRG1_Reg    PERI_CRG1; /**< CRG1 register. Offset address 0x00000004U. */
    PERI_CRG2_Reg    PERI_CRG2; /**< CRG2 register. Offset address 0x00000008U. */
    PERI_CRG3_Reg    PERI_CRG3; /**< CRG3 register. Offset address 0x0000000CU. */
    PERI_CRG4_Reg    PERI_CRG4; /**< CRG4 register. Offset address 0x00000010U. */
    PERI_CRG5_Reg    PERI_CRG5; /**< CRG5 register. Offset address 0x00000014U. */
    PERI_CRG6_Reg    PERI_CRG6; /**< CRG6 register. Offset address 0x00000018U. */
    PERI_CRG7_Reg    PERI_CRG7; /**< CRG7 register. Offset address 0x0000001CU. */
    PERI_CRG8_Reg    PERI_CRG8; /**< CRG8 register. Offset address 0x00000020U. */
    unsigned int     reserved_1[3];
    PERI_CRG20_Reg   PERI_CRG20; /**< CRG20 register. Offset address 0x00000030U. */
    PERI_CRG21_Reg   PERI_CRG21; /**< CRG21 register. Offset address 0x00000034U. */
    PERI_CRG22_Reg   PERI_CRG22; /**< CRG22 register. Offset address 0x00000038U. */
    PERI_CRG23_Reg   PERI_CRG23; /**< CRG23 register. Offset address 0x0000003CU. */
    PERI_CRG24_Reg   PERI_CRG24; /**< CRG24 register. Offset address 0x00000040U. */
    PERI_CRG25_Reg   PERI_CRG25; /**< CRG25 register. Offset address 0x00000044U. */
    PERI_CRG26_Reg   PERI_CRG26; /**< CRG26 register. Offset address 0x00000048U. */
    PERI_CRG27_Reg   PERI_CRG27; /**< CRG27 register. Offset address 0x0000004CU. */
    PERI_CRG28_Reg   PERI_CRG28; /**< CRG28 register. Offset address 0x00000050U. */
    PERI_CRG29_Reg   PERI_CRG29; /**< CRG29 register. Offset address 0x00000054U. */
    PERI_CRG30_Reg   PERI_CRG30; /**< CRG30 register. Offset address 0x00000058U. */
    PERI_CRG31_Reg   PERI_CRG31; /**< CRG31 register. Offset address 0x0000005CU. */
    PERI_CRG32_Reg   PERI_CRG32; /**< CRG32 register. Offset address 0x00000060U. */
    PERI_CRG33_Reg   PERI_CRG33; /**< CRG33 register. Offset address 0x00000064U. */
    PERI_CRG34_Reg   PERI_CRG34; /**< CRG34 register. Offset address 0x00000068U. */
    PERI_CRG35_Reg   PERI_CRG35; /**< CRG35 register. Offset address 0x0000006CU. */
    PERI_CRG36_Reg   PERI_CRG36; /**< CRG36 register. Offset address 0x00000070U. */
    unsigned int     reserved_3;
    PERI_CRG38_Reg   PERI_CRG38; /**< CRG38 register. Offset address 0x00000078U. */
    PERI_CRG39_Reg   PERI_CRG39; /**< CRG39 register. Offset address 0x0000007CU. */
    PERI_CRG40_Reg   PERI_CRG40; /**< CRG40 register. Offset address 0x00000080U. */
    PERI_CRG41_Reg   PERI_CRG41; /**< CRG40 register. Offset address 0x00000084U. */
    PERI_CRG42_Reg   PERI_CRG42; /**< CRG42 register. Offset address 0x00000088U. */
    PERI_CRG43_Reg   PERI_CRG43; /**< CRG43 register. Offset address 0x0000008CU. */
    PERI_CRG44_Reg   PERI_CRG44; /**< CRG44 register. Offset address 0x00000090U. */
    PERI_CRG45_Reg   PERI_CRG45; /**< CRG45 register. Offset address 0x00000094U. */
    PERI_CRG46_Reg   PERI_CRG46; /**< CRG46 register. Offset address 0x00000098U. */
    PERI_CRG47_Reg   PERI_CRG47; /**< CRG47 register. Offset address 0x0000009CU. */
    PERI_CRG48_Reg   PERI_CRG48; /**< CRG48 register. Offset address 0x000000A0U. */
} volatile CRG_RegStruct;
/**
  * @}
  */

/* Parameter Check -----------------------------------------------------------*/
/**
  * @brief Verify XTAL Freq
  * @param freq The freq of User-defined XTAL
  * @retval true
  * @retval false
  */
static inline bool IsCrgXtalFreq(unsigned int freq)
{
    return ((freq >= CRG_CLK_XTAL_MIN_FREQ) &&
            (freq <= CRG_CLK_XTAL_MAX_FREQ));
}

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
    return ((preDiv >= CRG_PLL_NO_PREDV) &&
            (preDiv <= CRG_PLL_PREDIV_4));
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
            (postDiv <= CRG_PLL_POSTDIV_32_MAX));
}

/**
  * @brief Verify Crg pll_fbdiv configuration
  * @param fbDiv  pll fbdiv value
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllFbDiv(unsigned int fbDiv)
{
    return (fbDiv >= CRG_PLL_FBDIV_MIN) && (fbDiv <= CRG_PLL_FBDIV_MAX);
}

/**
  * @brief Verify Crg pll_test_clk select configuration
  * @param select  pll_test_clk value
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllTestClkSelect(CRG_PllTestClkSelect select)
{
    return ((select == CRG_PLL_TEST_SELECT_PFD) ||
            (select == CRG_PLL_TEST_SELECT_PLL_DIV_128));
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
  * @brief Verify Crg Ls Ip clock select configuration
  * @param select  ls ip clock select value
  * @retval true
  * @retval false
  */
static inline bool IsCrgAPBLsCkSel(CRG_APBLsClkSelect select)
{
    return ((select == CRG_CLK_LS) ||
            (select == CRG_CLK_LS_DIV_2) ||
            (select == CRG_CLK_LS_DIV_4) ||
            (select == CRG_CLK_LS_DIV_8));
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
  * @param preDiv PLL Previous Division
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
  * @param fdDiv PLL FD Division
  * @retval true
  * @retval false
  */
static inline bool IsCrgValidFdDiv(unsigned int clkPfdFreq, unsigned int fdDiv)
{
    unsigned int freq = clkPfdFreq * fdDiv;
    return (freq >= CRG_CLK_VCO_MIN_FREQ) && (freq <= CRG_CLK_VCO_MAX_FREQ);
}

/**
  * @brief Check the PLL PostDiv is valid or not
  * @param clkPllRef PLL Vco clock
  * @param postDiv PLL Post Division
  * @retval true
  * @retval false
  */
static inline bool IsCrgValidPostDiv(unsigned int clkVcoFreq, unsigned int postDiv)
{
    unsigned int freq = clkVcoFreq;
    if (postDiv != 0) {
        freq /= postDiv;
    }
    return (freq >= CRG_CLK_TARGET_MIN_FREQ) && (freq <= CRG_CLK_TARGET_MAX_FREQ);
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
    clk->PERI_CRG0.pll_ref_cksel = (unsigned int)clkSel;
}

/**
  * @brief Get Pll Ref clock selection
  * @param clk             Clock register base address
  * @retval pll_ref_cksel  Ref clock selection
  */
static inline CRG_PllRefClkSelect DCL_CRG_GetPllRefClkSel(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return (CRG_PllRefClkSelect)clk->PERI_CRG0.pll_ref_cksel;
}

/**
  * @brief Set previous division ratio
  * @param clk     Clock register base address
  * @param preDiv  previous division ratio
  * @retval None
  */
static inline void DCL_CRG_SetPllPreDiv(CRG_RegStruct *clk, CRG_PllPreDiv preDiv)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllPreDiv(preDiv));
    clk->PERI_CRG1.pll_prediv = (unsigned int)preDiv;
}

/**
  * @brief Get previous division ratio
  * @param clk      Clock register base address
  * @retval prediv  previous division ratio
  */
static inline CRG_PllPreDiv DCL_CRG_GetPllPreDiv(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return (CRG_PllPreDiv)clk->PERI_CRG1.pll_prediv;
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
    clk->PERI_CRG2.pll_fbdiv = div;
}

/**
  * @brief Get PLL frequency multiplication factor
  * @param clk         Clock register base address
  * @retval pll_fbdiv  Multiplication factor
  */
static inline unsigned int DCL_CRG_GetPllFbDiv(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->PERI_CRG2.pll_fbdiv;
}

/**
  * @brief Set PLL post division ratio
  * @param clk     Clock register base address
  * @param postDiv Post division ratio
  * @retval None
  */
static inline void DCL_CRG_SetPllPostDiv(CRG_RegStruct *clk, CRG_PllPostDiv postDiv)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllPostDiv(postDiv));
    clk->PERI_CRG3.pll_postdiv = (unsigned int)postDiv;
}

/**
  * @brief Get PLL post division ratio
  * @param clk           Clock register base address
  * @retval pll_postdiv  Post division ratio
  */
static inline CRG_PllPostDiv DCL_CRG_GetPllPostDiv(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return (CRG_PllPostDiv)clk->PERI_CRG3.pll_postdiv;
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
    clk->PERI_CRG4.pll_pd = (unsigned int)pd;
}

/**
  * @brief Get PLL power status
  * @param clk  Clock register base address
  * @retval 0: power up, 1: power down
  */
static inline bool DCL_CRG_GetPllPd(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->PERI_CRG4.pll_pd;
}

/**
  * @brief Get pll diagnose test enable
  * @param clk  Clock register base address
  * @retval bool lockdet enable
  */
static inline bool DCL_CRG_GetPllDigEbLockDet(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return (bool)clk->PERI_CRG5.BIT.pll_dig_eb_lockdet;
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
    clk->PERI_CRG7.BIT.core_cksel = select;
}

/**
  * @brief Get core clock selection
  * @param clk  Clock register base address
  * @retval Core clock selection
  */
static inline unsigned int DCL_CRG_GetCoreClkSel(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->PERI_CRG7.BIT.core_cksel;
}

/**
  * @brief Set Pll test clock selection
  * @param clk  Clock register base address
  * @param select  PLL test clock selection
  * @retval None
  */
static inline void DCL_CRG_SetPllTestClockSelect(CRG_RegStruct *clk, CRG_PllTestClkSelect select)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllTestClkSelect(select));
    clk->PERI_CRG6.BIT.pll_test_clk = select;
}

/**
  * @brief Get Pll test clock selection
  * @param clk  Clock register base address
  * @retval PLL test clock selection
  */
static inline unsigned int DCL_CRG_GetPllTestClockSelect(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->PERI_CRG6.BIT.pll_test_clk;
}

/**
  * @brief Set refer clock selection
  * @param clk Clock register base address
  * @param sel Refer clock selection
  * @retval None
  */
static inline void DCL_CRG_SetRefClkSel(CRG_RegStruct *clk, bool switchEn)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    clk->PERI_CRG7.BIT.ck_switchen = switchEn;
}

/**
  * @brief  Get refer clock selection
  * @param clk  Clock register base address
  * @retval unsigned int
  */
static inline unsigned int DCL_CRG_GetRefClkSel(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->PERI_CRG7.BIT.ck_switchen;
}
/**
  * @brief  Enable PVD reset function
  * @param clk  Clock register base address
  * @retval None
  */
static inline void DCL_CRG_PvdResetEnable(CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    clk->PERI_CRG48.BIT.pvd_rst_enable = BASE_CFG_ENABLE;
}

/**
  * @brief  Disable PVD reset function
  * @param clk  Clock register base address
  * @retval None
  */
static inline void DCL_CRG_PvdResetDisable(CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    clk->PERI_CRG48.BIT.pvd_rst_enable = BASE_CFG_DISABLE;
}

/**
  * @brief Coresight auto clock gate select.
  * @param clk  Clock register base address
  * @param clkEn enable select.
  * @retval None
  */
static inline void DCL_CRG_CoresightClkGateSel(CRG_RegStruct *clk, bool clkEn)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    clk->PERI_CRG38.BIT.cs_cken = clkEn;
}

/**
  * @brief  Enable test clock function
  * @param clk  Clock register base address
  * @retval None
  */
static inline void DCL_CRG_TestClkEnable(CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    clk->PERI_CRG47.BIT.test_clk_en = BASE_CFG_ENABLE;
}

/**
  * @brief  Disable test clock function
  * @param clk  Clock register base address
  * @retval None
  */
static inline void DCL_CRG_TestClkDisable(CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    clk->PERI_CRG47.BIT.test_clk_en = BASE_CFG_DISABLE;
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
    CRG_PARAM_CHECK_NO_RET(clkSel >= CRG_TEST_CLK_PLL);
    CRG_PARAM_CHECK_NO_RET(clkSel <= CRG_TEST_CLK_ADC2);
    clk->PERI_CRG47.BIT.test_clk_sel = clkSel;
}
/**
  * @}
  */

/**
 * @}
 */
#endif /* McuMagicTag_CRG_IP_H */
