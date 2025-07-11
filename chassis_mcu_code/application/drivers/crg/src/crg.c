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
 * @file    crg.c
 * @author  MCU Driver Team
 * @brief   CRG module driver.
 * @details This file provides firmware functions to manage the following
 *          functionalities of the CRG.
 *           + Initialization and de-initialization functions
 *           + Config the register of CRG
 *           + Config the register of IP,such as Uart,Timer and so on
 */

/* Includes ------------------------------------------------------------------*/
#include "crg.h"
/* Macro definitions ---------------------------------------------------------*/
#define CRG_HOSC_CTRL2_ADDR   0x10000F04
/* Private Function -----------------------------------------------------------*/
static unsigned int CRG_GetPllRefIni(CRG_PllRefClkSelect pllRefClkSelect);
static unsigned int CRG_GetPreDivValue(CRG_PllPreDiv pllPredDiv);
static unsigned int CRG_GetPllFbDivValue(unsigned int pllFbDiv);
static unsigned int CRG_GetPllPostDivValue(unsigned int pllPostDiv);
static inline unsigned int CRG_GetVcoFreq(void);
static BASE_StatusType CRG_IsValidPllConfig(const CRG_Handle *handle);
static void CRG_GetPllOptConfig(unsigned int targetFreq, unsigned int pllRefFreq, CRG_PllDivCfg *div);
static BASE_StatusType CRG_IsValid1MHzConfig(const CRG_Handle *handle);

static void CRG_IpWithClkSelEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable);
static void CRG_IpWithClkSelClkSelSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int clkSelect);
static void CRG_IpWithClkSelResetSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int reset);
static unsigned int CRG_IpWithClkSelEnableGet(const CHIP_CrgIpMatchInfo *matchInfo);
static unsigned int CRG_IpWithClkSelClkSelGet(const CHIP_CrgIpMatchInfo *matchInfo);
static unsigned int CRG_IpWithClkSelResetGet(const CHIP_CrgIpMatchInfo *matchInfo);

static void CRG_IpWoClkSelEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable);
static void CRG_IpWoClkSelResetSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int reset);
static unsigned int CRG_IpWoClkSelEnableGet(const CHIP_CrgIpMatchInfo *matchInfo);
static unsigned int CRG_IpWoClkSelResetGet(const CHIP_CrgIpMatchInfo *matchInfo);

static void CRG_AdcEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable);
static void CRG_AdcDivSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int div);
static void CRG_AdcClkSelectSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int clkSelect);
static unsigned int CRG_AdcEnableGet(const CHIP_CrgIpMatchInfo *matchInfo);
static unsigned int CRG_AdcClkSelectGet(const CHIP_CrgIpMatchInfo *matchInfo);
static unsigned int CRG_AdcDivGet(const CHIP_CrgIpMatchInfo *matchInfo);

static void CRG_EfcEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable);
static unsigned int CRG_EfcEnableGet(const CHIP_CrgIpMatchInfo *matchInfo);
static void CRG_AnaEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable);
static unsigned int CRG_AnaEnableGet(const CHIP_CrgIpMatchInfo *matchInfo);

#ifndef FPGA
static unsigned int CRG_GetLsIpFreq(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int baseClkRate);
static unsigned int CRG_GetAdcIpFreq(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int baseClkRate,
                                     unsigned int coreClkFreq);
#endif

typedef CHIP_CrgIpMatchInfo *(*FindFunc)(const void *baseAddress);
typedef void (*SetFunc)(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int value);
typedef unsigned int (*GetFunc)(const CHIP_CrgIpMatchInfo *matchInfo);

typedef struct {
    CHIP_CrgIpType type;
    SetFunc    resetSet;
    SetFunc    enableSet;
    SetFunc    clkSelSet;
    SetFunc    clkDivSet;
    GetFunc    resetGet;
    GetFunc    enableGet;
    GetFunc    clkSelGet;
    GetFunc    clkDivGet;
} CRG_IpProc;

static CRG_IpProc g_ipClkProc[CRG_IP_MAX_TYPE] = {
    {CRG_IP_WITH_CLK_SEL,
     CRG_IpWithClkSelResetSet, CRG_IpWithClkSelEnableSet, CRG_IpWithClkSelClkSelSet, NULL,
     CRG_IpWithClkSelResetGet, CRG_IpWithClkSelEnableGet, CRG_IpWithClkSelClkSelGet, NULL},
    {CRG_IP_NONE_CLK_SEL,
     CRG_IpWoClkSelResetSet, CRG_IpWoClkSelEnableSet, NULL, NULL,
     CRG_IpWoClkSelResetGet, CRG_IpWoClkSelEnableGet, NULL, NULL},
    {CRG_IP_CAN,
     CRG_IpWoClkSelResetSet, CRG_IpWoClkSelEnableSet, NULL, NULL,
     CRG_IpWoClkSelResetGet, CRG_IpWoClkSelEnableGet, NULL, NULL},
    {CRG_IP_ADC,
     NULL, CRG_AdcEnableSet, CRG_AdcClkSelectSet, CRG_AdcDivSet,
     NULL, CRG_AdcEnableGet, CRG_AdcClkSelectGet, CRG_AdcDivGet},
    {CRG_IP_EFC,
     NULL, CRG_EfcEnableSet, NULL, NULL,
     NULL, CRG_EfcEnableGet, NULL, NULL},
    {CRG_IP_IWDG,
     CRG_IpWoClkSelResetSet, CRG_IpWoClkSelEnableSet, NULL, NULL,
     CRG_IpWoClkSelResetGet, CRG_IpWoClkSelEnableGet, NULL, NULL},
    {CRG_IP_ANA,
     CRG_IpWoClkSelResetSet, CRG_AnaEnableSet, NULL, NULL,
     CRG_IpWoClkSelResetGet, CRG_AnaEnableGet, NULL, NULL}
};

static CRG_RegStruct *g_crgBaseAddr;
static unsigned char g_anaEnableFlag = 0;

/* Public Function -----------------------------------------------------------*/
/**
  * @brief Clock Init
  * @param handle CRG Handle
  * @retval BASE_STATUS_ERROR  Parameter Check fail
  * @retval BASE_STATUS_OK     Success
  */
BASE_StatusType HAL_CRG_Init(const CRG_Handle *handle)
{
    CRG_ASSERT_PARAM(handle != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(handle->baseAddress));
    /* Check the validity of PLL-related parameters. */
    CRG_PARAM_CHECK_WITH_RET(IsCrgPllRefClkSelect(handle->pllRefClkSelect), BASE_STATUS_ERROR);
    CRG_PARAM_CHECK_WITH_RET(IsCrgPllPreDiv(handle->pllPreDiv), BASE_STATUS_ERROR);
    CRG_PARAM_CHECK_WITH_RET(IsCrgPllFbDiv(handle->pllFbDiv), BASE_STATUS_ERROR);
    CRG_PARAM_CHECK_WITH_RET(IsCrgPllPostDiv(handle->pllPostDiv), BASE_STATUS_ERROR);
    CRG_PARAM_CHECK_WITH_RET(IsCrgPllPostDiv2(handle->handleEx.pllPostDiv2), BASE_STATUS_ERROR);
    /* Check the Clock Source and Frequency Divider of the 1 MHz Clock. */
    CRG_PARAM_CHECK_WITH_RET(IsCrg1MCkSel(handle->handleEx.clk1MSelect), BASE_STATUS_ERROR);
    CRG_PARAM_CHECK_WITH_RET(IsCrg1MCkDiv(handle->handleEx.clk1MDiv), BASE_STATUS_ERROR);
    CRG_PARAM_CHECK_WITH_RET(IsCrgCoreCkSel(handle->coreClkSelect), BASE_STATUS_ERROR);

    *(unsigned int *)CRG_HOSC_CTRL2_ADDR = 0x306E; /* Optimized HOSC temperature drift performance parameter. */

    CRG_RegStruct *reg = handle->baseAddress;
    g_crgBaseAddr = (void *)reg;
    /* Check the validity of the PLL parameter configuration. */
    if (CRG_IsValidPllConfig(handle) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    /* Disable the write protection function of the CRG register. */
    DCL_SYSCTRL_CrgWriteProtectionDisable();

    reg->PERI_CRG0.BIT.pll_ref_cksel   = handle->pllRefClkSelect;
    reg->PERI_CRG1.BIT.pll_prediv      = handle->pllPreDiv;
    reg->PERI_CRG2.BIT.pll_fbdiv       = handle->pllFbDiv;
    reg->PERI_CRG3.BIT.pll_postdiv1    = handle->pllPostDiv;
    reg->PERI_CRG3.BIT.pll_postdiv2    = handle->handleEx.pllPostDiv2;
    reg->PERI_CRG4.BIT.pll_pd          = BASE_CFG_UNSET;

    while (reg->PERI_CRG7.BIT.pll_lock != BASE_CFG_SET) {
        ;  /* Wait for PLL to lock */
    }

    DCL_SYSCTRL_CrgWriteProtectionEnable();
    /* Check the 1MHz clock parameter configuration. */
    if (CRG_IsValid1MHzConfig(handle) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    while (reg->HOSC_LOCK.BIT.hosc_lock != BASE_CFG_SET) {
        ; /* Wait for HOSC to lock */
    }
    /* Set the Clock Source and Frequency Divider of the 1 MHz Clock. */
    reg->PERI_CRG67.BIT.clk_1m_div = handle->handleEx.clk1MDiv;
    reg->PERI_CRG66.BIT.clk_1m_ini_cksel = handle->handleEx.clk1MSelect;
    return BASE_STATUS_OK;
}

/**
 * @brief Set Crg Core clock by target frequecy
 * @param handle CRG handle
 * @param targetFreq Target Frequency
 * @retval BASE_STATUS_ERROR  Parameter Check fail
 * @retval BASE_STATUS_OK     Success
 */
BASE_StatusType HAL_CRG_InitWithTargetFrequence(const CRG_Handle *handle, unsigned int targetFreq)
{
    CRG_ASSERT_PARAM(handle != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(handle->baseAddress));
    CRG_PARAM_CHECK_WITH_RET(IsCrgPllRefClkSelect(handle->pllRefClkSelect), BASE_STATUS_ERROR);
    CRG_PARAM_CHECK_WITH_RET((targetFreq <= CRG_CLK_TARGET_MAX_FREQ), BASE_STATUS_ERROR);

    CRG_Handle crgHandle;
    CRG_PllDivCfg divCfg;
    unsigned int pllRefFreq;
    /* Check the validity of the external crystal oscillator frequency. */
    if ((handle->pllRefClkSelect == CRG_PLL_REF_CLK_SELECT_XTAL) && \
               (XTRAIL_FREQ > 30000000U)) { /* The maximum of the external clock source is 30000000U. */
        return BASE_STATUS_ERROR;
    }
    /* Obtain the clock frequency based on the clock source. */
    pllRefFreq = (handle->pllRefClkSelect == CRG_PLL_REF_CLK_SELECT_HOSC) ? HOSC_FREQ : XTRAIL_FREQ;
    CRG_GetPllOptConfig(targetFreq, pllRefFreq, &divCfg);
    crgHandle = *handle;
    crgHandle.pllPreDiv  = divCfg.PreDiv;
    crgHandle.pllFbDiv   = divCfg.fbDiv;
    crgHandle.pllPostDiv = divCfg.postDiv;
    return HAL_CRG_Init(&crgHandle);
}

/**
  * @brief Clock Deinit
  * @param handle CRG Handle
  * @retval BASE_STATUS_OK
  */
BASE_StatusType HAL_CRG_DeInit(const CRG_Handle *handle)
{
    CRG_ASSERT_PARAM(handle != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(handle->baseAddress));
    CRG_RegStruct *reg = handle->baseAddress;
    DCL_SYSCTRL_CrgWriteProtectionDisable();

    reg->PERI_CRG0.BIT.pll_ref_cksel   = 0x0;  /* 0x0: default value */
    reg->PERI_CRG1.BIT.pll_prediv      = 0x3;  /* 0x3: default value */
    reg->PERI_CRG2.BIT.pll_fbdiv       = 0x30; /* 0x30: default value */
    reg->PERI_CRG3.BIT.pll_postdiv1     = 0x1;  /* 0x0: default value */
    reg->PERI_CRG4.BIT.pll_pd          = 0x1;  /* 0x1: default value */

    DCL_SYSCTRL_CrgWriteProtectionEnable();
    
    reg->PERI_CRG67.BIT.clk_1m_div = 0x29; /* 0x29: default value */
    reg->PERI_CRG66.BIT.clk_1m_ini_cksel = 0x0; /* 0x0: default value */
    return BASE_STATUS_OK;
}

/**
  * @brief Get Clock Config
  * @param handle CRG Handle
  * @retval BASE_STATUS_OK    Success
  */
BASE_StatusType HAL_CRG_GetConfig(CRG_Handle *handle)
{
    CRG_ASSERT_PARAM(handle != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(handle->baseAddress));
    /* Obtains configuration parameters from registers. */
    CRG_RegStruct *reg = handle->baseAddress;
    handle->pllRefClkSelect = reg->PERI_CRG0.BIT.pll_ref_cksel;
    handle->pllPreDiv       = reg->PERI_CRG1.BIT.pll_prediv;
    handle->pllFbDiv        = reg->PERI_CRG2.BIT.pll_fbdiv;
    handle->pllPostDiv      = reg->PERI_CRG3.BIT.pll_postdiv1;
    handle->handleEx.pllPostDiv2 = reg->PERI_CRG3.BIT.pll_postdiv2;
    /* Enable the PLL and start the PLL output clock frequency. */
    handle->pllPd           = reg->PERI_CRG4.BIT.pll_pd;
    handle->coreClkSelect   = reg->PERI_CRG64.BIT.clk_pst1_sw_sel;
    /* Get the 1MHz clock select and frequency division. */
    handle->handleEx.clk1MDiv = reg->PERI_CRG67.BIT.clk_1m_div;
    handle->handleEx.clk1MSelect = reg->PERI_CRG66.BIT.clk_1m_ini_cksel;
    return BASE_STATUS_OK;
}

/**
  * @brief Set CRG Core Clock Select
  * @param handle CRG Handle
  * @retval BASE_STATUS_OK  Success
  * @retval BASE_STATUS_ERROR Paramter check fail
  */
BASE_StatusType HAL_CRG_SetCoreClockSelect(CRG_Handle *handle)
{
    CRG_ASSERT_PARAM(handle != 0);
    CRG_ASSERT_PARAM(IsCRGInstance(handle->baseAddress));
    CRG_PARAM_CHECK_WITH_RET(IsCrgCoreCkSel(handle->coreClkSelect), BASE_STATUS_ERROR);

    CRG_RegStruct *reg = handle->baseAddress;
   /* The write protection of the CRG register needs to be disabled. */
    DCL_SYSCTRL_CrgWriteProtectionDisable();
    DCL_CRG_SetCoreClkSel(reg, handle->coreClkSelect);
    DCL_SYSCTRL_CrgWriteProtectionEnable();

    return BASE_STATUS_OK;
}

/**
 * @brief Get PLL Clock Frequence
 * @param None
 * @retval unsigned int PLL clock frequency
 */
static inline unsigned int CRG_GetVcoFreq(void)
{
    unsigned int freq;
    unsigned int regFbdiv;
    CRG_RegStruct *crg = g_crgBaseAddr;

    CRG_ASSERT_PARAM(IsCRGInstance(crg));
    CRG_ASSERT_PARAM((XTRAIL_FREQ <= 30000000U)); /* The maximum of the external clock source is 30000000U. */

    freq = CRG_GetPllRefIni(crg->PERI_CRG0.BIT.pll_ref_cksel);
    freq /= CRG_GetPreDivValue(crg->PERI_CRG1.BIT.pll_prediv);
    regFbdiv = CRG_GetPllFbDivValue(crg->PERI_CRG2.BIT.pll_fbdiv); /* Get the value of the fbdiv register. */
    freq *= (regFbdiv >= 0x06) ? regFbdiv : 0x06; /* 0x0-0x6: divided by 0x6 */
    return freq;
}

/**
 * @brief Get PLL Clock Frequence
 * @param None
 * @retval unsigned int PLL clock frequency
 */
unsigned int HAL_CRG_GetPllFreq(void)
{
    unsigned int freq;
    unsigned int pllPostDivValue;
    CRG_RegStruct *crg = g_crgBaseAddr;

    CRG_ASSERT_PARAM(IsCRGInstance(crg));
    freq = CRG_GetVcoFreq();
    pllPostDivValue = CRG_GetPllPostDivValue((CRG_PllPostDiv)crg->PERI_CRG3.BIT.pll_postdiv1);
    /* Calculate the PLL output clock frequency based on the VCO clock frequency and post-division coefficient. */
    if (pllPostDivValue != 0) {
        freq /= pllPostDivValue;
    }
    return freq;
}

/**
 * @brief Get Core Clock Frequence
 * @param None
 * @retval unsigned int  Core clock frequency
 */
unsigned int HAL_CRG_GetCoreClkFreq(void)
{
    unsigned int freq;
    unsigned int coreClkSelect;
    CRG_RegStruct *crg = g_crgBaseAddr;

    CRG_ASSERT_PARAM(IsCRGInstance(crg));
    coreClkSelect = crg->PERI_CRG64.BIT.clk_pst1_sw_sel;
    switch (coreClkSelect) {
        case CRG_CORE_CLK_SELECT_HOSC: /* The clock source is an internal high-speed clock. */
            freq = HOSC_FREQ;
            break;

        case CRG_CORE_CLK_SELECT_TCXO: /* The clock source is the external crystal oscillator clock. */
            freq = XTRAIL_FREQ;
            break;

        case CRG_CORE_CLK_SELECT_PLL: /* The clock source is the PLL. */
            freq = HAL_CRG_GetPllFreq();
            break;

        default:
            freq = LOSC_FREQ;
            break;
    }
    return freq;
}

/**
 * @brief Get Clock Frequence
 * @param handle CRG Handle
 * @retval Frequece of IP
 */
unsigned int HAL_CRG_GetIpFreq(const void *baseAddress)
{
    CRG_ASSERT_PARAM(baseAddress != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(g_crgBaseAddr));
#ifdef FPGA
    /* Use this function to obtain the clock frequency during the FPGA phase. */
    return CHIP_GetIpFreqHz(baseAddress);
#else
    unsigned int hclk = HAL_CRG_GetCoreClkFreq();
    unsigned int freq = LOSC_FREQ;
    unsigned int coreClkFreq;
    /* Get the CRG type of the target IP. */
    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if (p == NULL) {
        return freq;
    }
    switch (p->type) {
        case CRG_IP_WITH_CLK_SEL:
            /* Get the clock frequency of the module with the frequency division function. */
            freq = CRG_GetLsIpFreq(p, hclk);
            break;

        case CRG_IP_NONE_CLK_SEL:
        case CRG_IP_EFC:
        case CRG_IP_ANA:
            freq = hclk; /* Returns the internal high speed clock frequency. */
            break;

        case CRG_IP_CAN:
            freq = CRG_GetPllRefIni(g_crgBaseAddr->PERI_CRG0.BIT.pll_ref_cksel);
            break;

        case CRG_IP_ADC:
            /* Get core clock frequence for calculating the ADC clock frequency. */
            coreClkFreq = HAL_CRG_GetCoreClkFreq();
            freq = CRG_GetAdcIpFreq(p, CRG_GetVcoFreq(), coreClkFreq);
            break;

        case CRG_IP_IWDG: /* The IWDG clock frequency is an internal low-speed clock. */
        default:
            break;
    }
    if (freq == 0) {
        freq = LOSC_FREQ;
    }
    return freq;
#endif
}

/**
  * @brief Enable clock of ip
  * @param baseAddress Ip base address
  * @param enable enable mask
  * @retval BASE_STATUS_ERROR       Can't find the Match or operation is not support
  * @retval BASE_STATUS_OK          Operation Success
  */
BASE_StatusType HAL_CRG_IpEnableSet(const void *baseAddress, unsigned int enable)
{
    CRG_ASSERT_PARAM(baseAddress != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(g_crgBaseAddr));
    /* Check the validity of the input parameters. */
    CRG_PARAM_CHECK_WITH_RET((enable == IP_CLK_ENABLE || enable == IP_CLK_DISABLE), BASE_STATUS_ERROR);
    /* Get the CRG type of the target IP. */
    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].enableSet == NULL) {
        return BASE_STATUS_ERROR;
    }
    g_ipClkProc[p->type].enableSet(p, enable);
    return BASE_STATUS_OK;
}

/**
  * @brief Get clock enable status of ip
  * @param baseAddress Ip base address
  * @param enable parameter out for ip enable status
  * @retval BASE_STATUS_ERROR       Can't find the Match or operation is not support
  * @retval BASE_STATUS_OK          Operation Success
  */
BASE_StatusType HAL_CRG_IpEnableGet(const void *baseAddress, unsigned int *enable)
{
    CRG_ASSERT_PARAM(baseAddress != NULL);
    CRG_ASSERT_PARAM(enable != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(g_crgBaseAddr));
    /* Get the CRG type of the target IP. */
    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type < 0) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].enableGet == NULL) {
        return BASE_STATUS_ERROR;
    }
    *enable = g_ipClkProc[p->type].enableGet(p); /* Returns the module clock enable status. */
    return BASE_STATUS_OK;
}

/**
  * @brief Set clock select ip
  * @param baseAddress Ip base address
  * @param select clock select, @see CRG_APBLsClkSelect for ip in apb_ls_subsys or CRG_AdcClkSelect for adc
  * @retval BASE_STATUS_OK    success
  * @retval BASE_STATUS_ERROR fail
  */
BASE_StatusType HAL_CRG_IpClkSelectSet(const void *baseAddress, unsigned int select)
{
    CRG_ASSERT_PARAM(baseAddress != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(g_crgBaseAddr));

    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].clkSelSet == NULL) {
        return BASE_STATUS_ERROR;
    }
    g_ipClkProc[p->type].clkSelSet(p, select); /* Clock selection of the configuration module. */
    return BASE_STATUS_OK;
}

/**
  * @brief Get clock select of ip
  * @param baseAddress Ip base address
  * @param clkSel Get clkSet value
  * @retval BASE_STATUS_OK
  * @retval BASE_STATUS_ERROR  Match Fail or Not support
  */
BASE_StatusType HAL_CRG_IpClkSelectGet(const void *baseAddress, unsigned int *clkSel)
{
    CRG_ASSERT_PARAM(baseAddress != NULL);
    CRG_ASSERT_PARAM(clkSel != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(g_crgBaseAddr));
    /* Get the CRG type of the target IP. */
    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].clkSelGet == NULL) {
        return BASE_STATUS_ERROR;
    }
    *clkSel = g_ipClkProc[p->type].clkSelGet(p); /* Obtains the module clock selection. */
    return BASE_STATUS_OK;
}

/**
  * @brief Reset/Set clock of ip
  * @param baseAddress Ip base address
  * @param reset Set reset value
  * @retval BASE_STATUS_OK     Success
  * @retval BASE_STATUS_ERROR  Match Fail or Not support
  */
BASE_StatusType HAL_CRG_IpClkResetSet(const void *baseAddress, unsigned int reset)
{
    CRG_ASSERT_PARAM(baseAddress != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(g_crgBaseAddr));
    CRG_PARAM_CHECK_WITH_RET((reset == BASE_CFG_SET || reset == BASE_CFG_UNSET), BASE_STATUS_ERROR);
    /* Get the CRG type of the target IP. */
    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].resetSet == NULL) {
        return BASE_STATUS_ERROR;
    }
    g_ipClkProc[p->type].resetSet(p, reset); /* Configure the reset value of the module clock. */
    return BASE_STATUS_OK;
}

/**
  * @brief Get clock select of ip
  * @param baseAddress Ip base address
  * @param reset Get reset value
  * @retval BASE_STATUS_OK  Success
  * @retval BASE_CFG_UNSET  Match Fail or Not support
  */
BASE_StatusType HAL_CRG_IpClkResetGet(const void *baseAddress, unsigned int *reset)
{
    CRG_ASSERT_PARAM(baseAddress != NULL);
    CRG_ASSERT_PARAM(reset != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(g_crgBaseAddr));
    /* Get the CRG type of the target IP. */
    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].resetGet == NULL) {
        return BASE_STATUS_ERROR;
    }
    *reset = g_ipClkProc[p->type].resetGet(p); /* Query the reset status of the module clock. */
    return BASE_STATUS_OK;
}

/**
  * @brief Reset/Set clock of ip
  * @param baseAddress Ip base address
  * @param div set div value
  * @retval BASE_STATUS_OK     Success
  * @retval BASE_STATUS_ERROR  Match Fail or Not support
  */
BASE_StatusType HAL_CRG_IpClkDivSet(const void *baseAddress, unsigned int div)
{
    CRG_ASSERT_PARAM(baseAddress != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(g_crgBaseAddr));

    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].clkDivSet == NULL) {
        return BASE_STATUS_ERROR;
    }
    g_ipClkProc[p->type].clkDivSet(p, div); /* Configure the clock frequency divider of the module. */
    return BASE_STATUS_OK;
}

/**
  * @brief Get clock select of ip
  * @param baseAddress Ip base address
  * @param div get div value
  * @retval BASE_STATUS_OK     Success
  * @retval BASE_STATUS_ERROR  Match Fail or Not support
  */
BASE_StatusType HAL_CRG_IpClkDivGet(const void *baseAddress, unsigned int *div)
{
    CRG_ASSERT_PARAM(baseAddress != NULL);
    CRG_ASSERT_PARAM(div != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(g_crgBaseAddr));
    /* Get the CRG type of the target IP. */
    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].clkDivGet == NULL) {
        return BASE_STATUS_ERROR;
    }
    *div = g_ipClkProc[p->type].clkDivGet(p); /* Get the clock frequency division coefficient of a module. */
    return BASE_STATUS_OK;
}

/**
  * @brief PVD reset function enable switch
  * @param pvd reset enable select
  * @retval None
  */
void HAL_CRG_PvdResetEnable(bool enable)
{
    CRG_ASSERT_PARAM(IsCRGInstance(g_crgBaseAddr));
    g_crgBaseAddr->PERI_CRG65.BIT.pvd_rst_enable = enable;
}

/**
  * @brief Based on the target frequency, obtain the post division of the pll
  * @param targetFreq Target frequency
  * @param clkPfdFreq The freq of Pll after frequency multiplication
  * @param divCfg Output Pll division config
  * @retval None
  */
static void CRG_GetPllTargetFreqPostDiv(unsigned int targetFreq, unsigned int preDiv, unsigned int fbDiv,
                                        unsigned int clkPfdFreq, CRG_PllDivCfg *divCfg)
{
    unsigned int clkVcoFreq;
    unsigned int freq;
    unsigned int delta;
    unsigned int minDelta = 0xFFFFFFFF; /* Set the maximum value and initialize the default value. */
    unsigned int postDiv;

    clkVcoFreq = clkPfdFreq * fbDiv;
    for (unsigned int i = CRG_PLL_POSTDIV_1; i <= CRG_PLL_POSTDIV_8; i++) {
        postDiv = i;
        /* Check whether the frequency after frequency division is valid. */
        if (!IsCrgValidPostDiv(clkVcoFreq, postDiv)) {
            continue;
        }
        freq = clkVcoFreq / (postDiv + 1);
        delta = (targetFreq >= freq) ? targetFreq - freq : freq - targetFreq;
        if (delta < minDelta) { /* Updating Configuration Parameter Values. */
            minDelta = delta;
            divCfg->PreDiv = preDiv;
            divCfg->fbDiv = fbDiv;
            divCfg->postDiv = i;
        }
    }
}

/**
  * @brief Based on the target frequency, obtain the optimal frequency division coefficient of the pll
  * @param targetFreq Target frequency
  * @param pllRefFreq Pll refer clock frequency
  * @param divCfg Output Pll division config
  * @retval None
  */
static void CRG_GetPllOptConfig(unsigned int targetFreq, unsigned int pllRefFreq, CRG_PllDivCfg *divCfg)
{
    unsigned int preDiv[] = {CRG_PLL_PREDIV_1, CRG_PLL_PREDIV_2, CRG_PLL_PREDIV_3, CRG_PLL_PREDIV_4, CRG_PLL_PREDIV_5,
                             CRG_PLL_PREDIV_6, CRG_PLL_PREDIV_7, CRG_PLL_PREDIV_8};
    unsigned int preDivOut;
    unsigned int clkPfdFreq;
    /* Configuring PLL Parameter Initialization. */
    divCfg->PreDiv  = CRG_PLL_PREDIV_1;
    divCfg->fbDiv   = CRG_PLL_FBDIV_MIN;
    divCfg->postDiv = CRG_PLL_POSTDIV_1;

    for (unsigned int i = 0; i < sizeof(preDiv) / sizeof(preDiv[0]); ++i) {
        preDivOut = CRG_GetPreDivValue(preDiv[i]);
        /* Check whether the frequency value after pre-division is valid. */
        if (!IsCrgValidPreDiv(pllRefFreq, preDivOut)) {
            continue;
        }
        clkPfdFreq = pllRefFreq / preDivOut;

        for (unsigned int j = CRG_PLL_FBDIV_MIN; j <= CRG_PLL_FBDIV_MAX; ++j) {
            /* Check whether the frequency value after frequency multiplication is valid. */
            if (!IsCrgValidFdDiv(clkPfdFreq, j)) {
                continue;
            }
            /* Get the post division of the pll. */
            CRG_GetPllTargetFreqPostDiv(targetFreq, preDiv[i], j, clkPfdFreq, divCfg);
        }
    }
}

#ifndef FPGA
/**
  * @brief Get Ls Ip Clock Frequence
  * @param matchInfo match info
  * @param baseClkRate clock rate
  * @retval Ip Frequence
  */
static unsigned int CRG_GetLsIpFreq(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int baseClkRate)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);

    unsigned int clkSel;
    CRG_IpProc *proc;
    
    if (matchInfo->type == CRG_IP_WITH_CLK_SEL) { /* type in legal scope */
        proc = &g_ipClkProc[matchInfo->type];
    } else {
        return 0;
    }

    if (proc->clkSelGet == NULL) {
        return 0;
    }
    clkSel = proc->clkSelGet(matchInfo);
    return (baseClkRate >> clkSel);
}

/**
  * @brief Get ADC Clock Frequence
  * @param matchInfo match info
  * @param baseClkRate clock rate
  * @param coreClkFreq core clock rate
  * @retval Ip Frequence
  */
static unsigned int CRG_GetAdcIpFreq(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int baseClkRate,
                                     unsigned int coreClkFreq)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);
    
    unsigned int clkSel;
    unsigned int clkDiv;
    unsigned int pst2Div;
    unsigned int freq = 0;
    
    /* Obtains the clock source selection of the ADC. */
    const CRG_IpProc *proc = &g_ipClkProc[matchInfo->type];
    if (proc->clkSelGet == NULL) {
        return 0;
    }
    clkSel = proc->clkSelGet(matchInfo);
    /* Calculate the frequency from the ADC's clock source. */
    if (clkSel == CRG_ADC_CLK_SYN_CORE) {
        freq = coreClkFreq;
    } else if (clkSel == CRG_ADC_CLK_ASYN_HOSC) {
        freq = HOSC_FREQ;
    } else if (clkSel == CRG_ADC_CLK_ASYN_TCXO) {
        /* The maximum speed of the external clock source is 30000000U. */
        freq = (XTRAIL_FREQ > 30000000U) ? 0 : XTRAIL_FREQ;
    } else if (clkSel == CRG_ADC_CLK_ASYN_PLL_DIV) {
        pst2Div = CRG_GetPllPostDivValue((CRG_PllPostDiv)g_crgBaseAddr->PERI_CRG3.BIT.pll_postdiv2);
        freq = baseClkRate / pst2Div;
    }

    /* Obtain the frequency divider based on the ADC clock source. */
    if (proc->clkDivGet == NULL) {
        return 0;
    }
    clkDiv = proc->clkDivGet(matchInfo);
    /* Calculate the clock frequency of the ADC. */
    return (freq / (clkDiv + 1));
}

#endif
/**
 * @brief Check is Valid Pll Config
 * @param  CRG_Handle CRG handle
  * @retval BASE_STATUS_OK     Check Success
  * @retval BASE_STATUS_ERROR  Check Fail
 */
static BASE_StatusType CRG_IsValidPllConfig(const CRG_Handle *handle)
{
    unsigned int preDiv;
    unsigned int freq;

    freq = CRG_GetPllRefIni(handle->pllRefClkSelect);
    preDiv = CRG_GetPreDivValue(handle->pllPreDiv);
    /* Check the validity of the prescaled clock frequency. */
    if (!IsCrgValidPreDiv(freq, preDiv)) {
        return BASE_STATUS_ERROR;
    }
    freq /= preDiv;
    /* Check the validity of the clock frequency after frequency multiplication. */
    if (!IsCrgValidFdDiv(freq, handle->pllFbDiv)) {
        return BASE_STATUS_ERROR;
    }
    freq *= (handle->pllFbDiv > 0x06) ? handle->pllFbDiv : 0x06; /* 0x0-0x6: divided by 0x6 */
    /* Check whether the PLL output frequency is valid. */
    if (IsCrgValidPostDiv(freq, handle->pllPostDiv) && IsCrgValidPostDiv2(freq, handle->handleEx.pllPostDiv2)) {
        return BASE_STATUS_OK;
    }
    return BASE_STATUS_ERROR;
}

/**
 * @brief Check is Valid 1MHz Config
 * @param  CRG_Handle CRG handle
  * @retval BASE_STATUS_OK     Check Success
  * @retval BASE_STATUS_ERROR  Check Fail
 */
static BASE_StatusType CRG_IsValid1MHzConfig(const CRG_Handle *handle)
{
    unsigned int freq;
    /* Get the ref frequency of the 1 MHz clock. */
    freq = (handle->handleEx.clk1MSelect == CRG_1M_CLK_SELECT_HOSC) ? HOSC_FREQ : XTRAIL_FREQ;
    /* Check whether the 1MHz output frequency is valid. */
    if ((freq / (handle->handleEx.clk1MDiv + 1)) == CRG_FREQ_1MHz) {
        return BASE_STATUS_OK;
    }
    return BASE_STATUS_ERROR;
}

/**
 * @brief Get clock frequence
 * @param  crg CRG_RegStruct
 * @retval The frequence fo clock
 */
static inline unsigned int CRG_GetPllRefIni(CRG_PllRefClkSelect pllRefClkSelect)
{
    /* The maximum speed of the external clock source is 30000000U. */
    if (pllRefClkSelect == CRG_PLL_REF_CLK_SELECT_XTAL && XTRAIL_FREQ > 30000000U) {
        return 0;
    }
    return (pllRefClkSelect == (unsigned int)CRG_PLL_REF_CLK_SELECT_HOSC) ? HOSC_FREQ : XTRAIL_FREQ;
}

/**
 * @brief Get previous division Value before PLL
 * @param  crg CRG_RegStruct
 * @retval Previous Div value
 */
static inline unsigned int CRG_GetPreDivValue(CRG_PllPreDiv pllPredDiv)
{
    unsigned int preDiv;
    if (pllPredDiv <= CRG_PLL_PREDIV_1) { /* 0 or 1 returns PLL_PREDIV_OUT_1. */
        preDiv = PLL_PREDIV_OUT_1;
    } else {
        preDiv = pllPredDiv + 1;
    }
    return preDiv;
}

/**
 * @brief Get PLL loop divider ratio
 * @param  crg CRG_RegStruct
 * @retval PLL loop divider ratio
 */
static inline unsigned int CRG_GetPllFbDivValue(unsigned int pllFbDiv)
{
    unsigned int div = pllFbDiv;
    /* Check the validity of the minimum frequency multiplication parameter. */
    if (div < CRG_PLL_FBDIV_MIN) {
        div = CRG_PLL_FBDIV_MIN;
    }
    /* Check the validity of the maximum frequency multiplication parameter. */
    if (div > CRG_PLL_FBDIV_MAX) {
        div = CRG_PLL_FBDIV_MAX;
    }
    return div;
}

/**
 * @brief Get post division Value after PLL
 * @param  crg CRG_RegStruct
 * @retval Previous Div value
 */
static inline unsigned int CRG_GetPllPostDivValue(unsigned int pllPostDiv)
{
    unsigned int div = pllPostDiv;
    if (div > CRG_PLL_POSTDIV_8) {
        div = (CRG_PLL_POSTDIV_8 + 1); /* If the postdiv is greater than 8, set this postdiv to 8. */
    } else {
        div += 1;
    }
    return div;
}

/**
 * @brief Enable Set for IP in APB_LS_SUBSYS
 * @param matchInfo IP with Clock select match info
 * @param enable  BASE_CFG_SET or BASE_CFG_UNSET
 * @retval None
 */
static void CRG_IpWithClkSelEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);

    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_IpWithClkSelectCfg *p = (CRG_IpWithClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    if ((enable & IP_CLK_ENABLE) == IP_CLK_ENABLE) {
        p->BIT.cken = BASE_CFG_SET;
        p->BIT.srst_req = BASE_CFG_UNSET;   /* Enable with soft reset disable */
    } else {
        p->BIT.cken = BASE_CFG_UNSET;
        p->BIT.srst_req = BASE_CFG_SET;   /* Enable with soft reset disable */
    }
}

/**
 * @brief Get Enable status for IP in APB_LS_SUBSYS
 * @param matchInfo IP with Clock select match info
 * @param unsigned int 0: disable, 1: enable
 * @retval Clock enable status
 */
static unsigned int CRG_IpWithClkSelEnableGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);
    /* Get Clock enable status or undo for target ip. */
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_IpWithClkSelectCfg *p = (CRG_IpWithClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    return p->BIT.cken;
}

/**
 * @brief Reset or undo Reset for IP in APB_LS_SUBSYS
 * @param matchInfo IP with Clock select match info
 * @param reset  BASE_CFG_SET or BASE_CFG_UNSET
 * @retval None
 */
static void CRG_IpWithClkSelResetSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int reset)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);
    /* Set Clock reset or undo for target ip. */
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_IpWithClkSelectCfg *p = (CRG_IpWithClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    p->BIT.srst_req = (reset & BASE_CFG_SET) ? BASE_CFG_SET : BASE_CFG_UNSET;
}

/**
 * @brief Get Reset status for IP in APB_LS_SUBSYS
 * @param matchInfo IP with Clock select match info
 * @param unsigned int 0: disable, 1: enable
 * @retval Clock reset status
 */
static unsigned int CRG_IpWithClkSelResetGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);
    /* Get Clock reset status for target ip. */
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_IpWithClkSelectCfg *p = (CRG_IpWithClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    return p->BIT.srst_req;
}

/**
 * @brief Set Clock Select for IP in APB_LS_SUBSYS
 * @param matchInfo IP with Clock select match info
 * @param clkSelect @see CRG_APBLsClkSelect
 * @param unsigned int BASE_CFG_SET or BASE_CFG_UNSET
 * @retval None
 */
static void CRG_IpWithClkSelClkSelSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int clkSelect)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);
    CRG_PARAM_CHECK_NO_RET(IsCrgAHBCkSel(clkSelect));
    /* Set Clock Select for target ip. */
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_IpWithClkSelectCfg *p = (CRG_IpWithClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    p->BIT.cksel = clkSelect;
}

/**
 * @brief Get Clock Select for IP in APB_LS_SUBSYS
 * @param matchInfo IP with Clock select match info
 * @retval Clock Select @see CRG_APBLsClkSelect
 */
static unsigned int CRG_IpWithClkSelClkSelGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);
    /* Get Clock Select for target ip. */
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_IpWithClkSelectCfg *p = (CRG_IpWithClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    return p->BIT.cksel;
}

/**
 * @brief Enable Set of IP in APB_HS_SUBSYS
 * @param matchInfo IP without Clock select match info
 * @param enable BASE_CFG_SET or BASE_CFG_UNSET
 * @retval None
 */
static void CRG_IpWoClkSelEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);

    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_IpWoClkSelectCfg *p = (CRG_IpWoClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    CRG_IpWoClkSelectCfg cfg;
    cfg.value = p->value;
    if (enable & IP_CLK_ENABLE) {     /* Set enable of target ip. */
        cfg.BIT.clkEnMask |= 1 << matchInfo->bitOffset;
        cfg.BIT.softResetReq &= ~(1 << matchInfo->bitOffset);
    } else {
        cfg.BIT.clkEnMask &= ~(1 << matchInfo->bitOffset); /* Disable of target ip. */
        cfg.BIT.softResetReq |= (1 << matchInfo->bitOffset);
    }
    p->value = cfg.value;
}

/**
 * @brief Get Enable status of IP in APB_HS_SUBSYS
 * @param matchInfo IP without Clock select match info
 * @retval Clock Enable status
 */
static unsigned int CRG_IpWoClkSelEnableGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);

    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    /* Get enable status of target ip. */
    CRG_IpWoClkSelectCfg *p = (CRG_IpWoClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    CRG_IpWoClkSelectCfg cfg;

    cfg.value = p->value;
    return (cfg.BIT.clkEnMask & (1 << matchInfo->bitOffset)) == 0 ? false : true;
}

/**
 * @brief Reset/undo reset of IP in APB_HS_SUBSYS
 * @param matchInfo IP without Clock select match info
 * @param reset BASE_CFG_SET or BASE_CFG_UNSET
 * @retval None
 */
static void CRG_IpWoClkSelResetSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int reset)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);

    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_IpWoClkSelectCfg *p = (CRG_IpWoClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    CRG_IpWoClkSelectCfg cfg;
    cfg.value = p->value;
    if (reset & BASE_CFG_SET) {
        cfg.BIT.softResetReq |= 1 << matchInfo->bitOffset; /* reset of target ip. */
    } else {
        cfg.BIT.softResetReq &= ~(1 << matchInfo->bitOffset);  /* Undo reset of target ip. */
    }
    p->value = cfg.value;
}

/**
 * @brief Get Reset status of IP in APB_HS_SUBSYS
 * @param matchInfo IP without Clock select match info
 * @retval Clock select reset status
 */
static unsigned int CRG_IpWoClkSelResetGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);

    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    /* Get the reset status of target ip. */
    CRG_IpWoClkSelectCfg *p = (CRG_IpWoClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    CRG_IpWoClkSelectCfg cfg;
    cfg.value = p->value;
    return (cfg.BIT.softResetReq & (1 << matchInfo->bitOffset)) ? BASE_CFG_SET : BASE_CFG_UNSET;
}

/**
 * @brief Enable/Disable ADC Clock
 * @param matchInfo ADC match info
 * @param enable IP_CLK_ENABLE
 * @retval None
 */
static void CRG_AdcEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);

    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    CRG_AdcIpCfg cfg;
    cfg.value[1] = p->value[1];
    if (enable) {     /* Enables and Deassert reset the ADC clock. */
        cfg.BIT.clk_adc_cken = BASE_CFG_SET;
        cfg.BIT.adc_srst_req = BASE_CFG_UNSET;
    } else {     /* Disable and reset the ADC clock. */
        cfg.BIT.clk_adc_cken = BASE_CFG_UNSET;
        cfg.BIT.adc_srst_req = BASE_CFG_UNSET;
    }
    p->value[1] = cfg.value[1];
}

/**
 * @brief Get Enable status of ADC
 * @param matchInfo ADC match info
 * @retval Cken of ADC
 */
static unsigned int CRG_AdcEnableGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);
    unsigned int enable;
    /* Get the enable status of the ADC clock gating. */
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    enable = ((p->BIT.clk_adc_cken != 0)) ? IP_CLK_ENABLE : IP_CLK_DISABLE;
    return enable;
}

/**
 * @brief Set ADC Clock Select
 * @param matchInfo ADC match info
 * @param clkSelect @see CRG_AdcClkSelect
 * @retval None
 */
static void CRG_AdcClkSelectSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int clkSelect)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);
    CRG_ASSERT_PARAM(IsCRGInstance(g_crgBaseAddr));
    CRG_PARAM_CHECK_NO_RET(IsCrgAdcClkModeSelect(clkSelect));

    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    if (clkSelect == CRG_ADC_CLK_SYN_CORE) {
        p->BIT.cfg_adc_ckmode_sel = BASE_CFG_SET; /* use sync clock */
    } else {
        DCL_SYSCTRL_CrgWriteProtectionDisable();
        g_crgBaseAddr->PERI_CRG64.BIT.clk_pst2_sw_sel = clkSelect; /* write clock selection */
        DCL_SYSCTRL_CrgWriteProtectionEnable();
        p->BIT.cfg_adc_ckmode_sel = BASE_CFG_UNSET;
    }
}

/**
 * @brief Get ADC Clock Select
 * @param matchInfo ADC match info
 * @retval Adc Clock select @see CRG_AdcClkSelect
 */
static unsigned int CRG_AdcClkSelectGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);

    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    if (p->BIT.cfg_adc_ckmode_sel == BASE_CFG_SET) {
        return CRG_ADC_CLK_SYN_CORE;                        /* Synchronous clock signal */
    }
    return g_crgBaseAddr->PERI_CRG64.BIT.clk_pst2_sw_sel;  /* asynchronous clock signal */
}

/**
 * @brief Set ADC Div
 * @param matchInfo ADC match info
 * @param div Adc clock division
 * @retval None
 */
static void CRG_AdcDivSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int div)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);
    CRG_PARAM_CHECK_NO_RET(IsCrgAdcClkDiv(div));

    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    unsigned int clkMode = p->BIT.cfg_adc_ckmode_sel;
    if (clkMode == CRG_ADC_CLK_SYNCHRONOUS) {
        p->BIT.clk_adc_div1 = div; /* write div to I1 */
    } else {
        p->BIT.clk_adc_div0 = div; /* write div to I0 */
    }
}

/**
 * @brief  Get ADC clock division
 * @param matchInfo  ADC match info
 * @retval Adc clock division
 */
static unsigned int CRG_AdcDivGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);

    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);

    unsigned int clkMode = p->BIT.cfg_adc_ckmode_sel;

    if (clkMode == CRG_ADC_CLK_SYNCHRONOUS) {
        return p->BIT.clk_adc_div1; /* return div value I1 */
    }
    return p->BIT.clk_adc_div0; /* return div valye I0 */
}

/**
 * @brief Enable Clock of EFC
 * @param matchInfo EFC match Info
 * @param enable IP_CLK_ENABLE or IP_CRG_DISABLE
 */
static void CRG_EfcEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);
    /* Enables or disables EFC clock gating. */
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_EfcIpCfg *p = (CRG_EfcIpCfg *)(void *)(base + matchInfo->regOffset);
    p->BIT.eflash_cken = (enable & IP_CLK_ENABLE) ? BASE_CFG_SET : BASE_CFG_UNSET;
}

/**
 * @brief Disable Clock of EFC
 * @param matchInfo EFC match Info
 * @return unsigned int IP_CLK_ENABLE or IP_CRG_DISABLE
 */
static unsigned int CRG_EfcEnableGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);
    /* Get the value of the EFC register in the CRG. */
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_EfcIpCfg *p = (CRG_EfcIpCfg *)(void *)(base + matchInfo->regOffset);
    return p->BIT.eflash_cken;
}


/**
 * @brief Enable Clock of ANA
 * @param matchInfo ANA match Info
 * @param enable IP_CLK_ENABLE or IP_CRG_DISABLE
 */
static void CRG_AnaEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);
    CRG_PARAM_CHECK_NO_RET(enable == IP_CLK_ENABLE || enable == IP_CLK_DISABLE);

    /* Get the value of the ANA IP register in the CRG. */
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AnaIpCfg *p = (CRG_AnaIpCfg *)(void *)(base + matchInfo->regOffset + matchInfo->bitOffset);

    if ((enable == IP_CLK_ENABLE) && (p->BIT.ip_srst_req == BASE_CFG_SET)) {
        p->BIT.ip_srst_req = BASE_CFG_UNSET;
        g_anaEnableFlag++; /* count enable analog IP number */
    } else if ((enable == IP_CLK_DISABLE) && (p->BIT.ip_srst_req == BASE_CFG_UNSET)) {
        p->BIT.ip_srst_req = BASE_CFG_SET;
        if (g_anaEnableFlag > 0) {
            g_anaEnableFlag--; /* Decreasing the count to enable the analog IP number. */
        }
    }

    if ((g_anaEnableFlag == 0) && (enable == IP_CLK_DISABLE)) { /* all analog clock disable */
        CRG->PERI_CRG660.BIT.clk_ana_cken = BASE_CFG_UNSET;
        CRG->PERI_CRG660.BIT.ana_srst_req = BASE_CFG_SET;
    } else if ((g_anaEnableFlag > 0) && (enable == IP_CLK_ENABLE)) {  /* all analog clock enable */
        CRG->PERI_CRG660.BIT.ana_srst_req = BASE_CFG_UNSET;
        CRG->PERI_CRG660.BIT.clk_ana_cken = BASE_CFG_SET;
    }
}

/**
 * @brief Get Clock of ANA
 * @param matchInfo ANA match Info
 * @return unsigned int IP_CLK_ENABLE or IP_CRG_DISABLE
 */
static unsigned int CRG_AnaEnableGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    CRG_ASSERT_PARAM(matchInfo != NULL);
    CRG_ASSERT_PARAM(g_crgBaseAddr != NULL);

    /* Get the value of the ANA IP register in the CRG. */
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AnaIpCfg *p = (CRG_AnaIpCfg *)(void *)(base + matchInfo->regOffset + matchInfo->bitOffset);
    /* The clock is enabled based on the IP reset status. */
    return (p->BIT.ip_srst_req) ? BASE_CFG_UNSET : BASE_CFG_SET;
}
