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

/* Private Function -----------------------------------------------------------*/
static unsigned int CRG_GetPllRefIni(CRG_PllRefClkSelect pllRefClkSelect);
static unsigned int CRG_GetPreDivValue(CRG_PllPreDiv pllPredDiv);
static unsigned int CRG_GetPllFbDivValue(unsigned int pllFbDiv);
static unsigned int CRG_GetPllPostDivValue(unsigned int pllPostDiv);
static inline unsigned int CRG_GetVcoFreq(void);
static BASE_StatusType CRG_IsValidPllConfig(const CRG_Handle *handle);
static void CRG_GetPllOptConfig(unsigned int targetFreq, unsigned int pllRefFreq, CRG_PllDivCfg *div);

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
static void CRG_AdcResetSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int reset);
static void CRG_AdcDivSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int div);
static void CRG_AdcClkSelectSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int clkSelect);
static unsigned int CRG_AdcEnableGet(const CHIP_CrgIpMatchInfo *matchInfo);
static unsigned int CRG_AdcResetGet(const CHIP_CrgIpMatchInfo *matchInfo);
static unsigned int CRG_AdcClkSelectGet(const CHIP_CrgIpMatchInfo *matchInfo);
static unsigned int CRG_AdcDivGet(const CHIP_CrgIpMatchInfo *matchInfo);

static void CRG_DacEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable);
static unsigned int CRG_DacEnableGet(const CHIP_CrgIpMatchInfo *matchInfo);
static void CRG_DacDivSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int div);
static unsigned int CRG_DacDivGet(const CHIP_CrgIpMatchInfo *matchInfo);
static void CRG_DacResetSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int reset);
static unsigned int CRG_DacResetGet(const CHIP_CrgIpMatchInfo *matchInfo);

static void CRG_EfcEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable);
static unsigned int CRG_EfcEnableGet(const CHIP_CrgIpMatchInfo *matchInfo);

#ifndef FPGA
static unsigned int CRG_GetLsIpFreq(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int baseClkRate);
static unsigned int CRG_GetAdcIpFreq(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int baseClkRate);
static unsigned int CRG_GetDacIpFreq(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int baseClkRate);
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

static const CRG_IpProc g_ipClkProc[CRG_IP_MAX_TYPE] = {
    {
        CRG_IP_WITH_LS,
        CRG_IpWithClkSelResetSet,
        CRG_IpWithClkSelEnableSet,
        CRG_IpWithClkSelClkSelSet,
        NULL,
        CRG_IpWithClkSelResetGet,
        CRG_IpWithClkSelEnableGet,
        CRG_IpWithClkSelClkSelGet,
        NULL,
    },
    {
        CRG_IP_WITH_HS,
        CRG_IpWoClkSelResetSet,
        CRG_IpWoClkSelEnableSet,
        NULL,
        NULL,
        CRG_IpWoClkSelResetGet,
        CRG_IpWoClkSelEnableGet,
        NULL,
        NULL,
    },
    {
        CRG_IP_CAN,
        CRG_IpWoClkSelResetSet,
        CRG_IpWoClkSelEnableSet,
        NULL,
        NULL,
        CRG_IpWoClkSelResetGet,
        CRG_IpWoClkSelEnableGet,
        NULL,
        NULL,
    },
    {
        CRG_IP_ADC,
        CRG_AdcResetSet,
        CRG_AdcEnableSet,
        CRG_AdcClkSelectSet,
        CRG_AdcDivSet,
        CRG_AdcResetGet,
        CRG_AdcEnableGet,
        CRG_AdcClkSelectGet,
        CRG_AdcDivGet,
    },
    {
        CRG_IP_DAC,
        CRG_DacResetSet,
        CRG_DacEnableSet,
        NULL,
        CRG_DacDivSet,
        CRG_DacResetGet,
        CRG_DacEnableGet,
        NULL,
        CRG_DacDivGet,
    },
    {
        CRG_IP_EFC,
        NULL,
        CRG_EfcEnableSet,
        NULL,
        NULL,
        NULL,
        CRG_EfcEnableGet,
        NULL,
        NULL,
    },
    {
        CRG_IP_IWDG,
        CRG_IpWoClkSelResetSet,
        CRG_IpWoClkSelEnableSet,
        NULL,
        NULL,
        CRG_IpWoClkSelResetGet,
        CRG_IpWoClkSelEnableGet,
        NULL,
        NULL,
    },
};
static CRG_RegStruct *g_crgBaseAddr;

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
    CRG_PARAM_CHECK_WITH_RET(IsCrgXtalFreq(XTRAIL_FREQ), BASE_STATUS_ERROR); /* Check the validity of XTRAIL_FREQ. */
    CRG_PARAM_CHECK_WITH_RET(IsCrgPllRefClkSelect(handle->pllRefClkSelect), BASE_STATUS_ERROR);
    CRG_PARAM_CHECK_WITH_RET(IsCrgPllPreDiv(handle->pllPreDiv), BASE_STATUS_ERROR);
    CRG_PARAM_CHECK_WITH_RET(IsCrgPllFbDiv(handle->pllFbDiv), BASE_STATUS_ERROR);
    CRG_PARAM_CHECK_WITH_RET(IsCrgPllPostDiv(handle->pllPostDiv), BASE_STATUS_ERROR);
    CRG_PARAM_CHECK_WITH_RET(IsCrgCoreCkSel(handle->coreClkSelect), BASE_STATUS_ERROR);

    CRG_RegStruct *reg = handle->baseAddress;
    g_crgBaseAddr = (void *)reg;
    /* Check the validity of PLL parameters. */
    if (CRG_IsValidPllConfig(handle) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    DCL_SYSCTRL_CrgWriteProtectionDisable(); /* Disable register write protection. */

    reg->PERI_CRG0.pll_ref_cksel   = handle->pllRefClkSelect;
    reg->PERI_CRG1.pll_prediv      = handle->pllPreDiv;
    reg->PERI_CRG2.pll_fbdiv       = handle->pllFbDiv;
    reg->PERI_CRG3.pll_postdiv     = handle->pllPostDiv;
    reg->PERI_CRG4.pll_pd          = BASE_CFG_UNSET;
    reg->PERI_CRG5.BIT.pll_dig_eb_lockdet = BASE_CFG_UNSET; /* PLL lock detection enable, 0 : enable, 1: disable */

    while (reg->PERI_CRG8.pll_lock != BASE_CFG_SET) {
        ;  /* Wait for PLL to lock */
    }
    reg->PERI_CRG7.BIT.ck_switchen = BASE_CFG_SET;

    DCL_SYSCTRL_CrgWriteProtectionEnable(); /* Enable register write protection. */
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
    CRG_PARAM_CHECK_WITH_RET((targetFreq <= CRG_CLK_VCO_MAX_FREQ), BASE_STATUS_ERROR);

    CRG_Handle crgHandle;
    CRG_PllDivCfg divCfg;
    unsigned int pllRefFreq;

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
    CRG_RegStruct *reg = handle->baseAddress;
    DCL_SYSCTRL_CrgWriteProtectionDisable();

    reg->PERI_CRG0.pll_ref_cksel   = 0x0;  /* 0x0: default value */
    reg->PERI_CRG1.pll_prediv      = 0x3;  /* 0x3: default value */
    reg->PERI_CRG2.pll_fbdiv       = 0x10; /* 0x10: default value */
    reg->PERI_CRG3.pll_postdiv     = 0x0;  /* 0x0: default value */
    reg->PERI_CRG4.pll_pd          = 0x1;  /* 0x1: default value */
    reg->PERI_CRG5.BIT.pll_dig_eb_lockdet = 0x1;  /* 0x1: default value */
    reg->PERI_CRG7.BIT.ck_switchen = 0x1;  /* 0x1: default value */

    DCL_SYSCTRL_CrgWriteProtectionEnable();
    return BASE_STATUS_OK;
}

/**
  * @brief Get Clock Config
  * @param handle CRG Handle
  * @retval BASE_STATUS_OK    Success
  */
BASE_StatusType HAL_CRG_GetConfig(CRG_Handle *handle)
{
    CRG_ASSERT_PARAM(handle != 0);
    CRG_ASSERT_PARAM(IsCRGInstance(handle->baseAddress));

    CRG_RegStruct *reg = handle->baseAddress;
    handle->pllRefClkSelect = reg->PERI_CRG0.pll_ref_cksel;
    handle->pllPreDiv       = reg->PERI_CRG1.pll_prediv;
    handle->pllFbDiv        = reg->PERI_CRG2.pll_fbdiv;
    handle->pllPostDiv      = reg->PERI_CRG3.pll_postdiv;
    handle->pllPd           = reg->PERI_CRG4.pll_pd;
    handle->handleEx.ckSwitchEn   = reg->PERI_CRG7.BIT.ck_switchen;
    handle->coreClkSelect   = reg->PERI_CRG7.BIT.core_cksel;

    return BASE_STATUS_OK;
}

/**
  * @brief Set CRG Core Clock Select
  * @param handle CRG Handle
  * @retval BASE_STATUS_OK  Success
  * @retval BASE_STATUS_ERROR Parameter check fail
  */
BASE_StatusType HAL_CRG_SetCoreClockSelect(CRG_Handle *handle)
{
    CRG_ASSERT_PARAM(handle != 0);
    CRG_ASSERT_PARAM(IsCRGInstance(handle->baseAddress));
    CRG_PARAM_CHECK_WITH_RET(IsCrgCoreCkSel(handle->coreClkSelect), BASE_STATUS_ERROR);

    CRG_RegStruct *reg = handle->baseAddress;

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
    CRG_RegStruct *crg = g_crgBaseAddr;

    CRG_ASSERT_PARAM(IsCRGInstance(crg));
    freq = CRG_GetPllRefIni(crg->PERI_CRG0.pll_ref_cksel);
    freq /= CRG_GetPreDivValue(crg->PERI_CRG1.pll_prediv);
    freq *= CRG_GetPllFbDivValue(crg->PERI_CRG2.pll_fbdiv);
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
    CRG_RegStruct *crg = g_crgBaseAddr;

    CRG_ASSERT_PARAM(IsCRGInstance(crg));
    freq = CRG_GetVcoFreq();
    freq /= CRG_GetPllPostDivValue((CRG_PllPostDiv)crg->PERI_CRG3.pll_postdiv);
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
    coreClkSelect = crg->PERI_CRG7.BIT.core_cksel;
    switch (coreClkSelect) {
        case CRG_CORE_CLK_SELECT_HOSC:
            freq = HOSC_FREQ;
            break;

        case CRG_CORE_CLK_SELECT_TCXO:
            freq = XTRAIL_FREQ;
            break;

        case CRG_CORE_CLK_SELECT_PLL:
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
    return CHIP_GetIpFreqHz(baseAddress);
#else
    unsigned int hclk = HAL_CRG_GetCoreClkFreq();
    unsigned int freq = LOSC_FREQ;
    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if (p == NULL) {
        return freq;
    }
    switch (p->type) {
        case CRG_IP_WITH_LS:
            freq = CRG_GetLsIpFreq(p, hclk / 0x2);  /* pclk is 1/2 of hclk */
            break;

        case CRG_IP_WITH_HS:
        case CRG_IP_EFC:
            freq = hclk;
            break;

        case CRG_IP_CAN:
            freq = CRG_GetPllRefIni(g_crgBaseAddr->PERI_CRG0.pll_ref_cksel);
            break;

        case CRG_IP_DAC:
            freq = CRG_GetPllRefIni(g_crgBaseAddr->PERI_CRG0.pll_ref_cksel);
            freq = CRG_GetDacIpFreq(p, freq);
            break;

        case CRG_IP_ADC:
            freq = CRG_GetAdcIpFreq(p, CRG_GetVcoFreq());
            break;

        case CRG_IP_IWDG:
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

    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].enableGet == NULL) {
        return BASE_STATUS_ERROR;
    }
    *enable = g_ipClkProc[p->type].enableGet(p);
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
    g_ipClkProc[p->type].clkSelSet(p, select);
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

    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].clkSelGet == NULL) {
        return BASE_STATUS_ERROR;
    }
    *clkSel = g_ipClkProc[p->type].clkSelGet(p);
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

    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].resetSet == NULL) {
        return BASE_STATUS_ERROR;
    }
    g_ipClkProc[p->type].resetSet(p, reset);
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

    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].resetGet == NULL) {
        return BASE_STATUS_ERROR;
    }
    *reset = g_ipClkProc[p->type].resetGet(p);
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
    g_ipClkProc[p->type].clkDivSet(p, div);
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

    CHIP_CrgIpMatchInfo *p = GetCrgIpMatchInfo(baseAddress);
    if ((p == NULL) || (p->type >= CRG_IP_MAX_TYPE)) {
        return BASE_STATUS_ERROR;
    }
    if (g_ipClkProc[p->type].clkDivGet == NULL) {
        return BASE_STATUS_ERROR;
    }
    *div = g_ipClkProc[p->type].clkDivGet(p);
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
    g_crgBaseAddr->PERI_CRG48.BIT.pvd_rst_enable = enable;
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
    unsigned int preDiv[] = {CRG_PLL_PREDIV_1, CRG_PLL_PREDIV_2, CRG_PLL_PREDIV_4};
    unsigned int freq;
    unsigned int delta;
    unsigned int minDelta = 0xFFFFFFFF;

    divCfg->PreDiv  = CRG_PLL_NO_PREDV;
    divCfg->fbDiv   = CRG_PLL_FBDIV_MIN;
    divCfg->postDiv = CRG_PLL_POSTDIV_1;

    for (unsigned int i = 0; i < sizeof(preDiv) / sizeof(preDiv[0]); ++i) {
        unsigned int preDivOut = CRG_GetPreDivValue(preDiv[i]);
        if (!IsCrgValidPreDiv(pllRefFreq, preDivOut)) {
            continue;
        }
        unsigned int clkPfdFreq = pllRefFreq / preDivOut;
        for (unsigned int j = CRG_PLL_FBDIV_MIN; j <= CRG_PLL_FBDIV_MAX; ++j) {
            if (!IsCrgValidFdDiv(clkPfdFreq, j)) {
                continue;
            }
            unsigned int clkVcoFreq = clkPfdFreq * j;
            for (unsigned int k = CRG_PLL_POSTDIV_1; k <= CRG_PLL_POSTDIV_32; k++) {
                unsigned int postDiv = 1 << k;
                if (!IsCrgValidPostDiv(clkVcoFreq, postDiv)) {
                    continue;
                }
                freq = clkVcoFreq / postDiv;
                delta = (targetFreq >= freq) ? targetFreq - freq : freq - targetFreq;
                if (delta < minDelta) {
                    minDelta = delta;
                    divCfg->PreDiv = preDiv[i];
                    divCfg->fbDiv = j;
                    divCfg->postDiv = k;
                }
            }
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
    unsigned int clkSel;
    const CRG_IpProc *proc = &g_ipClkProc[matchInfo->type];

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
  * @retval Ip Frequence
  */
static unsigned int CRG_GetAdcIpFreq(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int baseClkRate)
{
    unsigned int clkSel;
    unsigned int clkDiv;
    const CRG_IpProc *proc = &g_ipClkProc[matchInfo->type];
    if (proc->clkSelGet == NULL) {
        return 0;
    }
    clkSel = proc->clkSelGet(matchInfo);
    if (clkSel == CRG_ADC_CLK_SELECT_HOSC) {
        return HOSC_FREQ;
    }
    if (clkSel == CRG_ADC_CLK_SELECT_TCXO) {
        return XTRAIL_FREQ;
    }
    if (proc->clkDivGet == NULL) {
        return 0;
    }
    clkDiv = proc->clkDivGet(matchInfo);
    return (baseClkRate * ADC_DIV_FACTOR) / (clkDiv + ADC_DIV_FACTOR);
}

/**
  * @brief Get DAC Clock Frequence
  * @param matchInfo match info
  * @param baseClkRate clock rate
  * @retval Ip Frequence
  */
static unsigned int CRG_GetDacIpFreq(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int baseClkRate)
{
    unsigned int clkDiv;
    const CRG_IpProc *proc = &g_ipClkProc[matchInfo->type];

    if (proc->clkDivGet == NULL) {
        return 0;
    }
    clkDiv = proc->clkDivGet(matchInfo); /* get clock division value */
    return baseClkRate / (clkDiv + 1); /* return frequency value */
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
    if (!IsCrgValidPreDiv(freq, preDiv)) {
        return BASE_STATUS_ERROR;
    }
    freq /= preDiv;
    if (!IsCrgValidFdDiv(freq, handle->pllFbDiv)) {
        return BASE_STATUS_ERROR;
    }
    freq *= handle->pllFbDiv;
    return IsCrgValidPostDiv(freq, handle->pllPostDiv) ? BASE_STATUS_OK : BASE_STATUS_ERROR;
}

/**
 * @brief Get clock frequence
 * @param  crg CRG_RegStruct
 * @retval The frequence of clock
 */
static inline unsigned int CRG_GetPllRefIni(CRG_PllRefClkSelect pllRefClkSelect)
{
    return (pllRefClkSelect == (unsigned int)CRG_PLL_REF_CLK_SELECT_HOSC) ? HOSC_FREQ : XTRAIL_FREQ;
}

/**
 * @brief Get previous division Value before PLL
 * @param  crg CRG_RegStruct
 * @retval Previous Div value
 */
static inline unsigned int CRG_GetPreDivValue(CRG_PllPreDiv pllPredDiv)
{
    if (pllPredDiv <= CRG_PLL_PREDIV_1) {
        return PLL_PREDIV_OUT_1;
    } else if (pllPredDiv == CRG_PLL_PREDIV_2) {
        return PLL_PREDIV_OUT_2;
    } else {
        return PLL_PREDIV_OUT_4;
    }
}

/**
 * @brief Get PLL loop divider ratio
 * @param  crg CRG_RegStruct
 * @retval PLL loop divider ratio
 */
static inline unsigned int CRG_GetPllFbDivValue(unsigned int pllFbDiv)
{
    unsigned int div = pllFbDiv;

    if (div < CRG_PLL_FBDIV_MIN) {
        div = CRG_PLL_FBDIV_MIN;
    }
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
    if (div > CRG_PLL_POSTDIV_32) {
        div = CRG_PLL_POSTDIV_32;
    }
    return (1 << div);
}

/**
 * @brief Enable Set for IP in APB_LS_SUBSYS
 * @param matchInfo IP with Clock select match info
 * @param enable  BASE_CFG_SET or BASE_CFG_UNSET
 * @retval None
 */
static void CRG_IpWithClkSelEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable)
{
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_IpWithClkSelectCfg *p = (CRG_IpWithClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    if ((enable & IP_CLK_ENABLE) == IP_CLK_ENABLE) {
        p->BIT.cken = BASE_CFG_SET;
        p->BIT.srst_req = BASE_CFG_UNSET;   /* Enable with soft reset disable */
    } else {
        p->BIT.cken = BASE_CFG_UNSET;
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
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_IpWoClkSelectCfg *p = (CRG_IpWoClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    CRG_IpWoClkSelectCfg cfg;
    cfg.value = p->value;
    if (enable & IP_CLK_ENABLE) {
        cfg.BIT.clkEnMask |= 1 << matchInfo->bitOffset;
        cfg.BIT.softResetReq &= ~(1 << matchInfo->bitOffset);
    } else {
        cfg.BIT.clkEnMask &= ~(1 << matchInfo->bitOffset);
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
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
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
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_IpWoClkSelectCfg *p = (CRG_IpWoClkSelectCfg *)(void *)(base + matchInfo->regOffset);
    CRG_IpWoClkSelectCfg cfg;
    cfg.value = p->value;
    if (reset & BASE_CFG_SET) {
        cfg.BIT.softResetReq |= 1 << matchInfo->bitOffset;
    } else {
        cfg.BIT.softResetReq &= ~(1 << matchInfo->bitOffset);
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
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
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
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    CRG_AdcIpCfg cfg;
    cfg.value = p->value;
    if (enable) {
        cfg.BIT.cken = BASE_CFG_SET;
        cfg.BIT.sys_cken = BASE_CFG_SET;
        cfg.BIT.srst_req = BASE_CFG_UNSET;
        cfg.BIT.sys_srst_req = BASE_CFG_UNSET;
        cfg.BIT.ana_srst_req = BASE_CFG_UNSET;
    } else {
        cfg.BIT.cken = BASE_CFG_UNSET;
        cfg.BIT.sys_cken = BASE_CFG_UNSET;
    }
    p->value = cfg.value;
}

/**
 * @brief Get Enable status of ADC
 * @param matchInfo ADC match info
 * @retval Cken of ADC
 */
static unsigned int CRG_AdcEnableGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    unsigned int enable;
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    enable = ((p->BIT.cken != 0) && (p->BIT.sys_cken != 0)) ? IP_CLK_ENABLE : IP_CLK_DISABLE;
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
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    p->BIT.cksel = (unsigned int)clkSelect;
}

/**
 * @brief Get ADC Clock Select
 * @param matchInfo ADC match info
 * @retval Adc Clock select @see CRG_AdcClkSelect
 */
static unsigned int CRG_AdcClkSelectGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    return p->BIT.cksel;
}

/**
 * @brief Set ADC Div
 * @param matchInfo ADC match info
 * @param div Adc clock division
 * @retval None
 */
static void CRG_AdcDivSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int div)
{
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    p->BIT.div = div; /* write div */
}

/**
 * @brief  Get ADC clock division
 * @param matchInfo  ADC match info
 * @retval Adc clock division
 */
static unsigned int CRG_AdcDivGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    return p->BIT.div; /* return div value */
}

/**
 * @brief Reset/undo reset ADC
 * @param matchInfo ADC match Info
 * @param reset BASE_CFG_SET or BASE_CFG_UNSET
 * @retval None
 */
static void CRG_AdcResetSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int reset)
{
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    CRG_AdcIpCfg cfg;

    cfg.value = p->value;
    if (reset) {
        cfg.BIT.srst_req     = BASE_CFG_SET;
        cfg.BIT.sys_srst_req = BASE_CFG_SET;
        cfg.BIT.ana_srst_req = BASE_CFG_SET;
    } else {
        cfg.BIT.srst_req     = BASE_CFG_UNSET;
        cfg.BIT.sys_srst_req = BASE_CFG_UNSET;
        cfg.BIT.ana_srst_req = BASE_CFG_UNSET;
    }
    p->value = cfg.value;
}

/**
 * @brief Get Reset Status of ADC
 * @param matchInfo ADC match Info
 * @retval reset BASE_CFG_SET or BASE_CFG_UNSET
 */
static unsigned int CRG_AdcResetGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    unsigned int reset;
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_AdcIpCfg *p = (CRG_AdcIpCfg *)(void *)(base + matchInfo->regOffset);
    reset  = p->BIT.srst_req;
    reset |= p->BIT.sys_srst_req;
    reset |= p->BIT.ana_srst_req;
    return reset;
}

/**
 * @brief Enable/Disable DAC Clock
 * @param matchInfo DAC match info
 * @param enable IP_CLK_ENABLE
 * @retval None
 */
static void CRG_DacEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable)
{
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_DacIpCfg *p = (CRG_DacIpCfg *)(void *)(base + matchInfo->regOffset);
    CRG_DacIpCfg cfg;
    cfg.value = p->value;
    if (enable) {
        cfg.BIT.softResetReq &= ~(BASE_CFG_SET << matchInfo->bitOffset);
        cfg.BIT.clkEnMask |= BASE_CFG_SET << matchInfo->bitOffset;
    } else {
        cfg.BIT.clkEnMask &= ~(BASE_CFG_SET << matchInfo->bitOffset);
    }
    p->value = cfg.value;
}

/**
 * @brief Get Enable status of DAC
 * @param matchInfo DAC Match info
 * @retval Cken and Sys_cken of ADC
 */
static unsigned int CRG_DacEnableGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    unsigned int enable;
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_DacIpCfg *p = (CRG_DacIpCfg *)(void *)(base + matchInfo->regOffset);
    if ((p->BIT.clkEnMask & (1 << matchInfo->bitOffset)) != 0) {
        enable = BASE_CFG_SET;
    } else {
        enable = BASE_CFG_UNSET;
    }
    return enable;
}

/**
 * @brief Set DAC Div
 * @param matchInfo DAC Match info
 * @param div dac div parameter
 */
static void CRG_DacDivSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int div)
{
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_DacIpCfg *p = (CRG_DacIpCfg *)(void *)(base + matchInfo->regOffset);
    CRG_DacIpCfg cfg;
    cfg.value = p->value;
    cfg.BIT.div &= ~(DAC_DIV_MASK << (matchInfo->bitOffset * DAC_DIV_BITLEN));
    cfg.BIT.div |= (div & DAC_DIV_MASK) << (matchInfo->bitOffset * DAC_DIV_BITLEN);
    p->value = cfg.value;
}

/**
 * @brief  Get DAC Div
 * @param matchInfo DAC Match info
 * @return div dac div parameter
 */
static unsigned int CRG_DacDivGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    unsigned int div;
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_DacIpCfg *p = (CRG_DacIpCfg *)(void *)(base + matchInfo->regOffset);
    div = p->BIT.div;
    div >>= (matchInfo->bitOffset * DAC_DIV_BITLEN);
    return (div & DAC_DIV_MASK);
}

/**
 * @brief Reset/undo reset DAC
 * @param matchInfo DAC match Info
 * @param reset DAC_SOFTRESET
 */
static void CRG_DacResetSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int reset)
{
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_DacIpCfg *p = (CRG_DacIpCfg *)(void *)(base + matchInfo->regOffset);
    CRG_DacIpCfg cfg;

    cfg.value = p->value;
    if (reset != 0) {
        cfg.BIT.softResetReq |= BASE_CFG_SET << matchInfo->bitOffset;
    } else {
        cfg.BIT.softResetReq &= ~(BASE_CFG_SET << matchInfo->bitOffset);
    }
    p->value = cfg.value;
}

/**
 * @brief Get Reset Status of DAC
 * @param matchInfo DAC match Info
 * @return unsigned int  DAC_SOFTRESET
 */
static unsigned int CRG_DacResetGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    CRG_DacIpCfg *p = (CRG_DacIpCfg *)(void *)(base + matchInfo->regOffset);
    return ((p->BIT.softResetReq >> matchInfo->bitOffset) & BASE_CFG_SET);
}

/**
 * @brief Enable Clock of EFC
 * @param matchInfo EFC match Info
 * @param enable IP_CLK_ENABLE or IP_CRG_DISABLE
 */
static void CRG_EfcEnableSet(const CHIP_CrgIpMatchInfo *matchInfo, unsigned int enable)
{
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    PERI_CRG32_Reg *p = (PERI_CRG32_Reg *)(void *)(base + matchInfo->regOffset);
    p->eflash_cken = (enable & IP_CLK_ENABLE) ? BASE_CFG_SET : BASE_CFG_UNSET;
}

/**
 * @brief Disable Clock of EFC
 * @param matchInfo EFC match Info
 * @return unsigned int IP_CLK_ENABLE or IP_CRG_DISABLE
 */
static unsigned int CRG_EfcEnableGet(const CHIP_CrgIpMatchInfo *matchInfo)
{
    uintptr_t base = (uintptr_t)(void *)g_crgBaseAddr;
    PERI_CRG32_Reg *p = (PERI_CRG32_Reg *)(void *)(base + matchInfo->regOffset);
    return p->eflash_cken;
}
