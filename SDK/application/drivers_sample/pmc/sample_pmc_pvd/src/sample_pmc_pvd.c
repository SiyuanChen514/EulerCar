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
  * @file    sample_pmc_pvd.c
  * @author  MCU Driver Team
  * @brief   APT module sample of HAL API.
  *          This file provides some configuration example of PMC module's PVD function.
  *          + Debug printf information in  interrupt callback function if PVD happen.
  */
#include "debug.h"
#include "pmc.h"
#include "crg.h"
#include "sample_pmc_pvd.h"
#include "main.h"


/**
  * @brief PVD event interrupt callback.
  * @retval None.
  */
void PMC_PvdCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("PVD happen!\r\n");
    BASE_FUNC_DELAY_MS(10); /* delay 10 ms, wait for uart printf finish */
    HAL_CRG_PvdResetEnable(BASE_CFG_ENABLE); /* enable pvd reset function */
}

/**
  * @brief PVD sample.
  * @retval None.
  */
void PmcPvdSample(void)
{
    SystemInit();
    HAL_PMC_RegisterCallback(&g_pmc, PMC_PvdCallback); /* register callback */
    DBG_PRINTF("enter main!\r\n"); /* enter main */
    while (1) {
    }
}