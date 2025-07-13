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
  * @file      system_init.c
  * @author    MCU Driver Team
  * @brief     This file contains driver init functions.
  */

#include "main.h"
#include "ioconfig.h"
#include "iocmg_ip.h"

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;    /* Clock-related parameter configuration. */
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 48; /* PLL Multiplier 48 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_2;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;
    crg.handleEx.pllPostDiv2   = CRG_PLL_POSTDIV2_3;
    crg.handleEx.clk1MSelect   = CRG_1M_CLK_SELECT_HOSC;
    crg.handleEx.clk1MDiv = (25 - 1); /* 25,The 1 MHz freq is equal to the input clock frequency / (clk_1m_div + 1). */
    /* Initialize the clock configuration. */
    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static void GPIO_Init(void)
{
    HAL_CRG_IpEnableSet(GPIO1_BASE, IP_CLK_ENABLE);  /* Enable the GPIO1 clock. */
    g_gpio1.baseAddress = GPIO1;

    g_gpio1.pins = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_7;
    HAL_GPIO_Init(&g_gpio1);
    HAL_GPIO_SetDirection(&g_gpio1, g_gpio1.pins, GPIO_OUTPUT_MODE); /* Set output mode */
    HAL_GPIO_SetValue(&g_gpio1, g_gpio1.pins, GPIO_LOW_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio1, g_gpio1.pins, GPIO_INT_TYPE_NONE);

    HAL_CRG_IpEnableSet(GPIO2_BASE, IP_CLK_ENABLE);  /* Enable the GPIO2 clock. */
    g_gpio2.baseAddress = GPIO2;

    g_gpio2.pins = GPIO_PIN_0 | GPIO_PIN_6;
    HAL_GPIO_Init(&g_gpio2);
    HAL_GPIO_SetDirection(&g_gpio2, g_gpio2.pins, GPIO_OUTPUT_MODE); /* Set output mode */
    HAL_GPIO_SetValue(&g_gpio2, g_gpio2.pins, GPIO_LOW_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio2, g_gpio2.pins, GPIO_INT_TYPE_NONE);

    HAL_CRG_IpEnableSet(GPIO3_BASE, IP_CLK_ENABLE);  /* Enable the GPIO3 clock. */
    g_gpio3.baseAddress = GPIO3;

    g_gpio3.pins = GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_0 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(&g_gpio3);
    HAL_GPIO_SetDirection(&g_gpio3, g_gpio3.pins, GPIO_OUTPUT_MODE); /* Set output mode */
    HAL_GPIO_SetValue(&g_gpio3, g_gpio3.pins, GPIO_LOW_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio3, g_gpio3.pins, GPIO_INT_TYPE_NONE);

    HAL_CRG_IpEnableSet(GPIO4_BASE, IP_CLK_ENABLE);  /* Enable the GPIO4 clock. */
    g_gpio4.baseAddress = GPIO4;

    g_gpio4.pins = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(&g_gpio4);
    HAL_GPIO_SetDirection(&g_gpio4, g_gpio4.pins, GPIO_OUTPUT_MODE); /* Set output mode */
    HAL_GPIO_SetValue(&g_gpio4, g_gpio4.pins, GPIO_LOW_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio4, g_gpio4.pins, GPIO_INT_TYPE_NONE);

    return;
}

__weak void TIMER1_InterruptProcess(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN TIMER1_InterruptProcess */
    /* USER CODE END TIMER1_InterruptProcess */
}

static void TIMER1_Init(void)
{
    HAL_CRG_IpEnableSet(TIMER1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER1_BASE, CRG_AHB_CLK_NO_PREDV);

    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER1) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 2;

    g_timer1.baseAddress = TIMER1;
    g_timer1.load        = load - 1; /* Set timer value immediately */
    g_timer1.bgLoad      = load - 1; /* Set timer value */
    g_timer1.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer1.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer1.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer1.interruptEn = BASE_CFG_ENABLE;
    g_timer1.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer1.dmaReqEnable = BASE_CFG_DISABLE;
    HAL_TIMER_Init(&g_timer1);
    IRQ_Register(IRQ_TIMER1, HAL_TIMER_IrqHandler, &g_timer1);

    HAL_TIMER_RegisterCallback(&g_timer1, TIMER_PERIOD_FIN, TIMER1_InterruptProcess);
    IRQ_SetPriority(IRQ_TIMER1, 1);
    IRQ_EnableN(IRQ_TIMER1);
}

static void IOConfig(void)
{
    HAL_IOCMG_SetPinAltFuncMode(GPIO1_1_AS_GPIO1_1);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_1_AS_GPIO1_1, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_1_AS_GPIO1_1, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_1_AS_GPIO1_1, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_1_AS_GPIO1_1, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    HAL_IOCMG_SetPinAltFuncMode(GPIO1_4_AS_GPIO1_4);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_4_AS_GPIO1_4, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_4_AS_GPIO1_4, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_4_AS_GPIO1_4, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_4_AS_GPIO1_4, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    HAL_IOCMG_SetPinAltFuncMode(GPIO1_7_AS_GPIO1_7);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_7_AS_GPIO1_7, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_7_AS_GPIO1_7, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_7_AS_GPIO1_7, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_7_AS_GPIO1_7, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_0_AS_GPIO2_0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_0_AS_GPIO2_0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_0_AS_GPIO2_0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_0_AS_GPIO2_0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_0_AS_GPIO2_0, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_6_AS_GPIO2_6);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_6_AS_GPIO2_6, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_6_AS_GPIO2_6, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_6_AS_GPIO2_6, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_6_AS_GPIO2_6, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    HAL_IOCMG_SetPinAltFuncMode(GPIO3_1_AS_GPIO3_1);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_1_AS_GPIO3_1, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_1_AS_GPIO3_1, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_1_AS_GPIO3_1, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_1_AS_GPIO3_1, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    HAL_IOCMG_SetPinAltFuncMode(GPIO3_5_AS_GPIO3_5);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_5_AS_GPIO3_5, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_5_AS_GPIO3_5, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_5_AS_GPIO3_5, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_5_AS_GPIO3_5, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    HAL_IOCMG_SetPinAltFuncMode(GPIO3_0_AS_GPIO3_0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_0_AS_GPIO3_0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_0_AS_GPIO3_0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_0_AS_GPIO3_0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_0_AS_GPIO3_0, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    HAL_IOCMG_SetPinAltFuncMode(GPIO3_6_AS_GPIO3_6);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_6_AS_GPIO3_6, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_6_AS_GPIO3_6, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_6_AS_GPIO3_6, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_6_AS_GPIO3_6, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    HAL_IOCMG_SetPinAltFuncMode(GPIO3_7_AS_GPIO3_7);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_7_AS_GPIO3_7, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_7_AS_GPIO3_7, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_7_AS_GPIO3_7, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_7_AS_GPIO3_7, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    HAL_IOCMG_SetPinAltFuncMode(GPIO4_0_AS_GPIO4_0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_0_AS_GPIO4_0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_0_AS_GPIO4_0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_0_AS_GPIO4_0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_0_AS_GPIO4_0, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    HAL_IOCMG_SetPinAltFuncMode(GPIO4_1_AS_GPIO4_1);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_1_AS_GPIO4_1, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_1_AS_GPIO4_1, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_1_AS_GPIO4_1, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_1_AS_GPIO4_1, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();
    TIMER1_Init();
    GPIO_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}