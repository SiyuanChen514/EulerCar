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
  * @file    sample_acmp_interrupt.c
  * @author  MCU Driver Team
  * @brief   can sample module.
  * @details This file provides sample code for users to help use
  *          the interrupt function of the acmp.
  */
#include "sample_acmp_interrupt.h"
#include "main.h"
#include "debug.h"

/**
  * @brief ACMP Positve Callback function.
  * @param handle ACMP handle.
  * @retval None.
  */
void ACMP0PositveCallFunc(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN ACMP0PositveCallFunc */
    DBG_PRINTF("ACMP positive callback function.\r\n");
    /* USER CODE END ACMP0PositveCallFunc */
}

/**
  * @brief ACMP Negative Callback function.
  * @param handle ACMP handle.
  * @retval None.
  */
void ACMP0NegativeCallFunc(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN ACMP0NegativeCallFunc */
    DBG_PRINTF("ACMP negative callback function.\r\n");
    /* USER CODE END ACMP0NegativeCallFunc */
}

/**
  * @brief ACMP Edge callback function.
  * @param handle ACMP handle.
  * @retval None.
  */
void ACMP0EdgedCallFunc(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN ACMP0EdgedCallFunc */
    DBG_PRINTF("ACMP edge callback funtion.\r\n");
    /* USER CODE END ACMP0EdgedCallFunc */
}

/**
  * @brief Example of comparing ACMP results
  * @param None.
  * @retval None.
  */
void ACMP_CompareInt(void)
{
    DBG_PRINTF("Example: ACMP comparison result interrupt. The ACMP output comparison result triggers different types\
               of interrupts, such as rising edge triggle positve call back function, falling edge triggle negative\
               callback function, and flip edge triggle edge callback funtion.\r\n");
    SystemInit();
}