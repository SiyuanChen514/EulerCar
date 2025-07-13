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
  * @file      cfd_check_error_sample.c
  * @author    MCU Driver Team
  * @brief     The CFD module monitors abnormal clocks, inject clock error after loop times in cycles.
  * @details   Check whether the PLL reference clock (TCXO or HOSC) fails by using \
  *            the LOSC = 32Khz clock. If the clock fails, the interrupt service function \
  *            is triggered for processing, and the hardware system event 2 is triggered to \
  *            disable the APT and switch the master clock to the LOSC for protection. \
  *            Users can perform secure operations such as clock recovery or reset in the \
  *            interrupt service function. If the interrupt trigger mode is set to non-clock failure mode, \
  *            the user needs to determine whether the count value is within the threshold after each interrupt \
  *            is triggered. If the count value is not within the threshold, the clock is considered abnormal.
  *            For details about the PLL reference clock recovery mechanism, see the chip technical guide. \
  *            The interrupt type can be configured by users. Only the upper threshold is available. \
  *            For details about the calculation method, see the chip technical guide.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "debug.h"
#include "cfd.h"
#include "cfd_check_error_sample.h"
#include "main.h"
#include "crg.h"

#define LOOP_TIMES    5

unsigned int g_loopTimes = 0;

void CFD_CheckErrorCallback(CFD_Handle *handle);

/**
  * @brief Sample main function.
  * @param None.
  * @return @ref BASE_StatusType
  */
BASE_StatusType CFD_SampleMain(void)
{
    SystemInit();
    HAL_CFD_Start(&g_cfd);

    while (1) {
        /* If ref clk cnt value bigger than upperbound, then junp to callback */
        BASE_FUNC_DELAY_S(1);
        DBG_PRINTF("main loop, ref clk cnt value = %d\r\n", DCL_CFD_GetCntValue(g_cfd.baseAddress));
        if (g_loopTimes++ >= LOOP_TIMES) {
            g_loopTimes = 0;
            /* Inject interrupt type error, trig int jump to callback */
            DBG_PRINTF("Inject interrupt type error\r\n");
            DCL_CFD_EnableInterruptInject(g_cfd.baseAddress, g_cfd.interruptType);
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Interrupt callback function triggered by module check errors.
  * @param handle CFD handle.
  * @return None.
  */
void CFD_CheckErrorCallback(CFD_Handle *handle)
{
    CFD_Handle* cfdHandle = (CFD_Handle*)handle;
    while (1) {
        /* after inject error, core freq switch to losc = 32khz */
        DBG_PRINTF("In CFD interrupt function : clock frequency error and \
                   core freq = %dhz\r\n", HAL_CRG_GetCoreClkFreq());
        BASE_FUNC_DELAY_S(1);
        /* loop control */
        if (g_loopTimes++ >= LOOP_TIMES) {
            g_loopTimes = 0;
            /* Disable Inject interrupt type error, jump to main loop */
            DBG_PRINTF("Disable inject interrupt type error\r\n");
            DCL_CFD_DisableInterruptInject(cfdHandle->baseAddress, cfdHandle->interruptType);
            return;
        }
    }
}