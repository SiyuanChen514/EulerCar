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
  * @file      cmm_check_error_sample.c
  * @author    MCU Driver Team
  * @brief     The CMM module monitors abnormal clocks and inject error after loop times in circle.
  * @details   Use the reference clock (LOSC/HOSC/TCXO/HS_CLK) to check whether the target target \
  *            clock (LOSC/HOSC/TCXO/HS_CLK/LS_CLK) is invalid. If the (LOSC/HOSC/TCXO/HS_CLK/LS_CLK) is invalid, \
  *            the interrupt service function is triggered for processing. You can design security protection \
  *            measures for the interrupt service function. If the interrupt trigger mode is set to non-clock \
  *            failure mode, the user needs to determine whether the count value is within the threshold after \
  *            each interrupt is triggered. If the count value is not within the threshold, the clock is \
  *            considered abnormal.
  *            The threshold includes the upper and lower thresholds. The upper and lower thresholds reflect \
  *            the redundancy of the system for the target clock offset. You can design the thresholds by yourself. \
  *            For details about how to calculate the thresholds, see the chip technical guide.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "debug.h"
#include "cmm.h"
#include "cmm_check_error_sample.h"
#include "main.h"

#define LOOP_TIMES    5

unsigned int g_loopTimes = 0;

void CMM_CheckErrorCallback(CMM_Handle *handle);

/**
  * @brief Sample main function.
  * @param None.
  * @return @ref BASE_StatusType
  */
BASE_StatusType CMM_SampleMain(void)
{
    SystemInit();
    HAL_CMM_Start(&g_cmm);
    while (1) {
        /* If ref clk cnt value bigger than upperbound, then junp to callback */
        BASE_FUNC_DELAY_S(1);
        DBG_PRINTF("main loop, CMM count value = %d\r\n", DCL_CMM_GetCntValue(g_cmm.baseAddress));
        if (g_loopTimes++ >= LOOP_TIMES - 1) {
            g_loopTimes = 0;
            /* Inject error, trig int jump to callback */
            DBG_PRINTF("\r\n Inject interrupt type error !\r\n");
            DCL_CMM_EnableInterruptInject(g_cmm.baseAddress, g_cmm.interruptType);
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Interrupt callback function triggered by module check errors.
  * @param handle CMM handle.
  * @return None.
  */
void CMM_CheckErrorCallback(CMM_Handle *handle)
{
    CMM_Handle *cmmHandle = (CMM_Handle*)handle;
    while (1) {
        DBG_PRINTF("In CMM interrupt function : clock frequency error\r\n");
        BASE_FUNC_DELAY_S(1);
        if (g_loopTimes++ >= LOOP_TIMES - 1) {
            g_loopTimes = 0;
            /* Disable Inject interrupt type error, trig int jump to callback */
            DBG_PRINTF("\r\n Disable inject interrupt type error\r\n");
            DCL_CMM_DisableInterruptInject(cmmHandle->baseAddress, cmmHandle->interruptType);
            return;
        }
    }
}