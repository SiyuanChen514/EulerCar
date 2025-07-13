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
  * @file    event.c
  * @author  MCU Driver Team
  * @brief   Header file containing functions prototypes of erron module.
  *          + Defines the function of reporting initialization events.
  */
#include "event.h"
#include "console.h"
#include "command.h"
#include "common.h"
#include "typedefs.h"

UserMgr g_userMgr;

/**
 * @brief Event report.
 * @param eventObj : Unsolicitedly reported events
 * @retval Indicates whether the upload is successful.
 */
static inline int UserReport(UserEventObj *eventObj)
{
    unsigned int *reportAddr = (unsigned int *)&g_userMgr.reportAddr;

    /* Obtain reported events */
    *reportAddr = (uintptr_t)(void *)&eventObj->report;
    EXT_PRINT("event report type: %u event type: %u ", eventObj->report.event.eventType, eventObj->report.reportType);
    g_userMgr.reportLock = 0;
    return EXT_SUCCESS;
}

/**
 * @brief Obtains the address for reporting events.
 * @param None
 * @retval Address to which the event is reported.
 */
static UserEventObj *UserGetEventObj(void)
{
    /* The event is locked and cannot be reported */
    if (g_userMgr.reportLock == 1) {
        g_userMgr.reportFailedCount++;
        return NULL;
    }

    g_userMgr.reportLock = 1;
    return &g_userMgr.eventObj;
}

/**
 * @brief Reporting an event
 * @param eventObj: Structure for storing reported events
 * @retval Indicates whether the upload is successful.
 */
int UserReportEvent(UserEventObj *eventObj)
{
    UserEventObj *obj = (UserEventObj *)UserGetEventObj();
    /* If it is locked, it cannot be reported */
    if (obj == NULL) {
        return EXT_FAILURE;
    }

    *obj = *eventObj;

    return UserReport(&g_userMgr.eventObj);
}

/**
 * @brief RInitializing event reporting
 * @param None
 * @retval For user-defined
 */
int EventInit(void)
{
    /* Users can customize event reporting based on their requirements */
    return 0;
}