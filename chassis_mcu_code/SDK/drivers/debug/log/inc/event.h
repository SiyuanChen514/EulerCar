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
  * @file    event.h
  * @author  MCU Driver Team
  * @brief   Header file containing functions prototypes of erron module.
  *          + Defines the function of reporting initialization events.
  */
#ifndef EVENT_CODE_H
#define EVENT_CODE_H

#include "module.h"
#include "type.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

/**
  * @addtogroup DEBUG_Log
  * @brief DEBUG external module.
  * @{
  */

 /**
  * @defgroup EVENT_Def EVENT_Def
  * @brief Definition of the Event Reporting Function.
  * @{
  */

#define SYS_GPIO_GROUP_REPORT 0
#define USER_CMD_MAX_LEN 8
#define EVENT_MAX_LEN 5

#define REPORT_EVENT_DONE 0xFFFFFFFF


typedef enum {
    REPORT_LAST_WORD_EVENT,
} REPORT_EVENT;

typedef enum {
    BUS_SLAVE_IRQ_INT_WRITE_START,
    BUS_SLAVE_IRQ_INT_WRITE_END,
    BUS_SLAVE_IRQ_INT_READ_START,
    BUS_SLAVE_IRQ_INT_READ_END,
    BUS_SLAVE_IRQ_PGM_WRITE_START,
    BUS_SLAVE_IRQ_PGM_WRITE_END,
    BUS_SLAVE_IRQ_PGM_READ_START,
    BUS_SLAVE_IRQ_PGM_READ_END,
    BUS_SLAVE_IRQ_INT_FIFO,
    BUS_SLAVE_IRQ_EXCEPTION,
    BUS_SLAVE_IRQ_BUTT
} BUSS_IRQ_Type;

typedef struct {
    unsigned short cmd; /* Commands delivered by the user */
    unsigned char ack;
    unsigned char len;
    unsigned char param[USER_CMD_MAX_LEN];
} UserCmd;

typedef void (*pfnCB)(unsigned int);

typedef enum {
    DATA_TYPE_NOISE, DATA_TYPE_SELF, DATA_TYPE_STYLUS, DATA_TYPE_MUTUAL
} DataType;

typedef struct {
    unsigned char eventType;
    unsigned char ack;
    unsigned char len;
    unsigned char param[EVENT_MAX_LEN];
} McuEvent;

typedef struct {
    unsigned int       reportType; /* Report Type */
    McuEvent         event;
} McuReport;

typedef struct {
    McuReport report;
    pfnCB pfnevent;
} UserEventObj;

typedef struct {
    unsigned int reportLock;
    int gpioHandle;
    unsigned int reportAddr;
    unsigned short cmdNotFoudCount;
    unsigned short reportFailedCount;
    UserEventObj eventObj;
    UserCmd cmd;
} UserMgr;

/**
 * @brief report event
 * @attention None
 *
 * @param report event struct
 * @retval The return value indicates that the event is reported successfully or failed.
 */
int UserReportEvent(UserEventObj *eventObj);

/**
 * @brief init event
 * @attention None
 *
 * @retval The return value indicates that the event is reported successfully or failed.
 */
int EventInit(void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __EXT_DEBUG_H__ */