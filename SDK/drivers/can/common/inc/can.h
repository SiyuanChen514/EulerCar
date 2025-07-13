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
  * @file    can.h
  * @author  MCU Driver Team
  * @brief   CAN module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the CAN.
  *           + Definition of the CAN handle structure.
  *           + Initialization and de-initialization functions.
  *           + Sending and receiving CAN data frames functions.
  *           + Interrupt handler function and user registration callback function.
  */

/* Includes ------------------------------------------------------------------*/
#ifndef McuMagicTag_CAN_H
#define McuMagicTag_CAN_H

#include "can_ip.h"


/**
  * @defgroup CAN CAN
  * @brief CAN module.
  * @{
  */

/**
  * @defgroup CAN_Common CAN Common
  * @brief CAN common external module.
  * @{
  */

/**
  * @defgroup CAN_Handle_Definition ADC Handle Definition
  * @{
  */

/**
  * @brief Definition of the CAN handle structure.
  */
typedef struct _CAN_Handle {
    CAN_RegStruct                  *baseAddress;        /**< CAN registers base address */
    CAN_TypeMode                    typeMode;           /**< Work mode */
    CAN_TestMode_Configure         *testModeConfigure;  /**< Test mode configure */
    CAN_Seg1_Phase                  seg1Phase;          /**< Seg1Phase: Phase Buffer Section 1, propagation section */
    CAN_Seg2_Phase                  seg2Phase;          /**< Seg2Phase: Phase Buffer Section 2 */
    unsigned int                    prescalser;         /**< CAN frequency divider, range: 1 ~ 64 */
    CAN_Sync_Jump_Width             sjw;                /**< Sync jump width coefficient */
    volatile CAN_State_Type         state;              /**< Transmit status of the CAN. */
    volatile CANFrame              *rxFrame;            /**< Rx buff */
    CAN_FilterConfigure            *rxFilter;           /**< Received Frame Filtering Configuration */
    unsigned int                    rxFIFODepth;        /**< Number of receive FIFO composed by packet objects */
    bool                            autoRetrans;        /**< Automatic retransmission of interfered message */

    CAN_UserCallBack                userCallBack;       /**< User call back function of CAN */
    CAN_ExtendHandle                handleEx;           /**< CAN extend handle */
} CAN_Handle;

typedef void (* CAN_CallbackType)(void *handle);
/**
  * @}
  */

/**
  * @defgroup CAN_API_Declaration CAN HAL API
  * @{
  */
BASE_StatusType HAL_CAN_Init(CAN_Handle *canHandle);
BASE_StatusType HAL_CAN_DeInit(CAN_Handle *canHandle);
BASE_StatusType HAL_CAN_ReadIT(CAN_Handle *canHandle, CANFrame *data, CAN_FilterConfigure *filterConfigure);
BASE_StatusType HAL_CAN_Write(CAN_Handle *canHandle, CANFrame *data);

/* CAN status */
CAN_ErrorStatus HAL_CAN_GetErrorStatus(CAN_Handle *canHandle);
unsigned int HAL_CAN_GetErrorStatusCode(CAN_Handle *canHandle);
CAN_BusOffStatus HAL_CAN_GetBusOffStatus(CAN_Handle *canHandle);
CAN_MessageReceiveStatus HAL_CAN_MessageReceiveStatus(CAN_Handle *canHandle);
CAN_MessageSendStatus HAL_CAN_MessageSendStatus(CAN_Handle *canHandle);
/* CAN interrupt service funciton. */
void HAL_CAN_IrqHandler(void *handle);
BASE_StatusType HAL_CAN_RegisterCallBack(CAN_Handle *canHandle, CAN_CallBackFunType typeID,
                                         CAN_CallbackType pCallback);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_CAN_H */