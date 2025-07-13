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
  * @file    can.c
  * @author  MCU Driver Team
  * @brief   CAN module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the CAN.
  *           + Initialization and de-initialization functions
  *           + Sending and receiving CAN data frames functions
  *           + Interrupt handling function and user registration callback function
  *           + CAN data frame filtering function
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "interrupt.h"

/* Macro definitions ---------------------------------------------------------*/

#define BOUND_ID 24  /* ObjID 1 ~ 24 used for receive, 25 ~ 32 used for send */
#define CAN_OBJ_MAXNUM 32

#define CAN_EFF_FLAG 0x80000000 /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000 /* Remote transmission request */
#define CAN_ERR_FLAG 0x20000000 /* Error message frame */

/* Valid bits in CAN ID for frame formats */
#define CAN_STD_MASK 0x000007FF /* Standard frame format (SFF) */
#define CAN_EXT_MASK 0x1FFFFFFF /* Extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFF /* Omit EFF, RTR, ERR flags */
#define CAN_TIME_WAIT 11        /* CAN initialization wait time */

static unsigned int g_stdRecvMap = 0x00000FFF;
static unsigned int g_extRecvMap = 0x00FFF000;
static unsigned int g_allSendMap = 0xFF000000;
static unsigned int g_allRecvMap = 0x00FFFFFF;

static unsigned int g_msgObj[CAN_OBJ_MAXNUM] = {0};

static BASE_StatusType CAN_ReadCallback(CAN_Handle *canHandle, unsigned int objId);
static BASE_StatusType CAN_ConfigReadReq(CAN_Handle *canHandle, unsigned int objId);
static BASE_StatusType WriteFinishClear(CAN_Handle *canHandle, unsigned int objId);
static void CAN_ReceiveFilter(CAN_Handle *canHandle, const CAN_FilterConfigure *filterConfigure, unsigned int objId);
static void CAN_WaitTime(CAN_Handle *canHandle);
static void CAN_AutoRetrans(CAN_Handle *canHandle);

/* Initialization and de-initialization functions ----------------------------*/
/**
 * @brief Wait 11 CAN bit time.
 * @param canHandle CAN handle.
 * @retval void
 */
static void CAN_WaitTime(CAN_Handle *canHandle)
{
    unsigned int canFrep = HAL_CRG_GetIpFreq((void *)CAN_BASE) / (canHandle->prescalser); /* CAN clock frequency */
    unsigned int waitTime = canFrep / (1 + canHandle->seg1Phase + canHandle->seg2Phase);
    /* 1000000 equals 1 us to wait for 11 time bits */
    unsigned int waitTimeCount = CAN_TIME_WAIT * ((1000000) / waitTime);
    BASE_FUNC_DelayUs(waitTimeCount);
}

/**
 * @brief CAN Setting Automatic Retransmission.
 * @param canHandle CAN handle.
 * @retval void
 */
static void CAN_AutoRetrans(CAN_Handle *canHandle)
{
    if (canHandle->autoRetrans == BASE_CFG_DISABLE) {
        /* Turn off auto retransmission */
        canHandle->baseAddress->CAN_CONTROL.BIT.DAR = BASE_CFG_ENABLE;
    } else {
        /* Turn on auto retransmission */
        canHandle->baseAddress->CAN_CONTROL.BIT.DAR = BASE_CFG_DISABLE;
    }
}

/**
  * @brief Initialize the CAN hardware configuration and configure parameters based on the specified handle.
  * @param canHandle CAN handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT
  */
BASE_StatusType HAL_CAN_Init(CAN_Handle *canHandle)
{
    CAN_ASSERT_PARAM(canHandle != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    CAN_PARAM_CHECK_WITH_RET(IsCanMode(canHandle->typeMode), BASE_STATUS_ERROR); /* Check initialization parameters */
    CAN_PARAM_CHECK_WITH_RET(IsCanPrescalser(canHandle->prescalser), BASE_STATUS_ERROR);
    CAN_PARAM_CHECK_WITH_RET(IsCanSeg1phase(canHandle->seg1Phase), BASE_STATUS_ERROR);
    CAN_PARAM_CHECK_WITH_RET(IsCanSeg2phase(canHandle->seg2Phase), BASE_STATUS_ERROR);
    CAN_PARAM_CHECK_WITH_RET(IsCanSJW(canHandle->sjw), BASE_STATUS_ERROR);
    unsigned int busy;
    /* Step1: init enable */
    canHandle->baseAddress->CAN_CONTROL.BIT.Init = BASE_CFG_ENABLE;
    /* Step2: configuration command mask register, set 0xF3 to write into packet objects */
    canHandle->baseAddress->IF1_COMMAND_MASK.reg = 0xF3;
    /* Step3 ~ 4: init packet object 1 ~ 32 */
    for (int i = 1; i <= CAN_OBJ_MAXNUM; i++) {
        canHandle->baseAddress->IF1_COMMAND_REQUEST.reg = i;
        do {
            busy = canHandle->baseAddress->IF1_COMMAND_REQUEST.BIT.BUSY;
        } while (busy == BASE_CFG_ENABLE);
    }
    /* Step5: Bit_Timing setting enable, [0]bit and [6]bit need are set, others clear */
    canHandle->baseAddress->CAN_CONTROL.reg = 0x41;
    /* Step6: Bit_Timing configuration */
    unsigned int val = canHandle->prescalser - 1;  /* The prescalser is set to the lower 6 bits, [5:0] */
    val |= (canHandle->sjw - 1) << 6;              /* The sjw needs to be shifted leftwards by 6 bits, [7:6] */
    val |= (canHandle->seg1Phase - 1) << 8;        /* The seg1Phase needs to be shifted leftwards by 8 bits, [11:8] */
    val |= (canHandle->seg2Phase - 1) << 12;       /* The seg2Phase needs to be shifted leftwards by 12 bits, [14:12] */
    canHandle->baseAddress->BIT_TIMING.reg = val;
    /* Step7: setting interrupt configuration, error interrupt and module interrupt */
    canHandle->baseAddress->CAN_CONTROL.reg = 0x0B;
    /* Step8: setting automatic retransmission */
    CAN_AutoRetrans(canHandle);
    /* Step9: finish init */
    if (canHandle->typeMode == CAN_MODE_TEST && canHandle->testModeConfigure != NULL) {
        canHandle->baseAddress->CAN_CONTROL.BIT.Test = BASE_CFG_ENABLE;
        canHandle->baseAddress->CAN_TEST.BIT.Lback = canHandle->testModeConfigure->loopBack;
        canHandle->baseAddress->CAN_TEST.BIT.Silent = canHandle->testModeConfigure->silent;
        canHandle->baseAddress->CAN_TEST.BIT.Basic = canHandle->testModeConfigure->basic;
    }
    canHandle->baseAddress->CAN_CONTROL.BIT.Init = BASE_CFG_DISABLE;
    /* Each packet object configuration before read CAN frame */
    for (int i = 1; i <= CAN_OBJ_MAXNUM; i++) {
        if (i <= BOUND_ID) {
            CAN_ConfigReadReq(canHandle, i);  /* The default configuration is no filter receive */
        }
        g_msgObj[i - 1] = 0;
    }
    CAN_WaitTime(canHandle);
    canHandle->state = CAN_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief DeInitialize the CAN and restoring default parameters based on the specified handle.
  * @param canHandle CAN handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT
  */
BASE_StatusType HAL_CAN_DeInit(CAN_Handle *canHandle)
{
    CAN_ASSERT_PARAM(canHandle != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    canHandle->baseAddress->CAN_CONTROL.reg = BASE_CFG_DISABLE; /* Disables the control register. */
    canHandle->baseAddress->CAN_STATUS.reg = BASE_CFG_DISABLE;  /* Clear the status of the CAN. */
    canHandle->userCallBack.ReadFinishCallBack = NULL;         /* Clear all user call back function. */
    canHandle->userCallBack.TransmitErrorCallBack = NULL;
    canHandle->userCallBack.WriteFinishCallBack = NULL;
    canHandle->state = CAN_STATE_NONE_INIT; /* Set the CAN to the uninitialized state. */
    return BASE_STATUS_OK;
}

/**
  * @brief CAN error status.
  * @param canHandle CAN handle.
  * @retval CAN_ErrorStatus:
  *         CAN_PASSIVE_ERROR: CAN is in passive error.
  *         CAN_ACTIVE_ERROR:  CAN is in active error.
  */
CAN_ErrorStatus HAL_CAN_GetErrorStatus(CAN_Handle *canHandle)
{
    CAN_ASSERT_PARAM(canHandle != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    CAN_STATUS_REG canStatus;
    /* CAN error status reg. */
    canStatus.reg = canHandle->baseAddress->CAN_STATUS.reg;
    if (canStatus.BIT.Epass == BASE_CFG_ENABLE) {
        return CAN_PASSIVE_ERROR;  /* Passive error */
    }
    return CAN_ACTIVE_ERROR;      /* active error */
}

/**
  * @brief Return the specified CAN error status code.
  * @param canHandle CAN handle.
  * @retval CAN bus error status. @ref CAN_ERROR_StatusCode
  * @note   CAN_ERROR_NONE    --  No error normal.
  *         CAN_ERROR_PADDING --  Filling error.
  *         CAN_ERROR_FORMAL  --  The format is incorrect.
  *         CAN_ERROR_ANSWER  --  response error.
  *         CAN_ERROR_BIT1    --  Bit 1 error.
  *         CAN_ERROR_BIT0    --  bit 0 error.
  *         CAN_ERROR_CRC     --  CRC error.
  */
unsigned int HAL_CAN_GetErrorStatusCode(CAN_Handle *canHandle)
{
    CAN_ASSERT_PARAM(canHandle != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    CAN_STATUS_REG canStatus;
    canStatus.reg = canHandle->baseAddress->CAN_STATUS.reg;  /* Obtains the full status of the CAN. */
    unsigned int errorStatus = canStatus.BIT.LEC;
    return errorStatus;
}

/**
  * @brief CAN Bus-off status.
  * @param canHandle CAN handle.
  * @retval CAN Bus off status:
  *         CAN_BUSOFF_ON: In bus-off state.
  *         CAN_BUSOFF_OFF: Not in the bus-off state.
  */
CAN_BusOffStatus HAL_CAN_GetBusOffStatus(CAN_Handle *canHandle)
{
    CAN_ASSERT_PARAM(canHandle != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    CAN_STATUS_REG canStatus;  /* CAN status reg. */
    canStatus.reg = canHandle->baseAddress->CAN_STATUS.reg; /* Obtains all status of the CAN. */
    if (canStatus.BIT.Boff == BASE_CFG_ENABLE) {
        return CAN_BUSOFF_ON;
    }
    return CAN_BUSOFF_OFF;
}

/**
  * @brief CAN message receive status.
  * @param canHandle CAN handle.
  * @retval CAN message receive status:
  *         CAN_MESSAGE_RECEIVE_OK: message receive successful.
  *         CAN_MESSAGE_RECEIVE_ERROR: message receive failed.
  */
CAN_MessageReceiveStatus HAL_CAN_MessageReceiveStatus(CAN_Handle *canHandle)
{
    CAN_ASSERT_PARAM(canHandle != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    CAN_STATUS_REG canStatus;
    /* Obtains the CAN RX status. */
    canStatus.reg = canHandle->baseAddress->CAN_STATUS.reg;
    if (canStatus.BIT.RxOk == BASE_CFG_ENABLE) {
        return CAN_MESSAGE_RECEIVE_OK;   /* CAN receive succeeded. */
    }
    return CAN_MESSAGE_RECEIVE_ERROR;
}

/**
  * @brief CAN message send status.
  * @param canHandle CAN handle.
  * @retval CAN message send status:
  *         CAN_MESSAGE_SEND_OK: message send successful.
  *         CAN_MESSAGE_SEND_ERROR: message send failed.
  */
CAN_MessageSendStatus HAL_CAN_MessageSendStatus(CAN_Handle *canHandle)
{
    CAN_ASSERT_PARAM(canHandle != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    CAN_STATUS_REG canStatus;
    /* Obtains the CAN RX status.  */
    canStatus.reg = canHandle->baseAddress->CAN_STATUS.reg;
    if (canStatus.BIT.TxOk == BASE_CFG_ENABLE) {
        return CAN_MESSAGE_SEND_OK;     /* CAN send succeeded. */
    }
    return CAN_MESSAGE_SEND_ERROR;
}

/**
  * @brief Padding can data frame 8-bit data.
  * @param canHandle CAN handle.
  * @param data Pointer address of the CAN data frame to be sent, @ref CANFrame
  * @retval None.
  */
static void WriteData(CAN_Handle *canHandle, CANFrame *data)
{
    IF1_DATAA1_REG dataA1;
    dataA1.BIT.DATA0 = data->frame[0];  /* Data of bit 0 */
    dataA1.BIT.DATA1 = data->frame[1];  /* Data of bit 0 */
    canHandle->baseAddress->IF1_DATAA1 = dataA1;
    IF1_DATAA2_REG dataA2;
    dataA2.BIT.DATA2 = data->frame[2];  /* Data of bit 2 */
    dataA2.BIT.DATA3 = data->frame[3];  /* Data of bit 3 */
    canHandle->baseAddress->IF1_DATAA2 = dataA2;
    IF1_DATAB1_REG dataB1;
    dataB1.BIT.DATA4 = data->frame[4];  /* Data of bit 4 */
    dataB1.BIT.DATA5 = data->frame[5];  /* Data of bit 5 */
    canHandle->baseAddress->IF1_DATAB1 = dataB1;
    IF1_DATAB2_REG dataB2;
    dataB2.BIT.DATA6 = data->frame[6];  /* Data of bit 6 */
    dataB2.BIT.DATA7 = data->frame[7];  /* Data of bit 7 */
    canHandle->baseAddress->IF1_DATAB2 = dataB2;
}

/**
  * @brief Send data immediately.
  * @param canHandle CAN handle.
  * @param data Pointer address of the CAN data frame to be sent, @ref CANFrame
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT
  * @note:
  *      IF1 and IF2 have the same functions. To facilitate management,
  *      IF1 is used for sending and IF2 is used for receiving.
  */
BASE_StatusType HAL_CAN_Write(CAN_Handle *canHandle, CANFrame *data)
{
    CAN_ASSERT_PARAM(canHandle != NULL && data != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    CAN_PARAM_CHECK_WITH_RET(data->dataLength <= 8, BASE_STATUS_ERROR);  /* CAN frame length: 1 ~ 8 */
    if (canHandle->state != CAN_STATE_READY) {
        return BASE_STATUS_BUSY;
    }
    canHandle->state = CAN_STATE_BUSY_TX;
    unsigned int objId = 0;
    unsigned int id;
    for (int i = BOUND_ID + 1; i <= CAN_OBJ_MAXNUM; i++) {
        if (g_msgObj[i - 1] == 0) {
            g_msgObj[i - 1] = 1;
            objId = i;
            break;
        }
    }
    if (objId == 0) {
        return BASE_STATUS_ERROR;
    }
    /* Step1: write id into register arbitration according frame type */
    switch (data->type) {
        case CAN_TYPEFRAME_STD_DATA:                                /* Standard data frame */
            id = (data->CANId & CAN_STD_MASK) << 2;                 /* Bit[12:2] = CANId */
            id |= 0xA000;                                           /* [15:13] = 0x05 */
            canHandle->baseAddress->IF1_ARBITRATION1.reg = 0x00;
            break;
        case CAN_TYPEFRAME_EXT_DATA:                                /* Extended data frame */
            id = (data->CANId & CAN_EXT_MASK) >> 16;                /* Bit[12:0] = CANId(28bit~16bit) */
            id |= 0xE000;                                           /* [15:13] = 0x07 */
            canHandle->baseAddress->IF1_ARBITRATION1.reg = (data->CANId & 0xFFFF);  /* write lower 16bits CANId */
            break;
        case CAN_TYPEFRAME_STD_REMOTE:                              /* Standard remote frame */
            id = (data->CANId & CAN_EXT_MASK) << 2;                 /* Bit[12:2] = CANId */
            id |= 0x8000;                                           /* [15:13] = 0x04 */
            canHandle->baseAddress->IF1_ARBITRATION1.reg = 0x00;
            break;
        case CAN_TYPEFRAME_EXT_REMOTE:                              /* Extended remote frame */
            id = (data->CANId & CAN_EXT_MASK) >> 16;                /* Bit[12:0] = CANId(28bit~16bit) */
            id |= 0xC000;                                           /* [15:13] = 0x06 */
            canHandle->baseAddress->IF1_ARBITRATION1.reg = (data->CANId & 0xFFFF);  /* write lower 16bits CANId */
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    canHandle->baseAddress->IF1_ARBITRATION2.reg = id;
    /* Step2: setting mask register 2 */
    canHandle->baseAddress->IF1_MASK2.reg = 0x8000;
    /* Step3: setting mask register 1 */
    canHandle->baseAddress->IF1_MASK1.reg = 0x0000;
    /* Step4: setting message control register */
    canHandle->baseAddress->IF1_MESSAGE_CONTROL.reg |= 0x8980;
    canHandle->baseAddress->IF1_MESSAGE_CONTROL.BIT.DLC = data->dataLength;
    /* Step5: write data to be sent */
    WriteData(canHandle, data);
    /* Step6: send configuration to packet objects */
    canHandle->baseAddress->IF1_COMMAND_MASK.reg = 0xF3;
    /* Step7: write IF1 request command */
    canHandle->baseAddress->IF1_COMMAND_REQUEST.BIT.MessageNumber = objId;
    canHandle->state = CAN_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief Interrupt receiving callback function.
  * @param canHandle CAN handle.
  * @param objId Indicates the packet object ID.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType CAN_ReadCallback(CAN_Handle *canHandle, unsigned int objId)
{
    CAN_ASSERT_PARAM(canHandle != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    CAN_PARAM_CHECK_WITH_RET(canHandle->rxFrame != NULL, BASE_STATUS_ERROR);
    CAN_PARAM_CHECK_WITH_RET((objID >= MESSAGE_NUMBER_MIN) && (objID <= MESSAGE_NUMBER_MAX), BASE_STATUS_ERROR);
    unsigned int busy, id, idLow, idHigh,  extendedFrame, remoteFrame;
    /* Step1: setting request transfer to packet object */
    canHandle->baseAddress->IF2_COMMAND_MASK.reg = 0x7F;  /* 0x7F indicates reading data from the packet object */
    /* Step2: Request specififed packet object */
    canHandle->baseAddress->IF2_COMMAND_REQUEST.BIT.MessageNumber = objId;
    do {
        busy = canHandle->baseAddress->IF2_COMMAND_REQUEST.BIT.BUSY;
    } while (busy != 0x00);
    /* Step3: Obtains packet information */
    extendedFrame = canHandle->baseAddress->IF2_ARBITRATION2.BIT.Xtd;
    remoteFrame = canHandle->baseAddress->IF2_ARBITRATION2.BIT.Dir;
    if (extendedFrame == BASE_CFG_ENABLE) {
        idLow = canHandle->baseAddress->IF2_ARBITRATION1.BIT.ID;
        idHigh = canHandle->baseAddress->IF2_ARBITRATION2.BIT.ID;
        id = idLow;
        idHigh <<= 16;  /* High 16 bits ID */
        id |= idHigh;
        canHandle->rxFrame->CANId = id;
        id |= CAN_EFF_FLAG;
        if (remoteFrame == BASE_CFG_ENABLE) {
            id |= CAN_RTR_FLAG;
            canHandle->rxFrame->type = CAN_TYPEFRAME_EXT_REMOTE;
        } else {
            canHandle->rxFrame->type = CAN_TYPEFRAME_EXT_DATA;
        }
    } else {
        id = canHandle->baseAddress->IF2_ARBITRATION2.BIT.ID;
        id >>= 2;  /* 2: Standard frame CAN id. */
        canHandle->rxFrame->CANId = id;
        if (remoteFrame == BASE_CFG_ENABLE) {
            id |= CAN_RTR_FLAG;
            canHandle->rxFrame->type = CAN_TYPEFRAME_STD_REMOTE;
        } else {
            canHandle->rxFrame->type = CAN_TYPEFRAME_STD_DATA;
        }
    }
    canHandle->rxFrame->dataLength = canHandle->baseAddress->IF2_MESSAGE_CONTROL.BIT.DLC;
    canHandle->rxFrame->frame[0] = canHandle->baseAddress->IF2_DATAA1.BIT.DATA0;  /* Data of bit 0 */
    canHandle->rxFrame->frame[1] = canHandle->baseAddress->IF2_DATAA1.BIT.DATA1;  /* Data of bit 1 */
    canHandle->rxFrame->frame[2] = canHandle->baseAddress->IF2_DATAA2.BIT.DATA2;  /* Data of bit 2 */
    canHandle->rxFrame->frame[3] = canHandle->baseAddress->IF2_DATAA2.BIT.DATA3;  /* Data of bit 3 */
    canHandle->rxFrame->frame[4] = canHandle->baseAddress->IF2_DATAB1.BIT.DATA4;  /* Data of bit 4 */
    canHandle->rxFrame->frame[5] = canHandle->baseAddress->IF2_DATAB1.BIT.DATA5;  /* Data of bit 5 */
    canHandle->rxFrame->frame[6] = canHandle->baseAddress->IF2_DATAB2.BIT.DATA6;  /* Data of bit 6 */
    canHandle->rxFrame->frame[7] = canHandle->baseAddress->IF2_DATAB2.BIT.DATA7;  /* Data of bit 7 */
    return BASE_STATUS_OK;
}

/**
  * @brief CAN Bus receive filtering configuration.
  * @param canHandle CAN handle.
  * @param CAN_FilterConfigure handle of filtering configuration, @ref CAN_FilterConfigure
  * @param objId Indicates the packet object ID.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static void CAN_ReceiveFilter(CAN_Handle *canHandle, const CAN_FilterConfigure *filterConfigure, unsigned int objId)
{
    unsigned int id, idChg;
    unsigned int mask, maskChg;
    idChg = filterConfigure->filterID & 0xFFFF;
    maskChg = filterConfigure->filterMask & 0xFFFF;
    switch (filterConfigure->receiveType) {
        case CAN_FILTERFRAME_STD_DATA:
            id = (filterConfigure->filterID & CAN_STD_MASK) << 2;       /* Bit[12:2] = CANId */
            id |= 0x8000;
            idChg = 0x0000;
            /* Shift left by 2 bits. The upper 11 bits of [12:0] are used */
            mask = (filterConfigure->filterMask & CAN_STD_MASK) << 2;
            mask |= 0xC000;
            maskChg = 0x0000;
            break;
        case CAN_FILTERFRAME_EXT_DATA:
            id = (filterConfigure->filterID & CAN_EXT_MASK) >> 16;              /* Bit[12:0] = CANId(28bit ~ 16bit) */
            id |= 0xC000;
            /* write lower 16bits CANId */
            mask = (filterConfigure->filterMask & CAN_EXT_MASK) >> 16;  /* Remove the lower 16 bits */
            mask |= 0xC000;
            break;
        case CAN_FILTERFRAME_STD_EXT_DATA:
            id = (filterConfigure->filterID & CAN_EXT_MASK) >> 16;       /* Remove the lower 16 bits */
            id |= 0xC000;
            mask = (filterConfigure->filterMask & CAN_EXT_MASK) >> 16;  /* Remove the lower 16 bits */
            mask |= 0x4000;                                             /* [15]MXtd = 0 */
            break;
        default:
            return;
    }
    canHandle->baseAddress->IF2_ARBITRATION2.reg = id;
    canHandle->baseAddress->IF2_ARBITRATION1.reg = idChg;
    canHandle->baseAddress->IF2_MASK2.reg = mask;
    canHandle->baseAddress->IF2_MASK1.reg = maskChg;
    if (canHandle->rxFIFODepth > BOUND_ID) {
        canHandle->rxFIFODepth = BOUND_ID;
    }
    if (objId < canHandle->rxFIFODepth) {  /* packet objects form the receiving FIFO */
        canHandle->baseAddress->IF2_MESSAGE_CONTROL.reg = 0x1408;   /* EOB is set 0 */
    } else {
        canHandle->baseAddress->IF2_MESSAGE_CONTROL.reg = 0x1488;   /* EOB is set 1 */
    }
    /* Step5: send configuration to packet objects */
    canHandle->baseAddress->IF2_COMMAND_MASK.reg = 0x00F3;
    /* Step6: write IF2 request command */
    canHandle->baseAddress->IF2_COMMAND_REQUEST.BIT.MessageNumber = objId;
    canHandle->baseAddress->IF2_COMMAND_REQUEST.BIT.BUSY = 0x01;
}

/**
  * @brief Receive CAN data frames asynchronously.
  * @param canHandle CAN handle.
  * @param data Address for storing CAN data frames, @ref CANFrame
  * @param filterConfigure handle of filtering configuration, @ref CAN_FilterConfigure
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_CAN_ReadIT(CAN_Handle *canHandle, CANFrame *data, CAN_FilterConfigure *filterConfigure)
{
    CAN_ASSERT_PARAM(canHandle != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    CAN_PARAM_CHECK_WITH_RET(data != NULL, BASE_STATUS_ERROR);
    CAN_PARAM_CHECK_WITH_RET(filterConfigure != NULL, BASE_STATUS_ERROR);
    if (canHandle->state != CAN_STATE_READY) {
        return BASE_STATUS_BUSY;
    }
    canHandle->state = CAN_STATE_BUSY_RX;
    canHandle->rxFrame = data;
    canHandle->rxFilter = filterConfigure;
    for (int i = 1; i <= BOUND_ID; i++) {
        CAN_ReceiveFilter(canHandle, filterConfigure, i);
    }
    canHandle->state = CAN_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief Pre-configuration of Receive CAN Data Frames.
  * @param canHandle CAN handle.
  * @param objId Indicates the packet object ID.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType CAN_ConfigReadReq(CAN_Handle *canHandle, unsigned int objId)
{
    CAN_ASSERT_PARAM(canHandle != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    unsigned int map = 1;
    map <<= objId - 1;
    /* Step1: write id into register arbitration according frame type */
    if ((map & g_stdRecvMap) != 0) {            /* STD DATA FRAME */
        canHandle->baseAddress->IF2_ARBITRATION2.reg = 0x8000;
        canHandle->baseAddress->IF2_ARBITRATION1.reg = 0x0000;
    } else if ((map & g_extRecvMap) != 0) {     /* EXTENDED DATA FRAME */
        canHandle->baseAddress->IF2_ARBITRATION2.reg = 0xC000;
        canHandle->baseAddress->IF2_ARBITRATION1.reg = 0x0000;
    } else {
        return BASE_STATUS_ERROR;
    }
    /* Step2: setting mask register 2 */
    canHandle->baseAddress->IF2_MASK2.reg = 0xC000;
    /* Step3: setting mask register 1 */
    canHandle->baseAddress->IF2_MASK1.reg = 0x0000;
    /* Step4: setting message control register. By default, there is no RX FIFO and no filtering is performed */
    canHandle->baseAddress->IF2_MESSAGE_CONTROL.reg = 0x1488;
    /* Step5: send configuration to packet objects */
    canHandle->baseAddress->IF2_COMMAND_MASK.reg = 0x00F3;
    /* Step6: write IF2 request command */
    canHandle->baseAddress->IF2_COMMAND_REQUEST.BIT.MessageNumber = objId;
    canHandle->baseAddress->IF2_COMMAND_REQUEST.BIT.BUSY = BASE_CFG_ENABLE;
    return BASE_STATUS_OK;
}

/**
  * @brief The object of the sent packet is cleared.
  * @param canHandle CAN handle.
  * @param objId Indicates the packet object ID.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType WriteFinishClear(CAN_Handle *canHandle, unsigned int objId)
{
    CAN_ASSERT_PARAM(canHandle != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    unsigned int busy;
    canHandle->baseAddress->IF2_COMMAND_MASK.reg = 0x7F;
    canHandle->baseAddress->IF2_COMMAND_REQUEST.BIT.MessageNumber = objId;
    do {
        busy = canHandle->baseAddress->IF2_COMMAND_REQUEST.BIT.BUSY;
    } while (busy != 0x00);
    return BASE_STATUS_OK;
}

/**
  * @brief Write interrupt service function.
  * @param canHandle CAN handle.
  * @param irqIndex Packet object interrupt ID.
  * @retval None.
  */
static void WriteIrqService(CAN_Handle *canHandle, unsigned int irqIndex)
{
    WriteFinishClear(canHandle, irqIndex);
    g_msgObj[irqIndex - 1] = 0;
    if (canHandle->userCallBack.WriteFinishCallBack != NULL) {
        canHandle->userCallBack.WriteFinishCallBack(canHandle);
    }
}

/**
  * @brief Read interrupt service function.
  * @param canHandle CAN handle.
  * @param irqIndex Packet object interrupt ID.
  * @retval None.
  */
static void ReadIrqService(CAN_Handle *canHandle, unsigned int irqIndex)
{
    CAN_ReadCallback(canHandle, irqIndex);
    if (canHandle->userCallBack.ReadFinishCallBack != NULL) {
        canHandle->userCallBack.ReadFinishCallBack(canHandle);
    }
}

/**
  * @brief CAN interrupt service processing function.
  * @param handle CAN handle.
  * @retval None.
  */
void HAL_CAN_IrqHandler(void *handle)
{
    CAN_ASSERT_PARAM(handle != NULL);
    CAN_Handle *canHandle = (CAN_Handle *)handle;
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    unsigned int irqIndex;
    unsigned int idLow, idHigh, id;
    irqIndex = canHandle->baseAddress->CAN_INTERRUPT.reg;
    /* Status interrupt ID: 0x8000 */
    if (irqIndex == 0x8000) {
        /* Offline status of the CAN bus. */
        unsigned int statusBusoff = canHandle->baseAddress->CAN_STATUS.BIT.Boff;
        if (statusBusoff == BASE_CFG_ENABLE) {  /* true when the bus-off state is displayed. */
            canHandle->baseAddress->CAN_CONTROL.BIT.Init = 0x01;
            __asm__ volatile ("nop");                   /* Hold-off time */
            canHandle->baseAddress->CAN_CONTROL.BIT.Init = 0x00;
        }
        if (canHandle->userCallBack.TransmitErrorCallBack != NULL) {
            canHandle->userCallBack.TransmitErrorCallBack(canHandle);
        }
    } else if (irqIndex >= 0x01 && irqIndex <= 0x20) { /* Packet object interrupt ID from 0x01 to 0x20 */
        idLow = canHandle->baseAddress->INTERRUPT_PENDING1.BIT.IntPnd16_1;
        idHigh = canHandle->baseAddress->INTERRUPT_PENDING2.BIT.IntPnd32_17;
        id = idLow;
        id |= idHigh << 16;          /* High 16 bits ID */
        if (id & g_allSendMap) {     /* Write complete */
            WriteIrqService(canHandle, irqIndex);
        }
        if (id & g_allRecvMap) {
            ReadIrqService(canHandle, irqIndex);
        }
    }
    return;
}

/**
  * @brief Handle CAN interrupt request.
  * @param canHandle CAN handle.
  * @param typeID Id of callback function type, @ref CAN_CallBackFunType
  * @param pCallback Pointer of the specified callbcak function, @ref CAN_CallbackType
  * @retval BASE_StatusType: BASE_STATUS_ERROR register error, BASE_STATUS_OK register success.
  */
BASE_StatusType HAL_CAN_RegisterCallBack(CAN_Handle *canHandle, CAN_CallBackFunType typeID, CAN_CallbackType pCallback)
{
    CAN_ASSERT_PARAM(canHandle != NULL);
    CAN_ASSERT_PARAM(IsCANInstance(canHandle->baseAddress));
    switch (typeID) {
        case CAN_WRITE_FINISH:    /* CAN write finish call back. */
            canHandle->userCallBack.WriteFinishCallBack = pCallback;
            break;
        case CAN_READ_FINISH:    /* CAN read finish call back. */
            canHandle->userCallBack.ReadFinishCallBack = pCallback;
            break;
        case CAN_TRNS_ERROR:     /* CAN transmit finish call back. */
            canHandle->userCallBack.TransmitErrorCallBack = pCallback;
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}