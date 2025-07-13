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
  * @file    can_ip.h
  * @author  MCU Driver Team
  * @brief   CAN module driver.
  * @details This file provides DCL functions to manage CAN and Definition of
  *          specific parameters
  *           + Definition of CAN configuration parameters.
  *           + CAN register mapping structure.
  *           + Parameters check functions.
  *           + Direct configuration layer interface
  */

/* Macro definitions */
#ifndef McuMagicTag_CAN_IP_H
#define McuMagicTag_CAN_IP_H

#define PRESCALSER_MIN 1
#define PRESCALSER_MAX 64

#define MESSAGE_NUMBER_MIN 1
#define MESSAGE_NUMBER_MAX 32

#ifdef CAN_PARAM_CHECK
#define CAN_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define CAN_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define CAN_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define CAN_ASSERT_PARAM(para) ((void)0U)
#define CAN_PARAM_CHECK_NO_RET(para) ((void)0U)
#define CAN_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

#include "baseinc.h"

/**
  * @addtogroup CAN
  * @{
  */

/**
  * @defgroup CAN_IP CAN_IP
  * @brief CAN_IP: can_v0.
  * @{
  */

/**
  * @defgroup CAN_Param_Def CAN Parameters Definition
  * @brief Definition of CAN configuration parameters.
  * @{
  */

/**
 * @brief Extent handle definition of CAN
 */
typedef struct {
} CAN_ExtendHandle;

/**
  * @brief Type ID of the callback function registered by the user.
  */
typedef enum {
    CAN_WRITE_FINISH  = 0x00000000U,
    CAN_READ_FINISH   = 0x00000001U,
    CAN_TRNS_ERROR    = 0x00000002U
} CAN_CallBackFunType;

/**
 * @brief Type define of user callback function
 */
typedef struct {
    void (* WriteFinishCallBack)(void *handle);    /**< CAN tx interrupt complete callback function for users */
    void (* ReadFinishCallBack)(void *handle);     /**< CAN rx interrupt complete callback function for users */
    void (* TransmitErrorCallBack)(void *handle);  /**< CAN mode error callback function for users */
} CAN_UserCallBack;

/**
  * @brief CAN state type.
  */
typedef enum {
    CAN_STATE_NONE_INIT = 0x00000000U,
    CAN_STATE_READY     = 0x00000001U,
    CAN_STATE_BUSY_TX   = 0x00000002U,
    CAN_STATE_BUSY_RX   = 0x00000003u,
} CAN_State_Type;

/**
  * @brief Error status code: the last error status on the CAN bus.
  */
typedef enum {
    CAN_ERROR_NONE    = 0x00000000U,
    CAN_ERROR_PADDING = 0x00000001U,
    CAN_ERROR_FORMAL  = 0x00000002U,
    CAN_ERROR_ANSWER  = 0x00000003U,
    CAN_ERROR_BIT1    = 0x00000004U,
    CAN_ERROR_BIT0    = 0x00000005U,
    CAN_ERROR_CRC     = 0x00000006U
} CAN_ERROR_StatusCode;

/**
  * @brief Indicates the error status.
  * @details CAN Error Status Code:
  *        +CAN_ACTIVE_ERROR: active error defined by the CAN protocol;
  *        +CAN_PASSIVE_ERROR: passive error defined by the CAN protocol.
  */
typedef enum {
    CAN_ACTIVE_ERROR  = 0x00000000U,
    CAN_PASSIVE_ERROR = 0x00000001U
} CAN_ErrorStatus;

/**
  * @brief Bus-off status.
  * @details CAN Bus-off Status:
  *          CAN_BUSSOFF_OFF: The CAN module is not in the bus-off state.
  *          CAN_BUSSOFF_ON: The CAN module is in the bus-off state.
  */
typedef enum {
    CAN_BUSOFF_OFF  = 0x00000000U,
    CAN_BUSOFF_ON   = 0x00000001U
} CAN_BusOffStatus;

/**
  * @brief Status of CAN message receive status.
  * @details CAN message receive status:
  *          CAN_MESSAGE_RECEIVE_OK: receive message success.
  *          CAN_MESSAGE_RECEIVE_ERROR: receive message error.
  */
typedef enum {
    CAN_MESSAGE_RECEIVE_OK    = 0x00000000U,
    CAN_MESSAGE_RECEIVE_ERROR = 0x00000001U
}CAN_MessageReceiveStatus;

/**
  * @brief Status of CAN message send status.
  * @details CAN message receive status:
  *          CAN_MESSAGE_SEND_OK: send message success.
  *          CAN_MESSAGE_SEND_ERROR: send message error.
  */
typedef enum {
    CAN_MESSAGE_SEND_OK    = 0x00000000U,
    CAN_MESSAGE_SEND_ERROR = 0x00000001U
}CAN_MessageSendStatus;

/**
  * @brief Work mode select.
  */
typedef enum {
    CAN_MODE_NORMAL = 0x00000000U,
    CAN_MODE_TEST   = 0x00000001U
} CAN_TypeMode;

/**
  * @brief Test status select in test mode.
  * @details Mode type:
  *          + loopBack mode, 1: enabele, rx can receive tx frame ; 0: disable
  *          + silent mode, 1: enabele, cannot send frame to others; 0: disable
  *          + basic mode, 1: enable, IF1 used for tx buffer, IF2 used for rx buffer; 0: disable
  */
typedef struct {
    unsigned int loopBack;
    unsigned int silent;
    unsigned int basic;
} CAN_TestMode_Configure;

/**
  * @brief The type of CAN frame.
  * @details CAN frame type:
  *          + CAN_TYPEFRAME_STD_DATA   -- Standard data frame
  *          + CAN_TYPEFRAME_EXT_DATA   -- Extended data Frame
  *          + CAN_TYPEFRAME_STD_REMOTE -- Standard remote frame
  *          + CAN_TYPEFRAME_EXT_REMOTE -- Extended remote Frame
  */
typedef enum {
    CAN_TYPEFRAME_STD_DATA = 0x00000000U,
    CAN_TYPEFRAME_EXT_DATA = 0x00000001U,
    CAN_TYPEFRAME_STD_REMOTE = 0x00000002U,
    CAN_TYPEFRAME_EXT_REMOTE = 0x00000003U
} CAN_TypeFrame;

/**
  * @brief Type of the received frame after filtering.
  * @details Filtering receive frame type:
  *          + CAN_FILTERFRAME_STD_DATA     -- Standard data frame
  *          + CAN_FILTERFRAME_EXT_DATA     -- Extended data frame
  *          + CAN_FILTERFRAME_STD_EXT_DATA -- Standard remote frame
  */
typedef enum {
    CAN_FILTERFRAME_STD_DATA = 0x00000000U,
    CAN_FILTERFRAME_EXT_DATA = 0x00000001U,
    CAN_FILTERFRAME_STD_EXT_DATA = 0x00000002U
} CAN_FilterFrame;

/**
  * @brief Time quanta of phase buffer section 1 and propagation section.
  */
typedef enum {
    CAN_SEG1_2TQ = 0x00000002U,
    CAN_SEG1_3TQ = 0x00000003U,
    CAN_SEG1_4TQ = 0x00000004U,
    CAN_SEG1_5TQ = 0x00000005U,
    CAN_SEG1_6TQ = 0x00000006U,
    CAN_SEG1_7TQ = 0x00000007U,
    CAN_SEG1_8TQ = 0x00000008U,
    CAN_SEG1_9TQ = 0x00000009U,
    CAN_SEG1_10TQ = 0x0000000AU,
    CAN_SEG1_11TQ = 0x0000000BU,
    CAN_SEG1_12TQ = 0x0000000CU,
    CAN_SEG1_13TQ = 0x0000000DU,
    CAN_SEG1_14TQ = 0x0000000EU,
    CAN_SEG1_15TQ = 0x0000000FU,
    CAN_SEG1_16TQ = 0x00000010U
} CAN_Seg1_Phase;

/**
  * @brief Time quanta of phase buffer section 2.
  */
typedef enum {
    CAN_SEG2_1TQ = 0x00000001U,
    CAN_SEG2_2TQ = 0x00000002U,
    CAN_SEG2_3TQ = 0x00000003U,
    CAN_SEG2_4TQ = 0x00000004U,
    CAN_SEG2_5TQ = 0x00000005U,
    CAN_SEG2_6TQ = 0x00000006U,
    CAN_SEG2_7TQ = 0x00000007U,
    CAN_SEG2_8TQ = 0x00000008U
} CAN_Seg2_Phase;

/**
  * @brief Time quanta of Sync Jump Width.
  */
typedef enum {
    CAN_SJW_1TQ = 0x00000001U,
    CAN_SJW_2TQ = 0x00000002U,
    CAN_SJW_3TQ = 0x00000003U,
    CAN_SJW_4TQ = 0x00000004U
} CAN_Sync_Jump_Width;

/**
  * @brief Error status code: the last error status on the CAN bus.
  */
typedef enum {
    CAN_WRITE_MASK = 0x00000008U,
    CAN_READ_MASK = 0x00000010U,
    CAN_EPASS_MASK = 0x00000020U,
    CAN_EWARN_MASK = 0x00000040U,
    CAN_BOFF_MASK = 0x00000080U,
} CAN_StatusMask;

/**
  * @brief CAN data frame format.
  */
typedef struct {
    CAN_TypeFrame type;
    unsigned int dataLength;
    unsigned int CANId;
    unsigned char frame[8];
} CANFrame;

/**
  * @brief Received frame filtering configuration parameters.
  */
typedef struct {
    CAN_FilterFrame receiveType;
    unsigned int filterID;
    unsigned int filterMask;
} CAN_FilterConfigure;

/**
  * @brief Bit timing parameters.
  */
typedef struct {
    unsigned int Tseg2 : 3;
    unsigned int Tseg1 : 4;
    unsigned int SJW : 2;
    unsigned int BRP : 1;
} Bit_Timing;
/**
  * @}
  */

/**
  * @defgroup CAN_Reg_Def CAN Register Definition
  * @brief CAN register mapping structure.
  * @{
  */

/**
  * @brief CAN control register, Control of basic functions.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int Init : 1;          /**< Initialization enable. */
        unsigned int IE : 1;            /**< Module interrupt enable. */
        unsigned int SIE : 1;           /**< Status change interrupt enable. */
        unsigned int EIE : 1;           /**< Error interrupt enable. */
        unsigned int reserved0 : 1;
        unsigned int DAR : 1;           /**< Automatic retransmission enable. */
        unsigned int CCE : 1;           /**< Configuration change enable. */
        unsigned int Test : 1;          /**< Test mode enable. */
        unsigned int reserved1 : 24;
    } BIT;
} volatile CAN_CONTROL_REG;

/**
  * @brief CAN status register register.CAN error count register
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int LEC : 3;            /**< Error status code, used to indicate last error status on CAN bus. */
        unsigned int TxOk : 1;           /**< Indicates the packet sending status. */
        unsigned int RxOk : 1;           /**< Indicates the packet receiving status. */
        unsigned int Epass : 1;          /**< Indicates the error status. */
        unsigned int Ewarn : 1;          /**< Warning status. */
        unsigned int Boff : 1;           /**< Bus-off status. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile CAN_STATUS_REG;

/**
  * @brief CAN error count register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int TEC : 8;           /**< Indicates transmission error counter. */
        unsigned int REC : 7;           /**< Receive error counter. */
        unsigned int RP : 1;            /**< Indicates passive error reception status. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile CAN_ERROR_COUNTER_REG;

/**
  * @brief bit time register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int BRP : 6;           /**< Baud rate coefficient. */
        unsigned int SJW : 2;           /**< Resync jump width. */
        unsigned int TSeg1 : 4;         /**< Phase buffer segment 1. */
        unsigned int TSeg2 : 3;         /**< Phase buffer segment 2. */
        unsigned int reserved0 : 17;
    } BIT;
} volatile BIT_TIMING_REG;

/**
  * @brief CAN interrupt register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int IntId : 16;       /**< Interrupt packet object ID. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile CAN_INTERRUPT_REG;

/**
  * @brief CAN test register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int : 2;
        unsigned int Basic : 1;        /**< Basic mode enable. */
        unsigned int Silent : 1;       /**< Silent mode enable. */
        unsigned int Lback : 1;        /**< Loop back mode enable. */
        unsigned int Tx : 2;           /**< CAN_TX pin control. */
        unsigned int Rx : 1;           /**< Monitors the CAN_RX pin. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile CAN_TEST_REG;

/**
  * @brief Baud rate coefficient extension register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int BRPE : 4;         /**< Baud rate coefficient expansion. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile BRP_EXTENSION_REG;

/**
  * @brief Request register for IF1 command.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int MessageNumber : 6; /**< Message object serial number. */
        unsigned int reserved0 : 9;
        unsigned int BUSY : 1;          /**< Busy signal. */
        unsigned int reserved1 : 16;
    } BIT;
} volatile IF1_COMMAND_REQUEST_REG;

/**
  * @brief IF1 command mask register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int DataB : 1;         /**< Command mask, which controls the transmission of IF1_DATAB. */
        unsigned int DataA : 1;         /**< Command mask, which controls the transmission of IF1_DATAA. */
        unsigned int TxRqstNewDat : 1;  /**< Command mask, which controls the TxRqst bit or NewDat bit of
                                             the packet object. */
        unsigned int ClrIntPnd : 1;     /**< Command mask, which is used to clear the interrupts of the
                                             packet object to be processed. */
        unsigned int Control : 1;       /**< Command mask, which controls the transmission of IF1_MESSAGE_CONTROL. */
        unsigned int Arb : 1;           /**< Command mask, which controls the transmission of IF1_ARBITRATION. */
        unsigned int Mask : 1;          /**< Command mask, which controls the transmission of IF1_MASK.*/
        unsigned int WRRD : 1;          /**< Read/Write command, which controls the transfer direction
                                             of the IF1 packet buffer register and Message RAM. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile IF1_COMMAND_MASK_REG;

/**
  * @brief IF1 mask register 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int Msk : 16;        /**< Mask of the 15th to 0th bits of the packet object ID
                                          which are used for packet receiving and filtering. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF1_MASK1_REG;

/**
  * @brief IF1 mask register 2.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int Msk : 13;       /**< Mask of the 28th to 16th bits of the packet object ID
                                          which are used to filter received packets. */
        unsigned int reserved0 : 1;
        unsigned int MDir : 1;       /**< Indicates the direction bit mask of the packet object
                                          which is used for filtering received packets. */
        unsigned int MXtd : 1;       /**< Indicates the extended ID (Xtd) mask of the packet object
                                         which is used for filtering received packets. */
        unsigned int reserved1 : 16;
    } BIT;
} volatile IF1_MASK2_REG;

/**
  * @brief IF1 arbitration register 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ID : 16;       /**< Bits 15 to 0 of the packet ID of the packet object. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF1_ARBITRATION1_REG;

/**
  * @brief IF1 arbitration register 2.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ID : 13;        /**< Bits 28 to 16 of packet ID of packet object. */
        unsigned int Dir : 1;        /**< Indicates direction of the packet object. */
        unsigned int Xtd : 1;        /**< Indicates format of received and sent frames of packet object. */
        unsigned int MsgVal : 1;     /**< Packet object validity enable. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF1_ARBITRATION2_REG;

/**
  * @brief IF1 packet control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int DLC : 4;        /**< Data length. */
        unsigned int reserved0 : 3;
        unsigned int EoB : 1;        /**< Indicates the multi-packet receiving mode. */
        unsigned int TxRqst : 1;     /**< Transfer request. */
        unsigned int RmtEn : 1;      /**< Remote frame enable. */
        unsigned int RxIE : 1;       /**< RX interrupt enable. */
        unsigned int TxIE : 1;       /**< TX interrupt enable. */
        unsigned int Umask : 1;      /**< Indicates whether the packet object uses the packet mask
                                          which is used for packet receiving and filtering. */
        unsigned int IntPnd : 1;     /**< Indicates the interrupt to be processed of the packet object. */
        unsigned int MsgLst : 1;     /**< Indicates the packet loss flag of the packet object
                                          parameter is valid only when packet object is in the receive direction. */
        unsigned int NewDat : 1;     /**< Write status of the new data of the message object. */
        unsigned int reserved1 : 16;
    } BIT;
} volatile IF1_MESSAGE_CONTROL_REG;

/**
  * @brief IF1 data A1 register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int DATA0 : 8;    /**< CAN frame data byte 0. */
        unsigned int DATA1 : 8;    /**< CAN frame data byte 1. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF1_DATAA1_REG;

/**
  * @brief IF1 data A2 register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int DATA2 : 8;   /**< CAN frame data byte 2. */
        unsigned int DATA3 : 8;   /**< CAN frame data byte 3. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF1_DATAA2_REG;

/**
  * @brief IF1 data B1 register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int DATA4 : 8;   /**< CAN frame data byte 4. */
        unsigned int DATA5 : 8;   /**< CAN frame data byte 5. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF1_DATAB1_REG;

/**
  * @brief IF1 data B2 register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int DATA6 : 8;  /**< CAN frame data byte 6. */
        unsigned int DATA7 : 8;  /**< CAN frame data byte 7. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF1_DATAB2_REG;

/**
  * @brief IF2 command request register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int MessageNumber : 6;    /**< Indicates the sequence number of a packet object. */
        unsigned int reserved0 : 9;
        unsigned int BUSY : 1;             /**< Busy sign. */
        unsigned int reserved1 : 16;
    } BIT;
} volatile IF2_COMMAND_REQUEST_REG;

/**
  * @brief IF2 command mask register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int DataB : 1;       /**< Command mask, which controls the transmission of IF2_DATAB. */
        unsigned int DataA : 1;       /**< Command mask, which controls the transmission of IF2_DATAA. */
        unsigned int TxRqstNewDat : 1;/**< Command mask, which controls TxRqst bit or NewDat bit of packet object. */
        unsigned int ClrIntPnd : 1;   /**< Command mask, which is used to clear interrupts of packet
                                           object to be processed. */
        unsigned int Control : 1;     /**< Command mask, which controls the transmission of IF2_MESSAGE_CONTROL. */
        unsigned int Arb : 1;         /**< Command mask, which controls the transmission of IF2_ARBITRATION. */
        unsigned int Mask : 1;        /**< Command mask, which controls the transmission of IF2_MASK. */
        unsigned int WRRD : 1;        /**< Read/Write command, which controls the transfer direction of
                                          the IF2 packet buffer register and Message RAM. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile IF2_COMMAND_MASK_REG;

/**
  * @brief IF2 mask register 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int Msk : 16;    /**< Mask of the 15th to 0th bits of the packet object ID
                                       which are used for packet receiving and filtering. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF2_MASK1_REG;

/**
  * @brief IF2 mask register 2.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int Msk : 13;       /**< Mask of the 28th to 16th bits of the packet object ID
                                          which are used for packet receiving and filtering. */
        unsigned int reserved0 : 1;
        unsigned int MDir : 1;       /**< Indicates the direction bit mask of the packet object
                                          which is used for packet receiving and filtering. */
        unsigned int MXtd : 1;       /**< Extended ID mask (Xtd) of a packet object
                                          which is used for packet receiving and filtering. */
        unsigned int reserved1 : 16;
    } BIT;
} volatile IF2_MASK2_REG;

/**
  * @brief IF2 arbitration register 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ID : 16;      /**< Bits 15 to 0 of the packet ID of the packet object. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF2_ARBITRATION1_REG;

/**
  * @brief IF2 arbitration register 2.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ID : 13;     /**< Bits 28 to 16 of the packet ID of the packet object.*/
        unsigned int Dir : 1;     /**< Indicates the direction of the packet object.*/
        unsigned int Xtd : 1;     /**< Indicates format of received and sent frames of packet object.*/
        unsigned int MsgVal : 1;  /**< Packet object validity enable.*/
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF2_ARBITRATION2_REG;

/**
  * @brief IF2 packet control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int DLC : 4;         /**< Data length */
        unsigned int reserved0 : 3;
        unsigned int EoB : 1;         /**< Indicates the multi-packet receiving mode. */
        unsigned int TxRqst : 1;      /**< Transfer request. */
        unsigned int RmtEn : 1;       /**< Remote frame enable. */
        unsigned int RxIE : 1;        /**< RX interrupt enable. */
        unsigned int TxIE : 1;        /**< TX interrupt enable. */
        unsigned int Umask : 1;       /**< Indicates whether the packet object uses the packet mask
                                           which is used for packet receiving and filtering. */
        unsigned int IntPnd : 1;      /**< Indicates the interrupt to be processed of the packet object. */
        unsigned int MsgLst : 1;      /**< Indicates the packet loss flag of the packet object
                                           This parameter is valid only when packet object is in receive direction. */
        unsigned int NewDat : 1;      /**< Indicates the frame data ID of the packet object. */
        unsigned int reserved1 : 16;
    } BIT;
} volatile IF2_MESSAGE_CONTROL_REG;

/**
  * @brief IF2 data A1 register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int DATA0 : 8;     /**< CAN frame data byte 0. */
        unsigned int DATA1 : 8;     /**< CAN frame data byte 1. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF2_DATAA1_REG;

/**
  * @brief IF2 data A2 register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int DATA2 : 8;    /**< CAN frame data byte 2. */
        unsigned int DATA3 : 8;    /**< CAN frame data byte 3. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF2_DATAA2_REG;

/**
  * @brief IF2 data B1 register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int DATA4 : 8;  /**< CAN frame data byte 4. */
        unsigned int DATA5 : 8;  /**< CAN frame data byte 5. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF2_DATAB1_REG;

/**
  * @brief IF2 data B2 register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int DATA6 : 8;   /**< CAN frame data byte 6. */
        unsigned int DATA7 : 8;   /**< CAN frame data byte 7. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile IF2_DATAB2_REG;

/**
  * @brief Transfer request status register 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int TxRqst16_1 : 16;   /**< Transfer request status.
                                             Each bit of TxRqst16-1 corresponds to packet object 16-1. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile TRANSMISSION_REQUEST1_REG;

/**
  * @brief Transfer request status register 2.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int TxRqst32_17 : 16;  /**< Transfer request status.
                                             Each bit of TxRqst32-17 corresponds to packet objects 32-17. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile TRANSMISSION_REQUEST2_REG;

/**
  * @brief New data status register 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int NewDat16_1 : 16;  /**< New data write status.
                                            NewDat16-1 Each bit corresponds to the packet object 16-1. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile NEW_DATA1_REG;

/**
  * @brief New data status register 2.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int NewDat32_17 : 16; /**< New data write status.
                                            Each bit of NewDat32-17 corresponds to packet objects 32-17. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile NEW_DATA2_REG;

/**
  * @brief Interrupt pending status register 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int IntPnd16_1 : 16;  /**< Interrupt Pending Status.
                                            Each bit of IntPnd16-1 corresponds to packet object 16-1. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile INTERRUPT_PENDING1_REG;

/**
  * @brief Interrupt pending status register 2.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int IntPnd32_17 : 16; /**< Interrupt Pending Status.
                                            Each bit of IntPnd32-17 corresponds to packet objects 32-17. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile INTERRUPT_PENDING2_REG;

/**
  * @brief Packet validity status register 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int MsgVal16_1 : 16;  /**< Indicates the validity status of the packet object
                                            Each bit of MsgVal16-1 corresponds to packet object 16-1. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile MESSAGE_VALID1_REG;

/**
  * @brief Packet validity status register 2.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int MsgVal32_17 : 16;  /**< Indicates the validity status of the packet object.
                                            Each bit of MsgVal32-17 corresponds to packet objects 32-17. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile MESSAGE_VALID2_REG;

/**
  * @brief Register mapping structure.
  */
typedef struct _CAN_RegStruct {
    CAN_CONTROL_REG             CAN_CONTROL;         /**< CAN control register, Offset address: 0x00000000U. */
    CAN_STATUS_REG              CAN_STATUS;          /**< CAN status register. Offset address: 0x00000004U.*/
    CAN_ERROR_COUNTER_REG       CAN_ERROR_COUNTER;   /**< CAN error count register. Offset address: 0x00000008U.*/
    BIT_TIMING_REG              BIT_TIMING;          /**< Bit time register. Offset address: 0x0000000CU. */
    CAN_INTERRUPT_REG           CAN_INTERRUPT;       /**< CAN interrupt register.  Offset address: 0x00000010U. */
    CAN_TEST_REG                CAN_TEST;            /**< CAN debug register. Offset address: 0x00000014U. */
    BRP_EXTENSION_REG           BRP_EXTENSION;       /**< BRP extension register.  Offset address: 0x00000018U. */
    char space0[4];
    IF1_COMMAND_REQUEST_REG     IF1_COMMAND_REQUEST; /**< IF1 command request register. Offset address: 0x00000020U. */
    IF1_COMMAND_MASK_REG        IF1_COMMAND_MASK;    /**< IF1 command mask register. Offset address: 0x00000024U. */
    IF1_MASK1_REG               IF1_MASK1;           /**< IF1 mask register 1. Offset address: 0x00000028U. */
    IF1_MASK2_REG               IF1_MASK2;           /**< IF1 mask register 2. Offset address: 0x0000002CU. */
    IF1_ARBITRATION1_REG        IF1_ARBITRATION1;    /**< IF1 arbitration register 1. Offset address: 0x00000030U. */
    IF1_ARBITRATION2_REG        IF1_ARBITRATION2;    /**< IF1 arbitration register 2. Offset address: 0x00000034U. */
    IF1_MESSAGE_CONTROL_REG     IF1_MESSAGE_CONTROL; /**< IF1 packet control register. Offset address: 0x00000038U. */
    IF1_DATAA1_REG              IF1_DATAA1;          /**< IF1 data A1 register. Offset address: 0x0000003CU. */
    IF1_DATAA2_REG              IF1_DATAA2;          /**< IF1 data A2 register. Offset address: 0x00000040U. */
    IF1_DATAB1_REG              IF1_DATAB1;          /**< IF1 data B1 register.  Offset address: 0x00000044U. */
    IF1_DATAB2_REG              IF1_DATAB2;          /**< IF1 data B2 register. Offset address: 0x00000048U. */
    char space1[52];
    IF2_COMMAND_REQUEST_REG     IF2_COMMAND_REQUEST; /**< IF2 command request register. Offset address: 0x00000080U. */
    IF2_COMMAND_MASK_REG        IF2_COMMAND_MASK;    /**< IF2 command mask register. Offset address: 0x00000084U. */
    IF2_MASK1_REG               IF2_MASK1;           /**< IF2 mask register 1. Offset address: 0x00000088U. */
    IF2_MASK2_REG               IF2_MASK2;           /**< IF2 mask register 2. Offset address: 0x0000008CU. */
    IF2_ARBITRATION1_REG        IF2_ARBITRATION1;    /**< IF2 arbitration register 1. Offset address: 0x00000090U. */
    IF2_ARBITRATION2_REG        IF2_ARBITRATION2;    /**< IF2 arbitration register 2. Offset address: 0x00000094U. */
    IF2_MESSAGE_CONTROL_REG     IF2_MESSAGE_CONTROL; /**< IF2 packet control register. Offset address: 0x00000098U.*/
    IF2_DATAA1_REG              IF2_DATAA1;          /**< IF2 data A1 register. Offset address: 0x0000009CU. */
    IF2_DATAA2_REG              IF2_DATAA2;          /**< IF2 data A2 register. Offset address: 0x000000A0U. */
    IF2_DATAB1_REG              IF2_DATAB1;          /**< IF2 data B1 register. Offset address: 0x000000A4U. */
    IF2_DATAB2_REG              IF2_DATAB2;          /**< IF2 data B2 register. Offset address: 0x000000A8U. */
    char space2[84];
    TRANSMISSION_REQUEST1_REG   TRANSMISSION_REQUEST1;/**< Trans_request status reg 1. Offset address: 0x00000100U. */
    TRANSMISSION_REQUEST2_REG   TRANSMISSION_REQUEST2;/**< Trans_request status reg 2. Offset address: 0x00000104U. */
    char space3[24];
    NEW_DATA1_REG               NEW_DATA1;            /**< New data status register 1. Offset address: 0x00000120U. */
    NEW_DATA2_REG               NEW_DATA2;            /**< New data status register 2. Offset address: 0x00000124U. */
    char space4[24];
    INTERRUPT_PENDING1_REG      INTERRUPT_PENDING1;   /**< INT pending status reg 1. Offset address: 0x00000140U. */
    INTERRUPT_PENDING2_REG      INTERRUPT_PENDING2;   /**< INT pending status reg 2. Offset address: 0x00000144U. */
    char space5[24];
    MESSAGE_VALID1_REG          MESSAGE_VALID1;     /**< Packet validity status reg 1. Offset address: 0x00000160U. */
    MESSAGE_VALID2_REG          MESSAGE_VALID2;     /**< Packet validity status reg 2. Offset address: 0x00000164U.*/
} volatile CAN_RegStruct;
/**
  * @}
  */


/**
  * @brief Check CAN typemode parameter.
  * @param typemode Work mode, @ref CAN_TypeMode
  * @retval bool
  */
static inline bool IsCanMode(CAN_TypeMode typemode)
{
    return (typemode == CAN_MODE_NORMAL) || (typemode == CAN_MODE_TEST);
}

/**
  * @brief Check CAN prescalser parameter.
  * @param prescalser Bit timing prescalser.
  * @retval bool
  */
static inline bool IsCanPrescalser(unsigned int prescalser)
{
    return prescalser >= PRESCALSER_MIN && prescalser <= PRESCALSER_MAX;
}

/**
  * @brief Check CAN seg1Phase parameter.
  * @param seg1Phase Phase buffer section 1, @ref CAN_Seg1_Phase
  * @retval bool
  */
static inline bool IsCanSeg1phase(CAN_Seg1_Phase seg1Phase)
{
    return (seg1Phase >= CAN_SEG1_2TQ) && (seg1Phase <= CAN_SEG1_16TQ);
}

/**
  * @brief Check CAN seg2Phase parameter.
  * @param seg2Phase Phase buffer section 2, @ref CAN_Seg2_Phase
  * @retval bool
  */
static inline bool IsCanSeg2phase(CAN_Seg2_Phase seg2Phase)
{
    return (seg2Phase >= CAN_SEG2_1TQ) && (seg2Phase <= CAN_SEG2_8TQ);
}

/**
  * @brief Check CAN syncJumpWidth parameter.
  * @param syncJumpWidth Sync jump width, @ref CAN_Sync_Jump_Width
  * @retval bool
  */
static inline bool IsCanSJW(CAN_Sync_Jump_Width syncJumpWidth)
{
    return (syncJumpWidth >= CAN_SJW_1TQ) && (syncJumpWidth <= CAN_SJW_4TQ);
}

/* Direct configuration layer */
/**
  * @brief CAN bit timing setting.
  * @param canx CAN register base address.
  * @param bitSetting CAN bit timing parameter, @ref Bit_Timing
  * @retval None.
  */
static inline void DCL_CAN_BitSetting(CAN_RegStruct * const canx, Bit_Timing bitSetting)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    CAN_PARAM_CHECK_NO_RET(bitSetting.Tseg1 > 0);
    canx->CAN_CONTROL.reg |= 0x00000041U;   /* Bit_Timing setting, [0] and [6] bit need are set, others clear */
    unsigned int val = bitSetting.BRP;      /* The prescalser is set to the lower 6 bits, [5:0] */
    val |= (bitSetting.SJW) << 6;           /* The sjw needs to be shifted leftwards by 6 bits, range : 0~3 */
    val |= (bitSetting.Tseg1) << 8;         /* The seg1Phase needs to be shifted leftwards by 8 bits, range : 1~15 */
    val |= (bitSetting.Tseg2) << 12;        /* The seg2Phase needs to be shifted leftwards by 12 bits, range : 0~63 */
    canx->BIT_TIMING.reg = val;
}

/**
  * @brief CAN interrupt enable.
  * @param canx CAN register base address.
  * @retval None.
  */
static inline void DCL_CAN_EnableInterrupt(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.IE = BASE_CFG_ENABLE;
}

/**
  * @brief CAN interrupt disable.
  * @param canx CAN register base address.
  * @retval None.
  */
static inline void DCL_CAN_DisableInterrupt(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.IE = BASE_CFG_DISABLE;
}

/**
  * @brief CAN status interrupt enable.
  * @param canx CAN register base address.
  * @retval None.
  */
static inline void DCL_CAN_EnableStatusInterrupt(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.SIE = BASE_CFG_ENABLE;
}

/**
  * @brief CAN status interrupt disable.
  * @param canx CAN register base address.
  * @retval None.
  */
static inline void DCL_CAN_DisableStatusInterrupt(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.SIE = BASE_CFG_DISABLE;
}

/**
  * @brief CAN error interrupt enable.
  * @param canx CAN register base address.
  * @retval None.
  */
static inline void DCL_CAN_EnableErrorInterrupt(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.EIE = BASE_CFG_ENABLE;
}

/**
  * @brief CAN error interrupt disable.
  * @param canx CAN register base address.
  * @retval None.
  */
static inline void DCL_CAN_DisableErrorInterrupt(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.EIE = BASE_CFG_DISABLE;
}

/**
  * @brief Enable Automatic Retransmission
  * @param canx CAN register base address.
  * @retval None
  */
static inline void DCL_CAN_EnableAutoRetrans(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.DAR = BASE_CFG_DISABLE;
}

/**
  * @brief Disable Automatic Retransmission
  * @param canx CAN register base address.
  * @retval None
  */
static inline void DCL_CAN_DisableAutoRetrans(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.DAR = BASE_CFG_ENABLE;
}

/**
  * @brief Enable CAN test mode
  * @param canx CAN register base address.
  * @retval None
  */
static inline void DCL_CAN_EnableTestMode(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.Test = BASE_CFG_ENABLE;
}

/**
  * @brief Disable CAN test mode
  * @param canx CAN register base address.
  * @retval None
  */
static inline void DCL_CAN_DisableTestMode(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.Test = BASE_CFG_DISABLE;
}

/**
  * @brief Enable CAN bit timing config
  * @param canx CAN register base address.
  * @retval None
  */
static inline void DCL_CAN_EnableBitTimingConfig(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.CCE = BASE_CFG_ENABLE;
}

/**
  * @brief Disable CAN bit timing config
  * @param canx CAN register base address.
  * @retval None
  */
static inline void DCL_CAN_DisableBitTimingConfig(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.CCE = BASE_CFG_DISABLE;
}

/**
  * @brief Enable CAN Init.
  * @param canx CAN register base address.
  * @retval None
  */
static inline void DCL_CAN_EnableInit(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.Init = BASE_CFG_ENABLE;
}

/**
  * @brief Disable CAN init.
  * @param canx CAN register base address.
  * @retval None
  */
static inline void DCL_CAN_DisableInit(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_CONTROL.BIT.Init = BASE_CFG_DISABLE;
}

/**
  * @brief Initializes a specified packet object.
  * @param canx CAN register base address.
  * @param objID ID of message object.
  * @retval None.
  */
static inline void DCL_CAN_InitObj(CAN_RegStruct * const canx, unsigned int objID)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    CAN_PARAM_CHECK_NO_RET((objID >= MESSAGE_NUMBER_MIN) && (objID <= MESSAGE_NUMBER_MAX));
    unsigned int busy;
    canx->IF1_COMMAND_REQUEST.reg = objID;
    do {
        busy = canx->IF1_COMMAND_REQUEST.BIT.BUSY;
    } while (busy == 0x00000001U);
    return;
}

/**
  * @brief Get IF1 CAN status
  * @param canx CAN register base address.
  * @retval bool: 0 command is being executed, 1 command has been completed.
  */
static inline bool DCL_CAN_GetIF1Status(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF1_COMMAND_REQUEST.BIT.BUSY;
}

/**
  * @brief  Setting IF1 message number
  * @param canx CAN register base address.
  * @param objID message number
  * @retval None
  */
static inline void DCL_CAN_SetIF1MessageNumber(CAN_RegStruct * const canx, unsigned int objID)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    CAN_PARAM_CHECK_NO_RET((objID >= MESSAGE_NUMBER_MIN) && (objID <= MESSAGE_NUMBER_MAX));
    canx->IF1_COMMAND_REQUEST.BIT.MessageNumber = objID;
}

/**
  * @brief Query the CAN interrupt generation source.
  * @param canx CAN register base address.
  * @retval IDs of the packet objects for which the interrupt is generated.
  */
static inline unsigned int DCL_CAN_GetInterruptID(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->CAN_INTERRUPT.reg;
}

/**
  * @brief Confrguration command mask
  * @param canx CAN register base address.
  * @param maskValue Mask value for command register.
  * @retval None
  */
static inline void DCL_CAN_ConfigMaskValue(CAN_RegStruct * const canx, unsigned int maskValue)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF1_COMMAND_MASK.reg = maskValue;
}

/**
  * @brief Get IF2 CAN status.
  * @param canx CAN register base address.
  * @retval bool: 0 command is being executed, 1 command has been completed.
  */
static inline bool DCL_CAN_GetIF2Status(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF2_COMMAND_REQUEST.BIT.BUSY;
}

/**
  * @brief  Setting IF2 message number.
  * @param canx CAN register base address.
  * @param objID message number.
  * @retval None
  */
static inline void DCL_CAN_SetIF2MessageNumber(CAN_RegStruct * const canx, unsigned int objID)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    CAN_PARAM_CHECK_NO_RET((objID >= MESSAGE_NUMBER_MIN) && (objID <= MESSAGE_NUMBER_MAX));
    canx->IF2_COMMAND_REQUEST.BIT.MessageNumber = objID;
}

/**
  * @brief  Enable CAN loop back mode.
  * @param canx CAN register base address.
  * @retval None.
  */
static inline void DCL_CAN_EnableLoopBack(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_TEST.BIT.Lback = BASE_CFG_ENABLE;
}

/**
  * @brief  Disable CAN loop back mode.
  * @param canx CAN register base address.
  * @retval None.
  */
static inline void DCL_CAN_DisableLoopBack(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_TEST.BIT.Lback = BASE_CFG_DISABLE;
}

/**
  * @brief  Enable CAN silent mode.
  * @param canx CAN register base address.
  * @retval None.
  */
static inline void DCL_CAN_EnableSilent(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_TEST.BIT.Silent = BASE_CFG_ENABLE;
}

/**
  * @brief  Disable CAN silent mode.
  * @param canx CAN register base address.
  * @retval None.
  */
static inline void DCL_CAN_DisableSilent(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_TEST.BIT.Silent = BASE_CFG_DISABLE;
}

/**
  * @brief  Enable CAN basic mode.
  * @param canx CAN register base address.
  * @retval None.
  */
static inline void DCL_CAN_EnableBasic(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_TEST.BIT.Basic = BASE_CFG_ENABLE;
}

/**
  * @brief Disable CAN basic mode.
  * @param canx CAN register base address.
  * @retval None.
  */
static inline void DCL_CAN_DisableBasic(CAN_RegStruct * const canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->CAN_TEST.BIT.Basic = BASE_CFG_DISABLE;
}

/**
  * @brief Config IF1_ARBITRATION1.
  * @param canx CAN register base address.
  * @param maskValue Mask value for arbitration 1 register.
  * @retval None.
  */
static inline void DCL_CAN_ConfigIF1ARBITRATION1(CAN_RegStruct * const canx, unsigned int maskValue)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF1_ARBITRATION1.reg = maskValue;
}

/**
  * @brief Low bit(0-15) obj ID number using IF1.
  * @param canx CAN register base address.
  * @retval unsigned int: Low bit ID number.
  */
static inline unsigned int DCL_CAN_GetIF1LoWBitObjNumber(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF1_ARBITRATION1.BIT.ID;
}

/**
  * @brief  Config IF1_ARBITRATION2.
  * @param canx CAN register base address.
  * @param maskValue Mask value for arbitration 2 register.
  * @retval None.
  */
static inline void DCL_CAN_ConfigIF1ARBITRATION2(CAN_RegStruct * const canx, unsigned int maskValue)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF1_ARBITRATION2.reg = maskValue;
}

/**
  * @brief Config IF2_ARBITRATION1.
  * @param canx CAN register base address.
  * @param maskValue Mask value for arbitration 1 register.
  * @retval None.
  */
static inline void DCL_CAN_ConfigIF2ARBITRATION1(CAN_RegStruct * const canx, unsigned int maskValue)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF2_ARBITRATION1.reg = maskValue;
}

/**
  * @brief Low bit(0-15) obj ID number using IF2.
  * @param canx CAN register base address.
  * @retval unsigned int: Low bit ID number.
  */
static inline unsigned int DCL_CAN_GetIF2LoWBitObjNumber(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF2_ARBITRATION1.BIT.ID;
}

/**
  * @brief Config IF2_ARBITRATION2.
  * @param canx CAN register base address.
  * @param maskValue Mask value for arbitration 2 register.
  * @retval None
  */
static inline void DCL_CAN_ConfigIF2ARBITRATION2(CAN_RegStruct * const canx, unsigned int maskValue)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF2_ARBITRATION2.reg = maskValue;
}

/**
  * @brief Get objection format of transmitted and received frame of IF2.
  * @param canx CAN register base address.
  * @retval bool: 0: Message object receives and transmits frames in standard format.
  * @retval       1: Message object receives and transmits frames in extended format.
  */
static inline bool DCL_CAN_GetIF2ObjFormat(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF2_ARBITRATION2.BIT.Xtd;
}

/**
  * @brief Get objection format of transmitted and received frame of IF1.
  * @param canx CAN register base address.
  * @retval bool: 0: Message object receives and transmits frames in standard format.
  * @retval       1: Message object receives and transmits frames in extended format.
  */
static inline bool DCL_CAN_GetIF1ObjFormat(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF1_ARBITRATION2.BIT.Xtd;
}

/**
  * @brief Get direction of transmitted and received frame of IF1.
  * @param canx CAN register base address.
  * @retval bool: 0: Message object is received in receive direction.
  * @retval       1: Message object is in transmit direction.
  */
static inline bool DCL_CAN_GetIF1ObjDirection(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF1_ARBITRATION2.BIT.Dir;
}

/**
  * @brief Get the status of whether objection is valid using IF1.
  * @param canx CAN register base address.
  * @retval bool: 1 message object valid, 0 message object invalid.
  */
static inline bool DCL_CAN_GetIF1ObjStatus(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF1_ARBITRATION2.BIT.MsgVal;
}

/**
  * @brief high bit(16-28) obj ID number using IF1.
  * @param canx CAN register base address.
  * @retval unsigned int: high bit ID number.
  */
static inline unsigned int DCL_CAN_GetIF1HighBitObjNumber(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF1_ARBITRATION2.BIT.ID;
}

/**
  * @brief Get direction of transmitted and received frame of IF2.
  * @param canx CAN register base address.
  * @retval bool: 0: Message object is received in receive direction.
  * @retval       1: Message object is in transmit direction.
  */
static inline bool DCL_CAN_GetIF2ObjDirection(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF2_ARBITRATION2.BIT.Dir;
}

/**
  * @brief Get the status of whether objection is valid using IF2.
  * @param canx CAN register base address.
  * @retval bool: 1: message object valid.
  * @retval       0: message object invalid.
  */
static inline bool DCL_CAN_GetIF2ObjStatus(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF2_ARBITRATION2.BIT.MsgVal;
}

/**
  * @brief high bit(16-28) obj ID number using IF2
  * @param canx CAN register base address.
  * @retval unsigned int: high bit ID number
  */
static inline unsigned int DCL_CAN_GetIF2HighBitObjNumber(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF2_ARBITRATION2.BIT.ID;
}

/**
  * @brief Get obj data length using IF2.
  * @param canx CAN register base address.
  * @retval unsigned int: data length
  */
static inline unsigned int DCL_CAN_GetIF2ObjDataLength(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF2_MESSAGE_CONTROL.BIT.DLC;
}

/**
  * @brief config message control using IF2
  * @param canx CAN register base address.
  * @param messageControlValue Message control command value.
  * @retval None
  */
static inline void DCL_CAN_ConfigIF2MessageControl(CAN_RegStruct * const canx, unsigned int messageControlValue)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF2_MESSAGE_CONTROL.reg = messageControlValue;
}

/**
  * @brief Get obj data length using IF1
  * @param canx CAN register base address.
  * @retval unsigned int: message object data length
  */
static inline unsigned int DCL_CAN_GetIF1ObjDataLength(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF1_MESSAGE_CONTROL.BIT.DLC;
}

/**
  * @brief config message control using IF1
  * @param canx CAN register base address.
  * @param messageControlValue Message control command value.
  * @retval None
  */
static inline void DCL_CAN_ConfigIF1MessageControl(CAN_RegStruct * const canx, unsigned int messageControlValue)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF1_MESSAGE_CONTROL.reg = messageControlValue;
}

/**
  * @brief Get A1 data using IF2
  * @param canx CAN register base address.
  * @retval byte 1 and byte 2 of data
  */
static inline unsigned int DCL_CAN_GetIF2ObjDataA1(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF2_DATAA1.reg;
}

/**
  * @brief Set A1 data using IF2
  * @param canx CAN register base address.
  * @param dataA1 Data of A1.
  * @retval None.
  */
static inline void DCL_CAN_SetIF2ObjDataA1(CAN_RegStruct * const canx, unsigned int dataA1)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF2_DATAA1.reg = dataA1;
}

/**
  * @brief Get A2 data using IF2
  * @param canx CAN register base address.
  * @retval byte 3 and byte 4 of data
  */
static inline unsigned int DCL_CAN_GetIF2ObjDataA2(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF2_DATAA2.reg;
}

/**
  * @brief Set A2 data using IF2
  * @param canx CAN register base address.
  * @param dataA2 Data of A2.
  * @retval None.
  */
static inline void DCL_CAN_SetIF2ObjDataA2(CAN_RegStruct * const canx, unsigned int dataA2)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF2_DATAA2.reg = dataA2;
}

/**
  * @brief Get B1 data using IF2
  * @param canx CAN register base address.
  * @retval byte 5 and byte 6 of data
  */
static inline unsigned int DCL_CAN_GetIF2ObjDataB1(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF2_DATAB1.reg;
}

/**
  * @brief Set B1 data using IF2, set byte 5 and byte 6 of data.
  * @param canx CAN register base address.
  * @param dataB1 Data of B1.
  * @retval None.
  */
static inline void DCL_CAN_SetIF2ObjDataB1(CAN_RegStruct * const canx, unsigned int dataB1)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF2_DATAB1.reg = dataB1;
}

/**
  * @brief Get B2 data using IF2
  * @param canx CAN register base address.
  * @retval byte 7 and byte 8 of data
  */
static inline unsigned int DCL_CAN_GetIF2ObjDataB2(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF2_DATAB2.reg;
}

/**
  * @brief Set B2 data using IF2, set byte 7 and byte 8 of data
  * @param canx CAN register base address.
  * @param dataB2 Data of B2.
  * @retval None.
  */
static inline void DCL_CAN_SetIF2ObjDataB2(CAN_RegStruct * const canx, unsigned int dataB2)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF2_DATAB2.reg = dataB2;
}

/**
  * @brief Get A1 data using IF1.
  * @param canx CAN register base address.
  * @retval byte 1 and byte 2 of data.
  */
static inline unsigned int DCL_CAN_GetIF1ObjDataA1(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF1_DATAA1.reg;
}

/**
  * @brief Set A1 data using IF1, Set byte 1 and byte 2 of data
  * @param canx CAN register base address.
  * @param dataA1 Data of A1.
  * @retval None.
  */
static inline void DCL_CAN_SetIF1ObjDataA1(CAN_RegStruct * const canx, unsigned int dataA1)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF1_DATAA1.reg = dataA1;
}

/**
  * @brief Get A2 data using IF1
  * @param canx CAN register base address.
  * @retval byte 3 and byte 4 of data
  */
static inline unsigned int DCL_CAN_GetIF1ObjDataA2(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF1_DATAA2.reg;
}

/**
  * @brief Set A2 data using IF1, Set byte 3 and byte 4 of data
  * @param canx CAN register base address.
  * @param dataA2 Data of A2.
  * @retval None.
  */
static inline void DCL_CAN_SetIF1ObjDataA2(CAN_RegStruct * const canx, unsigned int dataA2)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF1_DATAA2.reg = dataA2;
}

/**
  * @brief Get B1 data using IF1
  * @param canx CAN register base address.
  * @retval byte 5 and byte 6 of data
  */
static inline unsigned int DCL_CAN_GetIF1ObjDataB1(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF1_DATAB1.reg;
}

/**
  * @brief Set B1 data using IF1, Set byte 5 and byte 6 of data
  * @param canx CAN register base address.
  * @param dataB1 Data of B1.
  * @retval None.
  */
static inline void DCL_CAN_SetIF1ObjDataB1(CAN_RegStruct * const canx, unsigned int dataB1)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF1_DATAB1.reg = dataB1;
}

/**
  * @brief Get B2 data using IF1
  * @param canx CAN register base address.
  * @retval byte 7 and byte 8 of data
  */
static inline unsigned int DCL_CAN_GetIF1ObjDataB2(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->IF1_DATAB2.reg;
}

/**
  * @brief Set B2 data using IF1, Set byte 7 and byte 8 of data
  * @param canx CAN register base address.
  * @param dataB2 Data of B2.
  * @retval None.
  */
static inline void DCL_CAN_SetIF1ObjDataB2(CAN_RegStruct * const canx, unsigned int dataB2)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF1_DATAB2.reg = dataB2;
}

/**
  * @brief Set IF2 mask
  * @param canx CAN register base address.
  * @param mask Mask value
  * @retval None
  */
static inline void DCL_CAN_SetIF2Mask(CAN_RegStruct * const canx, unsigned int mask)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF2_MASK2.reg = mask;
}

/**
  * @brief Set objection mask using IF2
  * @param canx CAN register base address.
  * @param maskChg Mask value.
  * @retval None
  */
static inline void DCL_CAN_SetIF2ObjFilterMask(CAN_RegStruct * const canx, unsigned int maskChg)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF2_MASK1.reg = maskChg;
}

/**
  * @brief Set IF1 mask
  * @param canx CAN register base address.
  * @param mask Mask value.
  * @retval None
  */
static inline void DCL_CAN_SetIF1Mask(CAN_RegStruct * const canx, unsigned int mask)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF1_MASK2.reg = mask;
}

/**
  * @brief Set objection mask using IF1
  * @param canx CAN register base address.
  * @param maskChg Mask value.
  * @retval None
  */
static inline void DCL_CAN_SetIF1ObjFilterMask(CAN_RegStruct * const canx, unsigned int maskChg)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF1_MASK1.reg = maskChg;
}

/**
  * @brief Set command mask using IF2.return
  * @param canx CAN register base address.
  * @param commandMask command mask value.
  * @retval None
  */
static inline void DCL_CAN_SetIF2CommandMask(CAN_RegStruct * const canx, unsigned int commandMask)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    canx->IF2_COMMAND_MASK.reg = commandMask;
}

/**
  * @brief Get Can Status
  * @param canx CAN register base address.
  * @retval Overall status of the CAN.
  */
static inline unsigned int DCL_CAN_GetStatus(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->CAN_STATUS.reg;
}

/**
  * @brief Get object interrupt ID.
  * @param canx CAN register base address.
  * @retval Interrupt status of obj 1 to 16
  */
static inline unsigned int DCL_CAN_GetInterruptPend1(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->INTERRUPT_PENDING1.reg;
}

/**
  * @brief Get objects interrupt ID.
  * @param canx CAN register base address.
  * @retval Interrupt status of obj 17 to 32
  */
static inline unsigned int DCL_CAN_GetInterruptPend2(const CAN_RegStruct *canx)
{
    CAN_ASSERT_PARAM(IsCANInstance(canx));
    return canx->INTERRUPT_PENDING2.reg;
}

/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_CAN_IP_H */