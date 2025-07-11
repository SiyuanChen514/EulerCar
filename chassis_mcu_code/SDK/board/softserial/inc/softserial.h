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
  * @file      softserial.h
  * @author    MCU Application Team
  * @brief     Include the header file of the softserial.c file.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_SOFTSERIAL_H
#define McuMagicTag_SOFTSERIAL_H

/* Includes ------------------------------------------------------------------ */
#include "baseinc.h"
#include "timer.h"
#include "gpio.h"

#ifdef SOFTSERIAL_PARAM_CHECK
#define SOFTSERIAL_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define SOFTSERIAL_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define SOFTSERIAL_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define SOFTSERIAL_ASSERT_PARAM(para) ((void)0U)
#define SOFTSERIAL_PARAM_CHECK_NO_RET(para) ((void)0U)
#define SOFTSERIAL_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

/* Macro definitions --------------------------------------------------------- */
#define BOARD_SOFTSERIAL_TX_BUFFER_SIZE 64
#define BOARD_SOFTSERIAL_RX_BUFFER_SIZE 64
#define BOARD_SOFTSERIAL_RX_SAMPLING_CNT_PER_BIT 3

/* Typedef definitions ------------------------------------------------------- */
/**
  * @brief Return Type Definition.
  */
typedef enum {
    BOARD_SOFTSERIAL_OK = 0x00000000U,
    BOARD_SOFTSERIAL_ERR_TX_BUFFER_FULL = 0x00000001U,
    BOARD_SOFTSERIAL_ERR_RX_BUFFER_NULL = 0x00000002U,
    BOARD_SOFTSERIAL_ERR_PARITY_ERR = 0x00000003U,
    BOARD_SOFTSERIAL_ERR_STOP_BIT = 0x00000004U
} BOARD_SOFTSERIAL_Ret;

/**
  * @brief Data length.
  */
typedef enum {
    BOARD_SOFTSERIAL_DATALENGTH_5BIT = 0x00000005U,
    BOARD_SOFTSERIAL_DATALENGTH_6BIT = 0x00000006U,
    BOARD_SOFTSERIAL_DATALENGTH_7BIT = 0x00000007U,
    BOARD_SOFTSERIAL_DATALENGTH_8BIT = 0x00000008U
} BOARD_SOFTSERIAL_DataLength;

/**
  * @brief Parity Options.
  */
typedef enum {
    BOARD_SOFTSERIAL_PARITY_ODD = 0x00000000U,
    BOARD_SOFTSERIAL_PARITY_EVEN = 0x00000001U,
    BOARD_SOFTSERIAL_PARITY_NONE = 0x00000002U
} BOARD_SOFTSERIAL_ParityMode;

/**
  * @brief Stop bit length.
  */
typedef enum {
    BOARD_SOFTSERIAL_STOPBITS_ONE = 0x00000001U,
    BOARD_SOFTSERIAL_STOPBITS_TWO = 0x00000002U
} BOARD_SOFTSERIAL_StopBits;

/**
  * @brief Board soft serial config parameters.
  */
typedef struct {
    unsigned int                baudRate;
    BOARD_SOFTSERIAL_StopBits   stopBit;
    BOARD_SOFTSERIAL_DataLength dataLength;
    BOARD_SOFTSERIAL_ParityMode parityMode;
} BOARD_SOFTSERIAL_Config;

/**
  * @brief Soft serial port transmit handle.
  */
typedef struct {
    unsigned short txDataFrame; /**< Frames to be output. */
    unsigned short txDataFrameLength; /**< Length of the frame to be output. */
    unsigned short txDataBitCnt; /**< Number of bits that have been output in the current frame. */

    BOARD_SOFTSERIAL_DataLength dataLength; /**< Data length. */
    BOARD_SOFTSERIAL_ParityMode parityMode; /**< Parity check. */
    BOARD_SOFTSERIAL_StopBits stopBits; /**< Stop bits. */

    unsigned char txBuffer[BOARD_SOFTSERIAL_TX_BUFFER_SIZE]; /**< Send buffer. */
    unsigned char writeIndex; /**< Write index of the buffer. */
    unsigned char readIndex; /**< Read index of the buffer. */

    GPIO_Handle *gpioHandle; /**< GPIO for output. */
    TIMER_Handle *timerHandle; /**< Timer for timing based on baud rate. */
} BOARD_SOFTSERIAL_TxHandle;

/**
  * @brief Soft serial port receive handle.
  */
typedef struct {
    unsigned short rxDataFrameLength; /**< Length of the frame to be output. */
    unsigned short rxDataBitCnt; /**< Number of bits that have been output in the current frame. */

    unsigned char samplingCntThisBit; /**< Number of times the current bit has been sampled. */
    signed char samplingValue; /**< Accumulated sampled value. If the value is greater than 0, the level is high. */

    BOARD_SOFTSERIAL_DataLength dataLength; /**< Data length. */
    BOARD_SOFTSERIAL_ParityMode parityMode; /**< Parity check. */
    BOARD_SOFTSERIAL_StopBits stopBits; /**< Stop bits. */

    unsigned short rxBuffer[BOARD_SOFTSERIAL_RX_BUFFER_SIZE]; /**< Receive buffer. */
    unsigned char writeIndex; /**< Write index of the buffer. */
    unsigned char readIndex; /**< Read index of the buffer. */

    GPIO_Handle *gpioHandle; /**< GPIO for output. */
    TIMER_Handle *timerHandle; /**< Timer for timing based on baud rate. */
} BOARD_SOFTSERIAL_RxHandle;

/* Exported global functions ------------------------------------------------- */
void BOARD_SOFTSERIAL_TxInit(GPIO_Handle *gpioHandle, TIMER_Handle *timerHandle, BOARD_SOFTSERIAL_Config param);
BOARD_SOFTSERIAL_Ret BOARD_SOFTSERIAL_PrintCh(unsigned char ch);
void BOARD_SOFTSERIAL_TxTimerCallBack(void *param);
void BOARD_SOFTSERIAL_RxInit(GPIO_Handle *gpioHandle, TIMER_Handle *timerHandle, BOARD_SOFTSERIAL_Config param);
void BOARD_SOFTSERIAL_RxGpioCallBack(void *param);
void BOARD_SOFTSERIAL_RxTimerCallBack(void *param);
BOARD_SOFTSERIAL_Ret BOARD_SOFTSERIAL_GetChar(unsigned char *pData);

#endif /* McuMagicTag_SOFTSERIAL_H */