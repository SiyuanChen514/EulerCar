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
  * @file    i2c.c
  * @author  MCU Driver Team
  * @brief   I2C module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the I2C.
  *          + Initialization and de-initialization functions
  *          + Peripheral Control functions
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "i2c.h"

/* Macro definitions ---------------------------------------------------------*/
#define I2C_MAX_FIFO_SIZE      64
#define I2C_WAIT_TIMEOUT       0x400
#define I2C_MAX_DEV_ADDR       0x3FF

#define I2C_CFG_INTERRUPT_RX   0x1805    /* Enable all_cmd_done\arb_lost\rx_gt_watermark\ack_bit_unmatch */
#define I2C_CFG_INTERRUPT_TX   0x1811    /* Enable all_cmd_done\arb_lost\tx_lt_watermark\ack_bit_unmatch */

#define I2C_TICK_MS_DIV        1000

#define I2C_INTR_RAW_ALL_CMD_DONE_MASK      (0x1 << 12)
#define I2C_INTR_RAW_ARB_LOST_MASK          (0x1 << 11)
#define I2C_INTR_RAW_ACK_BIT_UNMATCH_MASK   (0x1 << 0)

/**
  * @brief Check all initial configuration parameters.
  * @param handle I2C handle.
  * @retval None.
  */
static void CheckAllInitParameters(I2C_Handle *handle)
{
#ifndef I2C_PARAM_CHECK
    BASE_FUNC_UNUSED(handle); /* If macro verification is not enabled, avoid alarms. */
#endif
    I2C_ASSERT_PARAM(IsI2cAddressMode(handle->addrMode));
    I2C_ASSERT_PARAM(IsI2cSdaHoldTime(handle->sdaHoldTime));
    I2C_ASSERT_PARAM(IsI2cFreq(handle->freq));
    I2C_ASSERT_PARAM(IsI2cIgnoreAckFlag(handle->ignoreAckFlag));
    I2C_ASSERT_PARAM(IsI2cTxWaterMark(handle->txWaterMark));
    I2C_ASSERT_PARAM(IsI2cRxWaterMark(handle->rxWaterMark));
}

/**
  * @brief I2C crash rescue.
  * @param handle i2c handle.
  * @retval None.
  */
static void I2cRescue(I2C_Handle *handle)
{
    unsigned int timeCnt;
    unsigned int index;

    handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_UNSET;
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Set the SCL and SDA pins of the I2C to GPIO mode. SCL = 1, SDA = 1 */
    handle->baseAddress->I2C_CTRL2.reg = 0x111; /* Set SCL = 1, SDA = 1, gpio mode enable. 0x111 = 0b10010001 */

    timeCnt = 0;
    do {
        /* The device that controls the bus to be pulled down needs to release the bus within the 9 clocks. */
        for (index = 0; index < 9; index++) {
            handle->baseAddress->I2C_CTRL2.BIT.i2c_scl_oen = BASE_CFG_UNSET;
            BASE_FUNC_DELAY_US(5); /* The I2C timing is required. The delay is about 5 μs. */
            handle->baseAddress->I2C_CTRL2.BIT.i2c_scl_oen = BASE_CFG_SET;
            BASE_FUNC_DELAY_US(5); /* The I2C timing is required. The delay is about 5 μs. */
        }

        timeCnt++;
        if (timeCnt > I2C_WAIT_TIMEOUT) {
            handle->baseAddress->I2C_CTRL2.reg = 0x11; /* Set SCL = 1, SDA = 1, gpio mode disable. 0x11 = 0b00010001 */
            return;
        }
    } while (!handle->baseAddress->I2C_CTRL2.BIT.i2c_sda_in);
    handle->baseAddress->I2C_CTRL2.reg = 0x111; /* Set SCL = 1, SDA = 1, gpio mode enable. 0x111 = 0b10010001 */
    /* I2C start */
    handle->baseAddress->I2C_CTRL2.BIT.force_sda_oen = BASE_CFG_UNSET;
    BASE_FUNC_DELAY_US(10); /* The I2C timing is required. The delay is about 10 μs. */
    /* I2C stop */
    handle->baseAddress->I2C_CTRL2.BIT.force_sda_oen = BASE_CFG_SET;
    /* Exit the I2C SCL and SDA pins to GPIO mode. */
    handle->baseAddress->I2C_CTRL2.reg = 0x11; /* Set SCL = 1, SDA = 1, gpio mode disable. 0x11 = 0b00010001 */
}

/**
  * @brief Setting a Single Timing Command.
  * @param handle I2C handle.
  * @param i2cCmd Timing Command.
  * @param offset Timing offset position.
  * @retval None.
  */
static void SetSingleTimingCmd(I2C_Handle *handle, I2C_CmdType i2cCmd, int *offset)
{
    handle->baseAddress->I2C_TIMING_CMD.BIT[*offset].timing_cmd = i2cCmd;
    (*offset)++;
}

/**
  * @brief Setting slavel address.
  * @param handle I2C handle.
  * @param offset Timing offset position.
  * @retval Current offset.
  */
static int SetSlaveAddress(I2C_Handle *handle, int offset)
{
    int currentOffset = offset;

    /* Write slave address */
    if (handle->addrMode == I2C_10_BITS) { /* 10bit address Configuration */
        if (handle->transferCount == 0) {
            SetSingleTimingCmd(handle, I2C_CMD_WDA2, &currentOffset);
            if (handle->ignoreAckFlag == I2C_IGNORE_NAK_ENABLE) {
                SetSingleTimingCmd(handle, I2C_CMD_RNC, &currentOffset); /* I2C Ignore Reply Configuration */
            } else {
                SetSingleTimingCmd(handle, I2C_CMD_RACK, &currentOffset);
            }
            SetSingleTimingCmd(handle, I2C_CMD_WDA1, &currentOffset);
        } else {
            SetSingleTimingCmd(handle, I2C_CMD_WDA2, &currentOffset);
        }
    } else {
        SetSingleTimingCmd(handle, I2C_CMD_WDA1, &currentOffset); /* 7bit address Configuration */
    }
    return currentOffset;
}

/**
  * @brief Set the Write Address Cmd object
  * @param handle I2C handle.
  * @retval Current instruction offset
  */
static int SetWriteAddressCmd(I2C_Handle *handle)
{
    int offset = 0;

    if (handle->transferCount == 0) { /* If no data is sent, send the start command first. */
        SetSingleTimingCmd(handle, I2C_CMD_S, &offset);
    } else {
        SetSingleTimingCmd(handle, I2C_CMD_SR, &offset);
    }
    /* Write slave address */
    offset = SetSlaveAddress(handle, offset);
    if (handle->ignoreAckFlag == I2C_IGNORE_NAK_ENABLE) {
        SetSingleTimingCmd(handle, I2C_CMD_RNC, &offset);
    } else {
        SetSingleTimingCmd(handle, I2C_CMD_RACK, &offset);
    }
    return offset;
}

/**
  * @brief Configuring the I2C Write Timing.
  * @param handle I2C handle.
  * @retval None.
  */
static void ConfigStandardWriteCmd(I2C_Handle *handle)
{
    int offset;

    offset = SetWriteAddressCmd(handle);
    /* Set the specifies the jump command. */
    handle->baseAddress->I2C_DST1.BIT.dst_timing_cmd1 = offset;
    SetSingleTimingCmd(handle, I2C_CMD_UDB1, &offset);
    SetSingleTimingCmd(handle, I2C_CMD_WDB1, &offset);
    /* If ignore ack is required, set the RNC command. */
    if (handle->ignoreAckFlag == I2C_IGNORE_NAK_ENABLE) {
        SetSingleTimingCmd(handle, I2C_CMD_RNC, &offset);
    } else {
        SetSingleTimingCmd(handle, I2C_CMD_RACK, &offset);
    }
    SetSingleTimingCmd(handle, I2C_CMD_JMPN1, &offset);
    /* If the data transfer is complete, set the STOP command. */
    SetSingleTimingCmd(handle, I2C_CMD_P, &offset);
    SetSingleTimingCmd(handle, I2C_CMD_EXIT, &offset);
}

/**
  * @brief Configuring the I2C Read Timing.
  * @param handle I2C handle.
  * @retval None.
  */
static void ConfigStandardReadCmd(I2C_Handle *handle)
{
    int offset;

    offset = SetWriteAddressCmd(handle);
    if (handle->transferSize > 1) {
        /* Set the specifies the jump command. */
        handle->baseAddress->I2C_DST1.BIT.dst_timing_cmd1 = offset;
        SetSingleTimingCmd(handle, I2C_CMD_RD, &offset);
        SetSingleTimingCmd(handle, I2C_CMD_SACK, &offset);
        SetSingleTimingCmd(handle, I2C_CMD_JMPN1, &offset);
    }

    SetSingleTimingCmd(handle, I2C_CMD_RD, &offset);
    SetSingleTimingCmd(handle, I2C_CMD_SNACK, &offset);
    /* If the data transfer is complete, set the STOP command. */
    SetSingleTimingCmd(handle, I2C_CMD_P, &offset);
    SetSingleTimingCmd(handle, I2C_CMD_EXIT, &offset);
}

/**
  * @brief Configuring the I2C Slave Device Address.
  * @param handle I2C handle.
  * @param devAddr Address of the communication device.
  * @retval None.
  */
static void SetDevAddr(I2C_Handle *handle, const unsigned short devAddr)
{
    unsigned short addr;
    if (handle->addrMode == I2C_10_BITS) {
        handle->baseAddress->I2C_DEV_ADDR.reg = devAddr;
    } else {
        addr = devAddr & 0xFF; /* The 8th digit is used */
        handle->baseAddress->I2C_DEV_ADDR.BIT.dev_addr_byte1 = addr;
    }
}

/**
  * @brief Waiting for RX not empty.
  * @param handle I2C handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType WaitRxNotEmpty(I2C_Handle *handle)
{
    unsigned int preTick = DCL_SYSTICK_GetTick();
    unsigned int curTick;
    unsigned long long delta = 0;
    unsigned long long targetDelta = SYSTICK_GetCRGHZ() / I2C_TICK_MS_DIV * handle->timeout;

    while (1) { /* Start timing. */
        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        if (handle->baseAddress->I2C_FIFO_STAT.BIT.rx_fifo_not_empty) {
            return BASE_STATUS_OK;
        }
        /* Check whether the timeout occurs. */
        if (delta >= targetDelta) {
            break;
        }
        preTick = curTick;
    };

    I2cRescue(handle); /* Perform related cleanup operations. */

    return BASE_STATUS_TIMEOUT;
}

/**
  * @brief Waiting for TX not full.
  * @param handle I2C handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType WaitTxNotFull(I2C_Handle *handle)
{
    unsigned int preTick = DCL_SYSTICK_GetTick();
    unsigned int curTick;
    unsigned long long delta = 0;
    unsigned long long targetDelta = SYSTICK_GetCRGHZ() / I2C_TICK_MS_DIV * handle->timeout;
    
    while (1) { /* Start timing. */
        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        if (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_not_full) { /* Check the tx fifo isn't full. */
            return BASE_STATUS_OK;
        } else if (handle->baseAddress->I2C_CTRL1.BIT.start != BASE_CFG_SET) {
            handle->baseAddress->I2C_CTRL1.BIT.start = BASE_CFG_SET;
        }
        preTick = curTick;
        /* Check whether the timeout occurs. */
        if (delta >= targetDelta) {
            break;
        }
    };

    I2cRescue(handle);

    return BASE_STATUS_TIMEOUT;
}

/**
  * @brief Waiting for idle.
  * @param handle I2C handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType WaitIdle(I2C_Handle *handle)
{
    unsigned int preTick = DCL_SYSTICK_GetTick();
    unsigned int curTick;
    unsigned long long delta = 0;
    unsigned long long targetDelta = SYSTICK_GetCRGHZ() / I2C_TICK_MS_DIV * handle->timeout;
    do {
        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        /* If any errors occurs, return error status. */
        if (handle->baseAddress->I2C_INTR_RAW.BIT.arb_lost_raw ||
            handle->baseAddress->I2C_INTR_RAW.BIT.ack_bit_unmatch_raw) {
            return BASE_STATUS_ERROR;
        }
        /* If all command has done, return ok status. */
        if (handle->baseAddress->I2C_INTR_RAW.BIT.all_cmd_done_raw) {
            return BASE_STATUS_OK;
        }
        /* Check whether the timeout occurs. */
        if (delta >= targetDelta) {
            break;
        }
        preTick = curTick;
    } while (true);

    I2cRescue(handle);

    return BASE_STATUS_TIMEOUT;
}

/**
  * @brief Check Sending Complete.
  * @param handle I2C handle.
  * @retval None.
  */
static void WaitSendComplete(I2C_Handle *handle)
{
    if (handle->baseAddress->I2C_GLB.BIT.i2c_enable) {
        WaitIdle(handle);
        handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_UNSET;
    }
}

/**
  * @brief Setting Error Handling.
  * @param handle I2C handle.
  * @retval None.
  */
static void SetErrorHandling(I2C_Handle *handle)
{
    handle->state = I2C_STATE_READY;
    /* Disable */
    handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_UNSET;
    /* Clears interrupts and disables interrupt reporting to facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    if (handle->errorCode != BASE_STATUS_OK && handle->userCallBack.ErrorCallback != NULL) {
        handle->userCallBack.ErrorCallback(handle);
    }
}

/**
  * @brief Checking Interrupts Caused by I2C Timing Errors.
  * @param handle I2C handle.
  * @retval true or false
  */
static bool CheckInterruptErrorStatus(I2C_Handle *handle)
{
    if (handle->baseAddress->I2C_INTR_STAT.BIT.arb_lost ||
        handle->baseAddress->I2C_INTR_STAT.BIT.ack_bit_unmatch) {
        handle->errorCode = BASE_STATUS_ERROR;
        /* Disable */
        handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_UNSET;
        /* Clears interrupts and disables interrupt reporting to
           facilitate switching between different working modes. */
        handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
        handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
        handle->state = I2C_STATE_READY;
        if (handle->userCallBack.ErrorCallback != NULL) {
            handle->userCallBack.ErrorCallback(handle);
        }
        return true;
    }
    return false;
}

/**
 * @brief I2C Interrupt TX Handling
 * @param handle I2C handle.
 * @retval None.
 */
static void InterruptTxHandle(I2C_Handle *handle)
{
    /* Cyclically move the data from the transfer buff to tx fifo. */
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_not_full &&
           handle->transferCount < handle->transferSize) {
        handle->baseAddress->I2C_TX_FIFO.BIT.tx_fifo = *handle->transferBuff++;
        handle->transferCount++;
    }
}

/**
 * @brief I2C Interrupt RX Handling
 * @param handle I2C handle.
 * @retval None.
 */
static void InterruptRxHandle(I2C_Handle *handle)
{
    /* Cyclically get the data from the rx fifo. */
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.rx_fifo_not_empty &&
           handle->transferCount < handle->transferSize) {
        *handle->transferBuff++ = handle->baseAddress->I2C_RX_FIFO.BIT.rx_fifo;
        handle->transferCount++;
    }
}

/**
 * @brief I2C Interrupt done Handling
 * @param handle I2C handle.
 * @retval None.
 */
static void InterruptAllDoneHandle(I2C_Handle *handle)
{
    /* Disable */
    handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_UNSET;
    /* Clears interrupts and disables interrupt reporting to
       facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    /* Called the user's callback based on the I2C status. */
    if (handle->userCallBack.RxCplCallback != NULL && handle->state == I2C_STATE_BUSY_MASTER_RX) {
        handle->userCallBack.RxCplCallback(handle);
    } else if (handle->userCallBack.TxCplCallback != NULL && handle->state == I2C_STATE_BUSY_MASTER_TX) {
        handle->userCallBack.TxCplCallback(handle);
    }
    handle->state = I2C_STATE_READY;
}

/**
  * @brief Initializing the I2C Module.
  * @param handle I2C handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_Init(I2C_Handle *handle)
{
    unsigned int freq;
    unsigned int clockFreq;
    unsigned int val;
    unsigned int glbReg;
    unsigned int temp;

    I2C_ASSERT_PARAM(handle != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    CheckAllInitParameters(handle); /* Check all config parameters. */

    freq = handle->freq;
    clockFreq = HAL_CRG_GetIpFreq((void *)handle->baseAddress);
    if (freq > clockFreq) {
        return BASE_STATUS_ERROR;
    }
    I2C_PARAM_CHECK_WITH_RET(freq, BASE_STATUS_ERROR);

    handle->state = I2C_STATE_BUSY;
    /* Disable */
    handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_UNSET;
    /* Clears interrupts and disables interrupt reporting to facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    /* Set SCL high and low duratiom time */
    if (freq <= I2C_STANDARD_FREQ_TH) {
        val = clockFreq / (freq * 2); /* The clockFreq / (freq * 2) = cloclFreq/0.5/freq */
        handle->baseAddress->I2C_HCNT.BIT.i2c_high_duration = val;
        handle->baseAddress->I2C_LCNT.BIT.i2c_low_duration = val;
    } else {
        val = ((clockFreq / 100) * 36) / freq; /* The ((clockFreq / 100) * 36) / freq = cloclFreq/0.36/freq */
        handle->baseAddress->I2C_HCNT.BIT.i2c_high_duration = val;
        val = ((clockFreq / 100) * 64) / freq; /* The ((clockFreq / 100) * 64) / freq = clockFreq/0.64/freq */
        handle->baseAddress->I2C_LCNT.BIT.i2c_low_duration = val;
    }
    /* Set sda hold duration.The value is fixed to 0xa */
    temp = ((unsigned int)I2C_SDA_HOLD_DURATION) << I2C_SDA_HOLD_DURATION_POS;
    glbReg = (handle->baseAddress->I2C_GLB.reg & (~I2C_SDA_HOLD_DURATION_MASK)) | temp;
    handle->baseAddress->I2C_GLB.reg = glbReg;

    /* Set I2C TX FIFO watermark */
    handle->baseAddress->I2C_TX_WATERMARK.BIT.tx_watermark = handle->txWaterMark;
    /* Set I2C RX FIFO watermark */
    handle->baseAddress->I2C_RX_WATERMARK.BIT.rx_watermark = handle->rxWaterMark;
    handle->state = I2C_STATE_READY;

    return BASE_STATUS_OK;
}

/**
  * @brief Deinitialize the I2C module.
  * @param handle I2C handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_Deinit(I2C_Handle *handle)
{
    I2C_ASSERT_PARAM(handle != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));

    handle->state = I2C_STATE_BUSY;

    /* Clears interrupts and disables interrupt reporting to facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    /* Clean interrupt callback functions. */
    handle->userCallBack.TxCplCallback = NULL;
    handle->userCallBack.RxCplCallback = NULL;
    handle->userCallBack.ErrorCallback = NULL;
    handle->state = I2C_STATE_RESET;

    return BASE_STATUS_OK;
}

/**
  * @brief Receiving data in blocking mode as master.
  * @param handle I2C handle.
  * @param devAddr Slave Device Address.
  * @param rData Address of the data buff to be receiving.
  * @param dataSize Number of the data to be receiving.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_MasterReadBlocking(I2C_Handle *handle,
                                           unsigned short devAddr,
                                           unsigned char *rData,
                                           unsigned int dataSize,
                                           unsigned int timeout)
{
    BASE_StatusType ret;

    I2C_ASSERT_PARAM(handle != NULL && rData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_PARAM_CHECK_WITH_RET(devAddr <= I2C_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);
    
    /* Configuring the I2C state and transmission parameters. */
    handle->state = I2C_STATE_BUSY_MASTER_RX;
    handle->transferBuff = rData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    handle->timeout = timeout;

    /* Wait I2C bus is idle. */
    WaitSendComplete(handle);

    /* Enable */
    handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_SET;
    SetDevAddr(handle, devAddr);
    /* Configuring the I2C Timing */
    ConfigStandardReadCmd(handle);
    /* The number of configuration cycles is used only when the read data is greater than 2. */
    if (handle->transferSize >= 2) {
        handle->baseAddress->I2C_LOOP1.BIT.loop_num1 = handle->transferSize - 2; /* Subtract 2 bytes from the header. */
    } else {
        handle->baseAddress->I2C_LOOP1.BIT.loop_num1 = 0;
    }

    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.start = BASE_CFG_SET;
    /* Cyclically get the data from the rx fifo. */
    while (handle->transferCount < handle->transferSize) {
        ret = WaitRxNotEmpty(handle);
        if (ret != BASE_STATUS_OK) {
            SetErrorHandling(handle);
            return ret;
        }
        *handle->transferBuff = handle->baseAddress->I2C_RX_FIFO.BIT.rx_fifo;
        handle->transferBuff++;
        handle->transferCount++;
    }
    
    ret = WaitIdle(handle); /* Waiting for all data transfer to complete. */
    handle->errorCode = ret;
    if (handle->userCallBack.RxCplCallback != NULL && ret == BASE_STATUS_OK) {
        handle->userCallBack.RxCplCallback(handle);
    }
    SetErrorHandling(handle);
    return ret;
}

/**
  * @brief Send data in blocking mode as master.
  * @param handle I2C handle.
  * @param devAddr Slave Device Address.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_MasterWriteBlocking(I2C_Handle *handle,
                                            unsigned short devAddr,
                                            unsigned char *wData,
                                            unsigned int dataSize,
                                            unsigned int timeout)
{
    BASE_StatusType ret;

    I2C_ASSERT_PARAM(handle != NULL && wData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_PARAM_CHECK_WITH_RET(devAddr <= I2C_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    /* Configuring the I2C state and transmission parameters. */
    handle->state = I2C_STATE_BUSY_MASTER_TX;
    handle->transferBuff = wData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    handle->timeout = timeout;

    /* Wait I2C bus is idle. */
    WaitSendComplete(handle);

    /* Enable */
    handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_SET;
    SetDevAddr(handle, devAddr);
    /* Configuring the I2C Timing */
    ConfigStandardWriteCmd(handle);
    handle->baseAddress->I2C_LOOP1.BIT.loop_num1 = handle->transferSize - 1;
    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    /* Cyclically move the data from the transfer buff to tx fifo. */
    while (handle->transferCount < handle->transferSize) {
        ret = WaitTxNotFull(handle);
        if (ret != BASE_STATUS_OK) {
            SetErrorHandling(handle);
            return ret;
        }
        handle->baseAddress->I2C_TX_FIFO.BIT.tx_fifo = *handle->transferBuff;
        handle->transferBuff++;
        handle->transferCount++;
    }
    /* If the size of the transferred data less than tx fifo size, set I2C start. */
    if (handle->baseAddress->I2C_CTRL1.BIT.start != BASE_CFG_SET &&
        handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_not_empty) {
        handle->baseAddress->I2C_CTRL1.BIT.start = BASE_CFG_SET;
    }
    ret = WaitIdle(handle); /* Waiting for all data transfer to complete. */
    handle->errorCode = ret;
    if (handle->userCallBack.TxCplCallback != NULL && ret == BASE_STATUS_OK) {
        handle->userCallBack.TxCplCallback(handle);
    }
    SetErrorHandling(handle);
    return ret;
}

/**
  * @brief Receiving data in blocking mode as slave.
  * @param handle I2C handle.
  * @param rData Address of the data buff to be receiving.
  * @param dataSize Number of the data to be receiving.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_SlaveReadBlocking(I2C_Handle *handle,
                                          unsigned char *rData,
                                          unsigned int dataSize,
                                          unsigned int timeout)
{
    BASE_FUNC_UNUSED(handle);
    BASE_FUNC_UNUSED(rData);
    BASE_FUNC_UNUSED(dataSize);
    BASE_FUNC_UNUSED(timeout);
    /* The slave mode is not supported. */
    return BASE_STATUS_NOT_SUPPORT;
}

/**
  * @brief Send data in blocking mode as slave.
  * @param handle I2C handle.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_SlaveWriteBlocking(I2C_Handle *handle,
                                           unsigned char *wData,
                                           unsigned int dataSize,
                                           unsigned int timeout)
{
    BASE_FUNC_UNUSED(handle);
    BASE_FUNC_UNUSED(wData);
    BASE_FUNC_UNUSED(dataSize);
    BASE_FUNC_UNUSED(timeout);
    /* The slave mode is not supported. */
    return BASE_STATUS_NOT_SUPPORT;
}

/**
  * @brief Receiving data in interrupts mode as master.
  * @param handle I2C handle.
  * @param devAddr Slave Device Address.
  * @param rData Address of the data buff to be receiving.
  * @param dataSize Number of the data to be receiving.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_MasterReadIT(I2C_Handle *handle, unsigned short devAddr,
                                     unsigned char *rData, unsigned int dataSize)
{
    I2C_ASSERT_PARAM(handle != NULL && rData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_PARAM_CHECK_WITH_RET(devAddr <= I2C_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    /* Configuring the I2C state and transmission parameters. */
    handle->state = I2C_STATE_BUSY_MASTER_RX;
    handle->transferCount = 0;
    handle->transferBuff = rData;
    handle->transferSize = dataSize;

    /* Wait I2C bus is idle. */
    WaitSendComplete(handle);

    /* Enable */
    handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_SET;
    /* Clean interrupt */
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Enable interrupt */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_CFG_INTERRUPT_RX;

    SetDevAddr(handle, devAddr);
    /* Configuring the I2C Timing */
    ConfigStandardReadCmd(handle);
    /* The number of configuration cycles is used only when the read data is greater than 2. */
    if (handle->transferSize >= 2) {
        handle->baseAddress->I2C_LOOP1.BIT.loop_num1 = handle->transferSize - 2; /* Subtract 2 bytes from the header. */
    } else {
        handle->baseAddress->I2C_LOOP1.BIT.loop_num1 = 0;
    }
    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.start = BASE_CFG_SET;

    return BASE_STATUS_OK;
}

/**
  * @brief Send data in interrupts mode as master.
  * @param handle I2C handle.
  * @param devAddr Slave Device Address.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_MasterWriteIT(I2C_Handle *handle, unsigned short devAddr,
                                      unsigned char *wData, unsigned int dataSize)
{
    I2C_ASSERT_PARAM(handle != NULL && wData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_PARAM_CHECK_WITH_RET(devAddr <= I2C_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    /* Configuring the I2C state and transmission parameters. */
    handle->state = I2C_STATE_BUSY_MASTER_TX;
    handle->transferBuff = wData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    
    /* Wait I2C bus is idle. */
    WaitSendComplete(handle);

    /* Enable */
    handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_SET;
    /* Clean interrupt */
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    /* Enable interrupt */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_CFG_INTERRUPT_TX;

    SetDevAddr(handle, devAddr);
    /* Configuring the I2C Timing */
    ConfigStandardWriteCmd(handle);
    handle->baseAddress->I2C_LOOP1.BIT.loop_num1 = handle->transferSize - 1;
    /* Clean interrupt */
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.start = BASE_CFG_SET;
    return BASE_STATUS_OK;
}

/**
  * @brief Receiving data in interrupts mode as slave.
  * @param handle I2C handle.
  * @param rData Address of the data buff to be receiving.
  * @param dataSize Number of the data to be receiving.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_SlaveReadIT(I2C_Handle *handle, unsigned char *rData, unsigned int dataSize)
{
    BASE_FUNC_UNUSED(handle);
    BASE_FUNC_UNUSED(rData);
    BASE_FUNC_UNUSED(dataSize);
    /* The slave mode is not supported. */
    return BASE_STATUS_NOT_SUPPORT;
}

/**
  * @brief Send data in interrupts mode as slave.
  * @param handle I2C handle.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_SlaveWriteIT(I2C_Handle *handle, unsigned char *wData, unsigned int dataSize)
{
    BASE_FUNC_UNUSED(handle);
    BASE_FUNC_UNUSED(wData);
    BASE_FUNC_UNUSED(dataSize);
    /* The slave mode is not supported. */
    return BASE_STATUS_NOT_SUPPORT;
}

/**
  * @brief I2C DMA Error Handling.
  * @param handle I2C handle.
  * @retval None.
  */
static void I2CDmaErrorHandle(I2C_Handle *handle)
{
    /* Disable and reset related registers. */
    handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_UNSET;
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.start = BASE_CFG_UNSET;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    handle->errorCode = BASE_STATUS_ERROR;
    /* Invoke the error callback function set by the user. */
    if (handle->userCallBack.ErrorCallback != NULL) {
        handle->userCallBack.ErrorCallback(handle);
    }
    handle->state = I2C_STATE_READY;
}

/**
 * @brief I2C DMA completes processing.
 * @param handle I2C handle.
 * @retval None.
 */
static void I2cDmaDoneHandle(I2C_Handle *handle)
{
    /* Disable and reset related registers. */
    handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_UNSET;
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.start = BASE_CFG_UNSET;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    /* Called the user's callback based on the I2C status. */
    if (handle->state == I2C_STATE_BUSY_MASTER_RX) {
        if (handle->userCallBack.RxCplCallback != NULL) {
            handle->userCallBack.RxCplCallback(handle);
        }
    }
    if (handle->state == I2C_STATE_BUSY_MASTER_TX) {
        if (handle->userCallBack.TxCplCallback != NULL) {
            handle->userCallBack.TxCplCallback(handle);
        }
    }
    handle->state = I2C_STATE_READY;
}

/**
 * @brief Wait until all I2C timings are processed.
 * @param handle I2C handle.
 * @retval None.
 */
static void WaitHandleFinish(I2C_Handle *handle)
{
    unsigned int intrRwa;

    while (1) { /* Wait until all I2C timings are processed in DMA mode. */
        intrRwa = handle->baseAddress->I2C_INTR_RAW.reg;
        /* Check whether errors occur. */
        if ((intrRwa & (I2C_INTR_RAW_ARB_LOST_MASK | I2C_INTR_RAW_ACK_BIT_UNMATCH_MASK)) > 0) {
            I2CDmaErrorHandle(handle);
            break;
        }
        if ((intrRwa & I2C_INTR_RAW_ALL_CMD_DONE_MASK) > 0) {
            I2cDmaDoneHandle(handle);
            break;
        }
    }
}

/**
 * @brief The I2C uses the DMA completion callback function registered by the DMA module.
 * @param handle I2C handle.
 * @retval None.
 */
static void DMAFinishFun(void *handle)
{
    I2C_ASSERT_PARAM(handle != NULL);
    I2C_Handle *i2cHandle = (I2C_Handle *)(handle);
    I2C_ASSERT_PARAM(IsI2CInstance(i2cHandle->baseAddress));

    WaitHandleFinish(i2cHandle);
}

/**
 * @brief The I2C uses the DMA error callback function registered by the DMA module.
 * @param handle I2C handle.
 * @retval None.
 */
static void DmaErrorHandlerFun(void *handle)
{
    I2C_ASSERT_PARAM(handle != NULL);
    I2C_Handle *i2cHandle = (I2C_Handle *)(handle);
    I2C_ASSERT_PARAM(IsI2CInstance(i2cHandle->baseAddress));

    /* Disable and reset related registers. */
    i2cHandle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_UNSET;
    i2cHandle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    i2cHandle->errorCode = BASE_STATUS_ERROR;
    if (i2cHandle->userCallBack.ErrorCallback != NULL) {
        i2cHandle->userCallBack.ErrorCallback(i2cHandle);
    }
    /* Called the user's callback based on the I2C status. */
    if (i2cHandle->state == I2C_STATE_BUSY_MASTER_TX) {
        HAL_DMA_StopChannel(i2cHandle->dmaHandle, i2cHandle->txDmaCh);
    } else if (i2cHandle->state == I2C_STATE_BUSY_MASTER_RX) {
        HAL_DMA_StopChannel(i2cHandle->dmaHandle, i2cHandle->rxDmaCh);
    }
    i2cHandle->state = I2C_STATE_READY;
}

/**
  * @brief Receiving data in DMA mode as master.
  * @param handle I2C handle.
  * @param devAddr Slave Device Address.
  * @param rData Address of the data buff to be receiving.
  * @param dataSize Number of the data to be receiving.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_MasterReadDMA(I2C_Handle *handle, unsigned short devAddr,
                                      unsigned char *rData, unsigned int dataSize)
{
    I2C_ASSERT_PARAM(handle != NULL && rData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_ASSERT_PARAM(handle->rxDmaCh < CHANNEL_MAX_NUM);
    I2C_PARAM_CHECK_WITH_RET(devAddr <= I2C_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    /* Configuring the I2C state and transmission parameters. */
    handle->state = I2C_STATE_BUSY_MASTER_RX;
    handle->transferBuff = rData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    
    /* Wait I2C bus is idle. */
    WaitSendComplete(handle);

    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->rxDmaCh].ChannelFinishCallBack = DMAFinishFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->rxDmaCh].ChannelErrorCallBack = DmaErrorHandlerFun;
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)&(handle->baseAddress->I2C_RX_FIFO),
                        (uintptr_t)handle->transferBuff, handle->transferSize, handle->rxDmaCh) != BASE_STATUS_OK) {
        handle->state = I2C_STATE_READY;
        return BASE_STATUS_ERROR;
    }

    /* Enable I2C */
    handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_SET;

    SetDevAddr(handle, devAddr);
    /* Configuring the I2C Timing */
    ConfigStandardReadCmd(handle);
    /* The number of configuration cycles is used only when the read data is greater than 2. */
    if (handle->transferSize >= 2) {
        handle->baseAddress->I2C_LOOP1.BIT.loop_num1 = handle->transferSize -2; /* Subtract 2 bytes from the header. */
    } else {
        handle->baseAddress->I2C_LOOP1.BIT.loop_num1 = 0;
    }
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_READ;
    handle->baseAddress->I2C_CTRL1.BIT.start = BASE_CFG_SET;
    return BASE_STATUS_OK;
}

/**
  * @brief Send data in DMA mode as master.
  * @param handle I2C handle.
  * @param devAddr Slave Device Address.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_MasterWriteDMA(I2C_Handle *handle, unsigned short devAddr,
                                       unsigned char *wData, unsigned int dataSize)
{
    I2C_ASSERT_PARAM(handle != NULL && wData != NULL);
    I2C_ASSERT_PARAM(handle->txDmaCh < CHANNEL_MAX_NUM);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_PARAM_CHECK_WITH_RET(devAddr <= I2C_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    /* Configuring the I2C state and transmission parameters. */
    handle->state = I2C_STATE_BUSY_MASTER_TX;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    handle->transferBuff = wData;

    /* Wait I2C bus is idle. */
    WaitSendComplete(handle);
    /* Registers the callback function after the DMA transfer is complete. */
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelFinishCallBack = DMAFinishFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelErrorCallBack = DmaErrorHandlerFun;
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)handle->transferBuff,
                        (uintptr_t)&handle->baseAddress->I2C_TX_FIFO,
                        handle->transferSize, handle->txDmaCh) != BASE_STATUS_OK) {
        handle->state = I2C_STATE_READY;
        return BASE_STATUS_ERROR;
    }

    /* Enable I2C */
    handle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_SET;

    SetDevAddr(handle, devAddr);
    /* Configuring the I2C Timing */
    ConfigStandardWriteCmd(handle);
    handle->baseAddress->I2C_LOOP1.BIT.loop_num1 = handle->transferSize - 1;
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_WRITE;
    handle->baseAddress->I2C_CTRL1.BIT.start = BASE_CFG_SET;
    return BASE_STATUS_OK;
}

/**
  * @brief Receiving data in DMA mode as slave.
  * @param handle I2C handle.
  * @param rData Address of the data buff to be receiving.
  * @param dataSize Number of the data to be receiving.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_SlaveReadDMA(I2C_Handle *handle, unsigned char *rData, unsigned int dataSize)
{
    BASE_FUNC_UNUSED(handle);
    BASE_FUNC_UNUSED(rData);
    BASE_FUNC_UNUSED(dataSize);
    /**< The slave mode is not supported. */
    return BASE_STATUS_NOT_SUPPORT;
}

/**
  * @brief Send data in DMA mode as salve.
  * @param handle I2C handle.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_SlaveWriteDMA(I2C_Handle *handle, unsigned char *wData, unsigned int dataSize)
{
    BASE_FUNC_UNUSED(handle);
    BASE_FUNC_UNUSED(wData);
    BASE_FUNC_UNUSED(dataSize);
    /**< The slave mode is not supported. */
    return BASE_STATUS_NOT_SUPPORT;
}

/**
  * @brief Callback Function Registration.
  * @param handle I2C handle.
  * @param callbackID Callback function ID..
  * @param pcallback Pointer to the address of the registered callback function.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_RegisterCallback(I2C_Handle *handle, I2C_CallbackId callbackID, I2C_CallbackFunType pcallback)
{
    BASE_StatusType ret = BASE_STATUS_OK;

    I2C_ASSERT_PARAM(handle != NULL && pcallback != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    /* Called the user's callback based on the I2C status. */
    if (handle->state == I2C_STATE_READY) {
        switch (callbackID) {
            case I2C_MASTER_TX_COMPLETE_CB_ID : /* Register the callback function when the transfer is complete. */
                handle->userCallBack.TxCplCallback = pcallback;
                break;
            case I2C_MASTER_RX_COMPLETE_CB_ID :
                handle->userCallBack.RxCplCallback = pcallback;
                break;
            case I2C_ERROR_CB_ID : /* Registering an Error Callback Function. */
                handle->userCallBack.ErrorCallback = pcallback;
                break;
            default:
                ret = BASE_STATUS_ERROR;
                handle->errorCode = BASE_STATUS_ERROR;
                break;
        }
    } else { /* If the status is not ready, an error is returned. */
        ret = BASE_STATUS_ERROR;
        handle->errorCode = BASE_STATUS_ERROR;
    }
    return ret;
}

/**
  * @brief Interrupt Handling Function.
  * @param handle Handle pointers
  * @retval None
  */
void HAL_I2C_IrqHandler(void *handle)
{
    I2C_Handle *i2cHandle = (I2C_Handle *)handle;

    I2C_ASSERT_PARAM(i2cHandle != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(i2cHandle->baseAddress));

    if (CheckInterruptErrorStatus(i2cHandle)) {
        return;
    }
    /* Called the corresponding processing function based on the I2C status. */
    if (i2cHandle->state == I2C_STATE_BUSY_MASTER_TX) {
        InterruptTxHandle(i2cHandle);
    } else if (i2cHandle->state == I2C_STATE_BUSY_MASTER_RX) {
        InterruptRxHandle(i2cHandle);
    } else {
        i2cHandle->errorCode = BASE_STATUS_ERROR;
        /* Disable */
        i2cHandle->baseAddress->I2C_GLB.BIT.i2c_enable = BASE_CFG_UNSET;
        /* Clears interrupts and disables interrupt reporting to
           facilitate switching between different working modes. */
        i2cHandle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
        i2cHandle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
        i2cHandle->state = I2C_STATE_READY;
        if (i2cHandle->userCallBack.ErrorCallback != NULL) {
            i2cHandle->userCallBack.ErrorCallback(i2cHandle);
        }
    }
    /* Check whether all data transmissions are complete. */
    if (i2cHandle->baseAddress->I2C_INTR_STAT.BIT.all_cmd_done) {
        InterruptAllDoneHandle(i2cHandle);
    }
    /* If all data transmissions are moved, close the water mark function. */
    if (i2cHandle->transferCount >= i2cHandle->transferSize) {
        i2cHandle->baseAddress->I2C_INTR_EN.BIT.tx_lt_watermark_en = BASE_CFG_UNSET;
        i2cHandle->baseAddress->I2C_INTR_EN.BIT.rx_gt_watermark_en = BASE_CFG_UNSET;
    }
    i2cHandle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
}