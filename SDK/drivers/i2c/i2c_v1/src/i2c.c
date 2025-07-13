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
#define I2C_INTERFACE_INDEX_0         0
#define I2C_INTERFACE_INDEX_1         1
#define I2C_MAX_INDEX_NUM             2

#define I2C_MASTER_STATUS             0x00
#define I2C_SLAVE_STATUS              0x01

#define I2C_MAX_FIFO_SIZE             16
#define I2C_WAIT_TIMEOUT              0x400
#define I2C_MAX_DEV_ADDR              0x3FF

#define I2C_OPERATION_WRITE           0
#define I2C_OPERATION_READ            1

#define I2C_SEND_ADDR_STATUS_NONE     0
#define I2C_SEND_ADDR_STATUS_WRITE    1
#define I2C_SEND_ADDR_STATUS_READ     2

#define I2C_DATA_OPT_SETP_PRE         1
#define I2C_DATA_OPT_SETP_NORMAL      2

/* Enable scl_low_timeout\mst_cmd_done\arb_lost\tx_fifo_not_full\rx_fifo_not_empty\mst_rx_ack_unmatch */
#define I2C_CFG_INTERRUPT_MASTER_RX   0x3843

/* Enable scl_low_timeout\mst_cmd_done\arb_lost\tx_fifo_not_full\mst_rx_ack_unmatch */
#define I2C_CFG_INTERRUPT_MASTER_TX   0x3841

/* Enable slv_addr_match_int\slv_rx_ack_unmatch_int\stop_det_int */
#define I2C_CFG_INTERRUPT_SLAVE       0x300200
#define I2C_TICK_MS_DIV               1000

#define I2C_INTR_RAW_SLAVE_ADDR_MATCH_MASK      (0x1 << 21)
#define I2C_INTR_RAW_SLAVE_ACK_UNMATCH_MASK     (0x1 << 20)
#define I2C_INTR_RAW_SLAVE_PEC_CHECK_FAIL_MASK  (0x1 << 19)
#define I2C_INTR_RAW_MASTER_PEC_CHECK_FAIL_MASK (0x1 << 18)
#define I2C_INTR_RAW_SLAVE_CMD_INT1_MASK        (0x1 << 17)
#define I2C_INTR_RAW_MASTER_CMD_INT1_MASK       (0x1 << 16)
#define I2C_INTR_RAW_SMB_SUSPEND_MASK           (0x1 << 15)
#define I2C_INTR_RAW_SMB_ALERT_MASK             (0x1 << 14)
#define I2C_INTR_RAW_SCL_LOW_TIMEOUT_MASK       (0x1 << 13)
#define I2C_INTR_RAW_ALL_CMD_DONE_MASK          (0x1 << 12)
#define I2C_INTR_RAW_ARB_LOST_MASK              (0x1 << 11)
#define I2C_INTR_RAW_START_DET_MASK             (0x1 << 10)
#define I2C_INTR_RAW_STOP_DET_MASK              (0x1 << 9)
#define I2C_INTR_RAW_TX_DATA_REQUEST_MASK       (0x1 << 8)
#define I2C_INTR_RAW_RX_DATA_READY_MASK         (0x1 << 7)
#define I2C_INTR_RAW_TX_FIFO_NOT_FULL_MASK      (0x1 << 6)
#define I2C_INTR_RAW_TX_FIFO_EMPTY_MASK         (0x1 << 5)
#define I2C_INTR_RAW_TX_LE_WATERMARK_MASK       (0x1 << 4)
#define I2C_INTR_RAW_RX_FIFO_FULL_MASK          (0x1 << 3)
#define I2C_INTR_RAW_RX_GE_WATERMARK_MASK       (0x1 << 2)
#define I2C_INTR_RAW_RX_FIFO_NOT_EMPTY_MASK     (0x1 << 1)
#define I2C_INTR_RAW_ACK_BIT_UNMATCH_MASK       (0x1 << 0)

#define I2C_10BIT_SLAVE_READ_ADDR_MASK          (0xFEFF0000)
#define I2C_10BIT_SLAVE_WRITE_ADDR_MASK         (0x0000FEFF)
#define I2C_10BIT_SLAVE_READ_OPT_MASK           (0x01000000)

#define I2C_7BIT_SLAVE_READ_ADDR_MASK           (0x00FE0000)
#define I2C_7BIT_SLAVE_WRITE_ADDR_MASK          (0x000000FE)
#define I2C_7BIT_SLAVE_READ_OPT_MASK            (0x00010000)

#define I2C_SLAVE_WRITE_ADDR_POS                 8
#define I2C_SLAVE_READ_FIX_ADDR_POS              24
#define I2C_SLAVE_READ_DEV_ADDR_POS              16
#define I2C_SLAVE_ADDR_MASK                      0xFF
#define I2C_10BIT_SLAVE_ADDR_POS                 16

#define HIGH_HOLD_TIME_POS                       16
#define HIGH_HOLD_TIME_MASK                      0xFFFF0000
#define LOW_HOLD_TIME_MASK                       0x0000FFFF

#define DMA_RX_CHANNEL_POS                       8
#define DMA_CHANNEL_MASK                         0x00FF

#define COMMAND_ALL_DONE                         0
#define I2C_BUS_IS_FREE                          1
#define SLAVE_ADDRESS_MATCH                      2
#define TX_FIFO_NOT_FULL                         3
#define RX_FIFO_NOT_EMPTY                        4

#define I2C_FREQ_HIGH_PARAMTER                   8
#define I2C_FREQ_LOW_PARAMTER                    9
#define I2C_ERROR_BIT_MASK                       0x100801 /* slv_rx_ack_unmatch\arb_lost\mst_rx_ack_unmatch */
#define I2C_SCL_LOW_TIMEOUT_MASK                 0x2000

static BASE_StatusType DmaMasterReadData(I2C_Handle *handle, unsigned int size, unsigned int index);
static BASE_StatusType DmaMasterWriteData(I2C_Handle *handle, unsigned int size, unsigned int index);
static BASE_StatusType DmaSlaveReadData(I2C_Handle *handle, unsigned int size, unsigned int index);
static BASE_StatusType DmaSlaveWriteData(I2C_Handle *handle, unsigned int size, unsigned int index);

typedef struct {
    unsigned int txReadCmdCnt;
    /* The lower 16 bits are used for write operations,
       and the high 16 bits are used for read operations. */
    unsigned int slaveAddress;
    unsigned int sendAddressStatus;
} I2C_InternalConfigParam;

/* Some global parameters used for module internal operations */
static volatile I2C_InternalConfigParam g_internalConfigParam[I2C_MAX_INDEX_NUM] = {0};
static volatile unsigned int g_internalTxBuffDMA[I2C_MAX_INDEX_NUM][I2C_ONCE_TRANS_MAX_NUM] = {0};
static volatile unsigned int g_dmaTransferSize = 0;
/**
  * @brief Check all initial configuration parameters.
  * @param handle I2C handle.
  * @param clockFreq  I2C work clock freq;
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType CheckAllInitParameters(I2C_Handle *handle, unsigned int clockFreq)
{
    /* Check the configuration of basic function parameters. */
    I2C_PARAM_CHECK_WITH_RET(IsI2cFunctionMode(handle->functionMode), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(IsI2cAddressMode(handle->addrMode), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(IsI2cSdaHoldTime(handle->sdaHoldTime), BASE_STATUS_ERROR);
    /* Check whether the I2C freq is valid. */
    I2C_PARAM_CHECK_WITH_RET(IsI2cFreq(handle->freq), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET((clockFreq > 0), BASE_STATUS_ERROR);

    if (handle->freq > clockFreq) {
        return BASE_STATUS_ERROR;
    }
    /* Check the configuration of basic function parameters. */
    I2C_PARAM_CHECK_WITH_RET(IsI2cIgnoreAckFlag(handle->ignoreAckFlag), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(IsI2cTxWaterMark(handle->txWaterMark), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(IsI2cRxWaterMark(handle->rxWaterMark), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(IsI2cSpikeFilterTime(handle->handleEx.spikeFilterTime), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(IsI2cSdaDelayTime(handle->handleEx.sdaDelayTime), BASE_STATUS_ERROR);

    /* Checking the own address and generalCall parameter enable when is used as slave. */
    if (handle->functionMode == I2C_MODE_SELECT_SLAVE_ONLY || handle->functionMode == I2C_MODE_SELECT_MASTER_SLAVE) {
        I2C_PARAM_CHECK_WITH_RET(IsI2cOwnAddressOrMask(handle->slaveOwnAddress), BASE_STATUS_ERROR);
        I2C_PARAM_CHECK_WITH_RET(IsI2cGeneralCallMode(handle->generalCallMode), BASE_STATUS_ERROR);
        I2C_PARAM_CHECK_WITH_RET(IsXMBusAddressEnable(handle->handleEx.slaveOwnXmbAddressEnable), BASE_STATUS_ERROR);
        I2C_PARAM_CHECK_WITH_RET(IsXMBusAddressOrMask(handle->handleEx.slaveOwnXmbAddress), BASE_STATUS_ERROR);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Configuring the I2C Slave Device Address.
  * @param handle I2C handle.
  * @param devAddr Slave device address
  * @retval None.
  */
static void SetSlaveDevAddr(I2C_Handle *handle, const unsigned int devAddr)
{
    unsigned int addr;

    if (handle->addrMode == I2C_10_BITS) {
        /* The upper 16 bits are the read operation address, and the lower 16 bits are the write operation address. */
        addr = (((devAddr << 16) & I2C_10BIT_SLAVE_READ_ADDR_MASK) | I2C_10BIT_SLAVE_READ_OPT_MASK) |
               (devAddr & I2C_10BIT_SLAVE_WRITE_ADDR_MASK);
    } else {
        /* The upper 16 bits are the read operation address, and the lower 16 bits are the write operation address. */
        addr = (((devAddr << 16) & I2C_7BIT_SLAVE_READ_ADDR_MASK) | I2C_7BIT_SLAVE_READ_OPT_MASK) |
               (devAddr & I2C_7BIT_SLAVE_WRITE_ADDR_MASK);
    }
    
    if (handle->baseAddress == I2C0) {
        g_internalConfigParam[I2C_INTERFACE_INDEX_0].slaveAddress = addr;
    } else if (handle->baseAddress == I2C1) {
        g_internalConfigParam[I2C_INTERFACE_INDEX_1].slaveAddress = addr;
    }
}

/**
  * @brief I2C Bus clear.
  * @param handle I2C handle.
  * @retval None.
  */
static void I2cBusClear(I2C_Handle *handle)
{
    handle->state = I2C_STATE_READY;
    handle->baseAddress->I2C_MODE.BIT.mst_slv_function = I2C_STATE_RESET;
    /* Clears interrupts and disables interrupt reporting to
       facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Set the SCL and SDA pins of the I2C to GPIO mode. */
    handle->baseAddress->I2C_CTRL2.BIT.gpio_mode = BASE_CFG_ENABLE;
    handle->baseAddress->I2C_CTRL2.BIT.force_scl = BASE_CFG_ENABLE;
    handle->baseAddress->I2C_CTRL2.BIT.force_sda = BASE_CFG_ENABLE;
    /* The device that controls the bus to be pulled down needs to release the bus within the 9 clocks. */
    for (unsigned int index = 0; index < 9; index++) {
        handle->baseAddress->I2C_CTRL2.BIT.force_scl = BASE_CFG_UNSET;
        BASE_FUNC_DELAY_US(5); /* The I2C timing is required. The delay is about 5 μs. */
        handle->baseAddress->I2C_CTRL2.BIT.force_scl = BASE_CFG_SET;
        BASE_FUNC_DELAY_US(5); /* The I2C timing is required. The delay is about 5 μs. */
    }
    handle->baseAddress->I2C_CTRL2.BIT.force_scl = BASE_CFG_ENABLE;
    handle->baseAddress->I2C_CTRL2.BIT.force_sda = BASE_CFG_ENABLE;
    /* I2C start */
    handle->baseAddress->I2C_CTRL2.BIT.force_sda = BASE_CFG_UNSET;
    BASE_FUNC_DELAY_US(10); /* The I2C timing is required. The delay is about 10 μs. */
    /* I2C stop */
    handle->baseAddress->I2C_CTRL2.BIT.force_sda = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL2.BIT.gpio_mode = BASE_CFG_DISABLE; /* Exit the I2C GPIO mode. */
}

/**
  * @brief Setting Error Handling.
  * @param handle I2C handle.
  * @retval None.
  */
static void SetErrorHandling(I2C_Handle *handle)
{
    /* If the low level times out, the I2C bus is cleared and the bus is expected to be released. */
    if (handle->baseAddress->I2C_INTR_RAW.BIT.scl_low_timeout_raw == BASE_CFG_ENABLE) {
        I2cBusClear(handle);
        handle->baseAddress->I2C_INTR_RAW.BIT.scl_low_timeout_raw = BASE_CFG_ENABLE;
    }

    if (handle->errorCode != BASE_STATUS_OK && handle->userCallBack.ErrorCallback != NULL) {
        handle->userCallBack.ErrorCallback(handle);
    }
    /* Clears interrupts and disables interrupt reporting to
       facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    handle->state = I2C_STATE_READY;
}

/**
  * @brief Item is checked for readiness.
  * @param handle I2C handle.
  * @param checkItem The item to be checked.
  * @param opt Read or write flag.
  * @retval false, item is not ready. true, item is ready.
  */
static unsigned int CheckItemStatus(I2C_Handle *handle, unsigned int checkItem, unsigned int opt)
{
    unsigned int ret = 0;
    unsigned int tempStatusValue = 0;
    switch (checkItem) {
        case COMMAND_ALL_DONE:
            /* The 0x1200 is the bit of mst_cmd_done_raw and stop_det_raw. */
            tempStatusValue = (handle->baseAddress->I2C_INTR_RAW.reg & 0x1200); /* Check the I2C is all command done. */
            ret = tempStatusValue;
            break;
        case I2C_BUS_IS_FREE:
            /* The I2C bus is free. */
            ret = handle->baseAddress->I2C_FSM_STAT.BIT.i2c_bus_free;
            break;
        case SLAVE_ADDRESS_MATCH:
            /* Slave servers are matched */
            tempStatusValue = (handle->baseAddress->I2C_RX_ADDR.BIT.rx_rw == opt) ? 1 : 0;
            tempStatusValue |= handle->baseAddress->I2C_INTR_RAW.BIT.slv_addr_match_raw;
            ret = tempStatusValue;
            break;
        case TX_FIFO_NOT_FULL:
            /* Tx fifo is not full. */
            ret = ((handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < I2C_MAX_FIFO_SIZE)) ? 1 : 0;
            break;
        case RX_FIFO_NOT_EMPTY:
            /* Rx fifo is not empty. */
            ret = handle->baseAddress->I2C_FIFO_STAT.BIT.rx_fifo_vld_num;
            break;
        default:
            break;
    }
    return ret;
}

/**
  * @brief Wait for the item status to be ready.
  * @param handle I2C handle.
  * @param checkItem The item to be checked.
  * @param opt Read or write flag.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType WaitStatusReady(I2C_Handle *handle, unsigned int checkItem, unsigned int opt)
{
    unsigned int preTick = DCL_SYSTICK_GetTick();
    unsigned int curTick;
    unsigned long long delta = 0;
    unsigned long long targetDelta = HAL_CRG_GetIpFreq(SYSTICK_BASE) / I2C_TICK_MS_DIV * handle->timeout;

    while (true) {
        if (handle->baseAddress->I2C_INTR_RAW.reg & I2C_ERROR_BIT_MASK) {
            SetErrorHandling(handle);
            return BASE_STATUS_ERROR;
        }
        
        /* Check the status of the item is ready. */
        if (CheckItemStatus(handle, checkItem, opt)) {
            if (checkItem == SLAVE_ADDRESS_MATCH) {
                /* Clear slave address match raw interrupt */
                handle->baseAddress->I2C_INTR_RAW.BIT.slv_addr_match_raw = BASE_CFG_SET;
            }
            return BASE_STATUS_OK;
        }

        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        if (delta >= targetDelta) { /* Check timeout. */
            break;
        }
        preTick = curTick;
    }
    return BASE_STATUS_TIMEOUT;
}

/**
  * @brief Set the sending data and operation commands.
  * @param handle I2C handle.
  * @param cmd Operation commands.
  * @param data Sending data.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SetTxFIFODataAndCmd(I2C_Handle *handle, I2C_CmdType cmd, unsigned char data)
{
    BASE_StatusType ret;
    unsigned int temp;

    ret = WaitStatusReady(handle, TX_FIFO_NOT_FULL, I2C_OPERATION_WRITE);
    if (ret != BASE_STATUS_OK) {
        return ret;
    }
    /* The 8 to 11 bits are the Timing Commands, and the 0 to 7 bits are the write data. */
    temp = (((unsigned int)cmd << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
    temp |= (((unsigned int)data << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
    handle->baseAddress->I2C_TX_FIFO.reg = temp; /* Sets the data and commands to be sent. */
    return BASE_STATUS_OK;
}

/**
  * @brief Send a write command to the slave device.
  * @param handle I2C handle.
  * @param index The number of I2C.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SendSlaveAddressWriteCmd(I2C_Handle *handle, unsigned int index)
{
    BASE_StatusType ret;
    unsigned char addr;
    /* Write slave address */
    if (handle->addrMode == I2C_10_BITS) { /* 10bit address Configuration */
        if (handle->transferCount == 0) {
            /* The first address of a 10-bit address configuration */
            addr = (unsigned char)((g_internalConfigParam[index].slaveAddress >> I2C_SLAVE_WRITE_ADDR_POS) &
                                   I2C_SLAVE_ADDR_MASK);
            ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
            /* The second address of the 10-bit address configuration */
            addr = (unsigned char)(g_internalConfigParam[index].slaveAddress & I2C_SLAVE_ADDR_MASK);
            ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
        } else {
            addr = (unsigned char)((g_internalConfigParam[index].slaveAddress >> I2C_SLAVE_WRITE_ADDR_POS) &
                                   I2C_SLAVE_ADDR_MASK);
            ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
        }
    } else { /* 7bit address Configuration */
        addr = (unsigned char)(g_internalConfigParam[index].slaveAddress & I2C_SLAVE_ADDR_MASK);
        ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Send a read command to the slave device.
  * @param handle I2C handle.
  * @param index The number of I2C.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SendSlaveAddressReadCmd(I2C_Handle *handle, unsigned int index)
{
    BASE_StatusType ret;
    unsigned char addr;
    /* Write slave address */
    if (handle->addrMode == I2C_10_BITS) { /* 10bit address Configuration */
        if (handle->transferCount == 0) {
            /* The first address of a 10-bit address configuration */
            addr = (unsigned char)((g_internalConfigParam[index].slaveAddress >> I2C_SLAVE_READ_FIX_ADDR_POS) &
                                   I2C_SLAVE_ADDR_MASK);
            ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
            /* The second address of the 10-bit address configuration */
            addr = (unsigned char)((g_internalConfigParam[index].slaveAddress >> I2C_SLAVE_READ_DEV_ADDR_POS) &
                                   I2C_SLAVE_ADDR_MASK);
            ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
        } else {
            addr = (unsigned char)((g_internalConfigParam[index].slaveAddress >> I2C_SLAVE_READ_FIX_ADDR_POS) &
                                   I2C_SLAVE_ADDR_MASK);
            ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
        }
    } else { /* 7bit address Configuration */
        addr = (unsigned char)((g_internalConfigParam[index].slaveAddress >> I2C_SLAVE_READ_DEV_ADDR_POS) &
                               I2C_SLAVE_ADDR_MASK);
        ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief I2C Parameter Configuration in blocking.
  * @param handle I2C handle.
  * @param transferStatus The status is used to indicate read or write.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType I2C_ConfigParametersAndStartBlocking(I2C_Handle *handle, unsigned int transferStatus)
{
    BASE_StatusType ret;
    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;

    handle->baseAddress->I2C_CTRL1.BIT.mst_start = (transferStatus == I2C_MASTER_STATUS) ? BASE_CFG_SET :
                                                    BASE_CFG_UNSET;
    if (transferStatus == I2C_SLAVE_STATUS) {
        return BASE_STATUS_OK;
    }
    /* Send I2C start */
    ret = SetTxFIFODataAndCmd(handle, I2C_CMD_S, 0); /* Sets the start command to be sent. */
    if (ret != BASE_STATUS_OK) {
        SetErrorHandling(handle);
        return ret;
    }
    return ret;
}

/**
  * @brief Master send stop command in blocking.
  * @param handle I2C handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType BlockingSendStopCommand(I2C_Handle *handle)
{
    BASE_StatusType ret;
    ret = SetTxFIFODataAndCmd(handle, I2C_CMD_P, 0);
    if (ret != BASE_STATUS_OK) {
        SetErrorHandling(handle);
        return ret;
    }
    /* Wait until all commands are executed. */
    ret = WaitStatusReady(handle, COMMAND_ALL_DONE, I2C_OPERATION_WRITE);
    handle->errorCode = ret;
    SetErrorHandling(handle);
    return ret;
}

/**
  * @brief The step of receive normal data in blocking as master.
  * @param handle I2C handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType BlockingMasterRxDataOptStepNormal(I2C_Handle *handle)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    while (handle->transferCount < handle->transferSize) {
        if (handle->transferCount == handle->transferSize - 1) {
            /* Reads the last frame of data without ack. */
            ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_RD_TNACK_S_TD_RNACK, 0);
        } else {
            ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_RD_TACK_S_TD_RACK, 0);
        }
        if (ret != BASE_STATUS_OK) {
            SetErrorHandling(handle);
            return ret;
        }
         /* Wait the RX FIFO is not empty. */
        ret = WaitStatusReady(handle, RX_FIFO_NOT_EMPTY, I2C_OPERATION_READ);
        if (ret != BASE_STATUS_OK) {
            SetErrorHandling(handle);
            return ret;
        }
        /* Obtains the data received from the RX FIFO. */
        *handle->transferBuff = handle->baseAddress->I2C_RX_FIFO.BIT.rx_fifo_rdata;
        handle->transferBuff++;
        handle->transferCount++;
    }
    return ret;
}

/**
  * @brief The step of transmit normal data in blocking as master.
  * @param handle I2C handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType BlockingMasterTxDataOptStepNormal(I2C_Handle *handle)
{
    BASE_StatusType ret;
    /* Sets data to be sent cyclically. */
    while (handle->transferCount < handle->transferSize) {
        ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, *handle->transferBuff);
        if (ret != BASE_STATUS_OK) {
            SetErrorHandling(handle);
            return ret;
        }
        handle->transferBuff++;
        handle->transferCount++;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief The step of receive normal data in blocking as slave.
  * @param handle I2C handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType BlockingSlaveRxDataOptStepNormal(I2C_Handle *handle)
{
    while (handle->transferCount < handle->transferSize) {
        /* Sets the data to be received. */
        if (SetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, 0) != BASE_STATUS_OK) {
            SetErrorHandling(handle);
            return BASE_STATUS_TIMEOUT;
        }
        if (WaitStatusReady(handle, RX_FIFO_NOT_EMPTY, I2C_OPERATION_READ) != BASE_STATUS_OK) {
            SetErrorHandling(handle);
            return BASE_STATUS_TIMEOUT;
        }
        /* Obtains the data received in the RX FIFO. */
        *handle->transferBuff = handle->baseAddress->I2C_RX_FIFO.BIT.rx_fifo_rdata;
        handle->transferBuff++;
        handle->transferCount++;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Checking Interrupts Caused by I2C Timing Errors.
  * @param handle I2C handle.
  * @param status Status of the I2C.
  * @retval true or false
  */
static bool IsInterruptErrorStatus(I2C_Handle *handle, unsigned int status)
{
    if (status & I2C_ERROR_BIT_MASK) {
        /* If the low level times out, the I2C bus is cleared and the bus is expected to be released. */
        if (status & I2C_SCL_LOW_TIMEOUT_MASK) {
            I2cBusClear(handle);
        }
        /* Disable */
        handle->errorCode = BASE_STATUS_ERROR;
        handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_DISABLE;
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
  * @brief Interrupt handle send start command.
  * @param handle I2C handle.
  * @param index The number of I2C.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType InterruptSendStart(I2C_Handle *handle, unsigned index)
{
    unsigned int temp;
    if (g_internalConfigParam[index].sendAddressStatus <= I2C_SEND_ADDR_STATUS_NONE) {
        return BASE_STATUS_OK;
    }

    if (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < I2C_MAX_FIFO_SIZE) {
        /* The 8 to 11 bits are the Timing Commands, and the 0 to 7 bits are the write data. */
        temp = (((unsigned int)I2C_CMD_S << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        handle->baseAddress->I2C_TX_FIFO.reg = temp; /* Sets the data and commands to be sent. */
    } else {
        return BASE_STATUS_ERROR;
    }
    switch (g_internalConfigParam[index].sendAddressStatus) {
        case I2C_SEND_ADDR_STATUS_WRITE:
            /* Send a write command to the slave. */
            if (SendSlaveAddressWriteCmd(handle, index) != BASE_STATUS_OK) {
                return BASE_STATUS_ERROR;
            }
            break;
        case I2C_SEND_ADDR_STATUS_READ:
            /* Send a read command to the slave. */
            if (SendSlaveAddressReadCmd(handle, index) != BASE_STATUS_OK) {
                return BASE_STATUS_ERROR;
            }
            break;
        default:
            break;
    }
    g_internalConfigParam[index].sendAddressStatus = I2C_SEND_ADDR_STATUS_NONE;
    return BASE_STATUS_OK;
}

/**
 * @brief I2C Interrupt done Handling
 * @param handle I2C handle.
 * @param status I2C interrupt raw status.
 * @retval None.
 */
static void InterruptAllDoneHandle(I2C_Handle *handle, unsigned int status)
{
    /* After all data transmission is complete, call the user's callback function. */
    unsigned int masterAllDone = status & I2C_INTR_RAW_ALL_CMD_DONE_MASK;
    unsigned int slaveReceiveStop = status & I2C_INTR_RAW_STOP_DET_MASK;
    unsigned int allDoneItFlag = (masterAllDone || slaveReceiveStop);
    if ((handle->transferCount >= handle->transferSize) && allDoneItFlag) {
        /* Clears interrupts and disables interrupt reporting to
           facilitate switching between different working modes. */
        handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_DISABLE;
        handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
        handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
        if (handle->userCallBack.RxCplCallback != NULL &&
           (handle->state == I2C_STATE_BUSY_MASTER_RX || handle->state == I2C_STATE_BUSY_SLAVE_RX)) {
            handle->userCallBack.RxCplCallback(handle); /* Invoke the RX callback processing function. */
        } else if (handle->userCallBack.TxCplCallback != NULL &&
                  (handle->state == I2C_STATE_BUSY_MASTER_TX || handle->state == I2C_STATE_BUSY_SLAVE_TX)) {
            handle->userCallBack.TxCplCallback(handle); /* Invoke the TX callback processing function. */
        }
        handle->state = I2C_STATE_READY;
    }
}

/**
  * @brief I2C interrupt TX handling
  * @param handle I2C handle.
  * @param index The number of I2C.
  * @retval None.
  */
static void InterruptMasterTxHandle(I2C_Handle *handle, unsigned int index)
{
    I2C_ASSERT_PARAM(handle != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_ASSERT_PARAM(handle->transferBuff != NULL);
    unsigned int temp;
    /* Send a start command to the slave. */
    if (InterruptSendStart(handle, index) != BASE_STATUS_OK) {
        return;
    }
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < I2C_MAX_FIFO_SIZE &&
           handle->transferCount < handle->transferSize) {
        /* Sets the data to be sent. */
        temp = (((unsigned int)I2C_CMD_M_TD_RACK_S_RD_TACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        temp |= ((unsigned int)(*handle->transferBuff) & I2C_TXFIFO_WDATA_MASK);
        handle->baseAddress->I2C_TX_FIFO.reg = temp; /* Sets the data and commands to be sent. */
        handle->transferBuff++;
        handle->transferCount++;
    }
}

/**
 * @brief I2C Interrupt RX Handling
 * @param handle I2C handle.
 * @param index The number of I2C.
 * @retval None.
 */
static void InterruptMasterRxHandle(I2C_Handle *handle, unsigned int index)
{
    I2C_ASSERT_PARAM(handle != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_ASSERT_PARAM(handle->transferBuff != NULL);
    /* Send a start command to the slave. */
    if (InterruptSendStart(handle, index) != BASE_STATUS_OK) {
        return;
    }
    /* The I2C controller fills in the receive command and starts to receive data. */
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < I2C_MAX_FIFO_SIZE &&
           g_internalConfigParam[index].txReadCmdCnt < handle->transferSize) {
        if (g_internalConfigParam[index].txReadCmdCnt == handle->transferSize - 1) {
            handle->baseAddress->I2C_TX_FIFO.reg =
                (((unsigned int)I2C_CMD_M_RD_TNACK_S_TD_RNACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        } else { /* Normal data transmission. */
            handle->baseAddress->I2C_TX_FIFO.reg =
                (((unsigned int)I2C_CMD_M_RD_TACK_S_TD_RACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        }
        g_internalConfigParam[index].txReadCmdCnt++;
    }
    /* Obtains the data received in the RX FIFO. */
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.rx_fifo_vld_num > 0 &&
           handle->transferCount < handle->transferSize) {
        *handle->transferBuff++ = handle->baseAddress->I2C_RX_FIFO.BIT.rx_fifo_rdata;
        handle->transferCount++;
    }
}

/**
  * @brief I2C interrupt slave TX handling
  * @param handle I2C handle.
  * @retval None.
  */
static void InterruptSlaveTxHandle(I2C_Handle *handle)
{
    I2C_ASSERT_PARAM(handle != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_ASSERT_PARAM(handle->transferBuff != NULL);
    unsigned int temp;
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < I2C_MAX_FIFO_SIZE &&
           handle->transferCount < handle->transferSize) {
        if (handle->transferCount == handle->transferSize - 1) { /* no need ack. */
            temp = (((unsigned int)I2C_CMD_M_RD_TNACK_S_TD_RNACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
            temp |= ((unsigned int)(*handle->transferBuff) & I2C_TXFIFO_WDATA_MASK);
            handle->baseAddress->I2C_TX_FIFO.reg = temp; /* Sets the data and commands to be sent. */
        } else { /* Normal data transmission. */
            temp = (((unsigned int)I2C_CMD_M_RD_TACK_S_TD_RACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
            temp |= ((unsigned int)(*handle->transferBuff) & I2C_TXFIFO_WDATA_MASK);
            handle->baseAddress->I2C_TX_FIFO.reg = temp; /* Sets the data and commands to be sent. */
        }
        handle->transferBuff++;
        handle->transferCount++;
    }
}

/**
  * @brief I2C interrupt slave RX handling
  * @param handle I2C handle.
  * @param index The number of I2C.
  * @retval None.
  */
static void InterruptSlaveRxHandle(I2C_Handle *handle, unsigned int index)
{
    I2C_ASSERT_PARAM(handle != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_ASSERT_PARAM(handle->transferBuff != NULL);
    /* Set the data receiving command. */
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < I2C_MAX_FIFO_SIZE &&
           g_internalConfigParam[index].txReadCmdCnt < handle->transferSize) {
            handle->baseAddress->I2C_TX_FIFO.reg =
                (((unsigned int)I2C_CMD_M_TD_RACK_S_RD_TACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        g_internalConfigParam[index].txReadCmdCnt++;
    }
    /* Obtained data from RX FIFO. */
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.rx_fifo_vld_num > 0 &&
           handle->transferCount < handle->transferSize) {
        *handle->transferBuff = handle->baseAddress->I2C_RX_FIFO.BIT.rx_fifo_rdata;
        handle->transferBuff++;
        handle->transferCount++;
    }
}

/**
  * @brief ICallback function corresponding to the interrupt processing function.
  * @param handle I2C handle.
  * @param status Status of the I2C.
  * @param index The number of I2C.
  * @retval None.
  */
static void InterruptHandle(I2C_Handle *handle, unsigned int status, unsigned int index)
{
    if (handle->state == I2C_STATE_BUSY_MASTER_TX) {
        InterruptMasterTxHandle(handle, index); /* Transfer data as a host. */
        return;
    } else if (handle->state == I2C_STATE_BUSY_MASTER_RX) {
        InterruptMasterRxHandle(handle, index); /* Receive data as a host. */
        return;
    } else if (handle->state == I2C_STATE_BUSY_SLAVE_TX) {
        if (status & I2C_INTR_RAW_SLAVE_ADDR_MATCH_MASK) {
            /* Set TX FIFO the waterline. */
            handle->baseAddress->I2C_INTR_EN.BIT.tx_fifo_not_full_en = BASE_CFG_SET;
        }
        if (handle->baseAddress->I2C_RX_ADDR.BIT.rx_rw == I2C_OPERATION_READ) {
            InterruptSlaveTxHandle(handle); /* Transfer data as slave. */
        }
        return;
    } else if (handle->state == I2C_STATE_BUSY_SLAVE_RX) {
        if (status & I2C_INTR_RAW_SLAVE_ADDR_MATCH_MASK) {
            /* Set TX FIFO the waterline. */
            handle->baseAddress->I2C_INTR_EN.BIT.tx_fifo_not_full_en = BASE_CFG_SET;
            /* Set RX FIFO the waterline. */
            handle->baseAddress->I2C_INTR_EN.BIT.rx_fifo_not_empty_en = BASE_CFG_SET;
        }
        if (handle->baseAddress->I2C_RX_ADDR.BIT.rx_rw == I2C_OPERATION_WRITE) {
            InterruptSlaveRxHandle(handle, index); /* Receive data as slave. */
        }
        return;
    }
    handle->errorCode = BASE_STATUS_ERROR;
    /* Clears interrupts and disables interrupt reporting to
       facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->state = I2C_STATE_READY; /* Changing the I2C Bus Status. */
    if (handle->userCallBack.ErrorCallback != NULL) {
        handle->userCallBack.ErrorCallback(handle);
    }
}

/**
  * @brief DMA Command Configuration.
  * @param handle I2C handle.
  * @param cmd The command type of I2C.
  * @param size The number of the data to be receiving or sending.
  * @retval Value of the command.
  */
static unsigned int DmaConfigCommandData(I2C_Handle *handle, I2C_CmdType cmd, unsigned int size)
{
    unsigned int temp;
    /* Sets the command data. */
    if ((cmd == I2C_CMD_M_RD_TACK_S_TD_RACK) && (size == 1) && (handle->transferCount >= handle->transferSize)) {
        temp = (((unsigned int)I2C_CMD_M_RD_TNACK_S_TD_RNACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
    } else {
        temp = (((unsigned int)cmd << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
    }
    return temp;
}

/**
  * @brief Config commands and data in dma as master.
  * @param handle I2C handle.
  * @param txBuff Address of the data buff to be receiving or sending.
  * @param cmd The command type of I2C.
  * @param size The number of the data to be receiving or sending.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType DmaMasterConfigDataAndCmd(I2C_Handle *handle, unsigned int *txBuff, I2C_CmdType cmd,
                                                 unsigned int size)
{
    unsigned int temp;
    unsigned int *tempTxBuff = txBuff;
    unsigned char *tempsrcTxBuff = (unsigned char*)handle->transferBuff;
    unsigned int tempSize = size;
    while (tempSize) {
        /* Sets the command data. */
        temp = DmaConfigCommandData(handle, cmd, tempSize);
        /* Sets the normal data. */
        if (cmd == I2C_CMD_M_TD_RACK_S_RD_TACK) {
            temp |= (((unsigned int)*tempsrcTxBuff << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
            tempsrcTxBuff++;
        } else if ((cmd == I2C_CMD_M_RD_TACK_S_TD_RACK) && (tempSize > 1)) {
            temp |= ((0x0 << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
        }
        *tempTxBuff = temp; /* Set the combined data. */
        tempTxBuff++;
        tempSize--;
        temp = 0;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Config commands and data in dma as slave.
  * @param handle I2C handle.
  * @param txBuff Address of the data buff to be receiving or sending.
  * @param cmd The command type of I2C.
  * @param size The number of the data to be receiving or sending.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType DmaSlaveConfigDataAndCmd(I2C_Handle *handle, unsigned int *txBuff, I2C_CmdType cmd,
                                                unsigned int size)
{
    unsigned int temp;
    unsigned int *tempTxBuff = txBuff;
    unsigned char *tempsrcTxBuff = (unsigned char*)handle->transferBuff;
    unsigned int tempSize = size;
    while (tempSize) {
        /* Sets the command data. */
        temp = DmaConfigCommandData(handle, cmd, tempSize);
        /* Sets the normal data. */
        if (cmd == I2C_CMD_M_RD_TACK_S_TD_RACK) {
            temp |= (((unsigned int)*tempsrcTxBuff << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
            tempsrcTxBuff++;
        } else if (cmd == I2C_CMD_M_TD_RACK_S_RD_TACK) {
            temp |= ((0x0 << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
        }
        *tempTxBuff = temp; /* Set the combined data. */
        tempTxBuff++;
        tempSize--;
        temp = 0;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief I2C DMA Error Handling.
  * @param handle I2C handle.
  * @retval None.
  */
static void I2C_DmaErrorHandle(I2C_Handle *handle)
{
    /* Some settings when an error occurs. */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_UNSET;
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    handle->errorCode = BASE_STATUS_ERROR;
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
static void I2C_DmaDoneHandle(I2C_Handle *handle)
{
    /* Disable the DMA operation and configure parameters. */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_UNSET;
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    /* Call the corresponding callback function. */
    if (handle->state == I2C_STATE_BUSY_MASTER_RX || handle->state == I2C_STATE_BUSY_SLAVE_RX) {
        if (handle->userCallBack.RxCplCallback != NULL) {
            handle->userCallBack.RxCplCallback(handle);
        }
    } else if (handle->state == I2C_STATE_BUSY_MASTER_TX || handle->state == I2C_STATE_BUSY_SLAVE_TX) {
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
static void DmaWaitHandleFinish(I2C_Handle *handle)
{
    unsigned int intrRwa;
    unsigned int preTick;
    unsigned int curTick;
    unsigned long long delta;
    unsigned long long targetDelta;

    delta = 0;
    preTick = DCL_SYSTICK_GetTick();
    /* Set the timeout threshold to 10000ms. */
    targetDelta = HAL_CRG_GetIpFreq(SYSTICK_BASE) / I2C_TICK_MS_DIV * handle->timeout;

    while (true) {
        /* Waiting for the last DMA transfer to complete. */
        intrRwa = handle->baseAddress->I2C_INTR_RAW.reg;
        /* Check for errors. */
        if ((intrRwa & (I2C_INTR_RAW_ARB_LOST_MASK | I2C_INTR_RAW_ACK_BIT_UNMATCH_MASK |
                        I2C_INTR_RAW_SLAVE_ACK_UNMATCH_MASK)) > 0) {
            I2C_DmaErrorHandle(handle);
            break;
        }
        /* DMA transfer completed normally. */
        if ((intrRwa & (I2C_INTR_RAW_ALL_CMD_DONE_MASK | I2C_INTR_RAW_STOP_DET_MASK)) > 0) {
            I2C_DmaDoneHandle(handle);
            break;
        }
        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        if (delta >= targetDelta) { /* Check timeout. */
            return;
        }
        preTick = curTick;
    }
}

/**
  * @brief The I2C uses the DMA completion callback function registered by the DMA module.
  * @param handle I2C handle.
  * @retval None.
  */
static void DmaOptStepNormalFinishFun(void *handle)
{
    I2C_ASSERT_PARAM(handle != NULL);
    I2C_Handle *i2cHandle = (I2C_Handle *)(handle);
    I2C_ASSERT_PARAM(IsI2CInstance(i2cHandle->baseAddress));

    BASE_StatusType ret = BASE_STATUS_OK;
    unsigned int tempOnceTransferSize;
    unsigned int index;
    unsigned int offset;

    /* Determine which I2C is used. */
    index = (i2cHandle->baseAddress == I2C0) ? I2C_INTERFACE_INDEX_0 : I2C_INTERFACE_INDEX_1;
    offset = i2cHandle->transferCount % I2C_ONCE_TRANS_MAX_NUM;
    i2cHandle->transferBuff += (offset == 0) ? I2C_ONCE_TRANS_MAX_NUM : offset; /* Update Transferred Data. */

    if (i2cHandle->transferCount < i2cHandle->transferSize) {
        /* Determine the amount of data transmitted at a time. */
        tempOnceTransferSize = (g_dmaTransferSize >= I2C_ONCE_TRANS_MAX_NUM) ? I2C_ONCE_TRANS_MAX_NUM :
                               g_dmaTransferSize;
        g_dmaTransferSize -= tempOnceTransferSize;
        i2cHandle->transferCount += tempOnceTransferSize;
        /* Configuring the I2C Timing */
        if (i2cHandle->state == I2C_STATE_BUSY_MASTER_RX) {
            ret = DmaMasterReadData(i2cHandle, tempOnceTransferSize, index);
        } else if (i2cHandle->state == I2C_STATE_BUSY_MASTER_TX) {
            ret = DmaMasterWriteData(i2cHandle, tempOnceTransferSize, index);
        } else if (i2cHandle->state == I2C_STATE_BUSY_SLAVE_RX) {
            ret = DmaSlaveReadData(i2cHandle, tempOnceTransferSize, index);
        } else if (i2cHandle->state == I2C_STATE_BUSY_SLAVE_TX) {
            ret = DmaSlaveWriteData(i2cHandle, tempOnceTransferSize, index);
        }
        /* Check whether errors occur. */
        if (ret != BASE_STATUS_OK) {
            SetErrorHandling(handle);
        }
        return;
    }
    SetTxFIFODataAndCmd(i2cHandle, I2C_CMD_P, 0);
    DmaWaitHandleFinish(i2cHandle);
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
    /* Disable the interrupt and call the error callback function. */
    i2cHandle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    i2cHandle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    i2cHandle->errorCode = BASE_STATUS_ERROR;
    if (i2cHandle->userCallBack.ErrorCallback != NULL) {
        i2cHandle->userCallBack.ErrorCallback(i2cHandle);
    }
    /* Stop DMA channel transfer. */
    HAL_DMA_StopChannel(i2cHandle->dmaHandle, i2cHandle->txDmaCh);
    if (i2cHandle->state == I2C_STATE_BUSY_MASTER_RX || i2cHandle->state == I2C_STATE_BUSY_SLAVE_RX) {
        HAL_DMA_StopChannel(i2cHandle->dmaHandle, i2cHandle->rxDmaCh);
    }
    i2cHandle->state = I2C_STATE_READY;
}

/**
  * @brief Receive data as master by the DMA module.
  * @param handle I2C handle.
  * @param size Number of the data to be transmitted.
  * @param index The number of I2C.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType DmaMasterReadData(I2C_Handle *handle, unsigned int size, unsigned int index)
{
    /* Combine commands and data. */
    DmaMasterConfigDataAndCmd(handle, (unsigned int *)g_internalTxBuffDMA[index], I2C_CMD_M_RD_TACK_S_TD_RACK, size);
    
    /* Configuring the DMA Callback Function. */
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelFinishCallBack = NULL;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelErrorCallBack = DmaErrorHandlerFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->rxDmaCh].ChannelFinishCallBack = DmaOptStepNormalFinishFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->rxDmaCh].ChannelErrorCallBack = DmaErrorHandlerFun;
    /* Start the DMA for data transmission. */
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)g_internalTxBuffDMA[index],
                        (uintptr_t)&(handle->baseAddress->I2C_TX_FIFO.reg),
                        size, handle->txDmaCh) != BASE_STATUS_OK) {
        handle->state = I2C_STATE_READY;
        return BASE_STATUS_ERROR;
    }
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)&(handle->baseAddress->I2C_RX_FIFO),
                        (uintptr_t)handle->transferBuff, size,
                        handle->rxDmaCh) != BASE_STATUS_OK) {
        handle->state = I2C_STATE_READY;
        return BASE_STATUS_ERROR;
    }
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_WRITE_READ;
    return BASE_STATUS_OK;
}

/**
  * @brief Transmit data as master by the DMA module.
  * @param handle I2C handle.
  * @param size Number of the data to be transmitted.
  * @param index The number of I2C.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType DmaMasterWriteData(I2C_Handle *handle, unsigned int size, unsigned int index)
{
    /* Combine commands and data. */
    DmaMasterConfigDataAndCmd(handle, (unsigned int *)g_internalTxBuffDMA[index], I2C_CMD_M_TD_RACK_S_RD_TACK, size);

    /* Configuring the DMA Callback Function. */
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelFinishCallBack = DmaOptStepNormalFinishFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelErrorCallBack = DmaErrorHandlerFun;
    /* Start the DMA for data transmission. */
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)g_internalTxBuffDMA[index],
                        (uintptr_t)&(handle->baseAddress->I2C_TX_FIFO.reg),
                        size, handle->txDmaCh) != BASE_STATUS_OK) {
        handle->state = I2C_STATE_READY;
        return BASE_STATUS_ERROR;
    }
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_WRITE;
    return BASE_STATUS_OK;
}

/**
  * @brief Receive data as slave by the DMA module.
  * @param handle I2C handle.
  * @param size Number of the data to be transmitted.
  * @param index The number of I2C.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType DmaSlaveReadData(I2C_Handle *handle, unsigned int size, unsigned int index)
{
    /* Combine commands and data. */
    DmaSlaveConfigDataAndCmd(handle, (unsigned int *)g_internalTxBuffDMA[index], I2C_CMD_M_TD_RACK_S_RD_TACK, size);

    /* Configuring the DMA Callback Function. */
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelFinishCallBack = NULL;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelErrorCallBack = DmaErrorHandlerFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->rxDmaCh].ChannelFinishCallBack = DmaOptStepNormalFinishFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->rxDmaCh].ChannelErrorCallBack = DmaErrorHandlerFun;
    /* Start the DMA for data transmission. */
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)&(handle->baseAddress->I2C_RX_FIFO),
                        (uintptr_t)handle->transferBuff, size,
                        handle->rxDmaCh) != BASE_STATUS_OK) {
        handle->state = I2C_STATE_READY;
        return BASE_STATUS_ERROR;
    }
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)g_internalTxBuffDMA[index],
                        (uintptr_t)&(handle->baseAddress->I2C_TX_FIFO),
                        size, handle->txDmaCh) != BASE_STATUS_OK) {
        handle->state = I2C_STATE_READY;
        return BASE_STATUS_ERROR;
    }
        handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_WRITE_READ;
    return BASE_STATUS_OK;
}

/**
  * @brief Transmit data as slave by the DMA module.
  * @param handle I2C handle.
  * @param size Number of the data to be transmitted.
  * @param index The number of I2C.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType DmaSlaveWriteData(I2C_Handle *handle, unsigned int size, unsigned int index)
{
    /* Combine commands and data. */
    DmaSlaveConfigDataAndCmd(handle, (unsigned int *)g_internalTxBuffDMA[index], I2C_CMD_M_RD_TACK_S_TD_RACK, size);
    /* Configure DMA Parameters */
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelFinishCallBack = DmaOptStepNormalFinishFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelErrorCallBack = DmaErrorHandlerFun;
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)g_internalTxBuffDMA[index],
                        (uintptr_t)&(handle->baseAddress->I2C_TX_FIFO.reg), size, handle->txDmaCh) != BASE_STATUS_OK) {
        handle->state = I2C_STATE_READY;
        return BASE_STATUS_ERROR;
    }
        handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_WRITE;
    return BASE_STATUS_OK;
}

/**
  * @brief Transmit data by the DMA module.
  * @param handle I2C handle.
  * @param index The number of I2C.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType I2cTransferDataDma(I2C_Handle *handle, unsigned int index)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    unsigned int tempOnceTransferSize;

    g_dmaTransferSize = handle->transferSize;
    /* Determine the amount of data transmitted at a time. */
    tempOnceTransferSize = (g_dmaTransferSize >= I2C_ONCE_TRANS_MAX_NUM) ? I2C_ONCE_TRANS_MAX_NUM :
                            g_dmaTransferSize;
    g_dmaTransferSize -= tempOnceTransferSize;
    handle->transferCount += tempOnceTransferSize;
    /* Configuring the I2C Timing */
    if (handle->state == I2C_STATE_BUSY_MASTER_RX) {
        ret = DmaMasterReadData(handle, tempOnceTransferSize, index);
    } else if (handle->state == I2C_STATE_BUSY_MASTER_TX) {
        ret = DmaMasterWriteData(handle, tempOnceTransferSize, index);
    } else if (handle->state == I2C_STATE_BUSY_SLAVE_RX) {
        ret = DmaSlaveReadData(handle, tempOnceTransferSize, index);
    } else if (handle->state == I2C_STATE_BUSY_SLAVE_TX) {
        ret = DmaSlaveWriteData(handle, tempOnceTransferSize, index);
    }
    /* Check whether errors occur. */
    if (ret != BASE_STATUS_OK) {
        SetErrorHandling(handle);
    }
    return ret;
}

/**
  * @brief As Slave Multiplex Interrupt Write or Read.
  * @param handle I2C handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType I2C_SlaveMultiplexIT(I2C_Handle *handle)
{
    unsigned int index = 0;
    /* Determine which I2C is used. */
    index = (handle->baseAddress == I2C0) ? I2C_INTERFACE_INDEX_0 : I2C_INTERFACE_INDEX_1;
    /* Parameter Settings. */
    g_internalConfigParam[index].txReadCmdCnt = 0;
    g_internalConfigParam[index].sendAddressStatus = I2C_SEND_ADDR_STATUS_NONE;
    /* Clean interrupt */
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;
    /* Enable interrupt */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_CFG_INTERRUPT_SLAVE;
    return BASE_STATUS_OK;
}

/**
  * @brief Initializing the I2C Module.
  * @param handle I2C handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_Init(I2C_Handle *handle)
{
    unsigned int clockFreq;
    unsigned int val;
    unsigned int tempReg;
    unsigned int temp;
    unsigned int tempSclLowTime;
    unsigned int tempSclHighTime;

    I2C_ASSERT_PARAM(handle != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));

    clockFreq = HAL_CRG_GetIpFreq((void *)handle->baseAddress);
    if (CheckAllInitParameters(handle, clockFreq) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    handle->state = I2C_STATE_BUSY;
    /* Clears interrupts and disables interrupt reporting to facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Set SDA and SCL glitch filtering time. */
    handle->baseAddress->I2C_FILTER.BIT.spike_filter_time = handle->handleEx.spikeFilterTime;
    /* Set SCL high and low duratiom time */
    tempSclLowTime = I2C_FREQ_LOW_PARAMTER + handle->handleEx.spikeFilterTime;
    tempSclHighTime = I2C_FREQ_HIGH_PARAMTER + handle->handleEx.spikeFilterTime;
    if (handle->freq <= I2C_STANDARD_FREQ_TH) {
        /* scl_high_time = (fclk_i2c/fSCL) x 0.5 - 8 - spike_filter_time. */
        val = clockFreq / (handle->freq * 2) - tempSclHighTime; /* The clockFreq / (freq * 2) = cloclFreq/0.5/freq. */
        /* scl_low_time = (fclk_i2c/fSCL) x 0.5 - 9 - spike_filter_time. */
        val = ((val - 1) & LOW_HOLD_TIME_MASK) | ((val << HIGH_HOLD_TIME_POS) & HIGH_HOLD_TIME_MASK);
    } else {
        /* scl_high_time = (fclk_i2c/fSCL) x 0.36 - 8 - spike_filter_time. (n/100*36)=0.36n. */
        val = ((((clockFreq / 100) * 36) / handle->freq) - tempSclHighTime) << HIGH_HOLD_TIME_POS;
        /* scl_low_time = (fclk_i2c/fSCL) x 0.64 - 9 - spike_filter_time. (n/100*64)=0.64n. */
        val |= (((((clockFreq / 100) * 64) / handle->freq) - tempSclLowTime) & LOW_HOLD_TIME_MASK);
    }
    handle->baseAddress->I2C_SCL_CFG.reg = val;

    /* Set sda hold duration.The value is fixed to 0xa */
    temp = ((handle->sdaHoldTime & 0x0000FFFF) << I2C_SDA_HOLD_DURATION_POS);
    tempReg = (handle->handleEx.sdaDelayTime & 0x0000000F) | temp;
    handle->baseAddress->I2C_SDA_CFG.reg = tempReg;

    /* Set I2C TX FIFO watermark */
    handle->baseAddress->I2C_TX_WATERMARK.BIT.tx_watermark = handle->txWaterMark;
    /* Set I2C RX FIFO watermark */
    handle->baseAddress->I2C_RX_WATERMARK.BIT.rx_watermark = handle->rxWaterMark;
    handle->baseAddress->I2C_MODE.BIT.mst_slv_function = handle->functionMode;
    handle->baseAddress->I2C_MODE.BIT.rack_mode = handle->ignoreAckFlag;

    if (handle->functionMode == I2C_MODE_SELECT_SLAVE_ONLY || handle->functionMode == I2C_MODE_SELECT_MASTER_SLAVE) {
        /* Sets the first own address of the slave. */
        handle->baseAddress->I2C_OWN_ADDR.BIT.own_address = handle->slaveOwnAddress;
        handle->baseAddress->I2C_OWN_ADDR.BIT.i2c_general_call_en = handle->generalCallMode;
        /* Sets the second own address of the slave. */
        if (handle->handleEx.slaveOwnXmbAddressEnable == BASE_CFG_ENABLE) {
            handle->baseAddress->XMB_DEV_ADDR.BIT.xmb_address_en = BASE_CFG_ENABLE;
            handle->baseAddress->XMB_DEV_ADDR.BIT.xmb_address = handle->handleEx.slaveOwnXmbAddress;
        }
    }
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
    /* Disable */
    handle->baseAddress->I2C_MODE.BIT.mst_slv_function = I2C_MODE_SELECT_NONE;
    /* Clears interrupts and disables interrupt reporting to
       facilitate switching between different working modes. */
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
  * @brief Callback Function Registration.
  * @param handle I2C handle.
  * @param callbackID Callback function ID.
  * @param pcallback Pointer to the address of the registered callback function.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_RegisterCallback(I2C_Handle *handle, I2C_CallbackId callbackID, I2C_CallbackFunType pcallback)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    /* Check the parameter validity. */
    I2C_ASSERT_PARAM(handle != NULL && pcallback != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_ASSERT_PARAM(pcallback != NULL);

    if (handle->state == I2C_STATE_READY) {
        switch (callbackID) {
            case I2C_MASTER_TX_COMPLETE_CB_ID :
            case I2C_SLAVE_TX_COMPLETE_CB_ID :
                handle->userCallBack.TxCplCallback = pcallback; /* Invoke the transfer completion callback function. */
                break;
            case I2C_MASTER_RX_COMPLETE_CB_ID :
            case I2C_SLAVE_RX_COMPLETE_CB_ID :
                handle->userCallBack.RxCplCallback = pcallback; /* Invoke the receive completion callback function. */
                break;
            case I2C_ERROR_CB_ID :
                handle->userCallBack.ErrorCallback = pcallback; /* Invoke the error callback function. */
                break;
            default:
                ret = BASE_STATUS_ERROR;
                handle->errorCode = BASE_STATUS_ERROR;
                break;
        }
    } else {  /* If I2C state is not ready, don't invoke callback function. */
        ret = BASE_STATUS_ERROR;
        handle->errorCode = BASE_STATUS_ERROR;
    }
    return ret;
}

/**
  * @brief Receiving data in blocking mode.
  * @param handle I2C handle.
  * @param devAddr Slave Device Address.
  * @param rData Address of the data buff to be receiving.
  * @param dataSize Number of the data to be receiving.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_MasterReadBlocking(I2C_Handle *handle, unsigned short devAddr, unsigned char *rData,
                                           unsigned int dataSize, unsigned int timeout)
{
    I2C_ASSERT_PARAM(handle != NULL && rData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_PARAM_CHECK_WITH_RET(devAddr <= I2C_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(timeout > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    BASE_StatusType ret;
    unsigned int index = 0;
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Determine which I2C is used. */
    index = (handle->baseAddress == I2C0) ? I2C_INTERFACE_INDEX_0 : I2C_INTERFACE_INDEX_1;
    /* Waiting for the i2c bus to be idle. */
    ret = WaitStatusReady(handle, I2C_BUS_IS_FREE, I2C_SEND_ADDR_STATUS_WRITE);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }
    /* Configuring I2C Parameters. */
    handle->state = I2C_STATE_BUSY_MASTER_RX;
    handle->transferBuff = rData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    handle->timeout = timeout;
    SetSlaveDevAddr(handle, devAddr);

    /* step1 : Parameter Settings and startup Control. */
    ret = I2C_ConfigParametersAndStartBlocking(handle, I2C_MASTER_STATUS);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }
    /* step2 : Send slave address and read command. */
    ret = SendSlaveAddressReadCmd(handle, index);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }
    /* step3 : start receive data. */
    ret = BlockingMasterRxDataOptStepNormal(handle);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }
    /* step4 ：send stop CMD. */
    ret = BlockingSendStopCommand(handle);
    
    return ret;
}

/**
  * @brief Send data in blocking mode.
  * @param handle I2C handle.
  * @param devAddr Slave Device Address.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_MasterWriteBlocking(I2C_Handle *handle, unsigned short devAddr, unsigned char *wData,
                                            unsigned int dataSize, unsigned int timeout)
{
    I2C_ASSERT_PARAM(handle != NULL && wData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_PARAM_CHECK_WITH_RET(devAddr <= I2C_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(timeout > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    BASE_StatusType ret = BASE_STATUS_OK;
    unsigned int index;
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Determine which I2C is used. */
    index = (handle->baseAddress == I2C0) ? I2C_INTERFACE_INDEX_0 : I2C_INTERFACE_INDEX_1;

    /* Configuring I2C Parameters. */
    handle->state = I2C_STATE_BUSY_MASTER_TX;
    handle->transferBuff = wData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    handle->timeout = timeout;
    SetSlaveDevAddr(handle, devAddr);

    /* Waiting for the i2c bus to be idle. */
    ret = WaitStatusReady(handle, I2C_BUS_IS_FREE, I2C_SEND_ADDR_STATUS_READ);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }

    /* step1 : Parameter Settings and startup Control. */
    ret = I2C_ConfigParametersAndStartBlocking(handle, I2C_MASTER_STATUS);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }
    /* step2 : send slave addr */
    ret = SendSlaveAddressWriteCmd(handle, index);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }
    /* step3 : Send to slave data */
    ret = BlockingMasterTxDataOptStepNormal(handle);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }
    /* step4 : send stop CMD */
    ret = BlockingSendStopCommand(handle);

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
BASE_StatusType HAL_I2C_SlaveReadBlocking(I2C_Handle *handle, unsigned char *rData,
                                          unsigned int dataSize, unsigned int timeout)
{
    I2C_ASSERT_PARAM(handle != NULL && rData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(timeout > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    BASE_StatusType ret = BASE_STATUS_OK;
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Configuring I2C Parameters. */
    handle->state = I2C_STATE_BUSY_SLAVE_RX;
    handle->transferBuff = rData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    handle->timeout = timeout;

    /* step1 : Parameter Settings. */
    I2C_ConfigParametersAndStartBlocking(handle, I2C_SLAVE_STATUS);
    /* step2 : Waiting for slave address match. */
    ret = WaitStatusReady(handle, SLAVE_ADDRESS_MATCH, I2C_OPERATION_WRITE);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }
    /* step3 : Slave receives data from the master device. */
    ret = BlockingSlaveRxDataOptStepNormal(handle);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }
    /* step4 : Send stop CMD */
    ret = BlockingSendStopCommand(handle);

    return ret;
}

/**
  * @brief Send data in blocking mode as slave.
  * @param handle I2C handle.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_SlaveWriteBlocking(I2C_Handle *handle, unsigned char *wData,
                                           unsigned int dataSize, unsigned int timeout)
{
    I2C_ASSERT_PARAM(handle != NULL && wData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(timeout > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    BASE_StatusType ret = BASE_STATUS_OK;
    /* Clean interrupt */
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Configuring Transmission Parameters of I2C. */
    handle->state = I2C_STATE_BUSY_SLAVE_TX;
    handle->transferBuff = wData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    handle->timeout = timeout;

    /* Parameter Settings. */
    I2C_ConfigParametersAndStartBlocking(handle, I2C_SLAVE_STATUS);

    /* step1 : Waiting for slave address match. */
    ret = WaitStatusReady(handle, SLAVE_ADDRESS_MATCH, I2C_OPERATION_READ);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }
    /* step2 : Slave send data to the master device. */
    while (handle->transferCount < (handle->transferSize - 1)) {
        ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_RD_TACK_S_TD_RACK, *handle->transferBuff);
        if (ret != BASE_STATUS_OK) {
            handle->errorCode = ret;
            SetErrorHandling(handle);
            return ret;
        }
        handle->transferBuff++;
        handle->transferCount++;
    }
    /* step3 : Slave send last data without ack to the master device. */
    if (handle->transferCount == (handle->transferSize - 1)) {
        ret = SetTxFIFODataAndCmd(handle, I2C_CMD_M_RD_TNACK_S_TD_RNACK, *handle->transferBuff);
        if (ret != BASE_STATUS_OK) {
            handle->errorCode = ret;
            SetErrorHandling(handle);
            return ret;
        }
    }
    /* step4 : send stop CMD */
    ret = BlockingSendStopCommand(handle);

    return ret;
}

/**
  * @brief Receiving data in interrupts mode.
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

    unsigned int index;
    /* Determine which I2C is used. */
    index = (handle->baseAddress == I2C0) ? I2C_INTERFACE_INDEX_0 : I2C_INTERFACE_INDEX_1;

    /* Configuring I2C Parameters. */
    handle->state = I2C_STATE_BUSY_MASTER_RX;
    handle->transferBuff = rData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    SetSlaveDevAddr(handle, devAddr);
    g_internalConfigParam[index].txReadCmdCnt = 0;
    g_internalConfigParam[index].sendAddressStatus = I2C_SEND_ADDR_STATUS_READ;

    /* Clean interrupt */
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_SET;

    /* Enable interrupt */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_CFG_INTERRUPT_MASTER_RX;

    return BASE_STATUS_OK;
}

/**
  * @brief Send data in interrupts mode.
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

    unsigned int index;
    /* Determine which I2C is used. */
    index = (handle->baseAddress == I2C0) ? I2C_INTERFACE_INDEX_0 : I2C_INTERFACE_INDEX_1;

    /* Configuring I2C Parameters. */
    handle->state = I2C_STATE_BUSY_MASTER_TX;
    handle->transferBuff = wData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    SetSlaveDevAddr(handle, devAddr);
    g_internalConfigParam[index].txReadCmdCnt = 0;
    g_internalConfigParam[index].sendAddressStatus = I2C_SEND_ADDR_STATUS_WRITE;

    /* Clean interrupt */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_RAW_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_SET;

    /* Enable interrupt */
    handle->baseAddress->I2C_INTR_EN.reg = I2C_CFG_INTERRUPT_MASTER_TX;

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
    BASE_StatusType ret = BASE_STATUS_OK;

    I2C_ASSERT_PARAM(handle != NULL && rData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    /* Configuring Transmission Parameters of I2C. */
    handle->state = I2C_STATE_BUSY_SLAVE_RX;
    handle->transferBuff = rData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;

    /* Configuring the I2C Timing */
    ret = I2C_SlaveMultiplexIT(handle);

    return ret;
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
    BASE_StatusType ret = BASE_STATUS_OK;

    I2C_ASSERT_PARAM(handle != NULL && wData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    /* Configuring Transmission Parameters of I2C. */
    handle->state = I2C_STATE_BUSY_SLAVE_TX;
    handle->transferBuff = wData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    
    /* Configuring the I2C Timing */
    ret = I2C_SlaveMultiplexIT(handle);
    return ret;
}

/**
  * @brief Receiving data in DMA mode.
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
    /* Check the DMA transfer handle and channel. */
    I2C_ASSERT_PARAM(handle->dmaHandle != NULL);
    I2C_PARAM_CHECK_WITH_RET((handle->txDmaCh < CHANNEL_MAX_NUM), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET((handle->rxDmaCh < CHANNEL_MAX_NUM), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET((handle->rxDmaCh != handle->txDmaCh), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(devAddr <= I2C_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    BASE_StatusType ret = BASE_STATUS_OK;

    unsigned int index;
    /* Determine which I2C is used. */
    index = (handle->baseAddress == I2C0) ? I2C_INTERFACE_INDEX_0 : I2C_INTERFACE_INDEX_1;
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Configuring I2C Parameters. */
    handle->state = I2C_STATE_BUSY_MASTER_RX;
    handle->transferBuff = rData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    SetSlaveDevAddr(handle, devAddr);

    /* Waiting for the i2c bus to be idle. */
    ret = WaitStatusReady(handle, I2C_BUS_IS_FREE, I2C_SEND_ADDR_STATUS_READ);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }

   /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_SET;
    ret = SetTxFIFODataAndCmd(handle, I2C_CMD_S, 0); /* Sets the start command to be sent. */
    if (ret != BASE_STATUS_OK) {
        SetErrorHandling(handle);
        return ret;
    }
    ret = SendSlaveAddressReadCmd(handle, index); /* Send Address to Slave. */
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }
    ret = I2cTransferDataDma(handle, index);
    return ret;
}

/**
  * @brief Send data in DMA mode.
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
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    /* Check the DMA transfer handle and channel. */
    I2C_ASSERT_PARAM(handle->dmaHandle != NULL);
    I2C_PARAM_CHECK_WITH_RET((handle->txDmaCh < CHANNEL_MAX_NUM), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(devAddr <= I2C_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    BASE_StatusType ret = BASE_STATUS_OK;
    unsigned int index;
    /* Determine which I2C is used. */
    index = (handle->baseAddress == I2C0) ? I2C_INTERFACE_INDEX_0 : I2C_INTERFACE_INDEX_1;
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;

    /* Waiting for the i2c bus to be idle. */
    ret = WaitStatusReady(handle, I2C_BUS_IS_FREE, I2C_SEND_ADDR_STATUS_READ);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SetErrorHandling(handle);
        return ret;
    }
    /* Configuring I2C Parameters. */
    handle->state = I2C_STATE_BUSY_MASTER_TX;
    handle->transferBuff = wData;
    handle->transferSize = dataSize;
    handle->transferCount = 0;
    SetSlaveDevAddr(handle, devAddr);

    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_SET;
    /* Send I2C start */
    ret = SetTxFIFODataAndCmd(handle, I2C_CMD_S, 0); /* Sets the start command to be sent. */
    if (ret != BASE_STATUS_OK) {
        SetErrorHandling(handle);
        return ret;
    }
    /* send slave addr */
    ret = SendSlaveAddressWriteCmd(handle, index);
    if (ret != BASE_STATUS_OK) {
        SetErrorHandling(handle);
        return ret;
    }
    ret = I2cTransferDataDma(handle, index);

    return ret;
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
    I2C_ASSERT_PARAM(handle != NULL && rData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    /* Check the DMA transfer handle and channel. */
    I2C_ASSERT_PARAM(handle->dmaHandle != NULL);
    I2C_PARAM_CHECK_WITH_RET((handle->txDmaCh < CHANNEL_MAX_NUM), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET((handle->rxDmaCh < CHANNEL_MAX_NUM), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET((handle->rxDmaCh != handle->txDmaCh), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    BASE_StatusType ret = BASE_STATUS_OK;
    unsigned int index;
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    /* Configuring Transmission Parameters of I2C. */
    handle->state = I2C_STATE_BUSY_SLAVE_RX;
    handle->transferSize = dataSize;
    handle->transferBuff = rData;
    handle->transferCount = 0;

    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_UNSET;
    ret = WaitStatusReady(handle, SLAVE_ADDRESS_MATCH, I2C_OPERATION_WRITE); /* Waiting to match master. */
    if (ret != BASE_STATUS_OK) {
        SetErrorHandling(handle);
        return ret;
    }
        
    /* Determine which I2C is used. */
    index = (handle->baseAddress == I2C0) ? I2C_INTERFACE_INDEX_0 : I2C_INTERFACE_INDEX_1;
    I2cTransferDataDma(handle, index);
    return ret;
}

/**
  * @brief Send data in DMA mode as slave.
  * @param handle I2C handle.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_I2C_SlaveWriteDMA(I2C_Handle *handle, unsigned char *wData, unsigned int dataSize)
{
    I2C_ASSERT_PARAM(handle != NULL && wData != NULL);
    I2C_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    /* Check the DMA transfer handle and channel. */
    I2C_ASSERT_PARAM(handle->dmaHandle != NULL);
    I2C_PARAM_CHECK_WITH_RET((handle->txDmaCh < CHANNEL_MAX_NUM), BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);
    I2C_PARAM_CHECK_WITH_RET(handle->state == I2C_STATE_READY, BASE_STATUS_ERROR);

    BASE_StatusType ret = BASE_STATUS_OK;
    unsigned int index;
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    /* Configuring Transmission Parameters of I2C. */
    handle->state = I2C_STATE_BUSY_SLAVE_TX;
    handle->transferSize = dataSize;
    handle->transferBuff = wData;
    handle->transferCount = 0;
    
    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_UNSET;
    /* Waiting for slave address match. */
    ret = WaitStatusReady(handle, SLAVE_ADDRESS_MATCH, I2C_OPERATION_READ);
    if (ret != BASE_STATUS_OK) {
        SetErrorHandling(handle);
        return ret;
    }
    /* Determine which I2C is used. */
    index = (handle->baseAddress == I2C0) ? I2C_INTERFACE_INDEX_0 : I2C_INTERFACE_INDEX_1;
    I2cTransferDataDma(handle, index);
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

    unsigned int status;
    unsigned int index;

    status = i2cHandle->baseAddress->I2C_INTR_STAT.reg;
    i2cHandle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    if (IsInterruptErrorStatus(i2cHandle, status)) {
        return;
    }
    /* Determine which I2C is used. */
    index = (i2cHandle->baseAddress == I2C0) ? I2C_INTERFACE_INDEX_0 : I2C_INTERFACE_INDEX_1;
    /* Callback interrupt handler function. */
    InterruptHandle(i2cHandle, status, index);
    if ((i2cHandle->transferCount >= i2cHandle->transferSize) &&
        (!(status & (I2C_INTR_RAW_ALL_CMD_DONE_MASK | I2C_INTR_RAW_STOP_DET_MASK)))) {
        if (i2cHandle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < I2C_MAX_FIFO_SIZE) {
            i2cHandle->baseAddress->I2C_TX_FIFO.reg =
                (((unsigned int)I2C_CMD_P << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
            i2cHandle->baseAddress->I2C_INTR_EN.BIT.tx_fifo_not_full_en = BASE_CFG_DISABLE;
            i2cHandle->transferCount++;
        }
    }
    /* After all data transmission is complete, call the user's callback function. */
    InterruptAllDoneHandle(i2cHandle, status);
}