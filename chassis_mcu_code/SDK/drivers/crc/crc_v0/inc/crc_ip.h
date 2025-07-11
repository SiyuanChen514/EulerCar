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
  * @file      crc_ip.h
  * @author    MCU Driver Team
  * @brief     CRC module driver
  * @details   The header file contains the following declaration:
  *             + CRC configuration enums.
  *             + CRC register structures.
  *             + CRC DCL Functions.
  *             + Parameters check functions.
  */

#ifndef McuMagicTag_CRC_IP_H
#define McuMagicTag_CRC_IP_H
/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"
/* Macro definitions -------------------------------------------------------*/
#ifdef CRC_PARAM_CHECK
    #define CRC_ASSERT_PARAM         BASE_FUNC_ASSERT_PARAM
    #define CRC_PARAM_CHECK_NO_RET   BASE_FUNC_PARAMCHECK_NO_RET
    #define CRC_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
    #define CRC_ASSERT_PARAM(para)                ((void)0U)
    #define CRC_PARAM_CHECK_NO_RET(para)          ((void)0U)
    #define CRC_PARAM_CHECK_WITH_RET(param, ret)  ((void)0U)
#endif
/**
  * @addtogroup CRC
  * @{
  */

/**
  * @defgroup CRC_IP CRC_IP
  * @brief CRC_IP: crc_v0.
  * @{
  */

/**
  * @defgroup CRC_Param_Def CRC Parameters Definition
  * @brief Description of CRC configuration parameters.
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/
/**
  * @brief CRC algorithm type.
  * @details CRC algorithm type:
  *          +CRC8_ROHC         -- CRC-8/ROHC algorithm, CRC8_07, crc_mode reg value:b000
  *          +CRC16_IBM         -- CRC-16/IBM algorithm, CRC16_8005, crc_mode reg value:b010
  *          +CRC16_MODBUS      -- CRC-16/MODBUS algorithm, CRC16_8005, crc_mode reg value:b010
  *          +CRC16_CCITT_FALSE -- CRC-16/CCITT-FALSE algorithm, CRC16_1021, crc_mode reg value:b011
  *          +CRC16_XMODEM      -- CRC-16/XMODEM algorithm, CRC16_1021, crc_mode reg value:b011
  *          +CRC32             -- CRC32 algorithm, CRC32_04C11D87, crc_mode reg value:b10x
  *          +CRC_ALG_MODE_MAX  -- CRC_mode bunder
  */
typedef enum {
    CRC8_ROHC = 0x00000001U,
    CRC16_IBM = 0x00000002U,
    CRC16_MODBUS = 0x00000003U,
    CRC16_CCITT_FALSE = 0x00000004U,
    CRC16_XMODEM = 0x00000005U,
    CRC32 = 0x00000006U,
    CRC_ALG_MODE_MAX
} CRC_AlgorithmMode;

/**
  * @brief CRC polynomial mode register configuration.
  */
typedef enum {
    CRC8_07_POLY_MODE = 0x00000000U,
    CRC8_07_POLY_MODE_BK = 0x00000001U,
    CRC16_8005_POLY_MODE = 0x00000002U,
    CRC16_1021_POLY_MODE = 0x00000003U,
    CRC32_04C11D87_POLY_MODE = 0x00000004U,
    CRC32_04C11D87_POLY_MODE_BK = 0x00000005U,
    CRC_POLY_MODE_MAX
} CRC_PolynomialMode;

/**
  * @brief CRC init value type.
  */
typedef enum {
    TYPE_CRC_INIT_VALUE_FF = 0x00000200U,
    TYPE_CRC_INIT_VALUE_0000 = 0x00000300U,
    TYPE_CRC_INIT_VALUE_FFFF = 0x00000400U,
    TYPE_CRC_INIT_VALUE_FFFFFFFF = 0x00000500U
} CRC_InitValueType;

/**
  * @brief CRC init value Enumeration.
  */
typedef enum {
    CRC_INIT_VALUE_FF = 0x000000FFU,
    CRC_INIT_VALUE_0000 = 0x00000000U,
    CRC_INIT_VALUE_FFFF = 0x0000FFFFU,
    CRC_INIT_VALUE_FFFFFFFF = 0xFFFFFFFFU
} CRC_InitValue;

/**
  * @brief CRC xor value type.
  */
typedef enum {
    TYPE_CRC_XOR_VALUE_00 = 0x00000001U,
    TYPE_CRC_XOR_VALUE_0000 = 0x00000002U,
    TYPE_CRC_XOR_VALUE_FFFFFFFF = 0x00000003U
} CRC_ResultXorValueType;

/**
  * @brief CRC input data or output data in reverse order.
  */
typedef enum {
    REVERSE_INPUT_FALSE_OUTPUT_FALSE = 0x00000001U,
    REVERSE_INPUT_TURE_OUTPUT_TRUE = 0x00000002U
} CRC_ReverseEnableType;

/**
  * @brief CRC xor result value and endian mode type.
  */
typedef enum {
    DISABLE_XOR_ENABLE_LSB = 0x00000000U,
    DISABLE_XOR_ENABLE_MSB = 0x10000000U,
    ENABLE_XOR_ENABLE_LSB = 0x20000000U,
    ENABLE_XOR_ENABLE_MSB = 0x30000000U
} CRC_XorEndianEnableType;

/**
  * @brief CRC byte type register configuration.
  */
typedef enum {
    CRC_MODE_BIT8 = 0x00000000U,
    CRC_MODE_BIT16 = 0x00000001U,
    CRC_MODE_BIT24 = 0x00000002U,
    CRC_MODE_BIT32 = 0x00000003U
} CRC_InputDataFormat;

/**
  * @brief CRC FIFO status.
  */
typedef enum {
    CRC_FIFO_FULL = 0x00000000U,
    CRC_FIFO_EMPTY = 0x00000001U
} CRC_FIFOStatus;

/**
  * @brief CRC extend handle.
  */
typedef struct _CRC_ExtendHandle {
    CRC_AlgorithmMode   algoMode;        /**< CRC calculate algorithm mode */
    bool                enableIT;        /**< Enable pready timeout interrupt */
    bool                enableErrInject; /**< Enable error inject */
    unsigned int        timeout;         /**< APB pready max timeout value */
} CRC_ExtendHandle;

/**
  * @brief CRC user callback.
  */
typedef struct {
    void (* PreadyTimeoutCallback)(void *handle); /**< CRC pready Timeout Callback */
} CRC_UserCallBack;
/**
  * @}
  */

/**
  * @defgroup CRC_Reg_Def CRC Register Definition
  * @brief Description CRC register mapping structure.
  * @{
  */

/**
  * @brief CRC control algorithm and valid bit register union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int crc_byte_mode : 2;    /**< data valid byte mode. */
        unsigned int crc_mode : 3;         /**< crc algorithm selection. */
        unsigned int reserved0 : 27;
    } BIT;
} volatile CRC_CTRL_CFG0_REG;

/**
  * @brief CRC init register union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int crc_init : 1;    /**< crc soft reset signal. */
        unsigned int reserved0 : 31;
    } BIT;
} volatile CRC_CTRL_CFG1_REG;

/**
  * @brief CRC load data register union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int crc_ld : 1;    /**< crc load value signal. */
        unsigned int reserved0 : 31;
    } BIT;
} volatile CRC_CTRL_CFG2_REG;

/**
  * @brief CRC state register union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int crc_fifo_full : 1;    /**< crc fifo full flag. */
        unsigned int crc_fifo_empty : 1;   /**< crc fifo empty flag. */
        unsigned int reserved1 : 29;
    } BIT;
} volatile CRC_CTRL_STATUS_REG;

/**
  * @brief CRC set pready timeout register union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pready_timeout : 1;    /**< pready timeout interrupt. */
        unsigned int reserved0 : 31;
    } BIT;
} volatile CRC_CTRL_INT_REG;

/**
  * @brief CRC timeout mask register union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pready_timeout_int_mask : 1;  /**< pready timeout interrupt mask. */
        unsigned int reserved0 : 31;
    } BIT;
} volatile CRC_CTRL_INTMASK_REG;

/**
  * @brief CRC timeout error inject register union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int time_out_err_inj : 1;    /**< pready timeout error inject. */
        unsigned int reserved0 : 31;
    } BIT;
} volatile CRC_CTRL_ERRINJ_REG;

/**
  * @brief CRC assemble registers structure definition
  */
typedef struct {
    CRC_CTRL_CFG0_REG        CRC_CTRL_CFG0;    /**< crc control algorithm and valid bit register. */
    CRC_CTRL_CFG1_REG        CRC_CTRL_CFG1;    /**< crc init control register. */
    CRC_CTRL_CFG2_REG        CRC_CTRL_CFG2;    /**< crc load data register. */
    unsigned int             cnt_max;          /**< timeout max value. */
    unsigned int             crc_check_in;     /**< crc output check in value. */
    unsigned int             crc_data_in;      /**< crc calculate data. */
    unsigned int             crc_out;          /**< crc calculate output result. */
    CRC_CTRL_STATUS_REG      CRC_CTRL_STATUS;  /**< crc state register. */
    CRC_CTRL_INT_REG         CRC_CTRL_INT;     /**< crc set pready timeout register. */
    CRC_CTRL_INTMASK_REG     CRC_CTRL_INTMASK; /**< crc timeout mask register. */
    CRC_CTRL_ERRINJ_REG      CRC_CTRL_ERRINJ;  /**< crc timeout error inject register. */
} volatile CRC_RegStruct;

/**
  * @}
  */

/**
  * @brief Set CRC polynomial mode.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param @ref polyMode of @ref CRC_PolynomialMode.
  * @retval None.
  */
static inline void DCL_CRC_SetPolynomialMode(CRC_RegStruct *crcx, CRC_PolynomialMode polyMode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    CRC_PARAM_CHECK_NO_RET(polyMode < CRC_POLY_MODE_MAX && polyMode >= CRC8_07_POLY_MODE);
    crcx->CRC_CTRL_CFG0.BIT.crc_mode = polyMode;
}

/**
  * @brief Set CRC calculate input Data Format.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param inputDataFormat Value of @ref CRC_InputDataFormat.
  * @retval None.
  */
static inline void DCL_CRC_SetByteMode(CRC_RegStruct *crcx, CRC_InputDataFormat inputDataFormat)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    CRC_PARAM_CHECK_NO_RET(inputDataFormat <= CRC_MODE_BIT32 && inputDataFormat >= CRC_MODE_BIT8);
    crcx->CRC_CTRL_CFG0.BIT.crc_byte_mode = inputDataFormat;
}

/**
  * @brief Set CRC timeout value.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval None.
  */
static inline void DCL_CRC_SetTimeOut(CRC_RegStruct *crcx, unsigned int timeout)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->cnt_max = timeout;
}

/**
  * @brief Set CRC soft reset function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval None.
  */
static inline void DCL_CRC_SoftReset(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->CRC_CTRL_CFG1.BIT.crc_init = BASE_CFG_SET;
}

/**
  * @brief Enable CRC init function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval None.
  */
static inline void DCL_CRC_LoadInitValue(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->CRC_CTRL_CFG2.BIT.crc_ld = BASE_CFG_SET;
}

/**
  * @brief Set CRC data in value function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param data value of CRC calulate data value.
  * @retval None.
  */
static inline void DCL_CRC_SetInputData(CRC_RegStruct *crcx, unsigned int data)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->crc_data_in = data;
}

/**
  * @brief Get CRC input data.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval unsigned int CRC Input data.
  */
static inline unsigned int DCL_CRC_GetInputData(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->crc_data_in;
}

/**
  * @brief Get CRC output value.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval unsigned int CRC Output value.
  */
static inline unsigned int DCL_CRC_GetOutputData(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->crc_out;
}

/**
  * @brief Clear CRC interrupt signal.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval None.
  */
static inline void DCL_CRC_ClearIrq(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->CRC_CTRL_INT.BIT.pready_timeout = BASE_CFG_SET;
}

/**
  * @brief Get pready interrrput flag.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval bool true is valid, false is invalid.
  */
static inline bool DCL_CRC_GetPreadyIrqFlag(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->CRC_CTRL_INT.BIT.pready_timeout;
}

/**
  * @brief Set pready timeout mask.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param mask true means enable, false means disable.
  * @retval None.
  */
static inline void DCL_CRC_SetPreadyTimeoutMask(CRC_RegStruct *crcx, bool mask)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->CRC_CTRL_INTMASK.BIT.pready_timeout_int_mask = mask;
}

/**
  * @brief Get pready timeout mask.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval bool pready timeout mask.
  */
static inline bool DCL_CRC_GetPreadyTimeoutMask(const CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->CRC_CTRL_INTMASK.BIT.pready_timeout_int_mask;
}

/**
  * @brief Set timeout error inject.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param errInject true means enable, false means disable .
  * @retval None.
  */
static inline void DCL_CRC_SetTimeoutErrInj(CRC_RegStruct *crcx, bool errInject)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->CRC_CTRL_ERRINJ.BIT.time_out_err_inj = errInject;
}

/**
  * @brief Get timeout error inject.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval bool timeout error inject.
  */
static inline bool DCL_CRC_GetTimeoutErrInj(const CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->CRC_CTRL_ERRINJ.BIT.time_out_err_inj;
}

/**
  * @brief Set CRC check in value function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param value value of CRC calulate check in value.
  * @retval None.
  */
static inline void DCL_CRC_SetCheckInValue(CRC_RegStruct *crcx, unsigned int value)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->crc_check_in = value;
}

/**
  * @brief Get CRC check in value function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval unsigned int crc check in value.
  */
static inline unsigned int DCL_CRC_GetCheckInValue(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->crc_check_in;
}

/**
  * @brief Get CRC FIFO status.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval None.
  */
static inline void DCL_CRC_WaitComplete(const CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    while (crcx->CRC_CTRL_STATUS.BIT.crc_fifo_empty != BASE_CFG_SET) {
    }
    while (crcx->CRC_CTRL_STATUS.BIT.crc_fifo_empty != BASE_CFG_SET) {
    }
}

/**
  * @brief Get fifo empty flag.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval bool of fifo empty flag.
  */
static inline bool DCL_CRC_GetFifoEmptyFlag(const CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->CRC_CTRL_STATUS.BIT.crc_fifo_empty;
}

/**
  * @brief Check crc algorithm mode.
  * @param mode Value of @ref CRC_AlgorithmMode.
  * @retval Bool
  */
static inline bool IsCrcAlgorithm(CRC_AlgorithmMode mode)
{
    /* Check crc algorithm mode. */
    return (mode == CRC8_ROHC || \
            mode == CRC16_IBM || \
            mode == CRC16_MODBUS || \
            mode == CRC16_CCITT_FALSE || \
            mode == CRC16_XMODEM || \
            mode == CRC32);
}

/**
  * @brief Check crc valid byte mode.
  * @param mode Value of @ref CRC_InputDataFormat.
  * @retval Bool
  */
static inline bool IsCrcInputDataFormat(CRC_InputDataFormat mode)
{
    return (mode == CRC_MODE_BIT8 ||
            mode == CRC_MODE_BIT16 ||
            mode == CRC_MODE_BIT24 ||
            mode == CRC_MODE_BIT32);
}

/**
  * @brief Check crc init value type.
  * @param mode Value of @ref CRC_InitValueType.
  * @retval Bool
  */
static inline bool IsCrcInitValueType(CRC_InitValueType mode)
{
    /* Check crc init value. */
    return (mode == TYPE_CRC_INIT_VALUE_FF || mode == TYPE_CRC_INIT_VALUE_0000 || \
            mode == TYPE_CRC_INIT_VALUE_FFFF || mode == TYPE_CRC_INIT_VALUE_FFFFFFFF);
}

/**
  * @brief Check crc polynomial mode.
  * @param mode Value of @ref CRC_PolynomialMode.
  * @retval Bool
  */
static inline bool IsCrcPolynomial(CRC_PolynomialMode mode)
{
    /* Check crc polynomial mode. */
    return (mode == CRC8_07_POLY_MODE || mode == CRC8_07_POLY_MODE_BK || \
            mode == CRC16_8005_POLY_MODE || mode == CRC16_1021_POLY_MODE || \
            mode == CRC32_04C11D87_POLY_MODE || mode == CRC32_04C11D87_POLY_MODE_BK);
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_CRC_IP_H */
