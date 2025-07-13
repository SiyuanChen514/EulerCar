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

#define TYPE_POLY_MASK                0x0000000FU
#define TYPE_INIT_MASK                0x00000F00U
#define TYPE_XOR_VALUE_MASK           0x000F0000U
#define TYPE_REVERSE_ENABLE_MASK      0x0F000000U
#define TYPE_XOR_ENDIAN_ENABLE_MASK   0xF0000000U

#define TYPE_ENDIAN_MSB_BIT               0x10000000U
#define TYPE_XOR_ENABLE_BIT               0x20000000U
#define TYPE_OUTPUT_REVERSE_ENABLE_BIT    0x01000000U
#define TYPE_BYTE_REVERSE_ENABLE_BIT      0x02000000U
/**
  * @addtogroup CRC
  * @{
  */

/**
  * @defgroup CRC_IP CRC_IP
  * @brief CRC_IP: crc_v1.
  * @{
  */

/**
  * @defgroup CRC_Param_Def CRC Parameters Definition
  * @brief Description of CRC configuration parameters.
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/
typedef enum {
    CRC8_07_POLY_MODE = 0x00000000U,
    CRC8_07_POLY_MODE_BK = 0x00000001U,
    CRC16_8005_POLY_MODE = 0x00000002U,
    CRC16_1021_POLY_MODE = 0x00000003U,
    CRC32_04C11D87_POLY_MODE = 0x00000004U,
    CRC32_04C11D87_POLY_MODE_BK = 0x00000005U,
    CRC_POLY_MODE_MAX
} CRC_PolynomialMode;

typedef enum {
    TYPE_CRC_INIT_VALUE_00 = 0x00000100U,
    TYPE_CRC_INIT_VALUE_FF = 0x00000200U,
    TYPE_CRC_INIT_VALUE_0000 = 0x00000300U,
    TYPE_CRC_INIT_VALUE_FFFF = 0x00000400U,
    TYPE_CRC_INIT_VALUE_FFFFFFFF = 0x00000500U
} CRC_InitValueType;

typedef enum {
    CRC_INIT_VALUE_00 = 0x00000000U,
    CRC_INIT_VALUE_FF = 0x000000FFU,
    CRC_INIT_VALUE_0000 = 0x00000000U,
    CRC_INIT_VALUE_FFFF = 0x0000FFFFU,
    CRC_INIT_VALUE_FFFFFFFF = 0xFFFFFFFFU
} CRC_InitValue;

typedef enum {
    TYPE_CRC_XOR_VALUE_00 = 0x00010000U,
    TYPE_CRC_XOR_VALUE_55 = 0x00020000U,
    TYPE_CRC_XOR_VALUE_0000 = 0x00030000U,
    TYPE_CRC_XOR_VALUE_FFFF = 0x00040000U,
    TYPE_CRC_XOR_VALUE_00000000 = 0x00050000U,
    TYPE_CRC_XOR_VALUE_FFFFFFFF = 0x00060000U
} CRC_ResultXorValueType;

typedef enum {
    CRC_XOR_VALUE_00 = 0x00000000U,
    CRC_XOR_VALUE_55 = 0x00000055U,
    CRC_XOR_VALUE_0000 = 0x00000000U,
    CRC_XOR_VALUE_FFFF = 0x0000FFFFU,
    CRC_XOR_VALUE_00000000 = 0x00000000U,
    CRC_XOR_VALUE_FFFFFFFF = 0xFFFFFFFFU
} CRC_ResultXorValue;

typedef enum {
    REVERSE_INPUT_FALSE_OUTPUT_FALSE = 0x00000000U,
    REVERSE_INPUT_FALSE_OUTPUT_TRUE = 0x01000000U,
    REVERSE_INPUT_TURE_OUTPUT_FALSE = 0x02000000U,
    REVERSE_INPUT_TURE_OUTPUT_TRUE = 0x03000000U
} CRC_ReverseEnableType;

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
    CRC_MODE_BIT32 = 0x00000002U
} CRC_InputDataFormat;

/**
  * @brief CRC algorithm type.
  */
typedef enum {
    CRC8              = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_FALSE_OUTPUT_FALSE | TYPE_CRC_XOR_VALUE_00       | \
                        TYPE_CRC_INIT_VALUE_00       | CRC8_07_POLY_MODE,
    CRC8_ITU          = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_FALSE_OUTPUT_FALSE | TYPE_CRC_XOR_VALUE_55       | \
                        TYPE_CRC_INIT_VALUE_00       | CRC8_07_POLY_MODE,
    CRC8_ROHC         = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_TURE_OUTPUT_TRUE   | TYPE_CRC_XOR_VALUE_00       | \
                        TYPE_CRC_INIT_VALUE_FF       | CRC8_07_POLY_MODE,
    CRC16_IBM         = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_TURE_OUTPUT_TRUE   | TYPE_CRC_XOR_VALUE_0000     | \
                        TYPE_CRC_INIT_VALUE_0000     | CRC16_8005_POLY_MODE,
    CRC16_MAXIM       = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_TURE_OUTPUT_TRUE   | TYPE_CRC_XOR_VALUE_FFFF     | \
                        TYPE_CRC_INIT_VALUE_0000     | CRC16_8005_POLY_MODE,
    CRC16_USB         = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_TURE_OUTPUT_TRUE   | TYPE_CRC_XOR_VALUE_FFFF     | \
                        TYPE_CRC_INIT_VALUE_FFFF     | CRC16_8005_POLY_MODE,
    CRC16_MODBUS      = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_TURE_OUTPUT_TRUE   | TYPE_CRC_XOR_VALUE_0000     | \
                        TYPE_CRC_INIT_VALUE_FFFF     | CRC16_8005_POLY_MODE,
    CRC16_CCITT       = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_TURE_OUTPUT_TRUE   | TYPE_CRC_XOR_VALUE_0000     | \
                        TYPE_CRC_INIT_VALUE_0000     | CRC16_1021_POLY_MODE,
    CRC16_CCITT_FALSE = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_FALSE_OUTPUT_FALSE | TYPE_CRC_XOR_VALUE_0000     | \
                        TYPE_CRC_INIT_VALUE_FFFF     | CRC16_1021_POLY_MODE,
    CRC16_X25         = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_TURE_OUTPUT_TRUE   | TYPE_CRC_XOR_VALUE_FFFF     | \
                        TYPE_CRC_INIT_VALUE_FFFF     | CRC16_1021_POLY_MODE,
    CRC16_XMODEM      = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_FALSE_OUTPUT_FALSE | TYPE_CRC_XOR_VALUE_0000     | \
                        TYPE_CRC_INIT_VALUE_0000     | CRC16_1021_POLY_MODE,
    CRC32             = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_TURE_OUTPUT_TRUE   | TYPE_CRC_XOR_VALUE_FFFFFFFF | \
                        TYPE_CRC_INIT_VALUE_FFFFFFFF | CRC32_04C11D87_POLY_MODE,
    CRC32_MPEG2       = ENABLE_XOR_ENABLE_LSB   | REVERSE_INPUT_FALSE_OUTPUT_FALSE | TYPE_CRC_XOR_VALUE_00000000 | \
                        TYPE_CRC_INIT_VALUE_FFFFFFFF | CRC32_04C11D87_POLY_MODE,
    CRC_ALG_MODE_MAX
} CRC_AlgorithmMode;

/**
  * @brief CRC extend handle.
  */
typedef struct _CRC_ExtendeHandle {
    CRC_AlgorithmMode   algoMode;    /**< CRC calculate algorithm mode */
} CRC_ExtendHandle;

/**
  * @brief CRC user callback.
  */
typedef struct {
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
  * @brief CRC calc poly register union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int crc_calc_poly : 3;    /**< crc calc polynomial set. */
        unsigned int reserved0 : 29;
    } BIT;
} volatile CRC_CALC_CFG_REG;

/**
  * @brief CRC soft reset register union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int crc_calc_reset : 1;    /**< crc calc soft reset signal. */
        unsigned int reserved0 : 31;
    } BIT;
} volatile CRC_CALC_RESET_REG;

/**
  * @brief CRC init register union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int crc_calc_init : 1;    /**< crc init value load signal. */
        unsigned int reserved0 : 31;
    } BIT;
} volatile CRC_CALC_INIT_REG;

/**
  * @brief CRC pre set register union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int crc_pre_byte_reverse : 1;  /**< crc pre byte reverse enable. */
        unsigned int crc_pre_endian_mode : 1;   /**< crc pre endian set mode. */
        unsigned int reserved0 : 30;
    } BIT;
} volatile CRC_PRE_CFG_REG;

/**
  * @brief CRC post set register union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int crc_post_xor_enable : 1;   /**< crc result xor enable. */
        unsigned int crc_post_out_reverse : 1;  /**< crc result reverse enable. */
        unsigned int reserved0 : 30;
    } BIT;
} volatile CRC_POST_CFG_REG;

/**
  * @brief CRC assemble registers structure definition
  */
typedef struct {
    CRC_CALC_CFG_REG         CRC_CALC_CFG;        /**< crc calc poly register. */
    CRC_CALC_RESET_REG       CRC_CALC_RESET;      /**< crc soft reset register. */
    CRC_CALC_INIT_REG        CRC_CALC_INIT;       /**< crc init register. */
    unsigned int             crc_post_xor_value;  /**< crc post process xor value register. */
    unsigned int             crc_calc_init_value; /**< crc init value register. */
    unsigned int             crc_data_in;         /**< crc input data register. */
    unsigned int             crc_out;             /**< crc result register. */
    CRC_PRE_CFG_REG          CRC_PRE_CFG;         /**< crc pre set register. */
    CRC_POST_CFG_REG         CRC_POST_CFG;        /**< crc post set register. */
} volatile CRC_RegStruct;

/**
  * @}
  */
/**
  * @brief Set CRC polyniaml mode.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param polyMode Value of @ref CRC_PolynomialMode.
  * @retval None.
  */
static inline void DCL_CRC_SetPolynomialMode(CRC_RegStruct *crcx, CRC_PolynomialMode polyMode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    CRC_PARAM_CHECK_NO_RET(polyMode < CRC_POLY_MODE_MAX && polyMode >= CRC8_07_POLY_MODE);
    crcx->CRC_CALC_CFG.BIT.crc_calc_poly = polyMode;
}

/**
  * @brief Get CRC polyniaml mode.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval CRC_PolynomialMode.
  */
static inline CRC_PolynomialMode DCL_CRC_GetPolynomialMode(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->CRC_CALC_CFG.BIT.crc_calc_poly;
}

/**
  * @brief Set CRC soft reset function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval None.
  */
static inline void DCL_CRC_SoftReset(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->CRC_CALC_RESET.BIT.crc_calc_reset = BASE_CFG_SET;
}

/**
  * @brief Enable CRC init function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval None.
  */
static inline void DCL_CRC_LoadInitValue(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->CRC_CALC_INIT.BIT.crc_calc_init = BASE_CFG_SET;
}

/**
  * @brief Set CRC result xor value function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param value Value of CRC calulate result.
  * @retval None.
  */
static inline void DCL_CRC_SetResultXorValue(CRC_RegStruct *crcx, unsigned int value)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->crc_post_xor_value = value;
}

/**
  * @brief Get CRC result xor value function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval unsigned int value of result xor value.
  */
static inline unsigned int DCL_CRC_GetResultXorValue(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->crc_post_xor_value;
}

/**
  * @brief Set CRC init value function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param initValue value of CRC calulate init value.
  * @retval None.
  */
static inline void DCL_CRC_SetInitValue(CRC_RegStruct *crcx, unsigned int initValue)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->crc_calc_init_value = initValue;
}

/**
  * @brief Get CRC init value function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval unsigned int init value.
  */
static inline unsigned int DCL_CRC_GetInitValue(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->crc_calc_init_value;
}

/**
  * @brief Set CRC data 8 in value function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param data value of CRC calulate data value.
  * @retval None.
  */
static inline void DCL_CRC_SetInputData8(CRC_RegStruct *crcx, unsigned char data)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    volatile unsigned char *crcData8 = (unsigned char *)(void *)(&crcx->crc_data_in);
    *(crcData8) = data;
}

/**
  * @brief Set CRC data 16 in value function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param data value of CRC calulate data value.
  * @retval None.
  */
static inline void DCL_CRC_SetInputData16(CRC_RegStruct *crcx, unsigned short data)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    volatile unsigned short *crcData16 = (unsigned short *)(void *)(&crcx->crc_data_in);
    *(crcData16) = data;
}

/**
  * @brief Set CRC data 32 in value function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param data value of CRC calulate data value.
  * @retval None.
  */
static inline void DCL_CRC_SetInputData32(CRC_RegStruct *crcx, unsigned int data)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->crc_data_in = data;
}

/**
  * @brief Get CRC data in value function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval unsigned int crc data in.
  */
static inline unsigned int DCL_CRC_GetInputData(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->crc_data_in;
}

/**
  * @brief Get CRC data in value function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval unsigned int crc data out.
  */
static inline unsigned int DCL_CRC_GetOutputData(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->crc_out;
}

/**
  * @brief Set CRC input data endian mode function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param mode true means big endian, false means little endian.
  * @retval None.
  */
static inline void DCL_CRC_SetEndianMode(CRC_RegStruct *crcx, bool mode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->CRC_PRE_CFG.BIT.crc_pre_endian_mode = mode;
}

/**
  * @brief Get CRC input data endian mode function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval bool crc endian mode.
  */
static inline bool DCL_CRC_GetEndianMode(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->CRC_PRE_CFG.BIT.crc_pre_endian_mode;
}

/**
  * @brief Set CRC input data byte reverse function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param mode true means reverse, false means none.
  * @retval None.
  */
static inline void DCL_CRC_SetByteReverseMode(CRC_RegStruct *crcx, bool mode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->CRC_PRE_CFG.BIT.crc_pre_byte_reverse = mode;
}

/**
  * @brief Get CRC input data byte reverse function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval bool crc byte reverse mode.
  */
static inline bool DCL_CRC_GetByteReverseMode(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->CRC_PRE_CFG.BIT.crc_pre_byte_reverse;
}

/**
  * @brief Set CRC output result reverse function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param mode true means reverse, false means none.
  * @retval None.
  */
static inline void DCL_CRC_SetOutputReverseMode(CRC_RegStruct *crcx, bool mode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->CRC_POST_CFG.BIT.crc_post_out_reverse = mode;
}

/**
  * @brief Get CRC output result reverse function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval bool crc out reverse mode.
  */
static inline bool DCL_CRC_GetOutputReverseMode(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->CRC_POST_CFG.BIT.crc_post_out_reverse;
}

/**
  * @brief Set CRC result xor mode function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @param mode true means reverse, false means none.
  * @retval None.
  */
static inline void DCL_CRC_SetXorResultMode(CRC_RegStruct *crcx, bool mode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    crcx->CRC_POST_CFG.BIT.crc_post_xor_enable = mode; /* 0 means disable, 1 means enable */
}

/**
  * @brief Get CRC result xor mode function.
  * @param crcx Value of @ref CRC_RegStruct.
  * @retval bool crc xor enable mode.
  */
static inline bool DCL_CRC_GetXorResultMode(CRC_RegStruct *crcx)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    return crcx->CRC_POST_CFG.BIT.crc_post_xor_enable;
}

/**
  * @brief Check crc polynomial mode.
  * @param mode Value of @ref CRC_PolynomialMode.
  * @retval Bool
  */
static inline bool IsCrcPolynomial(unsigned int mode)
{
    /* Check crc polynomial mode. */
    return (mode == CRC8_07_POLY_MODE || mode == CRC8_07_POLY_MODE_BK || \
            mode == CRC16_8005_POLY_MODE || mode == CRC16_1021_POLY_MODE || \
            mode == CRC32_04C11D87_POLY_MODE || mode == CRC32_04C11D87_POLY_MODE_BK);
}

/**
  * @brief Check crc init value type.
  * @param mode Value of @ref CRC_InitValueType.
  * @retval Bool
  */
static inline bool IsCrcInitValueType(unsigned int value)
{
    /* Check crc polynomial mode. */
    return (value == TYPE_CRC_INIT_VALUE_00 || value == TYPE_CRC_INIT_VALUE_FF || \
            value == TYPE_CRC_INIT_VALUE_0000 || value == TYPE_CRC_INIT_VALUE_FFFF || \
            value == TYPE_CRC_INIT_VALUE_FFFFFFFF);
}

/**
  * @brief Check crc result xor value type.
  * @param mode Value of @ref CRC_ResultXorValueType.
  * @retval Bool
  */
static inline bool IsCrcResultXorValueType(unsigned int value)
{
    /* Check crc polynomial mode. */
    return (value == TYPE_CRC_XOR_VALUE_00 || value == TYPE_CRC_XOR_VALUE_55 || \
            value == TYPE_CRC_XOR_VALUE_0000 || value == TYPE_CRC_XOR_VALUE_FFFF || \
            value == TYPE_CRC_XOR_VALUE_00000000 || value == TYPE_CRC_XOR_VALUE_FFFFFFFF);
}

/**
  * @brief Check crc reverse enable type.
  * @param mode Value of @ref CRC_ReverseEnableType.
  * @retval Bool
  */
static inline bool IsCrcXorEndianEnableType(unsigned int type)
{
    /* Check crc reverse enable type. */
    return (type == ENABLE_XOR_ENABLE_LSB || type == ENABLE_XOR_ENABLE_MSB || \
            type == DISABLE_XOR_ENABLE_LSB || type == DISABLE_XOR_ENABLE_MSB);
}

/**
  * @brief Check crc reverse enable type.
  * @param mode Value of @ref CRC_ReverseEnableType.
  * @retval Bool
  */
static inline bool IsCrcReverseEnableType(unsigned int type)
{
    /* Check crc reverse enable type. */
    return (type == REVERSE_INPUT_FALSE_OUTPUT_FALSE || type == REVERSE_INPUT_FALSE_OUTPUT_TRUE || \
            type == REVERSE_INPUT_TURE_OUTPUT_FALSE || type == REVERSE_INPUT_TURE_OUTPUT_TRUE);
}

/**
  * @brief Check crc valid byte mode.
  * @param mode Value of @ref CRC_InputDataFormat.
  * @retval Bool
  */
static inline bool IsCrcInputDataFormat(unsigned int mode)
{
    return (mode == CRC_MODE_BIT8 ||
            mode == CRC_MODE_BIT16 ||
            mode == CRC_MODE_BIT32);
}

/**
  * @brief Check crc algorithm mode.
  * @param mode Value of @ref CRC_AlgorithmMode.
  * @retval Bool
  */
static inline bool IsCrcAlgorithm(CRC_AlgorithmMode mode)
{
    /* Check crc algorithm mode. */
    return (mode == CRC8 || mode == CRC8_ITU || \
            mode == CRC8_ROHC || mode == CRC16_IBM || \
            mode == CRC16_MAXIM ||  mode == CRC16_USB || \
            mode == CRC16_MODBUS || mode == CRC16_CCITT || \
            mode == CRC16_CCITT_FALSE || mode == CRC16_X25 || \
            mode == CRC16_XMODEM || mode == CRC32 || \
            mode == CRC32_MPEG2);
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_CRC_IP_H */
