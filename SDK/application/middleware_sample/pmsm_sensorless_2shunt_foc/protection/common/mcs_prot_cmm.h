/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      mcs_prot_cmm.h
  * @author    MCU Algorithm Team
  * @brief     This file contains protection function common data struct and api declaration.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_PROT_CMM_H
#define McuMagicTag_MCS_PROT_CMM_H

#include "typedefs.h"
#include "apt_ip.h"

/* Macro definitions --------------------------------------------------------------------------- */
#define MOTOR_PHASE_NUMBER    (3)

#define PROT_VAL_THRESHOLD_NUMS  (4)
#define PROT_VAL_THRESHOLD_0     (0)
#define PROT_VAL_THRESHOLD_1     (1)
#define PROT_VAL_THRESHOLD_2     (2)
#define PROT_VAL_THRESHOLD_3     (3)

#define PROT_LIMIT_TIME_NUMS     (3)
#define PROT_LIMIT_TIME_0        (0)
#define PROT_LIMIT_TIME_1        (1)
#define PROT_LIMIT_TIME_2        (2)

#define MOTOR_PHASE_NUMBER    (3)

/**< Motor error status definition. */
typedef union {
    unsigned short all;
    struct {
        unsigned short overCurrErr      : 1; /**< Indicates that phase current(s) is over protected value. */
        unsigned short overVoltErr      : 1; /**< Indicates that dc-link voltage is over protected value. */
        unsigned short lowerVoltErr     : 1; /**< Indicates that dc-link voltage is lower than protected value */
        unsigned short overIpmTempErr   : 1; /**< Indicates that IPM temperature is over protected value. */
        unsigned short revRotErr        : 1; /**< Indicates that motor temperature is over protected value. */
        unsigned short motorStalling    : 1; /**< Indicates that rotor is stalling. */
        unsigned short overMotorTempErr : 1; /**< Indicates that three phase currents is out-of-balance. */
        unsigned short posSnsrCommsErr  : 1; /**< Indicates that position sensor communication is lost with MCU. */
        unsigned short posSnsrFuncErr   : 1; /**< Indicates that position sensor reports function error. */
        unsigned short posSnsrCalibrErr : 1; /**< Indicates that position sensor fails to calibrate itself. */
        unsigned short currOutOfBalance : 1; /**< Indicates that the rotor is reverse rotation.*/
        unsigned short phsOpenErr       : 1; /**< Indicates that phase winding(s) is open. */
        unsigned short phsU             : 1; /**< Indicates that u phase fails when phsOpenErr occurs. */
        unsigned short phsV             : 1; /**< Indicates that v phase fails when phsOpenErr occurs. */
        unsigned short phsW             : 1; /**< Indicates that w phase fails when phsOpenErr occurs. */
        unsigned short multiPhs         : 1; /**< Indicates that multi-phases fail when phsOpenErr occurs.*/
    } Bit;
} MotorErrStatusReg;

/**< Protection Status Bit Definition */
typedef enum {
    OCP_ERR_BIT,
    OVP_ERR_BIT,
    LVP_ERR_BIT,
    OTP_IPM_ERR_BIT,
    OTP_MOTOR_ERR_BIT,
    STALLING_ERR_BIT,
    CURR_OUT_BALANCE_ERR_BIT,
    POS_COMMS_ERR_BIT,
    POS_FUNC_ERR_BIT,
    POS_CALIB_ERR_BIT,
    REV_ROT_ERR_BIT,
    PHS_OPEN_ERR_BIT,
    PHS_U_ERR_BIT,
    PHS_V_ERR_BIT,
    PHS_W_ERR_BIT,
    PHS_MULTI_ERR_BIT,
} PROT_ErrBit;

/**< Motor error protection level. */
typedef enum {
    PROT_LEVEL_0 = 0,
    PROT_LEVEL_1,
    PROT_LEVEL_2,
    PROT_LEVEL_3,
    PROT_LEVEL_4 /**< The greater level number, the severe error is. */
} PROT_Level;

/**
  * @brief Obtains the status of a bit of data.
  * @param data data.
  * @param bits Number of digits.
  * @retval Bit status.
  */
static inline bool GetBit(unsigned short data, unsigned char bit)
{
    bool ret;
    ret = ((data >> bit) & 1);
    return ret;
}

/**
  * @brief Sets the status of a bit of data.
  * @param data data.
  * @param bit The setted bit.
  * @retval None.
  */
static inline void SetBit(unsigned short *data, unsigned char bit)
{
    *data |= (1 << bit);
}

/**
  * @brief Clear the status of a bit of data.
  * @param data data.
  * @param bit The Clear bit.
  * @retval None.
  */
static inline void ClearBit(unsigned short *data, unsigned char bit)
{
    *data &= ~(1 << bit);
}

/**< Protection action. */
void ProtSpo_Exec(APT_RegStruct **aptAddr);

#endif
