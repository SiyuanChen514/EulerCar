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
  * @file      cust_process.c
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of cust process interface.
  */

#include "cust_process.h"
#include "mcs_ctlmode_config.h"
#include "mcs_math_const.h"
#include "mcs_user_config.h"
#include "mcs_assert.h"
#include "main.h"
#include "uart_module.h"
#include "mcs_mtr_param.h"
/* Macro definitions --------------------------------------------------------------------------- */
/* Constant value. */
#define CONST_VALUE_60                  60.0f       /* Constant value 60. */
#define CONST_VALUE_DIV_1000            0.001f     /* Constant value 1/1000. */

/* Data array index. */
#define DATA_SEGMENT_ONE                0           /* First element of the data segment */
#define DATA_SEGMENT_TWO                1           /* Second element of the data segment  */
#define DATA_SEGMENT_THREE              2           /* Third element of the data segment  */
#define DATA_SEGMENT_FOUR               3           /* Fourth element of the data segment  */
#define CUSTACKCODELEN                  10          /* Ack code length  */

/* Command code. */
#define SET_PID_KP                      0x01    /* Set Pid Kp Command Params */
#define SET_PID_KI                      0x02    /* Set Pid Ki Command Params */
#define SET_PID_LIMIT                   0x03    /* Set Pid limit Command Params */

#define SET_SMO1TH_PLL_BDW              0x01    /* Set Smo1th Pll BandWidth Command Params */
#define SET_SMO1TH_SPDFLITER_FC         0x02    /* Set Smo1th Fc Command Params */
#define SET_SMO1TH_FILCOMPANGLE         0x03    /* Set Smo1th FillComp Command Params */

#define SET_SMO4TH_KD                   0x01     /* Set Smo4th  Kd Command Params */
#define SET_SMO4TH_KP                   0x02     /* Set Smo4th Kq Command Params */

#define SET_SPD_COMMAND_HZ              0x01   /* Set Target Speed Command Params  */
#define SET_SPD_RMG_SLOPE               0x02   /* Set Speed Slope Command Params */

#define SET_MAX_ELEC_SPD                0x01   /* Set Max Motor Speed Command  Params */
#define SET_MOTOR_NUM_OF_PAIRS          0x02   /* Set Motor Command Params */

#define SET_MOTOR_RES_OF_STATOR         0x01    /* Set Motor Res Command Params */
#define SET_MOTOR_DAXIS_INDUCTANCE      0x02    /* Set Motor Daxis Inductance Command Params */
#define SET_MOTOR_QAXIS_INDUCTANCE      0x03    /* Set Motor Qaxis Inductance Command Params */

#define SET_SVPWM_VOLTAGE_PER_UNIT      0x01    /* Set Svpwm Voltage Params */
#define SET_ADC_CURRENT_SAMPLE_COFE     0x02    /* Set Adc coffe Params */
#define SET_CURRENT_LOOP_CONTROL_PERIOD 0x03    /* Set Current loop period Command Params */

#define SET_IF_TARGET_CURRENT_VALUE     0x01    /* Set If Target Params */
#define SET_INCREMENT_OF_IF_CURRENT     0x02    /* Set If Step Command Params */
#define SET_SPEED_RING_BEGIN_SPEED      0x03    /* Set If to Smo Start Speed Command Params */

static unsigned char ackCode = 0;
static unsigned char g_uartTxBuf[CUSTACKCODELEN] = {0};

/**
  * @brief Set observer type.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  */
static void CMDCODE_SetObserverType(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData)
{
    /* Get function code. */
    int funcCode = (int)(rxData->data[DATA_SEGMENT_ONE].typeF);
    if (funcCode == FOC_OBSERVERTYPE_SMO1TH) {
        ackCode = 0X01;
        mtrCtrl->obserType = FOC_OBSERVERTYPE_SMO1TH;
        CUST_AckCode(g_uartTxBuf, ackCode, mtrCtrl->obserType);
    } else if (funcCode  == FOC_OBSERVERTYPE_SMO4TH) {
        ackCode = 0X02;
        mtrCtrl->obserType = FOC_OBSERVERTYPE_SMO4TH;
        CUST_AckCode(g_uartTxBuf, ackCode, mtrCtrl->obserType);
    }
}

/**
  * @brief Set pid parameter ack code.
  * @param funcCode Received data funccode.
  */
static unsigned char SetPidAckCode(int funcCode)
{
    switch (funcCode) {
        /* Set current loop D-Axis PID parameter ack code. */
        case FOC_CURDAXISPID_PARAMS:
            ackCode = 0xE0;
            break;
        /* Set current loop Q-Axis PID parameter ack code. */
        case FOC_CURQAXISPID_PARAMS:
            ackCode = 0xE3;
            break;
        /* Set speed loop PID parameter ack code. */
        case FOC_SPDPID_PARAMS:
            ackCode = 0xE6;
            break;
        default:
            break;
    }
    return ackCode;
}

/**
  * @brief Set pid parameters.
  * @param pidHandle The pid control handle.
  * @param rxData Receive buffer
  */
static void SetPidParams(PID_Handle *pidHandle, CUSTDATATYPE_DEF *rxData)
{
    /* Get function code. */
    int funcCode = (int)(rxData->data[DATA_SEGMENT_ONE].typeF);
    /* Get command code. */
    int cmdCode =  (int)(rxData->data[DATA_SEGMENT_TWO].typeF);
    switch (cmdCode) {
        case SET_PID_KP:    /* Set the P parameter. */
            PID_SetKp(pidHandle, rxData->data[DATA_SEGMENT_THREE].typeF);
            ackCode = SetPidAckCode(funcCode);
            CUST_AckCode(g_uartTxBuf, ackCode + SET_PID_KP, rxData->data[DATA_SEGMENT_THREE].typeF);
            break;
        case SET_PID_KI:    /* Set the I parameter. */
            PID_SetKi(pidHandle, rxData->data[DATA_SEGMENT_THREE].typeF);
            ackCode = SetPidAckCode(funcCode);
            CUST_AckCode(g_uartTxBuf, ackCode + SET_PID_KI, rxData->data[DATA_SEGMENT_THREE].typeF);
            break;
        case SET_PID_LIMIT: /* Set the pid limit. */
            PID_SetLimit(pidHandle, rxData->data[DATA_SEGMENT_THREE].typeF);
            ackCode = SetPidAckCode(funcCode);
            CUST_AckCode(g_uartTxBuf, ackCode + SET_PID_LIMIT, rxData->data[DATA_SEGMENT_THREE].typeF);
            break;
        default:
            ackCode = 0X77;
            CUST_AckCode(g_uartTxBuf, ackCode, 0);
            break;
    }
}

/**
  * @brief Set motor pid parameters.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  */
static void CMDCODE_SetMotorPidParams(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData)
{
    /* Get function code. */
    int funcCode = (int)(rxData->data[DATA_SEGMENT_ONE].typeF);

    if (funcCode == FOC_CURDAXISPID_PARAMS) {
        SetPidParams(&mtrCtrl->currCtrl.dAxisPi, rxData);    /* Set Curr loop Daxis pid params  */
    } else if (funcCode  == FOC_CURQAXISPID_PARAMS) {
        SetPidParams(&mtrCtrl->currCtrl.qAxisPi, rxData);    /* Set Curr loop Qaxis pid params  */
        mtrCtrl->currCtrl.dAxisPi.upperLimit = mtrCtrl->currCtrl.qAxisPi.upperLimit;
        mtrCtrl->currCtrl.dAxisPi.lowerLimit = mtrCtrl->currCtrl.qAxisPi.lowerLimit;
    } else if (funcCode  == FOC_SPDPID_PARAMS) {
        SetPidParams(&mtrCtrl->spdCtrl.spdPi, rxData);    /* Set Speed loop params  */
    }
}

/**
  * @brief Set first order sliding mode observer parameters.
  * @param smoHandle The observer control handle.
  * @param rxData Receive buffer
  */
static void SetObserverSmo1thParams(FOSMO_Handle *smoHandle, CUSTDATATYPE_DEF *rxData)
{
    smoHandle->kSmo = rxData->data[DATA_SEGMENT_TWO].typeF;
    ackCode = 0X09;
    CUST_AckCode(g_uartTxBuf, ackCode, smoHandle->kSmo);
}

/**
  * @brief Set first order sliding mode observer's phase-locked loop parameters.
  * @param smoHandle The observer control handle.
  * @param rxData Receive buffer
  */
static void SetObserverSmo1thPLLParams(FOSMO_Handle *smoHandle, CUSTDATATYPE_DEF *rxData)
{
    /* Get command code. */
    int cmdCode =  (int)(rxData->data[DATA_SEGMENT_TWO].typeF);

    switch (cmdCode) {
        case SET_SMO1TH_PLL_BDW:        /* Set the bandwidth. */
            smoHandle->pll.pllBdw = rxData->data[DATA_SEGMENT_THREE].typeF;
            smoHandle->pll.pi.kp =  2.0f * smoHandle->pll.pllBdw;     /* kp = 2.0f * pllBdw */
            smoHandle->pll.pi.ki = smoHandle->pll.pllBdw * smoHandle->pll.pllBdw;   /* ki = pllBdw * pllBdw */
            ackCode = 0X0A;
            CUST_AckCode(g_uartTxBuf, ackCode, smoHandle->pll.pllBdw);
            break;
        case SET_SMO1TH_SPDFLITER_FC:   /* Set the cutoff frequency. */
            smoHandle->spdFilter.fc = rxData->data[DATA_SEGMENT_THREE].typeF;
            smoHandle->spdFilter.a1 = 1.0f / (1.0f + DOUBLE_PI * smoHandle->spdFilter.fc * CTRL_CURR_PERIOD);
            smoHandle->spdFilter.b1 = 1.0f - smoHandle->spdFilter.a1;
            ackCode = 0X0B;
            CUST_AckCode(g_uartTxBuf, ackCode, smoHandle->spdFilter.fc);
            break;
        case SET_SMO1TH_FILCOMPANGLE:   /* Set the compensation angle. */
            smoHandle->filCompAngle = rxData->data[DATA_SEGMENT_THREE].typeF;
            ackCode = 0X0C;
            CUST_AckCode(g_uartTxBuf, ackCode, smoHandle->filCompAngle);
            break;
        default:
            ackCode = 0X77;
            CUST_AckCode(g_uartTxBuf, ackCode, 0);
            break;
    }
}

/**
  * @brief Set fourth order sliding mode observer parameters.
  * @param smo4thHandle The observer control handle.
  * @param rxData Receive buffer
  */
static void SetObserverSmo4thParams(SMO4TH_Handle *smo4thHandle, CUSTDATATYPE_DEF *rxData)
{
    /* Get command code. */
    int cmdCode =  (int)(rxData->data[DATA_SEGMENT_TWO].typeF);

    switch (cmdCode) {
        case SET_SMO4TH_KD: /* Set d axis gain. */
            smo4thHandle->kd = rxData->data[DATA_SEGMENT_THREE].typeF;
            ackCode = 0X0D;
            CUST_AckCode(g_uartTxBuf, ackCode, smo4thHandle->kd);
            break;
        case SET_SMO4TH_KP: /* Set q axis gain. */
            smo4thHandle->kq  = rxData->data[DATA_SEGMENT_THREE].typeF;
            ackCode = 0X0E;
            CUST_AckCode(g_uartTxBuf, ackCode, smo4thHandle->kq);
            break;
        default:
            break;
    }
}

/**
  * @brief Set fourth order sliding mode observer's phase-locked loop parameters.
  * @param smo4thHandle The observer control handle.
  * @param rxData Receive buffer
  */
static void SetObserverSmo4thPLLParams(SMO4TH_Handle *smo4thHandle, CUSTDATATYPE_DEF *rxData)
{
    smo4thHandle->pll.pllBdw = rxData->data[DATA_SEGMENT_TWO].typeF;
    smo4thHandle->pll.pi.kp = (2.0f) * smo4thHandle->pll.pllBdw;         /* kp = 2.0f * pllBdw */
    smo4thHandle->pll.pi.ki = smo4thHandle->pll.pllBdw * smo4thHandle->pll.pllBdw;
    ackCode = 0X11;
    CUST_AckCode(g_uartTxBuf, ackCode, smo4thHandle->pll.pllBdw);
}

/**
  * @brief Set observer parameters.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  */
static void CMDCODE_SetObserverParams(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData)
{
    /* Get function code. */
    int funcCode = (int)(rxData->data[DATA_SEGMENT_ONE].typeF);

    if (funcCode == FOC_OBSERVERTYPE_SMO1TH) {
        SetObserverSmo1thParams(&mtrCtrl->smo, rxData);
    } else if (funcCode  == FOC_OBSERVERTYPE_SMO1TH_PLL) {
        SetObserverSmo1thPLLParams(&mtrCtrl->smo, rxData);
    } else if (funcCode  == FOC_OBSERVERTYPE_SMO4TH) {
        SetObserverSmo4thParams(&mtrCtrl->smo4th, rxData);
    } else if (funcCode  == FOC_OBSERVERTYPE_SMO4TH_PLL) {
        SetObserverSmo4thPLLParams(&mtrCtrl->smo4th, rxData);
    }
}

/**
  * @brief Set motor speed and speed slope.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  */
static void CMDCODE_SetMotorSpdAndSlope(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData)
{
    /* Get command code. */
    int cmdCode =  (int)(rxData->data[DATA_SEGMENT_TWO].typeF);

    switch (cmdCode) {
        case SET_SPD_COMMAND_HZ:    /* Set target speed(hz). */
            mtrCtrl->spdCmdHz  = rxData->data[DATA_SEGMENT_THREE].typeF * mtrCtrl->mtrParam.mtrNp / CONST_VALUE_60;
            /* Judgement the value > 0.00001, make sure denominator != 0 */
            if (rxData->data[DATA_SEGMENT_FOUR].typeF > 0.00001f) {
                mtrCtrl->spdRmg.delta = mtrCtrl->spdCmdHz / rxData->data[DATA_SEGMENT_FOUR].typeF * CTRL_SYSTICK_PERIOD;
            }
            ackCode = 0X16;
            CUST_AckCode(g_uartTxBuf, ackCode, mtrCtrl->spdCmdHz);
            break;
        case SET_SPD_RMG_SLOPE:     /* Set speed slope. */
            mtrCtrl->spdRmg.delta = mtrCtrl->spdCmdHz / rxData->data[DATA_SEGMENT_THREE].typeF * CTRL_SYSTICK_PERIOD;
            ackCode = 0X17;
            CUST_AckCode(g_uartTxBuf, ackCode, rxData->data[DATA_SEGMENT_THREE].typeF);
            break;
        default:
            break;
    }
}

/**
  * @brief Set motor base parameters.
  * @param mtrParamHandle The motor parameters handle.
  * @param rxData Receive buffer
  */
static void SetMotorBaseParams(MOTOR_Param *mtrParamHandle, CUSTDATATYPE_DEF *rxData)
{
    /* Get command code. */
    int cmdCode =  (int)(rxData->data[DATA_SEGMENT_TWO].typeF);

    switch (cmdCode) {
        case SET_MAX_ELEC_SPD:          /* Set max electric speed. */
            mtrParamHandle->maxElecSpd = rxData->data[DATA_SEGMENT_THREE].typeF /
                CONST_VALUE_60  * mtrParamHandle->mtrNp;
            ackCode = 0X18;
            CUST_AckCode(g_uartTxBuf, ackCode, rxData->data[DATA_SEGMENT_THREE].typeF);
            break;
        case SET_MOTOR_NUM_OF_PAIRS:    /* Set the number of motor pole pairs. */
            mtrParamHandle->mtrNp = (unsigned short)(rxData->data[DATA_SEGMENT_THREE].typeF);
            ackCode = 0X19;
            CUST_AckCode(g_uartTxBuf, ackCode, (float)mtrParamHandle->mtrNp);
            break;
        default:
            break;
    }
}

/**
  * @brief Set motor special parameters.
  * @param mtrParamHandle The motor parameters handle.
  * @param rxData Receive buffer
  */
static void SetMotorSpecialParams(MOTOR_Param *mtrParamHandle, CUSTDATATYPE_DEF *rxData)
{
    /* Get command code. */
    int cmdCode =  (int)(rxData->data[DATA_SEGMENT_TWO].typeF);

    switch (cmdCode) {
        case SET_MOTOR_RES_OF_STATOR:       /* Set the resistor of stator. */
            mtrParamHandle->mtrRs = rxData->data[DATA_SEGMENT_THREE].typeF;
            ackCode = 0X1A;
            CUST_AckCode(g_uartTxBuf, ackCode, mtrParamHandle->mtrRs);
            break;
        case SET_MOTOR_DAXIS_INDUCTANCE:    /* Set the d axis inductance. */
            mtrParamHandle->mtrLd = rxData->data[DATA_SEGMENT_THREE].typeF;
            ackCode = 0X1B;
            CUST_AckCode(g_uartTxBuf, ackCode, mtrParamHandle->mtrLd);
            break;
        case SET_MOTOR_QAXIS_INDUCTANCE:    /* Set the q axis inductance. */
            mtrParamHandle->mtrLq = rxData->data[DATA_SEGMENT_THREE].typeF;
            ackCode = 0X1C;
            CUST_AckCode(g_uartTxBuf, ackCode, mtrParamHandle->mtrLq);
            break;
        default:
            ackCode = 0X77;
            CUST_AckCode(g_uartTxBuf, ackCode, 0);
            break;
    }
}

/**
  * @brief Set motor board parameters.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  */
static void SetMotorBoardParams(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData)
{
    /* Get command code. */
    int cmdCode =  (int)(rxData->data[DATA_SEGMENT_TWO].typeF);

    switch (cmdCode) {
        case SET_SVPWM_VOLTAGE_PER_UNIT:        /* Set svpwm voltage per unit. */
            mtrCtrl->sv.voltPu = rxData->data[DATA_SEGMENT_THREE].typeF * ONE_DIV_SQRT3;
            mtrCtrl->currCtrl.outLimit = mtrCtrl->sv.voltPu * ONE_DIV_SQRT3;
            ackCode = 0X1D;
            CUST_AckCode(g_uartTxBuf, ackCode, rxData->data[DATA_SEGMENT_THREE].typeF);
            break;
        case SET_ADC_CURRENT_SAMPLE_COFE:       /* Set adc current sample cofeature. */
            mtrCtrl->adcCurrCofe = rxData->data[DATA_SEGMENT_THREE].typeF;
            ackCode = 0X1E;
            CUST_AckCode(g_uartTxBuf, ackCode, mtrCtrl->adcCurrCofe);
            break;
        case SET_CURRENT_LOOP_CONTROL_PERIOD:   /* Set current loop control period. */
            mtrCtrl->currCtrlPeriod = 1 / rxData->data[DATA_SEGMENT_THREE].typeF * CONST_VALUE_DIV_1000;
            ackCode = 0X1F;
            CUST_AckCode(g_uartTxBuf, ackCode, rxData->data[DATA_SEGMENT_THREE].typeF);
            break;
        default:
            break;
    }
}

/**
  * @brief Set motor parameters.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  */
static void CMDCODE_SetMotorParams(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData)
{
    /* Get function code. */
    int funcCode = (int)(rxData->data[DATA_SEGMENT_ONE].typeF);

    if (funcCode == MOTOR_PARAMS_BASE) {
        SetMotorBaseParams(&mtrCtrl->mtrParam, rxData);
    } else if (funcCode  == MOTOR_PARAMS_SPECIAL) {
        SetMotorSpecialParams(&mtrCtrl->mtrParam, rxData);
    } else if (funcCode  == MOTOR_PARAMS_BOARD) {
        SetMotorBoardParams(mtrCtrl, rxData);
    }
}

/**
  * @brief Motor start.
  * @param mtrCtrl The motor control handle.
  */
static void CMDCODE_MotorStart(MTRCTRL_Handle *mtrCtrl)
{
    if (mtrCtrl->stateMachine != FSM_RUN) {
        SysCmdStartSet(&mtrCtrl->statusReg);    /* start motor. */
        mtrCtrl->motorStateFlag = 1;
        ackCode = 0X24; /* send ackcode to host. */
        CUST_AckCode(g_uartTxBuf, ackCode, 1);
    }
}

/**
  * @brief Motor stop.
  * @param mtrCtrl The motor control handle.
  */
static void CMDCODE_MotorStop(MTRCTRL_Handle *mtrCtrl)
{
    SysCmdStopSet(&mtrCtrl->statusReg);
    mtrCtrl->motorStateFlag = 0;
    ackCode = 0X25;
    CUST_AckCode(g_uartTxBuf, ackCode, 0);
}

/**
  * @brief Motor state reset.
  * @param mtrCtrl The motor control handle.
  */
static void CMDCODE_MotorReset(MTRCTRL_Handle *mtrCtrl)
{
    BASE_FUNC_UNUSED(mtrCtrl);
    BASE_FUNC_SoftReset();
}

/**
  * @brief Set IF-Startup parameters.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  */
static void SetStartupIFParams(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData)
{
    /* Get command code. */
    int cmdCode =  (int)(rxData->data[DATA_SEGMENT_TWO].typeF);

    switch (cmdCode) {
        case SET_IF_TARGET_CURRENT_VALUE:   /* Set I/F start up target current value. */
            mtrCtrl->ifCtrl.targetAmp = rxData->data[DATA_SEGMENT_THREE].typeF;
            ackCode = 0X26;
            CUST_AckCode(g_uartTxBuf, ackCode, mtrCtrl->ifCtrl.targetAmp);
            break;
        case SET_INCREMENT_OF_IF_CURRENT:   /* Set increment of I/F start up current. */
            mtrCtrl->ifCtrl.stepAmp = mtrCtrl->ifCtrl.targetAmp / rxData->data[DATA_SEGMENT_THREE].typeF *
                CTRL_SYSTICK_PERIOD;
            ackCode = 0X27;
            CUST_AckCode(g_uartTxBuf, ackCode, mtrCtrl->ifCtrl.stepAmp);
            break;
        case SET_SPEED_RING_BEGIN_SPEED:    /* Set speed ring begin speed. */
            mtrCtrl->startup.spdBegin = rxData->data[DATA_SEGMENT_THREE].typeF /
                CONST_VALUE_60 *  mtrCtrl->mtrParam.mtrNp;
            ackCode = 0X28;
            CUST_AckCode(g_uartTxBuf, ackCode, rxData->data[DATA_SEGMENT_THREE].typeF);
            break;
        default:
            break;
    }
}

/**
  * @brief Set start up parameters.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  */
static void CMDCODE_SetStartupParams(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData)
{
    /* Get function code. */
    int funcCode = (int)(rxData->data[DATA_SEGMENT_ONE].typeF);

    if (funcCode == FOC_STARTUP_IF) {
        SetStartupIFParams(mtrCtrl, rxData);
    }
}

/**
  * @brief Set adjust speed mode.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  */
static void CMDCODE_SetAdjustSpdMode(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData)
{
    /* Get function code. */
    int funcCode = (int)(rxData->data[DATA_SEGMENT_ONE].typeF);
    /* Get commond code. */
    int cmdCode =  (int)(rxData->data[DATA_SEGMENT_TWO].typeF);
    if (funcCode == HOST_SPEED_ADJUST) {
        mtrCtrl->spdAdjustMode = HOST_SPEED_ADJUST;
        /* Uart connect success. */
        mtrCtrl->uartConnectFlag = CONNECTING;
        ackCode = 0X2A;
        CUST_AckCode(g_uartTxBuf, ackCode, (float)mtrCtrl->spdAdjustMode);
    } else if (funcCode == CUST_SPEED_ADJUST) {
        if (cmdCode == 1) { /* If uart connection disconnected & stop motor commond. */
            SysCmdStopSet(&mtrCtrl->statusReg);
            mtrCtrl->motorStateFlag = 0;
        }
        mtrCtrl->spdAdjustMode = CUST_SPEED_ADJUST;
        ackCode = 0X2B;
        CUST_AckCode(g_uartTxBuf, ackCode, (float)mtrCtrl->spdAdjustMode);
        /* Uart disconnect. */
        mtrCtrl->uartConnectFlag = DISCONNECT;
    }
}

/**
  * @brief Check uart connect.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  */
static void CMDCODE_UartHandShake(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData)
{
    CMDCODE_SetAdjustSpdMode(mtrCtrl, rxData);
}

/**
  * @brief Set Motor Initial Status Parameters.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  * @param code Instruction code.
  */
static void CMDCODE_EXE_SetMotorInitParams(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData, unsigned char code)
{
    switch (code) {
        case CMDCODE_SET_OBSERVER_TYPE: {   /* Set observer type. */
                CMDCODE_SetObserverType(mtrCtrl, rxData);
            }
            break;
        case CMDCODE_SET_PID_PARAMS: {      /* Set motor pid parameters. */
                CMDCODE_SetMotorPidParams(mtrCtrl, rxData);
            }
            break;
        case CMDCODE_SET_OBSERVER_PARAMS: { /* Set observer parameters. */
                CMDCODE_SetObserverParams(mtrCtrl, rxData);
            }
            break;
        case CMDCODE_SET_MOTOR_TARGETSPD: { /* Set motor speed and speed slope. */
                CMDCODE_SetMotorSpdAndSlope(mtrCtrl, rxData);
            }
            break;
        case CMDCODE_SET_MOTOR_PARAMS: {    /* Set motor parameters. */
                CMDCODE_SetMotorParams(mtrCtrl, rxData);
            }
            break;
        case CMDCODE_SET_ADJUSTSPD_MODE:    /* Set adjust speed mode. */
                CMDCODE_SetAdjustSpdMode(mtrCtrl, rxData);
            break;
        default:
            break;
    }
}

/**
  * @brief Set Motor Control System Status.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  * @param code Instruction code.
  */
static void CMDCODE_EXE_SetMotorState(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData, unsigned char code)
{
    BASE_FUNC_UNUSED(rxData);
    switch (code) {
        case CMDCODE_MOTOR_START: {         /* Motor start command */
                CMDCODE_MotorStart(mtrCtrl);
            }
            break;
        case CMDCODE_MOTOR_STOP: {          /* Motor stop command */
                CMDCODE_MotorStop(mtrCtrl);
            }
            break;
        case CMDCODE_MOTORSTATE_RESET: {   /* Motor reset command */
                CMDCODE_MotorReset(mtrCtrl);
            }
            break;
        default:
            break;
    }
}

/**
  * @brief Set Startup and Uart Link Handshake Flag Parameters.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  * @param code Instruction code.
  */
static void CMDCODE_EXE_SetOtherParams(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData, unsigned char code)
{
    switch (code) {
        case CMDCODE_SET_STARTUP_PARAMS:    /* Set start up parameters. */
                CMDCODE_SetStartupParams(mtrCtrl, rxData);
            break;
        case CMDCODE_UART_HANDSHAKE: {    /* Check uart hand shake. */
                CMDCODE_UartHandShake(mtrCtrl, rxData);
            }
            break;
        case CMDCODE_UART_HEARTDETECT: {    /* Check uart hand shake. */
                mtrCtrl->uartHeartDetCnt++;
            }
            break;
        default:
            break;
    }
}
/**
  * @brief Instruction code executor.
  * @param mtrCtrl The motor control handle.
  * @param rxData Receive buffer
  * @param code Instruction code.
  */
static void CMDCODE_EXE_Process(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *rxData, unsigned char code)
{
    CMDCODE_EXE_SetMotorInitParams(mtrCtrl, rxData, code);
    CMDCODE_EXE_SetMotorState(mtrCtrl, rxData, code);
    CMDCODE_EXE_SetOtherParams(mtrCtrl, rxData, code);
}

/**
  * @brief Host data download callback and data parsing.
  * @param mtrCtrl The motor control handle.
  * @param rxBuf Receive buffer
  */
void CUST_UartDataProcess(MTRCTRL_Handle *mtrCtrl, unsigned char *rxBuf)
{
    /* Verify Parameters */
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(rxBuf != NULL);

    /* Uart data storage struct */
    CUSTDATATYPE_DEF data;
    volatile unsigned char *ptrCnt = &rxBuf[FRAME_CHECK_BEGIN + 1];
    volatile char *strCnt = &data.data[0].typeCh[0];
    for (unsigned int j = 0; j < FRAME_RECV_DATA_LENTH * FRAME_ONE_DATA_LENTH; j++) {
        *strCnt++ = *ptrCnt++;
    }
    /* Message function code. */
    data.code = rxBuf[FRAME_CHECK_BEGIN];
    CMDCODE_EXE_Process(mtrCtrl, &data, data.code);
}

/**
 * @brief The host computer displays data transmission.
 * @param mtrCtrl The motor control handle.
 * @param txData Message content.
 * @param stage Message status function code.
 */
void CUST_SetTxMsg(MTRCTRL_Handle *mtrCtrl, CUSTDATATYPE_DEF *txData)
{
    /* Verify Parameters */
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(txData != NULL);
    if (mtrCtrl->stateMachine == FSM_IDLE) {
        mtrCtrl->smo.spdEst = 0.0f;
    }
    /* Data send to host. */
    txData->data[CURRDQ_Q].typeF = mtrCtrl->idqFbk.q;
    txData->data[CURRDQ_D].typeF = mtrCtrl->idqFbk.d;
    txData->data[CURRREFDQ_Q].typeF = mtrCtrl->idqRef.q;
    txData->data[CURRREFDQ_D].typeF = mtrCtrl->idqRef.d;
    /* Motor current speed. */
    txData->data[CURRSPD].typeF = mtrCtrl->smo.spdEst * CONST_VALUE_60 / mtrCtrl->mtrParam.mtrNp;
    /* Motor commond speed. */
    txData->data[SPDCMDHZ].typeF = mtrCtrl->spdCmdHz * CONST_VALUE_60 / mtrCtrl->mtrParam.mtrNp;
    /* Bus voltage. */
    txData->data[UDC].typeF = mtrCtrl->udc;
    /* Power board temprature. */
    txData->data[POWERBOARDTEMP].typeF = mtrCtrl->powerBoardTemp;
    /* Motor protection status flag. */
    txData->data[CUST_ERR_CODE].typeI = mtrCtrl->prot.motorErrStatus.all;
    /* Three phase current. */
    txData->data[CURRUVW_U].typeF = mtrCtrl->currUvw.u;
    txData->data[CURRUVW_V].typeF = mtrCtrl->currUvw.v;
    txData->data[CURRUVW_W].typeF = mtrCtrl->currUvw.w;
    /* Three phase pwm duty. */
    txData->data[PWMDUTYUVW_U].typeF = mtrCtrl->dutyUvw.u;
    txData->data[PWMDUTYUVW_V].typeF = mtrCtrl->dutyUvw.v;
    txData->data[PWMDUTYUVW_W].typeF = mtrCtrl->dutyUvw.w;
    /* Motor electric angle. */
    txData->data[AXISANGLE].typeF = mtrCtrl->axisAngle;
    txData->data[VDQ_Q].typeF = mtrCtrl->vdqRef.q;
    txData->data[VDQ_D].typeF = mtrCtrl->vdqRef.d;
    txData->data[SPDREFHZ].typeF = mtrCtrl->spdRefHz * CONST_VALUE_60 / mtrCtrl->mtrParam.mtrNp;
    txData->data[SENDTIMESTAMP].typeF = mtrCtrl->uartTimeStamp;
}