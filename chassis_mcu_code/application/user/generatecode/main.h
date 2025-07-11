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
  * @file      main.h
  * @author    MCU Driver Team
  * @brief     This file contains driver init functions.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_SYSTEM_INIT_H
#define McuMagicTag_SYSTEM_INIT_H

#include "adc.h"
#include "adc_ex.h"
#include "apt.h"
#include "uart.h"
#include "uart_ex.h"
#include "i2c.h"
#include "i2c_ex.h"
#include "can.h"
#include "qdm.h"
#include "gpio.h"
#include "gpt.h"
#include "gpt_ex.h"
#include "timer.h"
#include "timer_ex.h"
#include "crg.h"
#include "iocmg.h"

#define    IO_SPEED_FAST     0x00U
#define    IO_SPEED_SLOW     0x01U

#define    IO_DRV_LEVEL4     0x00U
#define    IO_DRV_LEVEL3     0x01U
#define    IO_DRV_LEVEL2     0x02U
#define    IO_DRV_LEVEL1     0x03U

#define    XTAL_DRV_LEVEL4   0x03U
#define    XTAL_DRV_LEVEL3   0x02U
#define    XTAL_DRV_LEVEL2   0x01U
#define    XTAL_DRV_LEVEL1   0x00U

#define    MOTOR_TYPE_GEAR_HALL_45   0
#define    MOTOR_TYPE_GEAR_GMR_45    1
#define    MOTOR_TYPE_GEAR_HALL_90   2
#define    MOTOR_TYPE_GEAR_GMR_90    3

#define    MOTOR_LINE_NUM_HALL_45    585     
#define    MOTOR_LINE_NUM_GMR_45     22500
#define    MOTOR_LINE_NUM_HALL_90    1170
#define    MOTOR_LINE_NUM_GMR_90     45000

#define    MOTOR_TIRE_DIAMETER       67.0      //轮胎直径67mm

#define    MOTOR_MAX_ANGULAR_SPEED   100000.0  //最大角速度，单位mrad/s
#define    MOTOR_MAX_SPEED_GEAR_90   650.0     //空载情况下减速比为1:90的电机，带直径为67mm的轮胎，最高速度650mm/s
#define    MOTOR_MAX_SPEED_GEAR_45   1280.0    //空载情况下减速比为1:45的电机，带直径为67mm的轮胎，最高速度1280mm/s

extern GPT_Handle g_gpt0;
extern GPT_Handle g_gpt1;
extern GPT_Handle g_gpt2;
extern GPT_Handle g_gpt3;
extern CAN_Handle g_can;
extern QDM_Handle g_qdm0;
extern QDM_Handle g_qdm1;
extern TIMER_Handle g_timer0;
extern UART_Handle g_uart0;
extern UART_Handle g_uart2;
extern UART_Handle g_uart3;
extern I2C_Handle g_i2c0;
extern I2C_Handle g_i2c1;
extern APT_Handle g_apt0;
extern APT_Handle g_apt1;
extern ADC_Handle g_adc0;

extern GPIO_Handle g_gpio1;
extern GPIO_Handle g_gpio2;
extern GPIO_Handle g_gpio5;

extern float g_motorMaxSpeed;          //电机最大速度
extern unsigned int g_curMotorType;    //电机类型 
extern unsigned int g_motorLineNum;    //电机编码器线数

extern unsigned int g_dataSendPeriod;  //向上位机发送数据的周期  

extern float g_KP;
extern float g_KI;
extern float g_KD;

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect);
void SystemInit(void);

void UART2WriteInterruptCallback(void *handle);
void UART2ReadInterruptCallback(void *handle);
void UART3WriteInterruptCallback(void *handle);
void UART3ReadInterruptCallback(void *handle);

void UART2InterruptErrorCallback(void *handle);
void UART3InterruptErrorCallback(void *handle);
void QDM1PTUCycleCallback(void *handle);
void QDM1SpeedLoseCallback(void *handle);
void QDM1ZIndexLockedCallback(void *handle);
void QDM1PositionCompareMatchCallback(void *handle);
void QDM1PositionCompareReadyCallback(void *handle);
void QDM1PositionCounterOverflowCallback(void *handle);
void QDM1PositionCounterUnderflowCallback(void *handle);
void QDM1QuadratureDirectionChangeCallback(void *handle);
void QDM1QuadraturePhaseErrorCallback(void *handle);
void QDM1PositionCounterErrorCallback(void *handle);
void TIMER0_InterruptProcess(void *handle);
void TIMER0_DMAOverFlow_InterruptProcess(void *handle);
void QDM0PTUCycleCallback(void *handle);
void QDM0SpeedLoseCallback(void *handle);
void QDM0ZIndexLockedCallback(void *handle);
void QDM0PositionCompareMatchCallback(void *handle);
void QDM0PositionCompareReadyCallback(void *handle);
void QDM0PositionCounterOverflowCallback(void *handle);
void QDM0PositionCounterUnderflowCallback(void *handle);
void QDM0QuadratureDirectionChangeCallback(void *handle);
void QDM0QuadraturePhaseErrorCallback(void *handle);
void QDM0PositionCounterErrorCallback(void *handle);

void CANWriteCallbackFunc(void* handle);
void CANReadCallbackFunc(void* handle);
void CANTransCallbackFunc(void* handle);

void GPIO1_0_CallbackFunc(void *param);
void GPIO1_1_CallbackFunc(void *param);

#endif /* McuMagicTag_SYSTEM_INIT_H */