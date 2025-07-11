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
  * @file      main.c
  * @author    MCU Driver Team
  * @brief     Main program body.
  */

#include "typedefs.h"
#include "feature.h"
#include "main.h"
/* USER CODE BEGIN 0 */
/* USER CODE 区域内代码不会被覆盖，区域外会被生成的默认代码覆盖（其余USER CODE 区域同理） */
#include "debug.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "uart.h"
#include "user_motor_control.h"
#include "button.h"
#include "adc_demo.h"
#include "gpt.h"
#include "beep.h"
#include "gyro.h"
#include "encoder.h"
#include "eulercar_control.h"
#include "debug.h"
#include "pid.h"
#include "can_interfaces.h"
/* 建议用户放置头文件 */
#include "grab.h"
#include "debug_uart.h"
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
CAN_Handle g_can;
QDM_Handle g_qdm0;
QDM_Handle g_qdm1;
GPT_Handle g_gpt0;
GPT_Handle g_gpt1;
GPT_Handle g_gpt2;
GPT_Handle g_gpt3;
TIMER_Handle g_timer0;
UART_Handle g_uart0;
UART_Handle g_uart2;
UART_Handle g_uart3;
I2C_Handle g_i2c0;
I2C_Handle g_i2c1;
APT_Handle g_apt0;
APT_Handle g_apt1;
ADC_Handle g_adc0;
GPIO_Handle g_gpio1;
GPIO_Handle g_gpio2;
GPIO_Handle g_gpio5;

bool isRecv = false;
bool isWrite = false;
unsigned char ServoMsg;

char g_motorTypeStr[4][20] = {
     "1:45 HALL",
     "1:45 GMR",
     "1:90 HALL",
     "1:90 GMR",
};

/*
//减速比为1:90的霍尔传感器电机
float g_motorMaxSpeed = MOTOR_MAX_SPEED_GEAR_90;
unsigned int g_curMotorType  = MOTOR_TYPE_GEAR_HALL_90;
unsigned int g_motorLineNum  = MOTOR_LINE_NUM_HALL_90;

//减速比为1:90的GMR传感器电机
float g_motorMaxSpeed = MOTOR_MAX_SPEED_GEAR_90;
unsigned int g_curMotorType  = MOTOR_TYPE_GEAR_GMR_90;
unsigned int g_motorLineNum  = MOTOR_LINE_NUM_GMR_90;

//减速比为1:45的霍尔传感器电机
float g_motorMaxSpeed = MOTOR_MAX_SPEED_GEAR_45;
unsigned int g_curMotorType  = MOTOR_TYPE_GEAR_HALL_45;
unsigned int g_motorLineNum  = MOTOR_LINE_NUM_HALL_45;

//减速比为1:45的GMR传感器电机
float g_motorMaxSpeed = MOTOR_MAX_SPEED_GEAR_45;
unsigned int g_curMotorType  = MOTOR_TYPE_GEAR_GMR_45;
unsigned int g_motorLineNum  = MOTOR_LINE_NUM_GMR_45;
*/

//减速比为1:90的GMR传感器电机
float g_motorMaxSpeed = MOTOR_MAX_SPEED_GEAR_90;
unsigned int g_curMotorType  = MOTOR_TYPE_GEAR_GMR_90;
unsigned int g_motorLineNum  = MOTOR_LINE_NUM_GMR_90;

bool is_servo_done = true;
bool Action = 1;  // 1 表示放下; 0表示举起
/* 建议用户定义全局变量、结构体、宏定义或函数声明等 */
/* USER CODE END 1 */

int main(void)
{
    SystemInit();
    InitGearMotor();
    Pid_Init();
    ssd1306_Init();
    // HAL_GPT_Start(&g_gpt0);
    // HAL_GPT_Start(&g_gpt1);
    
    DBG_PRINTF("==============================================================\r\n");
    DBG_PRINTF("                • EulerCar Controller 1.0 •                   \r\n");
    DBG_PRINTF("                                                              \r\n");
    DBG_PRINTF("  ➤ System Information:                                      \r\n");
    DBG_PRINTF("      • Motor Type:%s\r\n",g_motorTypeStr[g_curMotorType]);
    DBG_PRINTF("      • Motor Encode Line Number:%05d\r\n",g_motorLineNum);
    DBG_PRINTF("      • Motor Max Speed:%.02fmm/s\r\n", g_motorMaxSpeed);
    DBG_PRINTF("      • EulerCar Data Send Period:%dms,%dHZ\r\n", EULER_CAR_DATA_SEND_PERIOD, 1000/EULER_CAR_DATA_SEND_PERIOD);
    DBG_PRINTF("                                                              \r\n");                                    
    DBG_PRINTF("  ➤ PID Information:                                         \r\n");
    DBG_PRINTF("      • KP:%.02f    • KI:%.02f    • KD:%.02f\r\n",g_KP,g_KI,g_KD);
    DBG_PRINTF("      • PID Control Period:%dms\r\n",MOTOR_PID_CONTROL_PERIOD);
    DBG_PRINTF("==============================================================\r\n");
    DBG_PRINTF("EulerCar MCU init success!!!\r\n");
    HAL_TIMER_Start(&g_timer0);
    DBG_PRINTF("TIMER start\r\n");
    // InitButtonFunction();
    // test_can();
    HAL_GPIO_SetValue(&g_gpio2, GPIO_PIN_6, GPIO_LOW_LEVEL);
    UART3_INTRxSimultaneously();
    // test_uart();
    /* USER CODE END 5 */
    return BASE_STATUS_OK;
}




