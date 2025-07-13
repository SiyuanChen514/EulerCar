#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <securec.h>
#include "debug.h"
#include "gpio.h"
#include "main.h"
#include "pid.h"

extern unsigned int g_TimerInterruptCount; 
unsigned int g_pid_count =0;

/* 增量式PID控制结构体 */
PidEulerCar g_pidEulerCarRight;
PidEulerCar g_pidEulerCarLeft;

//增量式PID参数
float g_KP = 1.2;
float g_KI = 15.0;
float g_KD = 0.0;

//通过按键调整PID参数的步进增量参数
float g_KP_Step  = 0.1;
float g_KI_Step  = 0.1;
float g_KD_Step  = 0;

void Pid_Init(void)
{
    g_pidEulerCarRight.SetSpeed = 0;
    g_pidEulerCarRight.ActualSpeed = 0;
    g_pidEulerCarRight.duty = 0;       
    g_pidEulerCarRight.err = 0;
    g_pidEulerCarRight.err_next = 0;
    g_pidEulerCarRight.err_last = 0;
    g_pidEulerCarRight.IncSpeed = 0; 
    g_pidEulerCarRight.TargetIncSpeed = 0; 
    //DBG_PRINTF("pid right init success\r\n");

    g_pidEulerCarLeft.SetSpeed = 0;
    g_pidEulerCarLeft.ActualSpeed = 0;
    g_pidEulerCarLeft.duty = 0;   
    g_pidEulerCarLeft.err = 0;
    g_pidEulerCarLeft.err_next = 0;
    g_pidEulerCarLeft.err_last = 0;
    g_pidEulerCarLeft.IncSpeed = 0; 
    g_pidEulerCarLeft.TargetIncSpeed = 0; 
    //DBG_PRINTF("pid left init success\r\n");
}


float Pid_Ctrl(PidEulerCar *pMotor)
{
    float IncrementSpeed;
    unsigned int duty;
   
    //计算当前误差
    pMotor->err = pMotor->SetSpeed - pMotor->ActualSpeed;
  
    //增量式PID算法计算出增量，越接近目标速度，增量越接近零
    //增量式PID算法参数设定策略，先确定KI，再调KP，最后式KD
    IncrementSpeed =  g_KP * (pMotor->err - pMotor->err_next)
                    + g_KI * pMotor->err 
                    + g_KD * (pMotor->err - 2 * pMotor->err_next + pMotor->err_last);

    pMotor->TargetIncSpeed =  pMotor->TargetIncSpeed + IncrementSpeed;
    pMotor->IncSpeed = IncrementSpeed;

    //限制幅度
    if(pMotor->SetSpeed > 0){
        if (pMotor->TargetIncSpeed > g_motorMaxSpeed){
            pMotor->TargetIncSpeed = g_motorMaxSpeed;
        }
        if (pMotor->TargetIncSpeed < 0){
            pMotor->TargetIncSpeed = 0;
        }
    }

    if(pMotor->SetSpeed < 0){
        if (pMotor->TargetIncSpeed < -g_motorMaxSpeed){
            pMotor->TargetIncSpeed = -g_motorMaxSpeed;
        }
        if (pMotor->TargetIncSpeed > 0){
            pMotor->TargetIncSpeed = 0;
        }
    }
     
    //电机需要设置的占空比
    duty = (abs)((int)(pMotor->TargetIncSpeed/g_motorMaxSpeed * 100.0));   
    if (duty > 99) {
        duty = 99;
    }
    pMotor->duty = duty;

    pMotor->err_last = pMotor->err_next;
    pMotor->err_next = pMotor->err;

    return pMotor->TargetIncSpeed;
}
