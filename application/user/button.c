#include "debug.h"
#include "gpio.h"
#include "main.h"
#include "button.h"
#include "user_motor_control.h"

#define PREVENT_SWIPE_SCREEN_TIME    50
#define CYCLE_INTERVAL_TIME   500

int g_button1State = 0;
int g_button2State = 0;

//按键滤波计数器
int g_button1_count = 0;
int g_button2_count = 0;

unsigned int g_testIndex = 2;
unsigned int g_testAngles = ADJUST_PLUS_90;

static unsigned int g_angleDate[][2] = {
    {1, 76},
    {1, 151},
    {1, 226},
    {1, 301},
    {1, 376}
};

void AngleAdjustment(unsigned int index, unsigned int angles)
{
    if (angles > ADJUST_PLUS_90) {
        return; 
    }

    GPT_ReferCfg refer;
    refer.refA0.refAction = GPT_ACTION_OUTPUT_HIGH;
    refer.refB0.refAction = GPT_ACTION_OUTPUT_LOW;
    refer.refA0.refdot = g_angleDate[angles][0];
    refer.refA0.refdot = g_angleDate[angles][1];
    switch (index) {
    case 0:
        HAL_GPT_SetReferCounterAndAction(&g_gpt0, &refer);
        break;
    case 1:
        HAL_GPT_SetReferCounterAndAction(&g_gpt1, &refer);
        break;
    case 2:
        HAL_GPT_SetReferCounterAndAction(&g_gpt2, &refer);
        break;
    default:
        break;
    }
}

/**
  * @brief GPIO register interrupt callback function.
  * @param param Value of @ref GPIO_Handle.
  * @retval None
  */


void GPIO1_0_CallbackFunc(void *param)
{
    BASE_FUNC_UNUSED(param);
    if((g_button1State == 0) && (g_button1_count == 0))
    {
        //g_button1State = !g_button1State;
        g_button1State = 1;
        g_button1_count = 1;    //在10ms定时中断里处理按键滤波业务逻辑
    }
}

void GPIO1_1_CallbackFunc(void *param)
{
    BASE_FUNC_UNUSED(param);
   if((g_button2State == 0) && (g_button2_count == 0))
   {
        //g_button2State = !g_button2State;
        g_button2State = 1;
        g_button2_count = 1;    //在10ms定时中断里处理按键滤波业务逻辑
   }
}

void InitButtonFunction(void)
{
    // HAL_GPT_Start(&g_gpt0);
    // HAL_GPT_Start(&g_gpt1);
    // HAL_GPT_Start(&g_gpt2);
}


BASE_StatusType ButtonPrintSample(void)
{
    while (1) {
        BASE_FUNC_DELAY_MS(1);
        if (g_button1State) {
            g_button1State = !g_button1State;
            DBG_PRINTF("g_button1State Evnet\r\n");
        }

        if (g_button2State) {
            g_button2State = !g_button2State;
            DBG_PRINTF("g_button2State Evnet\r\n");
        }
    }
    return BASE_STATUS_OK;
}

BASE_StatusType GPIO_KeySample(void)
{
    HAL_GPT_Start(&g_gpt0);
    HAL_GPT_Start(&g_gpt1);
    HAL_GPT_Start(&g_gpt2);
    while (1) {
        BASE_FUNC_DELAY_MS(100);
        if (g_button1State) {
            g_button1State = !g_button1State;
            startMotor();
            MotorForwardRotation();
            BASE_FUNC_DELAY_MS(3000);
            stopMotor();
        }
    }
    return BASE_STATUS_OK;
}