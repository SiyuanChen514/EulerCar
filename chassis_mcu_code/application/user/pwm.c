#include "pwm.h"
#include "uart.h"
#include "debug.h"


void set_duty(GPT_Handle* gpt_handle, int duty){
    // unsigned int period = HAL_GPT_GetCountPeriod(gpt_handle);
    HAL_GPT_GetConfig(gpt_handle);
    gpt_handle->refA0.refAction = GPT_ACTION_OUTPUT_HIGH;
    gpt_handle->refA0.refdot = 1;
    gpt_handle->refB0.refAction = GPT_ACTION_OUTPUT_LOW;
    gpt_handle->refB0.refdot = duty;
    HAL_GPT_Config(gpt_handle);
}


// void counterwise(GPT_Handle* gpt_handle, unsigned short time, unsigned short duty){
//     DBG_PRINTF("COUNTERWISE\r\n");
//     set_duty(gpt_handle, duty); // 650
//     HAL_GPT_Start(gpt_handle);
//     BASE_FUNC_DELAY_MS(time);
//     HAL_GPT_Stop(gpt_handle);
// }


void Rotate(GPT_Handle* gpt_handle, unsigned short time,  unsigned short duty){
    DBG_PRINTF("CLOCKWISE\r\n");
    set_duty(gpt_handle, duty); //350
    HAL_GPT_Start(gpt_handle);
    BASE_FUNC_DELAY_MS(time);
    HAL_GPT_Stop(gpt_handle);
}

void up(unsigned short time, unsigned short duty){
    Rotate(&g_gpt2, time, duty);
}

void down(unsigned short time, unsigned short duty){
    Rotate(&g_gpt2, time, duty);
}

void open(unsigned short time, unsigned short duty){
    Rotate(&g_gpt0, time, duty);
}

void _close(unsigned short time, unsigned short duty){
    Rotate(&g_gpt0, time, duty);
}