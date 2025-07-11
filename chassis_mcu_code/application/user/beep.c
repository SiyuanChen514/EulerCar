#include "beep.h"
#include "main.h"
#include "debug.h"

/**
  * @brief GPT run and modify period and duty during running.
  * @param None.
  * @retval None.
  */
void GPT_SampleMain(void)
{
    DBG_PRINTF("GPT Continued Run begin\r\n");
    HAL_GPT_Start(&g_gpt3);
    // BASE_FUNC_DelaySeconds(10); /* Delay 10 seconds */
    // DBG_PRINTF("Change the duty to 50%%\r\n");

    // HAL_GPT_GetConfig(&g_gpt3);
    // g_gpt3.period = 59999;          /* 59999 is the number of GPT counting cycles. */
    // g_gpt3.refA0.refdot = 20000;    /* 20000 is the value of PWM reference point A. */
    // g_gpt3.refB0.refdot = 50000;    /* 50000 is the value of PWM reference point A. */
    // HAL_GPT_Config(&g_gpt3);
}
