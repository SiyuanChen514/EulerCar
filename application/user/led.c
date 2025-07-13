#include "debug.h"
#include "gpio.h"
#include "main.h"
#include "led.h"

#define CYCLE_INTERVAL_TIME   500

/* ---------------------------------- Sample Parameters -------------------------------- */
/**
  * @brief Test GPIO PIN control LED.
  * @param None
  * @retval Value of @ref BASE_StatusType.
  */
BASE_StatusType GPIO_LedSample(void)
{
    /* Cycle control LED on and off. */
    while (1) {
        BASE_FUNC_DELAY_MS(2000);
        HAL_GPIO_TogglePin(&g_gpio2, GPIO_PIN_5);
        BASE_FUNC_DELAY_MS(2000);
        HAL_GPIO_TogglePin(&g_gpio2, GPIO_PIN_6);
        BASE_FUNC_DELAY_MS(2000);
        HAL_GPIO_TogglePin(&g_gpio2, GPIO_PIN_7);
        BASE_FUNC_DELAY_MS(2000);
        HAL_GPIO_TogglePin(&g_gpio5, GPIO_PIN_1);
        DBG_PRINTF("LED Stata reverse! \r\n");
    }
    return BASE_STATUS_OK;
}