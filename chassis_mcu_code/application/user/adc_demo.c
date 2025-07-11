#include "adc_demo.h"
#include "main.h"
#include "debug.h"

/**
  * @brief ADC single channel sample without DMA and interrupt.
  * @param None.
  * @retval None.
  */
void ADC_SingleTrigger(void)
{
    DBG_PRINTF("ADC_SingleTrigger begin\r\n");

    HAL_ADC_SoftTrigSample(&g_adc0, ADC_SOC_NUM0);  /* Software trigger ADC sampling */

    BASE_FUNC_DELAY_MS(10);  /* delay 10 ms */
  
    if (HAL_ADC_CheckSocFinish(&g_adc0, ADC_SOC_NUM0) == BASE_STATUS_ERROR) {
        DBG_PRINTF("ADC did not complete sampling and conversion\r\n");
        return;
    }

    unsigned int ret = HAL_ADC_GetConvResult(&g_adc0, ADC_SOC_NUM0); /* Software trigger ADC sampling */
    DBG_PRINTF("Sampling completed, result: %d\r\n", ret);
    float voltage = (float)ret / (float)4096 * 3.3;  /* 4096 and 3.3 are for Sample Value Conversion */
    DBG_PRINTF("voltage: %f\r\n", voltage);
    return;
}