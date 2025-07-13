#include "debug_uart.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include "uart.h"
#include "pwm.h"
#include "grab.h"

/* 接收完成标志 */
static volatile bool RxInterruptflag = true;
/* 发送完成标志 */
static volatile bool TxInterruptFlag = true;
unsigned char get_data;
bool Servo_flag = true;
bool flag = false;
short num = 0;

void UART2WriteInterruptCallback(void* handle){
    BASE_FUNC_UNUSED(handle);
    TxInterruptFlag = true;
}

void UART2ReadInterruptCallback(void* handle){
    BASE_FUNC_UNUSED(handle);
    RxInterruptflag = true;
    flag = true;
}



void test_uart(void){
    while(1){
        if(RxInterruptflag){
            RxInterruptflag = false;
            HAL_UART_ReadIT(&g_uart2, &get_data, 1);
            DBG_PRINTF("%c\r\n", get_data);
            HAL_GPIO_TogglePin(&g_gpio2, g_gpio2.pins);
            // num = atoi(get_data);
            // DBG_PRINTF("Get Data: %d\r\n", num);
            // clockwise(&g_gpt2, num);
            if(get_data == 'a' && Servo_flag && flag){
                Servo_flag = false;
                DBG_PRINTF("LIFTing\r\n");
                // open(100, 600); // 500 is close
                // clockwise(&g_gpt2, 50);
                lift();
                Servo_flag = true;
                flag = false;
            }
            else if(get_data == 'd' && Servo_flag && flag){
                Servo_flag = false;
                DBG_PRINTF("LAYing\r\n");
                _close(100, 200);   //200 is open
                // counterwise(&g_gpt2, 50);
                lay();
                Servo_flag = true;
                flag = false;
            }
        }
    }
}