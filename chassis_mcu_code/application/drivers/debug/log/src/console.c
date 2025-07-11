/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      console.c
  * @author    MCU Driver Team
  * @brief     console module driver
  * @details   The header file contains the following declaration:
  *             + GPIO configuration enums.
  *             + GPIO register structures.
  *             + GPIO DCL Functions.
  *             + Parameters check functions.
  */
#include "console.h"
#include "errno.h"
#include "ext_log.h"
#include "dfx_log.h"
#define UART_READ_TIME_MS 1000

#define VA_START(v, l) __builtin_va_start(v, l)
#define VA_ARG(v, l) __builtin_va_arg(v, l)
#define VA_END(v) __builtin_va_end(v)

#define DECIMAL_BASE 10U /* Cardinality of decimal numbers */
#define HALF_ADJUST_BOUNDARY 5U /* The boundary for rounding the floating number */
#define MAX_DIV_TIMES 31U

typedef __builtin_va_list va_list;

/* defines the number of output numbers */
typedef enum {
    BINARY = 2U,
    OCTAL = 8U,
    DECIMAL = 10U,
    HEXADECIMAL = 16U,
} NumBase;
UART_Handle g_console_uart;

/**
 * @brief query the status of a serial port reading register
 * @param uartHandle: indicates the serial port information corresponding to the value assignment
 * @param isEmpty: pointer to the array that stores status information
 * @retval whether data is received
 */
static BASE_StatusType QueryUartRxStatus(UART_Handle *uartHandle, unsigned char *isEmpty)
{
    *isEmpty = uartHandle->baseAddress->UART_FR.BIT.rxfe; /* read register status address */
    return BASE_STATUS_OK;
}

/**
 * @brief Single Character Output
 * @param c: single character to be output
 * @retval None
 */
void ConsolePutc(const char c)
{
    unsigned int length = 1;
    unsigned char p;
    /* add newline characters for standby */
    p = (unsigned char)c;
    if (c == '\n') {
        p = '\r';
        HAL_UART_WriteBlocking(&g_console_uart, &p, length, UART_READ_TIME_MS);
        p = '\n';
    }
    HAL_UART_WriteBlocking(&g_console_uart, &p, length, UART_READ_TIME_MS);
}

/**
 * @brief output the entire string.
 * @param str: string to be output.
 * @retval None
 */
int ConsolePuts(const char *str)
{
    int cnt = 0;
    /* decompose a string into a single character output */
    while (*str != '\0') {
        ConsolePutc(*str);
        str++;
        cnt++;
    }
    return cnt;
}

/**
 * @brief Read a single character
 * @param None
 * @retval ASCII value of the read character
 */
int ConsoleGetc(void)
{
    unsigned char rxStr;
    unsigned int length = 1;
    int ret;

    /* reads a single character from the serial port */
    ret = HAL_UART_ReadBlocking(&g_console_uart, &rxStr, length, UART_READ_TIME_MS);
    if (ret == EXT_SUCCESS) {
        return (int)rxStr;
    } else {
        return -1;
    }
}

/**
 * @brief reads the register reception status
 * @param None
 * @retval register Status
 */
int ConsoleGetQuery(void)
{
    unsigned char isEmpty;

    QueryUartRxStatus(&g_console_uart, &isEmpty);
    return !(isEmpty);
}

/**
 * @brief reads the pointer coordinates of the register.
 * @param base: pointer initial address value.
 * @param exponent: number of times the pointer needs to be moved
 * @retval pointer coordinate value
 */
static unsigned long DBG_Pow(unsigned int base, unsigned int exponent)
{
    unsigned long ret = 1;
    while (exponent--) {
        ret *= base;
    }
    return ret; /* ret = base ^ exponent */
}

/**
 * @brief calculate the number of digits entered
 * @param num: numbers to be calculated
 * @param base: number of digits entered
 * @retval number of digits of the calculated number
 */
static unsigned int DBG_CountDigits(int num, NumBase base)
{
    unsigned int cnt = 0;
    if (base == 0) {
        return 0;
    }
    /* Cyclic Conversion Count */
    while (num != 0) {
        cnt++;
        if (cnt > MAX_DIV_TIMES) {
            break;
        }
        num /= base;
    }
    /* Returns the number of digits */
    return cnt;
}

/**
 * @brief Output unsigned digits
 * @param num: numbers to be output
 * @param base: number of digits entered
 * @param digits: number of digits output
 * @retval None
 */
static void DBG_PutUnsignedNum(unsigned int num, NumBase base, unsigned int digits)
{
    unsigned char ch;
    while (digits != 0) {
        ch = num / DBG_Pow(base, digits - 1);
        num %= DBG_Pow(base, digits - 1);
        if (base == DECIMAL) {
            ConsolePutc(ch + '0'); /* characters that convert numbers to decimal numbers */
        } else if (base == HEXADECIMAL) {
            if (ch < DECIMAL_BASE) {
                ConsolePutc(ch + '0'); /* Character that converts a number to a hexadecimal number */
            } else {
                ConsolePutc(ch - DECIMAL_BASE + 'A');
            }
        }
        digits--;
    }
}

/**
 * @brief print Numbers
 * @param intNum: numbers to be output
 * @retval returns the number of digits of the output number
 */
static unsigned int DBG_PrintInt(int intNum)
{
    unsigned int cnt;
    if (intNum == 0) {
        ConsolePutc('0'); /* add '0' */
        return 1;
    }
    if (intNum < 0) {
        ConsolePutc('-'); /* need to manually add a negative sign */
        intNum = -intNum;
    }
    /* Calculate the number of digits */
    cnt = DBG_CountDigits(intNum, DECIMAL);
    DBG_PutUnsignedNum(intNum, DECIMAL, cnt);
    return cnt;
}

/**
 * @brief print hexadecimal digits
 * @param hexNum: numbers to be output
 * @retval returns the number of digits of the output number
 */
static unsigned int DBG_PrintHex(unsigned int hexNum)
{
    unsigned int cnt;
    if (hexNum == 0) {
        ConsolePutc('0'); /* add '0' */
        return 1;
    }
    /* Calculate the number of hexadecimal digits */
    cnt = DBG_CountDigits(hexNum, HEXADECIMAL);
    DBG_PutUnsignedNum(hexNum, HEXADECIMAL, cnt);
    return cnt;
}

/**
 * @brief Print Single Precision Decimals
 * @param fltNum: numbers to be output
 * @param precision: number of decimal places to print
 * @retval returns the number of digits of the output number
 */
static unsigned int DBG_PrintFlt(float fltNum, unsigned int precision)
{
    unsigned int cnt = 0;
    unsigned int floatScale;

    if (fltNum < 0) {
        ConsolePutc('-');
        cnt += 1;
        fltNum = -fltNum;
    }
    int integerVal = (int)fltNum;
    floatScale = DBG_Pow(10, (precision + 1)); /* 10: decimal */
    int floatVal = (long)(floatScale * (fltNum - integerVal));
    /* Half-adjust: round up or round down */
    if (floatVal % DECIMAL_BASE >= HALF_ADJUST_BOUNDARY) {
        floatVal = floatVal / DECIMAL_BASE + 1;
    } else {
        floatVal = floatVal / DECIMAL_BASE;
    }
    cnt += DBG_PrintInt(integerVal);
    ConsolePutc('.');
    cnt += 1;
    /* Pad 0 in float part */
    unsigned int fltCnt = DBG_CountDigits(floatVal, DECIMAL);
    if (precision > fltCnt) {
        for (unsigned int i = 0; i < precision - fltCnt; i++) {
            ConsolePutc('0'); /* add '0' */
        }
    }
    DBG_PutUnsignedNum(floatVal, DECIMAL, fltCnt); /* print unsigned number */
    cnt += precision;
    return cnt;
}

/**
 * @brief Resolving Special Characters
 * @param ch: single character to be parsed
 * @param *paramList: elements that implement parsing
 * @retval returns the number of digits of the output number
 */
static unsigned int ParseSpecifier(const char ch, va_list *paramList)
{
    /* Value Definition Initialization */
    unsigned int cnt = 0;
    unsigned int tmpCnt;
    char chVal = 0;
    const char *strVal = 0;
    int intVal = 0;
    unsigned int unsignedVal = 0;
    unsigned int hexVal = 0;
    float fltVal = 0;
    switch (ch) {
        case 'c':
            chVal = VA_ARG(*paramList, int); /* Use type int because of byte alignment */
            ConsolePutc(chVal);
            cnt += 1;
            break;
        case 's':
            /* received 's', print the string */
            strVal = VA_ARG(*paramList, const char *);
            cnt += ConsolePuts(strVal);
            break;
        case 'd':
            /* Received character'd', print initialization */
            intVal = VA_ARG(*paramList, int);
            cnt += DBG_PrintInt(intVal);
            break;
        case 'u':
            unsignedVal = VA_ARG(*paramList, unsigned int);
            tmpCnt = DBG_CountDigits(unsignedVal, DECIMAL);
            DBG_PutUnsignedNum(unsignedVal, DECIMAL, tmpCnt);
            cnt += tmpCnt;
            break;
        case 'x':
        case 'X':
        case 'p':
            /* Received'p' and returned hexadecimal number */
            hexVal = VA_ARG(*paramList, unsigned int);
            cnt += DBG_PrintHex(hexVal);
            break;
        case 'f':
            fltVal = VA_ARG(*paramList, double);
            cnt += DBG_PrintFlt(fltVal, 5); /* default precision: 5 */
            break;
        default:
            ConsolePutc(ch); /* Output the original input characters */
            cnt += 1;
            break;
    }
    /* returns the count value */
    return cnt;
}

/**
 * @brief Printed number with width
 * @param intNum: Numbers to be printed
 * @param *paramList: Number of digits to be printed
 * @retval returns the number of digits of the output number
 */
static unsigned int DBG_PrintIntWithField(int intNum, int fieldWidth)
{
    int zeroCnt = 0;
    int digitsCnt = 0;
    unsigned int cnt = 0;

    if (intNum == 0) {
        ConsolePutc('0');
        return 1;
    }
    if (intNum < 0) {
        ConsolePutc('-'); /* add symbol */
        cnt++;
        intNum = -intNum;
        digitsCnt = DBG_CountDigits(intNum, DECIMAL); /* get int value's width */
        zeroCnt = fieldWidth - digitsCnt;
        for (int i = 0; i < zeroCnt; i++) {
            ConsolePutc('0'); /* add '0' */
            cnt++;
        }
        cnt += digitsCnt;
    } else {
        digitsCnt = DBG_CountDigits(intNum, DECIMAL); /* get int value's width */
        cnt = digitsCnt;
        zeroCnt = fieldWidth - digitsCnt;
        for (int i = 0; i < zeroCnt; i++) {
            ConsolePutc('0'); /* add '0' */
            cnt++;
        }
    }
    DBG_PutUnsignedNum(intNum, DECIMAL, digitsCnt);
    return cnt;
}

/**
 * @brief Convert a numeric string to a number
 * @param **s: Number string to be converted
 * @retval Number after conversion
 */
static int DBG_Atoi(const char **s)
{
    int i, c;

    for (i = 0; '0' <= (c = **s) && c <= '9'; ++*s) {
        i = i * 10 + c - '0'; /* 10: decimal */
    }
    return i;
}

/**
 * @brief Print the entry parameters
 * @param *format: thing need to print
 * @retval returns the number of digits of the output number
 */
int UartPrintf(const char *format, ...)
{
    /* Define Value Initialization */
    int cnt = 0;
    int fieldWidth = 0;
    int floatPrecision = 0;
    float fltVal = 0;
    int intVal = 0;
    va_list paramList;
    VA_START(paramList, format);

    while (*format != '\0') {
        if (*format != '%') {
            /* received '%', print characters directly */
            ConsolePutc(*format);
            cnt += 1;
        } else {
            format++;
	    /* Check whether the value is an integer */
            if (*format == '0') {
                format++;
                fieldWidth = DBG_Atoi(&format);
                intVal = VA_ARG(paramList, int);
                cnt += DBG_PrintIntWithField(intVal, fieldWidth);
            } else if (*format == '.') {
                format++;
                floatPrecision = DBG_Atoi(&format); /* Convert to Integer */
                fltVal = VA_ARG(paramList, double);
                cnt += DBG_PrintFlt(fltVal, floatPrecision);
            } else {
                cnt += ParseSpecifier(*format, &paramList);
            }
        }
        format++;
    }
    VA_END(paramList);
    /* Returns the value of count */
    return cnt;
}

/* init console uart */
void ConsoleInit(UART_Handle uart)
{
    g_console_uart = uart;
    DfxCmdRegister();
}