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
  * @file    debug.c
  * @author  MCU Driver Team
  * @brief   DEBUG module driver.
  *          This file provides functions to manage the following functionalities of the DEBUG module.
  *          + Initialization and de-initialization functions
  *          + Format string print function
  */

#include "debug.h"

#if (DBG_PRINTF_USE == DBG_USE_UART_PRINTF)
/* Macro definitions of stdarg.h to prevent using standard library */
#define VA_START(v, l)  __builtin_va_start(v, l)
#define VA_ARG(v, l)    __builtin_va_arg(v, l)
#define VA_END(v)       __builtin_va_end(v)

#define DECIMAL_BASE            10U /* Cardinality of decimal numbers */
#define HALF_ADJUST_BOUNDARY    5U  /* The boundary for rounding the floating number */
#define MAX_DIV_TIMES           31U
/* FLOAT_SCALE = DECIMAL_BASE ^ (FLOAT_PRECISION + 1) */
#endif

typedef __builtin_va_list va_list;

#if (DBG_PRINTF_USE == DBG_USE_UART_PRINTF)
/**
  * @brief  Cardinality of binary, octal, decimal, and hexadecimal numbers.
  */
typedef enum {
    BINARY      = 2U,
    OCTAL       = 8U,
    DECIMAL     = 10U,
    HEXADECIMAL = 16U,
} NumBase;
#endif

#if (DBG_PRINTF_USE == DBG_USE_UART_PRINTF)
UART_Handle g_dbgUart;
/**
 * @brief   Initialize the UART port for DBG_UartPrintf().
 * @param   baudRate The baud rate of UART port.
 * @retval  BASE_StatusType BASE status type: OK, ERROR, BUSY, TIMEOUT.
 */
BASE_StatusType DBG_UartPrintInit(unsigned int baudRate)
{
    g_dbgUart.baseAddress   = DBG_PRINTF_UART_PORT;
    g_dbgUart.baudRate      = baudRate;
    g_dbgUart.dataLength    = UART_DATALENGTH_8BIT;
    g_dbgUart.stopBits      = UART_STOPBITS_ONE;
    g_dbgUart.parity        = UART_PARITY_NONE;
    g_dbgUart.txMode        = UART_MODE_BLOCKING;
    g_dbgUart.rxMode        = UART_MODE_BLOCKING;
    g_dbgUart.fifoMode      = true;
    g_dbgUart.fifoTxThr     = UART_FIFOFULL_ONE_EIGHT;
    g_dbgUart.fifoRxThr     = UART_FIFOFULL_ONE_EIGHT;
    g_dbgUart.hwFlowCtr     = UART_HW_FLOWCTR_DISABLE;
    return HAL_UART_Init(&g_dbgUart);
}

/**
 * @brief   De-initialize the UART port for DBG_UartPrintf().
 * @retval  BASE_StatusType BASE status type: OK, ERROR, BUSY, TIMEOUT.
 */
BASE_StatusType DBG_UartPrintDeInit(void)
{
    return HAL_UART_DeInit(&g_dbgUart);
}
#endif

/* Format string print function */
#if (DBG_PRINTF_USE == DBG_USE_UART_PRINTF)
/**
 * @brief   Write a character to the UART port.
 * @param   ch The int promotion of the character to be written.
 * @retval  None.
 */
static void DBG_PrintCh(unsigned int ch)
{
    while (DBG_PRINTF_UART_PORT->UART_FR.BIT.txff == 1) {
        ;
    }
    DBG_PRINTF_UART_PORT->UART_DR.BIT.data = (unsigned char)ch;
}

/**
 * @brief   Print a string through the UART port.
 * @param   str The string to be printed.
 * @retval  int If succeeded, the total number of characters printed is returned.
 *              If the input parameter is wrong, a BASE_STATUS_ERROR is returned.
 */
static int DBG_PrintStr(const char *str)
{
    DEBUG_ASSERT_PARAM(str != NULL);
    int cnt = 0;
    while (*str != '\0') {
        DBG_PrintCh(*str);
        str++;
        cnt++;
    }
    return cnt;
}

/**
 * @brief   Raise base value to the power exponent value.
 * @param   base Base value.
 * @param   exponent Exponent value.
 * @retval  unsigned long The result of raising base to the power exponent.
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
 * @brief   Count the digits of the number.
 * @param   num The number to be counted.
 * @param   base The number base of num.
 * @retval  unsigned int The number of digits.
 */
static unsigned int DBG_CountDigits(int num, NumBase base)
{
    unsigned int cnt = 0;
    if (base == 0) {
        return 0;
    }
    while (num != 0) {
        cnt++;
        if (cnt > MAX_DIV_TIMES) {
            break;
        }
        num /= base;
    }
    cnt = (cnt == 0) ? 1 : cnt;
    return cnt;
}

/**
 * @brief   Print unsigned number through UART port.
 * @param   num The unsigned number to be printed.
 * @param   base The number base of num.
 * @param   digits The digits of num.
 */
static void DBG_PutUnsignedNum(unsigned int num, NumBase base, unsigned int digits)
{
    unsigned char ch;
    while (digits != 0) {
        ch = num / DBG_Pow(base, digits - 1);
        num %= DBG_Pow(base, digits - 1);
        if (base == DECIMAL) {
            DBG_PrintCh(ch + '0');
        } else if (base == HEXADECIMAL) {
            if (ch < DECIMAL_BASE) {
                DBG_PrintCh(ch + '0');
            } else {
                DBG_PrintCh(ch - DECIMAL_BASE + 'A');
            }
        } else {
            break;
        }
        digits--;
    }
}

/**
 * @brief   Print decimal number through UART port.
 * @param   intNum The decimal number to be printed.
 * @retval  unsigned int The total number of characters printed.
 */
static unsigned int DBG_PrintInt(int intNum)
{
    unsigned int cnt;
    if (intNum == 0) {
        DBG_PrintCh('0');
        return 1;
    }
    if (intNum < 0) {
        DBG_PrintCh('-');
        intNum = -intNum;
    }
    cnt = DBG_CountDigits(intNum, DECIMAL);
    DBG_PutUnsignedNum(intNum, DECIMAL, cnt);
    return cnt;
}

/**
 * @brief   Print hexadecimal number through UART port.
 * @param   hexNum The hexadecimal number to be printed.
 * @retval  unsigned int The total number of characters printed.
 */
static unsigned int DBG_PrintHex(unsigned int hexNum)
{
    unsigned int cnt;
    if (hexNum == 0) {
        DBG_PrintCh('0');
        return 1;
    }
    cnt = DBG_CountDigits(hexNum, HEXADECIMAL);
    DBG_PutUnsignedNum(hexNum, HEXADECIMAL, cnt);
    return cnt;
}

/**
 * @brief   Print floating-point number through UART port.
 * @param   fltNum The floating-point number to be printed.
 * @retval  unsigned int The total number of characters printed.
 */
static unsigned int DBG_PrintFlt(float fltNum, unsigned int precision)
{
    unsigned int cnt = 0;
    unsigned int floatScale;

    if (fltNum < 0) {
        DBG_PrintCh('-');
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
    DBG_PrintCh('.');
    cnt += 1;
    /* Pad 0 in float part */
    unsigned int fltCnt = DBG_CountDigits(floatVal, DECIMAL);
    if (precision > fltCnt) {
        for (unsigned int i = 0; i < precision - fltCnt; i++) {
            DBG_PrintCh('0'); /* add '0' */
        }
    }
    DBG_PutUnsignedNum(floatVal, DECIMAL, fltCnt); /* print unsigned number */
    cnt += precision;
    return cnt;
}

/**
 * @brief   Parse the format specifier and print the parameter by format.
 * @param   ch The format specifier.
 * @param   paramList The pointer of the variable parameter list.
 * @retval  unsigned int The total number of characters printed.
 */
static unsigned int ParseSpecifier(const char ch, va_list *paramList)
{
    unsigned int cnt = 0;
    unsigned int tmpCnt;
    char chVal = 0;
    const char *strVal = NULL;
    int intVal = 0;
    unsigned int unsignedVal = 0;
    unsigned int hexVal = 0;
    float fltVal = 0;
    switch (ch) {
        case 'c': /* Character format data. */
            chVal = VA_ARG(*paramList, int); /* Use type int because of byte alignment */
            DBG_PrintCh(chVal);
            cnt += 1;
            break;
        case 's': /* String format data. */
            strVal = VA_ARG(*paramList, const char *);
            cnt += DBG_PrintStr(strVal);
            break;
        case 'd': /* Integer decimal data. */
            intVal = VA_ARG(*paramList, int);
            cnt += DBG_PrintInt(intVal);
            break;
        case 'u': /* Unsigned decimal data. */
            unsignedVal = VA_ARG(*paramList, unsigned int);
            tmpCnt = DBG_CountDigits(unsignedVal, DECIMAL);
            DBG_PutUnsignedNum(unsignedVal, DECIMAL, tmpCnt);
            cnt += tmpCnt;
            break;
        case 'x': /* Hexadecimal data. */
        case 'X':
        case 'p': /* Address data. */
            hexVal = VA_ARG(*paramList, unsigned int);
            cnt += DBG_PrintHex(hexVal);
            break;
        case 'f': /* Floating-point data. */
            fltVal = VA_ARG(*paramList, double);
            cnt += DBG_PrintFlt(fltVal, 5); /* default precision: 5 */
            break;
        default:
            DBG_PrintCh(ch);
            cnt += 1;
            break;
    }
    return cnt;
}

/**
 * @brief   Print decimal number with field width.
 * @param   intNum The decimal number to be printed.
 * @param   fieldWidth Field width.
 * @retval  unsigned int The total number of characters printed.
 */
static unsigned int DBG_PrintIntWithField(int intNum, int fieldWidth)
{
    int zeroCnt = 0;
    int digitsCnt = 0;
    unsigned int cnt = 0;

    if (intNum == 0) {
        DBG_PrintCh('0');
        return 1;
    }
    if (intNum < 0) {
        DBG_PrintCh('-'); /* add symbol */
        cnt++;
        intNum = -intNum;
        digitsCnt = DBG_CountDigits(intNum, DECIMAL); /* get int value's width */
        zeroCnt = fieldWidth - digitsCnt;
        for (int i = 0; i < zeroCnt; i++) {
            DBG_PrintCh('0'); /* add '0' */
            cnt++;
        }
        cnt += digitsCnt;
    } else {
        digitsCnt = DBG_CountDigits(intNum, DECIMAL); /* get int value's width */
        cnt = digitsCnt;
        zeroCnt = fieldWidth - digitsCnt;
        for (int i = 0; i < zeroCnt; i++) {
            DBG_PrintCh('0'); /* add '0' */
            cnt++;
        }
    }
    DBG_PutUnsignedNum(intNum, DECIMAL, digitsCnt);
    return cnt;
}

static int DBG_Atoi(const char **s)
{
    int i, c;

    for (i = 0; '0' <= (c = **s) && c <= '9'; ++*s) {
        i = i * 10  + c - '0'; /* 10: decimal */
    }
    return i;
}

/**
 * @brief   Print format string through UART port, supporting %c, %s, %d, %u, %x, %X, %p, %f.
 *          %c      To print a character.
 *          %s      To print a string.
 *          %d      To print a decimal value.
 *          %u      To print an unsigned decimal value.
 *          %x, %X  To print a hexadecimal value using upper case letters.
 *          %p      To print a pointer as a hexadecimal value.
 *          %f      To print a floating-point number with a fixed precision determined by FLOAT_PRECISION.
 * @param   format  A string that contains the text to be printed and the format specifiers.
 * @param   ...     Variable parameter list.
 * @retval  int     If succeeded, the total number of characters printed is returned.
 *                  If the input parameter is wrong, return BASE_STATUS_ERROR.
 */
int DBG_UartPrintf(const char *format, ...)
{
    DEBUG_ASSERT_PARAM(format != NULL);
    int cnt = 0;
    int fieldWidth = 0;
    int floatPrecision = 0;
    float fltVal = 0;
    int intVal = 0;
    va_list paramList;
    VA_START(paramList, format);

    while (*format != '\0') {
        if (*format != '%') {
            DBG_PrintCh(*format);
            cnt += 1;
        } else {
            format++;
            if (*format == '0') {
                format++;
                fieldWidth = DBG_Atoi(&format);
                intVal = VA_ARG(paramList, int);
                cnt += DBG_PrintIntWithField(intVal, fieldWidth);
            } else if (*format == '.') {
                format++;
                floatPrecision = DBG_Atoi(&format);
                fltVal = VA_ARG(paramList, double);
                cnt += DBG_PrintFlt(fltVal, floatPrecision);
            } else {
                cnt += ParseSpecifier(*format, &paramList);
            }
        }
        format++;
    }
    VA_END(paramList);
    return cnt;
}
#endif