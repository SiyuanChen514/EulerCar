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
 * @file    uttest.h
 * @author  MCU Driver Team
 * @brief   Macros for UT test
 * @details This file provides firmware functions to manage the following
 *          functionalities of the UT test.
 *          + Creat UT test suite
 *          + Define test modes
 *          + Output test summary.
 */

/* Macro definitions */
#ifndef McuMagicTag_UT_TEST_H
#define McuMagicTag_UT_TEST_H

#include "debug.h"
#ifndef NULL
#define NULL (void *)0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define PRINTF DBG_PRINTF
/* *************************************************************************** */
/* System  parameter configuration                                             */
/* *************************************************************************** */
#define UT_TEST_VERSION "v1.0"
#define UT_TEST_WRITE_STRING(msg) PRINTF("%s", msg)
#define UT_TEST_WRITE_INT(n) PRINTF("%d", n)

#define UT_TEST_MODE_SUMARY 0              /* Show only the final result */
#define UT_TEST_MODE_GENERAL 1             /* Only checks that fail are displays */
#define UT_TEST_MODE_DETAILED 2            /* Passed and failed checks are displayed */
#define UT_TEST_MODE UT_TEST_MODE_DETAILED /* log level */

/* Action to take if check fails */
#define UT_TEST_ACTION_WARNING 0   /* Goes through the checks with message depending on level */
#define UT_TEST_ACTION_SHUTDOWN 1  /* Stops on the end of the checklist if any check has failed */
#define UT_TEST_ACTION_SAFESTATE 2 /* Goes in safe state if check fails */

/* *************************************************************************** */
/* Variables */
/* *************************************************************************** */
static int g_utTestChecksFailed = 0;          /* Number of failed checks */
static int g_utTestChecksPassed = 0;          /* Number of passed checks */
static int g_utTestTestcasesFailed = 0;       /* Number of failed test cases */
static int g_utTestTestcasesPassed = 0;       /* Number of passed test cases */
static int g_utTestTestcasesFailedChecks = 0; /* Number of failed checks in a testcase */
static int g_utTestChecklistFailedChecks = 0; /* Number of failed checks in a checklist */

/* *************************************************************************** */
/* Internal (private) Macros                                                   */
/* *************************************************************************** */

#define UT_TEST_DEFINE_TO_STRING_HELPER(x) #x
/* Converts a define constant into a string. */
#define UT_TEST_DEFINE_TO_STRING(x) UT_TEST_DEFINE_TO_STRING_HELPER(x)

#if (UT_TEST_MODE == UT_TEST_MODE_DETAILED)
/**
 * @Macro:       UT_TEST_WRITE_PASSED_MSG(msg, args)
 *
 * @Description: Writes a message that check has passed.
 *
 * @Param msg:   Message to write. This is the name of the called
 * Check, without the substring UT_TEST_CHECK.
 * @Param args:  Argument list as string.
 *
 * @Remarks:     This macro is used by UT_TEST_CHECK(). A message will
 * only be written if verbose mode is set to UT_TEST_MODE_DETAILED.
 *
 */
#define UT_TEST_WRITE_PASSED_MSG(msg, args) do {                  \
        UT_TEST_WRITE_STRING(__FILE__);                           \
        UT_TEST_WRITE_STRING(":");                                \
        UT_TEST_WRITE_STRING(UT_TEST_DEFINE_TO_STRING(__LINE__)); \
        UT_TEST_WRITE_STRING(": passed:");                        \
        UT_TEST_WRITE_STRING(msg);                                \
        UT_TEST_WRITE_STRING("(");                                \
        UT_TEST_WRITE_STRING(args);                               \
        UT_TEST_WRITE_STRING(")\r\n");                            \
    } while (0)
#else
#define UT_TEST_WRITE_PASSED_MSG(msg, args)
#endif

#if (UT_TEST_MODE == UT_TEST_MODE_SUMARY)
#define UT_TEST_WRITE_FAILED_MSG(msg, args)
#else
/**
 * @Macro:       UT_TEST_WRITE_FAILED_MSG(msg, args)
 *
 * @Description: Writes a message that check has failed.
 *
 * @Param msg:   Message to write. This is the name of the called
 * Check, without the substring UT_TEST_CHECK.
 * @Param args:  Argument list as string.
 *
 * @Remarks:     This macro is used by UT_TEST_CHECK(). A message will
 * only be written if verbose mode is set
 * to UT_TEST_MODE_GENERAL and UT_TEST_MODE_DETAILED.
 *
 */
#define UT_TEST_WRITE_FAILED_MSG(msg, args) do {                  \
        UT_TEST_WRITE_STRING(__FILE__);                           \
        UT_TEST_WRITE_STRING(":");                                \
        UT_TEST_WRITE_STRING(UT_TEST_DEFINE_TO_STRING(__LINE__)); \
        UT_TEST_WRITE_STRING(": failed:");                        \
        UT_TEST_WRITE_STRING(msg);                                \
        UT_TEST_WRITE_STRING("(");                                \
        UT_TEST_WRITE_STRING(args);                               \
        UT_TEST_WRITE_STRING(")\r\n");                            \
    } while (0)
#endif

/**
 * @Macro:       UT_TEST_FAIL_CHECK(msg, args)
 *
 * @Description: Fails a check.
 *
 * @Param msg:   Message to write. This is the name of the called
 * Check, without the substring UT_TEST_CHECK.
 * @Param args:  Argument list as string.
 *
 * @Remarks:     This macro is used by UT_TEST_CHECK(). A message will
 * only be written if verbose mode is set
 * to UT_TEST_MODE_GENERAL and UT_TEST_MODE_DETAILED.
 *
 */
#define UT_TEST_FAIL_CHECK(msg, args)        \
    do {                                     \
        UT_TEST_WRITE_FAILED_MSG(msg, args); \
        g_utTestChecksFailed++;              \
        g_utTestChecklistFailedChecks++;     \
    } while (0)

/**
 * @Macro:       UT_TEST_PASS_CHECK(msg, args)
 *
 * @Description: Passes a check.
 *
 * @Param msg:   Message to write. This is the name of the called
 * Check, without the substring UT_TEST_CHECK.
 * @Param args:  Argument list as string.
 *
 * @Remarks:     This macro is used by UT_TEST_CHECK(). A message will
 * only be written if verbose mode is set
 * to UT_TEST_MODE_DETAILED.
 *
 */
#define UT_TEST_PASS_CHECK(message, args)        \
    do {                                         \
        UT_TEST_WRITE_PASSED_MSG(message, args); \
        g_utTestChecksPassed++;                  \
    } while (0)

/* *************************************************************************** */
/* Checklist Macros                                                            */
/* *************************************************************************** */

/**
 * @Macro:       UT_TEST_CHECKLIST_BEGIN(action)
 *
 * @Description: Begin of a checklist. You have to tell what action
 *               shall be taken if a check fails.
 *
 * @Param action: Action to take. This can be:
 *                * UT_TEST_ACTION_WARNING:   A warning message will be printed
 *                                           that a check has failed
 *                * UT_TEST_ACTION_SHUTDOWN:  The system will shutdown at
 *                                           the end of the checklist.
 *                * UT_TEST_ACTION_SAFESTATE: The system goes into the safe state
 *                                           on the first failed check.

 * @Remarks:     A checklist must be finished with UT_TEST_CHECKLIST_END()
 *
 */
#define UT_TEST_CHECKLIST_BEGIN(action)    \
    do {                                   \
        UT_TEST_action = action;           \
        g_utTestChecklistFailedChecks = 0; \
    } while (0)

/**
 * @Macro:       UT_TEST_CHECKLIST_END()
 *
 * @Description: End of a checklist. If the action was UT_TEST_ACTION_SHUTDOWN
 * the system will shutdown.
 *
 * @Remarks:     A checklist must begin with UT_TEST_CHECKLIST_BEGIN(action)
 *
 */
#define UT_TEST_CHECKLIST_END()                          \
    if (g_utTestChecklistFailedChecks != 0) {            \
        UT_TEST_WRITE_FAILED_MSG("Checklist", "");       \
        if (UT_TEST_ACTION_SHUTDOWN == UT_TEST_action) { \
            UT_TEST_Shutdown();                          \
        }                                                \
    } else {                                             \
        UT_TEST_WRITE_PASSED_MSG("Checklist", "");       \
    }

/* *************************************************************************** */
/* Check Macros                                                                */
/* *************************************************************************** */

/**
 * @Macro:       UT_TEST_CHECK(condition, msg, args)
 *
 * @Description: Checks a condition and prints a message.
 *
 * @Param condition: Check the establishment conditions.
 * @Param msg:   Message to write.
 * @Param args:  Argument list as string
 *
 * @Remarks:     Basic check. This macro is used by all higher level checks.
 *
 */
#define UT_TEST_CHECK(condition, msg, args) \
    do {                                    \
        if ((condition)) {                  \
            UT_TEST_PASS_CHECK(msg, args);  \
        } else {                            \
            UT_TEST_FAIL_CHECK(msg, args);  \
        }                                   \
    } while (0)

/**
 * @Macro:       UT_TEST_CHECK_IS_EQUAL(expected,actual)
 *
 * @Description: Checks that actual value equals the expected value.
 *
 * @Param expected: Expected value.
 * @Param actual: Actual value.
 *
 * @Remarks:     This macro uses UT_TEST_CHECK(condition, msg, args).
 *
 */
#define UT_TEST_CHECK_IS_EQUAL(expected, actual) UT_TEST_CHECK((expected) == (actual), "IsEqual", #expected "," #actual)

/**
 * @Macro:       UT_TEST_CHECK_IS_NULL(pointer)
 *
 * @Description: Checks that a pointer is NULL.
 *
 * @Param pointer: Pointer to check.
 *
 * @Remarks:     This macro uses UT_TEST_CHECK(condition, msg, args).
 *
 */
#define UT_TEST_CHECK_IS_NULL(pointer) UT_TEST_CHECK((pointer) == NULL, "IsNull", #pointer)

/**
 * @Macro:       UT_TEST_CHECK_IS_NOT_NULL(pointer)
 *
 * @Description: Checks that a pointer is not NULL.
 *
 * @Param pointer: Pointer to check.
 *
 * @Remarks:     This macro uses UT_TEST_CHECK(condition, msg, args).
 *
 */
#define UT_TEST_CHECK_IS_NOT_NULL(pointer) UT_TEST_CHECK((pointer) != NULL, "IsNotNull", #pointer)

/**
 * @Macro:       UT_TEST_CHECK_IS_IN_RANGE(value, lower, upper)
 *
 * @Description: Checks if a value is between lower and upper bounds (inclusive)
 * Mathematical: lower <= value <= upper
 *
 * @Param value: Value to check.
 * @Param lower: Lower bound.
 * @Param upper: Upper bound.
 *
 * @Remarks:     This macro uses UT_TEST_CHECK(condition, msg, args).
 *
 */
#define UT_TEST_CHECK_IS_IN_RANGE(value, lower, upper) \
    UT_TEST_CHECK((((value) >= (lower)) && ((value) <= (upper))), "IsInRange", #value "," #lower "," #upper)

/**
 * @Macro:       UUT_TEST_CHECK_IS_8BIT(value)
 *
 * @Description: Checks if a value fits into 8-bit.
 *
 * @Param value: Value to check.
 *
 * @Remarks:     This macro uses UT_TEST_CHECK(condition, msg, args).
 *
 */
#define UUT_TEST_CHECK_IS_8BIT(value) UT_TEST_CHECK((value) == ((value)&0xFF), "Is8Bit", #value)

/**
 * @Macro:       UUT_TEST_CHECK_IS_16BIT(value)
 *
 * @Description: Checks if a value fits into 16-bit.
 *
 * @Param value: Value to check.
 *
 * @Remarks:     This macro uses UT_TEST_CHECK(condition, msg, args).
 *
 */
#define UUT_TEST_CHECK_IS_16BIT(value) UT_TEST_CHECK((value) == ((value)&0xFFFF), "Is16Bit", #value)

/**
 * @Macro:       UUT_TEST_CHECK_IS_32BIT(value)
 *
 * @Description: Checks if a value fits into 32-bit.
 *
 * @Param value: Value to check.
 *
 * @Remarks:     This macro uses UT_TEST_CHECK(condition, msg, args).
 *
 */
#define UUT_TEST_CHECK_IS_32BIT(value) UT_TEST_CHECK((value) == ((value)&0xFFFFFFFF), "Is32Bit", #value)

/**
 * Checks if bit is set
 */
/**
 * @Macro:       UT_TEST_CHECK_IS_BIT_SET(value, bitno)
 *
 * @Description: Checks if a bit is set in value.
 *
 * @Param value: Value to check.
 * @Param bitno: Bit number. The least significant bit is 0.
 *
 * @Remarks:     This macro uses UT_TEST_CHECK(condition, msg, args).
 *
 */
#define UT_TEST_CHECK_IS_BIT_SET(value, bitno) \
    UT_TEST_CHECK((1 == (((value) >> (bitno)) & 0x01)), "IsBitSet", #value "," #bitno)

/**
 * @Macro:       UT_TEST_CHECK_IS_BIT_CLEAR(value, bitno)
 *
 * @Description: Checks if a bit is not set in value.
 *
 * @Param value: Value to check.
 * @Param bitno: Bit number. The least significant bit is 0.
 *
 * @Remarks:     This macro uses UT_TEST_CHECK(condition, msg, args).
 *
 */
#define UT_TEST_CHECK_IS_BIT_CLEAR(value, bitno) \
    UT_TEST_CHECK((0 == (((value) >> (bitno)) & 0x01)), "IsBitClear", #value "," #bitno)
/* *************************************************************************** */
/* Testcases */
/* *************************************************************************** */
/**
 * @Macro:       UT_TEST_TESTCASE_BEGIN(name)
 *
 * @Description: Marks the beginning of a test case and resets
 * the test case statistic.
 *
 * @Param name:  Name of the test case.
 *
 * @Remarks:     This macro uses UT_TEST_WRITE_STRING(msg) to print the name.
 *
 */
#define UT_TEST_TESTCASE_BEGIN(name)                                            \
    do {                                                                        \
        UT_TEST_WRITE_STRING("\r\n======================================\r\n"); \
        UT_TEST_WRITE_STRING(name);                                             \
        UT_TEST_WRITE_STRING("\r\n======================================\r\n"); \
        g_utTestTestcasesFailedChecks = g_utTestChecksFailed;                   \
    } while (0)

/**
 * @Macro:       UT_TEST_TESTCASE_END()
 *
 * @Description: Marks the end of a test case and calculates
 * the test case statistics.
 *
 * @Remarks:     This macro uses UT_TEST_WRITE_STRING(msg) to print the result.
 *
 */
#define UT_TEST_TESTCASE_DETATILS() do {                               \
    if ((g_utTestTestcasesFailedChecks - g_utTestChecksFailed) == 0) { \
        UT_TEST_WRITE_STRING("Testcase passed.\n\r");                  \
        g_utTestTestcasesPassed++;                                     \
    } else {                                                           \
        UT_TEST_WRITE_FAILED_MSG("EndTestcase", "");                   \
        g_utTestTestcasesFailed++;                                     \
    }                                                                  \
} while (0)

#define UT_TEST_TESTCASE_END() do {                                     \
    UT_TEST_WRITE_STRING("======================================\r\n"); \
    UT_TEST_TESTCASE_DETATILS();                                        \
    UT_TEST_WRITE_STRING("======================================\r\n"); \
} while (0)

/**
 * @Macro:       UT_TEST_WriteSummary()
 *
 * @Description: Writes the test suite summary.
 *
 * @Remarks:     This macro uses UT_TEST_WRITE_STRING(msg) and
 * UT_TEST_WRITE_INT(n) to write the summary.
 *
 */
#define UT_TEST_WRITE_SUMMARY_DETATILS() do {        \
    UT_TEST_WRITE_STRING("\r\nTestcases: failed: "); \
    UT_TEST_WRITE_INT(g_utTestTestcasesFailed);      \
    UT_TEST_WRITE_STRING("\r\n           passed: "); \
    UT_TEST_WRITE_INT(g_utTestTestcasesPassed);      \
    UT_TEST_WRITE_STRING("\r\nChecks:    failed: "); \
    UT_TEST_WRITE_INT(g_utTestChecksFailed);         \
    UT_TEST_WRITE_STRING("\r\n           passed: "); \
    UT_TEST_WRITE_INT(g_utTestChecksPassed);         \
} while (0)

#define UT_TEST_WRITE_SUMMARY() do {                                        \
    UT_TEST_WRITE_STRING("\r\n**************************************");     \
    UT_TEST_WRITE_SUMMARY_DETATILS();                                       \
    UT_TEST_WRITE_STRING("\r\n**************************************\r\n"); \
} while (0)

#endif /* McuMagicTag_UT_TEST_H */
