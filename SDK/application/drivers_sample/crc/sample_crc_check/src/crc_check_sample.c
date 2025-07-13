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
  * @file      crc_check_sample.c
  * @author    MCU Driver Team
  * @brief     Calculate and verify the CRC value of the data.
  * @details   A group of char data is given, and the corresponding algorithm mode and input data format are \
  *            cyclically configured, so that each algorithm and data format are traversed, a CRC value is \
  *            generated in each case, and then the CRC value and the original data are checked together to \
  *            determine whether each algorithm is normal.
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "crc.h"
#include "crc_check_sample.h"
#include "main.h"

#define SUPPORT_CRC_V1                  false

#if (SUPPORT_CRC_V1 == true)
#define CRC_TEST_INPUT_DATA             0x5AA5DAAD /* big endian */
#else
#define CRC_TEST_INPUT_DATA             0xADDAA55A /* little endian */
#endif
#define CRC_TEST_DATA_LENTH             4

#define CRC8_RESULT_VALUE               0x9A
#define CRC8_ITU_RESULT_VALUE           0xCF
#define CRC8_ROHC_RESULT_VALUE          0x05
#define CRC16_IBM_RESULT_VALUE          0x2698
#define CRC16_MAXIM_RESULT_VALUE        0xD967
#define CRC16_USB_RESULT_VALUE          0xFD67
#define CRC16_MODBUS_RESULT_VALUE       0x0298
#define CRC16_CCIT_RESULT_VALUE         0xE376
#define CRC16_CCIT_FALSE_RESULT_VALUE   0x5197
#define CRC16_X25_RESULT_VALUE          0x1FA8
#define CRC16_XMODEM_RESULT_VALUE       0xD557
#define CRC32_RESULT_VALUE              0x89F75D91
#define CRC32_MPEG2_RESULT_VALUE        0x4CC93FE2

typedef struct {
    char *name;
    CRC_AlgorithmMode algoMode;
    unsigned int inputData;
    unsigned int outputData;
} CRC_AlgorithmTest;

CRC_AlgorithmTest g_algorithmTestTable[] = {
#if (SUPPORT_CRC_V1 == true)
    {"CRC8",             CRC8,              CRC_TEST_INPUT_DATA, CRC8_RESULT_VALUE            },
    {"CRC8_ITU",         CRC8_ITU,          CRC_TEST_INPUT_DATA, CRC8_ITU_RESULT_VALUE        },
    {"CRC16_MAXIM",      CRC16_MAXIM,       CRC_TEST_INPUT_DATA, CRC16_MAXIM_RESULT_VALUE     },
    {"CRC16_USB",        CRC16_USB,         CRC_TEST_INPUT_DATA, CRC16_USB_RESULT_VALUE       },
    {"CRC16_X25",        CRC16_X25,         CRC_TEST_INPUT_DATA, CRC16_X25_RESULT_VALUE       },
    {"CRC16_CCIT",       CRC16_CCITT,       CRC_TEST_INPUT_DATA, CRC16_CCIT_RESULT_VALUE      },
    {"CRC32_MPEG2",      CRC32_MPEG2,       CRC_TEST_INPUT_DATA, CRC32_MPEG2_RESULT_VALUE     },
#endif
    {"CRC8_ROHC",        CRC8_ROHC,         CRC_TEST_INPUT_DATA, CRC8_ROHC_RESULT_VALUE       },
    {"CRC16_IBM",        CRC16_IBM,         CRC_TEST_INPUT_DATA, CRC16_IBM_RESULT_VALUE       },
    {"CRC16_MODBUS",     CRC16_MODBUS,      CRC_TEST_INPUT_DATA, CRC16_MODBUS_RESULT_VALUE    },
    {"CRC16_CCIT_FALSE", CRC16_CCITT_FALSE, CRC_TEST_INPUT_DATA, CRC16_CCIT_FALSE_RESULT_VALUE},
    {"CRC16_XMODEM",     CRC16_XMODEM,      CRC_TEST_INPUT_DATA, CRC16_XMODEM_RESULT_VALUE    },
    {"CRC32",            CRC32,             CRC_TEST_INPUT_DATA, CRC32_RESULT_VALUE           },
};

const unsigned int CRC_ALGORITHM_TABLE_SIZE = sizeof(g_algorithmTestTable) / sizeof(g_algorithmTestTable[0]);

void CRC_CallbackFunc(void *param);
/**
  * @brief Test the check-in function.
  * @param None
  * @retval Value of @ref BASE_StatusType.
  */
BASE_StatusType CRC_CheckSample(void)
{
    SystemInit();
    DBG_UartPrintInit(115200); /* 115200 : baud rate */
    /* Traversal of all algorithms and data formats */
    for (unsigned int i = 0; i < CRC_ALGORITHM_TABLE_SIZE; ++i) {
        g_checkCrcHandle.handleEx.algoMode = g_algorithmTestTable[i].algoMode;
        HAL_CRC_Init(&g_checkCrcHandle);
        if (HAL_CRC_CheckInputData(&g_checkCrcHandle, &g_algorithmTestTable[i].inputData, \
                                   CRC_TEST_DATA_LENTH, g_algorithmTestTable[i].outputData)) {
            DBG_PRINTF("CRC algorithm : %s   check success! \r\n", g_algorithmTestTable[i].name);
        } else {
            DBG_PRINTF("CRC algorithm : %s   check failed! value should be 0x%x \r\n", \
                g_algorithmTestTable[i].name, g_algorithmTestTable[i].outputData);
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief register callback function.
  * @param param Value of @ref CRC_Handle.
  * @retval None
  */
void CRC_CallbackFunc(void *param)
{
    /* User Add code here */
    BASE_FUNC_UNUSED(param);
    DBG_PRINTF("CRC pready timeout! \r\n");
}
