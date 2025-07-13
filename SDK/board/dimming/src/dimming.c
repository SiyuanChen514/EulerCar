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
  * @file      dimming.c
  * @author    MCU Application Team
  * @brief     LED Dimming module application.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the dimming.
  *            + Initialization and de-initialization functions
  *            + Dimming function
  * @verbatim
  * usage:
  * 1) Use the chip config tool to generate the GPT/TIMER initialization code.
  * 2) Call the BOARD_DIM_Init() function to select the specified GPT, TIMER, and dimming period.
  * 3) Call the BOARD_DIM_SetDuty() function to dimming.
  * @endverbatim
  */

/* Includes ------------------------------------------------------------------ */
#include "dimming.h"

/* Private variables --------------------------------------------------------- */
BOARD_DIM_Handle g_dimInfo[BOARD_DIM_NUM]; /**< Dimming handle. */
const unsigned short g_dimTables[] = {
    3, 7, 11, 17, 24, 32, 41, 52, 65, 78, 93, 110, 128, 148, 169, 192, 216, 242, 269, 298, 329, 361, 395, 431,
    468, 507, 548, 591, 635, 681, 729, 778, 829, 882, 937, 994, 1052, 1113, 1175, 1239, 1305, 1373, 1442, 1514, 1587,
    1662, 1739, 1818, 1899, 1982, 2067, 2154, 2243, 2333, 2426, 2521, 2617, 2716, 2817, 2919, 3024, 3130, 3239, 3350,
    3462, 3577, 3694, 3813, 3934, 4057, 4182, 4309, 4438, 4569, 4702, 4838, 4975, 5115, 5257, 5401, 5546, 5695, 5845,
    5997, 6152, 6308, 6467, 6628, 6791, 6956, 7124, 7294, 7465, 7639, 7815, 7994, 8174, 8357, 8542, 8729, 8919, 9110,
    9304, 9500, 9699, 9899, 10102, 10307, 10514, 10724, 10935, 11150, 11366, 11584, 11805, 12028, 12254, 12481, 12711,
    12944, 13178, 13415, 13654, 13896, 14140, 14386, 14634, 14885, 15138, 15393, 15651, 15911, 16174, 16438, 16705,
    16975, 17247, 17521, 17797, 18076, 18357, 18641, 18927, 19215, 19506, 19799, 20095, 20393, 20693, 20996, 21301,
    21608, 21918, 22231, 22545, 22862, 23182, 23504, 23828, 24155, 24484, 24816, 25150, 25487, 25826, 26167, 26511,
    26857, 27206, 27557, 27911, 28267, 28626, 28987, 29351, 29717, 30085, 30456, 30830, 31206, 31584, 31965, 32349,
    32735, 33123, 33514, 33907, 34303, 34702, 35103, 35506, 35912, 36321, 36732, 37145, 37561, 37980, 38401, 38825,
    39251, 39680, 40111, 40545, 40981, 41420, 41862, 42306, 42753, 43202, 43653, 44108, 44565, 45024, 45486, 45951,
    46418, 46888, 47360, 47835, 48312, 48792, 49275, 49760, 50248, 50738, 51232, 51727, 52225, 52726, 53230, 53736,
    54245, 54756, 55270, 55786, 56305, 56827, 57352, 57879, 58408, 58941, 59476, 60013, 60553, 61096, 61642, 62190,
    62741, 63294, 63850, 64409, 64970, 65535}; /**< dimming curve :y= 65535*（x/253）^(2.2). */

/**
  * @brief Initial configuration.
  * @param gptHandle GPT Handle.
  * @param timerHandle Timer Handle.
  * @param index Specifies the index value, in [0, BOARD_DIM_NUM).
  * @param targetTimeMs Full-dark to full-bright finish time.
  */
void BOARD_DIM_Init(GPT_Handle *gptHandle, TIMER_Handle *timerHandle, int index, int targetTimeMs)
{
    DIM_PARAM_CHECK_NO_RET(index >= 0 && index < BOARD_DIM_NUM);
    g_dimInfo[index].status = BOARD_DIM_NOT_RUNNING;
    g_dimInfo[index].gptHandle = gptHandle;

    unsigned int freq = HAL_CRG_GetIpFreq((void *)timerHandle->baseAddress);
    unsigned int loadVal;
    if (targetTimeMs == 0) {
        /* 1000 is the conversion unit for calculating the tick value. */
        loadVal = ((freq / 1000000u) * targetTimeMs * 1000) / 100; /* 10ms */
    } else {
        /* 1000 is the conversion unit for calculating the tick value. */
        loadVal = ((freq / 1000000u) * targetTimeMs * 1000)/(BOARD_DIM_TABLE_MAX_INDEX + 1);
    }
    timerHandle->load = loadVal - 1;
    timerHandle->bgLoad = loadVal -1;
    HAL_TIMER_Config(timerHandle, TIMER_CFG_LOAD);
    HAL_TIMER_Config(timerHandle, TIMER_CFG_BGLOAD);
    HAL_TIMER_RegisterCallback(timerHandle, TIMER_PERIOD_FIN, BOARD_DIM_TimerCallBack);
    HAL_TIMER_Start(timerHandle);
}

/**
  * @brief Deinitialization Function.
  * @param index Index value specified in function BOARD_DIM_Init(), in [0, BOARD_DIM_NUM).
  */
void BOARD_DIM_DeInit(int index)
{
    DIM_PARAM_CHECK_NO_RET(index >= 0 && index < BOARD_DIM_NUM);
    g_dimInfo[index].status = BOARD_DIM_NOT_REGIS;
}

/**
  * @brief Adjust the light dim level.
  * @param index Index value used in the initialization function BOARD_DIM_Init().
  * @param targetLevel Target value of light jumping, in [0, BOARD_DIM_TABLE_MAX_INDEX].
  * @return BOARD_DIM_Ret @ref BOARD_DIM_Ret.
  */
BOARD_DIM_Ret BOARD_DIM_SetDuty(int index, int targetLevel)
{
    DIM_PARAM_CHECK_WITH_RET((index >= 0 && index < BOARD_DIM_NUM), BOARD_DIM_RET_INVALID_PARAM);
    DIM_PARAM_CHECK_WITH_RET((targetLevel >= 0 && targetLevel <= BOARD_DIM_TABLE_MAX_INDEX), \
                             BOARD_DIM_RET_INVALID_PARAM);

    if (g_dimInfo[index].status == BOARD_DIM_NOT_REGIS) {
        return BOARD_DIM_RET_ERR_NOT_REGIS;
    }

    unsigned int currentDutyVal;
#if defined (CHIP_3065HRPICZ) || defined(CHIP_3065HRPIRZ) || defined(CHIP_3061HRPIKZ)
    currentDutyVal= g_dimInfo[index].gptHandle->duty;
#endif
#if defined (CHIP_ZDU3061MNPICA) || defined(CHIP_ZDU3061MNPIKA)
    currentDutyVal = g_dimInfo[index].gptHandle->refB0.refdot;
#endif
    if (currentDutyVal == BOARD_DIM_TABLE_MAX_VALUE) {
        g_dimInfo[index].nowIndex = BOARD_DIM_TABLE_MAX_INDEX;
    } else {
        g_dimInfo[index].nowIndex = BASE_FUNC_FindArrayValue(g_dimTables, BOARD_DIM_TABLE_MAX_INDEX + 1, \
                                                             currentDutyVal);
    }
    g_dimInfo[index].targetIndex = targetLevel;
    g_dimInfo[index].status = BOARD_DIM_IN_RUNNING;

    return BOARD_DIM_RET_OK;
}

/**
  * @brief Timer callback function. Set duty cycle according to state to complete dimming.
  * @param irqNum Interrupt number.
  * @param param Pointer to the timer handle.
  */
void BOARD_DIM_TimerCallBack(void *param)
{
    BASE_FUNC_UNUSED(param);
    for (int i = 0; i < BOARD_DIM_NUM; i++) {
        if (g_dimInfo[i].status == BOARD_DIM_NOT_RUNNING) {
            continue;
        }
        /* The current luminance index value is approximated to the target value. When the current luminance index
           value is equal to the target value, the status is set to BOARD_DIM_NOT_RUNNING. */
        if (g_dimInfo[i].nowIndex < g_dimInfo[i].targetIndex) {
            g_dimInfo[i].nowIndex++;
        } else if (g_dimInfo[i].nowIndex > g_dimInfo[i].targetIndex) {
            g_dimInfo[i].nowIndex--;
        } else {
            g_dimInfo[i].status = BOARD_DIM_NOT_RUNNING;
        }

#if defined (CHIP_3065HRPICZ) || defined(CHIP_3065HRPIRZ) || defined(CHIP_3061HRPIKZ)
        g_dimInfo[i].gptHandle->duty = g_dimTables[g_dimInfo[i].nowIndex];
#endif
#if defined (CHIP_ZDU3061MNPICA) || defined(CHIP_ZDU3061MNPIKA)
        g_dimInfo[i].gptHandle->refB0.refdot = g_dimTables[g_dimInfo[i].nowIndex];
#endif
        HAL_GPT_Config(g_dimInfo[i].gptHandle);
    }
}