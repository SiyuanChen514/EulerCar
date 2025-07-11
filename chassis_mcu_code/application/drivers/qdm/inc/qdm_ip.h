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
  * @file    qdm_ip.h
  * @author  MCU Driver Team
  * @brief   Header file containing QDM module DCL driver functions.
  *          This file provides functions to manage the following functionalities of QDM module.
  *          + Definition of QDM configuration parameters.
  *          + QDM registers mapping structure.
  *          + Direct Configuration Layer driver functions.
  */

#ifndef McuMagicTag_QDM_IP_H
#define McuMagicTag_QDM_IP_H

#include "baseinc.h"

#ifdef QDM_PARAM_CHECK
#define QDM_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define QDM_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define QDM_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define QDM_ASSERT_PARAM(para) ((void)0U)
#define QDM_PARAM_CHECK_NO_RET(para) ((void)0U)
#define QDM_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

#define QDM_MAX_FILTER_LEVEL 0x00001FFF
#define QDM_PPU_MAX_SYNCOUT_PW 0x00000FFF
/**
  * @addtogroup QDM
  * @{
  */

/**
  * @defgroup QDM_IP QDM_IP
  * @brief QDM_IP: qdm_v0.
  * @{
  */

/**
 * @defgroup QDM_Param_Def QDM Parameters Definition
 * @brief Definition of QDM configuration parameters
 * @{
 */


/**
  * @brief Emulation mode of QDM module.
  * @details Emulation mode:
  *         + QDM_EMULATION_MODE_STOP_IMMEDIATELY -- The position counter, unit timer,
  *             capture timer all stop immediately.
  *         + QDM_EMULATION_MODE_STOP_AT_ROLLOVER --
  *             The position counter, unit timer count until period rollover,
  *             and the capture timer counts until the next unit period event.
  *         + QDM_EMULATION_MODE_RUN_FREE --  The position counter, unit timer,
  *             capture timer are all unaffected by an emulation suspend.
  */
typedef enum {
    QDM_EMULATION_MODE_STOP_IMMEDIATELY = 0x00000000U,
    QDM_EMULATION_MODE_STOP_AT_ROLLOVER = 0x00000001U,
    QDM_EMULATION_MODE_RUN_FREE = 0x00000002U,
} QDM_EmulationMode;

/**
  * @brief Status flag of QDM module.
  * @details status flag:
  *         + QDM_STATUS_POS_CNT_ERR -- Position counter error
  *         + QDM_STATUS_1ST_IDX_OCCURRED -- First index pulse occurred
  *         + QDM_STATUS_DIR_ON_1ST_IDX -- Direction of first index event
  *         + QDM_STATUS_CAP_DIR_ERR -- Direction changed between position capture events
  *         + QDM_STATUS_TSU_OVERFLW_ERR -- Timer stamp timer overflow
  *         + QDM_STATUS_SPEED_LOST -- Speed lost status
  *         + QDM_STATUS_DIR_FLAG -- Quadrature direction
  *         + QDM_STATUS_UNIT_POS_EVENT -- Unit position event detected
  */
typedef enum {
    QDM_STATUS_POS_CNT_ERR = 0x00000001U,
    QDM_STATUS_1ST_IDX_OCCURRED = 0x00000002U,
    QDM_STATUS_DIR_ON_1ST_IDX = 0x00000004U,
    QDM_STATUS_CAP_DIR_ERR = 0x00000008U,
    QDM_STATUS_TSU_OVERFLW_ERR = 0x00000010U,
    QDM_STATUS_SPEED_LOST = 0x00000020U,
    QDM_STATUS_DIR_FLAG = 0x00000040U,
    QDM_STATUS_UNIT_POS_EVENT = 0x00000080U,
} QDM_StatusFlag;

/**
  * @brief Decoder mode of QDM module.
  * @details Decoder mode
  *         + QDM_QUADRATURE_COUNT -- Quadrature-clock mode
  *         + QDM_CLOCK_DIR_COUNT -- Direction-count mode
  *         + QDM_NONSTANDARD_TYPE1 -- Non-standard mode 1
  *         + QDM_NONSTANDARD_TYPE2 -- Non-standard mode 2
  */
typedef enum {
    QDM_QUADRATURE_COUNT = 0x00000000U,
    QDM_CLOCK_DIR_COUNT = 0x00000001U,
    QDM_NONSTANDARD_TYPE1 = 0x00000002U,
    QDM_NONSTANDARD_TYPE2 = 0x00000003U,
} QDM_DecoderMode;

/**
  * @brief Decode resolution of QDM module.
  * @details Decode resolution:
  *         + QDM_1X_RESOLUTION -- Count rising edge of QDMA/QDMB only
  *         + QDM_2X_RESOLUTION -- Count rising and falling edge of QDMA/QDMB
  *         + QDM_4X_RESOLUTION -- Count rising and falling edge of both QDMA and QDMB
  */
typedef enum {
    QDM_1X_RESOLUTION = 0x00000000U,
    QDM_2X_RESOLUTION = 0x00000001U,
    QDM_4X_RESOLUTION = 0x00000002U,
} QDM_Resolution;

/**
  * @brief Count mode of position processing submodule.
  */
typedef enum {
    QDM_PPU_COUNT_MODE_CLK_DIR = 0x00000000U,
    QDM_PPU_COUNT_MODE_INCREASE = 0x00000001U,
    QDM_PPU_COUNT_MODE_DECREASE = 0x00000002U,
} QDM_PPUCountMode;

/**
  * @brief Reset mode of position counter.
  * @details Reset mode:
  *         + QDM_POSITION_RESET_IDX -- Reset position on the rising edge of inde pulse
  *         + QDM_POSITION_RESET_MAX_POS -- Reset position on maximum position QCNTMAX
  *         + QDM_POSITION_RESET_1ST_IDX -- Reset position on the first index pulse
  *         + QDM_POSITION_RESET_UNIT_TIME_OUT -- Reset position on a unit time trigger
  */
typedef enum {
    QDM_POSITION_RESET_IDX = 0x00000000,
    QDM_POSITION_RESET_MAX_POS = 0x00000001,
    QDM_POSITION_RESET_1ST_IDX = 0x00000002,
    QDM_POSITION_RESET_UNIT_TIME_OUT = 0x00000003,
} QDM_PosResetMode;

/**
  * @brief Initializaion mode of the index of position counter.
  * @details Initializaion mode:
  *         + QDM_POSITION_INIT_DO_NOTHING -- No action is configured
  *         + QDM_POSITION_INIT_RISING_INDEX -- On rising edge of index
  *         + QDM_POSITION_INIT_FALLING_INDEX -- On falling edge of index
  */
typedef enum {
    QDM_POSITION_INIT_DO_NOTHING = 0x00000000U,
    QDM_POSITION_INIT_RISING_INDEX = 0x00000002U,
    QDM_POSITION_INIT_FALLING_INDEX = 0x00000003U,
} QDM_PosIdxInitMode;

/**
  * @brief Shadow load mode of compare counter.
  * @details Load mode:
  *         + QDM_COMPARE_LOAD_ON_ZERO -- Load on QPOSCNT = 0
  *         + QDM_COMPARE_LOAD_ON_MATCH -- Load on QPOSCNT = QPOSCMP
  */
typedef enum {
    QDM_COMPARE_LOAD_ON_ZERO = 0x00000000U,
    QDM_COMPARE_LOAD_ON_MATCH = 0x00000001U,
} QDM_CompShadowLoad;

/**
  * @brief Polarity of sync-out pulse for position compare.
  */
typedef enum {
    QDM_SYNC_OUT_HIGH = 0x00000000U,
    QDM_SYNC_OUT_LOW = 0x00000001U,
} QDM_CompSyncOutPolarity;

/**
  * @brief Lock mode of index event.
  * @details Lock mode:
  *         + QDM_LOCK_RESERVE -- Do not lock
  *         + QDM_LOCK_RISING_INDEX -- On rising edge of index
  *         + QDM_LOCK_FALLING_INDEX -- On falling edge of index
  *         + QDM_LOCK_SW_INDEX_MARKER -- On software index marker
  */
typedef enum {
    QDM_LOCK_RESERVE = 0x00000000,
    QDM_LOCK_RISING_INDEX = 0x00000001,
    QDM_LOCK_FALLING_INDEX = 0x00000002,
    QDM_LOCK_SW_INDEX_MARKER = 0x00000003,
} QDM_IndexLockMode;

/**
  * @brief Prescaler of Time Stamp Unit clock.
  * @details Prescaler:
  *         + QDM_TSU_CLK_DIV_1 -- TSUCLK = SYSCLKOUT/1
  *         + QDM_TSU_CLK_DIV_2 -- TSUCLK = SYSCLKOUT/2
  *         + QDM_TSU_CLK_DIV_4 -- TSUCLK = SYSCLKOUT/4
  *         + QDM_TSUE_CLK_DIV_8 -- TSUCLK = SYSCLKOUT/8
  *         + QDM_TSU_CLK_DIV_16 -- TSUCLK = SYSCLKOUT/16
  *         + QDM_TSU_CLK_DIV_32 -- TSUCLK = SYSCLKOUT/32
  *         + QDM_TSU_CLK_DIV_64 -- TSUCLK = SYSCLKOUT/64
  *         + QDM_TSU_CLK_DIV_128 -- TSUCLK = SYSCLKOUT/128
  *         + QDM_TSU_CLK_DIV_256 -- TSUCLK = SYSCLKOUT/256
  */
typedef enum {
    QDM_TSU_CLK_DIV_1 = 0x00000000U,
    QDM_TSU_CLK_DIV_2 = 0x00000001U,
    QDM_TSU_CLK_DIV_4 = 0x00000002U,
    QDM_TSUE_CLK_DIV_8 = 0x00000003U,
    QDM_TSU_CLK_DIV_16 = 0x00000004U,
    QDM_TSU_CLK_DIV_32 = 0x00000005U,
    QDM_TSU_CLK_DIV_64 = 0x00000006U,
    QDM_TSU_CLK_DIV_128 = 0x00000007U,
    QDM_TSU_CLK_DIV_256 = 0x00000008U,
} QDM_TSUCLKPrescale;

/**
  * @brief Prescaler of Unit Position Event.
  * @details Prescaler:
  *         + QDM_UNIT_POS_EVNT_DIV_1 -- UPEVNT = QCLK/1
  *         + QDM_UNIT_POS_EVNT_DIV_2 -- UPEVNT = QCLK/2
  *         + QDM_UNIT_POS_EVNT_DIV_4 -- UPEVNT = QCLK/4
  *         + QDM_UNIT_POS_EVNT_DIV_8 -- UPEVNT = QCLK/8
  *         + QDM_UNIT_POS_EVNT_DIV_16 -- UPEVNT = QCLK/16
  *         + QDM_UNIT_POS_EVNT_DIV_32 -- UPEVNT = QCLK/32
  *         + QDM_UNIT_POS_EVNT_DIV_64 -- UPEVNT = QCLK/64
  *         + QDM_UNIT_POS_EVNT_DIV_128 -- UPEVNT = QCLK/128
  *         + QDM_UNIT_POS_EVNT_DIV_256 -- UPEVNT = QCLK/256
  *         + QDM_UNIT_POS_EVNT_DIV_512 -- UPEVNT = QCLK/512
  *         + QDM_UNIT_POS_EVNT_DIV_1024 -- UPEVNT = QCLK/1024
  *         + QDM_UNIT_POS_EVNT_DIV_2048 -- UPEVNT = QCLK/2048
  */
typedef enum {
    QDM_UNIT_POS_EVNT_DIV_1 = 0x00000000U,
    QDM_UNIT_POS_EVNT_DIV_2 = 0x00000001U,
    QDM_UNIT_POS_EVNT_DIV_4 = 0x00000002U,
    QDM_UNIT_POS_EVNT_DIV_8 = 0x00000003U,
    QDM_UNIT_POS_EVNT_DIV_16 = 0x00000004U,
    QDM_UNIT_POS_EVNT_DIV_32 = 0x00000005U,
    QDM_UNIT_POS_EVNT_DIV_64 = 0x00000006U,
    QDM_UNIT_POS_EVNT_DIV_128 = 0x00000007U,
    QDM_UNIT_POS_EVNT_DIV_256 = 0x00000008U,
    QDM_UNIT_POS_EVNT_DIV_512 = 0x00000009U,
    QDM_UNIT_POS_EVNT_DIV_1024 = 0x0000000AU,
    QDM_UNIT_POS_EVNT_DIV_2048 = 0x0000000BU,
} QDM_UPEvntPrescale;

/**
  * @brief Lock mode of Time Stamp Unit.
  * @details Lock mode:
  *         + QDM_TSU_LOCK_ON_SW_READ -- When software read QPOSCNT
  *         + QDM_TSU_LOCK_ON_UTTRG -- When unit time trigger happens
  */
typedef enum {
    QDM_TSU_LOCK_ON_SW_READ = 0x00000000U,
    QDM_TSU_LOCK_ON_UTTRG = 0x00000001U,
} QDM_TSULockMode;

/**
  * @brief Working mode of Period Trigger Unit.
  */
typedef enum {
    QDM_PERIOD_TRIGGER_MODE = 0x00000000U,
    QDM_WATCHDOG_MODE = 0x00000001U,
} QDM_PTUMode;

/**
  * @brief Lock mode of Period Trigger Unit.
  * @details Lock mode:
  *         + QDM_LOCK_POSCNT_READ_BY_CPU -- When QPOSCNT read by CPU/DMA,
  *             QCTMR and QCPRD are locked
  *         + QDM_LOCK_UNIT_TIME_TRIGGER,-- When PTU is enabled and unit time triggers,
  *             QPOSCNT, QCTMR, QCPRD are locked
  */
typedef enum {
    QDM_LOCK_POSCNT_READ_BY_CPU,
    QDM_LOCK_UNIT_TIME_TRIGGER,
} QDM_TriggerLockMode;           /* QPOSCNT, QCTMR, QCPRD lock event */

/**
  * @brief Interrupt events of QMD module.
  * @details Interrupt events:
  *         +  QDM_INT_POS_CNT_ERROR -- Position count error
  *         +  QDM_INT_PHASE_ERROR -- Quadrature phase error
  *         +  QDM_INT_WATCHDOG -- Speed lost error
  *         +  QDM_INT_DIR_CHANGE -- Quadrature direction change
  *         +  QDM_INT_UNDERFLOW -- Position counter underflow
  *         +  QDM_INT_OVERFLOW -- Position counter overflow
  *         +  QDM_INT_POS_COMP_READY -- Position-compare ready
  *         +  QDM_INT_POS_COMP_MATCH -- Position-compare match
  *         +  QDM_INT_INDEX_EVNT_LATCH -- Index event lock
  *         +  QDM_INT_UNIT_TIME_OUT -- Unit time-out
  */
typedef enum {
    QDM_INT_POS_CNT_ERROR = 0x00000001U,
    QDM_INT_PHASE_ERROR = 0x00000002U,
    QDM_INT_WATCHDOG = 0x00000004U,
    QDM_INT_DIR_CHANGE = 0x00000008U,
    QDM_INT_UNDERFLOW = 0x00000010U,
    QDM_INT_OVERFLOW = 0x00000020U,
    QDM_INT_POS_COMP_READY = 0x00000040U,
    QDM_INT_POS_COMP_MATCH = 0x00000080U,
    QDM_INT_INDEX_EVNT_LATCH = 0x00000100U,
    QDM_INT_UNIT_TIME_OUT = 0x00000200U,
} QDM_InterruptEvent;

/**
  * @brief QDM TSU prescaler
  * @details prescaler values:
  *        + QDM_TSU_PRESCALER_EQUAL -- Equal to the clock cycle
  *        + QDM_TSU_PRESCALER_2X -- 2x clock cycle
  *        + QDM_TSU_PRESCALER_4X -- 2x clock cycle
  *        + QDM_TSU_PRESCALER_8X -- 8x clock cycle
  *        + QDM_TSU_PRESCALER_16X -- 16x clock cycle
  *        + QDM_TSU_PRESCALER_32X -- 32x clock cycle
  *        + QDM_TSU_PRESCALER_64X -- 64x clock cycle
  *        + QDM_TSU_PRESCALER_128X -- 128x clock cycle
  *        + QDM_TSU_PRESCALER_256X -- 256x clock cycle
  */
typedef enum {
    QDM_TSU_PRESCALER_EQUAL     = 0x00000000U,
    QDM_TSU_PRESCALER_2X        = 0x00000001U,
    QDM_TSU_PRESCALER_4X        = 0x00000002U,
    QDM_TSU_PRESCALER_8X        = 0x00000003U,
    QDM_TSU_PRESCALER_16X       = 0x00000004U,
    QDM_TSU_PRESCALER_32X       = 0x00000005U,
    QDM_TSU_PRESCALER_64X       = 0x00000006U,
    QDM_TSU_PRESCALER_128X      = 0x00000007U,
    QDM_TSU_PRESCALER_256X      = 0x00000008U,
} QDM_TSUPrescaler;

/**
  * @brief QDM CEVT prescaler
  * @details prescaler values:
  *         + QDM_CEVT_PRESCALER_DIVI1 -- Don't divided
  *         + QDM_CEVT_PRESCALER_DIVI2 -- Divide by 2
  *         + QDM_CEVT_PRESCALER_DIVI4 -- Divide by 4
  *         + QDM_CEVT_PRESCALER_DIVI8 -- Divide by 8
  *         + QDM_CEVT_PRESCALER_DIVI16 -- Divide by 16
  *         + QDM_CEVT_PRESCALER_DIVI32 -- Divide by 32
  *         + QDM_CEVT_PRESCALER_DIVI64 -- Divide by 64
  *         + QDM_CEVT_PRESCALER_DIVI128 -- Divide by 128
  *         + QDM_CEVT_PRESCALER_DIVI256 -- Divide by 256
  *         + QDM_CEVT_PRESCALER_DIVI512 -- Divide by 512
  *         + QDM_CEVT_PRESCALER_DIVI1024 -- Divide by 1024
  *         + QDM_CEVT_PRESCALER_DIVI2048 -- Divide by 2048
  */
typedef enum {
    QDM_CEVT_PRESCALER_DIVI1        = 0x00000000U,
    QDM_CEVT_PRESCALER_DIVI2        = 0x00000001U,
    QDM_CEVT_PRESCALER_DIVI4        = 0x00000002U,
    QDM_CEVT_PRESCALER_DIVI8        = 0x00000003U,
    QDM_CEVT_PRESCALER_DIVI16       = 0x00000004U,
    QDM_CEVT_PRESCALER_DIVI32       = 0x00000005U,
    QDM_CEVT_PRESCALER_DIVI64       = 0x00000006U,
    QDM_CEVT_PRESCALER_DIVI128      = 0x00000007U,
    QDM_CEVT_PRESCALER_DIVI256      = 0x00000008U,
    QDM_CEVT_PRESCALER_DIVI512      = 0x00000009U,
    QDM_CEVT_PRESCALER_DIVI1024     = 0x0000000AU,
    QDM_CEVT_PRESCALER_DIVI2048     = 0x0000000BU,
} QDM_CEVTPrescaler;

/**
  * @brief QDM counter reset mode
  */
typedef enum {
    QDM_IDX_INIT_DISABLE        = 0x00000000U,
    QDM_IDX_INIT_AUTO           = 0x00000001U,
    QDM_IDX_INIT_Z_UP           = 0x00000002U,
    QDM_IDX_INIT_Z_DOWN         = 0x00000003U,
} QDM_PcntIdxInitMode;

/**
  * @brief QDM lock triggle mode
  */
typedef enum {
    QDM_TRG_BY_READ             = 0x00000000U,
    QDM_TRG_BY_CYCLE            = 0x00000001U,
} QDM_QtrgLockMode;

/**
  * @brief QDM PTU work mode
  */
typedef enum {
    QDM_PTU_MODE_CYCLE          = 0x00000000U,
    QDM_PTU_MODE_WATCHDOG       = 0x00000001U,
} QDM_PtuMode;

/**
  * @brief QDM count mode
  */
typedef enum {
    QDM_PCNT_MODE_BY_DIR        = 0x00000000U,
    QDM_PCNT_MODE_UP            = 0x00000001U,
    QDM_PCNT_MODE_DOWN          = 0x00000002U,
} QDM_PcntMode;

/**
  * @brief QDM counter reset mode
  */
typedef enum {
    QDM_PCNT_RST_AUTO           = 0x00000000U,
    QDM_PCNT_RST_OVF            = 0x00000001U,
    QDM_PCNT_RST_HARDWARE_ONCE  = 0x00000002U,
    QDM_PCNT_RST_BY_PTU         = 0x00000003U,
} QDM_PcntRstMode;

/**
  * @brief  QDM swap selection
  */
typedef enum {
    QDM_SWAP_DISABLE            = 0x00000000U,
    QDM_SWAP_ENABLE             = 0x00000001U,
} QDM_SwapSelect;

/**
  * @brief Check whether the EMU mode is used.
  * @param mode QDM mode
  * @retval true
  * @retval false
  */
static inline bool IsEmuMode(unsigned int mode)
{
    if (mode == QDM_EMULATION_MODE_STOP_IMMEDIATELY || mode == QDM_EMULATION_MODE_STOP_AT_ROLLOVER ||
        mode == QDM_EMULATION_MODE_RUN_FREE) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the Z Index lock mode is used.
  * @param mode QDM mode
  * @retval true
  * @retval false
  */
static inline bool IsLockMode(unsigned int mode)
{
    if (mode == QDM_LOCK_RESERVE || mode == QDM_LOCK_RISING_INDEX ||
        mode == QDM_LOCK_FALLING_INDEX || mode == QDM_LOCK_SW_INDEX_MARKER) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the Decode mode is used.
  * @param mode QDM decode mode
  * @retval true
  * @retval false
  */
static inline bool IsDecodeMode(unsigned int mode)
{
    if (mode <= QDM_NONSTANDARD_TYPE2) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the resolution is right.
  * @param mode QDM mode
  * @retval true
  * @retval false
  */
static inline bool IsResolution(unsigned int mode)
{
    if (mode == QDM_1X_RESOLUTION || mode == QDM_2X_RESOLUTION || mode == QDM_4X_RESOLUTION) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the swap is right.
  * @param mode QDM mode
  * @retval true
  * @retval false
  */
static inline bool IsSwap(unsigned int mode)
{
    if (mode == QDM_SWAP_DISABLE || mode == QDM_SWAP_ENABLE) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the lock triggle mode is used.
  * @param mode QDM mode
  * @retval true
  * @retval false
  */
static inline bool IsTrgLockMode(unsigned int mode)
{
    if (mode == QDM_TRG_BY_READ || mode == QDM_TRG_BY_CYCLE) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the ptu mode is used.
  * @param mode QDM mode
  * @retval true
  * @retval false
  */
static inline bool IsPtuMode(unsigned int mode)
{
    if (mode == QDM_PTU_MODE_CYCLE || mode == QDM_PTU_MODE_WATCHDOG) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the position counter is used.
  * @param mode QDM mode
  * @retval true
  * @retval false
  */
static inline bool IsPcntMode(unsigned int mode)
{
    if (mode == QDM_PCNT_MODE_BY_DIR || mode == QDM_PCNT_MODE_UP || mode == QDM_PCNT_MODE_DOWN) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the PcntRstMode is used.
  * @param mode QDM mode
  * @retval true
  * @retval false
  */
static inline bool IsPcntRstMode(unsigned int mode)
{
    if (mode == QDM_PCNT_RST_AUTO || mode == QDM_PCNT_RST_OVF ||
        mode == QDM_PCNT_RST_HARDWARE_ONCE || mode == QDM_PCNT_RST_BY_PTU) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the PcntIdxInitMode is used.
  * @param mode QDM mode
  * @retval true
  * @retval false
  */
static inline bool IsPcntIdxInitMode(unsigned int mode)
{
    if (mode == QDM_IDX_INIT_DISABLE || mode == QDM_IDX_INIT_AUTO ||
        mode == QDM_IDX_INIT_Z_UP || mode == QDM_IDX_INIT_Z_DOWN) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the TsuPrescaler is used.
  * @param mode QDM mode
  * @retval true
  * @retval false
  */
static inline bool IsTsuPrescaler(unsigned int mode)
{
    /* Check whether the TSU prescaler is right. */
    if (mode == QDM_TSU_PRESCALER_EQUAL || mode == QDM_TSU_PRESCALER_2X ||
        mode == QDM_TSU_PRESCALER_4X || mode == QDM_TSU_PRESCALER_8X ||
        mode == QDM_TSU_PRESCALER_16X || mode == QDM_TSU_PRESCALER_32X ||
        mode == QDM_TSU_PRESCALER_64X || mode == QDM_TSU_PRESCALER_128X ||
        mode == QDM_TSU_PRESCALER_256X) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the CevtPrescaler is used.
  * @param mode QDM mode
  * @retval true
  * @retval false
  */
static inline bool IsCevtPrescaler(unsigned int mode)
{
    /* Check whether the CEVT prescaler is right. */
    if (mode == QDM_CEVT_PRESCALER_DIVI1 || mode == QDM_CEVT_PRESCALER_DIVI2 ||
        mode == QDM_CEVT_PRESCALER_DIVI4 || mode == QDM_CEVT_PRESCALER_DIVI8 ||
        mode == QDM_CEVT_PRESCALER_DIVI16 || mode == QDM_CEVT_PRESCALER_DIVI32 ||
        mode == QDM_CEVT_PRESCALER_DIVI64 || mode == QDM_CEVT_PRESCALER_DIVI128 ||
        mode == QDM_CEVT_PRESCALER_DIVI256 || mode == QDM_CEVT_PRESCALER_DIVI512 ||
        mode == QDM_CEVT_PRESCALER_DIVI1024 || mode == QDM_CEVT_PRESCALER_DIVI2048) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the QDM_StatusFlag is used.
  * @param status QDM status flag
  * @retval true
  * @retval false
  */
static inline bool IsQDMStatusMode(QDM_StatusFlag status)
{
    /* Check whether the QDM Status flag is right. */
    if (status == QDM_STATUS_POS_CNT_ERR || status == QDM_STATUS_1ST_IDX_OCCURRED ||
        status == QDM_STATUS_DIR_ON_1ST_IDX || status == QDM_STATUS_CAP_DIR_ERR ||
        status == QDM_STATUS_TSU_OVERFLW_ERR || status == QDM_STATUS_SPEED_LOST ||
        status == QDM_STATUS_DIR_FLAG || status == QDM_STATUS_UNIT_POS_EVENT) {
        return true;
    }
    return false;
}

/**
  * @brief Check whether the QDM interrupt event type is right.
  * @param status QDM interrupt event
  * @retval true
  * @retval false
  */
static inline bool IsQDMInterruptEvent(QDM_InterruptEvent intEvt)
{
    /* Check whether the QDM interrupt type is right. */
    QDM_PARAM_CHECK_WITH_RET(intEvt >= QDM_INT_POS_CNT_ERROR, BASE_STATUS_ERROR);
    QDM_PARAM_CHECK_WITH_RET(intEvt <= QDM_INT_UNIT_TIME_OUT, BASE_STATUS_ERROR);
    if (intEvt == QDM_INT_POS_CNT_ERROR || ((unsigned int)intEvt % 2U) == 0) {
        return true;
    }
    return false;
}

/**
  * @}
  */


/**
  * @defgroup QDM_REG_Definition QDM Register Structure.
  * @brief QDM Register Structure Definition.
  * @{
  */

/**
  * @brief QDM version registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int month_day : 16;            /**< Month and day. */
        unsigned int year : 8;                  /**< Year. */
        unsigned int release_substep : 1;       /**< Version information. */
        unsigned int release_step : 1;          /**< Version information. */
        unsigned int release_ver : 1;           /**< Version information. */
        unsigned int reserved_0 : 5;
    } BIT;
} volatile QDM_QDMVER_REG;

/**
  * @brief QDM emulation mode configuration registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int emu_mode : 2;              /**< QDM emulation access mode. */
        unsigned int reserved_0 : 30;
    } BIT;
} volatile QDM_QEMUMODE_REG;

/**
  * @brief QDM control registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ptu_en : 1;                /**< PTU period triggle unit enable. */
        unsigned int ppu_en : 1;                /**< PPU position process unit enable. */
        unsigned int tsu_en : 1;                /**< TSU timestamp unit enable. */
        unsigned int ptu_mode : 1;              /**< PTU work mode. */
        unsigned int qtrg_lock_mode : 1;        /**< QDM triggle locked mode selection. */
        unsigned int reserved_0 : 3;
        unsigned int qdmi_polarity : 1;         /**< Z pulse polarity selection. */
        unsigned int qdmb_polarity : 1;         /**< B pulse polarity selection. */
        unsigned int qdma_polarity : 1;         /**< A pulse polarity selection. */
        unsigned int qdm_ab_swap : 1;           /**< Input signal swap of A pulse and B pulse. */
        unsigned int qdu_xclk : 2;              /**< QDM position pulse frequency multiplication. */
        unsigned int qdu_mode : 2;              /**< QDM decode mode. */
        unsigned int reserved_1 : 16;
    } BIT;
} volatile QDM_QCTRL_REG;

/**
  * @brief PPU control registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ppu_syncout_pw : 12;       /**< Pulse width selection of position comparison sync output. */
        unsigned int ppu_syncout_pl : 1;        /**< Polarity of position comparison sync output. */
        unsigned int syncout_en : 1;            /**< Position comparison sync output enable. */
        unsigned int reserved_0 : 2;
        unsigned int ppu_poscmp_en : 1;         /**< Position comparison function enable. */
        unsigned int ppu_cmpshd_ld : 1;         /**< Load mode of position comparison buffer register. */
        unsigned int ppu_cmpshd_en : 1;         /**< Position comparison buffer register enable. */
        unsigned int reserved_1 : 1;
        unsigned int pcnt_idx_lock_mode : 2;    /**< Z pulse locked mode selection of position counter. */
        unsigned int pcnt_idx_init_mode : 2;    /**< Z pulse initialization mode of position counter. */
        unsigned int pcnt_rst_mode : 2;         /**< Reset selection of position counter. */
        unsigned int pcnt_mode : 2;             /**< Count mode of position counter. */
        unsigned int pcnt_sw_init : 1;          /**< Software initialization of position counter. */
        unsigned int reserved_2 : 3;
    } BIT;
} volatile QDM_QPPUCTRL_REG;

/**
  * @brief TSU control registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cevt_prescaler : 4;        /**< Frequency division selection of the capture event CEVT. */
        unsigned int tsu_prescaler : 4;         /**< TSU timing step length selection. */
        unsigned int qtmr_lock_mode : 1;        /**< TSU locked mode. */
        unsigned int reserved_0 : 23;
    } BIT;
} volatile QDM_QTSUCTRL_REG;

/**
  * @brief QDM interrupt enable registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pcnt_err_en : 1;         /**< Position count error interrupt enable. */
        unsigned int qphs_err_en : 1;         /**< Quadrature pulse error interrupt enable. */
        unsigned int sped_lst_en : 1;         /**< QDM speed loss interrupt enable. */
        unsigned int qdir_chg_en : 1;         /**< Quadrature direction change interrupt enable. */
        unsigned int pcnt_udf_en : 1;         /**< Position counter underflow interrupt enable. */
        unsigned int pcnt_ovf_en : 1;         /**< Position counter overflow interrupt enable. */
        unsigned int pcnt_cpr_en : 1;         /**< Position comparision ready interrupt enable. */
        unsigned int pcnt_cpm_en : 1;         /**< Position comparision match interrupt enable. */
        unsigned int indx_lck_en : 1;         /**< Z pulse locked fuction interrupt enable. */
        unsigned int utmr_prd_en : 1;         /**< PTU period interrupt enable. */
        unsigned int reserved_0 : 2;
        unsigned int reserved_1 : 20;
    } BIT;
} volatile QDM_QINTENA_REG;

/**
  * @brief QDM interrupt status registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pcnt_err_int : 1;        /**< Position count error interrupt. */
        unsigned int qphs_err_int : 1;        /**< Quadrature pulse error interrupt. */
        unsigned int sped_lst_int : 1;        /**< QDM speed loss interrupt. */
        unsigned int qdir_chg_int : 1;        /**< Quadrature direction change interrupt. */
        unsigned int pcnt_udf_int : 1;        /**< Position counter underflow interrupt. */
        unsigned int pcnt_ovf_int : 1;        /**< Position counter overflow interrupt. */
        unsigned int pcnt_cpr_int : 1;        /**< Position comparision ready interrupt. */
        unsigned int pcnt_cpm_int : 1;        /**< Position comparision match interrupt. */
        unsigned int indx_lck_int : 1;        /**< Z pulse locked fuction interrupt. */
        unsigned int utmr_prd_int : 1;        /**< PTU period interrupt. */
        unsigned int reserved_0 : 2;
        unsigned int reserved_1 : 20;
    } BIT;
} volatile QDM_QINTSTS_REG;

/**
  * @brief QDM initial interrupt registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pcnt_err_raw : 1;        /**< Position count error initial interrupt. */
        unsigned int qphs_err_raw : 1;        /**< Quadrature pulse error initial interrupt. */
        unsigned int sped_lst_raw : 1;        /**< QDM speed loss initial interrupt. */
        unsigned int qdir_chg_raw : 1;        /**< Quadrature direction change initial interrupt. */
        unsigned int pcnt_udf_raw : 1;        /**< Position counter underflow initial interrupt. */
        unsigned int pcnt_ovf_raw : 1;        /**< Position counter overflow initial interrupt. */
        unsigned int pcnt_cpr_raw : 1;        /**< Position comparision ready initial interrupt. */
        unsigned int pcnt_cpm_raw : 1;        /**< Position comparision match initial interrupt. */
        unsigned int indx_lck_raw : 1;        /**< Z pulse locked fuction initial interrupt. */
        unsigned int utmr_prd_raw : 1;        /**< PTU period initial interrupt. */
        unsigned int reserved_0 : 2;
        unsigned int reserved_1 : 20;
    } BIT;
} volatile QDM_QINTRAW_REG;

/**
  * @brief QDM injection interrupt registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pcnt_err_inj : 1;        /**< Position count error injection interrupt. */
        unsigned int qphs_err_inj : 1;        /**< Quadrature pulse error injection interrupt. */
        unsigned int sped_lst_inj : 1;        /**< QDM speed loss injection interrupt. */
        unsigned int qdir_chg_inj : 1;        /**< Quadrature direction change injection interrupt. */
        unsigned int pcnt_udf_inj : 1;        /**< Position counter underflow injection interrupt. */
        unsigned int pcnt_ovf_inj : 1;        /**< Position counter overflow injection interrupt. */
        unsigned int pcnt_cpr_inj : 1;        /**< Position comparision ready injection interrupt. */
        unsigned int pcnt_cpm_inj : 1;        /**< Position comparision match injection interrupt. */
        unsigned int indx_lck_inj : 1;        /**< Z pulse locked fuction injection interrupt. */
        unsigned int utmr_prd_inj : 1;        /**< PTU period injection interrupt. */
        unsigned int reserved_0 : 2;
        unsigned int reserved_1 : 20;
    } BIT;
} volatile QDM_QINTINJ_REG;

/**
  * @brief QDM status registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pcnt_err_sts : 1;        /**< Position count error status. */
        unsigned int fidx_is_sts : 1;         /**< Whether QDM passes the first Z-phase marker. */
        unsigned int fidx_dir_sts : 1;        /**< The direction of QDM firstly passes the Z-phase marker. */
        unsigned int qcdr_err_sts : 1;        /**< QDM capture direction error status. */
        unsigned int qctmr_ovf_sts : 1;       /**< TSU timing count overflow status. */
        unsigned int sepd_lst_sts : 1;        /**< QDM speed loss status. */
        unsigned int qdir_sts : 1;            /**< QDM quadrature direction status. */
        unsigned int cevt_sts : 1;            /**< QDM capture events status. */
        unsigned int reserved_0 : 24;
    } BIT;
} volatile QDM_QDMSTS_REG;

/**
  * @brief QDM A-phase signal filter registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int qdma_ft_level : 13;        /**< The filter level of A-phase signal. */
        unsigned int reserved_0 : 19;
    } BIT;
} volatile QDM_QDMAFT_REG;

/**
  * @brief QDM B-phase signal filter registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int qdmb_ft_level : 13;         /**< The filter level of B-phase signal. */
        unsigned int reserved_0 : 19;
    } BIT;
} volatile QDM_QDMBFT_REG;

/**
  * @brief QDM Z-phase signal filter registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int qdmi_ft_level : 13;         /**< The filter level of Z-phase signal. */
        unsigned int reserved_0 : 19;
    } BIT;
} volatile QDM_QDMIFT_REG;

/**
  * @brief QDM Interrupt callback functions.
  *
  */
typedef struct {
    void (* PtuCycleTrgCallback)(void *handle); /**< PTU triggle interrupt callback */
    void (* SpeedLoseCallback)(void *handle);   /**< speed lose detection callback */
    void (* ZIndexLockedCallBack)(void *handle);              /**< Z index lock interrupt callback.*/
    void (* PositionCompareMatchCallBack)(void *handle);      /**< Position compare match interrupt. */
    void (* PositionCompareReadyCallBack)(void *handle);      /**< Position compare ready interrupt. */
    void (* PositionCounterOverflowCallBack)(void *handle);   /**< Position counter overflow interrupt. */
    void (* PositionCounterUnderflowCallBack)(void *handle);  /**< Position counter underflow interrupt. */
    void (* OrthogonalDirectionChangeCallBack)(void *handle); /**< Orthogonal direction change interrupt. */
    void (* OrthogonalPhaseErrorCallBack)(void *handle);      /**< Orthogonal phase error interrupt. */
    void (* PositionCounterErrorCallBack)(void *handle);      /**< Position counter error interrupt. */
} QDM_UserCallBack;

/**
  * @brief QDM extend handle.
  */
typedef struct _QDM_ExtendeHandle {
} QDM_ExtendHandle;

/**
  * @brief QDM registers definition structure.
  */
typedef struct {
    QDM_QDMVER_REG QDMVER;              /**< QDM version register, offset address: 0x0000. */
    QDM_QEMUMODE_REG QEMUMODE;          /**< QDM emulation mode configuration register, offset address: 0x0004. */
    QDM_QCTRL_REG QCTRL;                /**< QDM control register, offset address: 0x0008. */
    QDM_QPPUCTRL_REG QPPUCTRL;          /**< PPU control register, offset address: 0x000C. */
    QDM_QTSUCTRL_REG QTSUCTRL;          /**< TSU control register, offset address: 0x0010. */
    QDM_QINTENA_REG QINTENA;            /**< QDM interrupt enable register, offset address: 0x0014. */
    QDM_QINTSTS_REG QINTSTS;            /**< QDM interrupt status register, offset address: 0x0018. */
    QDM_QINTRAW_REG QINTRAW;            /**< QDM initial interrupt register, offset address: 0x001C. */
    QDM_QINTINJ_REG QINTINJ;            /**< QDM injection interrupt register, offset address: 0x0020. */
    QDM_QDMSTS_REG QDMSTS;              /**< QDM status register, offset address: 0x0024. */
    unsigned int QPOSCNT;               /**< PPU position counter value, offset address: 0x0028. */
    unsigned int QPOSINIT;              /**< PPU position counter initialization value, offset address: 0x002C. */
    unsigned int QPOSMAX;               /**< PPU position counter maximum value, offset address: 0x0030. */
    unsigned int QPOSCMP;               /**< PPU position counter compare value, offset address: 0x0034. */
    unsigned int QPOSILOCK;             /**< PPU QPOSCNT inde locked value, offset address: 0x0038. */
    unsigned int QPOSLOCK;              /**< PPU QPOSCNT locked value, offset address: 0x003C. */
    unsigned int QUTMR;                 /**< PTU counter value, offset address: 0x0040. */
    unsigned int QUPRD;                 /**< PTU period value, offset address: 0x0044. */
    unsigned int QCTMR;                 /**< TSU counter value, offset address: 0x0048. */
    unsigned int QCMAX;                 /**< TSU counter maximum value, offset address: 0x004C. */
    unsigned int QCPRD;                 /**< TSU-captured CEVT's period, offset address: 0x0050. */
    unsigned int QCTMRLOCK;             /**< QCTMR locked value, offset address: 0x0054. */
    unsigned int QCPRDLOCK;             /**< QCPRD locked value, offset address: 0x0058. */
    QDM_QDMAFT_REG QDMAFT;              /**< QDM A-phase signal filter register, offset address: 0x005C. */
    QDM_QDMBFT_REG QDMBFT;              /**< QDM B-phase signal filter register, offset address: 0x0060. */
    QDM_QDMIFT_REG QDMIFT;              /**< QDM Z-phase signal filter register, offset address: 0x0064. */
    unsigned int QPOSCMPA;              /**< QDM Position Counter Active Compare Value, offset address: 0x0068. */
} volatile QDM_RegStruct;

/**
  * @brief Set the emulation mode of QDM module.
  * @param qdmx QDM register base address.
  * @param emuMode Emulation mode.
  * @retval None.
  */
static inline void DCL_QDM_SetEmulationMode(QDM_RegStruct *qdmx, QDM_EmulationMode emuMode)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(emuMode >= QDM_EMULATION_MODE_STOP_IMMEDIATELY);
    QDM_PARAM_CHECK_NO_RET(emuMode <= QDM_EMULATION_MODE_RUN_FREE);
    qdmx->QEMUMODE.BIT.emu_mode = emuMode;
}

/**
  * @brief Get the working status of QDM module.
  * @param qdmx QDM register base address.
  * @param status Working status flag.
  * @retval unsigned short The flag value.
  */
static inline bool DCL_QDM_GetModuleStatus(const QDM_RegStruct *qdmx, QDM_StatusFlag status)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_ASSERT_PARAM(IsQDMStatusMode(status));
    return ((qdmx->QDMSTS.reg & (unsigned int)status) == status);
}

/**
  * @brief Clear the specific working status of QDM module.
  * @param qdmx QDM register base address.
  * @param status Working status flag.
  * @retval None.
  */
static inline void DCL_QDM_ClearModuleStatus(QDM_RegStruct *qdmx, QDM_StatusFlag status)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_ASSERT_PARAM(IsQDMStatusMode(status));
    qdmx->QDMSTS.reg |= (unsigned int)status;
}

/* Quadrature Decoder Unit --------------------------------------------------------------------- */
/**
  * @brief Set the polarity of QDM module inputs.
  * @param qdmx QDM register base address.
  * @param ivtQDMA QDMA input.
  * @param ivtQDMB QDMB input.
  * @param ivtQDMI QDMI input.
  * @retval None.
  */
static inline void DCL_QDM_SetInputPolarity(QDM_RegStruct *qdmx, bool ivtQDMA, bool ivtQDMB, bool ivtQDMI)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QCTRL.BIT.qdma_polarity = ivtQDMA;
    qdmx->QCTRL.BIT.qdmb_polarity = ivtQDMB;
    qdmx->QCTRL.BIT.qdmi_polarity = ivtQDMI;
}

/**
  * @brief Set the filter width of QDM module inputs.
  * @param qdmx QDM register base address.
  * @param filtWidthQDMA Filter width of QDMA input.
  * @param filtWidthQDMB Filter width of QDMB input.
  * @param filtWidthQDMI Filter width of QDMI input.
  * @retval None.
  */
static inline void DCL_QDM_SetInputFilterWidth(QDM_RegStruct *qdmx,
                                               unsigned short filtWidthQDMA,
                                               unsigned short filtWidthQDMB,
                                               unsigned short filtWidthQDMI)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(filtWidthQDMA <= QDM_MAX_FILTER_LEVEL);
    QDM_PARAM_CHECK_NO_RET(filtWidthQDMB <= QDM_MAX_FILTER_LEVEL);
    QDM_PARAM_CHECK_NO_RET(filtWidthQDMI <= QDM_MAX_FILTER_LEVEL);
    /* Set QDM input filter width. */
    qdmx->QDMAFT.BIT.qdma_ft_level = filtWidthQDMA;
    qdmx->QDMBFT.BIT.qdmb_ft_level = filtWidthQDMB;
    qdmx->QDMIFT.BIT.qdmi_ft_level = filtWidthQDMI;
}

/**
  * @brief Swap the inputs of QDMA and QDMB.
  * @param qdmx QDM register base address.
  * @param swap Swap enable.
  * @retval None.
  */
static inline void DCL_QDM_SetABSwap(QDM_RegStruct *qdmx, bool swap)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QCTRL.BIT.qdm_ab_swap = swap;
}

/**
  * @brief Set the decoder mode of QDM module.
  * @param qdmx QDM register base address.
  * @param decoderMode Decoder mode.
  * @retval None.
  */
static inline void DCL_QDM_SetDecoderMode(QDM_RegStruct *qdmx, QDM_DecoderMode decoderMode)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(IsDecodeMode(decoderMode));
    qdmx->QCTRL.BIT.qdu_mode = decoderMode;
}

/**
  * @brief Set the resolution of decoder.
  * @param qdmx QDM register base address.
  * @param resolution Decoder resolution.
  * @retval None.
  */
static inline void DCL_QDM_SetResolution(QDM_RegStruct *qdmx, QDM_Resolution resolution)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(resolution >= QDM_1X_RESOLUTION);
    QDM_PARAM_CHECK_NO_RET(resolution <= QDM_4X_RESOLUTION);
    qdmx->QCTRL.BIT.qdu_xclk = resolution;
}

/* Position Process Unit ----------------------------------------------------------------------- */
/**
  * @brief Enable Position Process Unit.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_EnablePosProcess(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QCTRL.BIT.ppu_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable Position Process Unit.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_DisablePosProcess(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QCTRL.BIT.ppu_en = BASE_CFG_DISABLE;
}

/**
  * @brief Enable/Disable software initialization of position counter.
  * @param qdmx QDM register base address.
  * @param swInit Software enable.
  * @retval None.
  */
static inline void DCL_QDM_SetSWPosInit(QDM_RegStruct *qdmx, bool swInit)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QPPUCTRL.BIT.pcnt_sw_init = swInit;
}

/**
  * @brief Set the count mode of position counter.
  * @param qdmx QDM register base address.
  * @param cntMode Count mode.
  * @retval None.
  */
static inline void DCL_QDM_SetCountMode(QDM_RegStruct *qdmx, QDM_PPUCountMode cntMode)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(cntMode >= QDM_PPU_COUNT_MODE_CLK_DIR);
    QDM_PARAM_CHECK_NO_RET(cntMode <= QDM_PPU_COUNT_MODE_DECREASE);
    qdmx->QPPUCTRL.BIT.pcnt_mode = cntMode;
}

/**
  * @brief Set the reset mode of position counter.
  * @param qdmx QDM register base address.
  * @param rstMode Reset mode.
  * @retval None.
  */
static inline void DCL_QDM_SetPosResetMode(QDM_RegStruct *qdmx, QDM_PosResetMode rstMode)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(rstMode >= QDM_POSITION_RESET_IDX);
    QDM_PARAM_CHECK_NO_RET(rstMode <= QDM_POSITION_RESET_UNIT_TIME_OUT);
    qdmx->QPPUCTRL.BIT.pcnt_rst_mode = rstMode;
}

/**
  * @brief Set the initialization mode of position counter.
  * @param qdmx QDM register base address.
  * @param initMode Initialization mode.
  * @retval None.
  */
static inline void DCL_QDM_SetPosInitMode(QDM_RegStruct *qdmx, QDM_PosIdxInitMode initMode)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(initMode >= QDM_POSITION_INIT_DO_NOTHING);
    QDM_PARAM_CHECK_NO_RET(initMode <= QDM_POSITION_INIT_FALLING_INDEX);
    qdmx->QPPUCTRL.BIT.pcnt_idx_init_mode = initMode;
}

/**
  * @brief Set the index lock mode.
  * @param qdmx QDM register base address.
  * @param lockMode Lock mode of index.
  * @retval None.
  */
static inline void DCL_QDM_SetIndexLockMode(QDM_RegStruct *qdmx, QDM_IndexLockMode lockMode)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(lockMode >= QDM_LOCK_RESERVE);
    QDM_PARAM_CHECK_NO_RET(lockMode <= QDM_LOCK_SW_INDEX_MARKER);
    qdmx->QPPUCTRL.BIT.pcnt_idx_lock_mode = lockMode;
}

/**
  * @brief Set the initial value of position counter.
  * @param qdmx QDM register base address.
  * @param position Initial value.
  * @retval None.
  */
static inline void DCL_QDM_SetInitialPos(QDM_RegStruct *qdmx, unsigned int position)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QPOSINIT = position;
}

/**
  * @brief Set the max value of position counter.
  * @param qdmx QDM register base address.
  * @param maxPos Max value.
  * @retval None.
  */
static inline void DCL_QDM_SetMaxPos(QDM_RegStruct *qdmx, unsigned int maxPos)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QPOSMAX = maxPos;
}

/**
  * @brief Get the current value of position counter.
  * @param qdmx QDM register base address.
  * @retval unsigned int Value of position counter.
  * @retval None.
  */
static inline unsigned int DCL_QDM_GetCurPos(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return (qdmx->QPOSCNT);
}

/**
  * @brief Enable position compare.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_EnablePosComp(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QPPUCTRL.BIT.ppu_poscmp_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable position compare.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_DisablePosComp(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QPPUCTRL.BIT.ppu_poscmp_en = BASE_CFG_DISABLE;
}

/**
  * @brief Set the position compare value.
  * @param qdmx QDM register base address.
  * @param compVal Compare value.
  * @retval None.
  */
static inline void DCL_QDM_SetPosCompVal(QDM_RegStruct *qdmx, unsigned int compVal)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QPOSCMP = compVal;
}

/**
  * @brief Enable position compare sync-out pulse.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_EnableCompSyncOut(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QPPUCTRL.BIT.syncout_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable position compare sync-out pulse.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_DisableCompSyncOut(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QPPUCTRL.BIT.syncout_en = BASE_CFG_DISABLE;
}

/**
  * @brief Set the pulse width of position compare sync-out pulse.
  * @param qdmx QDM register base address.
  * @param width Pulse width.
  * @retval None.
  */
static inline void DCL_QDM_SetCompSyncOutWidth(QDM_RegStruct *qdmx, unsigned short width)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(width <= QDM_PPU_MAX_SYNCOUT_PW);
    /* In units of 4 PCLK cycles */
    qdmx->QPPUCTRL.BIT.ppu_syncout_pw = width;
}

/**
  * @brief Set the polarity of position compare sync-out pulse.
  * @param qdmx QDM register base address.
  * @param polarity Sync-out pulse polarity.
  * @retval None.
  */
static inline void DCL_QDM_SetCompSyncOutPolarity(QDM_RegStruct *qdmx, QDM_CompSyncOutPolarity polarity)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(polarity == QDM_SYNC_OUT_HIGH || polarity == QDM_SYNC_OUT_LOW);
    qdmx->QPPUCTRL.BIT.ppu_syncout_pl = polarity;
}

/**
  * @brief Enable shadow mode of position compare.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_EnablePosCompShadow(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QPPUCTRL.BIT.ppu_cmpshd_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable shadow mode of position compare.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_DisablePosCompShadow(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QPPUCTRL.BIT.ppu_cmpshd_en = BASE_CFG_DISABLE;
}

/**
  * @brief Set the shadow load mode of position compare.
  * @param qdmx QDM register base address.
  * @param shadowMode Shadow load mode.
  * @retval None.
  */
static inline void DCL_QDM_SetCompShadowMode(QDM_RegStruct *qdmx, QDM_CompShadowLoad shadowMode)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(shadowMode == QDM_COMPARE_LOAD_ON_ZERO || shadowMode == QDM_COMPARE_LOAD_ON_MATCH);
    qdmx->QPPUCTRL.BIT.ppu_cmpshd_ld = shadowMode;
}

/**
  * @brief Get the position index lock value.
  * @param qdmx QDM register base address.
  * @retval unsigned int Index lock value.
  */
static inline unsigned int DCL_QDM_GetPosIndexLock(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return (qdmx->QPOSILOCK);
}

/**
  * @brief Get the unit time position lock value.
  * @param qdmx QDM register base address.
  * @retval unsigned int Unit time position lock value.
  */
static inline unsigned int DCL_QDM_GetPosUnitTimeLock(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return (qdmx->QPOSLOCK);
}

/* Time Stamp Unit ----------------------------------------------------------------------------- */
/**
  * @brief Enable Time Stamp Unit capture.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_EnableTSUCap(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    /* EQEP_enableCapture */
    qdmx->QCTRL.BIT.tsu_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable Time Stamp Unit capture.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_DisableTSUCap(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    /* EQEP_disableCapture */
    qdmx->QCTRL.BIT.tsu_en = BASE_CFG_DISABLE;
}

/**
  * @brief Configure Time Stamp Unit capture.
  * @param qdmx QDM register base address.
  * @param tscPrsc Clock prescaler.
  * @param evtPrsc Unit position event prescaler.
  * @param tsuLock Time Stamp Unit lock mode.
  * @retval None.
  */
static inline void DCL_QDM_ConfigTSUCap(QDM_RegStruct *qdmx,
                                        QDM_TSUCLKPrescale tscPrsc,
                                        QDM_UPEvntPrescale evtPrsc,
                                        QDM_TSULockMode tsuLock)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    /* Check whether function parameters the  is right. */
    QDM_PARAM_CHECK_NO_RET(tscPrsc >= QDM_TSU_CLK_DIV_1);
    QDM_PARAM_CHECK_NO_RET(tscPrsc <= QDM_TSU_CLK_DIV_256);
    QDM_PARAM_CHECK_NO_RET(evtPrsc >= QDM_UNIT_POS_EVNT_DIV_1);
    QDM_PARAM_CHECK_NO_RET(evtPrsc <= QDM_UNIT_POS_EVNT_DIV_2048);
    QDM_PARAM_CHECK_NO_RET(tsuLock == QDM_TSU_LOCK_ON_SW_READ || tsuLock == QDM_TSU_LOCK_ON_UTTRG);
    qdmx->QTSUCTRL.BIT.tsu_prescaler = tscPrsc;
    qdmx->QTSUCTRL.BIT.cevt_prescaler = evtPrsc;
    qdmx->QTSUCTRL.BIT.qtmr_lock_mode = tsuLock;
}

/**
  * @brief Get the capture timer value.
  * @param qdmx QDM register base address.
  * @retval unsigned int The capture timer value.
  */
static inline unsigned int DCL_QDM_GetCapTimer(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return (qdmx->QCTMR);
}

/**
  * @brief Set the max value of capture timer.
  * @param qdmx QDM register base address.
  * @param maxCount Max value.
  * @retval None.
  */
static inline void DCL_QDM_SetCapMaxCnt(QDM_RegStruct *qdmx, unsigned int maxCount)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QCMAX = maxCount;
}

/**
  * @brief Get the period of capture timer.
  * @param qdmx QDM register base address.
  * @retval unsigned int Period of capture timer.
  */
static inline unsigned int DCL_QDM_GetCapPeriod(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return (qdmx->QCPRD);
}

/**
  * @brief Get the lock value of capture timer.
  * @param qdmx QDM register base address.
  * @retval unsigned int Lock value.
  */
static inline unsigned int DCL_QDM_GetCapTimerLock(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return (qdmx->QCTMRLOCK);
}

/**
  * @brief Get the period value of capture timer.
  * @param qdmx QDM register base address.
  * @retval unsigned int Period value of capture timer.
  */
static inline unsigned int DCL_QDM_GetCapPeriodLock(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return (qdmx->QCPRDLOCK);
}

/* Period Trigger Unit ------------------------------------------------------------------------- */
/**
  * @brief Enable Period Trigger Unit.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_EnablePeriodTrigger(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QCTRL.BIT.ptu_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable Period Trigger Unit.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_DisablePeriodTrigger(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QCTRL.BIT.ptu_en = BASE_CFG_DISABLE;
}

/**
  * @brief Set the working mode of Period Trigger Unit.
  * @param qdmx QDM register base address.
  * @param ptuMode Working mode of Period Trigger Unit.
  * @retval None.
  */
static inline void DCL_QDM_SetPeriodTriggerUnitMode(QDM_RegStruct *qdmx, QDM_PTUMode ptuMode)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(ptuMode == QDM_PERIOD_TRIGGER_MODE || ptuMode == QDM_WATCHDOG_MODE);
    qdmx->QCTRL.BIT.ptu_mode = ptuMode;
}

/**
  * @brief Set the trigger lock mode.
  * @param qdmx QDM register base address.
  * @param lockMode Trigger lock mode.
  * @retval None.
  */
static inline void DCL_QDM_SetTriggerLockMode(QDM_RegStruct *qdmx, QDM_TriggerLockMode lockMode)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    QDM_PARAM_CHECK_NO_RET(lockMode == QDM_LOCK_POSCNT_READ_BY_CPU || lockMode == QDM_LOCK_UNIT_TIME_TRIGGER);
    qdmx->QCTRL.BIT.qtrg_lock_mode = lockMode;
}

/**
  * @brief Set the period of unit time event.
  * @param qdmx QDM register base address.
  * @param period Period of unit time event.
  * @retval None.
  */
static inline void DCL_QDM_SetTriggerPeriod(QDM_RegStruct *qdmx, unsigned int period)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QUPRD = period;
}

/**
  * @brief Get the value of period counter.
  * @param qdmx QDM register base address.
  * @retval unsigned int Value of period counter.
  */
static inline unsigned int DCL_QDM_GetPeriodCounter(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return (qdmx->QUTMR);
}

/* Interrupt Generator ------------------------------------------------------------------------- */
/**
  * @brief Enable specific interrupt.
  * @param qdmx QDM register base address.
  * @param intEvt Interrupt event.
  * @retval None.
  */
static inline void DCL_QDM_EnableInterrupt(QDM_RegStruct *qdmx, QDM_InterruptEvent intEvt)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    /* Check whether the QDM interrupt type is right. */
    QDM_PARAM_CHECK_NO_RET(IsQDMInterruptEvent(intEvt));
    qdmx->QINTENA.reg |= (unsigned int)intEvt;
}

/**
  * @brief Disable specific interrupt.
  * @param qdmx QDM register base address.
  * @param intEvt Interrupt event.
  * @retval None.
  */
static inline void DCL_QDM_DisableInterrupt(QDM_RegStruct *qdmx, QDM_InterruptEvent intEvt)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    /* Check whether the QDM interrupt type is right. */
    QDM_PARAM_CHECK_NO_RET(IsQDMInterruptEvent(intEvt));
    qdmx->QINTENA.reg &= (~(unsigned int)intEvt);
}

/**
  * @brief Get the specific interrupt flag.
  * @param qdmx QDM register base address.
  * @param intEvt Interrupt event.
  * @retval bool true, false.
  */
static inline bool DCL_QDM_GetInterruptFlag(QDM_RegStruct *qdmx, QDM_InterruptEvent intEvt)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    /* Check whether the QDM interrupt type is right. */
    QDM_ASSERT_PARAM(IsQDMInterruptEvent(intEvt));
    return ((qdmx->QINTSTS.reg & (unsigned int)intEvt) == intEvt);
}

/**
  * @brief Clear the specific interrupt flag.
  * @param qdmx QDM register base address.
  * @param intEvt Interrupt event.
  * @retval None.
  */
static inline void DCL_QDM_ClearInterrupt(QDM_RegStruct *qdmx, QDM_InterruptEvent intEvt)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    /* Check whether the QDM interrupt type is right. */
    QDM_PARAM_CHECK_NO_RET(IsQDMInterruptEvent(intEvt));
    qdmx->QINTRAW.reg |= (unsigned int)intEvt;
}

/**
  * @brief Force a specific interrupt.
  * @param qdmx QDM register base address.
  * @param intEvt Interrupt event.
  * @retval None.
  */
static inline void DCL_QDM_ForceInterrupt(QDM_RegStruct *qdmx, QDM_InterruptEvent intEvt)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    /* Check whether the QDM interrupt type is right. */
    QDM_PARAM_CHECK_NO_RET(IsQDMInterruptEvent(intEvt));
    qdmx->QINTINJ.reg |= (unsigned int)intEvt;
}

/**
  * @brief Enable speed lost raw interrupt.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_EnableSpedLstRaw(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QINTRAW.BIT.sped_lst_raw = BASE_CFG_ENABLE;
}

/**
  * @brief Enable PTU period raw interrupt.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_EnableUtmrPrdRaw(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QINTRAW.BIT.utmr_prd_raw = BASE_CFG_ENABLE;
}

/**
  * @brief Set the event status.
  * @param qdmx QDM register base address.
  * @retval None.
  */
static inline void DCL_QDM_SetCevtSts(QDM_RegStruct *qdmx, unsigned int status)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    qdmx->QDMSTS.BIT.cevt_sts = status;
}

/**
  * @brief Get the event status.
  * @param qdmx QDM register base address.
  * @retval unsigned int Value of cevt_sts.
  */
static inline unsigned int DCL_QDM_GetCevtSts(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return qdmx->QDMSTS.BIT.cevt_sts;
}

/**
  * @brief Get TSU overflow status.
  * @param qdmx QDM register base address.
  * @retval unsigned int Value of qctmr_ovf_sts.
  */
static inline unsigned int DCL_QDM_GetQctmrOvfSts(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return qdmx->QDMSTS.BIT.qctmr_ovf_sts;
}

/**
  * @brief Get quadrature direction status.
  * @param qdmx QDM register base address.
  * @retval unsigned int Value of qdir_sts.
  */
static inline unsigned int DCL_QDM_GetQdirSts(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return qdmx->QDMSTS.BIT.qdir_sts;
}

/**
  * @brief Get the direction error status.
  * @param qdmx QDM register base address.
  * @retval unsigned int Value of qcdr_err_sts.
  */
static inline unsigned int DCL_QDM_GetQcdrErrSts(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return qdmx->QDMSTS.BIT.qcdr_err_sts;
}

/**
  * @brief Get the position counter active compare value.
  * @param qdmx QDM register base address.
  * @retval unsigned int Value of qposcmpa.
  */
static inline unsigned int DCL_QDM_GetPositionCompareValue(QDM_RegStruct *qdmx)
{
    QDM_ASSERT_PARAM(IsQDMInstance(qdmx));
    return qdmx->QPOSCMPA;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_QDM_IP_H */
