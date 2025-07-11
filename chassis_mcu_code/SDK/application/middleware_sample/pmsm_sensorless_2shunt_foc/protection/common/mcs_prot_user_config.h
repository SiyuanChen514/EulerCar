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
  * @file      mcs_prot_user_config.h
  * @author    MCU Algorithm Team
  * @brief     This file contans user macro definition of the protection function.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_PROT_USER_CONFIG_H
#define McuMagicTag_MCS_PROT_USER_CONFIG_H

/* Macro definitions --------------------------------------------------------------------------- */
/**< COMMON */
/**< Peak phase current (A) of the motor or IPM under continuous operations. */
#define PROT_MOTOR_RATED_CURR           (0.07f)
/**< Only several continuous fault detection can trigger error status. */
#define PROT_CNT_LIMIT                  (100)
/**< Only several contunuous none fault dectection can trigger elimination of error status. */
#define RECY_CNT_LIMIT                  (10000)
/**< Only several contunuous none fault dectection can trigger elimination of error status. */
#define OVER_VOLT_RECY_CNT_LIMIT        (100)
/**< Only several contunuous none fault dectection can trigger elimination of error status. */
#define LOWER_VOLT_RECY_CNT_LIMIT       (100)

/**< Over current protection */
/**< Over current trigger value (A) when in level 1. */
#define PROT_OVER_CURR_POW_DN1          (2.0f * PROT_MOTOR_RATED_CURR)
/**< Over current trigger value (A) when in level 2. */
#define PROT_OVER_CURR_POW_DN2          (2.0f * PROT_MOTOR_RATED_CURR)
/**< Over current trigger value (A) when in level 3. */
#define PROT_OVER_CURR_POW_DN3          (2.0f * PROT_MOTOR_RATED_CURR)
/**< Over current trigger value (A) when in level 4. */
#define PROT_OVER_CURR_POW_OFF          (2.0f * PROT_MOTOR_RATED_CURR)
#define PROT_OVER_CURR_RECY_DELTA       (0.0035f)     /**< Current gap (A) when recovers from protection status. */
#define PROT_OVER_CURR_LIMIT1_TIME_SEC  (30.0f)    /**< 20% overload can last maximum time: 30 sec. */
#define PROT_OVER_CURR_LIMIT2_TIME_SEC  (10.0f)    /**< 20% overload can last maximum time: 10 sec. */
#define PROT_OVER_CURR_LIMIT3_TIME_SEC  (2.0f)     /**< 20% overload can last maximum time: 2 sec. */

/**< Over voltage protection */
#define PROT_OVER_VOLT_BRK_ON1          (15.0f)   /**< Over dc-link voltage trigger value (V) when in level 1. */
#define PROT_OVER_VOLT_BRK_ON2          (16.0f)   /**< Over dc-link voltage trigger value (V) when in level 2. */
#define PROT_OVER_VOLT_BRK_ON3          (17.0f)   /**< Over dc-link voltage trigger value (V) when in level 3. */
#define PROT_OVER_VOLT_BRK_ALL          (18.0f)   /**< Over dc-link voltage trigger value (V) when in level 4. */
#define PROT_OVER_VOLT_RECY_DELTA       (0.5f)     /**< Voltage gap (V) when recovers from protection status. */
#define PROT_OVER_VOLT_LIMIT1_TIME_SEC  (5.0f)     /**< overload1 can last maximum time (sec). */
#define PROT_OVER_VOLT_LIMIT2_TIME_SEC  (3.0f)     /**< overload2 can last maximum time (sec). */
#define PROT_OVER_VOLT_LIMIT3_TIME_SEC  (1.0f)     /**< overload3 can last maximum time (sec). */
#define PROT_POW_DN1_PCT                (1.0f)     /* Power down level in level 1. */
#define PROT_POW_DN2_PCT                (1.0f)     /* Power down level in level 2. */
#define PROT_POW_DN3_PCT                (1.0f)     /* Power down level in level 3. */

/**< Conduction duty needs to be calibrated with the power of the brake loop. */
#define PROT_OVER_VOLT_BRK_DUTY1        (0.25f)    /**< Conduction duty of the brake loop in level 1. */
#define PROT_OVER_VOLT_BRK_DUTY2        (0.50f)    /**< Conduction duty of the brake loop in level 2. */
#define PROT_OVER_VOLT_BRK_DUTY3        (0.75f)    /**< Conduction duty of the brake loop in level 3. */
#define PROT_OVER_VOLT_BRK_DUTY4        (1.00f)    /**< Conduction duty of the brake loop in level 4. */

/**< Lower voltage protection */
#define PROT_LOWER_VOLT_POW_DN1         (10.3f)  /**< Lower dc-link voltage trigger value (V) when in level 1. */
#define PROT_LOWER_VOLT_POW_DN2         (10.0f)  /**< Lower dc-link voltage trigger value (V) when in level 2. */
#define PROT_LOWER_VOLT_POW_DN3         (9.0f)   /**< Lower dc-link voltage trigger value (V) when in level 3. */
#define PROT_LOWER_VOLT_POW_OFF         (8.0f)   /**< Lower dc-link voltage trigger value (V) when in level 4. */
#define PROT_LOWER_VOLT_RECY_DELTA      (0.5f)  /**< Voltage gap (A) when recovers from protection status. */
#define PROT_LOWER_VOLT_LIMIT1_TIME_SEC (3.0f)   /**< 20% overload can last maximum time: 3 sec. */
#define PROT_LOWER_VOLT_LIMIT2_TIME_SEC (3.0f)   /**< 20% overload can last maximum time: 3 sec. */
#define PROT_LOWER_VOLT_LIMIT3_TIME_SEC (3.0f)   /**< 20% overload can last maximum time: 3 sec. */

/**< Over IPM temperature protection */
#define PROT_OVER_IPM_TEMP_POW_DN1      (40.0f)  /**< Over IPM temperature trigger value (celsius) when in level 1. */
#define PROT_OVER_IPM_TEMP_POW_DN2      (42.0f)  /**< Over IPM temperature trigger value (celsius) when in level 2. */
#define PROT_OVER_IPM_TEMP_POW_DN3      (44.0f)  /**< Over IPM temperature trigger value (celsius) when in level 3. */
#define PROT_OVER_IPM_TEMP_POW_OFF      (45.0f)  /**< Over IPM temperature trigger value (celsius) when in level 4. */
#define PROT_OVER_IPM_TEMP_RECY_DELTA   (0.5f) /**< Temperature gap (celsius) when recovers from protection status. */
#define PROT_OVER_TEMP_LIMIT1_TIME_SEC  (10.0f)  /**< 20% overload can last maximum time: 10 sec. */
#define PROT_OVER_TEMP_LIMIT2_TIME_SEC  (10.0f)  /**< 20% overload can last maximum time: 10 sec. */
#define PROT_OVER_TEMP_LIMIT3_TIME_SEC  (10.0f)  /**< 20% overload can last maximum time: 10 sec. */

/**< Motor stalling detection */
/**< Feedback current higher than this value triggers fault. (A). */
#define PROT_STALLING_CURR_AMP_LIMIT  (PROT_MOTOR_RATED_CURR * 1.2f)
/**< Feedback speed lower than this value triggers fault (Hz). */
#define PROT_STALLING_SPD_LIMIT       30
/**< The threshold time that current and speed feedback over ranges (s). */
#define PROT_STALLING_TIME_LIMIT      (1.5f)

/**< Current out-of-balance detection */
#define UNBAL_STARTUP_DETECT_TIME_SEC (0.5f)   /**< Start detection delay (s) */
#define UNBAL_PROT_CNT_LIMIT          (50000)
#define UNBAL_RECY_CNT_LIMIT          (50000)
#define UNBAL_CURRENT_DELTA           (1.5f)   /**< Used to detect zero crossings in the current cycle. */
#define UNBAL_DEGREE_LIMIT            (0.035f) /**< unbalance degree threshold. */
#define UNBAL_DEGREE_AVG_FLT_COFFI    (0.03f)  /**< unbalance degree average Filter coefficient. */

/**< Position sensor detection */
#define POS_SNSR_FAULT_CNT          (100000)   /* Number of consecutive fault times */
#define POS_SNSR_RECY_CNT           (10000)   /* Number of consecutive communication loss times */

/**< Phase winding integrity detection */
#define OPEN_PHS_CURR_THR_A (0.1f) /* Threshold to determine open phase no current (A). */

/**< Position sensor zero position detection */
#define POS_SNSR_CALIBR_UD_REF      (15.0f)  /* V */
#define POS_SNSR_CALIBR_UD_SLOPE    (15.0f)  /* (V/S) */
#define POS_SNSR_CALIBR_DETECT_TIME (2.0f)   /* S */
#define POS_SNSR_RECORD_TIMES       (2000)   /* 2000 * TS */

/* Multi-cycle mode: > 1; One-cycle mode: < 0.5 */
#define POS_SNSR_IPD_INJ_PERIOD     (4)

#endif