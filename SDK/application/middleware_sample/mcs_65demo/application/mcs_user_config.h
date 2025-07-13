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
  * @file      mcs_user_config.h
  * @author    MCU Algorithm Team
  * @brief     This file provides config macros for 65demo app.
  */
#ifndef McuMagicTag_MCS_USER_CONFIG_H
#define McuMagicTag_MCS_USER_CONFIG_H

#include "debug.h"
#include "typedefs.h"
#include "crg.h"
#include "mcs_math_const.h"

/* Use sync-out pulse from APT_U as the sync-in source for slave APT module. */
#define APT_U_CP          APT3_BASE /* Base address of U phase APT module */
#define APT_V_CP          APT4_BASE /* Base address of V phase APT module */
#define APT_W_CP          APT5_BASE /* Base address of W phase APT module */

#define APT_U_FAN         APT0_BASE /* Base address of U phase APT module */
#define APT_V_FAN         APT1_BASE /* Base address of V phase APT module */
#define APT_W_FAN         APT2_BASE /* Base address of W phase APT module */
/* Some configuration values of APT modules. */
#define APT_PWM_FREQ        10000       /* PWM frequency 10K Hz. */
#define APT_DUTY_MAX        (HAL_CRG_GetIpFreq(APT_U_CP) / (APT_PWM_FREQ * 2))

#define INV_CAP_CHARGE_MS    3u
#define INV_VOLTAGE_BUS      24.0f   /* Bus voltage, V */

#define CTRL_CURR_PERIOD     (1.0f / APT_PWM_FREQ) /* carrier ISR period (S) */
#define CTRL_SYSTICK_PERIOD  0.0005f /* systick control period (S) */

#define SYSTICK_PERIOD_US    (CTRL_SYSTICK_PERIOD / 0.000001f) /* systick period (us) */
#define MICROSECOND_NUM_PER_MILLISECOND 1000 /* Conversion between milliseconds and microseconds */

/* Np, Rs, Ld, Lq, Psif, J, Nmax, Currmax, */
#define MOTOR_PARAM_NP               (4)          /* Pole pairs. */
#define MOTOR_PARAM_RS               (0.45)       /* Phase resistance (Ohm). */
#define MOTOR_PARAM_LD               (0.000294f)  /* D-axis inductance (H). */
#define MOTOR_PARAM_LQ               (0.000294f)  /* Q-axis inductance (H). */
#define MOTOR_PARAM_LS               (0.000294f)  /* Average inductance, (H). */
#define MOTOR_PARAM_PSIF             (0.0)        /* Permanent magnet flux amplitude (Wb). */
#define MOTOR_PARAM_JS               (0.0f)       /* Motion of inertia (kg.Nm). */
#define MOTOR_PARAM_MAX_SPD          (200.0f)     /* Maximum electical speed (Hz). */
#define MOTOR_PARAM_MAX_CURR         (2.0f)       /* Maximum phase current (A). */
#define MOTOR_PARAM_MAX_TRQ          (1.0f)       /* Maximum phase current (A). */
#define MOTOR_PARAM_ENCODER_PPMR     (65535)      /* Pulse per mechanical round. */
#define MOTOR_PARAM_ENCODER_ZSHIFT   (1)          /* Pulse Z shift. */

/* ----------------------------PI parameters ----------------------------------- */
#define CURR_KP            (0.07414f)
#define CURR_KI            (1200.0f)
#define CURR_LOWERLIM      (-INV_VOLTAGE_BUS * ONE_DIV_SQRT3 * 0.95f)
#define CURR_UPPERLIM      (INV_VOLTAGE_BUS * ONE_DIV_SQRT3 * 0.95f)

#define SPD_KP            (0.05f)
#define SPD_KI            (0.5f)
#define SPD_LOWERLIM      (-MOTOR_PARAM_MAX_CURR)
#define SPD_UPPERLIM      (MOTOR_PARAM_MAX_CURR)

/** Duty of sample window, the real time is 0.06*50us = 3us. */
#define SAMPLE_WINDOW_DUTY 0.06f

/** Duty of sample point shift as flip point, the real time is 0.008*50us = 0.4us. */
#define SAMPLE_POINT_SHIFT 0.008f

/* SMO */
#define FOSMO_GAIN                 (8.0f)
#define FOSMO_LAMBDA               (2.0f)    /* SMO coefficient of cut-off frequency, its value = lambda * we */
#define FOSMO_EMF_CUTOFF_FREQ      (2.0f)    /* SMO back emf cutoff frequency. */
#define FOSMO_SPD_CUTOFF_FREQ      (30.0f)   /* SMO speed cutoff frequency. of speed filter. */
#define FOSMO_PLL_BDW              (40.0f)

/* User Command */
#define CTRL_IF_CURR_AMP_A          1.0f      /* IF control current amplitude */
/* 0.008055 Sampling resistance20 Ohm  Sampling resistance 100mOhm 0.001611f */
#define ADC_CURR_COFFI_CP           0.001611f
#define USER_TARGET_SPD_HZ          (100.0f) /* Parentheses are used to enter negative instructions */
#define USER_SWITCH_SPDBEGIN_HZ     51.0f    /* Start of handover interval */
#define USER_SWITCH_SPDEND_HZ       52.0f    /* End of handover period */

#define USER_SPD_SLOPE  15.0f                       /* slope of velocity change */
#define USER_CURR_SLOPE (CTRL_IF_CURR_AMP_A * 0.5f) /* Current change slope  */

/* FAN ----------------------------------------------------------------------------------------- */
/* Np, Rs, Ld, Lq, Psif, J, Nmax, Currmax, */
#define MOTOR_PARAM_NP_FAN               (4)          /* Pole pairs. */
#define MOTOR_PARAM_RS_FAN               (0.45)       /* Phase resistance (Ohm). */
#define MOTOR_PARAM_LD_FAN               (0.000294f)  /* D-axis inductance (H). */
#define MOTOR_PARAM_LQ_FAN               (0.000294f)  /* Q-axis inductance (H). */
#define MOTOR_PARAM_LS_FAN               (0.000294f)  /* Average inductance, (H). */
#define MOTOR_PARAM_PSIF_FAN             (0.0)        /* Permanent magnet flux amplitude (Wb). */
#define MOTOR_PARAM_JS_FAN               (0.0f)       /* Motion of inertia (kg.Nm). */
#define MOTOR_PARAM_MAX_SPD_FAN          (200.0f)     /* Maximum electical speed (Hz). */
#define MOTOR_PARAM_MAX_CURR_FAN         (2.0f)       /* Maximum phase current (A). */
#define MOTOR_PARAM_MAX_TRQ_FAN          (1.0f)       /* Maximum phase current (A). */
#define MOTOR_PARAM_ENCODER_PPMR_FAN     (65535)      /* Pulse per mechanical round. */
#define MOTOR_PARAM_ENCODER_ZSHIFT_FAN   (1)          /* Pulse Z shift. */

/* ----------------------------PI parameters ----------------------------------- */
#define CURR_KP_FAN            (0.07414f)
#define CURR_KI_FAN            (1200.0f)
#define CURR_LOWERLIM_FAN      (-INV_VOLTAGE_BUS * ONE_DIV_SQRT3 * 0.92f)
#define CURR_UPPERLIM_FAN      (INV_VOLTAGE_BUS * ONE_DIV_SQRT3 * 0.92f)

#define SPD_KP_FAN             (0.05f)
#define SPD_KI_FAN             (0.5f)
#define SPD_LOWERLIM_FAN       (-MOTOR_PARAM_MAX_CURR)
#define SPD_UPPERLIM_FAN       (MOTOR_PARAM_MAX_CURR)

/* SMO */
#define FOSMO_GAIN_FAN             (8.0f)
#define FOSMO_LAMBDA_FAN           (2.0f)    /* SMO coefficient of cut-off frequency, its value = lambda * we */
#define FOSMO_EMF_CUTOFF_FREQ_FAN  (2.0f)    /* SMO back emf cutoff frequency. */
#define FOSMO_SPD_CUTOFF_FREQ_FAN  (30.0f)   /* SMO speed cutoff frequency. of speed filter. */
#define FOSMO_PLL_BDW_FAN          (40.0f)

#define CTRL_IF_CURR_AMP_A_FAN      0.8f     /* IF control current amplitude */

/* 0.008055 Sampling resistance20 Ohm  Sampling resistance 100mOhm 0.001611f */
#define ADC_CURR_COFFI_FAN          0.001611f

#define USER_TARGET_SPD_HZ_FAN      (100.0f)  /* Parentheses are used to enter negative instructions */
#define USER_SWITCH_SPDBEGIN_HZ_FAN  50.0f   /* Start of handover interval, Positive number only */
#define USER_SWITCH_SPDEND_HZ_FAN    50.2f   /* End of handover period, Positive number only */

#define USER_SPD_SLOPE_FAN  15.0f                      /* slope of velocity change  */
#define USER_CURR_SLOPE_FAN CTRL_IF_CURR_AMP_A_FAN     /* Current change slope */

/* Service Definition */
#define PTC_RELAY_DELAY 2
#define MOTOR_START_DELAY 5
#define IPM_90DEGRESS_V 2.9f /* IPM voltage at 90°C */
#define IPM_VOLT_COEFFI 0.0008056f /* 3.3 / 4096 */

/* ---------------------------- protection detection parameters ----------------------------------- */
#define OTD_TEMP_THR  (2.9f) /**< Temperature threshold. IPM voltage at 90°C */
#define OTD_TIME_THR  (1.0f) /**< Over Temperature time threshold. */


#endif