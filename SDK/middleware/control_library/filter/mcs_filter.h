/**
  * @ Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2023. All rights reserved.
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
  * @file      mcs_filter.h
  * @author    MCU Algorithm Team
  * @brief     filter library.
  *            This file provides functions declaration of the filter module.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_FILTER_H
#define McuMagicTag_MCS_FILTER_H

/* Typedef definitions ------------------------------------------------------------------------- */

/**
  * @brief 1st-order Filter struct members and parameters.
  * LPF(low-pass filter):  y(k)=a1*y(k-1)+b1*u(k)
  * HPF(high-pass filter): y(k)=a1*y(k-1)+b1*u(k)+b2*u(k-1)
  */
typedef struct {
    float yLast;      /**< Last output of 1st-order filter. */
    float uLast;      /**< Last input variable. */
    float fc;         /**< 1st-order filter cut-off frequency (Hz). */
    float ts;         /**< 1st-order filter running period. */
    float a1;         /**< Coefficient of 1st-order filter. */
    float b1;         /**< Coefficient of 1st-order filter. */
    float b2;         /**< Coefficient of 1st-order filter. */
} FOFLT_Handle;


/**
 * @defgroup FILTER_API FILTER API
 * @brief Filter function API declaration.
 * Transfer Func: G(s) = kw/(s+w), k = 1.
 */
void FOLPF_Init(FOFLT_Handle *lpfHandle, float ts, float fc);
void FOLPF_Clear(FOFLT_Handle *lpfHandle);
float FOLPF_Exec(FOFLT_Handle *lpfHandle, float u);
void FOLPF_SetTs(FOFLT_Handle *lpfHandle, float ts);


/**
  * @}
  */

#endif
