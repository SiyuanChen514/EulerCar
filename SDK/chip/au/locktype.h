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
  * @file      locktype.h
  * @author    MCU Driver Team
  * @brief     This file lists all types that need to be locked on the chip.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_LOCKTYPE_H
#define McuMagicTag_LOCKTYPE_H

/* Typedef definitions ------------------------------------------------------- */
/**
  * @brief This enum defines all hardware locks integrated by this MCU.
  */
typedef enum {
    CHIP_LOCK_GPIO0 = 0,
    CHIP_LOCK_GPIO1 = 1,
    CHIP_LOCK_GPIO2 = 2,
    CHIP_LOCK_GPIO3 = 3,
    CHIP_LOCK_GPIO4 = 4,
    CHIP_LOCK_GPIO5 = 5,
    CHIP_LOCK_GPIO6 = 6,
    CHIP_LOCK_GPIO7 = 7,
    CHIP_LOCK_TOTAL
} CHIP_LockType;

#endif /* McuMagicTag_LOCKTYPE_H */