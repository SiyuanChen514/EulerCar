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
  * @file    config.h
  * @author  MCU Driver Team
  * @brief   Header file containing functions prototypes of config module.
  *          + Initialization and de-initialization functions
  *          + Format config function
  */
#ifndef CONFIG_H
#define CONFIG_H


#include "module.h"
#include "type.h"
#include "typedefs.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */


#define DATA_ITEM_MAX_LEN       256 /* maximum length of data items */

enum DataItem {
    DATA_ITEM_EVENT,
    DATA_ITEM_NUM_MAX,
};

/**
  * @addtogroup DEBUG_Log
  * @brief DEBUG external module.
  * @{
  */

 /**
  * @defgroup CONFIG_Def CONFIG_Def
  * @brief Content read/write.
  * @{
  */

/**
 * @brief load_read Reads the content in the configuration address based on the address
 * @attention None
 *
 * @param add [IN] Indicates the address to be read
 * @param value [OUT] read content
 * @param len [OUT] Length of the read content
 * @retval None
 */
void ExtLoadRead(uintptr_t add, char *value, int len);

/**
 * @extLoadWrite Write the content in the configuration address according to the address
 * @attention None
 *
 * @param add [IN] Address to be written
 * @param value [IN] What is written
 * @param len [IN] Length of the content to be written
 * @retval None
 */
void ExtLoadWrite(uintptr_t add, const char *value, int len);

/**
 * @extConfigRead Reads content based on data items
 * @attention None
 *
 * @param item [IN] Read Data Items
 * @param value [OUT] Read content
 * @param len [OUT] Length of the read content
 * @retval None
 */
void ExtConfigRead(enum DataItem item, char *value, int len);

/**
 * @brief load_write Write content based on data items
 * @attention Nonw
 *
 * @param item [IN] Data Items Written
 * @param value [IN] Contents of write
 * @param len [IN] Length of the content to be written
 * @retval None
 */
void ExtConfigWrite(enum DataItem item, const char *value, int len);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */
/**
  * @}
  */

/**
  * @}
  */
#endif /* __EXT_DEBUG_H__ */
