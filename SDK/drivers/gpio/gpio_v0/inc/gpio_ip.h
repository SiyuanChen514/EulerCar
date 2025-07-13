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
  * @file      gpio_ip.h
  * @author    MCU Driver Team
  * @brief     GPIO module driver
  * @details   The header file contains the following declaration:
  *             + GPIO configuration enums.
  *             + GPIO register structures.
  *             + GPIO DCL Functions.
  *             + Parameters check functions.
  */

#ifndef McuMagicTag_GPIO_IP_H
#define McuMagicTag_GPIO_IP_H

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"
/* Macro definitions ---------------------------------------------------------*/
#ifdef GPIO_PARAM_CHECK
    #define GPIO_ASSERT_PARAM         BASE_FUNC_ASSERT_PARAM
    #define GPIO_PARAM_CHECK_NO_RET   BASE_FUNC_PARAMCHECK_NO_RET
    #define GPIO_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
    #define GPIO_ASSERT_PARAM(para)                 ((void)0U)
    #define GPIO_PARAM_CHECK_NO_RET(para)           ((void)0U)
    #define GPIO_PARAM_CHECK_WITH_RET(param, ret)   ((void)0U)
#endif

/**
  * @addtogroup GPIO
  * @{
  */

/**
  * @defgroup GPIO_IP
  * @{
  */

/* Macro definitions ---------------------------------------------------------*/
#define GPIO_PIN_NUM (0x00000008U)
#define GPIO_PIN_MASK (0x000000FFU)

/**
  * @defgroup GPIO_Param_Def GPIO Parameters Definition
  * @brief Description of GPIO configuration parameters.
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/
/**
  * @brief GPIO PIN enum definition
  */
typedef enum {
    GPIO_PIN_0 = 0x00000001U,
    GPIO_PIN_1 = 0x00000002U,
    GPIO_PIN_2 = 0x00000004U,
    GPIO_PIN_3 = 0x00000008U,
    GPIO_PIN_4 = 0x00000010U,
    GPIO_PIN_5 = 0x00000020U,
    GPIO_PIN_6 = 0x00000040U,
    GPIO_PIN_7 = 0x00000080U,
    GPIO_PIN_ALL = 0x000000FFU
} GPIO_PIN;

/**
  * @brief GPIO PIN value enum definition.
  */
typedef enum {
    GPIO_LOW_LEVEL = 0x00000000U,
    GPIO_HIGH_LEVEL = 0x00000001U
} GPIO_Value;

/**
  * @brief GPIO direction mode enum definition.
  * @details status flag:
  *         + GPIO_INPUT_MODE -- GPIO pin as input,
  *           maximum input voltage of 3.63V for all types of I/O except 5V I/O,
  *           maximum input voltage of 5.0V for 5V I/O type.
  *         + GPIO_OUTPUT_MODE -- GPIO pin as output,
  */
typedef enum {
    GPIO_INPUT_MODE = 0x00000000U,
    GPIO_OUTPUT_MODE = 0x00000001U
} GPIO_Direction;

/**
  * @brief GPIO interrupt mode enum definition.
  */
typedef enum {
    GPIO_INT_TYPE_FALL_EDGE = 0x00000000U,
    GPIO_INT_TYPE_RISE_EDGE = 0x00000001U,
    GPIO_INT_TYPE_LOW_LEVEL = 0x00000002U,
    GPIO_INT_TYPE_HIGH_LEVEL = 0x00000003U,
    GPIO_INT_TYPE_BOTH_EDGE = 0x00000004U,
    GPIO_INT_TYPE_NONE = 0x00000005U
} GPIO_InterruptMode;

/**
  * @brief GPIO extend handle, configuring some special parameters.
  */
typedef struct {
} GPIO_ExtendHandle;

/**
  * @brief User-defined callback function.
  */
typedef struct {
    /* GPIO pin callback functions */
    struct {
        GPIO_PIN pin;
        void (*callbackFunc)(void* handle);
    } GPIO_CallbackFuncs[GPIO_PIN_NUM];
} GPIO_UserCallBcak;

/**
  * @}
  */

/**
  * @defgroup GPIO_Reg_Def GPIO Register Definition
  * @brief Description GPIO register mapping structure.
  * @{
  */

/**
  * @brief GPIO data registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pin0 : 1;      /**< pin0 0:Input Data, 1:OutPut Data. */
        unsigned int pin1 : 1;      /**< pin1 0:Input Data, 1:OutPut Data. */
        unsigned int pin2 : 1;      /**< pin2 0:Input Data, 1:OutPut Data. */
        unsigned int pin3 : 1;      /**< pin3 0:Input Data, 1:OutPut Data. */
        unsigned int pin4 : 1;      /**< pin4 0:Input Data, 1:OutPut Data. */
        unsigned int pin5 : 1;      /**< pin5 0:Input Data, 1:OutPut Data. */
        unsigned int pin6 : 1;      /**< pin6 0:Input Data, 1:OutPut Data. */
        unsigned int pin7 : 1;      /**< pin7 0:Input Data, 1:OutPut Data. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile GPIO_DATA_REG[256];

/**
  * @brief GPIO direction registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pin0 : 1;       /**< pin0 0:Input Direction, 1:OutPut Direction. */
        unsigned int pin1 : 1;       /**< pin1 0:Input Direction, 1:OutPut Direction. */
        unsigned int pin2 : 1;       /**< pin2 0:Input Direction, 1:OutPut Direction. */
        unsigned int pin3 : 1;       /**< pin3 0:Input Direction, 1:OutPut Direction. */
        unsigned int pin4 : 1;       /**< pin4 0:Input Direction, 1:OutPut Direction. */
        unsigned int pin5 : 1;       /**< pin5 0:Input Direction, 1:OutPut Direction. */
        unsigned int pin6 : 1;       /**< pin6 0:Input Direction, 1:OutPut Direction. */
        unsigned int pin7 : 1;       /**< pin7 0:Input Direction, 1:OutPut Direction. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile GPIO_DIR_REG;

/**
  * @brief GPIO interrupt type registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pin0 : 1;       /**< pin0 0:edge interrupt, 1:level interrupt. */
        unsigned int pin1 : 1;       /**< pin1 0:edge interrupt, 1:level interrupt. */
        unsigned int pin2 : 1;       /**< pin2 0:edge interrupt, 1:level interrupt. */
        unsigned int pin3 : 1;       /**< pin3 0:edge interrupt, 1:level interrupt. */
        unsigned int pin4 : 1;       /**< pin4 0:edge interrupt, 1:level interrupt. */
        unsigned int pin5 : 1;       /**< pin5 0:edge interrupt, 1:level interrupt. */
        unsigned int pin6 : 1;       /**< pin6 0:edge interrupt, 1:level interrupt. */
        unsigned int pin7 : 1;       /**< pin7 0:edge interrupt, 1:level interrupt. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile GPIO_IS_REG;

/**
  * @brief GPIO edge type registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pin0 : 1;        /**< pin0 0:rising or falling edge, 1: both edge. */
        unsigned int pin1 : 1;        /**< pin1 0:rising or falling edge, 1: both edge. */
        unsigned int pin2 : 1;        /**< pin2 0:rising or falling edge, 1: both edge. */
        unsigned int pin3 : 1;        /**< pin3 0:rising or falling edge, 1: both edge. */
        unsigned int pin4 : 1;        /**< pin4 0:rising or falling edge, 1: both edge. */
        unsigned int pin5 : 1;        /**< pin5 0:rising or falling edge, 1: both edge. */
        unsigned int pin6 : 1;        /**< pin6 0:rising or falling edge, 1: both edge. */
        unsigned int pin7 : 1;        /**< pin7 0:rising or falling edge, 1: both edge. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile GPIO_IBE_REG;

/**
  * @brief GPIO interrupt condition registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pin0 : 1;        /**< pin0 0:falling edge / low level, 1:rising edge / high level. */
        unsigned int pin1 : 1;        /**< pin1 0:falling edge / low level, 1:rising edge / high level. */
        unsigned int pin2 : 1;        /**< pin2 0:falling edge / low level, 1:rising edge / high level. */
        unsigned int pin3 : 1;        /**< pin3 0:falling edge / low level, 1:rising edge / high level. */
        unsigned int pin4 : 1;        /**< pin4 0:falling edge / low level, 1:rising edge / high level. */
        unsigned int pin5 : 1;        /**< pin5 0:falling edge / low level, 1:rising edge / high level. */
        unsigned int pin6 : 1;        /**< pin6 0:falling edge / low level, 1:rising edge / high level. */
        unsigned int pin7 : 1;        /**< pin7 0:falling edge / low level, 1:rising edge / high level. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile GPIO_IEV_REG;

/**
  * @brief GPIO interrupt enable registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pin0 : 1;        /**< pin0 0:mask interrupt, 1:unmask interrupt. */
        unsigned int pin1 : 1;        /**< pin1 0:mask interrupt, 1:unmask interrupt. */
        unsigned int pin2 : 1;        /**< pin2 0:mask interrupt, 1:unmask interrupt. */
        unsigned int pin3 : 1;        /**< pin3 0:mask interrupt, 1:unmask interrupt. */
        unsigned int pin4 : 1;        /**< pin4 0:mask interrupt, 1:unmask interrupt. */
        unsigned int pin5 : 1;        /**< pin5 0:mask interrupt, 1:unmask interrupt. */
        unsigned int pin6 : 1;        /**< pin6 0:mask interrupt, 1:unmask interrupt. */
        unsigned int pin7 : 1;        /**< pin7 0:mask interrupt, 1:unmask interrupt. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile GPIO_IE_REG;

/**
  * @brief GPIO original interrupt signal registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pin0 : 1;        /**< pin0 0:no interrupt, 1:has interrupt. */
        unsigned int pin1 : 1;        /**< pin1 0:no interrupt, 1:has interrupt. */
        unsigned int pin2 : 1;        /**< pin2 0:no interrupt, 1:has interrupt. */
        unsigned int pin3 : 1;        /**< pin3 0:no interrupt, 1:has interrupt. */
        unsigned int pin4 : 1;        /**< pin4 0:no interrupt, 1:has interrupt. */
        unsigned int pin5 : 1;        /**< pin5 0:no interrupt, 1:has interrupt. */
        unsigned int pin6 : 1;;       /**< pin6 0:no interrupt, 1:has interrupt. */
        unsigned int pin7 : 1;        /**< pin7 0:no interrupt, 1:has interrupt. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile GPIO_RIS_REG;

/**
  * @brief GPIO mask interrupt signal registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pin0 : 1;        /**< pin0 0:no interrupt, 1:has interrupt. */
        unsigned int pin1 : 1;        /**< pin1 0:no interrupt, 1:has interrupt. */
        unsigned int pin2 : 1;        /**< pin2 0:no interrupt, 1:has interrupt. */
        unsigned int pin3 : 1;        /**< pin3 0:no interrupt, 1:has interrupt. */
        unsigned int pin4 : 1;        /**< pin4 0:no interrupt, 1:has interrupt. */
        unsigned int pin5 : 1;        /**< pin5 0:no interrupt, 1:has interrupt. */
        unsigned int pin6 : 1;        /**< pin6 0:no interrupt, 1:has interrupt. */
        unsigned int pin7 : 1;        /**< pin7 0:no interrupt, 1:has interrupt. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile GPIO_MIS_REG;

/**
  * @brief GPIO interrupt clear registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pin0 : 1;        /**< pin0 0:no effect, 1:clear interrupt. */
        unsigned int pin1 : 1;        /**< pin1 0:no effect, 1:clear interrupt. */
        unsigned int pin2 : 1;        /**< pin2 0:no effect, 1:clear interrupt. */
        unsigned int pin3 : 1;        /**< pin3 0:no effect, 1:clear interrupt. */
        unsigned int pin4 : 1;        /**< pin4 0:no effect, 1:clear interrupt. */
        unsigned int pin5 : 1;        /**< pin5 0:no effect, 1:clear interrupt. */
        unsigned int pin6 : 1;        /**< pin6 0:no effect, 1:clear interrupt. */
        unsigned int pin7 : 1;        /**< pin7 0:no effect, 1:clear interrupt. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile GPIO_IC_REG;

/**
  * @brief GPIO assemble registers structure definition
  */
typedef struct {
    GPIO_DATA_REG GPIO_DATA; /**< gpio data register. Offset Address: 0x000～0x3FC.*/
    GPIO_DIR_REG  GPIO_DIR;  /**< gpio direction register. Offset Address: 0x400. */
    GPIO_IS_REG   GPIO_IS;   /**< gpio interrupt type register. Offset Address: 0x404. */
    GPIO_IBE_REG  GPIO_IBE;  /**< gpio edge type register. Offset Address: 0x408. */
    GPIO_IEV_REG  GPIO_IEV;  /**< gpio interrupt condition register. Offset Address: 0x40C. */
    GPIO_IE_REG   GPIO_IE;   /**< gpio interrupt enable register. Offset Address: 0x410. */
    GPIO_RIS_REG  GPIO_RIS;  /**< gpio original interrupt register. Offset Address: 0x414. */
    GPIO_MIS_REG  GPIO_MIS;  /**< gpio mask interrupt register. Offset Address: 0x418. */
    GPIO_IC_REG   GPIO_IC;   /**< gpio interrupt clear register. Offset Address: 0x41C. */
} volatile GPIO_RegStruct;
/**
  * @}
  */

/**
  * @brief Struct of map GPIO register and lock type.
  */
typedef struct {
    GPIO_RegStruct *gpioGroup;
    CHIP_LockType lockType;
} GPIO_MatchLockType;

/**
  * @brief Check gpio value parameter.
  * @param value Value of @ref GPIO_Value
  * @retval Bool.
  */
static inline bool IsGpioValue(GPIO_Value value)
{
    return (value == GPIO_LOW_LEVEL || value == GPIO_HIGH_LEVEL);
}

/**
  * @brief Check gpio direction parameter.
  * @param dir Value of @ref GPIO_Direction.
  * @retval Bool.
  */
static inline bool IsGpioDirection(GPIO_Direction dir)
{
    return (dir == GPIO_INPUT_MODE || dir == GPIO_OUTPUT_MODE);
}

/**
  * @brief Check gpio pins parameter.
  * @param pins OR logical combination of pin.
  * @retval Bool.
  */
static inline bool IsGpioPins(unsigned int pins)
{
    return ((pins & GPIO_PIN_MASK) != BASE_CFG_UNSET) && ((pins & ~GPIO_PIN_MASK) == BASE_CFG_UNSET);
}

/**
  * @brief Check gpio pin parameter.
  * @param pin Value of @ref GPIO_PIN.
  * @retval Bool.
  */
static inline bool IsGpioPin(GPIO_PIN pin)
{
    /* Check whether gpio pin */
    return (pin == GPIO_PIN_0 || pin == GPIO_PIN_1 || \
            pin == GPIO_PIN_2 || pin == GPIO_PIN_3 || \
            pin == GPIO_PIN_4 || pin == GPIO_PIN_5 || \
            pin == GPIO_PIN_6 || pin == GPIO_PIN_7 || \
            pin == GPIO_PIN_ALL);
}

/**
  * @brief Check gpio interrupt mode parameter.
  * @param mode Value of @ref GPIO_InterruptMode.
  * @retval Bool.
  */
static inline bool IsGpioITMode(GPIO_InterruptMode mode)
{
    /* Check whether gpio interrupt mode */
    return (mode == GPIO_INT_TYPE_HIGH_LEVEL || \
            mode == GPIO_INT_TYPE_LOW_LEVEL || \
            mode == GPIO_INT_TYPE_RISE_EDGE || \
            mode == GPIO_INT_TYPE_FALL_EDGE || \
            mode == GPIO_INT_TYPE_BOTH_EDGE || \
            mode == GPIO_INT_TYPE_NONE);
}

/**
  * @brief Setting GPIO pin level
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pins OR logical combination of pin.
  * @param value Value of @ref GPIO_Value.
  * @retval None.
  */
static inline void DCL_GPIO_SetValue(GPIO_RegStruct *gpiox, unsigned int pins, GPIO_Value value)
{
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    GPIO_PARAM_CHECK_NO_RET(IsGpioValue(value));
    gpiox->GPIO_DATA[pins].reg = (value == GPIO_HIGH_LEVEL ? pins : BASE_CFG_UNSET); /* Set GPIO pin level */
}

/**
  * @brief Getting all GPIO level.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @retval unsigned int All GPIO pin level.
  */
static inline unsigned int DCL_GPIO_GetAllValue(const GPIO_RegStruct *gpiox)
{
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    return gpiox->GPIO_DATA[GPIO_PIN_MASK].reg & GPIO_PIN_MASK; /* Get all GPIO level. */
}

/**
  * @brief Getting pin GPIO level.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pin OR logical combination of pin.
  * @retval unsigned int GPIO pin level.
  */
static inline GPIO_Value DCL_GPIO_GetPinValue(const GPIO_RegStruct *gpiox, GPIO_PIN pin)
{
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_ASSERT_PARAM(IsGpioPin(pin));
    /* Get pin GPIO level. */
    return (gpiox->GPIO_DATA[GPIO_PIN_MASK].reg & pin) == BASE_CFG_UNSET ? GPIO_LOW_LEVEL : GPIO_HIGH_LEVEL;
}

/**
  * @brief Setting GPIO pin direction.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pins OR logical combination of pin.
  * @param dir Value of @ref GPIO_Direction.
  * @retval None.
  */
static inline void DCL_GPIO_SetDirection(GPIO_RegStruct *gpiox, unsigned int pins, GPIO_Direction dir)
{
    /* param check */
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    GPIO_PARAM_CHECK_NO_RET(IsGpioDirection(dir));
    if (dir == GPIO_INPUT_MODE) { /* Set GPIO pin direction */
        gpiox->GPIO_DIR.reg &= ~pins;
    } else if (dir == GPIO_OUTPUT_MODE) {
        gpiox->GPIO_DIR.reg |= pins;
    }
}

/**
  * @brief Getting GPIO pin direction.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pin OR logical combination of pin.
  * @retval GPIO direction, 0:input mode, 1:output mode.
  */
static inline GPIO_Direction DCL_GPIO_GetPinDirection(const GPIO_RegStruct *gpiox, GPIO_PIN pin)
{
    /* param check */
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_ASSERT_PARAM(IsGpioPin(pin));
    return (gpiox->GPIO_DIR.reg & pin) == BASE_CFG_UNSET ? GPIO_INPUT_MODE : GPIO_OUTPUT_MODE;
}

/**
  * @brief Getting GPIO all pin direction.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @retval unsigned int All GPIO pin direction.
  */
static inline unsigned int DCL_GPIO_GetAllPinDirection(const GPIO_RegStruct *gpiox)
{
    /* param check */
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    return gpiox->GPIO_DIR.reg & GPIO_PIN_MASK;
}

/**
  * @brief Setting GPIO pins edge trigger.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pins OR logical combination of pin.
  * @retval None.
  */
static inline void DCL_GPIO_SetPinsEdgeTrigger(GPIO_RegStruct *gpiox, unsigned int pins)
{
    /* param check */
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    gpiox->GPIO_IS.reg &= ~pins;
}

/**
  * @brief Setting GPIO pins level trigger.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pins OR logical combination of pin.
  * @retval None.
  */
static inline void DCL_GPIO_SetPinsLevelTrigger(GPIO_RegStruct *gpiox, unsigned int pins)
{
    /* param check */
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    gpiox->GPIO_IS.reg |= pins;
}

/**
  * @brief Getting GPIO pin trigger type.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @retval trigger type, 0:edge trigger; 1:level trigger.
  */
static inline unsigned int DCL_GPIO_GetPinsTriggerType(const GPIO_RegStruct *gpiox)
{
    /* param check */
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    return (gpiox->GPIO_IS.reg & GPIO_PIN_MASK);
}

/**
  * @brief Setting GPIO pins single edge trigger.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pins OR logical combination of pin.
  * @retval None.
  */
static inline void DCL_GPIO_SetPinsSingleEdgeTrigger(GPIO_RegStruct *gpiox, unsigned int pins)
{
    /* param check */
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    gpiox->GPIO_IBE.reg &= ~pins;
}

/**
  * @brief Setting GPIO pins both edge trigger.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pins OR logical combination of pin.
  * @retval None.
  */
static inline void DCL_GPIO_SetPinsBothEdgeTrigger(GPIO_RegStruct *gpiox, unsigned int pins)
{
    /* param check */
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    gpiox->GPIO_IBE.reg |= pins;
}

/**
  * @brief Getting GPIO pin edge trigger type.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @retval edge trigger type, pin value is 0:signle edge trigger; 1:both edge trigger.
  */
static inline unsigned int DCL_GPIO_GetPinsEdgeTriggerType(const GPIO_RegStruct *gpiox)
{
    /* param check */
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    return (gpiox->GPIO_IBE.reg & GPIO_PIN_MASK);
}

/**
  * @brief Setting GPIO pins falling edge or low level trigger.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pins OR logical combination of pin.
  * @retval None.
  */
static inline void DCL_GPIO_SetPinsFallingEdgeOrLowLevelTrigger(GPIO_RegStruct *gpiox, unsigned int pins)
{
    /* param check */
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    gpiox->GPIO_IEV.reg &= ~pins;
}

/**
  * @brief Setting GPIO pins rising edge or high level trigger.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pins OR logical combination of pin.
  * @retval None.
  */
static inline void DCL_GPIO_SetPinsRisingEdgeOrHighLevelTrigger(GPIO_RegStruct *gpiox, unsigned int pins)
{
    /* param check */
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    gpiox->GPIO_IEV.reg |= pins;
}

/**
  * @brief Getting GPIO pins trigger condition type.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @retval trigger condition type, pin value is 0:falling edge or low level trigger;
  * @retval trigger condition type, pin value is 1:rising edge or high level trigger.
  */
static inline unsigned int DCL_GPIO_GetPinsTriggerConditionType(const GPIO_RegStruct *gpiox)
{
    /* param check */
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    return (gpiox->GPIO_IEV.reg & GPIO_PIN_MASK);
}

/**
  * @brief Clear all gpio interrupt signal.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pins OR logical combination of pin.
  * @retval None.
  */
static inline void DCL_GPIO_ClearIrq(GPIO_RegStruct *gpiox, unsigned int pins)
{
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    gpiox->GPIO_IC.reg |= pins;
}

/**
  * @brief Enable gpio group interrupt.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pins OR logical combination of pins.
  * @retval None.
  */
static inline void DCL_GPIO_EnableIrq(GPIO_RegStruct *gpiox, unsigned int pins)
{
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    /* must clear interrupt first, prevents interrupts triggered by previous output mode. */
    DCL_GPIO_ClearIrq(gpiox, pins);
    gpiox->GPIO_IE.reg |= pins;
}

/**
  * @brief Disable gpio interrupt.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @param pins OR logical combination of pin.
  * @retval None.
  */
static inline void DCL_GPIO_DisableIrq(GPIO_RegStruct *gpiox, unsigned int pins)
{
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    GPIO_PARAM_CHECK_NO_RET(IsGpioPins(pins));
    gpiox->GPIO_IE.reg &= ~pins;
}

/**
  * @brief Getting all values of GPIO IE register.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @retval unsigned int All values of GPIO IE register.
  */
static inline unsigned int DCL_GPIO_GetIE(const GPIO_RegStruct *gpiox)
{
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    return gpiox->GPIO_IE.reg & GPIO_PIN_MASK;
}

/**
  * @brief Getting all values of GPIO RIS register.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @retval unsigned int All values of GPIO RIS register.
  */
static inline unsigned int DCL_GPIO_GetRIS(const GPIO_RegStruct *gpiox)
{
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    return gpiox->GPIO_RIS.reg & GPIO_PIN_MASK;
}

/**
  * @brief Getting all values of GPIO MIS register.
  * @param gpiox Value of @ref GPIO_RegStruct.
  * @retval unsigned int All values of GPIO MIS register.
  */
static inline unsigned int DCL_GPIO_GetMIS(const GPIO_RegStruct *gpiox)
{
    GPIO_ASSERT_PARAM(IsGPIOInstance(gpiox));
    return gpiox->GPIO_MIS.reg & GPIO_PIN_MASK;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_GPIO_IP_H */