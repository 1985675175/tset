/**
  ******************************************************************************
  * @file    bsp_led.h
  * @compony  Superior Synthesis Biotechnology Co., LTD
  * @web     http://www.superior-synthesis.com    
  * @phone   (+86) 400-0892-893    
  * @author  Superior Synthesis Embedded Team/Arcmer
  * @version V0.0.1
  * @date    30-November-2022
  * @brief   
  *           
  ******************************************************************************
  */
/* Copyright (c) 2020 - 2022 Superior Synthesis Biotechnology Co., LTD

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of SS nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

#ifndef __BSP_LED_H_
#define __BSP_LED_H_

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
  LED_OFF = 0,
  LED_ON  = 1,
  LED_TOGGLE = 2,
}LEDState_TypeDef;
#define IS_LED_STATE(STATE)           (((STATE) == LED_OFF) || ((STATE) == LED_ON) || ((STATE) == LED_TOGGLE))

/* 宏定义 --------------------------------------------------------------------*/
#define LED1                        (uint8_t)0x01
#define LED2                        (uint8_t)0x02
#define LED3                        (uint8_t)0x04
#define IS_LED_TYPEDEF(LED)         (((LED) == LED1) || ((LED) == LED2) || ((LED) == LED3))

/* GPIO初始化定义 */
#define LED1_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOH_CLK_ENABLE()
#define LED1_PIN                    GPIO_PIN_9
#define LED1_GPIO_PORT              GPIOH

#define LED2_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define LED2_PIN                    GPIO_PIN_5
#define LED2_GPIO_PORT              GPIOE

#define LED3_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define LED3_PIN                    GPIO_PIN_6
#define LED3_GPIO_PORT              GPIOE

/* 宏定义函数 */
#define LED1_ON()                   HAL_GPIO_WritePin(LED1_GPIO_PORT,LED1_PIN, GPIO_PIN_RESET)
#define LED1_OFF()                  HAL_GPIO_WritePin(LED1_GPIO_PORT,LED1_PIN, GPIO_PIN_SET)
#define LED1_TOGGLE()               HAL_GPIO_TogglePin(LED1_GPIO_PORT,LED1_PIN)

#define LED2_ON()                   HAL_GPIO_WritePin(LED2_GPIO_PORT,LED2_PIN, GPIO_PIN_RESET)
#define LED2_OFF()                  HAL_GPIO_WritePin(LED2_GPIO_PORT,LED2_PIN, GPIO_PIN_SET)
#define LED2_TOGGLE()               HAL_GPIO_TogglePin(LED2_GPIO_PORT,LED2_PIN)

#define LED3_ON()                   HAL_GPIO_WritePin(LED3_GPIO_PORT,LED3_PIN, GPIO_PIN_RESET)
#define LED3_OFF()                  HAL_GPIO_WritePin(LED3_GPIO_PORT,LED3_PIN, GPIO_PIN_SET)
#define LED3_TOGGLE()               HAL_GPIO_TogglePin(LED3_GPIO_PORT,LED3_PIN)

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/

void LED_GPIO_Init(void);
void LEDx_StateSet(uint8_t LEDx,LEDState_TypeDef state);

#endif /* __BSP_LED_H_ */


/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
