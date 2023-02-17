/**
  ******************************************************************************
  * @file    bsp_lamp.h
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

#ifndef __BSP_LAMP_H_
#define __BSP_LAMP_H_

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
  LAMP_OFF = 0,
  LAMP_ON  = 1,
  LAMP_TOGGLE = 2,
}LAMPState_TypeDef;
#define IS_LAMP_STATE(STATE)           (((STATE) == LAMP_OFF) || ((STATE) == LAMP_ON) || ((STATE) == LAMP_TOGGLE))

/* 宏定义 --------------------------------------------------------------------*/
/* GPIO初始化定义 */
#define LAMP_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define LAMP_GPIO_PIN               GPIO_PIN_13
#define LAMP_GPIO_PORT              GPIOF

/* 宏定义函数 */
#define LAMP_ON()                   HAL_GPIO_WritePin(LAMP_GPIO_PORT,LAMP_GPIO_PIN, GPIO_PIN_SET)
#define LAMP_OFF()                  HAL_GPIO_WritePin(LAMP_GPIO_PORT,LAMP_GPIO_PIN, GPIO_PIN_RESET)
#define LAMP_TOGGLE()               HAL_GPIO_TogglePin(LAMP_GPIO_PORT,LAMP_GPIO_PIN)

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/

void LAMP_GPIO_Init(void);
void LAMP_StateSet(LAMPState_TypeDef state);

#endif /* __BSP_LAMP_H_ */


/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
