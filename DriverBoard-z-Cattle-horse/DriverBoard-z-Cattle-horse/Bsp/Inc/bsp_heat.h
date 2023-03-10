/**
  ******************************************************************************
  * @file    bsp_heat.h
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
	 
#ifndef __BSP_HEAT_H__
#define __BSP_HEAT_H__

/* ?????????? ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ???????? ------------------------------------------------------------------*/
typedef enum
{
  HEATState_OFF = 0,
  HEATState_ON,
}HEATState_TypeDef;

#define IS_HEAT_STATE(STATE)           (((STATE) == HEATState_OFF) || ((STATE) == HEATState_ON))

/* ?????? --------------------------------------------------------------------*/
#define HEAT_RCC_CLK_ENABLE()         __HAL_RCC_GPIOF_CLK_ENABLE()
#define HEAT_GPIO_PIN                 GPIO_PIN_12
#define HEAT_GPIO_PORT                GPIOF

#define HEAT_ON()                       HAL_GPIO_WritePin(HEAT_GPIO_PORT,HEAT_GPIO_PIN,GPIO_PIN_SET)    // ??????????
#define HEAT_OFF()                      HAL_GPIO_WritePin(HEAT_GPIO_PORT,HEAT_GPIO_PIN,GPIO_PIN_RESET)  // ??????????
#define HEAT_TOGGLE()                   HAL_GPIO_TogglePin(HEAT_GPIO_PORT,HEAT_GPIO_PIN)                // ????????


/* ???????? ------------------------------------------------------------------*/
/* ???????? ------------------------------------------------------------------*/
void HEAT_GPIO_Init(void);
void HEAT_StateSet(HEATState_TypeDef state);
  
#endif  // __BSP_HEAT_H__

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
