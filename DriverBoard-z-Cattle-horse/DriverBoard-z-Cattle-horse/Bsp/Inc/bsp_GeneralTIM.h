/**
  ******************************************************************************
  * @file    bsp_GeneralTIM.h
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
	 
#ifndef __GENERAL_TIM_H__
#define __GENERAL_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
/********************通用定时器TIM参数定义，TIM2~TIM5************/

#define GENERAL_TIMx                     TIM5
#define GENERAL_TIM_RCC_CLK_ENABLE()     __HAL_RCC_TIM5_CLK_ENABLE()
#define GENERAL_TIM_RCC_CLK_DISABLE()    __HAL_RCC_TIM5_CLK_DISABLE()
#define GENERAL_TIM_IRQ                  TIM5_IRQn
#define GENERAL_TIM_INT_FUN              TIM5_IRQHandler

#define STEPMOTOR_TIM_CHANNEL_1          TIM_CHANNEL_1
#define STEPMOTOR_TIM_CHANNEL_2          TIM_CHANNEL_2
// 定义定时器预分频，定时器实际时钟频率为：84MHz/（GENERAL_TIM_PRESCALER+1）
#define GENERAL_TIM_PRESCALER           83  // 实际时钟频率为：1MHz
// 定义定时器周期，当定时器开始计数到BASIC_TIMx_PERIOD值是更新定时器并生成对应事件和中断
#define GENERAL_TIM_PERIOD              0xFFFF 

/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx;

/* 函数声明 ------------------------------------------------------------------*/

void GENERAL_TIMx_Init(void);

#endif	/* __GENERAL_TIM_H__ */

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
