/**
  ******************************************************************************
  * @file    bsp_GeneralTIM.c
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
	 
/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_GeneralTIM.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 基本定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void GENERAL_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sOCConfig;

  htimx.Instance = GENERAL_TIMx;
  htimx.Init.Prescaler = GENERAL_TIM_PRESCALER;
  htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx.Init.Period = GENERAL_TIM_PERIOD;
  htimx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OnePulse_Init(&htimx,TIM_OPMODE_SINGLE);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htimx, &sClockSourceConfig);

  /* 定时器比较输出配置 */
  sOCConfig.OCMode = TIM_OCMODE_TOGGLE;                // 比较输出模式：反转输出
  sOCConfig.Pulse = 0xFFFF;                      // 脉冲数
  sOCConfig.OCPolarity = TIM_OCPOLARITY_HIGH;          // 输出极性
  sOCConfig.OCNPolarity = TIM_OCNPOLARITY_LOW;         // 互补通道输出极性
  sOCConfig.OCFastMode = TIM_OCFAST_DISABLE;           // 快速模式
  sOCConfig.OCIdleState = TIM_OCIDLESTATE_RESET;       // 空闲电平
  sOCConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // 互补通道空闲电平
  HAL_TIM_OC_ConfigChannel(&htimx, &sOCConfig, STEPMOTOR_TIM_CHANNEL_1);
  HAL_TIM_OC_ConfigChannel(&htimx, &sOCConfig, STEPMOTOR_TIM_CHANNEL_2);
  
}

/**
  * 函数功能: 基本定时器硬件初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==GENERAL_TIMx)
  {
    /* 基本定时器外设时钟使能 */
    GENERAL_TIM_RCC_CLK_ENABLE();

    /* 外设中断配置 */
    HAL_NVIC_SetPriority(GENERAL_TIM_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(GENERAL_TIM_IRQ);
  }
}

///**
//  * 函数功能: 基本定时器硬件反初始化配置
//  * 输入参数: htim_base：基本定时器句柄类型指针
//  * 返 回 值: 无
//  * 说    明: 该函数被HAL库内部调用
//  */
//void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
//{

//  if(htim_base->Instance==GENERAL_TIMx)
//  {
//    /* 基本定时器外设时钟禁用 */
//    GENERAL_TIM_RCC_CLK_DISABLE();

//    /* 关闭外设中断 */
//    HAL_NVIC_DisableIRQ(GENERAL_TIM_IRQ);
//  }
//} 

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
