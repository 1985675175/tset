/**
  ******************************************************************************
  * @file    bsp_BasicTIM.c
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
#include "bsp_BasicTIM.h" 

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_led;
TIM_HandleTypeDef htim7;
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 基本定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void BASIC_TIMx_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htimx_led.Instance = BASIC_TIMx;
  htimx_led.Init.Prescaler = BASIC_TIMx_PRESCALER;
  htimx_led.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx_led.Init.Period = BASIC_TIMx_PERIOD;
  HAL_TIM_Base_Init(&htimx_led);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_led, &sMasterConfig);
}
void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = (SystemCoreClock / 1000) - 1;;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 50-1;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
  }

}
/**
  * 函数功能: 基本定时器硬件初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==BASIC_TIMx)
  {
    /* 基本定时器外设时钟使能 */
    BASIC_TIM_RCC_CLK_ENABLE();

    /* 外设中断配置 */
    HAL_NVIC_SetPriority(BASIC_TIM_IRQ, 1, 0);
    HAL_NVIC_EnableIRQ(BASIC_TIM_IRQ);
  }
	 else if(htim_base->Instance==TIM7)
  {
    /* 基本定时器外设时钟使能 */
    __HAL_RCC_TIM7_CLK_ENABLE();

    /* 外设中断配置 */
    HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
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

//  if(htim_base->Instance==BASIC_TIMx)
//  {
//    /* 基本定时器外设时钟禁用 */
//    BASIC_TIM_RCC_CLK_DISABLE();

//    /* 关闭外设中断 */
//    HAL_NVIC_DisableIRQ(BASIC_TIM_IRQ);
//  }
//} 

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
