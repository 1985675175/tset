/**
  ******************************************************************************
  * @file    bsp_beep.c
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
#include "bsp_beep.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 板载蜂鸣器IO引脚初始化.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：使用宏定义方法代替具体引脚号，方便程序移植，只要简单修改bsp_beep.h
  *           文件相关宏定义就可以方便修改引脚。
  */
void BEEP_GPIO_Init(void)
{
   /* 定义IO硬件初始化结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;
	
	/* 使能(开启)蜂鸣器引脚对应IO端口时钟 */  
  BEEP_RCC_CLK_ENABLE();
  
  /* 设置引脚输出为低电平，此时蜂鸣器不响 */
  HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_RESET);  
  
  /* 设定蜂鸣器对应引脚IO编号 */
  GPIO_InitStruct.Pin = BEEP_GPIO_PIN;
  /* 设定蜂鸣器对应引脚IO为输出模式 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  /* 设定蜂鸣器对应引脚IO操作速度 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  /* 初始化蜂鸣器对应引脚IO */
  HAL_GPIO_Init(BEEP_GPIO_PORT, &GPIO_InitStruct);  
}

/**
  * 函数功能: 设置板载蜂鸣器的状态
  * 输入参数：state:设置蜂鸣器的状态。
  *             可选值：BEEPState_OFF：蜂鸣器不响；
  *             可选值：BEEPState_ON： 蜂鸣器响。
  * 返 回 值: 无
  * 说    明：该函数使用类似HALA库函数的编程方法，方便理解HAL库函数编程思想。
  */
void BEEP_StateSet(BEEPState_TypeDef state)
{
  /* 检查输入参数是否合法 */
  assert_param(BEEPState_TypeDef(state));
   
  /* 判断设置的蜂鸣器状态，如果设置为蜂鸣器响 */
  if(state==BEEPState_ON)
  {    
    BEEP_ON();/* 蜂鸣器响 */
  }
  else /* state=BEEPState_OFF：设置蜂鸣器不响 */
  {    
    BEEP_OFF();/* 蜂鸣器不响 */
  }
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
