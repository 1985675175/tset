/**
  ******************************************************************************
  * @file    bsp_lamp.c
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
#include "bsp_lamp.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 板载照明灯IO引脚初始化.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：使用宏定义方法代替具体引脚号，方便程序移植，只要简单修改bsp_lamp.h
  *           文件相关宏定义就可以方便修改引脚。
  */
void LAMP_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 使能LAMP引脚对应的GPIO端口时钟 */
  LAMP_RCC_GPIO_CLK_ENABLE();

  /* 配置LAMP引脚默认输出电压 */
  HAL_GPIO_WritePin(LAMP_GPIO_PORT, LAMP_GPIO_PIN, GPIO_PIN_RESET);

  /*配置GPIO引脚 */
  GPIO_InitStruct.Pin = LAMP_GPIO_PIN;             // I/O编号
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // 工作模式，开漏输出
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // 不使用上拉/下拉输出
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;// I/O操作速度 低
  /* 根据上面的配置初始化LAMP的引脚 */
  HAL_GPIO_Init(LAMP_GPIO_PORT, &GPIO_InitStruct);
}


/**
  * 函数功能: 设置板载照明灯的状态
  * 输入参数：state:设置照明灯的输出状态。
  *             可选值：LAMP_OFF：照明灯灭；
  *                     LAMP_ON： 照明灯亮。
  *                     LAMP_TOGGLE：反转照明灯
  * 返 回 值: 无
  * 说    明：该函数使用类似HAL库函数的编程方法，方便理解HAL库函数编程思想。
  */
void LAMP_StateSet(LAMPState_TypeDef state)
{
  /* 检查输入参数是否合法 */
  assert_param(IS_LAMP_STATE(state));
  
  /* 判断设置的照明灯状态*/
  switch(state)
  {
    /* 点亮照明灯 */
    case LAMP_ON:
      LAMP_ON();
      break;
    /* 熄灭照明灯 */
    case LAMP_OFF:
      LAMP_OFF();
      break;
    /* 翻转照明灯 */      
    case LAMP_TOGGLE:
      LAMP_TOGGLE();
      break;
  }
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
