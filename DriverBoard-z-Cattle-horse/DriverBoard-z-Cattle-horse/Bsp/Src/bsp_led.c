/**
  ******************************************************************************
  * @file    bsp_led.c
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
#include "bsp_led.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 板载LED灯IO引脚初始化.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：使用宏定义方法代替具体引脚号，方便程序移植，只要简单修改bsp_led.h
  *           文件相关宏定义就可以方便修改引脚。
  */
void LED_GPIO_Init(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 使能LED引脚对应的GPIO端口时钟 */
  LED1_RCC_GPIO_CLK_ENABLE();
  LED2_RCC_GPIO_CLK_ENABLE();
  LED3_RCC_GPIO_CLK_ENABLE();

  /* 配置LED1引脚默认输出电压 */
  HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET);

  /*配置GPIO引脚 */
  GPIO_InitStruct.Pin = LED1_PIN;             // I/O编号
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 工作模式，推挽输出
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // 不使用上拉/下拉输出
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;// I/O操作速度 低
  /* 根据上面的配置初始化LED的引脚 */
  HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStruct);
  
  /*配置GPIO引脚 */
  GPIO_InitStruct.Pin = LED2_PIN;             // I/O编号
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 工作模式，推挽输出
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // 不使用上拉/下拉输出
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;// I/O操作速度 低
  /* 根据上面的配置初始化LED的引脚 */
  HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);

  /*配置GPIO引脚 */
  GPIO_InitStruct.Pin = LED3_PIN;             // I/O编号
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 工作模式，推挽输出
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // 不使用上拉/下拉输出
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;// I/O操作速度 低
  /* 根据上面的配置初始化LED的引脚 */
  HAL_GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStruct);
}


/**
  * 函数功能: 设置板载LED灯的状态
  * 输入参数: LEDx:其中x可设置为(1,2,3)用来选择对应的LED灯
  * 输入参数：state:设置LED灯的输出状态。
  *             可选值：LED_OFF：LED灯灭；
  *                     LED_ON： LED灯亮。
  *                     LED_TOGGLE：反转LED
  * 返 回 值: 无
  * 说    明：该函数使用类似HAL库函数的编程方法，方便理解HAL库函数编程思想。
  */
void LEDx_StateSet(uint8_t LEDx,LEDState_TypeDef state)
{
  uint8_t led ;
  
  /* 检查输入参数是否合法 */
  assert_param(IS_LED_TYPEDEF(LEDx));
  assert_param(IS_LED_STATE(state));
  
  /* 判断设置的LED灯状态*/
  switch(state)
  {
    /* 点亮LED */
    case LED_ON:
      led = LEDx & LED1;
      if(led == LED1)
        LED1_ON();
      
      led = LEDx & LED2;
      if(led == LED2)
        LED2_ON();
      
      led = LEDx & LED3;
      if(led == LED3)
        LED3_OFF();
      break;
      
    /* 熄灭LED */
    case LED_OFF:
      led = LEDx & LED1;
      if(led == LED1)            
        LED1_OFF();
      
      led = LEDx & LED2;
      if(LEDx & LED2)
        LED2_OFF();
      
      led = LEDx & LED2;
      if(LEDx & LED3)
        LED3_OFF();
      break;
    
    /* 翻转LED */      
    case LED_TOGGLE:
      led = LEDx & LED1;
      if(led == LED1)            
        LED1_TOGGLE();/* 设置引脚输出反转 */ 
      
      led = LEDx & LED2;
      if(LEDx & LED2)
        LED2_TOGGLE();
      
      led = LEDx & LED2;
      if(LEDx & LED3)
        LED3_TOGGLE();
      break;
  }
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
