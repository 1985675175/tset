/**
  ******************************************************************************
  * @file    bsp_limit.c
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
#include "bsp_limit.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
  TIM_HandleTypeDef  htim_SM3NEG;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void Limit_SM1_Init(void);
static void Limit_SM2_Init(void);
static void Limit_SM3_Init(void);
static void Limit_SM4_Init(void);


/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 限位开关引脚初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void Limit_GPIO_Init()
{
  /* 部分引脚共用了外部中断通道，所以使用了定时器的中断来作为限位控制 */
  /*--------------------- SMotor1 的限位开关 ----------------*/
  Limit_SM1_Init();
  
  /*--------------------- SMotor2 的限位开关 ----------------*/
  Limit_SM2_Init();

  /*--------------------- SMotor3 的限位开关 ----------------*/
  Limit_SM3_Init();
  
  /*--------------------- SMotor4 的限位开关 ----------------*/
  Limit_SM4_Init();

  /* 多个引脚共用同一中断线 */
  HAL_NVIC_SetPriority(LIMIT_n_IRQn,1,0);    // 9-5
  HAL_NVIC_EnableIRQ(LIMIT_n_IRQn);  
    
  HAL_NVIC_SetPriority(LIMIT_m_IRQn,1,0);    // 15-10
  HAL_NVIC_EnableIRQ(LIMIT_m_IRQn);  
}
/**
  * 函数功能: SM1限位开关引脚初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void Limit_SM1_Init()
{
  GPIO_InitTypeDef GPIO_InitStruct={0};

  /* 原点限位开关 */
  LIMIT1_ORI_RCC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin  = LIMIT1_ORI_PIN;
  GPIO_InitStruct.Mode = LIMIT1_ORI_ACTIVE_EDGE; //有效边沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = 0;                // 无复用
  HAL_GPIO_Init(LIMIT1_ORI_GPIO_PORT, &GPIO_InitStruct);  
  
  HAL_NVIC_SetPriority(LIMIT1_ORI_IRQn,1,0);
  HAL_NVIC_EnableIRQ(LIMIT1_ORI_IRQn);
  
  /* 正转限位开关 */
  LIMIT1_POS_RCC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin  = LIMIT1_POS_PIN;
  GPIO_InitStruct.Mode = LIMIT1_POS_ACTIVE_EDGE; //有效边沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = 0;                // 无复用
  HAL_GPIO_Init(LIMIT1_POS_GPIO_PORT, &GPIO_InitStruct);  
  
  HAL_NVIC_SetPriority(LIMIT1_POS_IRQn,1,0);
  HAL_NVIC_EnableIRQ(LIMIT1_POS_IRQn);
  
  /* 反转限位开关 */
  LIMIT1_NEG_RCC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin  = LIMIT1_NEG_PIN;
  GPIO_InitStruct.Mode = LIMIT1_NEG_ACTIVE_EDGE; //有效边沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = 0;                // 无复用
  HAL_GPIO_Init(LIMIT1_NEG_GPIO_PORT, &GPIO_InitStruct);  
  
  HAL_NVIC_SetPriority(LIMIT1_NEG_IRQn,1,0);
  HAL_NVIC_EnableIRQ(LIMIT1_NEG_IRQn);
}
/**
  * 函数功能: SM2限位开关引脚初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void Limit_SM2_Init()
{
  GPIO_InitTypeDef GPIO_InitStruct={0};

  /* 原点限位开关 */
  LIMIT2_ORI_RCC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin  = LIMIT2_ORI_PIN;
  GPIO_InitStruct.Mode = LIMIT2_ORI_ACTIVE_EDGE; //有效边沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = 0;                // 无复用
  HAL_GPIO_Init(LIMIT2_ORI_GPIO_PORT, &GPIO_InitStruct);  
  
  HAL_NVIC_SetPriority(LIMIT2_ORI_IRQn,1,0);
  HAL_NVIC_EnableIRQ(LIMIT2_ORI_IRQn);
  
  /* 正转限位开关 */
  LIMIT2_POS_RCC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin  = LIMIT2_POS_PIN;
  GPIO_InitStruct.Mode = LIMIT2_POS_ACTIVE_EDGE; //有效边沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = 0;                // 无复用
  HAL_GPIO_Init(LIMIT2_POS_GPIO_PORT, &GPIO_InitStruct);  
  
  /* 反转限位开关 */
  LIMIT2_NEG_RCC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin  = LIMIT2_NEG_PIN;
  GPIO_InitStruct.Mode = LIMIT2_NEG_ACTIVE_EDGE; //有效边沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = 0;                // 无复用
  HAL_GPIO_Init(LIMIT2_NEG_GPIO_PORT, &GPIO_InitStruct);  
  
  HAL_NVIC_SetPriority(LIMIT2_NEG_IRQn,1,0);
  HAL_NVIC_EnableIRQ(LIMIT2_NEG_IRQn);
}
/**
  * 函数功能: SM3限位开关引脚初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void Limit_SM3_Init()
{
  GPIO_InitTypeDef GPIO_InitStruct={0};

  /* 原点限位开关 */
  LIMIT3_ORI_RCC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin  = LIMIT3_ORI_PIN;
  GPIO_InitStruct.Mode = LIMIT3_ORI_ACTIVE_EDGE; //有效边沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = 0;                // 无复用
  HAL_GPIO_Init(LIMIT3_ORI_GPIO_PORT, &GPIO_InitStruct);  
  
  /* 正转限位开关 */
  LIMIT3_POS_RCC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin  = LIMIT3_POS_PIN;
  GPIO_InitStruct.Mode = LIMIT3_POS_ACTIVE_EDGE; //有效边沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = 0;                // 无复用
  HAL_GPIO_Init(LIMIT3_POS_GPIO_PORT, &GPIO_InitStruct);  
  
  /* 反转限位开关 */
  LIMIT3_NEG_RCC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin  = LIMIT3_NEG_PIN;
  GPIO_InitStruct.Mode = LIMIT3_NEG_ACTIVE_EDGE; //有效边沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = 0;                // 无复用
  HAL_GPIO_Init(LIMIT3_NEG_GPIO_PORT, &GPIO_InitStruct);  
}

/**
  * 函数功能: SM4限位开关引脚初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void Limit_SM4_Init()
{
  GPIO_InitTypeDef GPIO_InitStruct={0};

  /* 原点限位开关 */
  LIMIT4_ORI_RCC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin  = LIMIT4_ORI_PIN;
  GPIO_InitStruct.Mode = LIMIT4_ORI_ACTIVE_EDGE; //有效边沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = 0;                // 无复用
  HAL_GPIO_Init(LIMIT4_ORI_GPIO_PORT, &GPIO_InitStruct);  
  
  /* 正转限位开关 */
  LIMIT4_POS_RCC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin  = LIMIT4_POS_PIN;
  GPIO_InitStruct.Mode = LIMIT4_POS_ACTIVE_EDGE; //有效边沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = 0;                // 无复用
  HAL_GPIO_Init(LIMIT4_POS_GPIO_PORT, &GPIO_InitStruct);  
  
  /* 反转限位开关 */
  LIMIT4_NEG_RCC_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin  = LIMIT4_NEG_PIN;
  GPIO_InitStruct.Mode = LIMIT4_NEG_ACTIVE_EDGE; //有效边沿中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = 0;                // 无复用
  HAL_GPIO_Init(LIMIT4_NEG_GPIO_PORT, &GPIO_InitStruct);  
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
