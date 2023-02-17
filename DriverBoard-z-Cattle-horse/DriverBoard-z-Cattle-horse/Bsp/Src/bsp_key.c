/**
  ******************************************************************************
  * @file    bsp_key.c
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
#include "bsp_key.h"
#include "main.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 板载按键IO引脚初始化.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：使用宏定义方法代替具体引脚号，方便程序移植，只要简单修改bsp_key.h
  *           文件相关宏定义就可以方便修改引脚。
  */
void KEY_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct={0};
	
	/* 使能(开启)KEY引脚对应IO端口时钟 */  
  KEY1_RCC_CLK_ENABLE();
  KEY2_RCC_CLK_ENABLE();
  KEY3_RCC_CLK_ENABLE();
  KEY4_RCC_CLK_ENABLE();
  KEY5_RCC_CLK_ENABLE();
  
/************************* 配置KEY1 GPIO:输入上拉模式 *************************/
  GPIO_InitStruct.Pin = KEY1_GPIO_PIN;   // 按键的I/O编号
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// 输入模式 
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // 按键有效电平是低电平的时候是上拉  
  HAL_GPIO_Init(KEY1_GPIO, &GPIO_InitStruct);  

/************************* 配置KEY2 GPIO:输入上拉模式 *************************/
  GPIO_InitStruct.Pin = KEY2_GPIO_PIN;   // 按键的I/O编号
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// 输入模式 
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // 按键有效电平是低电平的时候是上拉
  HAL_GPIO_Init(KEY2_GPIO, &GPIO_InitStruct);  
  
/************************* 配置KE3 GPIO:输入上拉模式 *************************/
  GPIO_InitStruct.Pin = KEY3_GPIO_PIN;   // 按键的I/O编号
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// 输入模式
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // 按键有效电平是低电平的时候是上拉  
  HAL_GPIO_Init(KEY3_GPIO, &GPIO_InitStruct);  

/************************* 配置KEY4 GPIO:输入上拉模式 *************************/
  GPIO_InitStruct.Pin = KEY4_GPIO_PIN;   // 按键的I/O编号
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// 输入模式 
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // 按键有效电平是低电平的时候是上拉
  HAL_GPIO_Init(KEY4_GPIO, &GPIO_InitStruct);  

/************************* 配置KEY5 GPIO:输入上拉模式 *************************/
  GPIO_InitStruct.Pin = KEY5_GPIO_PIN;   // 按键的I/O编号
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// 输入模式 
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // 按键有效电平是低电平的时候是上拉
  HAL_GPIO_Init(KEY5_GPIO, &GPIO_InitStruct);  
}

/**
  * 函数功能: 读取按键KEY1的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下
  *           KEY_UP  ：按键没被按下
  * 说    明：无
  */
KEYState_TypeDef KEY1_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN) == KEY1_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 
     * 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    //HAL_Delay(30);
    if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN) == KEY1_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 
       * 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      while(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL);
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}

/**
  * 函数功能: 读取按键KEY2的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下
  *           KEY_UP  ：按键没被按下
  * 说    明：无
  */
KEYState_TypeDef KEY2_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN) == KEY2_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 
     * 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    //HAL_Delay(30);
    if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN) == KEY2_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 
       * 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      while(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL);
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}

/**
  * 函数功能: 读取按键KEY3的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下
  *           KEY_UP  ：按键没被按下
  * 说    明：无
  */
KEYState_TypeDef KEY3_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN) == KEY3_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 
     * 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    //HAL_Delay(30);
    if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN) == KEY3_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 
       * 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      while(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL);
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}
/**
  * 函数功能: 读取按键KEY4的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下
  *           KEY_UP  ：按键没被按下
  * 说    明：无
  */
KEYState_TypeDef KEY4_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN) == KEY4_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 
     * 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    //HAL_Delay(30);
    if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN) == KEY4_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 
       * 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      while(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL);
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}
/**
  * 函数功能: 读取按键KEY5的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下
  *           KEY_UP  ：按键没被按下
  * 说    明：无
  */
KEYState_TypeDef KEY5_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN) == KEY5_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 
     * 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    //HAL_Delay(30);
    if(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN) == KEY5_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 
       * 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      while(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN)==KEY5_DOWN_LEVEL);
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}
/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
