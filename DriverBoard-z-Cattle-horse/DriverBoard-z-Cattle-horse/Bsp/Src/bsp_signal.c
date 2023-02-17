/**
  ******************************************************************************
  * @file    bsp_signal.c
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
#include "bsp_signal.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
SIGNAL_TypeDef Signal[SIG_NUM_MAX] = \
								{1,0,SIG_DOOR_VALID_LEVEL,~SIG_DOOR_VALID_LEVEL,!SIG_DOOR_VALID_LEVEL,0,SIG_DOOR_REPEAT_FILT,
								 1,0,SIG_SLOT_VALID_LEVEL,~SIG_SLOT_VALID_LEVEL,!SIG_SLOT_VALID_LEVEL,0,SIG_SLOT_REPEAT_FILT};
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 板载信号IO引脚初始化.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：使用宏定义方法代替具体引脚号，方便程序移植，只要简单修改bsp_signal.h
  *           文件相关宏定义就可以方便修改引脚。
  */
void SIG_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct={0};
	
	/* 使能(开启)SIG引脚对应IO端口时钟 */  
  SIG_DOOR_RCC_CLK_ENABLE();
  SIG_SLOT_RCC_CLK_ENABLE();
  
/************************* 配置SIG1 GPIO:输入上拉模式 *************************/
  GPIO_InitStruct.Pin = SIG_DOOR_GPIO_PIN;   // 按键的I/O编号
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// 输入模式 
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // 按键有效电平是低电平的时候是上拉  
  HAL_GPIO_Init(SIG_DOOR_GPIO_PORT, &GPIO_InitStruct);  

/************************* 配置SIG2 GPIO:输入上拉模式 *************************/
  GPIO_InitStruct.Pin = SIG_SLOT_GPIO_PIN;   // 按键的I/O编号
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// 输入模式 
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // 按键有效电平是低电平的时候是上拉  
  HAL_GPIO_Init(SIG_SLOT_GPIO_PORT, &GPIO_InitStruct);  
}

/**
  * 函数功能: 设置信号初始变量
  * 输入参数：valid_level 有效电平,repeat_filt 滤波次数
  * 返 回 值: 无
  * 说    明：在默认初始变量不能满足要求时用
  */
void SIG_SetInit(SIGNAL_TypeDef *psig, uint8_t valid_level, uint8_t repeat_filt)
{
  psig->valid_level = valid_level;
  psig->repeat_filt = repeat_filt;
}

/**
  * 函数功能: 复位信号
  * 输入参数：psig 信号指针
  * 返 回 值: 无
  * 说    明：无
  */
void SIG_Reset(SIGNAL_TypeDef *psig)
{
  psig->trigger = 0;
  psig->flag = 0;
  psig->last_level = !psig->valid_level;
  psig->repeat_count = 0;
}

/**
  * 函数功能: 设置触发使能
  * 输入参数：psig 信号指针,trigger 触发使能
  * 返 回 值: 无
  * 说    明：无
  */
void SIG_SetTrigger(SIGNAL_TypeDef *psig, uint8_t trigger)
{
  psig->trigger = trigger;
}

/**
  * 函数功能: 读取信号的电平
  * 输入参数：psig 信号指针,level 引脚电平
  * 返 回 值: 无
  * 说    明：无
  */
void SIG_SetLevel(SIGNAL_TypeDef *psig, uint8_t level)
{
  psig->current_level = level;
}

/**
  * 函数功能: 读取信号的电平
  * 输入参数：psig 信号指针,num 引脚编号
  * 返 回 值: 无
  * 说    明：无
  */
void SIG_SetLevelFromNum(SIGNAL_TypeDef *psig, uint8_t num)
{
  uint8_t level;//引脚电平
	switch(num)
	{
		case SIG_NUM_DOOR:
			level = HAL_GPIO_ReadPin(SIG_DOOR_GPIO_PORT, SIG_DOOR_GPIO_PIN);
			break;
		case SIG_NUM_SLOT:
			level = HAL_GPIO_ReadPin(SIG_SLOT_GPIO_PORT, SIG_SLOT_GPIO_PIN);
			break;
		default:
			return;
	}
	psig->current_level = level;
}

/**
  * 函数功能: 运行信号
  * 输入参数：psig 信号指针
  * 返 回 值: 无
  * 说    明：无
  */
void SIG_RunTime(SIGNAL_TypeDef *psig)
{
  if(psig->trigger)
	{
		//滤波处理
		if(psig->current_level != psig->last_level)
		{
			psig->repeat_count++;
			if(psig->repeat_count >= psig->repeat_filt)
			{
				psig->repeat_count = 0;
				psig->last_level = psig->current_level;
//				if(psig->last_level == psig->valid_level)
					psig->flag = 1;//当前信号有效
			}
		}
		else
		{
			psig->repeat_count = 0;
		}
	}
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
