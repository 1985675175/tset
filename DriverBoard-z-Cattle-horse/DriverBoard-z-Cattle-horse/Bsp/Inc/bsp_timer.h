/**
  ******************************************************************************
  * @file    bsp_timer.h
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

#ifndef __BSP_TIMER_H_
#define __BSP_TIMER_H_

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/

typedef struct
{
	uint8_t trigger;//定时器触发使能
	uint8_t flag;//定时器有效标志，做事件驱动用
	uint8_t mode;//计数模式：0：无限循环；1：单次；其他：多次
	uint32_t repeat_count;//多次计数数值
	uint32_t repeat_value;//多次计数目标值
	uint32_t count;//计数数值
	uint32_t value;//计数目标值
}TIMER_TypeDef;

/* 宏定义 --------------------------------------------------------------------*/

#define TMR_NUM_DELAY 	    0//延时等待
#define TMR_NUM_STANDBY 	1//待机
#define TMR_NUM_SLEEP 	    2//休眠
#define TMR_NUM_MAX 	    3

/* GPIO初始化定义 */

/* 私有变量 ------------------------------------------------------------------*/
extern TIMER_TypeDef Timer[TMR_NUM_MAX];

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void TMR_SetInit(TIMER_TypeDef *ptmr, uint8_t mode, uint32_t repeat_value, uint32_t value);
void TMR_Reset(TIMER_TypeDef *ptmr);
void TMR_SetTrigger(TIMER_TypeDef *ptmr, uint8_t trigger);
void TMR_SetCount(TIMER_TypeDef *ptmr, uint32_t repeat_count, uint32_t count);
uint8_t TMR_Getleave(TIMER_TypeDef *ptmr, uint32_t *pleave);
void TMR_RunTime(TIMER_TypeDef *ptmr);

#endif /* __BSP_TIMER_H_ */

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
