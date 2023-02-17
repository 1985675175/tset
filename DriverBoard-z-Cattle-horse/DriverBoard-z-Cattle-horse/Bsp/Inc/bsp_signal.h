/**
  ******************************************************************************
  * @file    bsp_signal.h
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

#ifndef __BSP_SIGNAL_H_
#define __BSP_SIGNAL_H_

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/

typedef struct
{
	uint8_t trigger;//信号触发使能
	uint8_t flag;//信号有效标志，做事件驱动用
	uint8_t valid_level;//有效电平
	uint8_t last_level;//上次电平
	uint8_t current_level;//本次电平
	uint8_t repeat_count;//当前电平保持次数
	uint8_t repeat_filt;//滤波次数，达到该次数，当前电平有效
}SIGNAL_TypeDef;

/* 宏定义 --------------------------------------------------------------------*/

#define SIG_NUM_DOOR 	0//门锁
#define SIG_NUM_SLOT 	1//液槽
#define SIG_NUM_MAX 	2

/* GPIO初始化定义 */
#define SIG_DOOR_RCC_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define SIG_DOOR_GPIO_PIN                 GPIO_PIN_12
#define SIG_DOOR_GPIO_PORT                GPIOB
#define SIG_DOOR_VALID_LEVEL              GPIO_PIN_SET
#define SIG_DOOR_REPEAT_FILT              (8)

#define SIG_SLOT_RCC_CLK_ENABLE()           __HAL_RCC_GPIOI_CLK_ENABLE()
#define SIG_SLOT_GPIO_PIN                 GPIO_PIN_4
#define SIG_SLOT_GPIO_PORT                GPIOI
#define SIG_SLOT_VALID_LEVEL              GPIO_PIN_SET
#define SIG_SLOT_REPEAT_FILT              (8)

/* 私有变量 ------------------------------------------------------------------*/
extern SIGNAL_TypeDef Signal[SIG_NUM_MAX];

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void SIG_GPIO_Init(void);
void SIG_SetInit(SIGNAL_TypeDef *psig, uint8_t valid_level, uint8_t repeat_filt);
void SIG_Reset(SIGNAL_TypeDef *psig);
void SIG_SetTrigger(SIGNAL_TypeDef *psig, uint8_t trigger);
void SIG_SetLevel(SIGNAL_TypeDef *psig, uint8_t level);
void SIG_SetLevelFromNum(SIGNAL_TypeDef *psig, uint8_t num);
void SIG_RunTime(SIGNAL_TypeDef *psig);

#endif /* __BSP_SIGNAL_H_ */

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
