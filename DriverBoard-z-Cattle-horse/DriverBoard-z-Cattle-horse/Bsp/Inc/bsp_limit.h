/**
  ******************************************************************************
  * @file    bsp_limit.h
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

#ifndef __BSP_LIMIT1_H_
#define __BSP_LIMIT1_H_

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "bsp_stepmotor.h"

/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/

/* SMotor1 GPIO初始化定义 */
#define LIMIT1_ORI_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LIMIT1_ORI_PIN                    GPIO_PIN_1
#define LIMIT1_ORI_GPIO_PORT              GPIOG
#define LIMIT1_ORI_ACTIVE_LEVEL           GPIO_PIN_SET      // 有效电平 低电平
#define LIMIT1_ORI_ACTIVE_EDGE            GPIO_MODE_IT_RISING_FALLING// 中断有效边沿 
#define LIMIT1_ORI_IRQn                   EXTI1_IRQn
#define LIMIT1_ORI_IRQHandler             EXTI1_IRQHandler

#define LIMIT1_POS_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LIMIT1_POS_PIN                    GPIO_PIN_2
#define LIMIT1_POS_GPIO_PORT              GPIOG
#define LIMIT1_POS_ACTIVE_EDGE            GPIO_MODE_IT_RISING// 有效边沿 下降沿
#define LIMIT1_POS_ACTIVE_LEVEL           GPIO_PIN_SET// 正极限有效电平
#define LIMIT1_POS_IRQn                   EXTI2_IRQn
#define LIMIT1_POS_IRQHandler             EXTI2_IRQHandler

#define LIMIT1_NEG_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LIMIT1_NEG_PIN                    GPIO_PIN_0
#define LIMIT1_NEG_GPIO_PORT              GPIOG
#define LIMIT1_NEG_ACTIVE_EDGE            GPIO_MODE_IT_RISING// 有效边沿 下降沿
#define LIMIT1_NEG_ACTIVE_LEVEL           GPIO_PIN_SET// 负极限有效电平
#define LIMIT1_NEG_IRQn                   EXTI0_IRQn
#define LIMIT1_NEG_IRQHandler             EXTI0_IRQHandler

/* SMotor2 GPIO初始化定义 */
#define LIMIT2_ORI_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LIMIT2_ORI_PIN                    GPIO_PIN_4
#define LIMIT2_ORI_GPIO_PORT              GPIOG
#define LIMIT2_ORI_ACTIVE_LEVEL           GPIO_PIN_RESET      // 有效电平 低电平
#define LIMIT2_ORI_ACTIVE_EDGE            GPIO_MODE_IT_RISING_FALLING// 中断有效边沿 
#define LIMIT2_ORI_IRQn                   EXTI4_IRQn
#define LIMIT2_ORI_IRQHandler             EXTI4_IRQHandler

#define LIMIT2_POS_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LIMIT2_POS_PIN                    GPIO_PIN_5
#define LIMIT2_POS_GPIO_PORT              GPIOG
#define LIMIT2_POS_ACTIVE_EDGE            GPIO_MODE_IT_RISING// 有效边沿 下降沿
#define LIMIT2_POS_ACTIVE_LEVEL           GPIO_PIN_SET// 正极限有效电平

#define LIMIT2_NEG_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LIMIT2_NEG_PIN                    GPIO_PIN_3
#define LIMIT2_NEG_GPIO_PORT              GPIOG
#define LIMIT2_NEG_ACTIVE_EDGE            GPIO_MODE_IT_RISING// 有效边沿 下降沿
#define LIMIT2_NEG_ACTIVE_LEVEL           GPIO_PIN_RESET// 负极限有效电平
#define LIMIT2_NEG_IRQn                   EXTI3_IRQn
#define LIMIT2_NEG_IRQHandler             EXTI3_IRQHandler

/* SMotor3 GPIO初始化定义 */
#define LIMIT3_ORI_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LIMIT3_ORI_PIN                    GPIO_PIN_7
#define LIMIT3_ORI_GPIO_PORT              GPIOG
#define LIMIT3_ORI_ACTIVE_LEVEL           GPIO_PIN_SET      // 有效电平 低电平
#define LIMIT3_ORI_ACTIVE_EDGE            GPIO_MODE_IT_RISING_FALLING// 中断有效边沿 

#define LIMIT3_POS_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LIMIT3_POS_PIN                    GPIO_PIN_8
#define LIMIT3_POS_GPIO_PORT              GPIOG
#define LIMIT3_POS_ACTIVE_EDGE            GPIO_MODE_IT_RISING// 有效边沿 下降沿
#define LIMIT3_POS_ACTIVE_LEVEL           GPIO_PIN_SET// 正极限有效电平

#define LIMIT3_NEG_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LIMIT3_NEG_PIN                    GPIO_PIN_6
#define LIMIT3_NEG_GPIO_PORT              GPIOG
#define LIMIT3_NEG_ACTIVE_EDGE            GPIO_MODE_IT_RISING// 有效边沿 下降沿
#define LIMIT3_NEG_ACTIVE_LEVEL           GPIO_PIN_SET// 负极限有效电平

/* SMotor4 GPIO初始化定义 */
#define LIMIT4_ORI_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LIMIT4_ORI_PIN                    GPIO_PIN_10
#define LIMIT4_ORI_GPIO_PORT              GPIOG
#define LIMIT4_ORI_ACTIVE_LEVEL           GPIO_PIN_RESET      // 有效电平 低电平
#define LIMIT4_ORI_ACTIVE_EDGE            GPIO_MODE_IT_RISING_FALLING// 中断有效边沿 

#define LIMIT4_POS_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LIMIT4_POS_PIN                    GPIO_PIN_15
#define LIMIT4_POS_GPIO_PORT              GPIOG
#define LIMIT4_POS_ACTIVE_EDGE            GPIO_MODE_IT_RISING// 有效边沿 下降沿
#define LIMIT4_POS_ACTIVE_LEVEL           GPIO_PIN_SET// 正极限有效电平

#define LIMIT4_NEG_RCC_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LIMIT4_NEG_PIN                    GPIO_PIN_9
#define LIMIT4_NEG_GPIO_PORT              GPIOG
#define LIMIT4_NEG_ACTIVE_EDGE            GPIO_MODE_IT_RISING// 有效边沿 下降沿
#define LIMIT4_NEG_ACTIVE_LEVEL           GPIO_PIN_RESET// 负极限有效电平

/* 多个引脚共用同一中断线 */
#define LIMIT_n_IRQn                      EXTI9_5_IRQn
#define LIMIT_n_IRQHandler                EXTI9_5_IRQHandler

#define LIMIT_m_IRQn                      EXTI15_10_IRQn
#define LIMIT_m_IRQHandler                EXTI15_10_IRQHandler
/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef  htim_SM3NEG;


/* 函数声明 ------------------------------------------------------------------*/
void Limit_GPIO_Init(void);

#endif /* __BSP_LIMIT1_H_ */

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
