/**
  ******************************************************************************
  * @file    bsp_usartx.h
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

#ifndef __BSP_USARTX_H__
#define	__BSP_USARTX_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define MX_RS485_UARTx                              UART4
#define MX_RS485_UARTx_BAUDRATE                     115200
#define MX_RS485_UART_RCC_CLK_ENABLE()              __HAL_RCC_UART4_CLK_ENABLE()
#define MX_RS485_UARTx_GPIO_AFx                     GPIO_AF8_UART4

#define MX_RS485_UARTx_Tx_GPIO_ClK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define MX_RS485_UARTx_Tx_GPIO_PIN                  GPIO_PIN_10
#define MX_RS485_UARTx_Tx_GPIO                      GPIOC

#define MX_RS485_UARTx_Rx_GPIO_ClK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define MX_RS485_UARTx_Rx_GPIO_PIN                  GPIO_PIN_11
#define MX_RS485_UARTx_Rx_GPIO                      GPIOC

#define MX_RS485_UARTx_IRQn                         UART4_IRQn
#define MX_RS485_UARTx_IRQHandler                   UART4_IRQHandler



#define USARTx_CTRL_GPIO_ClK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
#define USARTx_CTRL_PIN                        GPIO_PIN_2
#define USARTx_CTRL_GPIO                       GPIOB

#define RX_MODE()                              HAL_GPIO_WritePin(USARTx_CTRL_GPIO,USARTx_CTRL_PIN,GPIO_PIN_RESET)
#define TX_MODE()                              HAL_GPIO_WritePin(USARTx_CTRL_GPIO,USARTx_CTRL_PIN,GPIO_PIN_SET)

/* 扩展变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef huartx_RS485;

/* 函数声明 ------------------------------------------------------------------*/
void MX_RS485_USARTx_Init(void);


#endif /* __BSP_USARTX_H__ */

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
