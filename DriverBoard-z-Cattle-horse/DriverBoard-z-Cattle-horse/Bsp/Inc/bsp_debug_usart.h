/**
  ******************************************************************************
  * @file    bsp_debug_usart.h
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
	 
#ifndef __BSP_DEBUG_USART_H__
#define __BSP_DEBUG_USART_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define DEBUG_USARTx                                 UART5
#define DEBUG_USARTx_BAUDRATE                        19200
#define DEBUG_USART_RCC_CLK_ENABLE()                 __HAL_RCC_UART5_CLK_ENABLE()
#define DEBUG_USART_RCC_CLK_DISABLE()                __HAL_RCC_UART5_CLK_DISABLE()

#define DEBUG_USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()
#define DEBUG_USARTx_Tx_GPIO_PIN                     GPIO_PIN_12
#define DEBUG_USARTx_Tx_GPIO                         GPIOC
#define DEBUG_USARTx_Rx_GPIO_PIN                     GPIO_PIN_2
#define DEBUG_USARTx_Rx_GPIO                         GPIOD

#define DEBUG_USARTx_AFx                             GPIO_AF8_UART5

#define DEBUG_USART_IRQn                             UART5_IRQn
#define DEBUG_USART_IRQHANDLER                       UART5_IRQHandler

/* 使用485通信的时候才会用到使能IO */
#define RS485_REDE_GPIO_ClK_ENABLE()                 __HAL_RCC_GPIOC_CLK_ENABLE()
#define RS485_REDE_PORT                              GPIOC
#define RS485_REDE_PIN                               GPIO_PIN_0
#define RS485_RX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_RESET)
#define RS485_TX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_SET)

/* RTU通信需要确定超时时间 */
#if DEBUG_USARTx_BAUDRATE <= 19200
  /* 1.5个字符的超时时间 T = BAUDRATE/11/1000*/
  #define OVERTIME_15CHAR             (((float)DEBUG_USARTx_BAUDRATE/11)*1.5f)
  /* 3个字符的超时时间 */
  #define OVERTIME_35CHAR             (((float)DEBUG_USARTx_BAUDRATE/11)*3.5f)
#else 
  /* 波特率超过19200bit/s的情况下建议的超时时间 */
  #define OVERTIME_15CHAR                750.0f    // 750us 
  #define OVERTIME_35CHAR               1750.0f  // 1.75ms  
  
#endif
/* 扩展变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef husart_debug;
extern __IO uint8_t Rx_Buf[256];    // 接收缓存,最大256字节
extern __IO uint8_t Tx_Buf[256];    // 接收缓存,最大256字节
extern __IO uint8_t tmp_Rx_Buf;     // 接收缓存
extern  __IO uint16_t RxCount;      // 接收字符计数
  
/* 函数声明 ------------------------------------------------------------------*/
void MX_DEBUG_USART_Init(void);
void UART_Tx(uint8_t *Tx_Buf,uint16_t TxCount);

#endif  /* __BSP_DEBUG_USART_H__ */

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
