/**
  ******************************************************************************
  * @file    bsp_usartx.c
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
#include "bsp_usartx.h"
#include <string.h>
#include <stdio.h>

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
UART_HandleTypeDef huartx_RS485;
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 串口硬件初始化配置
  * 输入参数: huart：串口句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_RS485_UART_MspInit(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

	/* 使能串口功能引脚GPIO时钟 */
	MX_RS485_UARTx_Rx_GPIO_ClK_ENABLE();
	MX_RS485_UARTx_Tx_GPIO_ClK_ENABLE();
	USARTx_CTRL_GPIO_ClK_ENABLE();
	/* 串口外设功能GPIO配置 */
	GPIO_InitStruct.Pin = MX_RS485_UARTx_Tx_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = MX_RS485_UARTx_GPIO_AFx;
	HAL_GPIO_Init(MX_RS485_UARTx_Tx_GPIO, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = MX_RS485_UARTx_Rx_GPIO_PIN;
	HAL_GPIO_Init(MX_RS485_UARTx_Rx_GPIO, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = USARTx_CTRL_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(USARTx_CTRL_GPIO, &GPIO_InitStruct);		

	  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MX_RS485_UARTx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MX_RS485_UARTx_IRQn);
}

/**
  * 函数功能: 串口参数配置.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void MX_RS485_USARTx_Init(void)
{
	HAL_RS485_UART_MspInit();
  /* 串口外设时钟使能 */
  MX_RS485_UART_RCC_CLK_ENABLE();
  
  huartx_RS485.Instance = MX_RS485_UARTx;
  huartx_RS485.Init.BaudRate = MX_RS485_UARTx_BAUDRATE;
  huartx_RS485.Init.WordLength = UART_WORDLENGTH_8B;
  huartx_RS485.Init.StopBits = UART_STOPBITS_1;
  huartx_RS485.Init.Parity = UART_PARITY_NONE;
  huartx_RS485.Init.Mode = UART_MODE_TX_RX;
  huartx_RS485.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huartx_RS485.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huartx_RS485);

}

/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
//int fputc(int ch, FILE *f)
//{
//  HAL_UART_Transmit(&husartx, (uint8_t *)&ch, 1, 0xffff);
//  return ch;
//}

///**
//  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
//  * 输入参数: 无
//  * 返 回 值: 无
//  * 说    明：无
//  */
//int fgetc(FILE * f)
//{
//  uint8_t ch = 0;
//  HAL_UART_Receive(&husartx,&ch, 1, 0xffff);
//  return ch;
//}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
