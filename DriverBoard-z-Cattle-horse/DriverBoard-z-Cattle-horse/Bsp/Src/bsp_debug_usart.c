/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
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
	 
#include "bsp_debug_usart.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
UART_HandleTypeDef husart_debug;
__IO uint8_t Rx_Buf[256];            // ���ջ���,���256�ֽ�
__IO uint8_t Tx_Buf[256];            // ���ͻ���,���256�ֽ�
__IO uint8_t tmp_Rx_Buf;             // ��ʱ���ջ���
__IO uint16_t RxCount = 0;      // �����ַ�����
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����Ӳ����ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  if(huart->Instance==DEBUG_USARTx)
  {
    /* ��������ʱ��ʹ�� */
    DEBUG_USART_RCC_CLK_ENABLE();
    RS485_REDE_GPIO_ClK_ENABLE();
    /* �������蹦��GPIO���� */
    GPIO_InitStruct.Pin = DEBUG_USARTx_Tx_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = DEBUG_USARTx_AFx;
    HAL_GPIO_Init(DEBUG_USARTx_Tx_GPIO, &GPIO_InitStruct);
//		HAL_GPIO_WritePin(DEBUG_USARTx_Tx_GPIO,DEBUG_USARTx_Tx_GPIO_PIN,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(DEBUG_USARTx_Tx_GPIO,DEBUG_USARTx_Tx_GPIO_PIN,GPIO_PIN_SET);
    
    GPIO_InitStruct.Pin = DEBUG_USARTx_Rx_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = DEBUG_USARTx_AFx;    
    HAL_GPIO_Init(DEBUG_USARTx_Rx_GPIO, &GPIO_InitStruct);    
    
    /* SP3485E��������ʹ�ܿ������ų�ʼ�� */
    HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = RS485_REDE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_REDE_PORT, &GPIO_InitStruct);
  }  
}

/**
  * ��������: ����Ӳ������ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==DEBUG_USARTx)
  {
    /* ��������ʱ�ӽ��� */
    DEBUG_USART_RCC_CLK_DISABLE();
  
    /* �������蹦��GPIO���� */
    HAL_GPIO_DeInit(DEBUG_USARTx_Tx_GPIO, DEBUG_USARTx_Tx_GPIO_PIN);
    HAL_GPIO_DeInit(DEBUG_USARTx_Rx_GPIO, DEBUG_USARTx_Rx_GPIO_PIN);
    
    /* �����жϽ��� */
    HAL_NVIC_DisableIRQ(DEBUG_USART_IRQn);
  }
}

/**
  * ��������: ���ڲ�������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void MX_DEBUG_USART_Init(void)
{
  /* ʹ�ܴ��ڹ�������GPIOʱ�� */
  DEBUG_USARTx_GPIO_ClK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  husart_debug.Instance = DEBUG_USARTx;
  husart_debug.Init.BaudRate = DEBUG_USARTx_BAUDRATE;
  husart_debug.Init.WordLength = UART_WORDLENGTH_8B;
  husart_debug.Init.StopBits = UART_STOPBITS_1;
  husart_debug.Init.Parity = UART_PARITY_NONE;
  husart_debug.Init.Mode = UART_MODE_TX_RX;
  husart_debug.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husart_debug.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husart_debug);
  
  HAL_NVIC_SetPriority(DEBUG_USART_IRQn,1,1);
  HAL_NVIC_EnableIRQ(DEBUG_USART_IRQn);
  
  /* �����ж�,�洢����ʱ������ */ 
  RS485_RX_MODE();
  HAL_UART_Receive_IT(&husart_debug,(uint8_t*)&tmp_Rx_Buf,1);
}
/**
  * ��������: ���ڷ��ͺ���
  * �������: Tx_Bul:�����ַ���,TxCount:���͵��ֽ���
  * �� �� ֵ: ��
  * ˵    ��: ���⺯�������װ.
  */
void UART_Tx(uint8_t *Tx_Buf,uint16_t TxCount)
{
  RS485_TX_MODE();
  HAL_UART_Transmit(&husart_debug, Tx_Buf, TxCount, 0xffff);
  RS485_RX_MODE();
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
