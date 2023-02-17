/**
  ******************************************************************************
  * @file    bsp_led.c
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

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_led.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/

/* ˽�б��� ------------------------------------------------------------------*/

/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����LED��IO���ų�ʼ��.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����ʹ�ú궨�巽������������źţ����������ֲ��ֻҪ���޸�bsp_led.h
  *           �ļ���غ궨��Ϳ��Է����޸����š�
  */
void LED_GPIO_Init(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* ʹ��LED���Ŷ�Ӧ��GPIO�˿�ʱ�� */
  LED1_RCC_GPIO_CLK_ENABLE();
  LED2_RCC_GPIO_CLK_ENABLE();
  LED3_RCC_GPIO_CLK_ENABLE();

  /* ����LED1����Ĭ�������ѹ */
  HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET);

  /*����GPIO���� */
  GPIO_InitStruct.Pin = LED1_PIN;             // I/O���
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // ����ģʽ���������
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // ��ʹ������/�������
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;// I/O�����ٶ� ��
  /* ������������ó�ʼ��LED������ */
  HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStruct);
  
  /*����GPIO���� */
  GPIO_InitStruct.Pin = LED2_PIN;             // I/O���
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // ����ģʽ���������
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // ��ʹ������/�������
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;// I/O�����ٶ� ��
  /* ������������ó�ʼ��LED������ */
  HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);

  /*����GPIO���� */
  GPIO_InitStruct.Pin = LED3_PIN;             // I/O���
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // ����ģʽ���������
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // ��ʹ������/�������
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;// I/O�����ٶ� ��
  /* ������������ó�ʼ��LED������ */
  HAL_GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStruct);
}


/**
  * ��������: ���ð���LED�Ƶ�״̬
  * �������: LEDx:����x������Ϊ(1,2,3)����ѡ���Ӧ��LED��
  * ���������state:����LED�Ƶ����״̬��
  *             ��ѡֵ��LED_OFF��LED����
  *                     LED_ON�� LED������
  *                     LED_TOGGLE����תLED
  * �� �� ֵ: ��
  * ˵    �����ú���ʹ������HAL�⺯���ı�̷������������HAL�⺯�����˼�롣
  */
void LEDx_StateSet(uint8_t LEDx,LEDState_TypeDef state)
{
  uint8_t led ;
  
  /* �����������Ƿ�Ϸ� */
  assert_param(IS_LED_TYPEDEF(LEDx));
  assert_param(IS_LED_STATE(state));
  
  /* �ж����õ�LED��״̬*/
  switch(state)
  {
    /* ����LED */
    case LED_ON:
      led = LEDx & LED1;
      if(led == LED1)
        LED1_ON();
      
      led = LEDx & LED2;
      if(led == LED2)
        LED2_ON();
      
      led = LEDx & LED3;
      if(led == LED3)
        LED3_OFF();
      break;
      
    /* Ϩ��LED */
    case LED_OFF:
      led = LEDx & LED1;
      if(led == LED1)            
        LED1_OFF();
      
      led = LEDx & LED2;
      if(LEDx & LED2)
        LED2_OFF();
      
      led = LEDx & LED2;
      if(LEDx & LED3)
        LED3_OFF();
      break;
    
    /* ��תLED */      
    case LED_TOGGLE:
      led = LEDx & LED1;
      if(led == LED1)            
        LED1_TOGGLE();/* �������������ת */ 
      
      led = LEDx & LED2;
      if(LEDx & LED2)
        LED2_TOGGLE();
      
      led = LEDx & LED2;
      if(LEDx & LED3)
        LED3_TOGGLE();
      break;
  }
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
