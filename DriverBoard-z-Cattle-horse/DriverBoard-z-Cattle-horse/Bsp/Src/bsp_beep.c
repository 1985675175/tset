/**
  ******************************************************************************
  * @file    bsp_beep.c
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
#include "bsp_beep.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ���ط�����IO���ų�ʼ��.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����ʹ�ú궨�巽������������źţ����������ֲ��ֻҪ���޸�bsp_beep.h
  *           �ļ���غ궨��Ϳ��Է����޸����š�
  */
void BEEP_GPIO_Init(void)
{
   /* ����IOӲ����ʼ���ṹ����� */
  GPIO_InitTypeDef GPIO_InitStruct;
	
	/* ʹ��(����)���������Ŷ�ӦIO�˿�ʱ�� */  
  BEEP_RCC_CLK_ENABLE();
  
  /* �����������Ϊ�͵�ƽ����ʱ���������� */
  HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_RESET);  
  
  /* �趨��������Ӧ����IO��� */
  GPIO_InitStruct.Pin = BEEP_GPIO_PIN;
  /* �趨��������Ӧ����IOΪ���ģʽ */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  /* �趨��������Ӧ����IO�����ٶ� */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  /* ��ʼ����������Ӧ����IO */
  HAL_GPIO_Init(BEEP_GPIO_PORT, &GPIO_InitStruct);  
}

/**
  * ��������: ���ð��ط�������״̬
  * ���������state:���÷�������״̬��
  *             ��ѡֵ��BEEPState_OFF�����������죻
  *             ��ѡֵ��BEEPState_ON�� �������졣
  * �� �� ֵ: ��
  * ˵    �����ú���ʹ������HALA�⺯���ı�̷������������HAL�⺯�����˼�롣
  */
void BEEP_StateSet(BEEPState_TypeDef state)
{
  /* �����������Ƿ�Ϸ� */
  assert_param(BEEPState_TypeDef(state));
   
  /* �ж����õķ�����״̬���������Ϊ�������� */
  if(state==BEEPState_ON)
  {    
    BEEP_ON();/* �������� */
  }
  else /* state=BEEPState_OFF�����÷��������� */
  {    
    BEEP_OFF();/* ���������� */
  }
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
