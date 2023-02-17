/**
  ******************************************************************************
  * @file    bsp_lamp.c
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
#include "bsp_lamp.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/

/* ˽�б��� ------------------------------------------------------------------*/

/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����������IO���ų�ʼ��.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����ʹ�ú궨�巽������������źţ����������ֲ��ֻҪ���޸�bsp_lamp.h
  *           �ļ���غ궨��Ϳ��Է����޸����š�
  */
void LAMP_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* ʹ��LAMP���Ŷ�Ӧ��GPIO�˿�ʱ�� */
  LAMP_RCC_GPIO_CLK_ENABLE();

  /* ����LAMP����Ĭ�������ѹ */
  HAL_GPIO_WritePin(LAMP_GPIO_PORT, LAMP_GPIO_PIN, GPIO_PIN_RESET);

  /*����GPIO���� */
  GPIO_InitStruct.Pin = LAMP_GPIO_PIN;             // I/O���
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // ����ģʽ����©���
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // ��ʹ������/�������
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;// I/O�����ٶ� ��
  /* ������������ó�ʼ��LAMP������ */
  HAL_GPIO_Init(LAMP_GPIO_PORT, &GPIO_InitStruct);
}


/**
  * ��������: ���ð��������Ƶ�״̬
  * ���������state:���������Ƶ����״̬��
  *             ��ѡֵ��LAMP_OFF����������
  *                     LAMP_ON�� ����������
  *                     LAMP_TOGGLE����ת������
  * �� �� ֵ: ��
  * ˵    �����ú���ʹ������HAL�⺯���ı�̷������������HAL�⺯�����˼�롣
  */
void LAMP_StateSet(LAMPState_TypeDef state)
{
  /* �����������Ƿ�Ϸ� */
  assert_param(IS_LAMP_STATE(state));
  
  /* �ж����õ�������״̬*/
  switch(state)
  {
    /* ���������� */
    case LAMP_ON:
      LAMP_ON();
      break;
    /* Ϩ�������� */
    case LAMP_OFF:
      LAMP_OFF();
      break;
    /* ��ת������ */      
    case LAMP_TOGGLE:
      LAMP_TOGGLE();
      break;
  }
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
