/**
  ******************************************************************************
  * @file    bsp_signal.c
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
#include "bsp_signal.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/

/* ˽�б��� ------------------------------------------------------------------*/
SIGNAL_TypeDef Signal[SIG_NUM_MAX] = \
								{1,0,SIG_DOOR_VALID_LEVEL,~SIG_DOOR_VALID_LEVEL,!SIG_DOOR_VALID_LEVEL,0,SIG_DOOR_REPEAT_FILT,
								 1,0,SIG_SLOT_VALID_LEVEL,~SIG_SLOT_VALID_LEVEL,!SIG_SLOT_VALID_LEVEL,0,SIG_SLOT_REPEAT_FILT};
/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/

/**
  * ��������: �����ź�IO���ų�ʼ��.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����ʹ�ú궨�巽������������źţ����������ֲ��ֻҪ���޸�bsp_signal.h
  *           �ļ���غ궨��Ϳ��Է����޸����š�
  */
void SIG_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct={0};
	
	/* ʹ��(����)SIG���Ŷ�ӦIO�˿�ʱ�� */  
  SIG_DOOR_RCC_CLK_ENABLE();
  SIG_SLOT_RCC_CLK_ENABLE();
  
/************************* ����SIG1 GPIO:��������ģʽ *************************/
  GPIO_InitStruct.Pin = SIG_DOOR_GPIO_PIN;   // ������I/O���
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// ����ģʽ 
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // ������Ч��ƽ�ǵ͵�ƽ��ʱ��������  
  HAL_GPIO_Init(SIG_DOOR_GPIO_PORT, &GPIO_InitStruct);  

/************************* ����SIG2 GPIO:��������ģʽ *************************/
  GPIO_InitStruct.Pin = SIG_SLOT_GPIO_PIN;   // ������I/O���
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// ����ģʽ 
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // ������Ч��ƽ�ǵ͵�ƽ��ʱ��������  
  HAL_GPIO_Init(SIG_SLOT_GPIO_PORT, &GPIO_InitStruct);  
}

/**
  * ��������: �����źų�ʼ����
  * ���������valid_level ��Ч��ƽ,repeat_filt �˲�����
  * �� �� ֵ: ��
  * ˵    ������Ĭ�ϳ�ʼ������������Ҫ��ʱ��
  */
void SIG_SetInit(SIGNAL_TypeDef *psig, uint8_t valid_level, uint8_t repeat_filt)
{
  psig->valid_level = valid_level;
  psig->repeat_filt = repeat_filt;
}

/**
  * ��������: ��λ�ź�
  * ���������psig �ź�ָ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void SIG_Reset(SIGNAL_TypeDef *psig)
{
  psig->trigger = 0;
  psig->flag = 0;
  psig->last_level = !psig->valid_level;
  psig->repeat_count = 0;
}

/**
  * ��������: ���ô���ʹ��
  * ���������psig �ź�ָ��,trigger ����ʹ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void SIG_SetTrigger(SIGNAL_TypeDef *psig, uint8_t trigger)
{
  psig->trigger = trigger;
}

/**
  * ��������: ��ȡ�źŵĵ�ƽ
  * ���������psig �ź�ָ��,level ���ŵ�ƽ
  * �� �� ֵ: ��
  * ˵    ������
  */
void SIG_SetLevel(SIGNAL_TypeDef *psig, uint8_t level)
{
  psig->current_level = level;
}

/**
  * ��������: ��ȡ�źŵĵ�ƽ
  * ���������psig �ź�ָ��,num ���ű��
  * �� �� ֵ: ��
  * ˵    ������
  */
void SIG_SetLevelFromNum(SIGNAL_TypeDef *psig, uint8_t num)
{
  uint8_t level;//���ŵ�ƽ
	switch(num)
	{
		case SIG_NUM_DOOR:
			level = HAL_GPIO_ReadPin(SIG_DOOR_GPIO_PORT, SIG_DOOR_GPIO_PIN);
			break;
		case SIG_NUM_SLOT:
			level = HAL_GPIO_ReadPin(SIG_SLOT_GPIO_PORT, SIG_SLOT_GPIO_PIN);
			break;
		default:
			return;
	}
	psig->current_level = level;
}

/**
  * ��������: �����ź�
  * ���������psig �ź�ָ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void SIG_RunTime(SIGNAL_TypeDef *psig)
{
  if(psig->trigger)
	{
		//�˲�����
		if(psig->current_level != psig->last_level)
		{
			psig->repeat_count++;
			if(psig->repeat_count >= psig->repeat_filt)
			{
				psig->repeat_count = 0;
				psig->last_level = psig->current_level;
//				if(psig->last_level == psig->valid_level)
					psig->flag = 1;//��ǰ�ź���Ч
			}
		}
		else
		{
			psig->repeat_count = 0;
		}
	}
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
