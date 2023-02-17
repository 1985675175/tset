/**
  ******************************************************************************
  * @file    bsp_key.c
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
#include "bsp_key.h"
#include "main.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/

/* ˽�б��� ------------------------------------------------------------------*/

/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ���ذ���IO���ų�ʼ��.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����ʹ�ú궨�巽������������źţ����������ֲ��ֻҪ���޸�bsp_key.h
  *           �ļ���غ궨��Ϳ��Է����޸����š�
  */
void KEY_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct={0};
	
	/* ʹ��(����)KEY���Ŷ�ӦIO�˿�ʱ�� */  
  KEY1_RCC_CLK_ENABLE();
  KEY2_RCC_CLK_ENABLE();
  KEY3_RCC_CLK_ENABLE();
  KEY4_RCC_CLK_ENABLE();
  KEY5_RCC_CLK_ENABLE();
  
/************************* ����KEY1 GPIO:��������ģʽ *************************/
  GPIO_InitStruct.Pin = KEY1_GPIO_PIN;   // ������I/O���
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// ����ģʽ 
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // ������Ч��ƽ�ǵ͵�ƽ��ʱ��������  
  HAL_GPIO_Init(KEY1_GPIO, &GPIO_InitStruct);  

/************************* ����KEY2 GPIO:��������ģʽ *************************/
  GPIO_InitStruct.Pin = KEY2_GPIO_PIN;   // ������I/O���
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// ����ģʽ 
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // ������Ч��ƽ�ǵ͵�ƽ��ʱ��������
  HAL_GPIO_Init(KEY2_GPIO, &GPIO_InitStruct);  
  
/************************* ����KE3 GPIO:��������ģʽ *************************/
  GPIO_InitStruct.Pin = KEY3_GPIO_PIN;   // ������I/O���
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// ����ģʽ
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // ������Ч��ƽ�ǵ͵�ƽ��ʱ��������  
  HAL_GPIO_Init(KEY3_GPIO, &GPIO_InitStruct);  

/************************* ����KEY4 GPIO:��������ģʽ *************************/
  GPIO_InitStruct.Pin = KEY4_GPIO_PIN;   // ������I/O���
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// ����ģʽ 
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // ������Ч��ƽ�ǵ͵�ƽ��ʱ��������
  HAL_GPIO_Init(KEY4_GPIO, &GPIO_InitStruct);  

/************************* ����KEY5 GPIO:��������ģʽ *************************/
  GPIO_InitStruct.Pin = KEY5_GPIO_PIN;   // ������I/O���
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// ����ģʽ 
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // ������Ч��ƽ�ǵ͵�ƽ��ʱ��������
  HAL_GPIO_Init(KEY5_GPIO, &GPIO_InitStruct);  
}

/**
  * ��������: ��ȡ����KEY1��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN������������
  *           KEY_UP  ������û������
  * ˵    ������
  */
KEYState_TypeDef KEY1_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN) == KEY1_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� 
     * ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    //HAL_Delay(30);
    if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN) == KEY1_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� 
       * ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      while(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL);
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}

/**
  * ��������: ��ȡ����KEY2��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN������������
  *           KEY_UP  ������û������
  * ˵    ������
  */
KEYState_TypeDef KEY2_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN) == KEY2_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� 
     * ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    //HAL_Delay(30);
    if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN) == KEY2_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� 
       * ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      while(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL);
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}

/**
  * ��������: ��ȡ����KEY3��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN������������
  *           KEY_UP  ������û������
  * ˵    ������
  */
KEYState_TypeDef KEY3_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN) == KEY3_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� 
     * ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    //HAL_Delay(30);
    if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN) == KEY3_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� 
       * ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      while(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL);
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}
/**
  * ��������: ��ȡ����KEY4��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN������������
  *           KEY_UP  ������û������
  * ˵    ������
  */
KEYState_TypeDef KEY4_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN) == KEY4_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� 
     * ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    //HAL_Delay(30);
    if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN) == KEY4_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� 
       * ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      while(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL);
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}
/**
  * ��������: ��ȡ����KEY5��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN������������
  *           KEY_UP  ������û������
  * ˵    ������
  */
KEYState_TypeDef KEY5_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN) == KEY5_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� 
     * ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    //HAL_Delay(30);
    if(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN) == KEY5_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� 
       * ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      while(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN)==KEY5_DOWN_LEVEL);
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}
/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
