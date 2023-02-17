/**
  ******************************************************************************
  * @file    stm32f4xx_assert.c
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


/**
  *  ���ļ��ṩ���Ժ���,��Ҫ���ڳ�����ԡ�HAL���еĺ��������Զ�����������м�飬��߳���Ľ�׳�ԡ�
  *  ���ļ���������촴�����ļ���������HAL����ļ���HAL��ķ����ǽ��ⲿ�ֺ�������main.c�ļ���
  */

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"	
#include <stdio.h>

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/* 
 *	HAL�⺯��ʹ����C�������Ķ��Թ��ܣ����������USE_FULL_ASSERT(��
 *  stm32f4xx_hal_conf.h���壬Ĭ�ϱ�ע�͵�)����ô���е�HAL�⺯������麯���β���
 *  ����ȷ���������ȷ������ assert_failed() ���������������һ����ѭ����������
 *  �������롣
 *
 *	�ؼ��� __LINE__ ��ʾԴ�����кš�
 *	�ؼ���__FILE__��ʾԴ�����ļ�����
 *
 *  ���Թ���ʹ�ܺ���������С���Ƽ��û����ڵ���ʱʹ�ܣ�����ʽ��������ǽ�ֹ��
 *
 *	�û�����ѡ���Ƿ�ʹ��HAL��Ķ��Թ��ܡ�ʹ�ܶ��Եķ��������֣�
 * 	(1) ��C��������Ԥ�����ѡ���ж���USE_FULL_ASSERT��
 *	(2) ��stm32f4xx_hal_conf.h�ļ�ȡ��"#define USE_FULL_ASSERT    1"�е�ע�͡�	
*/
#ifdef USE_FULL_ASSERT

/**
  * ��������: ����ʧ�ܷ�����
  * �������: file : Դ�����ļ����ơ��ؼ���__FILE__��ʾԴ�����ļ�����
  *           line �������кš��ؼ��� __LINE__ ��ʾԴ�����к�
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* 
	 *	�û���������Լ��Ĵ��뱨��Դ�����ļ����ʹ����кţ����罫�����ļ����кŴ�ӡ������
	 *	printf("Wrong parameters value: file %s on line %d\r\n", file, line)
	 */
	
	/* ����һ����ѭ��������ʧ��ʱ������ڴ˴��������Ա����û���� */
	while (1)
	{
	}
}
#endif
/************************ (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
