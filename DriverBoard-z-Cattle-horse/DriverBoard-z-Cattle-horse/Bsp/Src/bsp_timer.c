/**
  ******************************************************************************
  * @file    bsp_timer.c
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
#include "bsp_timer.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/

/* ˽�б��� ------------------------------------------------------------------*/
TIMER_TypeDef Timer[TMR_NUM_MAX] = \
								{0,0,1,0,0,100,100,//��ʱ�ȴ�
								 1,0,0,30,30,30000,30000,//30S����
								 1,0,0,60,60,120000,120000,//120S����
                                };
/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/


/**
  * ��������: ���ö�ʱ����ʼ����
  * ���������ptmr ��ʱ��ָ�룻mode ����ģʽ,repeat_value ��μ���Ŀ��ֵ,value ����Ŀ��ֵ
  * �� �� ֵ: ��
  * ˵    ������Ĭ�ϳ�ʼ������������Ҫ��ʱ��
  */
void TMR_SetInit(TIMER_TypeDef *ptmr, uint8_t mode, uint32_t repeat_value, uint32_t value)
{
  ptmr->mode = mode;
  ptmr->repeat_value = repeat_value;
  ptmr->value = value;
}

/**
  * ��������: ��λ��ʱ��
  * ���������ptmr ��ʱ��ָ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void TMR_Reset(TIMER_TypeDef *ptmr)
{
  ptmr->trigger = 0;
  ptmr->flag = 0;
  if(ptmr->mode == 0)
    ptmr->count = ptmr->value;
  else if(ptmr->mode == 1)
    ptmr->count = ptmr->value;
  else
  {
    ptmr->repeat_count = ptmr->repeat_value;
    ptmr->count = ptmr->value;
  }
}

/**
  * ��������: ���ô���ʹ��
  * ���������ptmr ��ʱ��ָ��,trigger ����ʹ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void TMR_SetTrigger(TIMER_TypeDef *ptmr, uint8_t trigger)
{
  ptmr->trigger = trigger;
}

/**
  * ��������: ���ö�ʱ���ĳ�ʼ����ֵ
  * ���������ptmr ��ʱ��ָ��,repeat_count ��μ�����ֵ, count ������ֵ
  * �� �� ֵ: ��
  * ˵    ������
  */
void TMR_SetCount(TIMER_TypeDef *ptmr, uint32_t repeat_count, uint32_t count)
{
  ptmr->repeat_count = repeat_count;
  ptmr->count = count;
}

/**
  * ��������: ��ȡ��ʱ��ʣ��ʱ��
  * ���������ptmr ��ʱ��ָ��,pleave ʣ��ʱ��
  * �� �� ֵ: 0 ������ȡ��1 ��ʱ��δ���У�2 ʱ��Ϊ����
  * ˵    ������
  */
uint8_t TMR_Getleave(TIMER_TypeDef *ptmr, uint32_t *pleave)
{
    if(ptmr->trigger)
    {
        switch(ptmr->mode)
        {
            case 0://����ѭ��
                *pleave = ptmr->count;//���ص�������ʱ��
                return 2;
            case 1://����
                *pleave = ptmr->count;
                break;
            default://���
                *pleave = ((uint32_t) ptmr->repeat_count)*((uint32_t) ptmr->value) + ptmr->count;
//                *pleave = ptmr->count;//���ص�������ʱ��
                break;
        }
        return 0;
    }
    else if(ptmr->flag)
    {
        *pleave = 0;
        return 0;
    }
    return 1;
}

/**
  * ��������: ���ж�ʱ��
  * ���������ptmr ��ʱ��ָ��
  * �� �� ֵ: ��
  * ˵    ����ÿ1ms����һ��
  */
void TMR_RunTime(TIMER_TypeDef *ptmr)
{
    if(ptmr->trigger)
	{
        if(ptmr->mode == 1)//���μ���ģʽ
        {
            if(ptmr->count)
            {
                ptmr->count--;
                if(!ptmr->count)
                {
                    ptmr->flag = 1;
                    ptmr->trigger = 0;//��ʱ��ֹͣ����
                }
            }
        }
        else if(ptmr->mode == 0)//����ѭ��ģʽ
        {
            if(ptmr->count)
            {
                ptmr->count--;
                if(!ptmr->count)
                {
                    ptmr->flag = 1;
                    ptmr->count = ptmr->value;
//                    ptmr->trigger = 0;//��ʱ��ֹͣ����
                }
            }
            else
            {
                ptmr->count = ptmr->value-1;
            }
        }
        else//��μ���ģʽ
        {
            if(ptmr->count)
            {
                ptmr->count--;
                if(!ptmr->count)
                {
                    ptmr->flag = 1;
                    if(ptmr->repeat_count)
                    {
                        ptmr->repeat_count--;
                        ptmr->count = ptmr->value;
                    }
                    else
                    {
                        ptmr->trigger = 0;//��ʱ��ֹͣ����
                    }
                }
            }
            else if(ptmr->repeat_count)
            {
                ptmr->repeat_count--;
                ptmr->count = ptmr->value-1;
            }
            else
            {
                ptmr->trigger = 0;//��ʱ��ֹͣ����
            }
        }
	}
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
