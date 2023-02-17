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
  
/* 包含头文件 ----------------------------------------------------------------*/  
#include "bsp_timer.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
TIMER_TypeDef Timer[TMR_NUM_MAX] = \
								{0,0,1,0,0,100,100,//延时等待
								 1,0,0,30,30,30000,30000,//30S待机
								 1,0,0,60,60,120000,120000,//120S休眠
                                };
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/


/**
  * 函数功能: 设置定时器初始变量
  * 输入参数：ptmr 定时器指针；mode 计数模式,repeat_value 多次计数目标值,value 计数目标值
  * 返 回 值: 无
  * 说    明：在默认初始变量不能满足要求时用
  */
void TMR_SetInit(TIMER_TypeDef *ptmr, uint8_t mode, uint32_t repeat_value, uint32_t value)
{
  ptmr->mode = mode;
  ptmr->repeat_value = repeat_value;
  ptmr->value = value;
}

/**
  * 函数功能: 复位定时器
  * 输入参数：ptmr 定时器指针
  * 返 回 值: 无
  * 说    明：无
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
  * 函数功能: 设置触发使能
  * 输入参数：ptmr 定时器指针,trigger 触发使能
  * 返 回 值: 无
  * 说    明：无
  */
void TMR_SetTrigger(TIMER_TypeDef *ptmr, uint8_t trigger)
{
  ptmr->trigger = trigger;
}

/**
  * 函数功能: 设置定时器的初始计数值
  * 输入参数：ptmr 定时器指针,repeat_count 多次计数数值, count 计数数值
  * 返 回 值: 无
  * 说    明：无
  */
void TMR_SetCount(TIMER_TypeDef *ptmr, uint32_t repeat_count, uint32_t count)
{
  ptmr->repeat_count = repeat_count;
  ptmr->count = count;
}

/**
  * 函数功能: 读取定时器剩余时间
  * 输入参数：ptmr 定时器指针,pleave 剩余时间
  * 返 回 值: 0 正常获取，1 定时器未运行，2 时间为无限
  * 说    明：无
  */
uint8_t TMR_Getleave(TIMER_TypeDef *ptmr, uint32_t *pleave)
{
    if(ptmr->trigger)
    {
        switch(ptmr->mode)
        {
            case 0://无限循环
                *pleave = ptmr->count;//返回当次运行时间
                return 2;
            case 1://单次
                *pleave = ptmr->count;
                break;
            default://多次
                *pleave = ((uint32_t) ptmr->repeat_count)*((uint32_t) ptmr->value) + ptmr->count;
//                *pleave = ptmr->count;//返回当次运行时间
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
  * 函数功能: 运行定时器
  * 输入参数：ptmr 定时器指针
  * 返 回 值: 无
  * 说    明：每1ms调用一次
  */
void TMR_RunTime(TIMER_TypeDef *ptmr)
{
    if(ptmr->trigger)
	{
        if(ptmr->mode == 1)//单次计数模式
        {
            if(ptmr->count)
            {
                ptmr->count--;
                if(!ptmr->count)
                {
                    ptmr->flag = 1;
                    ptmr->trigger = 0;//定时器停止运行
                }
            }
        }
        else if(ptmr->mode == 0)//无限循环模式
        {
            if(ptmr->count)
            {
                ptmr->count--;
                if(!ptmr->count)
                {
                    ptmr->flag = 1;
                    ptmr->count = ptmr->value;
//                    ptmr->trigger = 0;//定时器停止运行
                }
            }
            else
            {
                ptmr->count = ptmr->value-1;
            }
        }
        else//多次计数模式
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
                        ptmr->trigger = 0;//定时器停止运行
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
                ptmr->trigger = 0;//定时器停止运行
            }
        }
	}
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
