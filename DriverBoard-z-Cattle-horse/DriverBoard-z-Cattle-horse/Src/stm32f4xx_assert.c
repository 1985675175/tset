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
  *  本文件提供断言函数,主要用于程序调试。HAL库中的函数均可以对输入参数进行检查，提高程序的健壮性。
  *  本文件是臻合智造创建的文件，不属于HAL库的文件，HAL库的范例是将这部分函数放在main.c文件。
  */

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"	
#include <stdio.h>

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/* 
 *	HAL库函数使用了C编译器的断言功能，如果定义了USE_FULL_ASSERT(在
 *  stm32f4xx_hal_conf.h定义，默认被注释掉)，那么所有的HAL库函数将检查函数形参是
 *  否正确。如果不正确将调用 assert_failed() 函数，这个函数是一个死循环，便于用
 *  户检查代码。
 *
 *	关键字 __LINE__ 表示源代码行号。
 *	关键字__FILE__表示源代码文件名。
 *
 *  断言功能使能后将增大代码大小，推荐用户仅在调试时使能，在正式发布软件是禁止。
 *
 *	用户可以选择是否使能HAL库的断言供能。使能断言的方法有两种：
 * 	(1) 在C编译器的预定义宏选项中定义USE_FULL_ASSERT。
 *	(2) 在stm32f4xx_hal_conf.h文件取消"#define USE_FULL_ASSERT    1"行的注释。	
*/
#ifdef USE_FULL_ASSERT

/**
  * 函数功能: 断言失败服务函数
  * 输入参数: file : 源代码文件名称。关键字__FILE__表示源代码文件名。
  *           line ：代码行号。关键字 __LINE__ 表示源代码行号
  * 返 回 值: 无
  * 说    明: 无
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* 
	 *	用户可以添加自己的代码报告源代码文件名和代码行号，比如将错误文件和行号打印到串口
	 *	printf("Wrong parameters value: file %s on line %d\r\n", file, line)
	 */
	
	/* 这是一个死循环，断言失败时程序会在此处死机，以便于用户查错 */
	while (1)
	{
	}
}
#endif
/************************ (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
