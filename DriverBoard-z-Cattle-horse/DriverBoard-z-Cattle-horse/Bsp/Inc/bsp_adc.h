/**
  ******************************************************************************
  * @file    bsp_adc.h
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
	 
#ifndef __BSP_ADC_H
#define	__BSP_ADC_H

#include "stm32f4xx.h"

// ADC ��ź궨��
#define TEMP_ADC                        ADC1
#define TEMP_ADC_CLK_ENABLE()           __ADC1_CLK_ENABLE()

#define ADC_VBUS_IRQ                    ADC_IRQn
#define ADC_VBUS_IRQHandler             ADC_IRQHandler

#define VREF                            3.3f     // �ο���ѹ����������3.3����ͨ��ʵ�ʲ�����3.258
#define ADC_NUM_MAX                     2048       // ADC ת��������������ֵ

#define GET_ADC_VDC_VAL(val)            ((float)val/4096.0f*VREF)          // �õ���ѹֵ
  
/*********************** �¶ȴ�������ѹ�ɼ� ******************/
#define TEMP_MAX    80    // �¶����ֵ
#define TEMP_MIN    10    // �¶���Сֵ
// ADC GPIO �궨��
#define TEMP_ADC_GPIO_PORT              GPIOA
#define TEMP_ADC_GPIO_PIN               GPIO_PIN_0
#define TEMP_ADC_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define TEMP_ADC_CHANNEL                ADC_CHANNEL_0

// ADC DR�Ĵ����궨�壬ADCת���������ֵ����������
#define TEMP_ADC_DR_ADDR                ((uint32_t)ADC1+0x4c)

// ADC DMA ͨ���궨�壬��������ʹ��DMA����
#define TEMP_ADC_DMA_CLK_ENABLE()       __DMA2_CLK_ENABLE()
#define TEMP_ADC_DMA_CHANNEL            DMA_CHANNEL_0
#define TEMP_ADC_DMA_STREAM             DMA2_Stream0

#define ADC_DMA_IRQ                     DMA2_Stream0_IRQn
#define ADC_DMA_IRQ_Handler             DMA2_Stream0_IRQHandler

/*********************** ��Դ��ѹ�ɼ� ******************/

#define VBUS_GPIO_PORT                  GPIOA
#define VBUS_GPIO_PIN                   GPIO_PIN_3
#define VBUS_GPIO_CLK_ENABLE()          __GPIOA_CLK_ENABLE()

#define VBUS_ADC_CHANNEL                ADC_CHANNEL_4

#define GET_VBUS_VAL(val)               (((float)val - 1.24f) * 37.0f)      // ��ȡ��ѹֵ��������ѹ�ǵ�Դ��ѹ��1/37��
  
#define VBUS_MAX                        30    // ��ѹ���ֵ
#define VBUS_MIN                        15    // ��ѹ��Сֵ

#define VBUS_HEX_MAX                    ((VBUS_MAX/37.0+1.24)/VREF*4096)    // ��ѹ���ֵ��������ѹ�ǵ�Դ��ѹ��1/37��
#define VBUS_HEX_MIN                    ((VBUS_MIN/37.0+1.24)/VREF*4096)    // ��ѹ��Сֵ��������ѹ�ǵ�Դ��ѹ��1/37��

extern DMA_HandleTypeDef DMA_Init_Handle;
extern ADC_HandleTypeDef ADC_Handle;

void ADC_Init(void);
float get_ntc_v_val(void);
float get_ntc_r_val(void);
float get_ntc_t_val(void);
float get_vbus_val(void);
 
#endif /* __BSP_ADC_H */

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
