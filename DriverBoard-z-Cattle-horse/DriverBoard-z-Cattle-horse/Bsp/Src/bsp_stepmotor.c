/**
  ******************************************************************************
  * @file    bsp_stepmotor.c
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
#include "bsp_stepmotor.h"
#include "bsp_led.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_limit.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/
/**
  * ��ѧ����.�޸�Alpha�����޸��ٶȵ�λ
  * ���� :ALPHA = 360/SMOTOR1_PULSEREV,�ٶ�ֵ��λ���ǡ�/s 
  *       ALPHA = 1/SMOTOR1_PULSEREV,����Rev/s,�ٶ�ֵ��λ����Rev/s 
  * ALPHA ���������嵱����Ҳ����ÿһ�������Ӧ���ת������
  */
#define ALPHA                 ((float)(1.0f / (float)SMOTOR1_PULSE_REV)) // ��= 360/PPR�����
#define ALPHA_x2              ((float)(2.0f * ALPHA))

/* ˽�б��� ------------------------------------------------------------------*/

TIM_HandleTypeDef             htimx_SMotor[MOTORNUM]; // ��ʱ�����ƾ��
DMA_HandleTypeDef             hdma_tim[MOTORNUM];     // DMA���ƾ��
MotorObj_Typedef              SMotor[MOTORNUM];       // ����������ƾ��
MoveRunData_Typedef mrd_SMotor[MOTORNUM] = {{SMOTOR1_LIM_SPEED,SMOTOR1_CRAML_SPEED,SMOTOR1_ACC_TIME,SMOTOR1_DEC_TIME,SMOTOR1_DISTANCE_REV,SMOTOR1_STEP_ANGLE,SMOTOR1_MICRO_STEP},
                                            {SMOTOR2_LIM_SPEED,SMOTOR2_CRAML_SPEED,SMOTOR2_ACC_TIME,SMOTOR2_DEC_TIME,SMOTOR2_DISTANCE_REV,SMOTOR2_STEP_ANGLE,SMOTOR2_MICRO_STEP},
                                            {SMOTOR3_LIM_SPEED,SMOTOR3_CRAML_SPEED,SMOTOR3_ACC_TIME,SMOTOR3_DEC_TIME,SMOTOR3_DISTANCE_REV,SMOTOR3_STEP_ANGLE,SMOTOR3_MICRO_STEP},
                                            {SMOTOR4_LIM_SPEED,SMOTOR4_CRAML_SPEED,SMOTOR4_ACC_TIME,SMOTOR4_DEC_TIME,SMOTOR4_DISTANCE_REV,SMOTOR4_STEP_ANGLE,SMOTOR4_MICRO_STEP}};
PipetteRunData_Typedef prd_SMotor[PIPNUM] = {{PIPETTE1_VOLUME_DISTANCE_RATIO,PIPETTE1_MAX_STROKE,PIPETTE1_UNLOAD_DISTANCE},
                                             {PIPETTE2_VOLUME_DISTANCE_RATIO,PIPETTE2_MAX_STROKE,PIPETTE2_UNLOAD_DISTANCE}};

float TClkSrc_Frq[MOTORNUM] = {SMOTOR1_TIM_CLKSRC_FREQ, SMOTOR2_TIM_CLKSRC_FREQ, SMOTOR3_TIM_CLKSRC_FREQ, SMOTOR4_TIM_CLKSRC_FREQ};  //��ʱ��ʱ��ԴƵ��

/**
  * DMA������ڴ��ַ������0x2000 0000~0x2001 FFFF,�������ַ��Χ��������DTCM��
  * ��DMA1,2��û��ֱ�����ӵ�DTCM������DMA1,2������ֱ�ӷ���DTCM������DMA�����ӵ�
  * AXI SRAM��SRAM1/2/3/4����AXI SRAM�ĵ�ַ��Χ����0x2400 0000 - 0x2407 FFFF,
  * ����������ǽ����涨����AXI SRAM�С�
  */
// Buffer0[0],Buffer1[0]��SM1��ר�û�������Buffer0[1],Buffer1[1]��SM2��ר�û�������
#if defined (__ICCARM__)    /* IAR Compiler */
/* IAR �������� */
// ���������ַ��0x24000000֮��,ռ��AXI-SRAM
__IO uint16_t Spd_Buffer0[BUFFER_SIZE][BUFFER_SIZE]  @ "AXISRAM";
__IO uint16_t Spd_Buffer1[BUFFER_SIZE][BUFFER_SIZE]  @ "AXISRAM";

#elif defined   (__CC_ARM)  /* ARM Compiler */
/* Keil �������� */  
__attribute__ ((section("AXISRAM"))) // ���������ַ��0x24000000֮��,ռ��AXI-SRAM
__IO uint16_t Spd_Buffer0[MOTORNUM][BUFFER_SIZE] = {0};

__attribute__ ((section("AXISRAM"))) // ���Զ�����0x2400 0000 ~ 0x2408 0000֮��
__IO uint16_t Spd_Buffer1[MOTORNUM][BUFFER_SIZE] = {0};
#endif 


__IO div_t BufferMod[MOTORNUM]  = {0}; // �жϴ���,����

/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/

static void DMAx_Config(uint16_t SMotorNum);   // DMA����
static void DMA_TC_M0CpltCallback(DMA_HandleTypeDef *hdma);// M0������ɻص�����
static void DMA_TC_M1CpltCallback(DMA_HandleTypeDef *hdma);// M1������ɻص�����
static void DMA_TC_ErrorCallback(DMA_HandleTypeDef *hdma);//���������ú���
static void LSC_CalcSpd(uint16_t SMx, uint16_t  *Array,uint32_t Buffersize);              // �����ٶ�ֵ
static void SMotor_GPIO_MspInit(uint16_t SMotorNum);   // ���ų�ʼ��
static void Start_MotorMoving(uint16_t SMx, int32_t step);// �������

/* ������ --------------------------------------------------------------------*/

/**
  * ��������: DMA����
  * �������: SMotorNum ������
  * �� �� ֵ: ��
  * ˵    ��: ��ʼ������DMA����
  */
static void DMAx_Config(uint16_t SMx)
{
  DMA_HandleTypeDef  *ptrdma = &hdma_tim[SMx];// DMA���ƾ��
  IRQn_Type DMA_Stream_IRQn = (IRQn_Type)0;   // ��ʼ��Ϊ0��ʹ�ñ�����ʼ��
  uint16_t DMA_TIM_ID = 0;                    // ��ʼ��Ϊ0
  
  switch(SMx)
  {
    /* �������1��������� */
    case SM1:
      DMAx_CHANNELz1_CLK_ENABLE();           // ʹ��DMAx ��ʱ��
      DMA_Stream_IRQn = DMAx_Streamz1_IRQn;  // ʹ��SM1�����DMA����.
      DMA_TIM_ID = DMAx_TIMz1_ID;
      
      ptrdma->Instance      = DMAx_Streamz1;
      ptrdma->Init.Channel  = DMAx_TIMz1_REQUEST_CHx;
      break;
    /* �������2��������� */
    case SM2:
      DMAx_CHANNELp1_CLK_ENABLE();            // ʹ��DMAx ��ʱ��
      DMA_Stream_IRQn = DMAx_Streamp1_IRQn;
      DMA_TIM_ID = DMAx_TIMp1_ID;
    
      ptrdma->Instance      = DMAx_Streamp1;
      ptrdma->Init.Channel  = DMAx_TIMp1_REQUEST_CHx;
      break;
    /* �������3��������� */
    case SM3:
      DMAx_CHANNELz2_CLK_ENABLE();           // ʹ��DMAx ��ʱ��
      DMA_Stream_IRQn = DMAx_Streamz2_IRQn;  // ʹ��SM1�����DMA����.
      DMA_TIM_ID = DMAx_TIMz2_ID;
      
      ptrdma->Instance      = DMAx_Streamz2;
      ptrdma->Init.Channel  = DMAx_TIMz2_REQUEST_CHx;
      break;
    /* �������4��������� */
    case SM4:
      DMAx_CHANNELp2_CLK_ENABLE();            // ʹ��DMAx ��ʱ��
      DMA_Stream_IRQn = DMAx_Streamp2_IRQn;
      DMA_TIM_ID = DMAx_TIMp2_ID;
    
      ptrdma->Instance      = DMAx_Streamp2;
      ptrdma->Init.Channel  = DMAx_TIMp2_REQUEST_CHx;
      break;
  }
  
  /*********** ����ͨ��DMA���� ***********************/
  ptrdma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
  ptrdma->Init.PeriphInc           = DMA_PINC_DISABLE;
  ptrdma->Init.MemInc              = DMA_MINC_ENABLE;
  ptrdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  ptrdma->Init.MemDataAlignment    = DMA_PDATAALIGN_HALFWORD;
  ptrdma->Init.Mode                = DMA_NORMAL;
  ptrdma->Init.Priority            = DMA_PRIORITY_HIGH;
  ptrdma->Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  ptrdma->Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  ptrdma->Init.MemBurst            = DMA_MBURST_SINGLE;
  ptrdma->Init.PeriphBurst         = DMA_PBURST_SINGLE;
  
  HAL_DMA_DeInit(ptrdma);
  
  /* ����DMA��TIM */
  __HAL_LINKDMA(&htimx_SMotor[SMx], hdma[DMA_TIM_ID], *ptrdma);

  HAL_DMA_Init(ptrdma);

  /* ����M0�жϵ��ú��� */
  ptrdma -> XferCpltCallback   = DMA_TC_M0CpltCallback;
  /* ����M1�жϵ��ú��� */
  ptrdma -> XferM1CpltCallback = DMA_TC_M1CpltCallback;
  /* ���ô��������ú��� */
  ptrdma -> XferErrorCallback = DMA_TC_ErrorCallback;
  
  /* ����DMA�ж� */
  HAL_NVIC_SetPriority(DMA_Stream_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA_Stream_IRQn);  
  
}

/**
  * ��������: ��ʱ��Ӳ����ʼ������
  * �������: SMotorNum ������
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void  SMotor_GPIO_MspInit(uint16_t SMx)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  switch(SMx)
  {
    case SM1:// StepMotor1
      /* ��ʱ������ʱ��ʹ�� */
      SMOTOR1_PUL_GPIO_CLK_ENABLE();
      SMOTOR1_DIR_GPIO_CLK_ENABLE();
      SMOTOR1_ENA_GPIO_CLK_ENABLE();
    
      /* ��������������� */  
      GPIO_InitStruct.Pin       = SMOTOR1_PUL_GPIO_PIN ;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;      // ����ģʽ
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      GPIO_InitStruct.Alternate = SMOTOR1_PUL_PIN_AF; // GPIO��������TIM���ù���
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(SMOTOR1_PUL_GPIO_PORT, &GPIO_InitStruct);
      
      /* ��������������� */  
      GPIO_InitStruct.Pin       = SMOTOR1_DIR_GPIO_PIN;
      GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Alternate = 0;        
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      HAL_GPIO_Init(SMOTOR1_DIR_GPIO_PORT, &GPIO_InitStruct);
      
      /* ���ʹ���������� */  
      GPIO_InitStruct.Pin       = SMOTOR1_ENA_GPIO_PIN;
      HAL_GPIO_Init(SMOTOR1_ENA_GPIO_PORT, &GPIO_InitStruct);
      
      break;
    case SM2:// StepMotor2
      /* ��ʱ������ʱ��ʹ�� */
      SMOTOR2_PUL_GPIO_CLK_ENABLE();
      SMOTOR2_DIR_GPIO_CLK_ENABLE();
      SMOTOR2_ENA_GPIO_CLK_ENABLE();
    
      /* ��������������� */  
      GPIO_InitStruct.Pin       = SMOTOR2_PUL_GPIO_PIN ;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;      // ����ģʽ
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      GPIO_InitStruct.Alternate = SMOTOR2_PUL_PIN_AF; // GPIO��������TIM���ù���
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(SMOTOR2_PUL_GPIO_PORT, &GPIO_InitStruct);
      
      /* ��������������� */  
      GPIO_InitStruct.Pin       = SMOTOR2_DIR_GPIO_PIN;
      GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Alternate = 0;        
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      HAL_GPIO_Init(SMOTOR2_DIR_GPIO_PORT, &GPIO_InitStruct);
      
      /* ���ʹ���������� */  
      GPIO_InitStruct.Pin       = SMOTOR2_ENA_GPIO_PIN;
      HAL_GPIO_Init(SMOTOR2_ENA_GPIO_PORT, &GPIO_InitStruct);
      break;
    case SM3:// StepMotor3
      /* ��ʱ������ʱ��ʹ�� */
      SMOTOR3_PUL_GPIO_CLK_ENABLE();
      SMOTOR3_DIR_GPIO_CLK_ENABLE();
      SMOTOR3_ENA_GPIO_CLK_ENABLE();
    
      /* ��������������� */  
      GPIO_InitStruct.Pin       = SMOTOR3_PUL_GPIO_PIN ;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;      // ����ģʽ
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      GPIO_InitStruct.Alternate = SMOTOR3_PUL_PIN_AF; // GPIO��������TIM���ù���
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(SMOTOR3_PUL_GPIO_PORT, &GPIO_InitStruct);
      
      /* ��������������� */  
      GPIO_InitStruct.Pin       = SMOTOR3_DIR_GPIO_PIN;
      GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Alternate = 0;        
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      HAL_GPIO_Init(SMOTOR3_DIR_GPIO_PORT, &GPIO_InitStruct);
      
      /* ���ʹ���������� */  
      GPIO_InitStruct.Pin       = SMOTOR3_ENA_GPIO_PIN;
      HAL_GPIO_Init(SMOTOR3_ENA_GPIO_PORT, &GPIO_InitStruct);
      
      break;
    case SM4:// StepMotor4
      /* ��ʱ������ʱ��ʹ�� */
      SMOTOR4_PUL_GPIO_CLK_ENABLE();
      SMOTOR4_DIR_GPIO_CLK_ENABLE();
      SMOTOR4_ENA_GPIO_CLK_ENABLE();
    
      /* ��������������� */  
      GPIO_InitStruct.Pin       = SMOTOR4_PUL_GPIO_PIN ;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;      // ����ģʽ
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      GPIO_InitStruct.Alternate = SMOTOR4_PUL_PIN_AF; // GPIO��������TIM���ù���
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(SMOTOR4_PUL_GPIO_PORT, &GPIO_InitStruct);
      
      /* ��������������� */  
      GPIO_InitStruct.Pin       = SMOTOR4_DIR_GPIO_PIN;
      GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Alternate = 0;        
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      HAL_GPIO_Init(SMOTOR4_DIR_GPIO_PORT, &GPIO_InitStruct);
      
      /* ���ʹ���������� */  
      GPIO_InitStruct.Pin       = SMOTOR4_ENA_GPIO_PIN;
      HAL_GPIO_Init(SMOTOR4_ENA_GPIO_PORT, &GPIO_InitStruct);
      break;
  }
}

/**
  * ��������: ����������ƶ�ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��ʼ��PWM�������
  */
void SMOTOR1_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_BreakDeadTimeConfigTypeDef sconfigBDTR;
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_MasterConfigTypeDef sMasterConfig;
  /* ��ʱ��Ӳ������ */
  SMotor_GPIO_MspInit(SM1);
  
  /* ��ʱ���������� */
  SMOTOR1_TIM_RCC_CLK_ENABLE();
  htimx_SMotor[SM1].Instance               = SMOTOR1_TIMx;
  htimx_SMotor[SM1].Init.Prescaler         = SMOTOR1_TIM_PRESCALER;
  htimx_SMotor[SM1].Init.CounterMode       = TIM_COUNTERMODE_UP;
  htimx_SMotor[SM1].Init.Period            = SMOTOR1_TIM_PERIOD;
  htimx_SMotor[SM1].Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htimx_SMotor[SM1].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//����Ԥ��Ƶ    //error
  HAL_TIM_PWM_Init(&htimx_SMotor[SM1]);

  /* ʱ��Դ���� */
  sClockSourceConfig.ClockSource    = TIM_CLOCKSOURCE_INTERNAL; // �ڲ�ʱ��
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;  // ��ʱ��Ԥ��Ƶ
  sClockSourceConfig.ClockPolarity  = TIM_CLOCKPOLARITY_INVERTED;// ��Ч����
  sClockSourceConfig.ClockFilter    = 0x0;  
  HAL_TIM_ConfigClockSource(&htimx_SMotor[SM1], &sClockSourceConfig);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_SMotor[SM1],&sMasterConfig);
  
  /* ����ʱ���״̬���� */
  /** ������Ҫ���ø߼���ʱ���Ļ���ͨ���������,����ɲ�����ܵ���������Ч��
    * ���ʹ�÷Ǹ߼���ʱ��,����ע�͵���δ���
    */
  sconfigBDTR.OffStateRunMode  = TIM_OSSR_DISABLE;  // �ر�ʱ������ģʽ������� 
  sconfigBDTR.OffStateIDLEMode = TIM_OSSI_DISABLE;  // �ر�ʱ�Ŀ���ģʽ�������
  sconfigBDTR.BreakState       = TIM_BREAK_DISABLE; // ��ʹ��ɲ������
  sconfigBDTR.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
//  sconfigBDTR.BreakFilter      = 0;
  sconfigBDTR.DeadTime         = 0; // Min_Data = 0x00 and Max_Data = 0xFF
  sconfigBDTR.LockLevel        = TIM_LOCKLEVEL_OFF;
//  sconfigBDTR.Break2Filter     = 0; // Min_Data = 0x0 and Max_Data = 0xF
//  sconfigBDTR.Break2Polarity   = TIM_BREAK2POLARITY_LOW;
//  sconfigBDTR.Break2State      = TIM_BREAK2_DISABLE;
  sconfigBDTR.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx_SMotor[SM1],&sconfigBDTR);
  
  /* PWM����,CH1 PWM Mode*/
  sConfigOC.OCMode       = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse        = SMOTOR1_TIM_DEFAULT_PULSE;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htimx_SMotor[SM1], &sConfigOC, SMOTOR1_PUL_TIMx_CHANNELx);
  
  DMAx_Config(SM1);
    
  /* ���õ��Ĭ��״̬,Ĭ��ʹ�ܵ��ת��,���ҷ�����˳ʱ��ת�� */
  SMotor_Set_Dir  ( SM1, MOTOR_DIR_CW);
  SMotor_Set_State( SM1, MOTOR_DISABLE);
}

/**
  * ��������: ����������ƶ�ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��ʼ��PWM�������
  */
void SMOTOR2_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
  /* ��ʱ��Ӳ������ */
  SMotor_GPIO_MspInit(SM2);
  
  /* ��ʱ���������� */
  SMOTOR2_TIM_RCC_CLK_ENABLE();
  htimx_SMotor[SM2].Instance               = SMOTOR2_TIMx;
  htimx_SMotor[SM2].Init.Prescaler         = SMOTOR2_TIM_PRESCALER;
  htimx_SMotor[SM2].Init.CounterMode       = TIM_COUNTERMODE_UP;
  htimx_SMotor[SM2].Init.Period            = SMOTOR2_TIM_PERIOD;
  htimx_SMotor[SM2].Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htimx_SMotor[SM2].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//����Ԥ��Ƶ
  HAL_TIM_PWM_Init(&htimx_SMotor[SM2]);

  /* ʱ��Դ���� */
  sClockSourceConfig.ClockSource    = TIM_CLOCKSOURCE_INTERNAL; // �ڲ�ʱ��
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;  // ��ʱ��Ԥ��Ƶ
  sClockSourceConfig.ClockPolarity  = TIM_CLOCKPOLARITY_INVERTED;// ��Ч����
  sClockSourceConfig.ClockFilter    = 0x0;  
  HAL_TIM_ConfigClockSource(&htimx_SMotor[SM2], &sClockSourceConfig);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_SMotor[SM2],&sMasterConfig);
  
  /* PWM����,CH1 PWM Mode*/
  sConfigOC.OCMode       = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse        = SMOTOR2_TIM_DEFAULT_PULSE;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htimx_SMotor[SM2], &sConfigOC, SMOTOR2_PUL_TIMx_CHANNELx);
  
  DMAx_Config(SM2);
    
  /* ���õ��Ĭ��״̬,Ĭ�Ͻ�ֹ���ת��,���ҷ�����˳ʱ��ת�� */
  SMotor_Set_Dir  ( SM2, MOTOR_DIR_CW);
  SMotor_Set_State( SM2, MOTOR_DISABLE);
}

/**
  * ��������: ����������ƶ�ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��ʼ��PWM�������
  */
void SMOTOR3_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
  /* ��ʱ��Ӳ������ */
  SMotor_GPIO_MspInit(SM3);
  
  /* ��ʱ���������� */
  SMOTOR3_TIM_RCC_CLK_ENABLE();
  htimx_SMotor[SM3].Instance               = SMOTOR3_TIMx;
  htimx_SMotor[SM3].Init.Prescaler         = SMOTOR3_TIM_PRESCALER;
  htimx_SMotor[SM3].Init.CounterMode       = TIM_COUNTERMODE_UP;
  htimx_SMotor[SM3].Init.Period            = SMOTOR3_TIM_PERIOD;
  htimx_SMotor[SM3].Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htimx_SMotor[SM3].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//����Ԥ��Ƶ
  HAL_TIM_PWM_Init(&htimx_SMotor[SM3]);

  /* ʱ��Դ���� */
  sClockSourceConfig.ClockSource    = TIM_CLOCKSOURCE_INTERNAL; // �ڲ�ʱ��
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;  // ��ʱ��Ԥ��Ƶ
  sClockSourceConfig.ClockPolarity  = TIM_CLOCKPOLARITY_INVERTED;// ��Ч����
  sClockSourceConfig.ClockFilter    = 0x0;  
  HAL_TIM_ConfigClockSource(&htimx_SMotor[SM3], &sClockSourceConfig);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_SMotor[SM3],&sMasterConfig);
  
  /* PWM����,CH1 PWM Mode*/
  sConfigOC.OCMode       = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse        = SMOTOR3_TIM_DEFAULT_PULSE;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htimx_SMotor[SM3], &sConfigOC, SMOTOR3_PUL_TIMx_CHANNELx);
  
  DMAx_Config(SM3);
    
  /* ���õ��Ĭ��״̬,Ĭ�Ͻ�ֹ���ת��,���ҷ�����˳ʱ��ת�� */
  SMotor_Set_Dir  ( SM3, MOTOR_DIR_CW);
  SMotor_Set_State( SM3, MOTOR_DISABLE);
}

/**
  * ��������: ����������ƶ�ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��ʼ��PWM�������
  */
void SMOTOR4_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
  /* ��ʱ��Ӳ������ */
  SMotor_GPIO_MspInit(SM4);
  
  /* ��ʱ���������� */
  SMOTOR4_TIM_RCC_CLK_ENABLE();
  htimx_SMotor[SM4].Instance               = SMOTOR4_TIMx;
  htimx_SMotor[SM4].Init.Prescaler         = SMOTOR4_TIM_PRESCALER;
  htimx_SMotor[SM4].Init.CounterMode       = TIM_COUNTERMODE_UP;
  htimx_SMotor[SM4].Init.Period            = SMOTOR4_TIM_PERIOD;
  htimx_SMotor[SM4].Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htimx_SMotor[SM4].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//����Ԥ��Ƶ
  HAL_TIM_PWM_Init(&htimx_SMotor[SM4]);

  /* ʱ��Դ���� */
  sClockSourceConfig.ClockSource    = TIM_CLOCKSOURCE_INTERNAL; // �ڲ�ʱ��
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;  // ��ʱ��Ԥ��Ƶ
  sClockSourceConfig.ClockPolarity  = TIM_CLOCKPOLARITY_INVERTED;// ��Ч����
  sClockSourceConfig.ClockFilter    = 0x0;  
  HAL_TIM_ConfigClockSource(&htimx_SMotor[SM4], &sClockSourceConfig);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_SMotor[SM4],&sMasterConfig);
  
  /* PWM����,CH1 PWM Mode*/
  sConfigOC.OCMode       = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse        = SMOTOR4_TIM_DEFAULT_PULSE;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htimx_SMotor[SM4], &sConfigOC, SMOTOR4_PUL_TIMx_CHANNELx);
  
  DMAx_Config(SM4);
    
  /* ���õ��Ĭ��״̬,Ĭ�Ͻ�ֹ���ת��,���ҷ�����˳ʱ��ת�� */
  SMotor_Set_Dir  ( SM4, MOTOR_DIR_CW);
  SMotor_Set_State( SM4, MOTOR_DISABLE);
}

/**
  * ��������: ���õ������
  * �������: Dir ������� ������ѡ: 
  *               @arg  MOTOR_DIR_CW
  *               @arg  MOTOR_DIR_CCW
  * �� �� ֵ:
  * ˵    ��:���ò�������ķ���.������������DIR�˿�
  */
void SMotor_Set_Dir  (uint16_t SMx, MotorDir_Typedef Dir)
{
  /* ���ö�Ӧ��ŵĲ���������� */
  SMotor[SMx].Dir = Dir ;
  switch(SMx)
  {
    case SM1:
      if(Dir == MOTOR_DIR_CW)
      {
        SMOTOR1_DIR_FORWARE();
      }
      else 
      {
        SMOTOR1_DIR_REVERSAL();
      }
      break;
    case SM2:
      if(Dir == MOTOR_DIR_CW)
      {
        SMOTOR2_DIR_FORWARE();
      }
      else 
      {
        SMOTOR2_DIR_REVERSAL();
      }
      break;
    case SM3:
      if(Dir == MOTOR_DIR_CW)
      {
        SMOTOR3_DIR_FORWARE();
      }
      else 
      {
        SMOTOR3_DIR_REVERSAL();
      }
      break;
    case SM4:
      if(Dir == MOTOR_DIR_CW)
      {
        SMOTOR4_DIR_FORWARE();
      }
      else 
      {
        SMOTOR4_DIR_REVERSAL();
      }
      break;
  }
}


/**
  * ��������: ���õ��״̬
  * �������: Ena ���״̬ ������ѡ: 
  *               @arg  MOTOR_ENABLE
  *               @arg  MOTOR_DISABLE
  * �� �� ֵ:
  * ˵    ��: ���ò��������״̬,������������ENA�˿�
  */
void SMotor_Set_State(uint16_t SMx, MotorSta_Typedef Ena)
{
    /* ���ö�Ӧ��ŵĲ������ʹ�� */
  SMotor[SMx].State = Ena ;
  switch(SMx)
  {
    case SM1:
      if(Ena == MOTOR_DIR_CW)
      {
        SMOTOR1_ENABLE();
      }
      else 
      {
        SMOTOR1_DISABLE();
      }
      break;
    case SM2:
      if(Ena == MOTOR_DIR_CW)
      {
        SMOTOR2_ENABLE();
      }
      else 
      {
        SMOTOR2_DISABLE();
      }
      break;
    case SM3:
      if(Ena == MOTOR_DIR_CW)
      {
        SMOTOR3_ENABLE();
      }
      else 
      {
        SMOTOR3_DISABLE();
      }
      break;
    case SM4:
      if(Ena == MOTOR_DIR_CW)
      {
        SMOTOR4_ENABLE();
      }
      else 
      {
        SMOTOR4_DISABLE();
      }
      break;
  }
}

/****************************** ���μӼ��ٿ��� ********************************/
/**
  * ��������: �������ת��
  * �������: SMx ���������ţ�step �ߵĲ���
  * �� �� ֵ: ��
  * ˵    ��: ����ٶȻ�����,ʹ��DMA,ʹ�ܶ�ʱ��ͨ�����
  */
static void Start_MotorMoving(uint16_t SMx, int32_t step)
{
	uint32_t TIMx_CHANNEL = 0;     // dummy
	uint32_t RegAddr = 0;          // dummy
	uint16_t DMA_TIM_CCn = 0;      // dummy
	uint16_t *ptrBuf0 = NULL;
	uint16_t *ptrBuf1 = NULL;
  
	// ��������SM1��SM2����
	switch(SMx)
	{
	  case SM1:
      TIMx_CHANNEL = SMOTOR1_PUL_TIMx_CHANNELx;   // ���ö�ʱ��ͨ��
      DMA_TIM_CCn = DMAx_TIMz1_CCn;                // ���ö�ʱ��DMA����
      RegAddr = (uint32_t)SMOTOR1_TIMx_CCxADDRESS;// ���üĴ�����ַ
      
      DMAx_STREAMz1_CLEAR_FLAG();            // ��ձ�־λ
      DMAx_STREAMx_SET_CTM0(DMAx_Streamz1);  // ����Buffer0Ϊ�׸�����Դ

      break;
      case SM2:
      TIMx_CHANNEL = SMOTOR2_PUL_TIMx_CHANNELx;
      DMA_TIM_CCn = DMAx_TIMp1_CCn; 
      RegAddr = (uint32_t)SMOTOR2_TIMx_CCxADDRESS;    
      DMAx_STREAMp1_CLEAR_FLAG(); // ��ձ�־λ
      DMAx_STREAMx_SET_CTM0(DMAx_Streamp1); // ����Buffer0Ϊ�׸�����Դ

      break;
	  case SM3:
      TIMx_CHANNEL = SMOTOR3_PUL_TIMx_CHANNELx;   // ���ö�ʱ��ͨ��
      DMA_TIM_CCn = DMAx_TIMz2_CCn;                // ���ö�ʱ��DMA����
      RegAddr = (uint32_t)SMOTOR3_TIMx_CCxADDRESS;// ���üĴ�����ַ
      
      DMAx_STREAMz2_CLEAR_FLAG();            // ��ձ�־λ
      DMAx_STREAMx_SET_CTM0(DMAx_Streamz2);  // ����Buffer0Ϊ�׸�����Դ

      break;
      case SM4:
      TIMx_CHANNEL = SMOTOR4_PUL_TIMx_CHANNELx;
      DMA_TIM_CCn = DMAx_TIMp2_CCn; 
      RegAddr = (uint32_t)SMOTOR4_TIMx_CCxADDRESS;    
      DMAx_STREAMp2_CLEAR_FLAG(); // ��ձ�־λ
      DMAx_STREAMx_SET_CTM0(DMAx_Streamp2); // ����Buffer0Ϊ�׸�����Դ

      break;
	}
	ptrBuf0 = (uint16_t *)&Spd_Buffer0[SMx];// ʹ��ָ������������
	ptrBuf1 = (uint16_t *)&Spd_Buffer1[SMx];// 
  
	/* ���õ�һ����Ƶ�� */
	__HAL_TIM_SET_COMPARE(&htimx_SMotor[SMx], TIMx_CHANNEL, 1);
	/* ʹ��ʱ��Ԥ��Ƶ������Ч */
	HAL_TIM_GenerateEvent(&htimx_SMotor[SMx], TIM_EVENTSOURCE_UPDATE);

	/* �����жϵĴ��� */
	BufferMod[SMx] = div(step, (BUFFER_SIZE>>1) );// ��������
   
	/* ��仺���� */
	*ptrBuf0 = 1 ;    // buffer0
	LSC_CalcSpd(SMx, ptrBuf0, BUFFER_SIZE);
//	SCB_CleanDCache_by_Addr((uint32_t*)ptrBuf0, DATAWIDTH);
  
	*ptrBuf1 = ptrBuf0[BUFFER_SIZE-1]; // buffer1
	LSC_CalcSpd(SMx, ptrBuf1, BUFFER_SIZE);  
//	SCB_CleanDCache_by_Addr((uint32_t*)ptrBuf1, DATAWIDTH);
	
	/* ����DMA˫���� */
	HAL_DMAEx_MultiBufferStart_IT(&hdma_tim[SMx], 
                            (uint32_t)ptrBuf0,// ��һ������Դ
                            (uint32_t)RegAddr,// �Ƚ�����ַ
                            (uint32_t)ptrBuf1,// �ڶ�������Դ
                            BUFFER_SIZE);     // �������ݳ���   
  
	__HAL_TIM_ENABLE_DMA(&htimx_SMotor[SMx], DMA_TIM_CCn);// ʹ��DMA���ж�

	/* ʹ�ܵ��ת�� */
	SMotor_Set_State(SMx, MOTOR_ENABLE);
// ��������ʱ�����������ʹ����ʱ����ȷ������������Ϊʹ�ܵ������ģ�
// ��������Ϊ��ʼ�ٶ�ͻ������
//  HAL_Delay(1000); 
	SMotor[SMx].MotorRuning = true;

	/* �����ȽϷ�ת��ƽ */
	if((SMx == SM1) || (SMx == SM2) || (SMx == SM3) || (SMx == SM4))
	{
		HAL_TIM_OC_Start(&htimx_SMotor[SMx], TIMx_CHANNEL);
	}
	else
	{
		HAL_TIMEx_OCN_Start(&htimx_SMotor[SMx], TIMx_CHANNEL);
	}
}

/**
  * ��������: ֹͣ���ת��
  * �������: SMx ����������
  * �� �� ֵ: ��
  * ˵    ��: ��ֹDMA�����ж�,��ֹ���PWM,ʧ�ܵ��
  */
void Stop_MotorMoving(uint16_t SMx)
{
  switch(SMx)
  {
    case SM1:
      HAL_DMA_Abort_IT(htimx_SMotor[SMx].hdma[DMAx_TIMz1_ID]);
      HAL_TIM_OC_Stop(&htimx_SMotor[SMx], SMOTOR1_PUL_TIMx_CHANNELx);
      break;
    case SM2:
      HAL_DMA_Abort_IT(htimx_SMotor[SMx].hdma[DMAx_TIMp1_ID]);
      HAL_TIM_OC_Stop(&htimx_SMotor[SMx], SMOTOR2_PUL_TIMx_CHANNELx);
      break;
    case SM3:
      HAL_DMA_Abort_IT(htimx_SMotor[SMx].hdma[DMAx_TIMz2_ID]);
      HAL_TIM_OC_Stop(&htimx_SMotor[SMx], SMOTOR3_PUL_TIMx_CHANNELx);
      break;
    case SM4:
      HAL_DMA_Abort_IT(htimx_SMotor[SMx].hdma[DMAx_TIMp2_ID]);
      HAL_TIM_OC_Stop(&htimx_SMotor[SMx], SMOTOR4_PUL_TIMx_CHANNELx);
      break;
  }
//  HAL_TIM_Base_Stop(&htimx_SMotor[SM1]);
  SMotor_Set_State(SMx, MOTOR_DISABLE);
  memset( &SMotor[SMx].srd,0,sizeof(SMotor[SMx].srd));// ��0 ����ֵ
  SMotor[SMx].srd.run_state = STOP;
  SMotor[SMx].MotorRuning = false;
}
/**
  * ��������: ������λ���ص�״̬�ж��Ƿ�����ת��
  * �������: SMx :����������
  * �� �� ֵ: bool����ǰ����ת���򷵻�true�����򷵻�false
  * ˵    ��: ��
  */
bool NotAllow2Move(uint16_t SMx)
{
  switch(SMx)
  {
    /* �ж�SM1����λ����״̬ */
    case SM1:
      if(HAL_GPIO_ReadPin(LIMIT1_POS_GPIO_PORT,LIMIT1_POS_PIN) == LIMIT1_POS_ACTIVE_LEVEL)
        if(SMotor[SMx].Dir == MOTOR_DIR_CW)
          return true;
      if(HAL_GPIO_ReadPin(LIMIT1_NEG_GPIO_PORT,LIMIT1_NEG_PIN) == LIMIT1_NEG_ACTIVE_LEVEL)
        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
          return true;
      if(HAL_GPIO_ReadPin(LIMIT2_NEG_GPIO_PORT,LIMIT2_NEG_PIN) == LIMIT2_NEG_ACTIVE_LEVEL)
        if(SMotor[SMx].Dir == MOTOR_DIR_CW)
          if(LoadFlag[SMx])//�������װ��ܿ��ƣ�����Ч
          {
            LoadFlag[SMx] = false;
            LoadInitFlag[SMx] = true;
            UnloadFlag[SMP1] = false;//�����װ���ʱ��ж��ܸ�λ
            UnloadInitFlag[SMP1] = false;
            return true;
          }
      break;
    
    /* �ж�SM2����λ����״̬ */
    case SM2: //SMP1
//      if(HAL_GPIO_ReadPin(LIMIT2_POS_GPIO_PORT,LIMIT2_POS_PIN) == LIMIT2_POS_ACTIVE_LEVEL)
//        if(SMotor[SMx].Dir == MOTOR_DIR_CW)
//          return true;
//      if(HAL_GPIO_ReadPin(LIMIT2_NEG_GPIO_PORT,LIMIT2_NEG_PIN) == LIMIT2_NEG_ACTIVE_LEVEL)
//        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
//          return true;
    
//      if(HAL_GPIO_ReadPin(LIMIT2_ORI_GPIO_PORT,LIMIT2_ORI_PIN) == LIMIT2_ORI_ACTIVE_LEVEL)
//      {
//        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
//          if(OriginFlag[SMP1])
//            return true;
//      }
//      else
//      {
//        if(SMotor[SMx].Dir == MOTOR_DIR_CW)
//          if(OriginFlag[SMP1])
//            return true;
//      }
      if(HAL_GPIO_ReadPin(LIMIT2_NEG_GPIO_PORT,LIMIT2_NEG_PIN) != LIMIT2_NEG_ACTIVE_LEVEL)//������ж����ѳɹ�
      {
        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
          if(UnloadFlag[SMx])//���������ж��ܿ��ƣ�����Ч
          {
            UnloadFlag[SMx] = false;
            UnloadInitFlag[SMx] = true;
            LoadFlag[SMZ1] = false;//�����ж���ʱ��װ��ܸ�λ
            LoadInitFlag[SMZ1] = false;
//            return true;
          }
      }
      else if(HAL_GPIO_ReadPin(LIMIT2_ORI_GPIO_PORT,LIMIT2_ORI_PIN) == LIMIT2_ORI_ACTIVE_LEVEL)//������֮�²�����ȡ��ܣ���ʱ�������ź���Ч���������ƶ�
      {
        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
          return true;
      }
      break;
    /* �ж�SM3����λ����״̬ */
    case SM3:
      if(HAL_GPIO_ReadPin(LIMIT3_POS_GPIO_PORT,LIMIT3_POS_PIN) == LIMIT3_POS_ACTIVE_LEVEL)
        if(SMotor[SMx].Dir == MOTOR_DIR_CW)
          return true;
      if(HAL_GPIO_ReadPin(LIMIT3_NEG_GPIO_PORT,LIMIT3_NEG_PIN) == LIMIT3_NEG_ACTIVE_LEVEL)
        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
          return true;
      if(HAL_GPIO_ReadPin(LIMIT4_NEG_GPIO_PORT,LIMIT4_NEG_PIN) == LIMIT4_NEG_ACTIVE_LEVEL)
        if(SMotor[SMx].Dir == MOTOR_DIR_CW)
          if(LoadFlag[SMx])//�������װ��ܿ��ƣ�����Ч
          {
            LoadFlag[SMx] = false;
            LoadInitFlag[SMx] = true;
            UnloadFlag[SMP2] = false;//�����װ���ʱ��ж��ܸ�λ
            UnloadInitFlag[SMP2] = false;
            return true;
          }
      break;
    
    /* �ж�SM4����λ����״̬ */
    case SM4: //SMP2
//      if(HAL_GPIO_ReadPin(LIMIT4_POS_GPIO_PORT,LIMIT4_POS_PIN) == LIMIT4_POS_ACTIVE_LEVEL)
//        if(SMotor[SMx].Dir == MOTOR_DIR_CW)
//          return true;
//      if(HAL_GPIO_ReadPin(LIMIT4_NEG_GPIO_PORT,LIMIT4_NEG_PIN) == LIMIT4_NEG_ACTIVE_LEVEL)
//        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
//          return true;
//      if(HAL_GPIO_ReadPin(LIMIT4_ORI_GPIO_PORT,LIMIT4_ORI_PIN) == LIMIT4_ORI_ACTIVE_LEVEL)
//        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
//          if(!UnloadFlag[SMx])//���δ����ж��ܿ��ƣ�����Ч
//            return true;
      if(HAL_GPIO_ReadPin(LIMIT4_NEG_GPIO_PORT,LIMIT4_NEG_PIN) != LIMIT4_NEG_ACTIVE_LEVEL)//������ж����ѳɹ�
      {
        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
          if(UnloadFlag[SMx])//���������ж��ܿ��ƣ�����Ч
          {
            UnloadFlag[SMx] = false;
            UnloadInitFlag[SMx] = true;
            LoadFlag[SMZ2] = false;//�����ж���ʱ��װ��ܸ�λ
            LoadInitFlag[SMZ2] = false;
//            return true;
          }
      }
      else if(HAL_GPIO_ReadPin(LIMIT4_ORI_GPIO_PORT,LIMIT4_ORI_PIN) == LIMIT4_ORI_ACTIVE_LEVEL)//������֮�²�����ȡ��ܣ���ʱ�������ź���Ч���������ƶ�
      {
        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
          return true;
      }
      break;
  }
  return false;
}
  
/**
  * ��������: ������������ٶȿ���
  * �������: ���������μӼ����ٶȿ�������Ҫ�Ĳ���:
  *                     @arg  Step  �ܵĲ��� ��λ��������
  *                     @arg  Accel ����ʱ�� ��λ��ms 
  *                     @arg  Decel ����ʱ�� ��λ��ms
  *                     @arg  Speed ����ٶ� ��λ��r/m
  * �� �� ֵ: ��
  * ˵    ��: �����ʼ�ٶ�ֵ,�͸�����������,������仺����,�����������
  */
void STEPMOTOR_LSCMoveRel (int16_t SMx, float step, float accel, float decel, float speed)
{
	SpeedRampData_Typedef *srd = &SMotor[SMx].srd;
	/* ��ʱ��ʱ��ԴƵ�� */
	float Timx_ClkSrc_Frq = TClkSrc_Frq[SMx]; 
	/* ��ʱ������Ƶ�� */
	float Timx_Frq = 0; 
	/* �ﵽ�趨���ٶ�ʱ��Ҫ�Ĳ��� */
	float max_s_lim;
	/* ����Ҫ��ʼ���ٵĲ������������û�дﵽ����ٶȣ�*/
	float accel_lim;
	/* Ԥ��Ƶϵ�� */
	int32_t Prescaler = 0;// Ĭ�϶�ʱ��Ԥ��Ƶ
  
	if(SMotor[SMx].MotorRuning)// ��ǰ������ת��״̬
		return ;

	/* ���ת������ */
	if(step >= 0)
	{ 
		SMotor_Set_Dir(SMx, DIR_FORWARE);
	}
	else
	{
		step = fabs(step);
    
		SMotor_Set_Dir(SMx,DIR_REVERSAL);
	}
	/* ������λ�����ж��Ƿ������������ */
	if( NotAllow2Move(SMx) )
	{
		return;
	}
  
	srd ->Speed = speed ;// ��¼�ٶ�
	srd -> Step = fabs(step);
  
	speed = speed / 60.0f ;
	accel = speed / (accel / 1000.0f);// ��λת��,ms -> r/s^2
	decel = speed / (decel / 1000.0f);// ��λת��,ms -> r/s^2
  
	srd -> Acc_Time = accel;// ��¼���ٶ�ֵ����Ҫ���ڶ��α���
	srd -> Dec_Time = decel;// ��¼���ٶ�ֵ����Ҫ���ڶ��α���
  
	if(step == 1)// ����Ϊ1
	{
		srd -> accel_count = -1;    // ֻ�ƶ�һ��
		srd -> run_state   = DECEL; // ����״̬.
		srd -> step_delay  = 0xFFFF;// ����ʱ	
	}
	else
	{
		/* ������ʼ�ٶȺ�����ٶȶ�Ӧ�Ķ�ʱ������ֵ */
		do
		{ // ��һ���ǵ�����ʱ��Ԥ��Ƶ,ʹ�������������
			Prescaler++;
			Timx_Frq = (Timx_ClkSrc_Frq / (double)Prescaler);
      
			// ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶȣ�����accel��λΪ Rev/sec^2
			// step_delay = 1/tt * sqrt(2*alpha/accel)
			// step_delay = ( tfreq*0.676 ) * sqrt( (2*alpha) / accel )
			srd -> step_delay = round((Timx_Frq * sqrt(ALPHA_x2 / accel))*0.676f);//C0,��ʼ�ٶȵĶ�ʱ��ֵ
      
		}while((uint32_t)(srd -> step_delay>>1) > UINT16_MAX);
		__HAL_TIM_SET_PRESCALER(&htimx_SMotor[SMx], --Prescaler);
		// ��������ٶȼ���.
		srd -> min_delay = round((ALPHA * Timx_Frq ) / speed);
		srd -> sec_delay = srd -> min_delay ;
    
		/* ����Ӽ�����Ҫ�Ĳ��� */
		// ������ٲ�֮��ﵽ����ٶȵ�����
		// max_s_lim = speed^2 / (2*alpha*accel)
		max_s_lim = speed * speed / (ALPHA_x2 * accel) ;
      
		// ������ٲ�֮�����Ǳ��뿪ʼ����
		// n1 = (n1+n2)decel / (accel + decel)
		accel_lim =  step * decel / (accel + decel) ;  
    
		/* ����������������Ϳ�ʼ���ٵ�λ�� */
		// ʹ�������������ǿ��Լ�������ٽ׶β���
		if(accel_lim <= max_s_lim)
		{
			srd -> decel_val = (int32_t)(accel_lim - step);
		}
		else{
			srd -> decel_val = - ( round( (max_s_lim*accel / decel) ));
		}
		// ��ֻʣ��һ���������
		if(srd -> decel_val >= -1)
		{
			srd -> decel_val = -1;
		}
      
		// ���㿪ʼ����ʱ�Ĳ���
		srd -> decel_start = (int32_t)(step + srd -> decel_val);
		// �������ٶȺ��������ǾͲ���Ҫ���м����˶�
		if(srd->step_delay <= srd->min_delay)
		{
			srd -> step_delay = srd -> min_delay;
			srd -> run_state = RUN;
		}
		else 
		{
			srd -> run_state = ACCEL;
		}
		srd -> accel_count = 0;
	}  
	Start_MotorMoving(SMx, step);
}

/**
  * ��������: DMA M0��������жϺ���
  * �������: hdma DMA���ƾ��
  * �� �� ֵ: ��
  * ˵    ��: ���뱾����˵���Ѿ���Buffer0�����ݴ������,
  *           ��ǰʹ��Buffer1��Ϊ����Դ,�����޸�Buffer0
  */
static void DMA_TC_M0CpltCallback(DMA_HandleTypeDef *hdma)
{  
  DMA_Stream_TypeDef   *Instance = hdma->Instance;
  uint16_t *ptrBuf0 = NULL;// dummy  ���ݻ���ָ��
  uint16_t *ptrBuf1 = NULL;// dummy  ���ݻ���ָ��
  uint16_t SMx = 0;        // dummy  ������
  if( hdma == &hdma_tim[SM1])
  {
    SMx = SM1;
  }
  else if( hdma == &hdma_tim[SM2] )
  {
    SMx = SM2;
  }
  else if( hdma == &hdma_tim[SM3] )
  {
    SMx = SM3;
  }
  else if( hdma == &hdma_tim[SM4] )
  {
    SMx = SM4;
  }
  ptrBuf0 = (uint16_t *)&Spd_Buffer0[SMx];// ʹ��ָ���������
  ptrBuf1 = (uint16_t *)&Spd_Buffer1[SMx];
  
  /* ��¼�жϴ���,����ֹͣ������� */
  BufferMod[SMx].quot--;
  if(BufferMod[SMx].quot <= 0)
  {
    if(BufferMod[SMx].rem <= 0) // ͬʱΪ0����ʾ�Ѿ������������
      Stop_MotorMoving(SMx);
    else // ������Ϊ0��������Ҫ�޸�DMA��������������������
    {
      /* ʣ�µ������� */
      /* ����Ҫ����ͣDMA���䣬�������ô���������Ȼ����������DMA��
       * Ҫ������ٶ�Ҫ�죬����ֱ��ʹ�üĴ�������
       */
      BufferMod[SMx].rem <<= 1;// ���µ�������������DMA����
      
      DMAx_STREAMx_DISABLE(Instance);
      DMAx_STREAMx_SET_COUNTER(Instance, BufferMod[SMx].rem);
      switch(SMx)
      {
        case SM1:DMAx_STREAMz1_CLEAR_FLAG();break;
        case SM2:DMAx_STREAMp1_CLEAR_FLAG();break;
        case SM3:DMAx_STREAMz2_CLEAR_FLAG();break;
        case SM4:DMAx_STREAMp2_CLEAR_FLAG();break;
      }
      DMAx_STREAMx_ENABLE(Instance);
      BufferMod[SMx].rem = 0;// ��һ�ν����жϾ���ֹͣ��ʱ��
    }
    return ;
  }
  
  /* �������Buffer0���������� */  
  ptrBuf0[0] = ptrBuf1[BUFFER_SIZE-1];
  LSC_CalcSpd(SMx, ptrBuf0, BUFFER_SIZE);  
//  SCB_CleanDCache_by_Addr((uint32_t*)ptrBuf0, DATAWIDTH);
}

/**
  * ��������: DMA M1��������жϺ���
  * �������: hdma DMA���ƾ��
  * �� �� ֵ: ��
  * ˵    ��: ���뱾����˵���Ѿ���Buffer1�����ݴ������,
  *           ��ǰʹ��Buffer0��Ϊ����Դ,�����޸�Buffer1
  */
static void  DMA_TC_M1CpltCallback(DMA_HandleTypeDef *hdma)
{
	DMA_Stream_TypeDef   *Instance = hdma->Instance;
  uint16_t *ptrBuf0 = NULL;
  uint16_t *ptrBuf1 = NULL;
  uint16_t SMx = 0; // dummy
  if( hdma == &hdma_tim[SM1])
  {
    SMx = SM1;
  }
  else if( hdma == &hdma_tim[SM2] )
  {
    SMx = SM2;
  }
  else if( hdma == &hdma_tim[SM3] )
  {
    SMx = SM3;
  }
  else if( hdma == &hdma_tim[SM4] )
  {
    SMx = SM4;
  }
  ptrBuf0 = (uint16_t *)&Spd_Buffer0[SMx];
  ptrBuf1 = (uint16_t *)&Spd_Buffer1[SMx];
  
  /* ��¼�жϴ���,����ֹͣ������� */
  BufferMod[SMx].quot--;
  if(BufferMod[SMx].quot<=0)
  {
    if(BufferMod[SMx].rem <= 0) // ͬʱΪ0����ʾ�Ѿ������������
      Stop_MotorMoving(SMx);
    else// ������Ϊ0��������Ҫ�޸�DMA��������������������
    {  
      /* ����Ҫ����ͣDMA���䣬�������ô���������Ȼ����������DMA��
       * Ҫ������ٶ�Ҫ�죬����ֱ��ʹ�üĴ�������
       */
      BufferMod[SMx].rem <<= 1;// ���µ�������������DMA����
      DMAx_STREAMx_DISABLE(Instance);
      DMAx_STREAMx_SET_COUNTER(Instance, BufferMod[SMx].rem);
      switch( SMx )
      {
        case SM1:DMAx_STREAMz1_CLEAR_FLAG();break;
        case SM2:DMAx_STREAMp1_CLEAR_FLAG();break;
        case SM3:DMAx_STREAMz2_CLEAR_FLAG();break;
        case SM4:DMAx_STREAMp2_CLEAR_FLAG();break;
      }
      DMAx_STREAMx_ENABLE(Instance);
      BufferMod[SMx].rem = 0;// ��һ�ν����жϾ���ֹͣ��ʱ��
    }
    return ;
  }
  /* ��¼��һ�μ������ıȽ�ֵ,������һ������ļ��� */
  ptrBuf1[0] = ptrBuf0[BUFFER_SIZE-1];
  LSC_CalcSpd(SMx, ptrBuf1, BUFFER_SIZE);  
//  SCB_CleanDCache_by_Addr((uint32_t*)ptrBuf1, DATAWIDTH);
}
/**
  * ��������: DMA �����жϺ���
  * �������: hdma DMA���ƾ��
  * �� �� ֵ: ��
  * ˵    ��: ���뱾����˵���Ѿ���Buffer1�����ݴ������,
  *           ��ǰʹ��Buffer0��Ϊ����Դ,�����޸�Buffer1
  */
static void  DMA_TC_ErrorCallback(DMA_HandleTypeDef *hdma)
{
	(void) hdma;
}


/**
  * ��������: �����ٶȻ�������ֵ
  * �������: Array �������׵�ַ
  * �� �� ֵ: ��
  * ˵    ��: ����ÿһ�����ٶ�ֵ,���뻺��������
  */
static void  LSC_CalcSpd(uint16_t SMx, uint16_t *Array, uint32_t Buffersize)
{
  int32_t i = 0;// ѭ������
  int32_t new_step_delay = 0;              // ��¼�ٶ�ֵ���
  static int32_t last_accel_delay[MOTORNUM] = {0};     // ��¼���ٽ׶ε����һ���ٶ�ֵ
  // ��¼new_step_delay�е������������һ������ľ���
  static int32_t rest[MOTORNUM] = {0};
  static int32_t AccelCNT[MOTORNUM] = {0};

  uint16_t tmpCNT = 0;
  
  tmpCNT = Array[0];
  
  SpeedRampData_Typedef *pSMotor = &SMotor[SMx].srd;
  for(i = 0; i < Buffersize; i += 2 )  
  {
    // �����£��£�һ����ʱ����
    switch(pSMotor->run_state) // �Ӽ������߽׶�
    {
      /* ����״̬ */
      case ACCEL:
        pSMotor->step_count  ++;    // ������1
        pSMotor->accel_count ++; 		// ���ټ���ֵ��1
      
        new_step_delay = pSMotor->step_delay - \
                        (((2 * pSMotor->step_delay) + rest[SMx]) / (4 * pSMotor->accel_count + 1)); //������(��)һ����������(ʱ����)
        rest[SMx] = ((2 * pSMotor->step_delay) + rest[SMx]) % (4 * pSMotor->accel_count + 1);// �����������´μ��㲹���������������
      
        /* ���ٵ���ʼ���� */
        if(pSMotor->step_count >= pSMotor->decel_start)// ��ʼ���ٵ�0
        {
          pSMotor->sec_delay = INT32_MAX;
          AccelCNT[SMx] = pSMotor->accel_count ;
          pSMotor->accel_count = pSMotor->decel_val; // ���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
          pSMotor->run_state = DECEL;           	   // �¸����������ٽ׶�
        }
        /* ���ٵ����� */
        else if(new_step_delay <= pSMotor->min_delay)// ��������������ٶȣ�����
        {
          AccelCNT[SMx] = pSMotor->accel_count ;
          pSMotor->accel_count = pSMotor->decel_val; // ���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
          last_accel_delay[SMx] = new_step_delay;
          new_step_delay = pSMotor->min_delay;   // ʹ��min_delay����Ӧ����ٶ�speed��
          rest[SMx] = 0;                         // ������ֵ
          pSMotor->run_state = RUN;              // ����Ϊ��������״̬
        }	
        break;
        
      /* ����״̬ */
      case RUN:
        pSMotor->step_count ++;      		         // ������1     
        new_step_delay = pSMotor->min_delay;     // ʹ��min_delay����Ӧ����ٶ�speed��
            
        if(pSMotor->step_count >= pSMotor->decel_start)// ��ʼ���ٵ�0
        {
          pSMotor->sec_delay = INT32_MAX;
          pSMotor->accel_count = pSMotor->decel_val; // ���ٲ�����Ϊ���ټ���ֵ
          new_step_delay = last_accel_delay[SMx]; // �ӽ׶�������ʱ��Ϊ���ٽ׶ε���ʼ��ʱ(��������)
          pSMotor->run_state = DECEL;             
        }
        /* ��;���� */
        if( pSMotor->sec_delay < pSMotor->min_delay )//���ٵ��ڶ��ٶ�
        {
          pSMotor->accel_count = AccelCNT[SMx] ;  // �������ü��ٵļ���ֵ
          new_step_delay = last_accel_delay[SMx]; // �ӽ׶�������ʱ��Ϊ���ٽ׶ε���ʼ��ʱ(��������)
          pSMotor->min_delay = pSMotor->sec_delay;
          pSMotor->run_state = ACCEL;             // ����Ϊ��������״̬
        }
        /* ��;���� */
        if( pSMotor->sec_delay > pSMotor->min_delay)// ���ٵ��ڶ��ٶ�
        {
          AccelCNT[SMx] = pSMotor->accel_count ;      // ���ʱ���accel_count�Ǽ��ٵ�sec_delay�Ĳ���
          pSMotor->accel_count = pSMotor->decel_val; // ���ٲ�����Ϊ���ټ���ֵ
          new_step_delay = last_accel_delay[SMx]; // �ӽ׶�������ʱ��Ϊ���ٽ׶ε���ʼ��ʱ(��������)
          
          pSMotor->run_state = DECEL;            // ״̬�ı�Ϊ����
          
        }
        break;
        
      /* ����״̬ */
      case DECEL:
        pSMotor->step_count ++;     // ������1
        pSMotor->accel_count++; 	  // ���ټ���ֵ��1
      
        new_step_delay = pSMotor->step_delay - \
                        (((2 * pSMotor->step_delay) + rest[SMx]) / (4 * pSMotor->accel_count + 1)); //������(��)һ����������(ʱ����)
        rest[SMx] = ((2 * pSMotor->step_delay) + rest[SMx]) % (4 * pSMotor->accel_count + 1);// �����������´μ��㲹���������������
        //����Ƿ�Ϊ���һ��,���һ��,����ѭ��,����Ҫ�ټ���
        if(pSMotor->accel_count > 0)
        {
          /* ֹͣ״̬ */
          pSMotor->run_state = STOP;
          pSMotor->step_count = 0;   // ���㲽��������
          rest[SMx] = 0;              // ������ֵ
          last_accel_delay[SMx] = 0;
          i += Buffersize;      // ����ѭ��
          return;
        }
        /* ���ٵ��ڶ��ٶ� */
        if(new_step_delay >= pSMotor->sec_delay) 
        {
          last_accel_delay[SMx] = new_step_delay;
          pSMotor->min_delay = pSMotor->sec_delay;
          new_step_delay = pSMotor->min_delay;   // ʹ��min_delay����Ӧ����ٶ�speed��
          pSMotor->decel_val = pSMotor->accel_count ;// ����ʣ�ಽ��
          pSMotor->decel_start = pSMotor->Step + pSMotor->decel_val;// �޸ļ��ٵ�0����ʼλ�� 
          rest[SMx] = 0;                          // ������ֵ
          pSMotor->run_state = RUN;              // ����Ϊ��������״̬
        }
        break;
      default :break;
    }
    /* ���»�����,���αȽ�ֵ��һ���������� */
    Array[i]   = (uint16_t)(tmpCNT + (pSMotor->step_delay >> 1));
    Array[i+1] = (uint16_t)(Array[i] + (pSMotor->step_delay >> 1));
    pSMotor->step_delay = new_step_delay; // Ϊ�¸�(�µ�)��ʱ(��������)��ֵ
    tmpCNT = Array[i+1];
  }
}

/*-------------------------------- ���μ��ٿ��� ------------------------------*/

/**
  * ��������: ������������ٶȼ���
  * �������: pSMotor ���������μӼ����ٶȿ�������Ҫ�Ĳ���������:
  *                     @arg  Step  �ܵĲ���// ��λ�� Pulse
  *                     @arg  Accel ����ʱ��// ��λ�� r/s^2
  *                     @arg  Decel ����ʱ��// ��λ�� r/s^2
  *                     @arg  Speed ����ٶ�// ��λ�� r/m
  * �� �� ֵ: ��
  * ˵    ��: �����ʼ�ٶ�ֵ,�͸�����������,������仺����,�����������
  */
void STEPMOTOR_LSC_Calc(uint16_t SMx, SpeedRampData_Typedef *pSMotor)
{
  float step  = pSMotor -> Step;
  float speed = pSMotor -> Speed / 60.0f ; // ת��Ϊ r/s
  float accel = pSMotor -> Acc_Time ;
  float decel = pSMotor -> Dec_Time ;
  
	/* ��ʱ��ʱ��ԴƵ�� */
	float Timx_ClkSrc_Frq = TClkSrc_Frq[SMx]; 
  /* ��ʱ������Ƶ�� */
  float Timx_Frq = 0; 
  /* �ﵽ�趨���ٶ�ʱ��Ҫ�Ĳ��� */
	float max_s_lim;
  /* ����Ҫ��ʼ���ٵĲ������������û�дﵽ����ٶȣ�*/
	float accel_lim;
  /* Ԥ��Ƶϵ�� */
  int32_t Prescaler = 0;// ��ʱ��Ԥ��Ƶ, ��0��ʼ������ʵ�Ԥ��Ƶ
  
  /* ������ʼ�ٶȺ�����ٶȶ�Ӧ�Ķ�ʱ������ֵ */
  do
  { 
    Prescaler++;
    Timx_Frq = (Timx_ClkSrc_Frq / (double)Prescaler);
    
    pSMotor -> step_delay = round((Timx_Frq * sqrt(ALPHA_x2 / accel))*0.676f);//C0,��ʼ�ٶȵĶ�ʱ��ֵ
    
  }while((uint32_t)(pSMotor -> step_delay>>1) > UINT16_MAX);
  __HAL_TIM_SET_PRESCALER(&htimx_SMotor[SM1], --Prescaler);
  // ��������ٶȼ���.
  pSMotor -> min_delay = round((ALPHA * Timx_Frq ) / speed);
  
  /* ����Ӽ�����Ҫ�Ĳ��� */
  // ������ٲ�֮��ﵽ����ٶȵ�����
  max_s_lim = speed * speed / (ALPHA_x2 * accel) ;
    
  // ������ٲ�֮����뿪ʼ����
  accel_lim =  step * decel / (accel + decel) ;  
  
  /* ����������������Ϳ�ʼ���ٵ�λ�� */
  // ʹ�������������Լ�������ٽ׶β���
  if(accel_lim <= max_s_lim)
  {
    pSMotor -> decel_val = (int32_t)(accel_lim - step);
    pSMotor -> accel_count = accel_lim; // ��¼���ٲ���
  }
  else
  {
    pSMotor -> decel_val = - ( round( (max_s_lim*accel / decel) ));
    pSMotor -> accel_count = max_s_lim;// ��¼���ٲ���
  }
  // ��ֻʣ��һ���������
  if(pSMotor -> decel_val >= -1)
  {
    pSMotor -> decel_val = -1;
  }  
}
/**
  * ��������: �޸��˶�����
  * �������: SMx ������  Secspd ���ٺ���ٶ�ֵ Step ���ٺ��ߵĲ���
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void Modify_MovingParam(uint16_t SMx, float Secspd ,int32_t Step)
{
  uint32_t bufCNT = __HAL_DMA_GET_COUNTER(&hdma_tim[SMx]);// ��ȡDMA�Ѿ������������
  SpeedRampData_Typedef  srd; // ���ڼ�����ٺ��һЩ����
  Step = abs(Step);// �з�����ȡ����ֵ
  
  srd.Speed    = Secspd; //  Rev/min
  srd.Step     = Step;   //  ת���Ĳ���
  srd.Acc_Time = SMotor[SMx].srd.Acc_Time;// ��λ��r/s^2
  srd.Dec_Time = SMotor[SMx].srd.Dec_Time;// ��λ��r/s^2
  STEPMOTOR_LSC_Calc(SMx, (SpeedRampData_Typedef*)&srd); 
  
  /* ����DMAʣ��������������жϵ�ǰ����������Ƿ�Ϊһ������������ */
  /* ��Ҫʹ�õ�ǰ���������������������Ѿ������������ */
  if(bufCNT & 0x01)  
  {
    --bufCNT; //  �ٴ���һ������֮���������������
  }
  else 
  {
    bufCNT -= 2; // �ٴ�����������֮���������������
  }
  SMotor[SMx].srd.Speed = Secspd;
  SMotor[SMx].srd.sec_delay   = srd.min_delay; // �ڶ������ٵ��ٶ�ֵ
  SMotor[SMx].srd.step_count -= ((BUFFER_SIZE+bufCNT)>>1);// ��ǰ�������������
  SMotor[SMx].srd.Step = SMotor[SMx].srd.step_count + 1 + srd.Step;// �����ܵ�������
  // ���ٿ���
  if(srd.min_delay < SMotor[SMx].srd.min_delay) 
  {
    SMotor[SMx].srd.decel_val   = srd.decel_val; // ������Ҫ�޸ļ������貽��   
    SMotor[SMx].srd.decel_start = SMotor[SMx].srd.Step + srd.decel_val;// ��ʼ���ٵ�λ��
  }
  else 
  {
    //���ٲ���Ҫ�޸ļ������貽��
    SMotor[SMx].srd.accel_count = srd.accel_count;// ���Ǽ�����ٵĲ���
  }
  
  /* ���¼���DMA�жϴ��� */
  BufferMod[SMx] = div(Step, (BUFFER_SIZE>>1) );// ��������

}

/**
  * ��������: ������μ��ٿ���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ���¼����ٶ�ֵ����ͣDMA���л�������֮����������DMA
  */
void Modify_MotorMoving(uint16_t SMx, float Secspd ,int32_t Step)
{ 
  bool      tmpCT        = 0;    // ��ǰ����Դ
  uint32_t  tmpNDTR      = 0;    // һƬ��������ʣ��������������
  uint16_t  *ptrNewbuf   = NULL; // �µĻ�����ָ��
  uint16_t  *ptrNextbuf  = NULL; // ��һƬ������ָ��
  uint32_t  tmp          = 0;    
  
  DMA_Stream_TypeDef *DMAInstance = hdma_tim[SMx].Instance; // DMAʵ��
  uint32_t  tmpDMAxCR    = DMAInstance->CR; // DMA->CR ���üĴ���

  MotorRunSta_Typedef tmpRunState = SMotor[SMx].srd.run_state;// ���״̬
  
  
  /* ֻ���ڼ��ٺ�����״̬�²�������� */
  if((tmpRunState == STOP)||(tmpRunState == DECEL)  ) 
  {
    return;
  }
  
  /* �����µ��ٶ�ֵ���޸�DMA������� */
  Modify_MovingParam(SMx, Secspd, Step);// �����µ��ٶȲ���
  
  /* ��ȡ��ǰʹ�õ����ĸ������� */
  tmpCT = DMAx_STREAMx_GET_CT(DMAInstance);
  if(tmpCT)
  {
    ptrNewbuf  = (uint16_t *)&Spd_Buffer0[SMx];// ʹ�������һ���������ݴ��ٶ�ֵ
    ptrNextbuf = (uint16_t *)&Spd_Buffer1[SMx];
    tmpDMAxCR &= (~DMA_SxCR_CT); // Ԥ��M0Ϊ�µ�����Դ
  }
  else
  {
    ptrNewbuf  = (uint16_t *)&Spd_Buffer1[SMx]; 
    ptrNextbuf = (uint16_t *)&Spd_Buffer0[SMx];
    tmpDMAxCR |= (DMA_SxCR_CT); // Ԥ��M1Ϊ�µ�����Դ
  }
  
  /* �ȴ����һ������������ */
  tmpNDTR = __HAL_DMA_GET_COUNTER(&hdma_tim[SMx]);
  if(tmpNDTR & 0x01) // ����
  {
    while((__HAL_DMA_GET_COUNTER(&hdma_tim[SMx]) & 0x01))// ���� // ��������Ϊż����Ϊ���һ������������   
    {}
  }
  else
  {
    tmp = tmpNDTR - 2;
    while(__HAL_DMA_GET_COUNTER(&hdma_tim[SMx]) != tmp)// ż��//���һ������������
    {}
  }
  
    /* �޸�DMA�Ĳ�����ҪѸ����ɣ��������潫ֱ�Ӳ����Ĵ��� 
     * ���м���2�����ݣ����л�����֮���ټ���ʣ����ٶ�ֵ��
     * �����ԣ�����ĳ����ʱ��Լ600~700ns��������DMA�����ʱ�������ܵ���700ns��
     * �����˼���ǰ�����ܵ���700ns��
     */
  if(SMx == SM1)
  {
    *ptrNewbuf = *SMOTOR1_TIMx_CCxADDRESS;
    LSC_CalcSpd(SMx, ptrNewbuf, 2);   // �ȼ���2����������
//    SCB_CleanDCache_by_Addr((uint32_t*)ptrNewbuf, 4);

    /* ��ֹͣDMA���䣬��Դ��ַ����Ϊ��һƬ��������Ȼ����������DMA */
    DMAx_STREAMx_DISABLE_TC(DMAInstance);
    DMAx_STREAMz1_CLEAR_FLAG(); // ��ձ�־λ
    
  }else if( SMx == SM2 )
  {
    *ptrNewbuf = *SMOTOR2_TIMx_CCxADDRESS;
    LSC_CalcSpd(SMx, ptrNewbuf, 2);
//    SCB_CleanDCache_by_Addr((uint32_t*)ptrNewbuf, 4);

    /* ��ֹͣDMA���䣬��Դ��ַ����Ϊ��һƬ��������Ȼ����������DMA */
    DMAx_STREAMx_DISABLE_TC(DMAInstance);
    DMAx_STREAMp1_CLEAR_FLAG(); // ��ձ�־λ
  }
  else if(SMx == SM3)
  {
    *ptrNewbuf = *SMOTOR3_TIMx_CCxADDRESS;
    LSC_CalcSpd(SMx, ptrNewbuf, 2);   // �ȼ���2����������
//    SCB_CleanDCache_by_Addr((uint32_t*)ptrNewbuf, 4);

    /* ��ֹͣDMA���䣬��Դ��ַ����Ϊ��һƬ��������Ȼ����������DMA */
    DMAx_STREAMx_DISABLE_TC(DMAInstance);
    DMAx_STREAMz2_CLEAR_FLAG(); // ��ձ�־λ
    
  }else if( SMx == SM4 )
  {
    *ptrNewbuf = *SMOTOR4_TIMx_CCxADDRESS;
    LSC_CalcSpd(SMx, ptrNewbuf, 2);
//    SCB_CleanDCache_by_Addr((uint32_t*)ptrNewbuf, 4);

    /* ��ֹͣDMA���䣬��Դ��ַ����Ϊ��һƬ��������Ȼ����������DMA */
    DMAx_STREAMx_DISABLE_TC(DMAInstance);
    DMAx_STREAMp2_CLEAR_FLAG(); // ��ձ�־λ
  }
  
  /* ���Ļ�����������ʹ��DMA���ж� */
  DMAInstance->CR = tmpDMAxCR;

  /* ---------------------  */
  
  /* ����ʣ���������ٶ�ֵ*/
  ptrNewbuf+=2;
  *(ptrNewbuf) = *(ptrNewbuf - 1);
  LSC_CalcSpd(SMx, ptrNewbuf, BUFFER_SIZE - 2);  
//  SCB_CleanDCache_by_Addr((uint32_t*)ptrNewbuf, (BUFFER_SIZE-2)*(sizeof(ptrNewbuf[0])));
//	MemClr((uint8_t*)ptrNewbuf, (BUFFER_SIZE-2)*(sizeof(ptrNewbuf[0])));
  
  *ptrNextbuf = *(ptrNewbuf + BUFFER_SIZE - 3);// ��һƬ���������ٶ�ֵ
  LSC_CalcSpd(SMx, ptrNextbuf, BUFFER_SIZE);  
//  SCB_CleanDCache_by_Addr((uint32_t*)ptrNextbuf, DATAWIDTH);
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/

