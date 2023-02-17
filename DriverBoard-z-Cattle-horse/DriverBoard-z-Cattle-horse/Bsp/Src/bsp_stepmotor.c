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
	 
/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_stepmotor.h"
#include "bsp_led.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_limit.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
/**
  * 数学常数.修改Alpha即可修改速度单位
  * 例如 :ALPHA = 360/SMOTOR1_PULSEREV,速度值单位就是°/s 
  *       ALPHA = 1/SMOTOR1_PULSEREV,就是Rev/s,速度值单位就是Rev/s 
  * ALPHA 定义是脉冲当量，也就是每一个脉冲对应电机转动的量
  */
#define ALPHA                 ((float)(1.0f / (float)SMOTOR1_PULSE_REV)) // α= 360/PPR步距角
#define ALPHA_x2              ((float)(2.0f * ALPHA))

/* 私有变量 ------------------------------------------------------------------*/

TIM_HandleTypeDef             htimx_SMotor[MOTORNUM]; // 定时器控制句柄
DMA_HandleTypeDef             hdma_tim[MOTORNUM];     // DMA控制句柄
MotorObj_Typedef              SMotor[MOTORNUM];       // 步进电机控制句柄
MoveRunData_Typedef mrd_SMotor[MOTORNUM] = {{SMOTOR1_LIM_SPEED,SMOTOR1_CRAML_SPEED,SMOTOR1_ACC_TIME,SMOTOR1_DEC_TIME,SMOTOR1_DISTANCE_REV,SMOTOR1_STEP_ANGLE,SMOTOR1_MICRO_STEP},
                                            {SMOTOR2_LIM_SPEED,SMOTOR2_CRAML_SPEED,SMOTOR2_ACC_TIME,SMOTOR2_DEC_TIME,SMOTOR2_DISTANCE_REV,SMOTOR2_STEP_ANGLE,SMOTOR2_MICRO_STEP},
                                            {SMOTOR3_LIM_SPEED,SMOTOR3_CRAML_SPEED,SMOTOR3_ACC_TIME,SMOTOR3_DEC_TIME,SMOTOR3_DISTANCE_REV,SMOTOR3_STEP_ANGLE,SMOTOR3_MICRO_STEP},
                                            {SMOTOR4_LIM_SPEED,SMOTOR4_CRAML_SPEED,SMOTOR4_ACC_TIME,SMOTOR4_DEC_TIME,SMOTOR4_DISTANCE_REV,SMOTOR4_STEP_ANGLE,SMOTOR4_MICRO_STEP}};
PipetteRunData_Typedef prd_SMotor[PIPNUM] = {{PIPETTE1_VOLUME_DISTANCE_RATIO,PIPETTE1_MAX_STROKE,PIPETTE1_UNLOAD_DISTANCE},
                                             {PIPETTE2_VOLUME_DISTANCE_RATIO,PIPETTE2_MAX_STROKE,PIPETTE2_UNLOAD_DISTANCE}};

float TClkSrc_Frq[MOTORNUM] = {SMOTOR1_TIM_CLKSRC_FREQ, SMOTOR2_TIM_CLKSRC_FREQ, SMOTOR3_TIM_CLKSRC_FREQ, SMOTOR4_TIM_CLKSRC_FREQ};  //定时器时钟源频率

/**
  * DMA传输的内存地址不能是0x2000 0000~0x2001 FFFF,在这个地址范围内是属于DTCM，
  * 而DMA1,2都没有直接连接到DTCM，所以DMA1,2都不能直接访问DTCM，但是DMA有连接到
  * AXI SRAM、SRAM1/2/3/4，而AXI SRAM的地址范围就是0x2400 0000 - 0x2407 FFFF,
  * 所以下面就是将缓存定义在AXI SRAM中。
  */
// Buffer0[0],Buffer1[0]是SM1的专用缓存区，Buffer0[1],Buffer1[1]是SM2的专用缓存区，
#if defined (__ICCARM__)    /* IAR Compiler */
/* IAR 编译所用 */
// 定义变量地址在0x24000000之后,占用AXI-SRAM
__IO uint16_t Spd_Buffer0[BUFFER_SIZE][BUFFER_SIZE]  @ "AXISRAM";
__IO uint16_t Spd_Buffer1[BUFFER_SIZE][BUFFER_SIZE]  @ "AXISRAM";

#elif defined   (__CC_ARM)  /* ARM Compiler */
/* Keil 编译所用 */  
__attribute__ ((section("AXISRAM"))) // 定义变量地址在0x24000000之后,占用AXI-SRAM
__IO uint16_t Spd_Buffer0[MOTORNUM][BUFFER_SIZE] = {0};

__attribute__ ((section("AXISRAM"))) // 可以定义在0x2400 0000 ~ 0x2408 0000之间
__IO uint16_t Spd_Buffer1[MOTORNUM][BUFFER_SIZE] = {0};
#endif 


__IO div_t BufferMod[MOTORNUM]  = {0}; // 中断次数,余数

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

static void DMAx_Config(uint16_t SMotorNum);   // DMA配置
static void DMA_TC_M0CpltCallback(DMA_HandleTypeDef *hdma);// M0传输完成回调函数
static void DMA_TC_M1CpltCallback(DMA_HandleTypeDef *hdma);// M1传输完成回调函数
static void DMA_TC_ErrorCallback(DMA_HandleTypeDef *hdma);//传输错误调用函数
static void LSC_CalcSpd(uint16_t SMx, uint16_t  *Array,uint32_t Buffersize);              // 计算速度值
static void SMotor_GPIO_MspInit(uint16_t SMotorNum);   // 引脚初始化
static void Start_MotorMoving(uint16_t SMx, int32_t step);// 启动电机

/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: DMA配置
  * 输入参数: SMotorNum 电机编号
  * 返 回 值: 无
  * 说    明: 初始化配置DMA功能
  */
static void DMAx_Config(uint16_t SMx)
{
  DMA_HandleTypeDef  *ptrdma = &hdma_tim[SMx];// DMA控制句柄
  IRQn_Type DMA_Stream_IRQn = (IRQn_Type)0;   // 初始化为0，使用变量初始化
  uint16_t DMA_TIM_ID = 0;                    // 初始化为0
  
  switch(SMx)
  {
    /* 步进电机1的相关配置 */
    case SM1:
      DMAx_CHANNELz1_CLK_ENABLE();           // 使能DMAx 的时钟
      DMA_Stream_IRQn = DMAx_Streamz1_IRQn;  // 使用SM1的相关DMA配置.
      DMA_TIM_ID = DMAx_TIMz1_ID;
      
      ptrdma->Instance      = DMAx_Streamz1;
      ptrdma->Init.Channel  = DMAx_TIMz1_REQUEST_CHx;
      break;
    /* 步进电机2的相关配置 */
    case SM2:
      DMAx_CHANNELp1_CLK_ENABLE();            // 使能DMAx 的时钟
      DMA_Stream_IRQn = DMAx_Streamp1_IRQn;
      DMA_TIM_ID = DMAx_TIMp1_ID;
    
      ptrdma->Instance      = DMAx_Streamp1;
      ptrdma->Init.Channel  = DMAx_TIMp1_REQUEST_CHx;
      break;
    /* 步进电机3的相关配置 */
    case SM3:
      DMAx_CHANNELz2_CLK_ENABLE();           // 使能DMAx 的时钟
      DMA_Stream_IRQn = DMAx_Streamz2_IRQn;  // 使用SM1的相关DMA配置.
      DMA_TIM_ID = DMAx_TIMz2_ID;
      
      ptrdma->Instance      = DMAx_Streamz2;
      ptrdma->Init.Channel  = DMAx_TIMz2_REQUEST_CHx;
      break;
    /* 步进电机4的相关配置 */
    case SM4:
      DMAx_CHANNELp2_CLK_ENABLE();            // 使能DMAx 的时钟
      DMA_Stream_IRQn = DMAx_Streamp2_IRQn;
      DMA_TIM_ID = DMAx_TIMp2_ID;
    
      ptrdma->Instance      = DMAx_Streamp2;
      ptrdma->Init.Channel  = DMAx_TIMp2_REQUEST_CHx;
      break;
  }
  
  /*********** 配置通用DMA参数 ***********************/
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
  
  /* 链接DMA与TIM */
  __HAL_LINKDMA(&htimx_SMotor[SMx], hdma[DMA_TIM_ID], *ptrdma);

  HAL_DMA_Init(ptrdma);

  /* 配置M0中断调用函数 */
  ptrdma -> XferCpltCallback   = DMA_TC_M0CpltCallback;
  /* 配置M1中断调用函数 */
  ptrdma -> XferM1CpltCallback = DMA_TC_M1CpltCallback;
  /* 配置传输错误调用函数 */
  ptrdma -> XferErrorCallback = DMA_TC_ErrorCallback;
  
  /* 配置DMA中断 */
  HAL_NVIC_SetPriority(DMA_Stream_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA_Stream_IRQn);  
  
}

/**
  * 函数功能: 定时器硬件初始化配置
  * 输入参数: SMotorNum 电机编号
  * 返 回 值: 无
  * 说    明: 无
  */
static void  SMotor_GPIO_MspInit(uint16_t SMx)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  switch(SMx)
  {
    case SM1:// StepMotor1
      /* 定时器引脚时钟使能 */
      SMOTOR1_PUL_GPIO_CLK_ENABLE();
      SMOTOR1_DIR_GPIO_CLK_ENABLE();
      SMOTOR1_ENA_GPIO_CLK_ENABLE();
    
      /* 脉冲输出引脚配置 */  
      GPIO_InitStruct.Pin       = SMOTOR1_PUL_GPIO_PIN ;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;      // 输入模式
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      GPIO_InitStruct.Alternate = SMOTOR1_PUL_PIN_AF; // GPIO引脚用做TIM复用功能
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(SMOTOR1_PUL_GPIO_PORT, &GPIO_InitStruct);
      
      /* 方向控制引脚配置 */  
      GPIO_InitStruct.Pin       = SMOTOR1_DIR_GPIO_PIN;
      GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Alternate = 0;        
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      HAL_GPIO_Init(SMOTOR1_DIR_GPIO_PORT, &GPIO_InitStruct);
      
      /* 电机使能引脚配置 */  
      GPIO_InitStruct.Pin       = SMOTOR1_ENA_GPIO_PIN;
      HAL_GPIO_Init(SMOTOR1_ENA_GPIO_PORT, &GPIO_InitStruct);
      
      break;
    case SM2:// StepMotor2
      /* 定时器引脚时钟使能 */
      SMOTOR2_PUL_GPIO_CLK_ENABLE();
      SMOTOR2_DIR_GPIO_CLK_ENABLE();
      SMOTOR2_ENA_GPIO_CLK_ENABLE();
    
      /* 脉冲输出引脚配置 */  
      GPIO_InitStruct.Pin       = SMOTOR2_PUL_GPIO_PIN ;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;      // 输入模式
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      GPIO_InitStruct.Alternate = SMOTOR2_PUL_PIN_AF; // GPIO引脚用做TIM复用功能
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(SMOTOR2_PUL_GPIO_PORT, &GPIO_InitStruct);
      
      /* 方向控制引脚配置 */  
      GPIO_InitStruct.Pin       = SMOTOR2_DIR_GPIO_PIN;
      GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Alternate = 0;        
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      HAL_GPIO_Init(SMOTOR2_DIR_GPIO_PORT, &GPIO_InitStruct);
      
      /* 电机使能引脚配置 */  
      GPIO_InitStruct.Pin       = SMOTOR2_ENA_GPIO_PIN;
      HAL_GPIO_Init(SMOTOR2_ENA_GPIO_PORT, &GPIO_InitStruct);
      break;
    case SM3:// StepMotor3
      /* 定时器引脚时钟使能 */
      SMOTOR3_PUL_GPIO_CLK_ENABLE();
      SMOTOR3_DIR_GPIO_CLK_ENABLE();
      SMOTOR3_ENA_GPIO_CLK_ENABLE();
    
      /* 脉冲输出引脚配置 */  
      GPIO_InitStruct.Pin       = SMOTOR3_PUL_GPIO_PIN ;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;      // 输入模式
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      GPIO_InitStruct.Alternate = SMOTOR3_PUL_PIN_AF; // GPIO引脚用做TIM复用功能
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(SMOTOR3_PUL_GPIO_PORT, &GPIO_InitStruct);
      
      /* 方向控制引脚配置 */  
      GPIO_InitStruct.Pin       = SMOTOR3_DIR_GPIO_PIN;
      GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Alternate = 0;        
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      HAL_GPIO_Init(SMOTOR3_DIR_GPIO_PORT, &GPIO_InitStruct);
      
      /* 电机使能引脚配置 */  
      GPIO_InitStruct.Pin       = SMOTOR3_ENA_GPIO_PIN;
      HAL_GPIO_Init(SMOTOR3_ENA_GPIO_PORT, &GPIO_InitStruct);
      
      break;
    case SM4:// StepMotor4
      /* 定时器引脚时钟使能 */
      SMOTOR4_PUL_GPIO_CLK_ENABLE();
      SMOTOR4_DIR_GPIO_CLK_ENABLE();
      SMOTOR4_ENA_GPIO_CLK_ENABLE();
    
      /* 脉冲输出引脚配置 */  
      GPIO_InitStruct.Pin       = SMOTOR4_PUL_GPIO_PIN ;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;      // 输入模式
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      GPIO_InitStruct.Alternate = SMOTOR4_PUL_PIN_AF; // GPIO引脚用做TIM复用功能
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(SMOTOR4_PUL_GPIO_PORT, &GPIO_InitStruct);
      
      /* 方向控制引脚配置 */  
      GPIO_InitStruct.Pin       = SMOTOR4_DIR_GPIO_PIN;
      GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Alternate = 0;        
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      HAL_GPIO_Init(SMOTOR4_DIR_GPIO_PORT, &GPIO_InitStruct);
      
      /* 电机使能引脚配置 */  
      GPIO_InitStruct.Pin       = SMOTOR4_ENA_GPIO_PIN;
      HAL_GPIO_Init(SMOTOR4_ENA_GPIO_PORT, &GPIO_InitStruct);
      break;
  }
}

/**
  * 函数功能: 步进电机控制定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 初始化PWM输出功能
  */
void SMOTOR1_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_BreakDeadTimeConfigTypeDef sconfigBDTR;
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_MasterConfigTypeDef sMasterConfig;
  /* 定时器硬件配置 */
  SMotor_GPIO_MspInit(SM1);
  
  /* 定时器外设配置 */
  SMOTOR1_TIM_RCC_CLK_ENABLE();
  htimx_SMotor[SM1].Instance               = SMOTOR1_TIMx;
  htimx_SMotor[SM1].Init.Prescaler         = SMOTOR1_TIM_PRESCALER;
  htimx_SMotor[SM1].Init.CounterMode       = TIM_COUNTERMODE_UP;
  htimx_SMotor[SM1].Init.Period            = SMOTOR1_TIM_PERIOD;
  htimx_SMotor[SM1].Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htimx_SMotor[SM1].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//禁用预分频    //error
  HAL_TIM_PWM_Init(&htimx_SMotor[SM1]);

  /* 时钟源配置 */
  sClockSourceConfig.ClockSource    = TIM_CLOCKSOURCE_INTERNAL; // 内部时钟
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;  // 无时钟预分频
  sClockSourceConfig.ClockPolarity  = TIM_CLOCKPOLARITY_INVERTED;// 无效配置
  sClockSourceConfig.ClockFilter    = 0x0;  
  HAL_TIM_ConfigClockSource(&htimx_SMotor[SM1], &sClockSourceConfig);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_SMotor[SM1],&sMasterConfig);
  
  /* 死区时间和状态配置 */
  /** 这里主要配置高级定时器的互补通道输出控制,对于刹车功能的配置是无效的
    * 如果使用非高级定时器,可以注释掉这段代码
    */
  sconfigBDTR.OffStateRunMode  = TIM_OSSR_DISABLE;  // 关闭时的运行模式输出极性 
  sconfigBDTR.OffStateIDLEMode = TIM_OSSI_DISABLE;  // 关闭时的空闲模式输出极性
  sconfigBDTR.BreakState       = TIM_BREAK_DISABLE; // 不使用刹车功能
  sconfigBDTR.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
//  sconfigBDTR.BreakFilter      = 0;
  sconfigBDTR.DeadTime         = 0; // Min_Data = 0x00 and Max_Data = 0xFF
  sconfigBDTR.LockLevel        = TIM_LOCKLEVEL_OFF;
//  sconfigBDTR.Break2Filter     = 0; // Min_Data = 0x0 and Max_Data = 0xF
//  sconfigBDTR.Break2Polarity   = TIM_BREAK2POLARITY_LOW;
//  sconfigBDTR.Break2State      = TIM_BREAK2_DISABLE;
  sconfigBDTR.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx_SMotor[SM1],&sconfigBDTR);
  
  /* PWM配置,CH1 PWM Mode*/
  sConfigOC.OCMode       = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse        = SMOTOR1_TIM_DEFAULT_PULSE;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htimx_SMotor[SM1], &sConfigOC, SMOTOR1_PUL_TIMx_CHANNELx);
  
  DMAx_Config(SM1);
    
  /* 配置电机默认状态,默认使能电机转动,并且方向是顺时针转动 */
  SMotor_Set_Dir  ( SM1, MOTOR_DIR_CW);
  SMotor_Set_State( SM1, MOTOR_DISABLE);
}

/**
  * 函数功能: 步进电机控制定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 初始化PWM输出功能
  */
void SMOTOR2_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
  /* 定时器硬件配置 */
  SMotor_GPIO_MspInit(SM2);
  
  /* 定时器外设配置 */
  SMOTOR2_TIM_RCC_CLK_ENABLE();
  htimx_SMotor[SM2].Instance               = SMOTOR2_TIMx;
  htimx_SMotor[SM2].Init.Prescaler         = SMOTOR2_TIM_PRESCALER;
  htimx_SMotor[SM2].Init.CounterMode       = TIM_COUNTERMODE_UP;
  htimx_SMotor[SM2].Init.Period            = SMOTOR2_TIM_PERIOD;
  htimx_SMotor[SM2].Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htimx_SMotor[SM2].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//禁用预分频
  HAL_TIM_PWM_Init(&htimx_SMotor[SM2]);

  /* 时钟源配置 */
  sClockSourceConfig.ClockSource    = TIM_CLOCKSOURCE_INTERNAL; // 内部时钟
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;  // 无时钟预分频
  sClockSourceConfig.ClockPolarity  = TIM_CLOCKPOLARITY_INVERTED;// 无效配置
  sClockSourceConfig.ClockFilter    = 0x0;  
  HAL_TIM_ConfigClockSource(&htimx_SMotor[SM2], &sClockSourceConfig);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_SMotor[SM2],&sMasterConfig);
  
  /* PWM配置,CH1 PWM Mode*/
  sConfigOC.OCMode       = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse        = SMOTOR2_TIM_DEFAULT_PULSE;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htimx_SMotor[SM2], &sConfigOC, SMOTOR2_PUL_TIMx_CHANNELx);
  
  DMAx_Config(SM2);
    
  /* 配置电机默认状态,默认禁止电机转动,并且方向是顺时针转动 */
  SMotor_Set_Dir  ( SM2, MOTOR_DIR_CW);
  SMotor_Set_State( SM2, MOTOR_DISABLE);
}

/**
  * 函数功能: 步进电机控制定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 初始化PWM输出功能
  */
void SMOTOR3_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
  /* 定时器硬件配置 */
  SMotor_GPIO_MspInit(SM3);
  
  /* 定时器外设配置 */
  SMOTOR3_TIM_RCC_CLK_ENABLE();
  htimx_SMotor[SM3].Instance               = SMOTOR3_TIMx;
  htimx_SMotor[SM3].Init.Prescaler         = SMOTOR3_TIM_PRESCALER;
  htimx_SMotor[SM3].Init.CounterMode       = TIM_COUNTERMODE_UP;
  htimx_SMotor[SM3].Init.Period            = SMOTOR3_TIM_PERIOD;
  htimx_SMotor[SM3].Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htimx_SMotor[SM3].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//禁用预分频
  HAL_TIM_PWM_Init(&htimx_SMotor[SM3]);

  /* 时钟源配置 */
  sClockSourceConfig.ClockSource    = TIM_CLOCKSOURCE_INTERNAL; // 内部时钟
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;  // 无时钟预分频
  sClockSourceConfig.ClockPolarity  = TIM_CLOCKPOLARITY_INVERTED;// 无效配置
  sClockSourceConfig.ClockFilter    = 0x0;  
  HAL_TIM_ConfigClockSource(&htimx_SMotor[SM3], &sClockSourceConfig);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_SMotor[SM3],&sMasterConfig);
  
  /* PWM配置,CH1 PWM Mode*/
  sConfigOC.OCMode       = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse        = SMOTOR3_TIM_DEFAULT_PULSE;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htimx_SMotor[SM3], &sConfigOC, SMOTOR3_PUL_TIMx_CHANNELx);
  
  DMAx_Config(SM3);
    
  /* 配置电机默认状态,默认禁止电机转动,并且方向是顺时针转动 */
  SMotor_Set_Dir  ( SM3, MOTOR_DIR_CW);
  SMotor_Set_State( SM3, MOTOR_DISABLE);
}

/**
  * 函数功能: 步进电机控制定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 初始化PWM输出功能
  */
void SMOTOR4_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
  /* 定时器硬件配置 */
  SMotor_GPIO_MspInit(SM4);
  
  /* 定时器外设配置 */
  SMOTOR4_TIM_RCC_CLK_ENABLE();
  htimx_SMotor[SM4].Instance               = SMOTOR4_TIMx;
  htimx_SMotor[SM4].Init.Prescaler         = SMOTOR4_TIM_PRESCALER;
  htimx_SMotor[SM4].Init.CounterMode       = TIM_COUNTERMODE_UP;
  htimx_SMotor[SM4].Init.Period            = SMOTOR4_TIM_PERIOD;
  htimx_SMotor[SM4].Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htimx_SMotor[SM4].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//禁用预分频
  HAL_TIM_PWM_Init(&htimx_SMotor[SM4]);

  /* 时钟源配置 */
  sClockSourceConfig.ClockSource    = TIM_CLOCKSOURCE_INTERNAL; // 内部时钟
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;  // 无时钟预分频
  sClockSourceConfig.ClockPolarity  = TIM_CLOCKPOLARITY_INVERTED;// 无效配置
  sClockSourceConfig.ClockFilter    = 0x0;  
  HAL_TIM_ConfigClockSource(&htimx_SMotor[SM4], &sClockSourceConfig);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_SMotor[SM4],&sMasterConfig);
  
  /* PWM配置,CH1 PWM Mode*/
  sConfigOC.OCMode       = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse        = SMOTOR4_TIM_DEFAULT_PULSE;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htimx_SMotor[SM4], &sConfigOC, SMOTOR4_PUL_TIMx_CHANNELx);
  
  DMAx_Config(SM4);
    
  /* 配置电机默认状态,默认禁止电机转动,并且方向是顺时针转动 */
  SMotor_Set_Dir  ( SM4, MOTOR_DIR_CW);
  SMotor_Set_State( SM4, MOTOR_DISABLE);
}

/**
  * 函数功能: 设置电机方向
  * 输入参数: Dir 电机方向 参数可选: 
  *               @arg  MOTOR_DIR_CW
  *               @arg  MOTOR_DIR_CCW
  * 返 回 值:
  * 说    明:设置步进电机的方向.控制驱动器的DIR端口
  */
void SMotor_Set_Dir  (uint16_t SMx, MotorDir_Typedef Dir)
{
  /* 设置对应编号的步进电机方向 */
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
  * 函数功能: 设置电机状态
  * 输入参数: Ena 电机状态 参数可选: 
  *               @arg  MOTOR_ENABLE
  *               @arg  MOTOR_DISABLE
  * 返 回 值:
  * 说    明: 设置步进电机的状态,控制驱动器的ENA端口
  */
void SMotor_Set_State(uint16_t SMx, MotorSta_Typedef Ena)
{
    /* 设置对应编号的步进电机使能 */
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

/****************************** 梯形加减速控制 ********************************/
/**
  * 函数功能: 启动电机转动
  * 输入参数: SMx 步进电机编号，step 走的步数
  * 返 回 值: 无
  * 说    明: 填充速度缓冲区,使能DMA,使能定时器通道输出
  */
static void Start_MotorMoving(uint16_t SMx, int32_t step)
{
	uint32_t TIMx_CHANNEL = 0;     // dummy
	uint32_t RegAddr = 0;          // dummy
	uint16_t DMA_TIM_CCn = 0;      // dummy
	uint16_t *ptrBuf0 = NULL;
	uint16_t *ptrBuf1 = NULL;
  
	// 首先配置SM1和SM2变量
	switch(SMx)
	{
	  case SM1:
      TIMx_CHANNEL = SMOTOR1_PUL_TIMx_CHANNELx;   // 设置定时器通道
      DMA_TIM_CCn = DMAx_TIMz1_CCn;                // 设置定时器DMA配置
      RegAddr = (uint32_t)SMOTOR1_TIMx_CCxADDRESS;// 设置寄存器地址
      
      DMAx_STREAMz1_CLEAR_FLAG();            // 清空标志位
      DMAx_STREAMx_SET_CTM0(DMAx_Streamz1);  // 设置Buffer0为首个数据源

      break;
      case SM2:
      TIMx_CHANNEL = SMOTOR2_PUL_TIMx_CHANNELx;
      DMA_TIM_CCn = DMAx_TIMp1_CCn; 
      RegAddr = (uint32_t)SMOTOR2_TIMx_CCxADDRESS;    
      DMAx_STREAMp1_CLEAR_FLAG(); // 清空标志位
      DMAx_STREAMx_SET_CTM0(DMAx_Streamp1); // 设置Buffer0为首个数据源

      break;
	  case SM3:
      TIMx_CHANNEL = SMOTOR3_PUL_TIMx_CHANNELx;   // 设置定时器通道
      DMA_TIM_CCn = DMAx_TIMz2_CCn;                // 设置定时器DMA配置
      RegAddr = (uint32_t)SMOTOR3_TIMx_CCxADDRESS;// 设置寄存器地址
      
      DMAx_STREAMz2_CLEAR_FLAG();            // 清空标志位
      DMAx_STREAMx_SET_CTM0(DMAx_Streamz2);  // 设置Buffer0为首个数据源

      break;
      case SM4:
      TIMx_CHANNEL = SMOTOR4_PUL_TIMx_CHANNELx;
      DMA_TIM_CCn = DMAx_TIMp2_CCn; 
      RegAddr = (uint32_t)SMOTOR4_TIMx_CCxADDRESS;    
      DMAx_STREAMp2_CLEAR_FLAG(); // 清空标志位
      DMAx_STREAMx_SET_CTM0(DMAx_Streamp2); // 设置Buffer0为首个数据源

      break;
	}
	ptrBuf0 = (uint16_t *)&Spd_Buffer0[SMx];// 使用指针来代替数组
	ptrBuf1 = (uint16_t *)&Spd_Buffer1[SMx];// 
  
	/* 配置第一步的频率 */
	__HAL_TIM_SET_COMPARE(&htimx_SMotor[SMx], TIMx_CHANNEL, 1);
	/* 使定时器预分频配置生效 */
	HAL_TIM_GenerateEvent(&htimx_SMotor[SMx], TIM_EVENTSOURCE_UPDATE);

	/* 进入中断的次数 */
	BufferMod[SMx] = div(step, (BUFFER_SIZE>>1) );// 计算余数
   
	/* 填充缓冲区 */
	*ptrBuf0 = 1 ;    // buffer0
	LSC_CalcSpd(SMx, ptrBuf0, BUFFER_SIZE);
//	SCB_CleanDCache_by_Addr((uint32_t*)ptrBuf0, DATAWIDTH);
  
	*ptrBuf1 = ptrBuf0[BUFFER_SIZE-1]; // buffer1
	LSC_CalcSpd(SMx, ptrBuf1, BUFFER_SIZE);  
//	SCB_CleanDCache_by_Addr((uint32_t*)ptrBuf1, DATAWIDTH);
	
	/* 启动DMA双缓冲 */
	HAL_DMAEx_MultiBufferStart_IT(&hdma_tim[SMx], 
                            (uint32_t)ptrBuf0,// 第一个数据源
                            (uint32_t)RegAddr,// 比较器地址
                            (uint32_t)ptrBuf1,// 第二个数据源
                            BUFFER_SIZE);     // 传输数据长度   
  
	__HAL_TIM_ENABLE_DMA(&htimx_SMotor[SMx], DMA_TIM_CCn);// 使能DMA及中断

	/* 使能电机转动 */
	SMotor_Set_State(SMx, MOTOR_ENABLE);
// 刚启动的时候会有响声，使用延时可以确定，响声是因为使能电机引起的，
// 而不是因为初始速度突变引起
//  HAL_Delay(1000); 
	SMotor[SMx].MotorRuning = true;

	/* 启动比较翻转电平 */
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
  * 函数功能: 停止电机转动
  * 输入参数: SMx 步进电机编号
  * 返 回 值: 无
  * 说    明: 禁止DMA传输中断,禁止输出PWM,失能电机
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
  memset( &SMotor[SMx].srd,0,sizeof(SMotor[SMx].srd));// 清0 变量值
  SMotor[SMx].srd.run_state = STOP;
  SMotor[SMx].MotorRuning = false;
}
/**
  * 函数功能: 根据限位开关的状态判断是否允许转动
  * 输入参数: SMx :步进电机编号
  * 返 回 值: bool，当前不能转动则返回true，否则返回false
  * 说    明: 无
  */
bool NotAllow2Move(uint16_t SMx)
{
  switch(SMx)
  {
    /* 判断SM1的限位开关状态 */
    case SM1:
      if(HAL_GPIO_ReadPin(LIMIT1_POS_GPIO_PORT,LIMIT1_POS_PIN) == LIMIT1_POS_ACTIVE_LEVEL)
        if(SMotor[SMx].Dir == MOTOR_DIR_CW)
          return true;
      if(HAL_GPIO_ReadPin(LIMIT1_NEG_GPIO_PORT,LIMIT1_NEG_PIN) == LIMIT1_NEG_ACTIVE_LEVEL)
        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
          return true;
      if(HAL_GPIO_ReadPin(LIMIT2_NEG_GPIO_PORT,LIMIT2_NEG_PIN) == LIMIT2_NEG_ACTIVE_LEVEL)
        if(SMotor[SMx].Dir == MOTOR_DIR_CW)
          if(LoadFlag[SMx])//如果启动装针管控制，则有效
          {
            LoadFlag[SMx] = false;
            LoadInitFlag[SMx] = true;
            UnloadFlag[SMP1] = false;//已完成装针管时，卸针管复位
            UnloadInitFlag[SMP1] = false;
            return true;
          }
      break;
    
    /* 判断SM2的限位开关状态 */
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
      if(HAL_GPIO_ReadPin(LIMIT2_NEG_GPIO_PORT,LIMIT2_NEG_PIN) != LIMIT2_NEG_ACTIVE_LEVEL)//这里是卸针管已成功
      {
        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
          if(UnloadFlag[SMx])//如果已启动卸针管控制，则有效
          {
            UnloadFlag[SMx] = false;
            UnloadInitFlag[SMx] = true;
            LoadFlag[SMZ1] = false;//已完成卸针管时，装针管复位
            LoadInitFlag[SMZ1] = false;
//            return true;
          }
      }
      else if(HAL_GPIO_ReadPin(LIMIT2_ORI_GPIO_PORT,LIMIT2_ORI_PIN) == LIMIT2_ORI_ACTIVE_LEVEL)//零点光耦之下不允许取针管，此时负极限信号有效，可限制移动
      {
        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
          return true;
      }
      break;
    /* 判断SM3的限位开关状态 */
    case SM3:
      if(HAL_GPIO_ReadPin(LIMIT3_POS_GPIO_PORT,LIMIT3_POS_PIN) == LIMIT3_POS_ACTIVE_LEVEL)
        if(SMotor[SMx].Dir == MOTOR_DIR_CW)
          return true;
      if(HAL_GPIO_ReadPin(LIMIT3_NEG_GPIO_PORT,LIMIT3_NEG_PIN) == LIMIT3_NEG_ACTIVE_LEVEL)
        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
          return true;
      if(HAL_GPIO_ReadPin(LIMIT4_NEG_GPIO_PORT,LIMIT4_NEG_PIN) == LIMIT4_NEG_ACTIVE_LEVEL)
        if(SMotor[SMx].Dir == MOTOR_DIR_CW)
          if(LoadFlag[SMx])//如果启动装针管控制，则有效
          {
            LoadFlag[SMx] = false;
            LoadInitFlag[SMx] = true;
            UnloadFlag[SMP2] = false;//已完成装针管时，卸针管复位
            UnloadInitFlag[SMP2] = false;
            return true;
          }
      break;
    
    /* 判断SM4的限位开关状态 */
    case SM4: //SMP2
//      if(HAL_GPIO_ReadPin(LIMIT4_POS_GPIO_PORT,LIMIT4_POS_PIN) == LIMIT4_POS_ACTIVE_LEVEL)
//        if(SMotor[SMx].Dir == MOTOR_DIR_CW)
//          return true;
//      if(HAL_GPIO_ReadPin(LIMIT4_NEG_GPIO_PORT,LIMIT4_NEG_PIN) == LIMIT4_NEG_ACTIVE_LEVEL)
//        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
//          return true;
//      if(HAL_GPIO_ReadPin(LIMIT4_ORI_GPIO_PORT,LIMIT4_ORI_PIN) == LIMIT4_ORI_ACTIVE_LEVEL)
//        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
//          if(!UnloadFlag[SMx])//如果未启动卸针管控制，则有效
//            return true;
      if(HAL_GPIO_ReadPin(LIMIT4_NEG_GPIO_PORT,LIMIT4_NEG_PIN) != LIMIT4_NEG_ACTIVE_LEVEL)//这里是卸针管已成功
      {
        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
          if(UnloadFlag[SMx])//如果已启动卸针管控制，则有效
          {
            UnloadFlag[SMx] = false;
            UnloadInitFlag[SMx] = true;
            LoadFlag[SMZ2] = false;//已完成卸针管时，装针管复位
            LoadInitFlag[SMZ2] = false;
//            return true;
          }
      }
      else if(HAL_GPIO_ReadPin(LIMIT4_ORI_GPIO_PORT,LIMIT4_ORI_PIN) == LIMIT4_ORI_ACTIVE_LEVEL)//零点光耦之下不允许取针管，此时负极限信号有效，可限制移动
      {
        if(SMotor[SMx].Dir == MOTOR_DIR_CCW)
          return true;
      }
      break;
  }
  return false;
}
  
/**
  * 函数功能: 步进电机线性速度控制
  * 输入参数: 包含了梯形加减速速度控制所需要的参数:
  *                     @arg  Step  总的步数 单位是脉冲数
  *                     @arg  Accel 加速时间 单位是ms 
  *                     @arg  Decel 减速时间 单位是ms
  *                     @arg  Speed 最高速度 单位是r/m
  * 返 回 值: 无
  * 说    明: 计算初始速度值,和各种限制条件,并且填充缓冲区,启动脉冲输出
  */
void STEPMOTOR_LSCMoveRel (int16_t SMx, float step, float accel, float decel, float speed)
{
	SpeedRampData_Typedef *srd = &SMotor[SMx].srd;
	/* 定时器时钟源频率 */
	float Timx_ClkSrc_Frq = TClkSrc_Frq[SMx]; 
	/* 定时器计数频率 */
	float Timx_Frq = 0; 
	/* 达到设定的速度时需要的步数 */
	float max_s_lim;
	/* 必须要开始减速的步数（如果加速没有达到最大速度）*/
	float accel_lim;
	/* 预分频系数 */
	int32_t Prescaler = 0;// 默认定时器预分频
  
	if(SMotor[SMx].MotorRuning)// 当前正处于转动状态
		return ;

	/* 电机转动方向 */
	if(step >= 0)
	{ 
		SMotor_Set_Dir(SMx, DIR_FORWARE);
	}
	else
	{
		step = fabs(step);
    
		SMotor_Set_Dir(SMx,DIR_REVERSAL);
	}
	/* 根据限位开关判断是否允许可以运行 */
	if( NotAllow2Move(SMx) )
	{
		return;
	}
  
	srd ->Speed = speed ;// 记录速度
	srd -> Step = fabs(step);
  
	speed = speed / 60.0f ;
	accel = speed / (accel / 1000.0f);// 单位转换,ms -> r/s^2
	decel = speed / (decel / 1000.0f);// 单位转换,ms -> r/s^2
  
	srd -> Acc_Time = accel;// 记录加速度值，主要用于二次变速
	srd -> Dec_Time = decel;// 记录减速度值，主要用于二次变速
  
	if(step == 1)// 步数为1
	{
		srd -> accel_count = -1;    // 只移动一步
		srd -> run_state   = DECEL; // 减速状态.
		srd -> step_delay  = 0xFFFF;// 短延时	
	}
	else
	{
		/* 计算起始速度和最大速度对应的定时器周期值 */
		do
		{ // 这一步是调整定时器预分频,使计算结果不会溢出
			Prescaler++;
			Timx_Frq = (Timx_ClkSrc_Frq / (double)Prescaler);
      
			// 通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为 Rev/sec^2
			// step_delay = 1/tt * sqrt(2*alpha/accel)
			// step_delay = ( tfreq*0.676 ) * sqrt( (2*alpha) / accel )
			srd -> step_delay = round((Timx_Frq * sqrt(ALPHA_x2 / accel))*0.676f);//C0,初始速度的定时器值
      
		}while((uint32_t)(srd -> step_delay>>1) > UINT16_MAX);
		__HAL_TIM_SET_PRESCALER(&htimx_SMotor[SMx], --Prescaler);
		// 设置最大速度极限.
		srd -> min_delay = round((ALPHA * Timx_Frq ) / speed);
		srd -> sec_delay = srd -> min_delay ;
    
		/* 计算加减速需要的参数 */
		// 计算多少步之后达到最大速度的限制
		// max_s_lim = speed^2 / (2*alpha*accel)
		max_s_lim = speed * speed / (ALPHA_x2 * accel) ;
      
		// 计算多少步之后我们必须开始减速
		// n1 = (n1+n2)decel / (accel + decel)
		accel_lim =  step * decel / (accel + decel) ;  
    
		/* 计算加速限制条件和开始减速的位置 */
		// 使用限制条件我们可以计算出减速阶段步数
		if(accel_lim <= max_s_lim)
		{
			srd -> decel_val = (int32_t)(accel_lim - step);
		}
		else{
			srd -> decel_val = - ( round( (max_s_lim*accel / decel) ));
		}
		// 当只剩下一步必须减速
		if(srd -> decel_val >= -1)
		{
			srd -> decel_val = -1;
		}
      
		// 计算开始减速时的步数
		srd -> decel_start = (int32_t)(step + srd -> decel_val);
		// 如果最大速度很慢，我们就不需要进行加速运动
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
  * 函数功能: DMA M0传输完成中断函数
  * 输入参数: hdma DMA控制句柄
  * 返 回 值: 无
  * 说    明: 进入本函数说明已经将Buffer0的数据传输完成,
  *           当前使用Buffer1作为传输源,可以修改Buffer0
  */
static void DMA_TC_M0CpltCallback(DMA_HandleTypeDef *hdma)
{  
  DMA_Stream_TypeDef   *Instance = hdma->Instance;
  uint16_t *ptrBuf0 = NULL;// dummy  数据缓存指针
  uint16_t *ptrBuf1 = NULL;// dummy  数据缓存指针
  uint16_t SMx = 0;        // dummy  电机编号
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
  ptrBuf0 = (uint16_t *)&Spd_Buffer0[SMx];// 使用指针代替数组
  ptrBuf1 = (uint16_t *)&Spd_Buffer1[SMx];
  
  /* 记录中断次数,控制停止脉冲输出 */
  BufferMod[SMx].quot--;
  if(BufferMod[SMx].quot <= 0)
  {
    if(BufferMod[SMx].rem <= 0) // 同时为0即表示已经输出所有脉冲
      Stop_MotorMoving(SMx);
    else // 余数不为0，所以需要修改DMA传输数据量，继续传输
    {
      /* 剩下的脉冲数 */
      /* 这里要先暂停DMA传输，重新配置传输数量，然后重新启动DMA，
       * 要求操作速度要快，所以直接使用寄存器操作
       */
      BufferMod[SMx].rem <<= 1;// 余下的数据重新配置DMA传输
      
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
      BufferMod[SMx].rem = 0;// 下一次进入中断就是停止的时候
    }
    return ;
  }
  
  /* 重新填充Buffer0的数据内容 */  
  ptrBuf0[0] = ptrBuf1[BUFFER_SIZE-1];
  LSC_CalcSpd(SMx, ptrBuf0, BUFFER_SIZE);  
//  SCB_CleanDCache_by_Addr((uint32_t*)ptrBuf0, DATAWIDTH);
}

/**
  * 函数功能: DMA M1传输完成中断函数
  * 输入参数: hdma DMA控制句柄
  * 返 回 值: 无
  * 说    明: 进入本函数说明已经将Buffer1的数据传输完成,
  *           当前使用Buffer0作为传输源,可以修改Buffer1
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
  
  /* 记录中断次数,控制停止脉冲输出 */
  BufferMod[SMx].quot--;
  if(BufferMod[SMx].quot<=0)
  {
    if(BufferMod[SMx].rem <= 0) // 同时为0即表示已经输出所有脉冲
      Stop_MotorMoving(SMx);
    else// 余数不为0，所以需要修改DMA传输数据量，继续传输
    {  
      /* 这里要先暂停DMA传输，重新配置传输数量，然后重新启动DMA，
       * 要求操作速度要快，所以直接使用寄存器操作
       */
      BufferMod[SMx].rem <<= 1;// 余下的数据重新配置DMA传输
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
      BufferMod[SMx].rem = 0;// 下一次进入中断就是停止的时候
    }
    return ;
  }
  /* 记录上一次计算最后的比较值,用于另一个缓冲的计算 */
  ptrBuf1[0] = ptrBuf0[BUFFER_SIZE-1];
  LSC_CalcSpd(SMx, ptrBuf1, BUFFER_SIZE);  
//  SCB_CleanDCache_by_Addr((uint32_t*)ptrBuf1, DATAWIDTH);
}
/**
  * 函数功能: DMA 传输中断函数
  * 输入参数: hdma DMA控制句柄
  * 返 回 值: 无
  * 说    明: 进入本函数说明已经将Buffer1的数据传输完成,
  *           当前使用Buffer0作为传输源,可以修改Buffer1
  */
static void  DMA_TC_ErrorCallback(DMA_HandleTypeDef *hdma)
{
	(void) hdma;
}


/**
  * 函数功能: 更新速度缓冲区的值
  * 输入参数: Array 缓冲区首地址
  * 返 回 值: 无
  * 说    明: 计算每一步的速度值,存入缓冲区里面
  */
static void  LSC_CalcSpd(uint16_t SMx, uint16_t *Array, uint32_t Buffersize)
{
  int32_t i = 0;// 循环控制
  int32_t new_step_delay = 0;              // 记录速度值结果
  static int32_t last_accel_delay[MOTORNUM] = {0};     // 记录加速阶段的最后一步速度值
  // 记录new_step_delay中的余数，提高下一步计算的精度
  static int32_t rest[MOTORNUM] = {0};
  static int32_t AccelCNT[MOTORNUM] = {0};

  uint16_t tmpCNT = 0;
  
  tmpCNT = Array[0];
  
  SpeedRampData_Typedef *pSMotor = &SMotor[SMx].srd;
  for(i = 0; i < Buffersize; i += 2 )  
  {
    // 保存新（下）一个延时周期
    switch(pSMotor->run_state) // 加减速曲线阶段
    {
      /* 加速状态 */
      case ACCEL:
        pSMotor->step_count  ++;    // 步数加1
        pSMotor->accel_count ++; 		// 加速计数值加1
      
        new_step_delay = pSMotor->step_delay - \
                        (((2 * pSMotor->step_delay) + rest[SMx]) / (4 * pSMotor->accel_count + 1)); //计算新(下)一步脉冲周期(时间间隔)
        rest[SMx] = ((2 * pSMotor->step_delay) + rest[SMx]) % (4 * pSMotor->accel_count + 1);// 计算余数，下次计算补上余数，减少误差
      
        /* 加速到开始减速 */
        if(pSMotor->step_count >= pSMotor->decel_start)// 开始减速到0
        {
          pSMotor->sec_delay = INT32_MAX;
          AccelCNT[SMx] = pSMotor->accel_count ;
          pSMotor->accel_count = pSMotor->decel_val; // 加速计数值为减速阶段计数值的初始值
          pSMotor->run_state = DECEL;           	   // 下个脉冲进入减速阶段
        }
        /* 加速到匀速 */
        else if(new_step_delay <= pSMotor->min_delay)// 到达期望的最大速度，匀速
        {
          AccelCNT[SMx] = pSMotor->accel_count ;
          pSMotor->accel_count = pSMotor->decel_val; // 加速计数值为减速阶段计数值的初始值
          last_accel_delay[SMx] = new_step_delay;
          new_step_delay = pSMotor->min_delay;   // 使用min_delay（对应最大速度speed）
          rest[SMx] = 0;                         // 清零余值
          pSMotor->run_state = RUN;              // 设置为匀速运行状态
        }	
        break;
        
      /* 匀速状态 */
      case RUN:
        pSMotor->step_count ++;      		         // 步数加1     
        new_step_delay = pSMotor->min_delay;     // 使用min_delay（对应最大速度speed）
            
        if(pSMotor->step_count >= pSMotor->decel_start)// 开始减速到0
        {
          pSMotor->sec_delay = INT32_MAX;
          pSMotor->accel_count = pSMotor->decel_val; // 减速步数做为加速计数值
          new_step_delay = last_accel_delay[SMx]; // 加阶段最后的延时做为减速阶段的起始延时(脉冲周期)
          pSMotor->run_state = DECEL;             
        }
        /* 中途加速 */
        if( pSMotor->sec_delay < pSMotor->min_delay )//加速到第二速度
        {
          pSMotor->accel_count = AccelCNT[SMx] ;  // 重新设置加速的计数值
          new_step_delay = last_accel_delay[SMx]; // 加阶段最后的延时做为减速阶段的起始延时(脉冲周期)
          pSMotor->min_delay = pSMotor->sec_delay;
          pSMotor->run_state = ACCEL;             // 设置为加速运行状态
        }
        /* 中途减速 */
        if( pSMotor->sec_delay > pSMotor->min_delay)// 减速到第二速度
        {
          AccelCNT[SMx] = pSMotor->accel_count ;      // 这个时候的accel_count是加速到sec_delay的步数
          pSMotor->accel_count = pSMotor->decel_val; // 减速步数做为加速计数值
          new_step_delay = last_accel_delay[SMx]; // 加阶段最后的延时做为减速阶段的起始延时(脉冲周期)
          
          pSMotor->run_state = DECEL;            // 状态改变为减速
          
        }
        break;
        
      /* 减速状态 */
      case DECEL:
        pSMotor->step_count ++;     // 步数加1
        pSMotor->accel_count++; 	  // 加速计数值加1
      
        new_step_delay = pSMotor->step_delay - \
                        (((2 * pSMotor->step_delay) + rest[SMx]) / (4 * pSMotor->accel_count + 1)); //计算新(下)一步脉冲周期(时间间隔)
        rest[SMx] = ((2 * pSMotor->step_delay) + rest[SMx]) % (4 * pSMotor->accel_count + 1);// 计算余数，下次计算补上余数，减少误差
        //检查是否为最后一步,最后一步,跳出循环,不需要再计算
        if(pSMotor->accel_count > 0)
        {
          /* 停止状态 */
          pSMotor->run_state = STOP;
          pSMotor->step_count = 0;   // 清零步数计数器
          rest[SMx] = 0;              // 清零余值
          last_accel_delay[SMx] = 0;
          i += Buffersize;      // 跳出循环
          return;
        }
        /* 减速到第二速度 */
        if(new_step_delay >= pSMotor->sec_delay) 
        {
          last_accel_delay[SMx] = new_step_delay;
          pSMotor->min_delay = pSMotor->sec_delay;
          new_step_delay = pSMotor->min_delay;   // 使用min_delay（对应最大速度speed）
          pSMotor->decel_val = pSMotor->accel_count ;// 减速剩余步数
          pSMotor->decel_start = pSMotor->Step + pSMotor->decel_val;// 修改减速到0的起始位置 
          rest[SMx] = 0;                          // 清零余值
          pSMotor->run_state = RUN;              // 设置为匀速运行状态
        }
        break;
      default :break;
    }
    /* 更新缓冲区,两次比较值是一个脉冲周期 */
    Array[i]   = (uint16_t)(tmpCNT + (pSMotor->step_delay >> 1));
    Array[i+1] = (uint16_t)(Array[i] + (pSMotor->step_delay >> 1));
    pSMotor->step_delay = new_step_delay; // 为下个(新的)延时(脉冲周期)赋值
    tmpCNT = Array[i+1];
  }
}

/*-------------------------------- 二次加速控制 ------------------------------*/

/**
  * 函数功能: 步进电机线性速度计算
  * 输入参数: pSMotor 包含了梯形加减速速度控制所需要的参数包括有:
  *                     @arg  Step  总的步数// 单位是 Pulse
  *                     @arg  Accel 加速时间// 单位是 r/s^2
  *                     @arg  Decel 减速时间// 单位是 r/s^2
  *                     @arg  Speed 最高速度// 单位是 r/m
  * 返 回 值: 无
  * 说    明: 计算初始速度值,和各种限制条件,并且填充缓冲区,启动脉冲输出
  */
void STEPMOTOR_LSC_Calc(uint16_t SMx, SpeedRampData_Typedef *pSMotor)
{
  float step  = pSMotor -> Step;
  float speed = pSMotor -> Speed / 60.0f ; // 转换为 r/s
  float accel = pSMotor -> Acc_Time ;
  float decel = pSMotor -> Dec_Time ;
  
	/* 定时器时钟源频率 */
	float Timx_ClkSrc_Frq = TClkSrc_Frq[SMx]; 
  /* 定时器计数频率 */
  float Timx_Frq = 0; 
  /* 达到设定的速度时需要的步数 */
	float max_s_lim;
  /* 必须要开始减速的步数（如果加速没有达到最大速度）*/
	float accel_lim;
  /* 预分频系数 */
  int32_t Prescaler = 0;// 定时器预分频, 从0开始计算合适的预分频
  
  /* 计算起始速度和最大速度对应的定时器周期值 */
  do
  { 
    Prescaler++;
    Timx_Frq = (Timx_ClkSrc_Frq / (double)Prescaler);
    
    pSMotor -> step_delay = round((Timx_Frq * sqrt(ALPHA_x2 / accel))*0.676f);//C0,初始速度的定时器值
    
  }while((uint32_t)(pSMotor -> step_delay>>1) > UINT16_MAX);
  __HAL_TIM_SET_PRESCALER(&htimx_SMotor[SM1], --Prescaler);
  // 设置最大速度极限.
  pSMotor -> min_delay = round((ALPHA * Timx_Frq ) / speed);
  
  /* 计算加减速需要的参数 */
  // 计算多少步之后达到最大速度的限制
  max_s_lim = speed * speed / (ALPHA_x2 * accel) ;
    
  // 计算多少步之后必须开始减速
  accel_lim =  step * decel / (accel + decel) ;  
  
  /* 计算加速限制条件和开始减速的位置 */
  // 使用限制条件可以计算出减速阶段步数
  if(accel_lim <= max_s_lim)
  {
    pSMotor -> decel_val = (int32_t)(accel_lim - step);
    pSMotor -> accel_count = accel_lim; // 记录加速步数
  }
  else
  {
    pSMotor -> decel_val = - ( round( (max_s_lim*accel / decel) ));
    pSMotor -> accel_count = max_s_lim;// 记录加速步数
  }
  // 当只剩下一步必须减速
  if(pSMotor -> decel_val >= -1)
  {
    pSMotor -> decel_val = -1;
  }  
}
/**
  * 函数功能: 修改运动参数
  * 输入参数: SMx 电机编号  Secspd 加速后的速度值 Step 加速后走的步数
  * 返 回 值: 无
  * 说    明: 无
  */
void Modify_MovingParam(uint16_t SMx, float Secspd ,int32_t Step)
{
  uint32_t bufCNT = __HAL_DMA_GET_COUNTER(&hdma_tim[SMx]);// 获取DMA已经传输的数据量
  SpeedRampData_Typedef  srd; // 用于计算加速后的一些参数
  Step = abs(Step);// 有符号数取绝对值
  
  srd.Speed    = Secspd; //  Rev/min
  srd.Step     = Step;   //  转动的步数
  srd.Acc_Time = SMotor[SMx].srd.Acc_Time;// 单位是r/s^2
  srd.Dec_Time = SMotor[SMx].srd.Dec_Time;// 单位是r/s^2
  STEPMOTOR_LSC_Calc(SMx, (SpeedRampData_Typedef*)&srd); 
  
  /* 根据DMA剩余待传输数据量判断当前输出的脉冲是否为一个完整的脉冲 */
  /* 需要使用当前缓冲区的脉冲数来计算已经传输的数据量 */
  if(bufCNT & 0x01)  
  {
    --bufCNT; //  再传输一次数据之后就是完整的脉冲
  }
  else 
  {
    bufCNT -= 2; // 再传输两次数据之后就是完整的脉冲
  }
  SMotor[SMx].srd.Speed = Secspd;
  SMotor[SMx].srd.sec_delay   = srd.min_delay; // 第二次匀速的速度值
  SMotor[SMx].srd.step_count -= ((BUFFER_SIZE+bufCNT)>>1);// 当前已输出总脉冲数
  SMotor[SMx].srd.Step = SMotor[SMx].srd.step_count + 1 + srd.Step;// 计算总的脉冲数
  // 加速控制
  if(srd.min_delay < SMotor[SMx].srd.min_delay) 
  {
    SMotor[SMx].srd.decel_val   = srd.decel_val; // 加速需要修改减速所需步数   
    SMotor[SMx].srd.decel_start = SMotor[SMx].srd.Step + srd.decel_val;// 开始减速的位置
  }
  else 
  {
    //减速不需要修改减速所需步数
    SMotor[SMx].srd.accel_count = srd.accel_count;// 这是计算减速的步数
  }
  
  /* 重新计算DMA中断次数 */
  BufferMod[SMx] = div(Step, (BUFFER_SIZE>>1) );// 计算余数

}

/**
  * 函数功能: 电机二次加速控制
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 重新计算速度值，暂停DMA，切换缓存区之后重新启动DMA
  */
void Modify_MotorMoving(uint16_t SMx, float Secspd ,int32_t Step)
{ 
  bool      tmpCT        = 0;    // 当前数据源
  uint32_t  tmpNDTR      = 0;    // 一片缓存区中剩余待传输的数据量
  uint16_t  *ptrNewbuf   = NULL; // 新的缓存区指针
  uint16_t  *ptrNextbuf  = NULL; // 另一片缓存区指针
  uint32_t  tmp          = 0;    
  
  DMA_Stream_TypeDef *DMAInstance = hdma_tim[SMx].Instance; // DMA实例
  uint32_t  tmpDMAxCR    = DMAInstance->CR; // DMA->CR 配置寄存器

  MotorRunSta_Typedef tmpRunState = SMotor[SMx].srd.run_state;// 电机状态
  
  
  /* 只有在加速和匀速状态下才允许变速 */
  if((tmpRunState == STOP)||(tmpRunState == DECEL)  ) 
  {
    return;
  }
  
  /* 计算新的速度值和修改DMA传输控制 */
  Modify_MovingParam(SMx, Secspd, Step);// 计算新的速度参数
  
  /* 获取当前使用的是哪个缓存区 */
  tmpCT = DMAx_STREAMx_GET_CT(DMAInstance);
  if(tmpCT)
  {
    ptrNewbuf  = (uint16_t *)&Spd_Buffer0[SMx];// 使用另外的一个缓存区暂存速度值
    ptrNextbuf = (uint16_t *)&Spd_Buffer1[SMx];
    tmpDMAxCR &= (~DMA_SxCR_CT); // 预设M0为新的数据源
  }
  else
  {
    ptrNewbuf  = (uint16_t *)&Spd_Buffer1[SMx]; 
    ptrNextbuf = (uint16_t *)&Spd_Buffer0[SMx];
    tmpDMAxCR |= (DMA_SxCR_CT); // 预设M1为新的数据源
  }
  
  /* 等待输出一个完整的脉冲 */
  tmpNDTR = __HAL_DMA_GET_COUNTER(&hdma_tim[SMx]);
  if(tmpNDTR & 0x01) // 奇数
  {
    while((__HAL_DMA_GET_COUNTER(&hdma_tim[SMx]) & 0x01))// 奇数 // 从奇数变为偶数即为输出一个完整的脉冲   
    {}
  }
  else
  {
    tmp = tmpNDTR - 2;
    while(__HAL_DMA_GET_COUNTER(&hdma_tim[SMx]) != tmp)// 偶数//输出一个完整的脉冲
    {}
  }
  
    /* 修改DMA的操作需要迅速完成，所以下面将直接操作寄存器 
     * 先行计算2个数据，在切换缓存之后再计算剩余的速度值，
     * 经测试，下面的程序耗时大约600~700ns，即两次DMA传输的时间间隔不能低于700ns，
     * 限制了加速前脉宽不能低于700ns。
     */
  if(SMx == SM1)
  {
    *ptrNewbuf = *SMOTOR1_TIMx_CCxADDRESS;
    LSC_CalcSpd(SMx, ptrNewbuf, 2);   // 先计算2个数据量，
//    SCB_CleanDCache_by_Addr((uint32_t*)ptrNewbuf, 4);

    /* 先停止DMA传输，将源地址更改为另一片缓存区，然后重新启动DMA */
    DMAx_STREAMx_DISABLE_TC(DMAInstance);
    DMAx_STREAMz1_CLEAR_FLAG(); // 清空标志位
    
  }else if( SMx == SM2 )
  {
    *ptrNewbuf = *SMOTOR2_TIMx_CCxADDRESS;
    LSC_CalcSpd(SMx, ptrNewbuf, 2);
//    SCB_CleanDCache_by_Addr((uint32_t*)ptrNewbuf, 4);

    /* 先停止DMA传输，将源地址更改为另一片缓存区，然后重新启动DMA */
    DMAx_STREAMx_DISABLE_TC(DMAInstance);
    DMAx_STREAMp1_CLEAR_FLAG(); // 清空标志位
  }
  else if(SMx == SM3)
  {
    *ptrNewbuf = *SMOTOR3_TIMx_CCxADDRESS;
    LSC_CalcSpd(SMx, ptrNewbuf, 2);   // 先计算2个数据量，
//    SCB_CleanDCache_by_Addr((uint32_t*)ptrNewbuf, 4);

    /* 先停止DMA传输，将源地址更改为另一片缓存区，然后重新启动DMA */
    DMAx_STREAMx_DISABLE_TC(DMAInstance);
    DMAx_STREAMz2_CLEAR_FLAG(); // 清空标志位
    
  }else if( SMx == SM4 )
  {
    *ptrNewbuf = *SMOTOR4_TIMx_CCxADDRESS;
    LSC_CalcSpd(SMx, ptrNewbuf, 2);
//    SCB_CleanDCache_by_Addr((uint32_t*)ptrNewbuf, 4);

    /* 先停止DMA传输，将源地址更改为另一片缓存区，然后重新启动DMA */
    DMAx_STREAMx_DISABLE_TC(DMAInstance);
    DMAx_STREAMp2_CLEAR_FLAG(); // 清空标志位
  }
  
  /* 更改缓存区并重新使能DMA、中断 */
  DMAInstance->CR = tmpDMAxCR;

  /* ---------------------  */
  
  /* 计算剩余待计算的速度值*/
  ptrNewbuf+=2;
  *(ptrNewbuf) = *(ptrNewbuf - 1);
  LSC_CalcSpd(SMx, ptrNewbuf, BUFFER_SIZE - 2);  
//  SCB_CleanDCache_by_Addr((uint32_t*)ptrNewbuf, (BUFFER_SIZE-2)*(sizeof(ptrNewbuf[0])));
//	MemClr((uint8_t*)ptrNewbuf, (BUFFER_SIZE-2)*(sizeof(ptrNewbuf[0])));
  
  *ptrNextbuf = *(ptrNewbuf + BUFFER_SIZE - 3);// 另一片缓存区的速度值
  LSC_CalcSpd(SMx, ptrNextbuf, BUFFER_SIZE);  
//  SCB_CleanDCache_by_Addr((uint32_t*)ptrNextbuf, DATAWIDTH);
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/

