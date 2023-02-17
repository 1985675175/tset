/**
  ******************************************************************************
  * @file    bsp_stepmotor.h
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
#ifndef __BSP_STEPMOTOR_H__
#define __BSP_STEPMOTOR_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef enum 
{
  MOTOR_DIR_CW = 0,
  MOTOR_DIR_CCW
}MotorDir_Typedef;

/* 电机使能定义 */
typedef enum 
{
  MOTOR_ENABLE  = 0,
  MOTOR_DISABLE
}MotorSta_Typedef ;

/* 电机加减速曲线状态定义 */
typedef enum{
  STOP  = 0,   // 加减速曲线状态：停止
  ACCEL,       // 加减速曲线状态：加速阶段
  DECEL,       // 加减速曲线状态：减速阶段
  RUN,         // 加减速曲线状态：匀速阶段  
}MotorRunSta_Typedef;

/* 电机加减速参数类型定义 */
typedef struct {
// 加减速控制相关
__IO  MotorRunSta_Typedef             run_state ; // 电机转动状态
  int32_t                             decel_start;// 启动减速位置
  int32_t                             decel_val;  // 减速阶段步数
  int32_t                             min_delay;  // 最小脉冲周期(最大速度,即匀速段速度)
  int32_t                             sec_delay;  // 第二速度
__IO  int32_t                         step_delay; // 下个脉冲周期(时间间隔),启动时为加速度
  
__IO  int32_t                         accel_count;// 加(减)速阶段步数计数值
__IO  uint32_t                        step_count;
          
// 加减速参数设置相关                 
  float                               Speed;      // 速度   单位为 r/s
  float                               Acc_Time;   // 加速时间 单位为 ms
  float                               Dec_Time;   // 减速时间 单位为 ms
  int32_t                             Step;       // 步数   单位为 Pulse
} SpeedRampData_Typedef;        
        
/* 电机控制定义 */        
typedef struct{       
// 电机控制相关       
  __IO MotorDir_Typedef               Dir;        // 电机转动方向记录
  __IO MotorSta_Typedef               State;      // 电机转动使能记录
  __IO bool                           MotorRuning;// 电机正在运行标记

  SpeedRampData_Typedef               srd;
}MotorObj_Typedef;

typedef struct{
// 移动运行参数设置相关                 
  float                               Lim_Speed;     // 最高速度   单位为 r/s
  float                               Craml_Speed;   // 爬行速度   单位为 r/s
  float                               Acc_Time;      // 加速时间 单位为 ms
  float                               Dec_Time;      // 减速时间 单位为 ms
  float                               Distance_Rev;  // 每圈距离 单位为 mm/r
  float                               Step_Angle;    // 步距角 单位为 degrees
  float                               Micro_Step;    // 细分数
  float                               Coord;         // 坐标 单位为 mm
} MoveRunData_Typedef;        

typedef struct{
// 移液器运行参数设置相关                 
  float                               Volume_Distance;   // 体积距离比  单位为 ul/mm
  float                               Max_Stroke;             // 最大行程    单位为 mm
  float                               Unload_Distance;        // 卸针管距离  单位为 mm
} PipetteRunData_Typedef;        

/* 宏定义 --------------------------------------------------------------------*/
#define SMX                            SM1
#define SMY                            SM2

#define SMZ1                            SM1
#define SMP1                            SM2
#define SMZ2                            SM3
#define SMP2                            SM4

#define SM1                            0
#define SM2                            1
#define SM3                            2
#define SM4                            3
#define MOTORNUM                       4

#define PIP1                           0
#define PIP2                           1
#define PIPNUM                         2

/*------------------------------- 移液器1 ----------------------------------*/

/*********************** 移液器控制相关参数定义 ************************/
#define PIPETTE1_VOLUME_DISTANCE_RATIO                 (78.7401574f) //体积距离比
#define PIPETTE1_MAX_STROKE                            (12.7f)  //最大行程
#define PIPETTE1_UNLOAD_DISTANCE                       (5.5f)   //卸针管距离

/*------------------------------- 移液器2 ----------------------------------*/

/*********************** 移液器控制相关参数定义 ************************/
#define PIPETTE2_VOLUME_DISTANCE_RATIO                 (78.7401574f) //体积距离比
#define PIPETTE2_MAX_STROKE                            (12.7f)  //最大行程
#define PIPETTE2_UNLOAD_DISTANCE                       (5.5f)   //卸针管距离

/*------------------------------- 步进电机1 ----------------------------------*/

/*********************** 步进电机控制定时器TIM参数定义 ************************/
#define SMOTOR1_TIMx                          TIM8
#define SMOTOR1_TIM_RCC_CLK_ENABLE()          __HAL_RCC_TIM8_CLK_ENABLE()
#define SMOTOR1_TIM_RCC_CLK_DISABLE()         __HAL_RCC_TIM8_CLK_DISABLE()
#define SMOTOR1_TIM_IRQn                      TIM8_CC_IRQn
#define SMOTOR1_TIM_IRQHandler                TIM8_CC_IRQHandler
#define SMOTOR1_TIMx_CCxADDRESS               (&TIM8->CCR1) // CCR1比较器的地址,用于DMA传输
#define SMOTOR1_TIM_CLKSRC_FREQ               (168e6)

// 定义定时器预分频，定时器实际时钟频率为：200MHz/（SMOTOR1_TIM_PRESCALER+1）
#define SMOTOR1_TIM_PRESCALER                 5//7  

// 定义定时器周期，当定时器开始计数到SMOTOR1_TIM_PERIOD值是更新定时器并生成对应事
// 件和中断.定时器更新频率为：定时器频率/(SMOTOR1_TIM_PERIOD+1) Hz
#define SMOTOR1_TIM_PERIOD                    0xFFFF  
#define SMOTOR1_TIM_DEFAULT_PULSE             (SMOTOR1_TIM_PERIOD>>1)


/*  定义控制输出引脚 */
#define SMOTOR1_PUL_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOI_CLK_ENABLE();
#define SMOTOR1_PUL_GPIO_PORT                 GPIOI
#define SMOTOR1_PUL_GPIO_PIN                  GPIO_PIN_5
#define SMOTOR1_PUL_PIN_AF                    GPIO_AF3_TIM8 // 复用功能引脚
#define SMOTOR1_PUL_TIMx_CHANNELx             TIM_CHANNEL_1

#define SMOTOR1_DIR_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOD_CLK_ENABLE();
#define SMOTOR1_DIR_GPIO_PORT                 GPIOD
#define SMOTOR1_DIR_GPIO_PIN                  GPIO_PIN_3
#define SMOTOR1_DIR_PIN_AF                    0 // default 

#define SMOTOR1_ENA_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOD_CLK_ENABLE();
#define SMOTOR1_ENA_GPIO_PORT                 GPIOD
#define SMOTOR1_ENA_GPIO_PIN                  GPIO_PIN_7
#define SMOTOR1_ENA_PIN_AF                    0 // default 

/* 电机方向和使能控制 */
#define SMOTOR1_DIR_FORWARE()                 HAL_GPIO_WritePin(\
                                                SMOTOR1_DIR_GPIO_PORT,\
                                                SMOTOR1_DIR_GPIO_PIN,\
                                                GPIO_PIN_SET)
#define SMOTOR1_DIR_REVERSAL()                HAL_GPIO_WritePin(\
                                                SMOTOR1_DIR_GPIO_PORT,\
                                                SMOTOR1_DIR_GPIO_PIN,\
                                                GPIO_PIN_RESET)


#define SMOTOR1_ENABLE()                      HAL_GPIO_WritePin(\
                                                SMOTOR1_ENA_GPIO_PORT,\
                                                SMOTOR1_ENA_GPIO_PIN,\
                                                GPIO_PIN_SET)
#define SMOTOR1_DISABLE()                     HAL_GPIO_WritePin(\
                                                SMOTOR1_ENA_GPIO_PORT,\
                                                SMOTOR1_ENA_GPIO_PIN,\
                                                GPIO_PIN_RESET)



/*------------------------------- 步进电机2 ----------------------------------*/
#define SMOTOR2_TIMx                          TIM8
#define SMOTOR2_TIM_RCC_CLK_ENABLE()          __HAL_RCC_TIM8_CLK_ENABLE()
#define SMOTOR2_TIM_RCC_CLK_DISABLE()         __HAL_RCC_TIM8_CLK_DISABLE()
#define SMOTOR2_TIM_IRQn                      TIM8_CC_IRQn
#define SMOTOR2_TIM_IRQHandler                TIM8_CC_IRQHandler
#define SMOTOR2_TIMx_CCxADDRESS               (&TIM8->CCR2) // CCR2比较器的地址,用于DMA传输
#define SMOTOR2_TIM_CLKSRC_FREQ               (168e6)

// 定义定时器预分频，定时器实际时钟频率为：200MHz/（SMOTOR2_TIM_PRESCALER+1）
#define SMOTOR2_TIM_PRESCALER                 5//7  

// 定义定时器周期，当定时器开始计数到SMOTOR2_TIM_PERIOD值是更新定时器并生成对应事
// 件和中断.定时器更新频率为：定时器频率/(SMOTOR2_TIM_PERIOD+1) Hz
#define SMOTOR2_TIM_PERIOD                    0xFFFF  
#define SMOTOR2_TIM_DEFAULT_PULSE             (SMOTOR2_TIM_PERIOD>>1)


/*  定义控制输出引脚 */
#define SMOTOR2_PUL_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOI_CLK_ENABLE();
#define SMOTOR2_PUL_GPIO_PORT                 GPIOI
#define SMOTOR2_PUL_GPIO_PIN                  GPIO_PIN_6
#define SMOTOR2_PUL_PIN_AF                    GPIO_AF3_TIM8 // 复用功能引脚
#define SMOTOR2_PUL_TIMx_CHANNELx             TIM_CHANNEL_2 

#define SMOTOR2_DIR_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOD_CLK_ENABLE();
#define SMOTOR2_DIR_GPIO_PORT                 GPIOD
#define SMOTOR2_DIR_GPIO_PIN                  GPIO_PIN_11
#define SMOTOR2_DIR_PIN_AF                    0 // default 

#define SMOTOR2_ENA_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOF_CLK_ENABLE();
#define SMOTOR2_ENA_GPIO_PORT                 GPIOF
#define SMOTOR2_ENA_GPIO_PIN                  GPIO_PIN_11
#define SMOTOR2_ENA_PIN_AF                    0 // default 

/* 电机方向和使能控制 */
#define SMOTOR2_DIR_FORWARE()                 HAL_GPIO_WritePin(\
                                                SMOTOR2_DIR_GPIO_PORT,\
                                                SMOTOR2_DIR_GPIO_PIN,\
                                                GPIO_PIN_SET)
#define SMOTOR2_DIR_REVERSAL()                HAL_GPIO_WritePin(\
                                                SMOTOR2_DIR_GPIO_PORT,\
                                                SMOTOR2_DIR_GPIO_PIN,\
                                                GPIO_PIN_RESET)


#define SMOTOR2_ENABLE()                      HAL_GPIO_WritePin(\
                                                SMOTOR2_ENA_GPIO_PORT,\
                                                SMOTOR2_ENA_GPIO_PIN,\
                                                GPIO_PIN_SET)
#define SMOTOR2_DISABLE()                     HAL_GPIO_WritePin(\
                                                SMOTOR2_ENA_GPIO_PORT,\
                                                SMOTOR2_ENA_GPIO_PIN,\
                                                GPIO_PIN_RESET)

/*------------------------------- 步进电机3 ----------------------------------*/
#define SMOTOR3_TIMx                          TIM8
#define SMOTOR3_TIM_RCC_CLK_ENABLE()          __HAL_RCC_TIM8_CLK_ENABLE()
#define SMOTOR3_TIM_RCC_CLK_DISABLE()         __HAL_RCC_TIM8_CLK_DISABLE()
#define SMOTOR3_TIM_IRQn                      TIM8_CC_IRQn
#define SMOTOR3_TIM_IRQHandler                TIM8_CC_IRQHandler
#define SMOTOR3_TIMx_CCxADDRESS               (&TIM8->CCR3) // CCR3比较器的地址,用于DMA传输
#define SMOTOR3_TIM_CLKSRC_FREQ               (168e6)

// 定义定时器预分频，定时器实际时钟频率为：200MHz/（SMOTOR3_TIM_PRESCALER+1）
#define SMOTOR3_TIM_PRESCALER                 5//7  

// 定义定时器周期，当定时器开始计数到SMOTOR3_TIM_PERIOD值是更新定时器并生成对应事
// 件和中断.定时器更新频率为：定时器频率/(SMOTOR3_TIM_PERIOD+1) Hz
#define SMOTOR3_TIM_PERIOD                    0xFFFF  
#define SMOTOR3_TIM_DEFAULT_PULSE             (SMOTOR3_TIM_PERIOD>>1)


/*  定义控制输出引脚 */
#define SMOTOR3_PUL_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOI_CLK_ENABLE();
#define SMOTOR3_PUL_GPIO_PORT                 GPIOI
#define SMOTOR3_PUL_GPIO_PIN                  GPIO_PIN_7
#define SMOTOR3_PUL_PIN_AF                    GPIO_AF3_TIM8 // 复用功能引脚
#define SMOTOR3_PUL_TIMx_CHANNELx             TIM_CHANNEL_3 

#define SMOTOR3_DIR_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOF_CLK_ENABLE();
#define SMOTOR3_DIR_GPIO_PORT                 GPIOF
#define SMOTOR3_DIR_GPIO_PIN                  GPIO_PIN_1
#define SMOTOR3_DIR_PIN_AF                    0 // default 

#define SMOTOR3_ENA_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOF_CLK_ENABLE();
#define SMOTOR3_ENA_GPIO_PORT                 GPIOF
#define SMOTOR3_ENA_GPIO_PIN                  GPIO_PIN_2
#define SMOTOR3_ENA_PIN_AF                    0 // default 

/* 电机方向和使能控制 */
#define SMOTOR3_DIR_FORWARE()                 HAL_GPIO_WritePin(\
                                                SMOTOR3_DIR_GPIO_PORT,\
                                                SMOTOR3_DIR_GPIO_PIN,\
                                                GPIO_PIN_SET)
#define SMOTOR3_DIR_REVERSAL()                HAL_GPIO_WritePin(\
                                                SMOTOR3_DIR_GPIO_PORT,\
                                                SMOTOR3_DIR_GPIO_PIN,\
                                                GPIO_PIN_RESET)


#define SMOTOR3_ENABLE()                      HAL_GPIO_WritePin(\
                                                SMOTOR3_ENA_GPIO_PORT,\
                                                SMOTOR3_ENA_GPIO_PIN,\
                                                GPIO_PIN_SET)
#define SMOTOR3_DISABLE()                     HAL_GPIO_WritePin(\
                                                SMOTOR3_ENA_GPIO_PORT,\
                                                SMOTOR3_ENA_GPIO_PIN,\
                                                GPIO_PIN_RESET)

/*------------------------------- 步进电机4 ----------------------------------*/
#define SMOTOR4_TIMx                          TIM8
#define SMOTOR4_TIM_RCC_CLK_ENABLE()          __HAL_RCC_TIM8_CLK_ENABLE()
#define SMOTOR4_TIM_RCC_CLK_DISABLE()         __HAL_RCC_TIM8_CLK_DISABLE()
#define SMOTOR4_TIM_IRQn                      TIM8_CC_IRQn
#define SMOTOR4_TIM_IRQHandler                TIM8_CC_IRQHandler
#define SMOTOR4_TIMx_CCxADDRESS               (&TIM8->CCR4) // CCR4比较器的地址,用于DMA传输
#define SMOTOR4_TIM_CLKSRC_FREQ               (168e6)

// 定义定时器预分频，定时器实际时钟频率为：200MHz/（SMOTOR4_TIM_PRESCALER+1）
#define SMOTOR4_TIM_PRESCALER                 5//7  

// 定义定时器周期，当定时器开始计数到SMOTOR4_TIM_PERIOD值是更新定时器并生成对应事
// 件和中断.定时器更新频率为：定时器频率/(SMOTOR4_TIM_PERIOD+1) Hz
#define SMOTOR4_TIM_PERIOD                    0xFFFF  
#define SMOTOR4_TIM_DEFAULT_PULSE             (SMOTOR4_TIM_PERIOD>>1)


/*  定义控制输出引脚 */
#define SMOTOR4_PUL_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOI_CLK_ENABLE();
#define SMOTOR4_PUL_GPIO_PORT                 GPIOI
#define SMOTOR4_PUL_GPIO_PIN                  GPIO_PIN_2
#define SMOTOR4_PUL_PIN_AF                    GPIO_AF3_TIM8 // 复用功能引脚
#define SMOTOR4_PUL_TIMx_CHANNELx             TIM_CHANNEL_4 

#define SMOTOR4_DIR_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE();
#define SMOTOR4_DIR_GPIO_PORT                 GPIOC
#define SMOTOR4_DIR_GPIO_PIN                  GPIO_PIN_8
#define SMOTOR4_DIR_PIN_AF                    0 // default 

#define SMOTOR4_ENA_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOH_CLK_ENABLE();
#define SMOTOR4_ENA_GPIO_PORT                 GPIOH
#define SMOTOR4_ENA_GPIO_PIN                  GPIO_PIN_7
#define SMOTOR4_ENA_PIN_AF                    0 // default 

/* 电机方向和使能控制 */
#define SMOTOR4_DIR_FORWARE()                 HAL_GPIO_WritePin(\
                                                SMOTOR4_DIR_GPIO_PORT,\
                                                SMOTOR4_DIR_GPIO_PIN,\
                                                GPIO_PIN_SET)
#define SMOTOR4_DIR_REVERSAL()                HAL_GPIO_WritePin(\
                                                SMOTOR4_DIR_GPIO_PORT,\
                                                SMOTOR4_DIR_GPIO_PIN,\
                                                GPIO_PIN_RESET)


#define SMOTOR4_ENABLE()                      HAL_GPIO_WritePin(\
                                                SMOTOR4_ENA_GPIO_PORT,\
                                                SMOTOR4_ENA_GPIO_PIN,\
                                                GPIO_PIN_SET)
#define SMOTOR4_DISABLE()                     HAL_GPIO_WritePin(\
                                                SMOTOR4_ENA_GPIO_PORT,\
                                                SMOTOR4_ENA_GPIO_PIN,\
                                                GPIO_PIN_RESET)

/************************* 步进电机与驱动器参数宏定义 *************************/

/*------------ 步进电机1 -----------------*/
#define SMOTOR1_LIM_SPEED           60
#define SMOTOR1_CRAML_SPEED         30
#define SMOTOR1_ACC_TIME            1000
#define SMOTOR1_DEC_TIME            1000
#define SMOTOR1_DISTANCE_REV        (2.0f)
#define SMOTOR1_STEP_ANGLE          (1.8f) // 步距角
#define SMOTOR1_MICRO_STEP          (32U) // 细分数

#define SMOTOR1_PULSE_REV                  (uint32_t)((360.0f/SMOTOR1_STEP_ANGLE)\
                                                      *SMOTOR1_MICRO_STEP)
#define SMOTOR1_RATED_SPEED                    (2000U) // 额定转速RPM

/*------------ 步进电机2 -----------------*/
#define SMOTOR2_LIM_SPEED           30       
#define SMOTOR2_CRAML_SPEED         15
#define SMOTOR2_ACC_TIME            1000
#define SMOTOR2_DEC_TIME            1000        
#define SMOTOR2_DISTANCE_REV        (1.27f) 
#define SMOTOR2_STEP_ANGLE          (1.8f) // 步距角
#define SMOTOR2_MICRO_STEP          (32U)  // 细分数

#define SMOTOR2_PULSE_REV                  (uint32_t)((360.0f/SMOTOR3_STEP_ANGLE)\
                                                      *SMOTOR3_MICRO_STEP)
#define SMOTOR2_RATED_SPEED                    (2000U) // 额定转速RPM

/*------------ 步进电机3 -----------------*/
#define SMOTOR3_LIM_SPEED           SMOTOR1_LIM_SPEED    
#define SMOTOR3_CRAML_SPEED         SMOTOR1_CRAML_SPEED
#define SMOTOR3_ACC_TIME            SMOTOR1_ACC_TIME
#define SMOTOR3_DEC_TIME            SMOTOR1_DEC_TIME
#define SMOTOR3_DISTANCE_REV        SMOTOR1_DISTANCE_REV
#define SMOTOR3_STEP_ANGLE          SMOTOR1_STEP_ANGLE // 步距角
#define SMOTOR3_MICRO_STEP          SMOTOR1_MICRO_STEP// 细分数

#define SMOTOR3_PULSE_REV                    SMOTOR1_PULSE_REV
#define SMOTOR3_RATED_SPEED                  SMOTOR1_RATED_SPEED // 额定转速*0.1RPM

/*------------ 步进电机4 -----------------*/
#define SMOTOR4_LIM_SPEED           SMOTOR2_LIM_SPEED            
#define SMOTOR4_CRAML_SPEED         SMOTOR2_CRAML_SPEED 
#define SMOTOR4_ACC_TIME            SMOTOR2_ACC_TIME
#define SMOTOR4_DEC_TIME            SMOTOR2_DEC_TIME        
#define SMOTOR4_DISTANCE_REV        SMOTOR2_DISTANCE_REV 
#define SMOTOR4_STEP_ANGLE          SMOTOR2_STEP_ANGLE // 步距角
#define SMOTOR4_MICRO_STEP          SMOTOR2_MICRO_STEP // 细分数

#define SMOTOR4_PULSE_REV                    SMOTOR2_PULSE_REV
#define SMOTOR4_RATED_SPEED                  SMOTOR2_RATED_SPEED // 额定转速*0.1RPM


/*----------------------------- DMA 相关配置 ---------------------------------*/
/*------------ 步进电机1 -----------------*/
#define DMAx_Streamz1                          DMA2_Stream2
#define DMAx_CHANNELz1_CLK_ENABLE()            __HAL_RCC_DMA2_CLK_ENABLE()
#define DMAx_Streamz1_IRQn                     DMA2_Stream2_IRQn
#define DMAx_Streamz1_IRQHandler               DMA2_Stream2_IRQHandler

#define DMAx_TIMz1_CCn                         TIM_DMA_CC1
#define DMAx_TIMz1_REQUEST_CHx                 DMA_CHANNEL_7//DMA_REQUEST_TIM4_CH3
#define DMAx_TIMz1_ID                          TIM_DMA_ID_CC1

/* 将DMA2_Stream2的所有中断状态位清0 */
#define DMAx_STREAMz1_CLEAR_FLAG()             (DMA2->LIFCR = (0x0000003D<<16))

/*------------ 步进电机2 -----------------*/
#define DMAx_Streamp1                          DMA2_Stream3
#define DMAx_CHANNELp1_CLK_ENABLE()            __HAL_RCC_DMA2_CLK_ENABLE()
#define DMAx_Streamp1_IRQn                     DMA2_Stream3_IRQn
#define DMAx_Streamp1_IRQHandler               DMA2_Stream3_IRQHandler

#define DMAx_TIMp1_CCn                         TIM_DMA_CC2
#define DMAx_TIMp1_REQUEST_CHx                 DMA_CHANNEL_7//DMA_CHANNEL_TIM1_CH3
#define DMAx_TIMp1_ID                          TIM_DMA_ID_CC2

/* 将DMA2_Stream3的所有中断状态位清0 */
#define DMAx_STREAMp1_CLEAR_FLAG()             (DMA2->LIFCR = (0x0000003D<<22))

/*------------ 步进电机3 -----------------*/
#define DMAx_Streamz2                          DMA2_Stream4
#define DMAx_CHANNELz2_CLK_ENABLE()            __HAL_RCC_DMA2_CLK_ENABLE()
#define DMAx_Streamz2_IRQn                     DMA2_Stream4_IRQn
#define DMAx_Streamz2_IRQHandler               DMA2_Stream4_IRQHandler

#define DMAx_TIMz2_CCn                         TIM_DMA_CC3
#define DMAx_TIMz2_REQUEST_CHx                 DMA_CHANNEL_7//DMA_REQUEST_TIM4_CH3
#define DMAx_TIMz2_ID                          TIM_DMA_ID_CC3

/* 将DMA2_Stream4的所有中断状态位清0 */
#define DMAx_STREAMz2_CLEAR_FLAG()             (DMA2->HIFCR = (0x0000003D))

/*------------ 步进电机4 -----------------*/
#define DMAx_Streamp2                          DMA2_Stream7
#define DMAx_CHANNELp2_CLK_ENABLE()            __HAL_RCC_DMA2_CLK_ENABLE()
#define DMAx_Streamp2_IRQn                     DMA2_Stream7_IRQn
#define DMAx_Streamp2_IRQHandler               DMA2_Stream7_IRQHandler

#define DMAx_TIMp2_CCn                         TIM_DMA_CC4
#define DMAx_TIMp2_REQUEST_CHx                 DMA_CHANNEL_7//DMA_CHANNEL_TIM1_CH3
#define DMAx_TIMp2_ID                          TIM_DMA_ID_CC4

/* 将DMA2_Stream7的所有中断状态位清0 */
#define DMAx_STREAMp2_CLEAR_FLAG()             (DMA2->HIFCR = (0x0000003D<<22))

/*----------------------------------------------------------------------------*/
/* 使能和禁止DMA数据流 */
#define DMAx_STREAMx_DISABLE(_HANDLE_)        (_HANDLE_->CR &= (~DMA_SxCR_EN))
#define DMAx_STREAMx_ENABLE(_HANDLE_)         (_HANDLE_->CR |= DMA_SxCR_EN)

/* 设置和获取传输数据量 */
#define DMAx_STREAMx_SET_COUNTER(_HANDLE_,SIZE); (_HANDLE_->NDTR = SIZE )
#define DMAx_STREAMx_GET_COUNTER(_HANDLE_)    (_HANDLE_->NDTR)
/* 获取和设置当前数据源 */
#define DMAx_STREAMx_GET_CT(_HANDLE_)         (_HANDLE_->CR & (DMA_SxCR_CT))
#define DMAx_STREAMx_SET_CTM0(_HANDLE_)       (_HANDLE_->CR &= (~DMA_SxCR_CT))
/* 禁止传输完成中断 */
#define DMAx_STREAMx_DISABLE_TC(_HANDLE_)     (_HANDLE_->CR &= ~(DMA_SxCR_TCIE|DMA_SxCR_EN))


///* 将DMA1_Stream1的所有中断状态位清0 */
//#define DMA_STREAMx_CLEAR_FLAG()              DMA1->LIFCR = (0x0000003D<<6);
///* 将DMA1_Stream2的所有中断状态位清0 */
//#define DMA_STREAMx_CLEAR_FLAG()              DMA1->LIFCR = (0x0000003D<<16);
///* 将DMA1_Stream3的所有中断状态位清0 */
//#define DMA_STREAMx_CLEAR_FLAG()              DMA1->LIFCR = (0x0000003D<<22);
///* 将DMA1_Stream0的所有中断状态位清0 */
//#define DMA_STREAMx_CLEAR_FLAG()              DMA1->HIFCR = (0x0000003D);


/** 由于使用输出比较模式,DMA传输两次数据才是输出一个脉冲,需要传输的数据总量是
  * 输出脉冲数的2倍,相应的缓冲区的大小也要能被2整除
  */
#define BUFFER_SIZE                           128  // 缓冲区大小,缓存越大则进入中断的频率越低
#define DATAWIDTH                             (BUFFER_SIZE * sizeof(Spd_Buffer0[0]))


/* 定义顺时针方向为正转方向*/
#define DIR_FORWARE                           MOTOR_DIR_CW
#define DIR_REVERSAL                          MOTOR_DIR_CCW


//#define TIM_CLKSRC_FREQ                   (168e6)//(2e8)  // 时钟源频率 200,000,000Hz  //???
//#define TIM_CLKSRC_FREQ                   (16e6)


/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_SMotor[MOTORNUM];
extern MotorObj_Typedef SMotor[MOTORNUM];
extern MoveRunData_Typedef mrd_SMotor[MOTORNUM];
extern PipetteRunData_Typedef prd_SMotor[PIPNUM];
/* 函数声明 ------------------------------------------------------------------*/
void SMotor_Set_Dir  (uint16_t SMx, MotorDir_Typedef Dir);
void SMotor_Set_State(uint16_t SMx, MotorSta_Typedef Ena);

void STEPMOTOR_LSCMoveRel(int16_t SMx, float step, float accel, float decel, float speed);
void STEPMOTOR_LSC_Calc(uint16_t SMx, SpeedRampData_Typedef *pSMotor);
void Modify_MotorMoving(uint16_t SMx, float Secspd ,int32_t Step);
void Stop_MotorMoving(uint16_t SMx);              // 停止电机
bool NotAllow2Move(uint16_t SMx);

void SMOTOR1_TIMx_Init(void);
void SMOTOR2_TIMx_Init(void);
void SMOTOR3_TIMx_Init(void);
void SMOTOR4_TIMx_Init(void);


#endif	/* __SMOTOR_TIM_H__ */

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
