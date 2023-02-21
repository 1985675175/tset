/**
  ******************************************************************************
  * @file    main.c
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
#include "stm32f4xx_hal.h"

#include "bsp_spiflash.h"
#include "bsp_debug_usart.h"
#include "bsp_GeneralTIM.h"
#include "bsp_MB_slave.h"
#include "bsp_usartx.h"

#include "bsp_adc.h"
#include "bsp_stepmotor.h"
#include "bsp_key.h"
#include "bsp_BasicTIM.h" 
#include "bsp_led.h"
#include "bsp_beep.h"
#include "bsp_heat.h"
#include "string.h"
#include "bsp_limit.h"
#include "bsp_signal.h"
#include "bsp_lamp.h"
#include "bsp_timer.h"
#include <stdlib.h>
#include <math.h>

/* 私有类型定义 --------------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

typedef enum{
  NORMAL,
  INDOG,
  NEGLIM,
}BeginState_Typedef;

/* 私有宏定义 ----------------------------------------------------------------*/
#define MSG_ERR_FLAG  0xFFFF    // 接收错误 字符间超时
#define MSG_IDLE      0x0000    // 空闲状态
#define MSG_RXING     0x0001    // 正在接收数据
#define MSG_COM       0x0002    // 接收完成
#define MSG_INC       0x8000    // 数据帧不完整(两字符间的空闲间隔大于1.5个字符时间)
#define TIME_OVERRUN  100       // 定义超时时间 ms

#define MOTORSPEED        60.0f     //  单位是r/m
#define SLOWSPEED         30.0f      //  爬行速度 单位是r/m
#define ACCEL             1000.0f    //  1000 单位是ms
#define DECEL             1000.0f    //  1000 单位是ms
#define STEP              (10.0f*SMOTOR1_PULSE_REV) // (SMOTOR1_PULSEREV：转动一圈的脉冲数,6400)

/* 搜索机械原点的起始位置有4种情况 */
#define ORI_INITIAL_1_3     1 // 1,3两种情况无法区分
#define ORI_INITIAL_2_4     2 // 2,4两种情况无需区分 

#define PINSM3NEG (uint16_t)(~LIMIT3_NEG_PIN) // SM3的反转限位开关引脚，主要用于switch语句
/*
 *  回归原点的启动位置图解     +-----------------+
 *  4个启动位置,2种情况        |     D  O  G     |
 *                             +-----------------+
 *                             .                 .  
 *   |                         .                 .             |
 *   |           ↑             .                 .     ↑       |
 * --+-----------+---------------------+---------------+-------+-----
 *   ↓反转极限   +             △       ↓               +     正转极限
 *   4           3           (原点)    2               1
 */

/* 搜索机械原点的起始位置有3种情况 移液器 */
#define ORI_INITIALP_NULL     0x00 // 不确定状态，上电默认处于该状态
#define ORI_INITIALP_1        0x01 // 零点位置以上
#define ORI_INITIALP_2_3      0x02 // 零点位置以下

#define ORI_INITIALP_TREND    0x80 // 趋向零点位置内，可以爬行速度归零

/*
 *  回归原点的启动位置图解     
 *  2个启动位置,2种情况        
 *   +---------------------------------+
 *   |             D  O  G             |
 *   +---------------------------------+
 *   .                                 .  
 *   |-6.7mm   -5.5mm                  .0mm                  12.7mm|
 *   |           ↑                     .                           |
 * --+----+------+------+--------------+------------+--------------+-----
 *   ↓反转极限          +              △            +          正转极限
 *        3             2            (原点)         1         
 * Tip头：装入时，可确定不处于-6.7mm到-5.5mm区间
 */

/* 私有变量 ------------------------------------------------------------------*/
/* 发送缓冲区初始化 */
#define RX_MAX_COUNT           255  // 串口接收最大字节数

/* 私有变量 ------------------------------------------------------------------*/
__IO uint8_t aRxBuffer[RX_MAX_COUNT]={0}; // 接收缓冲区
__IO uint16_t RxCount1=0;                  // 已接收到的字节数
__IO uint8_t Frame_flag=0;                // 帧标志：1：一个新的数据帧  0：无数据帧
long double Tx_Buffer[7] = {0};
long double Rx_Buffer[7] = {0};
uint8_t cal_flag = 0;
uint8_t cal_f = 0;

uint32_t DeviceID = 0;
uint32_t FlashID = 0;
__IO TestStatus TransferStatus1 = FAILED;

__IO uint16_t Rx_MSG = MSG_IDLE;   // 接收报文状态
__IO uint8_t state=0;

__IO uint8_t cont_max_count = 0;//volt连续最大次数值
__IO uint16_t timer_count_volt=0;

__IO uint8_t timer_flag_volt=0;//volt状态计数时间到达标志
__IO uint16_t timer_count_cpu=0;
__IO uint8_t timer_flag_cpu=0;//cpu状态计数时间到达标志
__IO uint16_t timer_count_motor=0;
__IO uint16_t timer_flag_motor=0;//motor状态计数时间到达标志
__IO uint16_t timer_count_delay=0;

__IO uint32_t Initial_P[MOTORNUM]    = {0};          // 启动位置
__IO bool OriginFlag[MOTORNUM]       = {false,false,false,false};// 用于标记启动回原点控制
__IO bool PositionInitFlag[MOTORNUM] = {false,false,false,false};// 用于标记已经回到原点
__IO bool TrendFlag[MOTORNUM]       = {false,false,false,false};// 用于标记原点趋向控制
__IO uint32_t OriginCmdLast = 0; //上次原点控制命令,0表示默认未接收过

__IO bool LoadFlag[MOTORNUM]       = {false,false,false,false};// 用于标记启动装针管控制
__IO bool LoadInitFlag[MOTORNUM] = {false,false,false,false};// 用于标记已经装上针管

__IO bool UnloadFlag[MOTORNUM]     = {false,false,false,false};// 用于标记启动卸针管控制
__IO bool UnloadInitFlag[MOTORNUM] = {false,false,false,false};// 用于标记已经卸下针管

__IO bool NegTrend[MOTORNUM] = {false,false,false,false}; //NegTrend负极限趋势

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

static void SystemClock_Config(void);
void Position_Initial(uint16_t SMx, float speed);
void Position0(uint16_t SMx, uint16_t GPIO_Pin);

/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //打开HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * 函数功能: 填充内存
  * 输入参数: buf:内存空间首地址,Code:功能码
  * 返 回 值: 无
  * 说    明: 功能不同的功能码填充内容不一的内存空间
  */
void FillBuf(uint8_t* buf,uint8_t Code)
{
  uint16_t i = 0;
  uint16_t j = 1;
  switch(Code)
  {
    case FUN_CODE_01H:
    case FUN_CODE_02H:
    case FUN_CODE_05H:
      for(i= 0;i<0x200;i++)
        buf[i] = j = !j;
    break;
    case FUN_CODE_03H:
    case FUN_CODE_06H:
    case FUN_CODE_10H:
      j = 0x000F;
      for(i= 0;i<0x250;i++)
      buf[i] = j++;
    break;
  }
}

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{
	uint8_t txbuf[50];
  uint16_t crc_check = 0;
  uint8_t Ex_code = 0,i=0;

	/* 复位所有外设，初始化Flash接口和系统滴答定时器(HSI) */
	HAL_Init();
  
	/* 配置系统时钟 */
	SystemClock_Config();
  
  /* 板载串口log初始化 */
  MX_RS485_USARTx_Init();
  /* 板载SPIFlash初始化 */
  MX_SPIFlash_Init();
  /* Get SPI Flash Device ID */
  HAL_Delay(100);
	DeviceID = SPI_FLASH_ReadDeviceID();
  /* Get SPI Flash ID */
	FlashID = SPI_FLASH_ReadID();
	memcpy(txbuf,"检测到华邦串行flash\n",50);
  TX_MODE();
  HAL_UART_Transmit(&huartx_RS485,txbuf,strlen((char *)txbuf),1000);
 // printf("FlashID is 0x%X,  Manufacturer Device ID is 0x%X\n", FlashID, DeviceID);
	/* Check the SPI Flash ID */
	if (FlashID == SPI_FLASH_ID)  /* #define  sFLASH_ID  0XEF4018 */
	{	
		//printf("检测到华邦串行flash W25Q16 !\n");
		
		SPI_FLASH_BufferRead(&cal_flag, 0, 1);
    if( cal_flag == 0x55)
    {      
        SPI_FLASH_BufferRead((void*)Tx_Buffer, 1, sizeof(Tx_Buffer));
       // for(i=0;i<7;i++ )
         // printf("rx = %LF \n",Tx_Buffer[i]);
    }    
    else
    {
      cal_flag = 0x55;
      SPI_FLASH_SectorErase(0);
      SPI_FLASH_BufferWrite(&cal_flag, 0, 1); 
      
      for( i=0; i<7; i++ )
          Tx_Buffer[i] = i +0.1;
      
      SPI_FLASH_BufferWrite((void*)Tx_Buffer, 1, sizeof(Tx_Buffer));
      
     // for(i=0; i<7;i++ )
       // printf("tx = %LF \n",Tx_Buffer[i]);
    } 
	}
	else
	{    
		//printf("获取不到 W25Q128 ID!\n");
	}
	
  /* ADC 初始化 */
//	ADC_Init();

  /* 板载LED初始化 */
  LED_GPIO_Init();
  /* 基本定时器初始化：1ms中断一次 */
  BASIC_TIMx_Init();
  /* 在中断模式下启动定时器 */
  HAL_TIM_Base_Start_IT(&htimx_led);
	MX_TIM7_Init();
	HAL_TIM_Base_Start_IT(&htim7);
  /* 板载蜂鸣器初始化 */
//  BEEP_GPIO_Init();

  /* 加热管初始化 */
  HEAT_GPIO_Init();

  /* 初始化串口并配置串口中断优先级 */
  MX_DEBUG_USART_Init(); 

  /* 定时器初始化 */
  GENERAL_TIMx_Init();
  /* 使能定时器中断 */
  __HAL_TIM_ENABLE_IT(&htimx,TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&htimx,TIM_IT_UPDATE);
  /* 设置字符间超时时间1.5个字符的时间 */
  __HAL_TIM_SET_COMPARE(&htimx,TIM_CHANNEL_1,(uint16_t)OVERTIME_15CHAR);
  /* 设置帧间超时时间3个字符的时间 */
  __HAL_TIM_SET_AUTORELOAD(&htimx,(uint16_t)OVERTIME_35CHAR); // 设置帧内间隔时间
  /* 申请内存空间作为线圈和输入离散量,对应功能码01H和02H
   * 每一Byte就是一个Coil或者Input
   */
  PduData.PtrCoilbase = (uint8_t*)malloc(sizeof(uint8_t)*0x200);
  FillBuf((uint8_t*)PduData.PtrCoilbase,FUN_CODE_01H);
  
  PduData.PtrHoldingbase = (uint16_t*)malloc(sizeof(uint16_t)*0x125);
  FillBuf((uint8_t*)PduData.PtrHoldingbase,FUN_CODE_03H);
  
  //printf(" -------启用Modbus通信协议------ \n");
  //printf("xy-drive  Modbus从机  RTU通信模式\n");
  Rx_MSG = MSG_IDLE;

	/* 板载按键引脚初始化配置 */
	KEY_GPIO_Init();

  HEAT_StateSet(HEATState_ON);    
  BEEP_StateSet(BEEPState_ON);    
	/* 初始化配置步进电机1控制定时器 */
	SMOTOR1_TIMx_Init();
	SMOTOR2_TIMx_Init();
	SMOTOR3_TIMx_Init();
	SMOTOR4_TIMx_Init();
  HEAT_StateSet(HEATState_OFF);    
  BEEP_StateSet(BEEPState_OFF);    

	/* 限位开关初始化 */
	Limit_GPIO_Init();

	/* 信号检测初始化 */
	SIG_GPIO_Init();

	/* 照明灯控制初始化 */
	LAMP_GPIO_Init();

	/* 用于debug的时候可以在断点处停止定时器时钟.*/
	__HAL_DBGMCU_FREEZE_TIM8();
//	__HAL_DBGMCU_FREEZE_TIM4();

//	LAMP_StateSet(LAMP_ON);
//	STEPMOTOR_LSCMoveRel(SM1, fabs(STEP), ACCEL, DECEL, MOTORSPEED); 
//	STEPMOTOR_LSCMoveRel(SM2, fabs(STEP), ACCEL, DECEL, MOTORSPEED); 
//	LAMP_StateSet(LAMP_OFF);
 
memcpy(txbuf,"这是一个串口中断接收回显实验\n",50);
  TX_MODE();
  HAL_UART_Transmit(&huartx_RS485,txbuf,strlen((char *)txbuf),1000);
  
  memcpy(txbuf,"输入数据并以回车键结束\n",100);
  HAL_UART_Transmit(&huartx_RS485,txbuf,strlen((char *)txbuf),1000);

  /* 使能接收中断 */	
  __HAL_UART_ENABLE_IT(&huartx_RS485,UART_IT_RXNE);
		
  RX_MODE();
/* 无限循环 */
	while (1)
	{
		//温度、电压采样
		if(timer_flag_volt==1)
    {
      timer_flag_volt=0;
      float temp = 0;
      temp = get_ntc_t_val();

    //  printf("电源电压=%0.1fV, NTC=%0.0fΩ, T=%0.1f℃.\r\n", 
           //  get_vbus_val(), get_ntc_r_val(), temp);
      
      if (temp < TEMP_MIN || temp > TEMP_MAX)    // 判断是不是超过限定的值
      {
        if (cont_max_count++ > 5)    // 连续5次超过
        {
          cont_max_count = 0;
         // printf("温度超过限制！请检查原因！\r\n");
//          while(1);
        }
      }
    }
		
    //LED灯闪烁
		//机器运行状态
		if(timer_flag_cpu==1)
    {
      timer_flag_cpu=0;
      LED1_TOGGLE();
    }
		//电机运行状态
		if(timer_flag_motor==1)
    {
      timer_flag_motor=0;
			if(SMotor[SMZ1].MotorRuning || SMotor[SMZ2].MotorRuning)// 当前正处于转动状态
				LED2_TOGGLE();
			else
				LED2_OFF();
			if(SMotor[SMP1].MotorRuning || SMotor[SMP2].MotorRuning)// 当前正处于转动状态
				LED3_TOGGLE();
			else
				LED3_OFF();
		}
		
		//信号检测处理
		if(Signal[SIG_NUM_DOOR].flag)//门锁
		{
			Signal[SIG_NUM_DOOR].flag = 0;
			if(Signal[SIG_NUM_DOOR].last_level != Signal[SIG_NUM_DOOR].valid_level)
				  BEEP_StateSet(BEEPState_ON);    
			else
				  BEEP_StateSet(BEEPState_OFF);    
		}
		if(Signal[SIG_NUM_SLOT].flag)//液槽
		{
			Signal[SIG_NUM_SLOT].flag = 0;
			if(Signal[SIG_NUM_SLOT].last_level != Signal[SIG_NUM_SLOT].valid_level)
				  BEEP_StateSet(BEEPState_ON);    
			else
				  BEEP_StateSet(BEEPState_OFF);    
		}

    //modbus协议解析
		/* 协议解析生效 */
		if(state==1)
    {
      state=0;
       // if(MB_Effect(&PduData))
           // printf("Effect function running Error!!!\n Please check!\n");
    }
    /* 接收到一帧的数据,对缓存提取数据 */
    if(Rx_MSG == MSG_COM)
    {
      for(i=0;i<8;i++)
      {
       // printf("Rx_Buf[%d]=%d\n",i,Rx_Buf[i]);
      }      
      /* 收到非本机地址的响应请求 */
      if((Rx_Buf[0] != MB_SLAVEADDR )&&(Rx_Buf[0] != MB_ALLSLAVEADDR))
      {
        Rx_MSG = MSG_IDLE;
        continue;
      }
      /* 解析数据帧 */
      MB_Parse_Data();

      /* CRC 校验正确 */
      crc_check = ( (Rx_Buf[RxCount-1]<<8) | Rx_Buf[RxCount-2] );
      if(crc_check == PduData._CRC) 
      {
        /* 分析并执行 */
				Ex_code = MB_Analyze_Execute();
        /* 出现异常 */
        if(Ex_code !=EX_CODE_NONE)
        {
          MB_Exception_RSP(PduData.Code,Ex_code);
        }
        else
        {
          state=1;
          MB_RSP(PduData.Code);
        }
      }
      /* 重新标记为空闲状态 */
      Rx_MSG = MSG_IDLE;
    }

//		if(KEY1_StateRead() == KEY_DOWN)
//		{
//			Position_Initial(SM1, MOTORSPEED); // 回原点
//			Position_Initial(SM2, MOTORSPEED); // 回原点
//		}
//		if(KEY2_StateRead() == KEY_DOWN)
//		{
//			STEPMOTOR_LSCMoveRel(SM1, fabs(STEP), ACCEL, DECEL, 120); 
//			STEPMOTOR_LSCMoveRel(SM2, fabs(STEP), ACCEL, DECEL, 120); 
//		}
//    
//		/* KEY3，KEY4控制电机转动 */
//		if(KEY3_StateRead() == KEY_DOWN)
//		{
//			STEPMOTOR_LSCMoveRel(SM1, -fabs(STEP), ACCEL, DECEL, 120); 
//			STEPMOTOR_LSCMoveRel(SM2, -fabs(STEP), ACCEL, DECEL, 120); 
//		}
//		if(KEY4_StateRead() == KEY_DOWN)
//		{
//			STEPMOTOR_LSCMoveRel(SM1, fabs(STEP), ACCEL, DECEL, 120); 
//			STEPMOTOR_LSCMoveRel(SM2, fabs(STEP), ACCEL, DECEL, 120); 
//		}
//		if(KEY5_StateRead() == KEY_DOWN)
//		{
//			STEPMOTOR_LSCMoveRel(SM1, fabs(STEP), ACCEL, DECEL, 120); 
//			STEPMOTOR_LSCMoveRel(SM2, fabs(STEP), ACCEL, DECEL, 120); 
//		}
	}
}

/** 
  * 函数功能: 串口接收中断回调函数
  * 输入参数: 串口句柄
  * 返 回 值: 无
  * 说    明: 使用一个定时器的比较中断和更新中断作为接收超时判断
  *           只要接收到数据就将定时器计数器清0,当发生比较中断的时候
  *           说明已经超时1.5个字符的时间,认定为帧错误,如果是更新中断
  *           则认为是接受完成
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &husart_debug)
  {
    switch(Rx_MSG)
    {
      /* 接收到第一个字符之后开始计时1.5/3.5个字符的时间 */
      case MSG_IDLE:
        Rx_MSG = MSG_RXING;
        RxCount = 0;
        HAL_TIM_Base_Start(&htimx);
        break;
      
      /* 距离上一次接收到数据已经超过1.5个字符的时间间隔,认定为数据帧不完整 */
      case MSG_ERR_FLAG:
        Rx_MSG = MSG_INC; // 数据帧不完整
      break;
    }
    
    /* 使能继续接收 */
    Rx_Buf[RxCount] = tmp_Rx_Buf;
    RxCount++;
    __HAL_TIM_SET_COUNTER(&htimx,0); // 重设计数时间
    HAL_UART_Receive_IT(&husart_debug,(uint8_t*)&tmp_Rx_Buf,1);
  }
}
/** 
  * 函数功能: 定时器比较中断回调函数
  * 输入参数: 定时器句柄
  * 返 回 值: 无
  * 说    明: 标记已经超时1.5个字符的时间没有接收到数据
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 如果是第一次发生比较中断,则暂定为错误标记 */
  if(Rx_MSG != MSG_INC)
    Rx_MSG = MSG_ERR_FLAG;
  
  /* 如果是第二次进入比较中断,则认定为报文不完整 */
  else
    Rx_MSG = MSG_INC;
}
/** 
  * 函数功能: 定时器更新中断回调函数
  * 输入参数: 定时器句柄
  * 返 回 值: 无
  * 说    明: 超时3.5个字符的时间没有接收到数据,认为是空闲状态 
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim7)
	{
		timer_count_delay++;
		if(KEY1_StateRead() == KEY_DOWN)
		{
			Position_Initial(SM1, MOTORSPEED); // 回原点
			Position_Initial(SM2, MOTORSPEED); // 回原点
		}
		if(KEY2_StateRead() == KEY_DOWN)
		{
			STEPMOTOR_LSCMoveRel(SM1, fabs(STEP), ACCEL, DECEL, 120); 
			STEPMOTOR_LSCMoveRel(SM2, fabs(STEP), ACCEL, DECEL, 120); 
		}   
		/* KEY3，KEY4控制电机转动 */
		if(KEY3_StateRead() == KEY_DOWN)
		{
			STEPMOTOR_LSCMoveRel(SM1, -fabs(STEP), ACCEL, DECEL, 120); 
			STEPMOTOR_LSCMoveRel(SM2, -fabs(STEP), ACCEL, DECEL, 120); 
		}
		if(KEY4_StateRead() == KEY_DOWN)
		{
			STEPMOTOR_LSCMoveRel(SM1, fabs(STEP), ACCEL, DECEL, 120); 
			STEPMOTOR_LSCMoveRel(SM2, fabs(STEP), ACCEL, DECEL, 120); 
		}
		if(KEY5_StateRead() == KEY_DOWN)
		{
			STEPMOTOR_LSCMoveRel(SM1, fabs(STEP), ACCEL, DECEL, 120); 
			STEPMOTOR_LSCMoveRel(SM2, fabs(STEP), ACCEL, DECEL, 120); 
		}
	}
	if(htim == &htimx)
	{
		/* 如果已经标记了接收到不完整的数据帧,则继续标记为不完整的数据帧 */
		if(Rx_MSG == MSG_INC)
		{
			Rx_MSG = MSG_INC;
		}
		/* 在正常情况下时接收完成 */
		else
		{
			Rx_MSG = MSG_COM;
		}
	}
	if(htim == &htimx_led)
	{
		timer_count_volt++;
		if(timer_count_volt >= 50)
		{
			timer_count_volt = 0;
			timer_flag_volt = 1;
		}
		timer_count_cpu++;
		if(timer_count_cpu >= 500)
		{
			timer_count_cpu = 0;
			timer_flag_cpu = 1;
		}
		timer_count_motor++;
		if(timer_count_motor >= 400)
		{
			timer_count_motor = 0;
			timer_flag_motor = 1;
		}
		//信号时间处理
		for(uint8_t i=0;i<SIG_NUM_MAX;i++)
		{
			SIG_SetLevelFromNum(&Signal[i], i);
			SIG_RunTime(&Signal[i]);
		}
		//定时时间处理
		for(uint8_t i=0;i<TMR_NUM_MAX;i++)
		{
			TMR_RunTime(&Timer[i]);
		}
	}
}

/**
  * 函数功能: 回归原点
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 根据当前位置判断回原点的时候的方向和记录启动位置
  */
void Position_Initial(uint16_t SMx, float speed)
{
	float  step = 0;
	GPIO_TypeDef *LIMIT_NEG_GPIO = LIMIT1_NEG_GPIO_PORT;
	GPIO_TypeDef *LIMIT_ORI_GPIO = LIMIT1_ORI_GPIO_PORT;
	uint16_t NEG_Pin = LIMIT1_NEG_PIN;
	uint16_t ORI_Pin = LIMIT1_ORI_PIN;
	GPIO_PinState NEG_ACTIVE_LEVEL = LIMIT1_NEG_ACTIVE_LEVEL;
	GPIO_PinState ORI_ACTIVE_LEVEL = LIMIT1_ORI_ACTIVE_LEVEL;
  
	if(SMotor[SMx].MotorRuning)
		return;
  
	if(SMx == SM2)
	{
		LIMIT_NEG_GPIO = LIMIT2_NEG_GPIO_PORT;
		LIMIT_ORI_GPIO = LIMIT2_ORI_GPIO_PORT;
		NEG_Pin = LIMIT2_NEG_PIN;
		ORI_Pin = LIMIT2_ORI_PIN;
		NEG_ACTIVE_LEVEL = LIMIT2_NEG_ACTIVE_LEVEL;
		ORI_ACTIVE_LEVEL = LIMIT2_ORI_ACTIVE_LEVEL;
	}
	else if(SMx == SM3)
	{
		LIMIT_NEG_GPIO = LIMIT3_NEG_GPIO_PORT;
		LIMIT_ORI_GPIO = LIMIT3_ORI_GPIO_PORT;
		NEG_Pin = LIMIT3_NEG_PIN;
		ORI_Pin = LIMIT3_ORI_PIN;
		NEG_ACTIVE_LEVEL = LIMIT3_NEG_ACTIVE_LEVEL;
		ORI_ACTIVE_LEVEL = LIMIT3_ORI_ACTIVE_LEVEL;
	}
	else if(SMx == SM4)
	{
		LIMIT_NEG_GPIO = LIMIT4_NEG_GPIO_PORT;
		LIMIT_ORI_GPIO = LIMIT4_ORI_GPIO_PORT;
		NEG_Pin = LIMIT4_NEG_PIN;
		ORI_Pin = LIMIT4_ORI_PIN;
		NEG_ACTIVE_LEVEL = LIMIT4_NEG_ACTIVE_LEVEL;
		ORI_ACTIVE_LEVEL = LIMIT4_ORI_ACTIVE_LEVEL;
	}
  
	/*--------------------------------------*/
    
	/* 根据当前启动位置决定回原点的方向 */
	if(HAL_GPIO_ReadPin(LIMIT_NEG_GPIO, NEG_Pin) != NEG_ACTIVE_LEVEL )
	{ 
		/* 区分是否在原点范围内 */
		if(HAL_GPIO_ReadPin(LIMIT_ORI_GPIO, ORI_Pin) != ORI_ACTIVE_LEVEL)
		{
			step = INT32_MIN;      //  反转
			Initial_P[SMx] = ORI_INITIAL_1_3;
		}
		else
		{
			step = INT32_MAX;      //  正转
			Initial_P[SMx] = ORI_INITIAL_2_4;
		}
	}
	/* 是在负极限位置启动，则回归原点的方向是正转 */
	else
	{
		step = INT32_MAX;      //  正转
		Initial_P[SMx] = ORI_INITIAL_2_4;
	}
  if((SMx == SM2) || (SMx == SM4))//移液器原点，默认在[-5.5,12.7mm]区间运行
  {
      /* 区分相对原点的位置 */
      if(HAL_GPIO_ReadPin(LIMIT_ORI_GPIO, ORI_Pin) != ORI_ACTIVE_LEVEL)
      {
          step = INT32_MIN;      //  反转
          Initial_P[SMx] = ORI_INITIALP_1;
      }
      else
      {
          step = INT32_MAX;      //  正转
          Initial_P[SMx] = ORI_INITIALP_2_3;
      }
  }
STEPMOTOR_LSCMoveRel(SMx, step, mrd_SMotor[SMx].Acc_Time, mrd_SMotor[SMx].Dec_Time, speed); 
	OriginFlag[SMx] = true;
}

/**
  * 函数功能: 限位开关中断回调函数
  * 输入参数: GPIO_Pin 限位开关所连接的引脚
  * 返 回 值: 无
  * 说    明: 限位开关控制电机停止
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if( NotAllow2Move(SM1) )
	{
		Stop_MotorMoving(SM1);// 极限位置停止转动
	}
	if( NotAllow2Move(SM2) )
	{
		Stop_MotorMoving(SM2);// 极限位置停止转动
	}
	if( NotAllow2Move(SM3) )
	{
		Stop_MotorMoving(SM3);// 极限位置停止转动
	}
	if( NotAllow2Move(SM4) )
	{
		Stop_MotorMoving(SM4);// 极限位置停止转动
	}
	/* 搜索原点 */
	if(OriginFlag[SM1])
	{
		Position0(SM1, GPIO_Pin);
	}
	if(OriginFlag[SM2])
	{
		Position0(SM2, GPIO_Pin);
	}
	if(OriginFlag[SM3])
	{
		Position0(SM3, GPIO_Pin);
	}
	if(OriginFlag[SM4])
	{
		Position0(SM4, GPIO_Pin);
	}
}

/**
  * 函数功能: 机械原点控制
  * 输入参数: SMx：电机编号，GPIO_Pin：限位开关引脚
  * 返 回 值: 无
  * 说    明: 回原点过程中，限位开关动作。
  */
void Position0(uint16_t SMx, uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
    case LIMIT1_ORI_PIN:
		if(SMx != SM1) return;
		/* 碰到DOG前端，固定为减速 */
		if(HAL_GPIO_ReadPin(LIMIT1_ORI_GPIO_PORT, GPIO_Pin) == LIMIT1_ORI_ACTIVE_LEVEL)
		{
			if(Initial_P[SMx] == ORI_INITIAL_1_3)  // 减速
				Modify_MotorMoving(SMx, mrd_SMotor[SMx].Craml_Speed, INT32_MAX);
		}
		else/* 碰到DOG后端*/
		{
			Stop_MotorMoving(SMx);// 如果起点位置不是4，则可以直接停止，标记原点
			if( Initial_P[SMx] != ORI_INITIAL_2_4) // 如果启动位置式1,则直接停止，完成
			{
                OriginFlag[SMx] = false;
                PositionInitFlag[SMx] = true;
                mrd_SMotor[SMx].Coord = 0;//坐标清零
			}
			else 
			{
				/* 启动位置是2,4，碰到DOG后端则换方向，原本是正转，这里换成反转 */
				Position_Initial(SMx, mrd_SMotor[SMx].Craml_Speed);
			}
		}
		break;
    case LIMIT1_NEG_PIN:
		if(SMx != SM1) return;
		/* 启动位置是3,更换方向 */
		Position_Initial(SMx, mrd_SMotor[SMx].Lim_Speed);
		break;
    
    case LIMIT2_ORI_PIN:
		if(SMx != SM2) return;
		/* 碰到正原点，固定为爬行速度 */
		if(HAL_GPIO_ReadPin(LIMIT2_ORI_GPIO_PORT, GPIO_Pin) == LIMIT2_ORI_ACTIVE_LEVEL)
		{
			if(Initial_P[SMx] == ORI_INITIALP_1)  // 碰到原点后，原本是正转，换为反转
      {
        Stop_MotorMoving(SMx);
        TrendFlag[SMx] = true;
        Position_Initial(SMx, mrd_SMotor[SMx].Craml_Speed);
      }
		}
		else/* 碰到负原点*/
		{
			if(Initial_P[SMx] == ORI_INITIALP_2_3)  
      {
        Stop_MotorMoving(SMx);
        if(TrendFlag[SMx]) //如果启动位置是趋向位置,则直接停止，完成
        {
          // 如果起点位置趋向位置，则可以直接停止，标记原点
          OriginFlag[SMx] = false;
          PositionInitFlag[SMx] = true;
          TrendFlag[SMx] = false;
          mrd_SMotor[SMx].Coord = 0;//坐标清零
        }
        else // 碰到原点后，原本是正转，换为反转
        {
          TrendFlag[SMx] = true;
          Position_Initial(SMx, mrd_SMotor[SMx].Craml_Speed);
        }
      }
		}
		break;
//    case LIMIT2_NEG_PIN:
//		if(SMx != SM2) return;
//		/* 启动位置是3,更换方向 */
//		Position_Initial(SMx, mrd_SMotor[SMx].Lim_Speed);
//		break;
    case LIMIT3_ORI_PIN:
		if(SMx != SM3) return;
		/* 碰到DOG前端，固定为减速 */
		if(HAL_GPIO_ReadPin(LIMIT3_ORI_GPIO_PORT, GPIO_Pin) == LIMIT3_ORI_ACTIVE_LEVEL)
		{
			if(Initial_P[SMx] == ORI_INITIAL_1_3)  // 减速
			Modify_MotorMoving(SMx, mrd_SMotor[SMx].Craml_Speed, INT32_MAX);
		}
		else/* 碰到DOG后端*/
		{
			Stop_MotorMoving(SMx);// 如果起点位置不是4，则可以直接停止，标记原点
			if( Initial_P[SMx] != ORI_INITIAL_2_4) // 如果启动位置式1,则直接停止，完成
			{
				OriginFlag[SMx] = false;
				PositionInitFlag[SMx] = true;
                mrd_SMotor[SMx].Coord = 0;//坐标清零
			}
			else 
			{
				/* 启动位置是2,4，碰到DOG后端则换方向，原本是正转，这里换成反转 */
				Position_Initial(SMx, mrd_SMotor[SMx].Craml_Speed);
			}
		}
		break;
    case LIMIT3_NEG_PIN:
		if(SMx != SM3) return;
		/* 启动位置是3,更换方向 */
		Position_Initial(SMx, mrd_SMotor[SMx].Lim_Speed);
		break;
    case LIMIT4_ORI_PIN:
		if(SMx != SM4) return;
		/* 碰到正原点，固定为爬行速度 */
		if(HAL_GPIO_ReadPin(LIMIT4_ORI_GPIO_PORT, GPIO_Pin) == LIMIT4_ORI_ACTIVE_LEVEL)
		{
			if(Initial_P[SMx] == ORI_INITIALP_1)  // 碰到原点后，原本是正转，换为反转
      {
        Stop_MotorMoving(SMx);
        TrendFlag[SMx] = true;
        Position_Initial(SMx, mrd_SMotor[SMx].Craml_Speed);
      }
		}
		else/* 碰到负原点*/
		{
			if(Initial_P[SMx] == ORI_INITIALP_2_3)  
      {
        Stop_MotorMoving(SMx);
        if(TrendFlag[SMx]) //如果启动位置是趋向位置,则直接停止，完成
        {
          // 如果起点位置趋向位置，则可以直接停止，标记原点
          OriginFlag[SMx] = false;
          PositionInitFlag[SMx] = true;
          TrendFlag[SMx] = false;
          mrd_SMotor[SMx].Coord = 0;//坐标清零
        }
        else // 碰到原点后，原本是正转，换为反转
        {
          TrendFlag[SMx] = true;
          Position_Initial(SMx, mrd_SMotor[SMx].Craml_Speed);
        }
      }
		}
		break;
//    case LIMIT4_NEG_PIN:
//		if(SMx != SM4) return;
//		/* 启动位置是3,更换方向 */
//		Position_Initial(SMx, mrd_SMotor[SMx].Lim_Speed);
//		break;
	}
}

///**
//  * 函数功能: 非阻塞模式下定时器的回调函数
//  * 输入参数: htim：定时器句柄
//  * 返 回 值: 无
//  * 说    明: 无
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  timer_count++;
//  timer_count_motor++;
//}
/**
  * 函数功能: 基本定时器硬件反初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
	   if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspDeInit 0 */

  /* USER CODE END TIM7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();

    /* TIM7 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspDeInit 1 */

  /* USER CODE END TIM7_MspDeInit 1 */
  }
  if(htim_base->Instance==BASIC_TIMx)
  {
    /* 基本定时器外设时钟禁用 */
    BASIC_TIM_RCC_CLK_DISABLE();

    /* 关闭外设中断 */
    HAL_NVIC_DisableIRQ(BASIC_TIM_IRQ);
  }

  if(htim_base->Instance==GENERAL_TIMx)
  {
    /* 基本定时器外设时钟禁用 */
    GENERAL_TIM_RCC_CLK_DISABLE();

    /* 关闭外设中断 */
    HAL_NVIC_DisableIRQ(GENERAL_TIM_IRQ);
  }
} 
/**
  * 函数功能: 串口中断服务函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void MX_RS485_UARTx_IRQHandler(void)
{
	if(__HAL_USART_GET_FLAG(&huartx_RS485,USART_FLAG_RXNE)!= RESET) // 接收中断：接收到数据
	{
		uint8_t data;
		data=READ_REG(huartx_RS485.Instance->DR); // 读取数据
		if(RxCount1==0) // 如果是重新接收到数据帧，开启串口空闲中断
		{
			__HAL_UART_CLEAR_FLAG(&huartx_RS485,USART_FLAG_IDLE); // 清除空闲中断标志
		  __HAL_UART_ENABLE_IT(&huartx_RS485,UART_IT_IDLE);     // 使能空闲中断	    
		}
		if(RxCount1<RX_MAX_COUNT)    // 判断接收缓冲区未满
		{
			aRxBuffer[RxCount1]=data;  // 保存数据
			RxCount1++;                // 增加接收字节数计数
		}
	}
	else	if(__HAL_USART_GET_FLAG(&huartx_RS485,USART_FLAG_IDLE)!= RESET) // 串口空闲中断
	{
		__HAL_UART_CLEAR_FLAG(&huartx_RS485,USART_FLAG_IDLE); // 清除空闲中断标志
		__HAL_UART_DISABLE_IT(&huartx_RS485,UART_IT_IDLE);    // 关闭空闲中断
		Frame_flag=1;		                                 // 数据帧置位，标识接收到一个完整数据帧
	}
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/

