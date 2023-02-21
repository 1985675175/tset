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

  
/* ����ͷ�ļ� ----------------------------------------------------------------*/
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

/* ˽�����Ͷ��� --------------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

typedef enum{
  NORMAL,
  INDOG,
  NEGLIM,
}BeginState_Typedef;

/* ˽�к궨�� ----------------------------------------------------------------*/
#define MSG_ERR_FLAG  0xFFFF    // ���մ��� �ַ��䳬ʱ
#define MSG_IDLE      0x0000    // ����״̬
#define MSG_RXING     0x0001    // ���ڽ�������
#define MSG_COM       0x0002    // �������
#define MSG_INC       0x8000    // ����֡������(���ַ���Ŀ��м������1.5���ַ�ʱ��)
#define TIME_OVERRUN  100       // ���峬ʱʱ�� ms

#define MOTORSPEED        60.0f     //  ��λ��r/m
#define SLOWSPEED         30.0f      //  �����ٶ� ��λ��r/m
#define ACCEL             1000.0f    //  1000 ��λ��ms
#define DECEL             1000.0f    //  1000 ��λ��ms
#define STEP              (10.0f*SMOTOR1_PULSE_REV) // (SMOTOR1_PULSEREV��ת��һȦ��������,6400)

/* ������еԭ�����ʼλ����4����� */
#define ORI_INITIAL_1_3     1 // 1,3��������޷�����
#define ORI_INITIAL_2_4     2 // 2,4��������������� 

#define PINSM3NEG (uint16_t)(~LIMIT3_NEG_PIN) // SM3�ķ�ת��λ�������ţ���Ҫ����switch���
/*
 *  �ع�ԭ�������λ��ͼ��     +-----------------+
 *  4������λ��,2�����        |     D  O  G     |
 *                             +-----------------+
 *                             .                 .  
 *   |                         .                 .             |
 *   |           ��             .                 .     ��       |
 * --+-----------+---------------------+---------------+-------+-----
 *   ����ת����   +             ��       ��               +     ��ת����
 *   4           3           (ԭ��)    2               1
 */

/* ������еԭ�����ʼλ����3����� ��Һ�� */
#define ORI_INITIALP_NULL     0x00 // ��ȷ��״̬���ϵ�Ĭ�ϴ��ڸ�״̬
#define ORI_INITIALP_1        0x01 // ���λ������
#define ORI_INITIALP_2_3      0x02 // ���λ������

#define ORI_INITIALP_TREND    0x80 // �������λ���ڣ����������ٶȹ���

/*
 *  �ع�ԭ�������λ��ͼ��     
 *  2������λ��,2�����        
 *   +---------------------------------+
 *   |             D  O  G             |
 *   +---------------------------------+
 *   .                                 .  
 *   |-6.7mm   -5.5mm                  .0mm                  12.7mm|
 *   |           ��                     .                           |
 * --+----+------+------+--------------+------------+--------------+-----
 *   ����ת����          +              ��            +          ��ת����
 *        3             2            (ԭ��)         1         
 * Tipͷ��װ��ʱ����ȷ��������-6.7mm��-5.5mm����
 */

/* ˽�б��� ------------------------------------------------------------------*/
/* ���ͻ�������ʼ�� */
#define RX_MAX_COUNT           255  // ���ڽ�������ֽ���

/* ˽�б��� ------------------------------------------------------------------*/
__IO uint8_t aRxBuffer[RX_MAX_COUNT]={0}; // ���ջ�����
__IO uint16_t RxCount1=0;                  // �ѽ��յ����ֽ���
__IO uint8_t Frame_flag=0;                // ֡��־��1��һ���µ�����֡  0��������֡
long double Tx_Buffer[7] = {0};
long double Rx_Buffer[7] = {0};
uint8_t cal_flag = 0;
uint8_t cal_f = 0;

uint32_t DeviceID = 0;
uint32_t FlashID = 0;
__IO TestStatus TransferStatus1 = FAILED;

__IO uint16_t Rx_MSG = MSG_IDLE;   // ���ձ���״̬
__IO uint8_t state=0;

__IO uint8_t cont_max_count = 0;//volt����������ֵ
__IO uint16_t timer_count_volt=0;

__IO uint8_t timer_flag_volt=0;//volt״̬����ʱ�䵽���־
__IO uint16_t timer_count_cpu=0;
__IO uint8_t timer_flag_cpu=0;//cpu״̬����ʱ�䵽���־
__IO uint16_t timer_count_motor=0;
__IO uint16_t timer_flag_motor=0;//motor״̬����ʱ�䵽���־
__IO uint16_t timer_count_delay=0;

__IO uint32_t Initial_P[MOTORNUM]    = {0};          // ����λ��
__IO bool OriginFlag[MOTORNUM]       = {false,false,false,false};// ���ڱ��������ԭ�����
__IO bool PositionInitFlag[MOTORNUM] = {false,false,false,false};// ���ڱ���Ѿ��ص�ԭ��
__IO bool TrendFlag[MOTORNUM]       = {false,false,false,false};// ���ڱ��ԭ���������
__IO uint32_t OriginCmdLast = 0; //�ϴ�ԭ���������,0��ʾĬ��δ���չ�

__IO bool LoadFlag[MOTORNUM]       = {false,false,false,false};// ���ڱ������װ��ܿ���
__IO bool LoadInitFlag[MOTORNUM] = {false,false,false,false};// ���ڱ���Ѿ�װ�����

__IO bool UnloadFlag[MOTORNUM]     = {false,false,false,false};// ���ڱ������ж��ܿ���
__IO bool UnloadInitFlag[MOTORNUM] = {false,false,false,false};// ���ڱ���Ѿ�ж�����

__IO bool NegTrend[MOTORNUM] = {false,false,false,false}; //NegTrend����������

/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/

static void SystemClock_Config(void);
void Position_Initial(uint16_t SMx, float speed);
void Position0(uint16_t SMx, uint16_t GPIO_Pin);

/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //��HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/������������ȵ���PLL��Ƶϵ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2ʱ�ӣ�84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // ʹ��CSS���ܣ�����ʹ���ⲿ�����ڲ�ʱ��ԴΪ����
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * ��������: ����ڴ�
  * �������: buf:�ڴ�ռ��׵�ַ,Code:������
  * �� �� ֵ: ��
  * ˵    ��: ���ܲ�ͬ�Ĺ�����������ݲ�һ���ڴ�ռ�
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
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{
	uint8_t txbuf[50];
  uint16_t crc_check = 0;
  uint8_t Ex_code = 0,i=0;

	/* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ��(HSI) */
	HAL_Init();
  
	/* ����ϵͳʱ�� */
	SystemClock_Config();
  
  /* ���ش���log��ʼ�� */
  MX_RS485_USARTx_Init();
  /* ����SPIFlash��ʼ�� */
  MX_SPIFlash_Init();
  /* Get SPI Flash Device ID */
  HAL_Delay(100);
	DeviceID = SPI_FLASH_ReadDeviceID();
  /* Get SPI Flash ID */
	FlashID = SPI_FLASH_ReadID();
	memcpy(txbuf,"��⵽�����flash\n",50);
  TX_MODE();
  HAL_UART_Transmit(&huartx_RS485,txbuf,strlen((char *)txbuf),1000);
 // printf("FlashID is 0x%X,  Manufacturer Device ID is 0x%X\n", FlashID, DeviceID);
	/* Check the SPI Flash ID */
	if (FlashID == SPI_FLASH_ID)  /* #define  sFLASH_ID  0XEF4018 */
	{	
		//printf("��⵽�����flash W25Q16 !\n");
		
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
		//printf("��ȡ���� W25Q128 ID!\n");
	}
	
  /* ADC ��ʼ�� */
//	ADC_Init();

  /* ����LED��ʼ�� */
  LED_GPIO_Init();
  /* ������ʱ����ʼ����1ms�ж�һ�� */
  BASIC_TIMx_Init();
  /* ���ж�ģʽ��������ʱ�� */
  HAL_TIM_Base_Start_IT(&htimx_led);
	MX_TIM7_Init();
	HAL_TIM_Base_Start_IT(&htim7);
  /* ���ط�������ʼ�� */
//  BEEP_GPIO_Init();

  /* ���ȹܳ�ʼ�� */
  HEAT_GPIO_Init();

  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_DEBUG_USART_Init(); 

  /* ��ʱ����ʼ�� */
  GENERAL_TIMx_Init();
  /* ʹ�ܶ�ʱ���ж� */
  __HAL_TIM_ENABLE_IT(&htimx,TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&htimx,TIM_IT_UPDATE);
  /* �����ַ��䳬ʱʱ��1.5���ַ���ʱ�� */
  __HAL_TIM_SET_COMPARE(&htimx,TIM_CHANNEL_1,(uint16_t)OVERTIME_15CHAR);
  /* ����֡�䳬ʱʱ��3���ַ���ʱ�� */
  __HAL_TIM_SET_AUTORELOAD(&htimx,(uint16_t)OVERTIME_35CHAR); // ����֡�ڼ��ʱ��
  /* �����ڴ�ռ���Ϊ��Ȧ��������ɢ��,��Ӧ������01H��02H
   * ÿһByte����һ��Coil����Input
   */
  PduData.PtrCoilbase = (uint8_t*)malloc(sizeof(uint8_t)*0x200);
  FillBuf((uint8_t*)PduData.PtrCoilbase,FUN_CODE_01H);
  
  PduData.PtrHoldingbase = (uint16_t*)malloc(sizeof(uint16_t)*0x125);
  FillBuf((uint8_t*)PduData.PtrHoldingbase,FUN_CODE_03H);
  
  //printf(" -------����Modbusͨ��Э��------ \n");
  //printf("xy-drive  Modbus�ӻ�  RTUͨ��ģʽ\n");
  Rx_MSG = MSG_IDLE;

	/* ���ذ������ų�ʼ������ */
	KEY_GPIO_Init();

  HEAT_StateSet(HEATState_ON);    
  BEEP_StateSet(BEEPState_ON);    
	/* ��ʼ�����ò������1���ƶ�ʱ�� */
	SMOTOR1_TIMx_Init();
	SMOTOR2_TIMx_Init();
	SMOTOR3_TIMx_Init();
	SMOTOR4_TIMx_Init();
  HEAT_StateSet(HEATState_OFF);    
  BEEP_StateSet(BEEPState_OFF);    

	/* ��λ���س�ʼ�� */
	Limit_GPIO_Init();

	/* �źż���ʼ�� */
	SIG_GPIO_Init();

	/* �����ƿ��Ƴ�ʼ�� */
	LAMP_GPIO_Init();

	/* ����debug��ʱ������ڶϵ㴦ֹͣ��ʱ��ʱ��.*/
	__HAL_DBGMCU_FREEZE_TIM8();
//	__HAL_DBGMCU_FREEZE_TIM4();

//	LAMP_StateSet(LAMP_ON);
//	STEPMOTOR_LSCMoveRel(SM1, fabs(STEP), ACCEL, DECEL, MOTORSPEED); 
//	STEPMOTOR_LSCMoveRel(SM2, fabs(STEP), ACCEL, DECEL, MOTORSPEED); 
//	LAMP_StateSet(LAMP_OFF);
 
memcpy(txbuf,"����һ�������жϽ��ջ���ʵ��\n",50);
  TX_MODE();
  HAL_UART_Transmit(&huartx_RS485,txbuf,strlen((char *)txbuf),1000);
  
  memcpy(txbuf,"�������ݲ��Իس�������\n",100);
  HAL_UART_Transmit(&huartx_RS485,txbuf,strlen((char *)txbuf),1000);

  /* ʹ�ܽ����ж� */	
  __HAL_UART_ENABLE_IT(&huartx_RS485,UART_IT_RXNE);
		
  RX_MODE();
/* ����ѭ�� */
	while (1)
	{
		//�¶ȡ���ѹ����
		if(timer_flag_volt==1)
    {
      timer_flag_volt=0;
      float temp = 0;
      temp = get_ntc_t_val();

    //  printf("��Դ��ѹ=%0.1fV, NTC=%0.0f��, T=%0.1f��.\r\n", 
           //  get_vbus_val(), get_ntc_r_val(), temp);
      
      if (temp < TEMP_MIN || temp > TEMP_MAX)    // �ж��ǲ��ǳ����޶���ֵ
      {
        if (cont_max_count++ > 5)    // ����5�γ���
        {
          cont_max_count = 0;
         // printf("�¶ȳ������ƣ�����ԭ��\r\n");
//          while(1);
        }
      }
    }
		
    //LED����˸
		//��������״̬
		if(timer_flag_cpu==1)
    {
      timer_flag_cpu=0;
      LED1_TOGGLE();
    }
		//�������״̬
		if(timer_flag_motor==1)
    {
      timer_flag_motor=0;
			if(SMotor[SMZ1].MotorRuning || SMotor[SMZ2].MotorRuning)// ��ǰ������ת��״̬
				LED2_TOGGLE();
			else
				LED2_OFF();
			if(SMotor[SMP1].MotorRuning || SMotor[SMP2].MotorRuning)// ��ǰ������ת��״̬
				LED3_TOGGLE();
			else
				LED3_OFF();
		}
		
		//�źż�⴦��
		if(Signal[SIG_NUM_DOOR].flag)//����
		{
			Signal[SIG_NUM_DOOR].flag = 0;
			if(Signal[SIG_NUM_DOOR].last_level != Signal[SIG_NUM_DOOR].valid_level)
				  BEEP_StateSet(BEEPState_ON);    
			else
				  BEEP_StateSet(BEEPState_OFF);    
		}
		if(Signal[SIG_NUM_SLOT].flag)//Һ��
		{
			Signal[SIG_NUM_SLOT].flag = 0;
			if(Signal[SIG_NUM_SLOT].last_level != Signal[SIG_NUM_SLOT].valid_level)
				  BEEP_StateSet(BEEPState_ON);    
			else
				  BEEP_StateSet(BEEPState_OFF);    
		}

    //modbusЭ�����
		/* Э�������Ч */
		if(state==1)
    {
      state=0;
       // if(MB_Effect(&PduData))
           // printf("Effect function running Error!!!\n Please check!\n");
    }
    /* ���յ�һ֡������,�Ի�����ȡ���� */
    if(Rx_MSG == MSG_COM)
    {
      for(i=0;i<8;i++)
      {
       // printf("Rx_Buf[%d]=%d\n",i,Rx_Buf[i]);
      }      
      /* �յ��Ǳ�����ַ����Ӧ���� */
      if((Rx_Buf[0] != MB_SLAVEADDR )&&(Rx_Buf[0] != MB_ALLSLAVEADDR))
      {
        Rx_MSG = MSG_IDLE;
        continue;
      }
      /* ��������֡ */
      MB_Parse_Data();

      /* CRC У����ȷ */
      crc_check = ( (Rx_Buf[RxCount-1]<<8) | Rx_Buf[RxCount-2] );
      if(crc_check == PduData._CRC) 
      {
        /* ������ִ�� */
				Ex_code = MB_Analyze_Execute();
        /* �����쳣 */
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
      /* ���±��Ϊ����״̬ */
      Rx_MSG = MSG_IDLE;
    }

//		if(KEY1_StateRead() == KEY_DOWN)
//		{
//			Position_Initial(SM1, MOTORSPEED); // ��ԭ��
//			Position_Initial(SM2, MOTORSPEED); // ��ԭ��
//		}
//		if(KEY2_StateRead() == KEY_DOWN)
//		{
//			STEPMOTOR_LSCMoveRel(SM1, fabs(STEP), ACCEL, DECEL, 120); 
//			STEPMOTOR_LSCMoveRel(SM2, fabs(STEP), ACCEL, DECEL, 120); 
//		}
//    
//		/* KEY3��KEY4���Ƶ��ת�� */
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
  * ��������: ���ڽ����жϻص�����
  * �������: ���ھ��
  * �� �� ֵ: ��
  * ˵    ��: ʹ��һ����ʱ���ıȽ��жϺ͸����ж���Ϊ���ճ�ʱ�ж�
  *           ֻҪ���յ����ݾͽ���ʱ����������0,�������Ƚ��жϵ�ʱ��
  *           ˵���Ѿ���ʱ1.5���ַ���ʱ��,�϶�Ϊ֡����,����Ǹ����ж�
  *           ����Ϊ�ǽ������
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &husart_debug)
  {
    switch(Rx_MSG)
    {
      /* ���յ���һ���ַ�֮��ʼ��ʱ1.5/3.5���ַ���ʱ�� */
      case MSG_IDLE:
        Rx_MSG = MSG_RXING;
        RxCount = 0;
        HAL_TIM_Base_Start(&htimx);
        break;
      
      /* ������һ�ν��յ������Ѿ�����1.5���ַ���ʱ����,�϶�Ϊ����֡������ */
      case MSG_ERR_FLAG:
        Rx_MSG = MSG_INC; // ����֡������
      break;
    }
    
    /* ʹ�ܼ������� */
    Rx_Buf[RxCount] = tmp_Rx_Buf;
    RxCount++;
    __HAL_TIM_SET_COUNTER(&htimx,0); // �������ʱ��
    HAL_UART_Receive_IT(&husart_debug,(uint8_t*)&tmp_Rx_Buf,1);
  }
}
/** 
  * ��������: ��ʱ���Ƚ��жϻص�����
  * �������: ��ʱ�����
  * �� �� ֵ: ��
  * ˵    ��: ����Ѿ���ʱ1.5���ַ���ʱ��û�н��յ�����
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* ����ǵ�һ�η����Ƚ��ж�,���ݶ�Ϊ������ */
  if(Rx_MSG != MSG_INC)
    Rx_MSG = MSG_ERR_FLAG;
  
  /* ����ǵڶ��ν���Ƚ��ж�,���϶�Ϊ���Ĳ����� */
  else
    Rx_MSG = MSG_INC;
}
/** 
  * ��������: ��ʱ�������жϻص�����
  * �������: ��ʱ�����
  * �� �� ֵ: ��
  * ˵    ��: ��ʱ3.5���ַ���ʱ��û�н��յ�����,��Ϊ�ǿ���״̬ 
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim7)
	{
		timer_count_delay++;
		if(KEY1_StateRead() == KEY_DOWN)
		{
			Position_Initial(SM1, MOTORSPEED); // ��ԭ��
			Position_Initial(SM2, MOTORSPEED); // ��ԭ��
		}
		if(KEY2_StateRead() == KEY_DOWN)
		{
			STEPMOTOR_LSCMoveRel(SM1, fabs(STEP), ACCEL, DECEL, 120); 
			STEPMOTOR_LSCMoveRel(SM2, fabs(STEP), ACCEL, DECEL, 120); 
		}   
		/* KEY3��KEY4���Ƶ��ת�� */
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
		/* ����Ѿ�����˽��յ�������������֡,��������Ϊ������������֡ */
		if(Rx_MSG == MSG_INC)
		{
			Rx_MSG = MSG_INC;
		}
		/* �����������ʱ������� */
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
		//�ź�ʱ�䴦��
		for(uint8_t i=0;i<SIG_NUM_MAX;i++)
		{
			SIG_SetLevelFromNum(&Signal[i], i);
			SIG_RunTime(&Signal[i]);
		}
		//��ʱʱ�䴦��
		for(uint8_t i=0;i<TMR_NUM_MAX;i++)
		{
			TMR_RunTime(&Timer[i]);
		}
	}
}

/**
  * ��������: �ع�ԭ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ���ݵ�ǰλ���жϻ�ԭ���ʱ��ķ���ͼ�¼����λ��
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
    
	/* ���ݵ�ǰ����λ�þ�����ԭ��ķ��� */
	if(HAL_GPIO_ReadPin(LIMIT_NEG_GPIO, NEG_Pin) != NEG_ACTIVE_LEVEL )
	{ 
		/* �����Ƿ���ԭ�㷶Χ�� */
		if(HAL_GPIO_ReadPin(LIMIT_ORI_GPIO, ORI_Pin) != ORI_ACTIVE_LEVEL)
		{
			step = INT32_MIN;      //  ��ת
			Initial_P[SMx] = ORI_INITIAL_1_3;
		}
		else
		{
			step = INT32_MAX;      //  ��ת
			Initial_P[SMx] = ORI_INITIAL_2_4;
		}
	}
	/* ���ڸ�����λ����������ع�ԭ��ķ�������ת */
	else
	{
		step = INT32_MAX;      //  ��ת
		Initial_P[SMx] = ORI_INITIAL_2_4;
	}
  if((SMx == SM2) || (SMx == SM4))//��Һ��ԭ�㣬Ĭ����[-5.5,12.7mm]��������
  {
      /* �������ԭ���λ�� */
      if(HAL_GPIO_ReadPin(LIMIT_ORI_GPIO, ORI_Pin) != ORI_ACTIVE_LEVEL)
      {
          step = INT32_MIN;      //  ��ת
          Initial_P[SMx] = ORI_INITIALP_1;
      }
      else
      {
          step = INT32_MAX;      //  ��ת
          Initial_P[SMx] = ORI_INITIALP_2_3;
      }
  }
STEPMOTOR_LSCMoveRel(SMx, step, mrd_SMotor[SMx].Acc_Time, mrd_SMotor[SMx].Dec_Time, speed); 
	OriginFlag[SMx] = true;
}

/**
  * ��������: ��λ�����жϻص�����
  * �������: GPIO_Pin ��λ���������ӵ�����
  * �� �� ֵ: ��
  * ˵    ��: ��λ���ؿ��Ƶ��ֹͣ
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if( NotAllow2Move(SM1) )
	{
		Stop_MotorMoving(SM1);// ����λ��ֹͣת��
	}
	if( NotAllow2Move(SM2) )
	{
		Stop_MotorMoving(SM2);// ����λ��ֹͣת��
	}
	if( NotAllow2Move(SM3) )
	{
		Stop_MotorMoving(SM3);// ����λ��ֹͣת��
	}
	if( NotAllow2Move(SM4) )
	{
		Stop_MotorMoving(SM4);// ����λ��ֹͣת��
	}
	/* ����ԭ�� */
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
  * ��������: ��еԭ�����
  * �������: SMx�������ţ�GPIO_Pin����λ��������
  * �� �� ֵ: ��
  * ˵    ��: ��ԭ������У���λ���ض�����
  */
void Position0(uint16_t SMx, uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
    case LIMIT1_ORI_PIN:
		if(SMx != SM1) return;
		/* ����DOGǰ�ˣ��̶�Ϊ���� */
		if(HAL_GPIO_ReadPin(LIMIT1_ORI_GPIO_PORT, GPIO_Pin) == LIMIT1_ORI_ACTIVE_LEVEL)
		{
			if(Initial_P[SMx] == ORI_INITIAL_1_3)  // ����
				Modify_MotorMoving(SMx, mrd_SMotor[SMx].Craml_Speed, INT32_MAX);
		}
		else/* ����DOG���*/
		{
			Stop_MotorMoving(SMx);// ������λ�ò���4�������ֱ��ֹͣ�����ԭ��
			if( Initial_P[SMx] != ORI_INITIAL_2_4) // �������λ��ʽ1,��ֱ��ֹͣ�����
			{
                OriginFlag[SMx] = false;
                PositionInitFlag[SMx] = true;
                mrd_SMotor[SMx].Coord = 0;//��������
			}
			else 
			{
				/* ����λ����2,4������DOG����򻻷���ԭ������ת�����ﻻ�ɷ�ת */
				Position_Initial(SMx, mrd_SMotor[SMx].Craml_Speed);
			}
		}
		break;
    case LIMIT1_NEG_PIN:
		if(SMx != SM1) return;
		/* ����λ����3,�������� */
		Position_Initial(SMx, mrd_SMotor[SMx].Lim_Speed);
		break;
    
    case LIMIT2_ORI_PIN:
		if(SMx != SM2) return;
		/* ������ԭ�㣬�̶�Ϊ�����ٶ� */
		if(HAL_GPIO_ReadPin(LIMIT2_ORI_GPIO_PORT, GPIO_Pin) == LIMIT2_ORI_ACTIVE_LEVEL)
		{
			if(Initial_P[SMx] == ORI_INITIALP_1)  // ����ԭ���ԭ������ת����Ϊ��ת
      {
        Stop_MotorMoving(SMx);
        TrendFlag[SMx] = true;
        Position_Initial(SMx, mrd_SMotor[SMx].Craml_Speed);
      }
		}
		else/* ������ԭ��*/
		{
			if(Initial_P[SMx] == ORI_INITIALP_2_3)  
      {
        Stop_MotorMoving(SMx);
        if(TrendFlag[SMx]) //�������λ��������λ��,��ֱ��ֹͣ�����
        {
          // ������λ������λ�ã������ֱ��ֹͣ�����ԭ��
          OriginFlag[SMx] = false;
          PositionInitFlag[SMx] = true;
          TrendFlag[SMx] = false;
          mrd_SMotor[SMx].Coord = 0;//��������
        }
        else // ����ԭ���ԭ������ת����Ϊ��ת
        {
          TrendFlag[SMx] = true;
          Position_Initial(SMx, mrd_SMotor[SMx].Craml_Speed);
        }
      }
		}
		break;
//    case LIMIT2_NEG_PIN:
//		if(SMx != SM2) return;
//		/* ����λ����3,�������� */
//		Position_Initial(SMx, mrd_SMotor[SMx].Lim_Speed);
//		break;
    case LIMIT3_ORI_PIN:
		if(SMx != SM3) return;
		/* ����DOGǰ�ˣ��̶�Ϊ���� */
		if(HAL_GPIO_ReadPin(LIMIT3_ORI_GPIO_PORT, GPIO_Pin) == LIMIT3_ORI_ACTIVE_LEVEL)
		{
			if(Initial_P[SMx] == ORI_INITIAL_1_3)  // ����
			Modify_MotorMoving(SMx, mrd_SMotor[SMx].Craml_Speed, INT32_MAX);
		}
		else/* ����DOG���*/
		{
			Stop_MotorMoving(SMx);// ������λ�ò���4�������ֱ��ֹͣ�����ԭ��
			if( Initial_P[SMx] != ORI_INITIAL_2_4) // �������λ��ʽ1,��ֱ��ֹͣ�����
			{
				OriginFlag[SMx] = false;
				PositionInitFlag[SMx] = true;
                mrd_SMotor[SMx].Coord = 0;//��������
			}
			else 
			{
				/* ����λ����2,4������DOG����򻻷���ԭ������ת�����ﻻ�ɷ�ת */
				Position_Initial(SMx, mrd_SMotor[SMx].Craml_Speed);
			}
		}
		break;
    case LIMIT3_NEG_PIN:
		if(SMx != SM3) return;
		/* ����λ����3,�������� */
		Position_Initial(SMx, mrd_SMotor[SMx].Lim_Speed);
		break;
    case LIMIT4_ORI_PIN:
		if(SMx != SM4) return;
		/* ������ԭ�㣬�̶�Ϊ�����ٶ� */
		if(HAL_GPIO_ReadPin(LIMIT4_ORI_GPIO_PORT, GPIO_Pin) == LIMIT4_ORI_ACTIVE_LEVEL)
		{
			if(Initial_P[SMx] == ORI_INITIALP_1)  // ����ԭ���ԭ������ת����Ϊ��ת
      {
        Stop_MotorMoving(SMx);
        TrendFlag[SMx] = true;
        Position_Initial(SMx, mrd_SMotor[SMx].Craml_Speed);
      }
		}
		else/* ������ԭ��*/
		{
			if(Initial_P[SMx] == ORI_INITIALP_2_3)  
      {
        Stop_MotorMoving(SMx);
        if(TrendFlag[SMx]) //�������λ��������λ��,��ֱ��ֹͣ�����
        {
          // ������λ������λ�ã������ֱ��ֹͣ�����ԭ��
          OriginFlag[SMx] = false;
          PositionInitFlag[SMx] = true;
          TrendFlag[SMx] = false;
          mrd_SMotor[SMx].Coord = 0;//��������
        }
        else // ����ԭ���ԭ������ת����Ϊ��ת
        {
          TrendFlag[SMx] = true;
          Position_Initial(SMx, mrd_SMotor[SMx].Craml_Speed);
        }
      }
		}
		break;
//    case LIMIT4_NEG_PIN:
//		if(SMx != SM4) return;
//		/* ����λ����3,�������� */
//		Position_Initial(SMx, mrd_SMotor[SMx].Lim_Speed);
//		break;
	}
}

///**
//  * ��������: ������ģʽ�¶�ʱ���Ļص�����
//  * �������: htim����ʱ�����
//  * �� �� ֵ: ��
//  * ˵    ��: ��
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  timer_count++;
//  timer_count_motor++;
//}
/**
  * ��������: ������ʱ��Ӳ������ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
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
    /* ������ʱ������ʱ�ӽ��� */
    BASIC_TIM_RCC_CLK_DISABLE();

    /* �ر������ж� */
    HAL_NVIC_DisableIRQ(BASIC_TIM_IRQ);
  }

  if(htim_base->Instance==GENERAL_TIMx)
  {
    /* ������ʱ������ʱ�ӽ��� */
    GENERAL_TIM_RCC_CLK_DISABLE();

    /* �ر������ж� */
    HAL_NVIC_DisableIRQ(GENERAL_TIM_IRQ);
  }
} 
/**
  * ��������: �����жϷ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void MX_RS485_UARTx_IRQHandler(void)
{
	if(__HAL_USART_GET_FLAG(&huartx_RS485,USART_FLAG_RXNE)!= RESET) // �����жϣ����յ�����
	{
		uint8_t data;
		data=READ_REG(huartx_RS485.Instance->DR); // ��ȡ����
		if(RxCount1==0) // ��������½��յ�����֡���������ڿ����ж�
		{
			__HAL_UART_CLEAR_FLAG(&huartx_RS485,USART_FLAG_IDLE); // ��������жϱ�־
		  __HAL_UART_ENABLE_IT(&huartx_RS485,UART_IT_IDLE);     // ʹ�ܿ����ж�	    
		}
		if(RxCount1<RX_MAX_COUNT)    // �жϽ��ջ�����δ��
		{
			aRxBuffer[RxCount1]=data;  // ��������
			RxCount1++;                // ���ӽ����ֽ�������
		}
	}
	else	if(__HAL_USART_GET_FLAG(&huartx_RS485,USART_FLAG_IDLE)!= RESET) // ���ڿ����ж�
	{
		__HAL_UART_CLEAR_FLAG(&huartx_RS485,USART_FLAG_IDLE); // ��������жϱ�־
		__HAL_UART_DISABLE_IT(&huartx_RS485,UART_IT_IDLE);    // �رտ����ж�
		Frame_flag=1;		                                 // ����֡��λ����ʶ���յ�һ����������֡
	}
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/

