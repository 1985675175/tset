/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"


/* USER CODE BEGIN 0 */
#include "bsp_adc.h"
#include "bsp_debug_usart.h"
#include "bsp_GeneralTIM.h"
#include "bsp_stepmotor.h"
#include "bsp_BasicTIM.h" 
#include "bsp_led.h"
#include "bsp_limit.h"
#include "bsp_timer.h"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RCC global interrupt.
*/
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/


/* USER CODE BEGIN 1 */

void DEBUG_USART_IRQHANDLER(void)
{
  HAL_UART_IRQHandler(&husart_debug);
}
void GENERAL_TIM_INT_FUN(void)
{
  HAL_TIM_IRQHandler(&htimx);
}

void BASIC_TIM_INT_FUN(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htimx_led);
  /* USER CODE BEGIN TIM6_IRQn 1 */
  /* USER CODE END TIM6_IRQn 1 */
}
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}
/**
  * 函数功能: 步进电机定时器中断响应函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 处理中断事宜
  */
void SMOTOR1_TIM_IRQHandler(void)
{
//  HAL_TIM_IRQHandler(&htimx_SMotor[SM1]);
}
/**
  * 函数功能: 步进电机定时器DMA中断响应函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 处理中断事宜
  */
void DMAx_Streamz1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(htimx_SMotor[SM1].hdma[DMAx_TIMz1_ID]); 
}
void DMAx_Streamp1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(htimx_SMotor[SM2].hdma[DMAx_TIMp1_ID]); 
}
void DMAx_Streamz2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(htimx_SMotor[SM3].hdma[DMAx_TIMz2_ID]); 
}
void DMAx_Streamp2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(htimx_SMotor[SM4].hdma[DMAx_TIMp2_ID]); 
}
/**
  * 函数功能: 限位开关-原点中断响应函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 处理外部中断事宜
  */
void LIMIT1_ORI_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LIMIT1_ORI_PIN);
}
void LIMIT2_ORI_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LIMIT2_ORI_PIN);
}
/**
  * 函数功能: 限位开关-正向极限响应函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 处理外部中断事宜
  */
void LIMIT1_POS_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LIMIT1_POS_PIN);
}
/**
  * 函数功能: 限位开关-负向极限响应函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 处理外部中断事宜
  */
void LIMIT1_NEG_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LIMIT1_NEG_PIN);
}

void LIMIT2_NEG_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LIMIT2_NEG_PIN);
}

void LIMIT_n_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LIMIT2_POS_PIN);
  HAL_GPIO_EXTI_IRQHandler(LIMIT3_NEG_PIN);
  HAL_GPIO_EXTI_IRQHandler(LIMIT3_ORI_PIN);
  HAL_GPIO_EXTI_IRQHandler(LIMIT3_POS_PIN);
  HAL_GPIO_EXTI_IRQHandler(LIMIT4_NEG_PIN);
}

void LIMIT_m_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LIMIT4_ORI_PIN);
  HAL_GPIO_EXTI_IRQHandler(LIMIT4_POS_PIN);
}

/**
  * 函数功能: ADC温度DMA中断响应函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 处理外部中断事宜
  */
void ADC_DMA_IRQ_Handler(void)
{
  HAL_DMA_IRQHandler(&DMA_Init_Handle);
}

/**
  * 函数功能: ADC总线电压中断响应函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 处理外部中断事宜
  */
void ADC_VBUS_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&ADC_Handle);
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
