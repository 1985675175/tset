/**
  ******************************************************************************
  * @file    bsp_spiflash.h
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
	 
#ifndef __BSP_SPIFLASH_H__
#define __BSP_SPIFLASH_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
//#define  SPI_FLASH_ID                       0xEF3015     //W25X16   2MB
//#define  SPI_FLASH_ID                       0xEF4015	   //W25Q16   4MB
//#define  SPI_FLASH_ID                       0XEF4017     //W25Q64   8MB
#define  SPI_FLASH_ID                       0XEF4018     //W25Q128  16MB YS-F4Pro开发默认使用

#define FLASH_SPIx                                 SPI2
#define FLASH_SPIx_RCC_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()
#define FLASH_SPIx_RCC_CLK_DISABLE()               __HAL_RCC_SPI2_CLK_DISABLE()

#define FLASH_SPI_SCK_GPIO_ClK_ENABLE()            __HAL_RCC_GPIOI_CLK_ENABLE() 
#define FLASH_SPI_SCK_GPIO_PORT                    GPIOI
#define FLASH_SPI_SCK_GPIO_PIN                     GPIO_PIN_1
#define FLASH_SPI_SCK_PIN_AF                       GPIO_AF5_SPI2 // 复用功能引脚

#define FLASH_SPI_MOSI_GPIO_ClK_ENABLE()           __HAL_RCC_GPIOI_CLK_ENABLE() 
#define FLASH_SPI_MOSI_GPIO_PORT                   GPIOI
#define FLASH_SPI_MOSI_GPIO_PIN                    GPIO_PIN_3
#define FLASH_SPI_MOSI_PIN_AF                      GPIO_AF5_SPI2 // 复用功能引脚

#define FLASH_SPI_CS_GPIO_ClK_ENABLE()             __HAL_RCC_GPIOI_CLK_ENABLE() 
#define FLASH_SPI_CS_GPIO_PORT                     GPIOI
#define FLASH_SPI_CS_GPIO_PIN                      GPIO_PIN_0
#define FLASH_SPI_CS_PIN_AF                    	   0 // 复用功能引脚
    
#define FLASH_SPI_MISO_GPIO_ClK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE() 
#define FLASH_SPI_MISO_GPIO_PORT                   GPIOC
#define FLASH_SPI_MISO_GPIO_PIN                    GPIO_PIN_2
#define FLASH_SPI_MISO_PIN_AF                      GPIO_AF5_SPI2 // 复用功能引脚

#define FLASH_SPI_CS_ENABLE()                      HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_PORT, FLASH_SPI_CS_GPIO_PIN, GPIO_PIN_RESET)
#define FLASH_SPI_CS_DISABLE()                     HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_PORT, FLASH_SPI_CS_GPIO_PIN, GPIO_PIN_SET)

/* 扩展变量 ------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspiflash;

/* 函数声明 ------------------------------------------------------------------*/

void MX_SPIFlash_Init(void);
void SPI_FLASH_SectorErase(uint32_t SectorAddr);
void SPI_FLASH_BulkErase(void);
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t SPI_FLASH_ReadID(void);
uint32_t SPI_FLASH_ReadDeviceID(void);
void SPI_FLASH_StartReadSequence(uint32_t ReadAddr);
void SPI_Flash_PowerDown(void);
void SPI_Flash_WAKEUP(void);

uint8_t SPI_FLASH_ReadByte(void);
uint8_t SPI_FLASH_SendByte(uint8_t byte);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WaitForWriteEnd(void);

#endif  /* __BSP_SPIFLASH_H__ */

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
