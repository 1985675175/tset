/**
  ******************************************************************************
  * @file    bsp_MB_slave.c
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

#include "main.h"
#include "bsp_MB_slave.h"
#include "bsp_debug_usart.h"
#include "bsp_GeneralTIM.h"
#include "bsp_led.h"
#include "bsp_stepmotor.h"
#include "bsp_beep.h"
#include "bsp_heat.h"
#include "bsp_lamp.h"
#include "bsp_signal.h"
#include "bsp_timer.h"
#include "bsp_limit.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
PDUData_TypeDef PduData;

// CRC 高位字节值表
static const uint8_t auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
// CRC 低位字节值表
static const uint8_t auchCRCLo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
	0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
	0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
	0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
	0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
	0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
	0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
	0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
	0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
	0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
	0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
	0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
static uint8_t MB_JudgeAddr(uint16_t _Addr,uint16_t _RegNum);
static uint8_t MB_JudgeNum(uint16_t _RegNum,uint8_t _FunCode,uint16_t _ByteNum);
static uint8_t MB_UP_NCoil(uint8_t *_AddrOffset,uint16_t _CoilNum, uint16_t _CoilAddr);
static uint8_t MB_UP_NReg(uint16_t* _AddrOffset,uint16_t _RegNum , uint16_t _RegAddr);
static uint8_t MB_WR_NCoil(uint8_t* _AddrOffset,uint16_t _RegNum , uint8_t* _Datebuf);
static uint8_t MB_WR_NReg(uint16_t* _AddrOffset,uint16_t _RegNum , uint8_t* _Datebuf);
static uint8_t MB_RSP_10H(uint16_t _TxCount,uint16_t _AddrOffset,uint16_t _ByteNum);
static uint16_t MB_RSP_01H(uint16_t _TxCount,uint8_t *_AddrOffset ,uint16_t _CoilNum);
static uint8_t MB_RSP_03H(uint16_t _TxCount,uint16_t *_AddrOffset,uint16_t _RegNum );
static uint8_t MB_RSP_05H(uint16_t _TxCount,uint16_t _AddrOffset ,uint8_t *_AddrAbs);
static uint8_t MB_RSP_06H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t *_AddrAbs);
static uint8_t MB_RSP_10H(uint16_t _TxCount,uint16_t _AddrOffset,uint16_t _ByteNum);
static uint16_t MB_CoilAddr2Offset(uint16_t CoilAddr,uint16_t CoilNum);
static uint16_t MB_RegAddr2Offset(uint16_t RegAddr,uint16_t RegNum);

/* 函数体 --------------------------------------------------------------------*/

/** 
  * 函数功能: Modbus CRC16 校验计算函数
  * 输入参数: pushMsg:待计算的数据首地址,usDataLen:数据长度
  * 返 回 值: CRC16 计算结果
  * 说    明: 计算结果是高位在前,需要转换才能发送
  */
uint16_t MB_CRC16(uint8_t *_pushMsg,uint8_t _usDataLen)
{
  uint8_t uchCRCHi = 0xFF;
  uint8_t uchCRCLo = 0xFF;
  uint16_t uIndex;
  while(_usDataLen--)
  {
    uIndex = uchCRCLo ^ *_pushMsg++;
    uchCRCLo = uchCRCHi^auchCRCHi[uIndex];
    uchCRCHi = auchCRCLo[uIndex];
  }
  return (uchCRCHi<<8|uchCRCLo);
}
/* 提取数据帧,进行解析数据帧 */
void MB_Parse_Data()
{
  PduData.Code = Rx_Buf[1];                   // 功能码
  PduData.Addr = ((Rx_Buf[2]<<8) | Rx_Buf[3]);// 寄存器起始地址
  PduData.Num  = ((Rx_Buf[4]<<8) | Rx_Buf[5]);// 数量(Coil,Input,Holding Reg,Input Reg)
  PduData._CRC = MB_CRC16((uint8_t*)&Rx_Buf,RxCount-2);             // CRC校验码
  PduData.byteNums = Rx_Buf[6];               // 获得字节数
  PduData.ValueReg = (uint8_t*)&Rx_Buf[7];                          // 寄存器值起始地址
//  PduData.PtrCoilOffset = PduData.PtrCoilbase + PduData.Addr;       // 离散量的内存起始地址
//  PduData.PtrHoldingOffset = PduData.PtrHoldingbase + PduData.Addr; // 保持寄存器的起始地址
//  PduData.PtrCoilOffset = PduData.PtrCoilbase;       // 离散量的内存起始地址  临时解决越界
//  PduData.PtrHoldingOffset = PduData.PtrHoldingbase; // 保持寄存器的起始地址  临时解决越界
  PduData.PtrCoilOffset = PduData.PtrCoilbase + MB_CoilAddr2Offset(PduData.Addr,PduData.Num);       // 离散量的内存起始地址  极度节省空间
  PduData.PtrHoldingOffset = PduData.PtrHoldingbase + MB_RegAddr2Offset(PduData.Addr,PduData.Num); // 保持寄存器的起始地址  极度节省空间
}

#define COIL_BASIC_ADDR_IDEX_MAX 6

#define COIL_BASIC_ADDR0 0
#define COIL_CELL_SIZE0 (7)

#define COIL_BASIC_ADDR1 100
#define COIL_CELL_SIZE1 (16)

#define COIL_BASIC_ADDR2 132
#define COIL_CELL_SIZE2 (16)

#define COIL_BASIC_ADDR3 200
#define COIL_CELL_SIZE3 (16)

#define COIL_BASIC_ADDR4 300
#define COIL_CELL_SIZE4 (1)

#define COIL_BASIC_ADDR5 400
#define COIL_CELL_SIZE5 (18)

#define COIL_BASIC_ADDR6 500
#define COIL_CELL_SIZE6 (72)

//线圈基地址与数据长度数组
uint16_t CoilArray[][2] = 
{
  {COIL_BASIC_ADDR0,COIL_CELL_SIZE0},
  {COIL_BASIC_ADDR1,COIL_CELL_SIZE1},
  {COIL_BASIC_ADDR2,COIL_CELL_SIZE2},
  {COIL_BASIC_ADDR3,COIL_CELL_SIZE3},
  {COIL_BASIC_ADDR4,COIL_CELL_SIZE4},
  {COIL_BASIC_ADDR5,COIL_CELL_SIZE5},
  {COIL_BASIC_ADDR6,COIL_CELL_SIZE6},
};
/* 线圈地址转成实际偏移量 */
uint16_t MB_CoilAddr2Offset(uint16_t CoilAddr,uint16_t CoilNum)
{
  uint16_t Offset = 0;
  uint16_t ContinueOffset = 0;//连续偏移,跨越区间
  uint16_t CoilBasicAddr;
  uint16_t BasicAddrIdexMin = 0;//最小地址和实际基地址对应索引
  uint16_t BasicAddrIdexMax = COIL_BASIC_ADDR_IDEX_MAX;
  uint16_t i;
  //线圈基地址部分
  if(CoilArray[BasicAddrIdexMax][0] < CoilAddr)
  {
    if((CoilArray[BasicAddrIdexMax][0]+CoilArray[BasicAddrIdexMax][1]) > CoilAddr)
    {
      CoilBasicAddr = CoilArray[BasicAddrIdexMax][0];
      BasicAddrIdexMin = BasicAddrIdexMax;
    }
    else
    {
      Offset = 0xffff;//地址超出范围，设为0xffff
      return Offset;
    }
  }
  else if(CoilArray[BasicAddrIdexMin+1][0] > CoilAddr)
    CoilBasicAddr = CoilArray[BasicAddrIdexMin][0];
  else 
  {
    while(1)
    {
      i = (BasicAddrIdexMin + BasicAddrIdexMax)/2;
      if(CoilArray[i][0] <= CoilAddr)
        BasicAddrIdexMin = i;
      else if(CoilArray[i][0] >= CoilAddr)
        BasicAddrIdexMax = i;
      if(BasicAddrIdexMin == (BasicAddrIdexMin + BasicAddrIdexMax)/2)         
      {
        CoilBasicAddr = CoilArray[BasicAddrIdexMin][0];
        break;
      }
    }
  }
  //寄存器偏移部分
  Offset = CoilAddr - CoilBasicAddr;
  //确定连续区间大小,超出后，寄存器越界
  ContinueOffset += CoilArray[BasicAddrIdexMin][1] - 1;
  for(;BasicAddrIdexMin+1<=COIL_BASIC_ADDR_IDEX_MAX;)
  {
    if((CoilArray[BasicAddrIdexMin][0]+CoilArray[BasicAddrIdexMin][1]) == CoilArray[BasicAddrIdexMin+1][0])
      ContinueOffset += CoilArray[BasicAddrIdexMin+1][1];
    else
      break;
    BasicAddrIdexMin++;
  }
  if((Offset+CoilNum-1) > ContinueOffset)
  {
    Offset = 0xffff;//地址超出范围，设为0xffff
    return Offset;
  }
  //实际寄存器基地址部分
  switch(CoilBasicAddr)
  {
    case COIL_BASIC_ADDR6:
      Offset += (COIL_CELL_SIZE5 + COIL_CELL_SIZE4 + COIL_CELL_SIZE3 + COIL_CELL_SIZE2 + COIL_CELL_SIZE1 + COIL_CELL_SIZE0);
      break;
    case COIL_BASIC_ADDR5:
      Offset += (COIL_CELL_SIZE4 + COIL_CELL_SIZE3 + COIL_CELL_SIZE2 + COIL_CELL_SIZE1 + COIL_CELL_SIZE0);
      break;
    case COIL_BASIC_ADDR4:
      Offset += (COIL_CELL_SIZE3 + COIL_CELL_SIZE2 + COIL_CELL_SIZE1 + COIL_CELL_SIZE0);
      break;
    case COIL_BASIC_ADDR3:
      Offset += (COIL_CELL_SIZE2 + COIL_CELL_SIZE1 + COIL_CELL_SIZE0);
      break;
    case COIL_BASIC_ADDR2:
      Offset += (COIL_CELL_SIZE1 + COIL_CELL_SIZE0);
      break;
    case COIL_BASIC_ADDR1:
      Offset += (COIL_CELL_SIZE0);
      break;
    case COIL_BASIC_ADDR0:
      Offset += 0;
      break;
  }
  return Offset;
}

#define REG_BASIC_ADDR_IDEX_MAX 8

#define REG_BASIC_ADDR0 0
#define REG_CELL_SIZE0 (14)

#define REG_BASIC_ADDR1 100
#define REG_CELL_SIZE1 (12)

#define REG_BASIC_ADDR2 200
#define REG_CELL_SIZE2 (12)

#define REG_BASIC_ADDR3 300
#define REG_CELL_SIZE3 (12)

#define REG_BASIC_ADDR4 400
#define REG_CELL_SIZE4 (60)

#define REG_BASIC_ADDR5 1000
#define REG_CELL_SIZE5 (70)

#define REG_BASIC_ADDR6 2000
#define REG_CELL_SIZE6 (32)

#define REG_BASIC_ADDR7 2100
#define REG_CELL_SIZE7 (36)

#define REG_BASIC_ADDR8 9000
#define REG_CELL_SIZE8 (10)

//寄存器地址与数据长度数组
uint16_t RegArray[][2] = 
{
  {REG_BASIC_ADDR0,REG_CELL_SIZE0},
  {REG_BASIC_ADDR1,REG_CELL_SIZE1},
  {REG_BASIC_ADDR2,REG_CELL_SIZE2},
  {REG_BASIC_ADDR3,REG_CELL_SIZE3},
  {REG_BASIC_ADDR4,REG_CELL_SIZE4},
  {REG_BASIC_ADDR5,REG_CELL_SIZE5},
  {REG_BASIC_ADDR6,REG_CELL_SIZE6},
  {REG_BASIC_ADDR7,REG_CELL_SIZE7},
  {REG_BASIC_ADDR8,REG_CELL_SIZE8},
};
/* 寄存器地址转成实际偏移量 */
uint16_t MB_RegAddr2Offset(uint16_t RegAddr,uint16_t RegNum)
{
  uint16_t ContinueOffset = 0;//连续偏移,跨越区间
  uint16_t Offset = 0;
  uint16_t RegBasicAddr;
  uint16_t BasicAddrIdexMin = 0;//最小地址和实际基地址对应索引
  uint16_t BasicAddrIdexMax = REG_BASIC_ADDR_IDEX_MAX;
  uint16_t i;
  //寄存器基地址部分
  if(RegArray[BasicAddrIdexMax][0] < RegAddr)
  {
    if((RegArray[BasicAddrIdexMax][0]+RegArray[BasicAddrIdexMax][1]) > RegAddr)
    {
      RegBasicAddr = RegArray[BasicAddrIdexMax][0];
      BasicAddrIdexMin = BasicAddrIdexMax;
    }
    else
    {
      Offset = 0xffff;//地址超出范围，设为0xffff
      return Offset;
    }
  }
  else if(RegArray[BasicAddrIdexMin+1][0] > RegAddr)
    RegBasicAddr = RegArray[BasicAddrIdexMin][0];
  else 
  {
    while(1)
    {
      i = (BasicAddrIdexMin + BasicAddrIdexMax)/2;
      if(RegArray[i][0] <= RegAddr)
        BasicAddrIdexMin = i;
      else if(RegArray[i][0] >= RegAddr)
        BasicAddrIdexMax = i;
      if(BasicAddrIdexMin == (BasicAddrIdexMin + BasicAddrIdexMax)/2)         
      {
        RegBasicAddr = RegArray[BasicAddrIdexMin][0];
        break;
      }
    }
  }
  //寄存器偏移部分
  Offset = RegAddr - RegBasicAddr;
  //确定连续区间大小,超出后，寄存器越界
  ContinueOffset += RegArray[BasicAddrIdexMin][1] - 1;
  for(;BasicAddrIdexMin+1<=REG_BASIC_ADDR_IDEX_MAX;)
  {
    if((RegArray[BasicAddrIdexMin][0]+RegArray[BasicAddrIdexMin][1]) == RegArray[BasicAddrIdexMin+1][0])
      ContinueOffset += RegArray[BasicAddrIdexMin+1][1];
    else
      break;
    BasicAddrIdexMin++;
  }
  if((Offset+RegNum-1) > ContinueOffset)
  {
    Offset = 0xffff;//地址超出范围，设为0xffff
    return Offset;
  }
  //实际寄存器基地址部分
  switch(RegBasicAddr)
  {
    case REG_BASIC_ADDR8:
      Offset += (REG_CELL_SIZE7 + REG_CELL_SIZE6 + REG_CELL_SIZE5 + REG_CELL_SIZE4 + REG_CELL_SIZE3 + REG_CELL_SIZE2 + REG_CELL_SIZE1 + REG_CELL_SIZE0);
      break;
    case REG_BASIC_ADDR7:
      Offset += (REG_CELL_SIZE6 + REG_CELL_SIZE5 + REG_CELL_SIZE4 + REG_CELL_SIZE3 + REG_CELL_SIZE2 + REG_CELL_SIZE1 + REG_CELL_SIZE0);
      break;
    case REG_BASIC_ADDR6:
      Offset += (REG_CELL_SIZE5 + REG_CELL_SIZE4 + REG_CELL_SIZE3 + REG_CELL_SIZE2 + REG_CELL_SIZE1 + REG_CELL_SIZE0);
      break;
    case REG_BASIC_ADDR5:
      Offset += (REG_CELL_SIZE4 + REG_CELL_SIZE3 + REG_CELL_SIZE2 + REG_CELL_SIZE1 + REG_CELL_SIZE0);
      break;
    case REG_BASIC_ADDR4:
      Offset += (REG_CELL_SIZE3 + REG_CELL_SIZE2 + REG_CELL_SIZE1 + REG_CELL_SIZE0);
      break;
    case REG_BASIC_ADDR3:
      Offset += (REG_CELL_SIZE2 + REG_CELL_SIZE1 + REG_CELL_SIZE0);
      break;
    case REG_BASIC_ADDR2:
      Offset += (REG_CELL_SIZE1 + REG_CELL_SIZE0);
      break;
    case REG_BASIC_ADDR1:
      Offset += (REG_CELL_SIZE0);
      break;
    case REG_BASIC_ADDR0:
      Offset += 0;
      break;
  }
  return Offset;
}

/** 
  * 函数功能: 对接收到的数据进行分析并执行
  * 输入参数: 无
  * 返 回 值: 异常码或0x00
  * 说    明: 判断功能码,验证地址是否正确.数值内容是否溢出,数据没错误就发送响应信号
  */
uint8_t MB_Analyze_Execute(void )
{
  uint16_t ExCode = EX_CODE_NONE;
  /* 校验功能码 */
  if( IS_NOT_FUNCODE(PduData.Code) ) // 不支持的功能码
  {
    /* Modbus异常响应 */
    ExCode = EX_CODE_01H;            // 异常码01H
    return ExCode;
  }
  /* 根据功能码分别做判断 */
  switch(PduData.Code)
  {
    /* 这里认为01H功能码和02功能码是一样的,其实也没什么不一样
     * 只是操作地址可能不一样,这一点结合具体来实现,可以在main函数
     * 申请单独的内存使用不同的功能码,在实际应用中必须加以区分使用
     * 不同的内存空间
     */
/* ---- 01H  02H 读取离散量输入(Coil Input)---------------------- */
    case FUN_CODE_01H:
    case FUN_CODE_02H:
      /* 判断线圈数量是否正确 */  
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,1);
      if(ExCode != EX_CODE_NONE )
        return ExCode;      
      
      /* 判断地址是否正确*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      
      /* 判断数据更新是否正确*/
      ExCode = MB_UP_NCoil((uint8_t *)PduData.PtrCoilOffset, PduData.Num, PduData.Addr);
      if(ExCode != EX_CODE_NONE )
        return ExCode;
      break;
/* ---- 03H  04H 读取保持/输入寄存器---------------------- */
    case FUN_CODE_03H:
    case FUN_CODE_04H:
      /* 判断寄存器数量是否正确 */
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,PduData.byteNums);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      
      /* 判断地址是否正确*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
     
      /* 判断数据更新是否正确*/
      ExCode = MB_UP_NReg((uint16_t *)PduData.PtrHoldingOffset, PduData.Num, PduData.Addr);
      if(ExCode != EX_CODE_NONE )
        return ExCode;
      break;
/* ---- 05H 写入单个离散量---------------------- */
    case FUN_CODE_05H:
      /* 输出值!=OK */
      if((PduData.Num != 0x0000) && PduData.Num != 0xFF00)
      {
        ExCode = EX_CODE_03H;
        return ExCode;        
      }
      /* 写入一个线圈值 */
      if(PduData.Num == 0xFF00)
      { 
        *PduData.PtrCoilOffset = 1;
        /* 读取写入值,验证是否写入成功 */
        if( *PduData.PtrCoilOffset != 1)
        {
          ExCode = EX_CODE_04H;
          return ExCode;
        }
      }
      else 
      {
        *PduData.PtrCoilOffset = 0;
        /* 读取写入值,验证是否写入成功 */
        if( *PduData.PtrCoilOffset != 0)
        { 
          ExCode = EX_CODE_04H;
          return ExCode;
        }
      }
      break;
/* ---- 06H 写单个保持寄存器 ---------------------- */
    case FUN_CODE_06H:
      
      /* 写入寄存器值*/
      *PduData.PtrHoldingOffset = PduData.Num;
      /* 验证写成功 */
      if(*PduData.PtrHoldingOffset != PduData.Num)
      {
        ExCode = EX_CODE_04H;
         return ExCode;
      }
      break;
/* ---- 0FH 写多个线圈 ---------------------- */
    case FUN_CODE_0FH:
      /* 判断寄存器数量是否正确 */
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,PduData.byteNums);
      if(ExCode != EX_CODE_NONE )
        return ExCode;    
      
      /* 判断地址是否正确*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      
      /* 写入多个线圈 */
      ExCode = MB_WR_NCoil((uint8_t*)PduData.PtrCoilOffset,PduData.Num,(uint8_t*)PduData.ValueReg);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      break;
/* ---- 10H 写多个保持寄存器 ---------------------- */
    case FUN_CODE_10H:
      /* 判断寄存器数量是否正确 */
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,PduData.byteNums);
      if(ExCode != EX_CODE_NONE )
        return ExCode;    
      
      /* 判断地址是否正确*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      
      /* 写入多个寄存器 */
      ExCode = MB_WR_NReg((uint16_t*)PduData.PtrHoldingOffset,PduData.Num,(uint8_t*)PduData.ValueReg);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      break;
  }
  /* 数据帧没有异常 */
  return ExCode; //   EX_CODE_NONE
}
/**
  * 函数功能: 生效线圈数值
	* 输入参数: _CoilAddr:线圈地址, _CoilNum:线圈数量, _CoilData:线圈数据, _CoilIdex：线圈索引
  * 返 回 值: 异常码:0或其他
  * 说    明: 根据线圈地址和数量，生效线圈数值
  */
uint8_t MB_EF_CoilVal(uint16_t _CoilAddr, uint16_t _CoilNum, uint8_t *_CoilData, uint16_t *_CoilIdex)
{
//  uint16_t i = 0;
  uint8_t Error = 0;

//	可根据_CoilNum来优化读取（暂未使用），目前处理为能小不大
	switch(_CoilAddr + *_CoilIdex)
	{
		case 0://机器运行灯
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				LED1_ON();
			else
				LED1_OFF();
			*_CoilIdex += 1;//索引标号增加
			break;
		case 1://电机1运行灯
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				LED2_ON();
			else
				LED2_OFF();
			*_CoilIdex += 1;//索引标号增加
			break;
		case 2://电机2运行灯
		case 3://电机3运行灯
		case 4://电机4运行灯
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				LED3_ON();
			else
				LED3_OFF();
			*_CoilIdex += 1;//索引标号增加
			break;
		case 5://照明灯
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				LAMP_ON();
			else
				LAMP_OFF();
			*_CoilIdex += 1;//索引标号增加
			break;
		case 6://蜂鸣器报警
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				BEEP_ON();
			else
				BEEP_OFF();
			*_CoilIdex += 1;//索引标号增加
			break;
		case 100://加热管1
		case 101://加热管2
		case 102://加热管3
		case 103://加热管4
		case 104://加热管5
		case 105://加热管6
		case 106://加热管7
		case 107://加热管8
		case 108://加热管9
		case 109://加热管10
		case 110://加热管11
		case 111://加热管12
		case 112://加热管13
		case 113://加热管14
		case 114://加热管15
		case 115://加热管16
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HEAT_ON();
			else
				HEAT_OFF();
			*_CoilIdex += 1;//索引标号增加
			break;
		case 132://制冷管1
		case 133://制冷管2
		case 134://制冷管3
		case 135://制冷管4
		case 136://制冷管5
		case 137://制冷管6
		case 138://制冷管7
		case 139://制冷管8
		case 140://制冷管9
		case 141://制冷管10
		case 142://制冷管11
		case 143://制冷管12
		case 144://制冷管13
		case 145://制冷管14
		case 146://制冷管15
		case 147://制冷管16
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HEAT_ON();//临时用解热替代
			else
				HEAT_OFF();
			*_CoilIdex += 1;//索引标号增加
			break;
		case 500://X轴电机EN
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR1_ENA_GPIO_PORT, SMOTOR1_ENA_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_ENA_GPIO_PORT, SMOTOR1_ENA_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 501://X轴电机STEP
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR1_PUL_GPIO_PORT, SMOTOR1_PUL_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_PUL_GPIO_PORT, SMOTOR1_PUL_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 502://X轴电机DIR
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 503://X轴电机MD0
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 504://X轴电机MD1
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 505://X轴电机MD2
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 506://X轴电机DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if((*_CoilData == 1)&&(*(_CoilData+1) == 1))//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else if((*_CoilData == 0)&&(*(_CoilData+1) == 0))
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			else//配置为悬浮状态
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 2;//索引标号增加
		case 508://X轴电机RESET
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 509://X轴电机SLEEP
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 512://Y轴电机EN
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR2_ENA_GPIO_PORT, SMOTOR2_ENA_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_ENA_GPIO_PORT, SMOTOR2_ENA_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 513://Y轴电机STEP
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR2_PUL_GPIO_PORT, SMOTOR2_PUL_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_PUL_GPIO_PORT, SMOTOR2_PUL_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 514://Y轴电机DIR
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR2_DIR_GPIO_PORT, SMOTOR2_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_DIR_GPIO_PORT, SMOTOR2_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 515://Y轴电机MD0
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 516://Y轴电机MD1
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 517://Y轴电机MD2
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 518://Y轴电机DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if((*_CoilData == 1)&&(*(_CoilData+1) == 1))//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else if((*_CoilData == 0)&&(*(_CoilData+1) == 0))
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			else//配置为悬浮状态
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 2;//索引标号增加
		case 520://Y轴电机RESET
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 521://Y轴电机SLEEP
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 524://Z1轴电机EN
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR1_ENA_GPIO_PORT, SMOTOR1_ENA_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_ENA_GPIO_PORT, SMOTOR1_ENA_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 525://Z1轴电机STEP
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR1_PUL_GPIO_PORT, SMOTOR1_PUL_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_PUL_GPIO_PORT, SMOTOR1_PUL_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 526://Z1轴电机DIR
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 527://Z1轴电机MD0
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 528://Z1轴电机MD1
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 529://Z1轴电机MD2
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 530://Z1轴电机DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if((*_CoilData == 1)&&(*(_CoilData+1) == 1))//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else if((*_CoilData == 0)&&(*(_CoilData+1) == 0))
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			else//配置为悬浮状态
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 2;//索引标号增加
		case 532://Z1轴电机RESET
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 533://Z1轴电机SLEEP
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 536://P1轴电机EN
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR2_ENA_GPIO_PORT, SMOTOR2_ENA_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_ENA_GPIO_PORT, SMOTOR2_ENA_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 537://P1轴电机STEP
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR2_PUL_GPIO_PORT, SMOTOR2_PUL_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_PUL_GPIO_PORT, SMOTOR2_PUL_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 538://P1轴电机DIR
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR2_DIR_GPIO_PORT, SMOTOR2_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_DIR_GPIO_PORT, SMOTOR2_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 539://P1轴电机MD0
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 540://P1轴电机MD1
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 541://P1轴电机MD2
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 542://P1轴电机DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if((*_CoilData == 1)&&(*(_CoilData+1) == 1))//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else if((*_CoilData == 0)&&(*(_CoilData+1) == 0))
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			else//配置为悬浮状态
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 2;//索引标号增加
		case 544://P1轴电机RESET
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 545://P1轴电机SLEEP
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 548://Z2轴电机EN
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR1_ENA_GPIO_PORT, SMOTOR1_ENA_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_ENA_GPIO_PORT, SMOTOR1_ENA_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 549://Z2轴电机STEP
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR1_PUL_GPIO_PORT, SMOTOR1_PUL_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_PUL_GPIO_PORT, SMOTOR1_PUL_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 550://Z2轴电机DIR
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 551://Z2轴电机MD0
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 552://Z2轴电机MD1
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 553://Z2轴电机MD2
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 554://Z2轴电机DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if((*_CoilData == 1)&&(*(_CoilData+1) == 1))//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else if((*_CoilData == 0)&&(*(_CoilData+1) == 0))
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			else//配置为悬浮状态
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 2;//索引标号增加
		case 556://Z2轴电机RESET
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 557://Z2轴电机SLEEP
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 560://P2轴电机EN
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR2_ENA_GPIO_PORT, SMOTOR2_ENA_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_ENA_GPIO_PORT, SMOTOR2_ENA_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 561://P2轴电机STEP
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR2_PUL_GPIO_PORT, SMOTOR2_PUL_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_PUL_GPIO_PORT, SMOTOR2_PUL_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 562://P2轴电机DIR
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				HAL_GPIO_WritePin(SMOTOR2_DIR_GPIO_PORT, SMOTOR2_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_DIR_GPIO_PORT, SMOTOR2_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 563://P2轴电机MD0
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 564://P2轴电机MD1
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 565://P2轴电机MD2
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 566://P2轴电机DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if((*_CoilData == 1)&&(*(_CoilData+1) == 1))//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else if((*_CoilData == 0)&&(*(_CoilData+1) == 0))
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			else//配置为悬浮状态
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 2;//索引标号增加
		case 568://P2轴电机RESET
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		case 569://P2轴电机SLEEP
			if(*_CoilIdex+1 > _CoilNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//数据地址调整
			if(*_CoilData == 1)//根据相应状态赋值
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//索引标号增加
			break;
		default:
			Error = 1;
			break;
	}
  return Error;
}
/**
  * 函数功能: 生效N个线圈数值
  * 输入参数: _CoilData:线圈数据,_CoilNum:线圈数量,_CoilAddr:线圈地址
  * 返 回 值: 异常码:0或其他
  * 说    明: 生效_CoilNum个线圈数据，如生效异常则立即返回
  */
uint8_t MB_EF_NCoil(uint16_t _CoilAddr, uint16_t _CoilNum, uint8_t* _CoilData)
{
  uint8_t Error = 1;
  uint16_t i = 0;
  for(i=0;i<_CoilNum;)
  {
	Error = MB_EF_CoilVal(_CoilAddr, _CoilNum, _CoilData, &i);
	if(Error)//生效异常，则退出
      break;
  }
  return Error;
}
/**
  * 函数功能: 05H功能码线圈生效函数
  * 输入参数: _CoilAddr线圈地址, _CoilData:线圈数据
  * 返 回 值: 异常码:0或其他
  * 说    明: 使线圈中的数值生效
  */
uint8_t MB_Effect_05H(uint16_t _CoilAddr, uint8_t* _CoilData)
{
	return MB_EF_NCoil(_CoilAddr, 1, _CoilData);
}

/**
  * 函数功能: 0FH功能码线圈生效函数
  * 输入参数: _CoilAddr线圈地址, _CoilData:线圈数据, _CoilData:线圈数据
  * 返 回 值: 异常码:0或其他
  * 说    明: 使线圈中的数值生效
  */
uint8_t MB_Effect_0FH(uint16_t _CoilAddr, uint16_t _CoilNum, uint8_t* _CoilData)
{
	return MB_EF_NCoil(_CoilAddr, _CoilNum, _CoilData);
}

/**
  * 函数功能: 生效寄存器数值
	* 输入参数: _RegAddr:寄存器地址, _RegNum:寄存器数量, _RegData:寄存器数据, _RegIdex：寄存器索引
  * 返 回 值: 异常码:0或其他
  * 说    明: 根据寄存器地址、数量和寄存器数据，生效线圈数值
  */
uint8_t MB_EF_RegVal(uint16_t _RegAddr, uint16_t _RegNum, uint16_t *_RegData, uint16_t *_RegIdex)
{
//  uint16_t i = 0;
  uint8_t Error = 0;
  uint16_t Temp16;//作数据改变临时使用
  uint32_t Temp32;
  uint32_t TempFloat1,TempFloat2;

//	可根据_RegNum来优化读取（暂未使用），目前处理为能小不大
	switch(_RegAddr + *_RegIdex)
	{
		case 2://回归原点
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
            OriginCmdLast = Temp32;//保存原点控制命令
//			if(Temp32&0x00000001)//X轴
//				Position_Initial(SM1, mrd_SMotor[SM1].Lim_Speed); // 回原点
//			if(Temp32&0x00000002)//Y轴
//				Position_Initial(SM2, mrd_SMotor[SM2].Lim_Speed); // 回原点
			if(Temp32&0x00000004)//Z1轴
				Position_Initial(SM1, mrd_SMotor[SMZ1].Lim_Speed); // 回原点
			if(Temp32&0x00000008)//P1轴
				Position_Initial(SM2, mrd_SMotor[SMP1].Lim_Speed); // 回原点
			if(Temp32&0x00000010)//Z2轴
				Position_Initial(SM3, mrd_SMotor[SMZ2].Lim_Speed); // 回原点
			if(Temp32&0x00000020)//P2轴
				Position_Initial(SM4, mrd_SMotor[SMP2].Lim_Speed); // 回原点
			*_RegIdex += 2;//索引标号增加
			break;
		case 4://延时等待
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			TMR_SetInit(&Timer[TMR_NUM_DELAY], 1, 1, Temp32);//延时等待函数执行
      TMR_Reset(&Timer[TMR_NUM_DELAY]);
      TMR_SetTrigger(&Timer[TMR_NUM_DELAY], 1);
      *_RegIdex += 2;//索引标号增加
			break;
		case 8://工作状态控制
			if(*_RegIdex+1 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp16 = *_RegData;
			switch(Temp16&0x0003)
			{
				case 0://默认值
					break;
				case 1://停止
					break;
				case 2://暂停
					break;
				case 3://继续
					break;
			}
			*_RegIdex += 1;//索引标号增加
			break;
		case 200://X轴绝对坐标
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
            STEPMOTOR_LSCMoveRel(SM1, ((float) Temp32  - mrd_SMotor[SM1].Coord)/mrd_SMotor[SM1].Distance_Rev*(360.0f/mrd_SMotor[SM1].Step_Angle*mrd_SMotor[SM1].Micro_Step), mrd_SMotor[SM1].Acc_Time, mrd_SMotor[SM1].Dec_Time, mrd_SMotor[SM1].Lim_Speed);
			mrd_SMotor[SM1].Coord = (float) Temp32;//绝对坐标函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 202://Y轴绝对坐标
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
            STEPMOTOR_LSCMoveRel(SM2, ((float) Temp32  - mrd_SMotor[SM2].Coord)/mrd_SMotor[SM2].Distance_Rev*(360.0f/mrd_SMotor[SM2].Step_Angle*mrd_SMotor[SM2].Micro_Step), mrd_SMotor[SM2].Acc_Time, mrd_SMotor[SM2].Dec_Time, mrd_SMotor[SM2].Lim_Speed);
			mrd_SMotor[SM2].Coord = (float) Temp32;//绝对坐标函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 204://Z1轴绝对坐标
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
            STEPMOTOR_LSCMoveRel(SMZ1, (*((float *)(&Temp32)) - mrd_SMotor[SMZ1].Coord)/mrd_SMotor[SMZ1].Distance_Rev*(360.0f/mrd_SMotor[SMZ1].Step_Angle*mrd_SMotor[SMZ1].Micro_Step), mrd_SMotor[SMZ1].Acc_Time, mrd_SMotor[SMZ1].Dec_Time, mrd_SMotor[SMZ1].Lim_Speed);
			mrd_SMotor[SMZ1].Coord = *((float *)(&Temp32));//绝对坐标函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 206://P1轴绝对坐标
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
      //对移动距离做出限制
      TempFloat1 = *((float *)(&Temp32));
      if(TempFloat1 > prd_SMotor[PIP1].Max_Stroke) //最大值
        TempFloat1 = prd_SMotor[PIP1].Max_Stroke;
      else if(TempFloat1 < -prd_SMotor[PIP1].Unload_Distance) //最小值
        TempFloat1 = -prd_SMotor[PIP1].Unload_Distance;
      TempFloat2 = mrd_SMotor[SMP1].Distance_Rev*(360.0f/mrd_SMotor[SMP1].Step_Angle*mrd_SMotor[SMP1].Micro_Step);//每步移动距离
      STEPMOTOR_LSCMoveRel(SMP1, (TempFloat1 - mrd_SMotor[SMP1].Coord)/TempFloat2, mrd_SMotor[SMP1].Acc_Time, mrd_SMotor[SMP1].Dec_Time, mrd_SMotor[SMP1].Lim_Speed);
			mrd_SMotor[SMP1].Coord = TempFloat1;//绝对坐标函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 208://Z2轴绝对坐标
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
      _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
      STEPMOTOR_LSCMoveRel(SMZ2, (*((float *)(&Temp32)) - mrd_SMotor[SMZ2].Coord)/mrd_SMotor[SMZ2].Distance_Rev*(360.0f/mrd_SMotor[SMZ2].Step_Angle*mrd_SMotor[SMZ2].Micro_Step), mrd_SMotor[SMZ2].Acc_Time, mrd_SMotor[SMZ2].Dec_Time, mrd_SMotor[SMZ2].Lim_Speed);
			mrd_SMotor[SMZ2].Coord = *((float *)(&Temp32));//绝对坐标函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 210://P2轴绝对坐标
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
      //对移动距离做出限制
      TempFloat1 = *((float *)(&Temp32));
      if(TempFloat1 > prd_SMotor[PIP2].Max_Stroke) //最大值
        TempFloat1 = prd_SMotor[PIP2].Max_Stroke;
      else if(TempFloat1 < -prd_SMotor[PIP2].Unload_Distance) //最小值
        TempFloat1 = -prd_SMotor[PIP2].Unload_Distance;
      TempFloat2 = mrd_SMotor[SMP2].Distance_Rev*(360.0f/mrd_SMotor[SMP2].Step_Angle*mrd_SMotor[SMP2].Micro_Step);//每步移动距离
      STEPMOTOR_LSCMoveRel(SMP2, (TempFloat1 - mrd_SMotor[SMP2].Coord)/TempFloat2, mrd_SMotor[SMP2].Acc_Time, mrd_SMotor[SMP2].Dec_Time, mrd_SMotor[SMP2].Lim_Speed);
			mrd_SMotor[SMP2].Coord = TempFloat1;//绝对坐标函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 300://X轴相对坐标
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM1].Coord += (float) Temp32;//相对坐标函数执行
            STEPMOTOR_LSCMoveRel(SM1, (float) Temp32/mrd_SMotor[SM1].Distance_Rev*(360.0f/mrd_SMotor[SM1].Step_Angle*mrd_SMotor[SM1].Micro_Step), mrd_SMotor[SM1].Acc_Time, mrd_SMotor[SM1].Dec_Time, mrd_SMotor[SM1].Lim_Speed);
//            STEPMOTOR_LSCMoveRel(SM2, (float) Temp32/mrd_SMotor[SM2].Distance_Rev*(360.0f/mrd_SMotor[SM2].Step_Angle*mrd_SMotor[SM2].Micro_Step), mrd_SMotor[SM2].Acc_Time, mrd_SMotor[SM2].Dec_Time, mrd_SMotor[SM2].Lim_Speed);
			*_RegIdex += 2;//索引标号增加
			break;
		case 302://Y轴相对坐标
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM2].Coord += (float) Temp32;//相对坐标函数执行
            STEPMOTOR_LSCMoveRel(SM2, (float) Temp32/mrd_SMotor[SM2].Distance_Rev*(360.0f/mrd_SMotor[SM2].Step_Angle*mrd_SMotor[SM2].Micro_Step), mrd_SMotor[SM2].Acc_Time, mrd_SMotor[SM2].Dec_Time, mrd_SMotor[SM2].Lim_Speed);
			*_RegIdex += 2;//索引标号增加
			break;
		case 304://Z1轴相对坐标
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ1].Coord += *((float *)(&Temp32));//相对坐标函数执行
            STEPMOTOR_LSCMoveRel(SMZ1, *((float *)(&Temp32))/mrd_SMotor[SMZ1].Distance_Rev*(360.0f/mrd_SMotor[SMZ1].Step_Angle*mrd_SMotor[SMZ1].Micro_Step), mrd_SMotor[SMZ1].Acc_Time, mrd_SMotor[SMZ1].Dec_Time, mrd_SMotor[SMZ1].Lim_Speed);
			*_RegIdex += 2;//索引标号增加
			break;
		case 306://P1轴相对坐标
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Coord += *((float *)(&Temp32));//相对坐标函数执行
            STEPMOTOR_LSCMoveRel(SMP1, *((float *)(&Temp32))/mrd_SMotor[SMP1].Distance_Rev*(360.0f/mrd_SMotor[SMP1].Step_Angle*mrd_SMotor[SMP1].Micro_Step), mrd_SMotor[SMP1].Acc_Time, mrd_SMotor[SMP1].Dec_Time, mrd_SMotor[SMP1].Lim_Speed);
			*_RegIdex += 2;//索引标号增加
			break;
		case 308://Z2轴相对坐标
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ2].Coord += *((float *)(&Temp32));//相对坐标函数执行
            STEPMOTOR_LSCMoveRel(SMZ2, *((float *)(&Temp32))/mrd_SMotor[SMZ2].Distance_Rev*(360.0f/mrd_SMotor[SMZ2].Step_Angle*mrd_SMotor[SMZ2].Micro_Step), mrd_SMotor[SMZ2].Acc_Time, mrd_SMotor[SMZ2].Dec_Time, mrd_SMotor[SMZ2].Lim_Speed);
			*_RegIdex += 2;//索引标号增加
			break;
		case 310://P2轴相对坐标
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Coord += *((float *)(&Temp32));//相对坐标函数执行
            STEPMOTOR_LSCMoveRel(SMP2, *((float *)(&Temp32))/mrd_SMotor[SMP2].Distance_Rev*(360.0f/mrd_SMotor[SMP2].Step_Angle*mrd_SMotor[SMP2].Micro_Step), mrd_SMotor[SMP2].Acc_Time, mrd_SMotor[SMP2].Dec_Time, mrd_SMotor[SMP2].Lim_Speed);
			*_RegIdex += 2;//索引标号增加
			break;
		case 400://X轴最高速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM1].Lim_Speed = (float) Temp32;//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 402://X轴爬行速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM1].Craml_Speed = (float) Temp32;//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 404://X轴加速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM1].Acc_Time = (float) Temp32;//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 406://X轴减速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM1].Dec_Time = (float) Temp32;//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 408://X轴每圈距离   默认不可改，由机器决定 
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM1].Distance_Rev = (float) Temp32;//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 410://Y轴最高速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM2].Lim_Speed = (float) Temp32;//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 412://Y轴爬行速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM2].Craml_Speed = (float) Temp32;//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 414://Y轴加速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM2].Acc_Time = (float) Temp32;//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 416://Y轴减速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM2].Dec_Time = (float) Temp32;//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 418://Y轴每圈距离   默认不可改，由机器决定 
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM2].Distance_Rev = (float) Temp32;//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 420://Z1轴最高速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ1].Lim_Speed = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 422://Z1轴爬行速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ1].Craml_Speed = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 424://Z1轴加速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ1].Acc_Time = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 426://Z1轴减速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ1].Dec_Time = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 428://Z1轴每圈距离   默认不可改，由机器决定 
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ1].Distance_Rev = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 430://P1轴最高速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Lim_Speed = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 432://P1轴爬行速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Craml_Speed = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 434://P1轴加速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Acc_Time = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 436://P1轴减速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Dec_Time = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 438://P1轴每圈距离   默认不可改，由机器决定 
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Distance_Rev = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 440://Z2轴最高速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ2].Lim_Speed = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 442://Z2轴爬行速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ2].Craml_Speed = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 444://Z2轴加速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ2].Acc_Time = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 446://Z2轴减速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ2].Dec_Time = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 448://Z2轴每圈距离   默认不可改，由机器决定 
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ2].Distance_Rev = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 450://P2轴最高速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Lim_Speed = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 452://P2轴爬行速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Craml_Speed = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 454://P2轴加速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Acc_Time = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 456://P2轴减速度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Dec_Time = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 458://P2轴每圈距离   默认不可改，由机器决定 
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Distance_Rev = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 1002://装针管 
			if(*_RegIdex+1 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
      _RegData += *_RegIdex;//数据地址调整
			Temp16 = *_RegData;//参数设置函数执行
			if(Temp16&0x0001)//针管1
      {
        LoadFlag[SMZ1] = true;
        STEPMOTOR_LSCMoveRel(SMZ1, INT32_MAX, mrd_SMotor[SMZ1].Acc_Time, mrd_SMotor[SMZ1].Dec_Time, mrd_SMotor[SMZ1].Lim_Speed); 
      }
			if(Temp16&0x0002)//针管2
      {
        LoadFlag[SMZ2] = true;
        STEPMOTOR_LSCMoveRel(SMZ2, INT32_MAX, mrd_SMotor[SMZ2].Acc_Time, mrd_SMotor[SMZ2].Dec_Time, mrd_SMotor[SMZ2].Lim_Speed); 
      }
			*_RegIdex += 1;//索引标号增加
			break;
		case 1004://卸针管 
			if(*_RegIdex+1 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
      _RegData += *_RegIdex;//数据地址调整
			Temp16 = *_RegData;//参数设置函数执行
			if(Temp16&0x0001)//针管1
      {
        UnloadFlag[SMP1] = true;
        STEPMOTOR_LSCMoveRel(SMP1, (-prd_SMotor[PIP1].Unload_Distance)/mrd_SMotor[SMP1].Distance_Rev*mrd_SMotor[SMP1].Micro_Step*360/mrd_SMotor[SMP1].Step_Angle, mrd_SMotor[SMP1].Acc_Time, mrd_SMotor[SMP1].Dec_Time, mrd_SMotor[SMP1].Lim_Speed); 
      }
			if(Temp16&0x0002)//针管2
      {
        UnloadFlag[SMP2] = true;
        STEPMOTOR_LSCMoveRel(SMP2, (-prd_SMotor[PIP2].Unload_Distance)/mrd_SMotor[SMP2].Distance_Rev*mrd_SMotor[SMP2].Micro_Step*360/mrd_SMotor[SMP2].Step_Angle, mrd_SMotor[SMP2].Acc_Time, mrd_SMotor[SMP2].Dec_Time, mrd_SMotor[SMP2].Lim_Speed); 
      }
			*_RegIdex += 1;//索引标号增加
			break;
		case 1006://P1轴体积距离比
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
      _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			prd_SMotor[PIP1].Volume_Distance = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 1008://P2轴体积距离比
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
      _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			prd_SMotor[PIP2].Volume_Distance = *((float *)(&Temp32));//参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 1038://P1轴液体体积 
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Coord += *((float *)(&Temp32))/prd_SMotor[PIP1].Volume_Distance;//相对坐标函数执行
      STEPMOTOR_LSCMoveRel(SMP1, *((float *)(&Temp32))/prd_SMotor[PIP1].Volume_Distance/mrd_SMotor[SMP1].Distance_Rev*(360.0f/mrd_SMotor[SMP1].Step_Angle*mrd_SMotor[SMP1].Micro_Step), mrd_SMotor[SMP1].Acc_Time, mrd_SMotor[SMP1].Dec_Time, mrd_SMotor[SMP1].Craml_Speed);
			*_RegIdex += 2;//索引标号增加
			break;
		case 1040://P2轴液体体积 
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Coord += *((float *)(&Temp32))/prd_SMotor[PIP2].Volume_Distance;//相对坐标函数执行
      STEPMOTOR_LSCMoveRel(SMP2, *((float *)(&Temp32))/prd_SMotor[PIP2].Volume_Distance/mrd_SMotor[SMP2].Distance_Rev*(360.0f/mrd_SMotor[SMP2].Step_Angle*mrd_SMotor[SMP2].Micro_Step), mrd_SMotor[SMP2].Acc_Time, mrd_SMotor[SMP2].Dec_Time, mrd_SMotor[SMP2].Craml_Speed);
			*_RegIdex += 2;//索引标号增加
			break;
		case 2100://温控取消
			if(*_RegIdex+1 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp16 = *_RegData;
            //参数设置函数执行
			if(Temp16&0x0001)//目标1
                ;//
			if(Temp16&0x0002)//目标2
                ;//
			*_RegIdex += 1;//索引标号增加
			break;
		case 2101://温控保持
			if(*_RegIdex+1 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp16 = *_RegData;
            //参数设置函数执行
			if(Temp16&0x0001)//目标1
                ;//
			if(Temp16&0x0002)//目标2
                ;//
			*_RegIdex += 1;//索引标号增加
			break;
		case 2102://温控加热
			if(*_RegIdex+1 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp16 = *_RegData;
            //参数设置函数执行
			if(Temp16&0x0001)//目标1
                ;//
			if(Temp16&0x0002)//目标2
                ;//
			*_RegIdex += 1;//索引标号增加
			break;
		case 2103://温控制冷
			if(*_RegIdex+1 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp16 = *_RegData;
            //参数设置函数执行
			if(Temp16&0x0001)//目标1
                ;//
			if(Temp16&0x0002)//目标2
                ;//
			*_RegIdex += 1;//索引标号增加
			break;
		case 2104://设置目标1温度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			(float) Temp32;//参数设置函数执行
            //参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		case 2106://设置目标2温度
		case 2108://设置目标3温度
		case 2110://设置目标4温度
		case 2112://设置目标5温度
		case 2114://设置目标6温度
		case 2116://设置目标7温度
		case 2118://设置目标8温度
		case 2120://设置目标9温度
		case 2122://设置目标10温度
		case 2124://设置目标11温度
		case 2126://设置目标12温度
		case 2128://设置目标13温度
		case 2130://设置目标14温度
		case 2132://设置目标15温度
		case 2134://设置目标16温度
			if(*_RegIdex+2 > _RegNum)//生效数据大于存储空间，参数出错
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//数据地址调整
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			(float) Temp32;//参数设置函数执行
            //参数设置函数执行
			*_RegIdex += 2;//索引标号增加
			break;
		default:
			Error = 1;
			break;
	}
  return Error;
}
/**
  * 函数功能: 生效N个寄存器数值
  * 输入参数: _AddrOffset:偏移地址,_RegNum:寄存器数量,_RegAddr:寄存器地址
  * 返 回 值: 异常码:0或其他
  * 说    明: 生效_RegNum个线圈数据，如生效异常则立即返回
  */
uint8_t MB_EF_NReg(uint16_t* _AddrOffset,uint16_t _RegNum , uint16_t _RegAddr)
{
  uint8_t Error = 1;
  uint16_t i;
  for(i=0;i<_RegNum;)
  {
    Error = MB_EF_RegVal(_RegAddr, _RegNum, _AddrOffset, &i);
    if(Error)//生效异常，则退出
        break;
  }
  return Error;
}
/**
  * 函数功能: 06H功能码寄存器生效函数
  * 输入参数: _RegAddr寄存器地址, _AddrOffset寄存器数值存储地址
  * 返 回 值: 异常码:0或其他
  * 说    明: 使寄存器中的数值生效
  */
uint8_t MB_Effect_06H(uint16_t _RegAddr, uint16_t* _AddrOffset)
{
	return MB_EF_NReg(_AddrOffset, 1, _RegAddr);
}

/**
  * 函数功能: 10H功能码寄存器生效函数
  * 输入参数: _RegAddr寄存器地址, _RegNum寄存器数量, _AddrOffset寄存器数值存储地址
  * 返 回 值: 异常码:0或其他
  * 说    明: 使寄存器中的数值生效
  */
uint8_t MB_Effect_10H(uint16_t _RegAddr, uint16_t _RegNum, uint16_t* _AddrOffset)
{
	return MB_EF_NReg(_AddrOffset, _RegNum, _RegAddr);
}

/**
  * 函数功能: ModBus离散量和寄存器生效函数
  * 输入参数: 协议数据单元指针
  * 返 回 值: 异常码:0或其他
  * 说    明: 寄存器中的数值生效
  */
uint8_t MB_Effect(PDUData_TypeDef *_PduData)
{
	uint8_t Error;
    switch(_PduData->Code)
	{
		case FUN_CODE_01H:
			//读相关的后续操作，可在这里实现
            Error = 0;
			break;
		case FUN_CODE_02H:
			//读相关的后续操作，可在这里实现
            Error = 0;
			break;
		case FUN_CODE_03H:
			//读相关的后续操作，可在这里实现
            Error = 0;
			break;
		case FUN_CODE_04H:
			//读相关的后续操作，可在这里实现
            Error = 0;
			break;
		case FUN_CODE_05H:
			//写相关的后续操作，可在这里实现
			Error = MB_Effect_05H(_PduData->Addr,(uint8_t *) _PduData->PtrCoilOffset);
			break;
		case FUN_CODE_06H:
			//写相关的后续操作，可在这里实现
			Error = MB_Effect_06H(_PduData->Addr, (uint16_t *) _PduData->PtrHoldingOffset);
			break;
		case FUN_CODE_0FH:
			//写相关的后续操作，可在这里实现
			Error = MB_Effect_0FH(_PduData->Addr, _PduData->Num,(uint8_t *) _PduData->PtrCoilOffset);
			break;
		case FUN_CODE_10H:
			//写相关的后续操作，可在这里实现
			Error = MB_Effect_10H(_PduData->Addr, _PduData->Num,(uint16_t *) _PduData->PtrHoldingOffset);
			break;
        default:
            Error = 1;
            break;
	}
    return Error;
}

/**
  * 函数功能: 生成线圈数值
	* 输入参数: _CoilAddr:线圈地址, _CoilNum:线圈数量, _CoilBuf:线圈缓冲指针, _CoilIdex：线圈索引指针, _CellCoilNum: 单元格内线圈数量
  * 返 回 值: 异常码:04H或NONE
  * 说    明: 根据线圈地址和数量，生成线圈数值，并往_CoilBuf所指向的空间里写入
  */
uint8_t MB_MK_CoilVal(uint16_t _CoilAddr, uint16_t _CoilNum, uint8_t *_CoilBuf, uint16_t *_CoilIdex, uint16_t *_CellCoilNum)
{
//  uint16_t i = 0;
  uint8_t Value = 0;

//	可根据_CoilNum来优化读取（暂未使用），目前处理为能小不大
	switch(_CoilAddr + *_CoilIdex)
	{
		case 0://机器运行灯
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 1://电机1运行灯
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 2://电机2运行灯
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 3://电机3运行灯
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 4://电机4运行灯
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 5://照明灯
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			//根据相应状态赋值
            if(HAL_GPIO_ReadPin(LAMP_GPIO_PORT,LAMP_GPIO_PIN))
                *_CoilBuf = 1;
            else
                *_CoilBuf = 0;
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 6://蜂鸣器报警
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			//根据相应状态赋值
            if(HAL_GPIO_ReadPin(BEEP_GPIO_PORT,BEEP_GPIO_PIN))
                *_CoilBuf = 1;
            else
                *_CoilBuf = 0;
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 100://加热管1
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 101://加热管2
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 102://加热管3
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 103://加热管4
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 104://加热管5
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 105://加热管6
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 106://加热管7
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 107://加热管8
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 108://加热管9
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 109://加热管10
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 110://加热管11
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 111://加热管12
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 112://加热管13
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 113://加热管14
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 114://加热管15
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 115://加热管16
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 132://制冷管1
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 133://制冷管2
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 134://制冷管3
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 135://制冷管4
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 136://制冷管5
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 137://制冷管6
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 138://制冷管7
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 139://制冷管8
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 140://制冷管9
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 141://制冷管10
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 142://制冷管11
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 143://制冷管12
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 144://制冷管13
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 145://制冷管14
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 146://制冷管15
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 147://制冷管16
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;		
		case 200://液槽1
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			//根据相应状态赋值
            if(HAL_GPIO_ReadPin(SIG_SLOT_GPIO_PORT,SIG_SLOT_GPIO_PIN))
                *_CoilBuf = 1;
            else
                *_CoilBuf = 0;
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 201://液槽2
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 202://液槽3
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 203://液槽4
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 204://液槽5
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 205://液槽6
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 206://液槽7
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 207://液槽8
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 208://液槽9
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 209://液槽10
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 210://液槽11
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 211://液槽12
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 212://液槽13
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 213://液槽14
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 214://液槽15
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 215://液槽16
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 300://门锁
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			//根据相应状态赋值
            if(HAL_GPIO_ReadPin(SIG_DOOR_GPIO_PORT,SIG_DOOR_GPIO_PIN))
                *_CoilBuf = 1;
            else
                *_CoilBuf = 0;
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 400://X轴位置NEG
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 401://X轴位置ORI
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 402://X轴位置POS
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 403://Y轴位置NEG
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 404://Y轴位置ORI
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 405://Y轴位置POS
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 406://Z1轴位置NEG
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 407://Z1轴位置ORI
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 408://Z1轴位置POS
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 409://P1轴位置NEG
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 410://P1轴位置ORI
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 411://P1轴位置POS
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 412://Z2轴位置NEG
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 413://Z2轴位置ORI
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 414://Z2轴位置POS
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 415://P2轴位置NEG
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 416://P2轴位置ORI
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 417://P2轴位置POS
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 500://X轴电机EN
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 501://X轴电机STEP
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 502://X轴电机DIR
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 503://X轴电机MD0
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 504://X轴电机MD1
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 505://X轴电机MD2
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 506://X轴电机DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_CoilBuf+0) = 0;//根据相应状态赋值
			*(_CoilBuf+1) = 0;//根据相应状态赋值
			*_CoilIdex += 2;//索引标号增加
			*_CellCoilNum = 2;//单元格内线圈数量
			break;
		case 508://X轴电机RESET
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 509://X轴电机SLEEP
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 510://X轴电机FAULT
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 511://X轴电机HOME
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 512://Y轴电机EN
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 513://Y轴电机STEP
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 514://Y轴电机DIR
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 515://Y轴电机MD0
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 516://Y轴电机MD1
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 517://Y轴电机MD2
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 518://Y轴电机DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_CoilBuf+0) = 0;//根据相应状态赋值
			*(_CoilBuf+1) = 0;//根据相应状态赋值
			*_CoilIdex += 2;//索引标号增加
			*_CellCoilNum = 2;//单元格内线圈数量
			break;
		case 520://Y轴电机RESET
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 521://Y轴电机SLEEP
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 522://Y轴电机FAULT
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 523://Y轴电机HOME
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 524://Z1轴电机EN
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 525://Z1轴电机STEP
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 526://Z1轴电机DIR
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 527://Z1轴电机MD0
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 528://Z1轴电机MD1
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 529://Z1轴电机MD2
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 530://Z1轴电机DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_CoilBuf+0) = 0;//根据相应状态赋值
			*(_CoilBuf+1) = 0;//根据相应状态赋值
			*_CoilIdex += 2;//索引标号增加
			*_CellCoilNum = 2;//单元格内线圈数量
			break;
		case 532://Z1轴电机RESET
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 533://Z1轴电机SLEEP
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 534://Z1轴电机FAULT
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 535://Z1轴电机HOME
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 536://P1轴电机EN
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 537://P1轴电机STEP
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 538://P1轴电机DIR
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 539://P1轴电机MD0
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 540://P1轴电机MD1
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 541://P1轴电机MD2
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 542://P1轴电机DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_CoilBuf+0) = 0;//根据相应状态赋值
			*(_CoilBuf+1) = 0;//根据相应状态赋值
			*_CoilIdex += 2;//索引标号增加
			*_CellCoilNum = 2;//单元格内线圈数量
			break;
		case 544://P1轴电机RESET
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 545://P1轴电机SLEEP
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 546://P1轴电机FAULT
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 547://P1轴电机HOME
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 548://Z2轴电机EN
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 549://Z2轴电机STEP
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 550://Z2轴电机DIR
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 551://Z2轴电机MD0
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 552://Z2轴电机MD1
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 553://Z2轴电机MD2
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 554://Z2轴电机DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_CoilBuf+0) = 0;//根据相应状态赋值
			*(_CoilBuf+1) = 0;//根据相应状态赋值
			*_CoilIdex += 2;//索引标号增加
			*_CellCoilNum = 2;//单元格内线圈数量
			break;
		case 556://Z2轴电机RESET
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 557://Z2轴电机SLEEP
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 558://Z2轴电机FAULT
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 559://Z2轴电机HOME
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 560://P2轴电机EN
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 561://P2轴电机STEP
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 562://P2轴电机DIR
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 563://P2轴电机MD0
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 564://P2轴电机MD1
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 565://P2轴电机MD2
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 566://P2轴电机DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_CoilBuf+0) = 0;//根据相应状态赋值
			*(_CoilBuf+1) = 0;//根据相应状态赋值
			*_CoilIdex += 2;//索引标号增加
			*_CellCoilNum = 2;//单元格内线圈数量
			break;
		case 568://P2轴电机RESET
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 569://P2轴电机SLEEP
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 570://P2轴电机FAULT
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		case 571://P2轴电机HOME
			if(*_CoilIdex+1 > _CoilNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//根据相应状态赋值
			*_CoilIdex += 1;//索引标号增加
			*_CellCoilNum = 1;//单元格内线圈数量
			break;
		default:
			Value = 1;
			break;
	}
  return Value;
}
/**
  * 函数功能: 更新N个线圈或输入
  * 输入参数: _AddrOffset:偏移地址,_CoilNum:线圈数量,_CoilAddr:线圈地址
  * 返 回 值: 异常码:04H或NONE
  * 说    明: 生成_CoilNum个线圈数据，并向_AddrOffset所指向的空间里写入，如生成异常则立即返回
  */
uint8_t MB_UP_NCoil(uint8_t* _AddrOffset,uint16_t _CoilNum , uint16_t _CoilAddr)
{
  uint16_t i = 0,j = 0;
	uint16_t CellCoilNum;//单元格内线圈数量，可作优化，暂未使用
  uint8_t CoilBuf[128];//临时一个字节对应一个离散量
  if((_AddrOffset - PduData.PtrCoilbase)==0xffff)//线圈地址和数量超出实际定义的范围
    return EX_CODE_04H;
  for(i=0;i<_CoilNum;)
  {
		if(MB_MK_CoilVal(_CoilAddr, _CoilNum, CoilBuf, &i, &CellCoilNum))//生成异常，则退出
      return EX_CODE_04H;
    for(j=0;j<CellCoilNum;j++)
			*_AddrOffset++ = CoilBuf[j];
  }
  return EX_CODE_NONE;
}
/**
  * 函数功能: 生成寄存器数值
	* 输入参数: _RegAddr:寄存器地址, _RegNum:寄存器数量, _RegBuf:寄存器缓冲指针, _RegIdex：寄存器索引指针, _CellRegNum: 单元格内寄存器数量
  * 返 回 值: 异常码:04H或NONE
  * 说    明: 根据寄存器地址和数量，生成寄存器数值，并往_RegBuf所指向的空间里写入
  */
uint8_t MB_MK_RegVal(uint16_t _RegAddr, uint16_t _RegNum, uint16_t *_RegBuf, uint16_t *_RegIdex, uint16_t *_CellRegNum)
{
  uint16_t i = 0;
  uint8_t Value = 0;
    uint32_t RegTemp;

//	可根据_RegNum来优化读取（暂未使用），目前处理为能小不大
	switch(_RegAddr + *_RegIdex)
	{
		case 0://原点已校准
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			for(i=0;i<2;i++)//一个单元格最多4个寄存器
				*(_RegBuf+i) = 0;
//			if(PositionInitFlag[SMX])
//				*_RegBuf |= (uint16_t)1 << 0;
//			if(PositionInitFlag[SMY])
//				*_RegBuf |= (uint16_t)1 << 1;
			if(PositionInitFlag[SMZ1])
				*_RegBuf |= (uint16_t)1 << 2;
			if(PositionInitFlag[SMP1])
				*_RegBuf |= (uint16_t)1 << 3;
			if(PositionInitFlag[SMZ2])
				*_RegBuf |= (uint16_t)1 << 4;
			if(PositionInitFlag[SMP2])
				*_RegBuf |= (uint16_t)1 << 5;
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2://回归原点
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
            *_RegBuf = OriginCmdLast%65536;//相应变量赋值
            *(_RegBuf+1) = OriginCmdLast/65536;//相应变量赋值
            *_RegIdex += 2;//索引标号加1
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 4://延时等待
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
            *_RegBuf = Timer[TMR_NUM_DELAY].value%65536;//相应变量赋值
            *(_RegBuf+1) = Timer[TMR_NUM_DELAY].value/65536;//相应变量赋值
            *_RegIdex += 2;//索引标号加1
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 6://延时等待剩余
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			if(!TMR_Getleave(&Timer[TMR_NUM_DELAY], &RegTemp))
            {
                *_RegBuf = RegTemp%65536;//相应变量赋值
                *(_RegBuf+1) = RegTemp/65536;//相应变量赋值
            }
            else
            {
               *_RegBuf = 0xFFFF;//该值不可能达到，为异常值 
               *(_RegBuf+1) = 0xFFFF;//该值不可能达到，为异常值 
            }
			*_RegIdex += 2;//索引标号加1
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 8://工作状态控制
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 10://指令执行状态
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 12://电机运行状态
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = 0;
			*(_RegBuf+1) = 0;
//            if(SMotor[SMX].MotorRuning)// 当前正处于转动状态
//				*_RegBuf |= ((uint16_t) 1)<<0;//相应变量赋值
//			if(SMotor[SMY].MotorRuning)// 当前正处于转动状态
//				*_RegBuf |= ((uint16_t) 1)<<1;//相应变量赋值
            if(SMotor[SMZ1].MotorRuning)// 当前正处于转动状态
				*_RegBuf |= ((uint16_t) 1)<<2;//相应变量赋值
			if(SMotor[SMP1].MotorRuning)// 当前正处于转动状态
				*_RegBuf |= ((uint16_t) 1)<<3;//相应变量赋值
            if(SMotor[SMZ2].MotorRuning)// 当前正处于转动状态
				*_RegBuf |= ((uint16_t) 1)<<4;//相应变量赋值
			if(SMotor[SMP2].MotorRuning)// 当前正处于转动状态
				*_RegBuf |= ((uint16_t) 1)<<5;//相应变量赋值
			*_RegIdex += 2;//索引标号加1
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 100://X轴坐标
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM1].Coord%65536;//轴坐标低半字
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM1].Coord/65536;//轴坐标高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 102://Y轴坐标
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM2].Coord%65536;//轴坐标低半字
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM2].Coord/65536;//轴坐标高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 104://Z1轴坐标
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ1].Coord)%65536;//轴坐标低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ1].Coord)/65536;//轴坐标高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 106://P1轴坐标
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP1].Coord)%65536;//轴坐标低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP1].Coord)/65536;//轴坐标高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 108://Z2轴坐标
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ2].Coord)%65536;//轴坐标低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ2].Coord)/65536;//轴坐标高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 110://P2轴坐标
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP2].Coord)%65536;//轴坐标低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP2].Coord)/65536;//轴坐标高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 400://X轴最高速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM1].Lim_Speed%65536;//数据低半字
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM1].Lim_Speed/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 402://X轴爬行速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM1].Craml_Speed%65536;//数据低半字
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM1].Craml_Speed/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 404://X轴加速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM1].Acc_Time%65536;//数据低半字
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM1].Acc_Time/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 406://X轴减速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM1].Dec_Time%65536;//数据低半字
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM1].Dec_Time/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 408://X轴每圈距离
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM1].Distance_Rev%65536;//数据低半字
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM1].Distance_Rev/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 410://Y轴最高速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM2].Lim_Speed%65536;//数据低半字
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM2].Lim_Speed/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 412://Y轴爬行速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM2].Craml_Speed%65536;//数据低半字
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM2].Craml_Speed/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 414://Y轴加速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM2].Acc_Time%65536;//数据低半字
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM2].Acc_Time/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 416://Y轴减速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM2].Dec_Time%65536;//数据低半字
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM2].Dec_Time/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 418://Y轴每圈距离
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM2].Distance_Rev%65536;//数据低半字
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM2].Distance_Rev/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 420://Z1轴最高速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ1].Lim_Speed)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ1].Lim_Speed)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 422://Z1轴爬行速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ1].Craml_Speed)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ1].Craml_Speed)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 424://Z1轴加速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ1].Acc_Time)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ1].Acc_Time)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 426://Z1轴减速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ1].Dec_Time)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ1].Dec_Time)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 428://Z1轴每圈距离
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ1].Distance_Rev)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ1].Distance_Rev)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 430://P1轴最高速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP1].Lim_Speed)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP1].Lim_Speed)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 432://P1轴爬行速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP1].Craml_Speed)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP1].Craml_Speed)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 434://P1轴加速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP1].Acc_Time)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP1].Acc_Time)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 436://P1轴减速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP1].Dec_Time)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP1].Dec_Time)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 438://P1轴每圈距离
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP1].Distance_Rev)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP1].Distance_Rev)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 440://Z2轴最高速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ2].Lim_Speed)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ2].Lim_Speed)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 442://Z2轴爬行速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ2].Craml_Speed)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ2].Craml_Speed)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 444://Z2轴加速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ2].Acc_Time)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ2].Acc_Time)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 446://Z2轴减速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ2].Dec_Time)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ2].Dec_Time)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 448://Z2轴每圈距离
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ2].Distance_Rev)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ2].Distance_Rev)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 450://P2轴最高速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP2].Lim_Speed)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP2].Lim_Speed)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 452://P2轴爬行速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP2].Craml_Speed)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP2].Craml_Speed)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 454://P2轴加速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP2].Acc_Time)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP2].Acc_Time)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 456://P2轴减速度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP2].Dec_Time)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP2].Dec_Time)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 458://P2轴每圈距离
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP2].Distance_Rev)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP2].Distance_Rev)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 1000://针管获取
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = 0;
      if(HAL_GPIO_ReadPin(LIMIT2_NEG_GPIO_PORT,LIMIT2_NEG_PIN) == LIMIT2_NEG_ACTIVE_LEVEL)//相应变量赋值
        *_RegBuf |= (uint16_t) 1<<0;
			if(HAL_GPIO_ReadPin(LIMIT4_NEG_GPIO_PORT,LIMIT4_NEG_PIN) == LIMIT4_NEG_ACTIVE_LEVEL)//相应变量赋值
        *_RegBuf |= (uint16_t) 1<<1;
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 1006://P1轴体积距离比
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&prd_SMotor[PIP1].Volume_Distance)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&prd_SMotor[PIP1].Volume_Distance)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 1008://P2轴体积距离比
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&prd_SMotor[PIP2].Volume_Distance)%65536;//数据低半字
			*(_RegBuf+1) = *((uint32_t *)&prd_SMotor[PIP2].Volume_Distance)/65536;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2000://获取目标1温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2002://获取目标2温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2004://获取目标3温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2006://获取目标4温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2008://获取目标5温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2010://获取目标6温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2012://获取目标7温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2014://获取目标8温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2016://获取目标9温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2018://获取目标10温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2020://获取目标11温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2022://获取目标12温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2024://获取目标13温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2026://获取目标14温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2028://获取目标15温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2030://获取目标16温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2100://温控取消
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 2101://温控保持
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 2102://温控加热
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 2103://温控制冷
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 2104://设置目标1温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2106://设置目标2温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2108://设置目标3温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2110://设置目标4温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2112://设置目标5温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2114://设置目标6温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2116://设置目标7温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2118://设置目标8温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2120://设置目标9温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2122://设置目标10温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2124://设置目标11温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2126://设置目标12温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2128://设置目标13温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2130://设置目标14温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2132://设置目标15温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 2134://设置目标16温度
			if(*_RegIdex+2 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//数据低半字
			*(_RegBuf+1) = 0;//数据高半字
			*_RegIdex += 2;//索引标号加2
			*_CellRegNum = 2;//单元格内寄存器数量
			break;
		case 9000://异常报警
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 9002://电机故障
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 9003://针管脱落报警
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 9004://边界提醒
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 9005://位置编码器故障
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 9006://液槽报警
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 9007://缺液报警
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 9008://高温报警
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		case 9009://低温报警
			if(*_RegIdex+1 > _RegNum)//生成数据大于存储空间，参数出错
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//相应变量赋值
			*_RegIdex += 1;//索引标号加1
			*_CellRegNum = 1;//单元格内寄存器数量
			break;
		default:
			Value = 1;
			break;
	}
  return Value;
}
/**
  * 函数功能: 更新N个寄存器
  * 输入参数: _AddrOffset:偏移地址,_RegNum:寄存器数量,_RegAddr:寄存器地址
  * 返 回 值: 异常码:04H或NONE
  * 说    明: 生成_RegNum个寄存器数据，并向_AddrOffset所指向的空间里写入，如生成异常则立即返回
  */
uint8_t MB_UP_NReg(uint16_t* _AddrOffset,uint16_t _RegNum , uint16_t _RegAddr)
{
  uint16_t i = 0,j = 0;
	uint16_t CellRegNum;//单元格内寄存器数量
  uint16_t RegBuf[4];
  if((_AddrOffset - PduData.PtrHoldingbase)==0xffff)//寄存器地址和数量超出实际定义的范围
    return EX_CODE_04H;
  for(i=0;i<_RegNum;)
  {
		if(MB_MK_RegVal(_RegAddr, _RegNum, RegBuf, &i, &CellRegNum))//生成异常，则退出
      return EX_CODE_04H;
    for(j=0;j<CellRegNum;j++)
			*_AddrOffset++ = RegBuf[j];
  }
  return EX_CODE_NONE;
}
/**
  * 函数功能: 写,读N个线圈
  * 输入参数: _AddrOffset:偏移地址,_CoilNum:线圈数量,_Datebuf:数据指针
  * 返 回 值: 异常码:04H或NONE
  * 说    明: 在_AddrOffset所指向的空间里写入_RegNum个线圈数据,并且读取验证是否写入成功
  */
uint8_t MB_WR_NCoil(uint8_t* _AddrOffset,uint16_t _RegNum , uint8_t* _Datebuf)
{
  uint16_t i = 0,j ;
  uint16_t iquo,irem;//商和余数
  uint8_t Coil;
  uint8_t Value = 0;
  uint8_t* ValAddr = _AddrOffset;
  uint8_t* tempDatabuf = _Datebuf;
  iquo = _RegNum/8;
  irem = _RegNum%8;
  for(i=0;i<iquo-1;i++)
  {
      Value = *_Datebuf;
      for(j=0;j<8;j++)
        if(Value&((uint8_t)1<<j))
            *_AddrOffset++ = 1;
        else
            *_AddrOffset++ = 0;
      _Datebuf+=1;
  }
  Value = *_Datebuf;
  for(j=0;j<irem;j++)
    if(Value&((uint8_t)1<<j))
        *_AddrOffset++ = 1;
    else
        *_AddrOffset++ = 0;
/* 读取验证写入是否成功 */
  _Datebuf = tempDatabuf;
  for(i=0;i<iquo-1;i++)
  {
      Value = *_Datebuf;
      for(j=0;j<8;j++)
      {
        if(Value&((uint8_t)1<<j))
            Coil = 1;
        else
            Coil = 0;
        if(*ValAddr++ != Coil)
            return EX_CODE_04H;
      }
      _Datebuf+=1;
  }
  Value = *_Datebuf;
  for(j=0;j<irem;j++)
  {
    if(Value&((uint8_t)1<<j))
        Coil = 1;
    else
        Coil = 0;
    if(*ValAddr++ != Coil)
        return EX_CODE_04H;
  }
  return EX_CODE_NONE;
}
/**
  * 函数功能: 写,读N个寄存器
  * 输入参数: _AddrOffset:偏移地址,_RegNum:寄存器数量,_Datebuf:数据指针
  * 返 回 值: 异常码:04H或NONE
  * 说    明: 在_AddrOffset所指向的空间里写入_RegNum*2个数据,并且读取验证是否写入成功
  */
uint8_t MB_WR_NReg(uint16_t* _AddrOffset,uint16_t _RegNum , uint8_t* _Datebuf)
{
  uint16_t i = 0;
  uint16_t Value = 0;
  uint16_t* ValAddr = _AddrOffset;
  uint8_t* tempDatabuf = _Datebuf;
  for(i=0;i<_RegNum;i++)
  {
    Value = (uint16_t)((*_Datebuf<<8 ) | (*(_Datebuf+1)));
    *_AddrOffset++ = Value ;
    _Datebuf+=2;
  }
  /* 读取验证写入是否成功 */
  _Datebuf = tempDatabuf;
  for(i = 0;i<_RegNum;i++)
  {
    Value = (uint16_t)((*_Datebuf<<8 ) | (*(_Datebuf+1)));
    if(*ValAddr++ != Value)
    {
      return EX_CODE_04H;
    }
    _Datebuf+=2;
  }
  return EX_CODE_NONE;
}
/**
  * 函数功能: 判断地址是否符合协议范围
  * 输入参数: _Addr:起始地址,_RegNum:寄存器数量,_FunCode:功能码
  * 返 回 值: 异常码:02H或NONE
  * 说    明: 地址范围是0x0000~0xFFFF,可操作的空间范围不能超过这个区域
  */
uint8_t MB_JudgeAddr(uint16_t _Addr,uint16_t _RegNum)
{
  uint8_t Excode = EX_CODE_NONE;
  /* 地址+寄存器数量不能超过0xFFFF */
  if( ((uint32_t)_RegNum+(uint32_t)_Addr) > (uint32_t)0xFFFF)
  {
    Excode = EX_CODE_02H;// 异常码 02H
  }
  return Excode;
}
/**
  * 函数功能: 判断操作的数据量是否符合协议范围
  * 输入参数: _RegNum:寄存器数量,_FunCode:功能码,_ByteNum:字节数量
  * 返 回 值: 异常码:03或NONE
  * 说    明: 对可操作连续内存空间的功能码需要验证操作的地址是否符合范围
  */
uint8_t MB_JudgeNum(uint16_t _RegNum,uint8_t _FunCode,uint16_t _ByteNum)
{
  uint8_t Excode = EX_CODE_NONE;
  uint16_t _CoilNum = _RegNum; // 线圈(离散量)的数量
  switch(_FunCode)
  {
    case FUN_CODE_01H: 
    case FUN_CODE_02H:
      if( (_CoilNum<0x0001) || (_CoilNum>0x07D0))
        Excode = EX_CODE_03H;// 异常码03H;
      break;
    case FUN_CODE_03H:
    case FUN_CODE_04H:
      if( (_RegNum<0x0001) || (_RegNum>0x007D))
        Excode = EX_CODE_03H;// 异常码03H;      
      break;
    case FUN_CODE_0FH:
      if( (_RegNum<0x0001) || (_RegNum>0x07B0))
        Excode = EX_CODE_03H;// 异常码03H
      if( _ByteNum != (_RegNum/8 + ((_RegNum%8)?1:0)))
        Excode = EX_CODE_03H;// 异常码03H
      break;
    case FUN_CODE_10H:
      if( (_RegNum<0x0001) || (_RegNum>0x007B))
        Excode = EX_CODE_03H;// 异常码03H
      if( _ByteNum != (_RegNum<<1))
        Excode = EX_CODE_03H;// 异常码03H
      break;
  }
  return Excode;
}
/**
  * 函数功能: 读取离散输出
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_CoilNum:线圈数量
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 读取离散输出,并且填充Tx_Buf
  */
uint16_t MB_RSP_01H(uint16_t _TxCount,uint8_t *_AddrOffset ,uint16_t _CoilNum)
{
  uint16_t ByteNum = 0;
  uint16_t i = 0;
  uint16_t r = 0 ;
  ByteNum = _CoilNum/8;
  /* 如果存在余数,需要将多余的位置0 */
  r = _CoilNum%8;
  if(r != 0)
  {
    ByteNum += 1; //字节数+1
    Tx_Buf[_TxCount++] = ByteNum;  
    
    /* 在忽略余数的情况下读取线圈(bit)的状态 */
    for(i=0;i<ByteNum-1;i++)
    {
      Tx_Buf[_TxCount] = 0x00;
      /* 每8个线圈(bit)一次循环,读取8个线圈的状态放置在一个Byte*/
      for(uint8_t j=0;j<8;j++)
      {
        /* 如果是1,则将byte对应位(bit)置1 */
        if(*(_AddrOffset++))
        {
          /* 在这里相当于将一个Byte当成一个线圈(bit)映射到一个Byte上
           * 由8个Byte组成一个Byte的8个bit
           */
          Tx_Buf[_TxCount] |= (0x01<<j);
        }
      }
      _TxCount++;
    }
    /* 有余数部分组成一个字节,多余的bit设置为0 */
    Tx_Buf[_TxCount] = 0x00;
    for(uint8_t j=0;j<r;j++)
    {
      if(*(_AddrOffset++))
      {
        Tx_Buf[_TxCount] |= (0x01<<j);// 读取数据
      }
    }
    _TxCount++;
  }
  /* 如果余数r==0,说明需要读取的线圈数量(bits)刚好是Byte的整数倍 */
  else
  {
    Tx_Buf[_TxCount++] = ByteNum;
    for(i=0;i<ByteNum;i++)
    {
      Tx_Buf[_TxCount] = 0x00;
      for(uint8_t j=0;j<8;j++)
      {
        if(*(_AddrOffset++))
        {
          Tx_Buf[_TxCount] |= (0x01<<j);// 读取数据
        }
      }
      _TxCount++;
    }
  }
  return _TxCount;
}
/**
  * 函数功能: 读取保持寄存器
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_RegNum:寄存器数量
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 读取保持寄存器的内容,并且填充Tx_Buf
  */
uint8_t MB_RSP_03H(uint16_t _TxCount,uint16_t *_AddrOffset,uint16_t _RegNum )
{
  Tx_Buf[_TxCount++] = _RegNum<<1;
  for(uint8_t i = 0;i< _RegNum;i++)
  {
    Tx_Buf[_TxCount++] = ((*_AddrOffset)>>8);
    Tx_Buf[_TxCount++] = *_AddrOffset++;
  }
  return _TxCount;
}
/**
  * 函数功能: 根据05H功能码填充发送缓冲区
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_AddrAbs:绝对地址值
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 填充Tx_Buf
  */
uint8_t MB_RSP_05H(uint16_t _TxCount,uint16_t _AddrOffset ,uint8_t *_AddrAbs)
{
  /* 填充地址值 */
  Tx_Buf[_TxCount++] = _AddrOffset>>8;
  Tx_Buf[_TxCount++] = _AddrOffset;
  /* 填充输出值 */
  if((*_AddrAbs) == 1)
    Tx_Buf[_TxCount++] = 0xFF;
  else Tx_Buf[_TxCount++] = 0x00;
  Tx_Buf[_TxCount++] = 0x00;
  return _TxCount;
}
/**
  * 函数功能: 根据06H功能码填充发送缓冲区
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_AddrAbs:绝对地址值
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 填充Tx_Buf
  */
uint8_t MB_RSP_06H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t *_AddrAbs)
{
  /* 填充地址值 */
  Tx_Buf[_TxCount++] = _AddrOffset>>8;
  Tx_Buf[_TxCount++] = _AddrOffset;
  /* 填充输出值 */
  Tx_Buf[_TxCount++] = (*_AddrAbs)>>8;
  Tx_Buf[_TxCount++] = *_AddrAbs;
  return _TxCount;
}
/**
  * 函数功能: 根据10H功能码填充发送缓冲区
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_ByteNum:字节数量
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 填充Tx_Buf
  */
uint8_t MB_RSP_10H(uint16_t _TxCount,uint16_t _AddrOffset,uint16_t _ByteNum)
{
  Tx_Buf[_TxCount++] = _AddrOffset>>8;
  Tx_Buf[_TxCount++] = _AddrOffset;
  
  Tx_Buf[_TxCount++] = _ByteNum>>8;
  Tx_Buf[_TxCount++] = _ByteNum;
  return _TxCount;
}
/**
  * 函数功能: 异常响应
  * 输入参数: _FunCode :发送异常的功能码,_ExCode:异常码
  * 返 回 值: 无
  * 说    明: 当通信数据帧发生异常时,发送异常响应
  */
void MB_Exception_RSP(uint8_t _FunCode,uint8_t _ExCode)
{
  uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = MB_SLAVEADDR;		    /* 从站地址 */
	Tx_Buf[TxCount++] = _FunCode|0x80;		  /* 功能码 + 0x80*/	
	Tx_Buf[TxCount++] = _ExCode ;	          /* 异常码*/
	
  crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc 低字节 */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc 高字节 */
  UART_Tx((uint8_t*)Tx_Buf, TxCount);
}
/**
  * 函数功能: 正常响应
  * 输入参数: _FunCode :功能码
  * 返 回 值: 无
  * 说    明: 当通信数据帧没有异常时并且成功执行之后,发送响应数据帧
  */
void MB_RSP(uint8_t _FunCode)
{
  uint16_t TxCount = 0;
  uint16_t crc = 0;	Tx_Buf[TxCount++] = MB_SLAVEADDR;		 /* 从站地址 */
	Tx_Buf[TxCount++] = _FunCode;        /* 功能码   */	
  switch(_FunCode)
  {
    case FUN_CODE_01H:
    case FUN_CODE_02H:
      TxCount = MB_RSP_01H(TxCount,(uint8_t*)PduData.PtrCoilOffset,PduData.Num);
      break;
    case FUN_CODE_03H:
    case FUN_CODE_04H:
      TxCount = MB_RSP_03H(TxCount,(uint16_t*)PduData.PtrHoldingOffset,PduData.Num);
      break;
    case FUN_CODE_05H:
      TxCount = MB_RSP_05H(TxCount,PduData.Addr,(uint8_t*)PduData.PtrCoilOffset);
      break;
    case FUN_CODE_06H:
      TxCount = MB_RSP_06H(TxCount,PduData.Addr,(uint16_t*)PduData.PtrHoldingOffset);
      break;
    case FUN_CODE_0FH:
    case FUN_CODE_10H:
      TxCount = MB_RSP_10H(TxCount,PduData.Addr,PduData.Num);
      break;
  }
  crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc 低字节 */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc 高字节 */
  UART_Tx((uint8_t*)Tx_Buf, TxCount);
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
