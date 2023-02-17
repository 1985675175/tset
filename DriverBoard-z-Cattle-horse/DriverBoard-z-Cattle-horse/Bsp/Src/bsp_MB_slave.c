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
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
PDUData_TypeDef PduData;

// CRC ��λ�ֽ�ֵ��
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
// CRC ��λ�ֽ�ֵ��
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
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
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

/* ������ --------------------------------------------------------------------*/

/** 
  * ��������: Modbus CRC16 У����㺯��
  * �������: pushMsg:������������׵�ַ,usDataLen:���ݳ���
  * �� �� ֵ: CRC16 ������
  * ˵    ��: �������Ǹ�λ��ǰ,��Ҫת�����ܷ���
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
/* ��ȡ����֡,���н�������֡ */
void MB_Parse_Data()
{
  PduData.Code = Rx_Buf[1];                   // ������
  PduData.Addr = ((Rx_Buf[2]<<8) | Rx_Buf[3]);// �Ĵ�����ʼ��ַ
  PduData.Num  = ((Rx_Buf[4]<<8) | Rx_Buf[5]);// ����(Coil,Input,Holding Reg,Input Reg)
  PduData._CRC = MB_CRC16((uint8_t*)&Rx_Buf,RxCount-2);             // CRCУ����
  PduData.byteNums = Rx_Buf[6];               // ����ֽ���
  PduData.ValueReg = (uint8_t*)&Rx_Buf[7];                          // �Ĵ���ֵ��ʼ��ַ
//  PduData.PtrCoilOffset = PduData.PtrCoilbase + PduData.Addr;       // ��ɢ�����ڴ���ʼ��ַ
//  PduData.PtrHoldingOffset = PduData.PtrHoldingbase + PduData.Addr; // ���ּĴ�������ʼ��ַ
//  PduData.PtrCoilOffset = PduData.PtrCoilbase;       // ��ɢ�����ڴ���ʼ��ַ  ��ʱ���Խ��
//  PduData.PtrHoldingOffset = PduData.PtrHoldingbase; // ���ּĴ�������ʼ��ַ  ��ʱ���Խ��
  PduData.PtrCoilOffset = PduData.PtrCoilbase + MB_CoilAddr2Offset(PduData.Addr,PduData.Num);       // ��ɢ�����ڴ���ʼ��ַ  ���Ƚ�ʡ�ռ�
  PduData.PtrHoldingOffset = PduData.PtrHoldingbase + MB_RegAddr2Offset(PduData.Addr,PduData.Num); // ���ּĴ�������ʼ��ַ  ���Ƚ�ʡ�ռ�
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

//��Ȧ����ַ�����ݳ�������
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
/* ��Ȧ��ַת��ʵ��ƫ���� */
uint16_t MB_CoilAddr2Offset(uint16_t CoilAddr,uint16_t CoilNum)
{
  uint16_t Offset = 0;
  uint16_t ContinueOffset = 0;//����ƫ��,��Խ����
  uint16_t CoilBasicAddr;
  uint16_t BasicAddrIdexMin = 0;//��С��ַ��ʵ�ʻ���ַ��Ӧ����
  uint16_t BasicAddrIdexMax = COIL_BASIC_ADDR_IDEX_MAX;
  uint16_t i;
  //��Ȧ����ַ����
  if(CoilArray[BasicAddrIdexMax][0] < CoilAddr)
  {
    if((CoilArray[BasicAddrIdexMax][0]+CoilArray[BasicAddrIdexMax][1]) > CoilAddr)
    {
      CoilBasicAddr = CoilArray[BasicAddrIdexMax][0];
      BasicAddrIdexMin = BasicAddrIdexMax;
    }
    else
    {
      Offset = 0xffff;//��ַ������Χ����Ϊ0xffff
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
  //�Ĵ���ƫ�Ʋ���
  Offset = CoilAddr - CoilBasicAddr;
  //ȷ�����������С,�����󣬼Ĵ���Խ��
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
    Offset = 0xffff;//��ַ������Χ����Ϊ0xffff
    return Offset;
  }
  //ʵ�ʼĴ�������ַ����
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

//�Ĵ�����ַ�����ݳ�������
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
/* �Ĵ�����ַת��ʵ��ƫ���� */
uint16_t MB_RegAddr2Offset(uint16_t RegAddr,uint16_t RegNum)
{
  uint16_t ContinueOffset = 0;//����ƫ��,��Խ����
  uint16_t Offset = 0;
  uint16_t RegBasicAddr;
  uint16_t BasicAddrIdexMin = 0;//��С��ַ��ʵ�ʻ���ַ��Ӧ����
  uint16_t BasicAddrIdexMax = REG_BASIC_ADDR_IDEX_MAX;
  uint16_t i;
  //�Ĵ�������ַ����
  if(RegArray[BasicAddrIdexMax][0] < RegAddr)
  {
    if((RegArray[BasicAddrIdexMax][0]+RegArray[BasicAddrIdexMax][1]) > RegAddr)
    {
      RegBasicAddr = RegArray[BasicAddrIdexMax][0];
      BasicAddrIdexMin = BasicAddrIdexMax;
    }
    else
    {
      Offset = 0xffff;//��ַ������Χ����Ϊ0xffff
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
  //�Ĵ���ƫ�Ʋ���
  Offset = RegAddr - RegBasicAddr;
  //ȷ�����������С,�����󣬼Ĵ���Խ��
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
    Offset = 0xffff;//��ַ������Χ����Ϊ0xffff
    return Offset;
  }
  //ʵ�ʼĴ�������ַ����
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
  * ��������: �Խ��յ������ݽ��з�����ִ��
  * �������: ��
  * �� �� ֵ: �쳣���0x00
  * ˵    ��: �жϹ�����,��֤��ַ�Ƿ���ȷ.��ֵ�����Ƿ����,����û����ͷ�����Ӧ�ź�
  */
uint8_t MB_Analyze_Execute(void )
{
  uint16_t ExCode = EX_CODE_NONE;
  /* У�鹦���� */
  if( IS_NOT_FUNCODE(PduData.Code) ) // ��֧�ֵĹ�����
  {
    /* Modbus�쳣��Ӧ */
    ExCode = EX_CODE_01H;            // �쳣��01H
    return ExCode;
  }
  /* ���ݹ�����ֱ����ж� */
  switch(PduData.Code)
  {
    /* ������Ϊ01H�������02��������һ����,��ʵҲûʲô��һ��
     * ֻ�ǲ�����ַ���ܲ�һ��,��һ���Ͼ�����ʵ��,������main����
     * ���뵥�����ڴ�ʹ�ò�ͬ�Ĺ�����,��ʵ��Ӧ���б����������ʹ��
     * ��ͬ���ڴ�ռ�
     */
/* ---- 01H  02H ��ȡ��ɢ������(Coil Input)---------------------- */
    case FUN_CODE_01H:
    case FUN_CODE_02H:
      /* �ж���Ȧ�����Ƿ���ȷ */  
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,1);
      if(ExCode != EX_CODE_NONE )
        return ExCode;      
      
      /* �жϵ�ַ�Ƿ���ȷ*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      
      /* �ж����ݸ����Ƿ���ȷ*/
      ExCode = MB_UP_NCoil((uint8_t *)PduData.PtrCoilOffset, PduData.Num, PduData.Addr);
      if(ExCode != EX_CODE_NONE )
        return ExCode;
      break;
/* ---- 03H  04H ��ȡ����/����Ĵ���---------------------- */
    case FUN_CODE_03H:
    case FUN_CODE_04H:
      /* �жϼĴ��������Ƿ���ȷ */
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,PduData.byteNums);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      
      /* �жϵ�ַ�Ƿ���ȷ*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
     
      /* �ж����ݸ����Ƿ���ȷ*/
      ExCode = MB_UP_NReg((uint16_t *)PduData.PtrHoldingOffset, PduData.Num, PduData.Addr);
      if(ExCode != EX_CODE_NONE )
        return ExCode;
      break;
/* ---- 05H д�뵥����ɢ��---------------------- */
    case FUN_CODE_05H:
      /* ���ֵ!=OK */
      if((PduData.Num != 0x0000) && PduData.Num != 0xFF00)
      {
        ExCode = EX_CODE_03H;
        return ExCode;        
      }
      /* д��һ����Ȧֵ */
      if(PduData.Num == 0xFF00)
      { 
        *PduData.PtrCoilOffset = 1;
        /* ��ȡд��ֵ,��֤�Ƿ�д��ɹ� */
        if( *PduData.PtrCoilOffset != 1)
        {
          ExCode = EX_CODE_04H;
          return ExCode;
        }
      }
      else 
      {
        *PduData.PtrCoilOffset = 0;
        /* ��ȡд��ֵ,��֤�Ƿ�д��ɹ� */
        if( *PduData.PtrCoilOffset != 0)
        { 
          ExCode = EX_CODE_04H;
          return ExCode;
        }
      }
      break;
/* ---- 06H д�������ּĴ��� ---------------------- */
    case FUN_CODE_06H:
      
      /* д��Ĵ���ֵ*/
      *PduData.PtrHoldingOffset = PduData.Num;
      /* ��֤д�ɹ� */
      if(*PduData.PtrHoldingOffset != PduData.Num)
      {
        ExCode = EX_CODE_04H;
         return ExCode;
      }
      break;
/* ---- 0FH д�����Ȧ ---------------------- */
    case FUN_CODE_0FH:
      /* �жϼĴ��������Ƿ���ȷ */
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,PduData.byteNums);
      if(ExCode != EX_CODE_NONE )
        return ExCode;    
      
      /* �жϵ�ַ�Ƿ���ȷ*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      
      /* д������Ȧ */
      ExCode = MB_WR_NCoil((uint8_t*)PduData.PtrCoilOffset,PduData.Num,(uint8_t*)PduData.ValueReg);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      break;
/* ---- 10H д������ּĴ��� ---------------------- */
    case FUN_CODE_10H:
      /* �жϼĴ��������Ƿ���ȷ */
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,PduData.byteNums);
      if(ExCode != EX_CODE_NONE )
        return ExCode;    
      
      /* �жϵ�ַ�Ƿ���ȷ*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      
      /* д�����Ĵ��� */
      ExCode = MB_WR_NReg((uint16_t*)PduData.PtrHoldingOffset,PduData.Num,(uint8_t*)PduData.ValueReg);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      break;
  }
  /* ����֡û���쳣 */
  return ExCode; //   EX_CODE_NONE
}
/**
  * ��������: ��Ч��Ȧ��ֵ
	* �������: _CoilAddr:��Ȧ��ַ, _CoilNum:��Ȧ����, _CoilData:��Ȧ����, _CoilIdex����Ȧ����
  * �� �� ֵ: �쳣��:0������
  * ˵    ��: ������Ȧ��ַ����������Ч��Ȧ��ֵ
  */
uint8_t MB_EF_CoilVal(uint16_t _CoilAddr, uint16_t _CoilNum, uint8_t *_CoilData, uint16_t *_CoilIdex)
{
//  uint16_t i = 0;
  uint8_t Error = 0;

//	�ɸ���_CoilNum���Ż���ȡ����δʹ�ã���Ŀǰ����Ϊ��С����
	switch(_CoilAddr + *_CoilIdex)
	{
		case 0://�������е�
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				LED1_ON();
			else
				LED1_OFF();
			*_CoilIdex += 1;//�����������
			break;
		case 1://���1���е�
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				LED2_ON();
			else
				LED2_OFF();
			*_CoilIdex += 1;//�����������
			break;
		case 2://���2���е�
		case 3://���3���е�
		case 4://���4���е�
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				LED3_ON();
			else
				LED3_OFF();
			*_CoilIdex += 1;//�����������
			break;
		case 5://������
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				LAMP_ON();
			else
				LAMP_OFF();
			*_CoilIdex += 1;//�����������
			break;
		case 6://����������
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				BEEP_ON();
			else
				BEEP_OFF();
			*_CoilIdex += 1;//�����������
			break;
		case 100://���ȹ�1
		case 101://���ȹ�2
		case 102://���ȹ�3
		case 103://���ȹ�4
		case 104://���ȹ�5
		case 105://���ȹ�6
		case 106://���ȹ�7
		case 107://���ȹ�8
		case 108://���ȹ�9
		case 109://���ȹ�10
		case 110://���ȹ�11
		case 111://���ȹ�12
		case 112://���ȹ�13
		case 113://���ȹ�14
		case 114://���ȹ�15
		case 115://���ȹ�16
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HEAT_ON();
			else
				HEAT_OFF();
			*_CoilIdex += 1;//�����������
			break;
		case 132://�����1
		case 133://�����2
		case 134://�����3
		case 135://�����4
		case 136://�����5
		case 137://�����6
		case 138://�����7
		case 139://�����8
		case 140://�����9
		case 141://�����10
		case 142://�����11
		case 143://�����12
		case 144://�����13
		case 145://�����14
		case 146://�����15
		case 147://�����16
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HEAT_ON();//��ʱ�ý������
			else
				HEAT_OFF();
			*_CoilIdex += 1;//�����������
			break;
		case 500://X����EN
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR1_ENA_GPIO_PORT, SMOTOR1_ENA_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_ENA_GPIO_PORT, SMOTOR1_ENA_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 501://X����STEP
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR1_PUL_GPIO_PORT, SMOTOR1_PUL_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_PUL_GPIO_PORT, SMOTOR1_PUL_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 502://X����DIR
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 503://X����MD0
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 504://X����MD1
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 505://X����MD2
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 506://X����DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if((*_CoilData == 1)&&(*(_CoilData+1) == 1))//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else if((*_CoilData == 0)&&(*(_CoilData+1) == 0))
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			else//����Ϊ����״̬
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 2;//�����������
		case 508://X����RESET
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 509://X����SLEEP
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 512://Y����EN
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR2_ENA_GPIO_PORT, SMOTOR2_ENA_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_ENA_GPIO_PORT, SMOTOR2_ENA_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 513://Y����STEP
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR2_PUL_GPIO_PORT, SMOTOR2_PUL_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_PUL_GPIO_PORT, SMOTOR2_PUL_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 514://Y����DIR
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR2_DIR_GPIO_PORT, SMOTOR2_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_DIR_GPIO_PORT, SMOTOR2_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 515://Y����MD0
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 516://Y����MD1
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 517://Y����MD2
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 518://Y����DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if((*_CoilData == 1)&&(*(_CoilData+1) == 1))//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else if((*_CoilData == 0)&&(*(_CoilData+1) == 0))
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			else//����Ϊ����״̬
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 2;//�����������
		case 520://Y����RESET
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 521://Y����SLEEP
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 524://Z1����EN
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR1_ENA_GPIO_PORT, SMOTOR1_ENA_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_ENA_GPIO_PORT, SMOTOR1_ENA_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 525://Z1����STEP
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR1_PUL_GPIO_PORT, SMOTOR1_PUL_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_PUL_GPIO_PORT, SMOTOR1_PUL_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 526://Z1����DIR
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 527://Z1����MD0
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 528://Z1����MD1
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 529://Z1����MD2
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 530://Z1����DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if((*_CoilData == 1)&&(*(_CoilData+1) == 1))//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else if((*_CoilData == 0)&&(*(_CoilData+1) == 0))
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			else//����Ϊ����״̬
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 2;//�����������
		case 532://Z1����RESET
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 533://Z1����SLEEP
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 536://P1����EN
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR2_ENA_GPIO_PORT, SMOTOR2_ENA_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_ENA_GPIO_PORT, SMOTOR2_ENA_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 537://P1����STEP
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR2_PUL_GPIO_PORT, SMOTOR2_PUL_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_PUL_GPIO_PORT, SMOTOR2_PUL_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 538://P1����DIR
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR2_DIR_GPIO_PORT, SMOTOR2_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_DIR_GPIO_PORT, SMOTOR2_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 539://P1����MD0
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 540://P1����MD1
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 541://P1����MD2
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 542://P1����DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if((*_CoilData == 1)&&(*(_CoilData+1) == 1))//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else if((*_CoilData == 0)&&(*(_CoilData+1) == 0))
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			else//����Ϊ����״̬
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 2;//�����������
		case 544://P1����RESET
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 545://P1����SLEEP
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 548://Z2����EN
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR1_ENA_GPIO_PORT, SMOTOR1_ENA_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_ENA_GPIO_PORT, SMOTOR1_ENA_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 549://Z2����STEP
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR1_PUL_GPIO_PORT, SMOTOR1_PUL_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_PUL_GPIO_PORT, SMOTOR1_PUL_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 550://Z2����DIR
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 551://Z2����MD0
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 552://Z2����MD1
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 553://Z2����MD2
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 554://Z2����DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if((*_CoilData == 1)&&(*(_CoilData+1) == 1))//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else if((*_CoilData == 0)&&(*(_CoilData+1) == 0))
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			else//����Ϊ����״̬
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 2;//�����������
		case 556://Z2����RESET
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 557://Z2����SLEEP
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 560://P2����EN
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR2_ENA_GPIO_PORT, SMOTOR2_ENA_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_ENA_GPIO_PORT, SMOTOR2_ENA_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 561://P2����STEP
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR2_PUL_GPIO_PORT, SMOTOR2_PUL_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_PUL_GPIO_PORT, SMOTOR2_PUL_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 562://P2����DIR
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				HAL_GPIO_WritePin(SMOTOR2_DIR_GPIO_PORT, SMOTOR2_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(SMOTOR2_DIR_GPIO_PORT, SMOTOR2_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 563://P2����MD0
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 564://P2����MD1
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 565://P2����MD2
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 566://P2����DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if((*_CoilData == 1)&&(*(_CoilData+1) == 1))//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else if((*_CoilData == 0)&&(*(_CoilData+1) == 0))
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			else//����Ϊ����״̬
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 2;//�����������
		case 568://P2����RESET
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		case 569://P2����SLEEP
			if(*_CoilIdex+1 > _CoilNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _CoilData += *_CoilIdex;//���ݵ�ַ����
			if(*_CoilData == 1)//������Ӧ״̬��ֵ
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_SET);
			else
				;//HAL_GPIO_WritePin(SMOTOR1_DIR_GPIO_PORT, SMOTOR1_DIR_GPIO_PIN, GPIO_PIN_RESET);
			*_CoilIdex += 1;//�����������
			break;
		default:
			Error = 1;
			break;
	}
  return Error;
}
/**
  * ��������: ��ЧN����Ȧ��ֵ
  * �������: _CoilData:��Ȧ����,_CoilNum:��Ȧ����,_CoilAddr:��Ȧ��ַ
  * �� �� ֵ: �쳣��:0������
  * ˵    ��: ��Ч_CoilNum����Ȧ���ݣ�����Ч�쳣����������
  */
uint8_t MB_EF_NCoil(uint16_t _CoilAddr, uint16_t _CoilNum, uint8_t* _CoilData)
{
  uint8_t Error = 1;
  uint16_t i = 0;
  for(i=0;i<_CoilNum;)
  {
	Error = MB_EF_CoilVal(_CoilAddr, _CoilNum, _CoilData, &i);
	if(Error)//��Ч�쳣�����˳�
      break;
  }
  return Error;
}
/**
  * ��������: 05H��������Ȧ��Ч����
  * �������: _CoilAddr��Ȧ��ַ, _CoilData:��Ȧ����
  * �� �� ֵ: �쳣��:0������
  * ˵    ��: ʹ��Ȧ�е���ֵ��Ч
  */
uint8_t MB_Effect_05H(uint16_t _CoilAddr, uint8_t* _CoilData)
{
	return MB_EF_NCoil(_CoilAddr, 1, _CoilData);
}

/**
  * ��������: 0FH��������Ȧ��Ч����
  * �������: _CoilAddr��Ȧ��ַ, _CoilData:��Ȧ����, _CoilData:��Ȧ����
  * �� �� ֵ: �쳣��:0������
  * ˵    ��: ʹ��Ȧ�е���ֵ��Ч
  */
uint8_t MB_Effect_0FH(uint16_t _CoilAddr, uint16_t _CoilNum, uint8_t* _CoilData)
{
	return MB_EF_NCoil(_CoilAddr, _CoilNum, _CoilData);
}

/**
  * ��������: ��Ч�Ĵ�����ֵ
	* �������: _RegAddr:�Ĵ�����ַ, _RegNum:�Ĵ�������, _RegData:�Ĵ�������, _RegIdex���Ĵ�������
  * �� �� ֵ: �쳣��:0������
  * ˵    ��: ���ݼĴ�����ַ�������ͼĴ������ݣ���Ч��Ȧ��ֵ
  */
uint8_t MB_EF_RegVal(uint16_t _RegAddr, uint16_t _RegNum, uint16_t *_RegData, uint16_t *_RegIdex)
{
//  uint16_t i = 0;
  uint8_t Error = 0;
  uint16_t Temp16;//�����ݸı���ʱʹ��
  uint32_t Temp32;
  uint32_t TempFloat1,TempFloat2;

//	�ɸ���_RegNum���Ż���ȡ����δʹ�ã���Ŀǰ����Ϊ��С����
	switch(_RegAddr + *_RegIdex)
	{
		case 2://�ع�ԭ��
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
            OriginCmdLast = Temp32;//����ԭ���������
//			if(Temp32&0x00000001)//X��
//				Position_Initial(SM1, mrd_SMotor[SM1].Lim_Speed); // ��ԭ��
//			if(Temp32&0x00000002)//Y��
//				Position_Initial(SM2, mrd_SMotor[SM2].Lim_Speed); // ��ԭ��
			if(Temp32&0x00000004)//Z1��
				Position_Initial(SM1, mrd_SMotor[SMZ1].Lim_Speed); // ��ԭ��
			if(Temp32&0x00000008)//P1��
				Position_Initial(SM2, mrd_SMotor[SMP1].Lim_Speed); // ��ԭ��
			if(Temp32&0x00000010)//Z2��
				Position_Initial(SM3, mrd_SMotor[SMZ2].Lim_Speed); // ��ԭ��
			if(Temp32&0x00000020)//P2��
				Position_Initial(SM4, mrd_SMotor[SMP2].Lim_Speed); // ��ԭ��
			*_RegIdex += 2;//�����������
			break;
		case 4://��ʱ�ȴ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			TMR_SetInit(&Timer[TMR_NUM_DELAY], 1, 1, Temp32);//��ʱ�ȴ�����ִ��
      TMR_Reset(&Timer[TMR_NUM_DELAY]);
      TMR_SetTrigger(&Timer[TMR_NUM_DELAY], 1);
      *_RegIdex += 2;//�����������
			break;
		case 8://����״̬����
			if(*_RegIdex+1 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp16 = *_RegData;
			switch(Temp16&0x0003)
			{
				case 0://Ĭ��ֵ
					break;
				case 1://ֹͣ
					break;
				case 2://��ͣ
					break;
				case 3://����
					break;
			}
			*_RegIdex += 1;//�����������
			break;
		case 200://X���������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
            STEPMOTOR_LSCMoveRel(SM1, ((float) Temp32  - mrd_SMotor[SM1].Coord)/mrd_SMotor[SM1].Distance_Rev*(360.0f/mrd_SMotor[SM1].Step_Angle*mrd_SMotor[SM1].Micro_Step), mrd_SMotor[SM1].Acc_Time, mrd_SMotor[SM1].Dec_Time, mrd_SMotor[SM1].Lim_Speed);
			mrd_SMotor[SM1].Coord = (float) Temp32;//�������꺯��ִ��
			*_RegIdex += 2;//�����������
			break;
		case 202://Y���������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
            STEPMOTOR_LSCMoveRel(SM2, ((float) Temp32  - mrd_SMotor[SM2].Coord)/mrd_SMotor[SM2].Distance_Rev*(360.0f/mrd_SMotor[SM2].Step_Angle*mrd_SMotor[SM2].Micro_Step), mrd_SMotor[SM2].Acc_Time, mrd_SMotor[SM2].Dec_Time, mrd_SMotor[SM2].Lim_Speed);
			mrd_SMotor[SM2].Coord = (float) Temp32;//�������꺯��ִ��
			*_RegIdex += 2;//�����������
			break;
		case 204://Z1���������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
            STEPMOTOR_LSCMoveRel(SMZ1, (*((float *)(&Temp32)) - mrd_SMotor[SMZ1].Coord)/mrd_SMotor[SMZ1].Distance_Rev*(360.0f/mrd_SMotor[SMZ1].Step_Angle*mrd_SMotor[SMZ1].Micro_Step), mrd_SMotor[SMZ1].Acc_Time, mrd_SMotor[SMZ1].Dec_Time, mrd_SMotor[SMZ1].Lim_Speed);
			mrd_SMotor[SMZ1].Coord = *((float *)(&Temp32));//�������꺯��ִ��
			*_RegIdex += 2;//�����������
			break;
		case 206://P1���������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
      //���ƶ�������������
      TempFloat1 = *((float *)(&Temp32));
      if(TempFloat1 > prd_SMotor[PIP1].Max_Stroke) //���ֵ
        TempFloat1 = prd_SMotor[PIP1].Max_Stroke;
      else if(TempFloat1 < -prd_SMotor[PIP1].Unload_Distance) //��Сֵ
        TempFloat1 = -prd_SMotor[PIP1].Unload_Distance;
      TempFloat2 = mrd_SMotor[SMP1].Distance_Rev*(360.0f/mrd_SMotor[SMP1].Step_Angle*mrd_SMotor[SMP1].Micro_Step);//ÿ���ƶ�����
      STEPMOTOR_LSCMoveRel(SMP1, (TempFloat1 - mrd_SMotor[SMP1].Coord)/TempFloat2, mrd_SMotor[SMP1].Acc_Time, mrd_SMotor[SMP1].Dec_Time, mrd_SMotor[SMP1].Lim_Speed);
			mrd_SMotor[SMP1].Coord = TempFloat1;//�������꺯��ִ��
			*_RegIdex += 2;//�����������
			break;
		case 208://Z2���������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
      _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
      STEPMOTOR_LSCMoveRel(SMZ2, (*((float *)(&Temp32)) - mrd_SMotor[SMZ2].Coord)/mrd_SMotor[SMZ2].Distance_Rev*(360.0f/mrd_SMotor[SMZ2].Step_Angle*mrd_SMotor[SMZ2].Micro_Step), mrd_SMotor[SMZ2].Acc_Time, mrd_SMotor[SMZ2].Dec_Time, mrd_SMotor[SMZ2].Lim_Speed);
			mrd_SMotor[SMZ2].Coord = *((float *)(&Temp32));//�������꺯��ִ��
			*_RegIdex += 2;//�����������
			break;
		case 210://P2���������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
      //���ƶ�������������
      TempFloat1 = *((float *)(&Temp32));
      if(TempFloat1 > prd_SMotor[PIP2].Max_Stroke) //���ֵ
        TempFloat1 = prd_SMotor[PIP2].Max_Stroke;
      else if(TempFloat1 < -prd_SMotor[PIP2].Unload_Distance) //��Сֵ
        TempFloat1 = -prd_SMotor[PIP2].Unload_Distance;
      TempFloat2 = mrd_SMotor[SMP2].Distance_Rev*(360.0f/mrd_SMotor[SMP2].Step_Angle*mrd_SMotor[SMP2].Micro_Step);//ÿ���ƶ�����
      STEPMOTOR_LSCMoveRel(SMP2, (TempFloat1 - mrd_SMotor[SMP2].Coord)/TempFloat2, mrd_SMotor[SMP2].Acc_Time, mrd_SMotor[SMP2].Dec_Time, mrd_SMotor[SMP2].Lim_Speed);
			mrd_SMotor[SMP2].Coord = TempFloat1;//�������꺯��ִ��
			*_RegIdex += 2;//�����������
			break;
		case 300://X���������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM1].Coord += (float) Temp32;//������꺯��ִ��
            STEPMOTOR_LSCMoveRel(SM1, (float) Temp32/mrd_SMotor[SM1].Distance_Rev*(360.0f/mrd_SMotor[SM1].Step_Angle*mrd_SMotor[SM1].Micro_Step), mrd_SMotor[SM1].Acc_Time, mrd_SMotor[SM1].Dec_Time, mrd_SMotor[SM1].Lim_Speed);
//            STEPMOTOR_LSCMoveRel(SM2, (float) Temp32/mrd_SMotor[SM2].Distance_Rev*(360.0f/mrd_SMotor[SM2].Step_Angle*mrd_SMotor[SM2].Micro_Step), mrd_SMotor[SM2].Acc_Time, mrd_SMotor[SM2].Dec_Time, mrd_SMotor[SM2].Lim_Speed);
			*_RegIdex += 2;//�����������
			break;
		case 302://Y���������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM2].Coord += (float) Temp32;//������꺯��ִ��
            STEPMOTOR_LSCMoveRel(SM2, (float) Temp32/mrd_SMotor[SM2].Distance_Rev*(360.0f/mrd_SMotor[SM2].Step_Angle*mrd_SMotor[SM2].Micro_Step), mrd_SMotor[SM2].Acc_Time, mrd_SMotor[SM2].Dec_Time, mrd_SMotor[SM2].Lim_Speed);
			*_RegIdex += 2;//�����������
			break;
		case 304://Z1���������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ1].Coord += *((float *)(&Temp32));//������꺯��ִ��
            STEPMOTOR_LSCMoveRel(SMZ1, *((float *)(&Temp32))/mrd_SMotor[SMZ1].Distance_Rev*(360.0f/mrd_SMotor[SMZ1].Step_Angle*mrd_SMotor[SMZ1].Micro_Step), mrd_SMotor[SMZ1].Acc_Time, mrd_SMotor[SMZ1].Dec_Time, mrd_SMotor[SMZ1].Lim_Speed);
			*_RegIdex += 2;//�����������
			break;
		case 306://P1���������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Coord += *((float *)(&Temp32));//������꺯��ִ��
            STEPMOTOR_LSCMoveRel(SMP1, *((float *)(&Temp32))/mrd_SMotor[SMP1].Distance_Rev*(360.0f/mrd_SMotor[SMP1].Step_Angle*mrd_SMotor[SMP1].Micro_Step), mrd_SMotor[SMP1].Acc_Time, mrd_SMotor[SMP1].Dec_Time, mrd_SMotor[SMP1].Lim_Speed);
			*_RegIdex += 2;//�����������
			break;
		case 308://Z2���������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ2].Coord += *((float *)(&Temp32));//������꺯��ִ��
            STEPMOTOR_LSCMoveRel(SMZ2, *((float *)(&Temp32))/mrd_SMotor[SMZ2].Distance_Rev*(360.0f/mrd_SMotor[SMZ2].Step_Angle*mrd_SMotor[SMZ2].Micro_Step), mrd_SMotor[SMZ2].Acc_Time, mrd_SMotor[SMZ2].Dec_Time, mrd_SMotor[SMZ2].Lim_Speed);
			*_RegIdex += 2;//�����������
			break;
		case 310://P2���������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Coord += *((float *)(&Temp32));//������꺯��ִ��
            STEPMOTOR_LSCMoveRel(SMP2, *((float *)(&Temp32))/mrd_SMotor[SMP2].Distance_Rev*(360.0f/mrd_SMotor[SMP2].Step_Angle*mrd_SMotor[SMP2].Micro_Step), mrd_SMotor[SMP2].Acc_Time, mrd_SMotor[SMP2].Dec_Time, mrd_SMotor[SMP2].Lim_Speed);
			*_RegIdex += 2;//�����������
			break;
		case 400://X������ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM1].Lim_Speed = (float) Temp32;//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 402://X�������ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM1].Craml_Speed = (float) Temp32;//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 404://X����ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM1].Acc_Time = (float) Temp32;//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 406://X����ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM1].Dec_Time = (float) Temp32;//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 408://X��ÿȦ����   Ĭ�ϲ��ɸģ��ɻ������� 
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM1].Distance_Rev = (float) Temp32;//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 410://Y������ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM2].Lim_Speed = (float) Temp32;//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 412://Y�������ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM2].Craml_Speed = (float) Temp32;//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 414://Y����ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM2].Acc_Time = (float) Temp32;//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 416://Y����ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM2].Dec_Time = (float) Temp32;//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 418://Y��ÿȦ����   Ĭ�ϲ��ɸģ��ɻ������� 
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SM2].Distance_Rev = (float) Temp32;//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 420://Z1������ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ1].Lim_Speed = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 422://Z1�������ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ1].Craml_Speed = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 424://Z1����ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ1].Acc_Time = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 426://Z1����ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ1].Dec_Time = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 428://Z1��ÿȦ����   Ĭ�ϲ��ɸģ��ɻ������� 
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ1].Distance_Rev = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 430://P1������ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Lim_Speed = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 432://P1�������ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Craml_Speed = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 434://P1����ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Acc_Time = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 436://P1����ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Dec_Time = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 438://P1��ÿȦ����   Ĭ�ϲ��ɸģ��ɻ������� 
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Distance_Rev = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 440://Z2������ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ2].Lim_Speed = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 442://Z2�������ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ2].Craml_Speed = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 444://Z2����ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ2].Acc_Time = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 446://Z2����ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ2].Dec_Time = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 448://Z2��ÿȦ����   Ĭ�ϲ��ɸģ��ɻ������� 
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMZ2].Distance_Rev = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 450://P2������ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Lim_Speed = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 452://P2�������ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Craml_Speed = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 454://P2����ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Acc_Time = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 456://P2����ٶ�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Dec_Time = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 458://P2��ÿȦ����   Ĭ�ϲ��ɸģ��ɻ������� 
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Distance_Rev = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 1002://װ��� 
			if(*_RegIdex+1 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
      _RegData += *_RegIdex;//���ݵ�ַ����
			Temp16 = *_RegData;//�������ú���ִ��
			if(Temp16&0x0001)//���1
      {
        LoadFlag[SMZ1] = true;
        STEPMOTOR_LSCMoveRel(SMZ1, INT32_MAX, mrd_SMotor[SMZ1].Acc_Time, mrd_SMotor[SMZ1].Dec_Time, mrd_SMotor[SMZ1].Lim_Speed); 
      }
			if(Temp16&0x0002)//���2
      {
        LoadFlag[SMZ2] = true;
        STEPMOTOR_LSCMoveRel(SMZ2, INT32_MAX, mrd_SMotor[SMZ2].Acc_Time, mrd_SMotor[SMZ2].Dec_Time, mrd_SMotor[SMZ2].Lim_Speed); 
      }
			*_RegIdex += 1;//�����������
			break;
		case 1004://ж��� 
			if(*_RegIdex+1 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
      _RegData += *_RegIdex;//���ݵ�ַ����
			Temp16 = *_RegData;//�������ú���ִ��
			if(Temp16&0x0001)//���1
      {
        UnloadFlag[SMP1] = true;
        STEPMOTOR_LSCMoveRel(SMP1, (-prd_SMotor[PIP1].Unload_Distance)/mrd_SMotor[SMP1].Distance_Rev*mrd_SMotor[SMP1].Micro_Step*360/mrd_SMotor[SMP1].Step_Angle, mrd_SMotor[SMP1].Acc_Time, mrd_SMotor[SMP1].Dec_Time, mrd_SMotor[SMP1].Lim_Speed); 
      }
			if(Temp16&0x0002)//���2
      {
        UnloadFlag[SMP2] = true;
        STEPMOTOR_LSCMoveRel(SMP2, (-prd_SMotor[PIP2].Unload_Distance)/mrd_SMotor[SMP2].Distance_Rev*mrd_SMotor[SMP2].Micro_Step*360/mrd_SMotor[SMP2].Step_Angle, mrd_SMotor[SMP2].Acc_Time, mrd_SMotor[SMP2].Dec_Time, mrd_SMotor[SMP2].Lim_Speed); 
      }
			*_RegIdex += 1;//�����������
			break;
		case 1006://P1����������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
      _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			prd_SMotor[PIP1].Volume_Distance = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 1008://P2����������
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
      _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			prd_SMotor[PIP2].Volume_Distance = *((float *)(&Temp32));//�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 1038://P1��Һ����� 
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP1].Coord += *((float *)(&Temp32))/prd_SMotor[PIP1].Volume_Distance;//������꺯��ִ��
      STEPMOTOR_LSCMoveRel(SMP1, *((float *)(&Temp32))/prd_SMotor[PIP1].Volume_Distance/mrd_SMotor[SMP1].Distance_Rev*(360.0f/mrd_SMotor[SMP1].Step_Angle*mrd_SMotor[SMP1].Micro_Step), mrd_SMotor[SMP1].Acc_Time, mrd_SMotor[SMP1].Dec_Time, mrd_SMotor[SMP1].Craml_Speed);
			*_RegIdex += 2;//�����������
			break;
		case 1040://P2��Һ����� 
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			mrd_SMotor[SMP2].Coord += *((float *)(&Temp32))/prd_SMotor[PIP2].Volume_Distance;//������꺯��ִ��
      STEPMOTOR_LSCMoveRel(SMP2, *((float *)(&Temp32))/prd_SMotor[PIP2].Volume_Distance/mrd_SMotor[SMP2].Distance_Rev*(360.0f/mrd_SMotor[SMP2].Step_Angle*mrd_SMotor[SMP2].Micro_Step), mrd_SMotor[SMP2].Acc_Time, mrd_SMotor[SMP2].Dec_Time, mrd_SMotor[SMP2].Craml_Speed);
			*_RegIdex += 2;//�����������
			break;
		case 2100://�¿�ȡ��
			if(*_RegIdex+1 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp16 = *_RegData;
            //�������ú���ִ��
			if(Temp16&0x0001)//Ŀ��1
                ;//
			if(Temp16&0x0002)//Ŀ��2
                ;//
			*_RegIdex += 1;//�����������
			break;
		case 2101://�¿ر���
			if(*_RegIdex+1 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp16 = *_RegData;
            //�������ú���ִ��
			if(Temp16&0x0001)//Ŀ��1
                ;//
			if(Temp16&0x0002)//Ŀ��2
                ;//
			*_RegIdex += 1;//�����������
			break;
		case 2102://�¿ؼ���
			if(*_RegIdex+1 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp16 = *_RegData;
            //�������ú���ִ��
			if(Temp16&0x0001)//Ŀ��1
                ;//
			if(Temp16&0x0002)//Ŀ��2
                ;//
			*_RegIdex += 1;//�����������
			break;
		case 2103://�¿�����
			if(*_RegIdex+1 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp16 = *_RegData;
            //�������ú���ִ��
			if(Temp16&0x0001)//Ŀ��1
                ;//
			if(Temp16&0x0002)//Ŀ��2
                ;//
			*_RegIdex += 1;//�����������
			break;
		case 2104://����Ŀ��1�¶�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			(float) Temp32;//�������ú���ִ��
            //�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		case 2106://����Ŀ��2�¶�
		case 2108://����Ŀ��3�¶�
		case 2110://����Ŀ��4�¶�
		case 2112://����Ŀ��5�¶�
		case 2114://����Ŀ��6�¶�
		case 2116://����Ŀ��7�¶�
		case 2118://����Ŀ��8�¶�
		case 2120://����Ŀ��9�¶�
		case 2122://����Ŀ��10�¶�
		case 2124://����Ŀ��11�¶�
		case 2126://����Ŀ��12�¶�
		case 2128://����Ŀ��13�¶�
		case 2130://����Ŀ��14�¶�
		case 2132://����Ŀ��15�¶�
		case 2134://����Ŀ��16�¶�
			if(*_RegIdex+2 > _RegNum)//��Ч���ݴ��ڴ洢�ռ䣬��������
			{
				Error = 1;
				break;
			}
            _RegData += *_RegIdex;//���ݵ�ַ����
			Temp32 = (uint32_t)(*_RegData) + ((uint32_t)(*(_RegData+1))<<16);
			(float) Temp32;//�������ú���ִ��
            //�������ú���ִ��
			*_RegIdex += 2;//�����������
			break;
		default:
			Error = 1;
			break;
	}
  return Error;
}
/**
  * ��������: ��ЧN���Ĵ�����ֵ
  * �������: _AddrOffset:ƫ�Ƶ�ַ,_RegNum:�Ĵ�������,_RegAddr:�Ĵ�����ַ
  * �� �� ֵ: �쳣��:0������
  * ˵    ��: ��Ч_RegNum����Ȧ���ݣ�����Ч�쳣����������
  */
uint8_t MB_EF_NReg(uint16_t* _AddrOffset,uint16_t _RegNum , uint16_t _RegAddr)
{
  uint8_t Error = 1;
  uint16_t i;
  for(i=0;i<_RegNum;)
  {
    Error = MB_EF_RegVal(_RegAddr, _RegNum, _AddrOffset, &i);
    if(Error)//��Ч�쳣�����˳�
        break;
  }
  return Error;
}
/**
  * ��������: 06H������Ĵ�����Ч����
  * �������: _RegAddr�Ĵ�����ַ, _AddrOffset�Ĵ�����ֵ�洢��ַ
  * �� �� ֵ: �쳣��:0������
  * ˵    ��: ʹ�Ĵ����е���ֵ��Ч
  */
uint8_t MB_Effect_06H(uint16_t _RegAddr, uint16_t* _AddrOffset)
{
	return MB_EF_NReg(_AddrOffset, 1, _RegAddr);
}

/**
  * ��������: 10H������Ĵ�����Ч����
  * �������: _RegAddr�Ĵ�����ַ, _RegNum�Ĵ�������, _AddrOffset�Ĵ�����ֵ�洢��ַ
  * �� �� ֵ: �쳣��:0������
  * ˵    ��: ʹ�Ĵ����е���ֵ��Ч
  */
uint8_t MB_Effect_10H(uint16_t _RegAddr, uint16_t _RegNum, uint16_t* _AddrOffset)
{
	return MB_EF_NReg(_AddrOffset, _RegNum, _RegAddr);
}

/**
  * ��������: ModBus��ɢ���ͼĴ�����Ч����
  * �������: Э�����ݵ�Ԫָ��
  * �� �� ֵ: �쳣��:0������
  * ˵    ��: �Ĵ����е���ֵ��Ч
  */
uint8_t MB_Effect(PDUData_TypeDef *_PduData)
{
	uint8_t Error;
    switch(_PduData->Code)
	{
		case FUN_CODE_01H:
			//����صĺ�����������������ʵ��
            Error = 0;
			break;
		case FUN_CODE_02H:
			//����صĺ�����������������ʵ��
            Error = 0;
			break;
		case FUN_CODE_03H:
			//����صĺ�����������������ʵ��
            Error = 0;
			break;
		case FUN_CODE_04H:
			//����صĺ�����������������ʵ��
            Error = 0;
			break;
		case FUN_CODE_05H:
			//д��صĺ�����������������ʵ��
			Error = MB_Effect_05H(_PduData->Addr,(uint8_t *) _PduData->PtrCoilOffset);
			break;
		case FUN_CODE_06H:
			//д��صĺ�����������������ʵ��
			Error = MB_Effect_06H(_PduData->Addr, (uint16_t *) _PduData->PtrHoldingOffset);
			break;
		case FUN_CODE_0FH:
			//д��صĺ�����������������ʵ��
			Error = MB_Effect_0FH(_PduData->Addr, _PduData->Num,(uint8_t *) _PduData->PtrCoilOffset);
			break;
		case FUN_CODE_10H:
			//д��صĺ�����������������ʵ��
			Error = MB_Effect_10H(_PduData->Addr, _PduData->Num,(uint16_t *) _PduData->PtrHoldingOffset);
			break;
        default:
            Error = 1;
            break;
	}
    return Error;
}

/**
  * ��������: ������Ȧ��ֵ
	* �������: _CoilAddr:��Ȧ��ַ, _CoilNum:��Ȧ����, _CoilBuf:��Ȧ����ָ��, _CoilIdex����Ȧ����ָ��, _CellCoilNum: ��Ԫ������Ȧ����
  * �� �� ֵ: �쳣��:04H��NONE
  * ˵    ��: ������Ȧ��ַ��������������Ȧ��ֵ������_CoilBuf��ָ��Ŀռ���д��
  */
uint8_t MB_MK_CoilVal(uint16_t _CoilAddr, uint16_t _CoilNum, uint8_t *_CoilBuf, uint16_t *_CoilIdex, uint16_t *_CellCoilNum)
{
//  uint16_t i = 0;
  uint8_t Value = 0;

//	�ɸ���_CoilNum���Ż���ȡ����δʹ�ã���Ŀǰ����Ϊ��С����
	switch(_CoilAddr + *_CoilIdex)
	{
		case 0://�������е�
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 1://���1���е�
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 2://���2���е�
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 3://���3���е�
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 4://���4���е�
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 5://������
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			//������Ӧ״̬��ֵ
            if(HAL_GPIO_ReadPin(LAMP_GPIO_PORT,LAMP_GPIO_PIN))
                *_CoilBuf = 1;
            else
                *_CoilBuf = 0;
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 6://����������
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			//������Ӧ״̬��ֵ
            if(HAL_GPIO_ReadPin(BEEP_GPIO_PORT,BEEP_GPIO_PIN))
                *_CoilBuf = 1;
            else
                *_CoilBuf = 0;
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 100://���ȹ�1
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 101://���ȹ�2
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 102://���ȹ�3
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 103://���ȹ�4
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 104://���ȹ�5
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 105://���ȹ�6
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 106://���ȹ�7
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 107://���ȹ�8
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 108://���ȹ�9
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 109://���ȹ�10
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 110://���ȹ�11
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 111://���ȹ�12
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 112://���ȹ�13
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 113://���ȹ�14
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 114://���ȹ�15
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 115://���ȹ�16
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 132://�����1
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 133://�����2
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 134://�����3
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 135://�����4
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 136://�����5
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 137://�����6
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 138://�����7
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 139://�����8
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 140://�����9
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 141://�����10
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 142://�����11
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 143://�����12
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 144://�����13
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 145://�����14
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 146://�����15
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 147://�����16
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;		
		case 200://Һ��1
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			//������Ӧ״̬��ֵ
            if(HAL_GPIO_ReadPin(SIG_SLOT_GPIO_PORT,SIG_SLOT_GPIO_PIN))
                *_CoilBuf = 1;
            else
                *_CoilBuf = 0;
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 201://Һ��2
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 202://Һ��3
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 203://Һ��4
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 204://Һ��5
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 205://Һ��6
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 206://Һ��7
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 207://Һ��8
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 208://Һ��9
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 209://Һ��10
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 210://Һ��11
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 211://Һ��12
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 212://Һ��13
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 213://Һ��14
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 214://Һ��15
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 215://Һ��16
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 300://����
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			//������Ӧ״̬��ֵ
            if(HAL_GPIO_ReadPin(SIG_DOOR_GPIO_PORT,SIG_DOOR_GPIO_PIN))
                *_CoilBuf = 1;
            else
                *_CoilBuf = 0;
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 400://X��λ��NEG
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 401://X��λ��ORI
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 402://X��λ��POS
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 403://Y��λ��NEG
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 404://Y��λ��ORI
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 405://Y��λ��POS
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 406://Z1��λ��NEG
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 407://Z1��λ��ORI
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 408://Z1��λ��POS
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 409://P1��λ��NEG
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 410://P1��λ��ORI
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 411://P1��λ��POS
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 412://Z2��λ��NEG
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 413://Z2��λ��ORI
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 414://Z2��λ��POS
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 415://P2��λ��NEG
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 416://P2��λ��ORI
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 417://P2��λ��POS
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 500://X����EN
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 501://X����STEP
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 502://X����DIR
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 503://X����MD0
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 504://X����MD1
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 505://X����MD2
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 506://X����DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_CoilBuf+0) = 0;//������Ӧ״̬��ֵ
			*(_CoilBuf+1) = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 2;//�����������
			*_CellCoilNum = 2;//��Ԫ������Ȧ����
			break;
		case 508://X����RESET
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 509://X����SLEEP
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 510://X����FAULT
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 511://X����HOME
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 512://Y����EN
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 513://Y����STEP
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 514://Y����DIR
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 515://Y����MD0
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 516://Y����MD1
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 517://Y����MD2
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 518://Y����DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_CoilBuf+0) = 0;//������Ӧ״̬��ֵ
			*(_CoilBuf+1) = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 2;//�����������
			*_CellCoilNum = 2;//��Ԫ������Ȧ����
			break;
		case 520://Y����RESET
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 521://Y����SLEEP
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 522://Y����FAULT
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 523://Y����HOME
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 524://Z1����EN
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 525://Z1����STEP
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 526://Z1����DIR
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 527://Z1����MD0
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 528://Z1����MD1
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 529://Z1����MD2
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 530://Z1����DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_CoilBuf+0) = 0;//������Ӧ״̬��ֵ
			*(_CoilBuf+1) = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 2;//�����������
			*_CellCoilNum = 2;//��Ԫ������Ȧ����
			break;
		case 532://Z1����RESET
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 533://Z1����SLEEP
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 534://Z1����FAULT
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 535://Z1����HOME
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 536://P1����EN
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 537://P1����STEP
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 538://P1����DIR
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 539://P1����MD0
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 540://P1����MD1
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 541://P1����MD2
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 542://P1����DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_CoilBuf+0) = 0;//������Ӧ״̬��ֵ
			*(_CoilBuf+1) = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 2;//�����������
			*_CellCoilNum = 2;//��Ԫ������Ȧ����
			break;
		case 544://P1����RESET
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 545://P1����SLEEP
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 546://P1����FAULT
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 547://P1����HOME
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 548://Z2����EN
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 549://Z2����STEP
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 550://Z2����DIR
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 551://Z2����MD0
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 552://Z2����MD1
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 553://Z2����MD2
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 554://Z2����DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_CoilBuf+0) = 0;//������Ӧ״̬��ֵ
			*(_CoilBuf+1) = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 2;//�����������
			*_CellCoilNum = 2;//��Ԫ������Ȧ����
			break;
		case 556://Z2����RESET
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 557://Z2����SLEEP
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 558://Z2����FAULT
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 559://Z2����HOME
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 560://P2����EN
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 561://P2����STEP
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 562://P2����DIR
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 563://P2����MD0
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 564://P2����MD1
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 565://P2����MD2
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 566://P2����DECAY(0)(1)
			if(*_CoilIdex+2 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_CoilBuf+0) = 0;//������Ӧ״̬��ֵ
			*(_CoilBuf+1) = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 2;//�����������
			*_CellCoilNum = 2;//��Ԫ������Ȧ����
			break;
		case 568://P2����RESET
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 569://P2����SLEEP
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 570://P2����FAULT
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		case 571://P2����HOME
			if(*_CoilIdex+1 > _CoilNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_CoilBuf = 0;//������Ӧ״̬��ֵ
			*_CoilIdex += 1;//�����������
			*_CellCoilNum = 1;//��Ԫ������Ȧ����
			break;
		default:
			Value = 1;
			break;
	}
  return Value;
}
/**
  * ��������: ����N����Ȧ������
  * �������: _AddrOffset:ƫ�Ƶ�ַ,_CoilNum:��Ȧ����,_CoilAddr:��Ȧ��ַ
  * �� �� ֵ: �쳣��:04H��NONE
  * ˵    ��: ����_CoilNum����Ȧ���ݣ�����_AddrOffset��ָ��Ŀռ���д�룬�������쳣����������
  */
uint8_t MB_UP_NCoil(uint8_t* _AddrOffset,uint16_t _CoilNum , uint16_t _CoilAddr)
{
  uint16_t i = 0,j = 0;
	uint16_t CellCoilNum;//��Ԫ������Ȧ�����������Ż�����δʹ��
  uint8_t CoilBuf[128];//��ʱһ���ֽڶ�Ӧһ����ɢ��
  if((_AddrOffset - PduData.PtrCoilbase)==0xffff)//��Ȧ��ַ����������ʵ�ʶ���ķ�Χ
    return EX_CODE_04H;
  for(i=0;i<_CoilNum;)
  {
		if(MB_MK_CoilVal(_CoilAddr, _CoilNum, CoilBuf, &i, &CellCoilNum))//�����쳣�����˳�
      return EX_CODE_04H;
    for(j=0;j<CellCoilNum;j++)
			*_AddrOffset++ = CoilBuf[j];
  }
  return EX_CODE_NONE;
}
/**
  * ��������: ���ɼĴ�����ֵ
	* �������: _RegAddr:�Ĵ�����ַ, _RegNum:�Ĵ�������, _RegBuf:�Ĵ�������ָ��, _RegIdex���Ĵ�������ָ��, _CellRegNum: ��Ԫ���ڼĴ�������
  * �� �� ֵ: �쳣��:04H��NONE
  * ˵    ��: ���ݼĴ�����ַ�����������ɼĴ�����ֵ������_RegBuf��ָ��Ŀռ���д��
  */
uint8_t MB_MK_RegVal(uint16_t _RegAddr, uint16_t _RegNum, uint16_t *_RegBuf, uint16_t *_RegIdex, uint16_t *_CellRegNum)
{
  uint16_t i = 0;
  uint8_t Value = 0;
    uint32_t RegTemp;

//	�ɸ���_RegNum���Ż���ȡ����δʹ�ã���Ŀǰ����Ϊ��С����
	switch(_RegAddr + *_RegIdex)
	{
		case 0://ԭ����У׼
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			for(i=0;i<2;i++)//һ����Ԫ�����4���Ĵ���
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
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2://�ع�ԭ��
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
            *_RegBuf = OriginCmdLast%65536;//��Ӧ������ֵ
            *(_RegBuf+1) = OriginCmdLast/65536;//��Ӧ������ֵ
            *_RegIdex += 2;//������ż�1
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 4://��ʱ�ȴ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
            *_RegBuf = Timer[TMR_NUM_DELAY].value%65536;//��Ӧ������ֵ
            *(_RegBuf+1) = Timer[TMR_NUM_DELAY].value/65536;//��Ӧ������ֵ
            *_RegIdex += 2;//������ż�1
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 6://��ʱ�ȴ�ʣ��
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			if(!TMR_Getleave(&Timer[TMR_NUM_DELAY], &RegTemp))
            {
                *_RegBuf = RegTemp%65536;//��Ӧ������ֵ
                *(_RegBuf+1) = RegTemp/65536;//��Ӧ������ֵ
            }
            else
            {
               *_RegBuf = 0xFFFF;//��ֵ�����ܴﵽ��Ϊ�쳣ֵ 
               *(_RegBuf+1) = 0xFFFF;//��ֵ�����ܴﵽ��Ϊ�쳣ֵ 
            }
			*_RegIdex += 2;//������ż�1
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 8://����״̬����
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 10://ָ��ִ��״̬
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 12://�������״̬
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = 0;
			*(_RegBuf+1) = 0;
//            if(SMotor[SMX].MotorRuning)// ��ǰ������ת��״̬
//				*_RegBuf |= ((uint16_t) 1)<<0;//��Ӧ������ֵ
//			if(SMotor[SMY].MotorRuning)// ��ǰ������ת��״̬
//				*_RegBuf |= ((uint16_t) 1)<<1;//��Ӧ������ֵ
            if(SMotor[SMZ1].MotorRuning)// ��ǰ������ת��״̬
				*_RegBuf |= ((uint16_t) 1)<<2;//��Ӧ������ֵ
			if(SMotor[SMP1].MotorRuning)// ��ǰ������ת��״̬
				*_RegBuf |= ((uint16_t) 1)<<3;//��Ӧ������ֵ
            if(SMotor[SMZ2].MotorRuning)// ��ǰ������ת��״̬
				*_RegBuf |= ((uint16_t) 1)<<4;//��Ӧ������ֵ
			if(SMotor[SMP2].MotorRuning)// ��ǰ������ת��״̬
				*_RegBuf |= ((uint16_t) 1)<<5;//��Ӧ������ֵ
			*_RegIdex += 2;//������ż�1
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 100://X������
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM1].Coord%65536;//������Ͱ���
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM1].Coord/65536;//������߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 102://Y������
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM2].Coord%65536;//������Ͱ���
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM2].Coord/65536;//������߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 104://Z1������
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ1].Coord)%65536;//������Ͱ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ1].Coord)/65536;//������߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 106://P1������
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP1].Coord)%65536;//������Ͱ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP1].Coord)/65536;//������߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 108://Z2������
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ2].Coord)%65536;//������Ͱ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ2].Coord)/65536;//������߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 110://P2������
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP2].Coord)%65536;//������Ͱ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP2].Coord)/65536;//������߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 400://X������ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM1].Lim_Speed%65536;//���ݵͰ���
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM1].Lim_Speed/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 402://X�������ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM1].Craml_Speed%65536;//���ݵͰ���
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM1].Craml_Speed/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 404://X����ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM1].Acc_Time%65536;//���ݵͰ���
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM1].Acc_Time/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 406://X����ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM1].Dec_Time%65536;//���ݵͰ���
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM1].Dec_Time/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 408://X��ÿȦ����
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM1].Distance_Rev%65536;//���ݵͰ���
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM1].Distance_Rev/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 410://Y������ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM2].Lim_Speed%65536;//���ݵͰ���
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM2].Lim_Speed/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 412://Y�������ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM2].Craml_Speed%65536;//���ݵͰ���
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM2].Craml_Speed/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 414://Y����ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM2].Acc_Time%65536;//���ݵͰ���
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM2].Acc_Time/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 416://Y����ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM2].Dec_Time%65536;//���ݵͰ���
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM2].Dec_Time/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 418://Y��ÿȦ����
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = (uint32_t) mrd_SMotor[SM2].Distance_Rev%65536;//���ݵͰ���
			*(_RegBuf+1) = (uint32_t) mrd_SMotor[SM2].Distance_Rev/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 420://Z1������ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ1].Lim_Speed)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ1].Lim_Speed)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 422://Z1�������ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ1].Craml_Speed)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ1].Craml_Speed)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 424://Z1����ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ1].Acc_Time)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ1].Acc_Time)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 426://Z1����ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ1].Dec_Time)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ1].Dec_Time)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 428://Z1��ÿȦ����
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ1].Distance_Rev)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ1].Distance_Rev)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 430://P1������ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP1].Lim_Speed)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP1].Lim_Speed)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 432://P1�������ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP1].Craml_Speed)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP1].Craml_Speed)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 434://P1����ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP1].Acc_Time)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP1].Acc_Time)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 436://P1����ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP1].Dec_Time)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP1].Dec_Time)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 438://P1��ÿȦ����
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP1].Distance_Rev)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP1].Distance_Rev)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 440://Z2������ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ2].Lim_Speed)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ2].Lim_Speed)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 442://Z2�������ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ2].Craml_Speed)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ2].Craml_Speed)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 444://Z2����ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ2].Acc_Time)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ2].Acc_Time)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 446://Z2����ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ2].Dec_Time)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ2].Dec_Time)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 448://Z2��ÿȦ����
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMZ2].Distance_Rev)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMZ2].Distance_Rev)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 450://P2������ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP2].Lim_Speed)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP2].Lim_Speed)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 452://P2�������ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP2].Craml_Speed)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP2].Craml_Speed)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 454://P2����ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP2].Acc_Time)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP2].Acc_Time)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 456://P2����ٶ�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP2].Dec_Time)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP2].Dec_Time)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 458://P2��ÿȦ����
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&mrd_SMotor[SMP2].Distance_Rev)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&mrd_SMotor[SMP2].Distance_Rev)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 1000://��ܻ�ȡ
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = 0;
      if(HAL_GPIO_ReadPin(LIMIT2_NEG_GPIO_PORT,LIMIT2_NEG_PIN) == LIMIT2_NEG_ACTIVE_LEVEL)//��Ӧ������ֵ
        *_RegBuf |= (uint16_t) 1<<0;
			if(HAL_GPIO_ReadPin(LIMIT4_NEG_GPIO_PORT,LIMIT4_NEG_PIN) == LIMIT4_NEG_ACTIVE_LEVEL)//��Ӧ������ֵ
        *_RegBuf |= (uint16_t) 1<<1;
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 1006://P1����������
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&prd_SMotor[PIP1].Volume_Distance)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&prd_SMotor[PIP1].Volume_Distance)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 1008://P2����������
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = *((uint32_t *)&prd_SMotor[PIP2].Volume_Distance)%65536;//���ݵͰ���
			*(_RegBuf+1) = *((uint32_t *)&prd_SMotor[PIP2].Volume_Distance)/65536;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2000://��ȡĿ��1�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2002://��ȡĿ��2�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2004://��ȡĿ��3�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2006://��ȡĿ��4�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2008://��ȡĿ��5�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2010://��ȡĿ��6�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2012://��ȡĿ��7�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2014://��ȡĿ��8�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2016://��ȡĿ��9�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2018://��ȡĿ��10�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2020://��ȡĿ��11�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2022://��ȡĿ��12�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2024://��ȡĿ��13�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2026://��ȡĿ��14�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2028://��ȡĿ��15�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2030://��ȡĿ��16�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2100://�¿�ȡ��
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 2101://�¿ر���
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 2102://�¿ؼ���
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 2103://�¿�����
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 2104://����Ŀ��1�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2106://����Ŀ��2�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2108://����Ŀ��3�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2110://����Ŀ��4�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2112://����Ŀ��5�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2114://����Ŀ��6�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2116://����Ŀ��7�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2118://����Ŀ��8�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2120://����Ŀ��9�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2122://����Ŀ��10�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2124://����Ŀ��11�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2126://����Ŀ��12�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2128://����Ŀ��13�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2130://����Ŀ��14�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2132://����Ŀ��15�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 2134://����Ŀ��16�¶�
			if(*_RegIdex+2 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*(_RegBuf+0) = 0;//���ݵͰ���
			*(_RegBuf+1) = 0;//���ݸ߰���
			*_RegIdex += 2;//������ż�2
			*_CellRegNum = 2;//��Ԫ���ڼĴ�������
			break;
		case 9000://�쳣����
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 9002://�������
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 9003://������䱨��
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 9004://�߽�����
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 9005://λ�ñ���������
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 9006://Һ�۱���
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 9007://ȱҺ����
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 9008://���±���
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		case 9009://���±���
			if(*_RegIdex+1 > _RegNum)//�������ݴ��ڴ洢�ռ䣬��������
			{
				Value = 1;
				break;
			}
			*_RegBuf = *_RegBuf;//��Ӧ������ֵ
			*_RegIdex += 1;//������ż�1
			*_CellRegNum = 1;//��Ԫ���ڼĴ�������
			break;
		default:
			Value = 1;
			break;
	}
  return Value;
}
/**
  * ��������: ����N���Ĵ���
  * �������: _AddrOffset:ƫ�Ƶ�ַ,_RegNum:�Ĵ�������,_RegAddr:�Ĵ�����ַ
  * �� �� ֵ: �쳣��:04H��NONE
  * ˵    ��: ����_RegNum���Ĵ������ݣ�����_AddrOffset��ָ��Ŀռ���д�룬�������쳣����������
  */
uint8_t MB_UP_NReg(uint16_t* _AddrOffset,uint16_t _RegNum , uint16_t _RegAddr)
{
  uint16_t i = 0,j = 0;
	uint16_t CellRegNum;//��Ԫ���ڼĴ�������
  uint16_t RegBuf[4];
  if((_AddrOffset - PduData.PtrHoldingbase)==0xffff)//�Ĵ�����ַ����������ʵ�ʶ���ķ�Χ
    return EX_CODE_04H;
  for(i=0;i<_RegNum;)
  {
		if(MB_MK_RegVal(_RegAddr, _RegNum, RegBuf, &i, &CellRegNum))//�����쳣�����˳�
      return EX_CODE_04H;
    for(j=0;j<CellRegNum;j++)
			*_AddrOffset++ = RegBuf[j];
  }
  return EX_CODE_NONE;
}
/**
  * ��������: д,��N����Ȧ
  * �������: _AddrOffset:ƫ�Ƶ�ַ,_CoilNum:��Ȧ����,_Datebuf:����ָ��
  * �� �� ֵ: �쳣��:04H��NONE
  * ˵    ��: ��_AddrOffset��ָ��Ŀռ���д��_RegNum����Ȧ����,���Ҷ�ȡ��֤�Ƿ�д��ɹ�
  */
uint8_t MB_WR_NCoil(uint8_t* _AddrOffset,uint16_t _RegNum , uint8_t* _Datebuf)
{
  uint16_t i = 0,j ;
  uint16_t iquo,irem;//�̺�����
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
/* ��ȡ��֤д���Ƿ�ɹ� */
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
  * ��������: д,��N���Ĵ���
  * �������: _AddrOffset:ƫ�Ƶ�ַ,_RegNum:�Ĵ�������,_Datebuf:����ָ��
  * �� �� ֵ: �쳣��:04H��NONE
  * ˵    ��: ��_AddrOffset��ָ��Ŀռ���д��_RegNum*2������,���Ҷ�ȡ��֤�Ƿ�д��ɹ�
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
  /* ��ȡ��֤д���Ƿ�ɹ� */
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
  * ��������: �жϵ�ַ�Ƿ����Э�鷶Χ
  * �������: _Addr:��ʼ��ַ,_RegNum:�Ĵ�������,_FunCode:������
  * �� �� ֵ: �쳣��:02H��NONE
  * ˵    ��: ��ַ��Χ��0x0000~0xFFFF,�ɲ����Ŀռ䷶Χ���ܳ����������
  */
uint8_t MB_JudgeAddr(uint16_t _Addr,uint16_t _RegNum)
{
  uint8_t Excode = EX_CODE_NONE;
  /* ��ַ+�Ĵ����������ܳ���0xFFFF */
  if( ((uint32_t)_RegNum+(uint32_t)_Addr) > (uint32_t)0xFFFF)
  {
    Excode = EX_CODE_02H;// �쳣�� 02H
  }
  return Excode;
}
/**
  * ��������: �жϲ������������Ƿ����Э�鷶Χ
  * �������: _RegNum:�Ĵ�������,_FunCode:������,_ByteNum:�ֽ�����
  * �� �� ֵ: �쳣��:03��NONE
  * ˵    ��: �Կɲ��������ڴ�ռ�Ĺ�������Ҫ��֤�����ĵ�ַ�Ƿ���Ϸ�Χ
  */
uint8_t MB_JudgeNum(uint16_t _RegNum,uint8_t _FunCode,uint16_t _ByteNum)
{
  uint8_t Excode = EX_CODE_NONE;
  uint16_t _CoilNum = _RegNum; // ��Ȧ(��ɢ��)������
  switch(_FunCode)
  {
    case FUN_CODE_01H: 
    case FUN_CODE_02H:
      if( (_CoilNum<0x0001) || (_CoilNum>0x07D0))
        Excode = EX_CODE_03H;// �쳣��03H;
      break;
    case FUN_CODE_03H:
    case FUN_CODE_04H:
      if( (_RegNum<0x0001) || (_RegNum>0x007D))
        Excode = EX_CODE_03H;// �쳣��03H;      
      break;
    case FUN_CODE_0FH:
      if( (_RegNum<0x0001) || (_RegNum>0x07B0))
        Excode = EX_CODE_03H;// �쳣��03H
      if( _ByteNum != (_RegNum/8 + ((_RegNum%8)?1:0)))
        Excode = EX_CODE_03H;// �쳣��03H
      break;
    case FUN_CODE_10H:
      if( (_RegNum<0x0001) || (_RegNum>0x007B))
        Excode = EX_CODE_03H;// �쳣��03H
      if( _ByteNum != (_RegNum<<1))
        Excode = EX_CODE_03H;// �쳣��03H
      break;
  }
  return Excode;
}
/**
  * ��������: ��ȡ��ɢ���
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_CoilNum:��Ȧ����
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ��ȡ��ɢ���,�������Tx_Buf
  */
uint16_t MB_RSP_01H(uint16_t _TxCount,uint8_t *_AddrOffset ,uint16_t _CoilNum)
{
  uint16_t ByteNum = 0;
  uint16_t i = 0;
  uint16_t r = 0 ;
  ByteNum = _CoilNum/8;
  /* �����������,��Ҫ�������λ��0 */
  r = _CoilNum%8;
  if(r != 0)
  {
    ByteNum += 1; //�ֽ���+1
    Tx_Buf[_TxCount++] = ByteNum;  
    
    /* �ں�������������¶�ȡ��Ȧ(bit)��״̬ */
    for(i=0;i<ByteNum-1;i++)
    {
      Tx_Buf[_TxCount] = 0x00;
      /* ÿ8����Ȧ(bit)һ��ѭ��,��ȡ8����Ȧ��״̬������һ��Byte*/
      for(uint8_t j=0;j<8;j++)
      {
        /* �����1,��byte��Ӧλ(bit)��1 */
        if(*(_AddrOffset++))
        {
          /* �������൱�ڽ�һ��Byte����һ����Ȧ(bit)ӳ�䵽һ��Byte��
           * ��8��Byte���һ��Byte��8��bit
           */
          Tx_Buf[_TxCount] |= (0x01<<j);
        }
      }
      _TxCount++;
    }
    /* �������������һ���ֽ�,�����bit����Ϊ0 */
    Tx_Buf[_TxCount] = 0x00;
    for(uint8_t j=0;j<r;j++)
    {
      if(*(_AddrOffset++))
      {
        Tx_Buf[_TxCount] |= (0x01<<j);// ��ȡ����
      }
    }
    _TxCount++;
  }
  /* �������r==0,˵����Ҫ��ȡ����Ȧ����(bits)�պ���Byte�������� */
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
          Tx_Buf[_TxCount] |= (0x01<<j);// ��ȡ����
        }
      }
      _TxCount++;
    }
  }
  return _TxCount;
}
/**
  * ��������: ��ȡ���ּĴ���
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_RegNum:�Ĵ�������
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ��ȡ���ּĴ���������,�������Tx_Buf
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
  * ��������: ����05H��������䷢�ͻ�����
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_AddrAbs:���Ե�ֵַ
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ���Tx_Buf
  */
uint8_t MB_RSP_05H(uint16_t _TxCount,uint16_t _AddrOffset ,uint8_t *_AddrAbs)
{
  /* ����ֵַ */
  Tx_Buf[_TxCount++] = _AddrOffset>>8;
  Tx_Buf[_TxCount++] = _AddrOffset;
  /* ������ֵ */
  if((*_AddrAbs) == 1)
    Tx_Buf[_TxCount++] = 0xFF;
  else Tx_Buf[_TxCount++] = 0x00;
  Tx_Buf[_TxCount++] = 0x00;
  return _TxCount;
}
/**
  * ��������: ����06H��������䷢�ͻ�����
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_AddrAbs:���Ե�ֵַ
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ���Tx_Buf
  */
uint8_t MB_RSP_06H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t *_AddrAbs)
{
  /* ����ֵַ */
  Tx_Buf[_TxCount++] = _AddrOffset>>8;
  Tx_Buf[_TxCount++] = _AddrOffset;
  /* ������ֵ */
  Tx_Buf[_TxCount++] = (*_AddrAbs)>>8;
  Tx_Buf[_TxCount++] = *_AddrAbs;
  return _TxCount;
}
/**
  * ��������: ����10H��������䷢�ͻ�����
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_ByteNum:�ֽ�����
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ���Tx_Buf
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
  * ��������: �쳣��Ӧ
  * �������: _FunCode :�����쳣�Ĺ�����,_ExCode:�쳣��
  * �� �� ֵ: ��
  * ˵    ��: ��ͨ������֡�����쳣ʱ,�����쳣��Ӧ
  */
void MB_Exception_RSP(uint8_t _FunCode,uint8_t _ExCode)
{
  uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = MB_SLAVEADDR;		    /* ��վ��ַ */
	Tx_Buf[TxCount++] = _FunCode|0x80;		  /* ������ + 0x80*/	
	Tx_Buf[TxCount++] = _ExCode ;	          /* �쳣��*/
	
  crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  UART_Tx((uint8_t*)Tx_Buf, TxCount);
}
/**
  * ��������: ������Ӧ
  * �������: _FunCode :������
  * �� �� ֵ: ��
  * ˵    ��: ��ͨ������֡û���쳣ʱ���ҳɹ�ִ��֮��,������Ӧ����֡
  */
void MB_RSP(uint8_t _FunCode)
{
  uint16_t TxCount = 0;
  uint16_t crc = 0;	Tx_Buf[TxCount++] = MB_SLAVEADDR;		 /* ��վ��ַ */
	Tx_Buf[TxCount++] = _FunCode;        /* ������   */	
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
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  UART_Tx((uint8_t*)Tx_Buf, TxCount);
}

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
