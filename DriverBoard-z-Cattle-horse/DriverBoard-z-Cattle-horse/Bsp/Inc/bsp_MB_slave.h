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

#ifndef __BSP_MB_SLAVE_H__
#define __BSP_MB_SLAVE_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef struct {
  __IO uint8_t  Code ;  	        // ������
  __IO uint8_t byteNums; 	        // �ֽ���
  __IO uint16_t Addr ;            // �����ڴ����ʼ��ַ
  __IO uint16_t Num; 	            // �Ĵ���������Ȧ������
  __IO uint16_t _CRC;       	      // CRCУ����
  __IO uint8_t *ValueReg; 	      // 10H�����������
  __IO uint8_t *PtrCoilbase;		  // Coil��Input�ڴ��׵�ַ
  __IO uint8_t *PtrCoilOffset;    // Coil��Inputƫ���ڴ��׵�ַ
  __IO uint16_t *PtrHoldingbase;  // HoldingReg�ڴ��׵�ַ
  __IO uint16_t *PtrHoldingOffset;// HoldingReg�ڴ��׵�ַ
}PDUData_TypeDef;

/* �궨�� --------------------------------------------------------------------*/
#define MB_SLAVEADDR            0x0002
#define MB_ALLSLAVEADDR         0x00FF

#define FUN_CODE_01H            0x01  // ������01H 
#define FUN_CODE_02H            0x02  // ������02H
#define FUN_CODE_03H            0x03  // ������03H
#define FUN_CODE_04H            0x04  // ������04H
#define FUN_CODE_05H            0x05  // ������05H
#define FUN_CODE_06H            0x06  // ������06H
#define FUN_CODE_0FH            0x0F  // ������06H
#define FUN_CODE_10H            0x10  // ������10H

/* ��������֧�ֵĹ�����,��Ҫ����¹����뻹��Ҫ��.c�ļ�������� */
#define IS_NOT_FUNCODE(code)  (!((code == FUN_CODE_01H)||\
                                 (code == FUN_CODE_02H)||\
                                 (code == FUN_CODE_03H)||\
                                 (code == FUN_CODE_04H)||\
                                 (code == FUN_CODE_05H)||\
                                 (code == FUN_CODE_06H)||\
                                 (code == FUN_CODE_0FH)||\
                                 (code == FUN_CODE_10H)))

#define EX_CODE_NONE           0x00  // �쳣�� ���쳣
#define EX_CODE_01H            0x01  // �쳣��
#define EX_CODE_02H            0x02  // �쳣��
#define EX_CODE_03H            0x03  // �쳣��
#define EX_CODE_04H            0x04  // �쳣��
/* ��չ���� ------------------------------------------------------------------*/
extern PDUData_TypeDef PduData;
/* �������� ------------------------------------------------------------------*/
uint16_t MB_CRC16(uint8_t *pushMsg,uint8_t usDataLen);
void MB_Parse_Data(void);
void MB_WriteNumHoldingReg_10H(uint8_t _addr, uint16_t _reg, uint16_t _num,uint8_t *_databuf);
uint8_t MB_Analyze_Execute(void );
uint8_t MB_Effect(PDUData_TypeDef *_PduData);
uint8_t MB_JudgeNum(uint16_t _Num,uint8_t _FunCode,uint16_t ByteNum);
uint8_t MB_JudgeAddr(uint16_t _Addr,uint16_t _Num);
void MB_Exception_RSP(uint8_t _FunCode,uint8_t _ExCode);
void MB_RSP(uint8_t _FunCode);
#endif /* __BSP_MB_SLAVE_H__ */

/********** (C) COPYRIGHT Superior Synthesis Biotechnology *****END OF FILE****/
