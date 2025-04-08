/*
 * s93c46a.c
 *
 *  Created on: Jan 5, 2025
 *      Author: wataoxp
 */
#include "s93c46.h"

static uint8_t CheckPosition(uint16_t val);
static void SendBit(uint8_t bit);
static void SendOpCode(uint8_t code);
static void WriteAddress(uint8_t add);
static void WriteData(uint16_t data);
static uint16_t ReadData(void);
static void Verify(void);

static GPIO_TypeDef* CS_Port;
static GPIO_TypeDef* SK_Port;
static GPIO_TypeDef* DI_Port;
static GPIO_TypeDef* DO_Port;

static uint16_t Select;
static uint16_t Clock;
static uint16_t Output;
static uint16_t Input;

static uint8_t DIshift;
static uint8_t DOshift;

void SetHandle(S93C46_Typedef* init)
{
	CS_Port = init->PortCS;
	SK_Port = init->PortSK;
	DI_Port = init->PortDI;
	DO_Port = init->PortDO;

	Select = init->CS;
	Clock = init->SK;
	Output = init->DI;
	Input = init->DO;

	DIshift = CheckPosition(init->DI);
	DOshift = CheckPosition(init->DO);
}
static uint8_t CheckPosition(uint16_t val)
{
	uint8_t i;
	for(i = 0; i < 16;i++)
	{
		if(((val >> i) & 0x01) == 1)
		{
			break;
		}
	}
	return i;
}
static inline void DummyClock(void)
{
	WRITE_REG(DI_Port->BRR,Output);
	WRITE_REG(SK_Port->BSRR,Clock);
	Delay(SK_TIME);
	WRITE_REG(SK_Port->BRR,Clock);
	Delay(SK_TIME);
}
static inline void StartBit(void)					//CSをHiにした後、DIをHiにすることでスタートビットになる。
{
	SendBit(true);
}
static inline void EnableChip(void)
{
	WRITE_REG(CS_Port->BSRR,Select);
	Delay(1);										//wait tCSS
}
static inline void DisableChip(void)
{
	WRITE_REG(CS_Port->BRR,Select);
	Delay(1);										//wait tCDS
}
static void SendBit(uint8_t bit)
{
	WRITE_REG(DI_Port->BSRR,(bit << DIshift));			//データをセット
	Delay(SK_TIME);
	WRITE_REG(SK_Port->BSRR,Clock);						//MX_GPIO_Initですべてのピンは初期状態がLow
	Delay(SK_TIME);
	WRITE_REG(DI_Port->BRR,Output);						//アイドル時はDI、SK共にLowにする
	WRITE_REG(SK_Port->BRR,Clock);
}
static void SendOpCode(uint8_t code)
{
	for(int8_t i = 7; i >= 0; i--)
	{
		SendBit((code >> i) & 0x01);
	}
}
static uint16_t ReadData(void)
{
	uint16_t ret = 0;

	for(uint8_t i = 0; i < 16; i++)
	{
		ret <<= 1;
		Delay(SK_TIME);
		WRITE_REG(SK_Port->BSRR,Clock);
		Delay(SK_READ_TIME);

		ret |= (DO_Port->IDR & Input) >> DOshift;
		WRITE_REG(SK_Port->BRR,Clock);
	}
	return ret;
}
static void WriteAddress(uint8_t add)			//いずれもMSBから送信
{
	for(int8_t i = 7; i >= 0; i--)
	{
		SendBit((add >> i) & 0x01);
	}
}
static void WriteData(uint16_t data)
{
	for(int8_t i = 15; i >= 0; i--)
	{
		SendBit((data >> i) & 0x0001);
	}
}
static void Verify(void)
{
	EnableChip();
	while(!(DO_Port->IDR & Input));			//Check Busy
}
void WriteRom(uint8_t address,uint8_t code,uint16_t data)
{
	EnableChip();
	StartBit();
	WriteAddress(address | code);			//オペコード2ビット+アドレス
	WriteData(data);
	DisableChip();
	Verify();
}
void ReadRom(uint8_t address,uint16_t *val)
{
	EnableChip();
	StartBit();
	WriteAddress(address | READ_CODE);
	*val = ReadData();
	DisableChip();
}
void EnableWrite(void)
{
	Init1usTick();							//SysTickを1usカウントに切り替える
	EnableChip();
	StartBit();
	SendOpCode(ENABLE_CODE);
	DisableChip();
}
void DisableWrite(void)
{
	EnableChip();
	StartBit();
	SendOpCode(DISABLE_CODE);
	DisableChip();
	DeInit1usTick();						//SysTickを1msカウントに戻す
}
