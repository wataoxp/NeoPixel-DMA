/*
 * control.c
 *
 *  Created on: Dec 5, 2024
 *      Author: wataoxp
 */
#include "control.h"

#include "i2c.h"
#include "so1602.h"
#include <string.h>
#include <stdio.h>

typedef enum{
	ColorRainbow = 0x00,
	ColorRed = 0x01,
	ColorGreen = 0x02,
	ColorBlue = 0x03,
	ColorNeon = 0x04,

	ColorPoint = 0x07,

	SpeedUp = 0x0A,
	SpeedDown = 0x0B,
	LuxDown = 0x0C,
	LuxUp = 0x0D,

	LCDView = 0x10,

	ShiftUp = 0x80,
	ShiftDown = 0x81,

	Submit = 0x18,
}ControlerButton;

static uint8_t RemoteKeyArray[] = {
		ColorRainbow,ColorRed,ColorGreen,ColorBlue,ColorNeon,SpeedUp,SpeedDown,
		LuxDown,LuxUp,ShiftUp,ShiftDown,
		LCDView,Submit,
};

static PatternTypedef OldParam;

static inline uint8_t MinValue(uint8_t val,uint8_t min)
{
	if(val > min)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
static inline uint8_t MaxValue(uint8_t val,uint8_t max)
{
	if(val < max)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
uint8_t CheckParam(PatternTypedef *param)	//起動時のデータと同じ(countが0)であれば書き込まない
{
	uint8_t count = (OldParam.Lux ^ param->Lux) || (OldParam.Type ^ param->Type) ||
					(OldParam.Time ^ param->Time) || (OldParam.Shift ^ param->Shift);

	return count;
}
void ControlE2ROM(PatternTypedef *param,uint8_t mode)
{
	uint8_t buf[4];

	if(mode == Writemode)
	{
		buf[0] = param->Lux;
		buf[1] = param->Type;
		buf[2] = param->Time;
		buf[3] = param->Shift;

		SetNewData(buf);
	}
	else if(mode == Readmode)
	{
		GetNewData(buf);
		OldParam.Lux = param->Lux = buf[0];
		OldParam.Type = param->Type = buf[1];
		OldParam.Time = param->Time = buf[2];
		OldParam.Shift = param->Shift = buf[3];
	}
}
uint8_t CheckKey(uint8_t command,uint8_t inverse)
{
	uint8_t i = 0;
	uint8_t ret = mismatch;

	if((command ^ 0xFF) == inverse)
	{
		while(i < sizeof(RemoteKeyArray))
		{
			if(command == RemoteKeyArray[i])
			{
				ret = match;
				break;
			}
			i++;
		}
	}
	return ret;
}
void Sleep(TIM_TypeDef *TIMx,uint8_t time)
{
	LL_TIM_SetAutoReload(TIMx,((time * 50)-1));
	LL_TIM_EnableIT_UPDATE(TIMx);
	LL_TIM_EnableCounter(TIMx);

	__WFI();

	while(LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0);	//wait TIMxinterrupt
	LL_TIM_ClearFlag_UPDATE(TIMx);
	LL_TIM_DisableCounter(TIMx);
}
/* LCD出力、テスト用 */
static uint32_t ValidLCD = 0;
uint32_t ReturnFlag(void)
{
	return ValidLCD;
}
void RemoteToNeoPixel(PatternTypedef *Param,uint8_t command)
{
	Param->OldCommand = command;
	if(command < ColorPoint)
	{
		Param->Type = command;
		return;
	}
	switch(command)
	{
	case LuxUp:
		Param->Lux += MaxValue(Param->Lux, 8);
		break;
	case LuxDown:
		Param->Lux -= MinValue(Param->Lux, 0);
		break;
	case SpeedUp:
		Param->Time -= MinValue(Param->Time, VeryFast);
		break;
	case SpeedDown:
		Param->Time += MaxValue(Param->Time, VerySlow);
		break;
	case ShiftUp:
		Param->Shift <<= MaxValue(Param->Shift, 32);
		break;
	case ShiftDown:
		Param->Shift >>= MinValue(Param->Shift, 1);
		break;
	case LCDView:
		if(!ValidLCD) SO1602_Init(I2C2);
		ValidLCD = 1;
		break;
	case Submit:
		if(ValidLCD) LCD_E2ROM(Param, I2C2);
		break;
	default:
		break;
	}
}
void LCD_E2ROM(PatternTypedef *Param,I2C_TypeDef *I2Cx)
{
	char Message[16];
	char AsciStr[3];
	uint8_t AsciPage;

	AsciPage = GetPage() - 1;
	ControlE2ROM(Param, Readmode);		//ここでROMのデータに上書きされちゃうけどね

	AsciStr[0] = (AsciPage / 10) + '0';
	AsciStr[1] = (AsciPage % 10) + '0';
	AsciStr[2] = '\0';

	SetCusor(I2Cx, 0, 0);

	sprintf(Message,"%s,%X,%X,%X,%X",AsciStr,Param->Lux,Param->Type,Param->Time,Param->Shift);
	StringLCD(I2Cx, Message, strlen(Message));
}
void LCD_IR(ConvertMSB *MSB,I2C_TypeDef *I2Cx)
{
	char Message[16];

	SetCusor(I2Cx, 0, 1);
	sprintf(Message,"%X,%X,%X,%X",MSB->bit8,MSB->bit16,MSB->bit24,MSB->bit31);
	StringLCD(I2Cx, Message, strlen(Message));
}
