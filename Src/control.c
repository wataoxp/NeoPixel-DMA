/*
 * control.c
 *
 *  Created on: Dec 5, 2024
 *      Author: wataoxp
 */
#include "main.h"

#include "ll_dma.h"
#include "neopixel.h"
#include "control.h"

typedef enum{
	LuxDown = 0,
	LuxUp = 1,
	ColorRed = 4,
	ColorGreen = 5,
	ColorBlue = 6,
	ColorRainbow = 7,
	ShiftVerySlow = 11,
	ShiftSlow = 15,
	ShiftFast = 19,
	ShiftVeryFast = 23,
}ControlerButton;

void TimHanldeSet(TIM_HandlePack *Tims)
{
	/* Sleep 1ms Counter */
	LL_TIM_ClearFlag_UPDATE(Tims->SleepTimer);
	/* Remote 1us Counter */
	LL_TIM_EnableCounter(Tims->RemoteCounter);
	/* Remote DisableIR 1ms Counter */
	LL_TIM_ClearFlag_UPDATE(Tims->StopTimerIR);
}
void Sleep(TIM_TypeDef *TIMx,uint8_t time)
{
	LL_TIM_SetAutoReload(TIMx,time);
	LL_TIM_EnableIT_UPDATE(TIMx);
	LL_TIM_EnableCounter(TIMx);

	__WFI();

	while(LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0);	//wait TIM14interrupt
	LL_TIM_ClearFlag_UPDATE(TIMx);
	LL_TIM_DisableCounter(TIMx);
}
void BeginSPItoDMA(SPI_TypeDef *SPIx,DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *bufferAddress)
{
	InterruptSetDMA(DMAx, Channel);
	EnableSPItoDMA(SPIx);
	InitSPItoDMA(DMAx, Channel, SPIx, bufferAddress);
}
void BeginMEMtoMEM(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *bufferAddress)
{
	InterruptSetDMA(DMAx, Channel);
	InitMemtoMem(DMAx, Channel, (bufferAddress+24), bufferAddress);
}
void RemoteToNeoPixel(PatternTypedef *Param,uint8_t command)
{
	switch(command)
	{
	case LuxDown:
		Param->Lux = (Param->Lux > 0)? Param->Lux-1:0;
		break;
	case LuxUp:
		Param->Lux = (Param->Lux <= 8)? Param->Lux+1:8;
		break;
	case ColorRed:
		Param->Type = Red;
		break;
	case ColorGreen:
		Param->Type = Green;
		break;
	case ColorBlue:
		Param->Type = Blue;
		break;
	case ColorRainbow:
		Param->Type = Rainbow;
		break;
	case ShiftVerySlow:
		Param->Shift = VerySlow;
		break;
	case ShiftSlow:
		Param->Shift = Slow;
		break;
	case ShiftFast:
		Param->Shift = Fast;
		break;
	case ShiftVeryFast:
		Param->Shift = VeryFast;
		break;
	case 20:
		Param->time = 50;
		break;
	case 21:
		Param->time = 100;
		break;
	case 22:
		Param->time = 200;
		break;
	default:
		break;
	}
}
