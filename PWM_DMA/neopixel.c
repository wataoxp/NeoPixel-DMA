/*
 * neopixel.c
 *
 *  Created on: Sep 20, 2024
 *      Author: wataoxp
 */
#include "neopixel.h"
#include <string.h>

typedef unsigned int FixedPoint;
//(1/60) << 16
const FixedPoint SixTeenPoint = 1092;

static void Hue_Rainbow(uint16_t *Val,uint8_t num);
static void Hue_Red(uint16_t *Val,uint8_t num);
static void Hue_Green(uint16_t *Val,uint8_t num);
static void Hue_Blue(uint16_t *Val,uint8_t num);
static void Hue_Neon(uint16_t *Val,uint8_t num);
static inline void SetLux(NeoPixelTypedef *pixel,uint8_t num,uint8_t Lux);

void InitNeoPixel(NeoPixelTypedef *pixel,PatternTypedef *Param,uint8_t *NeoPixelBuffer,uint16_t *HueArray)
{
	uint8_t pix;

	BuildHueData(HueArray, Param);
	for(pix = 0; pix < NUM_LED; pix++)
	{
		HUEtoRGB(pixel, HueArray[pix], pix);
		//HSVtoRGB(pixel, HueArray[pix], 255, pix);
		SetLux(pixel, pix, Param->Lux);
	}
	MakeBuffer(pixel, NeoPixelBuffer, NUM_LED);
}
static void (*colmod[])(uint16_t*,uint8_t) = {Hue_Rainbow,Hue_Red,Hue_Green,Hue_Blue,Hue_Neon};
void BuildHueData(uint16_t *HueArray,PatternTypedef *param)
{
	uint8_t i;
	uint16_t angle = (360 / NUM_LED) * param->Shift;
	uint8_t color = param->Type & ColorType_Max;

	for(i = 0; i < NUM_LED; i++)
	{
		HueArray[i] = i * angle;
		colmod[color](HueArray,i);
	}
}
void HUEtoRGB(NeoPixelTypedef *pixel,uint16_t Hue,uint8_t num)
{
	if(Hue <= 60)
	{
		pixel[num].bit24.r = 255;
		pixel[num].bit24.g = ((Hue * HUE_FIXEDPOINT) >> POINT_SHIFT);
		pixel[num].bit24.b = 0;
	}
	else if(Hue <= 120)
	{
		Hue = 120 - Hue;
		pixel[num].bit24.r = ((Hue * HUE_FIXEDPOINT) >> POINT_SHIFT);
		pixel[num].bit24.g = 255;
		pixel[num].bit24.b = 0;
	}
	else if(Hue <= 180)
	{
		Hue -= 120;
		pixel[num].bit24.r = 0;
		pixel[num].bit24.g = 255;
		pixel[num].bit24.b = ((Hue * HUE_FIXEDPOINT) >> POINT_SHIFT);
	}
	else if(Hue <= 240)
	{
		Hue = 240 - Hue;
		pixel[num].bit24.r = 0;
		pixel[num].bit24.g = ((Hue * HUE_FIXEDPOINT) >> POINT_SHIFT);
		pixel[num].bit24.b = 255;
	}
	else if(Hue <= 300)
	{
		Hue -= 240;
		pixel[num].bit24.r = ((Hue * HUE_FIXEDPOINT) >> POINT_SHIFT);
		pixel[num].bit24.g = 0;
		pixel[num].bit24.b = 255;
	}
	else if(Hue <= 360)
	{
		Hue = 360 - Hue;
		pixel[num].bit24.r = 255;
		pixel[num].bit24.g = 0;
		pixel[num].bit24.b = ((Hue * HUE_FIXEDPOINT) >> POINT_SHIFT);
	}
}
void HSVtoRGB(NeoPixelTypedef *pixel,uint16_t Hue,uint8_t Chroma,uint8_t num)
{
	uint8_t max = 255,min = 255 - Chroma;


	if(Hue <= 60)
	{
		pixel[num].bit24.r = max;
		pixel[num].bit24.g = ((SixTeenPoint * Hue) * (max - min) >> 16 )+ min;
		pixel[num].bit24.b = min;
	}
	else if(Hue <= 120)
	{
		Hue = 120 - Hue;
		pixel[num].bit24.r = ((SixTeenPoint * Hue) * (max - min) >> 16 )+ min;
		pixel[num].bit24.g = max;
		pixel[num].bit24.b = min;
	}
	else if(Hue <= 180)
	{
		Hue -= 120;
		pixel[num].bit24.r = min;
		pixel[num].bit24.g = max;
		pixel[num].bit24.b = ((SixTeenPoint * Hue) * (max - min) >> 16 )+ min;
	}
	else if(Hue <= 240)
	{
		Hue = 240 - Hue;
		pixel[num].bit24.r = min;
		pixel[num].bit24.g = ((SixTeenPoint * Hue) * (max - min) >> 16 )+ min;
		pixel[num].bit24.b = max;
	}
	else if(Hue <= 300)
	{
		Hue -= 240;
		pixel[num].bit24.r = ((SixTeenPoint * Hue) * (max - min) >> 16 )+ min;
		pixel[num].bit24.g = min;
		pixel[num].bit24.b = max;
	}
	else if(Hue <= 360)
	{
		Hue = 360 - Hue;
		pixel[num].bit24.r = max;
		pixel[num].bit24.g = min;
		pixel[num].bit24.b = ((SixTeenPoint * Hue) * (max - min) >> 16 )+ min;
	}
}
static inline void SetLux(NeoPixelTypedef *pixel,uint8_t num,uint8_t Lux)
{
	pixel[num].bit24.r >>= Lux;
	pixel[num].bit24.g >>= Lux;
	pixel[num].bit24.b >>= Lux;
}
void MakeBuffer(NeoPixelTypedef *pixel,uint8_t *SrcBuf,uint8_t size)
{
	uint8_t *ptr = SrcBuf;
	int32_t mask;

//    for(uint8_t i = 0;i < size; i++)			//NUM_LED
//    {
//    	for(mask = 1 << 23; mask; mask >>= 1)	//1LED,24byte
//    	{
//    		*ptr++ = (pixel[i].fullcolor & mask)? NeoPixel_HIGH:NeoPixel_LOW;
//    	}
//    }
	for(uint8_t i = 0;i < size; i++)
	{
		for(mask = 23; mask >= 0; mask--)	//1LED,24byte
		{
			*ptr++ = NeoPixel_LOW << ((pixel[i].fullcolor >> mask) & 0x01);
		}
	}
}
//ShiftPixel->24byte
void ShiftPixel(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *srcBuf)
{
	uint8_t tmpBuf[24];

	memcpy(tmpBuf,srcBuf,24);
	StartMemtoMem(DMAx, Channel,(NUM_LED - 1)*24);
	memcpy(srcBuf+(NUM_LED - 1)*24,tmpBuf,24);
}
static void Hue_Rainbow(uint16_t *Val,uint8_t num)
{
	Val[num] %= 360;
}
static void Hue_Red(uint16_t *Val,uint8_t num)
{
	Val[num] &= 0x3F;
}
static void Hue_Green(uint16_t *Val,uint8_t num)
{
	Val[num] = (Val[num] & 0x3F) + 90;
}
static void Hue_Blue(uint16_t *Val,uint8_t num)
{
	Val[num]= (Val[num] & 0x3F) + 180;
}
static void Hue_Neon(uint16_t *Val,uint8_t num)
{
	switch(num & 0x0f)
	{
		case 1:
		case 3:
			Val[num] = 330;
			break;
		case 2:
		case 5:
			Val[num] = 0;
			break;
		case 4:
		case 6:
			Val[num] = 60;
			break;
		default:
			Val[num] = 240;
			break;
	}
}
