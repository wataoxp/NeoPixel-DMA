/*
 * neopixel.h
 *
 *  Created on: Sep 20, 2024
 *      Author: wataoxp
 */
#include "main.h"
#include "ll_dma.h"

#ifndef INC_NEOPIXEL_H_
#define INC_NEOPIXEL_H_

/***LED数***
 * カラーテープ 60個
 * ARGBファン 23個
 * */
#define NUM_LED 60
//SPI送信バッファ用
#define RESET_PULSE 10
#define NEOPIXEL_BUFFER_SIZE (NUM_LED * 24 + RESET_PULSE)

//NeoPixInit用
#define HUE_SHIFT 4

//SPI HIビット数
#define NeoPixel_HIGH 0xF8		//625ns,T1H
#define NeoPixel_LOW 0xC0		//250ns,T0H

//ShiftPixelフラグ用
#define PIXEL_MOVE_SHIFT 3

//HSV変換用固定小数
/*
 * Hueが1変化するごとにRGBは255/60ずつ変化する。
 * (255/60)*2^3=34
 * 2^3は(255/60)との乗算の結果が「整数」になる最小の数値。
 * 
 * 上記の式に演算したい乗数を掛けた後に、8で割る(右3シフト)
 * 今回の場合は(68*Hue)>>4によって小数と整数の疑似的な乗算ができる。
 */
#define HUE_FIXEDPOINT 34
#define POINT_SHIFT 3

//STMはリトルエンディアン。なのでメモリ上では以下の順番でgrbの順となる
typedef struct{
	uint8_t b;
	uint8_t r;
	uint8_t g;
}color;

typedef union{
	uint32_t fullcolor;
	color bit24;
}NeoPixelTypedef;

typedef struct{
	uint8_t Lux;
	uint8_t Type;
	uint8_t Time;
	uint8_t Shift;
	uint8_t OldCommand;
}PatternTypedef;

typedef enum{
	Rainbow,
	Red,
	Green,
	Blue,
	Purple,
	White,
}ColorType;

void InitNeoPixel(NeoPixelTypedef *pixel,PatternTypedef *Param,uint8_t *NeoPixelBuffer,uint16_t *HueArray);
/*
 * 色相Hueを設定、さらにそれをRGBへ、最後にRGBからNeoPixel用のビットデータに変換する。
 * リモコンからの信号受信後は変更要素を反映するため、この関数を呼ぶ
 */
void BuildHueData(uint16_t *HueArray,PatternTypedef *Param);
void HUEtoRGB(NeoPixelTypedef *pixel,uint16_t Hue,uint8_t num);
void HSVtoRGB(NeoPixelTypedef *pixel,uint16_t Hue,uint8_t Chroma,uint8_t num);

void MakeBuffer(NeoPixelTypedef *pixel,uint8_t *SrcBuf,uint8_t size);
void ShiftPixel(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *srcBuf);

void WaveAngle(uint16_t *HueArray,uint8_t pix,uint16_t angle);



#endif /* INC_NEOPIXEL_H_ */
