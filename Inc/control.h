/*
 * Control.h
 *
 *  Created on: Dec 5, 2024
 *      Author: wataoxp
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "main.h"
#include "neopixel.h"
#include "eeprom.h"
#include "IRremote.h"

typedef enum{
	mismatch,
	match,
}KeyValid;

typedef enum{
	VerySlow = 4,
	Slow = 3,
	Fast = 2,
	VeryFast = 1,
}SleepTimer;

typedef enum{
	Writemode,
	Readmode,
}E2ROM_Time;

uint32_t ReturnFlag(void);

uint8_t CheckParam(PatternTypedef *param);
void ControlE2ROM(PatternTypedef *param,uint8_t mode);
uint8_t CheckKey(uint8_t command,uint8_t inverse);

void Sleep(TIM_TypeDef *TIMx,uint8_t time);
/*
 * CPUスリープ
 * 割り込みハンドラでは次の割りこみのみを止める
 * この関数内ではもし赤外線受信割り込みで復帰した場合でも規定の時間待機するようにwhileチェックする
 * チェック後に割り込みフラグをクリアする
 */
void RemoteToNeoPixel(PatternTypedef *param,uint8_t command);
void LCD_E2ROM(PatternTypedef *Param,I2C_TypeDef *I2Cx);
void LCD_IR(ConvertMSB *MSB,I2C_TypeDef *I2Cx);
#endif /* INC_CONTROL_H_ */
