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

typedef struct{
	TIM_TypeDef *SleepTimer;
	TIM_TypeDef *RemoteCounter;
	TIM_TypeDef *StopTimerIR;
}TIM_HandlePack;

void TimHanldeSet(TIM_HandlePack *Tims);
/*
 * TIM14,TIM16,TIM17の設定
 * それぞれ順に
 * CPUスリープタイマ
 * 赤外線デコードカウンタ
 * 赤外線受信停止カウンタ
 */
void Sleep(TIM_TypeDef *TIMx,uint8_t time);
/*
 * CPUスリープ
 * 割り込みハンドラでは次の割りこみのみを止める
 * この関数内ではもし赤外線受信割り込みで復帰した場合でも規定の時間待機するようにwhileチェックする
 * チェック後に割り込みフラグをクリアする
 */
void BeginSPItoDMA(SPI_TypeDef *SPIx,DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *bufferAddress);
void BeginMEMtoMEM(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *bufferAddress);
void RemoteToNeoPixel(PatternTypedef *param,uint8_t command);
/*
 * 赤外線信号のデータコマンドによってParamを編集する。
 * Repeatコード用に都度コマンドはParam.OldCommandに格納し、Repeat時はその値に従い再度case文を実行する
 */

#endif /* INC_CONTROL_H_ */
