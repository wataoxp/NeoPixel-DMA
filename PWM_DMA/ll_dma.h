/*
 * ll_dma.h
 *
 *  Created on: Sep 18, 2024
 *      Author: wataoxp
 */
#ifndef INC_LL_DMA_H_
#define INC_LL_DMA_H_

#include <stdint.h>
#include "stm32g0xx.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_tim.h"

void InitTIMtoDMA(TIM_TypeDef *TIMx,DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *bufferAddress);
void InitMEMtoMEM(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *bufferAddress);
void TransferDMA(DMA_TypeDef *DMAx, uint32_t Channel,TIM_TypeDef *TIMx,uint32_t size);
/*
 * DMAの転送サイズを設定。DMAを有効化
 * 転送サイズは1転送毎にデクリメント、転送完了後は0になるので毎回設定が必要
 */
void StartMemtoMem(DMA_TypeDef *DMAx,uint32_t Channel,uint32_t size);
void StopDMA(DMA_TypeDef *DMAx, uint32_t Channel1,uint32_t Channel2);

/*
 * void InitSPItoDMA(DMA_TypeDef *DMAx, uint32_t Channel,TIM_TypeDef *TIMx, uint8_t *bufferAddress);
 * DMA転送元アドレスと転送先アドレスを設定
 * ここではSPI_DRレジスタのアドレスを取得、代入している
 */
/*
 * void InitMemtoMem(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *srcAdress,uint8_t *dstAdress);
 * メモリー間転送用にアドレスをセットする。
 * DMAのCCRレジスタが0であるので、転送先と転送元は以下のようになる。
 * 転送元->CPAR	転送先->CMAR
 * MemtoMemモードで生成されるDMA_InitではDIRがセットされない。
 * この為MemorytoPeriphモードとは転送方向がまったくの「逆」になる。
 */


#endif /* INC_LL_DMA_H_ */
