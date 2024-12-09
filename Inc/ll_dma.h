/*
 * ll_dma.h
 *
 *  Created on: Sep 18, 2024
 *      Author: wataoxp
 */
#ifndef INC_LL_DMA_H_
#define INC_LL_DMA_H_

void InterruptSetDMA(DMA_TypeDef *DMAx, uint32_t Channel);
/*
 * DMAの割り込み設定
 * エラー割り込み以外無効にしている
 */
void EnableSPItoDMA(SPI_TypeDef *SPIx);
/*
 * SPIのDMAリクエストを許可
 * SPIを有効化
 */
void InitSPItoDMA(DMA_TypeDef *DMAx, uint32_t Channel,SPI_TypeDef *SPIx, uint8_t *bufferAddress);
/*
 * DMA転送元アドレスと転送先アドレスを設定
 * ここではSPI_DRレジスタのアドレスを取得、代入している
 */
void SPI_Transfer_DMA(DMA_TypeDef *DMAx, uint32_t Channel,SPI_TypeDef *SPIx,uint32_t size);
/*
 * DMAの転送サイズを設定。DMAを有効化
 * 転送サイズは1転送毎にデクリメント、転送完了後は0になるので毎回設定が必要
 */
void StopDMA(DMA_TypeDef *DMAx, uint32_t Channel);
/*
 * DMAを停止する。
 * 申し訳程度のエラー処理
 */
void InitMemtoMem(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *srcAdress,uint8_t *dstAdress);
/*
 * メモリー間転送用にアドレスをセットする。
 * DMAのCCRレジスタが0であるので、転送先と転送元は以下のようになる。
 * 転送元->CPAR	転送先->CMAR
 * MemtoMemモードで生成されるDMA_InitではDIRがセットされない。
 * この為MemorytoPeriphモードとは転送方向がまったくの「逆」になる。
 */
void StartMemtoMem(DMA_TypeDef *DMAx,uint32_t Channel,uint32_t size);


#endif /* INC_LL_DMA_H_ */
