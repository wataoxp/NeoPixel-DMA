/*
 * ll_dma.c
 *
 *  Created on: Sep 18, 2024
 *      Author: wataoxp
 */
#include "ll_dma.h"

static void ExtiDMA(DMA_TypeDef *DMAx, uint32_t Channel);
static void EnableDMA(TIM_TypeDef *TIMx);
static void DMA_AddressSet(DMA_TypeDef *DMAx, uint32_t Channel,TIM_TypeDef *TIMx, uint8_t *bufferAddress);
static void M2M_AddressSet(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *srcAdress,uint8_t *dstAdress);

/* initial function */
void InitTIMtoDMA(TIM_TypeDef *TIMx,DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *bufferAddress)
{
	ExtiDMA(DMAx, Channel);
	DMA_AddressSet(DMAx, Channel, TIMx, bufferAddress);
	EnableDMA(TIMx);
}
void InitMEMtoMEM(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *bufferAddress)
{
	ExtiDMA(DMAx, Channel);
	M2M_AddressSet(DMAx, Channel, (bufferAddress+24), bufferAddress);
}
/* Static DMA Function */
static void ExtiDMA(DMA_TypeDef *DMAx, uint32_t Channel)
{
	LL_DMA_DisableChannel(DMAx, Channel);
	LL_DMA_EnableIT_TE(DMAx, Channel);
	LL_DMA_DisableIT_TC(DMAx, Channel);
	LL_DMA_DisableIT_HT(DMAx, Channel);
}
static void EnableDMA(TIM_TypeDef *TIMx)
{
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_TIM2_UP);		//DMAMUX,DMAトリガをカウンタ更新時に設定
	LL_TIM_CC_SetDMAReqTrigger(TIMx, LL_TIM_CCDMAREQUEST_UPDATE);				//TIM,カウンタ更新時にDMAリクエスト
	LL_TIM_EnableDMAReq_UPDATE(TIMx);											//TIM,カウンタ更新時DMA要求を許可
	LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3);							//TIM,PWM出力を有効化
	LL_TIM_EnableCounter(TIMx);
}
static void DMA_AddressSet(DMA_TypeDef *DMAx, uint32_t Channel,TIM_TypeDef *TIMx, uint8_t *bufferAddress)
{
	LL_DMA_SetMemoryAddress(DMAx, Channel, (uint32_t)bufferAddress);
	LL_DMA_SetPeriphAddress(DMAx, Channel, (uint32_t)&TIM2->CCR3);				//DMA,TIM比較レジスタを転送先に設定。TIM2は32ビットタイマ
}
static void M2M_AddressSet(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *srcAdress,uint8_t *dstAdress)
{
	LL_DMA_SetM2MSrcAddress(DMAx, Channel, (uint32_t)srcAdress);
	LL_DMA_SetM2MDstAddress(DMAx, Channel, (uint32_t)dstAdress);
}
/* main call Function */
void TransferDMA(DMA_TypeDef *DMAx, uint32_t Channel,TIM_TypeDef *TIMx,uint32_t size)
{
	if(!LL_TIM_IsEnabledCounter(TIMx))
	{
		LL_TIM_EnableCounter(TIMx);
	}
	LL_DMA_SetDataLength(DMAx, Channel, size);
	LL_DMA_EnableChannel(DMAx, Channel);
}
void StartMemtoMem(DMA_TypeDef *DMAx,uint32_t Channel,uint32_t size)
{
	LL_DMA_SetDataLength(DMAx, Channel, size);
	LL_DMA_EnableChannel(DMAx, Channel);
}
void StopDMA(DMA_TypeDef *DMAx, uint32_t Channel1,uint32_t Channel2)
{
	while(!(LL_DMA_IsActiveFlag_TC1(DMA1)) && !(LL_DMA_IsActiveFlag_TC2(DMA1)));
	LL_DMA_ClearFlag_TC1(DMA1);
	LL_DMA_ClearFlag_TC2(DMA1);
	LL_DMA_DisableChannel(DMAx, Channel1);
	LL_DMA_DisableChannel(DMAx, Channel2);
}
