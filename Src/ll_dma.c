/*
 * ll_dma.c
 *
 *  Created on: Sep 18, 2024
 *      Author: wataoxp
 */
#include "main.h"
#include "ll_dma.h"

void InterruptSetDMA(DMA_TypeDef *DMAx, uint32_t Channel)
{
	LL_DMA_DisableChannel(DMAx, Channel);
	LL_DMA_EnableIT_TE(DMAx, Channel);
	LL_DMA_DisableIT_TC(DMAx, Channel);
	LL_DMA_DisableIT_HT(DMAx, Channel);
}
/* SPItoDMA Function */
void EnableSPItoDMA(SPI_TypeDef *SPIx)
{
	LL_SPI_EnableDMAReq_TX(SPIx);
	LL_SPI_Enable(SPIx);
}
void InitSPItoDMA(DMA_TypeDef *DMAx, uint32_t Channel,SPI_TypeDef *SPIx, uint8_t *bufferAddress)
{
	LL_DMA_SetMemoryAddress(DMAx, Channel, (uint32_t)bufferAddress);
	LL_DMA_SetPeriphAddress(DMAx, Channel, LL_SPI_DMA_GetRegAddr(SPIx));
}
void SPI_Transfer_DMA(DMA_TypeDef *DMAx, uint32_t Channel,SPI_TypeDef *SPIx,uint32_t size)
{
	if(!LL_SPI_IsEnabled(SPIx)) { LL_SPI_Enable(SPIx); }

	LL_DMA_SetDataLength(DMAx, Channel, size);
	LL_DMA_EnableChannel(DMAx, Channel);
}
void StopDMA(DMA_TypeDef *DMAx, uint32_t Channel)
{
	LL_DMA_DisableChannel(DMAx, Channel);
}
/* MEM2MEM Function */
void InitMemtoMem(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *srcAdress,uint8_t *dstAdress)
{
	LL_DMA_SetM2MSrcAddress(DMAx, Channel, (uint32_t)srcAdress);
	LL_DMA_SetM2MDstAddress(DMAx, Channel, (uint32_t)dstAdress);
}
void StartMemtoMem(DMA_TypeDef *DMAx,uint32_t Channel,uint32_t size)
{
	LL_DMA_SetDataLength(DMAx, Channel, size);
	LL_DMA_EnableChannel(DMAx, Channel);
}
