/*
 * ll_dma.c
 *
 *  Created on: Sep 18, 2024
 *      Author: wataoxp
 */
#include "ll_dma.h"

static void ExtiDMA(DMA_TypeDef *DMAx, uint32_t Channel);
static void EnableSPItoDMA(SPI_TypeDef *SPIx);
static void SPI_AddressSet(DMA_TypeDef *DMAx, uint32_t Channel,SPI_TypeDef *SPIx, uint8_t *bufferAddress);
static void M2M_AddressSet(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *srcAdress,uint8_t *dstAdress);

/* initial function */
void InitSPItoDMA(SPI_TypeDef *SPIx,DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *bufferAddress)
{
	ExtiDMA(DMAx, Channel);
	EnableSPItoDMA(SPIx);
	SPI_AddressSet(DMAx, Channel, SPIx, bufferAddress);
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
static void EnableSPItoDMA(SPI_TypeDef *SPIx)
{
	LL_SPI_EnableDMAReq_TX(SPIx);
	LL_SPI_Enable(SPIx);
}
static void SPI_AddressSet(DMA_TypeDef *DMAx, uint32_t Channel,SPI_TypeDef *SPIx, uint8_t *bufferAddress)
{
	LL_DMA_SetMemoryAddress(DMAx, Channel, (uint32_t)bufferAddress);
	LL_DMA_SetPeriphAddress(DMAx, Channel, LL_SPI_DMA_GetRegAddr(SPIx));
}
static void M2M_AddressSet(DMA_TypeDef *DMAx,uint32_t Channel,uint8_t *srcAdress,uint8_t *dstAdress)
{
	LL_DMA_SetM2MSrcAddress(DMAx, Channel, (uint32_t)srcAdress);
	LL_DMA_SetM2MDstAddress(DMAx, Channel, (uint32_t)dstAdress);
}
/* main call Function */
void StartSPItoDMA(DMA_TypeDef *DMAx, uint32_t Channel,SPI_TypeDef *SPIx,uint32_t size)
{
	if(!LL_SPI_IsEnabled(SPIx)) { LL_SPI_Enable(SPIx); }

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
