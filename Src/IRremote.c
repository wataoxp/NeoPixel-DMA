/*
 * IRremote.c
 *
 *  Created on: Nov 16, 2024
 *      Author: wataoxp
 */
#include "IRremote.h"

static TIM_TypeDef *TIMx;

static IRQn_Type IRQn;
static uint8_t *pFlag;
static uint32_t *pBinary;
static uint32_t ExLine;

static inline void CheckLeader(uint32_t count,uint8_t *pStatus)
{
	if(count > IR_LEADER_TIMING && count < IR_MAX_TIMING)
	{
		*pStatus = ReadTime;
		TIMx->CNT = 0;
	}
	else
	{
		*pStatus = StopIR;
		*pFlag = IR_NACK;
	}
}
void EnableIR(void)
{
	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_EnableIRQ(IRQn);
	LL_EXTI_EnableIT_0_31(ExLine);
}
static inline void DisableIR(void)
{
	LL_EXTI_DisableIT_0_31(ExLine);
	__NVIC_DisableIRQ(IRQn);

	LL_TIM_GenerateEvent_UPDATE(TIMx);
	LL_TIM_ClearFlag_UPDATE(TIMx);
	LL_TIM_EnableIT_UPDATE(TIMx);
}
void SetISR(IR_Parameter *IR)
{
	TIMx = IR->TIM;
	IRQn = IR->IRQnumber;
	pFlag = IR->Flag;
	pBinary = IR->Binary;
	ExLine = IR->ExtiLine;
}
void RecieveIR_IT(void)
{
	static uint8_t Status = StartLeader;
	static uint8_t numBits = 0;

	switch(Status)
	{
	case StartLeader:
		TIMx->CNT = 0;
		Status = EndLeader;
		break;
	case EndLeader:
		CheckLeader(TIMx->CNT, &Status);
		break;
	case ReadTime:
		if(TIMx->CNT > NEC_HIGH)
		{
			*pBinary |= 1 << numBits;
		}
		numBits++;
		TIMx->CNT = 0;
		if(numBits >= 32)
		{
			Status = StopIR;
			*pFlag = IR_ACK;
			numBits = 0;
		}
		break;
	default:
		break;
	}
	if(Status == StopIR)
	{
		DisableIR();
		Status = StartLeader;
	}
}
void BinaryToHex(ConvertMSB *MSB,uint32_t Binary)
{
	MSB->bit8 = (uint8_t)(Binary & 0xFF);
	MSB->bit16 = (uint8_t)(Binary >> 8) & 0xFF;
	MSB->bit24 = (uint8_t)(Binary >> 16) & 0xFF;
	MSB->bit31 = (uint8_t)(Binary >> 24) & 0xFF;

}
void DataReset(ConvertMSB *MSB,uint32_t *Binary)
{
	MSB->bit8 = 0;
	MSB->bit16 = 0;
	MSB->bit24 = 0;
	MSB->bit31 = 0;

	*Binary = 0;
}
