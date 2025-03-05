/*
 * IRremote.h
 *
 *  Created on: Nov 16, 2024
 *      Author: wataoxp
 */

#ifndef INC_IRREMOTE_H_
#define INC_IRREMOTE_H_

#include <stdint.h>
#include "stm32g0xx.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_tim.h"

#define IR_DECODE_READY 0
#define IR_ACK 0x06
#define IR_NACK 0x15
#define IR_REPEAT 0x20

/* Falling Edge Exti Timing (TIMING*10us) */
#define IR_MIN_TIMING 1000 //10000us.RepeatCode Limit
#define IR_MAX_TIMING 1400 //14000us.LeaderCode Limit
#define IR_LEADER_TIMING 1300 //13000us.LeaderCode
#define IR_REPEAT_TIMING 1100 //11000us.RepeatCode
#define NEC_HIGH 200
#define NEC_LOW 140

#define IR_ERROR_DATA 0xFF

typedef enum{
	StartLeader,
	EndLeader,
	ErrorLeader,
	ReadTime,
	StopIR,
}IRstatus;

typedef struct{
	TIM_TypeDef *TIM;
	IRQn_Type IRQnumber;
	uint32_t *Binary;
	uint8_t *Flag;
	uint32_t ExtiLine;
}IR_Parameter;

typedef enum{
	Succces,
	Failed,
}IR_Bool;

typedef struct{
	uint8_t bit31;
	uint8_t bit24;
	uint8_t bit16;
	uint8_t bit8;
}ConvertMSB;

void EnableIR(void);
void SetISR(IR_Parameter *IR);
void RecieveIR_IT(void);
void BinaryToHex(ConvertMSB *MSB,uint32_t Binary);
void DataReset(ConvertMSB *MSB,uint32_t *Binary);


#endif /* INC_IRREMOTE_H_ */
