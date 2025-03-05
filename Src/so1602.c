/*
 * so1602.c
 *
 *  Created on: Mar 1, 2025
 *      Author: wataoxp
 */
#include "so1602.h"

void SO1602_Init(I2C_TypeDef *I2Cx)
{
	uint8_t config[] = {CLEAR_DISPLAY,RETURN_HOME,DISP_CMD,CLEAR_DISPLAY};
	uint8_t contrast[] = {0x2a,0x79,0x81,0xff,0x78,0x28};

	LL_mDelay(100);
	for(uint8_t i = 0;i < sizeof(config);i++)
	{
		PushI2C_Mem_Write(I2Cx,SO1602_ADDRESS , config[i], REG_CMD, I2C_MEMADD_SIZE_8BIT);
		LL_mDelay(20);
	}
	for(uint8_t j = 0;j < sizeof(contrast);j++)
	{
		PushI2C_Mem_Write(I2Cx, SO1602_ADDRESS, contrast[j], REG_CMD, I2C_MEMADD_SIZE_8BIT);
	}
}
