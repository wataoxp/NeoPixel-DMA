/*
 * so1602.h
 *
 *  Created on: Mar 1, 2025
 *      Author: wataoxp
 */

#ifndef INC_SO1602_H_
#define INC_SO1602_H_

#include "main.h"
#include "i2c.h"

//SA0 == Low
#define SO1602_ADDRESS (0x3C << 1)

#define REG_CMD 0x00
#define REG_DATA 0x40

#define CLEAR_DISPLAY 0x01
#define RETURN_HOME 0x02

/* SO1602 DDRAM */
/****************
 * Row1|0x00~0x0F
 * Row2|0x20~0x2F
 ****************/
#define HOME_CUSOR 0x00
#define ENTER_CUSOR 0x20

#define DISP_ON 0x04
#define CUSOR_ON 0x02
#define BLINK_CUSOR 0x01
#define DISP_CMD 0x08 | DISP_ON

#define DDRAM_ACCESS 0x80

static inline void StringLCD(I2C_TypeDef *I2Cx,const char *str,uint8_t size)
{
	SeqI2C_Mem_Write(I2Cx, SO1602_ADDRESS, (uint8_t*)str, REG_DATA, I2C_MEMADD_SIZE_8BIT, size);
}
static inline void ClearLCD(I2C_TypeDef *I2Cx)
{
	PushI2C_Mem_Write(I2Cx, SO1602_ADDRESS, CLEAR_DISPLAY, REG_CMD, I2C_MEMADD_SIZE_8BIT);
}
static inline void SetCusor(I2C_TypeDef *I2Cx,uint8_t x,uint8_t y)
{
	PushI2C_Mem_Write(I2Cx, SO1602_ADDRESS, (DDRAM_ACCESS | (x + y * ENTER_CUSOR)), REG_CMD, I2C_MEMADD_SIZE_8BIT);
}
static inline void SendCMD(I2C_TypeDef *I2Cx,uint8_t cmd)
{
	PushI2C_Mem_Write(I2Cx, SO1602_ADDRESS, cmd, REG_CMD, I2C_MEMADD_SIZE_8BIT);
}

void SO1602_Init(I2C_TypeDef *I2Cx);
#endif /* INC_SO1602_H_ */
