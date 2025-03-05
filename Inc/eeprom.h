/*
 * eeprom.h
 *
 *  Created on: Jan 13, 2025
 *      Author: wataoxp
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include <stdint.h>
#include "s93c46.h"
#include "delay.h"

#define EEPROM_MAX_PAGE 63

typedef enum{
	EEPROM_Success,
	EEPROM_Error,
}ROM_Code;

uint8_t GetPage(void);
ROM_Code GetNewPage(void);
void SetNewData(uint8_t *data);
void GetNewData(uint8_t *data);

#endif /* INC_EEPROM_H_ */
