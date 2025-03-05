/*
 * eeprom.c
 *
 *  Created on: Jan 13, 2025
 *      Author: wataoxp
 */

#include "eeprom.h"

static uint8_t page;

static inline uint16_t PackRomData(uint8_t *data)
{
	uint16_t val;
	val = data[0]  << 12 | data[1] << 8 | data[2] << 4 | data[3];

	return val;
}
static inline void UnPackRomData(uint16_t tmp,uint8_t *data)
{
	data[0] = (tmp >> 12) & 0x0F;
	data[1] = (tmp >> 8) & 0x0F;
	data[2] = (tmp >> 4) & 0x0F;
	data[3] = (tmp) & 0x0F;
}
static void CheckCurrentPage(uint16_t val)
{
	if(page > EEPROM_MAX_PAGE)				//pageが64の時にリセット
	{
		page = 0;
		EnableWrite();
		WriteRom(page, ER_ALL_CODE, 0);
		WriteRom(page, WRITE_CODE, val);
		DisableWrite();
	}
}
uint8_t GetPage(void)
{
	return page;
}
uint8_t GetNewPage(void)		//最新のページを確認する。Write時はpage。Read時は(page-1)
{
	uint16_t EEPROM_Buffer[EEPROM_MAX_PAGE];

	Init1usTick();
	while(page <= EEPROM_MAX_PAGE)
	{
		ReadRom(page, &EEPROM_Buffer[page]);
		if(EEPROM_Buffer[page] != UINT16_MAX)
		{
			page++;
		}
		else
		{
			break;
		}
	}
	DeInit1usTick();

	return page;
}
void SetNewData(uint8_t *data)
{
	uint16_t value = PackRomData(data);

	EnableWrite();
	WriteRom(page, WRITE_CODE, value);
	DisableWrite();
}
void GetNewData(uint8_t *data)
{
	uint16_t tmp;
	uint8_t ReadPage;

	if(page == 0)
	{
		ReadPage = 0;
	}
	else if(page > EEPROM_MAX_PAGE)
	{
		ReadPage = EEPROM_MAX_PAGE;
	}
	else
	{
		ReadPage = (page-1);
	}

	Init1usTick();
	ReadRom(ReadPage, &tmp);
	DeInit1usTick();

//	data[0] = (tmp >> 12) & 0x0F;
//	data[1] = (tmp >> 8) & 0x0F;
//	data[2] = (tmp >> 4) & 0x0F;
//	data[3] = (tmp) & 0x0F;
	UnPackRomData(tmp, data);

	CheckCurrentPage(tmp);		//最新ページデータの取得後、ページがMAXを超えていないか確認
}
