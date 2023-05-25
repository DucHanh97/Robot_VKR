#include "flash.h"

void flash_unlock(void)
{
	HAL_FLASH_Unlock();
}

void flash_lock(void)
{
	HAL_FLASH_Lock();
}

void flash_erease(uint32_t addr)
{
	FLASH_EraseInitTypeDef eraseInit;
	eraseInit.PageAddress = addr;
	eraseInit.NbPages = 1;
	eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	uint32_t PageError;
	HAL_FLASHEx_Erase(&eraseInit, &PageError);
}

void flash_write_arr(uint32_t addr, uint8_t *data, uint16_t len)
{
	for(uint16_t i = 0; i < len; i+=2)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, data[i] | (uint16_t)(data[i+1] << 8));
	}
}

void flash_read_arr(uint32_t addr, uint8_t *data, uint16_t len)
{
	for(uint16_t i = 0; i < len; i+= 2)
	{
//		volatile uint32_t *p = (volatile uint32_t *)(addr + i);
//		uint16_t data = *p;
		uint16_t data_temp = *(volatile uint32_t *)(addr + i);
//		data[i] = data_temp;
//		data[i+1] = data_temp >> 8;
		*(data + i) = data_temp;
		*(data +i + 1) = data_temp >> 8;
	}
}
