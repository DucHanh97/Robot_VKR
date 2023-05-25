#ifndef FLASH_H
#define FLASH_H
#include "stm32f1xx_hal.h"

void flash_unlock(void);
void flash_lock(void);
void flash_erease(uint32_t addr);
void flash_write_arr(uint32_t addr, uint8_t *data, uint16_t len);
void flash_read_arr(uint32_t addr, uint8_t *data, uint16_t len);
#endif
