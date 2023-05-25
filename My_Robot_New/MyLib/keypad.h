#ifndef KEYPAD_H
#define KEYPAD_H

#include "stm32f1xx_hal.h"

#define KEYPAD_ROW	4
#define KEYPAD_COL	4

void Keypad_Handle(void);
void Keypad_Init(void);

#endif
