#ifndef _SERVO_H_
#define _SERVO_H_
#include "stm32f1xx_hal.h"

typedef struct
{
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
	uint8_t Angle;
}Servo;

uint8_t Servo_Read(Servo *SV);
void Servo_Write(Servo *SV, uint32_t angle);
void Servo_Init(Servo *SV, TIM_HandleTypeDef *htim, uint32_t Channel);
#endif
