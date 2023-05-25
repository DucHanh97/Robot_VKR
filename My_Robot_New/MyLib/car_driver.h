#ifndef CAR_H
#define CAR_H
#include "stm32f1xx.h"

typedef enum
{
	CAR_STOP_STATE,
	CAR_FORWARD_STATE,
	CAR_BACKWARD_STATE,
	CAR_LEFT_STATE,
	CAR_RIGHT_STATE,
	CAR_FW_LEFT_STATE,
	CAR_FW_RIGHT_STATE,
	CAR_BW_LEFT_STATE,
	CAR_BW_RIGHT_STATE
}CarState;

void Car_Control(CarState State, uint8_t speed);
void Car_Control_Wheels(int8_t right_speed, int8_t left_speed);
void Car_Init(TIM_HandleTypeDef *htim, uint32_t left_motor_channel, uint32_t right_motor_channel, GPIO_TypeDef *left_motor_port,
									uint16_t left_motor_pin, GPIO_TypeDef *right_motor_port, uint16_t right_motor_pin);
#endif
