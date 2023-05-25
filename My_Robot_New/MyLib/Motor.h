#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "stm32f1xx.h"

typedef enum
{
	MOTOR_STOP,
	MOTOR_CW,					// clockwise
	MOTOR_CCW					// counter-clockwise
}MotorState;
typedef struct
{
	GPIO_TypeDef *io_Port;
	uint16_t io_pin;
	TIM_HandleTypeDef *htim;
	uint16_t tim_channel;
	MotorState motor_state;
	uint8_t speed;
}Motor_TypeDef;

void control_motor(Motor_TypeDef *motor, MotorState StateMotor, uint8_t speed);
void motor_init(Motor_TypeDef *Motor, GPIO_TypeDef *io_port, uint16_t io_pin,
								TIM_HandleTypeDef *htim, uint32_t tim_channel);
#endif
