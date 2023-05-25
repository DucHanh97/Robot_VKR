#include "car_driver.h"
#include "Motor.h"

Motor_TypeDef motor_left;
Motor_TypeDef motor_right;

void Car_Control(CarState car_state, uint8_t speed)
{
	switch(car_state)
	{
		case CAR_STOP_STATE:
			control_motor(&motor_left, MOTOR_STOP, speed);
			control_motor(&motor_right, MOTOR_STOP, speed);
			break;
		case CAR_FORWARD_STATE:
			control_motor(&motor_left, MOTOR_CW, speed);
			control_motor(&motor_right, MOTOR_CW, speed);
			break;
		case CAR_BACKWARD_STATE:
			control_motor(&motor_left, MOTOR_CCW, speed);
			control_motor(&motor_right, MOTOR_CCW, speed);
			break;
		case CAR_LEFT_STATE:
			control_motor(&motor_left, MOTOR_CW, speed);
			control_motor(&motor_right, MOTOR_CCW, speed);
			break;
		case CAR_RIGHT_STATE:
			control_motor(&motor_left, MOTOR_CCW, speed);
			control_motor(&motor_right, MOTOR_CW, speed);
			break;
		case CAR_FW_LEFT_STATE:
			control_motor(&motor_left, MOTOR_CW, (int8_t) speed/3);
			control_motor(&motor_right, MOTOR_CW, speed);
			break;
		case CAR_FW_RIGHT_STATE:
			control_motor(&motor_left, MOTOR_CW, speed);
			control_motor(&motor_right, MOTOR_CW, (int8_t) speed/3);
			break;
		case CAR_BW_LEFT_STATE:
			control_motor(&motor_left, MOTOR_CCW, (int8_t) speed/3);
			control_motor(&motor_right, MOTOR_CCW, speed);
			break;
		case CAR_BW_RIGHT_STATE:
			control_motor(&motor_left, MOTOR_CCW, speed);
			control_motor(&motor_right, MOTOR_CCW, (int8_t) speed/3);
			break;
		default:
			break;
	}
}

void Car_Control_Wheels(int8_t right_speed, int8_t left_speed)
{
	if (right_speed >= 0)
	{
		control_motor(&motor_right, MOTOR_CW, right_speed);
	}
	else
	{
		control_motor(&motor_right, MOTOR_CCW, -right_speed);
	}
	if (left_speed >= 0)
	{
		control_motor(&motor_left, MOTOR_CW, left_speed);
	}
	else
	{
		control_motor(&motor_left, MOTOR_CCW, -left_speed);
	}
}

void Car_Init(TIM_HandleTypeDef *motor_htim, uint32_t left_motor_channel, uint32_t right_motor_channel, GPIO_TypeDef *left_motor_port,
										  uint16_t left_motor_pin, GPIO_TypeDef *right_motor_port, uint16_t right_motor_pin)
{
	motor_init(&motor_left, left_motor_port, left_motor_pin, motor_htim, left_motor_channel);
	motor_init(&motor_right, right_motor_port, right_motor_pin, motor_htim, right_motor_channel);
	Car_Control(CAR_STOP_STATE, 0);
}
