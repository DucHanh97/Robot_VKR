#include "Motor.h"
#include "PWM.h"
/*Speed 0 -> 100*/
void control_motor(Motor_TypeDef *motor, MotorState StateMotor, uint8_t speed)
{
	switch(StateMotor)
	{
		case MOTOR_STOP:
			HAL_GPIO_WritePin(motor->io_Port, motor->io_pin, GPIO_PIN_RESET);
			PWM_set_duty(motor->htim, motor->tim_channel, 0);
			break;
		case MOTOR_CW:
			HAL_GPIO_WritePin(motor->io_Port, motor->io_pin, GPIO_PIN_RESET);
			PWM_set_duty(motor->htim, motor->tim_channel, speed);
			break;
		case MOTOR_CCW:
			HAL_GPIO_WritePin(motor->io_Port, motor->io_pin, GPIO_PIN_SET);
			PWM_set_duty(motor->htim, motor->tim_channel, 100 - speed);
			break;
	}
}
void motor_init(Motor_TypeDef *Motor, GPIO_TypeDef *io_port, uint16_t io_pin,
								TIM_HandleTypeDef *htim, uint32_t tim_channel)
{
	Motor->io_Port = io_port;
	Motor->io_pin = io_pin;
	Motor->htim = htim;
	Motor->tim_channel = tim_channel;
	Motor->motor_state = MOTOR_STOP;
	Motor->speed = 0;
	HAL_TIM_PWM_Start(htim, tim_channel);
}
