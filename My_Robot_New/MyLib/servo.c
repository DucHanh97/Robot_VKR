/* Cau hinh phan cung:
		Tan so Counter(CNT) = 1 MHz;
		Cai dat thanh ghi ARR = 19999 (Tan so PWM = 50 Hz (T = 20 ms));
		#define MIN_PULSE_WIDTH	544
		#define MAX_PULSE_WIDTH 2400
*/

#include "servo.h"
#define MIN_PULSE_WIDTH	530
#define MAX_PULSE_WIDTH 2450
uint32_t Map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void Servo_Write(Servo *SV, uint32_t angle)
{
	SV->Angle = angle;
	uint32_t ccr = Map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	switch(SV->Channel)
	{
		case TIM_CHANNEL_1:
			SV->htim->Instance->CCR1 = ccr;
			break;
		case TIM_CHANNEL_2:
			SV->htim->Instance->CCR2 = ccr;
			break;
		case TIM_CHANNEL_3:
			SV->htim->Instance->CCR3 = ccr;
			break;
		case TIM_CHANNEL_4:
			SV->htim->Instance->CCR4 = ccr;
			break;
	}
}
uint8_t Servo_Read(Servo *SV)
{
	return SV->Angle;
}
void Servo_Init(Servo *SV, TIM_HandleTypeDef *htim, uint32_t Channel)
{
	SV->htim = htim;
	SV->Channel = Channel;
	SV->Angle = 0;
	HAL_TIM_PWM_Start(htim, Channel);
}
