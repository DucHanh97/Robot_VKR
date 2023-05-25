#include "arm_driver.h"
#include "stm32f1xx_hal.h"

void Arm_Robot_Init(Arm_Robot_Typedef *arm_rb,TIM_HandleTypeDef *sv_tim)
{
	Servo_Init(&arm_rb->SV_Z, sv_tim, TIM_CHANNEL_1);
	Servo_Init(&arm_rb->SV_X, sv_tim, TIM_CHANNEL_2);
	Servo_Init(&arm_rb->SV_Y, sv_tim, TIM_CHANNEL_3);
	Servo_Init(&arm_rb->SV_K, sv_tim, TIM_CHANNEL_4);
	
	Set_Default_State(arm_rb);
}
/*
	SV1: 0 - 180;
	SV2: 0 - 180;
	SV3: 50 - 180
	SV4: 88 - 116;
	*/
void Arm_Control_by_Step(Arm_Robot_Typedef *arm_rb, int8_t Z, int8_t X, int8_t Y, int8_t K)
{
	if((arm_rb->SV_Z.Angle + Z) <=180 && (arm_rb->SV_Z.Angle + Z) >= 0)
	{
		Servo_Write(&arm_rb->SV_Z, arm_rb->SV_Z.Angle + Z);
	}
	if((arm_rb->SV_X.Angle + X) <=180 && (arm_rb->SV_X.Angle + X) >= 0)
	{
		Servo_Write(&arm_rb->SV_X, arm_rb->SV_X.Angle + X);
	}
	if((arm_rb->SV_Y.Angle + Y) <=155 && (arm_rb->SV_Y.Angle + Y) >= 0)
	{
		Servo_Write(&arm_rb->SV_Y, arm_rb->SV_Y.Angle + Y);
	}
	if((arm_rb->SV_K.Angle + K) <=116 && (arm_rb->SV_K.Angle + K) >= 87)
	{
		Servo_Write(&arm_rb->SV_K, arm_rb->SV_K.Angle + K);
	}
}

//uint32_t cur_time;

//void Arm_Go_to_Position(Arm_Robot_Typedef *arm_rb, Arm_Position_Typedef *position)
//{
//	uint32_t cur_time;
//	
//	while (arm_rb->SV_Z.Angle != position->Angle_Z || arm_rb->SV_X.Angle != position->Angle_X || arm_rb->SV_Y.Angle != position->Angle_Y || arm_rb->SV_K.Angle != position->Angle_K)
//	{
//		if (HAL_GetTick() - cur_time >= 30)
//		{
//			cur_time = HAL_GetTick();
//			int8_t Z, X, Y, K;
//			/* -------- By Z ---------- */
//			if (position->Angle_Z == arm_rb->SV_Z.Angle)
//				Z = 0;
//			else if (position->Angle_Z > arm_rb->SV_Z.Angle)
//				Z = 1;
//			else
//				Z = -1;
//			/* -------- By X ---------- */
//			if (position->Angle_X == arm_rb->SV_X.Angle)
//				X = 0;
//			else if (position->Angle_X > arm_rb->SV_X.Angle)
//				X = 1;
//			else
//				X = -1;
//			/* -------- By Y ---------- */
//			if (position->Angle_Y == arm_rb->SV_Y.Angle)
//				Y = 0;
//			else if (position->Angle_Y > arm_rb->SV_Y.Angle)
//				Y = 1;
//			else
//				Y = -1;
//			/* -------- By K ---------- */
//			if (position->Angle_K == arm_rb->SV_K.Angle)
//				K = 0;
//			else if (position->Angle_K > arm_rb->SV_K.Angle)
//				K = 1;
//			else
//				K = -1;
//			
//			Arm_Control_by_Step(arm_rb, Z, X, Y, K);
//		}
//	}
//}

void Set_Default_State(Arm_Robot_Typedef *arm_rb)
{
	Servo_Write(&arm_rb->SV_Z, ORIGIN_Z);
	Servo_Write(&arm_rb->SV_X, ORIGIN_X);
	Servo_Write(&arm_rb->SV_Y, ORIGIN_Y);
	Servo_Write(&arm_rb->SV_K, ORIGIN_K);
}
