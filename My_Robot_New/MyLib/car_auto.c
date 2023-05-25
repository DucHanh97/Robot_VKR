#include "car_auto.h"
#include "ir_sensor.h"
#include "car_driver.h"
#include "arm_driver.h"
#include "arm_auto.h"
#include "stm32f1xx_hal.h"


extern Arm_Robot_Typedef arm_robot;
extern UART_HandleTypeDef huart1;

static int8_t		P = 0;
static int16_t		I = 0;
static int8_t		D = 0;
static int8_t		Pre_Error = 0;

typedef enum
{
	FOLLOW_LINE_STATE,
	LOST_LINE_STATE,
	SEARCH_LINE_STATE,
	AT_STATION_STATE,
	OBSTACLE_STATE
}Follow_Line_State;

typedef enum
{
//	STOP_CMD,
	RUN_CMD,
	LEARN_CMD
}ArmRobotState;

typedef enum
{
	LOAD_STATE,
	UNLOAD_STATE
}ArmDutyState;

Follow_Line_State follow_line_state = FOLLOW_LINE_STATE;
ArmRobotState arm_robot_state;
ArmDutyState arm_duty_state;

uint8_t miss_way;
uint16_t search_count;
uint32_t search_time;

uint8_t arm_duty_compl = 0;
uint8_t arm_learn_cmd_flag = 0;

void miss_way_value(int8_t error)
{
	switch(error)
	{
		case 0:
			miss_way = 1;
			break;
		case 1:
			miss_way = 1;
			break;
		case -1:
			miss_way = 1;
			break;
		case -4:
			miss_way = 2;
			break;
		case -3:
			miss_way = 2;
			break;
		case -2:
			miss_way = 2;
			break;
		case 4:
			miss_way = 3;
			break;
		case 3:
			miss_way = 3;
			break;
		case 2:
			miss_way = 3;
			break;
		default:
			break;
	}
}
/*
int16_t PID_value_calculate(void)
{
	int8_t error = error_calculate();
	if (error == 6)
	{
		follow_line_state = AT_STATION_STATE;
		if (pick_state == 0)
			pick_state = 1;
		if (pick_state == 1)
			pick_state = 0;
	}
	else if (error == -6)
	{
		if (follow_line_state != LOST_LINE_STATE)
		{
			follow_line_state = SEARCH_LINE_STATE;
			search_count = 0;
		}
	}
	else
	{
		follow_line_state = FOLLOW_LINE_STATE;
		P = error;
		I += error;
		D = error - Pre_Error;
		Pre_Error = error;
		int PID_value = (int16_t)(Kp*P + Ki*I + Kd*D);
		return PID_value;
	}
	return 0;
}
*/
int16_t PID_value_calculate(void)
{
	int8_t error = error_calculate();
	
	P = error;
	I += error;
	D = error - Pre_Error;
	Pre_Error = error;
	
	int PID_value = (int16_t)(Kp*P + Ki*I + Kd*D);
	
	return PID_value;
}

void car_auto_state_switch(Servo *ServoHcsr04, float distance)
{
	int8_t error = error_calculate();
	miss_way_value(error);
	uint8_t distan_in_front_of;
	if (Servo_Read(ServoHcsr04) == 90)
	{
		distan_in_front_of = distance;
	}
	if (follow_line_state != OBSTACLE_STATE)
	{
		
	}
	if (distan_in_front_of <= 20)
	{
		if (follow_line_state == FOLLOW_LINE_STATE)
			follow_line_state = OBSTACLE_STATE;
	}
	else
	{
		if (error == -6)
		{
			if (follow_line_state != LOST_LINE_STATE)
			{
				follow_line_state = SEARCH_LINE_STATE;
			}
		}
		if (error == 6)
		{
			follow_line_state = AT_STATION_STATE;
		}
		if (error >= -5 && error <= 5)
		{
			follow_line_state = FOLLOW_LINE_STATE;
		}
		if (follow_line_state == AT_STATION_STATE)
		{
			if (arm_robot_state == RUN_CMD)
			{
				if (arm_duty_compl == 1)
				{
					arm_duty_compl = 0;
					if (arm_duty_state == LOAD_STATE)
					{
						arm_duty_state = UNLOAD_STATE;
					}
					else if (arm_duty_state == UNLOAD_STATE)
					{
						arm_duty_state = LOAD_STATE;
					}
				}
			}
		}
	}
}

//void arm_auto_state_switch(void)
//{
//	
//}

void arm_set_cmd(int8_t argv[6])
{
	if (follow_line_state == AT_STATION_STATE)
	{
		if (argv[4] == 2)
		{
			argv[4] = 0;
			arm_robot_state = LEARN_CMD;
			HAL_UART_Transmit(&huart1, (uint8_t *)"1", 1, 10);
		}
		if (arm_robot_state == LEARN_CMD)
		{
			Arm_Control_by_Step(&arm_robot, argv[0], argv[1], argv[3], argv[2]);
			if (argv[5] == 2)
			{
				Memorization_Position(&arm_robot);
			}
		}
		if (argv[5] == 5)
		{
			argv[5] = 0;
			HAL_UART_Transmit(&huart1, (uint8_t *)"2", 1, 10);
			if (arm_duty_state == LOAD_STATE)
			{
				Flash_Save_Position_Load();
				Flash_Read_Position_Load();
			}
			else if (arm_duty_state == UNLOAD_STATE)
			{
				Flash_Save_Position_Unload();
				Flash_Read_Position_Unload();
			}
			arm_robot_state = RUN_CMD;
		}
	}
}



void car_auto_state_handle(Servo *ServoHcsr04, float distance)
{
	switch (follow_line_state)
	{
		case FOLLOW_LINE_STATE:
		{
			int8_t delta_pid_speed = (int8_t)(PID_value_calculate() / 10);
			int8_t right_speed = BASE_SPEED + delta_pid_speed;
			int8_t left_speed = BASE_SPEED - delta_pid_speed;
			Car_Control_Wheels(right_speed, left_speed);
			break;
		}
		case LOST_LINE_STATE:
		{
			Car_Control_Wheels(0, 0);
			break;
		}
		case SEARCH_LINE_STATE:
		{
			if (miss_way == 1)
			{
				Car_Control(CAR_FORWARD_STATE, BASE_SPEED);
				if (HAL_GetTick() - search_time >= 5)
				{
					search_time = HAL_GetTick();
					search_count++;
//					if (error_calculate() != -6)
//					{
//						follow_line_state = FOLLOW_LINE_STATE;
//					}
					if (search_count >= 120)
					{
						Car_Control(CAR_STOP_STATE, 0);
						follow_line_state = LOST_LINE_STATE;
					}
				}
			}
			if (miss_way == 2)
			{
				Car_Control_Wheels(0, BASE_SPEED);
				if (HAL_GetTick() - search_time >= 5)
				{
					search_time = HAL_GetTick();
					search_count++;
//					if (error_calculate() != -6)
//					{
//						follow_line_state = FOLLOW_LINE_STATE;
//					}
					if (search_count >= 200)
					{
						Car_Control(CAR_STOP_STATE, 0);
						follow_line_state = LOST_LINE_STATE;
					}
				}
			}
			if (miss_way == 3)
			{
				Car_Control_Wheels(BASE_SPEED, 0);
				if (HAL_GetTick() - search_time >= 5)
				{
					search_time = HAL_GetTick();
					search_count++;
//					if (error_calculate() != -6)
//					{
//						follow_line_state = FOLLOW_LINE_STATE;
//					}
					if (search_count >= 200)
					{
						Car_Control(CAR_STOP_STATE, 0);
						follow_line_state = LOST_LINE_STATE;
					}
				}
			}
			break;
		}
		case AT_STATION_STATE:
		{
			Car_Control_Wheels(0, 0);
			
			switch (arm_robot_state)
			{
				case RUN_CMD:
				{
					switch (arm_duty_state)
					{
						case LOAD_STATE:
						{
							if (arm_auto_by_cmd_load())
							{
								arm_duty_compl = 1;
								Car_Control_Wheels(BASE_SPEED, BASE_SPEED);
								HAL_Delay(150);
							}
							break;
						}
						case UNLOAD_STATE:
						{
							if (arm_auto_by_cmd_unload())
							{
								arm_duty_compl = 1;
								Car_Control_Wheels(BASE_SPEED, BASE_SPEED);
								HAL_Delay(150);
							}
							break;
						}
						default:
							break;
					}
					break;
				}
				case LEARN_CMD:
				{
					
					break;
				}
				default:
					break;
			}
			break;
		}
		case OBSTACLE_STATE:
		{
			Car_Control_Wheels(0, 0);
			car_check_side(ServoHcsr04, distance);
			break;
		}
		default:
			break;
	}
}

int16_t servo_angle = 90;
uint8_t flag_angle = 0;
uint8_t distan_check_cpl = 0;
float right_distance;
float left_distance;
uint32_t time_real;

void car_check_side(Servo *ServoHcsr04, float distance)
{
	if (Servo_Read(ServoHcsr04) == 0)
	{
		right_distance = distance;
	}
	if (Servo_Read(ServoHcsr04) == 180)
	{
		left_distance = distance;
	}
	if (distan_check_cpl)
	{
		car_obstacle();
	}
	else
	{
		if (HAL_GetTick() - time_real >= 20)
		{
			time_real = HAL_GetTick();
			if (flag_angle == 0)																			// right side
			{
				if (--servo_angle >= 0)
				{
					Servo_Write(ServoHcsr04, servo_angle);
//					servo_angle--;
				}
				else
				{
					flag_angle = 1;
				}
			}
			else if (flag_angle == 1)																	// left side
			{
				if (++servo_angle <= 178)
				{
					Servo_Write(ServoHcsr04, servo_angle);
				}
				else
				{
					flag_angle = 0;
					distan_check_cpl =1;
				}
			}
		}
	}
}
uint8_t hihi_check;

void car_obstacle(void)
{
	if (right_distance > left_distance)
	{
		hihi_check = 1;
	}
}


/*********************************************************************************************************/





/*
Arm_Position_Typedef arm_position[5] = 
{
	{ORIGIN_Z, ORIGIN_X, ORIGIN_Y, ORIGIN_K},
	{90, 150, 155, 110},
	{90, 150, 155, 90},
	{90, 120, 110, 110},
	{90, 45, 165, 109}
};

void check_for_obstruction(float distance)
{
	if (distance <= 20)
	{
		if (follow_line_state == FOLLOW_LINE_STATE)
			follow_line_state = OBSTACLE_STATE;
	}
	if (follow_line_state == OBSTACLE_STATE)
		if (distance > 20)
			follow_line_state = FOLLOW_LINE_STATE;
}

void car_following_line_handle(void)
{
	switch(follow_line_state)
	{
		case FOLLOW_LINE_STATE:
		{
			miss_way_value();
			int8_t delta_pid_speed = (int8_t)(PID_value_calculate() / 10);
			int8_t right_speed = BASE_SPEED + delta_pid_speed;
			int8_t left_speed = BASE_SPEED - delta_pid_speed;
			Car_Control_Wheels(right_speed, left_speed);
			break;
		}
		case LOST_LINE_STATE:
		{
			if (error_calculate() != -6)
			{
				follow_line_state = FOLLOW_LINE_STATE;
			}
			break;
		}
		case SEARCH_LINE_STATE:
		{
			if (miss_way == 1)
			{
				Car_Control(CAR_FORWARD_STATE, BASE_SPEED);
				if (HAL_GetTick() - search_time >= 5)
				{
					search_time = HAL_GetTick();
					search_count++;
					if (error_calculate() != -6)
					{
						follow_line_state = FOLLOW_LINE_STATE;
					}
					if (search_count >= 120)
					{
						Car_Control(CAR_STOP_STATE, 0);
						follow_line_state = LOST_LINE_STATE;
					}
				}
			}
			if (miss_way == 2)
			{
				Car_Control_Wheels(0, BASE_SPEED);
				if (HAL_GetTick() - search_time >= 5)
				{
					search_time = HAL_GetTick();
					search_count++;
					if (error_calculate() != -6)
					{
						follow_line_state = FOLLOW_LINE_STATE;
					}
					if (search_count >= 200)
					{
						Car_Control(CAR_STOP_STATE, 0);
						follow_line_state = LOST_LINE_STATE;
					}
				}
			}
			if (miss_way == 3)
			{
				Car_Control_Wheels(BASE_SPEED, 0);
				if (HAL_GetTick() - search_time >= 5)
				{
					search_time = HAL_GetTick();
					search_count++;
					if (error_calculate() != -6)
					{
						follow_line_state = FOLLOW_LINE_STATE;
					}
					if (search_count >= 200)
					{
						Car_Control(CAR_STOP_STATE, 0);
						follow_line_state = LOST_LINE_STATE;
					}
				}
			}
			break;
		}
		case AT_STATION_STATE:
		{
			if (pick_state == 0)			// pick - up merchandise
			{
				Car_Control(CAR_STOP_STATE, 0);
				HAL_Delay(3000);
//				Arm_Go_to_Position(&arm_robot, &arm_position[1]);
//				Arm_Go_to_Position(&arm_robot, &arm_position[2]);
//				Arm_Go_to_Position(&arm_robot, &arm_position[0]);
				follow_line_state = FOLLOW_LINE_STATE;
				Car_Control(CAR_FORWARD_STATE, 100);
				Car_Control(CAR_FORWARD_STATE, BASE_SPEED);
				HAL_Delay(500);
			}
			if (pick_state == 1)			// pick - down
			{
				Car_Control(CAR_STOP_STATE, 0);
				HAL_Delay(3000);
				follow_line_state = FOLLOW_LINE_STATE;
				Car_Control(CAR_FORWARD_STATE, 100);
				Car_Control(CAR_FORWARD_STATE, BASE_SPEED);
				HAL_Delay(500);
			}
			break;
		}
		case OBSTACLE_STATE:
		{
			Car_Control(CAR_STOP_STATE, 0);
			break;
		}
		default:
			break;
	}
	
}
*/
