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
	RIGHT_SQUARE_STATE,
	LEFT_SQUARE_STATE,
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

/* variable for void car_check_side(Servo *ServoHcsr04, float distance) */
int16_t servo_angle = 90;
uint8_t flag_angle = 0;
uint8_t distan_check_cpl = 0;
float right_distance;
float left_distance;
uint32_t time_real;
/************************************************************************/

/* variable for void car_obstacle(void) */
typedef enum
{
	FIRST_SPIN,
	SECOND_SPIN,
	FIRST_FORWARD,
	SECOND_FORWARD,
	FIND_LINE
}ObstacleState;

ObstacleState obstancle_state = FIRST_SPIN;
uint32_t time_delay;
uint16_t count_delay = 0;
/****************************************/

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
//		case -5:
//			miss_way = 2;
//			break;
		case -4:
			miss_way = 3;
			break;
		case -3:
			miss_way = 3;
			break;
		case -2:
			miss_way = 3;
			break;
//		case 5:
//			miss_way = 4;
//			break;
		case 4:
			miss_way = 5;
			break;
		case 3:
			miss_way = 5;
			break;
		case 2:
			miss_way = 5;
			break;
		default:
			break;
	}
}

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

float distan_in_front_of;

void car_auto_state_switch(Servo *ServoHcsr04, float distance)
{
	int8_t error = error_calculate();
	miss_way_value(error);
//	float distan_in_front_of;
	if (Servo_Read(ServoHcsr04) == 90)
	{
		distan_in_front_of = distance;
	}
//	if (follow_line_state != OBSTACLE_STATE)
//	{
//		
//	}
	if (distan_in_front_of <= 10)
	{
		if (follow_line_state == FOLLOW_LINE_STATE)
			follow_line_state = OBSTACLE_STATE;
	}
	else
	{
		if (follow_line_state != OBSTACLE_STATE)
		{
			if (error == -6)
			{
				if (follow_line_state != LOST_LINE_STATE)
				{
					follow_line_state = SEARCH_LINE_STATE;
				}
			}
			else if (error == 6)
			{
				follow_line_state = AT_STATION_STATE;
			}
			else if (error == 5)
			{
				follow_line_state = RIGHT_SQUARE_STATE;
			}
			else if (error == -5)
			{
				follow_line_state = LEFT_SQUARE_STATE;
			}
			else if (error >= -4 && error <= 4)
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
}

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
		case RIGHT_SQUARE_STATE:
		{
			Car_Control_Wheels(BASE_SPEED, 0 - BASE_SPEED);
			break;
		}
		case LEFT_SQUARE_STATE:
		{
			Car_Control_Wheels(0 - BASE_SPEED, BASE_SPEED);
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
					if (search_count >= 200)
					{
						Car_Control(CAR_STOP_STATE, 0);
						follow_line_state = LOST_LINE_STATE;
					}
				}
			}
//			else if (miss_way == 2)
//			{
//				Car_Control_Wheels(0 - BASE_SPEED, BASE_SPEED);
//			}
			else if (miss_way == 3)
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
					if (search_count >= 500)
					{
						Car_Control(CAR_STOP_STATE, 0);
						follow_line_state = LOST_LINE_STATE;
					}
				}
			}
//			else if (miss_way == 4)
//			{
//				Car_Control_Wheels(BASE_SPEED, 0 - BASE_SPEED);
//			}
			else if (miss_way == 5)
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
					if (search_count >= 500)
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
			car_check_side(ServoHcsr04, distance);
			break;
		}
		default:
			break;
	}
}


void car_check_side(Servo *ServoHcsr04, float distance)
{
	if (distan_check_cpl)
	{
		car_obstacle();
	}
	else
	{
		Car_Control_Wheels(0, 0);
		if (HAL_GetTick() - time_real >= 15)
		{
			time_real = HAL_GetTick();
			if (flag_angle)																			// right side
			{
				if (--servo_angle >= 0)
				{
					Servo_Write(ServoHcsr04, servo_angle);
				}
				else
				{
					right_distance = distance;
					flag_angle = 0;
					distan_check_cpl =1;
					Servo_Write(ServoHcsr04, 90);
				}				
			}
			else																								// left side
			{
				if (++servo_angle <= 180)
				{
					Servo_Write(ServoHcsr04, servo_angle);
				}
				else
				{
					left_distance = distance;
					flag_angle = 1;
				}
			}
		}
	}
}

void car_obstacle(void)
{
	if (HAL_GetTick() - time_delay >= 30)
	{
		time_delay = HAL_GetTick();
		if (obstancle_state == FIRST_SPIN)
		{
			if (count_delay <= 18)
			{
				count_delay++;
			}
			else
			{
				count_delay = 0;
				obstancle_state = FIRST_FORWARD;
			}
		}
		else if (obstancle_state == FIRST_FORWARD)
		{
			if (count_delay <= 18)
			{
				count_delay++;
			}
			else
			{
				count_delay = 0;
				obstancle_state = SECOND_SPIN;
			}
		}
		else if (obstancle_state == SECOND_SPIN)
		{
			if (count_delay <= 18)
			{
				count_delay++;
			}
			else
			{
				count_delay = 0;
				obstancle_state = SECOND_FORWARD;
			}
		}
		else if (obstancle_state == SECOND_FORWARD)
		{
			if (count_delay <= 30)
			{
				count_delay++;
			}
			else
			{
				count_delay = 0;
				obstancle_state = SECOND_SPIN;
			}
		}
		
	}
	if (right_distance > left_distance)// && right_distance > 50)
	{		
		switch (obstancle_state)
		{
			case FIRST_SPIN:
			{
				Car_Control_Wheels(-50, 50);
				break;
			}
			case SECOND_SPIN:
			{
				Car_Control_Wheels(50, -50);
				break;
			}
			case FIRST_FORWARD:
			{
				if (error_calculate() != -6)
				{
					obstancle_state = FIND_LINE;
				}
				Car_Control_Wheels(BASE_SPEED, BASE_SPEED);
				break;
			}
			case SECOND_FORWARD:
			{
				if (error_calculate() != -6)
				{
					obstancle_state = FIND_LINE;
				}
				Car_Control_Wheels(BASE_SPEED, BASE_SPEED);
				break;
			}
			case FIND_LINE:
			{
				Car_Control_Wheels(BASE_SPEED, BASE_SPEED);
				HAL_Delay(250);
				Car_Control_Wheels(-50, 50);
				HAL_Delay(550);
				follow_line_state = FOLLOW_LINE_STATE;
				obstancle_state = FIRST_SPIN;
				distan_check_cpl = 0;
				break;
			}
			default:
				break;
		}
	}
	else if (right_distance < left_distance)// && left_distance > 50)
	{		
		switch (obstancle_state)
		{
			case FIRST_SPIN:
			{
				Car_Control_Wheels(50, -50);
				break;
			}
			case SECOND_SPIN:
			{
				Car_Control_Wheels(-50, 50);
				break;
			}
			case FIRST_FORWARD:
			{
				if (error_calculate() != -6)
				{
					obstancle_state = FIND_LINE;
				}
				Car_Control_Wheels(BASE_SPEED, BASE_SPEED);
				break;
			}
			case SECOND_FORWARD:
			{
				if (error_calculate() != -6)
				{
					obstancle_state = FIND_LINE;
				}
				Car_Control_Wheels(BASE_SPEED, BASE_SPEED);
				break;
			}
			case FIND_LINE:
			{
				Car_Control_Wheels(BASE_SPEED, BASE_SPEED);
				HAL_Delay(250);
				Car_Control_Wheels(50, -50);
				HAL_Delay(550);
				follow_line_state = FOLLOW_LINE_STATE;
				obstancle_state = FIRST_SPIN;
				distan_check_cpl = 0;
				break;
			}
			default:
				break;
		}
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
