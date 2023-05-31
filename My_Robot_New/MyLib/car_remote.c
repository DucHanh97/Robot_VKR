#include "car_remote.h"
#include "car_driver.h"
#include "arm_driver.h"

extern Arm_Robot_Typedef arm_robot;
CarState car_state;
RobotRemoteState remote_state = REMOTE_STOP_STATE;

uint8_t SPEED;
int8_t arm_step[4] = {0};
int8_t flag_arm = 0;

void set_car_params_remote(int8_t argv[6])
{
	if (remote_state == REMOTE_CAR_STATE)
	{
		if (argv[1] == 0 && argv[2] == 0)
		{
			car_state = CAR_STOP_STATE;
			SPEED = 0;
		}
		if (argv[1] > 0 && argv[2] == 0)
		{
			car_state = CAR_FORWARD_STATE;
			SPEED = argv[1] * 20;
		}
		else if (argv[1] < 0 && argv[2] == 0)
		{
			car_state = CAR_BACKWARD_STATE;
			SPEED = -argv[1] * 20;
		}
		else if (argv[1] == 0 && argv[2] > 0)
		{
			car_state = CAR_LEFT_STATE;
			SPEED = argv[2] * 10;
		}
		else if (argv[1] == 0 && argv[2] < 0)
		{
			car_state = CAR_RIGHT_STATE;
			SPEED = -argv[2] * 10;
		}
		else if (argv[1] > 0 && argv[2] > 0)
		{
			car_state = CAR_FW_RIGHT_STATE;
			SPEED = argv[1] * 20;
		}
		else if (argv[1] > 0 && argv[2] < 0)
		{
			car_state = CAR_FW_LEFT_STATE;
			SPEED = argv[1] * 20;
		}
		else if (argv[1] < 0 && argv[2] > 0)
		{
			car_state = CAR_BW_RIGHT_STATE;
			SPEED = -argv[1] * 20;
		}
		else if (argv[1] < 0 && argv[2] < 0)
		{
			car_state = CAR_BW_LEFT_STATE;
			SPEED = -argv[1] * 20;
		}
	}
	if (remote_state == REMOTE_ARM_STATE)
	{
		arm_step[0] = argv[0];
		arm_step[1] = argv[1];
		arm_step[2] = argv[3];
		arm_step[3] = argv[2];
		flag_arm = 1;
	}
	if (argv[4] == 1)
	{
		argv[4] = 0;
		remote_state = REMOTE_CAR_STATE;
	}
	if (argv[5] == 1)
	{
		argv[5] = 0;
		remote_state = REMOTE_ARM_STATE;
	}
}

void car_state_remote_excuse(void)
{
	switch(remote_state)
	{
		case REMOTE_STOP_STATE:
		{
			Car_Control(CAR_STOP_STATE, 0);
			break;
		}
		case REMOTE_CAR_STATE:
		{
			Car_Control(car_state, SPEED);
			break;
		}
		case REMOTE_ARM_STATE:
		{
			if (flag_arm)
			{
				Arm_Control_by_Step(&arm_robot, 0 - arm_step[0], arm_step[1], arm_step[2], arm_step[3]);
				flag_arm = 0;
			}
			break;
		}
		default:
			break;
	}
}

void car_remote_handle(void)
{
	car_state_remote_excuse();
}
