#include "arm_auto.h"

extern Arm_Robot_Typedef arm_robot;

//ArmRobotState arm_robot_state_1;
//ArmRobotState arm_robot_state_2;

typedef struct
{
	uint8_t angle_1;
	uint8_t angle_2;
	uint8_t angle_3;
	uint8_t angle_4;
}Position_Arm_Robot;

Position_Arm_Robot buff_location[20];
uint8_t position_id = 0;

Position_Arm_Robot flash_location_load[20];
uint8_t flash_pos_id_load;

Position_Arm_Robot flash_location_unload[20];
uint8_t flash_pos_id_unload;

void Memorization_Position(Arm_Robot_Typedef *arm_robot)
{
	buff_location[position_id].angle_1 = arm_robot->SV_Z.Angle;
	buff_location[position_id].angle_2 = arm_robot->SV_X.Angle;
	buff_location[position_id].angle_3 = arm_robot->SV_Y.Angle;
	buff_location[position_id].angle_4 = arm_robot->SV_K.Angle;
	position_id++;
}

void Flash_Save_Position_Load(void)
{
	flash_unlock();
	flash_erease(0x0801F000);								//page 124
	flash_erease(0x0801F400);								//page 125
	flash_write_arr(0x0801F000, (uint8_t *)&position_id, 1);
	flash_write_arr(0x0801F400, (uint8_t *)buff_location, sizeof(buff_location));
	flash_lock();
	position_id = 0;
}

void Flash_Save_Position_Unload(void)
{
	flash_unlock();
	flash_erease(0x0801F800);								//page 126
	flash_erease(0x0801FC00);								//page 127
	flash_write_arr(0x0801F800, (uint8_t *)&position_id, 1);
	flash_write_arr(0x0801FC00, (uint8_t *)buff_location, sizeof(buff_location));
	flash_lock();
	position_id = 0;
}

void Flash_Read_Position_Load(void)
{
	flash_unlock();
	flash_read_arr(0x0801F000, &flash_pos_id_load, 1);
	if(flash_pos_id_load != 255 && flash_pos_id_load != 0)
	{
//		flash_read_arr(0x0801F400, (uint8_t *)flash_location_load, sizeof(flash_location_load)*flash_pos_id_load);
		flash_read_arr(0x0801F400, (uint8_t *)flash_location_load, sizeof(flash_location_load));
	}
	flash_lock();
}

void Flash_Read_Position_Unload(void)
{
	flash_unlock();
	flash_read_arr(0x0801F800, &flash_pos_id_unload, 1);
	if(flash_pos_id_unload != 255 && flash_pos_id_unload != 0)
	{
//		flash_read_arr(0x0801FC00, (uint8_t *)flash_location_unload, sizeof(flash_location_unload)*flash_pos_id_unload);
		flash_read_arr(0x0801FC00, (uint8_t *)flash_location_unload, sizeof(flash_location_unload));
	}
	flash_lock();
}

Arm_Position_Typedef arm_default_cmd[5] = 
{
	{ORIGIN_Z, ORIGIN_X, ORIGIN_Y, ORIGIN_K},
	{90, 150, 150, 110},
	{90, 150, 150, 90},
	{90, 120, 110, 110},
	{90, 45, 130, 109}
};
uint8_t default_id = 5;
void set_default_cmd(void)
{
	flash_unlock();
	flash_erease(0x0801F800);								//page 126
	flash_erease(0x0801FC00);								//page 127
	flash_write_arr(0x0801F800, (uint8_t *)&default_id, 1);
	flash_write_arr(0x0801FC00, (uint8_t *)arm_default_cmd, sizeof(arm_default_cmd));
	flash_erease(0x0801F000);								//page 126
	flash_erease(0x0801F400);								//page 127
	flash_write_arr(0x0801F000, (uint8_t *)&default_id, 1);
	flash_write_arr(0x0801F400, (uint8_t *)arm_default_cmd, sizeof(arm_default_cmd));
	flash_lock();
}

uint32_t cur_time;
uint8_t i = 0;

uint8_t arm_auto_by_cmd_load(void)
{
	if(HAL_GetTick() - cur_time >= 15)
	{
		cur_time = HAL_GetTick();
		int8_t Z, X, Y, K;
		if(flash_location_load[i].angle_1 == arm_robot.SV_Z.Angle)
		{
			Z = 0;
		}
		else if(flash_location_load[i].angle_1 > arm_robot.SV_Z.Angle)
		{
			Z = 1;
		}
		else
		{
			Z = -1;
		}
		
		if(flash_location_load[i].angle_2 == arm_robot.SV_X.Angle)
		{
			X = 0;
		}
		else if(flash_location_load[i].angle_2 > arm_robot.SV_X.Angle)
		{
			X = 1;
		}
		else
		{
			X = -1;
		}
		
		if(flash_location_load[i].angle_3 == arm_robot.SV_Y.Angle)
		{
			Y = 0;
		}
		else if(flash_location_load[i].angle_3 > arm_robot.SV_Y.Angle)
		{
			Y = 1;
		}
		else
		{
			Y = -1;
		}
		
		if(flash_location_load[i].angle_4 == arm_robot.SV_K.Angle)
		{
			K = 0;
		}
		else if(flash_location_load[i].angle_4 > arm_robot.SV_K.Angle)
		{
			K = 1;
		}
		else
		{
			K = -1;
		}
		
		Arm_Control_by_Step(&arm_robot, Z, X, Y, K);
		if(Z == 0 && X == 0 && Y == 0 && K == 0)
		{
			i++;
		}
		
		if(i == flash_pos_id_load)
		{
			i = 0;
//			state_arm_robot = STOP_CMD;
			return 1;
		}		
	}
	return 0;
}

uint8_t arm_auto_by_cmd_unload(void)
{
	if(HAL_GetTick() - cur_time >= 15)
	{
		cur_time = HAL_GetTick();
		int8_t Z, X, Y, K;
		if(flash_location_unload[i].angle_1 == arm_robot.SV_Z.Angle)
		{
			Z = 0;
		}
		else if(flash_location_unload[i].angle_1 > arm_robot.SV_Z.Angle)
		{
			Z = 1;
		}
		else
		{
			Z = -1;
		}
		
		if(flash_location_unload[i].angle_2 == arm_robot.SV_X.Angle)
		{
			X = 0;
		}
		else if(flash_location_unload[i].angle_2 > arm_robot.SV_X.Angle)
		{
			X = 1;
		}
		else
		{
			X = -1;
		}
		
		if(flash_location_unload[i].angle_3 == arm_robot.SV_Y.Angle)
		{
			Y = 0;
		}
		else if(flash_location_unload[i].angle_3 > arm_robot.SV_Y.Angle)
		{
			Y = 1;
		}
		else
		{
			Y = -1;
		}
		
		if(flash_location_unload[i].angle_4 == arm_robot.SV_K.Angle)
		{
			K = 0;
		}
		else if(flash_location_unload[i].angle_4 > arm_robot.SV_K.Angle)
		{
			K = 1;
		}
		else
		{
			K = -1;
		}
		
		Arm_Control_by_Step(&arm_robot, Z, X, Y, K);
		if(Z == 0 && X == 0 && Y == 0 && K == 0)
		{
			i++;
		}
		
		if(i == flash_pos_id_unload)
		{
			i = 0;
//			state_arm_robot = STOP_CMD;
			return 1;
		}		
	}
	return 0;
}


