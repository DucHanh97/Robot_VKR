#include "uart.h"
#include <string.h>
#include <stdlib.h>
#include "car_remote.h"
#include "car_auto.h"

extern RobotState robot_state;

char buff_data[30];
uint8_t buff_id = 0;
uint8_t rx_cpl_flag = 0;

void Uart_Receive_Data(uint8_t rx_data)
{
	if(rx_data == '\n')
	{
		buff_data[buff_id] = '\0';
		buff_id = 0;
		rx_cpl_flag = 1;
	}
	else
	{
		buff_data[buff_id++] = rx_data;
	}
}

void Uart_Handle(void)
{
	if(rx_cpl_flag)
	{
		int8_t argv[6];
		char * token = strtok(buff_data, ",");
		while(token != NULL)
		{
			if(*token == 'X' && *(token+1) == ':')
			{
				argv[0] = atoi(token+2);
			}
			else if(*token == 'Y' && *(token+1) == ':')
			{
				argv[1] = atoi(token+2);
			}
			else if(*token == 'I' && *(token+1) == ':')
			{
				argv[2] = atoi(token+2);
			}
			else if(*token == 'K' && *(token+1) == ':')
			{
				argv[3] = atoi(token+2);
			}
			else if(*token == 'A' && *(token+1) == ':')
			{
				argv[4] = atoi(token+2);
			}
			else if(*token == 'B' && *(token+1) == ':')
			{
				argv[5] = atoi(token+2);
			}
			token = strtok(NULL, ",");
		}
		if (robot_state == REMOTE_STATE)
		{
			set_car_params_remote(argv);
		}
		if (robot_state == AUTO_STATE)
		{
			arm_set_cmd(argv);
		}
		rx_cpl_flag = 0;
	}
}
