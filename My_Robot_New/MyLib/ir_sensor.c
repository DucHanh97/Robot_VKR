#include "ir_sensor.h"

uint8_t ir_sensor_read(void)
{
	uint8_t ir_value = 0;
	ir_value |= HAL_GPIO_ReadPin(IR1_GPIO_Port, IR1_Pin) << 4;
	ir_value |= HAL_GPIO_ReadPin(IR2_GPIO_Port, IR2_Pin) << 3;
	ir_value |= HAL_GPIO_ReadPin(IR3_GPIO_Port, IR3_Pin) << 2;
	ir_value |= HAL_GPIO_ReadPin(IR4_GPIO_Port, IR4_Pin) << 1;
	ir_value |= HAL_GPIO_ReadPin(IR5_GPIO_Port, IR5_Pin) << 0;
	return ir_value;
}

/*
	11111 = 31	error = -6	lost line
	00001 = 01	error = -5	square line
	00011 = 03	error = -5	square line
	01111 = 15	error = -4
	00111 = 07	error = -3
	10111 = 23	error = -2
//	00011 = 03	error = -2
	10011 = 19	error = -1
	11011 = 27	error = 0
	10001 = 17	error = 0
	11001 = 25	error = 1
//	11000 = 24	error = 2
	11101 = 29	error = 2
	11100 = 28	error = 3
	11110 = 30	error = 4
	11000 = 24	error = 5		square line
	10000 = 16	error = 5		square line
	00000 = 0		error = 6
*/

int8_t error_calculate(void)
{
	int8_t error = 0;
	uint8_t ir_value = ir_sensor_read();
	switch(ir_value)
	{
		case 31:
			error = -6;
			break;
		case 1:
			error = -5;
			break;
		case 3:
			error = -5;
			break;
		case 15:
			error = -4;
			break;
		case 7:
			error = -3;
			break;
		case 23:
			error = -2;
			break;
//		case 3:
//			error = -2;
//			break;
		case 19:
			error = -1;
			break;
		case 27:
			error = 0;
			break;
		case 17:
			error = 0;
			break;
		case 25:
			error = 1;
			break;
//		case 24:
//			error = 2;
//			break;
		case 29:
			error = 2;
			break;
		case 28:
			error = 3;
			break;
		case 30:
			error = 4;
			break;
		case 24:
			error = 5;
			break;
		case 16:
			error = 5;
			break;
		case 0:
			error = 6;
			break;
		default:
			break;
	}
	return error;
}
