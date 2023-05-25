#ifndef ARM_AUTO_H
#define ARM_AUTO_H
#include "arm_driver.h"
#include "flash.h"
#include "stm32f1xx_hal.h"

//#define LOAD_ADDR_ID			((uint32_t)0x0801F000)				// Page 124
//#define LOAD_ADDR_CMD			((uint32_t)0x0801F400)				// Page 125
//#define UNLOAD_ADDR_ID		((uint32_t)0x0801F800)				// Page 126
//#define UNLOAD_ADDR_CMD		((uint32_t)0x0801FC00)				// Page 127

//typedef struct
//{
//	uint32_t addr_id;
//	uint32_t addr_cmd;
//}Addr_Id_Cmd;

//Addr_Id_Cmd load_addr = {LOAD_ADDR_ID, LOAD_ADDR_CMD};
//Addr_Id_Cmd unload_addr = {UNLOAD_ADDR_ID, UNLOAD_ADDR_CMD};

void Memorization_Position(Arm_Robot_Typedef *arm_robot);
//void Flash_Save_Position(void);
//void Flash_Read_Position(void);

void Flash_Save_Position_Load(void);
void Flash_Save_Position_Unload(void);
void Flash_Read_Position_Load(void);
void Flash_Read_Position_Unload(void);

void set_default_cmd(void);
//uint8_t arm_auto_by_cmd(void);

uint8_t arm_auto_by_cmd_load(void);
uint8_t arm_auto_by_cmd_unload(void);

#endif
