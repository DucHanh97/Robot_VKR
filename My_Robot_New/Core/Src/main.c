/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "keypad.h"
#include "LiquidCrystal_I2C.h"
#include "flash.h"
#include "uart.h"
#include "car_driver.h"
#include "car_remote.h"
#include "arm_driver.h"
#include "hcsr04.h"
#include "car_auto.h"
#include "arm_auto.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

LiquidCrystal_I2C hlcd1;
Arm_Robot_Typedef arm_robot;
HCSR04_TypeDef hc_sr04;
Servo servo_hcsr04;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*------------------- DECLARE STRUCTURES AND STATE MACHINE -------------------*/
typedef enum
{
	LOCK_STATE,
	ENTER_PASS_STATE,
	SET_PASS_STATE,
	NORMAL_STATE,
	LOCK_30S_STATE
}KeypadState;

typedef enum
{
	STOP_STATE,
	REMOTE_STATE,
	AUTO_STATE
}RobotState;

/*************************** GLOBAL PARAMETERS FOR MANY FUNCTIONS *******************************/
RobotState robot_state = STOP_STATE;

/***************************************** UART EXCUSE ******************************************/

uint8_t rx_data;
uint8_t buff_button[3];
uint8_t id_button;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		Uart_Receive_Data(rx_data);
		if (rx_data == 'A' || rx_data == 'B')
		{
			id_button = 0;
		}
		if (id_button < 3)
			buff_button[id_button++] = rx_data;
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}

/*										REMOTE SET STATE										*/
void remote_set_state(void)
{
	if (buff_button[0] == 'A' && buff_button[2] == '3')
	{
		memset(buff_button, 0, 3);
		HAL_UART_Transmit(&huart1, (uint8_t *)"2", 1, 10);
		
		lcd_clear_display(&hlcd1);
		lcd_printf(&hlcd1, "AUTO_STATE");
		lcd_set_cursor_off(&hlcd1);
		lcd_set_cursor_blink_off(&hlcd1);
		
		robot_state = AUTO_STATE;
	}
	if (buff_button[0] == 'B' && buff_button[2] == '3')
	{
		memset(buff_button, 0, 3);
		HAL_UART_Transmit(&huart1, (uint8_t *)"1", 1, 10);
		
		lcd_clear_display(&hlcd1);
		lcd_printf(&hlcd1, "REMOTE_STATE");
		lcd_set_cursor_off(&hlcd1);
		
		lcd_set_cursor_blink_off(&hlcd1);
		robot_state = REMOTE_STATE;
	}
}

/***************************************** HCSR04 EXCUSE ******************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == HCSR_ECHO_Pin)
	{
		EXTI_HCSR04_Callback(&hc_sr04);
	}
}

void HCSR04_Complete_Callback(HCSR04_TypeDef *hcsr04)
{
	if (&hc_sr04 == hcsr04)
	{
		if (hc_sr04.hcsr04_distan < 20)
		{
			lcd_clear_display(&hlcd1);
			lcd_printf(&hlcd1, "  DISTANCE");
			lcd_set_cursor(&hlcd1, 1, 0);
			lcd_printf(&hlcd1, "%.3f", hc_sr04.hcsr04_distan);
		}
//		check_for_obstruction(hc_sr04.hcsr04_distan);
	}
}


/***************************************** KEYPAD EXCUSE ******************************************/
typedef struct
{
	uint8_t buff[8];
	uint8_t index;
}password_Typedef;

KeypadState keypad_state = LOCK_STATE;
password_Typedef password;

uint8_t default_pass[8];
uint8_t count_error = 3;
uint16_t lock_start_time;
uint8_t keypad_flag = 1;
uint32_t hcsr04_time;

/*------------------------------- PRESS --------------------------------*/
void KeypadPressingCallback(uint8_t key)
{
	switch(keypad_state)
	{
		case LOCK_STATE:
			break;
		
		case ENTER_PASS_STATE:
		{	
			if(key >= '0' && key <= '9')
			{
				if(password.index < 8)
				{
					password.buff[password.index++] = key;
					lcd_putchar(&hlcd1, key);
				}
			}
			else if(key == 'A')
			{
				password.index++;
				lcd_set_cursor(&hlcd1, 1, password.index);
			}
			else if(key == 'B')
			{
				password.index--;
				lcd_set_cursor(&hlcd1, 1, password.index);
			}
			else if(key == 'C')
			{
				lcd_clear_display(&hlcd1);
				lcd_printf(&hlcd1, "Enter password");
				lcd_set_cursor(&hlcd1, 1, 0);
				password.index = 0;
			}
			else if(key == 'D')
			{
				/* Check password */
				if(strncmp((char *)default_pass, (char *)password.buff, 8) == 0)
//				if(strncmp((char *)default_pass, (char *)password.buff, 1) == 0)
				{
					/* password is true */
					keypad_state = NORMAL_STATE;
					lcd_clear_display(&hlcd1);
					lcd_printf(&hlcd1, "Press A to Auto");
					lcd_set_cursor(&hlcd1, 1, 0);
					lcd_printf(&hlcd1, "B to Remote: ");
				}
				else
				{
					/* password is fail */
					password.index = 0;
					count_error--;
					lcd_clear_display(&hlcd1);
					lcd_printf(&hlcd1, "%d time left", count_error);
					lcd_set_cursor(&hlcd1, 1, 0);
					if(count_error == 0)
					{
						keypad_state = LOCK_30S_STATE;
						lock_start_time = HAL_GetTick();
						keypad_flag = 1;
						lcd_set_cursor_off(&hlcd1);
						lcd_set_cursor_blink_off(&hlcd1);
					}
				}
			}
			break;
		}
		
		case SET_PASS_STATE:
		{
			if(key >= '0' && key <= '9')
			{
				if(password.index < 8)
				{
					password.buff[password.index++] = key;
					lcd_putchar(&hlcd1, key);
				}
			}
			else if(key == 'A')
			{
				password.index++;
				lcd_set_cursor(&hlcd1, 1, password.index);
			}
			else if(key == 'B')
			{
				password.index--;
				lcd_set_cursor(&hlcd1, 1, password.index);
			}
			if (password.index == 8)
			{
				lcd_set_cursor(&hlcd1, 0, 0);
				lcd_printf(&hlcd1, "L_Press B: save");
			}
			break;
		}
		
		case NORMAL_STATE:
		{
			if(key == 'A')
			{
				robot_state = AUTO_STATE;
				lcd_clear_display(&hlcd1);
				lcd_printf(&hlcd1, "AUTO_STATE");
				lcd_set_cursor_off(&hlcd1);
				lcd_set_cursor_blink_off(&hlcd1);
				HAL_UART_Transmit(&huart1, (uint8_t *)"2", 1, 10);
				HAL_UART_Receive_IT(&huart1, &rx_data, 1);
			}
			else if(key == 'B')
			{
				robot_state = REMOTE_STATE;
				lcd_clear_display(&hlcd1);
				lcd_printf(&hlcd1, "REMOTE_STATE");
				lcd_set_cursor_off(&hlcd1);
				lcd_set_cursor_blink_off(&hlcd1);
				HAL_UART_Transmit(&huart1, (uint8_t *)"1", 1, 10);
				HAL_UART_Receive_IT(&huart1, &rx_data, 1);
			}
			break;
		}
		
		case LOCK_30S_STATE:
			break;
	}
}

/*------------------------------- LONG PRESS --------------------------------*/
void KeypadPressingTimeoutCallback(uint8_t key)
{
	if(key == 'D')																					// Enter password
	{
		keypad_flag = 1;
		keypad_state = LOCK_STATE;
	}
	
	else if (key == 'B')																		// Save new password
	{
		if (keypad_state == SET_PASS_STATE)
		{
			//strcpy((char *)buff_pass, (char *)password.buff);
			flash_unlock();
			flash_erease(0x0801EC00);
			flash_write_arr(0x0801EC00, password.buff, 8);
			flash_lock();
			keypad_flag = 1;
			keypad_state = LOCK_STATE;
		}
	}
	
	else if (key == 'C')																		// SET new password
	{
		if (keypad_state == NORMAL_STATE || keypad_state == SET_PASS_STATE)
		{
			password.index = 0;
			lcd_clear_display(&hlcd1);
			lcd_set_cursor(&hlcd1, 0, 0);
			lcd_printf(&hlcd1, "Enter new pass:");
			lcd_set_cursor(&hlcd1, 1, 0);
			lcd_set_cursor_blink_on(&hlcd1);
			keypad_state = SET_PASS_STATE;
		}
	}
}

/*-------------------------- KEYPAD STATE HANDLE (is called in WHILE LOOP) ----------------------------*/
void Keypad_State_Handle(void)
{
	switch(keypad_state)
	{
		case LOCK_STATE:
		{
			if(keypad_flag)
			{
				keypad_flag = 0;
				/*----- READ password FROM FLASH MEMORY -----*/
				flash_unlock();
				flash_read_arr(0x0801EC00, default_pass, 8);
				flash_lock();
				/*-------------------------------------------*/
				
				/*---------------- CHECK PASS ---------------*/
//				for (uint8_t i = 0; i < 8; i++)
//				{
//					if (default_pass[i] < '0' || default_pass[i] > '9')
//					{
//						lcd_set_cursor(&hlcd1, 0, 0);
//						lcd_printf(&hlcd1, "Enter new pass:");
//						lcd_set_cursor(&hlcd1, 1, 0);
//						lcd_set_cursor_blink_on(&hlcd1);
//						keypad_state = SET_PASS_STATE;
//						break;
//					}
//					else
//					{
//						lcd_clear_display(&hlcd1);
//						lcd_printf(&hlcd1, "Long press D");
//						lcd_set_cursor(&hlcd1, 1, 0);
//						lcd_printf(&hlcd1, "to enter pass");
//					}
//				}
				/*-------------------------------------------*/
				password.index = 0;
				lcd_clear_display(&hlcd1);
				lcd_printf(&hlcd1, "Enter password:");
				lcd_set_cursor(&hlcd1, 1, 0);
				lcd_set_cursor_on(&hlcd1);
				lcd_set_cursor_blink_on(&hlcd1);
				keypad_state = ENTER_PASS_STATE;
			}
			break;
		}
		
		case ENTER_PASS_STATE:
			break;
		
		case SET_PASS_STATE:
			break;
		
		case NORMAL_STATE:
		{
			remote_set_state();
			Uart_Handle();
			if (HAL_GetTick() - hcsr04_time >= 100)
			{
				HCSR04_Start(&hc_sr04);
				hcsr04_time = HAL_GetTick();
			}
			switch(robot_state)
			{
				case STOP_STATE:
					break;
				case REMOTE_STATE:
				{
//					Uart_Handle();
					car_remote_handle();
					break;
				}
				case AUTO_STATE:
				{
					car_auto_state_switch(&servo_hcsr04 ,hc_sr04.hcsr04_distan);
					car_auto_state_handle(&servo_hcsr04, hc_sr04.hcsr04_distan);
					break;
				}
				default:
					break;
			}
			
			break;
		}
		
		case LOCK_30S_STATE:
		{
			uint16_t milis_lock;
			milis_lock = HAL_GetTick() - lock_start_time;
			if((HAL_GetTick() - lock_start_time)%1000 == 0)
			{
				lcd_clear_display(&hlcd1);
				lcd_printf(&hlcd1, "Lock %ds",(30 - milis_lock/1000));
			}
			if((HAL_GetTick() - lock_start_time) >= 30000)
			{
				keypad_state = LOCK_STATE;
				count_error = 3;
				keypad_flag = 1;
			}
			break;
		}
	}
}






/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  
  /*---------------------------- Init equipments functions -----------------------*/
  lcd_init(&hlcd1, &hi2c2, LCD_ADDR_DEFAULT);
  HCSR04_Init(&hc_sr04, &htim4, HCSR_TRIG_GPIO_Port, HCSR_TRIG_Pin, HCSR_ECHO_GPIO_Port, HCSR_ECHO_Pin);
  
  Car_Init(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15);
	Arm_Robot_Init(&arm_robot, &htim3); 
	
	Servo_Init(&servo_hcsr04, &htim4, TIM_CHANNEL_4);
	Servo_Write(&servo_hcsr04, 90);
	
//	set_default_cmd();
//	flash_unlock();
//	flash_erease(0x0801F000);
//	flash_erease(0x0801F400);
//	flash_erease(0x0801F800);
//	flash_erease(0x0801FC00);
//	flash_lock();

	Flash_Read_Position_Load();
	Flash_Read_Position_Unload();
	
	

	
	/*--------------- BLINK LED TO CHECK MICROCONTROLER IS NOT BLOCK ---------------*/
	static uint32_t time_blink = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Keypad_Handle();
		Keypad_State_Handle();
		HCSR04_Handle(&hc_sr04);
		
		if (HAL_GetTick() - time_blink > 500)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			time_blink = HAL_GetTick();
		}
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HCSR_TRIG_Pin|IN2_MOTOR_Pin|IN4_MOTOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW1_Pin ROW2_Pin ROW3_Pin ROW4_Pin */
  GPIO_InitStruct.Pin = ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : COL1_Pin COL2_Pin COL3_Pin COL4_Pin */
  GPIO_InitStruct.Pin = COL1_Pin|COL2_Pin|COL3_Pin|COL4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HCSR_TRIG_Pin IN2_MOTOR_Pin IN4_MOTOR_Pin */
  GPIO_InitStruct.Pin = HCSR_TRIG_Pin|IN2_MOTOR_Pin|IN4_MOTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HCSR_ECHO_Pin */
  GPIO_InitStruct.Pin = HCSR_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HCSR_ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IR1_Pin IR2_Pin IR3_Pin IR4_Pin */
  GPIO_InitStruct.Pin = IR1_Pin|IR2_Pin|IR3_Pin|IR4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IR5_Pin */
  GPIO_InitStruct.Pin = IR5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR5_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
