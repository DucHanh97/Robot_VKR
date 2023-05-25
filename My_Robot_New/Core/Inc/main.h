/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define ROW1_Pin GPIO_PIN_0
#define ROW1_GPIO_Port GPIOA
#define ROW2_Pin GPIO_PIN_1
#define ROW2_GPIO_Port GPIOA
#define ROW3_Pin GPIO_PIN_2
#define ROW3_GPIO_Port GPIOA
#define ROW4_Pin GPIO_PIN_3
#define ROW4_GPIO_Port GPIOA
#define COL1_Pin GPIO_PIN_4
#define COL1_GPIO_Port GPIOA
#define COL2_Pin GPIO_PIN_5
#define COL2_GPIO_Port GPIOA
#define COL3_Pin GPIO_PIN_6
#define COL3_GPIO_Port GPIOA
#define COL4_Pin GPIO_PIN_7
#define COL4_GPIO_Port GPIOA
#define SERVO3_Pin GPIO_PIN_0
#define SERVO3_GPIO_Port GPIOB
#define SERVO4_Pin GPIO_PIN_1
#define SERVO4_GPIO_Port GPIOB
#define SCL_LCD_Pin GPIO_PIN_10
#define SCL_LCD_GPIO_Port GPIOB
#define SDA_LCD_Pin GPIO_PIN_11
#define SDA_LCD_GPIO_Port GPIOB
#define HCSR_TRIG_Pin GPIO_PIN_12
#define HCSR_TRIG_GPIO_Port GPIOB
#define HCSR_ECHO_Pin GPIO_PIN_13
#define HCSR_ECHO_GPIO_Port GPIOB
#define HCSR_ECHO_EXTI_IRQn EXTI15_10_IRQn
#define IN2_MOTOR_Pin GPIO_PIN_14
#define IN2_MOTOR_GPIO_Port GPIOB
#define IN4_MOTOR_Pin GPIO_PIN_15
#define IN4_MOTOR_GPIO_Port GPIOB
#define IN1_MOTOR_Pin GPIO_PIN_8
#define IN1_MOTOR_GPIO_Port GPIOA
#define IN3_MOTOR_Pin GPIO_PIN_9
#define IN3_MOTOR_GPIO_Port GPIOA
#define IR1_Pin GPIO_PIN_10
#define IR1_GPIO_Port GPIOA
#define IR2_Pin GPIO_PIN_11
#define IR2_GPIO_Port GPIOA
#define IR3_Pin GPIO_PIN_12
#define IR3_GPIO_Port GPIOA
#define IR4_Pin GPIO_PIN_15
#define IR4_GPIO_Port GPIOA
#define IR5_Pin GPIO_PIN_3
#define IR5_GPIO_Port GPIOB
#define SERVO1_Pin GPIO_PIN_4
#define SERVO1_GPIO_Port GPIOB
#define SERVO2_Pin GPIO_PIN_5
#define SERVO2_GPIO_Port GPIOB
#define TX_HC05_Pin GPIO_PIN_6
#define TX_HC05_GPIO_Port GPIOB
#define RX_HC05_Pin GPIO_PIN_7
#define RX_HC05_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
