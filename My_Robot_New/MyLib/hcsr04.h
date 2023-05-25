/*	CONFIG TIMER FOR HCSR04
		PSC = F_sys/1000000 - 1
		ARR = MAX_ARR_VALUE
*/

#ifndef _HCSR04_H_
#define _HCSR04_H_
#include "stm32f1xx_hal.h"
typedef enum
{
	HCSR04_IDLE_STATE,
	HCSR04_WAIT_RISING_STATE,
	HCSR04_WAIT_FALLING_STATE,
	HCSR04_COMPLETE_STATE
}HCSR04_State;

typedef struct
{
	TIM_HandleTypeDef *hcsr04_tim;
	GPIO_TypeDef			*hcsr04_trig_port;
	uint16_t					hcsr04_trig_pin;
	GPIO_TypeDef			*hcsr04_echo_port;
	uint16_t					hcsr04_echo_pin;
	HCSR04_State			hcsr04_state;
	float							hcsr04_distan;
} HCSR04_TypeDef;

void HCSR04_Init(HCSR04_TypeDef *hcsr04, TIM_HandleTypeDef *htim, GPIO_TypeDef *trig_port,
									uint16_t trig_pin, GPIO_TypeDef *echo_port, uint16_t echo_pin);
void HCSR04_Start(HCSR04_TypeDef *hcsr04);
void HCSR04_Handle(HCSR04_TypeDef *hcsr04);
void EXTI_HCSR04_Callback(HCSR04_TypeDef *hcsr04);
#endif
