#include "hcsr04.h"

void HCSR04_Init(HCSR04_TypeDef *hcsr04, TIM_HandleTypeDef *htim, GPIO_TypeDef *trig_port,
									uint16_t trig_pin, GPIO_TypeDef *echo_port, uint16_t echo_pin)
{
	hcsr04->hcsr04_tim = htim;
	hcsr04->hcsr04_trig_port = trig_port;
	hcsr04->hcsr04_trig_pin = trig_pin;
	hcsr04->hcsr04_echo_port = echo_port;
	hcsr04->hcsr04_echo_pin = echo_pin;
	hcsr04->hcsr04_state = HCSR04_IDLE_STATE;
	hcsr04->hcsr04_distan = 0;
}

void delay_us(HCSR04_TypeDef *hcsr04, uint32_t us)
{
	hcsr04->hcsr04_tim->Instance->CNT = 0;
	HAL_TIM_Base_Start(hcsr04->hcsr04_tim);
	while(hcsr04->hcsr04_tim->Instance->CNT < us);
	HAL_TIM_Base_Stop(hcsr04->hcsr04_tim);
}
void HCSR04_Start(HCSR04_TypeDef *hcsr04)
{
	if(hcsr04->hcsr04_state == HCSR04_IDLE_STATE)
	{
		HAL_GPIO_WritePin(hcsr04->hcsr04_trig_port, hcsr04->hcsr04_trig_pin, GPIO_PIN_SET);
		delay_us(hcsr04, 12);
		HAL_GPIO_WritePin(hcsr04->hcsr04_trig_port, hcsr04->hcsr04_trig_pin, GPIO_PIN_RESET);
		hcsr04->hcsr04_state = HCSR04_WAIT_RISING_STATE;
	}
}
__weak void HCSR04_Complete_Callback(HCSR04_TypeDef *hcsr04)
{
	/* do something  */
}
void HCSR04_Handle(HCSR04_TypeDef *hcsr04)
{
	if(hcsr04->hcsr04_state == HCSR04_COMPLETE_STATE)
	{
		//Tinh khoang cach
		hcsr04->hcsr04_distan = (float)(0.017*(hcsr04->hcsr04_tim->Instance->CNT));
		HCSR04_Complete_Callback(hcsr04);
		hcsr04->hcsr04_state = HCSR04_IDLE_STATE;
	}
}
void EXTI_HCSR04_Callback(HCSR04_TypeDef *hcsr04)
{
	switch(hcsr04->hcsr04_state)
	{
		case HCSR04_WAIT_RISING_STATE:
			if(HAL_GPIO_ReadPin(hcsr04->hcsr04_echo_port, hcsr04->hcsr04_echo_pin) == 1)
			{
				hcsr04->hcsr04_tim->Instance->CNT = 0;
				hcsr04->hcsr04_state = HCSR04_WAIT_FALLING_STATE;
				HAL_TIM_Base_Start(hcsr04->hcsr04_tim);
			}
			else
			{
				hcsr04->hcsr04_state = HCSR04_IDLE_STATE;
			}
			break;
		case HCSR04_WAIT_FALLING_STATE:
			if(HAL_GPIO_ReadPin(hcsr04->hcsr04_echo_port, hcsr04->hcsr04_echo_pin) == 0)
			{
				HAL_TIM_Base_Stop(hcsr04->hcsr04_tim);
				hcsr04->hcsr04_state = HCSR04_COMPLETE_STATE;
			}
			else
			{
				hcsr04->hcsr04_state = HCSR04_IDLE_STATE;
			}
			break;
		case HCSR04_COMPLETE_STATE:
			break;
		case HCSR04_IDLE_STATE:
			break;
		default:
			break;
	}
}
