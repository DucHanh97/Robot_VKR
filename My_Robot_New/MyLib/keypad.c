#include "keypad.h"

/*---------------------------------KEYPAD-------------------------------------*/

const uint8_t key_code[KEYPAD_ROW][KEYPAD_COL] = 
{
	{'1', '2', '3', 'A'},
	{'4', '5', '6', 'B'},
	{'7', '8', '9', 'C'},
	{'*', '0', '#', 'D'}
};

static GPIO_TypeDef *Keypad_RowPort[KEYPAD_ROW] = {GPIOA, GPIOA, GPIOA, GPIOA};
static uint16_t Keypad_RowPin[KEYPAD_ROW] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};
static GPIO_TypeDef *Keypad_ColPort[KEYPAD_COL] = {GPIOA, GPIOA, GPIOA, GPIOA};
static uint16_t Keypad_ColPin[KEYPAD_COL] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};

/*---------------------Xu ly nhan, nha, nhan giu phim------------------------*/
__weak void KeypadPressingCallback(uint8_t key)
{
	
}

__weak void KeypadPressedShortCallback(uint8_t key)
{
	
}

__weak void KeypadReleaseCallback(uint8_t key)
{
	
}

__weak void KeypadPressingTimeoutCallback(uint8_t key)
{
	
}

static void Keypad_Select_Row(uint8_t row)
{
	HAL_GPIO_WritePin(Keypad_RowPort[row], Keypad_RowPin[row], GPIO_PIN_RESET);
}

static void Keypad_Unselect_Row()
{
	for(uint8_t row = 0; row < KEYPAD_ROW; row++)
	{
		HAL_GPIO_WritePin(Keypad_RowPort[row], Keypad_RowPin[row], GPIO_PIN_SET);
	}
}

uint8_t Keypad_GetKey()
{
	for(uint8_t row = 0; row < KEYPAD_ROW; row++)
	{
		Keypad_Unselect_Row();
		Keypad_Select_Row(row);
		for(uint8_t col = 0; col < KEYPAD_COL; col++)
		{
			if(HAL_GPIO_ReadPin(Keypad_ColPort[col], Keypad_ColPin[col]) == 0)
			{
				return key_code[row][col];
			}
		}
	}
	return 0;
}

static uint8_t key_current;
static uint8_t key_last;
static uint8_t key_debounce;
static uint8_t debouncing = 0;
static uint32_t t_debounce;
static uint32_t t_start_press;
static uint32_t is_press;

static void Keypad_Filter()
{
	uint8_t key = Keypad_GetKey();
	/* Khi van dang nhieu */
	if(key != key_debounce)
	{
		debouncing = 1;
		t_debounce = HAL_GetTick();
		key_debounce = key;
	}
	/* Khi trang thai nut nhan da duoc xac lap */
	if(debouncing && (HAL_GetTick() - t_debounce >= 15))
	{
		key_current = key_debounce;
		debouncing = 0;
	}
}

void Keypad_Handle(void)
{
	Keypad_Filter();
	if(key_current != key_last)
	{
		if(key_current != 0)		//Button pressed
		{
			is_press = 1;
			t_start_press = HAL_GetTick();
			KeypadPressingCallback(key_current);
		}
		else										//Button release
		{
			if(HAL_GetTick() - t_start_press <= 1000)						// short-press button
			{
				KeypadPressedShortCallback(key_last);
			}
			KeypadReleaseCallback(key_last);
		}
		key_last = key_current;
	}
	
	if(is_press && ((HAL_GetTick() - t_start_press) >= 3000))	// long-press button
	{
		KeypadPressingTimeoutCallback(key_current);
		is_press = 0;
	}
}

void Keypad_Init(void)
{
	
}
