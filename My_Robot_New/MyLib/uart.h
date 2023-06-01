#ifndef UART_H
#define UART_H
#include <stdint.h>

/*   ROBOT STATE TYPEDEF   */
typedef enum
{
	STOP_STATE,
	REMOTE_STATE,
	SET_AUTO_STATE,
	AUTO_STATE
}RobotState;

void Uart_Receive_Data(uint8_t rx_data);
void Uart_Handle(void);
#endif
