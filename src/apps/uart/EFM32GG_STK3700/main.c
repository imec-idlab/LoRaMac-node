#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_system.h"
#include "em_chip.h"
#include "board.h"

Uart_t uart; // some values have to be initialized, do not put declaration inside a function without settings some values

int main()
{
	BoardInitMcu();

	UartInit(&uart, UART_1, PE_0, PE_1);
	UartConfig(&uart, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL);

	uint8_t c;
	while (true)
	{
		if (UartGetChar(&uart, &c) == 0)
			UartPutChar(&uart, c);
	}
}
