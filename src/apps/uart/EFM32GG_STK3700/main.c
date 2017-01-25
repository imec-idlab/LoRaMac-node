#include "board.h"

Uart_t uart; // some members have to be initialized, do not put the declaration inside a function without setting some values

// This function will make printf work
void _write(int fd, const void *buf, size_t count)
{
	UartPutBuffer(&uart, (uint8_t*)buf, count);
}

int main()
{
	BoardInitMcu();

	UartInit(&uart, UART_1, PE_0, PE_1);
	UartConfig(&uart, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL);

	printf("Hello world\r\n");

	uint8_t c;
	while (true)
	{
		if (UartGetChar(&uart, &c) == 0)
			UartPutChar(&uart, c);
	}
}
