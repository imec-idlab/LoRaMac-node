#include "board.h"

// This function will make printf use the UART
void _write(int fd, const void *buf, size_t count)
{
	UartPutBuffer(&Uart1, (uint8_t*)buf, count);
}

int main()
{
	BoardInitMcu();

	printf("Hello world\r\n");

	uint8_t c;
	while (true)
	{
		if (UartGetChar(&Uart1, &c) == 0)
			UartPutChar(&Uart1, c);
	}
}
