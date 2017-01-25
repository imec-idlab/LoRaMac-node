#include "board.h"
#include "em_emu.h"

// This function will make printf use the UART
void _write(int fd, const void *buf, size_t count)
{
	UartPutBuffer(&Uart1, (uint8_t*)buf, count);
}

void UartIrqNotify(UartNotifyId_t id)
{
	// When we receive a byte, send the same byte back
	if (id == UART_NOTIFY_RX)
	{
		uint8_t byte;
		UartGetChar(&Uart1, &byte);
		UartPutChar(&Uart1, byte);
	}
}

int main()
{
	BoardInitMcu();

	printf("Hello world\r\n");

	// Set a callback for the UART RX interrupt
	Uart1.IrqNotify = UartIrqNotify;

	while (true)
		EMU_EnterEM1();
}
