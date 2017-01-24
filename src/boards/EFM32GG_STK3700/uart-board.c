/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board UART driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "uart-board.h"
#include "em_cmu.h"

uint8_t RxData = 0;

void UartMcuInit( Uart_t *obj, uint8_t uartId, PinNames tx, PinNames rx )
{
    obj->UartId = uartId;

    GpioInit(&obj->Tx, tx, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&obj->Rx, rx, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
}

void UartMcuConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
	CMU_ClockEnable(cmuClock_UART0, true);

	int dataBitsValue;
	if (wordLength == UART_9_BIT)
		dataBitsValue = usartDatabits9;
	else
		dataBitsValue = usartDatabits8;

	int stopBitsValue;
	if (stopBits == UART_2_STOP_BIT)
		stopBitsValue = usartStopbits2;
	else if (stopBits == UART_1_5_STOP_BIT)
		stopBitsValue = usartStopbits1p5;
	else if (stopBits == UART_0_5_STOP_BIT)
		stopBitsValue = usartStopbits0p5;
	else
		stopBitsValue = usartStopbits1;

	int parityValue;
	if (parity == EVEN_PARITY)
		parityValue = usartEvenParity;
	else if (parity == ODD_PARITY)
		parityValue = usartOddParity;
	else
		parityValue = usartNoParity;

	int enableValue;
	if (mode == TX_ONLY)
		enableValue = usartEnableTx;
	else if (mode == RX_ONLY)
		enableValue = usartEnableRx;
	else
		enableValue = usartEnable;

	USART_InitAsync_TypeDef uartInit = {
	    .enable       = usartDisable,   // wait to enable the transceiver
	    .refFreq      = 0,              // setting refFreq to 0 will invoke the CMU_ClockFreqGet() function and measure the HFPER clock
	    .baudrate     = baudrate,       // desired baud rate
	    .oversampling = usartOVS16,     // set oversampling value to x16
	    .databits     = dataBitsValue,
	    .parity       = parityValue,
	    .stopbits     = stopBitsValue,
	    .mvdis        = false,          // use majority voting
	    .prsRxEnable  = false,          // not using PRS input
	    .prsRxCh      = usartPrsRxCh0,  // doesn't matter which channel we select
	};

	USART_InitAsync(UART0, &uartInit);

	// clear RX/TX buffers and shift regs, enable transmitter and receiver pins
	UART0->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_TXPEN | UART_ROUTE_LOCATION_LOC1;

	USART_IntClear(UART0, _UART_IF_MASK); // Clear any USART interrupt flags
	NVIC_ClearPendingIRQ(UART0_RX_IRQn);   // Clear pending RX interrupt flag in NVIC
	NVIC_ClearPendingIRQ(UART0_TX_IRQn);   // Clear pending TX interrupt flag in NVIC

	USART_Enable(UART0, enableValue);
}

void UartMcuDeInit( Uart_t *obj )
{
	UART0->ROUTE = _UART_ROUTE_RESETVALUE;

	USART_Enable(UART0, usartDisable);
	CMU_ClockEnable(cmuClock_UART0, false);
}

uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data )
{
	if (UART0->STATUS & UART_STATUS_TXBL)
	{
		UART0->TXDATA = (uint32_t)data;
		return 0; // OK
	}
	else
		return 1; // Busy
}

uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data )
{
	if (UART0->STATUS & UART_STATUS_RXDATAV)
	{
		*data = (uint8_t)UART0->RXDATA;
		return 0;
	}
	else
		return 1;
}
