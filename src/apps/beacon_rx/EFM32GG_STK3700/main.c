/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Ping-Pong implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include <string.h>
#include "board.h"
#include "radio.h"
#include "em_emu.h"

#define MAGIC_CODE		0xCAFEBABEUL

#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm

#if defined( USE_MODEM_LORA )
    #define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
    #define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
    #define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
    #define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
    #define LORA_SYMBOL_TIMEOUT                         5         // Symbols
    #define LORA_FIX_LENGTH_PAYLOAD_ON                  false
    #define LORA_IQ_INVERSION_ON                        false
#elif defined( USE_MODEM_FSK )
    #define FSK_FDEV                                    25e3      // Hz
    #define FSK_DATARATE                                50e3      // bps
    #define FSK_BANDWIDTH                               50e3      // Hz
    #define FSK_AFC_BANDWIDTH                           83.333e3  // Hz
    #define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
    #define FSK_FIX_LENGTH_PAYLOAD_ON                   false
#else
    #error "Please define a modem in the compiler options."
#endif


// This function will make printf use the UART
void _write(int fd, const void *buf, size_t count)
{
	UartPutBuffer(&Uart1, (uint8_t*)buf, count);
}

static void UartIrqNotify(UartNotifyId_t id)
{}

typedef struct
{
    uint32_t	magic_code;
    uint8_t	eui[8];
    uint16_t	counter;
    uint16_t	crc;
} __attribute__((packed)) packet_t;

typedef struct
{
    packet_t pkt;
    int16_t rssi;
    int8_t snr;
} packet_entry_t;

#define QUEUE_SIZE	16
packet_entry_t packet_queue[QUEUE_SIZE];
volatile int queue_head = 0;
volatile int queue_tail = 0;





uint16_t crcByte(uint16_t crc, uint8_t b) {
  crc = (uint8_t)(crc >> 8) | (crc << 8);
  crc ^= b;
  crc ^= (uint8_t)(crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}


char const* RadioTypeToStr(RadioModems_t modem)
{
    if(modem == MODEM_LORA)
	return "LORA";
    else if (modem == MODEM_FSK)
	return "FSK";
    else
	return "UNKNOWN";
}

uint16_t packet_calc_crc(packet_t* pkt)
{
    uint8_t* pkt_as_array = (uint8_t*)(pkt);
    int i = 0;
    uint16_t crc = 0;
    for(i = 0; i < sizeof(packet_t)-sizeof(uint16_t); i++)
	crc = crcByte(crc,pkt_as_array[i]);
    return crc;
}

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnTxTimeout( void );
void OnRxTimeout( void );
void OnRxError( void );

int main( void )
{
    // Target board initialisation
    BoardInitMcu( );
    BoardInitPeriph( );

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
    
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )
    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );
    
    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band in the compiler options."
#endif

    Radio.Rx(0);
    printf("node_id,counter,rssi,snr\r\n");
    while (1)
    {
	uint16_t cur_crc = 0;
	//wait until we have something to do
	while(queue_tail == queue_head);

	cur_crc = packet_calc_crc(&(packet_queue[queue_tail].pkt));
	//if the crc is valid -> print the packet
	if(cur_crc == packet_queue[queue_tail].pkt.crc && packet_queue[queue_tail].pkt.magic_code == MAGIC_CODE)
	{
	    int i = 0;
	    //format == node_id (hex, all_caps),counter,rssi,snr
	    printf("0x");
	    for (i = 0; i < 8; i++)
		printf("%02X",packet_queue[queue_tail].pkt.eui[i]);
	    printf(",%d,%d,%d\r\n",packet_queue[queue_tail].pkt.counter,
				   packet_queue[queue_tail].rssi,
				   packet_queue[queue_tail].snr);
	}
	queue_tail = (queue_tail +1)% QUEUE_SIZE;
    }
}

void OnTxDone( void )
{
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    //if we got a packet of the correct length && there is space in the queue
    if((size == sizeof(packet_t)) && (((queue_head + 1)%QUEUE_SIZE) != queue_tail))
    {
	memcpy(&(packet_queue[queue_head].pkt), payload, sizeof(packet_t));
	packet_queue[queue_head].rssi = rssi;
	packet_queue[queue_head].snr = snr;
	queue_head = (queue_head + 1)%QUEUE_SIZE;
    }
}

void OnTxTimeout( void )
{
}

void OnRxTimeout( void )
{
}

void OnRxError( void )
{
}
