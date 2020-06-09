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

//from the efm32gg specsheet
#define UNIQUEH_ADDR		(0x0FE081F0UL)
#define UNIQUEH			(*((uint32_t*)UNIQUEH_ADDR))
#define UNIQUEL_ADDR		(0x0FE081F4UL)
#define UNIQUEL			(*((uint32_t*)UNIQUEL_ADDR))

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

static packet_t packet;

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
    RadioModems_t radio;
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
    radio = MODEM_LORA;
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
    
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )
    radio = MODEM_FSK;
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


    packet.magic_code = MAGIC_CODE;
    packet.counter = 0xFFFF;
    packet.eui[0] = (uint8_t)((UNIQUEH >> 24) & 0xFF);
    packet.eui[1] = (uint8_t)((UNIQUEH >> 16) & 0xFF);
    packet.eui[2] = (uint8_t)((UNIQUEH >> 8) & 0xFF);
    packet.eui[3] = (uint8_t)((UNIQUEH) & 0xFF);
    packet.eui[4] = (uint8_t)((UNIQUEL >> 24) & 0xFF);
    packet.eui[5] = (uint8_t)((UNIQUEL >> 16) & 0xFF);
    packet.eui[6] = (uint8_t)((UNIQUEL >> 8) & 0xFF);
    packet.eui[7] = (uint8_t)((UNIQUEL) & 0xFF);

    Radio.Sleep();

    while (1)
    {
	packet.counter++;
	packet.crc = packet_calc_crc(&packet);
	Radio.Send( (uint8_t*)&packet, sizeof(packet_t) );
	printf("Sent Packet using %s. size=%d, counter=%d TimeOnAir(us)=%d\r\n", 
		    RadioTypeToStr(radio), 
		    sizeof(packet), 
		    packet.counter, 
		    Radio.TimeOnAir(radio,sizeof(packet_t)));
	DelayMs(1000);
    }
}

void OnTxDone( void )
{
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
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
