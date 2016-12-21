/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "em_device.h"

typedef struct
{
  ADC_TypeDef* Instance;
}ADC_HandleTypeDef;

typedef struct
{
  I2C_TypeDef* Instance;
}I2C_HandleTypeDef;

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "utilities.h"
#include "timer.h"
#include "delay.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "radio.h"
#include "sx1276/sx1276.h"
#include "gps.h"
#include "gps-board.h"
#include "rtc-board.h"
#include "sx1276-board.h"
#include "uart-board.h"

#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))

#if defined( USE_USB_CDC )
#include "uart-usb-board.h"
#endif

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0
#endif


/*!
 * Board MCU pins definitions
 */

#define RADIO_MOSI                                  PD_0
#define RADIO_MISO                                  PD_1
#define RADIO_SCLK                                  PD_2
#define RADIO_NSS                                   PD_3
#define RADIO_RESET                                 PD_4

#define RADIO_DIO_0                                 PC_5
#define RADIO_DIO_1                                 PB_12
#define RADIO_DIO_2                                 PD_7 // TODO: or PC_6?
#define RADIO_DIO_3                                 PC_3
#define RADIO_DIO_4                                 PC_4
#define RADIO_DIO_5                                 PD_5

#define LED_0                                       PE_2
#define LED_1                                       PE_3

#define I2C_SCL                                     NC
#define I2C_SDA                                     NC


/*!
 * LED GPIO pins objects
 */
/*
extern Gpio_t IrqMpl3115;
extern Gpio_t IrqMag3110;
extern Gpio_t GpsPowerEn;
extern Gpio_t NcIoe3;
extern Gpio_t NcIoe4;
extern Gpio_t NcIoe5;
extern Gpio_t NcIoe6;
extern Gpio_t NcIoe7;
extern Gpio_t NIrqSX9500;
extern Gpio_t Irq1Mma8451;
extern Gpio_t Irq2Mma8451;
extern Gpio_t TxEnSX9500;
*/
extern Gpio_t Led0;
extern Gpio_t Led1;

/*!
 * Debug GPIO pins objects
 */
/*
#if defined( USE_DEBUG_PINS )
extern Gpio_t DbgPin1;
extern Gpio_t DbgPin2;
extern Gpio_t DbgPin3;
extern Gpio_t DbgPin4;
extern Gpio_t DbgPin5;
#endif
*/

/*!
 * MCU objects
 */
extern Adc_t Adc;
extern I2c_t I2c;
extern Uart_t Uart1;
#if defined( USE_USB_CDC )
extern Uart_t UartUsb;
#endif

/*
extern Gpio_t GpsPps;
extern Gpio_t GpsRx;
extern Gpio_t GpsTx;
*/
enum BoardPowerSource
{
    USB_POWER = 0,
    BATTERY_POWER
};

/*!
 * \brief Disbale interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Enable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardEnableIrq( void );

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level ( 0: very low, 254: fully charged )
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Get the board power source
 *
 * \retval value  power source ( 0: USB_POWER,  1: BATTERY_POWER )
 */
uint8_t GetBoardPowerSource( void );

#endif // __BOARD_H__
