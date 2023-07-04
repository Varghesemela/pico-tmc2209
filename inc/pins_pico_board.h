#ifndef PINS_PICO_BOARD_H
#define PINS_PICO_BOARD_H


#define BOARD_TYPE "RPi-Pico"

/*
* GPIO Pins declaration for strobing
*/


#define SPI_PORT    spi1
#define SCK         10
#define MOSI        11
#define MISO        12
#define CS          13


#define LED_STATUS  25
#define MONITOR_5V 29

#define TMC_UART        uart1
#define TMC_UART_TX     4
#define TMC_UART_RX     5
#define TMC_VREF        16
#define TMC_INDEX       17
#define TMC_STEP        18
#define TMC_DIRECTION   19
#define TMC_ENABLE      20
#define TMC_SPREADCYCLE 21

#endif