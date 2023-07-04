#ifndef CONFIG_H
#define CONFIG_H

#include <cstdio>
#include <cstdint>
#include <cmath>

#include "pico/stdlib.h"
//#include "hardware/pio.h"
#include "hardware/irq.h"
//#include "hardware/adc.h"
#include "hardware/uart.h"
#include "pico/bootrom.h"


#define FW_TYPE "master"
#define FW_VERSION "FwVer1_0"
#define AUTHOR      "SNJY"

// #define _DEBUG
#ifdef _DEBUG
#define printf_debug  printf
#else
#define printf_debug(s, ...) ((void)0)
#endif


#define SECRET_CODE 64209

#define HIGH        1
#define LOW         0

#define CW          HIGH
#define CCW         LOW

#define TICK_RATE   500

#endif