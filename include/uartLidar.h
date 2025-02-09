#pragma once


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include "config.h"

void usart1Setup(void);


bool usart1recev(uint16_t* data);

//Send information on uart2
//1000 char max
void usart1printf(const char* format,...);

