#pragma once


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include "config.h"

void usart1Setup(void);


bool usart1recev(uint8_t* data);

void usartSend1Data(const uint8_t *data, int size);

void usart1flushSerial(void);

bool usart1Error();

//Send information on uart2
//1000 char max
void usart1printf(const char* format,...);

