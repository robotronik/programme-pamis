#ifndef UARTLIDAR_H
#define UARTLIDAR_H

/*****************
 *  Rixae Dufour - CDFR 2025 PAMIS
 *  ROBOTRONIK - mars 2025
 *
 *  gestion de l'uart2, uniquement le Tx
 *  sur PB3 principalement pour le debug
 *
 *******************/
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include "system.h"
#define UART_TIMEOUT 100

void uartLidarSetup(void);

bool usart1recev(uint8_t* data);

void usartSend1Data(const uint8_t *data, int size);

void usart1flushSerial(void);

bool usart1Error();

uint32_t getFifoSize();


#endif