#ifndef UART_H
#define UART_H

/*****************
 *  Rixae Dufour - CDFR 2025 PAMIS
 *  ROBOTRONIK - mars 2025
 *
 *  gestion de l'uart2, uniquement le Tx
 *  sur PB3 principalement pour le debug
 *
 *******************/
#include "stm32g4xx_hal.h"
#define UART_TIMEOUT 100
#define UART_MAX_MSG 256

void uartSetup();
void uartTransmit(char *msg, size_t size);

// /!\ ne pas dépasser un message de la taille UART_MAX_MSG (256 by default)
void uartprintf(const char *format, ...);

#endif