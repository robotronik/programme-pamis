#ifndef NEOPIXEL_H
#define NEOPIXEL_H

/*****************
 *  Rixae Dufour - CDFR 2025 PAMIS
 *  ROBOTRONIK - mars 2025
 *
 *  controle  la led de status ainsi
 *  que le connecteur (pour les bâton lumineux)
 *
 *******************/
#include <stdint.h>
#include <stdlib.h>
#include "system.h"

// nombre de led sur le connecteur (la led de status déjà compté)
#define NEOPIXEL_NB         2

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} color_t;

void neopixelSetup(void);

void neopixelSetLed(uint16_t numLed, color_t color, uint8_t brightness);
void neopixelSetMoreLeds(uint16_t firstled, color_t color[], size_t size);
void neopixelClear(void);

#endif