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
#include <math.h>
#include "stm32g4xx_hal.h"
#include "uart.h"
#include "system.h"

// nombre de led sur le connecteur (la led de status déjà compté)
#define NEOPIXEL_NB 10

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} colorRGB_t;

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t alpha;
} colorRGBA_t;
typedef struct
{
    float H;
    float S;
    float V;
} colorHSV_t;

colorHSV_t RGB_to_HSV(colorRGB_t rgb);
colorRGBA_t HSV_to_RGBA(colorHSV_t hsv);
colorRGB_t HSV_to_RGB(colorHSV_t hsv);
colorRGB_t RGBA_to_RGB(colorRGBA_t rgba);

void neopixelSetup(void);

void neopixelSetLed(uint16_t numLed, colorRGB_t color);
void neopixelSetLed(uint16_t numLed, colorRGBA_t color);
void neopixelSetLed(uint16_t numLed, colorHSV_t color);
void neopixelSetMoreLeds(uint16_t firstled, colorRGB_t color[], size_t size);
void neopixelSetMoreLeds(uint16_t firstled, colorRGBA_t color[], size_t size);
void neopixelSetMoreLeds(uint16_t firstled, colorHSV_t color[], size_t size);
void neopixelClear(void);

#endif