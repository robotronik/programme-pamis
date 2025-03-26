#ifndef LEVEL_BATTERY_H
#define LEVEL_BATTERY_H

/*****************
 *  Rixae Dufour - CDFR 2025 PAMIS
 *  ROBOTRONIK - mars 2025
 *
 * Récupération du niveau de charge de la batterie
 * avec une fonction linéaire de la tension
 *
 *******************/
#include <stdint.h>

#define BATT_TENSION_MAX 8.4
#define BATT_TENSION_MIN 7.2
#define BATT_PDT_RATIO 0.359

void battSetup(void);

uint32_t battGetRawValue(void);
float battGetPourcentage(void);
float battGetVoltage(void);

#endif