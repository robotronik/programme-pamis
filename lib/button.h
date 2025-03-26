#ifndef BUTTON_H
#define BUTTON_H

/*****************
 *  Rixae Dufour - CDFR 2025 PAMIS
 *  ROBOTRONIK - mars 2025
 *
 *  gestion de la led et des boutons sur la pcb pour debug
 *
 *******************/

#include "stm32g4xx_hal.h"

#define BTTEAM_pin GPIO_PIN_12
#define BTTEAM_Port GPIOA
#define BTTIRETTE_Pin GPIO_PIN_11
#define BTTIRETTE_Port GPIOA

void ButtonSetup(void);

GPIO_PinState ButtonTeamGetValue(void);
GPIO_PinState ButtonTiretteGetValue(void);

#endif