#ifndef LEDBUTTON_H
#define LEDBUTTON_H

/*****************
 *  Rixae Dufour - CDFR 2025 PAMIS
 *  ROBOTRONIK - mars 2025
 *
 *  gestion de la led et des boutons sur la pcb pour debug
 *
 *******************/

#include "stm32g4xx_hal.h"

#define LEDPCB_Pin GPIO_PIN_7 
#define LEDPCB_Port GPIOA
#define BUTTONPCB_Pin GPIO_PIN_10
#define BUTTONPCB_Port GPIOA 

void LedPcbSetup(void);

void LedPcbToggle(void);
void LedPcbOn(void);
void LedPcbOff(void);

void ButtonPcbSetup(void);
GPIO_PinState ButtonPcbGetValue(void);
//todo bouton en interruption
void ButtonPcbSetupInterrupt(void *isr(void));
void ButtonPcbEnableInterrupt(void);
void ButtonPcbDisableInterrupt(void);

#endif