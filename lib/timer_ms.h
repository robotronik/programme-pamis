#ifndef TIMER_MS_H
#define TIMER_MS_H

/*****************
 *  rixae dufour - cdfr 2025 pamis
 *  robotronik - mars 2025
 *
 *  timer en ms non bloquant
 *
 *******************/
#include <stdint.h>
#include "stm32g4xx_hal.h"

#define timer_ms TIM17 

uint32_t millis(void); 
void timerMsSetup(void); 
void  timerMsFallback(void); //do not use 


#endif