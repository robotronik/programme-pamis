#ifndef timer_irq_h
#define timer_irq_h

/*****************
 *  rixae dufour - cdfr 2025 pamis
 *  robotronik - mars 2025
 *
 *  irq pour les timers
 *
 *******************/
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "timer_ms.h" 
#include "motor.h" 


#define TIMER_IRQ_NB  18

void timerirqdefinefallback (TIM_HandleTypeDef * htim,void * fallback(void));

#endif