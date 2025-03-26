

#include "timerIRQ.h"

void (*fallback_ptr[TIMER_IRQ_NB])(void);

void defaultIrq(void)
{
}

int timerHandleMap(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM17)
    {
        return 17;
    }
}

void timerirqdefinefallback(TIM_HandleTypeDef *htim, void *fallback(void))
{
    // fallback_ptr[timerHandleMap(htim)] = fallback;
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        motorStepper1Fallback();
    }
    if (htim->Instance == TIM7)
    {
        motorStepper2Fallback();
    }
    if (htim->Instance == TIM17)
    {
        timerMsFallback();
    }
}