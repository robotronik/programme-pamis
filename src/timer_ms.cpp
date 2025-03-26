#include "timer_ms.h"
#include "system.h"

TIM_HandleTypeDef htim17;
volatile uint32_t millis_counter = 0; // Compteur global des millisecondes

// Fonction d'initialisation de TIM17
void timerMsSetup(void)
{
    __HAL_RCC_TIM17_CLK_ENABLE(); // Activer l'horloge de TIM17

    htim17.Instance = TIM17;
    htim17.Init.Prescaler = 839; // Prescaler pour 1 ms
    htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim17.Init.Period = 100; // Auto-reload value pour 1 ms
    htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    // Initialiser TIM17 avec HAL
    if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
    {
        // Gestion d'erreur
        Error_Handler();
    }

    // Activer l'interruption de mise à jour
    HAL_TIM_Base_Start_IT(&htim17);

    // Configurer la priorité et activer l'IRQ dans le NVIC
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
    NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn,5); 
    //HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 2, 0);
    //HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
}

// Fonction qui retourne le temps écoulé en millisecondes
uint32_t millis(void)
{
    return millis_counter;
}

// Gestionnaire d'interruption pour TIM17
extern "C" void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim17);
}

// Callback appelé à chaque interruption du timer

void  timerMsFallback(void)
{
    millis_counter ++; 
}