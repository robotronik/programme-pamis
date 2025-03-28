#include "motor.h"
#include <math.h>
#include "system.h"
#include "ledButtonPcb.h"
#include "uart.h"

#define MOTOR_1_READY (1 << 0)
#define MOTOR_2_READY (1 << 1)


typedef struct
{
    int nbpas;  // nb de pas total à faire
    int accel;  // nb de pas d'accel
    int deccel; // nbpas de decel
    int timer;  // timer min
} deplacement_t;


// VARIABLE
volatile int motorReady;
volatile uint32_t nbStep1;
volatile uint32_t nbStep2;
volatile uint32_t dir1;
volatile uint32_t dir2;

volatile deplacement_t stepper1;
volatile deplacement_t stepper2;
// TIMER
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

int motorGPIOSetup(void);
int motorTimerSetup(void);

void MotorStartIRQTimer(TIM_HandleTypeDef *htim, uint16_t time);

void motorStepper1Fallback (void){

    HAL_TIM_Base_Stop_IT(&htim1);
    // Code à exécuter après 25 ms
    LedPcbToggle();
    HAL_GPIO_TogglePin(MOTOR_Port, MOTOR_STEP1_Pin);

    stepper1.nbpas--;
    if (stepper1.nbpas != 0)
    {
      MotorStartIRQTimer(&htim1, stepper1.timer);
    }
    else
    {
      motorReady |= MOTOR_1_READY;
    }
}
void motorStepper2Fallback (void){

    HAL_TIM_Base_Stop_IT(&htim2);

    HAL_GPIO_TogglePin(MOTOR_Port, MOTOR_STEP2_Pin);

    stepper2.nbpas--;
    if (stepper2.nbpas != 0)
    {
      MotorStartIRQTimer(&htim2, stepper2.timer);
    }
    else
    {
      motorReady |= MOTOR_2_READY;
    }
} 



void motorSetup(void)
{
  if (motorGPIOSetup())
  {
    Error_Handler();
  }
  if (motorTimerSetup())
  {
    Error_Handler();
  }
  motorDisable();
  motorReady = MOTOR_1_READY | MOTOR_2_READY;
}

int motorTimerSetup(void)
{

  __HAL_RCC_TIM6_CLK_ENABLE();

  __HAL_RCC_TIM7_CLK_ENABLE();
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM6;
  htim1.Init.Prescaler = 839;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2499;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    return 1;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    return 1;
  }

  //HAL_TIM_Base_Start_IT(&htim1);

  htim2.Instance = TIM7;
  htim2.Init.Prescaler = MOTOR_TIMER_PRESCALER - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    return 1;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    return 1;
  }
  //HAL_TIM_Base_Start_IT(&htim2);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
  NVIC_SetPriority(TIM6_DAC_IRQn, 2);
  NVIC_EnableIRQ(TIM7_DAC_IRQn);
  NVIC_SetPriority(TIM7_DAC_IRQn, 3);
  return 0;
}


int motorGPIOSetup(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_STEP2_Pin | MOTOR_DIR1_Pin | MOTOR_DIR2_Pin | MOTOR_ENABLE_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = MOTOR_STEP1_Pin | MOTOR_STEP2_Pin | MOTOR_DIR1_Pin | MOTOR_DIR2_Pin | MOTOR_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  return 0;
}

void motorEnable(void)
{
  HAL_GPIO_WritePin(MOTOR_Port, MOTOR_ENABLE_Pin, GPIO_PIN_RESET);
}

void motorDisable(void)
{
  HAL_GPIO_WritePin(MOTOR_Port, MOTOR_ENABLE_Pin, GPIO_PIN_SET);
}
// en mm & mm/s
void motorMove(int direction, float distance, float vitesse)
{
  distance *= 2; 
  motorReady = 0;
  if (direction == MOTOR_DIR_FORWARD)
  {
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR2_Pin, GPIO_PIN_SET);
  }
  else {
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);
  }

  float nbpas = distance / (((DIAM_ROUE / 2.0) * 2.0 * PI) / PAS_PAR_TOUR) ;

  stepper1.nbpas = nbpas;
  stepper2.nbpas = nbpas;

  float time = (distance / vitesse) / nbpas ;
  float timer = time * ((MOTOR_CLOCK_TIMER * 1000000) /   MOTOR_TIMER_PRESCALER);

  stepper1.timer = timer;
  stepper2.timer = timer;

  MotorStartIRQTimer(&htim1, timer);
  MotorStartIRQTimer(&htim2, timer);
  uartprintf("config timer 2 : %d ,nbpas : %d \n", stepper2.timer, stepper2.nbpas);
}
// en degres et degres par seconde
void motorRotate(int sens_horaire, float angle, float vitesse){
  motorReady = 0;
  if (sens_horaire == MOTOR_DIR_CLOCKWISE)
  {
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR2_Pin, GPIO_PIN_SET);
  }
  else {
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);
  }

  float length_cercle_rotation = (DIAM_INTER_ROUE/2.0)*2.0*PI;
  float distance = length_cercle_rotation * (angle / 360.0); 

  float nbpas = distance / (((DIAM_ROUE / 2.0) * 2.0 * PI) / PAS_PAR_TOUR) ;

  stepper1.nbpas = nbpas;
  stepper2.nbpas = nbpas;

  float time = (angle / vitesse) / nbpas * 1000;
  float timer = time * ((MOTOR_CLOCK_TIMER * 1000000) /   MOTOR_TIMER_PRESCALER);

  stepper1.timer = timer;
  stepper2.timer = timer;

  MotorStartIRQTimer(&htim1, timer);
  MotorStartIRQTimer(&htim2, timer);
 
  uartprintf("config timer 2 : %d ,nbpas : %d \n", stepper2.timer, stepper2.nbpas);
}


int motorIsReady(void)
{
  return (motorReady == (MOTOR_1_READY | MOTOR_2_READY));
}

/**
 * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
 */
extern "C" void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/**
 * @brief This function handles TIM7 global interrupt.
 */
extern "C" void TIM7_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

void MotorStartIRQTimer(TIM_HandleTypeDef *htim, uint16_t time)
{
  __HAL_TIM_SET_AUTORELOAD(htim, time);
  __HAL_TIM_SET_COUNTER(htim, 0);
  HAL_TIM_Base_Start_IT(htim);
}