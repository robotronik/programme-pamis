#include "motor.h"
#include <math.h>
#include "system.h"
#include "ledButtonPcb.h"
#include "uart.h"

#define MOTOR_1_READY (1 << 0)
#define MOTOR_2_READY (1 << 1)

typedef struct
{
  unsigned int nbpas; // nb de pas total à faire
  unsigned int nbpasactuel;
  unsigned int accel;  // nb de pas d'accel
  unsigned int deccel; // nbpas de decel
  unsigned int timer;  // timer min
} deplacement_t;

enum TYPE_DEPLACEMENT
{
  DEPLACEMENT_LINE,
  DEPLACEMENT_ROTATION,
  DEPLACEMENT_TURN
};

typedef struct
{
  TYPE_DEPLACEMENT type;
  void *parameter;
} element_t;

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

void motorStepper1Fallback(void)
{
  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_GPIO_TogglePin(MOTOR_Port, MOTOR_STEP1_Pin);
  stepper1.nbpasactuel++;

  if (stepper1.nbpasactuel < stepper1.nbpas)
  {
    uint32_t new_timer;

    if (stepper1.nbpasactuel <= stepper1.accel)
    {
      // Accélération : timer diminue
      float progress = (float)stepper1.nbpasactuel / stepper1.accel;
      new_timer = stepper1.timer + (uint32_t)((1.0f - progress) * stepper1.timer);
    }
    else if (stepper1.nbpasactuel >= (stepper1.nbpas - stepper1.deccel))
    {
      // Décélération : timer augmente
      float progress = (float)(stepper1.nbpas - stepper1.nbpasactuel) / stepper1.deccel;
      new_timer = stepper1.timer + (uint32_t)((1.0f - progress) * stepper1.timer);
    }
    else
    {
      // Vitesse constante
      new_timer = stepper1.timer;
    }

    MotorStartIRQTimer(&htim1, new_timer);
  }
  else
  {
    motorReady |= MOTOR_1_READY;
  }
}
void motorStepper2Fallback(void)
{
  HAL_TIM_Base_Stop_IT(&htim2);
  HAL_GPIO_TogglePin(MOTOR_Port, MOTOR_STEP2_Pin);
  stepper2.nbpasactuel++;

  if (stepper2.nbpasactuel < stepper2.nbpas)
  {
    uint32_t new_timer;

    if (stepper2.nbpasactuel <= stepper2.accel)
    {
      // Accélération : timer diminue
      float progress = (float)stepper2.nbpasactuel / stepper2.accel;
      new_timer = stepper2.timer + (uint32_t)((1.0f - progress) * stepper2.timer);
    }
    else if (stepper2.nbpasactuel >= (stepper2.nbpas - stepper2.deccel))
    {
      // Décélération : timer augmente
      float progress = (float)(stepper2.nbpas - stepper2.nbpasactuel) / stepper2.deccel;
      new_timer = stepper2.timer + (uint32_t)((1.0f - progress) * stepper2.timer);
    }
    else
    {
      // Vitesse constante
      new_timer = stepper2.timer;
    }

    MotorStartIRQTimer(&htim2, new_timer);
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

  // HAL_TIM_Base_Start_IT(&htim1);

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
  // HAL_TIM_Base_Start_IT(&htim2);
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
  motorMove(direction, distance, vitesse, 0, 0);
}
void motorMove(int direction, float distance, float vitesse, float accel, float deccel)
{
  uint32_t timeout = HAL_GetTick();
  while (!motorIsReady())
  {
    if (HAL_GetTick() - timeout > 100) // Timeout de 100 ms
    {
      Error_Handler();
      break;
    }
  }
  motorReady = 0;

  if (direction == MOTOR_DIR_FORWARD)
  {
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR2_Pin, GPIO_PIN_SET);
  }

  float nbpas = distance / ((DIAM_ROUE * PI) / PAS_PAR_TOUR);
  stepper1.nbpas = nbpas;
  stepper2.nbpas = nbpas;

  float t_min = 1.0f / (vitesse / ((DIAM_ROUE * PI) / PAS_PAR_TOUR)); // temps min entre deux pas
  uint32_t timer_min = t_min * ((MOTOR_CLOCK_TIMER * 1000000.0f) / MOTOR_TIMER_PRESCALER);

  stepper1.timer = timer_min;
  stepper2.timer = timer_min;

  stepper1.nbpasactuel = 0;
  stepper2.nbpasactuel = 0;

  // Calcul des zones d'accel/deccel (en pas)
  uint32_t accel_steps = (uint32_t)((vitesse * vitesse) / (2.0f * accel * ((DIAM_ROUE * PI) / PAS_PAR_TOUR)));
  uint32_t deccel_steps = (uint32_t)((vitesse * vitesse) / (2.0f * deccel * ((DIAM_ROUE * PI) / PAS_PAR_TOUR)));

  if ((accel_steps + deccel_steps) >= nbpas)
  {
    // Cas où le plateau n'existe pas → on ajuste pour faire un triangle
    accel_steps = nbpas / 2;
    deccel_steps = nbpas - accel_steps;
  }

  stepper1.accel = accel_steps;
  stepper1.deccel = deccel_steps;
  stepper2.accel = accel_steps;
  stepper2.deccel = deccel_steps;

  MotorStartIRQTimer(&htim1, timer_min + 2000); // Démarrage lent pour phase d'accel
  MotorStartIRQTimer(&htim2, timer_min + 2000);

  uartprintf("Timer min : %d, Pas: %d, Accel: %d, Deccel: %d\n", timer_min, (int)nbpas, accel_steps, deccel_steps);
}

//  en degres et degres par seconde
void motorRotate(int sens_horaire, float angle, float vitesse)
{
  motorRotate(sens_horaire, angle, vitesse, 0, 0);
}

void motorRotate(int sens_horaire, float angle, float vitesse, float accel, float deccel)
{
  uint32_t timeout = HAL_GetTick();
  while (!motorIsReady())
  {
    if (HAL_GetTick() - timeout > 100) // Timeout de 100 ms
    {
      Error_Handler();
      break;
    }
  }
  if (angle < 0)
  {
    angle = -angle;
    sens_horaire = !sens_horaire;
  }

  motorReady = 0;

  if (sens_horaire == MOTOR_DIR_CLOCKWISE)
  {
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR2_Pin, GPIO_PIN_SET);
  }

  float length_cercle_rotation = DIAM_INTER_ROUE * PI;
  float distance = length_cercle_rotation * (angle / 360.0f);
  float nbpas = distance / ((DIAM_ROUE * PI) / PAS_PAR_TOUR);

  if (nbpas <= 0)
    return;

  stepper1.nbpas = nbpas;
  stepper2.nbpas = nbpas;

  // Calcule du temps par pas
  float time = (angle / vitesse) / nbpas;
  float timer = time * ((MOTOR_CLOCK_TIMER * 1000000) / MOTOR_TIMER_PRESCALER);

  stepper1.timer = timer;
  stepper2.timer = timer;

  stepper1.nbpasactuel = 0;
  stepper2.nbpasactuel = 0;

  // Gestion accélération / décélération
  uint16_t accelSteps = (uint16_t)(nbpas * accel);
  uint16_t deccelSteps = (uint16_t)(nbpas * deccel);

  // On s'assure qu'on dépasse pas le nombre total de pas
  if (accelSteps + deccelSteps > nbpas)
  {
    accelSteps = deccelSteps = nbpas / 2;
  }

  stepper1.accel = accelSteps;
  stepper1.deccel = deccelSteps;

  stepper2.accel = accelSteps;
  stepper2.deccel = deccelSteps;

  MotorStartIRQTimer(&htim1, timer);
  MotorStartIRQTimer(&htim2, timer);

  uartprintf("config timer 2 : %d ,nbpas : %d \n", stepper2.timer, stepper2.nbpas);
}

// point de rotation positif : clockwise
void motorTurn(int direction, float angle, float PointOfRotation, float vitesse)
{
  uint32_t timeout = HAL_GetTick();
  while (!motorIsReady())
  {
    if (HAL_GetTick() - timeout > 100) // Timeout de 100 ms
    {
      Error_Handler();
      break;
    }
  }
  if (angle < 0)
  {
    angle = -angle;
    direction = !direction;
  }
  if (PointOfRotation < DIAM_INTER_ROUE / 2 && PointOfRotation > -DIAM_INTER_ROUE / 2)
  {
    uartprintf("You need to hava a point of rotation at the lenght between the to wheel\n");
    return;
  }
  motorReady = 0;
  if (direction == MOTOR_DIR_FORWARD)
  {
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_Port, MOTOR_DIR2_Pin, GPIO_PIN_SET);
  }

  float length_cercle_rotation_ext = (abs(PointOfRotation) + (DIAM_INTER_ROUE / 2.0)) * 2.0 * PI;
  float distance_ext = length_cercle_rotation_ext * angle / 360.0;
  float nbpas_ext = distance_ext / (((DIAM_ROUE)*PI) / PAS_PAR_TOUR);

  float time_ext = (distance_ext / vitesse) / nbpas_ext;
  float timer_ext = time_ext * ((MOTOR_CLOCK_TIMER * 1000000) / MOTOR_TIMER_PRESCALER);

  float length_cercle_rotation_int = (abs(PointOfRotation) - (DIAM_INTER_ROUE / 2.0)) * 2.0 * PI;
  float distance_int = length_cercle_rotation_int * (angle / 360.0);
  float nbpas_int = distance_int / (((DIAM_ROUE)*PI) / PAS_PAR_TOUR);

  float time_int = (distance_ext / vitesse) / nbpas_int;
  float timer_int = time_int * ((MOTOR_CLOCK_TIMER * 1000000) / MOTOR_TIMER_PRESCALER);

  uartprintf("INT timer : %f - distance : %f  - nbpasINT %f /\n EXT timer %f - distance %f nbpasEXT : %f \n ", timer_int, distance_int, nbpas_int, timer_ext, distance_ext, nbpas_ext);
  if (PointOfRotation < 0)
  {
    uartprintf("motor turn anticlowkwise");

    stepper1.timer = timer_ext;
    stepper2.timer = timer_int;

    stepper1.nbpas = nbpas_ext;
    stepper2.nbpas = nbpas_int;

    stepper1.nbpasactuel = 0;
    stepper2.nbpasactuel = 0;

    stepper1.accel = 0;
    stepper1.deccel = 0;
    stepper2.accel = 0;
    stepper2.deccel = 0;

    MotorStartIRQTimer(&htim1, timer_ext);
    MotorStartIRQTimer(&htim2, timer_int);
  }
  else
  {
    uartprintf("motor turn clowkwise");
    stepper1.timer = timer_int;
    stepper2.timer = timer_ext;

    stepper1.nbpas = nbpas_int;
    stepper2.nbpas = nbpas_ext;

    stepper1.nbpasactuel = 0;
    stepper2.nbpasactuel = 0;

    stepper1.accel = 0;
    stepper1.deccel = 0;
    stepper2.accel = 0;
    stepper2.deccel = 0;

    MotorStartIRQTimer(&htim1, timer_int);
    MotorStartIRQTimer(&htim2, timer_ext);
  }

  uartprintf("config timer 1 : %d ,nbpas : %d \n", stepper1.timer, stepper1.nbpas);
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