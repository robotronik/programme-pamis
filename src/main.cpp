/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include "servo.h"
#include "uart.h"
#include "uartLidar.h"
#include "CYdLidar.h"
#include "level_battery.h"
#include "neopixel.h"
#include "system.h"
#include "motor.h"
#include "ledButtonPcb.h"
#include "button.h"
#include "timer_ms.h"
/* Private variables ---------------------------------------------------------*/
// Prototype for the state machine
#define FLASH_STORAGE_ADDR (FLASH_BASE + (FLASH_SIZE - FLASH_PAGE_SIZE)) // Dernière page Flash

#define LED_WING_NB 2
#define LED_WING_POS 0

void Flash_Write(uint8_t value);
uint8_t Flash_Read(void);

uint8_t deplacement(void);
int deplacementSuperStar(void);
int deplacementPremierPamis(void);
int deplacementSecondPamis(void);
int deplacementTroisiemePamis(void);

void debugfun(void);

CYdLidar laser;

typedef enum
{
  team_blue,
  team_yellow,
} team_t;

team_t team;
// numéro du PAMI : 0->superstar / 3 ->  la fosse la plus loin
#define DEFAULT_NUM_PAMIS 1
uint8_t numPamis;

typedef enum
{
  state_debug,
  state_error,
  state_setup,
  state_ready,
  state_wait,
  state_move,
  state_dance,
  state_end,
} state_machine_t;

int main(void)
{

  state_machine_t state_machine;

  // SETUP
  HAL_Init();

  SystemClock_Config();

  timerMsSetup();
  motorSetup();
  LedPcbSetup();
  ButtonPcbSetup();
  ButtonSetup();
  uartSetup();
  uartLidarSetup();
  battSetup();
  servoSetup();
  neopixelSetup();

  NVIC_EnableIRQ;
  __enable_irq();

  HAL_Delay(20); // sinon le condensteur des bouttonPCB est pas chargé, donc le pami va en debug

  if (ButtonPcbGetValue() == GPIO_PIN_RESET)
  {
    neopixelSetLed(0, (colorRGB_t){255, 255, 255});
    state_machine = state_debug;
    HAL_Delay(1000);
    debugfun();
  }
  else
  {
    state_machine = state_setup;
  }
  // Initialisation des différants périphériques
  //  Démarrer PWM sur les canaux 1, 2 et 3
  servoEnable(SERVO1);
  servoEnable(SERVO2);
  servoWrite(SERVO1_CHAN, 1700);
  servoWrite(SERVO2_CHAN, 1700);

  numPamis = Flash_Read(); // Lire la valeur stockée

  if (numPamis == 0xFF)
  {                               // 0xFF signifie mémoire vide
    numPamis = DEFAULT_NUM_PAMIS; // Valeur par défaut
    Flash_Write(numPamis);        // Stocker cette valeur
  }

  // HAL_Delay(1000);//il parait que c'est obligatoirepour le lidar
  // laser.setup();

  uartprintf("Un petit uart de debug ! sur la pin PB3\n");

  // HAL_ADC_Start(&hadc2);
  // neopixelSetLed(0, coleur, 255);

  // //NONblocking
  // while (1){
  //     laser.scanDataNonBlocking();
  //     if(laser.newDataAvailable()){
  //         laser.printLidarPoints();
  //     }
  // }

  int oneTimeOnTen = 0;
  uartprintf("Pami setup Mode\n");
  uint16_t notSpam = 0;
  uint32_t timer;
  colorHSV_t hsv[3] = {{120.0, 1.0, 1.0}, {120.0, 1.0, 1.0}, {120.1, 1.0, 1.0}};
  colorRGB_t wing_color[LED_WING_NB];
  neopixelClear();

  uint32_t timer_button;
  uint32_t timer_battery = millis() + 1000;

  GPIO_PinState oldButtonPcb = GPIO_PIN_SET;
  bool wait85s = true;
  // LOOP
  while (1)
  {

    //    laser.scanDataNonBlocking();
    //    if (laser.newDataAvailable())
    //    {
    //      if (oneTimeOnTen == 10)
    //      {
    //        laser.printLidarPoints();
    //        oneTimeOnTen = 0;
    //      }
    //      else
    //        oneTimeOnTen++;
    //    }

    if (timer_battery < millis())
    {
      timer_battery = millis() + 1000;
      if (battGetPourcentage() < 25.0)
      {
        neopixelSetLed(0, (colorRGB_t){255, 0, 0});
      }
    }
    notSpam++;
    HAL_Delay(5);
    if (notSpam > 2000)
    {
      notSpam = 0;
      uartprintf("--- TIME : %ld --- \n", millis());
      uartprintf("level batt : %f%%\n", battGetPourcentage());
      uartprintf("level batt : %fV\n", battGetVoltage());
      uartprintf("level batt : %d\n", battGetRawValue());
    }

    switch (state_machine)
    {
    case state_setup:
      // on peut modifier le numéro du pamis

      if (ButtonPcbGetValue() == GPIO_PIN_RESET && oldButtonPcb == GPIO_PIN_SET)
      {
        timer_button = millis();
        uartprintf("timer button : %d\n", timer_button);
      }
      if (ButtonPcbGetValue() == GPIO_PIN_SET && oldButtonPcb == GPIO_PIN_RESET)
      {
        if (millis() > timer_button + 500)
        {
          wait85s = !wait85s;
          neopixelSetLed(0, (colorRGB_t){0, 255, 0});
          uartprintf("PAMIS il ne veux pas attendre ^^'\n");
          HAL_Delay(50);
        }
        else
        {
          numPamis++;
          if (numPamis > 3)
          {
            numPamis = 0;
          }
          uartprintf("Pamis change de numéro %d\n", numPamis);
        }
      }
      if (ButtonTeamGetValue() == GPIO_PIN_RESET)
      {
        team = team_blue;
      }
      else
      {
        team = team_yellow;
      }

      switch (numPamis)
      {
      // indication lumineux de la position du PAMIS
      case 0: // super star
        for (int i = 0; i < LED_WING_NB; i++)
        {
          wing_color[i].red = 255;
          wing_color[i].green = 255;
          wing_color[i].blue = 255;
        }
        neopixelSetMoreLeds(LED_WING_POS, wing_color, LED_WING_NB);
        break;
      case 1:
        for (int i = 0; i < LED_WING_NB; i++)
        {
          wing_color[i].red = 255;
          wing_color[i].green = 0;
          wing_color[i].blue = 255;
        }
        neopixelSetMoreLeds(LED_WING_POS, wing_color, LED_WING_NB);
        break;
      case 2:
        for (int i = 0; i < LED_WING_NB; i++)
        {
          wing_color[i].red = 255;
          wing_color[i].green = 0;
          wing_color[i].blue = 0;
        }
        neopixelSetMoreLeds(LED_WING_POS, wing_color, LED_WING_NB);
        break;
      case 3:
        for (int i = 0; i < LED_WING_NB; i++)
        {
          wing_color[i].red = 0;
          wing_color[i].green = 125;
          wing_color[i].blue = 255;
        }
        neopixelSetMoreLeds(LED_WING_POS, wing_color, LED_WING_NB);
        break;
      }

      if (ButtonTiretteGetValue() == GPIO_PIN_RESET)
      {
        // la tirette vien d'être mise en place, le macth va bientôt commencer
        Flash_Write(numPamis); // stock la place du pamis pour le prochain démarage

        uartprintf("Pami is ready \n");
        state_machine = state_ready;
      }

      break;
    case state_ready:
      if (ButtonTeamGetValue() == GPIO_PIN_RESET)
      {
        team = team_blue;
        for (int i = 0; i < LED_WING_NB; i++)
        {
          wing_color[i].red = 0;
          wing_color[i].green = 0;
          wing_color[i].blue = 255;
        }
        neopixelSetMoreLeds(LED_WING_POS, wing_color, LED_WING_NB);
      }
      else
      {
        team = team_yellow;
        for (int i = 0; i < LED_WING_NB; i++)
        {
          wing_color[i].red = 255;
          wing_color[i].green = 255;
          wing_color[i].blue = 0;
        }
        neopixelSetMoreLeds(LED_WING_POS, wing_color, LED_WING_NB);
      }
      if (ButtonTiretteGetValue() == GPIO_PIN_SET)
      {
        // le match vient de démarrer
        uartprintf("Pami wait, the match is started\n");
        timer = millis() + (wait85s == true ? 85000 : 1000); // TODO set correct value 85000

        state_machine = state_wait;
      }

      break;
    case state_wait:
      if (ButtonTiretteGetValue() == GPIO_PIN_RESET)
      {
        state_machine = state_setup;
      }
      if (millis() > timer || ButtonPcbGetValue() == GPIO_PIN_RESET)
      {
        uartprintf("PAMIS want to MOVE >:-(\n");
        state_machine = state_move;
        motorEnable();
        HAL_Delay(15);
      }
      break;
    case state_move:
      // TODO gestion du movement
      if (deplacement())
      {
        motorDisable();
        uartprintf("PAMIS DANCE");
        state_machine = state_dance;
        timer = millis();
      }
      break;
    case state_dance:
      static uint8_t dance_phase = 0;
      // TODO let's go dance !
      if (timer < millis())
      {
        timer = millis() + 300;
        switch (dance_phase)
        {
        case 0:
          servoWrite(SERVO1_CHAN, 750);
          servoWrite(SERVO2_CHAN, 750);
          dance_phase = 1;
          break;
        case 1:
          servoWrite(SERVO1_CHAN, 2200);
          servoWrite(SERVO2_CHAN, 2200);
          dance_phase = 0;
          break;

        default:
          dance_phase = 0;
          break;
        }
      }
      hsv[0].H = fmod(hsv[0].H + 0.2, 360.0);
      hsv[1].H = fmod(hsv[1].H + 0.2, 360.0);
      hsv[2].H = fmod(hsv[2].H + 0.2, 360.0);
      neopixelSetMoreLeds(0, hsv, 3);
      if (ButtonTiretteGetValue() == GPIO_PIN_RESET)
      {
        state_machine = state_setup;
      }
      break;
    case state_end:
      // c'est fini T-T
      break;
    default:
      state_machine = state_error;
    }
    // fin LOOP
    oldButtonPcb = ButtonPcbGetValue();
  }
}

void debugfun(void)
{
  servoEnable(SERVO1);
  servoEnable(SERVO2);

  // ici c'est la guerre, on test des trucs et ça marche pas...

  uint32_t level_battery = 1240;
  colorRGB_t coleur[3] = {{0, 0, 0}, {0, 55, 0}, {0, 0, 127}};

  uartprintf("MODE DEBUG ACTIVÉ !!!\n");
  while (1)
  {
    coleur[0].red = (coleur[0].red + 5) % 256;
    coleur[1].green = (coleur[1].green + 5) % 256;
    coleur[2].blue = (coleur[2].blue + 5) % 256;

    neopixelSetMoreLeds(0, coleur, 3);
    HAL_Delay(50);

    // Changer la position des servos
    servoWrite(SERVO1_CHAN, 1300);
    servoWrite(SERVO2_CHAN, 2000);

    HAL_Delay(100);
    level_battery = battGetRawValue();

    if (ButtonPcbGetValue() == GPIO_PIN_RESET)
    {
      uartprintf("level batt : %f\n", battGetPourcentage());
      motorEnable();

      motorRotate(MOTOR_DIR_ANTICLOCKWISE, 180, 90, 180, 180);
      motorRotate(MOTOR_DIR_CLOCKWISE, 180, 90, 180, 180);
      motorRotate(MOTOR_DIR_ANTICLOCKWISE, 180, 90, 180, 0);
      motorRotate(MOTOR_DIR_ANTICLOCKWISE, 180, 90, 0, 180);

      while (!motorIsReady())
        ;

      motorDisable();
      for (int i = 0; i < 1; i++)
      {
        servoWrite(SERVO1_CHAN, 2000);
        servoWrite(SERVO2_CHAN, 1300);

        HAL_Delay(300);

        // Changer la position des servos
        servoWrite(SERVO1_CHAN, 1300);
        servoWrite(SERVO2_CHAN, 2000);
        HAL_Delay(300);
      }
      motorDisable();
    }
  }
}

/**
 * @brief Écrit un uint8_t dans la mémoire Flash
 */
void Flash_Write(uint8_t value)
{
  HAL_FLASH_Unlock();

  // Effacer la page (obligatoire avant écriture)
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page = (FLASH_STORAGE_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
  EraseInitStruct.NbPages = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return;
  }

  // Écrire la nouvelle valeur
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_STORAGE_ADDR, value);

  HAL_FLASH_Lock();
}

/**
 * @brief Lit un uint8_t depuis la mémoire Flash
 */
uint8_t Flash_Read(void)
{
  return *((uint8_t *)FLASH_STORAGE_ADDR);
}

uint8_t deplacement(void)
{
  uartprintf("motorIsReady ? %d for pamis %d\n", motorIsReady(), numPamis);

  if (motorIsReady())
  {
    switch (numPamis)
    {
    case 0:
      if (deplacementSuperStar())
        return 1;
      break;

    case 1: // le pamis le plus proche
      if (deplacementPremierPamis())
        return 1;
      break;
    case 2: // le pamis du milieu
      if (deplacementSecondPamis())
        return 1;
      break;
    case 3: // le pamis du fond
      if (deplacementTroisiemePamis())
        return 1;
      break;
    default:
      neopixelSetLed(0, (colorRGB_t){255, 0, 0});
      break;
    }
  }
  return 0;
}

int deplacementPremierPamis(void)
{
  uint8_t rotation = (team == team_yellow ? MOTOR_DIR_CLOCKWISE : MOTOR_DIR_ANTICLOCKWISE);

  static uint8_t phaseDeplacement = 0;
  switch (phaseDeplacement)
  {
  case 0:
    motorMove(MOTOR_DIR_FORWARD, 300, 300, 300, 0);
    break;
  case 1:
    motorRotate(rotation, 35, 90);
    break;
  case 2:
    motorMove(MOTOR_DIR_FORWARD, 800, 300, 0, 300);
    break;
  case 3:
    motorRotate(!rotation, 85, 90);

    break;
  case 4:
    phaseDeplacement = 0;
    return 1;
    break;
  default:
    phaseDeplacement = 0;
    break;
  }
  phaseDeplacement++;
  return 0;
}

int deplacementSecondPamis(void)
{
  uint8_t rotation = (team == team_yellow ? MOTOR_DIR_CLOCKWISE : MOTOR_DIR_ANTICLOCKWISE);

  static uint8_t phaseDeplacement = 0;
  switch (phaseDeplacement)
  {
  case 0:
    motorMove(MOTOR_DIR_FORWARD, 250, 150, 100, 0);
    break;
  case 1:
    motorRotate(rotation, 35, 45);
    break;
  case 2:
    motorMove(MOTOR_DIR_FORWARD, 700, 180);
    break;
  case 3:
    motorRotate(!rotation, 35, 45);

    break;
  case 4:
    motorMove(MOTOR_DIR_FORWARD, 300, 150);
    break;
  case 5:
    motorRotate(!rotation, 90, 45);
    break;
  case 6:
    motorMove(MOTOR_DIR_FORWARD, 50, 150, 0, 100);
    break;
  case 7:
    phaseDeplacement = 0;
    return 1;
    break;
  default:
    phaseDeplacement = 0;
    break;
  }
  phaseDeplacement++;
  return 0;
}
int deplacementTroisiemePamis(void)
{
  uint8_t rotation = (team == team_yellow ? MOTOR_DIR_CLOCKWISE : MOTOR_DIR_ANTICLOCKWISE);

  static uint8_t phaseDeplacement = 0;
  switch (phaseDeplacement)
  {
  case 0:
    motorMove(MOTOR_DIR_FORWARD, 200, 150, 100, 0);
    break;
  case 1:
    motorRotate(rotation, 35, 45);
    break;
  case 2:
    motorMove(MOTOR_DIR_FORWARD, 800, 180);
    break;
  case 3:
    motorRotate(!rotation, 35, 45);

    break;
  case 4:
    motorMove(MOTOR_DIR_FORWARD, 1000, 150);
    break;
  case 5:
    motorRotate(!rotation, 115, 45);
    break;
  case 6:
    motorMove(MOTOR_DIR_FORWARD, 80, 180, 0, 100);
    break;
  case 7:
    phaseDeplacement = 0;
    return 1;
    break;
  default:
    phaseDeplacement = 0;
    break;
  }
  phaseDeplacement++;
  return 0;
}

int deplacementSuperStar(void)
{
  float rotation;
  
  HAL_Delay(30);
  static uint8_t phaseDeplacement = 0;
  uartprintf("step %d\n", phaseDeplacement);

  HAL_Delay(30);
  switch (phaseDeplacement)
  {
  case 0:
    motorMove(MOTOR_DIR_FORWARD, 1000, 250, 200, 0);
    break;
  case 1:
    rotation = 120;
    if (team_yellow == team)
    {
    }
    else
    {
      rotation = -120;
    }
    motorTurn(MOTOR_DIR_FORWARD, 80.0, rotation, 150);
    break;
  case 2:
    motorMove(MOTOR_DIR_FORWARD, 150, 300, 0, 300);
    break;
  case 3:
    phaseDeplacement = 0;
    return 1;
    break;

  default:
    phaseDeplacement = 0;
    break;
  }
  phaseDeplacement++;
  return 0;
}