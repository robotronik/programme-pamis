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

#define NB_LED_DEBUG 1
#define NB_LED_ARM 2
#define LED_ARM_POS 1

void Flash_Write(uint8_t value);
uint8_t Flash_Read(void);

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
  servoWrite(SERVO1_CHAN,1700); 
  servoWrite(SERVO2_CHAN,1700); 

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
  uint16_t notSpam;
  uint32_t timer;
  colorHSV_t hsv = {120.0, 1.0, 1.0};
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
      if (ButtonPcbGetValue() == GPIO_PIN_RESET)
      {
        // modif la position et rôle du pamis
        HAL_Delay(500);
        numPamis++;
        if (numPamis > 3)
        {
          numPamis = 0;
        }
        uartprintf("Pamis change de numéro %d\n", numPamis);
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
        neopixelSetLed(0, (colorRGB_t){255, 255, 255});
        break;
      case 1:
        neopixelSetLed(0, (colorRGB_t){255, 0, 255});
        break;
      case 2:
        neopixelSetLed(0, (colorRGB_t){255, 0, 0});
        break;
      case 3:
        neopixelSetLed(0, (colorRGB_t){0, 0, 255});
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
        neopixelSetLed(0, (colorRGB_t){0, 0, 255});
      }
      else
      {
        team = team_yellow;
        neopixelSetLed(0, (colorRGB_t){255, 255, 0});
      }
      if (ButtonTiretteGetValue() == GPIO_PIN_SET)
      {
        // le match vient de démarrer
        uartprintf("Pami wait, the match is started\n");
        timer = millis() + 3000; // TODO set correct value 85000
        state_machine = state_wait;
      }
      break;
    case state_wait:
      if (millis() > timer)
      {
        uartprintf("PAMIS want to MOVE >:-(\n");
        state_machine = state_move;
        motorEnable();
        motorMove(MOTOR_DIR_FORWARD, 1500, 200);
      }
      break;
    case state_move:
      // TODO gestion du movement
      if (motorIsReady() == 1)
      {
        motorDisable();
        uartprintf("PAMIS DANCE");
        state_machine = state_dance;
      }
      break;
    case state_dance:
      static uint8_t dance_phase = 0;
      // TODO let's go dance !
      if (timer < millis())
      {
        timer = millis() + 500;
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
      hsv.H = fmod(hsv.H + 0.2, 360.0);
      neopixelSetMoreLeds(0, &hsv, 1);
      break;
    case state_end:
      // c'est fini T-T
      break;
    default:
      state_machine = state_error;
    }
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
    servoWrite(SERVO1_CHAN, 1500);
    servoWrite(SERVO2_CHAN, 2000);

    HAL_Delay(100);
    level_battery = battGetRawValue();

    if (ButtonPcbGetValue() == GPIO_PIN_RESET)
    {
      uartprintf("level batt : %f\n", battGetPourcentage());
      motorEnable();

      motorMove(MOTOR_DIR_FORWARD, 100.0, 200);
      // motorRotate(MOTOR_DIR_CLOCKWISE, 90, 90);
      while (motorIsReady() == 0)
        ;

      motorDisable();
      for (int i = 0; i < 1; i++)
      {
        servoWrite(SERVO1_CHAN, 2000);
        servoWrite(SERVO2_CHAN, 1500);

        HAL_Delay(500);

        // Changer la position des servos
        servoWrite(SERVO1_CHAN, 1500);
        servoWrite(SERVO2_CHAN, 2000);
        HAL_Delay(500);
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