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
/* Private variables ---------------------------------------------------------*/

CYdLidar laser;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  // SETUP
  HAL_Init();

  SystemClock_Config();

  motorSetup();
  LedPcbSetup();
  ButtonPcbSetup();
  uartSetup();
  uartLidarSetup();
  battSetup();
  servoSetup();
  neopixelSetup();

  NVIC_EnableIRQ;
  __enable_irq();

  // Initialisation des différants périphériques
  //  Démarrer PWM sur les canaux 1, 2 et 3
  servoEnable(SERVO1);
  servoEnable(SERVO2);

  HAL_Delay(1000);
  laser.setup();

  uartprintf("Un petit uart de debug ! sur la pin PB3\n");

  // HAL_ADC_Start(&hadc2);
  color_t coleur[3] = {{0, 0, 0}, {0, 55, 0}, {0, 0, 127}};
  // neopixelSetLed(0, coleur, 255);
  uint32_t level_battery = 1240;

  // //NONblocking
  // while (1){
  //     laser.scanDataNonBlocking();
  //     if(laser.newDataAvailable()){
  //         laser.printLidarPoints();
  //     }
  // }

  int oneTimeOnTen = 0;
  // LOOP
  while (1)
  {
    laser.scanDataNonBlocking();
    if (laser.newDataAvailable())
    {
      if (oneTimeOnTen == 10)
      {
        laser.printLidarPoints();
        oneTimeOnTen = 0;
      }
      else
        oneTimeOnTen++;
    }
    coleur[0].red = (coleur[0].red + 5) % 256;
    coleur[1].green = (coleur[1].green + 5) % 256;
    coleur[2].blue = (coleur[2].blue + 5) % 256;

    neopixelSetMoreLeds(0, coleur, 3);
    HAL_Delay(50);

    // Changer la position des servos
    servoWrite(SERVO1_CHAN, 1500);
    servoWrite(SERVO2_CHAN, 2000);

    HAL_Delay(100);
    // uartprintf("level : %u :3\n", level_battery);

    if (ButtonPcbGetValue() == GPIO_PIN_RESET)
    {
      uartprintf("button 0 \n ");
      motorEnable();

      // motorMove(MOTOR_DIR_FORWARD, 100.0, 20);
      motorRotate(MOTOR_DIR_CLOCKWISE, 90, 90);
      while (motorIsReady() == 0)
        ;

      motorDisable();
      for (int i = 0; i < 0; i++)
      {
        servoWrite(SERVO1_CHAN, 2000);
        servoWrite(SERVO2_CHAN, 1500);

        HAL_Delay(500);

        // Changer la position des servos
        servoWrite(SERVO1_CHAN, 1500);
        servoWrite(SERVO2_CHAN, 2000);
        HAL_Delay(500);
      }
    }
  }
}
