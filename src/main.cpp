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
#include "CYdLidar.h"
#include "button.h"
#include "ledButtonPcb.h"
#include "level_battery.h"
#include "motor.h"
#include "neopixel.h"
#include "servo.h"
#include "system.h"
#include "timer_ms.h"
#include "uart.h"
#include "uartLidar.h"

#define LIDAR_TR_INTENS 6
/* Private variables ---------------------------------------------------------*/
// Prototype for the state machine
#define FLASH_STORAGE_ADDR                                                     \
  (FLASH_BASE + (FLASH_SIZE - FLASH_PAGE_SIZE)) // Dernière page Flash

#define LED_WING_NB 2
#define LED_WING_POS 1

#define SEUIL_LIDAR_DIST 80.0
#define SEUIL_LIDAR_ANGL 10.0

typedef enum {
  team_blue,
  team_yellow,
} team_t;

team_t team;
// numéro du PAMI : 0->superstar / 3 ->  la fosse la plus loin
typedef enum {
  // Ni Oublie Ni Pardon
  Geraldine, // TdS colombiene 2025
  Angelina,  // 2025
  Avril,     // lille lycéenne exclu pour avoir porté un jupe 2020
  Doona,     // Crous  2020
  Dinah      // lesbienne racisée 2021

} pamis_t;
#define DEFAULT_NUM_PAMIS 1

void Flash_Write(uint8_t value);
uint8_t Flash_Read(void);

uint8_t preSavedDeplacement(uint8_t numPamis, team_t team);

void debugfun(void);

CYdLidar laser;

uint8_t numPamis;

typedef enum {
  state_debug,
  state_error,
  state_setup,
  state_ready,
  state_wait,
  state_move,
  state_dance,
  state_end,
} state_machine_t;

int main(void) {

  state_machine_t state_machine;

  // SETUP
  HAL_Init();

  SystemClock_Config();
  
  LedPcbSetup();


  timerMsSetup();
  motorSetup();
  ButtonPcbSetup();
  ButtonSetup();
  uartSetup();
  uartLidarSetup();
  battSetup();
  servoSetup();
  neopixelSetup();
  neopixelSetFormat(0, FORMAT_GRB_INP33);

  __enable_irq();

  HAL_Delay(150); // sinon le condensteur des bouttonPCB est pas chargé, donc le
                  // pami va en debug

  laser.setup();

  if (ButtonPcbGetValue() == GPIO_PIN_RESET) {
    neopixelSetLed(0, (colorRGB_t){255, 255, 255});
    state_machine = state_debug;
    HAL_Delay(1000);
    debugfun();
  } else {
    state_machine = state_setup;
  }
  // Initialisation des différants périphériques
  //  Démarrer PWM sur les canaux 1, 2 et 3
  servoEnable(SERVO1);
  servoEnable(SERVO2);
  servoWrite(SERVO1_CHAN, 500);
  servoWrite(SERVO2_CHAN, 2500);

  numPamis = Flash_Read(); // Lire la valeur stockée

  if (numPamis == 0xFF) {         // 0xFF signifie mémoire vide
    numPamis = DEFAULT_NUM_PAMIS; // Valeur par défaut
    Flash_Write(numPamis);        // Stocker cette valeur
  }

  // HAL_Delay(1000);//il parait que c'est obligatoirepour le lidar
  // laser.setup();

  uartprintf("[main] Un petit uart de debug ! sur la pin PB3\n");

  // //NONblocking
  // while (1){
  //     laser.scanDataNonBlocking();
  //     if(laser.newDataAvailable()){
  //         laser.printLidarPoints();
  //     }
  // }

  uartprintf("[main] Pami setup Mode\n");
  uint32_t timer;
  colorHSV_t hsv = {120.0, 1.0, 1.0};
  colorRGB_t wing_color[LED_WING_NB];
  neopixelClear();

  uint8_t inMatch = 0;
  uint32_t timer_button = 0;
  uint32_t timer_nospam = 0;
  uint32_t timer_start;
  uint32_t timer_battery = millis() + 1000;

  GPIO_PinState oldButtonPcb = GPIO_PIN_SET;
  GPIO_PinState actualButtonPCBValue = GPIO_PIN_SET;

  bool wait85s = true;
  // LOOP
  while (1) {

    laser.scanDataNonBlocking();

    if (timer_battery < millis()) {
      timer_battery = millis() + 500;
      if (battGetPourcentage() < 30.0) {
        uartprintf("[main] : la batterie est à %f%% T-T\n",
                   battGetPourcentage());
        neopixelSetAllLeds((colorRGB_t){255, 0, 0});
        HAL_Delay(50);
      }
    }
    if (timer_nospam < millis()) {
      timer_nospam = millis() + 3000;
      uartprintf("[main] --- TIME : %ld - %ld--- \n", millis(), HAL_GetTick());
      uartprintf("[main] level batt : %f%%\n", battGetPourcentage());
      uartprintf("[main] level batt : %fV\n", battGetVoltage());
      uartprintf("[main] level batt : %d\n", battGetRawValue());
      uartprintf("[main] timer start + 100s : %d\n",timer_start); 
    }
    // timeout le match est fini
    if (timer_start < HAL_GetTick() && inMatch) {
      uartprintf("[main] le pami doit avoir attend sont objectif !");
      state_machine = state_dance;
      motorDisable();
    }

    switch (state_machine) {
    case state_error:
      uartprintf("[main] ERROR in the state machine D: at %ld\n", millis());
      motorDisable();
      neopixelSetAllLeds((colorRGB_t){255, 0, 0});
      while (1) {
        // ne rends jamais la main
      }
      break;
    case state_setup:
      // on peut modifier le numéro du pamis
      oldButtonPcb = actualButtonPCBValue;
      actualButtonPCBValue = ButtonPcbGetValue();

      if (actualButtonPCBValue == GPIO_PIN_RESET &&
          oldButtonPcb == GPIO_PIN_SET) {
        timer_button = millis();
        uartprintf("[main] timer button : %d\n", timer_button);
      }
      if (actualButtonPCBValue == GPIO_PIN_SET &&
          oldButtonPcb == GPIO_PIN_RESET) {
        if (millis() > timer_button + 500) {
          wait85s = !wait85s;
          neopixelSetAllLeds((colorRGB_t){0, 255, 0});
          uartprintf("[main] PAMIS il ne veux pas attendre ^^'\n");
          HAL_Delay(100);
        } else {
          numPamis++;
          if (numPamis > 3) {
            numPamis = 0;
          }
          uartprintf("[main] Pamis change de numéro %d\n", numPamis);
        }
      }
      if (ButtonTeamGetValue() == GPIO_PIN_RESET) {
        team = team_blue;
      } else {
        team = team_yellow;
      }

      switch (numPamis) {
      // indication lumineux de la position du PAMIS
      case 0: // super star
        neopixelSetAllLeds((colorRGB_t){255, 255, 255});
        break;
      case 1:
        neopixelSetAllLeds((colorRGB_t){255, 0, 255});
        break;
      case 2:
        neopixelSetAllLeds((colorRGB_t){255, 0, 0});
        break;
      case 3:
        neopixelSetAllLeds((colorRGB_t){0, 127, 255});
        break;
      }
      if (ButtonTiretteGetValue() == GPIO_PIN_RESET) {
        // la tirette vien d'être mise en place, le macth va bientôt commencer

        Flash_Write(
            numPamis); // stock la place du pamis pour le prochain démarage

        uartprintf("[main] Pami is ready \n");
        state_machine = state_ready;
      }
      break;
    case state_ready:
      if (ButtonTeamGetValue() == GPIO_PIN_RESET) {
        team = team_blue;
        neopixelSetAllLeds((colorRGB_t){0, 0, 255});
      } else {
        team = team_yellow;
        neopixelSetAllLeds((colorRGB_t){255, 255, 0});
      }
      if (ButtonTiretteGetValue() == GPIO_PIN_SET) {
        // le match vient de démarrer
        uartprintf("[main] Pami wait, the match is started\n");
        inMatch = 1;
        timer_start = millis() + 100000;
        timer =
            millis() + (wait85s == true ? (85000 - 150)
                                        : 1000); // TODO set correct value 85000

        state_machine = state_wait;
      }

      break;
    case state_wait:
      if (ButtonTiretteGetValue() == GPIO_PIN_RESET) {
        state_machine = state_setup;
      }
      if (millis() > timer || ButtonPcbGetValue() == GPIO_PIN_RESET) {
        uartprintf("[main] PAMIS want to MOVE >:-(\n");
        if (numPamis == 0) {
          CapteurSetup();
        }
        state_machine = state_move;
        motorEnable();
        HAL_Delay(150);
        preSavedDeplacement(numPamis, team);
      }
      break;
    case state_move:
      static uint8_t isPaused = false;

      laser.scanDataNonBlocking();
      if (laser.newDataAvailable()) {
        // laser.printLidarPoints() ;
        bool farEnough = true;
        for (int i = 0; i++ < GS_MAX_SAMPLE;) {

          if (laser.samples[i].intensity > LIDAR_TR_INTENS &&
              abs(laser.samples[i].angle) <
                  ((SEUIL_LIDAR_ANGL * 2 * PI) / 360.0)) {

            if (laser.samples[i].distance < SEUIL_LIDAR_DIST) {
              uartprintf("detection d'un obstacle \n");
              farEnough = false;
            }
          }
        }

        if (isPaused == false && farEnough == false) {
          uartprintf("[main] something is too near, pausing movt \n");

          isPaused = true;
          motorPause();
        } else if (isPaused == true && farEnough == true) {

          uartprintf("[main] the obstacle is gone\n");
          isPaused = false;
          motorResume();
        }
      }

      if (motorIsReady()) {
        if (numPamis == 0) {

          motorMove(MOTOR_DIR_FORWARD, 400, 100, 50, 100); // démarafe
          while (!motorIsReady() && CapteurGetValue() == GPIO_PIN_RESET) {
          }

          motorDisable();
          uartprintf("[main] PAMIS DANCE");
          state_machine = state_dance;
          timer = millis();

        } else {

          motorDisable();
          uartprintf("[main] PAMIS DANCE");
          state_machine = state_dance;
          timer = millis();
        }
      }
      break;
    case state_dance:
      static uint8_t dance_phase = 0;
      // TODO let's go dance !
      if (timer < millis()) {
        timer = millis() + 400;
        switch (dance_phase) {
        case 0:
          servoWrite(SERVO1_CHAN, 700);
          servoWrite(SERVO2_CHAN, 700);
          dance_phase = 1;
          break;
        case 1:
          servoWrite(SERVO1_CHAN, 2000);
          servoWrite(SERVO2_CHAN, 2000);
          dance_phase = 0;
          break;

        default:
          dance_phase = 0;
          break;
        }
      }
      hsv.H = fmod(hsv.H + 0.2, 360.0);
      neopixelSetAllLeds(hsv);
      if (ButtonTiretteGetValue() == GPIO_PIN_RESET) {
        inMatch = 0;
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
  }
}

void debugfun(void) {
  servoEnable(SERVO1);
  servoEnable(SERVO2);

  // ici c'est la guerre, on test des trucs et ça marche pas...

  uint32_t level_battery = 1240;
  uartprintf(">pos:0.0:0.0|clr\n>angle:0.0|clr\n");
  uartprintf(">pos:500.0:500.0\n>angle:360.0\n");
  uartprintf(">pos:-500.0:-500.0\n>angle:0.0\n");
  colorRGB_t coleur[3] = {{0, 0, 0}, {0, 55, 0}, {0, 0, 127}};

  uartprintf("[main] MODE DEBUG ACTIVÉ !!!\n");
  uint16_t oneTimeOnTen = 0;
  uint16_t noSpam_1on100 = 0;
  CapteurSetup();

  laser.setup();
  while (1) {
    coleur[0].red = (coleur[0].red + 5) % 256;
    coleur[1].green = (coleur[1].green + 5) % 256;
    coleur[2].blue = (coleur[2].blue + 5) % 256;

    neopixelSetMoreLeds(0, coleur, 3);
    HAL_Delay(50);

    if (noSpam_1on100 >= 100) {
      noSpam_1on100 = 0;
      pami_position_t pos;
      pos = motorGetPosition();
      uartprintf(">angle:%f\n", pos.theta * 360.0 / M_PI);
      uartprintf(">pos:%f:%f|xy\n", pos.x, pos.y);
      uartprintf(">lidar:%f:%f|xy\n", pos.x, pos.y);

      uartprintf(">capteur:%d\n", CapteurGetValue());
      uartprintf("BAGARRE : le capteur est %d\n", CapteurGetValue());
    }

    pami_position_t pos;
    pos = motorGetPosition();
    laser.scanDataNonBlocking();
    if (laser.newDataAvailable()) {
      uartprintf(">lidar:%f:%f|xy|clear\n", pos.x, pos.y);
      for (int i = 0; i < GS_MAX_SAMPLE; i++) {
        if (laser.samples[i].intensity > 4) {
          float ang = (laser.samples[i].angle + pos.theta);
          uartprintf(">lidar:%f:%f|xy\n",
                     pos.x + (laser.samples[i].distance * cos(ang)),
                     pos.y + (laser.samples[i].distance * sin(ang)));

          uartprintf(">capteur:%d\n", CapteurGetValue());
          uartprintf("BAGARRE : le capteur est %d\n", CapteurGetValue());
        }
      }
    } else {
      noSpam_1on100++;
    }

    /*/
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

          if (oneTimeOnTen == 5)
          {
            uartprintf("VALEUR GS2 YDLIDAR : \n");
            for (int i = 0 ; i < GS_MAX_SAMPLE; i++ ){
              uartprintf("[main] GS_SAMPLE[%d] : angle %f / distance %f /
       instensity %f \n",i,laser.samples[i].angle*360/(2*M_PI)
       ,laser.samples[i].distance,laser.samples[i].intensity);
            }
          }
        }
          */

    // Changer la position des servos
    servoWrite(SERVO1_CHAN, 1300);
    servoWrite(SERVO2_CHAN, 2000);

    HAL_Delay(100);
    level_battery = battGetRawValue();

    if (ButtonPcbGetValue() == GPIO_PIN_RESET) {
      uartprintf("[main] level batt : %f\n", battGetPourcentage());
      motorEnable();

      // motorRotate(MOTOR_DIR_ANTICLOCKWISE, 180, 90, 180, 180);
      // motorMove(MOTOR_DIR_FORWARD, 300, 300, 100, 100);
      motorRotate(MOTOR_DIR_CLOCKWISE, 90, 90, 180, 180);
      motorMove(MOTOR_DIR_FORWARD, 300, 300, 100, 100);

      while (!motorIsReady()) {

        laser.scanDataNonBlocking();
        if (noSpam_1on100 >= 100) {
          noSpam_1on100 = 0;
          pami_position_t pos;
          pos = motorGetPosition();
          uartprintf(">angle:%f\n", pos.theta * 360.0 / (2 * M_PI));
          uartprintf(">pos:%f:%f|xy\n", pos.x, pos.y);
          if (laser.newDataAvailable()) {
            for (int i = 0; i < GS_MAX_SAMPLE; i++) {
              if (laser.samples[i].intensity > 4) {
                float ang = (laser.samples[i].angle + pos.theta);
                uartprintf(">lidar:%f:%f|xy\n",
                           pos.x + (laser.samples[i].distance * cos(ang)),
                           pos.y + (laser.samples[i].distance * sin(ang)));
              }
            }
          }
        } else {
          noSpam_1on100++;
        }
      }

      motorDisable();
      for (int i = 0; i < 1; i++) {
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
void Flash_Write(uint8_t value) {
  HAL_FLASH_Unlock();

  // Effacer la page (obligatoire avant écriture)
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page = (FLASH_STORAGE_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
  EraseInitStruct.NbPages = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
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
uint8_t Flash_Read(void) { return *((uint8_t *)FLASH_STORAGE_ADDR); }

uint8_t preSavedDeplacement(uint8_t numPamis, team_t team) {

  uint8_t rotation =
      (team == team_yellow ? MOTOR_DIR_CLOCKWISE : MOTOR_DIR_ANTICLOCKWISE);
  float pointrotation = 80.0;
  switch (numPamis) {
  case 0:
    // superstar

    motorMove(MOTOR_DIR_FORWARD, 100, 75, 30, 100);  // démarafe
    motorMove(MOTOR_DIR_FORWARD, 400, 300, 100, 100); // avant rampe
    motorMove(MOTOR_DIR_FORWARD, 650, 200, 50, 100);  // rampe
    //motorMove(MOTOR_DIR_FORWARD, 200, 300, 10, 100);  // scéne

    if (team_blue == team) {
      pointrotation = -80.0;
    }
    // motorTurn(MOTOR_DIR_FORWARD, 90.0, pointrotation, 300);
    motorRotate(rotation, 90, 150);
    motorMove(MOTOR_DIR_FORWARD, 150, 300, 0, 300);
    break;
  case 1:

    motorMove(MOTOR_DIR_FORWARD, 100, 100, 50, 100); // démarafe
    motorMove(MOTOR_DIR_FORWARD, 200, 300, 50, 50);
    motorRotate(rotation, 35, 90);
    motorMove(MOTOR_DIR_FORWARD, 750, 300, 50, 50);
    motorRotate(!rotation, 85, 90);
    motorMove(MOTOR_DIR_FORWARD,50,200,50,50);
    break;
  case 2:
    motorMove(MOTOR_DIR_FORWARD, 250, 150, 500, 500);
    motorRotate(rotation, 35, 45, 50, 50);
    motorMove(MOTOR_DIR_FORWARD, 700, 580, 300, 300);
    motorRotate(!rotation, 35, 45, 50, 50);
    motorMove(MOTOR_DIR_FORWARD, 300, 350, 300, 300);
    motorRotate(!rotation, 90, 45, 50, 50);
    motorMove(MOTOR_DIR_FORWARD, 50, 150, 300, 300);
    break;
  case 3:
    motorMove(MOTOR_DIR_FORWARD, 200, 150, 100, 0);
    motorRotate(rotation, 35, 45);
    motorMove(MOTOR_DIR_FORWARD, 800, 180);
    motorRotate(!rotation, 35, 45);
    motorMove(MOTOR_DIR_FORWARD, 1000, 150);
    motorRotate(!rotation, 115, 45);
    motorMove(MOTOR_DIR_FORWARD, 80, 180, 0, 100);
    break;

  default:
    break;
  }
}
