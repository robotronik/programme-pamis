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

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c2;

// TIM_HandleTypeDef htim3;

// UART_HandleTypeDef huart2;
/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC2_Init(void);

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

  MX_GPIO_Init();
  HAL_GPIO_WritePin(EN_STEPPER_GPIO_Port, EN_STEPPER_Pin, (GPIO_PinState)1);
  // MX_I2C2_Init();
  // MX_ADC2_Init();
  //  MX_USART2_UART_Init();
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
  color_t coleur[3] = {{255, 0, 127}, {0, 55, 0}, {127, 0, 127}};
  // neopixelSetLed(0, coleur, 255);
  uint32_t level_battery = 1240;

  int oneTimeOnTen = 0;



  // LOOP
  while (1)
  {
    laser.scanDataNonBlocking();
    if(laser.newDataAvailable()){
        if(oneTimeOnTen == 10){
            laser.printLidarPoints();
            oneTimeOnTen = 0;
        }
        else
            oneTimeOnTen ++;
    }
    // if(laser.newDataAvailable()){
    //     for(int i = 0; i < GS_MAX_SAMPLE; i++){
    //         uartprintf("angle : %lf distance : %lf intensity : %lf\n", laser.samples[i].angle  * 180.0 / M_PI, laser.samples[i].distance, laser.samples[i].intensity);
    //     }
    // }

    coleur[0].red = (coleur[0].red + 5) % 256;
    coleur[1].green = (coleur[1].green + 5) % 256;
    coleur[2].blue = (coleur[2].blue + 5) % 256;
    // neopixelSetLed(0, coleur[0], 255);
    // neopixelSetLed(1, coleur, 255);
    // neopixelSetLed(2, coleur, 255);

    neopixelSetMoreLeds(0, coleur, 3);
    // NVIC_SetPendingIRQ(DMA1_Channel1_IRQn);
    //   Allumer la LED
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    // Mettre les servos à une position
    // servoWrite(SERVO1_CHAN, 2000);
    // servoWrite(SERVO2_CHAN, 1500);
    // servoWrite(SERVO3_CHAN, 1500);

    HAL_Delay(50);

    // Éteindre la LED
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    // Changer la position des servos
    servoWrite(SERVO1_CHAN, 1500);
    servoWrite(SERVO2_CHAN, 2000);

    HAL_Delay(50);
    // level_battery  = HAL_ADC_GetValue(&hadc2);
    uartprintf("level : %u :3\n", level_battery);

    if (0 == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10))
    {
      uartprintf("Petit tour de soi-même pour montrer sa robe UwU\n");

      HAL_GPIO_WritePin(EN_STEPPER_GPIO_Port, EN_STEPPER_Pin, (GPIO_PinState)0);

      HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, (GPIO_PinState)1);
      HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, (GPIO_PinState)0);

      for (int i = 0; i < 1200; i++)
      {

        HAL_GPIO_TogglePin(STEP1_GPIO_Port, STEP1_Pin);
        HAL_GPIO_TogglePin(STEP2_GPIO_Port, STEP2_Pin);
        HAL_Delay(5);
      }
      HAL_GPIO_WritePin(EN_STEPPER_GPIO_Port, EN_STEPPER_Pin, (GPIO_PinState)1);

      for (int i = 0; i < 3; i++)
      {
        servoWrite(SERVO1_CHAN, 2000);
        servoWrite(SERVO2_CHAN, 1500);

        HAL_Delay(2000);

        // Changer la position des servos
        servoWrite(SERVO1_CHAN, 1500);
        servoWrite(SERVO2_CHAN, 2000);
        HAL_Delay(2000);
      }
    }
  }
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
   */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0xC0100D10;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STEP1_Pin | STEP2_Pin | DIR1_Pin | DIR2_Pin | EN_STEPPER_Pin | LED_USER_Pin | ELECTROMAGNET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEP1_Pin STEP2_Pin DIR1_Pin DIR2_Pin
                           EN_STEPPER_Pin LED_USER_Pin ELECTROMAGNET_Pin */
  GPIO_InitStruct.Pin = STEP1_Pin | STEP2_Pin | DIR1_Pin | DIR2_Pin | EN_STEPPER_Pin | LED_USER_Pin | ELECTROMAGNET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_USER_Pin SW_TEAM_Pin */
  GPIO_InitStruct.Pin = SW_USER_Pin | SW_TEAM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_TIRETTE_Pin */
  GPIO_InitStruct.Pin = SW_TIRETTE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_TIRETTE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
