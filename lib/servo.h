#ifndef SERVO_H
#define SERVO_H

/*****************
 *  Rixae Dufour - CDFR 2025 PAMIS
 *  ROBOTRONIK - mars 2025
 *
 *  Il y a 3 servo (PB4,PB5,PB0)
 *
 *******************/

#include "stm32g4xx_hal.h"

#define SERVO_MIN_PERIOD 1000
#define SERVO_MAX_PERIOD 2000

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

#define SERVO_TIMER TIM3
#define SERVO_TIM_PWM

#define SERVO1_CHAN TIM_CHANNEL_1
#define SERVO2_CHAN TIM_CHANNEL_2
#define SERVO3_CHAN TIM_CHANNEL_3

#define SERVO1 SERVO1_CHAN
#define SERVO2 SERVO2_CHAN
#define SERVO3 SERVO3_CHAN

#define SERVO1_Pin GPIO_PIN_4
#define SERVO1_GPIO_Port GPIOB
#define SERVO2_Pin GPIO_PIN_5
#define SERVO2_GPIO_Port GPIOB
#define SERVO3_Pin GPIO_PIN_0
#define SERVO3_GPIO_Port GPIOB

void servoSetup(void);

void servoEnable(uint32_t numServo);
void servoDisable(uint32_t numServo);

void servoWrite(uint8_t numServo, uint16_t period);
void servoSetAngle(uint8_t numServo, uint16_t angle);

#endif