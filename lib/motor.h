#ifndef MOTOR_H
#define MOTOR_H

#include "stm32g4xx_hal.h"

/*****************
 *  Rixae Dufour - CDFR 2025 PAMIS
 *  ROBOTRONIK - mars 2025
 *
 *  gestion des moteur pas à pas
 *
 *******************/

 #ifndef PI
 #define PI 3.141592
 #endif

#define MOTOR_Port                        GPIOA
#define MOTOR_STEP1_Pin                   GPIO_PIN_0
#define MOTOR_STEP2_Pin                   GPIO_PIN_1
#define MOTOR_DIR1_Pin                    GPIO_PIN_2
#define MOTOR_DIR2_Pin                    GPIO_PIN_3
#define MOTOR_ENABLE_Pin                  GPIO_PIN_4

#define DIAM_ROUE                   60.0
#define DIAM_INTER_ROUE             87.0
#define PAS_PAR_TOUR                400 // prendre en compte le microstepping
#define MOTOR_CLOCK_TIMER           84 //MHz 
#define MOTOR_TIMER_PRESCALER       840

#define MOTOR_DIR_FORWARD           1
#define MOTOR_DIR_BACKWARD          0
#define MOTOR_DIR_CLOCKWISE         1
#define MOTOR_DIR_ANTICLOCKWISE     0

void motorSetup(void);

void motorEnable(void);
void motorDisable(void);

void motorMove(int direction, float distance, float vitesse); // en mm & mm/s
void motorRotate(int sens_horaire, float angle, float vitesse);  // en degres et degres par seconde

int motorIsReady(void);


void motorStepper1Fallback (void); 
void motorStepper2Fallback (void); 

#endif