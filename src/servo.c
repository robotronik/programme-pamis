#include "servo.h"

TIM_HandleTypeDef timer_servo;

static void servoGPIOInit(TIM_HandleTypeDef *htim);

void servoSetup(void)
{

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    timer_servo.Instance = SERVO_TIMER;
    timer_servo.Init.Prescaler = 84;
    timer_servo.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer_servo.Init.Period = 20000;
    timer_servo.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer_servo.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&timer_servo) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&timer_servo, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&timer_servo) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timer_servo, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&timer_servo, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&timer_servo, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&timer_servo, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    // HAL_TIM_MspPostInit(&timer_servo);
    servoGPIOInit(&timer_servo);
}

static void servoGPIOInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (htim->Instance == TIM3)
    {

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM3 GPIO Configuration
        PB0     ------> TIM3_CH3
        PB4     ------> TIM3_CH1
        PB5     ------> TIM3_CH2
        */
        GPIO_InitStruct.Pin = SERVO3_Pin | SERVO1_Pin | SERVO2_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void servoEnable(uint32_t numServo)
{
    HAL_TIM_PWM_Start(&timer_servo, numServo);
}
void servoDisable(uint32_t numServo)
{

    HAL_TIM_PWM_Stop(&timer_servo, numServo);
}

void servoWrite(uint8_t numServo, uint16_t period)
{
    if (period > SERVO_MAX_PERIOD)
        period = SERVO_MAX_PERIOD;
    if (period < SERVO_MIN_PERIOD)
        period = SERVO_MIN_PERIOD;
    __HAL_TIM_SET_COMPARE(&timer_servo, numServo, period);
}

void servoSetAngle(uint8_t numServo, uint16_t angle)
{
}
