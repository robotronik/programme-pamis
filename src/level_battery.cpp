#include "level_battery.h"
#include "stm32g4xx_hal.h"
#include "system.h"

ADC_HandleTypeDef hbatt;

#define IMAGE_BATT_Pin GPIO_PIN_6
#define IMAGE_BATT_GPIO_Port GPIOA

uint8_t battGpioSetup(void);
uint8_t battAdcSetup(void);

void battSetup(void)
{
    if (battGpioSetup())
    {
        Error_Handler();
    }
    if (battAdcSetup())
    {
        Error_Handler();
    }
    HAL_ADC_Start(&hbatt);
}

uint8_t battAdcSetup(void)
{

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        return 1;
    }

    __HAL_RCC_ADC12_CLK_ENABLE();

    ADC_ChannelConfTypeDef sConfig = {0};
    hbatt.Instance = ADC2;
    hbatt.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
    hbatt.Init.Resolution = ADC_RESOLUTION_12B;
    hbatt.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hbatt.Init.GainCompensation = 0;
    hbatt.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hbatt.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hbatt.Init.LowPowerAutoWait = DISABLE;
    hbatt.Init.ContinuousConvMode = ENABLE;
    hbatt.Init.NbrOfConversion = 1;
    hbatt.Init.DiscontinuousConvMode = DISABLE;
    hbatt.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hbatt.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hbatt.Init.DMAContinuousRequests = DISABLE;
    hbatt.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hbatt.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hbatt) != HAL_OK)
    {
        return 1;
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hbatt, &sConfig) != HAL_OK)
    {
        return 1;
    }

    return 0;
}

uint8_t battGpioSetup(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // ADC2 GPIO Configuration
    // PA6     ------> ADC2_IN3
    GPIO_InitStruct.Pin = IMAGE_BATT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(IMAGE_BATT_GPIO_Port, &GPIO_InitStruct);

    return 0;
}

uint32_t battGetRawValue(void)
{

    return HAL_ADC_GetValue(&hbatt);
}
float battGetPourcentage(void)
{

    return ((battGetVoltage() - (float)BATT_TENSION_MIN) / (BATT_TENSION_MAX - BATT_TENSION_MIN)) * 100.0;
}

float battGetVoltage(void)
{

    return battGetRawValue() * (3.3 / 4096.0) * BATT_PDT_RATIO;
}
