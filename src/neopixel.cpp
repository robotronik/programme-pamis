#include "neopixel.h"
#include "stm32g4xx_hal.h"
#include "uart.h"

#define NEOPIXEL_RESET_TIMER_TICK 64
#define PWM_HI 70
#define PWM_LOW 35

#define NB_BYTES ((NEOPIXEL_NB + 1) * 24 + NEOPIXEL_RESET_TIMER_TICK)

TIM_HandleTypeDef htimneopixel;
DMA_HandleTypeDef hdma_timneopixel_ch1;

volatile uint8_t datasentflag;

uint32_t data[NB_BYTES];

void neopixelSetup(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    htimneopixel.Instance = TIM2;
    htimneopixel.Init.Prescaler = 0;
    htimneopixel.Init.CounterMode = TIM_COUNTERMODE_UP;
    htimneopixel.Init.Period = 104; // Fréquence WS2812 (~800kHz)
    htimneopixel.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htimneopixel.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htimneopixel) != HAL_OK)
        Error_Handler();

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_Init(&htimneopixel) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htimneopixel, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_timneopixel_ch1.Instance = DMA1_Channel1;
    hdma_timneopixel_ch1.Init.Request = DMA_REQUEST_TIM2_CH1;
    hdma_timneopixel_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_timneopixel_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_timneopixel_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_timneopixel_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_timneopixel_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_timneopixel_ch1.Init.Mode = DMA_NORMAL;
    hdma_timneopixel_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_timneopixel_ch1) != HAL_OK)
        Error_Handler();
    __HAL_LINKDMA(&htimneopixel, hdma[TIM_DMA_ID_CC1], hdma_timneopixel_ch1);

    __HAL_DMA_ENABLE(&hdma_timneopixel_ch1);
    __HAL_TIM_ENABLE_DMA(&htimneopixel, TIM_DMA_CC1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    HAL_TIM_PWM_Start(&htimneopixel, TIM_CHANNEL_1);

    HAL_TIM_PWM_Stop_DMA(&htimneopixel, TIM_CHANNEL_1);
    HAL_DMA_Abort(&hdma_timneopixel_ch1);

    datasentflag = 1;
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM2)
    {
        datasentflag = 1;
        HAL_TIM_PWM_Stop_DMA(&htimneopixel, TIM_CHANNEL_1);
    }
}

extern "C" void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_timneopixel_ch1);
}

static void _sendNeopixel(void)
{
    for (int i = (NEOPIXEL_NB + 1) * 24; i < (NEOPIXEL_NB + 1) * 24 + NEOPIXEL_RESET_TIMER_TICK; i++)
    {
        data[i] = 0;
    }
    uint32_t timeout = HAL_GetTick();
    while (!datasentflag)
    {
        if (HAL_GetTick() - timeout > 100) // Timeout de 100 ms
        {
            Error_Handler();
            break;
        }
    }
    datasentflag = 0;

    if (htimneopixel.State != HAL_TIM_STATE_READY)
    {
        uartprintf("TIM2 n'est pas prêt !\n");
        Error_Handler();
    }
    if (hdma_timneopixel_ch1.State != HAL_DMA_STATE_READY)
    {
        uartprintf("DMA1_Channel1 n'est pas prêt !\n");
        Error_Handler();
    }
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start_DMA(&htimneopixel, TIM_CHANNEL_1, data, NB_BYTES);

    if (status != HAL_OK)
    {
        if (status == HAL_ERROR)
            uartprintf("Erreur: HAL_TIM_PWM_Start_DMA() a retourné HAL_ERROR\n");
        else if (status == HAL_BUSY)
            uartprintf("Erreur: HAL_TIM_PWM_Start_DMA() a retourné HAL_BUSY\n");
        else if (status == HAL_TIMEOUT)
            uartprintf("Erreur: HAL_TIM_PWM_Start_DMA() a retourné HAL_TIMEOUT\n");

        Error_Handler();
    }
}

void neopixelClear(void)
{
    for (int i = 0; i < (NEOPIXEL_NB + 1) * 24; i++)
    {
        data[i] = PWM_LOW;
    }
    _sendNeopixel();
}

void neopixelSetLed(uint16_t numLed, color_t color, uint8_t brightness)
{
    color.red = color.red * (float)brightness / 255;
    color.green = color.green * (float)brightness / 255;
    color.blue = color.blue * (float)brightness / 255;
    neopixelSetMoreLeds(numLed, &color, 1);
}

void neopixelSetMoreLeds(uint16_t firstled, color_t color[], size_t size)
{
    int32_t indx = firstled * 24;
    for (uint16_t j = 0; j < size; j++)
    {
        uint32_t col = color[j].green << 16 | color[j].red << 8 | color[j].blue;
        for (int i = 23; i >= 0; i--)
        {
            if (col & (1 << i))
            {
                data[indx] = PWM_HI;
            }

            else
            {
                data[indx] = PWM_LOW;
            }
            indx++;
        }
    }
    _sendNeopixel();
}