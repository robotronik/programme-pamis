#include "uart.h"
#include <stdarg.h>
#include <string.h>
#include <cstdio>

#define UART_PIN GPIO_PIN_3
#define UART_PORT GPIOB
#define UART_GPIO_RCC_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

UART_HandleTypeDef huart2;

void uartSetup(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Activer l'horloge de GPIOB
    UART_GPIO_RCC_CLK_ENABLE();

    // Configurer la pin en mode Alternate Function pour USART2_TX
    GPIO_InitStruct.Pin = UART_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Mode Alternate Function Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2; // AF7 pour USART2_TX
    HAL_GPIO_Init(UART_PORT, &GPIO_InitStruct);

    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

void uartTransmit(char *msg, size_t size)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, size, (uint32_t)UART_TIMEOUT);
}

void uartprintf(const char *format, ...)
{
    char buffer[UART_MAX_MSG];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, UART_MAX_MSG, format, args);
    size_t size = strlen(buffer);
    uartTransmit(buffer, size);
    va_end(args);
}
