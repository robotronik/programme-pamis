#include "uartLidar.h"
#include <stdarg.h>
#include <string.h>
#include "uart.h"

#define UART_TX_PIN GPIO_PIN_6
#define UART_TX_PORT GPIOB
#define UART_RX_PIN GPIO_PIN_7
#define UART_RX_PORT GPIOB

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;


#define DMA_BUF_SIZE (338 *10 + 1)
uint8_t dma_rx_buf[DMA_BUF_SIZE];
volatile uint16_t dma_rx_tail = 0; // index de lecture logiciel



uint32_t getFifoSize(void)
{
    uint16_t dma_head = DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    if (dma_head >= dma_rx_tail) {
        return dma_head - dma_rx_tail;
    } else {
        return (DMA_BUF_SIZE - dma_rx_tail) + dma_head;
    }
}

void usart1flushSerial(void)
{
    dma_rx_tail = DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
}

bool usart1recev(uint8_t* data)
{
    uint16_t dma_head = DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    if (dma_head == dma_rx_tail){
        return false; // buffer vide
    }

    *data = dma_rx_buf[dma_rx_tail];
    dma_rx_tail = (dma_rx_tail + 1) % DMA_BUF_SIZE;
    return true;
}

void usartSend1Data(const uint8_t *data, int size) {
    HAL_UART_Transmit(&huart1, data, size, UART_TIMEOUT);
}


void uartLidarSetup()
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    // Configuration de la clock USART
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        Error_Handler();

    // GPIO config (TX/RX)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    // Initialisation UART
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 921600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
    {
        Error_Handler();
    }

     // DMA config
    hdma_usart1_rx.Instance = DMA2_Channel1;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
        Error_Handler();

    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    // Lancer la réception DMA circulaire
    if (HAL_UART_Receive_DMA(&huart1, dma_rx_buf, DMA_BUF_SIZE) != HAL_OK){
        Error_Handler();
    }

    dma_rx_tail = 0;
}



