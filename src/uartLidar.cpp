#include "uartLidar.h"
#include <stdarg.h>
#include <string.h>

#define UART_TX_PIN GPIO_PIN_6
#define UART_TX_PORT GPIOB
#define UART_RX_PIN GPIO_PIN_7
#define UART_RX_PORT GPIOB

UART_HandleTypeDef huart1;
uint8_t UART1_rxBuffer[12] = {0};





#define FIFO_SIZE (338 *10 + 1)
typedef struct {
    uint8_t buffer[FIFO_SIZE];
    uint32_t head;
    uint32_t tail;
} fifo_t;

volatile fifo_t uart_fifo;
volatile bool fifo_full_flag;

int fifo_full(volatile fifo_t *fifo) {
    uint32_t next = (fifo->head + 2) % FIFO_SIZE;
    return next == fifo->tail;
}


void fifo_push(volatile fifo_t *fifo, uint8_t data) {
    if (!fifo_full(fifo)) {
        fifo->buffer[fifo->head] = data;
        fifo->head = (fifo->head + 1) % FIFO_SIZE;
    }
    else{
        fifo_full_flag = true;
    }
}

int fifo_empty(volatile fifo_t *fifo) {
    return fifo->head == fifo->tail;
}

uint8_t fifo_pop(volatile fifo_t *fifo) {
    if (fifo->head == fifo->tail) {
        return 0;
    }
    uint8_t data = fifo->buffer[fifo->tail];
    fifo->tail = (fifo->tail + 1) % FIFO_SIZE;
    return data;
}

uint32_t getFifoSize() {
    if (uart_fifo.head >= uart_fifo.tail) {
        return uart_fifo.head - uart_fifo.tail;
    } else {
        return (FIFO_SIZE - uart_fifo.tail) + uart_fifo.head;
    }
}


void usart1flushSerial(void){
    uart_fifo.head = 0;
    uart_fifo.tail = 0;
    fifo_full_flag = false;
}

bool usart1recev(uint8_t* data){
    if(!fifo_empty(&uart_fifo)){
        *data = fifo_pop(&uart_fifo);
        return true;
    }
    return false;
}

bool usart1Error(){
    return fifo_full_flag;
}

void usartSend1Data(const uint8_t *data, int size) {
    HAL_UART_Transmit(&huart1, data, size, UART_TIMEOUT);
}



void uartLidarSetup(void)
{

    uart_fifo.head = 0;
    uart_fifo.tail = 0;
    fifo_full_flag = false;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
    if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
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
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t data;
    HAL_UART_Receive_IT(&huart1, &data, 1);
    fifo_push(&uart_fifo, data);
}

