
#include "uartLidar.h"
#include <stdio.h>
#include <stdarg.h>

void _uart1Clock_setup(void);
void gpio1_setup(void);

#define FIFO_SIZE 64
typedef struct {
    uint8_t buffer[FIFO_SIZE];
    uint8_t head;
    uint8_t tail;
} fifo_t;

volatile fifo_t uart_fifo;

int fifo_full(volatile fifo_t *fifo) {
    uint8_t next = (fifo->head + 1) % FIFO_SIZE;
    return next == fifo->tail;
}


void fifo_push(volatile fifo_t *fifo, uint8_t data) {
    if (!fifo_full(fifo)) {
        fifo->buffer[fifo->head] = data;
        fifo->head = (fifo->head + 1) % FIFO_SIZE;
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

void usart1_isr(void) {
    if (usart_get_flag(USART1, USART_SR_RXNE)) {
        uint8_t data = usart_recv(USART1);
        fifo_push(&uart_fifo, data);
    }
}



void usart1Setup(void){

    uart_fifo.head = 0;
    uart_fifo.tail = 0;

    _uart1Clock_setup();
	gpio1_setup();

	/* Setup USART1 parameters. */
	usart_set_baudrate(USART1, 921600);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    //interupt
    usart_enable_rx_interrupt(USART1);
    nvic_enable_irq(NVIC_USART1_IRQ);

	/* Finally enable the USART. */
	usart_enable(USART1);

}

void usartSend1Message(uint32_t usart, char* Message){
	int i = 0;
	while (Message[i] != 0)	{
		usart_send_blocking(usart,Message[i]);
		i++;
	}
}

void usartSend1Data(const uint8_t *data, int size) {
    for(int i = 0; i < size; i++){
        usart_send_blocking(USART1,data[i]);
    }
}

void usart1flushSerial(void){
    uint8_t data;
    while(usart1recev(&data));
}

bool usart1recev(uint8_t* data){
    if(!fifo_empty(&uart_fifo)){
        *data = fifo_pop(&uart_fifo);
        return true;
    }
    return false;
}

void usart1printf(const char* format, ...) {
    char buffer[1000];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, 1000, format, args);
    usartSend1Message(USART1,buffer);
    va_end(args);
}


void _uart1Clock_setup(void){
	/* Enable GPIOD clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART1. */
	rcc_periph_clock_enable(RCC_USART1);
}

void gpio1_setup(void)
{

	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);


	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO10);
}