
#include "uartLidar.h"
#include <stdio.h>
#include <stdarg.h>

void _uart1Clock_setup(void);
void gpio1_setup(void);

void usart1Setup(void){

    _uart1Clock_setup();
	gpio1_setup();

	/* Setup USART1 parameters. */
	usart_set_baudrate(USART1, 921600);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

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

bool usart1recev(uint16_t* data){
    if((USART_SR(USART1) & USART_SR_RXNE) != 0){
        *data = usart_recv(USART1);
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