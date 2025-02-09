#include "config.h"
#include "uart.h"
#include "uartLidar.h"
#include "clock.h"
#include "sequence.h"



int main(void)
{

	//SETUP
	clock_setup();
	usartSetup();
    usart1Setup();



	//WAIT
	delay_ms(3000);
	usartprintf("Start\n");

    uint16_t data;
    sequence sendMessage;

	while (1){
        sendMessage.interval([](){
            usart1printf("ok\n");
        },1000);
        if(usart1recev(&data)){
            usartprintf("%c",data);
        }
	}

	return 0;
}

