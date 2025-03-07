#include "config.h"
#include "uart.h"
#include "uartLidar.h"
#include "clock.h"
#include "sequence.h"
#include "CYdLidar.h"



int main(void)
{

	//SETUP
	clock_setup();
	usartSetup();
    usart1Setup();



	//WAIT
    delay_ms(3000);
    usartprintf("Start\n");

    uint8_t data;
    sequence sendMessage;



    CYdLidar laser;
    laser.startScan();
    delay_ms(1000);
    while (1){
        laser.scanData();
        laser.printLidarPoints();
        delay_ms(1000);
    }

    laser.scanData();
    laser.printbuffer();
    laser.printLidarPoints();

    while (1){
    }

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

