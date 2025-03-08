#include "config.h"
#include "uart.h"
#include "uartLidar.h"
#include "clock.h"
#include "sequence.h"
#include "CYdLidar.h"



int main(void)
{
    CYdLidar laser;

	//SETUP
	clock_setup();
	usartSetup();
    usart1Setup();//set up befor lidar
    laser.setup();



	//WAIT
    delay_ms(1000);
    usartprintf("Start\n");



    // //NONblocking
    // while (1){
    //     laser.scanDataNonBlocking();
    //     if(laser.newDataAvailable()){
    //         laser.printLidarPoints();
    //     }
    // }

    //NONblocking
    while (1){
        laser.scanDataNonBlocking();
        if(laser.newDataAvailable()){
            for(int i = 0; i < GS_MAX_SAMPLE; i++){
                usartprintf("angle : %lf distance : %lf intensity : %lf\n",laser.samples[i].angle,laser.samples[i].distance,laser.samples[i].intensity);
            }
            usartprintf("\n\n");
            delay_ms(1000);
        }
    }

    //blocking
    while (1){
        laser.scanData();
        laser.printLidarPoints();
        delay_ms(1000);
    }


	return 0;
}

