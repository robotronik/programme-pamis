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
    gs_device_para info;
    laser.stopScan();
    laser.getDevicePara(info);
    laser.startScan(false);
    delay_ms(1000);
    laser.doProcessSimple();
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
    // //<! fixed angle resolution
    // laser.setFixedResolution(false);
    // //<! rotate 180
    // laser.setReversion(false); //rotate 180
    // //<! Counterclockwise
    // laser.setInverted(false);//ccw

    // //<! tof lidar
    // laser.setLidarType(TYPE_TRIANGLE);
    // //unit: °
    // laser.setMaxAngle(180);
    // laser.setMinAngle(-180);

    // //unit: m
    // laser.setMinRange(30);
    // laser.setMaxRange(1000);

    // //unit: Hz
    // laser.setScanFrequency(8.0);
    // std::vector<float> ignore_array;
    // ignore_array.clear();
    // laser.setIgnoreArray(ignore_array);
    // bool isIntensity = true;
    // laser.setIntensity(isIntensity);

    // bool ret = laser.initialize();

    // if (ret) {
    //     ret = laser.turnOn();
    // }

    // result_t isOk = RESULT_OK;
    // while (ret && ydlidar::ok())
    // {
    //     bool hardError;
    //     LaserScan scan;
    //     uint8_t *header = reinterpret_cast<uint8_t *>(&scan);
    //     size_t size = sizeof(scan);

    //     if (laser.doProcessSimple(scan, hardError))
    //     {
    //         fprintf(stdout, "Scan received[%lu]: %u ranges is [%f]Hz\n",
    //                 scan.stamp,
    //                 (unsigned int)scan.points.size(),
    //                 1.0 / scan.config.scan_time);
    //         fflush(stdout);

    //         for (size_t i=0; i<scan.points.size(); ++i)
    //         {
    //             printf("%d %.02f %.02f\n", i, scan.points.at(i).angle * 180.0 / M_PI, scan.points.at(i).range);
    //         }
    //         printf("\n");

    //         if (haveRecvPort)
    //         {
    //             isOk = _serial->writeData(header,size);
    //             if(!IS_OK(isOk)){
    //                 fprintf(stderr, "Failed to send data to the receiving port\n");
    //                 fflush(stderr);
    //             }
    //         }
    //     } else {
    //         fprintf(stderr, "Failed to get Lidar Data\n");
    //         fflush(stderr);
    //     }
    // }

    // laser.turnOff();
    // laser.disconnecting();




	return 0;
}

