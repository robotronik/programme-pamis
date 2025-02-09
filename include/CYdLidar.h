#pragma once
#include <stdint.h>
#include "uartLidar.h"
#include "uart.h"
#include "ydlidar_protocol.h"

#define size_t uint16_t

#define RESULT_FAIL false
#define RESULT_OK   true

class CYdLidar
{
private:
    /* data */
    bool has_device_header;
    uint8_t last_device_byte;

    double  d_compensateK0;
    double  d_compensateK1;
    double  d_compensateB0;
    double  d_compensateB1;
    uint16_t  u_compensateK0;
    uint16_t  u_compensateK1;
    uint16_t  u_compensateB0;
    uint16_t  u_compensateB1;
    double  bias;
    bool m_SingleChannel = false;
public:
    CYdLidar(/* args */);


    bool sendCommand(uint8_t cmd, const void *payload = NULL, size_t payloadsize = 0);
    bool sendCommand(uint8_t addr, uint8_t cmd, const void *payload = NULL, size_t payloadsize = 0);
    bool startScan(bool force);
    bool setDeviceAddress(uint32_t timeout);
    bool stop();
    bool stopScan();
    bool getDevicePara(gs_device_para &info);
    bool waitResponseHeader(gs_lidar_ans_header *header);

    ~CYdLidar();
};

