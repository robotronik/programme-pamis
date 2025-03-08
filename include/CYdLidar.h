#pragma once
#include <stdint.h>
#include "uartLidar.h"
#include "uart.h"
#include "ydlidar_protocol.h"
#include "angles.h"
#include "clock.h"

//print param
#define HEIGHT 100
#define WIDTH (HEIGHT*2)
#define MAX_RANGE 500

#define __FUNCTION_NAME__ __func__
#define checkHead(...) debug_checkHead(__FUNCTION_NAME__, __FILE__, __LINE__, __VA_ARGS__)

class CYdLidar
{
private:
    /* data */
    double  d_compensateK0;
    double  d_compensateK1;
    double  d_compensateB0;
    double  d_compensateB1;
    uint16_t  u_compensateK0;
    uint16_t  u_compensateK1;
    uint16_t  u_compensateB0;
    uint16_t  u_compensateB1;
    double  bias;

    static gs2_node_package package;

    bool    m_intensities = true;
    float   m_AngleOffset = 0;
    bool    m_Reversion = false;
    bool    m_Inverted = false;
    float   m_MaxRange = 1000;
    float   m_MinRange = 30;
    uint8_t m_address;

    //attribut for non blocking methode
    uint16_t m_recvPos = 0;
    bool    m_newDataAvailable = false;

public:
    static final_Node samples[GS_MAX_SAMPLE];


public:
    CYdLidar(/* args */);

//Communication
private:
    void sendCommand(uint8_t cmd, const void *payload = NULL, int payloadsize = 0);
    bool waitResponseHeader(gs_lidar_header *header);
    bool waitResponseHeaderNonBlocking(gs_lidar_header *header);

//Control
public:
    bool setup();
    bool startScan();
    bool getDeviceAddress();
    bool stopScan();
    bool getDevicePara();
    bool scanData();
    bool scanDataNonBlocking();
    bool newDataAvailable();

//Process
private:
    void angTransform(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist);
    bool processData(void);
    bool isRangeValid(double reading) const;
    bool debug_checkHead(const char* functionName, const char* functionFile, const int functionLine ,gs_lidar_header* head, uint8_t flag, uint16_t size = 0);

//Print
public:
    bool printbuffer(void);
    void printLidarPoints(void);
    void printPara(void);

    ~CYdLidar();
};

