#pragma once
#include <stdint.h>
#include "uartLidar.h"
#include "uart.h"
#include "ydlidar_protocol.h"
#include "angles.h"
#include "clock.h"

#define size_t uint16_t

#define RESULT_FAIL false
#define RESULT_OK   true

#define MAX_SCAN_NODES 160
#define HEIGHT 100
#define WIDTH (HEIGHT*2)
#define MAX_RANGE 500

class CYdLidar
{
private:
    /* data */
    bool has_device_header;

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

    static node_info scan_node_buf[MAX_SCAN_NODES];
    static final_Node final_node_buf[MAX_SCAN_NODES];

    gs2_node_package package;
    uint16_t package_Sample_Index;
    uint8_t CheckSumCal;
    uint8_t CheckSum;
    int package_index;
    uint8_t scan_frequence;
    bool m_intensities;
    float   m_AngleOffset;
    bool m_Reversion = false;
    bool m_Inverted = false;//
    float m_MaxRange          = 1000;
    float m_MinRange          = 30;
public:
    CYdLidar(/* args */);


    void sendCommand(uint8_t cmd, const void *payload = NULL, size_t payloadsize = 0);
    bool startScan();
    bool getDeviceAddress();
    bool stopScan();
    bool getDevicePara(gs_device_para &info);
    bool waitResponseHeader(gs_lidar_ans_header *header);
    bool scanData(node_info *node);
    void angTransform(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist);
    bool doProcessSimple(void);
    bool isRangeValid(double reading) const;
    bool printbuffer(void);
    void printLidarPoints(void);

    ~CYdLidar();
};

