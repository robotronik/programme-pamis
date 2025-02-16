#pragma once
#include <stdint.h>
#include "uartLidar.h"
#include "uart.h"
#include "ydlidar_protocol.h"
#include "angles.h"

#define size_t uint16_t

#define RESULT_FAIL false
#define RESULT_OK   true

#define MAX_SCAN_NODES 160

class CYdLidar
{
private:
    /* data */
    bool has_device_header;
    uint8_t last_device_byte;

    double  d_compensateK0[3];
    double  d_compensateK1[3];
    double  d_compensateB0[3];
    double  d_compensateB1[3];
    uint16_t  u_compensateK0[3];
    uint16_t  u_compensateK1[3];
    uint16_t  u_compensateB0[3];
    uint16_t  u_compensateB1[3];
    double  bias[3];
    bool m_SingleChannel = false;

    static node_info scan_node_buf[MAX_SCAN_NODES];

    gs2_node_package package;
    bool isValidPoint;
    uint16_t package_Sample_Index;
    uint8_t CheckSumCal;
    uint8_t   moduleNum;
    uint8_t CheckSum;
    bool CheckSumResult;
    bool has_package_error;
    int package_index;
    uint8_t scan_frequence;
    bool m_intensities;
    float   m_AngleOffset;
    bool m_Reversion = false;
    bool m_Inverted = false;//
    float m_MaxRange          = 64.0;
    float m_MinRange          = 0.01;
public:
    CYdLidar(/* args */);


    bool sendCommand(uint8_t cmd, const void *payload = NULL, size_t payloadsize = 0);
    bool sendCommand(uint8_t addr, uint8_t cmd, const void *payload = NULL, size_t payloadsize = 0);
    bool startScan(bool force);
    bool setDeviceAddress();
    bool stop();
    bool stopScan();
    bool getDevicePara(gs_device_para &info);
    bool waitResponseHeader(gs_lidar_ans_header *header);
    bool waitPackage(node_info *node);
    void angTransform(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist);
    bool waitScanData(node_info *nodebuffer, size_t &count);
    bool doProcessSimple(void);
    bool isRangeValid(double reading) const;
    bool printbuffer(void);

    ~CYdLidar();
};

