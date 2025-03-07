#pragma once
#include <vector>
#include <stdint.h>


#ifndef M_PI
#define M_PI 3.1415926
#endif


#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1


#define Angle_Px   1.22
#define Angle_Py   5.315
#define Angle_PAngle   22.5

//gs2
#define GS_LIDAR_CMD_GET_ADDRESS               0x60
#define GS_LIDAR_CMD_GET_PARAMETER             0x61
#define GS_LIDAR_CMD_GET_VERSION               0x62
#define GS_LIDAR_CMD_SCAN                      0x63
#define GS_LIDAR_ANS_SCAN                      0x63
#define GS_LIDAR_CMD_STOP                      0x64
#define GS_LIDAR_CMD_RESET                     0x67
#define GS_LIDAR_CMD_SET_BIAS                  0xD9
#define GS_LIDAR_CMD_SET_DEBUG_MODE            0xF0


#define PackageSampleMaxLngth_GS 0xA0  //160


#define Node_Default_Quality (10)
#define PackagePaidBytes 10
#define PackagePaidBytes_GS 8


struct final_Node {
  float angle;
  float distance;
  float intensity;
};

struct node_info {
  uint8_t    sync_flag;
  uint16_t   sync_quality;
  uint16_t   angle_q6_checkbit;
  uint16_t   distance_q2;
  uint64_t   stamp;
  uint8_t    scan_frequence;
  uint8_t    debug_info[12];
  uint8_t    index;
} __attribute__((packed));

struct GS2PackageNode {
  uint16_t PakageSampleDistance:9;
  uint16_t PakageSampleQuality:7;
} __attribute__((packed));

struct gs_lidar_header {
    uint8_t  syncByte0;
    uint8_t  syncByte1;
    uint8_t  syncByte2;
    uint8_t  syncByte3;
    uint8_t  address;
    uint8_t  cmdFlag;
    uint16_t size;
} __attribute__((packed));

struct gs2_node_package {
  gs_lidar_header   packageHead;
  uint16_t          BackgroudLight;
  GS2PackageNode    packageSample[PackageSampleMaxLngth_GS];
  uint8_t           checkSum;
} __attribute__((packed));

struct gs_device_para {
    uint16_t u_compensateK0;
    uint16_t u_compensateB0;
    uint16_t u_compensateK1;
    uint16_t u_compensateB1;
    int8_t   bias;
    uint8_t  crc;
} __attribute__((packed));


