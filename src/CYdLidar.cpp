#include "CYdLidar.h"

CYdLidar::CYdLidar(/* args */)
{
}

CYdLidar::~CYdLidar()
{
}


bool CYdLidar::sendCommand(uint8_t cmd,
                                    const void *payload,
                                    size_t payloadsize)
{
    return sendCommand(0x00, cmd, payload, payloadsize);
}

bool CYdLidar::sendCommand(uint8_t addr,
                                    uint8_t cmd,
                                    const void *payload,
                                    size_t payloadsize)
{
    uint8_t pkt_header[12];
    cmd_packet_gs *header = reinterpret_cast<cmd_packet_gs * >(pkt_header);
    uint8_t checksum = 0;


    header->syncByte0 = LIDAR_CMD_SYNC_BYTE;
    header->syncByte1 = LIDAR_CMD_SYNC_BYTE;
    header->syncByte2 = LIDAR_CMD_SYNC_BYTE;
    header->syncByte3 = LIDAR_CMD_SYNC_BYTE;
    header->address = addr;
    header->cmd_flag = cmd;
    header->size = 0xffff&payloadsize;
    usartSend1Data(pkt_header, 8) ;
    checksum += cmd;
    checksum += 0xff&header->size;
    checksum += 0xff&(header->size>>8);

    if (payloadsize && payload) {
      for (size_t pos = 0; pos < payloadsize; ++pos) {
        checksum += ((uint8_t *)payload)[pos];
      }
      uint8_t sizebyte = (uint8_t)(payloadsize);
      usartSend1Data((const uint8_t *)payload, sizebyte);
    }

    usartSend1Data(&checksum, 1);

    return RESULT_OK;
}

bool CYdLidar::startScan(bool force){
    bool ans;

    stop();
    //checkTransDelay();
    usart1flushSerial();

    setDeviceAddress(300);

    gs_device_para gs2_info;
    getDevicePara(gs2_info);
    {
        usart1flushSerial();

        if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : GS_LIDAR_CMD_SCAN)) !=
                RESULT_OK) {
            return ans;
        }

        if (!m_SingleChannel)
        {
            gs_lidar_ans_header response_header;

            if ((ans = waitResponseHeader(&response_header)) != RESULT_OK) {
                return ans;
            }

            if (response_header.type != GS_LIDAR_ANS_SCAN) {
                usartprintf("[CYdLidar] Response to start scan type error!\n");
                return RESULT_FAIL;
            }
        }

        //ans = this->createThread();
    }

    return ans;
}

bool CYdLidar::setDeviceAddress(uint32_t timeout){
    bool ans;

    if (m_SingleChannel) {
        return RESULT_OK;
    }

    usart1flushSerial();
    {

        if ((ans = sendCommand(GS_LIDAR_CMD_GET_ADDRESS)) != RESULT_OK) {
            return ans;
        }

        gs_lidar_ans_header response_header;
        if ((ans = waitResponseHeader(&response_header)) != RESULT_OK) {
            return ans;
        }

        if (response_header.type != GS_LIDAR_CMD_GET_ADDRESS) {
            return RESULT_FAIL;
        }

        usartprintf("[YDLIDAR] Lidar module count %d\n", (response_header.address << 1) + 1);
    }

    return RESULT_OK;
}

bool CYdLidar::stop(){
    stopScan();

    return RESULT_OK;
}

bool CYdLidar::stopScan(){
    bool  ans;

    if ((ans = sendCommand(GS_LIDAR_CMD_STOP)) != RESULT_OK) {
      return ans;
    }
    gs_lidar_ans_header response_header;
    if ((ans = waitResponseHeader(&response_header)) != RESULT_OK) {
        return ans;
    }
    if (response_header.type != GS_LIDAR_CMD_STOP) {
        return RESULT_FAIL;
    }

    return RESULT_OK;
}


bool CYdLidar::getDevicePara(gs_device_para &info) {
  bool  ans;
  uint8_t crcSum, mdNum;
  uint8_t *pInfo = reinterpret_cast<uint8_t *>(&info);

  usart1flushSerial();
  {

    if ((ans = sendCommand(GS_LIDAR_CMD_GET_PARAMETER)) != RESULT_OK) {
      return ans;
    }
    gs_lidar_ans_header response_header;
    if ((ans = waitResponseHeader(&response_header)) != RESULT_OK) {
        return ans;
    }
    if (response_header.type != GS_LIDAR_CMD_GET_PARAMETER) {
        return RESULT_FAIL;
    }
    if (response_header.size < (sizeof(gs_device_para) - 1)) {
        return RESULT_FAIL;
    }

    size_t recvSize = 0;
    while (recvSize < response_header.size+1)
    {
        if(usart1recev(&(reinterpret_cast<uint8_t *>(&info))[recvSize])){
            recvSize ++;
        }
    }

    crcSum = 0;
    crcSum += response_header.address;
    crcSum += response_header.type;
    crcSum += 0xff & response_header.size;
    crcSum += 0xff & (response_header.size >> 8);
    for(int j = 0; j < response_header.size; j++) {
        crcSum += pInfo[j];
    }
    if(crcSum != info.crc) {
        return RESULT_FAIL;
    }

    mdNum = response_header.address >> 1; // 1,2,4
    if( mdNum > 2) {
        return RESULT_FAIL;
    }
    u_compensateK0 = info.u_compensateK0;
    u_compensateK1 = info.u_compensateK1;
    u_compensateB0 = info.u_compensateB0;
    u_compensateB1 = info.u_compensateB1;
    d_compensateK0 = info.u_compensateK0 / 10000.00;
    d_compensateK1 = info.u_compensateK1 / 10000.00;
    d_compensateB0 = info.u_compensateB0 / 10000.00;
    d_compensateB1 = info.u_compensateB1 / 10000.00;
    bias = double(info.bias) * 0.1;
  }

  return RESULT_OK;
}


bool CYdLidar::waitResponseHeader(gs_lidar_ans_header *header) {
    int  recvPos     = 0;
    uint8_t  recvBuffer[sizeof(gs_lidar_ans_header)];
    uint8_t  *headerBuffer = reinterpret_cast<uint8_t *>(header);
    uint32_t waitTime = 0;
    has_device_header = false;
    last_device_byte = 0x00;

    while (1) {
      size_t remainSize = sizeof(gs_lidar_ans_header) - recvPos;
      size_t recvSize = 0;

      while (recvSize < remainSize)
      {
        if(usart1recev(&(recvBuffer[recvSize]))){
            recvSize ++;
        }
      }

      for (size_t pos = 0; pos < recvSize; ++pos) {
        uint8_t currentByte = recvBuffer[pos];

        switch (recvPos) {
            case 0:
              if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
                  recvPos = 0;
                  continue;
              }
              break;

            case 1:
              if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
                  recvPos = 0;
                  continue;
              }
              break;

            case 2:
              if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
                  recvPos = 0;
                  continue;
              }
              break;

            case 3:
              if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
                  recvPos = 0;
                  continue;
              }
              has_device_header = true;
              break;

            default:
              break;
        }

        headerBuffer[recvPos++] = currentByte;
        last_device_byte = currentByte;

        if (has_device_header && recvPos == sizeof(gs_lidar_ans_header)) {
          return RESULT_OK;
        }
      }
    }

    return RESULT_FAIL;
}

