#include "CYdLidar.h"
#include "math.h"

node_info CYdLidar::scan_node_buf[MAX_SCAN_NODES];
uint8_t* globalRecvBuffer = new uint8_t[sizeof(gs2_node_package)];

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

    setDeviceAddress();

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

    }

    return ans;
}

bool CYdLidar::setDeviceAddress(){
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
    while (recvSize < sizeof(info))
    {
        if(usart1recev(&(reinterpret_cast<uint8_t *>(&info))[recvSize])){
            recvSize ++;
        }
    }
    usartprintf("FFFFFFIN\n");

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
    u_compensateK0[0] = info.u_compensateK0;
    u_compensateK1[0] = info.u_compensateK1;
    u_compensateB0[0] = info.u_compensateB0;
    u_compensateB1[0] = info.u_compensateB1;
    d_compensateK0[0] = info.u_compensateK0 / 10000.00;
    d_compensateK1[0] = info.u_compensateK1 / 10000.00;
    d_compensateB0[0] = info.u_compensateB0 / 10000.00;
    d_compensateB1[0] = info.u_compensateB1 / 10000.00;
    bias[0] = double(info.bias) * 0.1;
    usartprintf("> %d\n",u_compensateK0[0]);
    usartprintf("> %d\n",u_compensateK1[0]);
    usartprintf("> %d\n",u_compensateB0[0]);
    usartprintf("> %d\n",u_compensateB1[0]);
    usartprintf("FFFFFFIN\n");
    usartprintf("> %lf\n",d_compensateK0[0]);
    usartprintf("> %lf\n",d_compensateK1[0]);
    usartprintf("> %lf\n",d_compensateB0[0]);
    usartprintf("> %lf\n",d_compensateB1[0]);
    usartprintf("> %lf\n",bias[0]);
  }

  return RESULT_OK;
}


bool CYdLidar::waitResponseHeader(gs_lidar_ans_header *header) {
    int  recvPos     = 0;
    uint8_t  recvBuffer[sizeof(gs_lidar_ans_header)];
    uint8_t  *headerBuffer = reinterpret_cast<uint8_t *>(header);
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


bool CYdLidar::waitPackage(node_info *node)
{
    int recvPos         = 0;
    uint8_t  *packageBuffer = (uint8_t *)&package;
    isValidPoint  =  true;
    int  package_recvPos    = 0;
    uint16_t sample_lens = 0;
    has_device_header = false;
    uint16_t package_Sample_Num = 0;

    (*node).index = 255;
    (*node).scan_frequence  = 0;

    if (package_Sample_Index == 0)
    {
        recvPos = 0;

        while (1)
        {
            size_t remainSize   = PackagePaidBytes_GS - recvPos;
            size_t recvSize     = 0;
            CheckSumCal = 0;

            while (recvSize < remainSize)
            {
                if(usart1recev(&(globalRecvBuffer[recvSize]))){
                    recvSize ++;
                }
            }

            for (size_t pos = 0; pos < recvSize; ++pos)
            {
                uint8_t currentByte = globalRecvBuffer[pos];
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

                case 4:
                    moduleNum = currentByte;
                    CheckSumCal += currentByte;
                    break;

                case 5:
                    if (currentByte != GS_LIDAR_ANS_SCAN) {
                        recvPos = 0;
                        CheckSumCal = 0;
                        moduleNum = 0;
                        has_device_header = false;
                        continue;
                    }
                    CheckSumCal += currentByte;
                    break;

                case 6:
                    sample_lens |= 0x00ff&currentByte;
                    CheckSumCal += currentByte;
                    break;

                case 7:
                    sample_lens |= (0x00ff&currentByte)<<8;
                    CheckSumCal += currentByte;
                    break;

                default :
                    break;
                }

                packageBuffer[recvPos++] = currentByte;
            }

            if (has_device_header &&
                recvPos  == PackagePaidBytes_GS) {
                package_Sample_Num = sample_lens+1;
                package_recvPos = recvPos;
                break;
            }
            else {
                recvPos = 0;
                usartprintf("invalid data\n");
                continue;
            }
        }

        if (PackagePaidBytes_GS == recvPos)
        {
            recvPos = 0;

            while (1)
            {
                size_t remainSize = package_Sample_Num - recvPos;
                size_t recvSize = 0;
                while (recvSize < remainSize)
                {
                    if(usart1recev(&(globalRecvBuffer[recvSize]))){
                        recvSize ++;
                    }
                }

                for (size_t pos = 0; pos < recvSize-1; pos++) {
                    CheckSumCal += globalRecvBuffer[pos];
                    packageBuffer[package_recvPos + recvPos] = globalRecvBuffer[pos];
                    recvPos++;
                }
                CheckSum = globalRecvBuffer[recvSize-1];//crc
                packageBuffer[package_recvPos + recvPos] = CheckSum;//crc
                recvPos+=1;

                if (package_Sample_Num == recvPos) {
                    package_recvPos += recvPos;
                    break;
                }
            }

            if (package_Sample_Num != recvPos) {
                return RESULT_FAIL;
            }
        } else {
            return RESULT_FAIL;
        }

        if (CheckSumCal != CheckSum) {
            CheckSumResult = false;
            has_package_error = true;
        } else {
            CheckSumResult = true;
        }
    }

    if (!has_package_error) {
        if (package_Sample_Index == 0) {
            package_index++;
            (*node).index = package_index;
        }
    } else {
        (*node).index = 255;
        package_index = 0xff;
    }

    if (CheckSumResult) {
        (*node).index = package_index;
        (*node).scan_frequence  = scan_frequence;
    }

    (*node).sync_quality = Node_Default_Quality;
    (*node).stamp = 0;
    (*node).scan_frequence = 0;

    double sampleAngle = 0;
    if (CheckSumResult)
    {
        (*node).distance_q2 =
                package.packageSample[package_Sample_Index].PakageSampleDistance;

        if (m_intensities) {
            (*node).sync_quality = (uint16_t)package.packageSample[package_Sample_Index].PakageSampleQuality;
        }

        if (node->distance_q2 > 0)
        {
            angTransform((*node).distance_q2,package_Sample_Index,&sampleAngle,&(*node).distance_q2);
        }

//        printf("%lf ", sampleAngle);
        if (sampleAngle< 0) {
            (*node).angle_q6_checkbit = (((uint16_t)(sampleAngle * 64 + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
        } else {
            if ((sampleAngle * 64) > 23040) {
                (*node).angle_q6_checkbit = (((uint16_t)(sampleAngle * 64 - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                        LIDAR_RESP_MEASUREMENT_CHECKBIT;
            } else {
                (*node).angle_q6_checkbit = (((uint16_t)(sampleAngle * 64)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                        LIDAR_RESP_MEASUREMENT_CHECKBIT;
            }
        }

        if(package_Sample_Index < 80){ //CT_RingStart  CT_Normal
            if((*node).angle_q6_checkbit <= 23041){
                (*node).distance_q2 = 0;
                isValidPoint = false;
            }
        }else {
            if((*node).angle_q6_checkbit > 23041){
                (*node).distance_q2 = 0;
                isValidPoint = false;
            }
        }

//        printf("%d(%d) ", node->distance_q2, package_Sample_Index);

    } else {
        (*node).sync_flag       = Node_NotSync;
        (*node).sync_quality    = Node_Default_Quality;
        (*node).angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
        (*node).distance_q2      = 0;
        (*node).scan_frequence  = 0;
    }

    uint8_t nowPackageNum = 160;

    package_Sample_Index++;
    (*node).sync_flag = Node_NotSync;

    if (package_Sample_Index >= nowPackageNum) {
        package_Sample_Index = 0;
        (*node).sync_flag = Node_Sync;
        CheckSumResult = false;
    }

    return RESULT_OK;
}



void CYdLidar::angTransform(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist)
{
    double pixelU = n, Dist, theta, tempTheta, tempDist, tempX, tempY;
    uint8_t mdNum = 0x03 & (moduleNum >> 1);//1,2,4
    if (n < 80)
    {
      pixelU = 80 - pixelU;
      if (d_compensateB0[mdNum] > 1) {
          tempTheta = d_compensateK0[mdNum] * pixelU - d_compensateB0[mdNum];
      }
      else
      {
          tempTheta = atan(d_compensateK0[mdNum] * pixelU - d_compensateB0[mdNum]) * 180 / M_PI;
      }
      tempDist = (dist - Angle_Px) / cos(((Angle_PAngle + bias[mdNum]) - (tempTheta)) * M_PI / 180);
      tempTheta = tempTheta * M_PI / 180;
      tempX = cos((Angle_PAngle + bias[mdNum]) * M_PI / 180) * tempDist * cos(tempTheta) + sin((Angle_PAngle + bias[mdNum]) * M_PI / 180) * (tempDist *
                                                                                             sin(tempTheta));
      tempY = -sin((Angle_PAngle + bias[mdNum]) * M_PI / 180) * tempDist * cos(tempTheta) + cos((Angle_PAngle + bias[mdNum]) * M_PI / 180) * (tempDist *
                                                                                              sin(tempTheta));
      tempX = tempX + Angle_Px;
      tempY = tempY - Angle_Py; //5.315
      Dist = sqrt(tempX * tempX + tempY * tempY);
      theta = atan(tempY / tempX) * 180 / M_PI;
    }
    else
    {
      pixelU = 160 - pixelU;
      if (d_compensateB1[mdNum] > 1)
      {
          tempTheta = d_compensateK1[mdNum] * pixelU - d_compensateB1[mdNum];
      }
      else
      {
          tempTheta = atan(d_compensateK1[mdNum] * pixelU - d_compensateB1[mdNum]) * 180 / M_PI;
      }
      tempDist = (dist - Angle_Px) / cos(((Angle_PAngle + bias[mdNum]) + (tempTheta)) * M_PI / 180);
      tempTheta = tempTheta * M_PI / 180;
      tempX = cos(-(Angle_PAngle + bias[mdNum]) * M_PI / 180) * tempDist * cos(tempTheta) + sin(-(Angle_PAngle + bias[mdNum]) * M_PI / 180) * (tempDist *
                                                                                               sin(tempTheta));
      tempY = -sin(-(Angle_PAngle + bias[mdNum]) * M_PI / 180) * tempDist * cos(tempTheta) + cos(-(Angle_PAngle + bias[mdNum]) * M_PI / 180) * (tempDist *
                                                                                                sin(tempTheta));
      tempX = tempX + Angle_Px;
      tempY = tempY + Angle_Py; //5.315
      Dist = sqrt(tempX * tempX + tempY * tempY);
      theta = atan(tempY / tempX) * 180 / M_PI;
    }
    if (theta < 0)
    {
      theta += 360;
    }
    *dstTheta = theta;
    *dstDist = Dist;
}


bool CYdLidar::waitScanData(node_info *nodebuffer, size_t &count){

    size_t     recvNodeCount    =  0;
    bool   ans              = RESULT_FAIL;

    while (recvNodeCount < count)
    {
        node_info node;
        ans = waitPackage(&node);

        if (!ans) {
            count = recvNodeCount;
            return ans;
        }
        nodebuffer[recvNodeCount++] = node;

        if (!package_Sample_Index)
        {
            count = recvNodeCount;
            return RESULT_OK;
        }

        if (recvNodeCount == count) {
            return RESULT_OK;
        }
    }

    count = recvNodeCount;
    return RESULT_FAIL;
}


bool CYdLidar::doProcessSimple(void){

    uint16_t count = MAX_SCAN_NODES;
    waitScanData(scan_node_buf,count);

    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;

//        printf("points %lu\n", count);
    for (size_t i = 0; i < count; i++)
    {
        angle = static_cast<float>((scan_node_buf[i].angle_q6_checkbit >>
                                    LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f) + m_AngleOffset;
        range = static_cast<float>(scan_node_buf[i].distance_q2);

        intensity = static_cast<float>(scan_node_buf[i].sync_quality);
        angle = angles::from_degrees(angle);

        //Rotate 180 degrees or not
        if (m_Reversion) {
            angle = angle + M_PI;
        }

        //Is it counter clockwise
        if (m_Inverted) {
            angle = 2 * M_PI - angle;
        }

        angle = angles::normalize_angle(angle);
//            angle = angles::normalize_angle_positive(angle);



        //valid range
        if (!isRangeValid(range)) {
            range = 0.0;
            intensity = 0.0;
        }

//            printf("%lu %f %f\n", i, angles::to_degrees(angle), range);

        scan_node_buf[i].angle_q6_checkbit = angle;
        scan_node_buf[i].distance_q2 = range;
        scan_node_buf[i].sync_quality = intensity;

    }


    return true;

}

bool CYdLidar::printbuffer(void){

    for (size_t i=0; i<MAX_SCAN_NODES; ++i)
    {
        usartprintf("%d %.02f %.02f\n", i, scan_node_buf[i].angle_q6_checkbit * 180.0 / M_PI, scan_node_buf[i].distance_q2);
    }
    usartprintf("\n");

    return true;

}

bool CYdLidar::isRangeValid(double reading) const {
    if (reading >= m_MinRange && reading <= m_MaxRange) {
        return true;
    }

    return false;
}