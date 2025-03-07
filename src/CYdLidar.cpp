#include "CYdLidar.h"
#include "math.h"

node_info CYdLidar::scan_node_buf[MAX_SCAN_NODES];
final_Node CYdLidar::final_node_buf[MAX_SCAN_NODES];
uint8_t* globalRecvBuffer = new uint8_t[sizeof(gs2_node_package)];

CYdLidar::CYdLidar(/* args */)
{
}

CYdLidar::~CYdLidar()
{
}


void CYdLidar::sendCommand(uint8_t cmd,const void *payload,size_t payloadsize)
{
    uint8_t pkt_header[12];
    gs_lidar_header *header = reinterpret_cast<gs_lidar_header * >(pkt_header);
    uint8_t checksum = 0;


    header->syncByte0 = LIDAR_CMD_SYNC_BYTE;
    header->syncByte1 = LIDAR_CMD_SYNC_BYTE;
    header->syncByte2 = LIDAR_CMD_SYNC_BYTE;
    header->syncByte3 = LIDAR_CMD_SYNC_BYTE;
    header->address = 0x00;
    header->cmdFlag = cmd;
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
}

bool CYdLidar::startScan(){
    gs_lidar_header response_header;
    gs_device_para gs2_info;

    stopScan();
    getDeviceAddress();
    getDevicePara(gs2_info);
    sendCommand(GS_LIDAR_CMD_SCAN);
    waitResponseHeader(&response_header);
    if (response_header.cmdFlag != GS_LIDAR_ANS_SCAN) {
        usartprintf("[YDLIDAR_ERROR] START SCAN FAIL : bad reponse\n");
        return RESULT_FAIL;
    }
    return RESULT_OK;
}

bool CYdLidar::getDeviceAddress(){
    gs_lidar_header response_header;

    sendCommand(GS_LIDAR_CMD_GET_ADDRESS);
    waitResponseHeader(&response_header);


    if (response_header.cmdFlag != GS_LIDAR_CMD_GET_ADDRESS) {
        usartprintf("[YDLIDAR_ERROR] GET_DEVICE_ADDRESS FAIL : bad reponse\n");
        return RESULT_FAIL;
    }
    usartprintf("[YDLIDAR_INFO] Lidar module count %d\n", (response_header.address << 1) + 1);

    return RESULT_OK;
}

bool CYdLidar::stopScan(){
    gs_lidar_header response_header;

    usart1flushSerial();
    sendCommand(GS_LIDAR_CMD_STOP);
    waitResponseHeader(&response_header);

    if(response_header.cmdFlag != GS_LIDAR_CMD_STOP) {
        usartprintf("[YDLIDAR_ERROR] STOP_SCAN FAIL : bad reponse\n");
        return RESULT_FAIL;
    }
    usartprintf("[YDLIDAR_INFO] stop scan\n");
    return RESULT_OK;
}


bool CYdLidar::getDevicePara(gs_device_para &info) {
    uint8_t crcSum;
    uint8_t *pInfo = reinterpret_cast<uint8_t *>(&info);
    gs_lidar_header response_header;

    usart1flushSerial();
    sendCommand(GS_LIDAR_CMD_GET_PARAMETER);
    waitResponseHeader(&response_header);

    if (response_header.cmdFlag != GS_LIDAR_CMD_GET_PARAMETER) {
        usartprintf("[YDLIDAR_ERROR] GET_DEVICE_PARA FAIL : bad reponse\n");
        return RESULT_FAIL;
    }
    if (response_header.size < (sizeof(gs_device_para) - 1)) {
        usartprintf("[YDLIDAR_ERROR] GET_DEVICE_PARA FAIL : reponse header to small\n");
        return RESULT_FAIL;
    }

    size_t recvSize = 0;
    while (recvSize < sizeof(info))
    {
        if(usart1recev(&(pInfo[recvSize]))){
            recvSize ++;
        }
    }

    crcSum = 0;
    crcSum += response_header.address;
    crcSum += response_header.cmdFlag;
    crcSum += 0xff & response_header.size;
    crcSum += 0xff & (response_header.size >> 8);
    for(int j = 0; j < response_header.size; j++) {
        crcSum += pInfo[j];
    }
    if(crcSum != info.crc) {
        usartprintf("[YDLIDAR_ERROR] GET_DEVICE_PARA FAIL : bad crc\n");
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
    usartprintf("> %d\n",u_compensateK0);
    usartprintf("> %d\n",u_compensateK1);
    usartprintf("> %d\n",u_compensateB0);
    usartprintf("> %d\n",u_compensateB1);
    usartprintf("> %lf\n",d_compensateK0);
    usartprintf("> %lf\n",d_compensateK1);
    usartprintf("> %lf\n",d_compensateB0);
    usartprintf("> %lf\n",d_compensateB1);
    usartprintf("> %lf\n",bias);

    return RESULT_OK;
}


bool CYdLidar::waitResponseHeader(gs_lidar_header *header) {
    uint8_t *headerBuffer = reinterpret_cast<uint8_t *>(header);
    size_t recvPos = 0;
    uint8_t currentByte;

    while (recvPos < sizeof(gs_lidar_header)) {

        while (!usart1recev(&currentByte));

        if (recvPos < 4) {
            if (currentByte == LIDAR_ANS_SYNC_BYTE1) {
                headerBuffer[recvPos] = currentByte;
                recvPos++;
            } else {
                //usartprintf("waitResponseHeader : invalid data : %02x\n",currentByte);
                recvPos = 0;
            }
        } else {
            headerBuffer[recvPos] = currentByte;
            recvPos++;
        }
    }
    return RESULT_OK;
}


bool CYdLidar::scanData(node_info *node)
{
    int recvPos = 0;
    uint8_t  *packageBuffer = (uint8_t *)&package;
    int  package_recvPos    = 0;
    uint16_t sample_lens = 0;
    has_device_header = false;
    uint16_t package_Sample_Num = 0;
    gs_lidar_header header;
    uint8_t *headerBuffer = reinterpret_cast<uint8_t *>(&header);

    node[0].index = 255;
    node[0].scan_frequence  = 0;


    usartprintf("START\n");
    usart1flushSerial();
    waitResponseHeader(&header);

    if(header.cmdFlag != GS_LIDAR_ANS_SCAN) {
        usartprintf("[YDLIDAR_ERROR] SCAN FAIL : bad reponse\n");
        return RESULT_FAIL;
    }

    sample_lens = header.size;
    package_recvPos = PackagePaidBytes_GS;
    CheckSumCal = headerBuffer[4] + headerBuffer[5] + headerBuffer[6] + headerBuffer[7];
    package_Sample_Num = sample_lens+1;

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
        for(int i = 0; i < 160; i++){
            node[i].sync_quality    = Node_Default_Quality;
            node[i].angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
            node[i].distance_q2      = 0;
        }
        usartprintf("[YDLIDAR_ERROR] SCAN FAIL : bad package_Sample_Num\n");
        return RESULT_FAIL;
    }

    if (CheckSumCal != CheckSum) {
        for(int i = 0; i < 160; i++){
            node[i].sync_quality    = Node_Default_Quality;
            node[i].angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
            node[i].distance_q2      = 0;
        }
        usartprintf("[YDLIDAR_ERROR] SCAN FAIL : bad checksumm\n");
        return RESULT_FAIL;
    }

    for(int i = 0; i < 160; i++){
        double sampleAngle = 0;

        node[i].sync_quality = Node_Default_Quality;
        node[i].distance_q2 = package.packageSample[i].PakageSampleDistance;

        if (m_intensities) {
            node[i].sync_quality = (uint16_t)package.packageSample[i].PakageSampleQuality;
        }

        if (node[i].distance_q2 > 0)
        {
            angTransform(node[i].distance_q2,i,&sampleAngle,&(node[i].distance_q2));
        }

        if (sampleAngle< 0) {
            node[i].angle_q6_checkbit = (((uint16_t)(sampleAngle * 64 + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
        } else {
            if ((sampleAngle * 64) > 23040) {
                node[i].angle_q6_checkbit = (((uint16_t)(sampleAngle * 64 - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                        LIDAR_RESP_MEASUREMENT_CHECKBIT;
            } else {
                node[i].angle_q6_checkbit = (((uint16_t)(sampleAngle * 64)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                        LIDAR_RESP_MEASUREMENT_CHECKBIT;
            }
        }

        if(i < 80){ //CT_RingStart  CT_Normal
            if(node[i].angle_q6_checkbit <= 23041){
                node[i].distance_q2 = 0;
            }
        }else {
            if(node[i].angle_q6_checkbit > 23041){
                node[i].distance_q2 = 0;
            }
        }
    }
    return RESULT_OK;
}



void CYdLidar::angTransform(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist)
{
    double pixelU = n, Dist, theta, tempTheta, tempDist, tempX, tempY;
    if (n < 80)
    {
      pixelU = 80 - pixelU;
      if (d_compensateB0 > 1) {
          tempTheta = d_compensateK0 * pixelU - d_compensateB0;
      }
      else
      {
          tempTheta = atan(d_compensateK0 * pixelU - d_compensateB0) * 180 / M_PI;
      }
      tempDist = (dist - Angle_Px) / cos(((Angle_PAngle + bias) - (tempTheta)) * M_PI / 180);
      tempTheta = tempTheta * M_PI / 180;
      tempX = cos((Angle_PAngle + bias) * M_PI / 180) * tempDist * cos(tempTheta) + sin((Angle_PAngle + bias) * M_PI / 180) * (tempDist *
                                                                                             sin(tempTheta));
      tempY = -sin((Angle_PAngle + bias) * M_PI / 180) * tempDist * cos(tempTheta) + cos((Angle_PAngle + bias) * M_PI / 180) * (tempDist *
                                                                                              sin(tempTheta));
      tempX = tempX + Angle_Px;
      tempY = tempY - Angle_Py; //5.315
      Dist = sqrt(tempX * tempX + tempY * tempY);
      theta = atan(tempY / tempX) * 180 / M_PI;
    }
    else
    {
      pixelU = 160 - pixelU;
      if (d_compensateB1 > 1)
      {
          tempTheta = d_compensateK1 * pixelU - d_compensateB1;
      }
      else
      {
          tempTheta = atan(d_compensateK1 * pixelU - d_compensateB1) * 180 / M_PI;
      }
      tempDist = (dist - Angle_Px) / cos(((Angle_PAngle + bias) + (tempTheta)) * M_PI / 180);
      tempTheta = tempTheta * M_PI / 180;
      tempX = cos(-(Angle_PAngle + bias) * M_PI / 180) * tempDist * cos(tempTheta) + sin(-(Angle_PAngle + bias) * M_PI / 180) * (tempDist *
                                                                                               sin(tempTheta));
      tempY = -sin(-(Angle_PAngle + bias) * M_PI / 180) * tempDist * cos(tempTheta) + cos(-(Angle_PAngle + bias) * M_PI / 180) * (tempDist *
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


bool CYdLidar::doProcessSimple(void){

    scanData(scan_node_buf);

    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;

    for (size_t i = 0; i < MAX_SCAN_NODES; i++)
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

        //valid range
        if (!isRangeValid(range)) {
            range = 0.0;
            intensity = 0.0;
        }

        final_node_buf[i].angle = angle;
        final_node_buf[i].distance = range;

    }
    return true;
}

bool CYdLidar::printbuffer(void){

    for (size_t i=0; i<MAX_SCAN_NODES; ++i)
    {
        usartprintf("%d %.02f %.02f\n", i, final_node_buf[i].angle * 180.0 / M_PI, final_node_buf[i].distance);
    }
    usartprintf("\n");

    return true;

}

void CYdLidar::printLidarPoints(void) {
    char grid[HEIGHT][WIDTH];

    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            grid[y][x] = ' ';
        }
    }

    int centerY = HEIGHT / 2;
    grid[centerY][0] = 'O';

    for (size_t i = 0; i < MAX_SCAN_NODES; ++i) {
        double angle = final_node_buf[i].angle;
        double range = final_node_buf[i].distance;

        if(range != 0){
            if (range > MAX_RANGE) range = MAX_RANGE;

            int x = (int)(cos(angle) * range / MAX_RANGE * (WIDTH / 2));
            int y = (int)(sin(angle) * range / MAX_RANGE * (HEIGHT / 2)) + centerY;

            if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
                grid[y][x] = '*';
            }
        }
    }

    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            usartprintf("%c", grid[y][x]);
        }
        usartprintf("\n");
    }
}


bool CYdLidar::isRangeValid(double reading) const {
    if (reading >= m_MinRange && reading <= m_MaxRange) {
        return true;
    }

    return false;
}