#include "CYdLidar.h"
#include "math.h"

final_Node CYdLidar::final_node_buf[PackageSampleMaxLngth_GS];
gs2_node_package CYdLidar::package;

CYdLidar::CYdLidar(/* args */)
{
}

CYdLidar::~CYdLidar()
{
}


void CYdLidar::sendCommand(uint8_t cmd,const void *payload,int payloadsize)
{
    uint8_t pkt_header[12];
    gs_lidar_header *header = reinterpret_cast<gs_lidar_header * >(pkt_header);
    uint8_t checksum = 0;


    header->syncByte0 = GS_LIDAR_CMD_SYNC_BYTE;
    header->syncByte1 = GS_LIDAR_CMD_SYNC_BYTE;
    header->syncByte2 = GS_LIDAR_CMD_SYNC_BYTE;
    header->syncByte3 = GS_LIDAR_CMD_SYNC_BYTE;
    header->address = 0x00;
    header->cmdFlag = cmd;
    header->size = 0xffff&payloadsize;
    usartSend1Data(pkt_header, 8) ;
    checksum += cmd;
    checksum += 0xff&header->size;
    checksum += 0xff&(header->size>>8);

    if (payloadsize && payload) {
      for (int pos = 0; pos < payloadsize; ++pos) {
        checksum += ((uint8_t *)payload)[pos];
      }
      uint8_t sizebyte = (uint8_t)(payloadsize);
      usartSend1Data((const uint8_t *)payload, sizebyte);
    }

    usartSend1Data(&checksum, 1);
}

bool CYdLidar::waitResponseHeader(gs_lidar_header *header) {
    uint8_t *headerBuffer = reinterpret_cast<uint8_t *>(header);
    unsigned int recvPos = 0;
    uint8_t currentByte;

    while (recvPos < sizeof(gs_lidar_header)) {

        while (!usart1recev(&currentByte));

        if (recvPos < 4) {
            if (currentByte == GS_LIDAR_CMD_SYNC_BYTE) {
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

bool CYdLidar::setup(){

    if(!stopScan()){
        return RESULT_FAIL;
    }

    if(!getDeviceAddress()){
        return RESULT_FAIL;
    }

    if(getDevicePara()){
        return RESULT_FAIL;
    }

    if(!startScan()){
        return RESULT_FAIL;
    }

    return RESULT_OK;
}

bool CYdLidar::startScan(){
    gs_lidar_header response_header;

    sendCommand(GS_LIDAR_CMD_SCAN);
    waitResponseHeader(&response_header);
    if(!checkHead(&response_header,GS_LIDAR_CMD_SCAN)){
        return RESULT_FAIL;
    }

    usartprintf("[YDLIDAR_INFO] start scan %d\n");

    return RESULT_OK;
}

bool CYdLidar::getDeviceAddress(){
    gs_lidar_header response_header;

    sendCommand(GS_LIDAR_CMD_GET_ADDRESS);
    waitResponseHeader(&response_header);
    if(!checkHead(&response_header,GS_LIDAR_CMD_GET_ADDRESS)){
        return RESULT_FAIL;
    }

    m_address = (response_header.address << 1) + 1;

    usartprintf("[YDLIDAR_INFO] Lidar module count %d\n", m_address);

    return RESULT_OK;
}

bool CYdLidar::stopScan(){
    gs_lidar_header response_header;

    usart1flushSerial();
    sendCommand(GS_LIDAR_CMD_STOP);
    waitResponseHeader(&response_header);
    if(!checkHead(&response_header,GS_LIDAR_CMD_STOP)){
        return RESULT_FAIL;
    }

    usartprintf("[YDLIDAR_INFO] stop scan\n");
    return RESULT_OK;
}


bool CYdLidar::getDevicePara() {
    uint8_t crcSum;
    gs_device_para info;
    uint8_t *pInfo = reinterpret_cast<uint8_t *>(&info);
    gs_lidar_header response_header;
    unsigned int recvSize = 0;

    usart1flushSerial();
    sendCommand(GS_LIDAR_CMD_GET_PARAMETER);
    waitResponseHeader(&response_header);
    if(!checkHead(&response_header,GS_LIDAR_CMD_GET_PARAMETER,sizeof(gs_device_para)-1)){
        return RESULT_FAIL;
    }

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

    return RESULT_OK;
}



bool CYdLidar::scanData()
{
    uint8_t *packageBuffer = (uint8_t *)&package;
    uint8_t CheckSumCal = 0;


    usart1flushSerial();
    waitResponseHeader(&(package.packageHead));
    if(!checkHead(&package.packageHead,GS_LIDAR_CMD_SCAN,(sizeof(package.packageSample) + sizeof(package.BackgroudLight)))){
        return RESULT_FAIL;
    }



    int remainSize = package.packageHead.size + 1 + sizeof(package.packageHead);
    int recvSize = sizeof(package.packageHead);
    while (recvSize < remainSize)
    {
        if(usart1recev(&(packageBuffer[recvSize]))){
            recvSize ++;
        }
    }

    for (int pos = 4; pos < recvSize-1; pos++) {
        CheckSumCal += packageBuffer[pos];
    }


    if (CheckSumCal != package.checkSum) {
        for(int i = 0; i < PackageSampleMaxLngth_GS; i++){
            final_node_buf[i].intensity = 0;
            final_node_buf[i].angle     = 0;
            final_node_buf[i].distance  = 0;
        }
        usartprintf("[YDLIDAR_ERROR] SCAN FAIL : bad checksumm\n");
        return RESULT_FAIL;
    }

    processData();

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


bool CYdLidar::processData(void){


    uint16_t sync_quality;
    uint16_t angle_q6_checkbit;
    uint16_t distance_q2;
    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;

    for (int i = 0; i < PackageSampleMaxLngth_GS; i++)
    {

        double sampleAngle = 0;

        sync_quality = Node_Default_Quality;
        distance_q2 = package.packageSample[i].PakageSampleDistance;

        if (m_intensities) {
            sync_quality = (uint16_t)package.packageSample[i].PakageSampleQuality;
        }

        if (distance_q2 > 0)
        {
            angTransform(distance_q2,i,&sampleAngle,&(distance_q2));
        }

        if (sampleAngle< 0) {
            angle_q6_checkbit = (((uint16_t)(sampleAngle * 64 + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
        } else {
            if ((sampleAngle * 64) > 23040) {
                angle_q6_checkbit = (((uint16_t)(sampleAngle * 64 - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                        LIDAR_RESP_MEASUREMENT_CHECKBIT;
            } else {
                angle_q6_checkbit = (((uint16_t)(sampleAngle * 64)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                        LIDAR_RESP_MEASUREMENT_CHECKBIT;
            }
        }

        if(i < 80){ //CT_RingStart  CT_Normal
            if(angle_q6_checkbit <= 23041){
                distance_q2 = 0;
            }
        }else {
            if(angle_q6_checkbit > 23041){
                distance_q2 = 0;
            }
        }


        angle = static_cast<float>((angle_q6_checkbit >>
                                    LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f) + m_AngleOffset;
        range = static_cast<float>(distance_q2);

        intensity = static_cast<float>(sync_quality);
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
        final_node_buf[i].intensity = intensity;

    }
    return true;
}

bool CYdLidar::debug_checkHead(const char* functionName, const char* functionFile, const int functionLine ,gs_lidar_header* head,uint8_t flag,uint16_t size){
    if(head->cmdFlag != flag) {
        usartprintf("[YDLIDAR_ERROR] %s: %s %d FAIL : bad reponse\n",functionName,functionFile,functionLine);
        return RESULT_FAIL;
    }

    if (head->size != size) {
        usartprintf("[YDLIDAR_ERROR] %s: %s %d FAIL : reponse header bad lenght\n",functionName,functionFile,functionLine);
        return RESULT_FAIL;
    }

    return RESULT_OK;
}

bool CYdLidar::printbuffer(void){

    for (int i=0; i<PackageSampleMaxLngth_GS; ++i)
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

    for (int i = 0; i < PackageSampleMaxLngth_GS; ++i) {
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

void CYdLidar::printPara(void){
    usartprintf("> %d\n",u_compensateK0);
    usartprintf("> %d\n",u_compensateK1);
    usartprintf("> %d\n",u_compensateB0);
    usartprintf("> %d\n",u_compensateB1);
    usartprintf("> %lf\n",d_compensateK0);
    usartprintf("> %lf\n",d_compensateK1);
    usartprintf("> %lf\n",d_compensateB0);
    usartprintf("> %lf\n",d_compensateB1);
    usartprintf("> %lf\n",bias);
}


bool CYdLidar::isRangeValid(double reading) const {
    if (reading >= m_MinRange && reading <= m_MaxRange) {
        return true;
    }

    return false;
}