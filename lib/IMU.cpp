#include "IMU.h"


IMU::IMU(MPU_SELECT mpuSelect)
{
    mpuSelected = mpuSelect;
    // initializing IMU
    byteWrite(MPU_ADDR, PWR_MGMT_1 , 0x00); // Start the sensor 
    byteWrite(MPU_ADDR, PWR_MGMT_1 , 0x01); // config clock to 1kHz


    if(mpuSelected == MPU_9250)
    {
        // initializing Magnetometer
        byteWrite(MPU_ADDR, INT_PIN_CFG, 0x02); // bypass mode for magnetometer
        byteWrite(AK8963_ADDR, AK8963_CNTL1, 0x00); // Power down to config the magnetometer
        byteWrite(AK8963_ADDR, AK8963_CNTL1, 0x0F); // Fuse ROM access mode
        byteWrite(AK8963_ADDR, AK8963_CNTL1, 0x16); // 16-bit output
    }
}

void IMU::setLowPassFilter(DLPF_CONFIG_RATE rateDLPF, ACCEL_LPF_SEL rateAccel, GYRO_LPF_SEL rateGyro, MAG_LPF_SEL rateMag)
{
    // Configuring data rate
    byteWrite(MPU_ADDR, DLPF_CFG , rateDLPF ); // configurating the DLPF
    byteWrite(MPU_ADDR, ACCEL_CFG, rateAccel); // configurating the Accel
    byteWrite(MPU_ADDR, GYRO_CFG , rateGyro ); // configurating the Gyro

    if(mpuSelected == MPU_9250)
    byteWrite(AK8963_ADDR, AK8963_CNTL1, rateMag); // configurating the Mag
    
}

void IMU::update()
{
    updateAccel();
    updateGyro();

    if(mpuSelected == MPU_9250)
    updateMag();
}
void IMU::updateAccel()
{
    uint8_t data[6];
    byteRead(MPU_ADDR, ACCEL_ADDR, 6, data);

    accelX = (int16_t)(data[1] << 8 | data[0]);
    accelY = (int16_t)(data[3] << 8 | data[2]);
    accelZ = (int16_t)(data[5] << 8 | data[4]);
}
void IMU::updateGyro()
{
    uint8_t data[6];
    byteRead(MPU_ADDR, GYRO_ADDR, 6, data);

    gyroX = (int16_t)(data[1] << 8 | data[0]);
    gyroY = (int16_t)(data[3] << 8 | data[2]);
    gyroZ = (int16_t)(data[5] << 8 | data[4]);
}
void IMU::updateMag()
{
    uint8_t data[6];
    byteRead(AK8963_ADDR, AK8963_XOUT_L, 6, data);

    magX = (int16_t)(data[1] << 8 | data[0]);
    magY = (int16_t)(data[3] << 8 | data[2]);
    magZ = (int16_t)(data[5] << 8 | data[4]);
}

void IMU::getAccel(float *xA, float *yA, float *zA)
{
    *xA = (float)accelX;
    *yA = (float)accelY;
    *zA = (float)accelZ;
}
void IMU::getGyro(float *xG, float *yG, float *zG)
{
    *xG = gyroX;
    *yG = gyroY;
    *zG = gyroZ;
}
void IMU::getMag(float *xM, float *yM, float *zM)
{
    *xM = magX;
    *yM = magY;
    *zM = magZ;
}

void IMU::byteWrite(uint8_t diviceAddress, uint8_t registerAddress, uint8_t data)
{
    Wire.beginTransmission(diviceAddress);
    Wire.write(registerAddress);
    Wire.write(data);
    Wire.endTransmission();
}
int  IMU::byteRead(uint8_t diviceAddress, uint8_t registerAddress)
{
    Wire.beginTransmission(diviceAddress);
    Wire.write(registerAddress);
    Wire.endTransmission();
    Wire.requestFrom(diviceAddress, 1);

    return Wire.read();
}
void IMU::byteRead(uint8_t diviceAddress, uint8_t registerAddress, uint8_t number, uint8_t *data)
{
    Wire.beginTransmission(diviceAddress);
    Wire.write(registerAddress);
    Wire.endTransmission();
    Wire.requestFrom(diviceAddress, number);

    for(int i = 0; i < number; i++)
    data[i] = Wire.read();
}