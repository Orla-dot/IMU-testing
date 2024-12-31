#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>


// decide the MPU 
enum MPU_SELECT {
    MPU_6050 = 0x00,
    MPU_6500 = 0x01,
    MPU_9250 = 0x02,
};

// accel configuration select
enum ACCEL_FS_SEL {
    ACCEL_A2G  = 0x00,
    ACCEL_A4G  = 0x01,
    ACCEL_A8G  = 0x02,
    ACCEL_A16G = 0x03,
};
enum ACCEL_LPF_SEL {
    ACCEL_460HZ = 0x00,
    ACCEL_44HZ  = 0x03,
    ACCEL_5HZ   = 0x06,
};
// gyro configuration select
enum GYRO_FS_SEL {
    GYRO_G250DPS  = 0x00,
    GYRO_G500DPS  = 0x01,
    GYRO_G1000DPS = 0x02,
    GYRO_G2000DPS = 0x03,
};
enum GYRO_LPF_SEL {
    ACCEL_250HZ = 0x00,
    ACCEL_44HZ  = 0x03,
    ACCEL_5HZ   = 0x06,
};
// magnetometer configuration select
enum MAG_LPF_SEL {
    MAG_POWER_DOWN       = 0x00,
    MAG_LOW_SM_14BITS    = 0x01,
    MAG_LOW_8HZ_14BITS   = 0x02,
    MAG_LOW_100HZ_14BITS = 0x06,
    MAG_HIGH_SM_16BITS   = 0x10,
    MAG_HIGH_8HZ_16BITS  = 0x12,
    MAG_HIGH_100HZ_16BITS= 0x16,
};
// digital low pass filter
enum DLPF_CONFIG_RATE {
    DLPF_260HZ = 0x00,
    DLPF_184HZ = 0x01,
    DLPF_94HZ  = 0x02,
    DLPF_44HZ  = 0x03,
    DLPF_21HZ  = 0x04,
    DLPF_10HZ  = 0x05,
    DLPF_5HZ   = 0x06,
};



class IMU
{
    private:
        float accelX, accelY, accelZ;
        float gyroX, gyroY, gyroZ;
        float magX, magY, magZ;

        MPU_SELECT mpuSelected;

        ACCEL_FS_SEL accelFsConfig;
        GYRO_FS_SEL  gyroFsConfig ;




        // divices address
        #define MPU_ADDR      0x68
        #define AK8963_ADDR   0x0C
        // data address
        #define ACCEL_ADDR    0x3B
        #define GYRO_ADDR     0x43
        #define MAG_ADDR      0x03
        // configurations address
        #define DLPF_CFG   0x1A
        #define ACCEL_CFG  0x1B
        #define GYRO_CFG   0x1C
        #define MAG_CFG    0x0A
        // 
        #define AK8963_CNTL1  0x0A
        #define AK8963_XOUT_L 0x03
        #define INT_PIN_CFG   0x37
        #define PWR_MGMT_1    0x6B


        void byteWrite(uint8_t diviceAddress, uint8_t registerAddress, uint8_t data);
        int  byteRead(uint8_t diviceAddress, uint8_t registerAddress);
        void byteRead(uint8_t diviceAddress, uint8_t registerAddress, uint8_t number, uint8_t *data);

    public:
        IMU(MPU_SELECT mpuSelect);
        
        void setLowPassFilter(DLPF_CONFIG_RATE rateDLPF, ACCEL_LPF_SEL rateAccel, GYRO_LPF_SEL rateGyro, MAG_LPF_SEL rateMag);

        void update();

        void updateAccel();
        void updateGyro();
        void updateMag();

        void getAccel(float *xA, float *yA, float *zA);
        void getGyro(float *xG, float *yG, float *zG);
        void getMag(float *xM, float *yM, float *zM);
};


#endif // IMU_H