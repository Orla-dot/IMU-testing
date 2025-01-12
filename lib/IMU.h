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
enum ACCEL_DLPF_SEL {
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
enum GYRO_DLPF_SEL {
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
    MAG_HIGH_SM_16BITS   = 0x04,
    MAG_HIGH_8HZ_16BITS  = 0x08,
    MAG_HIGH_100HZ_16BITS= 0x0F,
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

        MPU_SELECT mpuSelected;


        // divices address
        #define MPU_ADDR      0x68
        #define AK8963_ADDR   0x0C

        // data address
        #define ACCEL_ADDR    0x3B
        #define GYRO_ADDR     0x43
        #define MAG_ADDR      0x03

        // configurations address
        //      Full-Scale Range
        #define ACCEL_CONFIG  0x1C // Accel
        #define GYRO_CONFIG   0x1B // Gyro
        //      Digital Low-Pass Filter
        #define CONFIG        0x1A // Gyro
        #define ACCEL_CONFIG2 0x1D // Accel
        #define MAG_CONFIG    0x0A // Mag

        // magnetometer configurations address
        #define AK8963_CNTL1  0x0A
        #define AK8963_XOUT_L 0x03
        #define INT_PIN_CFG   0x37
        #define PWR_MGMT_1    0x6B


        void byteWrite(uint8_t diviceAddress, uint8_t registerAddress, uint8_t data);
        int  byteRead(uint8_t diviceAddress, uint8_t registerAddress);
        void byteRead(uint8_t diviceAddress, uint8_t registerAddress, uint8_t number, uint8_t *data);

    public:
        IMU(MPU_SELECT mpuSelect);
        
        void setLowPassFilter(ACCEL_DLPF_SEL rateAccel, GYRO_DLPF_SEL rateGyro, MAG_LPF_SEL rateMag);

        void update();
        void updateAccel();
        void updateGyro();
        void updateMag();

        void getAccel(float *xA, float *yA, float *zA);
        void getGyro(float *xG, float *yG, float *zG);
        void getMag(float *xM, float *yM, float *zM);
};


#endif // IMU_H