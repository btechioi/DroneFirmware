#include "imu.h"
#include <MadgwickAHRS.h>

Madgwick filter;

void initIMU() {
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while(1);
    }
    
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);
    calibrateIMU();
}

void updateIMU() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert to proper units
    float accel[3] = {ax/16384.0f, ay/16384.0f, az/16384.0f};
    float gyro[3] = {gx/131.0f, gy/131.0f, gz/131.0f};
    
    // Update filter
    filter.update(gyro[0], gyro[1], gyro[2], 
                 accel[0], accel[1], accel[2]);
    
    // Update global state
    sensorData.roll = filter.getRoll();
    sensorData.pitch = filter.getPitch();
    sensorData.yaw = filter.getYaw();
}

void calibrateIMU() {
    // Simple calibration - average 1000 samples
    int32_t ax = 0, ay = 0, az = 0;
    int32_t gx = 0, gy = 0, gz = 0;
    
    for(int i = 0; i < 1000; i++) {
        int16_t _ax, _ay, _az, _gx, _gy, _gz;
        mpu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);
        ax += _ax; ay += _ay; az += _az;
        gx += _gx; gy += _gy; gz += _gz;
        delay(2);
    }
    
    mpu.setXAccelOffset(-ax/1000);
    mpu.setYAccelOffset(-ay/1000);
    mpu.setZAccelOffset(-(az/1000-16384));
    mpu.setXGyroOffset(-gx/1000);
    mpu.setYGyroOffset(-gy/1000);
    mpu.setZGyroOffset(-gz/1000);
}