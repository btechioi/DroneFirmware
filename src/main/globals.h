#pragma once
#include <Arduino.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <LoRa.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_HMC5883_U.h>
#include "config/pins.h"
#include "config/constants.h"



// Global Instances
extern MPU6050 mpu;
extern TinyGPSPlus gps;
extern Adafruit_BMP085 bmp;
extern Adafruit_HMC5883_Unified mag;

// System State
enum class FlightMode { MANUAL, ALTITUDE_HOLD, POSITION_HOLD, RETURN_HOME, EMERGENCY };
enum class AutotuneState { IDLE, TUNING_ROLL, TUNING_PITCH, TUNING_YAW, COMPLETE, FAILED };
extern FlightMode currentFlightMode;
extern AutotuneState autotuneState;
extern bool motorsArmed;

// Sensor Data
struct SensorData {
    float roll, pitch, yaw;
    float altitude;
    float batteryVoltage;
};
extern SensorData sensorData;