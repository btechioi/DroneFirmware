#pragma once
#include <Arduino.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <LoRa.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_HMC5883_U.h>

// Pin Definitions
#define MOTOR_PIN_1 12
#define MOTOR_PIN_2 13
#define MOTOR_PIN_3 14
#define MOTOR_PIN_4 15
#define LORA_SS 5
#define LORA_RST 6
#define LORA_DIO0 7

// System Constants
constexpr float MIN_THROTTLE = 1000.0f;
constexpr float MAX_THROTTLE = 2000.0f;
constexpr uint32_t SIGNAL_LOSS_TIMEOUT_MS = 1000;

// Global Instances
extern MPU6050 mpu;
extern TinyGPSPlus gps;
extern Adafruit_BMP085 bmp;
extern Adafruit_HMC5883_Unified mag;

// System State
enum class FlightMode { MANUAL, ALTITUDE_HOLD, POSITION_HOLD, RETURN_HOME, EMERGENCY };
extern FlightMode currentFlightMode;
extern bool motorsArmed;

// Sensor Data
struct SensorData {
    float roll, pitch, yaw;
    float altitude;
    float batteryVoltage;
};
extern SensorData sensorData;