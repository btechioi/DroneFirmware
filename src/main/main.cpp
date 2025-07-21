#include "globals.h"
#include "drivers/imu.h"
#include "systems/control/pid.h"
#include "systems/comms/radio.h"
#include "systems/control/mixer.h"
#include "systems/safety/failsafe.h"
#include "drivers/barometer.h"
#include "drivers/magnetometer.h"

// Global instances
MPU6050 mpu;
TinyGPSPlus gps;
Adafruit_BMP085 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// PID Instances
PID pidAltitude;
PID pidVelocityX;
PID pidVelocityY;
PID pidRoll;
PID pidPitch;
PID pidYaw;
PID pidRollRate;
PID pidPitchRate;
PID pidYawRate;

// Control Targets
float targetVelocityX = 0;
float targetVelocityY = 0;
float targetAltitude = 0;
float targetRoll = 0;
float targetPitch = 0;
float targetYaw = 0;
float targetRollRate = 0;
float targetPitchRate = 0;
float targetYawRate = 0;
float baseThrottle = 1000;

// System state
FlightMode currentFlightMode = FlightMode::MANUAL;
bool motorsArmed = false;
SensorData sensorData;
float fusedVelocityX = 0.0;
float fusedVelocityY = 0.0;
float currentRollRate, currentPitchRate, currentYawRate;

void flightControllerUpdate() {
    // updateFlightMode();

    // Attitude Loops
    targetRollRate = computePID(&pidRoll, targetRoll - sensorData.roll);
    targetPitchRate = computePID(&pidPitch, targetPitch - sensorData.pitch);
    targetYawRate = computePID(&pidYaw, targetYaw - sensorData.yaw);

    // Angular Rate Loops
    float rollCorrection = computePID(&pidRollRate, targetRollRate - currentRollRate);
    float pitchCorrection = computePID(&pidPitchRate, targetPitchRate - currentPitchRate);
    float yawCorrection = computePID(&pidYawRate, targetYawRate - currentYawRate);

    updateMotors(baseThrottle, rollCorrection, pitchCorrection, yawCorrection);
}

void initMotors() {
    // TODO: Implement motor initialization
}

void setup() {
    Serial.begin(115200);

    // Initialize PID controllers
    pidAltitude.kp = 100.0; pidAltitude.ki = 10.0; pidAltitude.kd = 50.0; pidAltitude.outputLimit = 500.0;
    pidVelocityX.kp = 0.5; pidVelocityX.ki = 0.0; pidVelocityX.kd = 0.0; pidVelocityX.outputLimit = 20.0;
    pidVelocityY.kp = 0.5; pidVelocityY.ki = 0.0; pidVelocityY.kd = 0.0; pidVelocityY.outputLimit = 20.0;
    pidRoll.kp = 4.0; pidRoll.ki = 0.0; pidRoll.kd = 0.0; pidRoll.outputLimit = 200.0;
    pidPitch.kp = 4.0; pidPitch.ki = 0.0; pidPitch.kd = 0.0; pidPitch.outputLimit = 200.0;
    pidYaw.kp = 6.0; pidYaw.ki = 0.0; pidYaw.kd = 0.0; pidYaw.outputLimit = 200.0;
    pidRollRate.kp = 0.5; pidRollRate.ki = 0.0; pidRollRate.kd = 0.0; pidRollRate.outputLimit = 500.0;
    pidPitchRate.kp = 0.5; pidPitchRate.ki = 0.0; pidPitchRate.kd = 0.0; pidPitchRate.outputLimit = 500.0;
    pidYawRate.kp = 0.8; pidYawRate.ki = 0.0; pidYawRate.kd = 0.0; pidYawRate.outputLimit = 500.0;
    
    // Initialize systems
    initIMU();
    initRadio();
    initMotors();
    initBarometer();
    initMagnetometer();
    
    // Wait for sensors to stabilize
    delay(1000);
    
    Serial.println("Drone initialized");
}

void loop() {
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    
    // Main 100Hz loop
    if (now - lastUpdate >= 10) {
        updateIMU();
        updateRadio();
        flightControllerUpdate();
        lastUpdate = now;
    }
    
    // Safety checks
    checkFailsafes();
}