#include "globals.h"
#include "drivers/imu.h"
#include "systems/control/pid.h"
#include "systems/comms/radio.h"
#include "systems/control/mixer.h"
#include "systems/safety/failsafe.h"
#include "drivers/barometer.h"
#include "drivers/magnetometer.h"
#include <EEPROM.h>

void savePIDs();
void loadPIDs();

struct TestResult {
  bool passed;
  String message;
  int errorCode; // Unique error code for specific failures
};

struct RecoveryState {
  bool active;
  uint8_t retryCount;
  uint32_t lastAttempt;
  int errorCode;
};

TestResult testIMU_SelfTest();
TestResult testMotors_SelfTest();
TestResult testSensorConsistency_SelfTest();
void attemptRecovery(const TestResult& test);
void verifyRecovery();

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
AutotuneState autotuneState = AutotuneState::IDLE;
FailSafeState currentFailSafeState = FailSafeState::FAILSAFE_NONE;
bool motorsArmed = false;
SensorData sensorData;
float fusedVelocityX = 0.0;
float fusedVelocityY = 0.0;
float currentRollRate, currentPitchRate, currentYawRate;
unsigned long lastCommandRxTime = 0;

// Autotune variables
float autotuneKu;
float autotuneTu;
int autotuneStep = 0;
float autotuneP = 0;
long autotuneStartTime = 0;
float autotuneLastPitch = 0;
int autotuneCycleCount = 0;

void updateAutotune() {
    if (autotuneState == AutotuneState::IDLE) {
        return;
    }

    long now = millis();
    if (autotuneState == AutotuneState::TUNING_ROLL) {
        // TODO: Implement roll tuning
        autotuneState = AutotuneState::TUNING_PITCH;
    } else if (autotuneState == AutotuneState::TUNING_PITCH) {
        // Ziegler-Nichols tuning method
        if (autotuneStep == 0) {
            // Start of tuning
            pidPitch.ki = 0;
            pidPitch.kd = 0;
            autotuneP = 0;
            autotuneStartTime = now;
            autotuneLastPitch = sensorData.pitch;
            autotuneCycleCount = 0;
            autotuneStep = 1;
        } else if (autotuneStep == 1) {
            // Increase P until oscillation
            autotuneP += 0.1;
            pidPitch.kp = autotuneP;
            if (now - autotuneStartTime > 5000) {
                // Check for oscillation
                if (sensorData.pitch > autotuneLastPitch + 2 && autotuneCycleCount > 5) {
                    autotuneKu = autotuneP;
                    autotuneTu = (now - autotuneStartTime) / autotuneCycleCount;
                    autotuneStep = 2;
                } else {
                    autotuneStartTime = now;
                    autotuneCycleCount = 0;
                }
            }
            if (sensorData.pitch > autotuneLastPitch) {
                autotuneCycleCount++;
            }
            autotuneLastPitch = sensorData.pitch;
        } else if (autotuneStep == 2) {
            // Calculate PID gains
            pidPitch.kp = 0.6 * autotuneKu;
            pidPitch.ki = 2 * pidPitch.kp / autotuneTu;
            pidPitch.kd = pidPitch.kp * autotuneTu / 8;
            autotuneState = AutotuneState::TUNING_YAW;
        }
    } else if (autotuneState == AutotuneState::TUNING_YAW) {
        // TODO: Implement yaw tuning
        autotuneState = AutotuneState::COMPLETE;
savePIDs();
    }
}

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

void savePIDs() {
    EEPROM.put(0, pidRoll);
    EEPROM.put(sizeof(PID), pidPitch);
    EEPROM.put(2 * sizeof(PID), pidYaw);
    EEPROM.put(3 * sizeof(PID), pidRollRate);
    EEPROM.put(4 * sizeof(PID), pidPitchRate);
    EEPROM.put(5 * sizeof(PID), pidYawRate);
    EEPROM.commit();
}

void loadPIDs() {
    EEPROM.get(0, pidRoll);
    EEPROM.get(sizeof(PID), pidPitch);
    EEPROM.get(2 * sizeof(PID), pidYaw);
    EEPROM.get(3 * sizeof(PID), pidRollRate);
    EEPROM.get(4 * sizeof(PID), pidPitchRate);
    EEPROM.get(5 * sizeof(PID), pidYawRate);
}

void initMotors() {
    // TODO: Implement motor initialization
}

TestResult testIMU_SelfTest() {
    // TODO: Implement IMU self-test
    return {true, "IMU OK", 0};
}

TestResult testMotors_SelfTest() {
    // TODO: Implement motors self-test
    return {true, "Motors OK", 0};
}

TestResult testSensorConsistency_SelfTest() {
    // TODO: Implement sensor consistency self-test
    return {true, "Sensors consistent", 0};
}

void attemptRecovery(const TestResult& test) {
    // TODO: Implement recovery attempt
}

void verifyRecovery() {
    // TODO: Implement recovery verification
}

void setup() {
    Serial.begin(115200);
    EEPROM.begin(512);

    // Initialize PID controllers
    loadPIDs();
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

    // Run self-tests
    testIMU_SelfTest();
    testMotors_SelfTest();
    testSensorConsistency_SelfTest();
}

void loop() {
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    
    // Main 100Hz loop
    if (now - lastUpdate >= 10) {
        updateIMU();
        updateRadio();
        if (autotuneState != AutotuneState::IDLE) {
            updateAutotune();
        }
        flightControllerUpdate();
        lastUpdate = now;
    }
    
    // Safety checks
    checkFailsafes();

    if (currentFailSafeState != FailSafeState::FAILSAFE_NONE) {
        motorsArmed = false;
    }
}