#include "globals.h"
#include "drivers/imu.h"
#include "systems/control/pid.h"
#include "systems/comms/radio.h"
#include "systems/control/mixer.h"
#include "systems/safety/failsafe.h"
#include "drivers/barometer.h"
#include "drivers/magnetometer.h"
#include <EEPROM.h>

void logMessage(const char* message);
void sendTextMessage(const char* message);
void sendTelemetryDataRadio();
void setupPWM();
void setupIMU();
void calibrateIMU();
void setupBarometer();
void setupMagnetometer();
void calibrateMagnetometer();
void setupOpticalFlow();
void setupUltrasonic();
void setupGPS();
void setupBatteryMonitoring();
void setupRadio();
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len);
void connectToTrainer();
void sendTrainingData();
void processPIDUpdates();
void updateMotors(float throttle, float rollCorrection, float pitchCorrection, float yawCorrection);
bool checkMotorPerformanceInFlight();
void processRadioCommand(const uint8_t* data, size_t len);
void updateFlightMode();
float readBatteryVoltage();
float readCurrentSensor();
float calculateBearingTo(float lat1, float lon1, float lat2, float lon2);
float calculateDistanceTo(float lat1, float lon1, float lat2, float lon2);
void processWaypointNavigation();
TestResult testIMU_SelfTest();
TestResult testMotors_SelfTest();
TestResult testSensorConsistency_SelfTest();
void attemptRecovery(const TestResult& test);
void verifyRecovery();
void reconfigureMotorMix(int failedMotor);
void enterDegradedMode(int errorCode);
void enableTricopterMode(int failedMotor);
void useGPSHeadingOnly();
void useBaroAltitudeOnly();
void logToSD(const char* entry);
void logRecovery(const char* message);
void emergencyLand();
void setupLEDsBuzzer();
void setLEDColor(uint8_t ledIndex, uint8_t r, uint8_t g, uint8_t b);
void setLEDStatus(bool status, bool warning, bool error, bool debug_active);
void displayLEDStates();
void playBuzzer(int duration_ms, int frequency_hz);

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
FlightState state; // Consolidated state
RecoveryState recovery = {false, 0, 0, 0}; // Initialize recovery state
bool motorsArmed = false;
bool inDegradedMode = false; // Flag for degraded flight mode
int failedMotorIndex = -1; // -1 means no motor failed, 0-3 for specific motor
float currentMotorAmps = 0.0; // New: Total motor current
bool debugMode = false; // Flag to enable/disable debug telemetry

// Sensor health status (consolidated with existing error flags)
bool hasMPU6050 = false;
bool hasBMP180 = false;
bool hasHMC5883L = false;
bool hasPMW3901 = false;
bool hasHCSR04 = false;
bool hasGPS = false;

bool imuError = true; // Assume error until healthy
bool baroError = true;
bool magError = true;
bool flowError = true;
bool ultrasonicError = true;
bool gpsError = true;
bool batteryLowError = false;
bool batteryCriticalError = false;
bool geoFenceViolationError = false;
bool sdHealthy = false; // SD card health
bool radioHealthy = false; // LoRa/ESP-NOW health (implicitly handled by setupRadio success)
bool motorPerformanceError = false; // New: In-flight motor performance error

// IMU (MPU6050)
Madgwick filter;
float roll, pitch, yaw; // Current attitude angles (degrees)
float currentRollRate, currentPitchRate, currentYawRate; // Current angular rates (degrees/second)

// Barometer (BMP180)
float altitude = 0;       // Current altitude (in meters)
float initialPressure = 0; // Pressure at startup for relative altitude calculation

// Magnetometer (HMC5883L)
float magX, magY, magZ; // Calibrated magnetic field data (microTeslas)
float mag_offset_x = 0.0;
float mag_offset_y = 0.0;
float mag_offset_z = 0.0;
float mag_scale_x = 1.0;
float mag_scale_y = 1.0;
float mag_scale_z = 1.0;
bool magCalibrated = false; // Flag to indicate if magnetometer has been calibrated

// Optical Flow (PMW3901)
int16_t opticalFlowDeltaX = 0; // Accumulated X displacement (raw counts/pixels)
int16_t opticalFlowDeltaY = 0; // Accumulated Y displacement (raw counts/pixels)

// Ultrasonic Sensor (HC-SR04)
float ultrasonicDistance = 0; // Distance in meters

// GPS
float homeLatitude = 0; // Stored home latitude for RTH
float homeLongitude = 0; // Stored home longitude for RTH
bool homePositionSet = false; // Flag to indicate if home position has been set

// Battery Monitoring
float batteryVoltage = 0.0;
float batteryPercentage = 0.0;

// Sensor Fusion for Velocity (Complementary Filter)
float fusedVelocityX = 0.0;
float fusedVelocityY = 0.0;

// Waypoint Navigation
Waypoint waypoints[MAX_WAYPOINTS];
uint8_t numWaypoints = 0;
uint8_t currentWaypointIndex = 0;

// LED States for Multiplexing
uint8_t ledStates[4][3]; // [LED_Index][R, G, B] - stores 0-255 values
uint8_t currentMultiplexedLED = 0; // Current LED being displayed in the multiplexing cycle
const int CATHODE_PINS[4] = {18, 19, 21, 2};

unsigned long lastCommandRxTime = 0;
int motorCommands[4];
FlightMode currentFlightMode = FlightMode::MANUAL;
FailSafeState currentFailSafeState = FailSafeState::FAILSAFE_NONE;
AutotuneState autotuneState = AutotuneState::IDLE;

// Autotune variables
float autotuneKu;
float autotuneTu;
int autotuneStep = 0;
float autotuneP = 0;
long autotuneStartTime = 0;
float autotuneLastPitch = 0;
int autotuneCycleCount = 0;

void logMessage(const char* message) {
    Serial.println(message);
}

void sendTextMessage(const char* message) {
    // TODO: Implement sendTextMessage
}

void sendTelemetryDataRadio() {
    // TODO: Implement sendTelemetryDataRadio
}

void updateMotors(float throttle, float rollCorrection, float pitchCorrection, float yawCorrection) {
  // If not armed, keep motors at minimum throttle and turn off PWM
  if (!motorsArmed) { // Using motorsArmed from new code
    for (int i = 0; i < 4; i++) {
      motorCommands[i] = MIN_THROTTLE;
      ledcWrite(i, 0); // Ensure motors are truly off
    }
    return;
  }

  if (inDegradedMode && failedMotorIndex != -1) {
    // Degraded mode: one motor is considered failed
    // This is a simplified heuristic mixing for a quadcopter trying to fly with 3 motors.
    // A true tricopter conversion is more complex and often involves a servo for yaw.
    // Here, we try to compensate with the remaining motors.

    float activeMotorThrottle[4];

    // Distribute base throttle among the 3 active motors
    // This assumes the drone can still lift itself with 3 motors at this throttle.
    float baseThrustPerActiveMotor = throttle / 3.0f;

    // Initialize all to base, then set failed to MIN_THROTTLE
    for (int i = 0; i < 4; i++) {
        activeMotorThrottle[i] = baseThrustPerActiveMotor;
    }
    activeMotorThrottle[failedMotorIndex] = MIN_THROTTLE; // Failed motor is off

    // Apply corrections to active motors. This is a heuristic.
    // The coefficients (e.g., 1.5, 0.5) would need careful tuning.
    // This assumes an X-quad configuration.
    // Motor 0: FR, Motor 1: FL, Motor 2: BL, Motor 3: BR

    switch (failedMotorIndex) {
        case 0: // Front Right (FR) failed
            // Active: FL (1), BL (2), BR (3)
            // FL (1): +Pitch, -Roll, -Yaw (needs to compensate for FR's +Roll, +Pitch, +Yaw)
            activeMotorThrottle[1] += (pitchCorrection * 1.0) + (rollCorrection * -0.5) + (yawCorrection * -1.0);
            // BL (2): -Pitch, -Roll, +Yaw
            activeMotorThrottle[2] += (pitchCorrection * -1.0) + (rollCorrection * -0.5) + (yawCorrection * 1.0);
            // BR (3): -Pitch, +Roll, -Yaw
            activeMotorThrottle[3] += (pitchCorrection * -1.0) + (rollCorrection * 1.0) + (yawCorrection * -1.0);
            break;
        case 1: // Front Left (FL) failed
            // Active: FR (0), BL (2), BR (3)
            // FR (0): +Pitch, +Roll, +Yaw (needs to compensate for FL's +Pitch, -Roll, -Yaw)
            activeMotorThrottle[0] += (pitchCorrection * 1.0) + (rollCorrection * 0.5) + (yawCorrection * 1.0);
            // BL (2): -Pitch, -Roll, +Yaw
            activeMotorThrottle[2] += (pitchCorrection * -1.0) + (rollCorrection * -1.0) + (yawCorrection * 1.0);
            // BR (3): -Pitch, +Roll, -Yaw
            activeMotorThrottle[3] += (pitchCorrection * -1.0) + (rollCorrection * 1.0) + (yawCorrection * -1.0);
            break;
        case 2: // Back Left (BL) failed
            // Active: FR (0), FL (1), BR (3)
            // FR (0): +Pitch, +Roll, +Yaw
            activeMotorThrottle[0] += (pitchCorrection * 1.0) + (rollCorrection * 1.0) + (yawCorrection * 1.0);
            // FL (1): +Pitch, -Roll, -Yaw (needs to compensate for BL's -Pitch, -Roll, +Yaw)
            activeMotorThrottle[1] += (pitchCorrection * 1.0) + (rollCorrection * -0.5) + (yawCorrection * -1.0);
            // BR (3): -Pitch, +Roll, -Yaw
            activeMotorThrottle[3] += (pitchCorrection * -1.0) + (rollCorrection * 1.0) + (yawCorrection * -1.0);
            break;
        case 3: // Back Right (BR) failed
            // Active: FR (0), FL (1), BL (2)
            // FR (0): +Pitch, +Roll, +Yaw
            activeMotorThrottle[0] += (pitchCorrection * 1.0) + (rollCorrection * 1.0) + (yawCorrection * 1.0);
            // FL (1): +Pitch, -Roll, -Yaw
            activeMotorThrottle[1] += (pitchCorrection * 1.0) + (rollCorrection * -1.0) + (yawCorrection * -1.0);
            // BL (2): -Pitch, -Roll, +Yaw (needs to compensate for BR's -Pitch, +Roll, -Yaw)
            activeMotorThrottle[2] += (pitchCorrection * -1.0) + (rollCorrection * -0.5) + (yawCorrection * 1.0);
            break;
        default: // Should not happen if failedMotorIndex is valid
            // Fallback to normal mixing or disarm if this state is reached unexpectedly
            break;
    }

    // Assign to motorCommands and constrain
    for (int i = 0; i < 4; i++) {
        motorCommands[i] = constrain(activeMotorThrottle[i], MIN_THROTTLE, MAX_THROTTLE);
        ledcWrite(i, map(motorCommands[i], MIN_THROTTLE, MAX_THROTTLE, 0, 4095));
    }

  } else {
    // Normal quadcopter mixing (X configuration)
    motorCommands[0] = throttle + rollCorrection + pitchCorrection + yawCorrection; // FR: +Roll, +Pitch, +Yaw (clockwise)
    motorCommands[1] = throttle - rollCorrection + pitchCorrection - yawCorrection; // FL: -Roll, +Pitch, -Yaw (counter-clockwise)
    motorCommands[2] = throttle - rollCorrection - pitchCorrection + yawCorrection; // BL: -Roll, -Pitch, +Yaw (clockwise)
    motorCommands[3] = throttle + rollCorrection - pitchCorrection - yawCorrection; // BR: +Roll, -Pitch, -Yaw (counter-clockwise)

    for (int i = 0; i < 4; i++) {
        motorCommands[i] = constrain(motorCommands[i], MIN_THROTTLE, MAX_THROTTLE);
        ledcWrite(i, map(motorCommands[i], MIN_THROTTLE, MAX_THROTTLE, 0, 4095));
    }
  }
}

void loadPIDs() {
    EEPROM.get(0, pidRoll);
    EEPROM.get(sizeof(PID), pidPitch);
    EEPROM.get(2 * sizeof(PID), pidYaw);
    EEPROM.get(3 * sizeof(PID), pidRollRate);
    EEPROM.get(4 * sizeof(PID), pidPitchRate);
    EEPROM.get(5 * sizeof(PID), pidYawRate);
}

void updateFlightMode() {
  // These variables will hold the commanded targets for the inner PID loops.
  // They are initialized with the manual control targets, and then overridden
  // by higher-level flight modes.
  float commandedTargetRoll = targetRoll;
  float commandedTargetPitch = targetPitch;
  float commandedTargetYaw = targetYaw;
  float commandedBaseThrottle = baseThrottle;

  switch (currentFlightMode) {
    case FlightMode::MANUAL:
      // In manual mode, all control inputs (targetRoll, targetPitch, targetYaw, baseThrottle)
      // come directly from the radio commands. No higher-level PID loops are active here.
      // The variables are already set by processRadioCommand.
      break;

    case FlightMode::ALTITUDE_HOLD:
      // Altitude hold - maintain targetAltitude using PID.
      // Roll, Pitch, Yaw are still manually commanded.
      if (motorsArmed) {
        float throttleCorrection = computePID(&pidAltitude, targetAltitude - altitude);
        commandedBaseThrottle = constrain(baseThrottle + throttleCorrection, MIN_THROTTLE, MAX_THROTTLE);
      } else {
        commandedBaseThrottle = MIN_THROTTLE;
      }
      // Velocity PID loops are NOT active here.
      break;

    case FlightMode::POSITION_HOLD:
      // Position hold - maintain current position by targeting zero velocity.
      // Altitude hold is also active.
      if (motorsArmed) {
        // Target zero velocities for position hold
        targetVelocityX = 0;
        targetVelocityY = 0;

        // Velocity PID loops generate commanded Roll/Pitch angles
        commandedTargetPitch = computePID(&pidVelocityX, targetVelocityX - fusedVelocityX);
        commandedTargetRoll = computePID(&pidVelocityY, targetVelocityY - fusedVelocityY);

        // Altitude PID loop for vertical stability
        float throttleCorrection = computePID(&pidAltitude, targetAltitude - altitude);
        commandedBaseThrottle = constrain(baseThrottle + throttleCorrection, MIN_THROTTLE, MAX_THROTTLE);
      } else {
        commandedBaseThrottle = MIN_THROTTLE;
      }
      break;

    case FlightMode::RETURN_TO_HOME:
      // Return to home - navigate back to home position.
      if (motorsArmed && !gpsError && homePositionSet) { // Ensure GPS is healthy and home is set
        float distanceToHome = calculateDistanceTo(
          state.latitude, state.longitude,
          homeLatitude, homeLongitude);

        float courseToHome = calculateBearingTo(
          state.latitude, state.longitude,
          homeLatitude, homeLongitude);

        // Climb to RTH_ALTITUDE_M if currently lower
        if (altitude < RTH_ALTITUDE_M - 0.5) { // 0.5m buffer
            targetAltitude = RTH_ALTITUDE_M;
            targetVelocityX = 0; // Hover while climbing
            targetVelocityY = 0;
        } else if (distanceToHome > 1.0) { // If more than 1m away from home
          // Proportional speed towards home, constrained to a reasonable range
          float speed = constrain(distanceToHome * 0.2, 0.5, 2.0); // Min 0.5m/s, Max 2.0m/s
          targetVelocityX = speed * cos(courseToHome * DEG_TO_RAD);
          targetVelocityY = speed * sin(courseToHome * DEG_TO_RAD);
          targetAltitude = RTH_ALTITUDE_M; // Maintain RTH altitude
        } else {
          // Close enough to home - switch to position hold
          currentFlightMode = FlightMode::POSITION_HOLD;
          targetVelocityX = 0;
          targetVelocityY = 0;
          sendTextMessage("Arrived at home. Switching to Position Hold.");
        }

        // Velocity PID loops generate commanded Roll/Pitch angles
        commandedTargetPitch = computePID(&pidVelocityX, targetVelocityX - fusedVelocityX);
        commandedTargetRoll = computePID(&pidVelocityY, targetVelocityY - fusedVelocityY);

        // Altitude PID loop for vertical stability (maintain targetAltitude)
        float throttleCorrection = computePID(&pidAltitude, targetAltitude - altitude);
        commandedBaseThrottle = constrain(baseThrottle + throttleCorrection, MIN_THROTTLE, MAX_THROTTLE);

      } else {
        // If RTH conditions are not met, fallback to manual or disarm
        sendTextMessage("RTH failed: No GPS, no home, or not armed. Switching to Manual.");
        currentFlightMode = FlightMode::MANUAL;
        commandedBaseThrottle = MIN_THROTTLE;
      }
      break;

    case FlightMode::WAYPOINT_NAV:
      if (motorsArmed && !gpsError && numWaypoints > 0) { // Ensure GPS is healthy and waypoints exist
        processWaypointNavigation(); // This function updates targetVelocityX/Y and targetAltitude

        // Velocity PID loops generate commanded Roll/Pitch angles
        commandedTargetPitch = computePID(&pidVelocityX, targetVelocityX - fusedVelocityX);
        commandedTargetRoll = computePID(&pidVelocityY, targetVelocityY - fusedVelocityY);

        // Altitude PID loop for vertical stability
        float throttleCorrection = computePID(&pidAltitude, targetAltitude - altitude);
        commandedBaseThrottle = constrain(baseThrottle + throttleCorrection, MIN_THROTTLE, MAX_THROTTLE);
      } else {
        sendTextMessage("Waypoint Nav failed: No GPS, invalid fix, or no waypoints. Switching to Manual.");
        currentFlightMode = FlightMode::MANUAL;
        commandedBaseThrottle = MIN_THROTTLE;
      }
      break;

    case FlightMode::EMERGENCY_LAND:
      // Controlled descent to 0 altitude
      if (motorsArmed) {
        targetAltitude = max(0.0f, targetAltitude - (LANDING_DESCENT_RATE * (CONTROL_UPDATE_INTERVAL / 1000.0f))); // Decrease target alt
        if (altitude < 0.1 && abs(fusedVelocityX) < 0.1 && abs(fusedVelocityY) < 0.1) { // Close to ground and low velocity
          motorsArmed = false; // Disarm when landed
          sendTextMessage("Emergency landing complete. Disarmed.");
          logMessage("Emergency landing complete. Disarmed.");
          currentFlightMode = FlightMode::MANUAL; // Reset mode
          currentFailSafeState = FailSafeState::FAILSAFE_NONE; // Clear fail-safe
          setLEDStatus(false, false, false, debugMode); // All LEDs off
          playBuzzer(500, 100); // Long low tone for landing complete
        } else {
          float throttleCorrection = computePID(&pidAltitude, targetAltitude - altitude);
          commandedBaseThrottle = constrain(MIN_THROTTLE + throttleCorrection, MIN_THROTTLE, MAX_THROTTLE); // Use min throttle as base
        }
        // Try to maintain level attitude during landing
        commandedTargetRoll = 0;
        commandedTargetPitch = 0;
        commandedTargetYaw = 0; // Or maintain current yaw
      } else {
        commandedBaseThrottle = MIN_THROTTLE; // Ensure motors are off if disarmed
      }
      break;

    default:
      // Should not happen, but fallback to manual
      currentFlightMode = FlightMode::MANUAL;
      break;
  }

  // Apply the commanded targets to the global variables that the inner loops use
  // This ensures that PID loops always work on the most current target, whether
  // it's from manual input or a higher-level flight mode.
  targetRoll = commandedTargetRoll;
  targetPitch = commandedTargetPitch;
  targetYaw = commandedTargetYaw; // Yaw is currently always manual or heading hold
  baseThrottle = commandedBaseThrottle;
}

void initMotors() {
    // TODO: Implement motor initialization
}

void readIMU_Raw() {
  if (!hasMPU6050) return;
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert gyroscope data to degrees/second
  currentRollRate = gx/131.0f; // X-axis (Roll rate)
  currentPitchRate = gy/131.0f; // Y-axis (Pitch rate)
  currentYawRate = gz/131.0f;  // Z-axis (Yaw rate)

  // Update FlightState angularRate (in rad/s)
  state.angularRate.x = currentRollRate * DEG_TO_RAD;
  state.angularRate.y = currentPitchRate * DEG_TO_RAD;
  state.angularRate.z = currentYawRate * DEG_TO_RAD;
}

void readGPS_Data() {
  if (!hasGPS) return; // Return if GPS not present

  // while (gpsSerial.available() > 0) {
  //   if (gps.encode(gpsSerial.read())) {
  //     // Data is available and parsed. Validity checks done where data is used.
  //   }
  // }

  if (gps.location.isValid()) {
    state.latitude = gps.location.lat();
    state.longitude = gps.location.lng();
    state.velocity.x = gps.speed.mps() * cos(gps.course.rad()); // GPS velocity X (North)
    state.velocity.y = gps.speed.mps() * sin(gps.course.rad()); // GPS velocity Y (East)
  } else {
    state.latitude = 0;
    state.longitude = 0;
    state.velocity.x = 0;
    state.velocity.y = 0;
  }
}

float readBarometerAltitude() {
  if (!hasBMP180) return 0.0; // Return 0 if sensor not present

  float currentPressure = bmp.readPressure(); // Pressure in Pascals
  if (currentPressure != 0) { // Check for valid reading
    baroError = false;
    state.altitude = bmp.readAltitude(initialPressure); // Update FlightState altitude
    return state.altitude;
  }
  baroError = true; // Mark as error if reading is invalid
  return altitude; // Return last known altitude on error
}

float readBatteryVoltage() {
  // int adcValue = analogRead(BATTERY_PIN);
  // float voltage = (adcValue / ADC_MAX_VALUE) * ADC_REF_VOLTAGE / VOLTAGE_DIVIDER_RATIO;
  // // Assuming a 3S LiPo (12.6V max, 9.9V critical)
  // // Adjust these values for your specific battery type/cell count
  // batteryPercentage = map(voltage * 100, 990, 1260, 0, 100); // Map 9.9V-12.6V to 0-100%
  // batteryPercentage = constrain(batteryPercentage, 0, 100);
  // return voltage;
  return 12.0;
}

float readCurrentSensor() {
  // // Read analog value from current sensor
  // int adcValue = analogRead(CURRENT_PIN);
  // // Convert ADC value to voltage
  // float voltage = (adcValue / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
  // // Convert voltage to current based on sensor's V/A scale and offset
  // // (Voltage - Offset) / Scale
  // float current = (voltage - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SCALE;
  // // Ensure current is not negative
  // if (current < 0) current = 0;
  // return current;
  return 0.0;
}

TestResult testIMU_SelfTest() {
  if (!hasMPU6050) return {false, "IMU not detected for test.", 100};

  sendTextMessage("Self-Test: IMU. Keep drone still.");
  unsigned long start = millis();
  int16_t ax, ay, az;
  int stableCount = 0;
  const int requiredStableCount = 10; // Number of consecutive stable readings

  while (millis() - start < SELF_TEST_TIMEOUT) {
    mpu.getAcceleration(&ax, &ay, &az);
    float accelMag = sqrt(ax*ax + ay*ay + az*az) / 16384.0; // Convert to G's

    if (accelMag > 0.9 && accelMag < 1.1) { // Check if magnitude is close to 1G
      stableCount++;
      if (stableCount >= requiredStableCount) {
        imuError = false; // Mark as healthy
        return {true, "IMU OK", 0};
      }
    } else {
      stableCount = 0; // Reset count if unstable
    }
    delay(10); // Small delay for readings
  }
  imuError = true; // Mark as unhealthy
  return {false, "IMU unstable or not responding.", 101};
}

TestResult testMotors_SelfTest() {
  sendTextMessage("Self-Test: Motors. Stand clear!");
  delay(1000); // Give user time to move away

  for (int i = 0; i < 4; i++) {
    char msg[50];
    snprintf(msg, sizeof(msg), "Testing Motor %d...", i + 1);
    sendTextMessage(msg);
    logMessage(msg);

    ledcWrite(i, map(MOTOR_TEST_THROTTLE, MIN_THROTTLE, MAX_THROTTLE, 0, 4095));

    unsigned long start = millis();
    float maxCurrent = 0;
    while (millis() - start < MOTOR_TEST_DURATION) {
      maxCurrent = max(maxCurrent, readCurrentSensor());
      delay(10);
    }
    ledcWrite(i, 0); // Stop motor

    if (maxCurrent < CURRENT_THRESHOLD) {
      char failMsg[100];
      snprintf(failMsg, sizeof(failMsg), "Motor %d failed: Low current (%.2fA).", i + 1, maxCurrent);
      logMessage(failMsg);
      return {false, failMsg, 110 + i}; // Error code 110-113 for specific motor
    }
    delay(300); // Pause between motor tests
  }
  return {true, "All motors OK", 0};
}

TestResult testSensorConsistency_SelfTest() {
  sendTextMessage("Self-Test: Sensor Consistency (GPS/Baro/Mag).");

  // Check Baro/GPS altitude consistency
  if (hasGPS && hasBMP180) {
    // Wait for a valid GPS fix if not already available
    unsigned long start = millis();
    while (!gps.location.isValid() && millis() - start < SELF_TEST_TIMEOUT) {
      readGPS_Data(); // Attempt to get GPS data
      delay(100);
    }

    if (gps.location.isValid() && gps.satellites.value() >= 4) {
      float baroAlt = readBarometerAltitude(); // This updates 'altitude' and 'state.altitude'
      float gpsAlt = gps.altitude.meters();

      if (abs(baroAlt - gpsAlt) > 20.0) { // If altitude difference is significant (20m)
        return {false, "Altitude mismatch (Baro/GPS).", 201};
      }
    } else {
      // If no valid GPS fix, cannot do GPS/Baro consistency check, but not a failure.
      // We'll proceed to check heading if possible.
    }
  }

  // Check GPS/Mag heading consistency (if both are available and calibrated)
  if (hasGPS && hasHMC5883L && magCalibrated && gps.location.isValid() && gps.speed.isValid() && gps.speed.mps() > 1.0) { // Need some speed for reliable GPS course
    // readMagnetometer(magX, magY, magZ); // Ensure magX,Y,Z are updated
    // Calculate IMU/Mag heading (simplified 2D for comparison)
    float imuMagHeadingRad = atan2(magY, magX); // Yaw from magnetometer
    float gpsHeadingRad = gps.course.rad();

    // Normalize angles to -PI to PI
    imuMagHeadingRad = fmod(imuMagHeadingRad + PI, 2 * PI) - PI;
    gpsHeadingRad = fmod(gpsHeadingRad + PI, 2 * PI) - PI;

    float headingDiff = abs(imuMagHeadingRad - gpsHeadingRad);
    if (headingDiff > PI) headingDiff = 2 * PI - headingDiff; // Handle wrap-around

    if (headingDiff * 180 / PI > 30.0) { // If heading difference is significant (30 degrees)
      return {false, "Heading mismatch (Mag/GPS).", 200};
    }
  }

  return {true, "Sensors consistent", 0};
}

void attemptRecovery(const TestResult& test) {
  recovery.active = true;
  recovery.retryCount++;
  recovery.lastAttempt = millis();
  recovery.errorCode = test.errorCode;

  char msg[100];
  snprintf(msg, sizeof(msg), "Attempting recovery for error %d: %s", test.errorCode, test.message.c_str());
  sendTextMessage(msg);
  logRecovery(msg);
  setLEDStatus(false, true, true, debugMode); // Warning + Error LED during recovery attempt

  switch(test.errorCode) {
    case 101: // IMU unstable/not responding
      mpu.initialize(); // Reinitialize IMU
      delay(500); // Give time to reinitialize
      break;

    case 110 ... 113: // Motor failure (110=Motor1, 111=Motor2, etc.) - from self-test
      // This part is for initial self-test failures. In-flight failures handled by checkMotorPerformanceInFlight
      // For self-test motor failures, we don't attempt in-flight recovery, it leads to disarm.
      break;

    case 200: // Heading mismatch (Mag/GPS)
      // Re-calibrate magnetometer or switch to GPS-only heading
      calibrateMagnetometer(); // Try re-calibrating
      break;
    case 201: // Altitude mismatch (Baro/GPS)
      bmp.begin(); // Reinitialize barometer
      delay(500);
      break;
    // Add more cases for other specific error codes
  }
}

void verifyRecovery() {
  TestResult retestResult;

  switch(recovery.errorCode) {
    case 101: retestResult = testIMU_SelfTest(); break;
    case 110 ... 113: retestResult = testMotors_SelfTest(); break; // Only for startup tests
    case 200: retestResult = testSensorConsistency_SelfTest(); break; // Heading mismatch
    case 201: retestResult = testSensorConsistency_SelfTest(); break; // Altitude mismatch
    default: retestResult = {false, "Unknown error during re-test", recovery.errorCode};
  }

  if (retestResult.passed) {
    recovery.active = false;
    recovery.retryCount = 0;
    sendTextMessage("Recovery successful.");
    logRecovery("Recovery successful.");
    setLEDStatus(motorsArmed, inDegradedMode, false, debugMode); // Restore LED status
  } else if (recovery.retryCount >= 3) { // Max 3 retry attempts
    sendTextMessage("Recovery failed after multiple attempts. Initiating appropriate action.");
    logRecovery("Recovery failed. Initiating appropriate action.");
    
    // If recovery failed for a critical sensor (IMU), disarm.
    if (recovery.errorCode == 101) { // IMU failure
      currentFailSafeState = FailSafeState::FAILSAFE_CRITICAL_SENSOR_FAILURE; // Force disarm via main loop
      setLEDStatus(false, false, true, debugMode); // Critical error LED
      playBuzzer(500, 200); // Long low tone for unrecoverable
    } else {
      // For non-critical sensor failures that recovery failed for, enter degraded mode
      enterDegradedMode(recovery.errorCode);
      // If already in degraded mode and still failing, then emergency land.
      if (inDegradedMode) { // Check if degraded mode was already entered and still failing
          emergencyLand();
      }
    }
  }
}

void enterDegradedMode(int errorCode) {
  char msg[100];
  snprintf(msg, sizeof(msg), "Entering degraded mode for error %d.", errorCode);
  sendTextMessage(msg);
  logMessage(msg);
  inDegradedMode = true;
  setLEDStatus(false, true, false, debugMode); // Warning LED for degraded mode
  playBuzzer(100, 500); // Short high tone for degraded mode entry

  switch(errorCode) {
    case 6: // In-flight motor performance issue
      // For in-flight motor performance issues, we don't know which motor failed precisely.
      // We'll just set a flag that updateMotors should use the degraded mixing logic.
      // The enableTricopterMode here is more of a conceptual "prepare for 3-motor flight"
      // and will set failedMotorIndex to an arbitrary value, as the specific motor isn't known.
      // A more advanced system would try to identify the failed motor based on PID output analysis.
      enableTricopterMode(0); // Pass a dummy index, actual logic is in updateMotors
      break;

    case 200: // Heading mismatch (Mag/GPS)
      useGPSHeadingOnly(); // Rely solely on GPS for yaw (if available)
      break;

    case 201: // Altitude mismatch (Baro/GPS)
      useBaroAltitudeOnly(); // Prioritize barometer or other reliable source
      break;
    // Add more degraded modes as needed
  }
}

void enableTricopterMode(int failedMotor) {
  sendTextMessage("Motor failure detected. Attempting degraded flight mode.");
  logMessage("Attempting degraded flight mode due to motor issue.");

  // Set the global failedMotorIndex.
  // If this function is called from checkMotorPerformanceInFlight(),
  // we don't know the exact failed motor, so we might just set a flag
  // that a degraded motor mixing should be used.
  // For now, if called from self-test, it's specific. If from in-flight, it's generic.
  failedMotorIndex = failedMotor; // Store the index if known, otherwise it's a dummy value (e.g., 0)

  // Adjust PID gains to be more aggressive/conservative to try and compensate
  pidRoll.kp *= 1.2; // Try to be more aggressive on roll/pitch
  pidPitch.kp *= 1.2;
  pidYaw.kp *= 0.8; // Yaw might be harder to control, reduce gain

  // Also, consider setting a slightly higher minimum throttle to ensure enough thrust
  // for the remaining motors to lift the drone. (This would need a global MIN_THROTTLE override)
  // MIN_THROTTLE = 1100; // Example: if MIN_THROTTLE was a modifiable variable
}

void useGPSHeadingOnly() {
  sendTextMessage("Using GPS for heading only.");
  logMessage("Switched to GPS-only heading.");
  // Madgwick filter already falls back to 6-DOF if mag is unhealthy.
  // For yaw control, we can try to directly target GPS course if available.
  // This requires the yaw PID to use GPS.course.deg() as its target.
  // For simplicity, we'll just ensure the Madgwick filter ignores mag,
  // and the yaw PID will still try to hold targetYaw, but it will drift
  // if targetYaw is not updated by GPS.
  // A more advanced implementation would involve:
  // targetYaw = gps.course.deg(); // Continuously update targetYaw from GPS
}

void useBaroAltitudeOnly() {
  sendTextMessage("Using Barometer for altitude only.");
  logMessage("Switched to Barometer-only altitude.");
  // This means if ultrasonic was causing issues, we'd ignore it.
  // The altitude fusion already prioritizes ultrasonic at low alt, then barometer.
  // If this function is called due to a barometer/GPS conflict:
  if (baroError && hasGPS && gps.altitude.isValid()) {
    // If barometer is bad, and GPS altitude is good, use GPS altitude.
    // This is a conceptual override.
    // altitude = gps.altitude.meters(); // This would need to be a direct assignment or a new fusion strategy.
    // For now, the existing fusion logic will handle it based on error flags.
  } else if (!baroError) {
    // If barometer is healthy, ensure it's the primary source.
    // This implies potentially de-prioritizing ultrasonic if it was causing issues.
    // hasHCSR04 = false; // Disable ultrasonic if it was the source of conflict.
  }
}

void logToSD(const char* entry) {
  if (!sdHealthy) {
    // Serial.println("SD card not healthy, cannot log."); // Debugging
    return;
  }

  // logFile = SD.open("flightlog.csv", FILE_WRITE);
  // if (logFile) {
  //   logFile.print(millis());
  //   logFile.print(",");
  //   logFile.println(entry);
  //   logFile.close();
  // } else {
  //   // Serial.println("Failed to open flightlog.csv for writing."); // Debugging
  //   sdHealthy = false; // Mark SD as unhealthy if write fails
  //   sendTextMessage("CRITICAL: SD card write failed! Logging disabled.");
  //   setLEDStatus(false, false, true, debugMode); // Error LED
  //   playBuzzer(200, 200); // Low tone for SD error
  // }
}

void logRecovery(const char* message) {
  char entry[200];
  snprintf(entry, sizeof(entry), "RECOVERY,%d,%s", recovery.errorCode, message);
  sendTextMessage(entry); // Send to ground station
  logToSD(entry); // Log to SD card
}

void emergencyLand() {
  if (currentFlightMode != FlightMode::EMERGENCY_LAND) {
    sendTextMessage("EMERGENCY: Initiating controlled landing!");
    logMessage("EMERGENCY: Initiating controlled landing!");
    currentFlightMode = FlightMode::EMERGENCY_LAND;
    targetAltitude = altitude; // Start landing from current altitude
    setLEDStatus(false, true, true, debugMode); // Warning + Error LED for emergency landing
    playBuzzer(100, 800); // Rapid high tone for emergency
  }
}

void setupLEDsBuzzer() {
  // Setup PWM channels for RGB anodes
  // ledcSetup(0, 5000, 8); // Channel 0 for Red, 5kHz, 8-bit resolution (0-255)
  // ledcAttachPin(LED_R_PWM_PIN, 0);
  // ledcSetup(1, 5000, 8); // Channel 1 for Green
  // ledcAttachPin(LED_G_PWM_PIN, 1);
  // ledcSetup(2, 5000, 8); // Channel 2 for Blue
  // ledcAttachPin(LED_B_PWM_PIN, 2);

  // // Setup cathode pins as OUTPUT
  // for (int i = 0; i < 4; i++) {
  //   pinMode(CATHODE_PINS[i], OUTPUT);
  //   digitalWrite(CATHODE_PINS[i], HIGH); // Start with all cathodes OFF (HIGH for common cathode)
  // }

  // // Setup Buzzer pin
  // ledcSetup(3, 2000, 8); // Channel 3 for Buzzer, 2kHz, 8-bit
  // ledcAttachPin(BUZZER_PIN, 3);
  // ledcWrite(3, 0); // Buzzer off initially

  // // Initialize LED states to off
  // for (int i = 0; i < 4; i++) {
  //   ledStates[i][0] = 0; // R
  //   ledStates[i][1] = 0; // G
  //   ledStates[i][2] = 0; // B
  // }
  logMessage("LEDs and Buzzer initialized.");
}

void setLEDColor(uint8_t ledIndex, uint8_t r, uint8_t g, uint8_t b) {
  if (ledIndex < 4) {
    ledStates[ledIndex][0] = r;
    ledStates[ledIndex][1] = g;
    ledStates[ledIndex][2] = b;
  }
}

void setLEDStatus(bool status, bool warning, bool error, bool debug_active) {
  // LED 0: Armed/Disarmed Status (Green/Off)
  setLEDColor(0, status ? 0 : 0, status ? 255 : 0, status ? 0 : 0);

  // LED 1: Warning Status (Yellow/Off)
  setLEDColor(1, warning ? 255 : 0, warning ? 255 : 0, warning ? 0 : 0);

  // LED 2: Error Status (Red/Off)
  setLEDColor(2, error ? 255 : 0, error ? 0 : 0, error ? 0 : 0);

  // LED 3: Debug/Activity (Blue/Off)
  setLEDColor(3, debug_active ? 0 : 0, debug_active ? 0 : 0, debug_active ? 255 : 0);
}

void displayLEDStates() {
  // Turn off current LED's cathode
  // digitalWrite(CATHODE_PINS[currentMultiplexedLED], HIGH);

  // // Move to the next LED
  // currentMultiplexedLED = (currentMultiplexedLED + 1) % 4;

  // // Set global RGB PWM values for the new current LED
  // ledcWrite(0, ledStates[currentMultiplexedLED][0]); // Red
  // ledcWrite(1, ledStates[currentMultiplexedLED][1]); // Green
  // ledcWrite(2, ledStates[currentMultiplexedLED][2]); // Blue

  // // Turn on the new current LED's cathode
  // digitalWrite(CATHODE_PINS[currentMultiplexedLED], LOW);
}

void playBuzzer(int duration_ms, int frequency_hz) {
  // Use ledcWrite for tone generation
  // ledcWriteTone(3, frequency_hz); // Channel 3 for buzzer
  delay(duration_ms);
  // ledcWriteTone(3, 0); // Stop tone
}