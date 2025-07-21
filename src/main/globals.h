#pragma once
#include <Arduino.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <LoRa.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_HMC5883_U.h>
#include "config/pins.h"
#include "config/constants.h"
#include <MadgwickAHRS.h>
#include "systems/control/pid.h"

// Failsafe
#define SIGNAL_LOSS_TIMEOUT_MS 1000 // 1 second without command before signal loss is declared

// Conversions
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.01745329251994329576923690768489 // (PI / 180)
#endif

// PID
#define DERIVATIVE_FILTER_ALPHA 0.7 // Alpha for EMA filter (0.0 to 1.0, higher is more smoothing)

// Waypoint Navigation
#define MAX_WAYPOINTS 10 // Maximum number of waypoints
#define WAYPOINT_REACH_RADIUS_M 0.5 // How close to a waypoint to consider it reached (meters)
#define RTH_ALTITUDE_M 5.0 // Altitude to climb to for RTH if current alt is lower
#define LANDING_DESCENT_RATE 0.5 // m/s for emergency landing

// Self-Test
#define SELF_TEST_TIMEOUT 5000 // Max test duration (ms) for each test
#define MOTOR_TEST_DURATION 500 // Motor test pulse duration (ms)
#define MOTOR_TEST_THROTTLE 1200 // Test throttle value (us)
#define CURRENT_THRESHOLD 0.5 // Minimum motor current (A) to detect motor spin (adjust based on motor/prop size)

// Timing Control
const unsigned long IMU_UPDATE_INTERVAL = 10; // 100Hz
const unsigned long SENSOR_FUSION_INTERVAL = 50; // 20Hz
const unsigned long CONTROL_UPDATE_INTERVAL = 20; // 50Hz
const unsigned long TELEMETRY_UPDATE_INTERVAL = 100; // 10Hz
const unsigned long BATTERY_CHECK_INTERVAL = 1000; // 1Hz
const unsigned long HEALTH_CHECK_INTERVAL = 5000; // 0.2Hz for sensor consistency/degraded mode checks
const unsigned long MOTOR_PERFORMANCE_CHECK_INTERVAL = 200; // 5Hz for in-flight motor performance check
#define PID_SATURATION_THRESHOLD 0.9 // Percentage of PID outputLimit to consider saturation (e.g., 0.9 = 90%)
#define ATTITUDE_ERROR_THRESHOLD 5.0 // Degrees, sustained attitude error to consider a problem
#define SATURATION_DURATION_MS 500 // How long PID must be saturated with high error to trigger fail-safe

// Structures
struct Vector3 {
  float x, y, z;
};

struct Quaternion {
  float w, x, y, z;
};

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

struct FlightState {
  float latitude;    // Current GPS latitude (degrees)
  float longitude;   // Current GPS longitude (degrees)
  float altitude;    // Current altitude (meters, fused)
  Vector3 velocity;  // Fused velocity (m/s)
  float roll;        // Roll angle (degrees)
  float pitch;       // Pitch angle (degrees)
  float yaw;         // Yaw angle (degrees)
  Vector3 angularRate; // Angular rates (rad/s)
};

struct Waypoint {
  float lat;
  float lon;
  float alt;
};

typedef struct __attribute__((packed)) { // __attribute__((packed)) ensures no padding
  float roll;
  float pitch;
  float yaw;
  float altitude;
  float velocityX;
  float velocityY;
  float rollRate;
  float pitchRate;
  float yawRate;
  float baseThrottle;
  float ultrasonicDist; // Raw ultrasonic dist
  int16_t flowX;        // Raw optical flow delta X
  int16_t flowY;        // Raw optical flow delta Y
  uint8_t flightMode;   // Current flight mode
  uint8_t failSafeState; // Current fail-safe state
  uint8_t errors;       // Bitmask for sensor errors (0x01=IMU, 0x02=Baro, 0x04=Mag, 0x08=Flow, 0x10=Ultra, 0x20=GPS, 0x40=BattLow, 0x80=BattCrit)
  float batteryVoltage;
  float batteryPercentage;
  float motorCurrent; // New: Motor current
} TelemetryData_t;

// Global Instances
extern MPU6050 mpu;
extern TinyGPSPlus gps;
extern Adafruit_BMP085 bmp;
extern Adafruit_HMC5883_Unified mag;

// System State
enum class FlightMode { MANUAL, ALTITUDE_HOLD, POSITION_HOLD, RETURN_HOME, WAYPOINT_NAV, EMERGENCY_LAND };
enum class AutotuneState { IDLE, TUNING_ROLL, TUNING_PITCH, TUNING_YAW, COMPLETE, FAILED };
enum class FailSafeState {
  FAILSAFE_NONE = 0,
  FAILSAFE_SIGNAL_LOSS = 1,
  FAILSAFE_CRITICAL_SENSOR_FAILURE = 2,
  FAILSAFE_LOW_BATTERY = 3,
  FAILSAFE_CRITICAL_BATTERY = 4,
  FAILSAFE_GEO_FENCE = 5,
  FAILSAFE_MOTOR_FAILURE = 6,
  FAILSAFE_SENSOR_CONSISTENCY = 7
};
extern FlightMode currentFlightMode;
extern AutotuneState autotuneState;
extern FailSafeState currentFailSafeState;
extern bool motorsArmed;
extern bool inDegradedMode;
extern int failedMotorIndex;
extern float currentMotorAmps;
extern bool debugMode;

// Sensor health
extern bool hasMPU6050;
extern bool hasBMP180;
extern bool hasHMC5883L;
extern bool hasPMW3901;
extern bool hasHCSR04;
extern bool hasGPS;
extern bool imuError;
extern bool baroError;
extern bool magError;
extern bool flowError;
extern bool ultrasonicError;
extern bool gpsError;
extern bool batteryLowError;
extern bool batteryCriticalError;
extern bool geoFenceViolationError;
extern bool sdHealthy;
extern bool radioHealthy;
extern bool motorPerformanceError;

// IMU
extern float roll, pitch, yaw;
extern float currentRollRate, currentPitchRate, currentYawRate;

// Barometer
extern float altitude;
extern float initialPressure;

// Magnetometer
extern float magX, magY, magZ;
extern float mag_offset_x, mag_offset_y, mag_offset_z;
extern float mag_scale_x, mag_scale_y, mag_scale_z;
extern bool magCalibrated;

// Optical Flow
extern int16_t opticalFlowDeltaX, opticalFlowDeltaY;

// Ultrasonic
extern float ultrasonicDistance;

// GPS
extern float homeLatitude, homeLongitude;
extern bool homePositionSet;

// Battery
extern float batteryVoltage, batteryPercentage;

// Sensor Fusion
extern float fusedVelocityX, fusedVelocityY;

// Waypoint Navigation
extern Waypoint waypoints[MAX_WAYPOINTS];
extern uint8_t numWaypoints;
extern uint8_t currentWaypointIndex;

// LED
extern uint8_t ledStates[4][3];
extern uint8_t currentMultiplexedLED;
extern const int CATHODE_PINS[4];
extern unsigned long lastCommandRxTime;
extern RecoveryState recovery;
extern FlightState state;
extern int motorCommands[4];
extern float baseThrottle;
extern float targetRoll, targetPitch, targetYaw;
extern float targetRollRate, targetPitchRate, targetYawRate;
extern float targetAltitude, targetVelocityX, targetVelocityY;
extern Madgwick filter;
extern PID pidAltitude, pidVelocityX, pidVelocityY, pidRoll, pidPitch, pidYaw, pidRollRate, pidPitchRate, pidYawRate;