#pragma once
#include <Arduino.h>
#include "flight_mode.h"

constexpr float MIN_THROTTLE = 1000.0f;
constexpr float MAX_THROTTLE = 2000.0f;
constexpr float MAX_ANGLE = 30.0f;

constexpr unsigned long CONTROL_UPDATE_INTERVAL = 10;
constexpr unsigned long TELEMETRY_UPDATE_INTERVAL = 100;

struct TelemetryData {
    uint32_t timestamp;
    bool armed;
    float roll;
    float pitch;
    float yaw;
    float altitude;
    uint8_t mode;
    uint8_t errors;
};
