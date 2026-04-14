#pragma once

#include <stdint.h>

constexpr float ROLL_PID_KP = 0.5f;
constexpr float ROLL_PID_KI = 0.0f;
constexpr float ROLL_PID_KD = 0.05f;

constexpr float MAX_ANGLE = 30.0f;
constexpr float MAX_ALTITUDE = 100.0f;

constexpr float MIN_THROTTLE = 1000.0f;
constexpr float MAX_THROTTLE = 2000.0f;

constexpr uint32_t SIGNAL_LOSS_TIMEOUT_MS = 1000;

constexpr float CONTROL_RATE_HZ = 400.0f;
constexpr float ATTITUDE_RATE_HZ = 200.0f;
constexpr float POSITION_RATE_HZ = 50.0f;