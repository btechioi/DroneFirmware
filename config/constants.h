#pragma once

// PID Constants
constexpr float ROLL_PID_KP = 0.5f;
constexpr float ROLL_PID_KI = 0.0f;
constexpr float ROLL_PID_KD = 0.05f;

// System Limits
constexpr float MAX_ANGLE = 30.0f; // degrees
constexpr float MAX_ALTITUDE = 100.0f; // meters

// Throttle
constexpr float MIN_THROTTLE = 1000.0f;
constexpr float MAX_THROTTLE = 2000.0f;

// Failsafe
constexpr uint32_t SIGNAL_LOSS_TIMEOUT_MS = 1000;