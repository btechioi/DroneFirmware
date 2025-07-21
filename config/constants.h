#pragma once

// PID Constants
constexpr float ROLL_PID_KP = 0.5f;
constexpr float ROLL_PID_KI = 0.0f;
constexpr float ROLL_PID_KD = 0.05f;

// System Limits
constexpr float MAX_ANGLE = 30.0f; // degrees
constexpr float MAX_ALTITUDE = 100.0f; // meters