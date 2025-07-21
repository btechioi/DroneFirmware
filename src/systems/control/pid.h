#pragma once

#include <Arduino.h>

struct PID {
  float kp;          // Proportional gain
  float ki;          // Integral gain
  float kd;          // Derivative gain
  float prevError = 0; // Previous error for derivative calculation
  float integral = 0;  // Integral sum for integral calculation
  unsigned long lastComputeTime = 0; // For delta_t calculation in derivative
  float outputLimit; // Max/Min output for the PID controller (e.g., degrees/sec or throttle adjustment)
  float prevDerivative = 0; // For derivative filtering
  float lastOutput = 0; // Store last output for saturation check
};

float computePID(PID* pid, float error);