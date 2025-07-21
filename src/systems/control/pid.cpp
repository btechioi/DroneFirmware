#include "pid.h"
#include "globals.h"

#define DERIVATIVE_FILTER_ALPHA 0.7 // Alpha for EMA filter (0.0 to 1.0, higher is more smoothing)

float computePID(PID* pid, float error) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - pid->lastComputeTime) / 1000.0; // Convert to seconds
  if (deltaTime == 0) deltaTime = 0.001; // Avoid division by zero

  // Proportional term
  float proportional = pid->kp * error;

  // Integral term: sum of errors over time
  pid->integral += error * deltaTime; // Accumulate error over time
  // Constrain integral to prevent wind-up.
  pid->integral = constrain(pid->integral, -pid->outputLimit, pid->outputLimit);

  // Derivative term: rate of change of error
  float rawDerivative = (error - pid->prevError) / deltaTime;
  // Apply EMA filter to derivative to reduce noise
  float derivative = (DERIVATIVE_FILTER_ALPHA * rawDerivative) + ((1.0 - DERIVATIVE_FILTER_ALPHA) * pid->prevDerivative);
  pid->prevDerivative = derivative; // Store filtered derivative for next iteration

  pid->prevError = error; // Store current error for next iteration's derivative
  pid->lastComputeTime = currentTime; // Update last compute time

  // Combine terms for final PID output
  float output = proportional + (pid->ki * pid->integral) + (pid->kd * derivative);
  pid->lastOutput = constrain(output, -pid->outputLimit, pid->outputLimit); // Store constrained output
  return pid->lastOutput;
}
