#include "pid.h"

#define DERIVATIVE_FILTER_ALPHA 0.7f

PID::PID(float kp, float ki, float kd, float outputLimit)
    : kp(kp), ki(ki), kd(kd), outputLimit_(outputLimit) {}

void PID::setGains(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PID::setOutputLimit(float limit) {
    outputLimit_ = limit;
}

float PID::compute(float target, float measured) {
    return compute(target - measured);
}

float PID::compute(float error) {
    unsigned long now = millis();
    float dt = (now - lastComputeTime_) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    
    float p = kp * error;
    
    integral_ += error * dt;
    integral_ = constrain(integral_, -outputLimit_, outputLimit_);
    
    float derivative = 0;
    if (dt > 0) {
        float rawDerivative = (error - prevError_) / dt;
        derivative = (DERIVATIVE_FILTER_ALPHA * rawDerivative) + 
                     ((1.0f - DERIVATIVE_FILTER_ALPHA) * prevDerivative_);
        prevDerivative_ = derivative;
    }
    
    prevError_ = error;
    lastComputeTime_ = now;
    
    float output = p + (ki * integral_) + (kd * derivative);
    return constrain(output, -outputLimit_, outputLimit_);
}

void PID::reset() {
    prevError_ = 0;
    integral_ = 0;
    prevDerivative_ = 0;
    lastComputeTime_ = 0;
}
