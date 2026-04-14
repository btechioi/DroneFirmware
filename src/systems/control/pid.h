#pragma once
#include <Arduino.h>

class PID {
public:
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float outputLimit = 500.0f);
    
    void setGains(float kp, float ki, float kd);
    void setOutputLimit(float limit);
    float compute(float target, float measured);
    float compute(float error);
    void reset();
    
    float kp, ki, kd;

private:
    float outputLimit_;
    float prevError_ = 0;
    float integral_ = 0;
    unsigned long lastComputeTime_ = 0;
    float prevDerivative_ = 0;
};
