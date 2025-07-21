#pragma once

struct PID {
    float kp, ki, kd;
    float integral = 0;
    float prevError = 0;
    float outputLimit;
    
    float compute(float error, float dt);
};

float PID::compute(float error, float dt) {
    if (dt <= 0) dt = 0.001f;
    
    float p = kp * error;
    integral += ki * error * dt;
    integral = constrain(integral, -outputLimit, outputLimit);
    float d = kd * (error - prevError) / dt;
    
    prevError = error;
    return constrain(p + integral + d, -outputLimit, outputLimit);
}