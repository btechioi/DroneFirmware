#include "failsafe.h"

extern unsigned long lastCommandRxTime;

bool checkSignalLoss() {
    return (millis() - lastCommandRxTime > SIGNAL_LOSS_TIMEOUT_MS);
}

bool checkCriticalSensorFailure() {
    return imuError;
}

bool checkBatteryStatus() {
    if (batteryVoltage < CRITICAL_BATTERY_THRESHOLD) {
        return true;
    }
    return false;
}

bool checkGeoFence() {
    // TODO: Implement geo-fence check
    return false;
}

bool checkMotorPerformanceInFlight() {
    // TODO: Implement motor performance check
    return false;
}

void checkFailsafes() {
    if (checkSignalLoss()) {
        currentFailSafeState = FailSafeState::FAILSAFE_SIGNAL_LOSS;
    } else if (checkCriticalSensorFailure()) {
        currentFailSafeState = FailSafeState::FAILSAFE_CRITICAL_SENSOR_FAILURE;
    } else if (checkBatteryStatus()) {
        currentFailSafeState = FailSafeState::FAILSAFE_CRITICAL_BATTERY;
    } else if (checkGeoFence()) {
        currentFailSafeState = FailSafeState::FAILSAFE_GEO_FENCE;
    } else if (checkMotorPerformanceInFlight()) {
        currentFailSafeState = FailSafeState::FAILSAFE_MOTOR_FAILURE;
    } else {
        currentFailSafeState = FailSafeState::FAILSAFE_NONE;
    }
}
