#include "globals.h"
#include "drivers/imu.h"
#include "systems/control/pid.h"
#include "systems/comms/radio.h"

// Global instances
MPU6050 mpu;
TinyGPSPlus gps;
Adafruit_BMP085 bmp;
Adafruit_HMC5883_Unified mag(12345);

// System state
FlightMode currentFlightMode = FlightMode::MANUAL;
bool motorsArmed = false;
SensorData sensorData;

void setup() {
    Serial.begin(115200);
    
    // Initialize systems
    initIMU();
    initRadio();
    initMotors();
    
    // Wait for sensors to stabilize
    delay(1000);
    
    Serial.println("Drone initialized");
}

void loop() {
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    
    // Main 100Hz loop
    if (now - lastUpdate >= 10) {
        updateIMU();
        updateRadio();
        flightControllerUpdate();
        lastUpdate = now;
    }
    
    // Safety checks
    checkFailsafes();
}