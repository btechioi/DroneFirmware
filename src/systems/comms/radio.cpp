#include "radio.h"
#include "globals.h"

void initRadio() {
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(915E6)) {
        Serial.println("LoRa init failed!");
    }
}

void updateRadio() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        uint8_t buffer[256];
        LoRa.readBytes(buffer, packetSize);
        processCommand(buffer, packetSize);
    }
}

void sendTelemetry() {
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&sensorData, sizeof(sensorData));
    LoRa.endPacket();
}

void processCommand(uint8_t* data, int packetSize) {
    // Simple command protocol
    switch(data[0]) {
        case 0x01: // Arm/disarm
            motorsArmed = data[1] == 1;
            break;
        case 0x02: // Set flight mode
            currentFlightMode = static_cast<FlightMode>(data[1]);
            break;
        case 0x03: // Start autotune
            autotuneState = AutotuneState::TUNING_ROLL;
            break;
        case 0x04: // Stop autotune
            autotuneState = AutotuneState::IDLE;
            break;
        case 0x1A: // Set flight mode
            currentFlightMode = static_cast<FlightMode>(data[1]);
            break;
        case 0x23: // Calibration request
            calibrateIMU();
            calibrateMagnetometer();
            break;
        case 0x24: // Add waypoint
            // TODO: Implement add waypoint
            break;
        case 0x25: // Clear waypoints
            numWaypoints = 0;
            break;
        case 0x26: // Start waypoint mission
            currentFlightMode = FlightMode::WAYPOINT_NAV;
            break;
    }
}