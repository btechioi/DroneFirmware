#include "radio.h"
#include "../control/pid_tuner.h"

extern bool motorsArmed;
extern FlightMode currentFlightMode;

void initRadio() {
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(915E6)) {
        Serial.println("LoRa init failed!");
    } else {
        Serial.println("LoRa radio initialized");
    }
}

void updateRadio() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        uint8_t buffer[64];
        int len = LoRa.readBytes(buffer, packetSize);
        processCommand(buffer, len);
    }
}

void sendTelemetry() {
    if (!motorsArmed) return;
    
    LoRa.beginPacket();
    LoRa.print("T:");
    LoRa.print(millis());
    LoRa.print(",");
    LoRa.print(motorsArmed ? "1" : "0");
    LoRa.endPacket();
}

void processCommand(uint8_t* data, size_t len) {
    if (len < 2) return;
    
    switch (data[0]) {
        case 0x01:
            motorsArmed = (data[1] == 1);
            Serial.print("Motors: ");
            Serial.println(motorsArmed ? "ARMED" : "DISARMED");
            break;
            
        case 0x02:
            currentFlightMode = static_cast<FlightMode>(data[1]);
            Serial.print("Mode: ");
            Serial.println(data[1]);
            break;
            
        case 0x10:
            if (len >= 7) {
                extern float targetRoll, targetPitch, targetYaw;
                extern float baseThrottle;
                
                int16_t rollRaw = (int16_t)(data[1] << 8 | data[2]);
                int16_t pitchRaw = (int16_t)(data[3] << 8 | data[4]);
                uint16_t throttleRaw = (uint16_t)(data[5] << 8 | data[6]);
                
                targetRoll = rollRaw / 100.0f;
                targetPitch = pitchRaw / 100.0f;
                baseThrottle = 1000 + throttleRaw;
            }
            break;
            
        case 0xF0:
            PID_TUNING_mgr.handleCommand(data[1], &data[2], len - 2);
            break;
            
        case 0xF1:
            if (len >= 4) {
                uint8_t axis = data[1];
                float kp = *(float*)&data[2];
                float ki = *(float*)&data[6];
                float kd = *(float*)&data[10];
                
                PIDTuningProfile& profile = PID_TUNING_mgr.getCurrentProfile();
                switch (axis) {
                    case 0: profile.rollRateKp = kp; profile.rollRateKi = ki; profile.rollRateKd = kd; break;
                    case 1: profile.pitchRateKp = kp; profile.pitchRateKi = ki; profile.pitchRateKd = kd; break;
                    case 2: profile.yawRateKp = kp; profile.yawRateKi = ki; profile.yawRateKd = kd; break;
                    case 3: profile.rollAttKp = kp; profile.rollAttKi = ki; profile.rollAttKd = kd; break;
                    case 4: profile.pitchAttKp = kp; profile.pitchAttKi = ki; profile.pitchAttKd = kd; break;
                    case 5: profile.yawAttKp = kp; profile.yawAttKi = ki; profile.yawAttKd = kd; break;
                    case 6: profile.altKp = kp; profile.altKi = ki; profile.altKd = kd; break;
                    case 7: profile.posKp = kp; profile.posKi = ki; profile.posKd = kd; break;
                }
                
                Serial.print("[COMMS] PID gain update - Axis:");
                Serial.print(axis);
                Serial.print(" Kp:");
                Serial.print(kp, 4);
                Serial.print(" Ki:");
                Serial.print(ki, 6);
                Serial.print(" Kd:");
                Serial.println(kd, 6);
            }
            break;
            
        case 0xF2:
            PID_TUNING_mgr.sendTuningStatus();
            break;
            
        case 0xF3:
            PID_TUNING_mgr.sendAllGains();
            break;
    }
}
