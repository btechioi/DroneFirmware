#include <Arduino.h>
#include <SPI.h>
#include "flight_mode.h"
#include "globals.h"
#include "../config/pins.h"
#include "systems/sensors/flight_controller.h"
#include "systems/control/pid.h"
#include "systems/control/motor_audio.h"
#include "systems/control/led_status.h"
#include "systems/comms/spi_control.h"

FlightController fc;
MotorAudio motorAudio;
LEDStatus ledStatus;

static volatile bool motorsArmed = false;
static volatile FlightMode currentFlightMode = FlightMode::MANUAL;
static volatile FailSafeState failsafeState = FailSafeState::NONE;

static volatile float targetRoll = 0, targetPitch = 0, targetYaw = 0;
static volatile float baseThrottle = 1000;

static PID pidRoll(0.5f, 0.0f, 0.05f, 500.0f);
static PID pidPitch(0.5f, 0.0f, 0.05f, 500.0f);
static PID pidYaw(0.5f, 0.0f, 0.05f, 500.0f);

static SPITelemetryBus spiTelemetry;

static volatile uint16_t motorPWM[4] = {1000, 1000, 1000, 1000};

static volatile uint16_t rcRoll = 1500, rcPitch = 1500, rcThrottle = 1000, rcYaw = 1500;
static volatile uint16_t rcAux1 = 1000, rcAux2 = 1000, rcAux3 = 1000, rcAux4 = 1000;

static volatile bool companionConnected = false;
static volatile bool companionRCActive = false;
static uint32_t lastCompanionHeartbeat = 0;
static uint32_t lastCompanionRC = 0;

static volatile bool rcSignalLost = false;
static uint32_t lastRCInputTime = 0;
static constexpr uint32_t RC_SIGNAL_TIMEOUT_MS = 500;

static uint16_t prevRcRoll = 1500, prevRcPitch = 1500, prevRcYaw = 1500;
static uint32_t lastRcChangeTime = 0;
static constexpr uint32_t STUCK_CONTROL_TIMEOUT_MS = 3000;
static constexpr uint16_t STUCK_CONTROL_THRESHOLD = 20;

static constexpr uint32_t FAST_LOOP_HZ = 400;
static constexpr uint32_t FAST_LOOP_US = 2500;
static constexpr uint32_t COMPANION_HEARTBEAT_TIMEOUT_MS = 2000;
static constexpr uint32_t COMPANION_RC_TIMEOUT_MS = 100;
static constexpr uint32_t PING_INTERVAL_MS = 500;

static bool companionUnresponsive = false;
static uint32_t companionUnresponsiveStart = 0;
static constexpr uint32_t COMPANION_UNRESPONSIVE_MS = 5000;

enum class ControlSource {
    RC_RECEIVER,
    COMPANION,
    FAILSAFE
};

static volatile ControlSource activeControl = ControlSource::RC_RECEIVER;

static inline void fastLoop();
static inline void updateMotorsPWM();
static inline void audioMotorCallback(uint8_t motor, uint16_t pwm) {
    if (motor < 4) {
        uint16_t duty = map(pwm, 1000, 2000, 0, 65535);
        analogWrite(MOTOR_PINS[motor], duty >> 4);
    }
}
static inline void audioMotorCallback(uint8_t motor, uint16_t pwm);
static inline void processCompanion();
static inline void processRCInput();
static inline void checkFailsafes();
static inline void enterFailsafe(FailSafeState state);
static inline void returnToRC();
static inline void updateLEDStatus();

static inline void fastLoop() {
    fc.updateSensors();
    
    IMUSensor* imu = fc.getIMU();
    if (!imu) return;
    
    float roll = imu->getRoll();
    float pitch = imu->getPitch();
    
    float rollError = targetRoll - roll;
    float pitchError = targetPitch - pitch;
    
    float rollCorr = pidRoll.compute(rollError);
    float pitchCorr = pidPitch.compute(pitchError);
    
    int16_t m0 = baseThrottle + rollCorr + pitchCorr;
    int16_t m1 = baseThrottle - rollCorr + pitchCorr;
    int16_t m2 = baseThrottle - rollCorr - pitchCorr;
    int16_t m3 = baseThrottle + rollCorr - pitchCorr;
    
    m0 = constrain(m0, 1000, 2000);
    m1 = constrain(m1, 1000, 2000);
    m2 = constrain(m2, 1000, 2000);
    m3 = constrain(m3, 1000, 2000);
    
    motorPWM[0] = m0;
    motorPWM[1] = m1;
    motorPWM[2] = m2;
    motorPWM[3] = m3;
}

static inline void processRCInput() {
    if (activeControl == ControlSource::FAILSAFE) {
        return;
    }
    
    targetRoll = ((float)rcRoll - 1500.0f) / 500.0f;
    targetPitch = ((float)rcPitch - 1500.0f) / 500.0f;
    targetYaw = ((float)rcYaw - 1500.0f) / 500.0f;
    baseThrottle = rcThrottle;
    
    uint32_t now = millis();
    
    bool rollChanged = abs((int16_t)rcRoll - (int16_t)prevRcRoll) > STUCK_CONTROL_THRESHOLD;
    bool pitchChanged = abs((int16_t)rcPitch - (int16_t)prevRcPitch) > STUCK_CONTROL_THRESHOLD;
    bool yawChanged = abs((int16_t)rcYaw - (int16_t)prevRcYaw) > STUCK_CONTROL_THRESHOLD;
    
    if (rollChanged || pitchChanged || yawChanged) {
        lastRcChangeTime = now;
        prevRcRoll = rcRoll;
        prevRcPitch = rcPitch;
        prevRcYaw = rcYaw;
    }
    
    lastRCInputTime = now;
    rcSignalLost = false;
    
    if (rcAux1 > 1500 && !motorsArmed) {
        motorsArmed = true;
        motorAudio.playTone(MotorAudio::Tone::ARMED_SUCCESS);
    } else if (rcAux1 < 1200 && motorsArmed) {
        motorsArmed = false;
        motorAudio.playTone(MotorAudio::Tone::DISARMED);
    }
    
    if (rcAux2 > 1500) {
        currentFlightMode = FlightMode::ALTITUDE_HOLD;
    } else {
        currentFlightMode = FlightMode::MANUAL;
    }
}

static inline void processCompanion() {
    uint32_t now = millis();
    
    if (now - lastCompanionHeartbeat > PING_INTERVAL_MS) {
        spiTelemetry.sendPing();
        lastCompanionHeartbeat = now;
    }
    
    if (spiTelemetry.isPongReceived()) {
        if (!companionConnected) {
            motorAudio.playTone(MotorAudio::Tone::RC_FOUND);
        }
        companionConnected = true;
        companionUnresponsive = false;
        spiTelemetry.clearPongReceived();
    }
    
    uint16_t roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4;
    if (spiTelemetry.receiveRCChannels(&roll, &pitch, &throttle, &yaw, &aux1, &aux2, &aux3, &aux4)) {
        rcRoll = roll;
        rcPitch = pitch;
        rcThrottle = throttle;
        rcYaw = yaw;
        rcAux1 = aux1;
        rcAux2 = aux2;
        rcAux3 = aux3;
        rcAux4 = aux4;
        
        lastCompanionRC = now;
        companionRCActive = true;
        
        if (activeControl != ControlSource::COMPANION) {
            activeControl = ControlSource::COMPANION;
            Serial.println("Switched to companion control");
        }
        
        targetRoll = ((float)rcRoll - 1500.0f) / 500.0f;
        targetPitch = ((float)rcPitch - 1500.0f) / 500.0f;
        targetYaw = ((float)rcYaw - 1500.0f) / 500.0f;
        baseThrottle = rcThrottle;
    }
    
    if (now - lastCompanionHeartbeat > COMPANION_HEARTBEAT_TIMEOUT_MS) {
        if (companionConnected) {
            companionConnected = false;
            motorAudio.playTone(MotorAudio::Tone::RC_LOST);
            Serial.println("Companion disconnected");
        }
        
        if (activeControl == ControlSource::COMPANION && motorsArmed) {
            Serial.println("Companion timeout - returning to RC");
            returnToRC();
        }
    }
    
    if (now - lastCompanionRC > COMPANION_RC_TIMEOUT_MS) {
        companionRCActive = false;
    }
}

static inline void checkFailsafes() {
    uint32_t now = millis();
    
    if (activeControl == ControlSource::RC_RECEIVER) {
        if (now - lastRCInputTime > RC_SIGNAL_TIMEOUT_MS && !rcSignalLost) {
            rcSignalLost = true;
            Serial.println("WARNING: RC signal lost!");
            
            if (motorsArmed) {
                enterFailsafe(FailSafeState::SIGNAL_LOSS);
            }
        }
        
        if (now - lastRcChangeTime > STUCK_CONTROL_TIMEOUT_MS && motorsArmed) {
            Serial.println("WARNING: Controls may be stuck!");
            baseThrottle = 1200;
            targetRoll = 0;
            targetPitch = 0;
            targetYaw = 0;
        }
    }
    
    if (activeControl == ControlSource::COMPANION) {
        if (now - lastCompanionRC > COMPANION_UNRESPONSIVE_MS && !companionUnresponsive) {
            companionUnresponsive = true;
            companionUnresponsiveStart = now;
            Serial.println("WARNING: Companion unresponsive!");
        }
        
        if (companionUnresponsive && now - companionUnresponsiveStart > 3000) {
            Serial.println("Companion unresponsive timeout - returning to RC");
            returnToRC();
            companionUnresponsive = false;
        }
    }
    
    if (failsafeState != FailSafeState::NONE && motorsArmed) {
        switch (failsafeState) {
            case FailSafeState::SIGNAL_LOSS:
                baseThrottle = 1200;
                targetRoll = 0;
                targetPitch = 0;
                targetYaw = 0;
                break;
                
            case FailSafeState::LOW_BATTERY:
                baseThrottle = 1000;
                break;
                
            case FailSafeState::CRITICAL_SENSOR:
                motorsArmed = false;
                baseThrottle = 1000;
                break;
                
            default:
                break;
        }
    }
}

static inline void enterFailsafe(FailSafeState state) {
    if (failsafeState != FailSafeState::NONE) {
        return;
    }
    
    failsafeState = state;
    activeControl = ControlSource::FAILSAFE;
    
    Serial.print("FAILSAFE: ");
    switch (state) {
        case FailSafeState::SIGNAL_LOSS:
            Serial.println("RC signal lost");
            baseThrottle = 1200;
            motorAudio.playTone(MotorAudio::Tone::RC_LOST);
            break;
        case FailSafeState::LOW_BATTERY:
            Serial.println("Low battery");
            motorAudio.playTone(MotorAudio::Tone::LOW_BATTERY);
            break;
        case FailSafeState::CRITICAL_SENSOR:
            Serial.println("Critical sensor failure");
            motorAudio.playTone(MotorAudio::Tone::ERROR);
            break;
        default:
            Serial.println("Unknown");
            break;
    }
}

static inline void returnToRC() {
    if (failsafeState == FailSafeState::CRITICAL_SENSOR) {
        return;
    }
    
    failsafeState = FailSafeState::NONE;
    activeControl = ControlSource::RC_RECEIVER;
    
    baseThrottle = 1000;
    targetRoll = 0;
    targetPitch = 0;
    targetYaw = 0;
    
    motorAudio.playTone(MotorAudio::Tone::FAILSAFE_EXIT);
    Serial.println("Returned to RC control");
}

static inline void updateMotorsPWM() {
    if (!motorsArmed) {
        for (int i = 0; i < 4; i++) {
            motorPWM[i] = 0;
        }
    }
    
    for (int i = 0; i < 4; i++) {
        uint16_t pwm_us = motorPWM[i];
        uint16_t duty = map(pwm_us, 1000, 2000, 0, 65535);
        analogWrite(MOTOR_PINS[i], duty >> 4);
    }
}

static inline void updateLEDStatus() {
    if (failsafeState == FailSafeState::CRITICAL_SENSOR) {
        ledStatus.setColor(LEDStatus::RED);
        ledStatus.setState(LEDStatus::SOLID);
    } else if (failsafeState != FailSafeState::NONE) {
        ledStatus.setColor(LEDStatus::RED);
        ledStatus.setState(LEDStatus::BLINK_FAST);
    } else if (!fc.hasIMU()) {
        ledStatus.setColor(LEDStatus::RED);
        ledStatus.setState(LEDStatus::BLINK_SLOW);
    } else if (motorsArmed) {
        ledStatus.setColor(LEDStatus::GREEN);
        ledStatus.setState(LEDStatus::SOLID);
    } else if (companionConnected) {
        ledStatus.setColor(LEDStatus::BLUE);
        ledStatus.setState(LEDStatus::DOUBLE_BLINK);
    } else {
        ledStatus.setColor(LEDStatus::GREEN);
        ledStatus.setState(LEDStatus::BLINK_SLOW);
    }
}

static inline void telemetryLoop() {
    spiTelemetry.update();
    
    IMUSensor* imu = fc.getIMU();
    BarometerSensor* baro = fc.getBarometer();
    
    static uint16_t seq = 0;
    uint8_t packet[32];
    uint8_t* p = packet;
    
    *p++ = 0xAA;
    *p++ = 0x02;
    *p++ = seq++;
    
    uint8_t flags = 0;
    flags |= motorsArmed ? 0x01 : 0;
    flags |= companionConnected ? 0x02 : 0;
    flags |= (uint8_t)failsafeState << 4;
    
    *p++ = flags;
    *p++ = (uint8_t)activeControl;
    
    int16_t roll_i = imu ? (int16_t)(imu->getRoll() * 100) : 0;
    int16_t pitch_i = imu ? (int16_t)(imu->getPitch() * 100) : 0;
    int16_t yaw_i = imu ? (int16_t)(imu->getYaw() * 100) : 0;
    
    memcpy(p, &roll_i, 2); p += 2;
    memcpy(p, &pitch_i, 2); p += 2;
    memcpy(p, &yaw_i, 2); p += 2;
    
    int32_t alt_i = baro ? (int32_t)(baro->getAltitude() * 100) : 0;
    memcpy(p, &alt_i, 4); p += 4;
    
    int16_t gyro[3];
    if (imu) {
        gyro[0] = (int16_t)(imu->getRollRate() * 100);
        gyro[1] = (int16_t)(imu->getPitchRate() * 100);
        gyro[2] = (int16_t)(imu->getYawRate() * 100);
    } else {
        gyro[0] = gyro[1] = gyro[2] = 0;
    }
    memcpy(p, gyro, 6); p += 6;
    
    uint16_t crc = 0;
    for (int i = 0; i < (p - packet); i++) {
        crc += packet[i];
    }
    memcpy(p, &crc, 2);
    
    spiTelemetry.sendTelemetry(packet, p - packet + 2);
}

void setup() {
    Serial.begin(115200);
    delay(100);
    
    Serial.println("\n============================================");
    Serial.println("   DRONE FIRMWARE v2.3 - DUAL RC + FAILSAFE");
    Serial.println("============================================\n");
    
    for (int i = 0; i < 4; i++) {
        pinMode(MOTOR_PINS[i], OUTPUT);
        analogWriteResolution(12);
    }
    
    for (int i = 0; i < 8; i++) {
        pinMode(RC_PINS[i], INPUT);
    }
    
    ledStatus.begin();
    motorAudio.begin();
    motorAudio.setMotorOutputFunc(audioMotorCallback);
    motorAudio.playTone(MotorAudio::Tone::READY);
    
    fc.detectSensors();
    fc.initializeSensors();
    
    if (!fc.hasIMU()) {
        Serial.println("CRITICAL: No IMU!");
        motorAudio.playTone(MotorAudio::Tone::ERROR);
        enterFailsafe(FailSafeState::CRITICAL_SENSOR);
    }
    
    spiTelemetry.begin();
    
    lastRCInputTime = millis();
    
    Serial.print("Fast Loop: "); Serial.print(FAST_LOOP_HZ); Serial.println(" Hz");
    Serial.print("IMU: "); Serial.println(fc.hasIMU() ? "OK" : "FAIL");
    Serial.print("Barometer: "); Serial.println(fc.hasBarometer() ? "OK" : "N/A");
    Serial.print("Companion SPI: "); Serial.println(spiTelemetry.isConnected() ? "Ready" : "Waiting...");
    Serial.println("\nReady. ARM with RC or connect companion.\n");
}

uint32_t loopCounter = 0;

void loop() {
    static uint32_t lastTelemetryTime = 0;
    static uint32_t lastStatusTime = 0;
    static uint32_t lastIMUTime = 0;
    static uint32_t lastRCReadTime = 0;
    
    uint32_t now = micros();
    
    if (now - lastIMUTime >= FAST_LOOP_US) {
        lastIMUTime = now;
        
        checkFailsafes();
        fastLoop();
        updateMotorsPWM();
        
        if ((loopCounter % 400) == 0) {
            lastTelemetryTime = micros();
        }
        
        if ((loopCounter % 4000) == 0) {
            lastStatusTime = micros();
        }
        
        loopCounter++;
    }
    
    if (now - lastRCReadTime >= 2000) {
        lastRCReadTime = now;
        processCompanion();
        
        if (activeControl == ControlSource::RC_RECEIVER) {
            processRCInput();
        }
    }
    
    if (now - lastTelemetryTime >= 10000) {
        lastTelemetryTime = now;
        telemetryLoop();
    }
    
    if (now - lastStatusTime >= 100000) {
        lastStatusTime = now;
        fc.printSensorStatus();
        
        Serial.print("[STATUS] Control: ");
        switch (activeControl) {
            case ControlSource::RC_RECEIVER: Serial.print("RC"); break;
            case ControlSource::COMPANION: Serial.print("Companion"); break;
            case ControlSource::FAILSAFE: Serial.print("FAILSAFE"); break;
        }
        Serial.print(" | Companion: ");
        Serial.print(companionConnected ? "ON" : "OFF");
        Serial.print(" | Failsafe: ");
        Serial.println((int)failsafeState);
    }
    
    motorAudio.update();
    ledStatus.update();
    updateLEDStatus();
    
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'a') {
            motorsArmed = true;
            Serial.println("ARMED");
        } else if (cmd == 'd') {
            motorsArmed = false;
            failsafeState = FailSafeState::NONE;
            Serial.println("DISARMED");
        } else if (cmd == 'f') {
            Serial.print("Failsafe: ");
            Serial.println((int)failsafeState);
            Serial.print("Control: ");
            Serial.println((int)activeControl);
        } else if (cmd == 'r') {
            if (failsafeState != FailSafeState::CRITICAL_SENSOR) {
                returnToRC();
                Serial.println("RC control restored");
            }
        } else if (cmd == 's') {
            Serial.print("Mode: "); Serial.println((int)currentFlightMode);
            Serial.print("Companion: "); Serial.println(companionConnected ? "Connected" : "Disconnected");
            Serial.print("Armed: "); Serial.println(motorsArmed ? "YES" : "NO");
        } else if (cmd == 'p') {
            motorAudio.playTone(MotorAudio::Tone::FIND_DRONE);
            Serial.println("Playing find-drone siren");
        }
    }
}
